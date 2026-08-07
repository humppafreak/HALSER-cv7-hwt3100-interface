// Standalone HWT3100 Modbus bench test for a plain ESP32-WROOM-32 dev board
// (e.g. the AZ-Delivery ESP32 Dev Kit C), with no HALSER board, CV7, N2K bus,
// or SensESP app involved. Built by the `wroom32_hwt3100_test` PlatformIO
// environment (see platformio.ini) as a separate firmware image from the
// main HALSER application in src/main.cpp.
//
// Purpose: verify the HWT3100 Modbus RTU wiring and protocol handling (mode
// switch, register read of the MAGX..YAW block, register writes for
// calibration) on hardware that's actually on the bench, before the real
// HALSER + CV7 setup is available. This intentionally re-implements a
// trimmed-down copy of the Modbus framing from hwt3100_heading_reader.h
// rather than reusing that class directly, since Hwt3100HeadingReader
// derives from sensesp::ValueProducer<float> and pulls in the full SensESP
// app framework (WiFi, web UI, filesystem config) that a bare Modbus wiring
// test doesn't need.
//
// Wiring (adjust kHeadingRxPin/kHeadingTxPin below if different):
//   ESP32 GPIO16 (RX2) <- HWT3100 TX
//   ESP32 GPIO17 (TX2) -> HWT3100 RX
//   Common GND between ESP32 and HWT3100.
//   GPIO16/17 are UART2's default pins on the ESP32 WROOM-32 and aren't
//   strapping pins, so they're safe to use on any standard 30/38-pin dev
//   board without special jumpering. Confirm the HWT3100 variant's signal
//   levels (TTL vs RS-232) match the ESP32's 3.3 V logic before wiring —
//   this test assumes the TTL variant.
//
// Serial monitor commands (115200 baud):
//   v  - request the read-only VERSION register (0xD0)
//   c  - start magnetic field calibration (CAL=1)
//   x  - stop/end calibration (CAL=0)
//   b  - clear magnetic bias (CAL=2)
// Heading is printed once per poll cycle regardless of commands.

#include <Arduino.h>

namespace {

constexpr int kHeadingBitRate = 9600;
constexpr int kHeadingRxPin = 16;
constexpr int kHeadingTxPin = 17;
constexpr unsigned int kPollIntervalMs = 200;

constexpr uint8_t kDeviceId = 0x00;
constexpr uint16_t kRegMagXStart = 0x00DB;
constexpr uint16_t kRegCount = 4;  // MAGX, MAGY, MAGZ, YAW
constexpr uint16_t kRegCal = 0x00D9;
constexpr uint16_t kRegVersion = 0x00D0;
// ID + command + byte count + 4 registers (2 bytes each) + CRC (2 bytes).
constexpr size_t kResponseLength = 3 + 2 * kRegCount + 2;
// Function 0x06 (write single register) response is an 8-byte echo of the
// request: ID + command + register (2) + value (2) + CRC (2).
constexpr size_t kWriteResponseLength = 8;
constexpr unsigned long kResponseTimeoutMs = 100;

uint16_t ModbusCrc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void PrintHex(const uint8_t* data, size_t length) {
  for (size_t i = 0; i < length; i++) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
}

void SendReadBlockRequest() {
  uint8_t request[8] = {
      kDeviceId,
      0x03,
      static_cast<uint8_t>(kRegMagXStart >> 8),
      static_cast<uint8_t>(kRegMagXStart & 0xFF),
      static_cast<uint8_t>(kRegCount >> 8),
      static_cast<uint8_t>(kRegCount & 0xFF),
      0,
      0,
  };
  uint16_t crc = ModbusCrc16(request, 6);
  request[6] = static_cast<uint8_t>(crc & 0xFF);
  request[7] = static_cast<uint8_t>(crc >> 8);
  Serial2.write(request, sizeof(request));
}

bool ReadBlockResponse(float* heading_degrees) {
  uint8_t response[kResponseLength];
  size_t received = 0;
  unsigned long deadline = millis() + kResponseTimeoutMs;
  while (received < kResponseLength && millis() < deadline) {
    if (Serial2.available()) {
      response[received++] = Serial2.read();
    }
  }
  if (received < kResponseLength) {
    Serial.print("[HWT3100] read timeout, got ");
    Serial.print(received);
    Serial.print(" bytes: ");
    PrintHex(response, received);
    Serial.println();
    return false;
  }
  if (response[0] != kDeviceId || response[1] != 0x03 ||
      response[2] != kRegCount * 2) {
    Serial.print("[HWT3100] unexpected frame: ");
    PrintHex(response, received);
    Serial.println();
    return false;
  }
  uint16_t received_crc =
      response[kResponseLength - 2] | (response[kResponseLength - 1] << 8);
  uint16_t computed_crc = ModbusCrc16(response, kResponseLength - 2);
  if (received_crc != computed_crc) {
    Serial.print("[HWT3100] CRC mismatch: ");
    PrintHex(response, received);
    Serial.println();
    return false;
  }
  // Registers, in order: MAGX, MAGY, MAGZ, YAW. YAW is a signed value in 0.1
  // degree units.
  int16_t yaw_raw = static_cast<int16_t>((response[9] << 8) | response[10]);
  *heading_degrees = yaw_raw / 10.0f;
  return true;
}

// Writes a single holding register (function 0x06) and validates the
// sensor's echo response, per the Modbus spec for that function.
bool WriteRegister(uint16_t reg, uint16_t value) {
  uint8_t request[8] = {
      kDeviceId,
      0x06,
      static_cast<uint8_t>(reg >> 8),
      static_cast<uint8_t>(reg & 0xFF),
      static_cast<uint8_t>(value >> 8),
      static_cast<uint8_t>(value & 0xFF),
      0,
      0,
  };
  uint16_t crc = ModbusCrc16(request, 6);
  request[6] = static_cast<uint8_t>(crc & 0xFF);
  request[7] = static_cast<uint8_t>(crc >> 8);

  while (Serial2.available()) {
    Serial2.read();
  }
  Serial2.write(request, sizeof(request));

  uint8_t response[kWriteResponseLength];
  size_t received = 0;
  unsigned long deadline = millis() + kResponseTimeoutMs;
  while (received < kWriteResponseLength && millis() < deadline) {
    if (Serial2.available()) {
      response[received++] = Serial2.read();
    }
  }
  if (received < kWriteResponseLength) {
    Serial.print("[HWT3100] write timeout, got ");
    Serial.print(received);
    Serial.println(" bytes");
    return false;
  }
  for (size_t i = 0; i < kWriteResponseLength; i++) {
    if (response[i] != request[i]) {
      Serial.println("[HWT3100] write response did not echo request");
      return false;
    }
  }
  return true;
}

bool ReadSingleRegister(uint16_t reg, uint16_t* out_value) {
  uint8_t request[8] = {
      kDeviceId, 0x03,
      static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF),
      0, 1,
      0, 0,
  };
  uint16_t crc = ModbusCrc16(request, 6);
  request[6] = static_cast<uint8_t>(crc & 0xFF);
  request[7] = static_cast<uint8_t>(crc >> 8);

  while (Serial2.available()) {
    Serial2.read();
  }
  Serial2.write(request, sizeof(request));

  constexpr size_t kLength = 3 + 2 + 2;  // id+cmd+len + 1 register + CRC.
  uint8_t response[kLength];
  size_t received = 0;
  unsigned long deadline = millis() + kResponseTimeoutMs;
  while (received < kLength && millis() < deadline) {
    if (Serial2.available()) {
      response[received++] = Serial2.read();
    }
  }
  if (received < kLength) {
    Serial.print("[HWT3100] version read timeout, got ");
    Serial.print(received);
    Serial.println(" bytes");
    return false;
  }
  if (response[0] != kDeviceId || response[1] != 0x03 || response[2] != 2) {
    Serial.println("[HWT3100] unexpected version response frame");
    return false;
  }
  uint16_t received_crc = response[kLength - 2] | (response[kLength - 1] << 8);
  if (ModbusCrc16(response, kLength - 2) != received_crc) {
    Serial.println("[HWT3100] version response CRC mismatch");
    return false;
  }
  *out_value = (response[3] << 8) | response[4];
  return true;
}

void HandleSerialCommand() {
  if (!Serial.available()) {
    return;
  }
  char command = Serial.read();
  switch (command) {
    case 'v': {
      uint16_t version;
      if (ReadSingleRegister(kRegVersion, &version)) {
        Serial.print("[HWT3100] VERSION = 0x");
        Serial.println(version, HEX);
      }
      break;
    }
    case 'c':
      Serial.println(WriteRegister(kRegCal, 1)
                          ? "[HWT3100] calibration started"
                          : "[HWT3100] calibration start failed");
      break;
    case 'x':
      Serial.println(WriteRegister(kRegCal, 0)
                          ? "[HWT3100] calibration stopped"
                          : "[HWT3100] calibration stop failed");
      break;
    case 'b':
      Serial.println(WriteRegister(kRegCal, 2)
                          ? "[HWT3100] magnetic bias cleared"
                          : "[HWT3100] clear bias failed");
      break;
    default:
      break;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("HWT3100 Modbus bench test (ESP32-WROOM-32)");
  Serial.println("Commands: v=version  c=cal start  x=cal stop  b=clear bias");

  Serial2.begin(kHeadingBitRate, SERIAL_8N1, kHeadingRxPin, kHeadingTxPin);

  // Force Modbus mode. If the sensor is already in Modbus mode, this text is
  // simply an invalid frame (bad CRC) and is discarded, so it's safe to send
  // unconditionally on every boot.
  Serial2.print("AT+MODE=1\r\n");
  delay(50);
  while (Serial2.available()) {
    Serial2.read();
  }
}

void loop() {
  HandleSerialCommand();

  SendReadBlockRequest();
  float heading_degrees;
  if (ReadBlockResponse(&heading_degrees)) {
    Serial.print("[HWT3100] heading = ");
    Serial.print(heading_degrees, 1);
    Serial.println(" deg");
  }

  delay(kPollIntervalMs);
}
