// Polls a WitMotion HWT3100-TTL/232 fluxgate compass over Modbus RTU and
// emits magnetic heading in radians.
//
// Unlike the CV7, the HWT3100 does not speak NMEA 0183. Its serial port
// (see product manual, "HWT3100TTL_232") offers ASCII/AT-command mode and
// Modbus RTU mode; ASCII mode's continuous streaming line format is not
// documented byte-for-byte in the manual, so this reader instead switches
// the sensor into Modbus mode on startup (AT+MODE=1 — an AT command the
// manual documents as being accepted regardless of the sensor's current
// mode) and polls it as a Modbus RTU master, which the manual documents
// precisely down to the byte, CRC included.
//
// Each poll reads the 4-register MAGX..YAW block starting at 0xDB (the
// sensor only exposes that block starting at 0xDB, not as individually
// addressable reads) and uses only the YAW (heading) register. Runs on
// its own FreeRTOS task, mirroring NMEA0183IOTask's dedicated task for
// the CV7, because each poll blocks on a UART round trip.

#ifndef WIND_INTERFACE_SRC_HWT3100_HEADING_READER_H_
#define WIND_INTERFACE_SRC_HWT3100_HEADING_READER_H_

#include <Arduino.h>

#include <cmath>

#include "sensesp/system/valueproducer.h"

namespace wind_interface {

class Hwt3100HeadingReader : public sensesp::ValueProducer<float> {
 public:
  explicit Hwt3100HeadingReader(HardwareSerial* serial,
                                 unsigned int poll_interval_ms = 200)
      : serial_{serial}, poll_interval_ms_{poll_interval_ms} {
    xTaskCreate(&Hwt3100HeadingReader::TaskEntry, "hwt3100", 4096, this, 1,
                nullptr);
  }

 private:
  static constexpr uint8_t kDeviceId = 0x00;
  static constexpr uint16_t kRegMagXStart = 0x00DB;
  static constexpr uint16_t kRegCount = 4;  // MAGX, MAGY, MAGZ, YAW
  // ID + command + byte count + 4 registers (2 bytes each) + CRC (2 bytes).
  static constexpr size_t kResponseLength = 3 + 2 * kRegCount + 2;
  static constexpr unsigned long kResponseTimeoutMs = 100;

  static void TaskEntry(void* param) {
    static_cast<Hwt3100HeadingReader*>(param)->Run();
  }

  [[noreturn]] void Run() {
    // Force Modbus mode. If the sensor is already in Modbus mode, this
    // text is simply an invalid frame (bad CRC) and is discarded, so it's
    // safe to send unconditionally on every boot.
    serial_->print("AT+MODE=1\r\n");
    delay(50);
    while (serial_->available()) {
      serial_->read();
    }

    while (true) {
      SendReadRequest();
      float heading_radians;
      if (ReadResponse(&heading_radians)) {
        this->emit(heading_radians);
      }
      delay(poll_interval_ms_);
    }
  }

  void SendReadRequest() {
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
    serial_->write(request, sizeof(request));
  }

  bool ReadResponse(float* heading_radians) {
    uint8_t response[kResponseLength];
    size_t received = 0;
    unsigned long deadline = millis() + kResponseTimeoutMs;
    while (received < kResponseLength && millis() < deadline) {
      if (serial_->available()) {
        response[received++] = serial_->read();
      }
    }
    if (received < kResponseLength) {
      return false;
    }
    if (response[0] != kDeviceId || response[1] != 0x03 ||
        response[2] != kRegCount * 2) {
      return false;
    }
    // CRC is transmitted low byte first (confirmed against the manual's
    // worked example: CRC16(00 03 00 D0 00 01) = 0x2284, sent as 84 22).
    uint16_t received_crc = response[kResponseLength - 2] |
                             (response[kResponseLength - 1] << 8);
    uint16_t computed_crc = ModbusCrc16(response, kResponseLength - 2);
    if (received_crc != computed_crc) {
      return false;
    }
    // Registers, in order: MAGX, MAGY, MAGZ, YAW. Only YAW is used here;
    // it's a signed value in 0.1 degree units (manual example: raw 0x0405
    // = 1029 -> 102.9 degrees).
    int16_t yaw_raw = static_cast<int16_t>((response[9] << 8) | response[10]);
    float heading_degrees = yaw_raw / 10.0f;
    float heading = heading_degrees * static_cast<float>(M_PI) / 180.0f;
    if (heading < 0) {
      heading += 2 * static_cast<float>(M_PI);
    }
    *heading_radians = heading;
    return true;
  }

  static uint16_t ModbusCrc16(const uint8_t* data, size_t length) {
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

  HardwareSerial* serial_;
  unsigned int poll_interval_ms_;
};

}  // namespace wind_interface

#endif  // WIND_INTERFACE_SRC_HWT3100_HEADING_READER_H_
