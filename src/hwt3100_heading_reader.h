// Polls a WitMotion HWT3100-TTL/232 fluxgate compass over Modbus RTU and
// emits magnetic heading in radians. Also drives the sensor's on-board
// magnetic field calibration procedure (register 0xD9, CAL) on request
// from the web UI.
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
//
// Calibration: the sensor's CAL register (0xD9) is a magnetic field
// calibration control — write 1 to start, 0 to end, 2 to clear the
// existing offset. The physical procedure (per the manual) is: start,
// physically rotate the sensor through 2-3 full turns, then end. The web
// UI can request Start/Stop manually, or Auto, which starts calibration
// and then watches the YAW register itself (reusing the readings already
// being polled) to detect two full rotations before ending it
// automatically — see AutoCalibrationTick(). A request is handed off from
// the requesting (main/UI) task to this task's Run() loop via a
// std::atomic command flag, since the UART is exclusively owned by this
// task; status is reported back the same way.
//
// Settings: baud rate (BAUD, 0xD2), smoothing filter (FILT, 0xD8), and
// active push interval (MRATE, 0xDA) are exposed the same way — a
// std::atomic pending-value hand-off, applied in Run()'s loop. Baud rate
// is the only one requiring special handling: after a successful write
// (acked at whatever baud is currently in use), this class immediately
// reconfigures its own serial_ to match, since the sensor is assumed to
// switch immediately too — there is no working-baud fallback if that
// assumption is wrong or the sensor is slower to switch than expected,
// so a change that goes wrong may require power-cycling the sensor back
// to its 9600 default. VERSION (0xD0) is read-only and read once at
// startup (retried each poll cycle until it succeeds).

#ifndef WIND_INTERFACE_SRC_HWT3100_HEADING_READER_H_
#define WIND_INTERFACE_SRC_HWT3100_HEADING_READER_H_

#include <Arduino.h>

#include <atomic>
#include <cmath>

#include "sensesp/system/valueproducer.h"

namespace wind_interface {

enum class CalibrationCommand : uint8_t {
  kNone,
  kStart,
  kStop,
  kAutoStart,
  kClearBias,
};

enum class CalibrationStatus : uint8_t {
  kIdle,
  kCalibrating,
  kAutoCalibrating,
  kDone,
  kBiasCleared,
  kTimedOut,
  kError,
};

class Hwt3100HeadingReader : public sensesp::ValueProducer<float> {
 public:
  // rx_pin/tx_pin are only needed for RequestSetBaudRate()'s live
  // serial_->begin() call after a successful baud change; the caller must
  // already have called serial->begin(...) with these same pins once
  // before constructing this reader.
  Hwt3100HeadingReader(HardwareSerial* serial, gpio_num_t rx_pin,
                        gpio_num_t tx_pin, unsigned int poll_interval_ms = 200)
      : serial_{serial},
        rx_pin_{rx_pin},
        tx_pin_{tx_pin},
        poll_interval_ms_{poll_interval_ms} {
    xTaskCreate(&Hwt3100HeadingReader::TaskEntry, "hwt3100", 4096, this, 1,
                nullptr);
  }

  // Requests are cheap, lock-free hand-offs to the polling task via
  // std::atomic; the most recently requested command wins if more than one
  // arrives between poll cycles.
  void RequestCalibrationStart() {
    pending_command_.store(CalibrationCommand::kStart);
  }
  void RequestCalibrationStop() {
    pending_command_.store(CalibrationCommand::kStop);
  }
  void RequestAutoCalibration() {
    pending_command_.store(CalibrationCommand::kAutoStart);
  }
  void RequestClearMagneticBias() {
    pending_command_.store(CalibrationCommand::kClearBias);
  }

  CalibrationStatus GetCalibrationStatus() const {
    return calibration_status_.load();
  }

  // Settings requests, same lock-free hand-off pattern as the calibration
  // commands above. Values are applied as-is (Modbus register writes take
  // raw uint16_t values); RequestSetBaudRate() additionally maps the given
  // baud to the sensor's BAUD register code and, only on a successfully
  // acked write, reconfigures this object's own serial connection to
  // match — see the file comment above for the risk that implies.
  void RequestSetBaudRate(uint32_t baud) {
    pending_baud_.store(static_cast<int32_t>(baud));
  }
  void RequestSetFilter(uint16_t filter) {
    pending_filter_.store(static_cast<int32_t>(filter));
  }
  void RequestSetPushInterval(uint16_t interval_ms) {
    pending_push_interval_.store(static_cast<int32_t>(interval_ms));
  }

  // 0 until the initial read (retried each poll cycle until it succeeds)
  // completes. The manual documents no meaning for this value beyond "a
  // version code" (worked example: raw 0x34C1 = 13505), so it's exposed
  // as-is rather than decoded.
  uint16_t GetHardwareVersion() const { return hardware_version_.load(); }

 private:
  static constexpr uint8_t kDeviceId = 0x00;
  static constexpr uint16_t kRegMagXStart = 0x00DB;
  static constexpr uint16_t kRegCount = 4;  // MAGX, MAGY, MAGZ, YAW
  static constexpr uint16_t kRegCal = 0x00D9;
  static constexpr uint16_t kRegVersion = 0x00D0;
  static constexpr uint16_t kRegBaud = 0x00D2;
  static constexpr uint16_t kRegFilt = 0x00D8;
  static constexpr uint16_t kRegMrate = 0x00DA;
  // ID + command + byte count + 4 registers (2 bytes each) + CRC (2 bytes).
  static constexpr size_t kResponseLength = 3 + 2 * kRegCount + 2;
  // Function 0x06 (write single register) response is an 8-byte echo of
  // the request: ID + command + register (2) + value (2) + CRC (2).
  static constexpr size_t kWriteResponseLength = 8;
  static constexpr unsigned long kResponseTimeoutMs = 100;
  // Two full rotations, per the manual's calibration procedure.
  static constexpr float kAutoCalibrationTargetDegrees = 720.0f;
  // Safety net so Auto can't get stuck in kAutoCalibrating forever if the
  // sensor is never actually rotated.
  static constexpr unsigned long kAutoCalibrationTimeoutMs = 240000;

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
      if (!version_read_) {
        uint16_t version;
        if (ReadSingleRegister(kRegVersion, &version)) {
          hardware_version_.store(version);
          version_read_ = true;
        }
      }

      HandlePendingCommand();
      HandlePendingSettings();

      SendReadRequest();
      float heading_radians;
      if (ReadResponse(&heading_radians)) {
        CalibrationStatus status = calibration_status_.load();
        if (status == CalibrationStatus::kAutoCalibrating) {
          AutoCalibrationTick(heading_radians);
        }
        // Suppress emitting to downstream consumers (N2K/Signal K/UDP/OLED)
        // while a calibration is actively in progress — the sensor's
        // output is unreliable mid-calibration, so a transient reading
        // shouldn't reach anything treating it as a real heading.
        bool actively_calibrating =
            status == CalibrationStatus::kCalibrating ||
            status == CalibrationStatus::kAutoCalibrating;
        if (!actively_calibrating) {
          this->emit(heading_radians);
        }
      }
      delay(poll_interval_ms_);
    }
  }

  void HandlePendingCommand() {
    CalibrationCommand command =
        pending_command_.exchange(CalibrationCommand::kNone);
    if (command == CalibrationCommand::kNone) {
      return;
    }
    switch (command) {
      case CalibrationCommand::kStart:
        if (WriteRegister(kRegCal, 1)) {
          calibration_status_.store(CalibrationStatus::kCalibrating);
        } else {
          calibration_status_.store(CalibrationStatus::kError);
        }
        break;
      case CalibrationCommand::kStop:
        if (WriteRegister(kRegCal, 0)) {
          calibration_status_.store(CalibrationStatus::kDone);
        } else {
          calibration_status_.store(CalibrationStatus::kError);
        }
        break;
      case CalibrationCommand::kAutoStart:
        if (WriteRegister(kRegCal, 1)) {
          auto_rotation_started_ = false;
          auto_rotation_degrees_ = 0.0f;
          auto_calibration_started_at_ms_ = millis();
          calibration_status_.store(CalibrationStatus::kAutoCalibrating);
        } else {
          calibration_status_.store(CalibrationStatus::kError);
        }
        break;
      case CalibrationCommand::kClearBias:
        if (WriteRegister(kRegCal, 2)) {
          calibration_status_.store(CalibrationStatus::kBiasCleared);
        } else {
          calibration_status_.store(CalibrationStatus::kError);
        }
        break;
      case CalibrationCommand::kNone:
        break;
    }
  }

  void HandlePendingSettings() {
    int32_t requested_baud = pending_baud_.exchange(-1);
    if (requested_baud >= 0) {
      int8_t code = BaudRegisterCode(static_cast<uint32_t>(requested_baud));
      if (code >= 0 &&
          WriteRegister(kRegBaud, static_cast<uint16_t>(code))) {
        // Assumes the sensor switches immediately after acking, at
        // whatever baud the ack itself was just sent at -- see the file
        // comment for the risk if that assumption doesn't hold.
        serial_->begin(static_cast<uint32_t>(requested_baud), SERIAL_8N1,
                        rx_pin_, tx_pin_);
        delay(50);  // Let the UART settle after reconfiguring.
        while (serial_->available()) {
          serial_->read();
        }
      }
      // On an unsupported baud or a failed/unacked write, nothing is
      // changed; the request is simply dropped rather than retried, same
      // as filter/push-interval below.
    }

    int32_t requested_filter = pending_filter_.exchange(-1);
    if (requested_filter >= 0) {
      WriteRegister(kRegFilt, static_cast<uint16_t>(requested_filter));
    }

    int32_t requested_interval = pending_push_interval_.exchange(-1);
    if (requested_interval >= 0) {
      WriteRegister(kRegMrate, static_cast<uint16_t>(requested_interval));
    }
  }

  // Maps a baud rate to the sensor's BAUD register code, or -1 if
  // unsupported. The manual's two tables disagree on what code 2 means
  // (the AT+UART command table lists 460800; the Modbus register table
  // lists 921600) -- this firmware talks to the sensor via the register
  // interface, so the register table's value is authoritative here.
  static int8_t BaudRegisterCode(uint32_t baud) {
    switch (baud) {
      case 9600:
        return 0;
      case 115200:
        return 1;
      case 921600:
        return 2;
      default:
        return -1;
    }
  }

  // Accumulates absolute angular movement (so it doesn't matter which way
  // the sensor is rotated, or if direction reverses briefly) until two full
  // turns have been covered, then ends calibration automatically. Also
  // enforces a timeout so a forgotten/failed rotation doesn't leave the
  // sensor stuck in calibration mode indefinitely.
  void AutoCalibrationTick(float heading_radians) {
    float heading_degrees = heading_radians * 180.0f / static_cast<float>(M_PI);
    if (!auto_rotation_started_) {
      auto_last_heading_degrees_ = heading_degrees;
      auto_rotation_started_ = true;
    } else {
      float delta = heading_degrees - auto_last_heading_degrees_;
      // Wrap into (-180, 180] so a crossing of the 0/360 boundary doesn't
      // register as a ~360-degree jump.
      delta = fmodf(delta + 540.0f, 360.0f) - 180.0f;
      auto_rotation_degrees_ += fabsf(delta);
      auto_last_heading_degrees_ = heading_degrees;
    }

    if (auto_rotation_degrees_ >= kAutoCalibrationTargetDegrees) {
      calibration_status_.store(WriteRegister(kRegCal, 0)
                                     ? CalibrationStatus::kDone
                                     : CalibrationStatus::kError);
      return;
    }
    if (millis() - auto_calibration_started_at_ms_ > kAutoCalibrationTimeoutMs) {
      WriteRegister(kRegCal, 0);  // Best-effort exit; report timeout either way.
      calibration_status_.store(CalibrationStatus::kTimedOut);
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

  // Writes a single holding register (Modbus function 0x06) and validates
  // that the sensor echoed the request back exactly, as the spec requires
  // for a normal response to this function.
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

    while (serial_->available()) {
      serial_->read();  // Discard anything stray before we start.
    }
    serial_->write(request, sizeof(request));

    uint8_t response[kWriteResponseLength];
    size_t received = 0;
    unsigned long deadline = millis() + kResponseTimeoutMs;
    while (received < kWriteResponseLength && millis() < deadline) {
      if (serial_->available()) {
        response[received++] = serial_->read();
      }
    }
    if (received < kWriteResponseLength) {
      return false;
    }
    for (size_t i = 0; i < kWriteResponseLength; i++) {
      if (response[i] != request[i]) {
        return false;
      }
    }
    return true;
  }

  // Reads a single holding register (Modbus function 0x03, count 1) --
  // used for VERSION, which unlike MAGX..YAW is a single register, not a
  // 4-register block.
  bool ReadSingleRegister(uint16_t reg, uint16_t* out_value) {
    uint8_t request[8] = {
        kDeviceId,
        0x03,
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF),
        0,
        1,
        0,
        0,
    };
    uint16_t crc = ModbusCrc16(request, 6);
    request[6] = static_cast<uint8_t>(crc & 0xFF);
    request[7] = static_cast<uint8_t>(crc >> 8);

    while (serial_->available()) {
      serial_->read();
    }
    serial_->write(request, sizeof(request));

    constexpr size_t kLength = 3 + 2 + 2;  // id+cmd+len + 1 register + CRC.
    uint8_t response[kLength];
    size_t received = 0;
    unsigned long deadline = millis() + kResponseTimeoutMs;
    while (received < kLength && millis() < deadline) {
      if (serial_->available()) {
        response[received++] = serial_->read();
      }
    }
    if (received < kLength) {
      return false;
    }
    if (response[0] != kDeviceId || response[1] != 0x03 ||
        response[2] != 2) {
      return false;
    }
    uint16_t received_crc =
        response[kLength - 2] | (response[kLength - 1] << 8);
    if (ModbusCrc16(response, kLength - 2) != received_crc) {
      return false;
    }
    *out_value = (response[3] << 8) | response[4];
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
  gpio_num_t rx_pin_;
  gpio_num_t tx_pin_;
  unsigned int poll_interval_ms_;

  std::atomic<CalibrationCommand> pending_command_{CalibrationCommand::kNone};
  std::atomic<CalibrationStatus> calibration_status_{CalibrationStatus::kIdle};

  // Settings hand-off; -1 means "no pending request". Only touched via
  // atomic ops, safe across the main/UI task and this reader's task.
  std::atomic<int32_t> pending_baud_{-1};
  std::atomic<int32_t> pending_filter_{-1};
  std::atomic<int32_t> pending_push_interval_{-1};

  std::atomic<uint16_t> hardware_version_{0};
  // Only touched from Run()'s task, no synchronization needed.
  bool version_read_ = false;

  // Auto-calibration rotation tracking; only touched from Run()'s task, no
  // synchronization needed.
  bool auto_rotation_started_ = false;
  float auto_last_heading_degrees_ = 0.0f;
  float auto_rotation_degrees_ = 0.0f;
  unsigned long auto_calibration_started_at_ms_ = 0;
};

}  // namespace wind_interface

#endif  // WIND_INTERFACE_SRC_HWT3100_HEADING_READER_H_
