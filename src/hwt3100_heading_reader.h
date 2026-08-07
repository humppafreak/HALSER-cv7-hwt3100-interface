// Polls a WitMotion HWT3100-TTL/232 fluxgate compass over its TTL UART and
// emits magnetic heading in radians. Also drives the sensor's on-board
// magnetic field calibration and a handful of settings via AT commands.
//
// This TTL-variant unit does not support Modbus mode -- confirmed against
// real hardware, even though the sensor's manual describes a Modbus mode
// as an option (an earlier version of this file used it; see git history
// if that's ever useful again for a different HWT3100 variant). This
// firmware therefore uses the sensor's other documented mode exclusively:
// AT commands, sent as plain ASCII text ("AT+<NAME>=<value>\r\n", replied
// to with a short ASCII line such as "OK" or "ERROR").
//
// The AT command SET (mode/baud/calibration/filter/push-rate) is
// documented byte-for-byte in the manual. What the manual does NOT
// document is the byte format of the continuous data stream the
// AT+PRATE command turns on. Rather than guess at an undocumented
// format from nothing, this implementation assumes the stream uses
// WitMotion's own published standard binary packet protocol
// (https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol),
// which is shared across WitMotion's whole sensor catalog and is what
// every WitMotion Arduino/ESP32/STM32/Python SDK parses:
//   0x55, TYPE, D1L, D1H, D2L, D2H, D3L, D3H, D4L, D4H, SUM   (11 bytes)
// little-endian 16-bit data words, SUM = low byte of the sum of the
// preceding 10 bytes. Only the Angle packet (TYPE 0x53) is used here,
// for YAW (the third data word): degrees = int16(YawL, YawH) / 32768 * 180.
//
// THIS IS UNVERIFIED against the actual HWT3100: the unit available
// during development turned out to be defective, so none of this -- the
// AT command set as applied here, the assumed binary packet format, or
// the calibration/settings behavior -- has been confirmed against real
// hardware. Treat this file as a best-effort implementation to validate
// (and likely correct) once working hardware is available, particularly
// HandleAnglePacket()'s packet layout assumption and SendAtCommand()'s
// assumption that the sensor doesn't interleave binary stream bytes with
// an AT command's reply line.
//
// Calibration and settings behave as before this rewrite (see the
// class's public methods): requests are handed off from the main/UI task
// to this class's own FreeRTOS task via std::atomic, since the UART is
// exclusively owned by that task; status is reported back the same way.
// Hardware version display has been removed -- the AT command set has no
// documented way to query it (that was only available via the Modbus
// VERSION register, which this variant doesn't support).

#ifndef WIND_INTERFACE_SRC_HWT3100_HEADING_READER_H_
#define WIND_INTERFACE_SRC_HWT3100_HEADING_READER_H_

#include <Arduino.h>

#include <atomic>
#include <cmath>
#include <cstring>

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
  // rx_pin/tx_pin are needed for RequestSetBaudRate()'s live serial_->begin()
  // call after a successful baud change; the caller must already have
  // called serial->begin(...) with these same pins once before
  // constructing this reader. push_interval_ms sets the sensor's
  // continuous AT+PRATE streaming rate at startup -- see
  // RequestSetPushInterval() for what changing it later means.
  Hwt3100HeadingReader(HardwareSerial* serial, gpio_num_t rx_pin,
                        gpio_num_t tx_pin, unsigned int push_interval_ms = 200)
      : serial_{serial},
        rx_pin_{rx_pin},
        tx_pin_{tx_pin},
        push_interval_ms_{push_interval_ms} {
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
  // commands above. RequestSetBaudRate() additionally maps the given baud
  // to the sensor's AT+UART code and, only once the sensor acks, reconfigures
  // this object's own serial connection to match -- see the file comment
  // above for the risk that implies.
  void RequestSetBaudRate(uint32_t baud) {
    pending_baud_.store(static_cast<int32_t>(baud));
  }
  void RequestSetFilter(uint16_t filter) {
    pending_filter_.store(static_cast<int32_t>(filter));
  }
  // Sets the sensor's own AT+PRATE continuous-streaming interval, which is
  // also this firmware's effective heading update rate -- there is no
  // separate request/response polling mode available over the AT command
  // interface. 0 ("single return") pauses continuous updates entirely
  // until this is changed again or the device is rebooted; avoid it
  // unless that's actually intended.
  void RequestSetPushInterval(uint16_t interval_ms) {
    pending_push_interval_.store(static_cast<int32_t>(interval_ms));
  }

 private:
  static constexpr unsigned long kAtResponseTimeoutMs = 200;
  static constexpr size_t kPacketLength = 11;
  static constexpr uint8_t kFrameHeader = 0x55;
  static constexpr uint8_t kPacketTypeAngle = 0x53;
  // Two full rotations, per the manual's calibration procedure.
  static constexpr float kAutoCalibrationTargetDegrees = 720.0f;
  // Safety net so Auto can't get stuck in kAutoCalibrating forever if the
  // sensor is never actually rotated.
  static constexpr unsigned long kAutoCalibrationTimeoutMs = 240000;

  static void TaskEntry(void* param) {
    static_cast<Hwt3100HeadingReader*>(param)->Run();
  }

  [[noreturn]] void Run() {
    SetPushInterval(push_interval_ms_);

    while (true) {
      HandlePendingCommand();
      HandlePendingSettings();

      while (serial_->available()) {
        uint8_t b = serial_->read();
        if (packet_len_ == 0) {
          if (b == kFrameHeader) {
            packet_[packet_len_++] = b;
          }
          continue;
        }
        packet_[packet_len_++] = b;
        if (packet_len_ < kPacketLength) {
          continue;
        }
        // Full candidate packet buffered; validate the checksum.
        uint8_t sum = 0;
        for (size_t i = 0; i < kPacketLength - 1; i++) {
          sum += packet_[i];
        }
        if (sum == packet_[kPacketLength - 1]) {
          if (packet_[1] == kPacketTypeAngle) {
            HandleAnglePacket();
          }
          packet_len_ = 0;
        } else {
          // Resync: the assumed header may have been a stray byte that
          // just happened to match 0x55. Drop it and re-scan the rest of
          // what's buffered for the next 0x55, rather than discarding the
          // whole window -- keeps a false-positive header from costing
          // more than one byte of resync.
          std::memmove(packet_, packet_ + 1, kPacketLength - 1);
          packet_len_ = kPacketLength - 1;
          while (packet_len_ > 0 && packet_[0] != kFrameHeader) {
            std::memmove(packet_, packet_ + 1, packet_len_ - 1);
            packet_len_--;
          }
        }
      }
      delay(5);
    }
  }

  // packet_[0..10] is a validated Angle packet at this point:
  // 0x55, 0x53, RollL, RollH, PitchL, PitchH, YawL, YawH, VL, VH, SUM.
  void HandleAnglePacket() {
    int16_t yaw_raw = static_cast<int16_t>(packet_[6] | (packet_[7] << 8));
    float heading_degrees = yaw_raw / 32768.0f * 180.0f;
    float heading = heading_degrees * static_cast<float>(M_PI) / 180.0f;
    if (heading < 0) {
      heading += 2 * static_cast<float>(M_PI);
    }

    CalibrationStatus status = calibration_status_.load();
    if (status == CalibrationStatus::kAutoCalibrating) {
      AutoCalibrationTick(heading);
    }
    // Suppress emitting to downstream consumers (N2K/Signal K/UDP/OLED)
    // while a calibration is actively in progress -- the sensor's output
    // is unreliable mid-calibration, so a transient reading shouldn't
    // reach anything treating it as a real heading.
    bool actively_calibrating =
        status == CalibrationStatus::kCalibrating ||
        status == CalibrationStatus::kAutoCalibrating;
    if (!actively_calibrating) {
      this->emit(heading);
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
        calibration_status_.store(SendAtCommand("AT+CALI=1")
                                       ? CalibrationStatus::kCalibrating
                                       : CalibrationStatus::kError);
        break;
      case CalibrationCommand::kStop:
        calibration_status_.store(SendAtCommand("AT+CALI=0")
                                       ? CalibrationStatus::kDone
                                       : CalibrationStatus::kError);
        break;
      case CalibrationCommand::kAutoStart:
        if (SendAtCommand("AT+CALI=1")) {
          auto_rotation_started_ = false;
          auto_rotation_degrees_ = 0.0f;
          auto_calibration_started_at_ms_ = millis();
          calibration_status_.store(CalibrationStatus::kAutoCalibrating);
        } else {
          calibration_status_.store(CalibrationStatus::kError);
        }
        break;
      case CalibrationCommand::kClearBias:
        calibration_status_.store(SendAtCommand("AT+CALI=2")
                                       ? CalibrationStatus::kBiasCleared
                                       : CalibrationStatus::kError);
        break;
      case CalibrationCommand::kNone:
        break;
    }
  }

  void HandlePendingSettings() {
    int32_t requested_baud = pending_baud_.exchange(-1);
    if (requested_baud >= 0) {
      int8_t code = BaudCommandCode(static_cast<uint32_t>(requested_baud));
      if (code >= 0) {
        char command[20];
        snprintf(command, sizeof(command), "AT+UART=%d", code);
        if (SendAtCommand(command)) {
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
      }
      // On an unsupported baud or a failed/unacked command, nothing is
      // changed; the request is simply dropped rather than retried, same
      // as filter/push-interval below.
    }

    int32_t requested_filter = pending_filter_.exchange(-1);
    if (requested_filter >= 0) {
      char command[20];
      snprintf(command, sizeof(command), "AT+FILT=%d",
                static_cast<int>(requested_filter));
      SendAtCommand(command);
    }

    int32_t requested_interval = pending_push_interval_.exchange(-1);
    if (requested_interval >= 0) {
      SetPushInterval(static_cast<uint16_t>(requested_interval));
    }
  }

  void SetPushInterval(uint16_t interval_ms) {
    char command[20];
    snprintf(command, sizeof(command), "AT+PRATE=%d",
              static_cast<int>(interval_ms));
    SendAtCommand(command);
  }

  // Maps a baud rate to the AT+UART command code, or -1 if unsupported.
  static int8_t BaudCommandCode(uint32_t baud) {
    switch (baud) {
      case 9600:
        return 0;
      case 115200:
        return 1;
      case 460800:
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
    float heading_degrees =
        heading_radians * 180.0f / static_cast<float>(M_PI);
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
      calibration_status_.store(SendAtCommand("AT+CALI=0")
                                     ? CalibrationStatus::kDone
                                     : CalibrationStatus::kError);
      return;
    }
    if (millis() - auto_calibration_started_at_ms_ >
        kAutoCalibrationTimeoutMs) {
      SendAtCommand("AT+CALI=0");  // Best-effort exit either way.
      calibration_status_.store(CalibrationStatus::kTimedOut);
    }
  }

  // Sends a plain-text AT command and waits for a one-line reply,
  // considering anything other than a timeout or a literal "ERROR" a
  // success (the manual's replies vary by command -- "OK", "Calibrating",
  // "Calibration completed", etc. -- so this doesn't match exact text).
  // Flushes stray bytes before sending, and always resets the angle-packet
  // parser's buffer before returning, since bytes consumed here while
  // waiting for the reply may have been mid-packet binary stream data if
  // the sensor doesn't pause its own data stream while handling an AT
  // command -- unverified either way.
  bool SendAtCommand(const String& command) {
    while (serial_->available()) {
      serial_->read();
    }
    serial_->print(command);
    serial_->print("\r\n");

    String reply;
    unsigned long deadline = millis() + kAtResponseTimeoutMs;
    while (millis() < deadline) {
      if (serial_->available()) {
        char c = static_cast<char>(serial_->read());
        if (c == '\n') {
          if (reply.length() > 0) {
            break;
          }
        } else if (c != '\r') {
          reply += c;
        }
      }
    }
    packet_len_ = 0;
    reply.trim();
    return reply.length() > 0 && reply.indexOf("ERROR") == -1;
  }

  HardwareSerial* serial_;
  gpio_num_t rx_pin_;
  gpio_num_t tx_pin_;
  unsigned int push_interval_ms_;

  // Angle-packet parser state; only touched from Run()'s task.
  uint8_t packet_[kPacketLength];
  size_t packet_len_ = 0;

  std::atomic<CalibrationCommand> pending_command_{CalibrationCommand::kNone};
  std::atomic<CalibrationStatus> calibration_status_{CalibrationStatus::kIdle};

  // Settings hand-off; -1 means "no pending request". Only touched via
  // atomic ops, safe across the main/UI task and this reader's task.
  std::atomic<int32_t> pending_baud_{-1};
  std::atomic<int32_t> pending_filter_{-1};
  std::atomic<int32_t> pending_push_interval_{-1};

  // Auto-calibration rotation tracking; only touched from Run()'s task.
  bool auto_rotation_started_ = false;
  float auto_last_heading_degrees_ = 0.0f;
  float auto_rotation_degrees_ = 0.0f;
  unsigned long auto_calibration_started_at_ms_ = 0;
};

}  // namespace wind_interface

#endif  // WIND_INTERFACE_SRC_HWT3100_HEADING_READER_H_
