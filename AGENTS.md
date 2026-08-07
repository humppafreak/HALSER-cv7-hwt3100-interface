# AGENTS.md

## Project Overview

HALSER wind interface firmware — an ESP32-C3 firmware that bridges an LCJ Capteurs CV7 ultrasonic wind instrument to NMEA 2000, Signal K, and UDP NMEA 0183 broadcast, via the HALSER board. Optionally also bridges a WitMotion HWT3100-TTL/232 fluxgate compass for magnetic heading. Both inputs and all three outputs have independent, live-toggleable web UI enable/disable checkboxes. Also serves as a reference implementation for SensESP-based marine interface firmware.

## Build Commands

```bash
# Build firmware
pio run

# Upload to connected board
pio run -t upload

# Monitor serial output
pio device monitor
```

## Architecture

### Data Flow

```
CV7 (NMEA 0183, 4800 bit/s, GPIO 3 RX / GPIO 2 TX unused)
  → NMEA0183IOTask (dedicated FreeRTOS task)
    → WIMWVSentenceParser (apparent wind speed + angle, matches $IIMWV)
      → EnabledGate (Enable Wind Input, live-checked)
        → Reference angle offset (LambdaTransform, software-only)
          → N2kWindDataSender (PGN 130306, 100ms; checks Enable N2K live)
            → tNMEA2000_esp32 (TWAI, GPIO 4 TX / GPIO 5 RX)
          → EnabledGate (Enable Signal K) → Signal K output (via WiFi/WebSocket)
          → UdpNmea0183Sender ($IIMWV, checks Enable UDP live) → UDP broadcast :10110
          → SSD1306 OLED display (hostname, IP, uptime, AWS, AWA)

HWT3100 (AT commands + TTL UART, 9600 bit/s, GPIO 21 RX / GPIO 20 TX, optional)
  → Hwt3100HeadingReader (dedicated FreeRTOS task, parses streamed binary packets)
    → EnabledGate (Enable Heading Input, live-checked)
      → N2kHeadingSender (PGN 127250, 100ms; checks Enable N2K live)
        → tNMEA2000_esp32 (TWAI, GPIO 4 TX / GPIO 5 RX)
      → EnabledGate (Enable Signal K) → Signal K output (navigation.headingMagnetic)
      → UdpNmea0183Sender ($HCHDM, checks Enable UDP live) → UDP broadcast :10110
      → SSD1306 OLED display (heading)

Web UI ←→ Reference angle transform, 5 enable/disable toggles (persisted to ESP32 filesystem only)
```

All five toggles (2 inputs, 3 outputs) apply live — `EnabledGate` and the N2K/UDP senders' internal `output_enabled_config` both read their `CheckboxConfig` on every value/send cycle rather than caching at boot, so no restart is needed.

**Known limitation:** `NMEA0183IOTask` and `Hwt3100HeadingReader` each call `emit()` from their own FreeRTOS task directly into objects the main loop task also reads (e.g. `RepeatExpiring` in the N2K senders), with no mutex/queue between them — a genuine, accepted-tradeoff data race (single-core does not eliminate it, since FreeRTOS still preemptively interleaves tasks). See README.md's "Known Limitations" section and [issue #5](https://github.com/humppafreak/HALSER-cv7-hwt3100-interface/issues/5) for details.

### Source Layout

**NMEA 2000 Output** (`src/sender/`):
- `n2k_senders.h` — `N2kWindDataSender` (PGN 130306) and `N2kHeadingSender` (PGN 127250), both at 100ms interval, use RepeatExpiring (5s timeout) to send N2kDoubleNA for stale data; both are also ValueProducers that emit on TX for downstream consumers; both take an optional `CheckboxConfig*` (checked live each send cycle) to gate transmission entirely
- `udp_nmea0183_sender.h` — `UdpNmea0183Sender`: broadcasts synthesized `$IIMWV`/`$HCHDM` sentences on UDP port 10110 once per second, using plain `WiFiUDP`; same optional live `CheckboxConfig*` gating as the N2K senders

**Application** (`src/`):
- `main.cpp` — Entry point; initializes all components and wires the data pipeline, including the reference angle `LambdaTransform` and the 5 enable/disable toggles
- `ssd1306_display.h/.cpp` — OLED display driver (hostname, IP, uptime, AWS, AWA, HDG; updates every 1 second)
- `hwt3100_heading_reader.h` — AT-command/TTL reader (dedicated FreeRTOS task) that parses the HWT3100 fluxgate compass's streamed binary packets (WitMotion standard protocol, best-effort/unverified — see file header) for magnetic heading; also drives its on-sensor magnetic field calibration (`AT+CALI`) via `RequestCalibrationStart/Stop/AutoCalibration/ClearMagneticBias()`, and settings (`AT+UART`/`AT+FILT`/`AT+PRATE`) via `RequestSetBaudRate/Filter/PushInterval()` — cross-task hand-off for both via `std::atomic`
- `enabled_gate.h` — `EnabledGate<T>`, a pass-through `ValueConsumer`/`ValueProducer` that only forwards a value while its `CheckboxConfig` is checked (read live, not cached); used for the two input toggles and the Signal K output toggle, since `SKOutputFloat` is a third-party type with no internal enabled flag to add

### Hardware Pin Assignments

| Pin | Function |
|-----|----------|
| GPIO 2 | UART1 TX (unused — CV7 has no NMEA 0183 command channel) |
| GPIO 3 | UART1 RX (from CV7) |
| GPIO 4 | CAN TX |
| GPIO 5 | CAN RX |
| GPIO 6 | I2C SDA |
| GPIO 7 | I2C SCL |
| GPIO 8 | RGB LED (SK6805) |
| GPIO 9 | Button |
| GPIO 20 | UART0 TX (to HWT3100 RX, optional fluxgate compass) |
| GPIO 21 | UART0 RX (from HWT3100 TX, optional fluxgate compass) |

GPIO 20/21 are direct, non-isolated 3.3 V GPIO on the HALSER header — not the board's dedicated (isolated, level-shifted) NMEA 0183/RS-232/UART terminal block, whose receive side is a single hardware-muxed channel already claimed by the CV7. See README.md's "HWT3100 Fluxgate Compass" section for the reasoning.

### CV7 Protocol

NMEA 0183 at 4800 bit/s, 8N1, output every 0.5s, standard checksums included.

**Wind (talker `II`, relative reference):**
```
$IIMWV,225.0,R,000.0,N,A*38
```
Fields: wind angle (0.0–359.0°), reference (`R` = relative), wind speed, speed unit (`N`=knots, `M`=m/s, `K`=km/h), status (`A`=available, `V`=alarm). Parsed by SensESP's built-in `WIMWVSentenceParser`, which matches the `MWV` formatter regardless of talker ID — no code changes were needed to consume this sentence.

**Temperature (talker `WI`), not currently consumed by this firmware:**
```
$WIXDR,C,022.0,C,,*52
```

**`$PLCJ,...` / `$PLCJEA...` sentences** are an undocumented LCJ Capteurs technical-service protocol, not a public configuration interface — this firmware does not use them. Unlike the Autonnic A5120 this codebase originally targeted (which accepted `$PATC,IIMWV,AHD/DWD/DSP/TXP` commands with `$PATC,WIMWV,ACK` responses), the CV7 has no NMEA 0183 command channel: the reference angle offset is applied entirely in software (a `LambdaTransform` in `main.cpp`) rather than sent to the instrument, and there is no equivalent for direction/speed damping or message repetition rate (the CV7's 0.5s output interval is fixed).

### HWT3100 Protocol

Not NMEA 0183. AT commands over TTL UART at 9600 bit/s (default). The sensor's manual describes a Modbus RTU mode as an alternative, but **real hardware testing found this TTL variant does not actually support it**, so this firmware uses AT commands exclusively — plain ASCII text, `"AT+NAME=value\r\n"`, acknowledged with a short line such as `"OK"` or `"ERROR"`.

There is no documented on-demand query command, so heading comes from the sensor's continuous streaming output, enabled via `AT+PRATE=<ms>` (default 200) at boot. The manual does not document that stream's byte format, so this firmware assumes WitMotion's own published standard binary packet protocol (shared across WitMotion's sensor catalog and used by their official SDKs): `0x55, TYPE, D1L, D1H, D2L, D2H, D3L, D3H, D4L, D4H, SUM` (11 bytes, little-endian 16-bit words, `SUM` = low byte of the sum of the preceding 10 bytes). Only the Angle packet (`TYPE` = `0x53`) is used, taking its third data word as Yaw: `heading_degrees = int16(YawL, YawH) / 32768 * 180`. **This is an explicitly best-effort, unverified guess** — the unit available during development was defective, so this format has not been confirmed against real hardware. The parser self-resyncs on a checksum mismatch by shifting the buffer one byte and re-scanning for the next `0x55` header. See the file header comment in `src/hwt3100_heading_reader.h` for the full reasoning.

Calibration uses `AT+CALI=`: `1` enters magnetic field calibration, `0` exits it, `2` clears the existing offset.

Settings are exposed as web UI configuration items, applied live: `AT+UART=` (values 0/1/2 = 9600/115200/460800, per the manual's AT-command table), `AT+FILT=` (smoothing filter), `AT+PRATE=` (streaming interval in ms — this is now the sole heading-update mechanism, not an optional extra; 0 pauses updates entirely). There is no AT command to query hardware/firmware version, so the version display that existed under the earlier Modbus-based implementation has been removed.

Changing `AT+UART=` requires reconfiguring this device's own UART to match immediately after a successfully acked write, since the ack is necessarily sent at the pre-change baud and the sensor is assumed to switch right after — see `Hwt3100HeadingReader::HandlePendingSettings()`. Unverified against real hardware, like everything else HWT3100-related in this codebase; a failed assumption here means losing contact with the sensor until it's power-cycled back to its 9600 default.

### NMEA 2000 PGNs

| PGN | Description | Interval |
|-----|-------------|----------|
| 130306 | Wind Data (apparent wind speed + angle) | 100ms |
| 127250 | Vessel Heading (magnetic, from HWT3100 if connected) | 100ms |

## Dependencies

- SensESP ^3.4.0 — IoT framework (WiFi, web UI, Signal K)
- SensESP/NMEA0183 — NMEA 0183 sentence parsing
- NMEA2000-library v4.17.2 — NMEA 2000 message handling
- NMEA2000_twai — ESP32 TWAI (CAN) driver
- Adafruit SSD1306 v2.5.1 — OLED display
- elapsedMillis v1.0.6 — Timing utilities
- esp_websocket_client — WebSocket support (Espressif component)
