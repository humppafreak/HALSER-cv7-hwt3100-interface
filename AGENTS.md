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

HWT3100 (Modbus RTU, 9600 bit/s, GPIO 21 RX / GPIO 20 TX, optional)
  → Hwt3100HeadingReader (dedicated FreeRTOS task, polls MAGX..YAW block)
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
- `hwt3100_heading_reader.h` — Modbus RTU master (dedicated FreeRTOS task) that polls the HWT3100 fluxgate compass for magnetic heading; also drives its on-sensor magnetic field calibration (register `0xD9`, CAL) via `RequestCalibrationStart/Stop/AutoCalibration/ClearMagneticBias()`, and settings (`BAUD`/`FILT`/`MRATE`) via `RequestSetBaudRate/Filter/PushInterval()` plus a read-once `GetHardwareVersion()` — cross-task hand-off for both via `std::atomic`
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

Not NMEA 0183. Modbus RTU at 9600 bit/s (default), device ID `0x00`. This firmware sends `AT+MODE=1` once on boot (accepted regardless of the sensor's current mode, per its manual) to force Modbus mode, then polls as a Modbus RTU master every 200ms, reading the 4-register block at `0xDB` (MAGX, MAGY, MAGZ, YAW — the sensor only exposes this as a contiguous block, not individually addressable registers) and using only YAW (heading, signed int16, 0.1° units, register `0xDE`). Request/response framing (including a byte-for-byte manual example used to confirm the CRC16 variant and byte order) is in `src/hwt3100_heading_reader.h`.

Calibration uses Modbus function `0x06` (write single register) against `CAL` (register `0xD9`): `1` enters magnetic field calibration, `0` exits it, `2` clears the existing offset. The expected response is an exact echo of the request, per the Modbus spec for that function; `Hwt3100HeadingReader::WriteRegister()` validates it byte-for-byte.

Settings (also function `0x06`) are exposed as web UI configuration items, applied live: `BAUD` (`0xD2`, values 0/1/2 = 9600/115200/921600 — note the manual's AT-command table lists 460800 for code 2 where the register table lists 921600; the register table is authoritative since this firmware uses the register interface), `FILT` (`0xD8`, smoothing filter), `MRATE` (`0xDA`, active push interval — this firmware always polls request/response and never consumes the sensor's own push frames, so a nonzero value here can desync the read cycle). `VERSION` (`0xD0`, read-only, function `0x03` count 1) is read once at boot, retried each poll cycle until it succeeds, and shown as-is with no documented decoding.

Changing `BAUD` requires reconfiguring this device's own UART to match immediately after a successfully acked write, since the ack is necessarily sent at the pre-change baud and the sensor is assumed to switch right after — see `Hwt3100HeadingReader::HandlePendingSettings()`. Unverified against real hardware, like everything else in this codebase; a failed assumption here means losing contact with the sensor until it's power-cycled back to its 9600 default.

The sensor also has an ASCII/AT-command mode (factory default) with a continuous streaming output, but its exact line format isn't documented byte-for-byte in the manual, so this firmware doesn't use it.

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
