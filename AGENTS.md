# AGENTS.md

## Project Overview

HALSER wind interface firmware — an ESP32-C3 firmware that bridges an LCJ Capteurs CV7 ultrasonic wind instrument to NMEA 2000 and Signal K networks via the HALSER board. Optionally also bridges a WitMotion HWT3100-TTL/232 fluxgate compass for magnetic heading. Also serves as a reference implementation for SensESP-based marine interface firmware.

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
      → Reference angle offset (LambdaTransform, software-only)
        → N2kWindDataSender (PGN 130306, 100ms interval)
          → tNMEA2000_esp32 (TWAI, GPIO 4 TX / GPIO 5 RX)
        → Signal K output (via WiFi/WebSocket)
        → SSD1306 OLED display (hostname, IP, uptime, AWS, AWA)

HWT3100 (Modbus RTU, 9600 bit/s, GPIO 21 RX / GPIO 20 TX, optional)
  → Hwt3100HeadingReader (dedicated FreeRTOS task, polls MAGX..YAW block)
    → N2kHeadingSender (PGN 127250, 100ms interval)
      → tNMEA2000_esp32 (TWAI, GPIO 4 TX / GPIO 5 RX)
    → Signal K output (navigation.headingMagnetic, via WiFi/WebSocket)

Web UI ←→ Reference angle transform (persisted to ESP32 filesystem only)
```

### Source Layout

**NMEA 2000 Output** (`src/sender/`):
- `n2k_senders.h` — `N2kWindDataSender` (PGN 130306) and `N2kHeadingSender` (PGN 127250), both at 100ms interval, use RepeatExpiring (5s timeout) to send N2kDoubleNA for stale data; both are also ValueProducers that emit on TX for downstream consumers

**Application** (`src/`):
- `main.cpp` — Entry point; initializes all components and wires the data pipeline, including the reference angle `LambdaTransform`
- `ssd1306_display.h/.cpp` — OLED display driver (hostname, IP, uptime, AWS, AWA; updates every 1 second)
- `hwt3100_heading_reader.h` — Modbus RTU master (dedicated FreeRTOS task) that polls the HWT3100 fluxgate compass for magnetic heading

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
