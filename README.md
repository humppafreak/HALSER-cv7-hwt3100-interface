# HALSER Wind Interface

ESP32-C3 firmware for the [HALSER](https://shop.hatlabs.fi/products/halser) board that bridges an **LCJ Capteurs CV7** ultrasonic wind instrument to NMEA 2000 and Signal K networks.

This firmware serves as both a ready-to-use application and a reference example for building custom SensESP-based marine interface firmware.

## Features

- Receives apparent wind data (speed and angle) from the CV7 via NMEA 0183 `$IIMWV` sentences at 4800 bit/s
- Transmits wind data as NMEA 2000 PGN 130306 (Wind Data) at 100ms intervals
- Outputs wind data to Signal K via WiFi/WebSocket
- Configurable reference angle offset via web UI (wind vane alignment, applied entirely in software)
- OLED display showing hostname, IP, uptime, wind speed, and wind angle
- RGB LED activity indicator
- OTA firmware updates
- NMEA 2000 watchdog with configurable auto-reboot

## Hardware Required

- [HALSER](https://shop.hatlabs.fi/products/halser) board
- [LCJ Capteurs CV7](https://lcjcapteurs.com/) ultrasonic wind instrument
- NMEA 2000 network connection
- Optional: SSD1306 128x64 OLED display (I2C)

## Wiring

| HALSER Pin | Function |
|------------|----------|
| GPIO 2 | UART1 TX (unused — CV7 has no NMEA 0183 command channel) |
| GPIO 3 | UART1 RX ← CV7 TX |
| GPIO 4 | CAN TX → NMEA 2000 |
| GPIO 5 | CAN RX ← NMEA 2000 |
| GPIO 6 | I2C SDA (OLED display) |
| GPIO 7 | I2C SCL (OLED display) |
| GPIO 8 | RGB LED (SK6805) |
| GPIO 9 | Button |

The CV7 communicates via NMEA 0183 at 4800 bit/s (8N1), transmit-only — it accepts no configuration commands over the serial link.

## Usage

### Initial Setup

1. Flash the firmware to the HALSER board
2. The device creates a WiFi access point on first boot
3. Connect to the AP and configure your WiFi network credentials
4. Access the web UI at `http://wind.local`

### Configuring Reference Angle

Navigate to the **Reference Angle** section in the web UI. Enter the angle readout (in degrees) when the wind vane is pointing straight ahead. This offset corrects for misalignment between the wind instrument and the vessel's heading.

Unlike some wind instruments, the CV7 exposes no NMEA 0183 command to apply this offset in the instrument itself, so it is applied entirely in software (via a `LambdaTransform` in `main.cpp`) to the parsed wind angle, and persisted only to the ESP32 filesystem.

### OTA Firmware Updates

The firmware calls `enable_ota(...)` (SensESP's ArduinoOTA integration), which accepts pushed updates over WiFi — it does not pull updates from a URL itself.

Every tagged release (`v*`) is built by [`.github/workflows/release-firmware.yml`](.github/workflows/release-firmware.yml) and published as a downloadable `.bin` on the repository's [Releases](../../releases) page. To flash a downloaded release onto a device that's already running this firmware:

```bash
# Using PlatformIO (uses the OTA password set in main.cpp's enable_ota() call)
pio run -t upload --upload-port <device-ip> --upload-flags="--auth=thisisfine"

# Or using espota.py directly
python espota.py -i <device-ip> -a thisisfine -f HALSER-cv7-wind-interface-<version>.bin
```

Note that the published binary is the application image only (no bootloader/partition table), so it's only valid for OTA onto a device already running compatible firmware — first-time programming still requires a wired `pio run -t upload`. Non-tagged builds (e.g. `workflow_dispatch` runs) are uploaded as workflow artifacts instead of releases.

### Signal K Integration

Wind data is emitted to Signal K as:

- `environment.wind.speedApparent` — apparent wind speed in m/s
- `environment.wind.angleApparent` — apparent wind angle in radians

### NMEA 2000 Watchdog

An optional watchdog can be enabled in the web UI under **Enable NMEA 2000 Watchdog**. When enabled, the device reboots if no NMEA 2000 messages are received for two minutes. This helps recover from CAN bus lockups. The setting requires a device restart to take effect.

### NMEA 2000 Stale Data Handling

The N2K sender uses `RepeatExpiring` to handle stale wind data. If no new wind measurement arrives within 5 seconds, the sender transmits `N2kDoubleNA` ("not available") values instead of repeating stale data. PGN 130306 messages continue at 100ms regardless — downstream devices always see a consistent message rate and can distinguish "no data" from silence.

### OLED Display

If connected, a 128x64 SSD1306 OLED display shows:
- Device hostname
- WiFi IP address
- Uptime in seconds
- *(blank line)*
- Apparent wind speed (m/s)
- Apparent wind angle (degrees, -180 to +180)

## Architecture

```
CV7 (NMEA 0183, 4800 bit/s)
  │
  │ UART1 (GPIO 3 RX only)
  ▼
NMEA0183IOTask
  └── WIMWVSentenceParser (apparent wind speed + angle, matches $IIMWV)
        ├── Reference angle offset (LambdaTransform, software-only)
        │     ├── N2kWindDataSender → NMEA 2000 bus (TWAI, GPIO 4/5)
        │     ├── SKOutputFloat     → Signal K server (angle)
        │     └── InfoDisplay       → OLED (angle)
        ├── SKOutputFloat           → Signal K server (speed)
        └── InfoDisplay             → OLED (speed)

Web UI (SensESP) ──── Reference angle transform ──── Filesystem (persistent storage)
```

The firmware is built on [SensESP](https://github.com/SignalK/SensESP), which provides WiFi connectivity, a web UI for configuration, Signal K protocol support, and OTA updates.

### Key Design Patterns

**Producer/Consumer pipeline:** SensESP uses a reactive pipeline where producers emit values that flow to connected consumers. The WIMWV parser produces apparent wind speed and angle values consumed (via the reference angle transform, for angle) by the N2K sender, Signal K outputs, and OLED display. Producers and consumers are connected via `connect_to()`.

**Software-only calibration:** The reference angle offset is a `LambdaTransform` with a single configurable parameter, persisted to the ESP32 filesystem. Because the CV7 has no NMEA 0183 command channel, there is no device-side equivalent to keep in sync — unlike instruments (such as the Autonnic A5120 this codebase originally targeted) that accept serial configuration commands.

**NMEA 2000 value expiry:** The `N2kWindDataSender` wraps inputs in `RepeatExpiring<double>`, which returns `N2kDoubleNA` when the source value is older than 5 seconds. This prevents stale wind data from being transmitted as valid measurements while maintaining the 100ms PGN 130306 transmission rate.

## Code Structure

### NMEA 2000 Output (`src/sender/`)

| File | Purpose |
|------|---------|
| `n2k_senders.h` | `N2kWindDataSender` — PGN 130306 at 100ms with `RepeatExpiring` for stale data |

### Application

| File | Purpose |
|------|---------|
| `main.cpp` | Entry point — wires all components together, including the reference angle `LambdaTransform` |
| `ssd1306_display.h/.cpp` | OLED display driver (hostname, IP, uptime, AWS, AWA) |

### CV7 Protocol

The CV7 outputs standard NMEA 0183 sentences at 4800 bit/s, every 0.5s, with standard checksums:

| Sentence | Description |
|----------|-------------|
| `$IIMWV,<angle>,R,<speed>,<unit>,<status>` | Apparent wind angle + speed, relative reference |
| `$WIXDR,C,<temp>,C,,` | Wind sensor temperature (not currently consumed by this firmware) |

SensESP's built-in `WIMWVSentenceParser` matches the `MWV` formatter regardless of talker ID, so it parses the CV7's `$IIMWV` sentences unmodified.

The CV7 also emits undocumented `$PLCJ,...` / `$PLCJEA...` sentences described only as "for LCJ Capteurs technical service" — this firmware does not use them, and the CV7 has no public NMEA 0183 command interface for configuration (unlike the Autonnic A5120, which accepted `$PATC,IIMWV,AHD/DWD/DSP/TXP` commands with ACK confirmation).

### NMEA 2000 Device Identity

| Field | Value |
|-------|-------|
| Device function | 130 (Weather Instruments) |
| Device class | 85 (Sensor Communication Interface) |
| Manufacturer code | 2046 |
| Transmitted PGN | 130306 (Wind Data) |
| Wind reference | Apparent |

## Building

Requires [PlatformIO](https://platformio.org/).

```bash
# Build firmware
pio run

# Upload to connected board
pio run -t upload

# Monitor serial output
pio device monitor
```

## Testing

This repository does not yet have automated tests. Contributions are welcome.

## Using as a Template

This firmware demonstrates several patterns useful for building custom SensESP marine interfaces:

1. **NMEA 0183 sentence parsing** — Using SensESP's built-in `WIMWVSentenceParser` for standard sentences
2. **Software-only calibration** — A `LambdaTransform` with persisted config for devices with no command channel
3. **NMEA 2000 output with value expiry** — `RepeatExpiring` prevents stale data transmission while maintaining constant PGN rate
4. **Producer/Consumer pipeline** — Connecting a single data source (wind parser) to multiple sinks (N2K, Signal K, OLED)

To adapt this for a different device:
- Modify or replace `WIMWVSentenceParser` if your device uses different NMEA 0183 sentences
- If your device accepts NMEA 0183 configuration commands, replace the `LambdaTransform`-based reference angle with a command/ACK pattern (persist to filesystem, send the command, wait on a `SemaphoreValue` for the response)
- Update the N2K sender for your target PGNs
- Adjust pin assignments and bit rate in `main.cpp`

## License

See [LICENSE](LICENSE) for details.
