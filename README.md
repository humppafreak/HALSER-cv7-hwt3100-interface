# HALSER Wind Interface

ESP32-C3 firmware for the [HALSER](https://shop.hatlabs.fi/products/halser) board that bridges an **LCJ Capteurs CV7** ultrasonic wind instrument to NMEA 2000 and Signal K networks.

This firmware serves as both a ready-to-use application and a reference example for building custom SensESP-based marine interface firmware.

## Features

- Receives apparent wind data (speed and angle) from the CV7 via NMEA 0183 `$IIMWV` sentences at 4800 bit/s
- Transmits wind data as NMEA 2000 PGN 130306 (Wind Data) at 100ms intervals
- Outputs wind data to Signal K via WiFi/WebSocket
- Configurable reference angle offset via web UI (wind vane alignment, applied entirely in software)
- Optional HWT3100 fluxgate compass support: polls magnetic heading over Modbus RTU, transmits NMEA 2000 PGN 127250 (Vessel Heading) and Signal K `navigation.headingMagnetic`
- UDP NMEA 0183 broadcast output (port 10110) — synthesized `$IIMWV`/`$HCHDM` sentences for chartplotter apps (e.g. OpenCPN) that consume NMEA 0183 directly over WiFi, no Signal K server required
- Independent, live web UI toggles for both inputs (CV7 wind, HWT3100 heading) and all three outputs (Signal K, NMEA 2000, UDP) — no restart needed to apply
- OLED display showing hostname, IP, uptime, wind speed, wind angle, and (if the HWT3100 is connected) magnetic heading
- RGB LED activity indicator
- OTA firmware updates
- NMEA 2000 watchdog with configurable auto-reboot

## Hardware Required

- [HALSER](https://shop.hatlabs.fi/products/halser) board
- [LCJ Capteurs CV7](https://lcjcapteurs.com/) ultrasonic wind instrument
- NMEA 2000 network connection
- Optional: SSD1306 128x64 OLED display (I2C)
- Optional: WitMotion HWT3100-TTL/232 fluxgate compass

## Wiring

### CV7 (Required)

The CV7's 4-wire cable connects to HALSER's **NMEA 0183 RX** 3-pin terminal block (not a raw GPIO pin — it's the isolated, level-shifted RS-485 receiver on the board):

| CV7 Wire | Function | Connect To |
|----------|----------|------------|
| Yellow | NMEA TX + | NMEA 0183 RX terminal block, pin **A** |
| Green | NMEA TX − | NMEA 0183 RX terminal block, pin **B** |
| Blue | − Power | NMEA 0183 RX terminal block, pin **GND**, *and* your power supply's negative/return |
| Red | + Power (8–33 VDC) | A suitable 8–33 VDC supply — e.g. HALSER's **Vin** connector (5–32 V, sourced from the NMEA 2000 bus), which is within the CV7's range as long as system voltage is ≥ 8 V |

Then set HALSER's **RX SEL** jumper to **N** (NMEA 0183 / RS-485) so the RS-485 receiver is the one actually wired through to the ESP32-C3.

!!! tip "No data coming through?"
    Try swapping the A/B (Yellow/Green) connections — RS-485 polarity varies by convention between manufacturers, and reversing it is safe.

The CV7 is transmit-only (8N1, 4800 bit/s) — it has no NMEA 0183 command channel, so there's nothing to wire for a return/command path. Internally, this signal reaches the ESP32-C3 on UART1 (GPIO 3 RX; GPIO 2 TX is unused).

### HWT3100 Fluxgate Compass (Optional)

An optional WitMotion HWT3100-TTL/232 fluxgate compass can be connected for magnetic heading. It does **not** go on HALSER's dedicated NMEA 0183/RS-232/UART terminal block — that block's receive side is a single hardware-muxed channel selected by the **RX SEL** jumper, and the CV7 already occupies it (set to **N**, above). Instead, wire the HWT3100 directly to the 4 pins on HALSER's **GPIO** header labeled `20`, `21`, `GND`, and to the **Vin** connector for power:

| HWT3100 Wire | Connect To |
|--------------|------------|
| VCC (red) | **Vin** connector (5–36 V, matches the HWT3100's input range) |
| TX (yellow) | GPIO header, pin **21** (UART0 RX) |
| RX (green) | GPIO header, pin **20** (UART0 TX) |
| GND (black) | GPIO header, pin **GND** |

!!! note
    GPIO 20/21 are direct, non-isolated 3.3 V ESP32-C3 logic — unlike the board's dedicated serial connectors, this path has no galvanic isolation or level shifting. Confirm the HWT3100's TTL logic levels are 3.3 V-tolerant before wiring, and keep this cable run short.

The HWT3100 does not speak NMEA 0183; it uses a proprietary AT-command/Modbus RTU protocol at 9600 bit/s (default). This firmware switches it into Modbus mode on boot and polls it as a Modbus RTU master — see [`src/hwt3100_heading_reader.h`](src/hwt3100_heading_reader.h) for details. Heading is transmitted as NMEA 2000 PGN 127250 (Vessel Heading, magnetic reference) and Signal K `navigation.headingMagnetic`.

### Internal Pin Reference

The terminal blocks and headers above are already routed to these ESP32-C3 pins on the board; you don't wire to them directly except where noted (GPIO 20/21/GND on the GPIO header):

| HALSER Pin | Function |
|------------|----------|
| GPIO 2 | UART1 TX (unused — CV7 has no NMEA 0183 command channel) |
| GPIO 3 | UART1 RX ← CV7 TX (via the NMEA 0183 RX terminal block, RX SEL = N) |
| GPIO 4 | CAN TX → NMEA 2000 |
| GPIO 5 | CAN RX ← NMEA 2000 |
| GPIO 6 | I2C SDA (OLED display) |
| GPIO 7 | I2C SCL (OLED display) |
| GPIO 8 | RGB LED (SK6805) |
| GPIO 9 | Button |
| GPIO 20 | UART0 TX → HWT3100 RX (optional fluxgate compass, direct GPIO header) |
| GPIO 21 | UART0 RX ← HWT3100 TX (optional fluxgate compass, direct GPIO header) |

## Usage

### Initial Setup

1. Flash the firmware to the HALSER board
2. The device creates a WiFi access point on first boot
3. Connect to the AP and configure your WiFi network credentials
4. Access the web UI at `http://wind.local`

### Configuring Reference Angle

Navigate to the **Reference Angle** section in the web UI. Enter the angle readout (in degrees) when the wind vane is pointing straight ahead. This offset corrects for misalignment between the wind instrument and the vessel's heading.

Unlike some wind instruments, the CV7 exposes no NMEA 0183 command to apply this offset in the instrument itself, so it is applied entirely in software (via a `LambdaTransform` in `main.cpp`) to the parsed wind angle, and persisted only to the ESP32 filesystem.

### Enabling/Disabling Inputs and Outputs

The web UI has independent checkboxes for both inputs and all three outputs:

| Toggle | Effect while unchecked |
|--------|------------------------|
| Enable Wind Input (CV7) | Wind data isn't forwarded to Signal K, NMEA 2000, UDP, or the OLED |
| Enable Heading Input (HWT3100) | Heading data isn't forwarded to Signal K, NMEA 2000, UDP, or the OLED |
| Enable Signal K Output | No Signal K deltas are sent, regardless of input state |
| Enable NMEA 2000 Output | Neither PGN 130306 nor PGN 127250 is transmitted, regardless of input state |
| Enable UDP NMEA 0183 Output | No UDP broadcast, regardless of input state |

An input toggle and an output toggle are independent: disabling the wind input silences wind data on *every* output at once, while disabling just the NMEA 2000 output leaves Signal K and UDP unaffected (as long as the wind/heading input itself is still enabled). All five take effect immediately — unlike the NMEA 2000 Watchdog setting below, none of them require a device restart.

### OTA Firmware Updates

The firmware calls `enable_ota(...)` (SensESP's ArduinoOTA integration), which accepts pushed updates over WiFi — it does not pull updates from a URL itself.

The current release binary is committed directly in [`firmware/`](firmware/) (e.g. [`HALSER-cv7-wind-interface-v1.2.0.bin`](firmware/HALSER-cv7-wind-interface-v1.2.0.bin)) — download it from there. [`.github/workflows/release-firmware.yml`](.github/workflows/release-firmware.yml) can also build and publish binaries to the repository's [Releases](../../releases) page (on a `v*` tag push, or via manual `workflow_dispatch`), but Actions runner availability for this org has been unreliable, so the committed file in `firmware/` is the dependable source until that's sorted out.

To flash a downloaded binary onto a device that's already running this firmware:

```bash
# Using PlatformIO (uses the OTA password set in main.cpp's enable_ota() call)
pio run -t upload --upload-port <device-ip> --upload-flags="--auth=thisisfine"

# Or using espota.py directly
python espota.py -i <device-ip> -a thisisfine -f firmware/HALSER-cv7-wind-interface-<version>.bin
```

Note that the binary is the application image only (no bootloader/partition table), so it's only valid for OTA onto a device already running compatible firmware — first-time programming still requires a wired `pio run -t upload`.

### Signal K Integration

Wind data is emitted to Signal K as:

- `environment.wind.speedApparent` — apparent wind speed in m/s
- `environment.wind.angleApparent` — apparent wind angle in radians

### UDP NMEA 0183 Output

If enabled (see [Enabling/Disabling Inputs and Outputs](#enablingdisabling-inputs-and-outputs) above), the firmware broadcasts NMEA 0183 sentences on UDP port 10110 — the convention chartplotter apps such as OpenCPN listen for — once per second:

- `$IIMWV,<angle>,R,<speed>,M,A` — apparent wind, relative reference, speed in m/s
- `$HCHDM,<heading>,M` — magnetic heading (only once the HWT3100 has produced at least one reading)

These are synthesized from the same post-processing values sent to N2K/Signal K (wind angle after the reference offset), not relayed raw from the CV7. The broadcast address is recomputed from the device's current IP/subnet mask on every send, so it self-corrects across WiFi reconnects. See [`src/sender/udp_nmea0183_sender.h`](src/sender/udp_nmea0183_sender.h).

### NMEA 2000 Watchdog

An optional watchdog can be enabled in the web UI under **Enable NMEA 2000 Watchdog**. When enabled, the device reboots if no NMEA 2000 messages are received for two minutes. This helps recover from CAN bus lockups. The setting requires a device restart to take effect.

### NMEA 2000 Stale Data Handling

The N2K senders use `RepeatExpiring` to handle stale data. If no new wind measurement (or, when the HWT3100 is connected, heading reading) arrives within 5 seconds, the sender transmits `N2kDoubleNA` ("not available") values instead of repeating stale data. PGN 130306 and 127250 messages continue at 100ms regardless — downstream devices always see a consistent message rate and can distinguish "no data" from silence.

### OLED Display

If connected, a 128x64 SSD1306 OLED display shows:
- Device hostname
- WiFi IP address
- Uptime in seconds
- *(blank line)*
- Apparent wind speed (m/s)
- Apparent wind angle (degrees, -180 to +180)
- Magnetic heading (degrees, 0 to 360) — reads 0.0 until an HWT3100 heading is received, since it's not connected/expiry-aware like the N2K senders

## Architecture

```
CV7 (NMEA 0183, 4800 bit/s)
  │
  │ UART1 (GPIO 3 RX only)
  ▼
NMEA0183IOTask
  └── WIMWVSentenceParser (apparent wind speed + angle, matches $IIMWV)
        └── EnabledGate (Enable Wind Input toggle) ── gates every consumer below
              ├── Reference angle offset (LambdaTransform, software-only)
              │     ├── N2kWindDataSender ── EnabledGate (Enable N2K)     → NMEA 2000 bus (TWAI, GPIO 4/5)
              │     ├── EnabledGate (Enable Signal K) → SKOutputFloat     → Signal K server (angle)
              │     ├── UdpNmea0183Sender ── EnabledGate (Enable UDP)     → UDP broadcast :10110 ($IIMWV)
              │     └── InfoDisplay                                      → OLED (angle)
              ├── N2kWindDataSender ── EnabledGate (Enable N2K)           → NMEA 2000 bus (speed)
              ├── EnabledGate (Enable Signal K) → SKOutputFloat           → Signal K server (speed)
              ├── UdpNmea0183Sender ── EnabledGate (Enable UDP)           → UDP broadcast :10110 (speed)
              └── InfoDisplay                                            → OLED (speed)

Web UI (SensESP) ──── Reference angle transform, all five toggles ──── Filesystem (persistent storage)

HWT3100 (Modbus RTU, 9600 bit/s, optional)
  │
  │ UART0 (GPIO 20/21, non-isolated GPIO header)
  ▼
Hwt3100HeadingReader (dedicated FreeRTOS task, polls MAGX..YAW register block)
  └── EnabledGate (Enable Heading Input toggle) ── gates every consumer below
        ├── N2kHeadingSender ── EnabledGate (Enable N2K)         → NMEA 2000 bus (TWAI, GPIO 4/5), PGN 127250
        ├── EnabledGate (Enable Signal K) → SKOutputFloat        → Signal K server (navigation.headingMagnetic)
        ├── UdpNmea0183Sender ── EnabledGate (Enable UDP)        → UDP broadcast :10110 ($HCHDM)
        └── InfoDisplay                                         → OLED (heading)
```

Each `EnabledGate` reads its `CheckboxConfig` live (not cached), so any of the five web UI toggles takes effect immediately — see [Enabling/Disabling Inputs and Outputs](#enablingdisabling-inputs-and-outputs). The N2K senders and `UdpNmea0183Sender` check their output toggle internally rather than through a separate gate object, since they're firmware-local classes; `SKOutputFloat` is a third-party SensESP type, so it's gated externally via `EnabledGate` like the inputs are.

The firmware is built on [SensESP](https://github.com/SignalK/SensESP), which provides WiFi connectivity, a web UI for configuration, Signal K protocol support, and OTA updates.

### Key Design Patterns

**Producer/Consumer pipeline:** SensESP uses a reactive pipeline where producers emit values that flow to connected consumers. The WIMWV parser produces apparent wind speed and angle values consumed (via the reference angle transform, for angle) by the N2K sender, Signal K outputs, and OLED display. Producers and consumers are connected via `connect_to()`.

**Software-only calibration:** The reference angle offset is a `LambdaTransform` with a single configurable parameter, persisted to the ESP32 filesystem. Because the CV7 has no NMEA 0183 command channel, there is no device-side equivalent to keep in sync — unlike instruments (such as the Autonnic A5120 this codebase originally targeted) that accept serial configuration commands.

**NMEA 2000 value expiry:** The `N2kWindDataSender` and `N2kHeadingSender` wrap inputs in `RepeatExpiring<double>`, which returns `N2kDoubleNA` when the source value is older than 5 seconds. This prevents stale data from being transmitted as valid measurements while maintaining the 100ms PGN transmission rate.

**Dedicated FreeRTOS tasks for blocking I/O:** Both the CV7 (`NMEA0183IOTask`) and the HWT3100 (`Hwt3100HeadingReader`) run their serial I/O on their own FreeRTOS task rather than the ReactESP event loop, since each read cycle blocks waiting on the UART. `ValueProducer::emit()` is called directly from those tasks — safe here because the ESP32-C3 is single-core, so there's no real concurrency to guard against, only preemption.

**Live-toggleable inputs/outputs:** `EnabledGate<T>` (`src/enabled_gate.h`) is a minimal pass-through `ValueConsumer`/`ValueProducer` that only forwards a value when its `CheckboxConfig` is checked, read live on every value rather than cached at boot. It's used to gate the two inputs and the Signal K output (a third-party type with no internal enabled flag to add); the N2K senders and `UdpNmea0183Sender` check their own `CheckboxConfig*` internally instead, since those classes are ours to modify. All five toggles apply immediately — no restart, unlike the "requires restart" NMEA 2000 Watchdog setting.

## Code Structure

### NMEA 2000 Output (`src/sender/`)

| File | Purpose |
|------|---------|
| `n2k_senders.h` | `N2kWindDataSender` (PGN 130306) and `N2kHeadingSender` (PGN 127250), both at 100ms with `RepeatExpiring` for stale data; each takes an optional `CheckboxConfig*` checked live to gate transmission |
| `udp_nmea0183_sender.h` | `UdpNmea0183Sender` — broadcasts synthesized `$IIMWV`/`$HCHDM` sentences on UDP port 10110, once per second; same live `CheckboxConfig*` gating |

### Application

| File | Purpose |
|------|---------|
| `main.cpp` | Entry point — wires all components together, including the reference angle `LambdaTransform` and the five enable/disable toggles |
| `ssd1306_display.h/.cpp` | OLED display driver (hostname, IP, uptime, AWS, AWA, HDG) |
| `hwt3100_heading_reader.h` | Modbus RTU master polling the HWT3100 fluxgate compass for magnetic heading |
| `enabled_gate.h` | `EnabledGate<T>` — live `CheckboxConfig`-gated pass-through, used for the input toggles and the Signal K output toggle |

### CV7 Protocol

The CV7 outputs standard NMEA 0183 sentences at 4800 bit/s, every 0.5s, with standard checksums:

| Sentence | Description |
|----------|-------------|
| `$IIMWV,<angle>,R,<speed>,<unit>,<status>` | Apparent wind angle + speed, relative reference |
| `$WIXDR,C,<temp>,C,,` | Wind sensor temperature (not currently consumed by this firmware) |

SensESP's built-in `WIMWVSentenceParser` matches the `MWV` formatter regardless of talker ID, so it parses the CV7's `$IIMWV` sentences unmodified.

The CV7 also emits undocumented `$PLCJ,...` / `$PLCJEA...` sentences described only as "for LCJ Capteurs technical service" — this firmware does not use them, and the CV7 has no public NMEA 0183 command interface for configuration (unlike the Autonnic A5120, which accepted `$PATC,IIMWV,AHD/DWD/DSP/TXP` commands with ACK confirmation).

### HWT3100 Protocol

The HWT3100-TTL/232 does not speak NMEA 0183. Per its manual, it has two serial modes, switched with an `AT+MODE=` command:

- **ASCII mode** (factory default) — AT commands (`AT+PRATE`, `AT+CALI`, etc.); the continuous streaming line format isn't documented byte-for-byte, so this firmware doesn't use it.
- **Modbus RTU mode** — standard Modbus read/write frames (function `0x03` read, `0x06` write), fully documented with a worked byte example and CRC16 in the manual.

This firmware sends `AT+MODE=1` once on boot to force Modbus mode — the manual documents that command as being accepted regardless of the sensor's current mode, so it's safe to send unconditionally — then polls as a Modbus RTU master, reading the 4-register block starting at `0xDB` (MAGX, MAGY, MAGZ, YAW) every 200ms and using only the YAW (heading) register:

| Register | Address | Description |
|----------|---------|-------------|
| MAGX / MAGY / MAGZ | 0xDB–0xDD | Magnetic field, X/Y/Z axes (not used by this firmware) |
| YAW | 0xDE | Heading, signed int16, 0.1° units |

See [`src/hwt3100_heading_reader.h`](src/hwt3100_heading_reader.h) for the full request/response framing and CRC.

### NMEA 2000 Device Identity

| Field | Value |
|-------|-------|
| Device function | 130 (Weather Instruments) |
| Device class | 85 (Sensor Communication Interface) |
| Manufacturer code | 2046 |
| Transmitted PGNs | 130306 (Wind Data), 127250 (Vessel Heading) |
| Wind reference | Apparent |
| Heading reference | Magnetic |

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
5. **Non-NMEA-0183 devices** — `Hwt3100HeadingReader` shows the pattern for a device that speaks a different serial protocol entirely (Modbus RTU): a custom `ValueProducer` running its own FreeRTOS task, polling on its own schedule, that plugs into the same N2K/Signal K sender pattern as the NMEA 0183-based wind path

To adapt this for a different device:
- Modify or replace `WIMWVSentenceParser` if your device uses different NMEA 0183 sentences
- If your device accepts NMEA 0183 configuration commands, replace the `LambdaTransform`-based reference angle with a command/ACK pattern (persist to filesystem, send the command, wait on a `SemaphoreValue` for the response)
- If your device doesn't speak NMEA 0183 at all, write a custom `ValueProducer` (see `hwt3100_heading_reader.h`) instead of a `SentenceParser`
- Update the N2K sender for your target PGNs
- Adjust pin assignments and bit rate in `main.cpp`

## License

See [LICENSE](LICENSE) for details.
