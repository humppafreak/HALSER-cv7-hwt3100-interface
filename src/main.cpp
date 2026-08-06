// HALSER Wind Interface Firmware — application entry point.
// Wires the data pipeline: LCJ Capteurs CV7 (NMEA 0183 over UART) → MWV
// sentence parser → reference angle offset → N2K wind data sender (PGN
// 130306) + Signal K + OLED.
//
// Unlike the Autonnic A5120 this firmware was originally written for, the
// CV7 exposes no NMEA 0183 command channel for configuring the instrument
// (its $PLCJ,... sentences are an undocumented technical-service protocol,
// not a public configuration interface). The reference angle offset is
// therefore applied entirely in software via a LambdaTransform, persisted
// to the ESP32 filesystem only.
//
// Also wires an optional WitMotion HWT3100-TTL/232 fluxgate compass, read
// over Modbus RTU on a second, non-isolated UART (GPIO 20/21, the HALSER
// GPIO header) → N2K heading sender (PGN 127250) + Signal K + OLED. The
// HWT3100 isn't wired to the board's dedicated RS-485/RS-232/UART terminal
// block because that block's receive side is a single hardware-muxed
// channel (the RX SEL jumper) already claimed by the CV7's RS-485
// connection; see hwt3100_heading_reader.h for the sensor's protocol.

#include <NMEA2000_esp32.h>

#include <cmath>
#include <memory>

#include "Wire.h"
#include "elapsedMillis.h"
#include "hwt3100_heading_reader.h"
#include "sender/n2k_senders.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/serial_number.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/ui/status_page_item.h"
#include "sensesp/ui/ui_controls.h"
#include "sensesp_app_builder.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/wind_sentence_parser.h"
#include "ssd1306_display.h"

using namespace sensesp;
using namespace sensesp::nmea0183;
using namespace wind_interface;

// HALSER pin assignments
constexpr int kWindBitRate = 4800;
constexpr gpio_num_t kUART1RxPin = GPIO_NUM_3;
// The CV7 is transmit-only over NMEA 0183 (no command channel), so this pin
// carries no traffic, but Serial1.begin() still requires a TX pin argument.
constexpr gpio_num_t kUART1TxPin = GPIO_NUM_2;
constexpr gpio_num_t kCANTxPin = GPIO_NUM_4;
constexpr gpio_num_t kCANRxPin = GPIO_NUM_5;
constexpr int kI2CSDAPin = 6;
constexpr int kI2CSCLPin = 7;
constexpr int kButtonPin = 9;
// HWT3100 fluxgate compass, wired to the GPIO header rather than the
// board's dedicated (and already-claimed) NMEA 0183 RX interface — see the
// file comment above and hwt3100_heading_reader.h. Unlike the isolated
// serial connectors, GPIO 20/21 are direct, non-isolated ESP32-C3 3.3 V
// logic; confirm the HWT3100's TTL levels are 3.3 V-tolerant before wiring.
constexpr int kHeadingBitRate = 9600;
constexpr gpio_num_t kHeadingTxPin = GPIO_NUM_20;
constexpr gpio_num_t kHeadingRxPin = GPIO_NUM_21;

ObservableValue<int> n2k_rx_counter = 0;
ObservableValue<int> n2k_tx_counter = 0;

elapsedMillis n2k_time_since_rx = 0;
elapsedMillis n2k_time_since_tx = 0;

void setup() {
  Serial.setTxTimeoutMs(0);
  SetupLogging(ESP_LOG_DEBUG);

  Wire.setPins(kI2CSDAPin, kI2CSCLPin);
  Wire.begin();

  Serial1.begin(kWindBitRate, SERIAL_8N1, kUART1RxPin, kUART1TxPin);
  // UART0 is otherwise idle: the console (Serial) runs over USB CDC.
  Serial0.begin(kHeadingBitRate, SERIAL_8N1, kHeadingRxPin, kHeadingTxPin);

  // SensESP application
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    ->set_hostname("wind")
                    ->set_button_pin(kButtonPin)
                    ->enable_ota("thisisfine")
                    ->get_app();

  // NMEA 0183 I/O task
  auto nmea0183_io_task = std::make_shared<NMEA0183IOTask>(&Serial1);

  // Wind sentence parser — connected directly to parser, no TaskQueueProducer
  // (ESP32-C3 is single-core, so cross-task bridging is unnecessary). The
  // CV7 emits standard $IIMWV sentences (relative reference); SensESP's
  // built-in wind parser matches MWV regardless of talker ID, so it works
  // unmodified with the CV7's output.
  auto wind_parser =
      std::make_shared<WIMWVSentenceParser>(&(nmea0183_io_task->parser_));

  // Reference angle offset — corrects for misalignment between the CV7's
  // mounting orientation and the vessel's centerline. The CV7 has no NMEA
  // 0183 command to apply this in the instrument itself (unlike the
  // Autonnic A5120's $PATC,IIMWV,AHD command), so it is applied here in
  // software to the parsed wind angle before it reaches any consumer.
  const ParamInfo* reference_angle_param_info =
      new ParamInfo[1]{{"offset", "Offset (degrees)"}};

  auto reference_angle_transform =
      std::make_shared<LambdaTransform<float, float, float>>(
          [](float angle, float offset_degrees) -> float {
            float offset_radians = offset_degrees * M_PI / 180.0;
            float result = fmodf(angle + offset_radians, 2 * M_PI);
            if (result < 0) {
              result += 2 * M_PI;
            }
            return result;
          },
          0.0f, reference_angle_param_info, "/Wind/Reference Angle");

  ConfigItem(reference_angle_transform)
      ->set_title("Reference Angle")
      ->set_description(
          "Reference angle offset for wind data (in degrees). "
          "Enter the angle readout when the wind vane is pointing "
          "straight ahead.")
      ->set_sort_order(300);

  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 functionality

  tNMEA2000* nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);

  // 64-frame CAN buffers: enough to absorb a fast-packet burst (up to 32 frames
  // each) while keeping the static footprint modest on the memory-constrained C3.
  nmea2000->SetN2kCANSendFrameBufSize(64);
  nmea2000->SetN2kCANReceiveFrameBufSize(64);

  nmea2000->SetProductInformation(
      "20240601",  // Manufacturer's Model serial code (max 32 chars)
      105,         // Manufacturer's product code
      "Wind-N2K",  // Manufacturer's Model ID (max 33 chars)
      "1.0.0",     // Manufacturer's Software version code (max 40 chars)
      "1.0.0"      // Manufacturer's Model version (max 24 chars)
  );

  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // Unique number
      130,                     // Device function: Weather Instruments
      85,                      // Device class: Sensor Communication Interface
      2046);                   // Manufacturer code

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 72);
  nmea2000->SetMsgHandler([](const tN2kMsg& msg) {
    n2k_rx_counter = n2k_rx_counter.get() + 1;
    n2k_time_since_rx = 0;
  });
  nmea2000->EnableForward(false);
  nmea2000->Open();

  event_loop()->onRepeat(1, [nmea2000]() { nmea2000->ParseMessages(); });

  /////////////////////////////////////////////////////////////////////
  // NMEA 2000 wind data sender

  auto wind_data_sender = std::make_shared<N2kWindDataSender>(
      "/Wind/NMEA2000", tN2kWindReference::N2kWind_Apparent, nmea2000, true);

  // Wire wind parser outputs to N2K sender. Wind angle is routed through the
  // reference angle offset transform first.
  wind_parser->apparent_wind_speed_.connect_to(&(wind_data_sender->wind_speed_));
  wind_parser->apparent_wind_angle_.connect_to(reference_angle_transform);
  reference_angle_transform->connect_to(&(wind_data_sender->wind_angle_));

  wind_data_sender->connect_to(
      std::make_shared<LambdaConsumer<std::pair<double, double>>>(
          [](std::pair<double, double>) {
            n2k_tx_counter = n2k_tx_counter.get() + 1;
            n2k_time_since_tx = 0;
          }));

  /////////////////////////////////////////////////////////////////////
  // NMEA 2000 heading sender (HWT3100 fluxgate compass, optional)

  auto heading_reader =
      std::make_shared<Hwt3100HeadingReader>(&Serial0);

  auto heading_sender = std::make_shared<N2kHeadingSender>(
      "/Heading/NMEA2000", nmea2000, true);

  heading_reader->connect_to(&(heading_sender->heading_));

  heading_sender->connect_to(std::make_shared<LambdaConsumer<double>>(
      [](double) {
        n2k_tx_counter = n2k_tx_counter.get() + 1;
        n2k_time_since_tx = 0;
      }));

  /////////////////////////////////////////////////////////////////////
  // Signal K outputs

  auto wind_speed_sk = std::make_shared<SKOutputFloat>(
      "environment.wind.speedApparent", "/SK Path/Apparent Wind Speed",
      new SKMetadata("m/s", "Apparent Wind Speed"));

  auto wind_angle_sk = std::make_shared<SKOutputFloat>(
      "environment.wind.angleApparent", "/SK Path/Apparent Wind Angle",
      new SKMetadata("rad", "Apparent Wind Angle"));

  wind_parser->apparent_wind_speed_.connect_to(wind_speed_sk);
  reference_angle_transform->connect_to(wind_angle_sk);

  auto heading_sk = std::make_shared<SKOutputFloat>(
      "navigation.headingMagnetic", "/SK Path/Heading Magnetic",
      new SKMetadata("rad", "Magnetic Heading"));

  heading_reader->connect_to(heading_sk);

  /////////////////////////////////////////////////////////////////////
  // Configuration elements

  auto enable_n2k_watchdog_config = std::make_shared<CheckboxConfig>(
      false, "Enable NMEA 2000 Watchdog", "/NMEA2000/Enable Watchdog");

  ConfigItem(enable_n2k_watchdog_config)
      ->set_title("Enable NMEA 2000 Watchdog")
      ->set_description(
          "Enable the NMEA 2000 watchdog. If enabled, the device will reboot "
          "after two minutes if no NMEA 2000 messages are received. This "
          "setting requires a device restart to take effect.")
      ->set_sort_order(100);

  if (enable_n2k_watchdog_config->get_value()) {
    event_loop()->onRepeat(1000, [nmea2000]() {
      if (n2k_time_since_rx > 120000) {
        ESP_LOGE("NMEA2000", "No messages received in 2 minutes. Restarting.");
        delay(10);
        ESP.restart();
      }
    });
  }

  auto n2k_rx_ui_output = std::make_shared<StatusPageItem<int>>(
      "NMEA 2000 Received Messages", 0, "NMEA 2000", 300);

  n2k_rx_counter.connect_to(n2k_rx_ui_output);

  auto n2k_tx_ui_output = std::make_shared<StatusPageItem<int>>(
      "NMEA 2000 Transmitted Messages", 0, "NMEA 2000", 310);

  n2k_tx_counter.connect_to(n2k_tx_ui_output);

  // Largest contiguous free block. This, not total free memory, gates large
  // allocations like the ~40 KB TLS handshake, so surface it on the status page
  // to make heap fragmentation visible.
  auto largest_block_status = std::make_shared<StatusPageItem<int>>(
      "Largest free block (bytes)", 0, "System", 250);
  event_loop()->onRepeat(2000, [largest_block_status]() {
    largest_block_status->set(static_cast<int>(ESP.getMaxAllocHeap()));
  });

  /////////////////////////////////////////////////////////////////////
  // OLED display

  auto display = std::make_shared<InfoDisplay>(&Wire);
  wind_parser->apparent_wind_speed_.connect_to(
      &(display->apparent_wind_speed_consumer));
  reference_angle_transform->connect_to(
      &(display->apparent_wind_angle_consumer));
  heading_reader->connect_to(&(display->heading_consumer));

  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
