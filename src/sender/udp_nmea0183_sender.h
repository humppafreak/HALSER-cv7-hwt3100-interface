// Broadcasts wind and heading data as NMEA 0183 sentences over UDP on the
// local WiFi network (port 10110, the de facto standard chartplotter apps
// such as OpenCPN listen on), as an additional output alongside NMEA 2000
// and Signal K.
//
// Sentences are synthesized from the same post-processing pipeline values
// already sent to Signal K/N2K — apparent wind speed/angle after the
// reference angle offset, and HWT3100 magnetic heading — rather than
// relayed verbatim from the CV7's own NMEA 0183 output, so the reference
// angle correction (and, if connected, heading) are both reflected here
// too. $--HDM (not $--HDG) is used for heading since the HWT3100 supplies
// no deviation/variation data.
//
// The broadcast address is recomputed from WiFi.localIP()/subnetMask() on
// every send rather than cached, so it stays correct across reconnects
// without needing a network-state hook.

#ifndef WIND_INTERFACE_SRC_SENDER_UDP_NMEA0183_SENDER_H_
#define WIND_INTERFACE_SRC_SENDER_UDP_NMEA0183_SENDER_H_

#include <WiFi.h>
#include <WiFiUdp.h>

#include <cmath>

#include "sensesp/system/lambda_consumer.h"
#include "sensesp_base_app.h"

namespace wind_interface {

class UdpNmea0183Sender {
 public:
  explicit UdpNmea0183Sender(unsigned int repeat_interval_ms = 1000,
                              uint16_t port = 10110)
      : port_{port} {
    sensesp::event_loop()->onRepeat(repeat_interval_ms,
                                     [this]() { this->send(); });
  }

  sensesp::LambdaConsumer<float> wind_speed_consumer{
      [this](float value) { this->wind_speed_ = value; }};
  sensesp::LambdaConsumer<float> wind_angle_consumer{
      [this](float value) { this->wind_angle_ = value; }};
  sensesp::LambdaConsumer<float> heading_consumer{[this](float value) {
    this->heading_ = value;
    this->heading_valid_ = true;
  }};

 private:
  uint16_t port_;
  WiFiUDP udp_;

  float wind_speed_ = 0;  // m/s
  float wind_angle_ = 0;  // radians, 0..2pi (apparent, relative reference)
  float heading_ = 0;     // radians, 0..2pi (magnetic)
  // HWT3100 is optional hardware; don't broadcast a bogus 0.0 heading
  // sentence until a real reading has arrived at least once.
  bool heading_valid_ = false;

  static uint8_t Checksum(const String& sentence_body) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < sentence_body.length(); i++) {
      checksum ^= sentence_body[i];
    }
    return checksum;
  }

  // sentence_body excludes the leading '$' and the trailing checksum.
  static String WithChecksum(const String& sentence_body) {
    char suffix[8];
    snprintf(suffix, sizeof(suffix), "*%02X\r\n", Checksum(sentence_body));
    return "$" + sentence_body + suffix;
  }

  void broadcast(const String& sentence) {
    IPAddress local_ip = WiFi.localIP();
    if (local_ip == IPAddress(0, 0, 0, 0)) {
      return;  // Not connected to WiFi yet.
    }
    IPAddress subnet = WiFi.subnetMask();
    IPAddress broadcast_ip;
    for (int i = 0; i < 4; i++) {
      broadcast_ip[i] = local_ip[i] | ~subnet[i];
    }
    udp_.beginPacket(broadcast_ip, port_);
    udp_.write(reinterpret_cast<const uint8_t*>(sentence.c_str()),
               sentence.length());
    udp_.endPacket();
  }

  void send() {
    float wind_angle_degrees =
        wind_angle_ * 180.0f / static_cast<float>(M_PI);
    char body[64];
    // Apparent wind, relative reference, speed in m/s — matches the values
    // already sent to N2K/Signal K.
    snprintf(body, sizeof(body), "IIMWV,%.1f,R,%.1f,M,A", wind_angle_degrees,
              wind_speed_);
    broadcast(WithChecksum(String(body)));

    if (heading_valid_) {
      float heading_degrees = heading_ * 180.0f / static_cast<float>(M_PI);
      snprintf(body, sizeof(body), "HCHDM,%.1f,M", heading_degrees);
      broadcast(WithChecksum(String(body)));
    }
  }
};

}  // namespace wind_interface

#endif  // WIND_INTERFACE_SRC_SENDER_UDP_NMEA0183_SENDER_H_
