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
//
// output_enabled_config (see the constructor) is an optional live toggle:
// while unchecked, send() is a no-op, so nothing is broadcast.
//
// Staleness: wind and heading are each tracked with their own "last
// received" timestamp, expiring after kExpiryMs (5s, matching the N2K
// senders' RepeatExpiring). Wind is still sent every cycle — so listeners
// always see a consistent message rate, same rationale as the N2K
// senders — but its NMEA 0183 status field is V (invalid) rather than A
// once the data is stale or has never arrived. $--HDM has no equivalent
// status field, so a stale/never-received heading is withheld entirely
// instead of sent with a misleading "valid" tag.

#ifndef WIND_INTERFACE_SRC_SENDER_UDP_NMEA0183_SENDER_H_
#define WIND_INTERFACE_SRC_SENDER_UDP_NMEA0183_SENDER_H_

#include <WiFi.h>
#include <WiFiUdp.h>

#include <algorithm>
#include <cmath>

#include "sensesp/system/lambda_consumer.h"
#include "sensesp/ui/ui_controls.h"
#include "sensesp_base_app.h"

namespace wind_interface {

class UdpNmea0183Sender {
 public:
  // output_enabled_config, if given, is checked live on every send cycle
  // (not just at construction) so a web UI toggle takes effect immediately.
  explicit UdpNmea0183Sender(
      sensesp::CheckboxConfig* output_enabled_config = nullptr,
      unsigned int repeat_interval_ms = 1000, uint16_t port = 10110)
      : port_{port}, output_enabled_config_{output_enabled_config} {
    sensesp::event_loop()->onRepeat(repeat_interval_ms,
                                     [this]() { this->send(); });
  }

  sensesp::LambdaConsumer<float> wind_speed_consumer{[this](float value) {
    this->wind_speed_ = value;
    this->wind_received_at_ms_ = millis();
    this->wind_ever_received_ = true;
  }};
  sensesp::LambdaConsumer<float> wind_angle_consumer{[this](float value) {
    this->wind_angle_ = value;
    this->wind_received_at_ms_ = millis();
    this->wind_ever_received_ = true;
  }};
  sensesp::LambdaConsumer<float> heading_consumer{[this](float value) {
    this->heading_ = value;
    this->heading_received_at_ms_ = millis();
    this->heading_ever_received_ = true;
  }};

 private:
  // Matches the N2K senders' RepeatExpiring expiry, so "stale" means the
  // same thing across every output.
  static constexpr unsigned long kExpiryMs = 5000;
  // Comfortably above any real wind reading (200 m/s ~= 389 kt); guards
  // against an out-of-range/garbage value blowing out the sentence via
  // %.1f's decimal expansion.
  static constexpr float kMaxSpeedMs = 200.0f;

  uint16_t port_;
  sensesp::CheckboxConfig* output_enabled_config_;
  WiFiUDP udp_;

  float wind_speed_ = 0;  // m/s
  float wind_angle_ = 0;  // radians, 0..2pi (apparent, relative reference)
  float heading_ = 0;     // radians, 0..2pi (magnetic)

  bool wind_ever_received_ = false;
  unsigned long wind_received_at_ms_ = 0;
  bool heading_ever_received_ = false;
  unsigned long heading_received_at_ms_ = 0;

  bool wind_stale() const {
    return !wind_ever_received_ ||
           (millis() - wind_received_at_ms_ > kExpiryMs);
  }

  bool heading_stale() const {
    return !heading_ever_received_ ||
           (millis() - heading_received_at_ms_ > kExpiryMs);
  }

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
    if (output_enabled_config_ != nullptr &&
        !output_enabled_config_->get_value()) {
      return;
    }

    // Clamp to a sane range before formatting: an out-of-range value (e.g.
    // NaN/Inf, or an absurd garbage reading) could otherwise expand to far
    // more digits than fit in the buffer under %.1f.
    float speed = wind_speed_;
    if (!std::isfinite(speed)) {
      speed = 0.0f;
    }
    speed = std::max(0.0f, std::min(speed, kMaxSpeedMs));
    float wind_angle_degrees =
        wind_angle_ * 180.0f / static_cast<float>(M_PI);

    char body[64];
    int written = snprintf(body, sizeof(body), "IIMWV,%.1f,R,%.1f,M,%s",
                            wind_angle_degrees, speed,
                            wind_stale() ? "V" : "A");
    // A non-negative, in-bounds return means the sentence fit; anything
    // else (truncated, or an encoding error) is skipped rather than
    // broadcasting a checksummed-but-malformed sentence.
    if (written > 0 && static_cast<size_t>(written) < sizeof(body)) {
      broadcast(WithChecksum(String(body)));
    }

    if (!heading_stale()) {
      float heading_degrees = heading_ * 180.0f / static_cast<float>(M_PI);
      written = snprintf(body, sizeof(body), "HCHDM,%.1f,M", heading_degrees);
      if (written > 0 && static_cast<size_t>(written) < sizeof(body)) {
        broadcast(WithChecksum(String(body)));
      }
    }
  }
};

}  // namespace wind_interface

#endif  // WIND_INTERFACE_SRC_SENDER_UDP_NMEA0183_SENDER_H_
