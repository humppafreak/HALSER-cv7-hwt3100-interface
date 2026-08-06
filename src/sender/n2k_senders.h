// Converts apparent wind data to NMEA 2000 PGN 130306 (Wind Data), and
// fluxgate compass heading to PGN 127250 (Vessel Heading).
// Uses RepeatExpiring for wind_speed_/wind_angle_/heading_: remembers the
// last received values but expires them after 5s of no input. Expired
// values are sent as N2kDoubleNA so the N2K bus always sees messages at the
// 100ms interval required by the NMEA 2000 standard.
// Also a ValueProducer: emits a (speed, angle) pair or heading value on
// each successful TX, allowing downstream consumers (like the TX counter)
// to track activity.

#ifndef WIND_INTERFACE_SRC_SENDER_N2K_SENDERS_H_
#define WIND_INTERFACE_SRC_SENDER_N2K_SENDERS_H_

#include <N2kMessages.h>
#include <NMEA2000.h>

#include <tuple>

#include "sensesp/system/expiring_value.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/saveable.h"
#include "sensesp/system/serializable.h"
#include "sensesp/transforms/repeat.h"
#include "sensesp/ui/ui_controls.h"

namespace wind_interface {

/**
 * @brief Base class for NMEA 2000 senders.
 *
 */
class N2kSender : public sensesp::FileSystemSaveable,
                  virtual public sensesp::Serializable {
 public:
  N2kSender(String config_path)
      : sensesp::FileSystemSaveable{config_path}, sensesp::Serializable() {}

  virtual void enable() = 0;

  void disable() {
    if (this->sender_reaction_ != nullptr) {
      sensesp::event_loop()->remove(this->sender_reaction_);
      this->sender_reaction_ = nullptr;
    }
  }

 protected:
  reactesp::RepeatReaction* sender_reaction_ = nullptr;
};

class N2kWindDataSender
    : public N2kSender,
      public sensesp::ValueProducer<std::pair<double, double>> {
 public:
  // output_enabled_config, if given, is checked live on every send cycle
  // (not just at construction) so a web UI toggle takes effect immediately:
  // while unchecked, this PGN simply isn't transmitted, rather than being
  // sent as N2kDoubleNA.
  N2kWindDataSender(String config_path, tN2kWindReference wind_reference,
                    tNMEA2000* nmea2000, bool enable = true,
                    sensesp::CheckboxConfig* output_enabled_config = nullptr)
      : N2kSender{config_path},
        wind_reference_{wind_reference},
        nmea2000_{nmea2000},
        output_enabled_config_{output_enabled_config},
        repeat_interval_{100},  // In ms. Dictated by NMEA 2000 standard!
        expiry_{5000}           // In ms. When the inputs expire.
  {
    if (enable) {
      this->enable();
    }
  }

  void enable() override {
    if (this->sender_reaction_ == nullptr) {
      this->sender_reaction_ =
          sensesp::event_loop()->onRepeat(repeat_interval_, [this]() {
            if (this->output_enabled_config_ != nullptr &&
                !this->output_enabled_config_->get_value()) {
              return;
            }
            tN2kMsg N2kMsg;
            SetN2kWindSpeed(N2kMsg, 255, this->wind_speed_.get(),
                            this->wind_angle_.get(), this->wind_reference_);
            this->nmea2000_->SendMsg(N2kMsg);
            std::pair<double, double> wind_data = std::make_pair(
                this->wind_speed_.get(), this->wind_angle_.get());
            this->emit(wind_data);
          });
    }
  }

  // wind_angle_ and wind_speed_ depend on repeat_interval_ and expiry_
  // for initialization, but those are public API so we keep them all public.
  unsigned int repeat_interval_;
  unsigned int expiry_;

  sensesp::RepeatExpiring<double> wind_angle_{repeat_interval_, expiry_};
  sensesp::RepeatExpiring<double> wind_speed_{repeat_interval_, expiry_};

 protected:
  tNMEA2000* nmea2000_;
  tN2kWindReference wind_reference_;
  sensesp::CheckboxConfig* output_enabled_config_;
};

// Vessel Heading (PGN 127250), magnetic reference. The HWT3100 is a
// fluxgate compass with no GPS input of its own, so it can only report
// magnetic heading — deviation and variation are left as N2kDoubleNA.
class N2kHeadingSender : public N2kSender,
                          public sensesp::ValueProducer<double> {
 public:
  // output_enabled_config, if given, is checked live on every send cycle
  // (not just at construction) so a web UI toggle takes effect immediately:
  // while unchecked, this PGN simply isn't transmitted, rather than being
  // sent as N2kDoubleNA.
  N2kHeadingSender(String config_path, tNMEA2000* nmea2000,
                    bool enable = true,
                    sensesp::CheckboxConfig* output_enabled_config = nullptr)
      : N2kSender{config_path},
        nmea2000_{nmea2000},
        output_enabled_config_{output_enabled_config},
        repeat_interval_{100},  // In ms. Dictated by NMEA 2000 standard!
        expiry_{5000}           // In ms. When the input expires.
  {
    if (enable) {
      this->enable();
    }
  }

  void enable() override {
    if (this->sender_reaction_ == nullptr) {
      this->sender_reaction_ =
          sensesp::event_loop()->onRepeat(repeat_interval_, [this]() {
            if (this->output_enabled_config_ != nullptr &&
                !this->output_enabled_config_->get_value()) {
              return;
            }
            tN2kMsg N2kMsg;
            SetN2kMagneticHeading(N2kMsg, 255, this->heading_.get());
            this->nmea2000_->SendMsg(N2kMsg);
            this->emit(this->heading_.get());
          });
    }
  }

  // heading_ depends on repeat_interval_ and expiry_ for initialization,
  // but those are public API so we keep them all public.
  unsigned int repeat_interval_;
  unsigned int expiry_;

  sensesp::RepeatExpiring<double> heading_{repeat_interval_, expiry_};

 protected:
  tNMEA2000* nmea2000_;
  sensesp::CheckboxConfig* output_enabled_config_;
};

}  // namespace wind_interface

#endif  // WIND_INTERFACE_SRC_SENDER_N2K_SENDERS_H_
