// A pass-through producer/consumer that only forwards values downstream
// while a CheckboxConfig is checked. Used to gate the CV7/HWT3100 inputs
// and the Signal K output (both third-party types we can't add an internal
// enabled_ check to) at a single connection point.
//
// The config is read live on every value, not cached, so toggling it in the
// web UI takes effect immediately — no device restart needed, unlike the
// existing "requires restart" config items in main.cpp that gate whether a
// whole feature gets wired up at boot at all.

#ifndef WIND_INTERFACE_SRC_ENABLED_GATE_H_
#define WIND_INTERFACE_SRC_ENABLED_GATE_H_

#include "sensesp/system/valueconsumer.h"
#include "sensesp/system/valueproducer.h"
#include "sensesp/ui/ui_controls.h"

namespace wind_interface {

template <typename T>
class EnabledGate : public sensesp::ValueConsumer<T>,
                     public sensesp::ValueProducer<T> {
 public:
  explicit EnabledGate(sensesp::CheckboxConfig* enabled_config)
      : sensesp::ValueProducer<T>(T{}), enabled_config_{enabled_config} {}

  void set(const T& new_value) override {
    if (enabled_config_->get_value()) {
      this->emit(new_value);
    }
  }

 private:
  sensesp::CheckboxConfig* enabled_config_;
};

}  // namespace wind_interface

#endif  // WIND_INTERFACE_SRC_ENABLED_GATE_H_
