#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/hlk_ld2410s/hlk_ld2410s.h"

namespace esphome {
namespace hlk_ld2410s {

class HLKLD2410SSensor : public sensor::Sensor, public PollingComponent {
 public:
  void setup() override {
    this->hlk_ld2410s_->setup();
  }

  void update() override {
    this->hlk_ld2410s_->loop();
  }

  void set_hlk_ld2410s(HLK_LD2410S *hlk_ld2410s) {
    this->hlk_ld2410s_ = hlk_ld2410s;
  }

 private:
  HLK_LD2410S *hlk_ld2410s_;
};

}  // namespace hlk_ld2410s
}  // namespace esphome
