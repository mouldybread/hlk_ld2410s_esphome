#include "hlk_ld2410s_sensor.h"

namespace esphome {
namespace hlk_ld2410s {

void HLKLD2410SSensor::setup() {
  this->hlk_ld2410s_->setup();
}

void HLKLD2410SSensor::update() {
  this->hlk_ld2410s_->loop();
}

void HLKLD2410SSensor::set_hlk_ld2410s(HLK_LD2410S *hlk_ld2410s) {
  this->hlk_ld2410s_ = hlk_ld2410s;
}

}  // namespace hlk_ld2410s
}  // namespace esphome