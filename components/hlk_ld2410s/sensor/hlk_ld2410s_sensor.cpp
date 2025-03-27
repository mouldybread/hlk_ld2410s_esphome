#include "hlk_ld2410s_sensor.h"

namespace esphome {
namespace hlk_ld2410s {

HLKLD2410SSensor::HLKLD2410SSensor() {}

void HLKLD2410SSensor::setup() {
  // Initialization code
}

void HLKLD2410SSensor::loop() {
  const int max_data_length = 64;
  uint8_t data[max_data_length];
  size_t length = this->available();
  if (length > 0) {
    length = this->read_array(data, std::min(length, max_data_length));
    this->parse_data_(data, length);
  }
}

void HLKLD2410SSensor::update() {
  // Update sensor values
}

void HLKLD2410SSensor::parse_data_(const uint8_t *data, size_t length) {
  // Parse data and update sensors
  if (length >= 2 && this->distance_sensor_ != nullptr) {
    int distance = data[0] * 256 + data[1];
    this->distance_sensor_->publish_state(distance);
  }
  if (length >= 3 && this->presence_sensor_ != nullptr) {
    int presence = data[2];
    this->presence_sensor_->publish_state(presence);
  }
}

}  // namespace hlk_ld2410s
}  // namespace esphome