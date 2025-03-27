#include "hlk_ld2410s.h"

namespace esphome {
namespace hlk_ld2410s {

HLKLD2410SComponent::HLKLD2410SComponent(uart::UARTComponent *parent) : uart::UARTDevice(parent) {}

void HLKLD2410SComponent::setup() {
  // Initialization code
}

void HLKLD2410SComponent::loop() {
  const int max_data_length = 64;
  uint8_t data[max_data_length];
  size_t length = this->available();
  if (length > 0) {
    length = this->read_array(data, std::min(length, max_data_length));
    this->parse_data_(data, length);
  }
}

void HLKLD2410SComponent::parse_data_(const uint8_t *data, size_t length) {
  // Parse data and update sensors
  if (length >= 2 && this->distance_sensor_ != nullptr) {
    int distance = (data[0] << 8) | data[1];  // Combine two bytes into a 16-bit integer
    this->distance_sensor_->publish_state(distance);
  } else {
    // Log or handle insufficient data for distance
    ESP_LOGW("HLKLD2410SComponent", "Insufficient data for distance");
  }

  if (length >= 3 && this->presence_sensor_ != nullptr) {
    int presence = data[2];
    this->presence_sensor_->publish_state(presence);
  } else {
    // Log or handle insufficient data for presence
    ESP_LOGW("HLKLD2410SComponent", "Insufficient data for presence");
  }
}

}  // namespace hlk_ld2410s
}  // namespace esphome