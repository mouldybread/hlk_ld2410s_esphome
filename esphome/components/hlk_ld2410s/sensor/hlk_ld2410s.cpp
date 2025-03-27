#include "hlk_ld2410s.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hlk_ld2410s {

static const char *const TAG = "hlk_ld2410s";

HLKLD2410SComponent::HLKLD2410SComponent(uart::UARTComponent *parent) : UARTDevice(parent) {}

void HLKLD2410SComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HLK-LD2410S...");
}

void HLKLD2410SComponent::loop() {
  const size_t available = this->available();
  if (available == 0)
    return;

  uint8_t buffer[available];
  this->read_array(buffer, available);
  this->parse_data_(buffer, available);
}

void HLKLD2410SComponent::parse_data_(const uint8_t *data, size_t length) {
  // Implement data parsing based on the HLK-LD2410S protocol.
  // This is an example for minimal data format parsing.
  if (length < 5) {
    ESP_LOGW(TAG, "Received data too short");
    return;
  }

  if (data[0] == 0x6E && data[length - 1] == 0x62) {
    uint8_t target_state = data[1];
    uint16_t object_distance = data[2] | (data[3] << 8);

    if (this->distance_sensor_ != nullptr)
      this->distance_sensor_->publish_state(object_distance);
    if (this->presence_sensor_ != nullptr)
      this->presence_sensor_->publish_state(target_state == 2 || target_state == 3);
  } else {
    ESP_LOGW(TAG, "Received invalid data format");
  }
}

}  // namespace hlk_ld2410s
}  // namespace esphome