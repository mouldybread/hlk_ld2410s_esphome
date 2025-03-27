#include "hlk_ld2410s.h"

namespace esphome {
namespace hlk_ld2410s {

static const uint8_t FRAME_HEADER = 0x6E;
static const uint8_t FRAME_END = 0x62;
static const uint8_t FRAME_LENGTH = 5;  // Header(1) + State(1) + Distance(2) + End(1)

HLKLD2410SComponent::HLKLD2410SComponent(uart::UARTComponent *parent) : uart::UARTDevice(parent) {}

void HLKLD2410SComponent::setup() {
  // Nothing special needed for setup
  last_update_ = millis();
}

void HLKLD2410SComponent::loop() {
  const uint32_t now = millis();
  
  // Handle throttling
  if (throttle_ms_ > 0 && (now - last_update_) < throttle_ms_) {
    // Clear the buffer if we're throttled to prevent data buildup
    while (available() >= FRAME_LENGTH) {
      read();
    }
    return;
  }

  while (available() >= FRAME_LENGTH) {
    uint8_t header = read();
    if (header != FRAME_HEADER) {
      continue;  // Look for frame header
    }

    uint8_t state = read();
    uint8_t distance_low = read();
    uint8_t distance_high = read();
    uint8_t end = read();

    if (end != FRAME_END) {
      continue;  // Invalid frame
    }

    // Parse target state
    // 0/1 indicates no one; 2/3 indicate someone
    bool presence = (state >= 2);
    
    // Parse distance (little endian format)
    uint16_t distance = (distance_high << 8) | distance_low;

    // Update sensors
    if (this->presence_sensor_ != nullptr) {
      this->presence_sensor_->publish_state(presence ? 1 : 0);
    }
    if (this->distance_sensor_ != nullptr) {
      this->distance_sensor_->publish_state(distance);
    }

    last_update_ = now;
    break;  // Only process one valid frame per loop when throttling
  }
}

void HLKLD2410SComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "HLK-LD2410S:");
  LOG_SENSOR("  ", "Distance", this->distance_sensor_);
  LOG_SENSOR("  ", "Presence", this->presence_sensor_);
  ESP_LOGCONFIG(TAG, "  Throttle: %ums", this->throttle_ms_);
}

}  // namespace hlk_ld2410s
}  // namespace esphome