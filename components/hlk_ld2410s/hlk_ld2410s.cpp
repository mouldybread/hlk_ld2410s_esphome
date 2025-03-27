#include "hlk_ld2410s.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hlk_ld2410s {

static const uint8_t CONFIG_FRAME_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t CONFIG_FRAME_END[] = {0x04, 0x03, 0x02, 0x01};

HLKLD2410SComponent::HLKLD2410SComponent(uart::UARTComponent *parent) : UARTDevice(parent) {}

void HLKLD2410SComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HLK-LD2410S");
  
  // Clear any pending data
  while (available() > 0) {
    read();
  }
  
  last_update_ = millis();
  in_config_mode_ = false;
  
  // Initialize config mode sensor to known state
  if (config_mode_sensor_ != nullptr) {
    config_mode_sensor_->publish_state(false);
  }

  // Log initial state
  ESP_LOGD(TAG, "Initial state:");
  ESP_LOGD(TAG, "  Config mode: %s", in_config_mode_ ? "ON" : "OFF");
  ESP_LOGD(TAG, "  Throttle: %ums", throttle_ms_);
  ESP_LOGD(TAG, "  Distance sensor: %s", distance_sensor_ ? "configured" : "not configured");
  ESP_LOGD(TAG, "  Presence sensor: %s", presence_sensor_ ? "configured" : "not configured");
}

void HLKLD2410SComponent::loop() {
  // Don't process data packets in config mode
  if (in_config_mode_)
    return;

  // Check if enough time has passed since last update
  if (throttle_ms_ > 0 && (millis() - last_update_) < throttle_ms_)
    return;

  // Check if we have enough data for a complete packet
  if (available() >= 12) {  // Standard data frame is 12 bytes
    uint8_t header[4];
    bool valid_header = true;

    // Check for frame header (0xF4, 0xF3, 0xF2, 0xF1)
    for (int i = 0; i < 4; i++) {
      header[i] = read();
      if (header[i] != ((0xF4 - i) & 0xFF)) {
        valid_header = false;
        break;
      }
    }

    if (valid_header) {
      ESP_LOGD(TAG, "Valid header received");
      // Read data frame
      uint8_t data_length = read();  // Should be 0x05
      uint8_t data_type = read();    // Should be 0x02 for target data

      if (data_length == 0x05 && data_type == 0x02) {
        uint8_t target_state = read();  // Target state (0x00: no target, 0x01: moving target, 0x02: stationary target)
        uint8_t moving_distance = read();  // Moving target distance in decimeters
        uint8_t stationary_distance = read();  // Stationary target distance in decimeters
        
        // Skip detection threshold byte
        read();

        // Read tail (0xF8, 0xF7)
        uint8_t tail[2];
        tail[0] = read();
        tail[1] = read();

        if (tail[0] == 0xF8 && tail[1] == 0xF7) {
          ESP_LOGD(TAG, "Received: state=%02X, moving=%u, stationary=%u", 
                   target_state, moving_distance, stationary_distance);
          
          // Update sensors
          if (presence_sensor_ != nullptr) {
            presence_sensor_->publish_state(target_state != 0x00);
          }

          if (distance_sensor_ != nullptr) {
            float distance = 0;
            if (target_state == 0x01) {
              distance = moving_distance * 10.0f;  // Convert to centimeters
            } else if (target_state == 0x02) {
              distance = stationary_distance * 10.0f;  // Convert to centimeters
            }
            distance_sensor_->publish_state(distance);
          }

          last_update_ = millis();
          return;
        } else {
          ESP_LOGW(TAG, "Invalid tail: %02X %02X", tail[0], tail[1]);
        }
      } else {
        ESP_LOGW(TAG, "Invalid data frame: length=%02X, type=%02X", data_length, data_type);
      }
    }

    // If we get here, the packet was invalid
    // Flush the buffer and wait for next packet
    while (available() > 0) {
      read();
    }
  }
}

void HLKLD2410SComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "HLK-LD2410S:");
  LOG_SENSOR("  ", "Distance", this->distance_sensor_);
  LOG_BINARY_SENSOR("  ", "Presence", this->presence_sensor_);
  LOG_BINARY_SENSOR("  ", "Config Mode", this->config_mode_sensor_);
  ESP_LOGCONFIG(TAG, "  Throttle: %ums", this->throttle_ms_);
  ESP_LOGCONFIG(TAG, "  Configuration Mode: %s", this->in_config_mode_ ? "ON" : "OFF");
}

bool HLKLD2410SComponent::write_command_(uint16_t command, const uint8_t *data, size_t len) {
  // Write frame header
  for (uint8_t byte : CONFIG_FRAME_HEADER) {
    write(byte);
  }
  
  // Write data length (2 bytes, little endian)
  write(len + 2);  // +2 for command
  write(0x00);
  
  // Write command (2 bytes, little endian)
  write(command & 0xFF);
  write((command >> 8) & 0xFF);
  
  // Write data if provided
  if (data != nullptr && len > 0) {
    for (size_t i = 0; i < len; i++) {
      write(data[i]);
    }
  }
  
  // Write frame end
  for (uint8_t byte : CONFIG_FRAME_END) {
    write(byte);
  }
  
  return true;
}

bool HLKLD2410SComponent::read_ack_(uint16_t expected_command) {
  uint32_t start_time = millis();
  while ((millis() - start_time) < 1000) {  // 1 second timeout
    if (available() >= 8) {  // Minimum ACK size
      // Read and verify header
      for (uint8_t byte : CONFIG_FRAME_HEADER) {
        if (read() != byte) {
          return false;
        }
      }
      
      // Read length
      uint16_t length = read() | (read() << 8);
      
      // Read and verify command
      uint16_t cmd = read() | (read() << 8);
      if (cmd != expected_command) {
        return false;
      }
      
      // Skip any additional data
      for (uint16_t i = 4; i < length; i++) {
        if (available()) {
          read();
        } else {
          return false;
        }
      }
      
      // Read and verify end sequence
      for (uint8_t byte : CONFIG_FRAME_END) {
        if (read() != byte) {
          return false;
        }
      }
      
      return true;
    }
    delay(1);
  }
  return false;
}

void HLKLD2410SComponent::enable_configuration() {
  ESP_LOGD(TAG, "Enabling configuration mode");
  if (write_command_(CMD_ENABLE_CONFIG) && read_ack_(CMD_ENABLE_CONFIG)) {
    in_config_mode_ = true;
    if (config_mode_sensor_ != nullptr) {
      config_mode_sensor_->publish_state(true);
    }
    ESP_LOGD(TAG, "Configuration mode enabled");
  } else {
    ESP_LOGW(TAG, "Failed to enable configuration mode");
  }
}

void HLKLD2410SComponent::disable_configuration() {
  ESP_LOGD(TAG, "Disabling configuration mode");
  if (write_command_(CMD_DISABLE_CONFIG) && read_ack_(CMD_DISABLE_CONFIG)) {
    in_config_mode_ = false;
    if (config_mode_sensor_ != nullptr) {
      config_mode_sensor_->publish_state(false);
    }
    ESP_LOGD(TAG, "Configuration mode disabled");
  } else {
    ESP_LOGW(TAG, "Failed to disable configuration mode");
  }
}

void HLKLD2410SComponent::set_response_speed(uint8_t speed) {
  if (!in_config_mode_) {
    ESP_LOGW(TAG, "Must be in configuration mode to set response speed");
    return;
  }
  
  if (speed > 9) {
    ESP_LOGW(TAG, "Invalid response speed value: %u", speed);
    return;
  }
  
  uint8_t data[] = {speed};
  if (write_command_(CMD_SET_RESPONSE_SPEED, data, sizeof(data)) && 
      read_ack_(CMD_SET_RESPONSE_SPEED)) {
    ESP_LOGD(TAG, "Response speed set to %u", speed);
  } else {
    ESP_LOGW(TAG, "Failed to set response speed");
  }
}

}  // namespace hlk_ld2410s
}  // namespace esphome