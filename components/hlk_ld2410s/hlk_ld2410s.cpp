#include "hlk_ld2410s.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hlk_ld2410s {

// Protocol frame headers and end sequences
static const uint8_t CONFIG_FRAME_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t CONFIG_FRAME_END[] = {0x04, 0x03, 0x02, 0x01};

HLKLD2410SComponent::HLKLD2410SComponent(uart::UARTComponent *parent) : UARTDevice(parent) {}

void HLKLD2410SComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HLK-LD2410S");
  last_update_ = millis();
  in_config_mode_ = false;
  
  // Initialize config mode sensor to known state
  if (config_mode_sensor_ != nullptr) {
    config_mode_sensor_->publish_state(false);
  }
}

void HLKLD2410SComponent::loop() {
  // Implementation of main loop
  // ... existing loop code ...
}

void HLKLD2410SComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "HLK-LD2410S:");
  if (!firmware_version_.empty()) {
    ESP_LOGCONFIG(TAG, "  Firmware Version: %s", firmware_version_.c_str());
    ESP_LOGCONFIG(TAG, "  Protocol Version: %d", protocol_version_);
  }
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

void HLKLD2410SComponent::read_firmware_version() {
  ESP_LOGD(TAG, "Reading firmware version");
  
  if (!in_config_mode_) {
    ESP_LOGW(TAG, "Must be in configuration mode to read firmware version");
    return;
  }

  // Send firmware version read command
  if (write_command_(CMD_READ_FIRMWARE_VERSION)) {
    // Wait for response
    uint32_t start_time = millis();
    while ((millis() - start_time) < 1000) {  // 1 second timeout
      if (available() >= 8) {  // Minimum response size
        uint8_t buffer[32];  // Buffer for response
        size_t len = 0;
        
        // Read header
        for (int i = 0; i < 4; i++) {
          buffer[len++] = read();
          if (buffer[i] != CONFIG_FRAME_HEADER[i]) {
            ESP_LOGW(TAG, "Invalid firmware version response header at position %d", i);
            return;
          }
        }
        
        // Read length
        uint16_t data_length = read() | (read() << 8);
        if (data_length > sizeof(buffer) - len) {
          ESP_LOGW(TAG, "Firmware version response too long: %u bytes", data_length);
          return;
        }
        
        // Read data
        for (uint16_t i = 0; i < data_length; i++) {
          if (available()) {
            buffer[len++] = read();
          } else {
            ESP_LOGW(TAG, "Timeout reading firmware version data");
            return;
          }
        }
        
        // Read end sequence
        for (int i = 0; i < 4; i++) {
          if (read() != CONFIG_FRAME_END[i]) {
            ESP_LOGW(TAG, "Invalid firmware version response end at position %d", i);
            return;
          }
        }
        
        // Parse the firmware version data
        parse_firmware_version_(buffer, len);
        return;
      }
      delay(1);
    }
    ESP_LOGW(TAG, "Timeout waiting for firmware version response");
  } else {
    ESP_LOGW(TAG, "Failed to send firmware version read command");
  }
}

void HLKLD2410SComponent::parse_firmware_version_(const uint8_t *data, size_t length) {
  // Protocol version is typically in byte 6
  if (length >= 7) {
    protocol_version_ = data[6];
    
    // Firmware version string typically starts at byte 7
    // Format is usually like "VX.Y.Z"
    if (length >= 11) {  // Assuming minimum version string length
      char version[16];
      snprintf(version, sizeof(version), "V%d.%d.%d", 
               data[7], data[8], data[9]);
      firmware_version_ = version;
      
      ESP_LOGI(TAG, "Firmware Version: %s, Protocol Version: %d", 
               firmware_version_.c_str(), protocol_version_);
    } else {
      ESP_LOGW(TAG, "Firmware version data too short");
    }
  } else {
    ESP_LOGW(TAG, "Invalid firmware version data length");
  }
}

}  // namespace hlk_ld2410s
}  // namespace esphome