#include "hlk_ld2410s.h"

namespace esphome {
namespace hlk_ld2410s {

static const uint8_t FRAME_HEADER = 0x6E;
static const uint8_t FRAME_END = 0x62;
static const uint8_t FRAME_LENGTH = 5;  // Header(1) + State(1) + Distance(2) + End(1)

// Protocol constants
static const uint8_t CONFIG_FRAME_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t CONFIG_FRAME_END[] = {0x04, 0x03, 0x02, 0x01};
static const uint16_t CMD_ENABLE_CONFIG = 0x00FF;
static const uint16_t CMD_DISABLE_CONFIG = 0x00FE;

HLKLD2410SComponent::HLKLD2410SComponent(uart::UARTComponent *parent) : uart::UARTDevice(parent) {
  ESP_LOGD(TAG, "Initializing HLK-LD2410S");
}

void HLKLD2410SComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HLK-LD2410S");
  last_update_ = millis();
}

bool HLKLD2410SComponent::write_command_(uint16_t command, const uint8_t *data, size_t len) {
  // Write frame header
  this->write_array(CONFIG_FRAME_HEADER, sizeof(CONFIG_FRAME_HEADER));
  
  // Write data length (2 bytes, little endian)
  uint16_t total_len = 2 + len;  // command (2 bytes) + data length
  this->write_byte(total_len & 0xFF);
  this->write_byte((total_len >> 8) & 0xFF);
  
  // Write command (2 bytes, little endian)
  this->write_byte(command & 0xFF);
  this->write_byte((command >> 8) & 0xFF);
  
  // Write data if any
  if (data != nullptr && len > 0) {
    this->write_array(data, len);
  }
  
  // Write frame end
  this->write_array(CONFIG_FRAME_END, sizeof(CONFIG_FRAME_END));
  
  return true;
}

bool HLKLD2410SComponent::read_ack_(uint16_t expected_command) {
  uint32_t start_time = millis();
  
  while ((millis() - start_time) < 1000) {  // 1 second timeout
    if (available() >= 8) {  // Minimum ACK size
      uint8_t header[4];
      for (int i = 0; i < 4; i++) {
        header[i] = read();
        if (header[i] != CONFIG_FRAME_HEADER[i]) {
          ESP_LOGW(TAG, "Invalid ACK header");
          return false;
        }
      }
      
      uint16_t len = read() | (read() << 8);
      uint16_t cmd = read() | (read() << 8);
      
      if (cmd != (expected_command | 0x0100)) {  // ACK commands are original + 0x0100
        ESP_LOGW(TAG, "Unexpected ACK command: 0x%04X", cmd);
        return false;
      }
      
      // Read rest of data and verify frame end
      for (uint16_t i = 0; i < len - 2; i++) {
        read();
      }
      
      uint8_t end[4];
      for (int i = 0; i < 4; i++) {
        end[i] = read();
        if (end[i] != CONFIG_FRAME_END[i]) {
          ESP_LOGW(TAG, "Invalid ACK end");
          return false;
        }
      }
      
      return true;
    }
    delay(1);
  }
  
  ESP_LOGW(TAG, "ACK timeout");
  return false;
}

void HLKLD2410SComponent::enable_configuration() {
  ESP_LOGD(TAG, "Enabling configuration mode");
  
  uint8_t data[] = {0x01, 0x00};  // Enable configuration command value
  if (write_command_(CMD_ENABLE_CONFIG, data, sizeof(data))) {
    if (read_ack_(CMD_ENABLE_CONFIG)) {
      ESP_LOGI(TAG, "Configuration mode enabled");
      in_config_mode_ = true;
    } else {
      ESP_LOGW(TAG, "Failed to enable configuration mode");
    }
  }
}

void HLKLD2410SComponent::disable_configuration() {
  ESP_LOGD(TAG, "Disabling configuration mode");
  
  if (write_command_(CMD_DISABLE_CONFIG)) {
    if (read_ack_(CMD_DISABLE_CONFIG)) {
      ESP_LOGI(TAG, "Configuration mode disabled");
      in_config_mode_ = false;
    } else {
      ESP_LOGW(TAG, "Failed to disable configuration mode");
    }
  }
}

// ... (rest of the existing loop() and other methods remain the same)

}  // namespace hlk_ld2410s
}  // namespace esphome