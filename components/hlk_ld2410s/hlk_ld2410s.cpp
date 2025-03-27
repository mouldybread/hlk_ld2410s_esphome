/*
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * Created by github.com/mouldybread
 * Creation Date/Time: 2025-03-27 11:15:34 UTC
 */

 #include "hlk_ld2410s.h"

 namespace esphome {
 namespace hlk_ld2410s {
 
 static const size_t MINIMAL_FRAME_LENGTH = 5;  // header(1) + state(1) + distance(2) + end(1)
 static const size_t STANDARD_FRAME_MIN_LENGTH = 72;  // header(4) + len(2) + type(1) + state(1) + distance(2) + reserved(2) + energy(64) + end(4)
 static const size_t CONFIG_FRAME_MIN_LENGTH = 10;  // header(4) + len(2) + cmd(2) + end(4)
 
 void HLKLD2410SComponent::setup() {
   ESP_LOGCONFIG(TAG, "Setting up HLK-LD2410S");
   last_update_ = millis();
   in_config_mode_ = false;
   
   // Initialize all sensors to known states
   if (config_mode_sensor_ != nullptr) {
     config_mode_sensor_->publish_state(false);
   }
   
   // Initialize thresholds vectors
   trigger_thresholds_.resize(16, 0);
   hold_thresholds_.resize(16, 0);
 }
 
 void HLKLD2410SComponent::loop() {
   const uint32_t now = millis();
   
   // Handle throttling
   if (throttle_ms_ > 0 && (now - last_update_) < throttle_ms_) {
     // Clear the buffer if we're throttled to prevent data buildup
     while (available()) {
       read();
     }
     return;
   }
 
   if (standard_output_mode_) {
     process_standard_data_();
   } else {
     process_minimal_data_();
   }
 }


void HLKLD2410SComponent::process_minimal_data_() {
  while (available() >= MINIMAL_FRAME_LENGTH) {
    uint8_t header = read();
    if (header != MINIMAL_FRAME_HEADER) {
      ESP_LOGV(TAG, "Invalid minimal frame header: 0x%02X", header);
      continue;
    }

    uint8_t state = read();
    uint8_t distance_low = read();
    uint8_t distance_high = read();
    uint8_t end = read();

    if (end != MINIMAL_FRAME_END) {
      ESP_LOGV(TAG, "Invalid minimal frame end: 0x%02X", end);
      continue;
    }

    uint16_t distance = (distance_high << 8) | distance_low;
    process_minimal_frame_(state, distance);
    last_update_ = millis();
    break;  // Process one frame per loop iteration when throttling
  }
}

void HLKLD2410SComponent::process_standard_data_() {
  while (available() >= STANDARD_FRAME_MIN_LENGTH) {
    // Look for standard frame header
    bool header_found = false;
    while (available() >= sizeof(STANDARD_FRAME_HEADER)) {
      uint8_t header_buf[sizeof(STANDARD_FRAME_HEADER)];
      for (size_t i = 0; i < sizeof(STANDARD_FRAME_HEADER); i++) {
        header_buf[i] = read();
      }
      
      if (verify_frame_header_(header_buf, sizeof(STANDARD_FRAME_HEADER))) {
        header_found = true;
        break;
      }
      
      // Reset position by one byte if header not found
      for (size_t i = 1; i < sizeof(STANDARD_FRAME_HEADER); i++) {
        header_buf[i-1] = header_buf[i];
      }
    }
    
    if (!header_found) {
      return;
    }

    // Read frame length and verify enough data available
    uint16_t length = read() | (read() << 8);
    if (length < 69) {  // Minimum valid length for standard frame
      ESP_LOGW(TAG, "Invalid standard frame length: %u", length);
      continue;
    }

    // Read frame type
    uint8_t type = read();
    if (type != 0x01) {
      ESP_LOGW(TAG, "Invalid standard frame type: 0x%02X", type);
      continue;
    }

    // Read state and distance
    uint8_t state = read();
    uint16_t distance = read() | (read() << 8);
    
    // Skip reserved bytes
    read();
    read();

    // Read energy values
    uint8_t energy_values[64];
    for (size_t i = 0; i < 64; i++) {
      energy_values[i] = read();
    }

    // Verify frame end
    uint8_t end_buf[sizeof(STANDARD_FRAME_END)];
    for (size_t i = 0; i < sizeof(STANDARD_FRAME_END); i++) {
      end_buf[i] = read();
    }

    if (!verify_frame_end_(end_buf, sizeof(STANDARD_FRAME_END))) {
      ESP_LOGW(TAG, "Invalid standard frame end");
      continue;
    }

    process_standard_frame_(state, distance, energy_values);
    last_update_ = millis();
    break;  // Process one frame per loop iteration when throttling
  }
}

void HLKLD2410SComponent::process_minimal_frame_(uint8_t state, uint16_t distance) {
  bool presence = (state >= 2);
  
  ESP_LOGV(TAG, "Minimal frame - State: %u, Distance: %u cm, Presence: %s", 
           state, distance, presence ? "true" : "false");

  if (presence_sensor_ != nullptr) {
    presence_sensor_->publish_state(presence);
  }
  
  if (distance_sensor_ != nullptr && distance > 0) {
    distance_sensor_->publish_state(distance);
  }
}

void HLKLD2410SComponent::process_standard_frame_(uint8_t state, uint16_t distance, const uint8_t *energy_values) {
  bool presence = (state >= 2);
  
  ESP_LOGV(TAG, "Standard frame - State: %u, Distance: %u cm, Presence: %s", 
           state, distance, presence ? "true" : "false");

  if (presence_sensor_ != nullptr) {
    presence_sensor_->publish_state(presence);
  }
  
  if (distance_sensor_ != nullptr && distance > 0) {
    distance_sensor_->publish_state(distance);
  }

  // Log energy values for debugging if needed
  if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERY_VERBOSE) {
    for (size_t i = 0; i < 16; i++) {
      ESP_LOGVV(TAG, "Gate %zu energy: %u", i, energy_values[i]);
    }
  }
}

bool HLKLD2410SComponent::write_command_(CommandWord cmd, const uint8_t *data, size_t len) {
  // Clear any pending data
  while (available()) {
    read();
  }

  // Write frame header
  write_array(CONFIG_FRAME_HEADER, sizeof(CONFIG_FRAME_HEADER));
  
  // Write data length (2 bytes, little endian)
  uint16_t total_len = 2 + len;  // command (2 bytes) + data length
  write_byte(total_len & 0xFF);
  write_byte((total_len >> 8) & 0xFF);
  
  // Write command (2 bytes, little endian)
  uint16_t cmd_val = static_cast<uint16_t>(cmd);
  write_byte(cmd_val & 0xFF);
  write_byte((cmd_val >> 8) & 0xFF);
  
  // Write data if any
  if (data != nullptr && len > 0) {
    write_array(data, len);
  }
  
  // Write frame end
  write_array(CONFIG_FRAME_END, sizeof(CONFIG_FRAME_END));
  
  return true;
}

bool HLKLD2410SComponent::read_ack_(CommandWord expected_cmd) {
  uint32_t start_time = millis();
  
  while ((millis() - start_time) < 1000) {  // 1 second timeout
    if (available() >= CONFIG_FRAME_MIN_LENGTH) {
      // Read and verify header
      uint8_t header_buf[sizeof(CONFIG_FRAME_HEADER)];
      for (size_t i = 0; i < sizeof(CONFIG_FRAME_HEADER); i++) {
        header_buf[i] = read();
      }
      
      if (!verify_frame_header_(header_buf, sizeof(CONFIG_FRAME_HEADER))) {
        ESP_LOGW(TAG, "Invalid ACK header");
        return false;
      }
      
      // Read length and command
      uint16_t len = read() | (read() << 8);
      uint16_t cmd = read() | (read() << 8);
      
      // Expected ACK command is original command | 0x0100
      uint16_t expected_ack = static_cast<uint16_t>(expected_cmd) | 0x0100;
      if (cmd != expected_ack) {
        ESP_LOGW(TAG, "Unexpected ACK command: 0x%04X, expected: 0x%04X", 
                 cmd, expected_ack);
        return false;
      }
      
      // Skip any additional data
      for (uint16_t i = 0; i < len - 2; i++) {
        read();
      }
      
      // Read and verify end sequence
      uint8_t end_buf[sizeof(CONFIG_FRAME_END)];
      for (size_t i = 0; i < sizeof(CONFIG_FRAME_END); i++) {
        end_buf[i] = read();
      }
      
      if (!verify_frame_end_(end_buf, sizeof(CONFIG_FRAME_END))) {
        ESP_LOGW(TAG, "Invalid ACK end");
        return false;
      }
      
      ESP_LOGD(TAG, "Valid ACK received for command 0x%04X", static_cast<uint16_t>(expected_cmd));
      return true;
    }
    delay(1);
  }
  
  ESP_LOGW(TAG, "ACK timeout waiting for command 0x%04X", static_cast<uint16_t>(expected_cmd));
  return false;
}

[Continue in Part 3...]