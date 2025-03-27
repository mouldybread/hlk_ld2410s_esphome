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

bool HLKLD2410SComponent::verify_frame_header_(const uint8_t *header, size_t len) {
  if (len == sizeof(CONFIG_FRAME_HEADER)) {
    return memcmp(header, CONFIG_FRAME_HEADER, len) == 0;
  } else if (len == sizeof(STANDARD_FRAME_HEADER)) {
    return memcmp(header, STANDARD_FRAME_HEADER, len) == 0;
  }
  return false;
}

bool HLKLD2410SComponent::verify_frame_end_(const uint8_t *end, size_t len) {
  if (len == sizeof(CONFIG_FRAME_END)) {
    return memcmp(end, CONFIG_FRAME_END, len) == 0;
  } else if (len == sizeof(STANDARD_FRAME_END)) {
    return memcmp(end, STANDARD_FRAME_END, len) == 0;
  }
  return false;
}

void HLKLD2410SComponent::enable_configuration() {
  ESP_LOGD(TAG, "Enabling configuration mode");
  
  uint8_t data[] = {0x01, 0x00};  // Enable configuration command value
  
  if (write_command_(CommandWord::ENABLE_CONFIG, data, sizeof(data))) {
    if (read_ack_(CommandWord::ENABLE_CONFIG)) {
      ESP_LOGI(TAG, "Configuration mode enabled");
      in_config_mode_ = true;
      if (config_mode_sensor_ != nullptr) {
        config_mode_sensor_->publish_state(true);
      }
    }
  }
}

void HLKLD2410SComponent::disable_configuration() {
  ESP_LOGD(TAG, "Disabling configuration mode");
  
  if (write_command_(CommandWord::END_CONFIG)) {
    if (read_ack_(CommandWord::END_CONFIG)) {
      ESP_LOGI(TAG, "Configuration mode disabled");
      in_config_mode_ = false;
      if (config_mode_sensor_ != nullptr) {
        config_mode_sensor_->publish_state(false);
      }
    }
  }
}

bool HLKLD2410SComponent::switch_output_mode(bool standard_mode) {
  if (!in_config_mode_) {
    ESP_LOGW(TAG, "Must be in configuration mode to switch output mode");
    return false;
  }

  uint8_t data[] = {
    0x00, 0x00, 0x00, standard_mode ? 0x01 : 0x00, 0x00, 0x00
  };

  if (write_command_(CommandWord::SWITCH_OUTPUT_MODE, data, sizeof(data))) {
    if (read_ack_(CommandWord::SWITCH_OUTPUT_MODE)) {
      standard_output_mode_ = standard_mode;
      ESP_LOGI(TAG, "Switched to %s output mode", standard_mode ? "standard" : "minimal");
      return true;
    }
  }
  return false;
}

bool HLKLD2410SComponent::read_firmware_version() {
  if (!in_config_mode_) {
    ESP_LOGW(TAG, "Must be in configuration mode to read firmware version");
    return false;
  }

  if (write_command_(CommandWord::READ_FIRMWARE_VERSION)) {
    // Wait for response
    uint32_t start_time = millis();
    while ((millis() - start_time) < 1000) {
      if (available() >= CONFIG_FRAME_MIN_LENGTH + 6) {  // 6 additional bytes for version info
        // Skip header and length
        for (size_t i = 0; i < 6; i++) {
          read();
        }
        
        // Read version information
        firmware_version_.major = read() | (read() << 8);
        firmware_version_.minor = read() | (read() << 8);
        firmware_version_.patch = read() | (read() << 8);
        
        ESP_LOGI(TAG, "Firmware version: %u.%u.%u", 
                firmware_version_.major,
                firmware_version_.minor,
                firmware_version_.patch);
        return true;
      }
      delay(1);
    }
  }
  return false;
}

bool HLKLD2410SComponent::write_general_parameters(uint16_t param_word, uint32_t value) {
  if (!in_config_mode_) {
    ESP_LOGW(TAG, "Must be in configuration mode to write general parameters");
    return false;
  }

  // Validate parameter ranges according to protocol specification
  switch (static_cast<GeneralParamWord>(param_word)) {
    case GeneralParamWord::DETECT_FARTHEST_GATE:
      if (value > 16 || value < 1) {
        ESP_LOGW(TAG, "Invalid farthest gate value (1-16): %u", value);
        return false;
      }
      break;
    case GeneralParamWord::DETECT_NEAREST_GATE:
      if (value > 16) {
        ESP_LOGW(TAG, "Invalid nearest gate value (0-16): %u", value);
        return false;
      }
      break;
    case GeneralParamWord::UNMANNED_DELAY:
      if (value < 10 || value > 120) {
        ESP_LOGW(TAG, "Invalid unmanned delay value (10-120s): %u", value);
        return false;
      }
      break;
    case GeneralParamWord::STATUS_REPORT_FREQ:
    case GeneralParamWord::DISTANCE_REPORT_FREQ:
      if (value < 5 || value > 80) {  // Converting 0.5-8Hz to 5-80 (internal representation * 10)
        ESP_LOGW(TAG, "Invalid reporting frequency value (0.5-8Hz): %.1f", value / 10.0f);
        return false;
      }
      break;
    case GeneralParamWord::RESPONSE_SPEED:
      if (value != 5 && value != 10) {  // 5=Normal, 10=Fast
        ESP_LOGW(TAG, "Invalid response speed value (5 or 10): %u", value);
        return false;
      }
      break;
    default:
      ESP_LOGW(TAG, "Unknown parameter word: 0x%04X", param_word);
      return false;
  }

  uint8_t data[6];
  data[0] = param_word & 0xFF;
  data[1] = (param_word >> 8) & 0xFF;
  data[2] = value & 0xFF;
  data[3] = (value >> 8) & 0xFF;
  data[4] = (value >> 16) & 0xFF;
  data[5] = (value >> 24) & 0xFF;

  if (write_command_(CommandWord::WRITE_GENERAL_PARAMS, data, sizeof(data))) {
    if (read_ack_(CommandWord::WRITE_GENERAL_PARAMS)) {
      ESP_LOGI(TAG, "Successfully wrote parameter 0x%04X = %u", param_word, value);
      return true;
    }
  }
  return false;
}

bool HLKLD2410SComponent::read_general_parameters() {
  if (!in_config_mode_) {
    ESP_LOGW(TAG, "Must be in configuration mode to read general parameters");
    return false;
  }

  // Prepare request for all parameters
  const uint16_t params[] = {
    static_cast<uint16_t>(GeneralParamWord::DETECT_FARTHEST_GATE),
    static_cast<uint16_t>(GeneralParamWord::DETECT_NEAREST_GATE),
    static_cast<uint16_t>(GeneralParamWord::UNMANNED_DELAY),
    static_cast<uint16_t>(GeneralParamWord::STATUS_REPORT_FREQ),
    static_cast<uint16_t>(GeneralParamWord::DISTANCE_REPORT_FREQ),
    static_cast<uint16_t>(GeneralParamWord::RESPONSE_SPEED)
  };

  uint8_t data[sizeof(params)];
  memcpy(data, params, sizeof(params));

  if (write_command_(CommandWord::READ_GENERAL_PARAMS, data, sizeof(data))) {
    if (available() >= CONFIG_FRAME_MIN_LENGTH + 24) {  // 24 = 6 parameters * 4 bytes
      // Skip header and command acknowledgment
      for (size_t i = 0; i < CONFIG_FRAME_MIN_LENGTH; i++) {
        read();
      }

      // Read parameter values
      for (size_t i = 0; i < 6; i++) {
        uint32_t value = read() | (read() << 8) | (read() << 16) | (read() << 24);
        const char *param_name = "Unknown";
        switch (static_cast<GeneralParamWord>(params[i])) {
          case GeneralParamWord::DETECT_FARTHEST_GATE:
            param_name = "Farthest Gate";
            break;
          case GeneralParamWord::DETECT_NEAREST_GATE:
            param_name = "Nearest Gate";
            break;
          case GeneralParamWord::UNMANNED_DELAY:
            param_name = "Unmanned Delay";
            break;
          case GeneralParamWord::STATUS_REPORT_FREQ:
            param_name = "Status Report Freq";
            value = value / 10;  // Convert internal representation to Hz
            break;
          case GeneralParamWord::DISTANCE_REPORT_FREQ:
            param_name = "Distance Report Freq";
            value = value / 10;  // Convert internal representation to Hz
            break;
          case GeneralParamWord::RESPONSE_SPEED:
            param_name = "Response Speed";
            break;
        }
        ESP_LOGI(TAG, "%s: %u", param_name, value);
      }
      return true;
    }
  }
  return false;
}

bool HLKLD2410SComponent::write_trigger_threshold(const std::vector<uint32_t> &thresholds) {
  if (!in_config_mode_) {
    ESP_LOGW(TAG, "Must be in configuration mode to write trigger thresholds");
    return false;
  }

  if (thresholds.size() != 16) {
    ESP_LOGW(TAG, "Must provide exactly 16 threshold values");
    return false;
  }

  // Prepare data buffer: 16 * (2 bytes gate + 4 bytes value) = 96 bytes
  std::vector<uint8_t> data;
  data.reserve(96);

  for (size_t i = 0; i < 16; i++) {
    // Add gate number (2 bytes, little endian)
    data.push_back(i & 0xFF);
    data.push_back(0x00);
    
    // Add threshold value (4 bytes, little endian)
    data.push_back(thresholds[i] & 0xFF);
    data.push_back((thresholds[i] >> 8) & 0xFF);
    data.push_back((thresholds[i] >> 16) & 0xFF);
    data.push_back((thresholds[i] >> 24) & 0xFF);
  }

  if (write_command_(CommandWord::WRITE_TRIGGER_THRESHOLD, data.data(), data.size())) {
    if (read_ack_(CommandWord::WRITE_TRIGGER_THRESHOLD)) {
      trigger_thresholds_ = thresholds;
      ESP_LOGI(TAG, "Successfully wrote trigger thresholds");
      return true;
    }
  }
  return false;
}

bool HLKLD2410SComponent::read_trigger_threshold() {
  if (!in_config_mode_) {
    ESP_LOGW(TAG, "Must be in configuration mode to read trigger thresholds");
    return false;
  }

  // Prepare request for all gates (0-15)
  std::vector<uint8_t> data;
  data.reserve(32);  // 16 gates * 2 bytes per gate
  for (uint8_t i = 0; i < 16; i++) {
    data.push_back(i);  // Gate number
    data.push_back(0);  // High byte of gate number (always 0)
  }

  if (write_command_(CommandWord::READ_TRIGGER_THRESHOLD, data.data(), data.size())) {
    uint32_t start_time = millis();
    while ((millis() - start_time) < 1000) {
      if (available() >= CONFIG_FRAME_MIN_LENGTH + 64) {  // 64 = 16 gates * 4 bytes per value
        // Skip header and command acknowledgment
        for (size_t i = 0; i < CONFIG_FRAME_MIN_LENGTH; i++) {
          read();
        }

        // Read threshold values
        trigger_thresholds_.clear();
        trigger_thresholds_.reserve(16);
        
        for (size_t i = 0; i < 16; i++) {
          uint32_t value = read() | (read() << 8) | (read() << 16) | (read() << 24);
          trigger_thresholds_.push_back(value);
          ESP_LOGD(TAG, "Gate %u trigger threshold: %u", i, value);
        }
        
        ESP_LOGI(TAG, "Successfully read trigger thresholds");
        return true;
      }
      delay(1);
    }
    ESP_LOGW(TAG, "Timeout waiting for trigger threshold data");
  }
  return false;
}

bool HLKLD2410SComponent::write_hold_threshold(const std::vector<uint32_t> &thresholds) {
  if (!in_config_mode_) {
    ESP_LOGW(TAG, "Must be in configuration mode to write hold thresholds");
    return false;
  }

  if (thresholds.size() != 16) {
    ESP_LOGW(TAG, "Must provide exactly 16 threshold values");
    return false;
  }

  // Prepare data buffer: 16 * (2 bytes gate + 4 bytes value) = 96 bytes
  std::vector<uint8_t> data;
  data.reserve(96);

  for (size_t i = 0; i < 16; i++) {
    // Add gate number (2 bytes, little endian)
    data.push_back(i & 0xFF);
    data.push_back(0x00);
    
    // Add threshold value (4 bytes, little endian)
    data.push_back(thresholds[i] & 0xFF);
    data.push_back((thresholds[i] >> 8) & 0xFF);
    data.push_back((thresholds[i] >> 16) & 0xFF);
    data.push_back((thresholds[i] >> 24) & 0xFF);
  }

  if (write_command_(CommandWord::WRITE_HOLD_THRESHOLD, data.data(), data.size())) {
    if (read_ack_(CommandWord::WRITE_HOLD_THRESHOLD)) {
      hold_thresholds_ = thresholds;
      ESP_LOGI(TAG, "Successfully wrote hold thresholds");
      return true;
    }
  }
  return false;
}
// [Previous code from Parts 1-4...]

bool HLKLD2410SComponent::read_hold_threshold() {
  if (!in_config_mode_) {
    ESP_LOGW(TAG, "Must be in configuration mode to read hold thresholds");
    return false;
  }

  // Prepare request for all gates (0-15)
  std::vector<uint8_t> data;
  data.reserve(32);  // 16 gates * 2 bytes per gate
  for (uint8_t i = 0; i < 16; i++) {
    data.push_back(i);  // Gate number
    data.push_back(0);  // High byte of gate number (always 0)
  }

  if (write_command_(CommandWord::READ_HOLD_THRESHOLD, data.data(), data.size())) {
    uint32_t start_time = millis();
    while ((millis() - start_time) < 1000) {
      if (available() >= CONFIG_FRAME_MIN_LENGTH + 64) {  // 64 = 16 gates * 4 bytes per value
        // Skip header and command acknowledgment
        for (size_t i = 0; i < CONFIG_FRAME_MIN_LENGTH; i++) {
          read();
        }

        // Read threshold values
        hold_thresholds_.clear();
        hold_thresholds_.reserve(16);
        
        for (size_t i = 0; i < 16; i++) {
          uint32_t value = read() | (read() << 8) | (read() << 16) | (read() << 24);
          hold_thresholds_.push_back(value);
          ESP_LOGD(TAG, "Gate %u hold threshold: %u", i, value);
        }
        
        ESP_LOGI(TAG, "Successfully read hold thresholds");
        return true;
      }
      delay(1);
    }
    ESP_LOGW(TAG, "Timeout waiting for hold threshold data");
  }
  return false;
}

bool HLKLD2410SComponent::auto_update_threshold(uint8_t trigger_factor, uint8_t hold_factor, uint8_t scan_time) {
  if (!in_config_mode_) {
    ESP_LOGW(TAG, "Must be in configuration mode to auto-update thresholds");
    return false;
  }

  // Validate parameters
  if (trigger_factor < 1 || trigger_factor > 10) {
    ESP_LOGW(TAG, "Invalid trigger factor (1-10): %u", trigger_factor);
    return false;
  }
  if (hold_factor < 1 || hold_factor > 10) {
    ESP_LOGW(TAG, "Invalid hold factor (1-10): %u", hold_factor);
    return false;
  }
  if (scan_time < 30 || scan_time > 600) {
    ESP_LOGW(TAG, "Invalid scan time (30-600s): %u", scan_time);
    return false;
  }

  uint8_t data[] = {
    trigger_factor, 0x00,  // Trigger factor
    hold_factor, 0x00,     // Hold factor
    scan_time, 0x00       // Scanning time
  };

  if (write_command_(CommandWord::AUTO_UPDATE_THRESHOLD, data, sizeof(data))) {
    ESP_LOGI(TAG, "Starting auto threshold update (scan time: %us)", scan_time);
    
    // Monitor progress
    uint32_t start_time = millis();
    uint32_t last_progress = 0;
    while ((millis() - start_time) < (scan_time * 1000UL + 2000)) {  // Add 2s margin
      if (available() >= 8) {  // Minimum size for progress frame
        uint8_t header[4];
        for (size_t i = 0; i < 4; i++) {
          header[i] = read();
        }
        
        if (memcmp(header, STANDARD_FRAME_HEADER, 4) == 0) {
          uint16_t length = read() | (read() << 8);
          uint8_t type = read();
          
          if (type == 0x03) {  // Progress report type
            uint16_t progress = read() | (read() << 8);
            if (progress != last_progress) {
              process_threshold_progress_(progress);
              last_progress = progress;
            }
            
            if (progress >= 100) {
              ESP_LOGI(TAG, "Auto threshold update completed");
              return true;
            }
          }
        }
      }
      delay(100);
    }
    ESP_LOGW(TAG, "Auto threshold update timeout");
  }
  return false;
}

bool HLKLD2410SComponent::write_serial_number(const std::string &serial) {
  if (!in_config_mode_) {
    ESP_LOGW(TAG, "Must be in configuration mode to write serial number");
    return false;
  }

  if (serial.length() > 8) {
    ESP_LOGW(TAG, "Serial number must be 8 characters or less");
    return false;
  }

  // Prepare data: 2 bytes length + up to 8 bytes serial
  std::vector<uint8_t> data;
  data.push_back(serial.length() & 0xFF);
  data.push_back((serial.length() >> 8) & 0xFF);
  data.insert(data.end(), serial.begin(), serial.end());

  if (write_command_(CommandWord::WRITE_SERIAL_NUMBER, data.data(), data.size())) {
    if (read_ack_(CommandWord::WRITE_SERIAL_NUMBER)) {
      ESP_LOGI(TAG, "Successfully wrote serial number: %s", serial.c_str());
      return true;
    }
  }
  return false;
}

bool HLKLD2410SComponent::read_serial_number(std::string &serial) {
  if (!in_config_mode_) {
    ESP_LOGW(TAG, "Must be in configuration mode to read serial number");
    return false;
  }

  if (write_command_(CommandWord::READ_SERIAL_NUMBER)) {
    uint32_t start_time = millis();
    while ((millis() - start_time) < 1000) {
      if (available() >= CONFIG_FRAME_MIN_LENGTH + 10) {  // 10 = 2 bytes len + 8 bytes max serial
        // Skip header and command
        for (size_t i = 0; i < 8; i++) {
          read();
        }
        
        // Read length
        uint16_t length = read() | (read() << 8);
        
        if (length > 8) {
          ESP_LOGW(TAG, "Invalid serial number length: %u", length);
          return false;
        }
        
        // Read serial number
        serial.clear();
        for (uint16_t i = 0; i < length; i++) {
          serial.push_back(read());
        }
        
        ESP_LOGI(TAG, "Serial number: %s", serial.c_str());
        return true;
      }
      delay(1);
    }
    ESP_LOGW(TAG, "Timeout waiting for serial number");
  }
  return false;
}

void HLKLD2410SComponent::process_threshold_progress_(uint16_t progress) {
  ESP_LOGI(TAG, "Auto threshold update progress: %u%%", progress);
}

void HLKLD2410SComponent::set_response_speed(uint8_t speed) {
  if (!in_config_mode_) {
    ESP_LOGW(TAG, "Must be in configuration mode to set response speed");
    return;
  }

  if (speed != 5 && speed != 10) {
    ESP_LOGW(TAG, "Invalid response speed value (5=Normal, 10=Fast): %u", speed);
    return;
  }

  if (write_general_parameters(static_cast<uint16_t>(GeneralParamWord::RESPONSE_SPEED), speed)) {
    ESP_LOGI(TAG, "Response speed set to %s", speed == 5 ? "Normal" : "Fast");
  }
}

void HLKLD2410SComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "HLK-LD2410S:");
  LOG_SENSOR("  ", "Distance", this->distance_sensor_);
  LOG_BINARY_SENSOR("  ", "Presence", this->presence_sensor_);
  LOG_BINARY_SENSOR("  ", "Config Mode", this->config_mode_sensor_);
  ESP_LOGCONFIG(TAG, "  Throttle: %ums", this->throttle_ms_);
  ESP_LOGCONFIG(TAG, "  Configuration Mode: %s", this->in_config_mode_ ? "ON" : "OFF");
  ESP_LOGCONFIG(TAG, "  Output Mode: %s", this->standard_output_mode_ ? "Standard" : "Minimal");
  if (firmware_version_.major > 0) {
    ESP_LOGCONFIG(TAG, "  Firmware Version: %u.%u.%u",
                  firmware_version_.major,
                  firmware_version_.minor,
                  firmware_version_.patch);
  }
}

void EnableConfigButton::press() {
  this->parent_->enable_configuration();
}

void DisableConfigButton::press() {
  this->parent_->disable_configuration();
}

}  // namespace hlk_ld2410s
}  // namespace esphome