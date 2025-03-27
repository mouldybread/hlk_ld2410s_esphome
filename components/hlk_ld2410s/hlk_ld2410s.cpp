/*
 * HLK-LD2410S Component for ESPHome
 * Created by: mouldybread
 * Creation Date/Time: 2025-03-27 12:38:38 UTC
 */

 #include "hlk_ld2410s.h"
 #include "esphome/core/log.h"
 #include "esphome/core/hal.h"
 
 namespace esphome {
 namespace hlk_ld2410s {
 
 void HLKLD2410SComponent::setup() {
     ESP_LOGCONFIG(TAG, "Setting up HLK-LD2410S...");
     
     if (this->throttle_ == 0) {
         this->throttle_ = 50;  // Default throttle of 50ms
     }
 
     // Initialize trigger and hold thresholds if not set
     if (this->trigger_thresholds_.empty()) {
         this->trigger_thresholds_.resize(MAX_GATES, 50);  // Default trigger threshold
     }
     if (this->hold_thresholds_.empty()) {
         this->hold_thresholds_.resize(MAX_GATES, 15);    // Default hold threshold
     }
 }
 
 void HLKLD2410SComponent::loop() {
     // Handle throttling
     if (this->throttle_ != 0) {
         const uint32_t now = millis();
         if ((now - this->last_update_) < this->throttle_) {
             return;
         }
         this->last_update_ = now;
     }
 
     // Process incoming data based on output mode
     if (this->output_mode_standard_) {
         this->process_standard_frame_(0, 0, nullptr);
     } else {
         this->process_simple_frame_(0, 0, nullptr);
     }
 }
 
 void HLKLD2410SComponent::process_simple_frame_(uint8_t frame_type, uint16_t data_length, const uint8_t *data) {
     if (available() < 4) {
         return;  // Not enough data for a minimal frame
     }
 
     // Check frame header
     if (read() != 0xF4 || read() != 0xF3 || read() != 0xF2 || read() != 0xF1) {
         // Invalid header, flush buffer
         while (available() > 0) {
             read();
         }
         return;
     }
 
     // Read state and distance
     uint8_t state = read();
     uint16_t distance = read() | (read() << 8);
 
     // Process the data
     if (this->presence_sensor_ != nullptr) {
         this->presence_sensor_->publish_state(state != 0);
     }
 
     if (this->distance_sensor_ != nullptr && state != 0) {
         this->distance_sensor_->publish_state(distance);
     } else if (this->distance_sensor_ != nullptr) {
         this->distance_sensor_->publish_state(0);
     }
 
     // Read frame end
     if (read() != 0xF8 || read() != 0xF7 || read() != 0xF6 || read() != 0xF5) {
         ESP_LOGW(TAG, "Invalid frame end in simple mode");
     }
 }
 
 void HLKLD2410SComponent::process_standard_frame_(uint8_t frame_type, uint16_t data_length, const uint8_t *data) {
     if (available() < 10) {
         return;  // Not enough data for a standard frame
     }
 
     // Check frame header
     if (read() != 0xF4 || read() != 0xF3 || read() != 0xF2 || read() != 0xF1) {
         // Invalid header, flush buffer
         while (available() > 0) {
             read();
         }
         return;
     }
 
     // Read frame data
     uint8_t target_state = read();
     uint16_t moving_distance = read() | (read() << 8);
     uint16_t stationary_distance = read() | (read() << 8);
     uint8_t detection_distance = read();
 
     // Read gate energy data
     std::vector<uint8_t> moving_energies(MAX_GATES);
     std::vector<uint8_t> stationary_energies(MAX_GATES);
 
     // Read moving target energies
     for (size_t i = 0; i < MAX_GATES; i++) {
         moving_energies[i] = read();
     }
 
     // Read stationary target energies
     for (size_t i = 0; i < MAX_GATES; i++) {
         stationary_energies[i] = read();
     }
 
     // Update sensors
     if (this->presence_sensor_ != nullptr) {
         bool presence = (target_state != 0);
         this->presence_sensor_->publish_state(presence);
         
         if (presence) {
             this->last_presence_detected_ = millis();
         } else if (this->unmanned_delay_ > 0) {
             // Check if we're within the unmanned delay period
             if ((millis() - this->last_presence_detected_) < (this->unmanned_delay_ * 1000)) {
                 this->presence_sensor_->publish_state(true);
                 presence = true;
             }
         }
     }
 
     if (this->distance_sensor_ != nullptr) {
         float distance = 0;
         if (moving_distance > 0 && (target_state & 0x01)) {
             distance = moving_distance;
         } else if (stationary_distance > 0 && (target_state & 0x02)) {
             distance = stationary_distance;
         }
         this->distance_sensor_->publish_state(distance);
     }
 
     // Update gate energy sensors
     for (const auto &pair : this->gate_energy_sensors_) {
         uint8_t gate = pair.first;
         sensor::Sensor *energy_sensor = pair.second;
         
         if (gate < MAX_GATES) {
             uint8_t energy = std::max(moving_energies[gate], stationary_energies[gate]);
             energy_sensor->publish_state(energy);
         }
     }
 
     // Read frame end
     if (read() != 0xF8 || read() != 0xF7 || read() != 0xF6 || read() != 0xF5) {
         ESP_LOGW(TAG, "Invalid frame end in standard mode");
     }
 }
 
 bool HLKLD2410SComponent::enable_configuration() {
     if (this->config_mode_) {
         ESP_LOGW(TAG, "Already in configuration mode");
         return true;
     }
 
     if (!write_command_(CommandWord::ENABLE_CONFIGURATION)) {
         ESP_LOGE(TAG, "Failed to send enable configuration command");
         return false;
     }
 
     if (!read_ack_(CommandWord::ENABLE_CONFIGURATION)) {
         ESP_LOGE(TAG, "Failed to receive enable configuration ACK");
         return false;
     }
 
     this->config_mode_ = true;
     if (this->config_mode_sensor_ != nullptr) {
         this->config_mode_sensor_->publish_state(true);
     }
     
     ESP_LOGI(TAG, "Configuration mode enabled");
     return true;
 }
 
 bool HLKLD2410SComponent::disable_configuration() {
     if (!this->config_mode_) {
         ESP_LOGW(TAG, "Not in configuration mode");
         return true;
     }
 
     if (!write_command_(CommandWord::DISABLE_CONFIGURATION)) {
         ESP_LOGE(TAG, "Failed to send disable configuration command");
         return false;
     }
 
     if (!read_ack_(CommandWord::DISABLE_CONFIGURATION)) {
         ESP_LOGE(TAG, "Failed to receive disable configuration ACK");
         return false;
     }
 
     this->config_mode_ = false;
     if (this->config_mode_sensor_ != nullptr) {
         this->config_mode_sensor_->publish_state(false);
     }
     
     ESP_LOGI(TAG, "Configuration mode disabled");
     return true;
 }
 
 bool HLKLD2410SComponent::write_command_(CommandWord command, const std::vector<uint8_t> &data) {
     // Write frame header
     for (uint8_t byte : CONFIG_FRAME_HEADER) {
         write(byte);
     }
 
     // Write command
     write(static_cast<uint16_t>(command) & 0xFF);
     write((static_cast<uint16_t>(command) >> 8) & 0xFF);
 
     // Write data length
     uint16_t length = data.size();
     write(length & 0xFF);
     write((length >> 8) & 0xFF);
 
     // Write data
     for (uint8_t byte : data) {
         write(byte);
     }
 
     // Write frame end
     for (uint8_t byte : CONFIG_FRAME_END) {
         write(byte);
     }
 
     return true;
 }
 
 bool HLKLD2410SComponent::read_ack_(CommandWord expected_cmd) {
     uint32_t start = millis();
     
     while ((millis() - start) < ACK_TIMEOUT_MS) {
         if (available() >= CONFIG_FRAME_MIN_LENGTH) {
             // Read and verify frame header
             uint8_t header_buf[sizeof(CONFIG_FRAME_HEADER)];
             for (size_t i = 0; i < sizeof(CONFIG_FRAME_HEADER); i++) {
                 header_buf[i] = read();
             }
             
             if (!verify_frame_header_(header_buf, sizeof(CONFIG_FRAME_HEADER))) {
                 ESP_LOGW(TAG, "Invalid ACK header");
                 continue;
             }
 
             // Read command
             uint16_t cmd = read() | (read() << 8);
             
             // Read data length
             uint16_t length = read() | (read() << 8);
 
             if (cmd != static_cast<uint16_t>(expected_cmd)) {
                 ESP_LOGW(TAG, "Unexpected ACK command: 0x%04X, expected: 0x%04X",
                        cmd, static_cast<uint16_t>(expected_cmd));
                 // Skip data
                 for (uint16_t i = 0; i < length; i++) {
                     read();
                 }
                 continue;
             }
 
             // Skip any data
             for (uint16_t i = 0; i < length; i++) {
                 read();
             }
 
             // Read and verify frame end
             uint8_t end_buf[sizeof(CONFIG_FRAME_END)];
             for (size_t i = 0; i < sizeof(CONFIG_FRAME_END); i++) {
                 end_buf[i] = read();
             }
             
             if (!verify_frame_end_(end_buf, sizeof(CONFIG_FRAME_END))) {
                 ESP_LOGW(TAG, "Invalid ACK end");
                 continue;
             }
 
             ESP_LOGD(TAG, "Valid ACK received for command 0x%04X", static_cast<uint16_t>(expected_cmd));
             return true;
         }
         delay(1);
     }
 
     ESP_LOGW(TAG, "ACK timeout waiting for command 0x%04X", static_cast<uint16_t>(expected_cmd));
     return false;
 }
 
 bool HLKLD2410SComponent::verify_frame_header_(const uint8_t *buf, size_t len) {
     if (len != sizeof(CONFIG_FRAME_HEADER)) {
         return false;
     }
     
     for (size_t i = 0; i < len; i++) {
         if (buf[i] != CONFIG_FRAME_HEADER[i]) {
             return false;
         }
     }
     
     return true;
 }
 
 bool HLKLD2410SComponent::verify_frame_end_(const uint8_t *buf, size_t len) {
     if (len != sizeof(CONFIG_FRAME_END)) {
         return false;
     }
     
     for (size_t i = 0; i < len; i++) {
         if (buf[i] != CONFIG_FRAME_END[i]) {
             return false;
         }
     }
     
     return true;
 }
 
 void EnableConfigButton::press_action() {
     if (this->parent_) {
         this->parent_->enable_configuration();
     }
 }
 
 void DisableConfigButton::press_action() {
     if (this->parent_) {
         this->parent_->disable_configuration();
     }
 }
 
 } // namespace hlk_ld2410s
 } // namespace esphome