/**
 * HLK-LD2410S mmWave Radar Sensor Component for ESPHome.
 * Version: 49
 * Created by github.com/mouldybread
 * Creation Date/Time: 2025-03-27 14:49:50 UTC
 */

 #include "hlk_ld2410s.h"
 #include "esphome/core/log.h"
 
 namespace esphome {
 namespace hlk_ld2410s {
 
 void EnableConfigButton::press_action() { this->parent_->enable_configuration(); }
 void DisableConfigButton::press_action() { this->parent_->disable_configuration(); }
 
 void HLKLD2410SComponent::setup() {
     ESP_LOGI(TAG, "Setting up HLK-LD2410S...");
     
     this->check_uart_settings_();
     this->flush();
     delay(COMMAND_DELAY_MS);
     
     // Set initial state
     if (this->config_mode_sensor_ != nullptr) {
         this->config_mode_sensor_->publish_state(false);
     }
 
     // Apply configuration if any parameters are set
     bool needs_config = false;
     if (this->response_speed_ != 5 || this->unmanned_delay_ != 40 ||
         this->status_report_freq_ != 0.5f || this->distance_report_freq_ != 0.5f ||
         this->farthest_gate_ != 12 || this->nearest_gate_ != 0 ||
         !this->trigger_thresholds_.empty() || !this->hold_thresholds_.empty()) {
         needs_config = true;
     }
 
     if (needs_config) {
         ESP_LOGI(TAG, "Applying custom configuration...");
         if (enable_configuration()) {
             delay(COMMAND_DELAY_MS);
             if (write_parameters_()) {
                 ESP_LOGI(TAG, "Parameters written successfully");
             }
             
             if (!this->trigger_thresholds_.empty()) {
                 delay(COMMAND_DELAY_MS);
                 if (write_trigger_thresholds_()) {
                     ESP_LOGI(TAG, "Trigger thresholds written successfully");
                 }
             }
             
             if (!this->hold_thresholds_.empty()) {
                 delay(COMMAND_DELAY_MS);
                 if (write_hold_thresholds_()) {
                     ESP_LOGI(TAG, "Hold thresholds written successfully");
                 }
             }
             
             if (this->auto_threshold_trigger_ != 2 || this->auto_threshold_hold_ != 1 ||
                 this->auto_threshold_scan_ != 120) {
                 delay(COMMAND_DELAY_MS);
                 if (set_auto_threshold_()) {
                     ESP_LOGI(TAG, "Auto threshold parameters written successfully");
                 }
             }
             
             delay(COMMAND_DELAY_MS);
             disable_configuration();
         }
     }
 
     // Switch to standard mode if configured
     if (this->output_mode_standard_) {
         if (!this->switch_output_mode_(true)) {
             ESP_LOGE(TAG, "Failed to switch to standard output mode");
         }
     }
 
     ESP_LOGI(TAG, "Setup complete");
 }
 
 void HLKLD2410SComponent::dump_config() {
     ESP_LOGCONFIG(TAG, "HLK-LD2410S:");
     ESP_LOGCONFIG(TAG, "  Configuration Mode: %s", this->config_mode_ ? "ON" : "OFF");
     ESP_LOGCONFIG(TAG, "  Output Mode: %s", this->output_mode_standard_ ? "Standard" : "Minimal");
     ESP_LOGCONFIG(TAG, "  Response Speed: %d", this->response_speed_);
     ESP_LOGCONFIG(TAG, "  Unmanned Delay: %ds", this->unmanned_delay_);
     ESP_LOGCONFIG(TAG, "  Status Report Frequency: %.1fHz", this->status_report_freq_);
     ESP_LOGCONFIG(TAG, "  Distance Report Frequency: %.1fHz", this->distance_report_freq_);
     ESP_LOGCONFIG(TAG, "  Gate Range: %d-%d", this->nearest_gate_, this->farthest_gate_);
     
     if (!this->trigger_thresholds_.empty()) {
         ESP_LOGCONFIG(TAG, "  Trigger Thresholds:");
         std::string thresh;
         for (size_t i = 0; i < this->trigger_thresholds_.size(); i++) {
             char buf[8];
             sprintf(buf, "%d ", this->trigger_thresholds_[i]);
             thresh += buf;
             if ((i + 1) % 8 == 0) {
                 ESP_LOGCONFIG(TAG, "    %s", thresh.c_str());
                 thresh.clear();
             }
         }
         if (!thresh.empty()) {
             ESP_LOGCONFIG(TAG, "    %s", thresh.c_str());
         }
     }
     
     if (!this->hold_thresholds_.empty()) {
         ESP_LOGCONFIG(TAG, "  Hold Thresholds:");
         std::string thresh;
         for (size_t i = 0; i < this->hold_thresholds_.size(); i++) {
             char buf[8];
             sprintf(buf, "%d ", this->hold_thresholds_[i]);
             thresh += buf;
             if ((i + 1) % 8 == 0) {
                 ESP_LOGCONFIG(TAG, "    %s", thresh.c_str());
                 thresh.clear();
             }
         }
         if (!thresh.empty()) {
             ESP_LOGCONFIG(TAG, "    %s", thresh.c_str());
         }
     }
 }
 
 void HLKLD2410SComponent::loop() {
     uint32_t now = millis();
     if (now - this->last_update_ < this->throttle_) {
         return;
     }
     this->last_update_ = now;
 
     static std::vector<uint8_t> buffer;
     
     while (available()) {
         uint8_t byte;
         if (read_byte(&byte)) {
             buffer.push_back(byte);
         }
     }
 
     while (buffer.size() >= 5) {
         if (this->output_mode_standard_) {
             // Standard data format: F4 F3 F2 F1 | length(2) | type(1) | state(1) | distance(2) | reserved(2) | energy(64) | F8 F7 F6 F5
             if (buffer.size() >= 4 && 
                 buffer[0] == 0xF1 && buffer[1] == 0xF2 && 
                 buffer[2] == 0xF3 && buffer[3] == 0xF4) {
                 
                 if (buffer.size() < 74) {  // Full standard packet size
                     return;  // Wait for more data
                 }
 
                 // Parse standard format
                 uint16_t length = buffer[5] << 8 | buffer[4];
                 uint8_t type = buffer[6];
                 uint8_t state = buffer[7];
                 uint16_t distance = buffer[9] << 8 | buffer[8];  // cm
 
                 ESP_LOGV(TAG, "Standard packet - Length: %u, Type: 0x%02X, State: %u, Distance: %u cm",
                          length, type, state, distance);
 
                 if (this->distance_sensor_ != nullptr) {
                     this->distance_sensor_->publish_state(distance / 100.0f);  // Convert to meters
                 }
 
                 bool presence = (state >= 2);  // 0/1 = no one, 2/3 = someone
                 if (this->presence_sensor_ != nullptr) {
                     this->presence_sensor_->publish_state(presence);
                 }
 
                 // Process energy values for each gate
                 for (size_t i = 0; i < 16; i++) {
                     uint8_t energy = buffer[12 + i];  // Energy values start after reserved bytes
                     auto it = this->gate_energy_sensors_.find(i);
                     if (it != this->gate_energy_sensors_.end() && it->second != nullptr) {
                         it->second->publish_state(energy);
                     }
                 }
 
                 buffer.erase(buffer.begin(), buffer.begin() + 74);
             } else {
                 buffer.erase(buffer.begin());
             }
         } else {
             // Minimal data format: 6E | state(1) | distance(2) | 62
             if (buffer[4] == 0x02 && buffer[3] == 0x6E && buffer[2] == 0x62) {
                 uint8_t target_state = buffer[0];
                 uint16_t distance = buffer[1];  // Distance in cm
 
                 ESP_LOGV(TAG, "Minimal packet - State: 0x%02X, Distance: %u cm", 
                          target_state, distance);
 
                 if (this->distance_sensor_ != nullptr) {
                     this->distance_sensor_->publish_state(distance / 100.0f);  // Convert to meters
                 }
 
                 bool presence = (target_state >= 2);
                 if (this->presence_sensor_ != nullptr) {
                     this->presence_sensor_->publish_state(presence);
                 }
 
                 buffer.erase(buffer.begin(), buffer.begin() + 5);
             } else {
                 buffer.erase(buffer.begin());
             }
         }
     }
 
     if (buffer.size() > 256) {
         buffer.clear();
     }
 }
 
 bool HLKLD2410SComponent::enable_configuration() {
     ESP_LOGI(TAG, "Enabling configuration mode");
     if (write_command_(CommandWord::ENABLE_CONFIGURATION, {0x01, 0x00})) {
         this->config_mode_ = true;
         if (this->config_mode_sensor_ != nullptr) {
             this->config_mode_sensor_->publish_state(true);
         }
         return true;
     }
     return false;
 }
 
 bool HLKLD2410SComponent::disable_configuration() {
     ESP_LOGI(TAG, "Disabling configuration mode");
     if (write_command_(CommandWord::DISABLE_CONFIGURATION)) {
         this->config_mode_ = false;
         if (this->config_mode_sensor_ != nullptr) {
             this->config_mode_sensor_->publish_state(false);
         }
         return true;
     }
     return false;
 }
 
 bool HLKLD2410SComponent::write_command_(CommandWord command, const std::vector<uint8_t> &data) {
     ESP_LOGV(TAG, "Writing command 0x%04X", static_cast<uint16_t>(command));
     
     // Write frame header
     write_array(CONFIG_FRAME_HEADER, sizeof(CONFIG_FRAME_HEADER));
     
     // Write data length (2 bytes, little endian)
     uint16_t length = data.size() + 2;  // +2 for command word
     write_byte(length & 0xFF);
     write_byte(length >> 8);
     
     // Write command word (2 bytes, little endian)
     uint16_t cmd = static_cast<uint16_t>(command);
     write_byte(cmd & 0xFF);
     write_byte(cmd >> 8);
     
     // Write data if any
     if (!data.empty()) {
         write_array(data.data(), data.size());
     }
     
     // Write frame end
     write_array(CONFIG_FRAME_END, sizeof(CONFIG_FRAME_END));
     
     // Wait for ACK
     return read_ack_(command);
 }
 
 bool HLKLD2410SComponent::read_ack_(CommandWord expected_cmd) {
     uint32_t start = millis();
     std::vector<uint8_t> buffer;
     buffer.reserve(32);  // Reserve space for the ACK frame
     
     ESP_LOGV(TAG, "Waiting for ACK for command 0x%04X", static_cast<uint16_t>(expected_cmd));
     
     while (millis() - start < ACK_TIMEOUT_MS) {
         if (available()) {
             uint8_t byte;
             read_byte(&byte);
             buffer.push_back(byte);
             
             // Look for frame header in the buffer
             if (buffer.size() >= 4) {
                 for (size_t i = 0; i <= buffer.size() - 4; i++) {
                     if (verify_frame_header_(&buffer[i], 4)) {
                         // Found header, ensure we have enough data for a complete ACK frame
                         if (buffer.size() >= i + 8) {  // Header(4) + Length(2) + Command(2)
                             uint16_t length = buffer[i+4] | (buffer[i+5] << 8);
                             uint16_t received_cmd = buffer[i+6] | (buffer[i+7] << 8);
                             
                             // Make sure we have the complete frame
                             if (buffer.size() >= i + 8 + length + 4) {  // Add frame end length
                                 ESP_LOGV(TAG, "Received ACK - CMD: 0x%04X, Length: %u", 
                                          received_cmd, length);
                                 
                                 // Verify frame end
                                 if (verify_frame_end_(&buffer[i + 8 + length], 4)) {
                                     // For most commands, status is in the next two bytes
                                     uint16_t status = 0;
                                     if (length >= 4) {  // Make sure we have status bytes
                                         status = buffer[i+8] | (buffer[i+9] << 8);
                                     }
                                     
                                     if (received_cmd == (static_cast<uint16_t>(expected_cmd) | 0x0100)) {
                                         ESP_LOGI(TAG, "Command 0x%04X acknowledged with status 0x%04X", 
                                                 static_cast<uint16_t>(expected_cmd), status);
                                         return (status == 0x0000);
                                     }
                                 }
                             }
                         }
                     }
                 }
             }
             
             // Prevent buffer from growing too large
             if (buffer.size() > 64) {
                 buffer.erase(buffer.begin(), buffer.begin() + 32);
             }
         }
         delay(1);
     }
     
     ESP_LOGW(TAG, "Command 0x%04X not acknowledged within timeout", 
              static_cast<uint16_t>(expected_cmd));
     return false;
 }
 
 bool HLKLD2410SComponent::verify_frame_header_(const uint8_t *buf, size_t len) {
     if (len < sizeof(CONFIG_FRAME_HEADER)) {
         return false;
     }
     return memcmp(buf, CONFIG_FRAME_HEADER, sizeof(CONFIG_FRAME_HEADER)) == 0;
 }
 
 bool HLKLD2410SComponent::verify_frame_end_(const uint8_t *buf, size_t len) {
     if (len < sizeof(CONFIG_FRAME_END)) {
         return false;
     }
     return memcmp(buf, CONFIG_FRAME_END, sizeof(CONFIG_FRAME_END)) == 0;
 }
 
 void HLKLD2410SComponent::check_uart_settings_() {
     auto *uart = (esphome::uart::UARTComponent *)this->parent_;
     if (uart->get_baud_rate() != 115200) {
         ESP_LOGE(TAG, "Incorrect baud rate! HLK-LD2410S requires 115200 baud. Current: %u", uart->get_baud_rate());
     }
     ESP_LOGI(TAG, "UART Settings - Baud: %u, Data Bits: 8, Stop Bits: 1, Parity: None", uart->get_baud_rate());
 }
 
 bool HLKLD2410SComponent::write_parameters_() {
     ESP_LOGI(TAG, "Writing general parameters");
     
     std::vector<uint8_t> data;
     
     // Add farthest gate parameter (0x05)
     data.push_back(0x05);
     data.push_back(0x00);
     data.push_back(this->farthest_gate_);
     data.push_back(0x00);
     data.push_back(0x00);
     data.push_back(0x00);
     
     // Add nearest gate parameter (0x0A)
     data.push_back(0x0A);
     data.push_back(0x00);
     data.push_back(this->nearest_gate_);
     data.push_back(0x00);
     data.push_back(0x00);
     data.push_back(0x00);
     
     // Add unmanned delay parameter (0x06)
     data.push_back(0x06);
     data.push_back(0x00);
     data.push_back(this->unmanned_delay_ & 0xFF);
     data.push_back((this->unmanned_delay_ >> 8) & 0xFF);
     data.push_back(0x00);
     data.push_back(0x00);
     
     // Add status report frequency parameter (0x02)
     data.push_back(0x02);
     data.push_back(0x00);
     uint8_t status_freq = static_cast<uint8_t>(this->status_report_freq_ * 2);  // Convert to 0.5Hz steps
     data.push_back(status_freq);
     data.push_back(0x00);
     data.push_back(0x00);
     data.push_back(0x00);
     
     // Add distance report frequency parameter (0x0C)
     data.push_back(0x0C);
     data.push_back(0x00);
     uint8_t dist_freq = static_cast<uint8_t>(this->distance_report_freq_ * 2);  // Convert to 0.5Hz steps
     data.push_back(dist_freq);
     data.push_back(0x00);
     data.push_back(0x00);
     data.push_back(0x00);
     
     // Add response speed parameter (0x0B)
     data.push_back(0x0B);
     data.push_back(0x00);
     data.push_back(this->response_speed_);
     data.push_back(0x00);
     data.push_back(0x00);
     data.push_back(0x00);
     
     return write_command_(CommandWord::WRITE_PARAMETERS, data);
 }
 
 bool HLKLD2410SComponent::write_trigger_thresholds_() {
     if (this->trigger_thresholds_.size() != 16) {
         ESP_LOGE(TAG, "Invalid trigger thresholds size: %d", this->trigger_thresholds_.size());
         return false;
     }
     
     std::vector<uint8_t> data;
     for (size_t i = 0; i < 16; i++) {
         data.push_back(i & 0xFF);
         data.push_back(0x00);
         data.push_back(this->trigger_thresholds_[i]);
         data.push_back(0x00);
         data.push_back(0x00);
         data.push_back(0x00);
     }
     
     return write_command_(CommandWord::WRITE_TRIGGER_THRESHOLD, data);
 }
 
 bool HLKLD2410SComponent::write_hold_thresholds_() {
     if (this->hold_thresholds_.size() != 16) {
         ESP_LOGE(TAG, "Invalid hold thresholds size: %d", this->hold_thresholds_.size());
         return false;
     }
     
     std::vector<uint8_t> data;
     for (size_t i = 0; i < 16; i++) {
         data.push_back(i & 0xFF);
         data.push_back(0x00);
         data.push_back(this->hold_thresholds_[i]);
         data.push_back(0x00);
         data.push_back(0x00);
         data.push_back(0x00);
     }
     
     return write_command_(CommandWord::WRITE_HOLD_THRESHOLD, data);
 }
 
 bool HLKLD2410SComponent::set_auto_threshold_() {
     std::vector<uint8_t> data = {
         this->auto_threshold_trigger_,
         0