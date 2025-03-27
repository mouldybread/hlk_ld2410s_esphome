/**
 * Implementation of HLK-LD2410S mmWave Radar Sensor component for ESPHome.
 * 
 * Author: github.com/mouldybread
 * Created: 2025-03-27 15:13:43 UTC
 */

 #include "hlk_ld2410s.h"
 #include "esphome/core/log.h"
 
 namespace esphome {
 namespace hlk_ld2410s {
 
 static const char *const TAG = "hlk_ld2410s";
 
 void HLKLD2410SComponent::setup() {
     ESP_LOGCONFIG(TAG, "Setting up HLK-LD2410S...");
     this->enable_configuration_();
     this->apply_cached_config_();
     this->disable_configuration_();
 }
 
 void HLKLD2410SComponent::loop() {
     const uint32_t now = millis();
     if (now - this->last_update_ < this->throttle_) {
         return;
     }
     this->last_update_ = now;
 
     while (this->available()) {
         this->read_data_();
     }
 }
 
 void HLKLD2410SComponent::dump_config() {
     ESP_LOGCONFIG(TAG, "HLK-LD2410S:");
     LOG_UPDATE_INTERVAL(this);
     LOG_SENSOR("  ", "Distance", this->distance_sensor_);
     LOG_BINARY_SENSOR("  ", "Presence", this->presence_sensor_);
     LOG_BINARY_SENSOR("  ", "Config Mode", this->config_mode_sensor_);
     ESP_LOGCONFIG(TAG, "  Output Mode: %s", this->output_mode_ ? "Engineering" : "Simple");
     ESP_LOGCONFIG(TAG, "  Response Speed: %d", this->response_speed_);
     ESP_LOGCONFIG(TAG, "  Unmanned Delay: %d", this->unmanned_delay_);
     ESP_LOGCONFIG(TAG, "  Status Report Frequency: %.1f", this->status_report_frequency_);
     ESP_LOGCONFIG(TAG, "  Distance Report Frequency: %.1f", this->distance_report_frequency_);
     ESP_LOGCONFIG(TAG, "  Farthest Gate: %d", this->farthest_gate_);
     ESP_LOGCONFIG(TAG, "  Nearest Gate: %d", this->nearest_gate_);
 }
 
 void HLKLD2410SComponent::apply_cached_config_() {
     if (!this->set_output_mode_()) {
         ESP_LOGE(TAG, "Failed to set output mode");
         return;
     }
 
     if (!this->set_response_speed_()) {
         ESP_LOGE(TAG, "Failed to set response speed");
         return;
     }
 
     if (!this->set_unmanned_delay_()) {
         ESP_LOGE(TAG, "Failed to set unmanned delay");
         return;
     }
 
     if (!this->set_status_report_frequency_()) {
         ESP_LOGE(TAG, "Failed to set status report frequency");
         return;
     }
 
     if (!this->set_distance_report_frequency_()) {
         ESP_LOGE(TAG, "Failed to set distance report frequency");
         return;
     }
 
     if (!this->set_farthest_gate_()) {
         ESP_LOGE(TAG, "Failed to set farthest gate");
         return;
     }
 
     if (!this->set_nearest_gate_()) {
         ESP_LOGE(TAG, "Failed to set nearest gate");
         return;
     }
 
     if (!this->trigger_thresholds_.empty() && !this->hold_thresholds_.empty()) {
         if (!this->set_trigger_thresholds_()) {
             ESP_LOGE(TAG, "Failed to set trigger thresholds");
             return;
         }
 
         if (!this->set_hold_thresholds_()) {
             ESP_LOGE(TAG, "Failed to set hold thresholds");
             return;
         }
     }
 
     if (this->trigger_factor_ > 0 && this->hold_factor_ > 0 && this->scan_time_ > 0) {
         if (!this->set_auto_threshold_()) {
             ESP_LOGE(TAG, "Failed to set auto threshold");
             return;
         }
     }
 }
 
 void HLKLD2410SComponent::read_data_() {
     uint8_t data;
     std::vector<uint8_t> buffer;
 
     // Read until we find the header sequence
     while (this->available()) {
         data = this->read();
         buffer.push_back(data);
 
         if (buffer.size() >= 4) {
             if (buffer[buffer.size() - 4] == 0xF4 &&
                 buffer[buffer.size() - 3] == 0xF3 &&
                 buffer[buffer.size() - 2] == 0xF2 &&
                 buffer[buffer.size() - 1] == 0xF1) {
                 break;
             }
         }
     }
 
     // Read length bytes
     if (!this->read_byte(&data)) {
         return;
     }
     uint16_t length = data;
     if (!this->read_byte(&data)) {
         return;
     }
     length |= (data << 8);
 
     // Read command
     if (!this->read_byte(&data)) {
         return;
     }
     uint8_t command = data;
 
     // Read payload
     std::vector<uint8_t> payload;
     for (uint16_t i = 0; i < length - 1; i++) {
         if (!this->read_byte(&data)) {
             return;
         }
         payload.push_back(data);
     }
 
     // Read checksum
     if (!this->read_byte(&data)) {
         return;
     }
     uint8_t checksum = data;
 
     // Verify checksum
     std::vector<uint8_t> check_data = {
         0xF4, 0xF3, 0xF2, 0xF1,
         static_cast<uint8_t>(length & 0xFF),
         static_cast<uint8_t>((length >> 8) & 0xFF),
         command
     };
     check_data.insert(check_data.end(), payload.begin(), payload.end());
 
     if (checksum != this->calculate_checksum_(check_data)) {
         ESP_LOGW(TAG, "Invalid checksum");
         return;
     }
 
     // Process command
     switch (command) {
         case 0x01:
             this->handle_engineering_data_(payload);
             break;
         case 0x02:
             this->handle_simple_data_(payload);
             break;
         default:
             ESP_LOGW(TAG, "Unknown command: 0x%02X", command);
             break;
     }
 }
 
 void HLKLD2410SComponent::handle_engineering_data_(const std::vector<uint8_t> &data) {
     if (data.size() < 34) {
         ESP_LOGW(TAG, "Invalid engineering data length");
         return;
     }
 
     const uint8_t head = data[0];
     if (head != 0xAA) {
         ESP_LOGW(TAG, "Invalid engineering data header");
         return;
     }
 
     const uint8_t target_state = data[1];
     const uint16_t moving_distance = data[2] | (data[3] << 8);
     const uint16_t static_distance = data[4] | (data[5] << 8);
     
     // Update sensors
     if (this->distance_sensor_ != nullptr) {
         float distance = (target_state & 0x01) ? moving_distance / 100.0f : 
                         (target_state & 0x02) ? static_distance / 100.0f : 0.0f;
         this->distance_sensor_->publish_state(distance);
     }
 
     if (this->presence_sensor_ != nullptr) {
         this->presence_sensor_->publish_state(target_state > 0);
     }
 
     // Update gate energy sensors
     for (uint8_t i = 0; i < 16; i++) {
         if (this->gate_energy_sensors_[i] != nullptr) {
             const uint8_t energy = data[i + 6];
             this->gate_energy_sensors_[i]->publish_state(energy);
         }
     }
 }
 
 void HLKLD2410SComponent::handle_simple_data_(const std::vector<uint8_t> &data) {
     if (data.size() < 6) {
         ESP_LOGW(TAG, "Invalid simple data length");
         return;
     }
 
     const uint8_t head = data[0];
     if (head != 0xAA) {
         ESP_LOGW(TAG, "Invalid simple data header");
         return;
     }
 
     const uint8_t target_state = data[1];
     const uint16_t distance = data[2] | (data[3] << 8);
 
     // Update sensors
     if (this->distance_sensor_ != nullptr) {
         this->distance_sensor_->publish_state(distance / 100.0f);
     }
 
     if (this->presence_sensor_ != nullptr) {
         this->presence_sensor_->publish_state(target_state > 0);
     }
 }
 
 bool HLKLD2410SComponent::enable_configuration_() {
     std::vector<uint8_t> data = {
         0xFD, 0xFC, 0xFB, 0xFA,  // Header
         0x04, 0x00,              // Length
         0x04,                    // Command (Enable Configuration)
         0x00                     // Reserved
     };
 
     data.push_back(calculate_checksum_(data));
     this->write_array(data);
 
     if (this->config_mode_sensor_ != nullptr) {
         this->config_mode_sensor_->publish_state(true);
     }
 
     return true;
 }
 
 bool HLKLD2410SComponent::disable_configuration_() {
     std::vector<uint8_t> data = {
         0xFD, 0xFC, 0xFB, 0xFA,  // Header
         0x04, 0x00,              // Length
         0x05,                    // Command (Disable Configuration)
         0x00                     // Reserved
     };
 
     data.push_back(calculate_checksum_(data));
     this->write_array(data);
 
     if (this->config_mode_sensor_ != nullptr) {
         this->config_mode_sensor_->publish_state(false);
     }
 
     return true;
 }
 
 bool HLKLD2410SComponent::set_output_mode_() {
     std::vector<uint8_t> data = {
         0xFD, 0xFC, 0xFB, 0xFA,  // Header
         0x04, 0x00,              // Length
         0x06,                    // Command (Set Output Mode)
         static_cast<uint8_t>(this->output_mode_ ? 0x01 : 0x00)
     };
 
     data.push_back(calculate_checksum_(data));
     this->write_array(data);
     return true;
 }
 
 bool HLKLD2410SComponent::set_response_speed_() {
     std::vector<uint8_t> data = {
         0xFD, 0xFC, 0xFB, 0xFA,  // Header
         0x04, 0x00,              // Length
         0x07,                    // Command (Set Response Speed)
         static_cast<uint8_t>(this->response_speed_)
     };
 
     data.push_back(calculate_checksum_(data));
     this->write_array(data);
     return true;
 }
 
 bool HLKLD2410SComponent::set_unmanned_delay_() {
     std::vector<uint8_t> data = {
         0xFD, 0xFC, 0xFB, 0xFA,  // Header
         0x04, 0x00,              // Length
         0x08,                    // Command (Set Unmanned Delay)
         static_cast<uint8_t>(this->unmanned_delay_)
     };
 
     data.push_back(calculate_checksum_(data));
     this->write_array(data);
     return true;
 }
 
 bool HLKLD2410SComponent::set_status_report_frequency_() {
     std::vector<uint8_t> data = {
         0xFD, 0xFC, 0xFB, 0xFA,  // Header
         0x04, 0x00,              // Length
         0x09,                    // Command (Set Status Report Frequency)
         static_cast<uint8_t>(this->status_report_frequency_ * 10)
     };
 
     data.push_back(calculate_checksum_(data));
     this->write_array(data);
     return true;
 }
 
 bool HLKLD2410SComponent::set_distance_report_frequency_() {
     std::vector<uint8_t> data = {
         0xFD, 0xFC, 0xFB, 0xFA,  // Header
         0x04, 0x00,              // Length
         0x0A,                    // Command (Set Distance Report Frequency)
         static_cast<uint8_t>(this->distance_report_frequency_ * 10)
     };
 
     data.push_back(calculate_checksum_(data));
     this->write_array(data);
     return true;
 }
 
 bool HLKLD2410SComponent::set_auto_threshold_() {
     std::vector<uint8_t> data = {
         0xFD, 0xFC, 0xFB, 0xFA,  // Header
         0x07, 0x00,              // Length
         0x0B,                    // Command (Set Auto Threshold)
         this->trigger_factor_,
         this->hold_factor_,
         this->scan_time_,
         0                        // Reserved
     };
 
     data.push_back(calculate_checksum_(data));
     this->write_array(data);
     return true;
 }
 
 bool HLKLD2410SComponent::set_farthest_gate_() {
     std::vector<uint8_t> data = {
         0xFD, 0xFC, 0xFB, 0xFA,  // Header
         0x04, 0x00,              // Length
         0x0C,                    // Command (Set Farthest Gate)
         static_cast<uint8_t>(this->farthest_gate_)
     };
 
     data.push_back(calculate_checksum_(data));
     this->write_array(data);
     return true;
 }
 
 bool HLKLD2410SComponent::set_nearest_gate_() {
     std::vector<uint8_t> data = {
         0xFD, 0xFC, 0xFB, 0xFA,  // Header
         0x04, 0x00,              // Length
         0x0D,                    // Command (Set Nearest Gate)
         static_cast<uint8_t>(this->nearest_gate_)
     };
 
     data.push_back(calculate_checksum_(data));
     this->write_array(data);
     return true;
 }
 
 bool HLKLD2410SComponent::set_trigger_thresholds_() {
     std::vector<uint8_t> data = {
         0xFD, 0xFC, 0xFB, 0xFA,  // Header
         0x14, 0x00,              // Length (20)
         0x0E,                    // Command (Set Trigger Thresholds)
     };
 
     // Add thresholds
     data.insert(data.end(), this->trigger_thresholds_.begin(), this->trigger_thresholds_.end());
     
     // Add checksum
     data.push_back(calculate_checksum_(data));
     this->write_array(data);
     return true;
 }
 
 bool HLKLD2410SComponent::set_hold_thresholds_() {
     std::vector<uint8_t> data = {
         0xFD, 0xFC, 0xFB, 0xFA,  // Header
         0x14, 0x00,              // Length (20)
         0x0F,                    // Command (Set Hold Thresholds)
     };
 
     // Add thresholds
     data.insert(data.end(), this->hold_thresholds_.begin(), this->hold_thresholds_.end());
     
     // Add checksum
     data.push_back(calculate_checksum_(data));
     this->write_array(data);
     return true;
 }
 
 uint8_t HLKLD2410SComponent::calculate_checksum_(const std::vector<uint8_t> &data) {
     uint8_t sum = 0;
     for (uint8_t byte : data) {
         sum += byte;
     }
     return sum;
 }
 
 }  // namespace hlk_ld2410s
 }  // namespace esphome