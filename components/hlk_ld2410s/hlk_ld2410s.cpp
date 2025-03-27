/**
 * Implementation of HLK-LD2410S mmWave Radar Sensor component for ESPHome.
 * 
 * Author: mouldybread
 * Created: 2025-03-27 15:44:50 UTC
 */

 #include "hlk_ld2410s.h"
 #include "esphome/core/log.h"
 
 namespace esphome {
 namespace hlk_ld2410s {
 
 void HLKLD2410SComponent::setup() {
     ESP_LOGCONFIG(TAG, "Setting up HLK-LD2410S...");
     this->reset_input_buffer_();
     if (!this->enable_configuration_()) {
         ESP_LOGE(TAG, "%s: Failed to enable configuration mode", ERROR_CONFIGURATION);
         return;
     }
     this->apply_cached_config_();
     if (!this->disable_configuration_()) {
         ESP_LOGE(TAG, "%s: Failed to disable configuration mode", ERROR_CONFIGURATION);
     }
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
     ESP_LOGCONFIG(TAG, "  Update Interval: %ums", this->throttle_);
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
 
 void HLKLD2410SComponent::reset_input_buffer_() {
     while (this->available()) {
         this->read();
     }
 }
 
 bool HLKLD2410SComponent::read_byte_(uint8_t *data, uint32_t timeout) {
     uint32_t start = millis();
     while ((millis() - start) < timeout) {
         if (this->available()) {
             *data = this->read();
             return true;
         }
         yield();
     }
     ESP_LOGW(TAG, "%s: Read timeout", ERROR_TIMEOUT);
     return false;
 }
 
 bool HLKLD2410SComponent::read_array_(std::vector<uint8_t> &data, size_t count) {
     data.clear();
     data.reserve(count);
     
     for (size_t i = 0; i < count; i++) {
         uint8_t byte;
         if (!this->read_byte_(&byte)) {
             return false;
         }
         data.push_back(byte);
     }
     return true;
 }
 
 bool HLKLD2410SComponent::write_array_(const std::vector<uint8_t> &data) {
     if (data.empty()) {
         return true;
     }
     this->write_array(data.data(), data.size());
     return true;
 }
 
 void HLKLD2410SComponent::read_data_() {
     uint8_t data;
     std::vector<uint8_t> buffer;
 
     // Read until we find the header sequence
     while (this->available()) {
         if (!this->read_byte_(&data)) {
             return;
         }
         buffer.push_back(data);
 
         if (buffer.size() >= 4) {
             if (buffer[buffer.size() - 4] == DATA_FRAME_HEADER[0] &&
                 buffer[buffer.size() - 3] == DATA_FRAME_HEADER[1] &&
                 buffer[buffer.size() - 2] == DATA_FRAME_HEADER[2] &&
                 buffer[buffer.size() - 1] == DATA_FRAME_HEADER[3]) {
                 break;
             }
         }
     }
 
     // Read length bytes
     if (!this->read_byte_(&data)) {
         return;
     }
     uint16_t length = data;
     if (!this->read_byte_(&data)) {
         return;
     }
     length |= (data << 8);
 
     if (length < DATA_FRAME_MIN_LENGTH) {
         ESP_LOGW(TAG, "%s: Frame too short", ERROR_VALIDATION);
         return;
     }
 
     // Read command
     if (!this->read_byte_(&data)) {
         return;
     }
     uint8_t command = data;
 
     // Read payload
     std::vector<uint8_t> payload;
     if (!this->read_array_(payload, length - 1)) {
         return;
     }
 
     // Read checksum
     if (!this->read_byte_(&data)) {
         return;
     }
     uint8_t checksum = data;
 
     // Verify checksum
     std::vector<uint8_t> check_data;
     check_data.reserve(4 + 2 + 1 + payload.size());
     check_data.insert(check_data.end(), DATA_FRAME_HEADER, DATA_FRAME_HEADER + 4);
     check_data.push_back(length & 0xFF);
     check_data.push_back((length >> 8) & 0xFF);
     check_data.push_back(command);
     check_data.insert(check_data.end(), payload.begin(), payload.end());
 
     if (checksum != this->calculate_checksum_(check_data)) {
         ESP_LOGW(TAG, "%s: Invalid checksum", ERROR_VALIDATION);
         return;
     }
 
     // Process command
     switch (command) {
         case static_cast<uint8_t>(CommandType::CMD_ENGINEERING_DATA):
             this->handle_engineering_data_(payload);
             break;
         case static_cast<uint8_t>(CommandType::CMD_SIMPLE_DATA):
             this->handle_simple_data_(payload);
             break;
         default:
             ESP_LOGW(TAG, "Unknown command: 0x%02X", command);
             break;
     }
 }
 
 void HLKLD2410SComponent::handle_engineering_data_(const std::vector<uint8_t> &data) {
     if (data.size() < DATA_FRAME_ENGINEERING_LENGTH) {
         ESP_LOGW(TAG, "%s: Engineering data frame too short", ERROR_VALIDATION);
         return;
     }
 
     const uint8_t head = data[0];
     if (head != 0xAA) {
         ESP_LOGW(TAG, "%s: Invalid engineering data header", ERROR_VALIDATION);
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
     for (uint8_t i = 0; i < MAX_GATES; i++) {
         if (this->gate_energy_sensors_[i] != nullptr) {
             const uint8_t energy = data[i + 6];
             this->gate_energy_sensors_[i]->publish_state(energy);
         }
     }
 }
 
 void HLKLD2410SComponent::handle_simple_data_(const std::vector<uint8_t> &data) {
     if (data.size() < DATA_FRAME_SIMPLE_LENGTH) {
         ESP_LOGW(TAG, "%s: Simple data frame too short", ERROR_VALIDATION);
         return;
     }
 
     const uint8_t head = data[0];
     if (head != 0xAA) {
         ESP_LOGW(TAG, "%s: Invalid simple data header", ERROR_VALIDATION);
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
     if (this->config_mode_sensor_ != nullptr) {
         this->config_mode_sensor_->publish_state(true);
     }
 
     return this->send_command_(CommandWord::ENABLE_CONFIGURATION);
 }
 
 bool HLKLD2410SComponent::disable_configuration_() {
     if (this->config_mode_sensor_ != nullptr) {
         this->config_mode_sensor_->publish_state(false);
     }
 
     return this->send_command_(CommandWord::DISABLE_CONFIGURATION);
 }
 
 void HLKLD2410SComponent::apply_cached_config_() {
     bool success = true;
 
     if (!this->set_output_mode_()) {
         ESP_LOGE(TAG, "%s: Failed to set output mode", ERROR_CONFIGURATION);
         success = false;
     }
 
     if (!this->set_response_speed_()) {
         ESP_LOGE(TAG, "%s: Failed to set response speed", ERROR_CONFIGURATION);
         success = false;
     }
 
     if (!this->set_unmanned_delay_()) {
         ESP_LOGE(TAG, "%s: Failed to set unmanned delay", ERROR_CONFIGURATION);
         success = false;
     }
 
     if (!this->set_status_report_frequency_()) {
         ESP_LOGE(TAG, "%s: Failed to set status report frequency", ERROR_CONFIGURATION);
         success = false;
     }
 
     if (!this->set_distance_report_frequency_()) {
         ESP_LOGE(TAG, "%s: Failed to set distance report frequency", ERROR_CONFIGURATION);
         success = false;
     }
 
     if (!this->set_farthest_gate_()) {
         ESP_LOGE(TAG, "%s: Failed to set farthest gate", ERROR_CONFIGURATION);
         success = false;
     }
 
     if (!this->set_nearest_gate_()) {
         ESP_LOGE(TAG, "%s: Failed to set nearest gate", ERROR_CONFIGURATION);
         success = false;
     }
 
     if (!this->trigger_thresholds_.empty() && !this->hold_thresholds_.empty()) {
         if (!this->set_trigger_thresholds_()) {
             ESP_LOGE(TAG, "%s: Failed to set trigger thresholds", ERROR_CONFIGURATION);
             success = false;
         }
 
         if (!this->set_hold_thresholds_()) {
             ESP_LOGE(TAG, "%s: Failed to set hold thresholds", ERROR_CONFIGURATION);
             success = false;
         }
     }
 
     if (this->trigger_factor_ > 0 && this->hold_factor_ > 0 && this->scan_time_ > 0) {
         if (!this->set_auto_threshold_()) {
             ESP_LOGE(TAG, "%s: Failed to set auto threshold", ERROR_CONFIGURATION);
             success = false;
         }
     }
 
     if (!success) {
         ESP_LOGE(TAG, "%s: One or more configuration commands failed", ERROR_CONFIGURATION);
     }
 }
 
 bool HLKLD2410SComponent::send_command_(CommandWord cmd, const std::vector<uint8_t> &payload) {
     std::vector<uint8_t> data;
     data.insert(data.end(), CONFIG_FRAME_HEADER, CONFIG_FRAME_HEADER + 4);
     
     uint16_t length = payload.size() + 1;  // +1 for command byte
     data.push_back(length & 0xFF);
     data.push_back((length >> 8) & 0xFF);
     
     data.push_back(static_cast<uint8_t>(cmd));
     data.insert(data.end(), payload.begin(), payload.end());
     
     data.push_back(this->calculate_checksum_(data));
     
     if (!this->write_array_(data)) {
         ESP_LOGE(TAG, "%s: Failed to send command 0x%04X", ERROR_COMMUNICATION, static_cast<uint16_t>(cmd));
         return false;
     }
     
     return this->wait_for_ack_();
 }
 
 bool HLKLD2410SComponent::wait_for_ack_(uint32_t timeout) {
     uint32_t start = millis();
     while ((millis() - start) < timeout) {
         if (this->available()) {
             if (this->read_ack_()) {
                 return true;
             }
         }
         yield();
     }
     ESP_LOGW(TAG, "%s: No ACK received", ERROR_TIMEOUT);
     return false;
 }
 
 bool HLKLD2410SComponent::read_ack_() {
     std::vector<uint8_t> ack_data;
     if (!this->read_array_(ack_data, CONFIG_FRAME_MIN_LENGTH)) {
         return false;
     }
 
     return this->validate_response_(ack_data);
 }
 
 bool HLKLD2410SComponent::validate_response_(const std::vector<uint8_t> &data) {
     if (data.size() < CONFIG_FRAME_MIN_LENGTH) {
         ESP_LOGW(TAG, "%s: Response too short", ERROR_VALIDATION);
         return false;
     }
 
     // Verify header
     for (size_t i = 0; i < 4; i++) {
         if (data[i] != CONFIG_FRAME_HEADER[i]) {
             ESP_LOGW(TAG, "%s: Invalid response header", ERROR_VALIDATION);
             return false;
         }
     }
 
     // Verify length
     uint16_t length = data[4] | (data[5] << 8);
     if (length + 7 != data.size()) {  // 7 = header(4) + length(2) + checksum(1)
         ESP_LOGW(TAG, "%s: Invalid response length", ERROR_VALIDATION);
         return false;
     }
 
     // Verify checksum
     std::vector<uint8_t> check_data(data.begin(), data.end() - 1);
     if (data.back() != this->calculate_checksum_(check_data)) {
         ESP_LOGW(TAG, "%s: Invalid response checksum", ERROR_VALIDATION);
         return false;
     }
 
     // Check response status
     ResponseStatus status = static_cast<ResponseStatus>(data[6]);
     if (status != ResponseStatus::SUCCESS) {
         ESP_LOGW(TAG, "%s: Command failed with status 0x%02X", ERROR_VALIDATION, static_cast<uint8_t>(status));
         return false;
     }
 
     return true;
 }
 
 uint8_t HLKLD2410SComponent::calculate_checksum_(const std::vector<uint8_t> &data) {
     uint8_t sum = 0;
     for (uint8_t byte : data) {
         sum += byte;
     }
     return sum;
 }
 
 bool HLKLD2410SComponent::set_output_mode_() {
     return this->send_command_(CommandWord::SWITCH_OUTPUT_MODE, {static_cast<uint8_t>(this->output_mode_ ? 0x01 : 0x00)});
 }
 
 bool HLKLD2410SComponent::set_response_speed_() {
     return this->send_command_(CommandWord::WRITE_PARAMETERS, {this->response_speed_});
 }
 
 bool HLKLD2410SComponent::set_unmanned_delay_() {
     return this->send_command_(CommandWord::WRITE_PARAMETERS, {this->unmanned_delay_});
 }
 
 bool HLKLD2410SComponent::set_status_report_frequency_() {
     return this->send_command_(CommandWord::WRITE_PARAMETERS, {static_cast<uint8_t>(this->status_report_frequency_ * 10)});
 }
 
 bool HLKLD2410SComponent::set_distance_report_frequency_() {
     return this->send_command_(CommandWord::WRITE_PARAMETERS, {static_cast<uint8_t>(this->distance_report_frequency_ * 10)});
 }
 
 bool HLKLD2410SComponent::set_auto_threshold_() {
     return this->send_command_(CommandWord::AUTO_THRESHOLD, {
         this->trigger_factor_,
         this->hold_factor_,
         this->scan_time_,
         0  // Reserved
     });
 }
 
 bool HLKLD2410SComponent::set_farthest_gate_() {
     return this->send_command_(CommandWord::WRITE_PARAMETERS, {this->farthest_gate_});
 }
 
 bool HLKLD2410SComponent::set_nearest_gate_() {
     return this->send_command_(CommandWord::WRITE_PARAMETERS, {this->nearest_gate_});
 }
 
 bool HLKLD2410SComponent::set_trigger_thresholds_() {
     return this->send_command_(CommandWord::WRITE_TRIGGER_THRESHOLD, this->trigger_thresholds_);
 }
 
 bool HLKLD2410SComponent::set_hold_thresholds_() {
     return this->send_command_(CommandWord::WRITE_HOLD_THRESHOLD, this->hold_thresholds_);
 }
 
 }  // namespace hlk_ld2410s
 }  // namespace esphome