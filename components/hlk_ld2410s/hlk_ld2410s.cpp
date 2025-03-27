/**
 * HLK-LD2410S mmWave Radar Sensor Component for ESPHome.
 * 
 * Created by github.com/mouldybread
 * Creation Date/Time: 2025-03-27 13:06:32 UTC
 */

 #include "hlk_ld2410s.h"
 #include "esphome/core/log.h"
 
 namespace esphome {
 namespace hlk_ld2410s {
 
 static const char *const TAG = "hlk_ld2410s";
 
 void EnableConfigButton::press_action() { this->parent_->enable_configuration(); }
 void DisableConfigButton::press_action() { this->parent_->disable_configuration(); }
 
 void HLKLD2410SComponent::setup() {
     // Clear any garbage data in the buffer
     this->flush();
     
     // Set initial state
     if (this->config_mode_sensor_ != nullptr) {
         this->config_mode_sensor_->publish_state(false);
     }
 }
 
 void HLKLD2410SComponent::loop() {
     uint32_t now = millis();
     
     // Throttle updates
     if (now - this->last_update_ < this->throttle_) {
         return;
     }
     this->last_update_ = now;
 
     uint8_t header[4];
     if (available() >= sizeof(header)) {
         if (read_array(header, sizeof(header)) != sizeof(header)) {
             ESP_LOGW(TAG, "Failed to read frame header");
             return;
         }
 
         if (!verify_frame_header_(header, sizeof(header))) {
             // Clear buffer and return if header is invalid
             flush();
             return;
         }
 
         // Read frame type and data length
         uint8_t frame_type;
         uint16_t data_length;
         if (!read_byte(&frame_type) || !read_byte(&data_length)) {
             ESP_LOGW(TAG, "Failed to read frame type or length");
             return;
         }
 
         // Read frame data
         std::vector<uint8_t> data(data_length);
         if (read_array(data.data(), data_length) != data_length) {
             ESP_LOGW(TAG, "Failed to read frame data");
             return;
         }
 
         // Read and verify frame end
         uint8_t end[4];
         if (read_array(end, sizeof(end)) != sizeof(end) || !verify_frame_end_(end, sizeof(end))) {
             ESP_LOGW(TAG, "Invalid frame end");
             return;
         }
 
         // Process frame based on output mode
         if (this->output_mode_standard_) {
             process_standard_frame_(frame_type, data_length, data.data());
         } else {
             process_simple_frame_(frame_type, data_length, data.data());
         }
     }
 }
 
 void HLKLD2410SComponent::process_simple_frame_(uint8_t frame_type, uint16_t data_length, const uint8_t *data) {
     // Process simple frame data
     if (data_length < 2) {
         ESP_LOGW(TAG, "Simple frame data too short");
         return;
     }
 
     uint16_t distance = (data[0] << 8) | data[1];
     
     if (this->distance_sensor_ != nullptr) {
         this->distance_sensor_->publish_state(distance);
     }
 
     bool presence = (distance > 0);
     if (this->presence_sensor_ != nullptr) {
         this->presence_sensor_->publish_state(presence);
     }
 
     // Update last presence time for unmanned delay calculation
     if (presence) {
         this->last_presence_detected_ = millis();
     }
 }
 
 void HLKLD2410SComponent::process_standard_frame_(uint8_t frame_type, uint16_t data_length, const uint8_t *data) {
     // Process standard frame data
     if (data_length < 4) {
         ESP_LOGW(TAG, "Standard frame data too short");
         return;
     }
 
     uint16_t distance = (data[0] << 8) | data[1];
     uint16_t energy = (data[2] << 8) | data[3];
     
     if (this->distance_sensor_ != nullptr) {
         this->distance_sensor_->publish_state(distance);
     }
 
     bool presence = (distance > 0);
     if (this->presence_sensor_ != nullptr) {
         this->presence_sensor_->publish_state(presence);
     }
 
     // Update last presence time for unmanned delay calculation
     if (presence) {
         this->last_presence_detected_ = millis();
     }
 
     // Process gate energy data if available
     if (data_length >= 4 + (MAX_GATES * 2)) {
         for (uint8_t i = 0; i < MAX_GATES; i++) {
             uint16_t gate_energy = (data[4 + (i * 2)] << 8) | data[5 + (i * 2)];
             auto it = this->gate_energy_sensors_.find(i);
             if (it != this->gate_energy_sensors_.end() && it->second != nullptr) {
                 it->second->publish_state(gate_energy);
             }
         }
     }
 }
 
 bool HLKLD2410SComponent::enable_configuration() {
     if (this->config_mode_) {
         ESP_LOGW(TAG, "Already in configuration mode");
         return false;
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
 
     ESP_LOGI(TAG, "Entered configuration mode");
     return true;
 }
 
 bool HLKLD2410SComponent::disable_configuration() {
     if (!this->config_mode_) {
         ESP_LOGW(TAG, "Not in configuration mode");
         return false;
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
 
     ESP_LOGI(TAG, "Exited configuration mode");
     return true;
 }
 
 bool HLKLD2410SComponent::write_command_(CommandWord command, const std::vector<uint8_t> &data) {
     // Write frame header
     write_array(CONFIG_FRAME_HEADER, sizeof(CONFIG_FRAME_HEADER));
 
     // Write command
     uint16_t cmd = static_cast<uint16_t>(command);
     write_byte(cmd >> 8);
     write_byte(cmd & 0xFF);
 
     // Write data length
     write_byte(data.size());
 
     // Write data if any
     if (!data.empty()) {
         write_array(data.data(), data.size());
     }
 
     // Write frame end
     write_array(CONFIG_FRAME_END, sizeof(CONFIG_FRAME_END));
 
     return true;
 }
 
 bool HLKLD2410SComponent::read_ack_(CommandWord expected_cmd) {
     uint8_t buffer[8];  // ACK frame is 8 bytes
     uint32_t start_time = millis();
     
     // Wait for data with timeout
     while ((millis() - start_time) < ACK_TIMEOUT_MS) {
         if (available() >= sizeof(buffer)) {
             if (read_array(buffer, sizeof(buffer)) != sizeof(buffer)) {
                 ESP_LOGW(TAG, "Failed to read complete ACK frame");
                 return false;
             }
 
             // Verify frame header (first 4 bytes should be FD FC FB FA)
             if (!verify_frame_header_(buffer, 4)) {
                 // Clear buffer and try again
                 flush();
                 continue;
             }
 
             // Extract command word (bytes 4-5)
             uint16_t received_cmd = (buffer[4] << 8) | buffer[5];
             
             // Verify frame end (last 2 bytes should be 04 03 02 01)
             if (!verify_frame_end_(&buffer[6], 2)) {
                 ESP_LOGW(TAG, "Invalid ACK frame end");
                 return false;
             }
 
             // Check if received command matches expected
             if (received_cmd != static_cast<uint16_t>(expected_cmd)) {
                 ESP_LOGW(TAG, "Unexpected ACK command: 0x%04X (expected: 0x%04X)", 
                          received_cmd, static_cast<uint16_t>(expected_cmd));
                 return false;
             }
 
             return true;
         }
         delay(1);  // Small delay to prevent tight loop
     }
 
     ESP_LOGW(TAG, "ACK timeout");
     return false;
 }
 
 bool HLKLD2410SComponent::verify_frame_header_(const uint8_t *buf, size_t len) {
     static const uint8_t EXPECTED_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
     
     if (len != sizeof(EXPECTED_HEADER)) {
         ESP_LOGV(TAG, "Invalid header length: %d", len);
         return false;
     }
 
     for (size_t i = 0; i < len; i++) {
         if (buf[i] != EXPECTED_HEADER[i]) {
             ESP_LOGV(TAG, "Invalid header byte at pos %d: 0x%02X (expected: 0x%02X)", 
                      i, buf[i], EXPECTED_HEADER[i]);
             return false;
         }
     }
 
     return true;
 }
 
 bool HLKLD2410SComponent::verify_frame_end_(const uint8_t *buf, size_t len) {
     static const uint8_t EXPECTED_END[] = {0x04, 0x03, 0x02, 0x01};
     
     if (len != sizeof(EXPECTED_END)) {
         ESP_LOGV(TAG, "Invalid end frame length: %d", len);
         return false;
     }
 
     for (size_t i = 0; i < len; i++) {
         if (buf[i] != EXPECTED_END[i]) {
             ESP_LOGV(TAG, "Invalid end byte at pos %d: 0x%02X (expected: 0x%02X)", 
                      i, buf[i], EXPECTED_END[i]);
             return false;
         }
     }
 
     return true;
 }
 
 }  // namespace hlk_ld2410s
 }  // namespace esphome