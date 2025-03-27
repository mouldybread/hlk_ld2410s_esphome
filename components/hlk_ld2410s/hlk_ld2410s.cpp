/**
 * HLK-LD2410S mmWave Radar Sensor Component for ESPHome.
 * 
 * Created by github.com/mouldybread
 * Creation Date/Time: 2025-03-27 13:14:53 UTC
 */

 #include "hlk_ld2410s.h"
 #include "esphome/core/log.h"
 
 namespace esphome {
 namespace hlk_ld2410s {
 
 // Remove the TAG definition as it's already in the header
 static const uint32_t UART_READ_TIMEOUT_MS = 50;
 static const uint32_t COMMAND_DELAY_MS = 100;
 static const size_t RX_BUFFER_SIZE = 256;
 
 void EnableConfigButton::press_action() { this->parent_->enable_configuration(); }
 void DisableConfigButton::press_action() { this->parent_->disable_configuration(); }
 
 void HLKLD2410SComponent::loop() {
     uint32_t now = millis();
     
     // Throttle updates
     if (now - this->last_update_ < this->throttle_) {
         return;
     }
     this->last_update_ = now;
 
     uint8_t header[4];
     if (available() >= sizeof(header)) {
         size_t read_size = read_array(header, sizeof(header));
         if (read_size < sizeof(header)) {
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
         uint8_t length_bytes[2];
         
         if (!read_byte(&frame_type)) {
             ESP_LOGW(TAG, "Failed to read frame type");
             return;
         }
         
         if (read_array(length_bytes, 2) != 2) {
             ESP_LOGW(TAG, "Failed to read data length");
             return;
         }
         
         uint16_t data_length = (length_bytes[0] << 8) | length_bytes[1];
 
         // Read frame data
         std::vector<uint8_t> data(data_length);
         if (read_array(data.data(), data_length) != data_length) {
             ESP_LOGW(TAG, "Failed to read frame data");
             return;
         }
 
         // Read and verify frame end
         uint8_t end[4];
         size_t end_size = read_array(end, sizeof(end));
         if (end_size < sizeof(end) || !verify_frame_end_(end, sizeof(end))) {
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
 
     // Try multiple times if needed
     for (int retry = 0; retry < 3; retry++) {
         if (retry > 0) {
             ESP_LOGW(TAG, "Retrying enable configuration (attempt %d)", retry + 1);
             delay(COMMAND_DELAY_MS * 2);
         }
 
         if (!write_command_(CommandWord::ENABLE_CONFIGURATION)) {
             ESP_LOGE(TAG, "Failed to send enable configuration command");
             continue;
         }
 
         if (read_ack_(CommandWord::ENABLE_CONFIGURATION)) {
             this->config_mode_ = true;
             if (this->config_mode_sensor_ != nullptr) {
                 this->config_mode_sensor_->publish_state(true);
             }
             ESP_LOGI(TAG, "Entered configuration mode");
             return true;
         }
     }
 
     ESP_LOGE(TAG, "Failed to enter configuration mode after 3 attempts");
     return false;
 }
 
 bool HLKLD2410SComponent::disable_configuration() {
     if (!this->config_mode_) {
         ESP_LOGW(TAG, "Not in configuration mode");
         return false;
     }
 
     // Try multiple times if needed
     for (int retry = 0; retry < 3; retry++) {
         if (retry > 0) {
             ESP_LOGW(TAG, "Retrying disable configuration (attempt %d)", retry + 1);
             delay(COMMAND_DELAY_MS * 2);
         }
 
         if (!write_command_(CommandWord::DISABLE_CONFIGURATION)) {
             ESP_LOGE(TAG, "Failed to send disable configuration command");
             continue;
         }
 
         if (read_ack_(CommandWord::DISABLE_CONFIGURATION)) {
             this->config_mode_ = false;
             if (this->config_mode_sensor_ != nullptr) {
                 this->config_mode_sensor_->publish_state(false);
             }
             ESP_LOGI(TAG, "Exited configuration mode");
             return true;
         }
     }
 
     ESP_LOGE(TAG, "Failed to exit configuration mode after 3 attempts");
     return false;
 }
 
 bool HLKLD2410SComponent::write_command_(CommandWord command, const std::vector<uint8_t> &data) {
     // Flush any existing data first
     flush();
     
     // Add a small delay before sending new command
     delay(COMMAND_DELAY_MS);
     
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
     
     // Flush the written data
     flush();
     
     // Add a small delay after sending command
     delay(COMMAND_DELAY_MS);
 
     return true;
 }
 
 bool HLKLD2410SComponent::read_ack_(CommandWord expected_cmd) {
     uint8_t buffer[RX_BUFFER_SIZE];
     size_t pos = 0;
     uint32_t start_time = millis();
     
     // Clear any existing data
     flush();
     
     while ((millis() - start_time) < UART_READ_TIMEOUT_MS) {
         if (available()) {
             if (pos >= sizeof(buffer)) {
                 ESP_LOGW(TAG, "Buffer overflow while reading ACK");
                 flush();
                 return false;
             }
             
             int byte = read();
             if (byte < 0) {
                 ESP_LOGW(TAG, "UART read error");
                 continue;
             }
             
             buffer[pos++] = static_cast<uint8_t>(byte);
             
             // Look for frame header in received data
             if (pos >= 4) {
                 for (size_t i = 0; i <= pos - 4; i++) {
                     if (verify_frame_header_(&buffer[i], 4)) {
                         // We found a valid header, now check if we have enough data for a complete frame
                         if (pos >= i + 8) {  // Complete ACK frame is 8 bytes
                             // Extract command word
                             uint16_t received_cmd = (buffer[i + 4] << 8) | buffer[i + 5];
                             
                             // Verify frame end
                             if (verify_frame_end_(&buffer[i + 6], 2)) {
                                 if (received_cmd != static_cast<uint16_t>(expected_cmd)) {
                                     ESP_LOGW(TAG, "Unexpected ACK command: 0x%04X, expected: 0x%04X", 
                                              received_cmd, static_cast<uint16_t>(expected_cmd));
                                     return false;
                                 }
                                 return true;
                             }
                         }
                     }
                 }
             }
         }
         delay(1);
     }
     
     ESP_LOGW(TAG, "ACK timeout after %u ms", UART_READ_TIMEOUT_MS);
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