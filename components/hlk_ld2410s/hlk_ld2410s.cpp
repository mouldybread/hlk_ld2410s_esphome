/**
 * HLK-LD2410S mmWave Radar Sensor Component for ESPHome.
 * 
 * Created by github.com/mouldybread
 * Creation Date/Time: 2025-03-27 13:23:47 UTC
 */

 #include "hlk_ld2410s.h"
 #include "esphome/core/log.h"
 
 namespace esphome {
 namespace hlk_ld2410s {
 
 static const uint32_t UART_READ_TIMEOUT_MS = 50;
 static const uint32_t COMMAND_DELAY_MS = 100;
 static const size_t RX_BUFFER_SIZE = 256;
 
 void EnableConfigButton::press_action() { this->parent_->enable_configuration(); }
 void DisableConfigButton::press_action() { this->parent_->disable_configuration(); }
 
 void HLKLD2410SComponent::setup() {
     // Clear any garbage data in the buffer
     flush();
     delay(COMMAND_DELAY_MS);  // Give some time for the sensor to initialize
     
     // Set initial state
     if (this->config_mode_sensor_ != nullptr) {
         this->config_mode_sensor_->publish_state(false);
     }
 }
 
 void HLKLD2410SComponent::dump_data_(const char* prefix, const uint8_t* data, size_t len) {
     if (len == 0) return;
     
     char hex[128];
     char* ptr = hex;
     for (size_t i = 0; i < len && i < 16; i++) {  // Limit to 16 bytes to avoid buffer overflow
         ptr += sprintf(ptr, "%02X ", data[i]);
     }
     if (len > 16) {
         strcat(ptr, "...");
     }
     ESP_LOGV(TAG, "%s: %s", prefix, hex);
 }
 
 void HLKLD2410SComponent::loop() {
     uint32_t now = millis();
     
     // Throttle updates
     if (now - this->last_update_ < this->throttle_) {
         return;
     }
     this->last_update_ = now;
 
     // Look for frame header
     while (available() >= 4) {  // We need at least 4 bytes for the header
         uint8_t peek_byte = peek();  // Look at next byte without removing it
         
         if (peek_byte == 0xFD) {  // Potential start of header
             uint8_t header[4];
             size_t header_read = read_array(header, sizeof(header));
             
             if (header_read == sizeof(header) && verify_frame_header_(header, sizeof(header))) {
                 // Valid header found, now read rest of frame
                 uint8_t frame_type;
                 uint8_t length_bytes[2];
                 
                 if (!read_byte(&frame_type)) {
                     ESP_LOGV(TAG, "Failed to read frame type");
                     continue;
                 }
                 
                 size_t bytes_read = read_array(length_bytes, 2);
                 if (bytes_read < 2) {
                     ESP_LOGV(TAG, "Failed to read data length");
                     continue;
                 }
                 
                 uint16_t data_length = (length_bytes[0] << 8) | length_bytes[1];
                 
                 // Sanity check the data length
                 if (data_length > 256) {  // Arbitrary reasonable maximum
                     ESP_LOGW(TAG, "Invalid data length: %u", data_length);
                     continue;
                 }
                 
                 // Wait until we have all the data
                 if (available() < data_length + 4) {  // data + end frame
                     ESP_LOGV(TAG, "Waiting for more data");
                     return;  // Come back when we have more data
                 }
 
                 // Read frame data
                 std::vector<uint8_t> data(data_length);
                 size_t data_read = read_array(data.data(), data_length);
                 if (data_read < data_length) {
                     ESP_LOGV(TAG, "Short read on frame data: expected %u, got %u", data_length, data_read);
                     continue;
                 }
 
                 // Read and verify frame end
                 uint8_t end[4];
                 size_t end_size = read_array(end, sizeof(end));
                 if (end_size < sizeof(end) || !verify_frame_end_(end, sizeof(end))) {
                     ESP_LOGV(TAG, "Invalid frame end");
                     continue;
                 }
 
                 // We have a complete valid frame, process it
                 if (this->output_mode_standard_) {
                     process_standard_frame_(frame_type, data_length, data.data());
                 } else {
                     process_simple_frame_(frame_type, data_length, data.data());
                 }
                 
                 return;  // Successfully processed a frame
             } else {
                 // Invalid header, skip one byte and continue looking
                 read();  // Remove the byte we peeked at
                 ESP_LOGV(TAG, "Skipping invalid header byte: 0x%02X", peek_byte);
             }
         } else {
             // Not a header byte, skip it
             read();  // Remove the byte we peeked at
             ESP_LOGV(TAG, "Skipping non-header byte: 0x%02X", peek_byte);
         }
     }
 }
 
 void HLKLD2410SComponent::process_simple_frame_(uint8_t frame_type, uint16_t data_length, const uint8_t *data) {
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
 
     if (presence) {
         this->last_presence_detected_ = millis();
     }
 }
 
 void HLKLD2410SComponent::process_standard_frame_(uint8_t frame_type, uint16_t data_length, const uint8_t *data) {
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
     flush();
     delay(COMMAND_DELAY_MS);
     
     write_array(CONFIG_FRAME_HEADER, sizeof(CONFIG_FRAME_HEADER));
 
     uint16_t cmd = static_cast<uint16_t>(command);
     write_byte(cmd >> 8);
     write_byte(cmd & 0xFF);
 
     write_byte(data.size());
 
     if (!data.empty()) {
         write_array(data.data(), data.size());
     }
 
     write_array(CONFIG_FRAME_END, sizeof(CONFIG_FRAME_END));
     
     flush();
     delay(COMMAND_DELAY_MS);
 
     return true;
 }
 
 bool HLKLD2410SComponent::read_ack_(CommandWord expected_cmd) {
     uint8_t buffer[RX_BUFFER_SIZE];
     size_t pos = 0;
     uint32_t start_time = millis();
     
     flush();
     
     while ((millis() - start_time) < UART_READ_TIMEOUT_MS) {
         if (available()) {
             if (pos >= sizeof(buffer)) {
                 ESP_LOGW(TAG, "Buffer overflow while reading ACK");
                 dump_data_("Buffer overflow", buffer, pos);
                 flush();
                 return false;
             }
             
             int byte = read();
             if (byte < 0) {
                 ESP_LOGW(TAG, "UART read error");
                 continue;
             }
             
             buffer[pos++] = static_cast<uint8_t>(byte);
             
             if (pos >= 4) {
                 for (size_t i = 0; i <= pos - 4; i++) {
                     if (verify_frame_header_(&buffer[i], 4)) {
                         if (pos >= i + 8) {
                             uint16_t received_cmd = (buffer[i + 4] << 8) | buffer[i + 5];
                             
                             if (verify_frame_end_(&buffer[i + 6], 2)) {
                                 if (received_cmd != static_cast<uint16_t>(expected_cmd)) {
                                     ESP_LOGW(TAG, "Unexpected ACK command: 0x%04X, expected: 0x%04X", 
                                              received_cmd, static_cast<uint16_t>(expected_cmd));
                                     dump_data_("Received ACK", &buffer[i], 8);
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
     if (pos > 0) {
         dump_data_("Timeout buffer", buffer, pos);
     }
     return false;
 }
 
 bool HLKLD2410SComponent::verify_frame_header_(const uint8_t *buf, size_t len) {
     static const uint8_t EXPECTED_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
     
     if (len != sizeof(EXPECTED_HEADER)) {
         ESP_LOGV(TAG, "Invalid header length: %d", len);
         dump_data_("Received header", buf, len);
         return false;
     }
 
     for (size_t i = 0; i < len; i++) {
         if (buf[i] != EXPECTED_HEADER[i]) {
             ESP_LOGV(TAG, "Invalid header byte at pos %d: 0x%02X (expected: 0x%02X)", 
                      i, buf[i], EXPECTED_HEADER[i]);
             dump_data_("Received header", buf, len);
             return false;
         }
     }
 
     return true;
 }
 
 bool HLKLD2410SComponent::verify_frame_end_(const uint8_t *buf, size_t len) {
     static const uint8_t EXPECTED_END[] = {0x04, 0x03, 0x02, 0x01};
     
     if (len != sizeof(EXPECTED_END)) {
         ESP_LOGV(TAG, "Invalid end frame length: %d", len);
         dump_data_("Received end", buf, len);
         return false;
     }
 
     for (size_t i = 0; i < len; i++) {
         if (buf[i] != EXPECTED_END[i]) {
             ESP_LOGV(TAG, "Invalid end byte at pos %d: 0x%02X (expected: 0x%02X)", 
                      i, buf[i], EXPECTED_END[i]);
             dump_data_("Received end", buf, len);
             return false;
         }
     }
 
     return true;
 }
 
 }  // namespace hlk_ld2410s
 }  // namespace esphome