/**
 * HLK-LD2410S mmWave Radar Sensor Component for ESPHome.
 * 
 * Created by github.com/mouldybread
 * Creation Date/Time: 2025-03-27 13:58:04 UTC
 */

 #include "hlk_ld2410s.h"
 #include "esphome/core/log.h"
 
 namespace esphome {
 namespace hlk_ld2410s {
 
 static const uint32_t UART_BAUD_RATE = 256000;
 static const uint32_t UART_READ_TIMEOUT_MS = 50;
 static const uint32_t COMMAND_DELAY_MS = 100;
 static const size_t RX_BUFFER_SIZE = 256;
 
 void EnableConfigButton::press_action() { this->parent_->enable_configuration(); }
 void DisableConfigButton::press_action() { this->parent_->disable_configuration(); }
 
 void HLKLD2410SComponent::setup() {
     ESP_LOGI(TAG, "Setting up HLK-LD2410S...");
     
     // Set UART baud rate
     this->check_uart_settings_();
     
     // Clear any garbage data in the buffer
     this->flush();
     delay(COMMAND_DELAY_MS);  // Give some time for the sensor to initialize
     
     // Set initial state
     if (this->config_mode_sensor_ != nullptr) {
         this->config_mode_sensor_->publish_state(false);
     }
 
     ESP_LOGI(TAG, "Setup complete");
 }
 
 void HLKLD2410SComponent::check_uart_settings_() {
     auto *uart = (UARTComponent *)this->parent_;
     if (uart->get_baud_rate() != UART_BAUD_RATE) {
         ESP_LOGE(TAG, "Incorrect baud rate! HLK-LD2410S requires 256000 baud. Current: %u", uart->get_baud_rate());
     }
     ESP_LOGI(TAG, "UART Settings - Baud: %u, Data Bits: 8, Stop Bits: 1, Parity: None", uart->get_baud_rate());
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
 
     if (!available()) {
         return;  // No data available
     }
 
     ESP_LOGV(TAG, "Data available: %d bytes", available());
 
     // Look for frame header
     while (available() >= 4) {  // We need at least 4 bytes for the header
         uint8_t peek_byte = peek();  // Look at next byte without removing it
         
         if (peek_byte == 0xFD) {  // Potential start of header
             ESP_LOGV(TAG, "Found potential frame header");
             uint8_t header[4];
             size_t header_read = read_array(header, sizeof(header));
             
             if (header_read == sizeof(header) && verify_frame_header_(header, sizeof(header))) {
                 ESP_LOGD(TAG, "Valid frame header found");
                 // Valid header found, now read rest of frame
                 uint8_t frame_type;
                 uint8_t length_bytes[2];
                 
                 if (!read_byte(&frame_type)) {
                     ESP_LOGW(TAG, "Failed to read frame type");
                     continue;
                 }
                 
                 size_t bytes_read = read_array(length_bytes, 2);
                 if (bytes_read < 2) {
                     ESP_LOGW(TAG, "Failed to read data length");
                     continue;
                 }
                 
                 uint16_t data_length = (length_bytes[0] << 8) | length_bytes[1];
                 ESP_LOGD(TAG, "Frame type: 0x%02X, length: %u", frame_type, data_length);
                 
                 // Sanity check the data length
                 if (data_length > RX_BUFFER_SIZE) {
                     ESP_LOGW(TAG, "Invalid data length: %u", data_length);
                     continue;
                 }
                 
                 // Wait until we have all the data
                 if (available() < data_length + 4) {  // data + end frame
                     ESP_LOGV(TAG, "Waiting for more data (available: %d, needed: %d)", 
                             available(), data_length + 4);
                     return;  // Come back when we have more data
                 }
 
                 // Read frame data
                 std::vector<uint8_t> data(data_length);
                 size_t data_read = read_array(data.data(), data_length);
                 if (data_read < data_length) {
                     ESP_LOGW(TAG, "Short read on frame data: expected %u, got %u", data_length, data_read);
                     continue;
                 }
 
                 // Read and verify frame end
                 uint8_t end[4];
                 size_t end_size = read_array(end, sizeof(end));
                 if (end_size < sizeof(end) || !verify_frame_end_(end, sizeof(end))) {
                     ESP_LOGW(TAG, "Invalid frame end");
                     dump_data_("Invalid end frame", end, end_size);
                     continue;
                 }
 
                 // We have a complete valid frame, process it
                 ESP_LOGD(TAG, "Processing frame type 0x%02X with length %u", frame_type, data_length);
                 dump_data_("Frame data", data.data(), data_length);
                 
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
     ESP_LOGD(TAG, "Simple frame - distance: %u cm", distance);
     
     if (this->distance_sensor_ != nullptr) {
         this->distance_sensor_->publish_state(distance / 100.0f);  // Convert to meters
     }
 
     bool presence = (distance > 0);
     if (this->presence_sensor_ != nullptr) {
         this->presence_sensor_->publish_state(presence);
     }
 
     if (presence) {
         this->last_presence_detected_ = millis();
         ESP_LOGI(TAG, "Presence detected at distance: %u cm", distance);
     }
 }
 
 void HLKLD2410SComponent::process_standard_frame_(uint8_t frame_type, uint16_t data_length, const uint8_t *data) {
     if (data_length < 4) {
         ESP_LOGW(TAG, "Standard frame data too short");
         return;
     }
 
     uint16_t distance = (data[0] << 8) | data[1];
     ESP_LOGD(TAG, "Standard frame - distance: %u cm", distance);
     
     // Process energy values for each gate if available
     if (data_length >= 4 + MAX_GATES) {
         for (size_t i = 0; i < MAX_GATES; i++) {
             uint8_t energy = data[4 + i];
             auto it = this->gate_energy_sensors_.find(i);
             if (it != this->gate_energy_sensors_.end() && it->second != nullptr) {
                 it->second->publish_state(energy);
             }
         }
     }
 
     if (this->distance_sensor_ != nullptr) {
         this->distance_sensor_->publish_state(distance / 100.0f);  // Convert to meters
     }
 
     bool presence = (distance > 0);
     if (this->presence_sensor_ != nullptr) {
         this->presence_sensor_->publish_state(presence);
     }
 
     if (presence) {
         this->last_presence_detected_ = millis();
         ESP_LOGI(TAG, "Presence detected at distance: %u cm with motion energy: %u", 
                  distance, data[2]);
     }
 }
 
 bool HLKLD2410SComponent::enable_configuration() {
     ESP_LOGI(TAG, "Enabling configuration mode");
     if (write_command_(CommandWord::ENABLE_CONFIGURATION)) {
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
     ESP_LOGD(TAG, "Writing command 0x%04X", static_cast<uint16_t>(command));
     
     // Write frame header
     write_array(CONFIG_FRAME_HEADER, sizeof(CONFIG_FRAME_HEADER));
     
     // Write command word (2 bytes, big endian)
     uint16_t cmd = static_cast<uint16_t>(command);
     write_byte(cmd >> 8);
     write_byte(cmd & 0xFF);
     
     // Write data length (2 bytes, big endian)
     uint16_t length = data.size();
     write_byte(length >> 8);
     write_byte(length & 0xFF);
     
     // Write data if any
     if (length > 0) {
         write_array(data.data(), length);
     }
     
     // Write frame end
     write_array(CONFIG_FRAME_END, sizeof(CONFIG_FRAME_END));
     
     // Wait for ACK
     return read_ack_(command);
 }
 
 bool HLKLD2410SComponent::read_ack_(CommandWord expected_cmd) {
     uint32_t start = millis();
     
     while (millis() - start < ACK_TIMEOUT_MS) {
         if (available() >= CONFIG_FRAME_MIN_LENGTH) {
             uint8_t header[4];
             if (read_array(header, sizeof(header)) == sizeof(header) && 
                 verify_frame_header_(header, sizeof(header))) {
                 
                 uint8_t cmd_bytes[2];
                 if (read_array(cmd_bytes, sizeof(cmd_bytes)) == sizeof(cmd_bytes)) {
                     uint16_t received_cmd = (cmd_bytes[0] << 8) | cmd_bytes[1];
                     
                     if (received_cmd == static_cast<uint16_t>(expected_cmd)) {
                         ESP_LOGI(TAG, "Command 0x%04X acknowledged", static_cast<uint16_t>(expected_cmd));
                         return true;
                     }
                 }
             }
         }
         delay(10);
     }
     
     ESP_LOGW(TAG, "Command 0x%04X not acknowledged within timeout", static_cast<uint16_t>(expected_cmd));
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
 
 }  // namespace hlk_ld2410s
 }  // namespace esphome