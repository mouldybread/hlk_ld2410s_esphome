/*
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * Created by github.com/mouldybread
 * Creation Date/Time: 2025-03-27 06:17:32 UTC
 */

 #include "hlk_ld2410s.h"

 namespace esphome {
 namespace hlk_ld2410s {
 
 static const uint8_t FRAME_HEADER = 0x6E;
 static const uint8_t FRAME_END = 0x62;
 static const uint8_t FRAME_LENGTH = 5;  // Header(1) + State(1) + Distance(2) + End(1)
 
 // Protocol constants
 static const uint8_t CONFIG_FRAME_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
 static const uint8_t CONFIG_FRAME_END[] = {0x04, 0x03, 0x02, 0x01};
 
 HLKLD2410SComponent::HLKLD2410SComponent(uart::UARTComponent *parent) : uart::UARTDevice(parent) {
   ESP_LOGD(TAG, "Initializing HLK-LD2410S");
 }
 
 void HLKLD2410SComponent::setup() {
   ESP_LOGCONFIG(TAG, "Setting up HLK-LD2410S");
   last_update_ = millis();
   
   // Initialize config mode sensor to known state
   if (config_mode_sensor_ != nullptr) {
     config_mode_sensor_->publish_state(false);
   }
 }
 
 void HLKLD2410SComponent::loop() {
   const uint32_t now = millis();
   
   // Handle throttling
   if (throttle_ms_ > 0 && (now - last_update_) < throttle_ms_) {
     // Clear the buffer if we're throttled to prevent data buildup
     while (available() >= FRAME_LENGTH) {
       read();
     }
     return;
   }
 
   while (available() >= FRAME_LENGTH) {
     uint8_t header = read();
     if (header != FRAME_HEADER) {
       ESP_LOGV(TAG, "Invalid header: 0x%02X", header);
       continue;  // Look for frame header
     }
 
     uint8_t state = read();
     uint8_t distance_low = read();
     uint8_t distance_high = read();
     uint8_t end = read();
 
     if (end != FRAME_END) {
       ESP_LOGV(TAG, "Invalid end byte: 0x%02X", end);
       continue;  // Invalid frame
     }
 
     // Parse target state
     // 0/1 indicates no one; 2/3 indicate someone
     bool presence = (state >= 2);
     
     // Parse distance (little endian format)
     uint16_t distance = (distance_high << 8) | distance_low;
 
     ESP_LOGV(TAG, "Frame received - State: %u, Distance: %u cm, Presence: %s", 
              state, distance, presence ? "true" : "false");
 
     // Update sensors
     if (this->presence_sensor_ != nullptr) {
       this->presence_sensor_->publish_state(presence);
       ESP_LOGV(TAG, "Published presence: %s", presence ? "ON" : "OFF");
     }
     if (this->distance_sensor_ != nullptr && distance > 0) {
       this->distance_sensor_->publish_state(distance);
       ESP_LOGV(TAG, "Published distance: %u cm", distance);
     }
 
     last_update_ = now;
     break;  // Only process one valid frame per loop when throttling
   }
 }
 
 void HLKLD2410SComponent::dump_config() {
   ESP_LOGCONFIG(TAG, "HLK-LD2410S:");
   LOG_SENSOR("  ", "Distance", this->distance_sensor_);
   LOG_BINARY_SENSOR("  ", "Presence", this->presence_sensor_);
   LOG_BINARY_SENSOR("  ", "Config Mode", this->config_mode_sensor_);
   ESP_LOGCONFIG(TAG, "  Throttle: %ums", this->throttle_ms_);
   ESP_LOGCONFIG(TAG, "  Configuration Mode: %s", this->in_config_mode_ ? "ON" : "OFF");
 }
 
 bool HLKLD2410SComponent::write_command_(uint16_t command, const uint8_t *data, size_t len) {
   // Write frame header
   this->write_array(CONFIG_FRAME_HEADER, sizeof(CONFIG_FRAME_HEADER));
   
   // Write data length (2 bytes, little endian)
   uint16_t total_len = 2 + len;  // command (2 bytes) + data length
   this->write_byte(total_len & 0xFF);
   this->write_byte((total_len >> 8) & 0xFF);
   
   // Write command (2 bytes, little endian)
   this->write_byte(command & 0xFF);
   this->write_byte((command >> 8) & 0xFF);
   
   // Write data if any
   if (data != nullptr && len > 0) {
     this->write_array(data, len);
   }
   
   // Write frame end
   this->write_array(CONFIG_FRAME_END, sizeof(CONFIG_FRAME_END));
   
   return true;
 }
 
 bool HLKLD2410SComponent::read_ack_(uint16_t expected_command) {
   uint32_t start_time = millis();
   
   while ((millis() - start_time) < 1000) {  // 1 second timeout
     if (available() >= 8) {  // Minimum ACK size
       uint8_t header[4];
       for (int i = 0; i < 4; i++) {
         header[i] = read();
         if (header[i] != CONFIG_FRAME_HEADER[i]) {
           ESP_LOGW(TAG, "Invalid ACK header");
           return false;
         }
       }
       
       uint16_t len = read() | (read() << 8);
       uint16_t cmd = read() | (read() << 8);
       
       if (cmd != (expected_command | 0x0100)) {  // ACK commands are original + 0x0100
         ESP_LOGW(TAG, "Unexpected ACK command: 0x%04X", cmd);
         return false;
       }
       
       // Read rest of data and verify frame end
       for (uint16_t i = 0; i < len - 2; i++) {
         read();
       }
       
       uint8_t end[4];
       for (int i = 0; i < 4; i++) {
         end[i] = read();
         if (end[i] != CONFIG_FRAME_END[i]) {
           ESP_LOGW(TAG, "Invalid ACK end");
           return false;
         }
       }
       
       return true;
     }
     delay(1);
   }
   
   ESP_LOGW(TAG, "ACK timeout");
   return false;
 }
 
 void HLKLD2410SComponent::enable_configuration() {
   ESP_LOGD(TAG, "Enabling configuration mode");
   
   uint8_t data[] = {0x01, 0x00};  // Enable configuration command value
   if (write_command_(CMD_ENABLE_CONFIG, data, sizeof(data))) {
     if (read_ack_(CMD_ENABLE_CONFIG)) {
       ESP_LOGI(TAG, "Configuration mode enabled");
       in_config_mode_ = true;
       if (config_mode_sensor_ != nullptr) {
         config_mode_sensor_->publish_state(true);
       }
     } else {
       ESP_LOGW(TAG, "Failed to enable configuration mode");
     }
   }
 }
 
 void HLKLD2410SComponent::disable_configuration() {
   ESP_LOGD(TAG, "Disabling configuration mode");
   
   if (write_command_(CMD_DISABLE_CONFIG)) {
     if (read_ack_(CMD_DISABLE_CONFIG)) {
       ESP_LOGI(TAG, "Configuration mode disabled");
       in_config_mode_ = false;
       if (config_mode_sensor_ != nullptr) {
         config_mode_sensor_->publish_state(false);
       }
     } else {
       ESP_LOGW(TAG, "Failed to disable configuration mode");
     }
   }
 }
 
 void esphome::hlk_ld2410s::HLKLD2410SComponent::set_response_speed(uint8_t speed) {
   if (!in_config_mode_) {
     ESP_LOGW(TAG, "Must be in configuration mode to set response speed");
     return;
   }
 
   if (speed > 9) {
     ESP_LOGW(TAG, "Invalid response speed value (0-9): %u", speed);
     return;
   }
 
   ESP_LOGD(TAG, "Setting response speed to %u", speed);
   
   uint8_t data[] = {speed};
   if (write_command_(CMD_SET_RESPONSE_SPEED, data, sizeof(data))) {
     if (read_ack_(CMD_SET_RESPONSE_SPEED)) {
       ESP_LOGI(TAG, "Response speed set to %u", speed);
     } else {
       ESP_LOGW(TAG, "Failed to set response speed");
     }
   }
 }
 
 }  // namespace hlk_ld2410s
 }  // namespace esphome