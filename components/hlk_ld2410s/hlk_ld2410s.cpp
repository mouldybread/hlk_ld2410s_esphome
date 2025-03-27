/**
 * HLK-LD2410S mmWave Radar Sensor Component for ESPHome.
 * 
 * Created by github.com/mouldybread
 * Creation Date/Time: 2025-03-27 14:29:01 UTC
 */

 #include "hlk_ld2410s.h"
 #include "esphome/core/log.h"
 
 namespace esphome {
 namespace hlk_ld2410s {
 
 static const uint32_t UART_BAUD_RATE = 115200;
 static const uint32_t UART_READ_TIMEOUT_MS = 50;
 static const uint32_t COMMAND_DELAY_MS = 100;
 static const size_t RX_BUFFER_SIZE = 256;
 
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
 
     // Switch to standard mode if configured
     if (this->output_mode_standard_) {
         if (!this->switch_output_mode_(true)) {
             ESP_LOGE(TAG, "Failed to switch to standard output mode");
         }
     }
 
     ESP_LOGI(TAG, "Setup complete");
 }
 
 void HLKLD2410SComponent::check_uart_settings_() {
     auto *uart = (esphome::uart::UARTComponent *)this->parent_;
     if (uart->get_baud_rate() != UART_BAUD_RATE) {
         ESP_LOGE(TAG, "Incorrect baud rate! HLK-LD2410S requires 115200 baud. Current: %u", uart->get_baud_rate());
     }
     ESP_LOGI(TAG, "UART Settings - Baud: %u, Data Bits: 8, Stop Bits: 1, Parity: None", uart->get_baud_rate());
 }
 
 void HLKLD2410SComponent::dump_data_(const char* prefix, const uint8_t* data, size_t len) {
     if (len == 0) return;
     
     char hex[512];
     char* ptr = hex;
     for (size_t i = 0; i < len && i < 32; i++) {
         ptr += sprintf(ptr, "%02X ", data[i]);
         if ((i + 1) % 16 == 0 && i < len - 1) {
             ptr += sprintf(ptr, "\n");
         }
     }
     if (len > 32) {
         strcat(ptr, "...");
     }
     ESP_LOGD(TAG, "%s (%d bytes):\n%s", prefix, len, hex);
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
 
                 ESP_LOGD(TAG, "Standard packet - Length: %u, Type: 0x%02X, State: %u, Distance: %u cm",
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
 
                 ESP_LOGD(TAG, "Minimal packet - State: 0x%02X, Distance: %u cm", 
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
 
 bool HLKLD2410SComponent::switch_output_mode_(bool standard_mode) {
     ESP_LOGI(TAG, "Switching to %s mode", standard_mode ? "standard" : "minimal");
     
     if (!enable_configuration()) {
         ESP_LOGE(TAG, "Failed to enable configuration mode");
         return false;
     }
 
     std::vector<uint8_t> data = {
         0x00, 0x00, 0x00,  // First three bytes are always 0
         static_cast<uint8_t>(standard_mode ? 0x01 : 0x00),  // Mode selection
         0x00, 0x00  // Last two bytes are always 0
     };
 
     bool success = write_command_(CommandWord::SWITCH_OUTPUT_MODE, data);
     
     if (!disable_configuration()) {
         ESP_LOGE(TAG, "Failed to disable configuration mode");
         return false;
     }
 
     return success;
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