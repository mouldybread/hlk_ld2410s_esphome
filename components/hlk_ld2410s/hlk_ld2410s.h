/**
 * HLK-LD2410S mmWave Radar Sensor component for ESPHome.
 * 
 * Author: github.com/mouldybread
 * Created: 2025-03-27 15:25:00 UTC
 */

 #pragma once

 #include "esphome/core/component.h"
 #include "esphome/components/uart/uart.h"
 #include "esphome/components/sensor/sensor.h"
 #include "esphome/components/binary_sensor/binary_sensor.h"
 #include "esphome/components/button/button.h"
 
 namespace esphome {
 namespace hlk_ld2410s {
 
 static const char *const TAG = "hlk_ld2410s";
 static const char *const CONF_CATEGORY = "hlk_ld2410s";
 
 // Error handling constants
 static const char *const ERROR_CONFIGURATION = "Configuration error";
 static const char *const ERROR_COMMUNICATION = "Communication error";
 static const char *const ERROR_VALIDATION = "Validation error";
 static const char *const ERROR_TIMEOUT = "Timeout error";
 
 // Protocol constants
 static const uint8_t MAX_GATES = 16;
 
 // Configuration frame constants
 static const uint8_t CONFIG_FRAME_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
 static const uint8_t CONFIG_FRAME_END[] = {0x04, 0x03, 0x02, 0x01};
 static const uint16_t CONFIG_FRAME_MIN_LENGTH = 8;
 static const uint32_t ACK_TIMEOUT_MS = 1000;
 static const uint32_t COMMAND_DELAY_MS = 100;
 
 // Data frame constants
 static const uint8_t DATA_FRAME_HEADER[] = {0xF4, 0xF3, 0xF2, 0xF1};
 static const uint8_t DATA_FRAME_END[] = {0x04, 0x03, 0x02, 0x01};
 static const uint16_t DATA_FRAME_MIN_LENGTH = 8;
 static const uint8_t DATA_FRAME_ENGINEERING_LENGTH = 34;
 static const uint8_t DATA_FRAME_SIMPLE_LENGTH = 6;
 
 // Parameter validation constants
 static const uint8_t MIN_RESPONSE_SPEED = 5;
 static const uint8_t MAX_RESPONSE_SPEED = 10;
 static const uint8_t MIN_UNMANNED_DELAY = 10;
 static const uint8_t MAX_UNMANNED_DELAY = 120;
 static const float MIN_REPORT_FREQUENCY = 0.5;
 static const float MAX_REPORT_FREQUENCY = 8.0;
 static const uint8_t MIN_GATE_INDEX = 0;
 static const uint8_t MAX_GATE_INDEX = 15;
 static const uint8_t MIN_THRESHOLD = 0;
 static const uint8_t MAX_THRESHOLD = 100;
 static const uint8_t MIN_TRIGGER_FACTOR = 1;
 static const uint8_t MAX_TRIGGER_FACTOR = 5;
 static const uint8_t MIN_HOLD_FACTOR = 1;
 static const uint8_t MAX_HOLD_FACTOR = 5;
 static const uint8_t MIN_SCAN_TIME = 10;
 static const uint8_t MAX_SCAN_TIME = 250;
 
 // Engineering mode command types
 static const uint8_t CMD_ENGINEERING_DATA = 0x01;
 static const uint8_t CMD_SIMPLE_DATA = 0x02;
 
 // Command words for configuration
 enum class CommandWord : uint16_t {
     ENABLE_CONFIGURATION = 0xFF00,
     DISABLE_CONFIGURATION = 0xFE00,
     SWITCH_OUTPUT_MODE = 0x7A00,
     READ_FIRMWARE_VERSION = 0x0000,
     WRITE_SERIAL_NUMBER = 0x1000,
     READ_SERIAL_NUMBER = 0x1100,
     WRITE_PARAMETERS = 0x7000,
     READ_PARAMETERS = 0x7100,
     AUTO_THRESHOLD = 0x0900,
     WRITE_TRIGGER_THRESHOLD = 0x7200,
     READ_TRIGGER_THRESHOLD = 0x7300,
     WRITE_HOLD_THRESHOLD = 0x7600,
     READ_HOLD_THRESHOLD = 0x7700
 };
 
 // Response status codes
 enum class ResponseStatus : uint8_t {
     SUCCESS = 0x00,
     INVALID_COMMAND = 0x01,
     INVALID_LENGTH = 0x02,
     INVALID_DATA = 0x03,
     DEVICE_BUSY = 0x04
 };
 
 // Forward declaration for button classes
 class HLKLD2410SComponent;
 
 class EnableConfigButton : public button::Button {
  public:
     explicit EnableConfigButton(HLKLD2410SComponent *parent) : parent_(parent) {}
     void press() override;
  protected:
     HLKLD2410SComponent *parent_;
 };
 
 class DisableConfigButton : public button::Button {
  public:
     explicit DisableConfigButton(HLKLD2410SComponent *parent) : parent_(parent) {}
     void press() override;
  protected:
     HLKLD2410SComponent *parent_;
 };
 
 class HLKLD2410SComponent : public Component, public uart::UARTDevice {
  public:
     explicit HLKLD2410SComponent(uart::UARTComponent *parent) : uart::UARTDevice(parent) {}
 
     void setup() override;
     void loop() override;
     void dump_config() override;
     float get_setup_priority() const override { return setup_priority::LATE; }
     const char *get_component_type() const override { return CONF_CATEGORY; }
 
     void set_throttle(uint32_t throttle) { this->throttle_ = throttle; }
     void set_distance_sensor(sensor::Sensor *distance_sensor) { this->distance_sensor_ = distance_sensor; }
     void set_presence_sensor(binary_sensor::BinarySensor *presence_sensor) { this->presence_sensor_ = presence_sensor; }
     void set_config_mode_sensor(binary_sensor::BinarySensor *config_mode_sensor) { this->config_mode_sensor_ = config_mode_sensor; }
     void set_enable_config_button(EnableConfigButton *button) { this->enable_config_button_ = button; }
     void set_disable_config_button(DisableConfigButton *button) { this->disable_config_button_ = button; }
     void set_output_mode(bool engineering_mode) { this->output_mode_ = engineering_mode; }
     
     void set_response_speed(uint8_t speed) {
         if (speed >= MIN_RESPONSE_SPEED && speed <= MAX_RESPONSE_SPEED) {
             this->response_speed_ = speed;
         }
     }
     
     void set_unmanned_delay(uint8_t delay) {
         if (delay >= MIN_UNMANNED_DELAY && delay <= MAX_UNMANNED_DELAY) {
             this->unmanned_delay_ = delay;
         }
     }
     
     void set_status_report_frequency(float frequency) {
         if (frequency >= MIN_REPORT_FREQUENCY && frequency <= MAX_REPORT_FREQUENCY) {
             this->status_report_frequency_ = frequency;
         }
     }
     
     void set_distance_report_frequency(float frequency) {
         if (frequency >= MIN_REPORT_FREQUENCY && frequency <= MAX_REPORT_FREQUENCY) {
             this->distance_report_frequency_ = frequency;
         }
     }
     
     void set_farthest_gate(uint8_t gate) {
         if (gate >= MIN_GATE_INDEX && gate <= MAX_GATE_INDEX) {
             this->farthest_gate_ = gate;
         }
     }
     
     void set_nearest_gate(uint8_t gate) {
         if (gate >= MIN_GATE_INDEX && gate <= MAX_GATE_INDEX) {
             this->nearest_gate_ = gate;
         }
     }
     
     void set_trigger_thresholds(const std::vector<uint8_t> &thresholds) {
         if (thresholds.size() == MAX_GATES) {
             this->trigger_thresholds_ = thresholds;
         }
     }
     
     void set_hold_thresholds(const std::vector<uint8_t> &thresholds) {
         if (thresholds.size() == MAX_GATES) {
             this->hold_thresholds_ = thresholds;
         }
     }
     
     void set_auto_threshold(uint8_t trigger_factor, uint8_t hold_factor, uint8_t scan_time) {
         if (trigger_factor >= MIN_TRIGGER_FACTOR && trigger_factor <= MAX_TRIGGER_FACTOR &&
             hold_factor >= MIN_HOLD_FACTOR && hold_factor <= MAX_HOLD_FACTOR &&
             scan_time >= MIN_SCAN_TIME && scan_time <= MAX_SCAN_TIME) {
             this->trigger_factor_ = trigger_factor;
             this->hold_factor_ = hold_factor;
             this->scan_time_ = scan_time;
         }
     }
     
     void set_gate_energy_sensor(uint8_t gate, sensor::Sensor *energy_sensor) {
         if (gate < MAX_GATES) {
             this->gate_energy_sensors_[gate] = energy_sensor;
         }
     }
 
  protected:
     // Button classes need access to enable/disable methods
     friend class EnableConfigButton;
     friend class DisableConfigButton;
 
     // Configuration methods
     void apply_cached_config_();
     void on_frame_(const std::vector<uint8_t> &data);
     void read_data_();
     void handle_engineering_data_(const std::vector<uint8_t> &data);
     void handle_simple_data_(const std::vector<uint8_t> &data);
     bool enable_configuration_();
     bool disable_configuration_();
     bool set_output_mode_();
     bool set_response_speed_();
     bool set_unmanned_delay_();
     bool set_status_report_frequency_();
     bool set_distance_report_frequency_();
     bool set_auto_threshold_();
     bool set_farthest_gate_();
     bool set_nearest_gate_();
     bool set_trigger_thresholds_();
     bool set_hold_thresholds_();
 
     // UART communication methods
     bool read_byte_(uint8_t *data, uint32_t timeout = 100);
     bool read_array_(std::vector<uint8_t> &data, size_t count);
     bool write_array_(const std::vector<uint8_t> &data);
     void reset_input_buffer_();
     uint8_t calculate_checksum_(const std::vector<uint8_t> &data);
     bool send_command_(CommandWord cmd, const std::vector<uint8_t> &payload = {});
     bool wait_for_ack_(uint32_t timeout = ACK_TIMEOUT_MS);
     bool read_ack_();
     bool validate_response_(const std::vector<uint8_t> &data);
 
     // Component state
     uint32_t throttle_{50};
     uint32_t last_update_{0};
     sensor::Sensor *distance_sensor_{nullptr};
     binary_sensor::BinarySensor *presence_sensor_{nullptr};
     binary_sensor::BinarySensor *config_mode_sensor_{nullptr};
     EnableConfigButton *enable_config_button_{nullptr};
     DisableConfigButton *disable_config_button_{nullptr};
     bool output_mode_{true};
     uint8_t response_speed_{5};
     uint8_t unmanned_delay_{40};
     float status_report_frequency_{0.5};
     float distance_report_frequency_{0.5};
     uint8_t farthest_gate_{12};
     uint8_t nearest_gate_{0};
     std::vector<uint8_t> trigger_thresholds_{};
     std::vector<uint8_t> hold_thresholds_{};
     uint8_t trigger_factor_{0};
     uint8_t hold_factor_{0};
     uint8_t scan_time_{0};
     sensor::Sensor *gate_energy_sensors_[MAX_GATES]{nullptr};
 };
 
 }  // namespace hlk_ld2410s
 }  // namespace esphome