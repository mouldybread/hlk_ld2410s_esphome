/**
 * HLK-LD2410S mmWave Radar Sensor component for ESPHome.
 * 
 * Author: mouldybread
 * Created: 2025-03-27 15:54:48 UTC
 */

 #pragma once

 #include "esphome/core/component.h"
 #include "esphome/core/helpers.h"
 #include "esphome/components/uart/uart.h"
 #include "esphome/components/sensor/sensor.h"
 #include "esphome/components/binary_sensor/binary_sensor.h"
 #include "esphome/components/button/button.h"
 
 namespace esphome {
 namespace hlk_ld2410s {
 
 static const char *const TAG = "hlk_ld2410s";
 static const char *const ERROR_COMMUNICATION = "Communication Error";
 static const char *const ERROR_VALIDATION = "Validation Error";
 static const char *const ERROR_TIMEOUT = "Timeout Error";
 static const char *const ERROR_CONFIGURATION = "Configuration Error";
 
 static const uint8_t DATA_FRAME_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
 static const uint8_t CONFIG_FRAME_HEADER[4] = {0xFD, 0xFC, 0xFB, 0xFA};
 static const uint8_t DATA_FRAME_MIN_LENGTH = 4;
 static const uint8_t CONFIG_FRAME_MIN_LENGTH = 7;
 static const uint8_t DATA_FRAME_ENGINEERING_LENGTH = 22;
 static const uint8_t DATA_FRAME_SIMPLE_LENGTH = 4;
 static const uint8_t MAX_GATES = 16;
 static const uint8_t MIN_RESPONSE_SPEED = 5;
 static const uint8_t MAX_RESPONSE_SPEED = 10;
 static const uint8_t MIN_UNMANNED_DELAY = 10;
 static const uint8_t MAX_UNMANNED_DELAY = 120;
 static const float MIN_REPORT_FREQUENCY = 0.5f;
 static const float MAX_REPORT_FREQUENCY = 8.0f;
 static const uint8_t MIN_FARTHEST_GATE = 1;
 static const uint8_t MAX_FARTHEST_GATE = 16;
 static const uint8_t MIN_NEAREST_GATE = 0;
 static const uint8_t MAX_NEAREST_GATE = 15;
 static const uint8_t MIN_GATE_THRESHOLD = 0;
 static const uint8_t MAX_GATE_THRESHOLD = 100;
 static const uint8_t MIN_FACTOR = 1;
 static const uint8_t MAX_FACTOR = 5;
 static const uint8_t MIN_SCAN_TIME = 10;
 static const uint8_t MAX_SCAN_TIME = 250;
 
 enum class CommandWord : uint8_t {
     ENABLE_CONFIGURATION = 0x04,
     DISABLE_CONFIGURATION = 0x05,
     READ_FIRMWARE_VERSION = 0x06,
     WRITE_PARAMETERS = 0x07,
     READ_PARAMETERS = 0x08,
     SWITCH_OUTPUT_MODE = 0x0A,
     RESTART = 0x0B,
     RESTORE_FACTORY_SETTINGS = 0x0C,
     READ_TRIGGER_THRESHOLD = 0x0D,
     WRITE_TRIGGER_THRESHOLD = 0x0E,
     READ_HOLD_THRESHOLD = 0x0F,
     WRITE_HOLD_THRESHOLD = 0x10,
     AUTO_THRESHOLD = 0x11,
 };
 
 enum class ResponseStatus : uint8_t {
     SUCCESS = 0x01,
     FAILURE = 0x02,
     COMMAND_ERROR = 0x03,
 };
 
 enum class CommandType : uint8_t {
     CMD_ENGINEERING_DATA = 0x01,
     CMD_SIMPLE_DATA = 0x02,
 };
 
 class EnableConfigButton;
 class DisableConfigButton;
 
 class HLKLD2410SComponent : public Component, public uart::UARTDevice {
  public:
     // Default constructor for ESPHome registration
     HLKLD2410SComponent() = default;
     
     // Constructor with UART parent
     explicit HLKLD2410SComponent(uart::UARTComponent *parent) : uart::UARTDevice(parent) {}
 
     // Method to set UART parent after construction
     void set_uart_parent(uart::UARTComponent *parent) { UARTDevice::set_uart_parent(parent); }
 
     void setup() override;
     void loop() override;
     void dump_config() override;
     float get_setup_priority() const override { return setup_priority::LATE; }
 
     void set_distance_sensor(sensor::Sensor *distance_sensor) { distance_sensor_ = distance_sensor; }
     void set_presence_sensor(binary_sensor::BinarySensor *presence_sensor) { presence_sensor_ = presence_sensor; }
     void set_config_mode_sensor(binary_sensor::BinarySensor *config_mode_sensor) { config_mode_sensor_ = config_mode_sensor; }
     void set_enable_config_button(EnableConfigButton *enable_config_button) { enable_config_button_ = enable_config_button; }
     void set_disable_config_button(DisableConfigButton *disable_config_button) { disable_config_button_ = disable_config_button; }
     void set_gate_energy_sensor(uint8_t gate, sensor::Sensor *gate_energy_sensor) { gate_energy_sensors_[gate] = gate_energy_sensor; }
     void set_throttle(uint32_t throttle) { throttle_ = throttle; }
     void set_output_mode(bool output_mode) { output_mode_ = output_mode; }
     void set_response_speed(uint8_t response_speed) { response_speed_ = response_speed; }
     void set_unmanned_delay(uint8_t unmanned_delay) { unmanned_delay_ = unmanned_delay; }
     void set_status_report_frequency(float status_report_frequency) { status_report_frequency_ = status_report_frequency; }
     void set_distance_report_frequency(float distance_report_frequency) { distance_report_frequency_ = distance_report_frequency; }
     void set_farthest_gate(uint8_t farthest_gate) { farthest_gate_ = farthest_gate; }
     void set_nearest_gate(uint8_t nearest_gate) { nearest_gate_ = nearest_gate; }
     void set_trigger_thresholds(const std::vector<uint8_t> &trigger_thresholds) { trigger_thresholds_ = trigger_thresholds; }
     void set_hold_thresholds(const std::vector<uint8_t> &hold_thresholds) { hold_thresholds_ = hold_thresholds; }
     void set_auto_threshold(uint8_t trigger_factor, uint8_t hold_factor, uint8_t scan_time) {
         trigger_factor_ = trigger_factor;
         hold_factor_ = hold_factor;
         scan_time_ = scan_time;
     }
 
  protected:
     friend class EnableConfigButton;
     friend class DisableConfigButton;
 
     bool enable_configuration_();
     bool disable_configuration_();
     void apply_cached_config_();
 
     void reset_input_buffer_();
     bool read_byte_(uint8_t *data, uint32_t timeout = 100);
     bool read_array_(std::vector<uint8_t> &data, size_t count);
     bool write_array_(const std::vector<uint8_t> &data);
     void read_data_();
     void handle_engineering_data_(const std::vector<uint8_t> &data);
     void handle_simple_data_(const std::vector<uint8_t> &data);
     bool send_command_(CommandWord cmd, const std::vector<uint8_t> &payload = {});
     bool wait_for_ack_(uint32_t timeout = 1000);
     bool read_ack_();
     bool validate_response_(const std::vector<uint8_t> &data);
     uint8_t calculate_checksum_(const std::vector<uint8_t> &data);
 
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
 
     sensor::Sensor *distance_sensor_{nullptr};
     binary_sensor::BinarySensor *presence_sensor_{nullptr};
     binary_sensor::BinarySensor *config_mode_sensor_{nullptr};
     EnableConfigButton *enable_config_button_{nullptr};
     DisableConfigButton *disable_config_button_{nullptr};
     sensor::Sensor *gate_energy_sensors_[MAX_GATES]{nullptr};
 
     uint32_t last_update_{0};
     uint32_t throttle_{50};
     bool output_mode_{true};
     uint8_t response_speed_{5};
     uint8_t unmanned_delay_{40};
     float status_report_frequency_{0.5f};
     float distance_report_frequency_{0.5f};
     uint8_t farthest_gate_{12};
     uint8_t nearest_gate_{0};
     std::vector<uint8_t> trigger_thresholds_;
     std::vector<uint8_t> hold_thresholds_;
     uint8_t trigger_factor_{0};
     uint8_t hold_factor_{0};
     uint8_t scan_time_{0};
 };
 
 class EnableConfigButton : public button::Button {
  public:
     explicit EnableConfigButton(HLKLD2410SComponent *parent) : parent_(parent) {}
     void press_action() override { parent_->enable_configuration_(); }
  protected:
     HLKLD2410SComponent *parent_;
 };
 
 class DisableConfigButton : public button::Button {
  public:
     explicit DisableConfigButton(HLKLD2410SComponent *parent) : parent_(parent) {}
     void press_action() override { parent_->disable_configuration_(); }
  protected:
     HLKLD2410SComponent *parent_;
 };
 
 }  // namespace hlk_ld2410s
 }  // namespace esphome