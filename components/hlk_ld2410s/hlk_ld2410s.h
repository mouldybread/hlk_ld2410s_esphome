/*
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * Created by github.com/mouldybread
 * Creation Date/Time: 2025-03-27 11:29:06 UTC
 */

 #pragma once

 #include "esphome.h"
 #include "esphome/core/component.h"
 #include "esphome/components/uart/uart.h"
 #include "esphome/components/sensor/sensor.h"
 #include "esphome/components/binary_sensor/binary_sensor.h"
 #include "esphome/components/button/button.h"
 #include "esphome/components/select/select.h"
 
 namespace esphome {
 namespace hlk_ld2410s {
 
 static const char *const TAG = "hlk_ld2410s";
 
 // Forward declaration of main component class
 class HLKLD2410SComponent;
 
 // Button class definitions
 class EnableConfigButton : public button::Button {
  public:
   explicit EnableConfigButton(HLKLD2410SComponent *parent) : parent_(parent) {}
   
  protected:
   void press() override;
   HLKLD2410SComponent *parent_;
 };
 
 class DisableConfigButton : public button::Button {
  public:
   explicit DisableConfigButton(HLKLD2410SComponent *parent) : parent_(parent) {}
   
  protected:
   void press() override;
   HLKLD2410SComponent *parent_;
 };
 
 // Protocol Constants
 static const uint8_t MINIMAL_FRAME_HEADER = 0x6E;
 static const uint8_t MINIMAL_FRAME_END = 0x62;
 static const uint8_t CONFIG_FRAME_HEADER[4] = {0xFD, 0xFC, 0xFB, 0xFA};
 static const uint8_t CONFIG_FRAME_END[4] = {0x04, 0x03, 0x02, 0x01};
 static const uint8_t STANDARD_FRAME_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
 static const uint8_t STANDARD_FRAME_END[4] = {0xF8, 0xF7, 0xF6, 0xF5};
 
 // Command Words
 enum class CommandWord : uint16_t {
   READ_FIRMWARE_VERSION = 0x0000,
   ENABLE_CONFIG = 0x00FF,
   END_CONFIG = 0x00FE,
   WRITE_SERIAL_NUMBER = 0x0010,
   READ_SERIAL_NUMBER = 0x0011,
   AUTO_UPDATE_THRESHOLD = 0x0009,
   WRITE_TRIGGER_THRESHOLD = 0x0072,
   READ_TRIGGER_THRESHOLD = 0x0073,
   WRITE_HOLD_THRESHOLD = 0x0076,
   READ_HOLD_THRESHOLD = 0x0077,
   WRITE_GENERAL_PARAMS = 0x0070,
   READ_GENERAL_PARAMS = 0x0071,
   SWITCH_OUTPUT_MODE = 0x007A
 };
 
 // Parameter Words for General Parameters
 enum class GeneralParamWord : uint16_t {
   DETECT_FARTHEST_GATE = 0x0005,
   DETECT_NEAREST_GATE = 0x000A,
   UNMANNED_DELAY = 0x0006,
   STATUS_REPORT_FREQ = 0x0002,
   DISTANCE_REPORT_FREQ = 0x000C,
   RESPONSE_SPEED = 0x000B
 };
 
 class HLKLD2410SComponent : public Component, public uart::UARTDevice {
  public:
   explicit HLKLD2410SComponent(uart::UARTComponent *parent) : UARTDevice(parent) {}
   
   void setup() override;
   void loop() override;
   void dump_config() override;
 
   void set_distance_sensor(sensor::Sensor *sensor) { distance_sensor_ = sensor; }
   void set_presence_sensor(binary_sensor::BinarySensor *sensor) { presence_sensor_ = sensor; }
   void set_config_mode_sensor(binary_sensor::BinarySensor *sensor) { config_mode_sensor_ = sensor; }
   void set_throttle(uint32_t throttle_ms) { throttle_ms_ = throttle_ms; }
   void set_enable_config_button(button::Button *button) { enable_config_button_ = button; }
   void set_disable_config_button(button::Button *button) { disable_config_button_ = button; }
   void set_response_speed_select(select::Select *select) { response_speed_select_ = select; }
   
   // Configuration commands - made public for button access
   void enable_configuration();
   void disable_configuration();
   void set_response_speed(uint8_t speed);
   bool read_firmware_version();
   bool switch_output_mode(bool standard_mode);
   bool write_general_parameters(uint16_t param_word, uint32_t value);
   bool read_general_parameters();
   bool write_trigger_threshold(const std::vector<uint32_t> &thresholds);
   bool read_trigger_threshold();
   bool write_hold_threshold(const std::vector<uint32_t> &thresholds);
   bool read_hold_threshold();
   bool auto_update_threshold(uint8_t trigger_factor, uint8_t hold_factor, uint8_t scan_time);
   bool write_serial_number(const std::string &serial);
   bool read_serial_number(std::string &serial);
 
  protected:
   // Sensors
   sensor::Sensor *distance_sensor_{nullptr};
   binary_sensor::BinarySensor *presence_sensor_{nullptr};
   binary_sensor::BinarySensor *config_mode_sensor_{nullptr};
   button::Button *enable_config_button_{nullptr};
   button::Button *disable_config_button_{nullptr};
   select::Select *response_speed_select_{nullptr};
   
   // State variables
   uint32_t throttle_ms_{0};
   uint32_t last_update_{0};
   bool in_config_mode_{false};
   bool standard_output_mode_{false};
 
   // Device information
   struct Version {
     uint16_t major;
     uint16_t minor;
     uint16_t patch;
   } firmware_version_;
 
   // Threshold storage
   std::vector<uint32_t> trigger_thresholds_;
   std::vector<uint32_t> hold_thresholds_;
   
   // Helper methods for protocol handling
   bool write_command_(CommandWord cmd, const uint8_t *data = nullptr, size_t len = 0);
   bool read_ack_(CommandWord expected_cmd);
   bool verify_frame_header_(const uint8_t *header, size_t len);
   bool verify_frame_end_(const uint8_t *end, size_t len);
   void process_minimal_frame_(uint8_t state, uint16_t distance);
   void process_standard_frame_(uint8_t state, uint16_t distance, const uint8_t *energy_values);
   void process_threshold_progress_(uint16_t progress);
   void process_minimal_data_();
   void process_standard_data_();
 };
 
 }  // namespace hlk_ld2410s
 }  // namespace esphome