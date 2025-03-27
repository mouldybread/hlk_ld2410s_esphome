/*
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * Created by github.com/mouldybread
 * Creation Date/Time: 2025-03-27 07:56:51 UTC
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
 
 class HLKLD2410SComponent : public Component, public uart::UARTDevice {
  public:
   explicit HLKLD2410SComponent(uart::UARTComponent *parent);
   
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
   void set_response_speed(uint8_t speed);
 
   // Configuration mode methods
   void enable_configuration();
   void disable_configuration();
 
  protected:
   sensor::Sensor *distance_sensor_{nullptr};
   binary_sensor::BinarySensor *presence_sensor_{nullptr};
   binary_sensor::BinarySensor *config_mode_sensor_{nullptr};
   button::Button *enable_config_button_{nullptr};
   button::Button *disable_config_button_{nullptr};
   select::Select *response_speed_select_{nullptr};
   uint32_t throttle_ms_{0};  // Throttle in milliseconds
   uint32_t last_update_{0};  // Last update timestamp
   bool in_config_mode_{false};
 
   // Protocol constants
   static const uint16_t CMD_ENABLE_CONFIG = 0x00FF;
   static const uint16_t CMD_DISABLE_CONFIG = 0x00FE;
   static const uint16_t CMD_SET_RESPONSE_SPEED = 0xFF01;
 
   // Helper methods for sending commands
   bool write_command_(uint16_t command, const uint8_t *data = nullptr, size_t len = 0);
   bool read_ack_(uint16_t expected_command);
 };
 
 class EnableConfigButton : public button::Button {
  public:
   explicit EnableConfigButton(HLKLD2410SComponent *parent) : parent_(parent) {}
 
  protected:
   void press_action() override {
     parent_->enable_configuration();
   }
 
   HLKLD2410SComponent *parent_;
 };
 
 class DisableConfigButton : public button::Button {
  public:
   explicit DisableConfigButton(HLKLD2410SComponent *parent) : parent_(parent) {}
 
  protected:
   void press_action() override {
     parent_->disable_configuration();
   }
 
   HLKLD2410SComponent *parent_;
 };
 
 class ResponseSpeedSelect : public select::Select, public Component {
  public:
   explicit ResponseSpeedSelect(HLKLD2410SComponent *parent) : parent_(parent) {}
 
  protected:
   void control(const std::string &value) override {
     uint8_t speed = std::stoi(value);
     parent_->set_response_speed(speed);
     this->publish_state(value);
   }
 
   HLKLD2410SComponent *parent_;
 };
 
 }  // namespace hlk_ld2410s
 }  // namespace esphome