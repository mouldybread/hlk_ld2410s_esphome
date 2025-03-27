#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace hlk_ld2410s {

static const char *const TAG = "hlk_ld2410s";  // Add TAG definition

class HLKLD2410SComponent : public Component, public uart::UARTDevice {
 public:
  explicit HLKLD2410SComponent(uart::UARTComponent *parent);
  
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_distance_sensor(sensor::Sensor *sensor) { distance_sensor_ = sensor; }
  void set_presence_sensor(sensor::Sensor *sensor) { presence_sensor_ = sensor; }
  void set_throttle(uint32_t throttle_ms) { throttle_ms_ = throttle_ms; }

 protected:
  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *presence_sensor_{nullptr};
  uint32_t throttle_ms_{0};  // Throttle in milliseconds
  uint32_t last_update_{0};  // Last update timestamp
};

}  // namespace hlk_ld2410s
}  // namespace esphome