#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart_component.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace hlk_ld2410s {

class HLK_LD2410S : public Component, public uart::UARTDevice {
 public:
  HLK_LD2410S(uart::UARTComponent *parent) : uart::UARTDevice(parent) {}

  void set_distance_sensor(Sensor *sensor) { distance_sensor = sensor; }
  void set_presence_sensor(Sensor *sensor) { presence_sensor = sensor; }

  void setup() override;
  void loop() override;

 private:
  Sensor *distance_sensor = nullptr;
  Sensor *presence_sensor = nullptr;

  void parse_byte(uint8_t byte);
};

}  // namespace hlk_ld2410s
}  // namespace esphome