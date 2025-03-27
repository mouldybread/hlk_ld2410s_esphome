#pragma once

#include "esphome.h"

namespace esphome {
namespace hlk_ld2410s {

class HLK_LD2410S : public Component, public UARTDevice {
 public:
  HLK_LD2410S(UARTComponent *parent) : UARTDevice(parent) {}

  Sensor *distance_sensor = new Sensor();
  Sensor *presence_sensor = new Sensor();

  void setup() override;
  void loop() override;

 private:
  void parse_byte(uint8_t byte);
};

}  // namespace hlk_ld2410s
}  // namespace esphome
