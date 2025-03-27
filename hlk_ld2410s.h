#pragma once

#include "esphome.h"

class HLK_LD2410S : public Component, public UARTDevice {
 public:
  HLK_LD2410S(UARTComponent *parent) : UARTDevice(parent) {}

  Sensor *distance_sensor = new Sensor();
  Sensor *presence_sensor = new Sensor();

  void setup() override {
    // Setup code here
  }

  void loop() override {
    while (available() > 0) {
      uint8_t byte;
      read_byte(&byte);
      parse_byte(byte);
    }
  }

 private:
  void parse_byte(uint8_t byte);
};