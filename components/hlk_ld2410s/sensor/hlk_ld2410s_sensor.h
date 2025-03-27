#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace hlk_ld2410s {

class HLKLD2410SSensor : public sensor::Sensor, public Component {
 public:
  HLKLD2410SSensor();
  void setup() override;
  void loop() override;
  void update() override;

  void set_distance_sensor(sensor::Sensor *sensor) { distance_sensor_ = sensor; }
  void set_presence_sensor(sensor::Sensor *sensor) { presence_sensor_ = sensor; }

 protected:
  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *presence_sensor_{nullptr};

  void parse_data_(const uint8_t *data, size_t length);
};

}  // namespace hlk_ld2410s
}  // namespace esphome