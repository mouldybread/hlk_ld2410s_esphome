# HLK LD2410S ESPHome Component
## TESTING/EXPERIMENTAL - MAY BE BROKEN
This repository contains the ESPHome component for the HLK LD2410S module.

## Usage

To use this component in your ESPHome configuration, add the following to your `.yaml` file:

```
external_components:
  - source: github://mouldybread/hlk_ld2410s_esphome

esphome:
  name: hlk_ld2410s
  platform: ESP8266
  board: d1_mini

logger:
  level: DEBUG

uart:
  id: uart_bus
  tx_pin: D4
  rx_pin: D3
  baud_rate: 115200

sensor:
  - platform: custom
    lambda: |-
      auto hlk_ld2410s = new HLK_LD2410S(id(uart_bus));
      App.register_component(hlk_ld2410s);
      return {hlk_ld2410s->distance_sensor, hlk_ld2410s->presence_sensor};
    sensors:
      - name: "HLK LD2410S Distance"
        unit_of_measurement: "cm"
      - name: "HLK LD2410S Presence"

custom_component:
  - id: hlk_ld2410s
    lambda: |-
      auto hlk_ld2410s = new HLK_LD2410S(id(uart_bus));
      App.register_component(hlk_ld2410s);
      return {hlk_ld2410s};
```
