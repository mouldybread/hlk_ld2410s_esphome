# HLK LD2410S ESPHome Component
## TESTING/EXPERIMENTAL - MAY BE BROKEN
This repository contains the ESPHome component for the HLK LD2410S module.

## Usage

To use this component in your ESPHome configuration, add the following to your `.yaml` file:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/mouldybread/hlk_ld2410s_esphome
      ref: main
      components: [hlk_ld2410s]

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

hlk_ld2410s:
  id: my_ld2410s

sensor:
  - platform: hlk_ld2410s
    id: my_ld2410s
    distance:
      name: "HLK LD2410S Distance"
      unit_of_measurement: "cm"
    presence:
      name: "HLK LD2410S Presence"