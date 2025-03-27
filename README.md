# HLK LD2410S ESPHome Component
## TESTING/EXPERIMENTAL - MAY BE BROKEN
This repository contains the ESPHome component for the HLK LD2410S module.

The code herein is AI generated. The existing ESPHome components I found didn't work, so I gave the protocol specification to AI and guided it to write the code. I really have no idea what I'm doing, but i've managed to make this work. It's here incase it's useful to someone else, but I can't promise I'll be able to help with any issues should they arise.

## Usage

To use this component in your ESPHome configuration, add the following to your `.yaml` file:

```yaml
# Example configuration entry
uart:
  tx_pin: GPIO1
  rx_pin: GPIO3
  baud_rate: 115200

hlk_ld2410s:
  uart_id: uart_bus
  # Optional: Throttle sensor updates (minimum time between updates)
  throttle: 3s
  # Optional: Distance sensor
  distance:
    name: "Object Distance"
    unit_of_measurement: "cm"
  # Optional: Presence detection
  presence:
    name: "Presence"
    device_class: occupancy
  # Optional: Configuration mode buttons
  enable_configuration:
    name: "Enable Configuration Mode"
  disable_configuration:
    name: "Disable Configuration Mode"
```

The `hlk_ld2410s` component provides support for the Hi-Link HLK-LD2410S microwave radar sensor. This sensor can detect human presence and measure the distance to detected objects.

## Configuration variables:

### Base Options:
- **uart_id** (*Required*, id): The ID of the UART bus component
- **throttle** (*Optional*, Time): Minimum time between sensor updates. Defaults to no throttling.

### Distance Sensor Options:
- **distance** (*Optional*): Configure the distance measurement sensor
  - **name** (*Required*, string): The name of the distance sensor
  - All other options from [Sensor](https://esphome.io/components/sensor/index.html#config-sensor)

### Presence Sensor Options:
- **presence** (*Optional*): Configure the presence detection sensor
  - **name** (*Required*, string): The name of the presence sensor
  - All other options from [Binary Sensor](https://esphome.io/components/binary_sensor/index.html#config-binary-sensor)

### Configuration Mode Options:
- **enable_configuration** (*Optional*): Button to enable configuration mode
  - **name** (*Required*, string): The name of the enable configuration button
  - All other options from [Button](https://esphome.io/components/button/index.html#config-button)
- **disable_configuration** (*Optional*): Button to disable configuration mode
  - **name** (*Required*, string): The name of the disable configuration button
  - All other options from [Button](https://esphome.io/components/button/index.html#config-button)

## Example Sensor Data in Home Assistant:

- **Distance Sensor**: Shows the distance to detected objects in centimeters
- **Presence Sensor**: Shows whether a person is detected (ON) or not (OFF)
- **Configuration Buttons**: Allow enabling/disabling configuration mode for sensor settings

## Requirements:

- The sensor must be connected via UART
- Default UART settings: 115200 baud rate, 8 data bits, no parity, 1 stop bit

## Common Issues:

1. If you see no data from the sensor, check:
   - UART wiring (TX/RX might need to be swapped)
   - UART configuration (baud rate, pins)
   - Power supply (sensor requires stable 3.3V)

2. If distance readings are unstable:
   - Try increasing the throttle time
   - Check for interference sources
   - Ensure proper mounting height and angle
```
