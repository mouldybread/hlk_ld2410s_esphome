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
  throttle: 500ms
  # Optional: Output mode (standard or minimal)
  output_mode: true  # true = standard mode, false = minimal mode
  # Optional: Distance sensor
  distance:
    name: "Object Distance"
    unit_of_measurement: "m"  # Outputs in meters
  # Optional: Presence detection
  presence:
    name: "Presence"
    device_class: occupancy
  # Optional: Configuration mode buttons
  enable_configuration:
    name: "Enable Configuration Mode"
  disable_configuration:
    name: "Disable Configuration Mode"
  # Optional: Energy gate sensors (only in standard mode)
  gate_0_energy:
    name: "Gate 0 Energy"
  gate_1_energy:
    name: "Gate 1 Energy"
  # Add more gates as needed (up to 15)
```

# HLK-LD2410S ESPHome Component

Support for the Hi-Link HLK-LD2410S microwave radar sensor in ESPHome. This sensor can detect human presence and measure the distance to detected objects.

## Features
- Distance measurement
- Presence detection
- Standard and minimal data output modes
- Configuration mode support
- Energy gate values (in standard mode)

## Usage

To use this component in your ESPHome configuration, add the following to your .yaml file:

## Configuration Variables

### Base Options:
- **uart_id** (Required, id): The ID of the UART bus component
- **throttle** (Optional, Time): Minimum time between sensor updates. Defaults to 50ms.
- **output_mode** (Optional, boolean): Set to true for standard mode with detailed data, false for minimal mode. Defaults to true.

### Distance Sensor Options:
- **distance** (Optional): Configure the distance measurement sensor
  - **name** (Required, string): The name of the distance sensor
  - **unit_of_measurement** (Optional, string): The unit of measurement. Defaults to "m" (meters)
  - All other options from Sensor

### Presence Sensor Options:
- **presence** (Optional): Configure the presence detection sensor
  - **name** (Required, string): The name of the presence sensor
  - **device_class** (Optional, string): Sensor device class. Defaults to "occupancy"
  - All other options from Binary Sensor

### Configuration Mode Options:
- **enable_configuration** (Optional): Button to enable configuration mode
  - **name** (Required, string): The name of the enable configuration button
  - All other options from Button
- **disable_configuration** (Optional): Button to disable configuration mode
  - **name** (Required, string): The name of the disable configuration button
  - All other options from Button

### Energy Gate Sensors (Standard Mode Only):
- **gate_X_energy** (Optional): Configure energy sensors for each gate (X = 0-15)
  - **name** (Required, string): The name of the gate energy sensor
  - All other options from Sensor

## Output Modes

### Minimal Mode
- Smaller data packets
- Basic presence detection and distance measurement
- Lower bandwidth usage

### Standard Mode
- Detailed data including energy values for each gate
- Motion and static energy values
- More accurate presence detection
- Requires more bandwidth

## Requirements:
- The sensor must be connected via UART
- Default UART settings: 115200 baud rate, 8 data bits, no parity, 1 stop bit
- Stable 3.3V power supply

## Common Issues:

### No Data from Sensor:
- Check UART wiring (TX/RX might need to be swapped)
- Verify UART configuration (baud rate, pins)
- Confirm power supply (sensor requires stable 3.3V)
- Try both output modes (minimal/standard)

### Unstable Readings:
- Increase the throttle time
- Check for interference sources
- Verify proper mounting height and angle
- Consider switching output modes

### Configuration Issues:
1. Enable configuration mode
2. Make changes
3. Disable configuration mode
4. If stuck, power cycle the sensor

## Version History
- 2025-03-27: Added standard mode support, improved data parsing
- Initial Release: Basic functionality with minimal mode

Created by: github.com/mouldybread
Last Updated: 2025-03-27 14:40:47 UTC
```
