# HLK LD2410S ESPHome Component
## Functional with caveats
This repository contains an ESPHome component for the HLK LD2410S module.

The code herein is AI generated. The existing ESPHome components I found didn't work so I gave the protocol specification to AI prompted it to write the code and troubleshoot. I really have no idea what I'm doing so I can't speak to the quality of this implementation but i've managed to make this work. It's here incase it's useful to someone else but I can't promise I'll be able to help with any issues should they arise.

I've implemented quite extensive feature support but it has not been thoroughly tested. Presence and distance sensors are functional. This module doesn't have bluetooth and so can only be configured via serial. I've left the configuration out of the UI for simplicities sake, honestly the compilation is so fast it almost doesn't matter. 

I'm using this with a Wemos D1 Mini ESP8266 board.

## Usage

To use this component in your ESPHome configuration, add the following to your `.yaml` file:

```yaml
# Basic configuration for D1 Mini with HLK-LD2410S radar sensor
# Created: 2025-03-27 17:02:17 UTC
# Author: mouldybread

esphome:
  name: radar-d1mini
  friendly_name: Radar Sensor

esp8266:
  board: d1_mini

# Enable logging
logger:
  level: DEBUG
  logs:
    hlk_ld2410s: DEBUG

# Enable Home Assistant API
api:

ota:
  password: !secret ota_password

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

  # Enable fallback hotspot (captive portal) in case wifi connection fails
  ap:
    ssid: "Radar-D1Mini Fallback"
    password: !secret fallback_password

# D1 Mini UART Configuration
# GPIO1 (TX) -> HLK-LD2410S RX
# GPIO3 (RX) -> HLK-LD2410S TX
uart:
  id: uart_hlk_ld2410s
  tx_pin: GPIO1
  rx_pin: GPIO3
  baud_rate: 115200
  stop_bits: 1
  parity: NONE

# HLK-LD2410S Configuration
hlk_ld2410s:
  id: radar_sensor
  uart_id: uart_hlk_ld2410s
  throttle: 2s  # Good default value
  output_mode: false  # Simple mode is recommended
  response_speed: 5  # Valid options are 5 or 10
  distance:
    name: "Object Distance"
    unit_of_measurement: m
    device_class: distance
    state_class: measurement
    accuracy_decimals: 2
  presence:
    name: "Presence"
    device_class: occupancy
  config_mode:
    name: "Configuration Mode"
    device_class: running
  unmanned_delay: 40  # Valid range: 10-120
  status_report_frequency: 0.5  # Valid range: 0.5-8.0
  distance_report_frequency: 0.5  # Valid range: 0.5-8.0
  farthest_gate: 12  # Valid range: 1-16
  nearest_gate: 0  # Valid range: 0-15

  # Optional threshold configuration
  trigger_thresholds:  # 16 values, all within 0-100 range
    - 50
    - 46
    - 34
    - 32
    - 32
    - 32
    - 32
    - 32
    - 50
    - 46
    - 34
    - 32
    - 32
    - 32
    - 32
    - 32

  hold_thresholds:  # 16 values, all within 0-100 range
    - 15
    - 15
    - 15
    - 15
    - 15
    - 15
    - 15
    - 15
    - 9
    - 9
    - 9
    - 9
    - 9
    - 9
    - 9
    - 9

  # Optional auto threshold configuration
  auto_threshold:
    trigger_factor: 2  # Valid range: 1-5
    hold_factor: 1  # Valid range: 1-5
    scan_time: 120  # Valid range: 10-250

  # Configuration mode control buttons
  enable_configuration:
    name: "Enable Configuration Mode"
  disable_configuration:
    name: "Disable Configuration Mode"

# Optional status LED
status_led:
  pin: GPIO2  # D1 Mini onboard LED
```

## Features
- Distance measurement (in meters)
- Presence detection with state indication (0x02 = presence)
- Simple and engineering mode data output
- Configuration mode support with parameter adjustments
- Energy gate values (in engineering mode)
- Configurable sensor parameters
- Automatic configuration application on startup

## Configuration Variables

### Base Options:
- **uart_id** (Required, id): The ID of the UART bus component
- **throttle** (Optional, Time): Minimum time between sensor updates. Default: 2s
- **output_mode** (Optional, boolean): Set to true for engineering mode with detailed data, false for simple mode. Default: false
- **response_speed** (Optional, int): Response speed setting (5 or 10). Default: 5
- **unmanned_delay** (Optional, int): Time before reporting no presence (10-120s). Default: 40

### Sensor Parameters:
- **status_report_frequency** (Optional, float): Status report frequency (0.5-8.0Hz). Default: 0.5
- **distance_report_frequency** (Optional, float): Distance report frequency (0.5-8.0Hz). Default: 0.5
- **farthest_gate** (Optional, int): Farthest detection gate (1-16). Default: 12
- **nearest_gate** (Optional, int): Nearest detection gate (0-15). Default: 0

### Threshold Settings:
- **trigger_thresholds** (Optional, list): 16 values for motion trigger thresholds (0-100)
- **hold_thresholds** (Optional, list): 16 values for motion hold thresholds (0-100)
- **auto_threshold** (Optional):
  - **trigger_factor** (int): Trigger sensitivity factor (1-5)
  - **hold_factor** (int): Hold sensitivity factor (1-5)
  - **scan_time** (int): Scan time in seconds (10-250)

### Sensor Options:
- **distance** (Optional): Distance measurement sensor configuration
- **presence** (Optional): Presence detection sensor configuration
- **config_mode** (Optional): Configuration mode status sensor
- **gate_energy_sensors** (Optional, engineering mode only): Energy values for detection gates

## Output Modes

### Simple Mode (Recommended)
- 5-byte data packets: `[0x6E, state, distance, 0x00, 0x62]`
- Basic presence detection (state = 0x02 indicates presence)
- Distance in centimeters (single byte)
- Lower bandwidth usage
- More stable operation

### Engineering Mode
- 75-byte detailed data packets
- Energy values for each detection gate
- Motion and static target information
- Higher bandwidth requirement

## Requirements:
- UART connection (115200 baud, 8N1)
- 3.3V power supply
- ESPHome 2024.1.0 or newer

## Common Issues:

### No Data or Unknown Values:
- Verify UART connections (TX→RX, RX→TX)
- Check power supply stability
- Enable debug logging to verify data reception
- Try simple mode first (output_mode: false)

### Unstable Readings:
- Increase throttle time (2s recommended)
- Use simple mode for more stable operation
- Verify proper mounting location
- Check for interference sources

### Configuration Issues:
1. Start in configuration mode
2. Apply settings
3. Disable configuration mode
4. If problems persist, power cycle

## Debug Logging
Add logger configuration with DEBUG level to help troubleshoot issues with data reception and parsing.

## Version History
- 2025-03-27 17:00:44: Improved simple mode parsing, added debug logging
- 2025-03-27 14:40:47: Added engineering mode support
- Initial Release: Basic functionality with simple mode

Created by: mouldybread
Last Updated: 2025-03-27 17:00:44 UTC
