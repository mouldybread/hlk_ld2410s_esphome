"""
HLK-LD2410S mmWave Radar Sensor Component for ESPHome.

SPDX-License-Identifier: GPL-3.0-only

Created by github.com/mouldybread
Creation Date/Time: 2025-03-27 11:24:24 UTC
"""

import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import (
    CONF_ID,
    CONF_DISTANCE,
    UNIT_CENTIMETER,
    DEVICE_CLASS_DISTANCE,
    STATE_CLASS_MEASUREMENT,
    ICON_RULER,
    ICON_MOTION_SENSOR,
    CONF_NAME,
    CONF_TYPE,
)
from esphome.components import sensor, uart, binary_sensor, button, select, number
from esphome.core import CORE

DEPENDENCIES = ['uart']
AUTO_LOAD = ['sensor', 'binary_sensor', 'button', 'select', 'number']

CONF_PRESENCE = "presence"
CONF_UART_ID = "uart_id"
CONF_THROTTLE = "throttle"
CONF_ENABLE_CONFIG = "enable_configuration"
CONF_DISABLE_CONFIG = "disable_configuration"
CONF_CONFIG_MODE = "config_mode"
CONF_RESPONSE_SPEED = "response_speed"
CONF_OUTPUT_MODE = "output_mode"
CONF_UNMANNED_DELAY = "unmanned_delay"
CONF_STATUS_REPORT_FREQ = "status_report_frequency"
CONF_DISTANCE_REPORT_FREQ = "distance_report_frequency"
CONF_FARTHEST_GATE = "farthest_gate"
CONF_NEAREST_GATE = "nearest_gate"
CONF_TRIGGER_THRESHOLDS = "trigger_thresholds"
CONF_HOLD_THRESHOLDS = "hold_thresholds"
CONF_AUTO_THRESHOLD = "auto_threshold"
CONF_TRIGGER_FACTOR = "trigger_factor"
CONF_HOLD_FACTOR = "hold_factor"
CONF_SCAN_TIME = "scan_time"

# Generate namespaces
hlk_ld2410s_ns = cg.esphome_ns.namespace('hlk_ld2410s')
HLKLD2410SComponent = hlk_ld2410s_ns.class_('HLKLD2410SComponent', cg.Component, uart.UARTDevice)
EnableConfigButton = hlk_ld2410s_ns.class_('EnableConfigButton', button.Button)
DisableConfigButton = hlk_ld2410s_ns.class_('DisableConfigButton', button.Button)
ResponseSpeedSelect = hlk_ld2410s_ns.class_('ResponseSpeedSelect', select.Select, cg.Component)

# Validators
RESPONSE_SPEED_OPTIONS = {
    "normal": "Normal",
    "fast": "Fast"
}

OUTPUT_MODE_OPTIONS = {
    "minimal": "Minimal",
    "standard": "Standard"
}

THRESHOLD_SCHEMA = cv.Schema({
    cv.Required(CONF_NAME): cv.string,
    cv.Optional(CONF_TYPE, default="number"): cv.one_of("number", "select", lower=True),
    cv.Optional("min_value", default=0): cv.int_range(min=0),
    cv.Optional("max_value", default=100): cv.int_range(min=0),
    cv.Optional("step", default=1): cv.positive_float,
})

AUTO_THRESHOLD_SCHEMA = cv.Schema({
    cv.Optional(CONF_TRIGGER_FACTOR, default=2): cv.int_range(min=1, max=10),
    cv.Optional(CONF_HOLD_FACTOR, default=1): cv.int_range(min=1, max=10),
    cv.Optional(CONF_SCAN_TIME, default=120): cv.int_range(min=30, max=600),
})

# Configuration schema for the component
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HLKLD2410SComponent),
    cv.GenerateID(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    
    # Basic sensors
    cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
        unit_of_measurement=UNIT_CENTIMETER,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_DISTANCE,
        state_class=STATE_CLASS_MEASUREMENT,
        icon=ICON_RULER,
    ),
    cv.Optional(CONF_PRESENCE): binary_sensor.binary_sensor_schema(
        device_class="occupancy",
        icon=ICON_MOTION_SENSOR,
    ),
    cv.Optional(CONF_CONFIG_MODE): binary_sensor.binary_sensor_schema(
        icon="mdi:cog",
    ),
    
    # Configuration buttons
    cv.Optional(CONF_ENABLE_CONFIG): button.button_schema(EnableConfigButton),
    cv.Optional(CONF_DISABLE_CONFIG): button.button_schema(DisableConfigButton),
    
    # Basic settings
    cv.Optional(CONF_THROTTLE): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_OUTPUT_MODE, default="minimal"): cv.enum(OUTPUT_MODE_OPTIONS, lower=True),
    cv.Optional(CONF_RESPONSE_SPEED, default="normal"): cv.enum(RESPONSE_SPEED_OPTIONS, lower=True),
    
    # General parameters
    cv.Optional(CONF_UNMANNED_DELAY, default=40): cv.int_range(min=10, max=120),
    cv.Optional(CONF_STATUS_REPORT_FREQ, default=0.5): cv.float_range(min=0.5, max=8.0),
    cv.Optional(CONF_DISTANCE_REPORT_FREQ, default=0.5): cv.float_range(min=0.5, max=8.0),
    cv.Optional(CONF_FARTHEST_GATE, default=12): cv.int_range(min=1, max=16),
    cv.Optional(CONF_NEAREST_GATE, default=0): cv.int_range(min=0, max=16),
    
    # Threshold configuration
    cv.Optional(CONF_TRIGGER_THRESHOLDS): cv.All(
        cv.ensure_list(cv.int_range(min=0, max=100)),
        cv.Length(max=16),
    ),
    cv.Optional(CONF_HOLD_THRESHOLDS): cv.All(
        cv.ensure_list(cv.int_range(min=0, max=100)),
        cv.Length(max=16),
    ),
    
    # Auto threshold configuration
    cv.Optional(CONF_AUTO_THRESHOLD): AUTO_THRESHOLD_SCHEMA,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    """Generate code for HLK-LD2410S component."""
    var = cg.new_Pvariable(config[CONF_ID], await cg.get_variable(config[CONF_UART_ID]))
    await cg.register_component(var, config)
    
    # Configure basic sensors
    if CONF_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))
    
    if CONF_PRESENCE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_PRESENCE])
        cg.add(var.set_presence_sensor(sens))
    
    if CONF_CONFIG_MODE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CONFIG_MODE])
        cg.add(var.set_config_mode_sensor(sens))
    
    # Configure buttons
    if CONF_ENABLE_CONFIG in config:
        sens = cg.new_Pvariable(config[CONF_ENABLE_CONFIG][CONF_ID], var)
        await button.register_button(sens, config[CONF_ENABLE_CONFIG])
        cg.add(var.set_enable_config_button(sens))
    
    if CONF_DISABLE_CONFIG in config:
        sens = cg.new_Pvariable(config[CONF_DISABLE_CONFIG][CONF_ID], var)
        await button.register_button(sens, config[CONF_DISABLE_CONFIG])
        cg.add(var.set_disable_config_button(sens))
    
    # Configure basic settings
    if CONF_THROTTLE in config:
        cg.add(var.set_throttle(config[CONF_THROTTLE]))
    
    # Set output mode
    if config[CONF_OUTPUT_MODE] == "standard":
        cg.add(var.switch_output_mode(True))
    
    # Set response speed
    speed_value = 5 if config[CONF_RESPONSE_SPEED] == "normal" else 10
    cg.add(var.set_response_speed(speed_value))
    
    # Configure general parameters
    cg.add(var.write_general_parameters(0x05, config[CONF_FARTHEST_GATE]))
    cg.add(var.write_general_parameters(0x0A, config[CONF_NEAREST_GATE]))
    cg.add(var.write_general_parameters(0x06, config[CONF_UNMANNED_DELAY]))
    
    # Convert frequencies to internal representation (multiply by 10)
    status_freq = int(config[CONF_STATUS_REPORT_FREQ] * 10)
    distance_freq = int(config[CONF_DISTANCE_REPORT_FREQ] * 10)
    cg.add(var.write_general_parameters(0x02, status_freq))
    cg.add(var.write_general_parameters(0x0C, distance_freq))
    
    # Configure thresholds
    if CONF_TRIGGER_THRESHOLDS in config:
        thresholds = config[CONF_TRIGGER_THRESHOLDS]
        # Pad with zeros if less than 16 values provided
        while len(thresholds) < 16:
            thresholds.append(0)
        cg.add(var.write_trigger_threshold(thresholds))
    
    if CONF_HOLD_THRESHOLDS in config:
        thresholds = config[CONF_HOLD_THRESHOLDS]
        # Pad with zeros if less than 16 values provided
        while len(thresholds) < 16:
            thresholds.append(0)
        cg.add(var.write_hold_threshold(thresholds))
    
    # Configure auto threshold if specified
    if CONF_AUTO_THRESHOLD in config:
        auto_config = config[CONF_AUTO_THRESHOLD]
        cg.add(var.auto_update_threshold(
            auto_config[CONF_TRIGGER_FACTOR],
            auto_config[CONF_HOLD_FACTOR],
            auto_config[CONF_SCAN_TIME]
        ))