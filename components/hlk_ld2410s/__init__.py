"""
HLK-LD2410S mmWave Radar Sensor Component for ESPHome.

Created by github.com/mouldybread
Creation Date/Time: 2025-03-27 13:32:02 UTC
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, binary_sensor, button
from esphome.const import (
    CONF_ID,
    CONF_THROTTLE,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_MOTION,
    DEVICE_CLASS_ENERGY,
    ICON_MOTION_SENSOR,
    ICON_RULER,
    ICON_RADIATOR,
    STATE_CLASS_MEASUREMENT,
    UNIT_METER,
    UNIT_DECIBEL,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor", "button"]

# Namespaces
hlk_ld2410s_ns = cg.esphome_ns.namespace("hlk_ld2410s")
HLKLD2410SComponent = hlk_ld2410s_ns.class_(
    "HLKLD2410SComponent", cg.Component, uart.UARTDevice
)
EnableConfigButton = hlk_ld2410s_ns.class_("EnableConfigButton", button.Button, cg.Component)
DisableConfigButton = hlk_ld2410s_ns.class_("DisableConfigButton", button.Button, cg.Component)

# Configuration constants
CONF_DISTANCE = "distance"
CONF_PRESENCE = "presence"
CONF_CONFIG_MODE = "config_mode"
CONF_GATE_ENERGY = "gate_energy"
CONF_GATE = "gate"
CONF_OUTPUT_MODE = "output_mode"
CONF_RESPONSE_SPEED = "response_speed"
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
CONF_ENABLE_CONFIG = "enable_config"
CONF_DISABLE_CONFIG = "disable_config"

# Constants
ICON_RADAR = "mdi:radar"
DEFAULT_THROTTLE_MS = 50
DEFAULT_RESPONSE_SPEED = 5
DEFAULT_UNMANNED_DELAY = 0
DEFAULT_REPORT_FREQ = 0.5
DEFAULT_FARTHEST_GATE = 12
DEFAULT_NEAREST_GATE = 0
MAX_GATES = 16

# Custom validation helpers
def validate_gate_number(value):
    """Validate gate number is within acceptable range."""
    value = cv.positive_int(value)
    if value >= MAX_GATES:
        raise cv.Invalid(f"Gate number must be less than {MAX_GATES}")
    return value

def validate_thresholds(value):
    """Validate threshold list."""
    values = cv.ensure_list(value)
    if len(values) > MAX_GATES:
        raise cv.Invalid(f"Cannot have more than {MAX_GATES} threshold values")
    return [cv.uint8_t(x) for x in values]

def validate_gate_order(config):
    """Validate nearest gate is not greater than farthest gate."""
    if CONF_NEAREST_GATE in config and CONF_FARTHEST_GATE in config:
        if config[CONF_NEAREST_GATE] > config[CONF_FARTHEST_GATE]:
            raise cv.Invalid(
                f"Nearest gate ({config[CONF_NEAREST_GATE]}) cannot be greater than "
                f"farthest gate ({config[CONF_FARTHEST_GATE]})"
            )
    return config

# Sensor schemas
DISTANCE_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_METER,
    icon=ICON_RULER,
    accuracy_decimals=2,
    device_class=DEVICE_CLASS_DISTANCE,
    state_class=STATE_CLASS_MEASUREMENT,
)

GATE_ENERGY_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_DECIBEL,
    icon=ICON_RADIATOR,
    accuracy_decimals=0,
    device_class=DEVICE_CLASS_ENERGY,
    state_class=STATE_CLASS_MEASUREMENT,
)

PRESENCE_SENSOR_SCHEMA = binary_sensor.binary_sensor_schema(
    device_class=DEVICE_CLASS_MOTION,
    icon=ICON_MOTION_SENSOR,
)

# Main configuration schema
CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(HLKLD2410SComponent),
            cv.Optional(CONF_DISTANCE): DISTANCE_SENSOR_SCHEMA,
            cv.Optional(CONF_PRESENCE): PRESENCE_SENSOR_SCHEMA,
            cv.Optional(CONF_CONFIG_MODE): binary_sensor.binary_sensor_schema(
                icon=ICON_RADAR,
            ),
            cv.Optional(CONF_GATE_ENERGY): cv.Schema(
                {
                    cv.Required(CONF_GATE): validate_gate_number,
                    cv.Required(CONF_ID): cv.declare_id(sensor.Sensor),
                }
            ),
            cv.Optional(CONF_THROTTLE, default=f"{DEFAULT_THROTTLE_MS}ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_OUTPUT_MODE, default=True): cv.boolean,
            cv.Optional(CONF_RESPONSE_SPEED, default=DEFAULT_RESPONSE_SPEED): cv.int_range(min=0, max=9),
            cv.Optional(CONF_UNMANNED_DELAY, default=DEFAULT_UNMANNED_DELAY): cv.uint16_t,
            cv.Optional(CONF_STATUS_REPORT_FREQ, default=DEFAULT_REPORT_FREQ): cv.float_range(min=0.1, max=10.0),
            cv.Optional(CONF_DISTANCE_REPORT_FREQ, default=DEFAULT_REPORT_FREQ): cv.float_range(min=0.1, max=10.0),
            cv.Optional(CONF_FARTHEST_GATE, default=DEFAULT_FARTHEST_GATE): validate_gate_number,
            cv.Optional(CONF_NEAREST_GATE, default=DEFAULT_NEAREST_GATE): validate_gate_number,
            cv.Optional(CONF_TRIGGER_THRESHOLDS): validate_thresholds,
            cv.Optional(CONF_HOLD_THRESHOLDS): validate_thresholds,
            cv.Optional(CONF_AUTO_THRESHOLD): cv.Schema(
                {
                    cv.Required(CONF_TRIGGER_FACTOR): cv.uint8_t,
                    cv.Required(CONF_HOLD_FACTOR): cv.uint8_t,
                    cv.Required(CONF_SCAN_TIME): cv.uint8_t,
                }
            ),
            cv.Optional(CONF_ENABLE_CONFIG): button.button_schema(icon=ICON_RADAR),
            cv.Optional(CONF_DISABLE_CONFIG): button.button_schema(icon=ICON_RADAR),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA),
    validate_gate_order,
)

async def to_code(config):
    """Generate code for HLK-LD2410S component."""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))

    if CONF_PRESENCE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_PRESENCE])
        cg.add(var.set_presence_sensor(sens))

    if CONF_CONFIG_MODE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CONFIG_MODE])
        cg.add(var.set_config_mode_sensor(sens))

    if CONF_GATE_ENERGY in config:
        gate_conf = config[CONF_GATE_ENERGY]
        sens = await sensor.new_sensor(gate_conf)
        cg.add(var.set_gate_energy_sensor(gate_conf[CONF_GATE], sens))

    if CONF_THROTTLE in config:
        cg.add(var.set_throttle(config[CONF_THROTTLE]))

    if CONF_OUTPUT_MODE in config:
        cg.add(var.set_output_mode(config[CONF_OUTPUT_MODE]))

    if CONF_RESPONSE_SPEED in config:
        cg.add(var.set_response_speed(config[CONF_RESPONSE_SPEED]))

    if CONF_UNMANNED_DELAY in config:
        cg.add(var.set_unmanned_delay(config[CONF_UNMANNED_DELAY]))

    if CONF_STATUS_REPORT_FREQ in config:
        cg.add(var.set_status_report_frequency(config[CONF_STATUS_REPORT_FREQ]))

    if CONF_DISTANCE_REPORT_FREQ in config:
        cg.add(var.set_distance_report_frequency(config[CONF_DISTANCE_REPORT_FREQ]))

    if CONF_FARTHEST_GATE in config:
        cg.add(var.set_farthest_gate(config[CONF_FARTHEST_GATE]))

    if CONF_NEAREST_GATE in config:
        cg.add(var.set_nearest_gate(config[CONF_NEAREST_GATE]))

    if CONF_TRIGGER_THRESHOLDS in config:
        cg.add(var.set_trigger_thresholds(config[CONF_TRIGGER_THRESHOLDS]))

    if CONF_HOLD_THRESHOLDS in config:
        cg.add(var.set_hold_thresholds(config[CONF_HOLD_THRESHOLDS]))

    if CONF_AUTO_THRESHOLD in config:
        threshold_conf = config[CONF_AUTO_THRESHOLD]
        cg.add(
            var.set_auto_threshold(
                threshold_conf[CONF_TRIGGER_FACTOR],
                threshold_conf[CONF_HOLD_FACTOR],
                threshold_conf[CONF_SCAN_TIME],
            )
        )

    if CONF_ENABLE_CONFIG in config:
        conf = config[CONF_ENABLE_CONFIG]
        sens = cg.new_Pvariable(conf[CONF_ID], var)
        await button.register_button(sens, conf)
        cg.add(var.set_enable_config_button(sens))

    if CONF_DISABLE_CONFIG in config:
        conf = config[CONF_DISABLE_CONFIG]
        sens = cg.new_Pvariable(conf[CONF_ID], var)
        await button.register_button(sens, conf)
        cg.add(var.set_disable_config_button(sens))