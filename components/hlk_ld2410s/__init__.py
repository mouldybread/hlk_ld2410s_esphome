"""
HLK-LD2410S mmWave Radar Sensor Component for ESPHome.

Created by github.com/mouldybread
Creation Date/Time: 2025-03-27 12:03:26 UTC
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, binary_sensor, button, select, number
from esphome.const import (
    CONF_ID,
    CONF_THROTTLE,
    CONF_NAME,
    CONF_ICON,
    CONF_MIN_VALUE,
    CONF_MAX_VALUE,
    CONF_STEP,
    CONF_MODE,
    CONF_UNIT_OF_MEASUREMENT,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_MOTION,
    ICON_RADIATOR,  # Using as radar icon
    ICON_MOTION_SENSOR,
    ICON_RULER,
    ICON_TIMER,
    STATE_CLASS_MEASUREMENT,
    UNIT_METER,
)

CODEOWNERS = ["@mouldybread"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor", "button", "select", "number"]

hlk_ld2410s_ns = cg.esphome_ns.namespace("hlk_ld2410s")
HLKLD2410SComponent = hlk_ld2410s_ns.class_(
    "HLKLD2410SComponent", cg.Component, uart.UARTDevice
)
EnableConfigButton = hlk_ld2410s_ns.class_("EnableConfigButton", button.Button)
DisableConfigButton = hlk_ld2410s_ns.class_("DisableConfigButton", button.Button)
ResponseSpeedSelect = hlk_ld2410s_ns.class_("ResponseSpeedSelect", select.Select)
GateThresholdNumber = hlk_ld2410s_ns.class_("GateThresholdNumber", number.Number)

# Custom configuration keys
CONF_DISTANCE = "distance"
CONF_PRESENCE = "presence"
CONF_CONFIG_MODE = "config_mode"
CONF_ENABLE_CONFIG = "enable_config"
CONF_DISABLE_CONFIG = "disable_config"
CONF_RESPONSE_SPEED = "response_speed"
CONF_OUTPUT_MODE = "output_mode"
CONF_GATES = "gates"
CONF_GATE = "gate"
CONF_ENERGY = "energy"
CONF_UNMANNED_DELAY = "unmanned_delay"
CONF_STATUS_REPORT_FREQUENCY = "status_report_frequency"
CONF_DISTANCE_REPORT_FREQUENCY = "distance_report_frequency"

# Custom units and constants
UNIT_CENTIMETER = "cm"
UNIT_PERCENT = "%"
UNIT_HERTZ = "Hz"

ICON_ENERGY = "mdi:lightning-bolt"
ICON_RADAR = "mdi:radar"

GATE_COUNT = 16
MAX_THRESHOLD = 100
MAX_UNMANNED_DELAY = 120
MIN_UNMANNED_DELAY = 0
MAX_REPORT_FREQ = 8.0
MIN_REPORT_FREQ = 0.5

# Validation schemas
GATE_SCHEMA = cv.Schema({
    cv.Optional(CONF_ENERGY): sensor.sensor_schema(
        icon=ICON_ENERGY,
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
        unit_of_measurement=UNIT_PERCENT,
    ),
})

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(HLKLD2410SComponent),
            cv.Optional(CONF_THROTTLE, default="50ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_OUTPUT_MODE, default="Standard"): cv.enum(
                {"Standard": True, "Simple": False}, upper=False
            ),
            cv.Optional(CONF_RESPONSE_SPEED, default="Normal"): cv.enum(
                {"Normal": 5, "Fast": 10}, upper=False
            ),
            cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CENTIMETER,
                icon=ICON_RULER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_DISTANCE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_PRESENCE): binary_sensor.binary_sensor_schema(
                icon=ICON_MOTION_SENSOR,
                device_class=DEVICE_CLASS_MOTION,
            ),
            cv.Optional(CONF_CONFIG_MODE): binary_sensor.binary_sensor_schema(
                icon=ICON_RADAR,
            ),
            cv.Optional(CONF_ENABLE_CONFIG): button.button_schema(
                icon=ICON_RADAR,
            ),
            cv.Optional(CONF_DISABLE_CONFIG): button.button_schema(
                icon=ICON_RADAR,
            ),
            cv.Optional(CONF_UNMANNED_DELAY, default=0): cv.int_range(
                min=MIN_UNMANNED_DELAY, max=MAX_UNMANNED_DELAY
            ),
            cv.Optional(CONF_STATUS_REPORT_FREQUENCY, default=0.5): cv.float_range(
                min=MIN_REPORT_FREQ, max=MAX_REPORT_FREQ
            ),
            cv.Optional(CONF_DISTANCE_REPORT_FREQUENCY, default=0.5): cv.float_range(
                min=MIN_REPORT_FREQ, max=MAX_REPORT_FREQ
            ),
            cv.Optional(CONF_GATES): cv.Schema({
                cv.Range(min=0, max=GATE_COUNT-1): GATE_SCHEMA,
            }),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA),
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_THROTTLE in config:
        cg.add(var.set_throttle(config[CONF_THROTTLE]))

    if CONF_OUTPUT_MODE in config:
        cg.add(var.set_output_mode(config[CONF_OUTPUT_MODE]))

    if CONF_RESPONSE_SPEED in config:
        cg.add(var.set_response_speed(config[CONF_RESPONSE_SPEED]))

    if CONF_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))

    if CONF_PRESENCE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_PRESENCE])
        cg.add(var.set_presence_sensor(sens))

    if CONF_CONFIG_MODE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CONFIG_MODE])
        cg.add(var.set_config_mode_sensor(sens))

    if CONF_ENABLE_CONFIG in config:
        btn = await button.new_button(config[CONF_ENABLE_CONFIG])
        cg.add(var.set_enable_config_button(btn))

    if CONF_DISABLE_CONFIG in config:
        btn = await button.new_button(config[CONF_DISABLE_CONFIG])
        cg.add(var.set_disable_config_button(btn))

    if CONF_UNMANNED_DELAY in config:
        cg.add(var.set_unmanned_delay(config[CONF_UNMANNED_DELAY]))

    if CONF_STATUS_REPORT_FREQUENCY in config:
        cg.add(var.set_status_report_frequency(config[CONF_STATUS_REPORT_FREQUENCY]))

    if CONF_DISTANCE_REPORT_FREQUENCY in config:
        cg.add(var.set_distance_report_frequency(config[CONF_DISTANCE_REPORT_FREQUENCY]))

    if CONF_GATES in config:
        for gate_id, gate_config in config[CONF_GATES].items():
            if CONF_ENERGY in gate_config:
                sens = await sensor.new_sensor(gate_config[CONF_ENERGY])
                cg.add(var.set_gate_energy_sensor(gate_id, sens))