"""
HLK-LD2410S mmWave Radar Sensor Component for ESPHome.

Created by github.com/mouldybread
Creation Date/Time: 2025-03-27 12:01:40 UTC
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, binary_sensor, button, select, number
from esphome.const import (
    CONF_ID,
    CONF_THROTTLE,
    CONF_TRIGGER,
    CONF_NAME,
    CONF_ICON,
    CONF_TYPE,
    CONF_MIN_VALUE,
    CONF_MAX_VALUE,
    CONF_STEP,
    CONF_MODE,
    CONF_INITIAL_VALUE,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_MOTION,
    DEVICE_CLASS_ENERGY,
    ICON_RADAR,
    ICON_MOTION_SENSOR,
    ICON_RULER,
    ICON_ENERGY,
    ICON_TIMER,
    STATE_CLASS_MEASUREMENT,
    UNIT_METER,
    UNIT_PERCENT,
    UNIT_SECOND,
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

# Configuration keys
CONF_DISTANCE = "distance"
CONF_PRESENCE = "presence"
CONF_CONFIG_MODE = "config_mode"
CONF_ENABLE_CONFIG = "enable_config"
CONF_DISABLE_CONFIG = "disable_config"
CONF_RESPONSE_SPEED = "response_speed"
CONF_GATES = "gates"
CONF_GATE = "gate"
CONF_ENERGY = "energy"
CONF_TRIGGER_THRESHOLD = "trigger_threshold"
CONF_HOLD_THRESHOLD = "hold_threshold"
CONF_UNMANNED_DELAY = "unmanned_delay"
CONF_STATUS_REPORT_FREQ = "status_report_freq"
CONF_DISTANCE_REPORT_FREQ = "distance_report_freq"
CONF_AUTO_UPDATE = "auto_update"
CONF_TRIGGER_FACTOR = "trigger_factor"
CONF_HOLD_FACTOR = "hold_factor"
CONF_SCAN_TIME = "scan_time"

# Constants
GATE_COUNT = 16
MAX_THRESHOLD = 100
MAX_UNMANNED_DELAY = 120
MIN_UNMANNED_DELAY = 10
MAX_REPORT_FREQ = 8.0
MIN_REPORT_FREQ = 0.5

# Validation schemas
GATE_THRESHOLD_SCHEMA = cv.Schema({
    cv.Optional(CONF_NAME): cv.string,
    cv.Optional(CONF_ICON, default=ICON_ENERGY): cv.icon,
    cv.Optional(CONF_MIN_VALUE, default=0): cv.int_range(min=0, max=MAX_THRESHOLD),
    cv.Optional(CONF_MAX_VALUE, default=MAX_THRESHOLD): cv.int_range(min=0, max=MAX_THRESHOLD),
    cv.Optional(CONF_STEP, default=1): cv.positive_float,
    cv.Optional(CONF_INITIAL_VALUE, default=0): cv.int_range(min=0, max=MAX_THRESHOLD),
})

GATE_SCHEMA = cv.Schema({
    cv.Optional(CONF_ENERGY): sensor.sensor_schema(
        icon=ICON_ENERGY,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_MEASUREMENT,
        unit_of_measurement=UNIT_PERCENT,
    ),
    cv.Optional(CONF_TRIGGER_THRESHOLD): GATE_THRESHOLD_SCHEMA,
    cv.Optional(CONF_HOLD_THRESHOLD): GATE_THRESHOLD_SCHEMA,
})

AUTO_UPDATE_SCHEMA = cv.Schema({
    cv.Required(CONF_TRIGGER_FACTOR): cv.int_range(min=0, max=MAX_THRESHOLD),
    cv.Required(CONF_HOLD_FACTOR): cv.int_range(min=0, max=MAX_THRESHOLD),
    cv.Required(CONF_SCAN_TIME): cv.int_range(min=0, max=MAX_THRESHOLD),
})

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(HLKLD2410SComponent),
            cv.Optional(CONF_THROTTLE, default="50ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
                unit_of_measurement=UNIT_METER,
                icon=ICON_RULER,
                accuracy_decimals=2,
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
            cv.Optional(CONF_RESPONSE_SPEED): select.select_schema(
                icon=ICON_RADAR,
            ),
            cv.Optional(CONF_UNMANNED_DELAY): number.number_schema(
                icon=ICON_TIMER,
                min_value=MIN_UNMANNED_DELAY,
                max_value=MAX_UNMANNED_DELAY,
                step=1,
                unit_of_measurement=UNIT_SECOND,
            ),
            cv.Optional(CONF_STATUS_REPORT_FREQ): number.number_schema(
                icon=ICON_RADAR,
                min_value=MIN_REPORT_FREQ,
                max_value=MAX_REPORT_FREQ,
                step=0.1,
                unit_of_measurement="Hz",
            ),
            cv.Optional(CONF_DISTANCE_REPORT_FREQ): number.number_schema(
                icon=ICON_RADAR,
                min_value=MIN_REPORT_FREQ,
                max_value=MAX_REPORT_FREQ,
                step=0.1,
                unit_of_measurement="Hz",
            ),
            cv.Optional(CONF_GATES): cv.Schema({
                cv.Range(min=0, max=GATE_COUNT-1): GATE_SCHEMA,
            }),
            cv.Optional(CONF_AUTO_UPDATE): AUTO_UPDATE_SCHEMA,
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

    if CONF_RESPONSE_SPEED in config:
        sel = await select.new_select(config[CONF_RESPONSE_SPEED])
        cg.add(var.set_response_speed_select(sel))

    if CONF_UNMANNED_DELAY in config:
        num = await number.new_number(config[CONF_UNMANNED_DELAY])
        cg.add(var.set_unmanned_delay_number(num))

    if CONF_STATUS_REPORT_FREQ in config:
        num = await number.new_number(config[CONF_STATUS_REPORT_FREQ])
        cg.add(var.set_status_report_freq_number(num))

    if CONF_DISTANCE_REPORT_FREQ in config:
        num = await number.new_number(config[CONF_DISTANCE_REPORT_FREQ])
        cg.add(var.set_distance_report_freq_number(num))

    if CONF_GATES in config:
        for gate_id, gate_config in config[CONF_GATES].items():
            if CONF_ENERGY in gate_config:
                sens = await sensor.new_sensor(gate_config[CONF_ENERGY])
                cg.add(var.set_gate_energy_sensor(gate_id, sens))
            
            if CONF_TRIGGER_THRESHOLD in gate_config:
                num = await number.new_number(gate_config[CONF_TRIGGER_THRESHOLD])
                cg.add(var.set_gate_trigger_threshold_number(gate_id, num))
            
            if CONF_HOLD_THRESHOLD in gate_config:
                num = await number.new_number(gate_config[CONF_HOLD_THRESHOLD])
                cg.add(var.set_gate_hold_threshold_number(gate_id, num))

    if CONF_AUTO_UPDATE in config:
        auto_config = config[CONF_AUTO_UPDATE]
        cg.add(var.set_auto_update_config(
            auto_config[CONF_TRIGGER_FACTOR],
            auto_config[CONF_HOLD_FACTOR],
            auto_config[CONF_SCAN_TIME]
        ))