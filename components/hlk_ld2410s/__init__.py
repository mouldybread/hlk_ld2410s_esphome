"""
HLK-LD2410S mmWave Radar Sensor Component for ESPHome.

Created by github.com/mouldybread
Creation Date/Time: 2025-03-27 13:29:57 UTC
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, binary_sensor, button
from esphome.const import (
    CONF_ID,
    CONF_THROTTLE,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_MOTION,
    ICON_MOTION_SENSOR,
    ICON_RULER,
    STATE_CLASS_MEASUREMENT,
    UNIT_METER,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor", "button"]

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

# Gate energy sensor schema
GATE_ENERGY_SCHEMA = sensor.sensor_schema(
    unit_of_measurement="",
    icon=ICON_MOTION_SENSOR,
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
)

# Configuration validation schema
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(HLKLD2410SComponent),
            cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
                unit_of_measurement=UNIT_METER,
                icon=ICON_RULER,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_DISTANCE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_PRESENCE): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_MOTION,
                icon=ICON_MOTION_SENSOR,
            ),
            cv.Optional(CONF_CONFIG_MODE): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_GATE_ENERGY): cv.Schema(
                {
                    cv.Required(CONF_GATE): cv.int_range(min=0, max=15),
                    cv.Required(CONF_ID): cv.declare_id(sensor.Sensor),
                }
            ),
            cv.Optional(CONF_THROTTLE, default="50ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_OUTPUT_MODE, default=True): cv.boolean,
            cv.Optional(CONF_RESPONSE_SPEED, default=5): cv.int_range(min=0, max=9),
            cv.Optional(CONF_UNMANNED_DELAY, default=0): cv.uint16_t,
            cv.Optional(CONF_STATUS_REPORT_FREQ, default=0.5): cv.float_range(min=0.1, max=10.0),
            cv.Optional(CONF_DISTANCE_REPORT_FREQ, default=0.5): cv.float_range(min=0.1, max=10.0),
            cv.Optional(CONF_FARTHEST_GATE, default=12): cv.int_range(min=0, max=15),
            cv.Optional(CONF_NEAREST_GATE, default=0): cv.int_range(min=0, max=15),
            cv.Optional(CONF_TRIGGER_THRESHOLDS): cv.ensure_list(cv.uint8_t),
            cv.Optional(CONF_HOLD_THRESHOLDS): cv.ensure_list(cv.uint8_t),
            cv.Optional(CONF_AUTO_THRESHOLD): cv.Schema(
                {
                    cv.Required(CONF_TRIGGER_FACTOR): cv.uint8_t,
                    cv.Required(CONF_HOLD_FACTOR): cv.uint8_t,
                    cv.Required(CONF_SCAN_TIME): cv.uint8_t,
                }
            ),
            cv.Optional(CONF_ENABLE_CONFIG): button.button_schema(),
            cv.Optional(CONF_DISABLE_CONFIG): button.button_schema(),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
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