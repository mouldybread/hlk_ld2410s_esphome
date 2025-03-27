"""HLK-LD2410S mmWave Radar Sensor integration for ESPHome.

Created by github.com/mouldybread
Creation Date/Time: 2025-03-27 15:01:49 UTC
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, binary_sensor, button
from esphome.const import (
    CONF_ID,
    CONF_THROTTLE,
    CONF_DISTANCE,
    CONF_NAME,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_OCCUPANCY,
    DEVICE_CLASS_RUNNING,
    STATE_CLASS_MEASUREMENT,
    UNIT_METER,
    ICON_MOTION_SENSOR,
)

DEPENDENCIES = ['uart']
AUTO_LOAD = ['sensor', 'binary_sensor', 'button']

hlk_ld2410s_ns = cg.esphome_ns.namespace('hlk_ld2410s')
HLKLD2410SComponent = hlk_ld2410s_ns.class_(
    'HLKLD2410SComponent', cg.Component, uart.UARTDevice
)
EnableConfigButton = hlk_ld2410s_ns.class_('EnableConfigButton', button.Button)
DisableConfigButton = hlk_ld2410s_ns.class_('DisableConfigButton', button.Button)

# Configuration Constants
CONF_UART_ID = 'uart_id'
CONF_PRESENCE = 'presence'
CONF_CONFIG_MODE = 'config_mode'
CONF_ENABLE_CONFIGURATION = 'enable_configuration'
CONF_DISABLE_CONFIGURATION = 'disable_configuration'
CONF_OUTPUT_MODE = 'output_mode'
CONF_RESPONSE_SPEED = 'response_speed'
CONF_UNMANNED_DELAY = 'unmanned_delay'
CONF_STATUS_REPORT_FREQUENCY = 'status_report_frequency'
CONF_DISTANCE_REPORT_FREQUENCY = 'distance_report_frequency'
CONF_FARTHEST_GATE = 'farthest_gate'
CONF_NEAREST_GATE = 'nearest_gate'
CONF_TRIGGER_THRESHOLDS = 'trigger_thresholds'
CONF_HOLD_THRESHOLDS = 'hold_thresholds'
CONF_AUTO_THRESHOLD = 'auto_threshold'
CONF_TRIGGER_FACTOR = 'trigger_factor'
CONF_HOLD_FACTOR = 'hold_factor'
CONF_SCAN_TIME = 'scan_time'
CONF_GATE_ENERGY = 'gate_{}_energy'

# Validation schemas
AUTO_THRESHOLD_SCHEMA = cv.Schema({
    cv.Required(CONF_TRIGGER_FACTOR): cv.int_range(min=1, max=5),
    cv.Required(CONF_HOLD_FACTOR): cv.int_range(min=1, max=5),
    cv.Required(CONF_SCAN_TIME): cv.int_range(min=10, max=250),
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HLKLD2410SComponent),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_THROTTLE, default='50ms'): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_OUTPUT_MODE, default=True): cv.boolean,
    cv.Optional(CONF_RESPONSE_SPEED, default=5): cv.one_of(5, 10),
    cv.Optional(CONF_UNMANNED_DELAY, default=40): cv.int_range(min=10, max=120),
    cv.Optional(CONF_STATUS_REPORT_FREQUENCY, default=0.5): cv.float_range(min=0.5, max=8.0),
    cv.Optional(CONF_DISTANCE_REPORT_FREQUENCY, default=0.5): cv.float_range(min=0.5, max=8.0),
    cv.Optional(CONF_FARTHEST_GATE, default=12): cv.int_range(min=1, max=16),
    cv.Optional(CONF_NEAREST_GATE, default=0): cv.int_range(min=0, max=15),
    cv.Optional(CONF_TRIGGER_THRESHOLDS): cv.All(
        cv.ensure_list(cv.int_range(min=0, max=100)),
        cv.Length(exact=16),
    ),
    cv.Optional(CONF_HOLD_THRESHOLDS): cv.All(
        cv.ensure_list(cv.int_range(min=0, max=100)),
        cv.Length(exact=16),
    ),
    cv.Optional(CONF_AUTO_THRESHOLD): AUTO_THRESHOLD_SCHEMA,
    cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
        unit_of_measurement=UNIT_METER,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_DISTANCE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_PRESENCE): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_OCCUPANCY,
    ),
    cv.Optional(CONF_CONFIG_MODE): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_RUNNING,
    ),
    cv.Optional(CONF_ENABLE_CONFIGURATION): button.button_schema(),
    cv.Optional(CONF_DISABLE_CONFIGURATION): button.button_schema(),
}).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

# Add gate energy sensor schemas
for i in range(16):
    CONFIG_SCHEMA = CONFIG_SCHEMA.extend({
        cv.Optional(CONF_GATE_ENERGY.format(i)): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            icon=ICON_MOTION_SENSOR,
        ),
    })

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

    if CONF_UNMANNED_DELAY in config:
        cg.add(var.set_unmanned_delay(config[CONF_UNMANNED_DELAY]))

    if CONF_STATUS_REPORT_FREQUENCY in config:
        cg.add(var.set_status_report_frequency(config[CONF_STATUS_REPORT_FREQUENCY]))

    if CONF_DISTANCE_REPORT_FREQUENCY in config:
        cg.add(var.set_distance_report_frequency(config[CONF_DISTANCE_REPORT_FREQUENCY]))

    if CONF_FARTHEST_GATE in config:
        cg.add(var.set_farthest_gate(config[CONF_FARTHEST_GATE]))

    if CONF_NEAREST_GATE in config:
        cg.add(var.set_nearest_gate(config[CONF_NEAREST_GATE]))

    if CONF_TRIGGER_THRESHOLDS in config:
        cg.add(var.set_trigger_thresholds(config[CONF_TRIGGER_THRESHOLDS]))

    if CONF_HOLD_THRESHOLDS in config:
        cg.add(var.set_hold_thresholds(config[CONF_HOLD_THRESHOLDS]))

    if CONF_AUTO_THRESHOLD in config:
        at = config[CONF_AUTO_THRESHOLD]
        cg.add(var.set_auto_threshold(
            at[CONF_TRIGGER_FACTOR],
            at[CONF_HOLD_FACTOR],
            at[CONF_SCAN_TIME]
        ))

    if CONF_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))

    if CONF_PRESENCE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_PRESENCE])
        cg.add(var.set_presence_sensor(sens))

    if CONF_CONFIG_MODE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CONFIG_MODE])
        cg.add(var.set_config_mode_sensor(sens))

    if CONF_ENABLE_CONFIGURATION in config:
        conf = config[CONF_ENABLE_CONFIGURATION]
        sens = cg.new_Pvariable(conf[CONF_ID], var)
        await button.register_button(sens, conf)
        cg.add(var.set_enable_config_button(sens))

    if CONF_DISABLE_CONFIGURATION in config:
        conf = config[CONF_DISABLE_CONFIGURATION]
        sens = cg.new_Pvariable(conf[CONF_ID], var)
        await button.register_button(sens, conf)
        cg.add(var.set_disable_config_button(sens))

    # Register gate energy sensors
    for i in range(16):
        if gate_conf := config.get(CONF_GATE_ENERGY.format(i)):
            sens = await sensor.new_sensor(gate_conf)
            cg.add(var.set_gate_energy_sensor(i, sens))

hlk_ld2410s_ns.end_namespace()