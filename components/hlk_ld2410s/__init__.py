"""
HLK-LD2410S mmWave Radar Sensor Component for ESPHome.

SPDX-License-Identifier: GPL-3.0-only

Created by github.com/mouldybread
Creation Date/Time: 2025-03-27 06:05:29 UTC
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
)
from esphome.components import sensor, uart, binary_sensor, button

DEPENDENCIES = ['uart']
AUTO_LOAD = ['sensor', 'binary_sensor', 'button']

CONF_PRESENCE = "presence"
CONF_UART_ID = "uart_id"
CONF_THROTTLE = "throttle"
CONF_ENABLE_CONFIG = "enable_configuration"
CONF_DISABLE_CONFIG = "disable_configuration"
CONF_CONFIG_MODE = "config_mode"

# Generate namespaces
hlk_ld2410s_ns = cg.esphome_ns.namespace('hlk_ld2410s')
HLKLD2410SComponent = hlk_ld2410s_ns.class_('HLKLD2410SComponent', cg.Component, uart.UARTDevice)
EnableConfigButton = hlk_ld2410s_ns.class_('EnableConfigButton', button.Button)
DisableConfigButton = hlk_ld2410s_ns.class_('DisableConfigButton', button.Button)

# Configuration schema for the component
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HLKLD2410SComponent),
    cv.GenerateID(CONF_UART_ID): cv.use_id(uart.UARTComponent),
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
    cv.Optional(CONF_ENABLE_CONFIG): button.button_schema(EnableConfigButton),
    cv.Optional(CONF_DISABLE_CONFIG): button.button_schema(DisableConfigButton),
    cv.Optional(CONF_THROTTLE): cv.positive_time_period_milliseconds,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    """Generate code for HLK-LD2410S component."""
    var = cg.new_Pvariable(config[CONF_ID], await cg.get_variable(config[CONF_UART_ID]))
    await cg.register_component(var, config)
    
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
        sens = cg.new_Pvariable(config[CONF_ENABLE_CONFIG][CONF_ID], var)
        await button.register_button(sens, config[CONF_ENABLE_CONFIG])
        cg.add(var.set_enable_config_button(sens))

    if CONF_DISABLE_CONFIG in config:
        sens = cg.new_Pvariable(config[CONF_DISABLE_CONFIG][CONF_ID], var)
        await button.register_button(sens, config[CONF_DISABLE_CONFIG])
        cg.add(var.set_disable_config_button(sens))

    if CONF_THROTTLE in config:
        cg.add(var.set_throttle(config[CONF_THROTTLE]))