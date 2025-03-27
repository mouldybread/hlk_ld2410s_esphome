"""
HLK-LD2410S mmWave Radar Sensor Component for ESPHome.

SPDX-License-Identifier: GPL-3.0-only

Created by github.com/mouldybread
Creation Date/Time: 2025-03-27 07:48:51 UTC
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
from esphome.components import sensor, uart, binary_sensor, button, select

DEPENDENCIES = ['uart']
AUTO_LOAD = ['sensor', 'binary_sensor', 'button', 'select']

# Configuration options
CONF_PRESENCE = "presence"
CONF_UART_ID = "uart_id"
CONF_THROTTLE = "throttle"
CONF_ENABLE_CONFIG = "enable_configuration"
CONF_DISABLE_CONFIG = "disable_configuration"
CONF_CONFIG_MODE = "config_mode"
CONF_RESPONSE_SPEED = "response_speed"
CONF_RESPONSE_SPEED_SELECT = "response_speed_select"
CONF_READ_FIRMWARE = "read_firmware"

RESPONSE_SPEED_OPTIONS = [str(x) for x in range(10)]  # 0-9

# Generate namespaces
hlk_ld2410s_ns = cg.esphome_ns.namespace('hlk_ld2410s')
HLKLD2410SComponent = hlk_ld2410s_ns.class_('HLKLD2410SComponent', cg.Component, uart.UARTDevice)
EnableConfigButton = hlk_ld2410s_ns.class_('EnableConfigButton', button.Button)
DisableConfigButton = hlk_ld2410s_ns.class_('DisableConfigButton', button.Button)
ResponseSpeedSelect = hlk_ld2410s_ns.class_('ResponseSpeedSelect', select.Select, cg.Component)
ReadFirmwareButton = hlk_ld2410s_ns.class_('ReadFirmwareButton', button.Button)

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
    cv.Optional(CONF_RESPONSE_SPEED): cv.int_range(min=0, max=9),
    cv.Optional(CONF_RESPONSE_SPEED_SELECT): select.SELECT_SCHEMA.extend({
        cv.GenerateID(): cv.declare_id(ResponseSpeedSelect),
    }).extend(cv.COMPONENT_SCHEMA),
    cv.Optional(CONF_READ_FIRMWARE): button.button_schema(ReadFirmwareButton),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    """Generate code for HLK-LD2410S component."""
    var = cg.new_Pvariable(config[CONF_ID], await cg.get_variable(config[CONF_UART_ID]))
    await cg.register_component(var, config)
    