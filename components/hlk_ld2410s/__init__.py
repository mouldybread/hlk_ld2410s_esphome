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
from esphome.components import sensor, uart

DEPENDENCIES = ['uart']
AUTO_LOAD = ['sensor']

# Define our own constants
CONF_PRESENCE = "presence"
CONF_UART_ID = "uart_id"
CONF_THROTTLE = "throttle"

hlk_ld2410s_ns = cg.esphome_ns.namespace('hlk_ld2410s')
HLKLD2410SComponent = hlk_ld2410s_ns.class_('HLKLD2410SComponent', cg.Component, uart.UARTDevice)

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
    cv.Optional(CONF_PRESENCE): sensor.sensor_schema(
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
        icon=ICON_MOTION_SENSOR,
    ),
    cv.Optional(CONF_THROTTLE): cv.positive_time_period_milliseconds,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    uart_component = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], uart_component)
    await cg.register_component(var, config)
    
    if CONF_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))
    
    if CONF_PRESENCE in config:
        sens = await sensor.new_sensor(config[CONF_PRESENCE])
        cg.add(var.set_presence_sensor(sens))

    if CONF_THROTTLE in config:
        cg.add(var.set_throttle(config[CONF_THROTTLE]))