import esphome.config_validation as cv
import esphome.codegen as cg
# Removed CONF_PRESENCE from this import since we define it ourselves
from esphome.const import CONF_ID, CONF_DISTANCE, UNIT_CENTIMETER
from esphome.components import sensor, uart

DEPENDENCIES = ['uart']
AUTO_LOAD = ['sensor']

# Define our own constants
CONF_PRESENCE = "presence"  # Our own constant

hlk_ld2410s_ns = cg.esphome_ns.namespace('hlk_ld2410s')
HLKLD2410SComponent = hlk_ld2410s_ns.class_('HLKLD2410SSensor', cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HLKLD2410SComponent),
    cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
        unit_of_measurement=UNIT_CENTIMETER,
        accuracy_decimals=0,
    ),
    cv.Optional(CONF_PRESENCE): sensor.sensor_schema(
        accuracy_decimals=0,
    ),
}).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    
    if CONF_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))
    
    if CONF_PRESENCE in config:
        sens = await sensor.new_sensor(config[CONF_PRESENCE])
        cg.add(var.set_presence_sensor(sens))