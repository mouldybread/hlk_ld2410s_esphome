import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID, CONF_NAME, UNIT_CENTIMETER
from esphome.components import uart, sensor
from esphome import automation

hlk_ld2410s_ns = cg.esphome_ns.namespace('hlk_ld2410s')
HLK_LD2410S = hlk_ld2410s_ns.class_('HLK_LD2410S', cg.Component, uart.UARTDevice)

CONF_DISTANCE_SENSOR = 'distance_sensor'
CONF_PRESENCE_SENSOR = 'presence_sensor'
CONF_HLK_LD2410S_ID = 'hlk_ld2410s_id'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HLK_LD2410S),
    cv.Required(CONF_DISTANCE_SENSOR): cv.use_id(sensor.Sensor),
    cv.Required(CONF_PRESENCE_SENSOR): cv.use_id(sensor.Sensor),
}).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)

SENSOR_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HLK_LD2410S),
    cv.Required(CONF_HLK_LD2410S_ID): cv.use_id(HLK_LD2410S),
    cv.Required(CONF_DISTANCE_SENSOR): sensor.sensor_schema(UNIT_CENTIMETER).extend({
        cv.Required(CONF_NAME): cv.string,
    }),
    cv.Required(CONF_PRESENCE_SENSOR): sensor.sensor_schema().extend({
        cv.Required(CONF_NAME): cv.string,
    }),
})

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)

    uart_component = yield cg.get_variable(config[uart.CONF_UART_ID])
    cg.add(var.set_parent_id(uart_component))

    distance_sensor = yield cg.get_variable(config[CONF_DISTANCE_SENSOR])
    cg.add(var.set_distance_sensor(distance_sensor))

    presence_sensor = yield cg.get_variable(config[CONF_PRESENCE_SENSOR])
    cg.add(var.set_presence_sensor(presence_sensor))

def sensor_to_code(config):
    parent = yield cg.get_variable(config[CONF_HLK_LD2410S_ID])
    distance_sensor = yield sensor.new_sensor(config[CONF_DISTANCE_SENSOR])
    cg.add(parent.set_distance_sensor(distance_sensor))
    presence_sensor = yield sensor.new_sensor(config[CONF_PRESENCE_SENSOR])
    cg.add(parent.set_presence_sensor(presence_sensor))

BUILD_FLAGS = '-DUSE_HLK_LD2410S'

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield uart.register_uart_device(var, config)

    if CONF_DISTANCE_SENSOR in config:
        sens = yield sensor.new_sensor(config[CONF_DISTANCE_SENSOR])
        cg.add(var.set_distance_sensor(sens))

    if CONF_PRESENCE_SENSOR in config:
        sens = yield sensor.new_sensor(config[CONF_PRESENCE_SENSOR])
        cg.add(var.set_presence_sensor(sens))