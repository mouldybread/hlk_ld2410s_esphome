import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID, UNIT_CENTIMETER
from esphome.components import sensor
from . import HLKLD2410SComponent, hlk_ld2410s_ns

CONF_HLK_LD2410S_ID = 'hlk_ld2410s_id'
CONF_DISTANCE = 'distance'
CONF_PRESENCE = 'presence'

HLKLD2410SSensor = hlk_ld2410s_ns.class_('HLKLD2410SSensor', sensor.Sensor)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_HLK_LD2410S_ID): cv.use_id(HLKLD2410SComponent),
    cv.Optional(CONF_DISTANCE): sensor.sensor_schema(UNIT_CENTIMETER),
    cv.Optional(CONF_PRESENCE): sensor.sensor_schema(),
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    parent = yield cg.get_variable(config[CONF_HLK_LD2410S_ID])
    if CONF_DISTANCE in config:
        distance_sensor = yield sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(parent.set_distance_sensor(distance_sensor))
    if CONF_PRESENCE in config:
        presence_sensor = yield sensor.new_sensor(config[CONF_PRESENCE])
        cg.add(parent.set_presence_sensor(presence_sensor))