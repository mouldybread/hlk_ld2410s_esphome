from esphome.components.sensor import Sensor
from esphome.const import CONF_ID
import esphome.config_validation as cv
from esphome import automation

from .hlk_ld2410s import HLK_LD2410S

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HLK_LD2410S),
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    hlk_ld2410s = cg.new_Pvariable(config[CONF_ID])
    cg.add(hlk_ld2410s.set_parent_id(config[CONF_ID]))
    cg.add_component(hlk_ld2410s, config)