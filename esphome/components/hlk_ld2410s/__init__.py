import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome import core, automation

from .hlk_ld2410s import HLK_LD2410S

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HLK_LD2410S),
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    var = core.Pvariable(config[CONF_ID], HLK_LD2410S())
    core.add(var)
    core.add(var.set_parent_id(config[CONF_ID]))