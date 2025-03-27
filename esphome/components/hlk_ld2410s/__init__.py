from esphome import automation
from esphome.const import CONF_ID
import esphome.config_validation as cv
from esphome.core import CORE
from esphome.components import sensor
from esphome.components.hlk_ld2410s import hlk_ld2410s

HLK_LD2410SComponent = hlk_ld2410s.HLK_LD2410S

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HLK_LD2410SComponent),
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_parent_id(config[CONF_ID]))
    cg.add_component(var, config)