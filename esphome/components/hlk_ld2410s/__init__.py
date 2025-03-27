import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome import automation
import esphome.codegen as cg

hlk_ld2410s_ns = cg.esphome_ns.namespace('hlk_ld2410s')
HLK_LD2410S = hlk_ld2410s_ns.class_('HLK_LD2410S', cg.Component, cg.UARTDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HLK_LD2410S),
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_parent_id(config[CONF_ID]))
    cg.add_component(var, config)