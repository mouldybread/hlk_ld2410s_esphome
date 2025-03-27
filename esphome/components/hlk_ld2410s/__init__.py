import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import uart
from esphome import automation
from esphome import core as cg

HLK_LD2410SComponent = cg.global_ns.class_("HLK_LD2410S", cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HLK_LD2410SComponent),
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_parent_id(config[CONF_ID]))
    cg.add_component(var, config)