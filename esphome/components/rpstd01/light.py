"""The lamp, as a brightness light written through the link, whose fades run at a constant rate."""

import esphome.codegen as cg
from esphome.components import light
import esphome.config_validation as cv
from esphome.const import CONF_OUTPUT_ID
from esphome.types import ConfigType

from . import CONF_RPSTD01_ID, PARENT, rpstd01_ns

LampLight = rpstd01_ns.class_("LampLight", light.LightOutput)

CONFIG_SCHEMA = light.BRIGHTNESS_ONLY_LIGHT_SCHEMA.extend(
    {cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(LampLight), **PARENT}
)


async def to_code(config: ConfigType) -> None:
    parent = await cg.get_variable(config[CONF_RPSTD01_ID])
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID], parent)
    await light.register_light(var, config)
