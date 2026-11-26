"""The lamp and the night light, as outputs written through the link."""

import esphome.codegen as cg
from esphome.components import output
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_TYPE
from esphome.types import ConfigType

from . import LinkComponent, rpstd01_ns

CONF_RPSTD01_ID = "rpstd01_id"
TYPE_LAMP = "lamp"
TYPE_NIGHT_LIGHT = "night_light"

LampOutput = rpstd01_ns.class_("LampOutput", output.FloatOutput)
NightLightOutput = rpstd01_ns.class_("NightLightOutput", output.FloatOutput)

PARENT = {cv.GenerateID(CONF_RPSTD01_ID): cv.use_id(LinkComponent)}

CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_LAMP: output.FLOAT_OUTPUT_SCHEMA.extend(
            {cv.Required(CONF_ID): cv.declare_id(LampOutput), **PARENT}
        ),
        TYPE_NIGHT_LIGHT: output.FLOAT_OUTPUT_SCHEMA.extend(
            {cv.Required(CONF_ID): cv.declare_id(NightLightOutput), **PARENT}
        ),
    },
    key=CONF_TYPE,
)


async def to_code(config: ConfigType) -> None:
    parent = await cg.get_variable(config[CONF_RPSTD01_ID])
    var = cg.new_Pvariable(config[CONF_ID], parent)
    await output.register_output(var, config)
