"""The night light, as an output written through the link."""

import esphome.codegen as cg
from esphome.components import output
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_TYPE
from esphome.types import ConfigType

from . import CONF_RPSTD01_ID, PARENT, rpstd01_ns

TYPE_NIGHT_LIGHT = "night_light"

NightLightOutput = rpstd01_ns.class_("NightLightOutput", output.FloatOutput)

CONFIG_SCHEMA = cv.typed_schema(
    {
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
