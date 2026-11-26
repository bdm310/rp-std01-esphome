"""RP-STD01 dimmer: the link to our MCU firmware."""

from pathlib import Path
import re

import esphome.codegen as cg
from esphome.components import binary_sensor, light, number, select, text_sensor, uart
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_LIGHT, CONF_OPTIONS
import esphome.final_validate as fv
from esphome.types import ConfigType

from ..rpstd01_link import link_uart_schema

AUTO_LOAD = ["rpstd01_link"]
DEPENDENCIES = ["uart", "light", "select", "number", "binary_sensor", "text_sensor"]

CONF_THUMBWHEEL_RAW_A = "thumbwheel_raw_a"
CONF_THUMBWHEEL_RAW_B = "thumbwheel_raw_b"
CONF_THUMBWHEEL_USE = "thumbwheel_use"
CONF_BRIGHTNESS_BAR = "brightness_bar"
CONF_NIGHT_LIGHT_MODE = "night_light_mode"
CONF_DIM_END = "dim_end"
CONF_BRIGHT_END = "bright_end"
CONF_FADE_LIMIT = "fade_limit"
CONF_ADDON_THRESHOLDS = "addon_thresholds"
CONF_UP_DRIVE_OFF = "up_drive_off"
CONF_DOWN_DRIVE_OFF = "down_drive_off"
CONF_UP_DRIVE_ON = "up_drive_on"
CONF_DOWN_DRIVE_ON = "down_drive_on"
CONF_INPUTS = "inputs"
CONF_TEXT_SENSORS = "text_sensors"

rpstd01_ns = cg.global_ns.namespace("rpstd01")
LinkComponent = rpstd01_ns.class_("LinkComponent", cg.Component, uart.UARTDevice)
Input = rpstd01_ns.enum("Input")

THRESHOLD_KEYS = (CONF_UP_DRIVE_OFF, CONF_DOWN_DRIVE_OFF, CONF_UP_DRIVE_ON, CONF_DOWN_DRIVE_ON)
NUMBER_KEYS = (CONF_DIM_END, CONF_BRIGHT_END, CONF_FADE_LIMIT)
# Each select, and the option of it that the C++ code acts on.
SELECT_OPTIONS = {
    CONF_THUMBWHEEL_USE: "Dim End",
    CONF_BRIGHTNESS_BAR: "Level",
    CONF_NIGHT_LIGHT_MODE: "On When Lamp Off",
}
INPUTS = {
    "onoff": Input.IN_ONOFF,
    "up": Input.IN_UP,
    "down": Input.IN_DOWN,
    "addon_up": Input.IN_ADDON_UP,
    "addon_down": Input.IN_ADDON_DOWN,
}
TEXT_SENSOR_KEYS = ("state", "faults", "firmware", "firmware_expected", "reset_cause")
ADC_MAX = int(re.search(r"\bADC_MAX = (\d+);", (Path(__file__).parent / "rpstd01.h").read_text()).group(1))
WHEEL_RAW = cv.int_range(min=0, max=ADC_MAX)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LinkComponent),
            cv.Required(CONF_THUMBWHEEL_RAW_A): WHEEL_RAW,
            cv.Required(CONF_THUMBWHEEL_RAW_B): WHEEL_RAW,
            **{cv.Required(key): cv.use_id(select.Select) for key in SELECT_OPTIONS},
            cv.Required(CONF_LIGHT): cv.use_id(light.LightState),
            **{cv.Required(key): cv.use_id(number.Number) for key in NUMBER_KEYS},
            cv.Required(CONF_ADDON_THRESHOLDS): cv.Schema(
                {cv.Required(key): cv.use_id(number.Number) for key in THRESHOLD_KEYS}
            ),
            cv.Required(CONF_INPUTS): cv.Schema(
                {cv.Required(key): cv.use_id(binary_sensor.BinarySensor) for key in INPUTS}
            ),
            cv.Required(CONF_TEXT_SENSORS): cv.Schema(
                {cv.Required(key): cv.use_id(text_sensor.TextSensor) for key in TEXT_SENSOR_KEYS}
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


def _selects_have_options(config: ConfigType) -> ConfigType:
    full = fv.full_config.get()
    for key, option in SELECT_OPTIONS.items():
        select_config = full.get_config_for_path(full.get_path_for_id(config[key])[:-1])
        if option not in select_config.get(CONF_OPTIONS, []):
            raise cv.Invalid(f"select '{config[key]}' needs the option '{option}'", path=[key])
    return config


FINAL_VALIDATE_SCHEMA = cv.All(link_uart_schema("rpstd01"), _selects_have_options)


async def to_code(config: ConfigType) -> None:
    thresholds = config[CONF_ADDON_THRESHOLDS]
    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_THUMBWHEEL_RAW_A],
        config[CONF_THUMBWHEEL_RAW_B],
        *[await cg.get_variable(config[key]) for key in NUMBER_KEYS],
        *[await cg.get_variable(thresholds[key]) for key in THRESHOLD_KEYS],
    )
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    for key, option in SELECT_OPTIONS.items():
        cg.add(getattr(var, f"set_{key}")(await cg.get_variable(config[key]), option))
    cg.add(var.set_light(await cg.get_variable(config[CONF_LIGHT])))
    for key, input_ in INPUTS.items():
        cg.add(var.set_input(input_, await cg.get_variable(config[CONF_INPUTS][key])))
    texts = config[CONF_TEXT_SENSORS]
    cg.add(var.set_text_sensors(*[await cg.get_variable(texts[key]) for key in TEXT_SENSOR_KEYS]))
