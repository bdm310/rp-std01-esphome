"""RP-STD01 MCU tool: installs and updates our MCU firmware."""

import esphome.codegen as cg
from esphome.components import uart
from esphome.components.esp32 import VARIANT_ESP32C3, only_on_variant
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_NUMBER,
    CONF_RX_PIN,
    CONF_TX_PIN,
    CONF_UART_ID,
    PLATFORM_ESP32,
    PLATFORM_HOST,
    Framework,
)
from esphome.core import CORE
import esphome.final_validate as fv
from esphome.types import ConfigType

from ..rpstd01_link import link_uart_schema

AUTO_LOAD = ["rpstd01_link"]
DEPENDENCIES = ["uart"]

rpstd01tool_ns = cg.global_ns.namespace("rpstd01tool")
ToolComponent = rpstd01tool_ns.class_("ToolComponent", cg.Component, uart.UARTDevice)

# The power-on step drives UART0 through the ESP32-C3's IO MUX, before ESPHome's UART driver
# exists: its pins are the ones UART0 has there.
MCU_PINS = {CONF_RX_PIN: 20, CONF_TX_PIN: 21}


def _unit_chip(config: ConfigType) -> ConfigType:
    """The power-on step drives the ESP32-C3's UART0 and RTC watchdog directly."""
    if CORE.is_esp32:
        only_on_variant(supported=VARIANT_ESP32C3, msg_prefix="rpstd01_tool")(config)
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema({cv.GenerateID(): cv.declare_id(ToolComponent)})
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA),
    cv.only_on([PLATFORM_ESP32, PLATFORM_HOST]),
    cv.only_with_framework([Framework.ESP_IDF, Framework.NATIVE]),
    _unit_chip,
)


def _uart_on_mcu_pins(config: ConfigType) -> ConfigType:
    if not CORE.is_esp32:
        return config
    full = fv.full_config.get()
    uart_config = full.get_config_for_path(full.get_path_for_id(config[CONF_UART_ID])[:-1])
    for key, number in MCU_PINS.items():
        if key not in uart_config or uart_config[key][CONF_NUMBER] != number:
            raise cv.Invalid(f"the UART of rpstd01_tool needs {key}: GPIO{number}, as the power-on step uses")
    return config


FINAL_VALIDATE_SCHEMA = cv.All(_uart_on_mcu_pins, link_uart_schema("rpstd01_tool"))


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    for key, number in MCU_PINS.items():
        cg.add_define(f"RPSTD01TOOL_{key.upper()}", number)
