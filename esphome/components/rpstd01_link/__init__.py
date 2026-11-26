"""Headers shared by the RP-STD01 main configuration and MCU tool: the link and the firmware constants."""

from pathlib import Path
import re

from esphome.components import uart

FW_CONSTANTS = (Path(__file__).parent / "rpstd01_fw_constants.h").read_text()
LINK_BAUD = int(re.search(r"\bLINK_BAUD = (\d+)u;", FW_CONSTANTS).group(1))


def link_uart_schema(name: str):
    return uart.final_validate_device_schema(
        name, baud_rate=LINK_BAUD, require_rx=True, require_tx=True, data_bits=8, parity="NONE", stop_bits=1
    )
