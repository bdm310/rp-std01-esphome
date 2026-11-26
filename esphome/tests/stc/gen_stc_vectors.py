#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later AND MIT
"""Record what stcgal sends while programming an image, for test_stc_isp.cpp.

stcgal's Stc15Protocol runs with its packet layer replaced: it is fed recorded bootloader
responses and every packet it writes is saved. The responses come from stcgal's own trace of a
real STC15W4K56S4 (tests/stc15w4k56s4.yml), with the status packet changed to look like our
IAP15W413AS (its model, bootloader 7.2.5T, factory option bits as read from the chip).

Output, one record per line:  S <hex>  status packet;  R <hex>  a later response;
W <hex>  a payload stcgal wrote.

The packets depend only on stcgal and the image, so they are recorded once for a fixed test image
(test_image.bin) and committed (vectors.txt). Record them again with make stc-vectors, which
clones stcgal at a pinned commit.

Usage: gen_stc_vectors.py <stcgal source dir> <image.bin> <out.txt>

The packets below are stcgal's, under its MIT notice in LICENSES/stcgal-MIT.txt.
"""

from __future__ import annotations

from pathlib import Path
import re
import struct
import sys
from unittest import mock

HERE = Path(__file__).resolve().parent
COMPONENTS = HERE.parent.parent / "components"


def header_value(header: str, name: str) -> int:
    return int(re.search(rf"\b{name} = (0x[0-9A-Fa-f]+|\d+)u?;", (COMPONENTS / header).read_text()).group(1), 0)


SYSCLK_HZ = header_value("rpstd01_link/rpstd01_fw_constants.h", "SYSCLK_HZ")
MCU_MODEL = header_value("rpstd01_tool/rpstd01_tool.h", "MCU_MODEL")

STATUS_W4K56S4 = [
    0x50, 0x8D, 0xFF, 0x73, 0x96, 0xF5, 0x7B, 0x9F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x27, 0xED, 0x00,
    0x00, 0x73, 0x54, 0x00, 0xF5, 0x28, 0x04, 0x06, 0x70, 0x96, 0x02, 0x15, 0x19, 0x1C, 0x1E, 0x23,
    0x00, 0xEC, 0xE0, 0x04, 0xD7, 0xF8, 0x73, 0xBF, 0xFF, 0xFF, 0x15, 0x09, 0x25, 0x60,
]
CALIB_1 = [
    0x00, 0x0B, 0x0D, 0x21, 0x12, 0xBC, 0x18, 0x3E, 0x1A, 0x05, 0x24, 0xFA, 0x2F, 0xB3, 0x34, 0xD1,
    0x4A, 0x52, 0x5E, 0xC0, 0x52, 0xDB, 0x73, 0x1A, 0x00, 0x00,
]
CALIB_2 = [
    0x00, 0x0C, 0x23, 0xBF, 0x23, 0xD3, 0x23, 0xE7, 0x23, 0xF6, 0x24, 0x0F, 0x24, 0x23, 0x47, 0x73,
    0x47, 0xB9, 0x47, 0xE1, 0x48, 0x09, 0x48, 0x36, 0x48, 0x59,
]
ERASE = [0x03, 0xF5, 0x28, 0x00, 0xA5, 0x03, 0x27, 0x49]


def our_status() -> bytes:
    s = bytearray(STATUS_W4K56S4)
    s[7] = 0xFB        # internal clock, long power-on delay, reset pin off, P2.0 high
    s[8:12] = struct.pack(">I", SYSCLK_HZ)
    s[17] = 0x72       # bootloader 7.2
    s[18] = ord("T")
    s[20:22] = struct.pack(">H", MCU_MODEL)
    s[22] = (s[22] & 0xF0) | 0x05
    return bytes(s)


def main() -> int:
    stcgal_src, image_path, out_path = (Path(a) for a in sys.argv[1:4])
    sys.path.insert(0, str(HERE / "stubs"))
    sys.path.insert(0, str(stcgal_src))
    from stcgal import protocols

    image = image_path.read_bytes()
    blocks = (len(image) + 63) // 64
    responses = [our_status(), bytes(CALIB_1), bytes(CALIB_2), b"\x01", b"\x05", bytes(ERASE)]
    responses += [b"\x02\x54"] * blocks + [b"\x07\x54", b"\x04\x54"]
    pending = list(responses)
    writes: list[bytes] = []

    class Recorder(protocols.Stc15Protocol):
        def read_packet(self) -> bytes:
            return pending.pop(0)

        def write_packet(self, packet_data: bytes, epilogue_len: int = 0) -> None:
            writes.append(bytes(packet_data))

    # stcgal's frontend passes the trim in kHz times 1000.
    proto = Recorder("", 2400, 115200, SYSCLK_HZ // 1000 * 1000)
    proto.ser = mock.MagicMock()
    proto.ser.inWaiting.return_value = 1
    proto.progress_cb = lambda *args: None
    with mock.patch("stcgal.protocols.time.sleep"), mock.patch("sys.stdout"):
        proto.status_packet = proto.get_status_packet()
        proto.initialize_model()
        proto.initialize()
        proto.handshake()
        proto.erase_flash(len(image), proto.model.code)
        proto.program_flash(image)
        proto.program_options()
        proto.disconnect()
    if pending:
        raise SystemExit(f"{len(pending)} responses left over")

    lines = ["S " + responses[0].hex()]
    lines += ["R " + r.hex() for r in responses[1:]]
    lines += ["W " + w.hex() for w in writes]
    out_path.write_text("\n".join(lines) + "\n")
    print(f"{len(writes)} packets from stcgal, trim {proto.trim_frequency} Hz")
    return 0


if __name__ == "__main__":
    sys.exit(main())
