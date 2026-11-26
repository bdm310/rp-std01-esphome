from __future__ import annotations

import contextlib
import io
import os
from pathlib import Path
import tempfile
import unittest

import mkesphome as m

C = m.constants()
FM = m.layout()


def make_app(version: int = 0x0102, build: int = 0xDEADBEEF) -> bytes:
    base = FM.app_base
    return m.build_app({base: 0x02, base + 1: 0x06, base + 2: 0x10, base + 0x100: 0xAA}, version, build)


def make_full(boot_version: int = 3) -> bytes:
    return m.build_full({0x0000: 0x02, FM.boot_version_addr: boot_version}, make_app())


class ConstantsTest(unittest.TestCase):
    def test_defines(self) -> None:
        text = "\n".join(
            [
                "#ifndef X_H",
                "#define X_H",
                "#define A 0x10u /* a comment */",
                "#define B (A - 1u)",
                "#define F(x) ((x) + 1)",
                "  #define C 3",
                "/* #define D 4 */",
                "#endif",
            ]
        )
        self.assertEqual(m.read_defines(text), [("A", "0x10u"), ("B", "(A - 1u)"), ("C", "3")])
        with self.assertRaises(ValueError):
            m.read_defines("#define A 1 /* goes on")

    def test_firmware_constants(self) -> None:
        self.assertEqual(C["APP_SIZE"], C["APP_END"] - C["APP_BASE"])
        self.assertEqual(C["BOOT_VERSION_ADDR"], C["APP_BASE"] - 1)
        self.assertEqual(C["PCA_HZ"], 921600)
        self.assertEqual(C["EARLIEST_FIRE"], round(500e-6 * C["PCA_HZ"]))
        self.assertEqual(C["PERIOD_MIN"], C["PCA_HZ"] // 65)

    def test_host_and_sdcc_programs(self) -> None:
        self.assertIn("SHOW(PCA_HZ);", m.constants_program())
        self.assertIsInstance(C["APP_MAGIC"], str)
        self.assertIn(
            f'_Static_assert((PERIOD_MIN) == {C["PERIOD_MIN"]}ul, "PERIOD_MIN is not {C["PERIOD_MIN"]} on the MCU");',
            m.sdcc_check(),
        )
        self.assertIn(f"FWC_APP_BASE := {C['APP_BASE']}\n", m.make_variables())

    def test_types(self) -> None:
        values = m.evaluated()
        self.assertEqual(values["FRAME_OVERHEAD"], m.Constant(5, True))
        self.assertEqual(values["SCHED_MARGIN"], m.Constant(100, False))
        self.assertFalse(values["RELEASE_OVERDUE"].unsigned)
        self.assertEqual(values["APP_MAGIC"], m.Constant("RPA1", False))
        self.assertEqual(m.cpp_type(m.Constant(0xA5, True)), "uint8_t")
        self.assertEqual(m.cpp_type(m.Constant(1229, True)), "uint16_t")
        self.assertEqual(m.cpp_type(m.Constant(921600, True)), "uint32_t")
        self.assertEqual(m.cpp_type(m.Constant(300, False)), "int")
        self.assertEqual(m.cpp_constant(m.Define("A", "0x00F0u"), m.Constant(0xF0, True)), "inline constexpr uint8_t A = 0x00F0u;")
        self.assertEqual(m.cpp_constant(m.Define("A", "(B - 1u)"), m.Constant(0xDFF, True)), "inline constexpr uint16_t A = 3583u;")
        self.assertEqual(m.cpp_constant(m.Define("A", "((int) B)"), m.Constant(-3, False)), "inline constexpr int A = -3;")
        self.assertEqual(m.cpp_constant(m.Define("FW_VERSION", "0x0001u"), m.Constant(1, True)), "inline constexpr uint16_t FW_VERSION = 0x0001u;")
        self.assertEqual(m.cpp_constant(m.Define("S", "\"AB\""), m.Constant("AB", False)), 'inline constexpr char S[] = "AB";')


class ImageTest(unittest.TestCase):
    def test_crc16_check_value(self) -> None:
        self.assertEqual(m.crc16(b"123456789"), 0x29B1)

    def test_app_image_and_trailer(self) -> None:
        image = make_app()
        self.assertEqual(len(image), FM.app_size)
        self.assertEqual(image[0:4], b"\x02\x06\x10\xff")
        t = m.parse_trailer(image)
        self.assertEqual(t, m.Trailer(FM.magic, 0x101, m.crc16(image[:0x101]), 0x0102, 0xDEADBEEF, 0xFF))
        with self.assertRaises(ValueError):
            m.parse_trailer(image[:-1])

    def test_app_out_of_range(self) -> None:
        with self.assertRaises(ValueError):
            m.build_app({FM.app_base - 1: 0}, 1, 0)
        with self.assertRaises(ValueError):
            m.build_app({FM.app_end - FM.trailer_size: 0}, 1, 0)

    def test_full_image(self) -> None:
        full = make_full()
        self.assertEqual(len(full), FM.app_end)
        self.assertEqual(full[0], 0x02)
        self.assertEqual(full[FM.boot_version_addr], 3)
        self.assertEqual(full[FM.app_end - 1], 0x00)
        self.assertEqual(m.app_area(full), make_app())
        with self.assertRaises(ValueError):
            m.build_full({FM.app_base: 0}, make_app())
        with self.assertRaises(ValueError):  # no version byte
            m.build_full({0x0000: 0x02}, make_app())

    def test_missing_bumps(self) -> None:
        app = {FM.app_base: 0x02}
        boot = {0: 0x02, FM.boot_version_addr: 1}
        old = m.build_images(app, boot, 1)
        self.assertEqual(m.missing_bumps(old, old), [])
        changed_boot = {**boot, 1: 0x00}
        (message,) = m.missing_bumps(old, m.build_images(app, changed_boot, 1))
        self.assertIn("BOOT_VERSION above 1 in firmware/include/protocol.h", message)
        self.assertEqual(m.missing_bumps(old, m.build_images(app, {**changed_boot, FM.boot_version_addr: 2}, 1)), [])
        (message,) = m.missing_bumps(old, m.build_images(app, {**changed_boot, FM.boot_version_addr: 0}, 1))
        self.assertIn("raise BOOT_VERSION above 1", message)
        changed_app = {**app, FM.app_base + 1: 0x00}
        (message,) = m.missing_bumps(old, m.build_images(changed_app, boot, 1))
        self.assertIn("FW_VERSION above 0x0001 in firmware/include/board.h", message)
        self.assertEqual(m.missing_bumps(old, m.build_images(changed_app, boot, 2)), [])
        (message,) = m.missing_bumps(old, m.build_images(changed_app, boot, 0))
        self.assertIn("raise FW_VERSION above 0x0001", message)

    def test_build_id(self) -> None:
        app = {FM.app_base: 0x02, FM.app_base + 0x40: 0x11}

        def bid(app: dict[int, int], version: int = 2) -> int:
            return m.build_id(m.build_app(app, version, 0), version)

        first = bid(app)
        self.assertEqual(bid(dict(app)), first)
        self.assertNotEqual(bid({**app, FM.app_base + 0x41: 0}), first)
        self.assertNotEqual(bid(app, 3), first)
        moved = {a + 1: b for a, b in app.items() if a != FM.app_base}
        moved[FM.app_base] = 0x02
        self.assertNotEqual(bid(moved), first)
        self.assertEqual(m.build_id(m.build_app(app, 2, 0x12345678), 2), first)
        # Another bootloader, same application: same build ID.
        one = m.build_images(app, {0: 0x02, FM.boot_version_addr: 3}, 2)
        two = m.build_images(app, {0: 0x03, FM.boot_version_addr: 4}, 2)
        self.assertEqual(m.parse_trailer(m.app_area(one)).build, first)
        self.assertEqual(m.app_area(one), m.app_area(two))

    def test_ihex_round_trip(self) -> None:
        data = bytes(range(40)) + b"\xff" * 20 + b"\x01"
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "x.ihx"
            path.write_text(m.render_ihex(data))
            mem = m.read_ihex(path)
        self.assertEqual(bytes(mem.get(i, 0xFF) for i in range(len(data))), data)
        self.assertEqual(max(mem), len(data) - 1)


class HeadersTest(unittest.TestCase):
    def test_main_writes_and_checks(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            build, esphome = Path(tmp) / "build", Path(tmp) / "components"
            build.mkdir()
            for name in (m.IMAGE_H, m.CONSTANTS_H):
                (esphome / name).parent.mkdir(parents=True, exist_ok=True)
            boot = bytearray(b"\xff" * FM.app_base)
            boot[0:3] = b"\x02\x00\xc0"
            boot[FM.boot_version_addr] = 5
            (build / "boot.ihx").write_text(m.render_ihex(bytes(boot)))
            (build / "app.ihx").write_text(m.render_ihex(b"\xff" * FM.app_base + b"\x02\x0e\x10\x22"))
            args = ["--build", str(build), "--components", str(esphome)]
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(m.main(args), 0)
            full = m.read_image_header(esphome / m.IMAGE_H)
            written = {p.relative_to(esphome).as_posix() for p in esphome.rglob("*") if p.is_file()}
            self.assertEqual(written, {m.IMAGE_H, m.CONSTANTS_H})
            mem = m.read_ihex(build / "full.ihx")
            self.assertEqual(bytes(mem.get(a, 0xFF) for a in range(len(full))), full)
            trailer = m.parse_trailer(m.app_area(full))
            self.assertEqual(trailer.version, C["FW_VERSION"])
            constants = (esphome / m.CONSTANTS_H).read_text()
            self.assertIn(f"inline constexpr uint32_t MCU_IMAGE_BUILD = 0x{trailer.build:08X}u;", constants)
            self.assertIn("inline constexpr uint8_t MCU_IMAGE_BOOT_VERSION = 5u;", constants)
            self.assertIn("inline constexpr uint16_t APP_BASE = 0x0E00u;", constants)

            os.utime(esphome / m.IMAGE_H, (1000, 1000))
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(m.main(args), 0)
                self.assertEqual(m.main([*args, "--check"]), 0)
            self.assertEqual((esphome / m.IMAGE_H).stat().st_mtime, 1000)  # unchanged: not rewritten

            (esphome / m.CONSTANTS_H).write_text(constants.replace("= 5u;", "= 4u;"))
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                self.assertEqual(m.main([*args, "--check"]), 1)
            self.assertIn(f"{esphome / m.CONSTANTS_H} does not match the firmware: run make firmware", out.getvalue())

    def test_committed_headers_agree(self) -> None:
        esphome = m.COMPONENTS
        full = m.read_image_header()
        trailer = m.parse_trailer(m.app_area(full))
        self.assertEqual(trailer.magic, FM.magic)
        self.assertEqual(trailer.crc, m.crc16(full[FM.app_base : FM.app_base + trailer.code_len]))
        self.assertEqual(trailer.version, C["FW_VERSION"])
        self.assertEqual(full[FM.boot_version_addr], C["BOOT_VERSION"])
        self.assertEqual(trailer.build, m.build_id(m.app_area(full), trailer.version))
        for name, text in m.esphome_headers(full).items():
            with self.subTest(name):
                self.assertEqual((esphome / name).read_text(), text, f"run make firmware and commit {name}")


if __name__ == "__main__":
    unittest.main()
