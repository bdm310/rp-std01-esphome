"""Host-platform tests of the MCU tool: the ESPHome host build of rp-std01-mcu-tool-core.yaml
(tool-host.yaml) against the fake MCU, for the application update through our bootloader. The STC
install needs a power-on, so it is not tested here.

Run with: make test-host
"""

from __future__ import annotations

import base64
from pathlib import Path
import re
import time

import pytest
import pytest_asyncio

from conftest import TESTS, compile_host, config_rejected, substitution
from esp_api import Esp, until
from mcu_proto import (
    APP_SIZE,
    BOOT_VERSION,
    FW,
    MSG_BOOT_BEGIN,
    MSG_BOOT_FINISH,
    MSG_BOOT_RUN,
    MSG_BOOT_STATUS,
    MSG_BOOT_WRITE,
    MSG_UPDATE,
    FakeMcu,
    image_with_build,
    image_with_version,
    parse_trailer,
)

pytestmark = pytest.mark.asyncio

MCU_PORT = substitution("tool-host.yaml", "mcu_port")
API_PORT = int(substitution("tool-host.yaml", "api_port"))
OLD_BUILD = 0x12345678
TOOL_H = (Path(__file__).parent.parent / "components" / "rpstd01_tool" / "rpstd01_tool.h").read_text()
CHECK_LISTEN_S = int(re.search(r"\bCHECK_LISTEN_MS = (\d+);", TOOL_H).group(1)) / 1000
WRITES = APP_SIZE // FW["WRITE_CHUNK"]


@pytest.fixture(scope="session")
def tool_bin() -> Path:
    return compile_host("tool-host.yaml")


class ToolRig:
    def __init__(self, make_mcu, tool_bin: Path, workdir: Path) -> None:
        self.make_mcu = make_mcu
        self.esp = Esp(tool_bin, workdir, API_PORT)
        self.workdir = workdir
        self.mcu: FakeMcu | None = None

    def start_mcu(self, **mcu_kwargs) -> FakeMcu:
        self.mcu = self.make_mcu(link=MCU_PORT, **mcu_kwargs)
        return self.mcu

    async def start(self, **mcu_kwargs) -> None:
        self.start_mcu(**mcu_kwargs)
        await self.esp.start()

    def state(self) -> str:
        return self.esp.value("textsensor", "MCU State") or ""

    async def until_state(self, part: str, timeout: float = 10.0) -> str:
        return await until(lambda: part in self.state() and self.state(), timeout, repr(part))

    async def until_update(self, prefix: str, timeout: float = 20.0) -> str:
        return await until(
            lambda: (t := self.esp.value("textsensor", "Application Update") or "").startswith(prefix) and t,
            timeout,
            prefix,
        )

    async def close(self) -> None:
        await self.esp.stop()
        for i in range(1, self.esp.runs + 1):
            log = self.workdir / f"esp{i}.log"
            if log.exists():
                tail = log.read_text(errors="replace").splitlines()[-60:]
                print(f"---- {log.name} (tail)\n" + "\n".join(tail))


@pytest_asyncio.fixture
async def rig(make_mcu, tool_bin, tmp_path):
    r = ToolRig(make_mcu, tool_bin, tmp_path)
    yield r
    await r.close()


def log_has(mcu, event: str, count: int, timeout: float = 5.0) -> None:
    """Waits until the fake MCU has logged an event count times since its log was last read."""
    seen: list[str] = []

    def enough(_: dict) -> bool:
        seen.extend(e for e in mcu.log() if event in e)
        return len(seen) >= count

    mcu.wait_for(enough, timeout)


CURRENT = "Everything is current: install the main configuration."
UPDATED = "Application updated. Everything is current: install the main configuration."


async def test_current_does_nothing(rig: ToolRig) -> None:
    await rig.start()
    await rig.until_state(CURRENT, 8)
    rig.mcu.cmd("clear")
    await until(rig.mcu.statuses_sent(2.0), 8, "statuses")
    assert rig.mcu.get()["rx"] == {}  # nothing sent after the check
    assert rig.esp.value("textsensor", "Application Update") == "Idle"
    assert f"bootloader v{BOOT_VERSION}" in rig.esp.value("textsensor", "MCU Installed")


async def test_app_old_updated_at_once(rig: ToolRig, app_image: bytes) -> None:
    """Only the application out of date: updated through our bootloader, no power cycle."""
    await rig.start(image_bytes=image_with_build(app_image, OLD_BUILD))
    t0 = time.monotonic()
    await rig.until_state("Updating the application through our bootloader", 8)
    await rig.until_update("Done", 20)
    took = time.monotonic() - t0
    await rig.until_state(UPDATED, 5)
    s = rig.mcu.get()
    assert s["mode"] == "app" and s["flash_valid"] == 1 and s["build"] == f"{parse_trailer(app_image).build:08X}"
    assert s["rx"][MSG_BOOT_WRITE] == WRITES and s["rx"][MSG_BOOT_BEGIN] == 1 and s["rx"][MSG_UPDATE] >= 1
    await until(lambda: rig.esp.value("sensor", "Application Update Progress") == 100, 3, "progress 100")
    assert "Install at next power-on" not in (rig.esp.value("textsensor", "MCU Install Armed") or "")
    print(f"update took {took:.1f} s")
    assert took < 10


async def test_repair_invalid_application(rig: ToolRig) -> None:
    """Our bootloader with no valid application: repaired at once."""
    await rig.start(invalid=True)
    await rig.until_state("with no valid application. Updating the application", 8)
    await rig.until_update("Done", 20)
    await rig.until_state(UPDATED, 5)
    rig.mcu.wait_for(lambda s: s["mode"] == "app" and s["flash_valid"] == 1, 2)


async def in_update_mode(rig: ToolRig, image: bytes) -> None:
    rig.start_mcu(image_bytes=image)
    rig.mcu.cmd("updatemode")
    rig.mcu.wait_for(lambda s: s["mode"] == "boot" and s["boot_reason"] == FW["BOOT_REASON_REQUEST"], 2)
    await rig.esp.start()


async def test_update_mode_old_application(rig: ToolRig, app_image: bytes) -> None:
    """Update mode with a valid application: it is started, and its info frame shows it is old."""
    await in_update_mode(rig, image_with_build(app_image, OLD_BUILD))
    await rig.until_state("another build (its bootloader is current). Updating the application", 8)
    await rig.until_update("Done", 20)
    await rig.until_state(UPDATED, 5)
    s = rig.mcu.get()
    assert s["rx"][MSG_BOOT_RUN] == 2 and s["rx"][MSG_BOOT_BEGIN] == 1 and s["rx"][MSG_UPDATE] >= 1


async def test_update_mode_newer_application_kept(rig: ToolRig, app_image: bytes) -> None:
    """Update mode with a valid newer application: it is started and left as it is."""
    newer = image_with_version(image_with_build(app_image, OLD_BUILD), FW["FW_VERSION"] + 1)
    await in_update_mode(rig, newer)
    await rig.until_state("The MCU's application is newer than this tool's", 8)
    assert (rig.esp.value("textsensor", "MCU Install Armed") or "").startswith("Nothing")
    s = rig.mcu.get()
    assert s["mode"] == "app" and s["build"] == f"{OLD_BUILD:08X}"
    assert s["rx"][MSG_BOOT_RUN] == 1 and MSG_BOOT_BEGIN not in s["rx"]


async def test_update_lost_replies_and_corruption(rig: ToolRig) -> None:
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    await rig.until_state(CURRENT, 8)
    esp.client.button_command(esp.info("button", "Update Application Now").key)
    mcu.wait_for(lambda s: s["receiving"] == 1 and s["boot_next"] >= 1024, 10)
    mcu.log()
    mcu.cmd(f"drop tx 2 {MSG_BOOT_STATUS:#x}")
    log_has(mcu, "dropped tx", 2)
    mcu.cmd(f"corrupt rx 1 {MSG_BOOT_WRITE:#x}")  # a bad CRC
    log_has(mcu, "corrupted rx", 1)
    mcu.cmd(f"drop rx 1 {MSG_BOOT_WRITE:#x}")
    await rig.until_update("Done", 25)
    s = mcu.wait_for(lambda s: s["mode"] == "app", 2)
    assert s["flash_valid"] == 1 and s["rx"][MSG_BOOT_BEGIN] == 1 and s["rx"][MSG_BOOT_WRITE] > WRITES


async def test_update_tampered_write_restarts(rig: ToolRig) -> None:
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    await rig.until_state(CURRENT, 8)
    mcu.cmd(f"tamper rx 1 {MSG_BOOT_WRITE:#x}")  # bad data with a good CRC: Finish fails, Begin again
    esp.client.button_command(esp.info("button", "Update Application Now").key)
    await rig.until_update("Done", 30)
    s = mcu.wait_for(lambda s: s["mode"] == "app", 2)
    assert s["flash_valid"] == 1 and s["rx"][MSG_BOOT_BEGIN] == 2 and s["rx"][MSG_BOOT_FINISH] >= 2


async def test_update_mcu_power_cut(rig: ToolRig) -> None:
    """Power cut of the MCU alone during the writes: the update starts again or repairs."""
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    await rig.until_state(CURRENT, 8)
    esp.client.button_command(esp.info("button", "Update Application Now").key)
    mcu.wait_for(lambda s: s["receiving"] == 1 and s["boot_next"] >= 2048, 10)
    mcu.cmd("powercut 300")
    mcu.wait_for(lambda s: s["mode"] == "boot" and s["boot_reason"] == FW["BOOT_REASON_INVALID"], 3)
    await rig.until_update("Done", 40)
    mcu.wait_for(lambda s: s["mode"] == "app" and s["flash_valid"] == 1, 2)


async def test_power_cut_both_repaired_at_start(rig: ToolRig) -> None:
    """Power cut of both chips during the writes: at its next start the tool repairs the MCU at once."""
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    await rig.until_state(CURRENT, 8)
    esp.client.button_command(esp.info("button", "Update Application Now").key)
    mcu.wait_for(lambda s: s["receiving"] == 1 and s["boot_next"] >= 2048, 10)
    await esp.stop(kill=True)
    mcu.cmd("powercut 300")
    mcu.wait_for(lambda s: s["mode"] == "boot" and s["boot_reason"] == FW["BOOT_REASON_INVALID"] and s["flash_valid"] == 0, 3)
    t0 = time.monotonic()
    await esp.start()
    await rig.until_state("Updating the application", CHECK_LISTEN_S)
    decided = time.monotonic() - t0
    assert decided < CHECK_LISTEN_S, decided  # the check decides on the first boot status
    await rig.until_update("Done", 20)
    mcu.wait_for(lambda s: s["mode"] == "app" and s["flash_valid"] == 1, 2)
    await rig.until_state(UPDATED, 5)


async def test_update_aux_ram_lost(rig: ToolRig, app_image: bytes) -> None:
    rig.start_mcu(image_bytes=image_with_build(app_image, OLD_BUILD))
    rig.mcu.cmd("auxram false")
    await rig.esp.start()
    text = await rig.until_update("Failed", 15)
    assert "without entering update mode" in text
    await rig.until_state("Press Update Application Now", 3)
    s = rig.mcu.get()
    assert s["mode"] == "app" and s["boot_entries"] == 0


async def test_longer_frames_hold_the_install(rig: ToolRig) -> None:
    """A later release whose frames are all longer: no install is armed, so nothing is downgraded."""
    await rig.start(extra=("--pad", "4"))
    await rig.until_state("The MCU runs firmware this tool does not understand", 8)
    assert "Nothing is armed" in rig.state()
    assert (rig.esp.value("textsensor", "MCU Install Armed") or "").startswith("Nothing")


async def test_old_bootloader_arms_install(rig: ToolRig) -> None:
    """A bootloader older than the carried one: an install through STC's bootloader is armed."""
    await rig.start(extra=("--boot-version", str(BOOT_VERSION - 1)))
    await rig.until_state("Install armed", 8)
    assert "older than the carried one" in rig.state()
    await until(
        lambda: (rig.esp.value("textsensor", "MCU Install Armed") or "").startswith("Install at next power-on"),
        3,
        "install armed",
    )
    rig.mcu.cmd("clear")
    await until(rig.mcu.statuses_sent(2.0), 8, "statuses")
    assert rig.mcu.get()["rx"] == {}


@pytest.mark.parametrize("pin", ["rx_pin", "tx_pin"])
async def test_uart_off_the_power_on_pins_is_rejected(pin: str, tmp_path: Path) -> None:
    out = config_rejected(
        [
            "substitutions:",
            "  name: pins",
            "  friendly_name: Pins",
            "  wifi_ssid: ssid",
            "  wifi_password: password",
            f"  api_key: {base64.b64encode(bytes(range(32))).decode()}",
            "  fallback_ap_password: password",
            "packages:",
            f"  tool: !include {TESTS.parent / 'rp-std01-mcu-tool.yaml'}",
            "uart:",
            f"  {pin}: GPIO3",
        ],
        tmp_path,
    )
    assert f"the UART of rpstd01_tool needs {pin}: GPIO" in out
