"""Host-platform tests: the ESPHome build of rp-std01-core.yaml (host.yaml) against the fake MCU.

MCU updates belong to the MCU tool's tests: here only what the main configuration reports, and its
silence while the MCU is in update mode.
"""

from __future__ import annotations

import asyncio
from collections.abc import Callable
import math
from pathlib import Path
import re
import time

from esphome import yaml_util
import pytest
import pytest_asyncio

from conftest import TESTS, compile_host, config_rejected, substitution
from esp_api import Esp, until
from mcu_proto import (
    BOOT_VERSION,
    FW,
    MSG_GATE,
    MSG_OUTPUTS,
    ST_FAULT,
    ST_FIRING,
    ST_OFF,
    ST_SETTLE,
    ST_STOPPING,
    FakeMcu,
    half_cycle_counts,
    image_with_build,
    image_with_version,
    parse_trailer,
)

pytestmark = pytest.mark.asyncio

MCU_PORT = substitution("host.yaml", "mcu_port")
API_PORT = int(substitution("host.yaml", "api_port"))
HALF_60HZ, HALF_50HZ = half_cycle_counts(60), half_cycle_counts(50)
EARLIEST_FIRE, LATEST_BEFORE_EDGE = FW["EARLIEST_FIRE"], FW["LATEST_BEFORE_EDGE"]

RPSTD01_H = (TESTS.parent / "components" / "rpstd01" / "rpstd01.h").read_text()
CLAMP_MARGIN = int(re.search(r"\bCLAMP_MARGIN = (\d+);", RPSTD01_H).group(1))
K_A_MS, K_B_MS = (float(k) for k in re.search(r"\bK_A_MS = ([\d.]+)f, K_B_MS = ([\d.]+)f;", RPSTD01_H).groups())
WHEEL_RAW_A, WHEEL_RAW_B = (int(substitution("host.yaml", f"thumbwheel_raw_{end}")) for end in "ab")
LED_B3 = FW["OUTPUTS_B3"]
HOT, COOL = FW["THERM_TRIP"], FW["THERM_CLEAR"]
BAR_LEDS = [FW[name] for name in sorted(n for n in FW if re.fullmatch(r"OUTPUTS_LED\d", n))]
BAR_MASK = sum(BAR_LEDS)

CORE_YAML = (TESTS.parent / "rp-std01-core.yaml").read_text()
INITIAL = {str(n["id"]): n["initial_value"] for n in yaml_util.load_yaml(TESTS.parent / "rp-std01-core.yaml")["number"]}
DIM_END, BRIGHT_END = INITIAL["dim_end"] / 100, INITIAL["bright_end"] / 100
GESTURE_COOLDOWN = max(int(ms) for ms in re.findall(r"invalid_cooldown: (\d+)ms", CORE_YAML)) / 1000


def bar(lit: int) -> int:
    """The LED bits of the brightness bar with this many LEDs lit."""
    return sum(BAR_LEDS[:lit])


async def past_gesture_cooldown() -> None:
    """A multi-click trigger ignores presses for its invalid_cooldown after a press it did not match.
    The end of it shows nowhere (ESPHome logs it only at VERBOSE), so this waits out the longest,
    plus a little for the press to reach ESPHome."""
    await asyncio.sleep(GESTURE_COOLDOWN + 0.1)


def gate_received(s: dict) -> bool:
    return s["rx"].get(MSG_GATE, 0) > 0


def held(button: int) -> Callable[[dict], bool]:
    """The fake MCU's button pin reads low; button is the number in its press command."""
    return lambda s: s["buttons"] & (1 << button) == 0


def power_share(delay: float, half: float) -> float:
    t = math.pi * delay / half
    return 1 - t / math.pi + math.sin(2 * t) / (2 * math.pi)


def wheel_dim(raw: int, half: int = HALF_60HZ) -> float:
    """The dim end the thumbwheel sets at this raw reading, as a fraction of the half cycle."""
    k_ms = K_A_MS + (raw - WHEEL_RAW_A) * (K_B_MS - K_A_MS) / (WHEEL_RAW_B - WHEEL_RAW_A)
    return min(max(k_ms, K_A_MS), K_B_MS) * 1e-3 * FW["PCA_HZ"] / half


def expected_target(level: float, half: int = HALF_60HZ, dim: float = DIM_END, bright: float = BRIGHT_END) -> int:
    """The firing delay for a brightness level."""
    if level <= 0:
        return 0
    d_dim = dim * half
    d_bright = max(bright * d_dim, EARLIEST_FIRE)
    l_dim, l_bright = power_share(d_dim, half) ** (1 / 3), power_share(d_bright, half) ** (1 / 3)
    want = (l_dim + level * (l_bright - l_dim)) ** 3
    lo, hi = 0.0, float(half)
    for _ in range(40):
        mid = (lo + hi) / 2
        if power_share(mid, half) > want:
            lo = mid
        else:
            hi = mid
    d = round((lo + hi) / 2)
    return max(EARLIEST_FIRE, min(d, half - LATEST_BEFORE_EDGE - CLAMP_MARGIN))


@pytest.fixture(scope="session")
def host_bin() -> Path:
    return compile_host("host.yaml")


class Rig:
    def __init__(self, make_mcu, host_bin: Path, workdir: Path) -> None:
        self.make_mcu = make_mcu
        self.esp = Esp(host_bin, workdir, API_PORT)
        self.workdir = workdir
        self.mcu: FakeMcu | None = None

    async def start(self, lamp_off: bool = True, env: dict[str, str] | None = None, **mcu_kwargs) -> None:
        self.mcu = self.make_mcu(link=MCU_PORT, **mcu_kwargs)
        self.mcu.wait_past_startup()
        await self.esp.start(env)
        await self.select("Thumbwheel Use", "Input Only")
        await until(lambda: self.esp.value("textsensor", "MCU State") not in (None, "No Response"), 5, "MCU State")
        if lamp_off:
            await self.esp.light(False)
            await until(lambda: self.mcu.get()["state"] == ST_OFF, 3, "lamp off")

    async def close(self) -> None:
        await self.esp.stop()
        for i in range(1, self.esp.runs + 1):
            log = self.workdir / f"esp{i}.log"
            if log.exists():
                tail = log.read_text(errors="replace").splitlines()[-60:]
                print(f"---- {log.name} (tail)\n" + "\n".join(tail))

    async def select(self, name: str, option: str) -> None:
        await self.esp.select(name, option)
        await until(lambda: self.esp.value("select", name) == option, 2, f"{name}: {option}")

    def mcu_until(self, pred: Callable[[dict], bool], timeout: float = 5.0) -> dict:
        return self.mcu.wait_for(pred, timeout)


@pytest_asyncio.fixture
async def rig(make_mcu, host_bin, tmp_path):
    r = Rig(make_mcu, host_bin, tmp_path)
    yield r
    await r.close()


def firing_at(target: int, tol: int = 3) -> Callable[[dict], bool]:
    return lambda s: s["state"] == ST_FIRING and abs(s["target"] - target) <= tol and abs(s["current"] - target) <= tol


def bar_shows(lit: int) -> Callable[[dict], bool]:
    return lambda s: s["leds"] & BAR_MASK == bar(lit)


async def test_status_decoding(rig: Rig, app_image: bytes) -> None:
    await rig.start()
    esp = rig.esp
    await until(lambda: esp.value("textsensor", "MCU State") == "Off", 2, "MCU State Off")
    await until(lambda: esp.value("textsensor", "MCU Faults") == "none", 2, "faults none")
    await until(lambda: esp.value("textsensor", "MCU Firmware") is not None, 2, "firmware")
    assert esp.value("textsensor", "MCU Firmware") == f"{parse_trailer(app_image).version:04X} build {rig.mcu.get()['build']}"
    assert esp.value("textsensor", "MCU Firmware Expected") == esp.value("textsensor", "MCU Firmware")
    assert esp.value("binarysensor", "MCU Update Available") is False
    assert esp.value("binarysensor", "MCU Problem") is False
    await until(lambda: esp.value("sensor", "Mains Frequency") is not None, 7, "frequency")
    assert esp.value("sensor", "Mains Frequency") == pytest.approx(60.0, abs=0.01)
    await until(lambda: esp.value("sensor", "Temperature") is not None, 7, "temperature")
    assert 20 < esp.value("sensor", "Temperature") < 28  # the simulated room
    sim = rig.mcu.get()
    for name, raw in (("Thermistor Raw", sim["therm_raw"]), ("Thumbwheel Raw", sim["wheel_raw"])):
        await until(lambda name=name, raw=raw: esp.value("sensor", name) == raw, 7, name)
    reset_cause = f"Power-on, watchdog count 0, bootloader {BOOT_VERSION}"
    assert esp.value("textsensor", "MCU Reset Cause") == reset_cause
    rig.mcu.cmd(f"half {HALF_50HZ}")
    rig.mcu.cmd("therm 850")
    await until(lambda: esp.value("sensor", "Mains Frequency") == pytest.approx(50.0, abs=0.01), 7, "50 Hz")
    await until(lambda: esp.value("sensor", "Thermistor Raw") == 850, 7, "therm raw")
    rig.mcu.cmd("zo off")
    await until(lambda: "mains lost" in (esp.value("textsensor", "MCU Faults") or ""), 2, "mains lost")


async def test_light_on_brightness_off(rig: Rig) -> None:
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    mcu.log()
    await esp.light(True, 1.0, transition=1.0)
    s = rig.mcu_until(firing_at(expected_target(1.0)), 4)
    assert s["limits"] == [INITIAL["fade_limit"]] * 2 and s["relay"] == 1
    log = mcu.log()
    gates = [int(e.split()[2]) for e in log if e.split()[1] == "gate"]
    # the transition starts near the dim end and only gets brighter (smaller delays)
    assert gates[0] >= expected_target(0.1)
    assert all(a >= b for a, b in zip(gates, gates[1:])), gates
    assert abs(gates[-1] - expected_target(1.0)) <= 3, gates
    assert len(set(gates)) >= 8, gates  # stepped through the transition, not jumped
    seq = [e.split()[1:3] for e in log if e.split()[1] in ("state", "relay")]
    assert seq[:3] == [["state", str(ST_SETTLE)], ["relay", "1"], ["state", str(ST_FIRING)]]
    t = {f"{a} {b}": int(e.split()[0]) for e, (a, b) in zip([e for e in log if e.split()[1] in ("state", "relay")], seq)}
    assert t[f"state {ST_FIRING}"] - t[f"state {ST_SETTLE}"] >= FW["RELAY_SETTLE_MS"]
    rig.mcu_until(bar_shows(len(BAR_LEDS)), 2)
    assert mcu.get()["night"] == 0  # On When Lamp Off

    for level in (0.5, 0.01, 0.8):
        await esp.light(True, level, transition=0)
        rig.mcu_until(firing_at(expected_target(level)), 3)
    rig.mcu_until(bar_shows(4), 2)

    mcu.log()
    await esp.light(False, transition=0.5)
    rig.mcu_until(lambda s: s["state"] == ST_OFF and s["relay"] == 0, 3)
    log = mcu.log()
    seq = [e.split()[1:3] for e in log if e.split()[1] in ("state", "relay")]
    assert seq == [["state", str(ST_STOPPING)], ["state", str(ST_OFF)], ["relay", "0"]]
    gates = [int(e.split()[2]) for e in log if e.split()[1] == "gate"]
    assert gates[-1] == 0 and all(a <= b for a, b in zip(gates[:-1], gates[1:-1]))
    rig.mcu_until(lambda s: bar_shows(0)(s) and s["night"] == 77, 2)


async def test_brightness_bar_off(rig: Rig) -> None:
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    await esp.light(True, 0.5)
    rig.mcu_until(bar_shows(3), 3)
    await rig.select("Brightness Bar", "Off")
    rig.mcu_until(bar_shows(0), 2)
    await esp.light(True, 0.9)
    rig.mcu_until(firing_at(expected_target(0.9)), 3)
    await until(mcu.statuses_sent(1.0), 5, "statuses")
    assert bar_shows(0)(mcu.get())
    await rig.select("Brightness Bar", "Level")
    rig.mcu_until(bar_shows(len(BAR_LEDS)), 2)


async def test_night_light_mode_light(rig: Rig) -> None:
    """In Light mode the night light no longer follows the lamp."""
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    rig.mcu_until(lambda s: s["night"] > 0, 2)
    await rig.select("Night Light Mode", "Light")
    await esp.light(True, 0.5)
    rig.mcu_until(firing_at(expected_target(0.5)), 3)
    await until(mcu.statuses_sent(1.0), 5, "statuses")
    assert mcu.get()["night"] > 0
    await rig.select("Night Light Mode", "On When Lamp Off")
    rig.mcu_until(lambda s: s["night"] == 0, 2)


async def test_thumbwheel_use(rig: Rig) -> None:
    """Dim End: the calibrated thumbwheel sets the dim end. Input Only: the Dim End number does."""
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    await esp.light(True, 0.5)
    rig.mcu_until(firing_at(expected_target(0.5)), 3)
    await rig.select("Thumbwheel Use", "Dim End")
    for raw in (WHEEL_RAW_A, WHEEL_RAW_B):
        mcu.cmd(f"wheel {raw}")
        rig.mcu_until(firing_at(expected_target(0.5, dim=wheel_dim(raw))), 3)
    await rig.select("Thumbwheel Use", "Input Only")
    rig.mcu_until(firing_at(expected_target(0.5)), 3)


async def test_dropped_commands_recovered(rig: Rig) -> None:
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    await esp.light(True, 0.5)
    rig.mcu_until(firing_at(expected_target(0.5)))
    mcu.cmd(f"drop rx 3 {MSG_GATE:#x}")
    t0 = time.monotonic()
    await esp.light(True, 0.9)
    rig.mcu_until(firing_at(expected_target(0.9)), 2)
    assert time.monotonic() - t0 < 0.6
    assert mcu.get()["rx_dropped"] == 3

    mcu.cmd(f"drop rx 1 {MSG_OUTPUTS:#x}")
    t0 = time.monotonic()
    await esp.light(True, 0.3)
    rig.mcu_until(bar_shows(2), 2.5)
    assert time.monotonic() - t0 < 0.5  # sent again RESEND_MS after the status still showed the old LEDs

    # corrupted frames are dropped by the MCU's CRC check and sent again
    mcu.cmd(f"corrupt rx 2 {MSG_GATE:#x}")
    await esp.light(True, 0.7)
    rig.mcu_until(firing_at(expected_target(0.7)), 2)


async def test_mcu_reset_lamp_returns(rig: Rig) -> None:
    """The MCU resets on its own while the lamp is on."""
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    await esp.light(True, 0.6)
    rig.mcu_until(firing_at(expected_target(0.6)))
    mcu.cmd("powercut 100")
    rig.mcu_until(lambda s: s["mode"] == "app" and s["uptime_ms"] < 1000 and s["app_starts"] == 2, 2)
    # The firmware reports FAULT (thermistor average not ready) for its first ~80 ms after any reset:
    # taken as a fault event, it would turn the light off before the lamp could fire again.
    rig.mcu_until(firing_at(expected_target(0.6)), 3)
    await until(lambda: esp.lamp() == (True, pytest.approx(0.6, abs=0.01)), 2, "light on at 0.6 after the MCU reset")
    await until(lambda: (esp.value("textsensor", "MCU Reset Cause") or "").startswith("Power-on"), 2)

    mcu.log()
    mcu.cmd("wdtreset")
    rig.mcu_until(firing_at(expected_target(0.6)), 3)
    # The MCU starts with its outputs off; the bar is sent again.
    rig.mcu_until(bar_shows(3), 1)
    assert any(e.split()[1:3] == ["outputs", str(bar(3))] for e in mcu.log())
    await until(lambda: (esp.value("textsensor", "MCU Reset Cause") or "").startswith("Watchdog"), 2, "watchdog")
    assert esp.lamp()[0]

    # A reset that cuts a status frame off, whose claimed bytes take in the new application's info
    # frame: that info frame is still read, so the outputs are restored before the MCU sends its next
    # one. The restart is counted once, and the start-up FAULT is not a fault event.
    await until(lambda: esp.mcu_restarts() == mcu.get()["app_starts"] - 1, 2, "restart count logged")
    restarts = esp.mcu_restarts()
    mcu.log()
    mcu.cmd("wdtreset cut")
    log: list[list[str]] = []

    def outputs_restored(_: dict) -> bool:
        log.extend(e.split() for e in mcu.log())
        return any(e[1:3] == ["outputs", str(bar(3))] for e in log)

    rig.mcu_until(outputs_restored, 1 + FW["INFO_PERIOD_MS"] / 1000)
    started = next(int(e[0]) for e in log if e[1:3] == ["app", "start"])
    restored = next(int(e[0]) for e in log if e[1:3] == ["outputs", str(bar(3))])
    assert restored - started < FW["INFO_PERIOD_MS"], "outputs held back: the info frame inside the cut frame was lost"
    rig.mcu_until(firing_at(expected_target(0.6)), 3)
    assert esp.lamp()[0], "light turned off after an MCU reset that cut a status frame off"
    await until(lambda: esp.mcu_restarts() == restarts + 1, 2, "restart counted")
    await until(
        lambda: (esp.value("textsensor", "MCU Reset Cause") or "").startswith("Watchdog, watchdog count 2"),
        2,
        "second watchdog reset",
    )
    assert esp.mcu_restarts() == restarts + 1


async def test_esp_restart_soft_reboot_adopts_mcu(rig: Rig) -> None:
    """Soft reboot (forced on host with RPSTD01_SOFT_REBOOT): the lamp is not touched and the
    light shows the MCU's brightness, whatever ESPHome restored."""
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    await esp.light(True, 0.6)
    target = expected_target(0.6)
    rig.mcu_until(firing_at(target))
    await esp.light(True, 0.2)  # saved to prefs on the graceful stop below
    rig.mcu_until(firing_at(expected_target(0.2)))
    await esp.stop()
    # change the lamp behind ESPHome's back: as if the restored state were a minute old
    mcu.cmd("clear")
    peer_target = expected_target(0.6)
    from test_fake_mcu import Peer, gate  # noqa: PLC0415

    p = Peer()
    gate(p, peer_target)
    p.close()
    rig.mcu_until(firing_at(peer_target))
    mcu.cmd("clear")
    await esp.start({"RPSTD01_SOFT_REBOOT": "1"})
    rig.mcu_until(gate_received, 3)
    await until(lambda: esp.lamp() == (True, pytest.approx(0.6, abs=0.01)), 2, "light on at the MCU's level")
    s = mcu.get()
    assert abs(s["target"] - peer_target) <= 3 and s["state"] == ST_FIRING
    targets = [int(e.split()[2]) for e in mcu.log() if e.split()[1] == "gate"]
    assert all(abs(t - peer_target) <= 3 for t in targets), targets

    # MCU off while the ESP is away: after a soft reboot the light is off
    await esp.stop()
    p = Peer()
    gate(p, 0)
    p.close()
    rig.mcu_until(lambda s: s["state"] == ST_OFF)
    mcu.cmd("clear")
    await esp.start({"RPSTD01_SOFT_REBOOT": "1"})
    rig.mcu_until(gate_received, 3)
    await until(lambda: esp.lamp()[0] is False, 2, "light off")
    assert mcu.get()["state"] == ST_OFF


async def test_esp_restart_power_on(rig: Rig) -> None:
    """On host every start counts as a power-on: the MCU keeps firing while ESPHome is away, and
    after the start the light's restore_mode applies (the state saved at the graceful stop)."""
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    await esp.light(True, 0.4)
    rig.mcu_until(firing_at(expected_target(0.4)))
    await esp.stop()
    await until(mcu.statuses_sent(1.0), 5, "statuses")
    assert firing_at(expected_target(0.4))(mcu.get())  # no link: the MCU keeps its state
    mcu.cmd("clear")
    await esp.start()
    rig.mcu_until(gate_received, 3)
    await until(lambda: esp.lamp() == (True, pytest.approx(0.4, abs=0.01)), 2, "restored level")
    rig.mcu_until(firing_at(expected_target(0.4)), 3)


async def test_power_on_default_on(rig: Rig) -> None:
    """Fresh preferences: RESTORE_DEFAULT_ON turns the lamp on once the MCU answers."""
    await rig.start(lamp_off=False)
    rig.mcu_until(lambda s: s["state"] == ST_FIRING, 4)
    assert rig.esp.lamp()[0]


async def test_over_temperature(rig: Rig) -> None:
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    await esp.light(True, 0.7)
    rig.mcu_until(firing_at(expected_target(0.7)))
    mcu.cmd(f"therm {HOT}")
    rig.mcu_until(lambda s: s["state"] == ST_FAULT and s["relay"] == 0, 2)
    await until(lambda: esp.lamp()[0] is False, 2, "light off")
    await until(lambda: esp.value("binarysensor", "MCU Problem") is True, 2, "problem")
    await until(lambda: "over-temperature" in (esp.value("textsensor", "MCU Faults") or ""), 2)
    await until(lambda: esp.value("textsensor", "MCU State") == "Fault", 2)
    assert mcu.get()["leds"] & LED_B3
    await until(lambda: mcu.get()["night"] > 0, 2, "night light on")  # On When Lamp Off
    mcu.cmd(f"therm {COOL}")
    rig.mcu_until(lambda s: s["state"] == ST_OFF, 3)
    await until(lambda: esp.value("textsensor", "MCU Faults") == "none", 2, "faults clear")
    await until(lambda: esp.value("binarysensor", "MCU Problem") is False, 2)
    await until(mcu.statuses_sent(1.0), 5, "statuses")
    assert esp.lamp()[0] is False and mcu.get()["state"] == ST_OFF  # does not come back by itself
    # Turned on during a FAULT: the light shows on, the lamp stays dark until the FAULT clears.
    mcu.cmd(f"therm {HOT}")
    rig.mcu_until(lambda s: s["state"] == ST_FAULT, 2)
    await until(lambda: esp.value("binarysensor", "MCU Problem") is True, 2, "problem")
    await esp.light(True, 0.5)
    await until(mcu.statuses_sent(1.0), 5, "statuses")
    assert esp.lamp()[0]
    s = mcu.get()
    assert s["state"] == ST_FAULT and s["target"] == 0 and s["leds"] & LED_B3
    mcu.cmd(f"therm {COOL}")
    rig.mcu_until(firing_at(expected_target(0.5)), 3)
    assert esp.lamp() == (True, pytest.approx(0.5, abs=0.01))


async def test_onoff_single_and_double_are_events_only(rig: Rig) -> None:
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    mcu.cmd("press 0 120")
    await until(lambda: ("ON-OFF Button", "single") in esp.events, 2, "ON-OFF Button single event")
    await past_gesture_cooldown()
    mcu.cmd("press 0 100")
    await asyncio.sleep(0.2)
    mcu.cmd("press 0 100")
    await until(lambda: ("ON-OFF Button", "double") in esp.events, 2, "ON-OFF Button double event")
    await past_gesture_cooldown()
    assert not esp.lamp()[0]


async def test_up_press_turns_on_at_last_brightness_at_once(rig: Rig) -> None:
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    await esp.light(True, 0.6)
    await esp.light(False)
    mcu.cmd("press 1 120")
    await until(lambda: esp.lamp() == (True, pytest.approx(0.6, abs=0.01)), 2, "on after UP press")
    assert ("UP Button", "single") not in esp.events, "waited for the gesture instead of acting on the press"
    await until(lambda: ("UP Button", "single") in esp.events, 2, "UP Button single event")
    await past_gesture_cooldown()
    mcu.cmd("press 1 120")
    await until(lambda: esp.events.count(("UP Button", "single")) == 2, 2, "second UP single")
    b = esp.lamp()[1]
    assert esp.lamp()[0] and 0.6 < b < 0.8, f"a tap while on should brighten a little and stay: {b}"


async def test_down_dims_from_the_press_and_a_tap_turns_off(rig: Rig) -> None:
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    await esp.light(True, 0.6)
    rig.mcu_until(firing_at(expected_target(0.6)))
    mcu.cmd("press 2 600")
    await until(lambda: esp.lamp()[1] == pytest.approx(0.01), 2, "dimming toward the low end")
    assert held(2)(mcu.get()), "dimming waited for the release"
    rig.mcu_until(lambda s: not held(2)(s), 3)
    await until(lambda: esp.lamp()[1] > 0.01, 2, "held where released")
    b = esp.lamp()[1]
    assert esp.lamp()[0] and 0.1 < b < 0.55, f"a press past the tap time should dim, never turn off: {b}"
    await asyncio.sleep(0.5)
    assert esp.lamp() == (True, pytest.approx(b, abs=0.01)), "the release changed the lamp"
    await past_gesture_cooldown()
    singles = esp.events.count(("DOWN Button", "single"))
    mcu.cmd("press 2 120")
    await until(lambda: not esp.lamp()[0], 2, "off after DOWN tap")
    assert esp.events.count(("DOWN Button", "single")) == singles, "waited for the gesture instead of the release"
    await past_gesture_cooldown()
    mcu.cmd("press 1 120")
    await until(lambda: esp.lamp()[0], 2, "on after UP tap")
    assert esp.lamp()[1] == pytest.approx(b, abs=0.01), "a DOWN tap kept the low end it was dimming toward"


async def test_down_double_while_off_is_an_event_only(rig: Rig) -> None:
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    mcu.cmd("press 2 100")
    await asyncio.sleep(0.2)
    mcu.cmd("press 2 100")
    await until(lambda: ("DOWN Button", "double") in esp.events, 2, "DOWN Button double event")
    assert not esp.lamp()[0]


async def test_up_hold_from_off_brightens_from_last_brightness_until_release(rig: Rig) -> None:
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    await esp.light(True, 0.3)
    await esp.light(False)
    mcu.cmd("press 1 900")
    rig.mcu_until(held(1), 1)
    rig.mcu_until(lambda s: not held(1)(s), 3)
    await until(lambda: ("UP Button", "hold") in esp.events, 2, "UP Button hold event")
    await until(lambda: esp.lamp()[1] < 1.0, 2, "held where released")
    b = esp.lamp()[1]
    assert esp.lamp()[0] and 0.4 < b < 0.8, b
    await asyncio.sleep(0.5)
    assert esp.lamp() == (True, pytest.approx(b, abs=0.01)), "still brightening after the release"


async def test_onoff_hold_toggles_night_light(rig: Rig) -> None:
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    night = esp.state("light", "Night Light").state
    mcu.cmd("press 0 1000")
    await until(lambda: esp.state("light", "Night Light").state != night, 2, "night light toggled")
    await until(lambda: ("ON-OFF Button", "hold") in esp.events, 2, "ON-OFF Button hold event")


async def test_bounce_in_one_status_window_is_not_a_press(rig: Rig) -> None:
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    events = len(esp.events)
    mcu.cmd("press 2 3")
    await asyncio.sleep(0.8)
    assert len(esp.events) == events


async def test_gestures_in_one_loop(rig: Rig) -> None:
    """A press and its release that reach ESPHome together, as after a stall (the fake MCU holds back
    its statuses and then sends them at once), still make single, double and hold."""
    await rig.start()
    esp, mcu = rig.esp, rig.mcu

    def click() -> None:
        mcu.cmd("txhold 150")
        mcu.cmd("press 0 50")

    click()
    await until(lambda: ("ON-OFF Button", "single") in esp.events, 2, "single")
    await past_gesture_cooldown()
    events = len(esp.events)
    click()
    await asyncio.sleep(0.25)
    click()
    await until(lambda: ("ON-OFF Button", "double") in esp.events[events:], 2, "double")
    assert ("ON-OFF Button", "single") not in esp.events[events:]
    await past_gesture_cooldown()
    events = len(esp.events)
    mcu.cmd("txhold 150")
    mcu.cmd("press 0 1000")
    await until(lambda: ("ON-OFF Button", "hold") in esp.events[events:], 2, "hold")
    await asyncio.sleep(0.5)
    assert esp.events[events:] == [("ON-OFF Button", "hold")]


async def test_addon(rig: Rig) -> None:
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    await asyncio.sleep(0.5)
    # add-on UP single with the lamp off: on
    mcu.cmd("addon up")
    await asyncio.sleep(0.12)
    mcu.cmd("addon idle")
    await until(lambda: esp.lamp()[0], 2, "on")
    await until(lambda: ("Add-on UP", "single") in esp.events, 2, "Add-on UP single event")
    # add-on DOWN single with the lamp lit (relay drive on, its own Ks levels): off
    rig.mcu_until(lambda s: s["relay"] == 1, 3)
    await past_gesture_cooldown()
    mcu.cmd("addon down")
    await asyncio.sleep(0.12)
    mcu.cmd("addon idle")
    await until(lambda: not esp.lamp()[0], 2, "off")
    # add-on DOWN with the lamp off: the event fires, the light stays off
    rig.mcu_until(lambda s: s["relay"] == 0, 3)
    await past_gesture_cooldown()
    events = len(esp.events)
    mcu.cmd("addon down")
    await asyncio.sleep(0.12)
    mcu.cmd("addon idle")
    await until(lambda: len(esp.events) > events, 2, "add-on DOWN single")
    assert esp.events[-1] == ("Add-on DOWN", "single")
    assert not esp.lamp()[0]


async def test_update_available(rig: Rig, app_image: bytes) -> None:
    await rig.start(image_bytes=image_with_build(app_image, 0x12345678))
    esp = rig.esp
    await until(lambda: esp.value("binarysensor", "MCU Update Available") is True, 3, "update available")
    state = "Off (update available: use the MCU tool)"
    await until(lambda: esp.value("textsensor", "MCU State") == state, 3, "MCU State")
    assert esp.value("binarysensor", "MCU Problem") is False
    await esp.light(True, 0.5)
    rig.mcu_until(firing_at(expected_target(0.5)))  # the lamp still works
    log = (rig.workdir / f"esp{esp.runs}.log").read_text(errors="replace")
    assert "update it with the MCU tool" in log


async def test_other_protocol_mcu_needs_tool(rig: Rig, app_image: bytes) -> None:
    """An MCU whose info frame shows another link protocol: MCU Problem, MCU State names the tool,
    and nothing at all is sent to it."""
    await rig.start(lamp_off=False, image_bytes=image_with_version(image_with_build(app_image, 0x12345678), 0x0200))
    esp, mcu = rig.esp, rig.mcu
    await until(lambda: esp.value("textsensor", "MCU State") == "MCU firmware needs the MCU tool", 3, "MCU State")
    assert esp.value("binarysensor", "MCU Problem") is True
    mcu.cmd("clear")
    await esp.light(True, 0.5)
    await until(mcu.statuses_sent(2.0), 8, "statuses")
    s = mcu.get()
    assert s["rx"] == {} and s["state"] == ST_OFF


async def test_longer_frames_need_tool(rig: Rig) -> None:
    """An MCU whose frames are all longer (a later release): MCU State names the tool, and nothing
    is sent to it."""
    await rig.start(lamp_off=False, extra=("--pad", "4"))
    esp, mcu = rig.esp, rig.mcu
    await until(lambda: esp.value("textsensor", "MCU State") == "MCU firmware needs the MCU tool", 3, "MCU State")
    assert esp.value("binarysensor", "MCU Problem") is True
    mcu.cmd("clear")
    await esp.light(True, 0.5)
    await until(mcu.statuses_sent(2.0), 8, "statuses")
    assert mcu.get()["rx"] == {}


async def test_silent_in_update_mode(rig: Rig) -> None:
    """While the MCU is in update mode the main configuration sends it nothing, so the bootloader
    returns to the application after its idle time; the lamp then comes back."""
    await rig.start()
    esp, mcu = rig.esp, rig.mcu
    await esp.light(True, 0.5)
    rig.mcu_until(firing_at(expected_target(0.5)))
    idle_return_s = 8
    mcu.cmd(f"idle_return_ms {idle_return_s * 1000}")
    mcu.cmd("updatemode")
    rig.mcu_until(lambda s: s["mode"] == "boot" and s["boot_reason"] == FW["BOOT_REASON_REQUEST"], 2)
    mcu.cmd("clear")
    await until(lambda: esp.value("textsensor", "MCU State") == "Update mode (use the MCU tool)", 3, "update mode")
    assert esp.value("binarysensor", "MCU Problem") is True
    await esp.light(True, 0.7)  # kept for later, not sent

    def back_in_app(s: dict) -> bool:
        assert s["mode"] == "app" or s["rx"] == {}, s["rx"]
        return s["mode"] == "app"

    rig.mcu_until(back_in_app, idle_return_s + 2)
    rig.mcu_until(firing_at(expected_target(0.7)), 3)
    await until(lambda: esp.value("binarysensor", "MCU Problem") is False, 3, "problem off")
    assert esp.lamp()[0], "light turned off after update mode (start-up FAULT taken as a fault event?)"
    log = (rig.workdir / f"esp{esp.runs}.log").read_text(errors="replace")
    assert "MCU in update mode: nothing is sent to it" in log


async def test_select_without_the_option_acted_on_is_rejected(tmp_path: Path) -> None:
    out = config_rejected(
        [
            "packages:",
            f"  host: !include {TESTS / 'host.yaml'}",
            "select:",
            "  - id: !remove brightness_bar",
            "  - platform: template",
            "    id: brightness_bar",
            "    name: Brightness Bar",
            "    options: [Shown, 'Off']",
            "    optimistic: true",
        ],
        tmp_path,
    )
    assert "select 'brightness_bar' needs the option 'Level'" in out
