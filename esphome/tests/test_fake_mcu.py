"""Self-tests of the fake MCU, driven over the pty by a small Python link (no ESPHome)."""

from __future__ import annotations

import os
import select
import struct
import time

import pytest
from conftest import PTY_LINK
from mcu_proto import (
    APP_SIZE,
    BOOT_VERSION,
    FW,
    MSG_BOOT_BEGIN,
    MSG_BOOT_FINISH,
    MSG_BOOT_RUN,
    MSG_BOOT_STATUS,
    MSG_BOOT_WRITE,
    MSG_GATE,
    MSG_INFO,
    MSG_INFO_REQ,
    MSG_OUTPUTS,
    MSG_STATUS,
    MSG_UPDATE,
    ST_FAULT,
    ST_FIRING,
    ST_OFF,
    ST_SETTLE,
    ST_STOPPING,
    UPDATE_KEY,
    FrameParser,
    build_frame,
    half_cycle_counts,
    image_with_build,
    open_serial,
    parse_trailer,
)

S_RELAY, S_STARTING = FW["S_RELAY"], FW["S_STARTING"]
F_OVERTEMP, F_THERM, F_ZO = FW["F_OVERTEMP"], FW["F_THERM"], FW["F_ZO"]
HOT, COOL = FW["THERM_TRIP"], FW["THERM_CLEAR"]
WRITE_CHUNK = FW["WRITE_CHUNK"]


class Peer:
    def __init__(self) -> None:
        self.fd = open_serial(PTY_LINK)
        self.parser = FrameParser()

    def send(self, msg_type: int, payload: bytes = b"") -> None:
        os.write(self.fd, build_frame(msg_type, payload))

    def frames(self, seconds: float) -> list[tuple[int, bytes]]:
        out = []
        end = time.monotonic() + seconds
        while (left := end - time.monotonic()) > 0:
            if select.select([self.fd], [], [], left)[0]:
                out += self.parser.feed(os.read(self.fd, 4096))
        return out

    def wait(self, msg_type: int, timeout: float = 2.0, pred=lambda p: True) -> bytes:
        end = time.monotonic() + timeout
        while (left := end - time.monotonic()) > 0:
            if select.select([self.fd], [], [], left)[0]:
                for t, p in self.parser.feed(os.read(self.fd, 4096)):
                    if t == msg_type and pred(p):
                        return p
        raise AssertionError(f"no frame {msg_type:#x} in {timeout} s")

    def close(self) -> None:
        os.close(self.fd)


@pytest.fixture
def peer():
    p = None

    def _open() -> Peer:
        nonlocal p
        p = Peer()
        return p

    yield _open
    if p:
        p.close()


def status(p: bytes) -> dict:
    (state, faults, buttons, target, current, half, ks, flags, therm, wheel, boot_id, leds, night) = struct.unpack(
        "<BBBHHHHBHHBBB", p
    )
    return dict(state=state, faults=faults, buttons=buttons, target=target, current=current, half=half, ks=ks,
                relay=bool(flags & S_RELAY), therm=therm, wheel=wheel, starting=bool(flags & S_STARTING),
                boot_id=boot_id, leds=leds, night=night)


def info_frame(p: bytes) -> dict:
    version, build, reset_cause, wdt_count, boot_version, boot_id = struct.unpack("<HIBBBB", p)
    return dict(version=version, build=build, reset_cause=reset_cause, wdt_count=wdt_count,
                boot_version=boot_version, boot_id=boot_id)


def boot_status(p: bytes) -> dict:
    boot_version, app_valid, reason, result, next_offset = struct.unpack("<BBBBH", p)
    return dict(boot_version=boot_version, app_valid=bool(app_valid), reason=reason, result=result,
                next=next_offset)


def gate(peer: Peer, target: int, limit: int = 48) -> None:
    peer.send(MSG_GATE, struct.pack("<HBB", target, limit, limit))


def request_update(peer: Peer) -> None:
    peer.send(MSG_UPDATE, UPDATE_KEY)


def test_status_and_info(make_mcu, peer, app_image: bytes) -> None:
    mcu = make_mcu()
    p = peer()
    p.send(MSG_INFO_REQ)
    info = info_frame(p.wait(MSG_INFO))
    assert info["version"] == parse_trailer(app_image).version
    assert info["reset_cause"] == FW["RESET_POWER_ON"] and info["wdt_count"] == 0
    assert info["boot_version"] == BOOT_VERSION == mcu.get()["boot_version"]
    assert f"{info['build']:08X}" == mcu.get()["build"]
    assert info["boot_id"] == mcu.get()["boot_id"]
    frames = [f for f in p.frames(1.0) if f[0] == MSG_STATUS]
    assert 90 <= len(frames) <= 102
    s = status(frames[-1][1])
    assert s["boot_id"] == info["boot_id"] and not s["starting"]
    sim = mcu.get()
    assert s["half"] == half_cycle_counts(60) and s["therm"] == sim["therm_raw"] and s["wheel"] == sim["wheel_raw"]
    assert s["state"] == ST_OFF
    assert s["ks"] == 556 and s["buttons"] & FW["BTN_ALL"] == FW["BTN_ALL"]


def test_boot_id_and_info_every_second(make_mcu, peer) -> None:
    """The info frame every second with the boot ID, which changes at every start."""
    mcu = make_mcu()
    p = peer()
    frames = p.frames(3.2)
    infos = [f[1] for f in frames if f[0] == MSG_INFO]
    assert 3 <= len(infos) <= 4 and all(len(i) == FW["INFO_LEN"] for i in infos)
    boot_id = mcu.get()["boot_id"]
    assert {info_frame(i)["boot_id"] for i in infos} == {boot_id}
    assert {status(f[1])["boot_id"] for f in frames if f[0] == MSG_STATUS} == {boot_id}
    # A power-on leaves whatever the RAM holds (the fake: another value each time); other resets
    # keep the byte, and the application raises it by 1.
    for cmd in ("wdtreset", "powercut 50", "updatemode"):
        mcu.cmd(cmd)
        if cmd == "updatemode":
            p.wait(MSG_BOOT_STATUS)
            p.send(MSG_BOOT_RUN)  # the bootloader keeps the boot ID; the application raises it
        new_boot_id = info_frame(p.wait(MSG_INFO, timeout=3))["boot_id"]
        if cmd.startswith("powercut"):
            assert new_boot_id != boot_id
        else:
            assert new_boot_id == (boot_id + 1) & 0xFF, cmd
        s = status(p.wait(MSG_STATUS))
        assert s["boot_id"] == new_boot_id
        boot_id = new_boot_id


def test_starting_flag(make_mcu, peer) -> None:
    """The start-up FAULT comes with the starting flag; once it clears, a FAULT shows without it."""
    mcu = make_mcu()
    p = peer()
    mcu.cmd("powercut 50")
    s = status(p.wait(MSG_STATUS, timeout=1))
    assert s["state"] == ST_FAULT and s["faults"] & F_THERM and s["starting"]
    s = status(p.wait(MSG_STATUS, pred=lambda x: not status(x)["starting"], timeout=1))
    assert s["state"] == ST_OFF
    mcu.cmd("therm 10")
    s = status(p.wait(MSG_STATUS, pred=lambda x: status(x)["state"] == ST_FAULT, timeout=1))
    assert s["faults"] & F_THERM and not s["starting"]


def test_outputs_echo(make_mcu, peer) -> None:
    """Every status shows the outputs in use; they start off."""
    mcu = make_mcu()
    p = peer()
    s = status(p.wait(MSG_STATUS))
    assert s["leds"] == 0 and s["night"] == 0
    p.send(MSG_OUTPUTS, bytes([0x2A, 99]))
    p.wait(MSG_STATUS, pred=lambda x: status(x)["leds"] == 0x2A and status(x)["night"] == 99)
    mcu.cmd("wdtreset")
    p.wait(MSG_STATUS, pred=lambda x: status(x)["leds"] == 0 and status(x)["night"] == 0)


def test_startup_fault(make_mcu, peer) -> None:
    """The real firmware reports FAULT (thermistor not ready) for a short while after any reset."""
    mcu = make_mcu()
    p = peer()
    mcu.cmd("powercut 50")
    s = status(p.wait(MSG_STATUS, timeout=1))
    assert s["state"] == ST_FAULT and s["faults"] & F_THERM
    p.wait(MSG_STATUS, pred=lambda x: status(x)["state"] == ST_OFF, timeout=1)


def test_relay_sequence(make_mcu, peer) -> None:
    mcu = make_mcu()
    p = peer()
    p.frames(0.3)
    mcu.log()
    gate(p, 4000)
    p.wait(MSG_STATUS, pred=lambda x: status(x)["state"] == ST_FIRING and status(x)["current"] == 4000)
    gate(p, 2000, 48)
    s = status(p.wait(MSG_STATUS, pred=lambda x: status(x)["current"] == 2000))
    assert s["relay"]
    gate(p, 0)
    p.wait(MSG_STATUS, pred=lambda x: status(x)["state"] == ST_OFF and not status(x)["relay"])
    log = mcu.log()
    events = [e.split(" ", 1) for e in log]
    names = [e[1].split()[0] + " " + e[1].split()[1] for e in events if e[1].startswith(("state", "relay"))]
    assert names == [f"state {ST_SETTLE}", "relay 1", f"state {ST_FIRING}", f"state {ST_STOPPING}",
                     f"state {ST_OFF}", "relay 0"]
    t = {n: int(e[0]) for n, e in zip(names, [e for e in events if e[1].startswith(("state", "relay"))])}
    assert t[f"state {ST_FIRING}"] - t[f"state {ST_SETTLE}"] >= FW["RELAY_SETTLE_MS"]


def test_buttons_ks_faults(make_mcu, peer) -> None:
    mcu = make_mcu()
    p = peer()
    p.frames(0.3)
    mcu.cmd("press 1 100")
    s = status(p.wait(MSG_STATUS, pred=lambda x: not status(x)["buttons"] & FW["BTN_UP"]))
    mcu.cmd("ks 600")
    p.wait(MSG_STATUS, pred=lambda x: status(x)["ks"] == 600)
    gate(p, 4000)
    p.wait(MSG_STATUS, pred=lambda x: status(x)["state"] == ST_FIRING)
    mcu.cmd(f"therm {HOT}")
    s = status(p.wait(MSG_STATUS, pred=lambda x: status(x)["state"] == ST_FAULT, timeout=3))
    assert s["faults"] & F_OVERTEMP
    # The relay opens after the half-cycle that may still conduct.
    p.wait(MSG_STATUS, pred=lambda x: status(x)["state"] == ST_FAULT and not status(x)["relay"], timeout=0.2)
    mcu.cmd(f"therm {COOL}")
    s = status(p.wait(MSG_STATUS, pred=lambda x: status(x)["therm"] >= FW["THERM_CLEAR"]))
    assert s["state"] == ST_FAULT  # cool again, but it needs a target of 0 first
    gate(p, 0)
    p.wait(MSG_STATUS, pred=lambda x: status(x)["state"] == ST_OFF, timeout=3)
    mcu.cmd("zo off")
    s = status(p.wait(MSG_STATUS, pred=lambda x: status(x)["half"] == 0))
    assert s["faults"] & F_ZO


def test_drop_and_outputs(make_mcu, peer) -> None:
    mcu = make_mcu()
    p = peer()
    mcu.cmd(f"drop rx 1 {MSG_GATE:#x}")
    gate(p, 3000)
    assert mcu.wait_for(lambda s: s["rx_dropped"] == 1)["target"] == 0
    gate(p, 3000)
    mcu.wait_for(lambda s: s["target"] == 3000)
    p.send(MSG_OUTPUTS, bytes([0x21, 77]))
    s = mcu.wait_for(lambda s: s["leds"] == 0x21)
    assert s["night"] == 77 and s["rx"][MSG_GATE] == 2


def update(p: Peer, image: bytes) -> None:
    """The update as the ESP32 does it, one frame per boot status."""
    gate(p, 0)
    request_update(p)
    p.wait(MSG_BOOT_STATUS, timeout=3)
    p.send(MSG_BOOT_BEGIN, struct.pack("<H", APP_SIZE))
    b = boot_status(p.wait(MSG_BOOT_STATUS, timeout=3))
    assert b["result"] == FW["BR_OK"] and not b["app_valid"]
    off = 0
    while off < APP_SIZE:
        p.send(MSG_BOOT_WRITE, struct.pack("<H", off) + image[off : off + WRITE_CHUNK])
        b = boot_status(p.wait(MSG_BOOT_STATUS, timeout=1))
        assert b["result"] == FW["BR_OK"], b
        off = b["next"]
    p.send(MSG_BOOT_FINISH)
    b = boot_status(p.wait(MSG_BOOT_STATUS, timeout=2))
    assert b["result"] == FW["BR_OK"] and b["app_valid"]
    p.send(MSG_BOOT_RUN)


def test_full_update(make_mcu, peer, app_image) -> None:
    mcu = make_mcu(image_bytes=image_with_build(app_image, 0x12345678))
    p = peer()
    assert mcu.get()["build"] == "12345678"
    t0 = time.monotonic()
    update(p, app_image)
    info = info_frame(p.wait(MSG_INFO, timeout=3))
    assert (info["build"], info["reset_cause"]) == (parse_trailer(app_image).build, FW["RESET_OTHER"])
    assert time.monotonic() - t0 < 6
    assert mcu.get()["mode"] == "app"


def test_powercut_mid_update_and_aux_ram_lost(make_mcu, peer, app_image) -> None:
    mcu = make_mcu()
    p = peer()
    request_update(p)
    p.wait(MSG_BOOT_STATUS)
    p.send(MSG_BOOT_BEGIN, struct.pack("<H", APP_SIZE))
    p.wait(MSG_BOOT_STATUS, timeout=3)
    p.send(MSG_BOOT_WRITE, struct.pack("<H", 0) + app_image[:WRITE_CHUNK])
    p.wait(MSG_BOOT_STATUS)
    mcu.cmd("powercut 200")
    b = boot_status(p.wait(MSG_BOOT_STATUS, timeout=3))
    assert not b["app_valid"] and b["reason"] == FW["BOOT_REASON_INVALID"]
    assert mcu.get()["mode"] == "boot"
    # a repair, with a lost reply on the way
    mcu.cmd(f"drop tx 1 {MSG_BOOT_STATUS:#x}")
    p.send(MSG_BOOT_BEGIN, struct.pack("<H", APP_SIZE))
    mcu.wait_for(lambda s: s["receiving"] == 1)
    p.frames(0.05)
    p.wait(MSG_BOOT_STATUS)  # the next periodic one; send just after it
    p.send(MSG_BOOT_BEGIN, struct.pack("<H", APP_SIZE))
    p.wait(MSG_BOOT_STATUS, timeout=3)
    for off in range(0, APP_SIZE, WRITE_CHUNK):
        p.send(MSG_BOOT_WRITE, struct.pack("<H", off) + app_image[off : off + WRITE_CHUNK])
        assert boot_status(p.wait(MSG_BOOT_STATUS))["result"] == FW["BR_OK"]
    p.send(MSG_BOOT_FINISH)
    assert boot_status(p.wait(MSG_BOOT_STATUS, timeout=2))["app_valid"]
    p.send(MSG_BOOT_RUN)
    p.wait(MSG_INFO, timeout=3)
    # the update request comes back as an application restart
    mcu.cmd("auxram false")
    request_update(p)
    assert info_frame(p.wait(MSG_INFO, timeout=3))["reset_cause"] == FW["RESET_OTHER"]
    assert mcu.get()["mode"] == "app"


def test_tampered_write_fails_finish(make_mcu, peer, app_image) -> None:
    mcu = make_mcu()
    p = peer()
    mcu.cmd(f"tamper rx 1 {MSG_BOOT_WRITE:#x}")
    request_update(p)
    p.wait(MSG_BOOT_STATUS)
    p.send(MSG_BOOT_BEGIN, struct.pack("<H", APP_SIZE))
    p.wait(MSG_BOOT_STATUS, timeout=3)
    for off in range(0, APP_SIZE, WRITE_CHUNK):
        p.send(MSG_BOOT_WRITE, struct.pack("<H", off) + app_image[off : off + WRITE_CHUNK])
        p.wait(MSG_BOOT_STATUS)
    p.send(MSG_BOOT_FINISH)
    b = boot_status(p.wait(MSG_BOOT_STATUS, timeout=2))
    assert b["result"] == FW["BR_BAD_IMAGE"] and not b["app_valid"]
    assert mcu.get()["flash_valid"] == 0


def test_boot_idle_return(make_mcu, peer) -> None:
    mcu = make_mcu(extra=("--idle-return-ms", "1500"))
    p = peer()
    request_update(p)
    assert boot_status(p.wait(MSG_BOOT_STATUS))["result"] == FW["BR_NO_COMMAND"]
    # idle return to the application after the shortened idle time
    assert info_frame(p.wait(MSG_INFO, timeout=3))["reset_cause"] == FW["RESET_OTHER"]
    assert mcu.get()["mode"] == "app"
