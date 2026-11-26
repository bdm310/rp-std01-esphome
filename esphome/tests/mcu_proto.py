"""Helpers for the host tests: the firmware's constants and image, link frames, a client for the
fake MCU's control socket, and a process wrapper that starts the fake MCU."""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "firmware" / "tools"))

import mkesphome  # noqa: E402

# Process groups of the programs the tests started (fake MCUs, ESPHome host binaries), each in a
# session of its own, so that whatever a failed test left running can be killed.
LIVE_GROUPS: set[int] = set()


def kill_group(pid: int) -> None:
    """Kills a process started with start_new_session=True, and anything it started."""
    try:
        os.killpg(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    LIVE_GROUPS.discard(pid)


_prctl = ctypes.CDLL(None).prctl
PR_SET_PDEATHSIG = 1


def die_with_parent() -> None:
    """For preexec_fn: the child gets SIGKILL when the test process ends, even if it is killed."""
    _prctl(PR_SET_PDEATHSIG, signal.SIGKILL)


FW = mkesphome.constants()
FRAME_START, BOOT_VERSION, APP_SIZE = FW["FRAME_START"], FW["BOOT_VERSION"], FW["APP_SIZE"]
MSG_GATE, MSG_OUTPUTS, MSG_INFO_REQ, MSG_UPDATE = FW["MSG_GATE"], FW["MSG_OUTPUTS"], FW["MSG_INFO_REQ"], FW["MSG_UPDATE"]
MSG_STATUS, MSG_INFO, MSG_BOOT_STATUS = FW["MSG_STATUS"], FW["MSG_INFO"], FW["MSG_BOOT_STATUS"]
MSG_BOOT_BEGIN, MSG_BOOT_WRITE = FW["MSG_BOOT_BEGIN"], FW["MSG_BOOT_WRITE"]
MSG_BOOT_FINISH, MSG_BOOT_RUN = FW["MSG_BOOT_FINISH"], FW["MSG_BOOT_RUN"]
ST_OFF, ST_SETTLE, ST_FIRING, ST_STOPPING, ST_FAULT = (
    FW["ST_OFF"], FW["ST_SETTLE"], FW["ST_FIRING"], FW["ST_STOPPING"], FW["ST_FAULT"]
)
UPDATE_KEY = bytes(FW[f"UPDATE_KEY{i}"] for i in range(FW["UPDATE_LEN"]))
crc16 = mkesphome.crc16
parse_trailer = mkesphome.parse_trailer


def half_cycle_counts(mains_hz: int) -> int:
    """A mains half-cycle in the MCU's PCA counts."""
    return FW["PCA_HZ"] // (2 * mains_hz)


def committed_app_image() -> bytes:
    """The application area of the committed MCU image, as the tool sends it (valid mark 0xFF)."""
    return mkesphome.app_area(mkesphome.read_image_header())


def build_frame(msg_type: int, payload: bytes = b"") -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    crc = crc16(body)
    return bytes([FRAME_START]) + body + bytes([crc & 0xFF, crc >> 8])


class FrameParser:
    """Splits bytes into (type, payload) frames by the rules of firmware/common/link.c: after a bad
    frame it looks for the next start byte from the byte that ended the bad frame on."""

    def __init__(self, max_len: int = FW["BOOT_MAX_PAYLOAD"]) -> None:
        self.max_len = max_len
        self.frame: bytearray | None = None  # type, length, payload and CRC received so far

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        out = []
        for b in data:
            if self.frame is None:
                if b == FRAME_START:
                    self.frame = bytearray()
                continue
            f = self.frame
            f.append(b)
            if len(f) == 2 and f[1] > self.max_len:
                good = False
            elif len(f) >= 2 and len(f) == f[1] + 4:
                good = crc16(f[:-2]) == f[-2] | f[-1] << 8
                if good:
                    out.append((f[0], bytes(f[2:-2])))
            else:
                continue
            self.frame = bytearray() if not good and b == FRAME_START else None
        return out


def _with_trailer_field(image: bytes, field: str, value: bytes) -> bytes:
    img = bytearray(image)
    off = APP_SIZE - FW["TRAILER_SIZE"] + FW[field]
    img[off : off + len(value)] = value
    return bytes(img)


def image_with_build(image: bytes, build: int) -> bytes:
    """Same image with another build ID in its trailer (the CRC covers only the code)."""
    return _with_trailer_field(image, "TR_BUILD", build.to_bytes(4, "little"))


def image_with_version(image: bytes, version: int) -> bytes:
    """Same image with another version in its trailer (the CRC covers only the code)."""
    return _with_trailer_field(image, "TR_VERSION", version.to_bytes(2, "little"))


class Control:
    """Line client for the fake MCU's control socket."""

    def __init__(self, path: str | Path) -> None:
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        path = str(path)
        self.sock.connect("\0" + path[1:] if path.startswith("@") else path)
        self.file = self.sock.makefile("rb")

    def cmd(self, line: str) -> str:
        self.sock.sendall(line.encode() + b"\n")
        reply = self.file.readline().decode().strip()
        if reply.startswith("err"):
            raise RuntimeError(f"fake MCU: {line!r}: {reply}")
        return reply

    def get(self) -> dict:
        """The fake MCU's state; "rx" and "tx" count the received and sent frames by message type."""
        state = json.loads(self.cmd("get"))
        for direction in ("rx", "tx"):
            state[direction] = {int(msg_type, 16): count for msg_type, count in state[direction].items()}
        return state

    def log(self) -> list[str]:
        return json.loads(self.cmd("log"))

    def close(self) -> None:
        self.file.close()
        self.sock.close()


@dataclass
class FakeMcu:
    """The fake MCU process. Its flash file survives restarts of the process."""

    binary: Path
    workdir: Path
    link: str
    image: Path | None = None
    invalid: bool = False
    extra: tuple[str, ...] = ()
    proc: subprocess.Popen | None = None
    ctl: Control | None = None

    @property
    def flash(self) -> Path:
        return self.workdir / "flash.bin"

    @property
    def ctl_path(self) -> str:
        # Abstract socket: tmp_path names are often too long for a unix socket path.
        return f"@rpstd01-fake-mcu-{os.getpid()}-{id(self)}"

    def start(self) -> None:
        args = [str(self.binary), "--flash", str(self.flash), "--link", self.link]
        args += ["--ctl", self.ctl_path]
        if self.image:
            args += ["--image", str(self.image)]
        if self.invalid:
            args.append("--invalid")
        args += list(self.extra)
        with open(self.workdir / "fake_mcu.log", "ab") as log:
            self.proc = subprocess.Popen(
                args, stdout=subprocess.PIPE, stderr=log, start_new_session=True, preexec_fn=die_with_parent
            )
        LIVE_GROUPS.add(self.proc.pid)
        for _ in range(2):
            line = self.proc.stdout.readline().decode()
            if not line:
                raise RuntimeError("fake MCU did not start; see fake_mcu.log")
        self.ctl = Control(self.ctl_path)

    def stop(self) -> None:
        if self.ctl:
            self.ctl.close()
            self.ctl = None
        if self.proc:
            self.proc.terminate()
            try:
                self.proc.wait(5)
            finally:
                kill_group(self.proc.pid)
                self.proc = None

    def cmd(self, line: str) -> str:
        return self.ctl.cmd(line)

    def get(self) -> dict:
        return self.ctl.get()

    def log(self) -> list[str]:
        return self.ctl.log()

    def wait_for(self, pred, timeout: float = 5.0, interval: float = 0.02) -> dict:
        end = time.monotonic() + timeout
        while True:
            state = self.get()
            if pred(state):
                return state
            if time.monotonic() > end:
                raise AssertionError(f"fake MCU condition not met in {timeout} s: {state}")
            time.sleep(interval)

    def statuses_sent(self, seconds: float):
        """A test that turns true once the fake MCU has sent the status frames of this many seconds
        more: the time in which the ESP would have acted on them."""
        count = round(seconds * 1000 / FW["STATUS_PERIOD_MS"])
        start = self.get()["tx"].get(MSG_STATUS, 0)
        return lambda: self.get()["tx"].get(MSG_STATUS, 0) >= start + count

    def wait_past_startup(self, timeout: float = 5.0) -> dict:
        """Until the application is past its start-up FAULT (thermistor average not ready)."""
        return self.wait_for(lambda s: s["mode"] == "app" and not s["startup"], timeout)


def open_serial(path: str):
    """Opens the pty slave raw, non-blocking, as ESPHome's host UART would."""
    import termios
    import tty

    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd
