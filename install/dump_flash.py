#!/usr/bin/env python3
"""Read the whole flash of an ESP32 running Tasmota over Wi-Fi, with Tasmota's Berry
(`flash.read`), 1 kB per web request. Takes about 3.5 minutes for 4 MB. Standard library only.

The dump is a backup of the ESP32 side only (Tasmota, its settings and its partition table); the
MCU's firmware is not in it, so it can only take a unit back to Tasmota before the MCU install.
Writing it back is a serial flash with the box open.

Usage: dump_flash.py <device-ip> <out.bin>
"""

from __future__ import annotations

import base64
import binascii
import json
import os
from pathlib import Path
import socket
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

CHUNK = 1024
TRIES = 5
# A read that failed is tried again once the unit accepts connections again, which a restart of
# Tasmota, Wi-Fi included, does well within this.
RECOVER_S = 60
POLL_S = 0.5


class Refused(Exception):
    """An answer that trying again does not change."""


def command(ip: str, cmd: str):
    url = f"http://{ip}/cm?cmnd=" + urllib.parse.quote(cmd)
    try:
        with urllib.request.urlopen(url, timeout=10) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        if e.code == 401:
            raise Refused(f"{ip} asks for Tasmota's web password: clear it (WebPassword 0), then try again") from e
        raise


def flash_size(ip: str) -> int:
    """As Tasmota reports it (Status 4, in kB)."""
    reply = command(ip, "Status 4")
    kb = reply.get("StatusMEM", {}).get("FlashSize") if isinstance(reply, dict) else None
    if isinstance(kb, int) and kb > 0:
        return kb * 1024
    raise Refused(f"{ip} does not report its flash size (Status 4): {str(reply)[:100]}")


def answers(ip: str) -> bool:
    try:
        socket.create_connection((ip, 80), timeout=POLL_S).close()
        return True
    except OSError:
        return False


def wait_until_answers(ip: str) -> bool:
    deadline = time.monotonic() + RECOVER_S
    while not answers(ip):
        if time.monotonic() > deadline:
            return False
        time.sleep(POLL_S)
    return True


def read_chunk(ip: str, addr: int) -> bytes:
    reply = command(ip, f"Br import flash return flash.read({addr:#x}, {CHUNK}).tob64()")
    encoded = reply.get("Br") if isinstance(reply, dict) else None
    if not isinstance(encoded, str):
        raise Refused(f"{ip} does not answer as Tasmota with Berry: {str(reply)[:100]}")
    try:
        data = base64.b64decode(encoded, validate=True)
    except binascii.Error as e:
        raise Refused(f"Berry on {ip} reports an error: {encoded[:200]}") from e
    if len(data) != CHUNK:
        raise ValueError(f"short read: {len(data)} bytes")
    return data


def dump(ip: str, f) -> str | None:
    """Writes the flash to f; returns an error, or None."""
    try:
        size = flash_size(ip)
    except Refused as e:
        return str(e)
    except (OSError, ValueError) as e:
        return f"{ip} does not answer: {e}"
    for addr in range(0, size, CHUNK):
        data: bytes | None = None
        err: Exception | None = None
        for _ in range(TRIES):
            try:
                data = read_chunk(ip, addr)
                break
            except Refused as e:
                return str(e)
            except (OSError, ValueError) as e:
                err = e
                if not wait_until_answers(ip):
                    break
        if data is None:
            return f"failed at {addr:#x}: {err}"
        f.write(data)
        done = addr + CHUNK
        if done % 0x40000 == 0:
            print(f"{done // 1024} kB of {size // 1024} kB", flush=True)
    return None


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__.strip().splitlines()[-1], file=sys.stderr)
        return 2
    ip, out = sys.argv[1], Path(sys.argv[2]).resolve()
    part = out.with_name(out.name + ".part")
    try:
        with open(part, "wb") as f:
            err = dump(ip, f)
        if err:
            print(err, file=sys.stderr)
            return 1
        os.replace(part, out)
    finally:
        part.unlink(missing_ok=True)
    print(f"Saved {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
