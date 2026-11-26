"""The ESPHome host binary as the host tests drive it: a process with an API client that keeps the
latest state of each entity."""

from __future__ import annotations

import asyncio
from collections.abc import Callable
import os
from pathlib import Path
import re
import signal
import subprocess
import time
from typing import Any, NamedTuple

from aioesphomeapi import APIClient

from mcu_proto import LIVE_GROUPS, die_with_parent, kill_group


def domain(info: Any) -> str:
    return type(info).__name__.removesuffix("Info").lower()


class Event(NamedTuple):
    name: str
    type: str


class Esp:
    def __init__(self, binary: Path, workdir: Path, api_port: int) -> None:
        self.binary = binary
        self.api_port = api_port
        self.workdir = workdir
        self.proc: asyncio.subprocess.Process | None = None
        self.client: APIClient | None = None
        self.entities: dict[tuple[str, str], Any] = {}
        self.states: dict[int, Any] = {}
        self.events: list[Event] = []
        self.runs = 0

    async def start(self, env: dict[str, str] | None = None) -> None:
        self.runs += 1
        full_env = {k: v for k, v in os.environ.items() if k != "RPSTD01_SOFT_REBOOT"}
        full_env.update(ESPHOME_PREFDIR=str(self.workdir / "prefs"), **(env or {}))
        with open(self.workdir / f"esp{self.runs}.log", "wb") as log:
            self.proc = await asyncio.create_subprocess_exec(
                str(self.binary),
                stdout=log,
                stderr=subprocess.STDOUT,
                env=full_env,
                start_new_session=True,
                preexec_fn=die_with_parent,
            )
        LIVE_GROUPS.add(self.proc.pid)
        end = time.monotonic() + 15
        while True:
            client = APIClient("127.0.0.1", self.api_port, "", client_info="rpstd01-host-test")
            try:
                await client.connect(login=True)
                break
            except Exception:
                await client.disconnect(force=True)
                if time.monotonic() > end or self.proc.returncode is not None:
                    raise
                await asyncio.sleep(0.1)
        self.client = client
        infos, _ = await client.list_entities_services()
        self.entities = {(domain(i), i.name): i for i in infos}
        self.names = {i.key: i.name for i in infos}
        self.states = {}
        client.subscribe_states(self._on_state)

    def _on_state(self, state: Any) -> None:
        self.states[state.key] = state
        if type(state).__name__ == "Event":
            self.events.append(Event(self.names[state.key], state.event_type))

    async def stop(self, kill: bool = False) -> None:
        if self.client:
            await self.client.disconnect(force=True)
            self.client = None
        if self.proc:
            if self.proc.returncode is None:
                self.proc.send_signal(signal.SIGKILL if kill else signal.SIGTERM)
                try:
                    await asyncio.wait_for(self.proc.wait(), 10)
                except asyncio.TimeoutError:
                    pass
            kill_group(self.proc.pid)
        self.proc = None

    def info(self, dom: str, name: str) -> Any:
        return self.entities[(dom, name)]

    def state(self, dom: str, name: str) -> Any:
        return self.states.get(self.info(dom, name).key)

    def value(self, dom: str, name: str) -> Any:
        s = self.state(dom, name)
        if s is None or getattr(s, "missing_state", False):
            return None
        return s.state

    async def light(self, on: bool, brightness: float | None = None, transition: float | None = 0) -> None:
        self.client.light_command(
            self.info("light", "").key, state=on, brightness=brightness, transition_length=transition
        )

    async def select(self, name: str, option: str) -> None:
        self.client.select_command(self.info("select", name).key, option)

    async def press(self, name: str) -> None:
        self.client.button_command(self.info("button", name).key)

    def lamp(self) -> tuple[bool, float]:
        s = self.state("light", "")
        return (s.state, s.brightness) if s else (False, 0.0)

    def mcu_restarts(self) -> int:
        """The MCU restart count from the newest "MCU restarted" warning in this run's log."""
        text = (self.workdir / f"esp{self.runs}.log").read_text(errors="replace")
        found = re.findall(r"MCU restarted \((\d+) times since boot\)", text)
        return int(found[-1]) if found else 0


async def until(pred: Callable[[], Any], timeout: float = 5.0, what: str = "") -> Any:
    end = time.monotonic() + timeout
    while True:
        result = pred()
        if result:
            return result
        if time.monotonic() > end:
            raise AssertionError(f"timed out after {timeout} s waiting for {what or pred}")
        await asyncio.sleep(0.02)
