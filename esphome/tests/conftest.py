"""Fixtures shared by the fake MCU self-tests and the ESPHome host tests. Run with: make test-host"""

from __future__ import annotations

from pathlib import Path
import re
import subprocess
import sys

import pytest

TESTS = Path(__file__).resolve().parent
REPO = TESTS.parent.parent
BUILD = REPO / "build" / "host"
sys.path.insert(0, str(TESTS))

from mcu_proto import LIVE_GROUPS, FakeMcu, committed_app_image, kill_group  # noqa: E402


def substitution(yaml: str, name: str) -> str:
    """The value of a substitution in a host test configuration."""
    text = (TESTS / yaml).read_text()
    m = re.search(rf'^  {name}: "?([^"\n]+)"?$', text, re.M)
    if not m:
        raise KeyError(f"{yaml}: {name}")
    return m.group(1)


PTY_LINK = substitution("host.yaml", "mcu_port")


def compile_host(yaml: str) -> Path:
    """Builds a host test configuration with ESPHome and returns the program."""
    log = BUILD / f"{Path(yaml).stem}-compile.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    with open(log, "wb") as out:
        command = [sys.executable, "-m", "esphome", "compile", str(TESTS / yaml)]
        done = subprocess.run(command, stdout=out, stderr=subprocess.STDOUT, check=False)
    if done.returncode:
        tail = "\n".join(log.read_text(errors="replace").splitlines()[-40:])
        pytest.fail(f"esphome compile {yaml} failed ({log}):\n{tail}", pytrace=False)
    name = re.search(r"^  name: (\S+)$", (TESTS / yaml).read_text(), re.M).group(1)
    return TESTS / ".esphome" / "build" / name / ".pioenvs" / name / "program"


def config_rejected(lines: list[str], workdir: Path) -> str:
    """Validates a configuration with `esphome config` and returns its output; it must be rejected."""
    yaml = workdir / "rejected.yaml"
    yaml.write_text("\n".join(lines) + "\n")
    command = [sys.executable, "-m", "esphome", "config", str(yaml)]
    done = subprocess.run(command, capture_output=True, text=True, check=False)
    assert done.returncode != 0, f"{yaml} was accepted"
    return done.stdout + done.stderr


def pytest_sessionfinish(session, exitstatus) -> None:
    """Kills programs a failed or interrupted test left running."""
    for pid in list(LIVE_GROUPS):
        kill_group(pid)


@pytest.fixture(scope="session")
def fake_bin() -> Path:
    subprocess.run(["make", "-s", "-C", str(REPO), "build/host/fake_mcu"], check=True)
    return BUILD / "fake_mcu"


@pytest.fixture(scope="session")
def app_image() -> bytes:
    return committed_app_image()


@pytest.fixture(scope="session")
def app_image_file(app_image: bytes, tmp_path_factory) -> Path:
    path = tmp_path_factory.mktemp("image") / "app_area.bin"
    path.write_bytes(app_image)
    return path


@pytest.fixture
def make_mcu(fake_bin: Path, app_image_file: Path, tmp_path: Path):
    """Factory for fake MCUs, on host.yaml's MCU port unless given a link; all are stopped at the
    end of the test."""
    made: list[FakeMcu] = []

    def _make(**kwargs) -> FakeMcu:
        image = kwargs.pop("image_bytes", None)
        workdir = kwargs.pop("workdir", tmp_path)
        if image is not None:
            path = workdir / "image.bin"
            path.write_bytes(image)
            kwargs["image"] = path
        kwargs.setdefault("image", app_image_file)
        kwargs.setdefault("link", PTY_LINK)
        mcu = FakeMcu(binary=fake_bin, workdir=workdir, **kwargs)
        mcu.start()
        made.append(mcu)
        return mcu

    yield _make
    for mcu in made:
        mcu.stop()
