# Development

## Layout

| Path | Contents |
|---|---|
| `firmware/` | MCU firmware: bootloader, application, host tests, and `tools/mkesphome.py`, which turns the built firmware into the files ESPHome builds from |
| `esphome/` | ESPHome configuration: the main configuration, the MCU tool, the shared device package, the examples, the configuration for units that keep the stock MCU firmware (`rp-std01-stock.yaml`), and their tests |
| `install/` | `upload.sh` and `dump_flash.py`, used by [install.md](install.md) |
| `hardware/` | Hardware notes and board photos |
| `Makefile` | Builds and tests everything, from the top folder |

## Commands

| Command | What it does | Needs |
|---|---|---|
| `make test` | Every host test: the MCU firmware, the ESPHome side, the MCU tool, and `mkesphome.py`. Then `make check`, if SDCC is on `PATH`. What each covers is in the "Testing" section of [mcu-firmware-design.md](mcu-firmware-design.md#testing), [esphome-config-design.md](esphome-config-design.md#testing) and [mcu-tool.md](mcu-tool.md#testing) | gcc, g++, GNU Make, Python 3 |
| `make test-host` | ESPHome's host platform runs the main configuration and the MCU tool against a fake MCU, driven over ESPHome's API (a few minutes). `PYTEST_ARGS="-k update"` picks tests. It uses the Python of `ESPHOME_VENV` (a virtual environment), or else the one next to `esphome` on `PATH` | The above, and ESPHome with pytest, pytest-asyncio and aioesphomeapi |
| `make firmware` | Builds the MCU programs and writes the files made from them (below). Fails on any SDCC compiler or linker message (SDCC reports a failed static assert, and code overlapping an absolute area, only as a warning), and on any problem `firmware/tools/isr_overlay.py` finds with `--nooverlay` (`ISR_SRC`) or reentrancy ([mcu-firmware-design.md, "Toolchain and build"](mcu-firmware-design.md#toolchain-and-build)) | SDCC, a host C compiler (`cc`, or `$CC`), Python 3 |
| `make check` | Rebuilds the MCU programs and fails, naming the file, if a committed file that `make firmware` writes is out of date; and, naming the constant to raise, if the bootloader or the application differs from the image in the latest git tag (the last release) while `BOOT_VERSION` or `FW_VERSION` is not higher; before the first tag versions are free | SDCC, a host C compiler, Python 3, git |
| `make stc-vectors` | Records again what `stcgal` sends for the STC programmer's test, with `stcgal` cloned at a pinned commit. Only needed when that test changes | git, network, Python 3 |
| `make clean` | Removes `build/`, where everything is built | |

| Tool | Tested version | Ubuntu package |
|---|---|---|
| SDCC, with `sdas8051` | 4.4.0 | `sdcc` |
| GNU Make | 4.3 | `make` |
| gcc and g++ | 14.2 | `gcc`, `g++` |
| Python | 3.12, standard library only | `python3` |
| ESPHome | as in [install.md, "Requirements"](install.md#1-requirements); to move to another version, change it there and `min_version` in `esphome/rp-std01-device.yaml` and `esphome/rp-std01-stock.yaml` together | |

## What `make firmware` writes

| File | Use |
|---|---|
| `esphome/components/rpstd01_tool/rpstd01_mcu_full_image.h` | Bootloader and application, which the MCU tool installs. Committed |
| `esphome/components/rpstd01_link/rpstd01_fw_constants.h` | The constants of `firmware/include/protocol.h`, `flashmap.h` and `board.h` as C++ constants in namespace `rpstd01` (including the C headers would put their short macro names into ESPHome's `main.cpp`), and the version, build ID and bootloader version of the image, which the main configuration expects. Committed, so an ESPHome build needs no SDCC |
| `build/mcu/full.ihx` | The same image as the first header, for `stcgal` with the box open ([mcu-firmware-design.md, "First flash and option bits"](mcu-firmware-design.md#first-flash-and-option-bits)) |
| `build/mcu/app.ihx`, `boot.ihx`, `*.map` | The separate programs and their memory maps |

It also prints how much of each flash region is used. After changing the MCU firmware, run
`make firmware` and commit both headers; `make check` (and so `make test`, where SDCC is installed)
fails until they match the sources. How the build ID and the version are made is in
[mcu-firmware-design.md, "Toolchain and build"](mcu-firmware-design.md#toolchain-and-build).
