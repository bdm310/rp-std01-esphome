# rp-std01-esphome

ESPHome for the Martin Jerry RP-STD01 3-way smart dimmer kit, with open firmware replacing the
Tuya firmware on the dimmer's microcontroller (MCU). The unit's
ESP32-C3 runs ESPHome and drives the open MCU firmware; Home Assistant sees the dimmer as an ordinary
ESPHome device, with the buttons, add-on switch, thumbwheel, night light and heatsink temperature
as entities.

## Before you start

- **This is a one-way change.** The factory MCU firmware cannot be backed up or put back, and
  afterwards neither Tasmota nor any Tuya firmware can control the dimmer: only this project's
  ESPHome configuration can.
- **The install is done over Wi-Fi with the box closed.** It does require a power cycle for the
  first flash. Make sure you have easy control over device power.
- **The buttons work only while the ESP32 runs this project's main configuration.** The MCU has
  no local control, so while the ESP32 restarts, or runs the MCU tool during an install or
  update, the buttons do nothing.

This repository was built entirely with AI tools. The firmware works on the device but has not been
rigorously tested. Use at your own risk, though it isn't clear whether the original firmware is
demonstrably any "safer".

To keep the stock MCU firmware and only replace Tasmota with ESPHome, use
`esphome/rp-std01-stock.yaml` instead: on/off and brightness only, as its header explains. This is
the least tested path and is provided only as a reference.

## Installing

Follow [docs/install.md](docs/install.md): from a stock unit to the open MCU firmware
and the ESPHome configuration, from a checkout of this repository on a Linux or macOS computer.
Later updates are in the same guide.

## Buttons

| Button | Press | Hold |
|---|---|---|
| Paddle UP, add-on paddle UP | Lamp on at its last brightness; if already on, a little brighter | Brighten until released |
| Paddle DOWN, add-on paddle DOWN | Lamp off (a press shorter than 300 ms) | Dim until released, starting at the press |
| WiFi ("On/Off" on the board) | Nothing | Night light on or off |

The add-on has only its paddle. The RESET button on the unit is not read by the firmware; it
shorts the supply of both chips, so they restart.

The thumbwheel sets the dim end of the range, as on the stock unit. The LED bar shows the
brightness; of the two LEDs by the WiFi button, one is the night light and the other lights on a
fault and blinks while the ESP32 hears nothing from the MCU's firmware. Every press, double press
and hold of each button is also an event in Home Assistant, for automations of your own.

## Tested

On two units, at 120 V and 60 Hz, with an incandescent lamp and with LED bulbs:

- dimming, smooth and without flicker, down to the thumbwheel's dim end; the lamp stays lit while
  the ESP32 restarts after an update;
- the buttons and all their gestures, the add-on, the thumbwheel, the night light, mains frequency
  and temperature (not checked against a thermometer);
- the install over Wi-Fi in [docs/install.md](docs/install.md), steps 1 to 10, from factory
  Tasmota 13.0.0 and the factory MCU firmware;
- MCU application updates with the MCU tool, also back to an older application;
- the ESP32 reports a power-on after a real mains cut.

Not yet tried:

- mains other than 120 V and 60 Hz;
- the recovery routes of [docs/install.md](docs/install.md) step 11 that open the box;
- the ESP32's reset reason after an OTA restart, a crash or a watchdog reset;
- whether the MCU's count of watchdog resets survives a real watchdog reset (assumption A1 in
  [docs/mcu-firmware-design.md](docs/mcu-firmware-design.md#assumptions-to-confirm));
- the thermistor curve: the temperature and the over-temperature thresholds are provisional until
  it is measured;
- macOS.

## Documentation

| Document | Contents |
|---|---|
| [docs/install.md](docs/install.md) | Installing on a unit, and later updates, step by step |
| [hardware/README.md](hardware/README.md) | The hardware: chips, pins, wiring, measurements of the stock firmware, photos |
| [docs/mcu-firmware-design.md](docs/mcu-firmware-design.md) | Design of the MCU firmware: timing, safety, link protocol, bootloader |
| [docs/esphome-config-design.md](docs/esphome-config-design.md) | Design of the ESPHome configuration |
| [docs/mcu-tool.md](docs/mcu-tool.md) | Design of the MCU tool, the ESPHome firmware that installs and updates the MCU firmware |
| [docs/development.md](docs/development.md) | Repository layout, building and testing |

## License

GPL-3.0-or-later ([LICENSE](LICENSE)). The STC programmer, ported from `stcgal`, and the
`stcgal` packets its test uses are also under `stcgal`'s MIT license
([LICENSES/stcgal-MIT.txt](LICENSES/stcgal-MIT.txt)).
