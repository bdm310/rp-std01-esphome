# Installing

The path is: factory Tasmota -> the MCU tool (an ESPHome firmware that installs the MCU firmware)
-> the main configuration, over Wi-Fi, with the box closed. Step 10 covers later updates.

Everything is done from a terminal on a Linux or macOS computer, in a checkout of this repository,
with ESPHome installed from Python's package index. The Home Assistant ESPHome add-on (the ESPHome
dashboard) cannot do this install: it cannot run `install/upload.sh` (step 6).

> **Read first**
>
> - Read ["Before you start"](../README.md#before-you-start) in the README. The point of no return
>   is step 6. The other steps that must not be cut short are the partition table uploads in step 9.
> - **Mains.** Do not open the box while it is connected to mains: its low-voltage side, including the serial header, is not isolated from mains
>   ([../hardware/README.md](../hardware/README.md)).
> - **No password.** The web pages of the MCU tool and the main configuration have none: anyone on
>   your network can use them while they run.

## 1. Requirements

| What | Tested | Notes |
|---|---|---|
| Linux or macOS | Ubuntu 24.10 | macOS should work, but has not been tried. Windows has not been tried |
| git | | To clone this repository |
| Python | 3.12 | ESPHome needs 3.12 to 3.14. On macOS: `brew install python@3.12` (Homebrew), then use `python3.12` below |
| curl | | For the Tasmota backup and `install/upload.sh` |

The first ESPHome build downloads its own toolchain (about 1 to 2 GB), so it needs internet access
and takes several minutes. Later builds are quicker. Nothing else is needed: the MCU firmware is
already built and committed.

Check the Python version with `python3 --version`. With an older one, the `pip install` below stops
with "No matching distribution found for esphome". Install ESPHome in a Python virtual environment
(on Ubuntu, `python3 -m venv` first needs `sudo apt install python3-venv`):

```sh
python3 -m venv ~/esphome-venv
source ~/esphome-venv/bin/activate
pip install esphome==2026.10.1
esphome version
```

This is the tested version; older ones cannot replace the partition table (step 9). **In every new
terminal**, run `source ~/esphome-venv/bin/activate` again before the commands below.

## 2. Cloning

Clone this repository with `git clone`, and change into the folder it made. All commands below run
from there (the repository root).

## 3. Checking the unit, and backups

Find the unit's IP address in your router's list of clients. Below it is `<ip>`. A new unit that
is not on your Wi-Fi yet opens its own access point, `tasmota-...`: join it, open
`http://192.168.4.1`, and enter your Wi-Fi there.

Open `http://<ip>/` in a browser. A factory unit shows Tasmota's page for "STD01 by MJ", and
Tasmota 13.0.0 at the bottom of the page. This guide was tested only from that state. Another
Tasmota version has not been tried: steps 6 and 9 rely on the factory partition table, with
Tasmota's safeboot partition.

## 4. The device files

Each unit gets a unit file with its names, a device file for the main configuration and one for
the MCU tool, all in `esphome/local/`, next to one `secrets.yaml`. Copy the examples:

```sh
mkdir -p esphome/local
cp esphome/example-unit.yaml esphome/local/unit.yaml
cp esphome/example-dimmer.yaml esphome/local/dimmer.yaml
cp esphome/example-mcu-tool.yaml esphome/local/dimmer-tool.yaml
cp esphome/example-secrets.yaml esphome/local/secrets.yaml
```

Make an API key:

```sh
python3 -c "import base64, os; print(base64.b64encode(os.urandom(32)).decode())"
```

Then edit `esphome/local/secrets.yaml`: your Wi-Fi name and password, the API key (44 characters
ending in `=`), and a password of at least 8 characters for the fallback hotspot. Keep the API key:
Home Assistant asks for it, and every later upload needs it.

In `esphome/local/unit.yaml`:

- **`name`**: the unit's network name (letters, digits and `-`). The unit is then reachable as
  `<name>.local`. **`friendly_name`**: the name Home Assistant shows. The fallback hotspot is
  called "`<friendly_name>` Fallback".

Optional, in `esphome/local/dimmer.yaml` under `substitutions:`: `night_light_mode: Light` (the
default is `On When Lamp Off`), the night light's behaviour at the first start; it is a setting in
Home Assistant afterwards.

**Another unit:** copy the three device files again under new names (not `secrets.yaml`, which
would be overwritten), change `unit.yaml` in the two `!include` lines to the new unit file, and
give it its own `name`. For its own API key, add one to `secrets.yaml` under another name and use
that name for `api_key` in its unit file.

## 5. Building

```sh
esphome compile esphome/local/dimmer-tool.yaml
esphome compile esphome/local/dimmer.yaml
```

The tool carries the MCU firmware that the main configuration expects, so always build both from
the same checkout. At the end of the tool's build ESPHome prints the file to upload in step 6
("Created: ...firmware.ota.bin"):

```
esphome/local/.esphome/build/<name>-tool/build/firmware.ota.bin
```

## 6. Uploading the tool: the point of no return

> **Point of no return.** As soon as the tool runs on a unit with the factory MCU firmware, it
> arms the MCU install by itself. From then on, **any power cut**, a power failure included,
> replaces the factory MCU firmware for good, unless **Cancel Install** is pressed on the tool's
> page first.

The upload goes through Tasmota's safeboot partition, which writes the tool where Tasmota's own
program was. Upload `firmware.ota.bin`, never `firmware.factory.bin` (a whole-flash image for a
serial flash, which `install/upload.sh` refuses):

```sh
install/upload.sh <ip> esphome/local/.esphome/build/<name>-tool/build/firmware.ota.bin
```

It prints "Restarting <ip> into safeboot...", "Uploading ...", safeboot's answer, which should
contain "Upload Successful", then "Done". It stops with an error, having uploaded nothing, if
Tasmota asks for a web password (`WebPassword 0`, step 3), or if the unit does not answer as
Tasmota does.

**From rp-std01-stock.yaml:** upload the tool with ESPHome instead:

```sh
esphome upload esphome/local/dimmer-tool.yaml --device <ip>
```

The tool joins your Wi-Fi with the settings from `secrets.yaml`, not Tasmota's. Find it at the
same IP address (your router usually gives it the same one), or as `http://<name>.local`, or in
your router's list of clients under the new name. If it does not appear within 2 minutes, see step
11. While the tool runs, Home Assistant shows the unit as unavailable.

## 7. The tool's page

Open `http://<ip>/`. On a factory unit, after a few seconds, MCU State says "The MCU does not
answer our protocol (factory firmware, or none). Install armed: press the RESET button. ...". The install is armed.

While MCU State starts with "Wait until this tool has run for", wait until that sentence is gone,
then go on to step 8.

## 8. The install

1. Press the RESET button once.
2. The tool writes the MCU at the power-on, before it starts Wi-Fi, so the unit comes back on the
   network about 10 s later than after an ordinary restart. The lamp stays off.

**Success:** MCU State says "Everything is current: install the main configuration.", and Last MCU
Install starts with "Install OK".

**Failures** (Last MCU Install, and what to do):

| Text | What to do |
|---|---|
| Install failed: no reply, timeout, framing or bad reply; or Install failed at calibrate, baud or handshake | Nothing on the MCU was changed. Press RESET again |
| Install failed at erase, write, finish or options: ..., N bytes written; or Install failed: did not finish (the attempt was cut short) | The MCU's firmware may be incomplete, and the lamp does not work until an install completes. Press RESET again: the tool installs at every power-on until a write completes. Do not install the main configuration yet |
| Install refused: ... Nothing was changed | The MCU is not the expected chip, or a setting the install relies on differs from the factory one. Nothing was written, and trying again gives the same refusal on the same chip: stop, this unit needs a closer look ([../hardware/README.md, "Programming the MCU"](../hardware/README.md#programming-the-mcu)) |
| Install written, but the new firmware did not answer | If MCU Installed and MCU State show our firmware, all is well. Otherwise follow MCU State: if the MCU still does not answer, the tool arms another install |

After the automatic attempts fail several times in a row, MCU State says "... automatic attempts
failed, so no more are made": press **Install at Next Power-On** and press RESET again.

## 9. Moving to the main configuration

With the tool reporting "Everything is current", replace Tasmota's partition table with ESPHome's
standard one first. Skip this if the unit ran `rp-std01-stock.yaml` and its partition table was
replaced then. This is "Indirect Method A" of ESPHome's own
[Tasmota migration](https://esphome.io/guides/migrate_sonoff_tasmota/#esp32-partition-table-migration),
which the tool fits.

**Do not cut the power while these run.** They replace Tasmota's partition table, and with it
Tasmota's safeboot partition: afterwards `install/upload.sh` no longer works on this unit, and a
power cut in the middle of either one can leave the ESP32 unable to start until it is flashed
over serial (step 11).

Download the two partition tables:

```sh
curl -fsSL --create-dirs -o build/partitions_tasmota_migration_a.bin https://esphome.io/files/ota/partitions_tasmota_migration_a.bin
curl -fsSL --create-dirs -o build/partitions_esphome.bin https://esphome.io/files/ota/partitions_esphome.bin
```

Then upload them in this order, each once the one before has ended. `--device` takes `<ip>` or
`<name>.local`:

```sh
esphome upload esphome/local/dimmer.yaml --partition-table --file build/partitions_tasmota_migration_a.bin --device <ip>
esphome upload esphome/local/dimmer.yaml --partition-table --file build/partitions_esphome.bin --device <ip>
```

Each ends with "OTA successful", and the unit restarts. If the unit refuses a table ("The new
partition table must contain a compatible app partition"), it has changed nothing: the running
image is too big for this method, and ESPHome's guide lists the others. Then upload the main
configuration itself:

```sh
esphome upload esphome/local/dimmer.yaml --device <ip>
```

The device log (`esphome logs esphome/local/dimmer.yaml --device <ip>`) then shows the warning
"Bootloader too old for OTA rollback. Flash via USB once to update the bootloader", because
Tasmota's second-stage bootloader stays. It is harmless: ignore the advice to flash over USB.

**Check afterwards** on `http://<ip>/`:

- the lamp follows the page and the paddle: UP turns it on, DOWN off, and holding either one
  brightens or dims; the small WiFi button does not switch the lamp ([../README.md,
  "Buttons"](../README.md#buttons));
- MCU Firmware shows the same version and build as MCU Firmware Expected;
- MCU Update Available is off, and MCU State does not end in "(update available: use the MCU
  tool)";
- the device log does not keep showing "MCU restarted (N times since boot)";
- the thumbwheel: hold DOWN until the lamp is at its lowest level, then turn the wheel fully one
  way and then the other. End A is the end where the lamp is brighter, end B the end where it is
  dimmer. The Thumbwheel sensor should read 0 % at end A and 100 % at end B. If it does not, read
  the Thumbwheel Raw sensor (press Show All on the unit's web page) at each end, put the reading
  at end A into `thumbwheel_raw_a` and the one at end B into `thumbwheel_raw_b` in
  `esphome/local/dimmer.yaml`, and upload again.
- the dim end with your lamps: hold DOWN until the lamp is at its lowest level and leave it there
  for a minute. Every lamp should stay lit and steady. If the lamp turns itself off, or MCU Faults
  shows "mains lost", the dim end is deeper than these lamps can follow (step 11, "The lamp turns
  itself off"): turn the thumbwheel toward end A, or with Thumbwheel Use set to Input Only raise
  Dim End, until the lamp stays lit, then a little further.

**Home Assistant:** *Settings* -> *Devices & services*. The unit is usually discovered as an
ESPHome device; choose *Add* and enter the API key from `secrets.yaml`. If it is not discovered,
*Add integration* -> *ESPHome*, with host `<ip>` (or `<name>.local`), port 6053, and the same key.
If Home Assistant had the unit as a Tasmota device, delete that device (*Settings* -> *Devices &
services* -> *Tasmota*), because it no longer answers.

## 10. Updates

1. `git pull`. Then `git diff ORIG_HEAD HEAD -- 'esphome/example-*.yaml'` shows what the pull
   changed in the example files: make the same changes to your copies in `esphome/local/`, which
   are not changed by themselves. Then build both files (step 5).
2. Upload the main configuration: `esphome upload esphome/local/dimmer.yaml --device <ip>`.
3. If **MCU Update Available** is then on, or MCU State says "Update mode (use the MCU tool)" or
   "MCU firmware needs the MCU tool", update the MCU:
   1. Upload the tool: `esphome upload esphome/local/dimmer-tool.yaml --device <ip>`. The buttons
      do nothing while it runs, and Home Assistant shows the unit as unavailable.
   2. Open the tool's page and follow MCU State. If only the application is out of date, the tool
      updates it at once, with no power cycle, and then says "Application updated. Everything is
      current: install the main configuration." If the bootloader is out of date, it asks for a
      press of RESET, as in steps 7 and 8.
   3. Upload the main configuration again, as in 2.

`--partition-table` is needed only the first time.

## 11. Troubleshooting and recovery

**The unit does not join Wi-Fi.** After about 90 s (ESPHome's `ap_timeout`) it opens the fallback
hotspot "`<friendly_name>` Fallback". Join it with the fallback password; the captive portal opens
(or go to `http://192.168.4.1`), where you can choose a network and enter its password. Then
correct `secrets.yaml` before the next upload.

**The MCU's firmware is newer than the build.** The tool says, for example, "The MCU's application
is newer than this tool's, so nothing is armed", or the main configuration's MCU State ends in
"(MCU firmware newer than this configuration)". The build comes from an older checkout than the
firmware on the MCU: update the checkout and build again. The tool's buttons (Update Application
Now, Install at Next Power-On) force its firmware onto the MCU, also when it is older.

**The lamp turns itself off, and MCU Faults shows "mains lost".** The MCU times the triac from the
zero-cross signal, which it appears to read across the dimmer's switch, so the signal reaches it
through the lamps ([../hardware/README.md, "Zo with LED
lamps"](../hardware/README.md#zo-with-led-lamps)). LED lamps that stop conducting take the signal
with them. When the MCU has seen no usable zero crossing with the relay closed, for 0.33 s with
none at all or 0.75 s with them in the wrong place, it treats mains as lost and turns the lamp off rather than fire blind. A commercial
dimmer would let the lamp flicker or drop out instead. The usual causes:

- the dim end is deeper than the lamps can follow: raise it, as in step 9's dim end check;
- the load is too light, for example a single small LED lamp: add lamps, or keep the dim end
  higher. Commercial dimmers set a minimum lamp count for the same reason.

If it happens at levels the lamps handle well, or with the lamp off, note the time and keep the
device log (`esphome logs esphome/local/dimmer.yaml --device <ip>`): the faults are logged with the
status the MCU reported.

**"Link fault: the MCU answered after the install, but not now."** The new firmware answered right
after the install, but the check then heard nothing. The tool does not install again. This points
to the serial link between the chips. Restart the tool to check again (a RESET press with nothing
armed is safe). If it stays, the board needs a look with the box open: disconnect the dimmer from
mains first ("Read first").

**The ESP32 keeps restarting.** After several failed starts in a row, ESPHome starts in safe mode:
Wi-Fi and OTA only. Upload the main configuration (or the tool) again with `esphome upload`.

**Serial flash of the ESP32** (last resort, box open, not tried on this unit). For an ESP32 that
does not start at all, write ESPHome's `firmware.factory.bin` (next to the `firmware.ota.bin` of
the same build), or a backup from step 3 while the factory MCU firmware is still installed:

- **Disconnect the dimmer from mains first**, at the breaker, and remove it from the wall. Its
  low-voltage side is not isolated from mains.
- The 9-pin header is unpopulated: solder a header or wires to it
  ([../hardware/README.md, "9-pin header"](../hardware/README.md#9-pin-header-unpopulated)).
- Use a **3.3 V** USB-serial adapter: its 3.3 V and GND to header pins 1 and 2, its TX to pin 8
  (ESP32 RX0) and its RX to pin 9 (ESP32 TX0). Hold GPIO9 (header pin 5) low while the adapter's
  power comes on, for the ESP32's download mode; GPIO2 (pin 3) and GPIO8 (pin 7) must stay high,
  so connect nothing that pulls them low. The MCU shares pins 8 and 9 and runs too: its serial
  output on pin 8 may disturb the transfer.
- **Never press the RESET button while the adapter powers the board:** nothing would limit the
  current.
- `esptool` (v5) comes with ESPHome, in its virtual environment: `esptool --port <port> write-flash
  0 <file>`, with the adapter's port (for example `/dev/ttyUSB0`) if it is not found by itself.

**The MCU** can also be written with the box open, with `stcgal`. This is for experts: it needs
SDCC and `make firmware` ([development.md](development.md),
[../hardware/README.md, "With the box open"](../hardware/README.md#with-the-box-open-not-tried)).

**Going back to Tasmota** is not supported: our MCU firmware does not speak Tuya's protocol, so
Tasmota's TuyaMCU driver could not control the lamp.
