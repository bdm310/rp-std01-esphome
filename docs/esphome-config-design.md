# ESPHome configuration design

Design of the ESP32 side of the RP-STD01 port (`../esphome/`): an ESPHome YAML configuration that
drives our MCU firmware over the link in [mcu-firmware-design.md](mcu-firmware-design.md). Test
results are in [../README.md](../README.md#tested), the files in [development.md,
"Layout"](development.md#layout). A mark such as **[E1]** points to a row of "Assumptions to
check".

## Decisions

The MCU-side decisions are in [mcu-firmware-design.md](mcu-firmware-design.md). Here:

- **Local external components, logic in plain C++.** The packages load `esphome/components/`
  with `external_components:` (`source: local`, a path next to the package, so a device file can
  sit in any folder). `rpstd01_link.h` (frames, CRC, parser, decoders, shared with the MCU tool)
  and `rpstd01.h` (the link handling, the curve, inputs and reports) have no dependency on
  ESPHome, so g++ tests them directly. A thin component (`rpstd01_component.h`) owns the link,
  runs it, applies the settings entities and provides the outputs; lambdas only connect the rest
  to entities. There are three components: `rpstd01_link` (shared), `rpstd01` (the main
  configuration's) and `rpstd01_tool` (the MCU tool's, which also holds the MCU image as a header),
  because ESPHome includes every header of a loaded component: the main configuration must not
  carry the MCU image.
- **MCU updates are not done here.** A separate ESPHome firmware for the same ESP32, the MCU tool
  ([mcu-tool.md](mcu-tool.md)), installs and updates the MCU firmware. This configuration only says
  when an update is available, and keeps quiet while the MCU is in update mode ("Update mode").
- **The dimming curve is linear in power** between a dim end and a bright end.

Choices of behaviour someone else might make differently:

- **Button and add-on actions**, in particular add-on DOWN single = off and WiFi button hold = night
  light.
- **LED meaning:** the brightness bar, B3 for faults, and the night light coming on when the lamp
  turns off.
- **A fault turns the light off** once, when it starts; a turn-on during the fault waits for it
  to clear.
- **A Fade Limit faster than the stock fade** (`LIMIT_BRIGHTER`, `LIMIT_DIMMER`), and a default
  transition ("Transitions and the MCU's smoothing").

## Partition table

The configurations use ESPHome's standard partition table. A unit migrated from the factory Tasmota
first runs the MCU tool from Tasmota's application slot at 0xE0000, then moves to the standard
table with ESPHome's own Tasmota migration ([install.md](install.md), step 9). The tool's image is
too big for ESPHome's direct method, which copies it into Tasmota's 832 kB safeboot slot, so step 9
uses "Indirect Method A", which takes images up to 960 kB. The OTA in `rp-std01-device.yaml` has
`allow_partition_access: true`, which `esphome upload --partition-table` needs.

## Entities

"Diagnostic" entities are hidden in Home Assistant's (HA) main view; "disabled" ones are off until
enabled. Ranges and defaults of the numbers are in `rp-std01-core.yaml`.

| Entity | Type | HA | Notes |
|---|---|---|---|
| (device name) | `light`, monochromatic | yes | The lamp |
| Night Light | `light`, monochromatic | yes | B2, PWM |
| Night Light Mode | `select` | config | Light / On When Lamp Off (default); first-boot default from the `night_light_mode` substitution |
| Brightness Bar | `select` | config | Level (default) / Off |
| ON-OFF (the WiFi button), UP, DOWN Button | `binary_sensor` | diagnostic, disabled | Raw presses, after debouncing |
| Add-on UP, Add-on DOWN | `binary_sensor` | diagnostic, disabled | From Ks |
| Button | `event` | yes | One per button and add-on button: `single`, `double`, `hold` |
| Temperature | `sensor`, C | diagnostic | Heatsink, from the thermistor |
| Mains Frequency | `sensor`, Hz | diagnostic | From the half-period |
| MCU State | `text_sensor` | diagnostic | Off, Starting, On, Stopping, Fault, Unknown, Update mode (use the MCU tool), No Response, MCU firmware needs the MCU tool; the first six followed by " (update available: use the MCU tool)" while MCU Update Available is on, or by " (MCU firmware newer than this configuration)" |
| MCU Problem | `binary_sensor`, problem | yes | On while the MCU reports a fault, does not answer, is in update mode, has not sent its info frame yet, or reports a link protocol other than its own |
| MCU Faults | `text_sensor` | diagnostic | For example "over-temperature, mains lost"; "none" |
| MCU Firmware | `text_sensor` | diagnostic | Installed version and build, as `VVVV build BBBBBBBB` (the MCU tool shows them the same way) |
| MCU Firmware Expected | `text_sensor` | diagnostic | The version and build this configuration expects (`MCU_IMAGE_*` in `rpstd01_fw_constants.h`) |
| MCU Update Available | `binary_sensor`, update | diagnostic | The MCU's firmware is not the expected build and not newer ("The expected build"); update with the MCU tool |
| MCU Reset Cause | `text_sensor` | diagnostic, disabled | For example "Power-on, watchdog count 0, bootloader <n>" (Power-on, Watchdog or Other) |
| Thumbwheel | `sensor`, % | yes | 0 % at end A, 100 % at end B, in both Thumbwheel Use modes, once calibrated |
| Thumbwheel Use | `select` | config | Dim End (default) / Input Only |
| Dim End | `number`, % of half-cycle | config | Used with Input Only, or an uncalibrated wheel. The default is the stock K in the wheel's starting position |
| Bright End | `number`, % of dim end | config | The default is the brightest the stock paddle reached |
| Fade Limit | `number`, counts per half-cycle | config, disabled | MCU smoothing; 0 = no limit |
| Add-on Thresholds | 4 x `number`, raw | config, disabled | UP and DOWN, relay drive off and on |
| Thermistor, Thumbwheel | `sensor`, raw | diagnostic, disabled | For calibration |

Raw sensors publish at most every few seconds, never at the status rate. The `rpstd01:` component
(id `mcu_link`) runs the link each loop on its UART and applies Thumbwheel Use, Dim End, Bright End,
Fade Limit and the add-on thresholds at start-up and on every change; lambdas reach the link
through `id(mcu_link)->link()`. The lamp and night light are `rpstd01` outputs; the component sets
the LEDs itself, and reaches the entities it needs through the ids given under `rpstd01:`.

MCU State is meant to be read. Automations should use MCU Update Available and MCU Problem, or
test only the start of MCU State.

## Link handling

All of this is in `rpstd01.h`, run once per main loop (about every 16 ms). The receive side
(parser, newest frames, link state, restarts) is `McuLink` in `rpstd01_link.h`, shared with the
MCU tool.

### Receiving

- Each loop feeds every available byte to a frame parser with the framing of
  `firmware/common/link.c`, shared with the MCU tool. A frame that turns out bad (a wrong CRC, a
  type or length it does not take) is scanned again from the byte after its start, so a false
  start (an `0xA5` inside a payload, or a frame cut off by an MCU reset) cannot hide the frames
  whose bytes it took.
- The parser also drops a frame at its type byte unless the MCU sends that type (`MSG_STATUS`,
  `MSG_INFO`, `MSG_BOOT_STATUS`), so a false start such as the thermistor's `0x03A5` ends at once.
  It takes any length, so that a release with frames of other lengths is still seen to answer
  ("Protocol check"). A false start that passes the type (night light 165 followed by a CRC byte
  that is a frame type, [mcu-firmware-design.md, "Link
  protocol"](mcu-firmware-design.md#link-protocol)) holds back the frames after it until its length
  has arrived, and they are then scanned again. This happens only when the parser joins inside a
  frame: in step, it takes whole frames.
- A part frame is given up after `RX_GAP_MS` without a byte, and the bytes after its start are
  scanned again. This covers a cut frame whose length is longer than all that follows it.
- `rx_buffer_size` in `rp-std01-device.yaml` holds enough status frames for ESPHome's longer pauses.
  An overflow only loses whole frames, each of which is complete in itself.
- **Status:** button and add-on processing runs for **every** frame, including a backlog
  after a pause, so a press and its release can reach the binary sensors in the same loop (see
  "Buttons").
- **Restarts:** the MCU restarted when the boot ID in a status differs from the last one seen, or
  when a status shows "starting" again after one without it ([mcu-firmware-design.md, "What the MCU
  states, so the ESP32 does not have to
  guess"](mcu-firmware-design.md#what-the-mcu-states-so-the-esp32-does-not-have-to-guess)). Lost
  frames do not count. The log counts these, the application's return from update mode included.
  After a restart the status shows the lamp and the outputs off, and the usual resends restore them
  (below).
- **Phase lock running on:** every 500 ms in which statuses showed `S_ZO_COASTED`, the log says,
  at debug level, in how many windows the MCU's timing ran on without a falling Zo edge near its
  prediction ([mcu-firmware-design.md, "Zo handling"](mcu-firmware-design.md#zo-handling)).
- **Info:** it carries the boot ID of the start that sent it. The link uses the last info frame
  whose boot ID matches the current status; one that arrives before its start's first status (the
  MCU sends it first) counts from that status on. Nothing asks for it.
- **Boot status:** the MCU is in our bootloader's update mode (see "MCU firmware").

| Seen | Meaning |
|---|---|
| A status within the last `APP_SEEN_MS` | Application |
| A boot status within the last `BOOT_SEEN_MS`, and no status | Update mode |
| Neither | No Response: MCU Problem turns on |

At start-up the link waits for the info frame ("Protocol check").

### Sending, with the status frame as acknowledgement

- **Gate (`MSG_GATE`):** while the status shows a half-period, the link clamps its wanted target
  itself, to the MCU's earliest firing point and `CLAMP_MARGIN` counts before its latest, so the
  MCU has nothing to clamp. The command counts as taken when the status shows that target and no
  "clamped" bit; otherwise it is resent at most every `RESEND_MS`. With Zo not good, the MCU reports
  target 0 with the "clamped" bit, and the link accepts that. The smoothing limits are not in the
  status; they go out with every gate frame, so a frame that changed only them and was lost is
  corrected with the next change of the target.
- **Outputs (`MSG_OUTPUTS`):** the status shows the LED bits and night light value in use. A change
  of the outputs (status LEDs, night light) is sent at once; while a status shows other values, they
  are sent again, at most every `RESEND_MS`. This also restores the outputs after an MCU restart, as
  the MCU starts with all outputs off.
- Nothing is refreshed while the status shows what was sent.
- At most one frame per loop (gate, then outputs, taking turns during a long transition), because
  the MCU's receive ring (`HAL_RX_RING`) is small. A newer target replaces one not yet sent.

### Silence and MCU resets

- While the MCU is silent (No Response), the link keeps its wanted state, but resends an
  unacknowledged gate or outputs frame only every `SILENT_RESEND_MS` instead of every
  `RESEND_MS`. Changes still go out at once.
- An MCU reset on its own (watchdog, or a brown-out the ESP32 survives) leaves the lamp and the
  outputs off. The status then shows target 0 against a non-zero wanted target, and outputs that
  differ, so the resends turn the lamp back on through the normal relay sequence and set the
  outputs again.

### ESP32 reboots keep the lamp

The MCU keeps its state when the ESP32 reboots, but the light state ESPHome restores can be up to
a minute old. So:

1. No gate command is sent until the first status frame and the info frame have arrived
   ("Protocol check").
2. **If the MCU is firing** (or starting), its state wins: the light is set on, without a
   transition, at the brightness that maps to the MCU's target. After a real power-on the MCU
   cannot be firing, so this is always an ESP32-only reboot.
3. **Otherwise** (off, stopping or in a fault), the reset reason decides: after OTA, crash or
   watchdog the light is set off; after a power-on or brown-out (`ESP_RST_POWERON`,
   `ESP_RST_BROWNOUT`) the light's `restore_mode` applies, then any power-on automation in the
   device file. The RESET button counts as a power-on. **[E1]**: the reset reason after a
   dip.

## Brightness to firing delay

### The curve

The light's output (0 to 1 after `gamma_correct`) becomes a firing delay in PCA counts:

1. **Dim end** `d_dim`: K from the thumbwheel (Dim End mode), or the Dim End number x the
   half-period. **Bright end** `d_bright` = Bright End x `d_dim`, at least the earliest firing
   point.
2. For a triac fired at angle `a = pi x d / H` (H, the half-period from the status) into a
   resistive load, the share of full power is `P(a) = 1 - a/pi + sin(2a) / (2 pi)`.
3. The wanted power for output `s` is `P_dim + s x (P_bright - P_dim)`.
4. The delay is the `a` that gives it, found by `CURVE_BISECTION_STEPS` bisection steps when an
   input changes, then clamped to the MCU's limits.

Output 0 is off (target 0). Until the status shows a half-period, the link uses the last one it
saw, or 60 Hz if none, and works the curve out again when the half-period changes by more than
`CURVE_HALF_CHANGE` counts.

At 60 Hz with the stock ends (K about 4.6 ms, brightest at about 0.47 x K, [../hardware/README.md,
"Stock firmware measurements"](../hardware/README.md#stock-firmware-measurements)):

| | Delay | Share of full power |
|---|---|---|
| Dim end (1.0 x K) | 4.60 ms | 40 % |
| Bright end (0.47 x K) | 2.16 ms | 90 % |
| Earliest firing point | 0.50 ms | 99.9 % |

So the stock ends give 40 to 90 % of full power, as stock. Toward end B (K = `K_B_MS`, limited by
the latest firing point) the dim end drops to about 3 %. Bright End can go down to the earliest
firing point for full brightness.

`gamma_correct: 1.0` is the default, so the slider sets power. A lamp's light rises faster than
its power, so a gamma above 1 could look more even; with 1.0 it looked even with both lamps tried
([../README.md, "Tested"](../README.md#tested)).

### Thumbwheel

The mapping from raw value to K is a straight line between two calibration points:

```
K = K_A_MS + (raw - raw_A) x (K_B_MS - K_A_MS) / (raw_B - raw_A)
```

`raw_A` and `raw_B` are the substitutions `thumbwheel_raw_a` and `thumbwheel_raw_b`, the raw
values at each end ([../hardware/README.md, "Stock firmware
measurements"](../hardware/README.md#stock-firmware-measurements)); the example device file has
them. Equal values mean "not calibrated": the Dim End number then sets the dim end, and the
Thumbwheel sensor has no value. Changes under `WHEEL_DEADBAND` counts are
ignored.

- **Dim End** (default, as stock): the wheel sets K, and so the lowest dimming level.
- **Input Only**: the wheel does not touch the lamp.

In both modes the Thumbwheel sensor publishes 0 to 100 % whenever it moves by 1 % or more, so the
wheel can drive anything, for example the night light:

```yaml
sensor:
  - id: !extend thumbwheel
    on_value:
      - light.turn_on:
          id: night_light
          brightness: !lambda return x / 100.0;
```

A device file that uses the wheel for something else should also set Thumbwheel Use to Input
Only.

### Transitions and the MCU's smoothing

ESPHome transitions change the target every loop (16 ms, about two half-cycles). The link sets
both MCU limits to the Fade Limit, so with the default a transition slower than about 0.4 s
between the default ends passes through unchanged, and a sudden change between them is spread over
at most 0.4 s. The stock limits (`LIMIT_BRIGHTER`, `LIMIT_DIMMER`) would take 0.8 to 1.2 s and
slow ESPHome's shorter transitions. The default transition (`default_transition_length` in
`rp-std01-core.yaml`) is slower than 0.4 s, so it passes through unchanged.

### On and off

- **On:** the MCU fires its held target, with no fade, as soon as its relay wait (`RELAY_SETTLE_MS`)
  ends. So until the status shows it firing, the link sends the **dim end**, then the wanted target,
  and the MCU fades up. This matches the stock "start dim and fade up", including after a power-on
  restore, an MCU reset, the return from update mode, and an HA turn-on with no transition.
- **Off:** the transition brings the target to the dim end, then the link sends 0, and the MCU
  stops firing and switches the relay off `RELAY_OFF_DELAY_MS` after the second half-cycle start
  since firing stopped.

### Faults

- When a **FAULT** starts, the link turns the light off once (so HA sees it) and sends target 0;
  MCU Faults, MCU Problem and B3 show it. The MCU returns to OFF by itself once the cause clears.
  The lamp does **not** come back on by itself.
- A turn-on during the FAULT is not undone: the light shows on while the lamp stays dark (the link
  sends target 0 while the MCU shows `FAULT`), and the lamp lights when the FAULT clears, unless
  the light is turned off first. MCU Problem and B3 say why it is dark. Turning the light off again
  and again would only fight an automation that turns it back on.
- The link reports no FAULT while the status shows the "starting" flag (the MCU's start-up FAULT,
  [mcu-firmware-design.md, "Relay sequence"](mcu-firmware-design.md#relay-sequence)): no fault
  event, MCU Problem stays off, B3 stays dark. Once the flag clears, a FAULT that is still there
  starts at once.
- A FAULT in the first status after a restart is a new FAULT, even if the last status before the
  restart was a FAULT too. The application's start after update mode is a restart like any other.
- Fault bit 4 ("target was clamped") only matters for the acknowledgement.

## Inputs

### Buttons

The link publishes a template `binary_sensor` per button from each status frame, inverted
(active low). A window in which a button changed counts as "no change yet", so bounces are
ignored; there is no filter in YAML on top of that. Gestures use `on_multi_click`, with the
timings in `rp-std01-core.yaml`. Double fires at the release of the second press, not while it is
still down: a press and its release can arrive in the same loop (a backlog after a pause), and a
timing that ends in "ON for at least" would see the release before its timer and cancel. After an
ESPHome pause longer than the hold time, a held button's press and release can arrive together and
count as a single press. Each gesture fires the button's `event` entity; the actions below come on
top.

The actions are in [../README.md, "Buttons"](../README.md#buttons). The WiFi button does not
switch the lamp. The paddles on the unit and on the add-on act the same. Hold brightens from 1 % if
the lamp is off, dims never below 1 %, and steps with `light.dim_relative` while the button is
held (the step and its interval are in `rp-std01-core.yaml`).

### Add-on

Each 10 ms window is classified from the Ks minimum:

1. Skip windows with no samples, with a relay-drive change, or within `ADDON_BLANK_FRAMES` windows
   of one. A change counts when the window's "changed" bit is set (the MCU sets it for a drive
   change during the window or after its last sample, and for a window with no samples) or the drive
   bit differs from the last frame's.
2. DOWN if the minimum is at or above `down_min`, else UP if at or above `up_min`, else idle,
   using the thresholds for the current drive state. The minimum ignores upward spikes.
3. A change counts once two judged windows in a row show it, so a single odd window is ignored.
   Taps (88 to 156 ms) span 9 to 15 windows. There is no filter in YAML.

The thresholds (the Add-on Threshold numbers, defaults in `rp-std01-core.yaml`) sit midway between
the levels measured at the MCU pin on one unit ([../hardware/README.md,
"Harness"](../hardware/README.md#harness)), and can be tuned without a rebuild.

## Thermistor and over-temperature

A template sensor turns the raw value into resistance (`therm_resistance_ohms()` in `rpstd01.h`):

```
R = THERM_PULLUP_OHMS x raw / (ADC_MAX - raw)
```

ESPHome's `ntc` sensor turns that into a temperature, with a B constant and a resistance at 25 C (in
`rp-std01-core.yaml`) that are assumed until the curve is measured **[E2]**. A raw value outside the
MCU's own limits (`THERM_OPEN_SHORT_LOW`, `THERM_OPEN_SHORT_HIGH`), where the MCU reports a
thermistor fault, gives no value rather than a wrong temperature. The ESP32 adds no over-temperature
limit of its own: the MCU's thresholds are already low ([mcu-firmware-design.md,
"Safety"](mcu-firmware-design.md#safety)). An earlier ESP32-side cut can be added in YAML once the
curve is known.

## LEDs and night light

The component sets the bits and values of the link's outputs command.

- **Brightness bar** (LED1 to LED5): on Level, one LED per fifth of the brightness, at least one,
  while the lamp is on; all off while it is off. On Off, the bar stays dark.
- **B3:** on during a reported MCU fault (not the start-up FAULT). It is set blinking (1 s) while
  the MCU does not run its application, but in update mode the blink never shows (no outputs frame
  is sent, and the bootloader does not drive the LEDs); with No Response it shows only if the MCU
  still receives.
- **Night light** (B2): a monochromatic light, 0 to 255 on the MCU. In On When Lamp Off mode it
  turns on (at its last brightness, default 30 %) when the light turns off, and off when it turns
  on, also during a FAULT. In Light mode only HA and the WiFi button hold change it.

## MCU firmware

### The expected build

`make firmware` writes the version, build ID and bootloader version of the image the MCU tool
carries into `rpstd01_fw_constants.h` (`MCU_IMAGE_VERSION`, `MCU_IMAGE_BUILD`,
`MCU_IMAGE_BOOT_VERSION`). This is the build the configuration was made and tested with.
The comparison is the one the MCU tool makes (`firmware_age()` in `rpstd01_link.h`): a higher
bootloader version or firmware version on the MCU is newer; otherwise an older bootloader or
another build is an update. **MCU Update Available** is on for an update, and MCU State then ends
in "(update available: use the MCU tool)"; for newer firmware it is off, and MCU State ends in
"(MCU firmware newer than this configuration)". The log says once which firmware the MCU runs and
which is expected. The lamp works as usual, as long as the info frame shows the link protocol this
configuration was built for.

### Protocol check

The high byte of the MCU's firmware version is its link protocol. This configuration supports **only
the protocol in its own `FW_VERSION`**. It reads a status only at exactly `STATUS_LEN` bytes, an
info frame only at exactly `INFO_LEN`, and a boot status only at exactly `BOOT_STATUS_LEN`.

- An info frame that shows another protocol, or a status, info frame or boot status of another
  length within the last `UNREADABLE_SEEN_MS`: the link sends no gate and no outputs frames (it
  still reads what comes), MCU Problem is on, and MCU State shows "MCU firmware needs the MCU
  tool". The lengths catch most changes of layout; the protocol covers a change of meaning that
  keeps the lengths.
- The protocol is known from the info frame, so no gate or outputs frame goes out before it, also
  when the ESP32 starts with the MCU already running; meanwhile MCU Problem is on. If it never comes
  (a firmware that sends statuses but no info frame), the lamp is not driven at all and MCU Problem
  stays on.

### Update mode

The MCU is in update mode when the link sees boot statuses: the MCU tool asked for it, an
application update was cut off, or the application kept crashing. MCU State then shows "Update mode
(use the MCU tool)", MCU Problem is on, and the link **sends nothing at all**, not even the gate
frame, until a status frame arrives again. Changes to the lamp and LEDs are kept and sent then. When
the application returns without its info frame (lost on the way), the link waits for the next one.

Our bootloader takes only its own commands and drops any other frame at its type byte, so gate and
outputs frames would only be lost; they wait until the application is back. Only the bootloader's
own commands restart its idle time, so an MCU left in update mode with a valid application (for
example when the main configuration is put back right after the tool requested an update) runs its
application again `IDLE_RETURN_MS` after the last command, and the lamp comes back through the
usual resends. An MCU with no valid application, or one whose application kept crashing, stays in
update mode until the MCU tool writes it.

## Use from other configurations

A larger device file can include the configuration as a package. The light has an `id` and
`restore_mode: RESTORE_DEFAULT_ON`. Power-on and "HA lost" logic, and `api: reboot_timeout`,
belong to the device file. The link's start-up rule only overrides the restored state when the
MCU is firing or after a soft reboot, so it does not conflict with a power-on rule.

## Testing

`rpstd01.h` and `rpstd01_link.h` have only the UART behind an interface, so g++ unit tests (`make
test`) run them directly, also against the simulated MCU, which runs the firmware's own
application loop and bootloader ([mcu-firmware-design.md,
"Testing"](mcu-firmware-design.md#testing)). The host tests (`make test-host`) run
`rp-std01-core.yaml` on ESPHome's `host` platform, with its UART on a pseudo-terminal to the same
simulated MCU in real time, and drive it through ESPHome's native API while a control socket
presses buttons, changes inputs, corrupts frames and cuts power.

## Assumptions to check

| # | Assumption | Status |
|---|---|---|
| E1 | The ESP32 reports something other than `ESP_RST_POWERON` after OTA, crash and watchdog, and `ESP_RST_POWERON` or `ESP_RST_BROWNOUT` after a mains dip | Power-on after a real mains cut confirmed; the rest not checked |
| E2 | The `ntc` sensor's B constant and resistance at 25 C | Needs a thermometer |
