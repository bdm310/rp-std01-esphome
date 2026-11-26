# MCU firmware design

Design of the replacement firmware for the RP-STD01's MCU, an STC IAP15W413AS (`firmware/`).
Hardware facts are in [../hardware/README.md](../hardware/README.md), including the signal names
used here (Zo, Ks, SCR, B2, B3; [MCU pin map](../hardware/README.md#mcu-pin-map) and
[Harness](../hardware/README.md#harness)); test results are in
[../README.md](../README.md#tested). Section numbers refer to STC's English STC15 datasheet (2016).
Names in capitals are constants of the firmware, most of them in `firmware/include/`.

## The split

**The MCU is as simple as possible.** It does only what needs microsecond timing or must keep
working without the ESP32: the gate, the relay sequence, and the safety cut-offs. Everything else
runs in ESPHome ([esphome-config-design.md](esphome-config-design.md)), where it is easy to change
and update.

| Job | MCU | ESP32 (ESPHome) |
|---|---|---|
| Zo edges, mains period | yes | sees the period in the status frame |
| Gate timing (fire, release) | yes | sends the firing delay |
| Smoothing delay changes (limit per half-cycle) | yes | sets the limits |
| Relay sequence around the gate | yes | - |
| Over-temperature, Zo loss, gate clamps, watchdog | yes | can add stricter limits |
| Brightness -> firing delay, dimming curve, trims | - | yes |
| Fades and transitions | - | yes (ESPHome light transitions) |
| Thumbwheel meaning | reads raw ADC | yes |
| Buttons: debounce, gestures, actions | reads pin levels | yes |
| Add-on: UP/DOWN levels | reads Ks raw ADC | yes |
| LEDs, night light | drives the pins | yes, decides what they show |
| Thermistor curve, temperature | reads raw ADC, trips at a fixed raw value | yes (`ntc` sensor) |
| Firmware updates | bootloader writes flash | yes, the MCU tool sends the image ([mcu-tool.md](mcu-tool.md)) |

The chips talk over a small binary protocol of our own ("Link protocol"), not Tuya.

## Decisions

1. **Our own bootloader** handles updates. STC's built-in bootloader is only for the first flash
   and recovery.
2. **No local fallback.** The MCU never acts on the buttons itself; while the ESP32 is rebooting
   or crashed, they do nothing.
3. **On link loss, everything keeps its state**, so an ESP32 reboot (for example an OTA update)
   does not change the lamp. The safety cut-offs still work.
4. **Lamp off after power-on** until the ESP32 sends a gate command.

## Toolchain and build

**SDCC**, with the options in `SDCC_FLAGS` in the Makefile. Requirements and commands are in
[development.md](development.md).

- SDCC functions are not reentrant by default (locals and parameters at fixed RAM addresses). So a
  function reachable from more than one context (the main loop, low-priority interrupts,
  high-priority interrupts; interrupts of the same priority count as one) must keep no local or
  parameter at a fixed address, which `REENTRANT` guarantees. Files with interrupt-called functions
  are compiled with `--nooverlay` (`ISR_SRC` in the Makefile). `make firmware` checks both from
  SDCC's output (`tools/isr_overlay.py`). Interrupt code uses no multiply or divide library calls.
- Interrupts are masked by clearing the enable bit **twice** (`IRQ_OFF_*()` in `app/hal.h`): on
  a 1T 8051 a pending interrupt can still run after the next instruction (SiLabs documents this
  for its 1T core). `hal_ms()` instead reads the tick until two reads agree.
- Ordinary 3-byte pointers make large code, so structs are passed by 1-byte `__idata`
  (interrupt-time state) or 2-byte `__xdata` (main-loop state) pointers.
- `tools/mkesphome.py` (run by `make firmware`) pads the application, adds its trailer and merges
  both programs into the first-flash image. It also makes the **build ID**: the first 4 bytes of a
  SHA-256 over the linked application area (without its trailer) and the version. Any change to the
  application binary gives a new ID, whatever caused it (sources, options in the Makefile, SDCC
  version, the script), and the same binary always gives the same ID. The application reads the ID
  from its own trailer.
- The build ID leaves the bootloader out on purpose. An update over the link rewrites only the
  application, so if the ID also followed the bootloader, a change to the bootloader alone would
  make the MCU tool rewrite identical application code over the link (and turn the lamp off for
  it), and leave the old bootloader in place. The bootloader is identified by its version
  (`BOOT_VERSION`, in the boot status and in the last byte of its region, see "Flash"). `make
  check` makes sure a changed bootloader raises `BOOT_VERSION` and a changed application
  `FW_VERSION` ([development.md](development.md#commands)). The MCU tool compares both:
  a different build ID with the carried bootloader is an update over the link, an older bootloader
  an install through STC's bootloader ([mcu-tool.md](mcu-tool.md)).
- **`FW_VERSION`** (`include/board.h`) has two parts. The high byte is the **link protocol
  version**: it is raised for any change to the link that an older ESP32 side would get wrong (the
  layout or meaning of a frame), and the low byte then starts again at 0. The low byte is the
  **release**, raised for every MCU release. For example, 0x0203 is protocol 2, release 3.
- The same script writes the two committed headers ESPHome builds from ([development.md, "What
  `make firmware` writes"](development.md#what-make-firmware-writes)).
- Register access stays in `app/hal.c`, `app/main.c`, `boot/boot_hal.c` and `boot/boot_main.c`;
  the rest builds with `gcc` for the host tests. `main.c` and `boot_main.c` hold only the
  start-up and the loop. The loops themselves are `app_loop()` in `app/app.c` (the received bytes
  until the tick advances, then that tick's work) and `boot_loop()` in `boot/boot.c` (update mode),
  with `boot_reason()` for the start-up decision. They reach the hardware only through `app/hal.h`,
  the `boot_hal_*` functions in `boot/boot.h` and the flash and watchdog functions in
  `boot/update.h`, which the simulator also implements (Testing).

## Memory

### Flash

13 KB: 26 sectors of 512 bytes, 0x0000 to 0x33FF (sections 9.1, 9.2). An erase stops the CPU for
about 21 ms and a byte write for about 55 us, with no interrupts, so **only the bootloader writes
flash**, and only with the gate and relay off.

| From | Up to | Use |
|---|---|---|
| 0x0000 | `APP_BASE` | **Bootloader**, with the reset and interrupt vectors. Its version in the last byte, `BOOT_VERSION_ADDR` |
| `APP_BASE` | `APP_END` | **Application**. Trailer in the last `TRAILER_SIZE` bytes |
| `APP_END` | end of flash | Unused, clear of the unique-ID bytes other STC15 parts keep at the end (section 1.12) |

- The bootloader can never be changed over the link, so its region keeps room to spare.
- Section 9.1 says IAP writes fail if any of `IAP_ADDRH[7:5]` is set, which would stop at 0x2000,
  but section 9.2 gives 0x0000-0x33FF for this part. Writes above 0x2000 work.
- The bootloader refuses to erase or write outside the application area.
- **Bootloader version byte:** the bootloader keeps `BOOT_VERSION` at `BOOT_VERSION_ADDR`, a
  constant placed there by the linker (`boot_version` in `boot/boot_main.c`). The application reads
  it (`MOVC`) for its info frame, so the ESP32 can learn the bootloader version without sending the
  MCU into update mode. The byte is outside the application area, so an update never changes it, and
  it changes only when STC's bootloader writes a new bootloader. `mkesphome.py` refuses a bootloader
  without this byte. The byte counts towards the bootloader's `--code-size` limit, which keeps the
  code below it.

### RAM

256 bytes internal plus 256 bytes auxiliary (`__xdata`, section 3.2).

- SDCC's start-up clears all internal RAM but only each program's own auxiliary RAM. Both programs
  are linked with `--xram-size XRAM_SIZE`, so neither uses anything from there up: the hand-over
  bytes sit at `HO_ADDR` and the boot ID at `BOOT_ID_ADDR` (see [What the MCU
  states](#what-the-mcu-states-so-the-esp32-does-not-have-to-guess)). The linker enforces the limit;
  an `#error` in `flashmap.h` keeps those two placements (`__at`) above it and apart.
- Application: direct RAM is all but full, so the UART rings, the receive buffer, the frame
  buffer and the main-loop state are in auxiliary RAM, which leaves the stack its room. The
  worst-case stack (the main loop's deepest path, plus the deepest path of an interrupt at each of
  the two priority levels) was counted from the generated code, and a test build on both units
  that filled the stack at start-up and reported its high-water mark confirmed a good margin. The
  `.map` files show what each program uses.
- Bootloader: the auxiliary RAM holds one frame of up to `BOOT_MAX_PAYLOAD` and a `BOOT_RX_RING`
  receive ring.

## Clock and timers

- **System clock:** internal RC at `SYSCLK_HZ` (11.0592 MHz, `stcgal -t 11059`), +/-1 % over
  temperature (section 1.5.1). The mains period is measured with the same clock, so the error
  mostly cancels.
- **PCA counter:** SYSclk/12 (`PCA_HZ`), **1.085 us** per count, 16 bits, wraps every 71.1 ms.
  A 60 Hz half-cycle is **7680 counts**.
  - Module 0 -> P3.5 (SCR): gate, high-speed output mode.
  - Module 1: unused; its pin P3.6 stays an input.
  - Module 2 -> P3.7 (B2): night light, 8-bit PWM at 3.6 kHz.
  - `P_SW1.CCP_S1:CCP_S0 = 01` (section 1.5.3), written explicitly because the datasheet gives
    two reset values for `P_SW1`.
- **Timer 0:** 1 ms tick, 1T, reload `T0_RELOAD_1MS` (`include/stc15.h`). Its clock output
  T0CLKO is on the gate pin, so `INT_CLKO` is cleared at start-up.
- **Timer 2:** UART baud clock, 1T, reload `T2_RELOAD_BAUD` -> exactly `LINK_BAUD` (section 8.3;
  the build fails if the clock does not divide exactly). There is no Timer 1 on this series.
- **UART1** on P3.0/P3.1, 8N1.
- **ADC:** 540 clocks per conversion (49 us), one per 1 ms tick, read on the next.

| Interrupt | Priority | Work |
|---|---|---|
| INT0 (Zo, both edges) | high | Time-stamp the edge, schedule the gate |
| PCA module 0 match | high | Re-arm for the release, then disarm |
| Timer 0 (1 ms) | low | Tick, ADC, input sampling, safety checks |
| UART1 | low | Bytes in and out |

## Gate timing

### Method

In high-speed output mode the PCA flips the pin when the counter equals the compare value
(section 11.3.3), so both gate edges come from hardware, accurate to one count. Each half-cycle:

1. **Zo edge interrupt** (for an accepted edge): force P3.5 low and disarm module 0. Work out
   `t_fire = reference + delay`. If it is at least `SCHED_MARGIN` counts after the edge (always,
   for a valid delay), write `CCAP0L` then `CCAP0H` (section 11.3.2) and arm the module.
2. **Fire match:** hardware sets P3.5 high. The interrupt checks it and writes `t_release`.
3. **Release match:** hardware sets P3.5 low. The interrupt also writes it low and disarms the
   module, so the latch cannot flip again when the counter wraps.

The release time is loaded by the PCA interrupt at the fire match, so the hardware alone does
**not** release the gate. If a bug blocks interrupts after a fire match, the compare value stays at
the fire time and the gate pin flips at each counter wrap: high for about 71 ms, low for about 71
ms, and so on. The 1 ms tick then stops too, so the main loop no longer clears the watchdog, which
resets the MCU within `WDT_TIMEOUT_MS`; the reset vector drives the gate and the relay low. While
interrupts run, the 1 ms tick forces P3.5 low outside the firing window, and the watchdog catches a
stuck main loop.

### Times

| Item | Value | At 60 Hz | Source |
|---|---|---|---|
| Half-cycle | measured half-period | 8.333 ms | measured |
| Gate release | half-period - `RELEASE_BEFORE_EDGE` after the reference | about 7.40 ms | stock |
| Latest firing point | half-period - `LATEST_BEFORE_EDGE` (release - 0.40 ms) | 7.00 ms | our margin |
| Earliest firing point | `EARLIEST_FIRE` | 0.50 ms | our margin; stock went to 0.40 ms |
| Relay on -> first gate | `RELAY_SETTLE_MS`, then the next half-cycle | | stock |
| Target 0 -> relay off | `RELAY_OFF_DELAY_MS` after the second Zo edge since firing stopped | | stock: about 1 ms after the next edge |
| Fault -> relay off | as for target 0, but at most `FAULT_RELAY_MS` after the fault | | ours |

The release follows the **measured** half-period, so it stays about 0.93 ms before the next
crossing whatever the clock error. Every delay is clamped to the firing range, so the stock bug of
firing after the release point cannot happen.

### Delay smoothing

The MCU moves its current delay toward the target by at most a set number of counts per
half-cycle, separately for brighter and dimmer; the ESP32 sets both limits (0 = no limit). The
MCU's defaults (`LIMIT_BRIGHTER`, `LIMIT_DIMMER`) copy the stock fade; the ESPHome side sends its
own. It exists because ESPHome updates the target only every loop (about 16 ms, two half-cycles).

### Jitter

Under 2 us normally, a tenth of a stock fade step: one PCA count (1.09 us), under 1 us from Zo
edge to counter read, and the short main-loop sections with interrupts off. Up to about 13 us when
a Zo edge lands in the 1 ms tick's masked gate check (about 1 % of edges). The Zo interrupt keeps
its own valid byte, so the main loop only copies plain bytes with interrupts off. A PCA interrupt
cannot delay the Zo interrupt, because gate events are at least about 0.45 ms from any Zo edge.
No noise edges on Zo were seen in many minutes on mains.

## Zo handling

- The interrupt reads the PCA counter (high, low, high again to catch a carry), then P3.2 for the
  edge direction.
- **Noise:** an edge is ignored if it has the same level as the last accepted one, if it comes
  less than `EDGE_MIN` after it, or, once locked, more than `LOCK_WINDOW` before the expected
  crossing. A gap of more than `EDGE_MAX` restarts the count of good edges.
- **Period:** rising edge to rising edge, filtered by 1/8 per new value (with fractional bits).
  Only 45 to 65 Hz (`PERIOD_MIN` to `PERIOD_MAX`) is accepted.
- **Reference (as stock):** a rising edge is its own reference; a falling-edge half-cycle uses
  `last rising edge + period / 2`. This reproduces the stock firmware's 40 to 50 us later firing
  in those half-cycles.
- **Zo lost:** after more than `ZO_LOST_MS` with no edge, Zo is not good, and the 1 ms tick drops
  the phase lock entirely: after such a gap the 16-bit edge timestamps cannot be trusted, because a
  gap of a multiple of the counter's period looks like a normal half-cycle. While the relay drive is
  on, Zo not good is a fault ("Relay sequence"). With the relay drive off it only shows in the
  status (`F_ZO`).
- **Start-up, and after a gap:** no gate until `ZO_GOOD_EDGES` good edges in a row (about
  0.13 s).
- In the gate simulation, noise near an edge moved the release by up to about 20 us; the gate
  still stayed low for more than 0.8 ms before every crossing.

## Relay sequence

The ESP32 never switches the relay. The MCU ties it to the delay, so the relay never switches
current:

| State | Relay drive | Gate | Leaves when |
|---|---|---|---|
| `OFF` | off | off | Target not 0, Zo good, no fault -> `SETTLE` |
| `SETTLE` | on | off | Target 0 -> `OFF`; `RELAY_SETTLE_MS` passed -> `FIRING`, current delay = target |
| `FIRING` | on | firing | Target 0 -> `STOPPING` |
| `STOPPING` | on | off | `RELAY_OFF_DELAY_MS` after the second Zo edge since target 0 -> `OFF` |
| `FAULT` | on, then off | off | Relay off, cause gone **and** a target of 0 seen -> `OFF` |

"A target of 0 seen" means the target was 0 when the fault started, or a target of 0 arrived since;
a non-zero target that arrives during the `FAULT` cancels it.

**The start-up FAULT** is the one exception to "a target of 0 seen". From its first tick
until the thermistor average is ready (about 80 ms) the application is in `FAULT`, because the
safety check reports a thermistor fault. When the average is ready this FAULT ends at once, also
if a non-zero target arrived meanwhile (for example a copy the ESP32 sent just before the
restart), as long as no other fault was seen since the start and none is present then
(`gate_startup_done()`). The relay was never on, and the target then starts the lamp through the
usual sequence. Any other FAULT, including one present when the average becomes ready, still
needs a target of 0.

When the contacts actually move is not known (A3); `RELAY_SETTLE_MS` is copied from the stock
firmware.

**Why the second edge.** The main loop reads the Zo edge count, then clears the delay. A Zo edge
in between still arms the gate with the old delay, and the triac then conducts until the next
crossing. Waiting for the second edge after target 0 covers that half-cycle.

**Turning off after a fault.** Over-temperature, a thermistor fault, a gate fault, or Zo lost or
not good while the relay drive is on, all enter `FAULT`:

- The delay is cleared in the same tick, and it is never set again while in `FAULT`, so no new
  gate pulse is armed. Only a pulse armed just before, as above, can still fire.
- If firing had started (`FIRING`, `STOPPING`), the relay opens as after target 0, but at the latest
  `FAULT_RELAY_MS` after the fault. The time limit is needed when Zo edges have stopped; it is
  longer than one half-cycle at 45 Hz (11.1 ms), so a pulse fired before the fault has ended by
  then. This avoids opening the contacts under current, which wears them.
- From `OFF` or `SETTLE` nothing has fired, and the relay opens at once.
- `FAULT` is left only with the relay open, so every fault ends with the relay open within
  `FAULT_RELAY_MS`. The watchdog does not depend on this: a watchdog reset drives both pins low at
  once.

## Inputs

The MCU reports; the ESP32 decides.

- **Buttons** (WiFi P2.7, paddle UP P2.0 and DOWN P3.3, active low): read every 1 ms. Each status
  frame carries each button's level and whether it changed during the 10 ms window.
- **Ks (add-on), P1.1/ADC1:** sampled every 2 ms. Each status frame carries the minimum over its
  window, and whether its samples were all taken with the relay drive as it is now (the idle level
  follows the drive). The minimum ignores a single sample that jumps up, so a level counts only once
  it holds for the whole window. Measured levels are in [../hardware/README.md,
  "Harness"](../hardware/README.md#harness).
- **Thumbwheel** (P1.3/ADC3) and **thermistor** (P1.2/ADC2): read every 10 ms, averaged over 8
  readings, sent raw.

ADC slots per 10 ms: Ks on even ticks; thumbwheel and thermistor on two of the odd ones.

## Outputs

- **LED1 to LED5 and B3:** on/off as the ESP32 sets them. The MCU inverts them (active low), so
  "1" means lit.
- **B2, night light:** PWM 0 to 255. The PCA output is low for `CCAP2L`/256 of each period, so
  with the active-low LED the value is the brightness.
- On link loss, outputs keep their last state.

## Safety

These stay on the MCU so they work even if the ESP32 is wrong, crashed or rebooting.

| Rule | How |
|---|---|
| SCR and relay drive low at reset | The reset vector clears P3.5 and P3.4 and makes them push-pull before C start-up (`boot/vectors.asm`). After reset the pins are weak pull-ups (section 4.1); whether the power board pulls them low is not known (A5). |
| Nothing runs on from a stray jump to 0x0000 | A real reset stops interrupts and the PCA; a jump to the reset vector would not. So the vector first clears `EA`, `CCAPM0` (before it clears P3.5, so module 0 cannot toggle it again) and the PCA run bit `CR`, and interrupts stay off until `hal_init()`. The bootloader uses neither; the application sets both up again. If the jump came from inside an interrupt, its priority level stays marked as in service, which blocks the 1 ms tick, so the watchdog resets the MCU within `WDT_TIMEOUT_MS`. |
| No gate at or after the release point | Hardware release; delays clamped; every accepted Zo edge forces the gate low and disarms; the 1 ms tick forces it low outside the window. |
| Gate off if Zo stops | No edge for more than `ZO_LOST_MS` while the relay drive is on -> `FAULT`. |
| Relay off after any fault | "Turning off after a fault", under "Relay sequence". |
| Over-temperature | Fixed raw thresholds, below. Not settable over the link. |
| Thermistor broken | Reading under `THERM_OPEN_SHORT_LOW` or over `THERM_OPEN_SHORT_HIGH` -> `FAULT`. |
| Watchdog | `WDT_CONTR = WDT_RUN_CLEAR`: prescale `WDT_PRESCALE`, timeout `WDT_TIMEOUT_MS` (section 2.2.6). Cleared in the main loop only while the 1 ms tick advances. |
| No flash writes while the lamp can run | Only the bootloader can erase or write flash, and it holds SCR and the relay drive low. |
| Gate fault | P3.5 reads back low at the fire match, or high outside the window, or the gate is still armed `RELEASE_OVERDUE` past its release time -> `FAULT`. |
| T0CLKO on P3.5 | `INT_CLKO = 0` at start-up. |
| Low supply | Low-voltage reset stays on (factory option). |

**Over-temperature thresholds.** The thermistor reading fits a 100 kohm NTC
([../hardware/README.md, "Harness"](../hardware/README.md#harness)); hotter reads lower. Assuming
B = 3950, and deliberately low until the curve is measured (A2):

- trip at `THERM_TRIP` or less (about 60 C), clear at `THERM_CLEAR` or more (about 50 C);
- below `THERM_OPEN_SHORT_LOW` (short) or above `THERM_OPEN_SHORT_HIGH` (open, or below about
  -7 C) is a thermistor fault.

Once the curve is measured, the thresholds become reviewed build constants.

## Link protocol

Both directions use the same frame:

| Byte | Content |
|---|---|
| 0 | `FRAME_START` (`0xA5`) |
| 1 | Type (`MSG_*`) |
| 2 | Payload length (to the application: up to `APP_MAX_PAYLOAD`; to the bootloader: up to `BOOT_MAX_PAYLOAD`) |
| 3 ... | Payload |
| last 2 | CRC-16/CCITT (initial 0xFFFF) over type, length and payload, low byte first |

16-bit values are little-endian. A frame with a type the receiver does not take, a length over its
limit, or a bad CRC is dropped, and the receiver looks for the next `0xA5`, starting with the byte
that ended the bad frame. A `0xA5` inside a payload can look like a frame start, and its false frame
can swallow the start of the real frame after it, which is then lost too. Nothing depends on a
single frame:

- ESP32 to MCU: the status is the acknowledgement. The ESP32 sends a gate or outputs frame again,
  at most every `RESEND_MS` (ESP32 side), until a status shows what it sent.
- MCU to ESP32: a status every `STATUS_PERIOD_MS`, each complete in itself, and the info frame
  (below). A lost one only delays things; the button levels are in every status.

A false start must not come back in every frame, though, or the receiver never gets back in step.
Two rules prevent that:

- **Type check.** Every receiver drops a frame at its type byte unless it takes that type, and
  looks for a start again from that byte. The application takes `MSG_GATE` to `MSG_UPDATE`, the
  bootloader `MSG_BOOT_BEGIN` to `MSG_BOOT_RUN` (each sets the range with `link_rx_types()`), and
  the ESP32 `MSG_STATUS`, `MSG_INFO` and `MSG_BOOT_STATUS`. Without it, identical frames back to
  back can lock a receiver out for good. For example, a gate frame for target 256 with limits 220
  and 0 is `A5 01 04 00 01 DC 00 CC A5`. Its last byte, the CRC's high byte, looks like a start,
  the next frame's real `0xA5` is taken as its type, and so on in every frame.

  With the type check, identical frames back to back with no gap get back in step by themselves
  after any desync, as long as the frame holds no `0xA5` followed by an accepted type except at
  its start. The false frame left by the desync ends within the receiver's payload limit plus 3
  bytes, and every frame that starts after that arrives (`make test` checks this).
- **Receive timeout.** A frame that does hold such a pair can still lock the receiver out when
  frames come back to back. For example a gate frame for target 0x02A5 (677) with a limit of 3
  starts a false frame of type 2 at its target, which ends inside the next frame just before that
  frame's own false start. So the MCU also drops a partly received frame after a pause:
  `APP_RX_TIMEOUT_MS` and `BOOT_RX_TIMEOUT_MS` count whole 1 ms ticks without a byte, so a part
  frame is dropped after a gap of that many ms, or up to 1 ms more, depending on where in its tick
  the last byte came. The ESP32 writes each frame to the application in one go, and at most one per
  loop (about 16 ms), so a false frame that runs past the end of its frame is dropped before the
  next one (which may be an identical resend) arrives.

The work per received byte is small and bounded, which matters because it runs in the main loop:
one parser step with at most one CRC step (about 22 us; the CRC step itself is about 70 clocks,
6 us), and at most one complete frame to handle, which sends nothing: an info request only marks
the info frame as wanted.

- The main loop takes received bytes only until the 1 ms tick advances, then does the tick work.
  So the safety checks, the relay sequence and the watchdog clear run at least once per tick plus
  at most one byte's work, whatever arrives.
- The receive ring (`HAL_RX_RING`, one byte less usable) holds about 2.7 ms of bytes at
  `LINK_BAUD`. The longest gap in reading it is the tick that builds and sends the status, at most
  about 0.8 ms. After the one at start-up, an info frame is only sent in the same tick, behind the
  status, which adds about 0.3 ms: about 1.1 ms, or about 1.3 ms with the receive interrupt busy,
  well under what the ring holds. A byte takes 87 us to arrive, longer than the work for it, so the
  main loop catches up after each gap.

The bootloader uses the same receiver (`common/link.c`) with one frame in flight, and the ESP32
sends a lost frame again after waiting for its reply. Its receive timeout, `BOOT_RX_TIMEOUT_MS`, is
longer than the application's: a 135-byte write takes 12 ms on the wire, and a busy ESP32 can pause
inside it while it refills its UART FIFO, so `APP_RX_TIMEOUT_MS` would cut good frames. A lost frame
gets no reply, and the ESP32 waits longer than `BOOT_RX_TIMEOUT_MS` for one before it sends again,
so a false start is still dropped before the resend arrives. Each write is read back, and Finish
also checks the CRC in the image trailer, which covers the code (as many bytes from the start of
the application area as the trailer's code length gives), not the trailer itself.

**ESP32 -> MCU:**

| Type | Payload | Meaning |
|---|---|---|
| `MSG_GATE` | target delay (u16, PCA counts; 0 = off), limit brighter (u8), limit dimmer (u8) | Set the firing delay, from the next half-cycle |
| `MSG_OUTPUTS` | LED bits (u8: bits 0-4 = LED1-LED5, bit 5 = B3), night light (u8) | Set the LEDs |
| `MSG_INFO_REQ` | - | Ask for the info frame (the MCU tool uses it to get an answer at once) |
| `MSG_UPDATE` | `UPDATE_KEY0` to `UPDATE_KEY3` | Turn off, then restart into update mode ("Updates") |

A frame whose payload length is not its type's (`GATE_LEN`, `OUTPUTS_LEN`, none for the info
request, `UPDATE_LEN`) is ignored.

**MCU -> ESP32:**

| Type | Payload | When |
|---|---|---|
| `MSG_STATUS` | `STATUS_LEN` bytes, below | every `STATUS_PERIOD_MS` |
| `MSG_INFO` | `INFO_LEN` bytes, below | at each start; then every `INFO_PERIOD_MS` and on request, right behind the next status |

Info payload:

| Byte | Field | Size |
|---|---|---|
| 0 | Firmware version (`FW_VERSION`: protocol version in the high byte, release in the low byte) | u16 |
| 2 | Build ID | u32 |
| 6 | Reset cause: 1 power-on, 2 watchdog, 3 other | u8 |
| 7 | Watchdog reset count | u8 |
| 8 | Bootloader version, read from `BOOT_VERSION_ADDR` | u8 |
| 9 | Boot ID of the start that sends it | u8 |

Status payload:

| Offset | Field | Size |
|---|---|---|
| `SF_STATE` | State (`ST_*`) | u8 |
| `SF_FAULTS` | Fault bits (`F_*`): over-temperature, thermistor, Zo not good, gate fault seen, target was clamped | u8 |
| `SF_BUTTONS` | Buttons (`BTN_*`), raw pin levels: the last level of each, and from `BTN_CHANGED_SHIFT` up, whether it changed during the window | u8 |
| `SF_TARGET` | Target delay in use (after clamping) | u16 |
| `SF_CURRENT` | Current delay (0 unless firing) | u16 |
| `SF_HALF` | Half-period, PCA counts (0 = Zo not good) | u16 |
| `SF_KS_MIN` | Ks minimum (raw 0-1023; `KS_NO_SAMPLE` if none) | u16 |
| `SF_FLAGS` | `S_RELAY`: relay drive on now. `S_KS_STALE`: the Ks minimum of this window may not belong to that drive state, because the drive changed during the window or after its last Ks sample, or because the window has no Ks sample. `S_STARTING`: the application has just started and its thermistor average is not ready yet | u8 |
| `SF_THERM` | Thermistor raw, 8-reading average | u16 |
| `SF_WHEEL` | Thumbwheel raw, 8-reading average | u16 |
| `SF_BOOT_ID` | Boot ID | u8 |
| `SF_LEDS` | LED bits in use (as in the outputs frame) | u8 |
| `SF_NIGHT` | Night light value in use | u8 |

The status uses about 21 % of the link (24 bytes every 10 ms). It **is** the acknowledgement: it
shows the target and the outputs in use, so the ESP32 can resend a lost or rejected command.
Delays are in PCA counts, the same unit as the half-period, so the ESP32 can work in fractions of
a half-cycle at any clock error or mains frequency.

How the ESP32 side treats frames of other lengths, or another protocol, is in
[esphome-config-design.md, "Protocol check"](esphome-config-design.md#protocol-check).

### What the MCU states, so the ESP32 does not have to guess

- **Boot ID: did the MCU restart?** A byte in auxiliary RAM at `BOOT_ID_ADDR`, next to the
  hand-over bytes, that the application raises by 1 at each start (`hal_init()`). It is in every
  status and info frame. A boot ID that differs from the last status means a restart; lost frames
  do not matter. A restart can keep the boot ID only when the RAM does not survive it (below).
  - `BOOT_ID_ADDR` is above both programs' auxiliary RAM (`XRAM_SIZE`), and nothing else writes
    it: the hand-over code writes only its `HO_SIZE` bytes at `HO_ADDR`, and the bootloader never
    touches the boot ID.
  - It keeps its value through a software reset (an update, or update mode returning to the
    application), and through a watchdog reset if assumption A1 holds. If A1 does not hold, the
    byte holds whatever the RAM holds after the reset. If that is random, the new boot ID differs
    from the last one 255 times in 256. If the RAM comes back in a fixed state (for example all
    zero), every such start gets the same boot ID, and only the first is seen from it. The
    starting flag (below) still shows each start, as long as one of its statuses arrives, and the
    ESP32 side counts a status that shows it again as a restart too.
  - After a power-on it holds whatever the RAM holds. That does not matter: a mains power-on
    restarts the ESP32 too, and it starts from scratch.
- **Starting flag: is a FAULT only the start-up FAULT?** `S_STARTING` is set for exactly as long
  as the start-up FAULT ("Relay sequence"), so a FAULT shown after it clears is a real one.
- **Outputs echo: did the MCU take the outputs?** `SF_LEDS` and `SF_NIGHT` show the LED bits and
  night light value in use. The application starts with all outputs off.
- **Which firmware runs?** The info frame carries the boot ID of its start, so the ESP32 keeps the
  last one whose boot ID matches the status, and needs no requests and no reply windows.
- **No frame is dropped for lack of room.** `hal_uart_send()` queues nothing when the transmit ring
  (`HAL_TX_RING`) is full, so a burst of info requests could otherwise crowd out a status. After
  the one at start-up, an info frame, whether due or asked for, is sent only right behind a status,
  at most one per status. The ring then never holds more than a status and an info frame, which
  fit, and the wire empties it well within the `STATUS_PERIOD_MS` to the next status. A burst of
  requests gets one info frame, with the next status.

## Updates

Our bootloader, below `APP_BASE`, writes application firmware sent by the ESP32 over the link.
Unlike STC's bootloader, it holds SCR and the relay drive low throughout.

### Start-up

```
reset (any kind) --> 0x0000 bootloader
   1. interrupts off, PCA module 0 and counter off; clr P3.5, clr P3.4; both push-pull
   2. watchdog on (`WDT_TIMEOUT_MS`)
   3. read the hand-over bytes (auxiliary RAM at HO_ADDR) and the reset cause
   4. update requested by the application? -- yes -> update mode
   5. application valid?                     -- no --> update mode
   6. BOOT_WDT_RESETS watchdog resets in a row? -- yes -> update mode
   7. jump to APP_BASE (application)
```

- Hand-over bytes: two magic bytes, update request, watchdog reset count, reset cause (power-on,
  watchdog, other). The application reports the last two in its info frame. The watchdog count is
  cleared at a power-on, by the application after `WDT_CLEAR_COUNT_MS` of normal running, and by
  the bootloader when an update finishes with a good image or it is told to run the application.
  An update request is taken once: the bootloader clears it as it enters update mode. The boot ID
  is the application's own; the bootloader leaves it alone.
- Auxiliary RAM survives a software reset. Survival through a watchdog reset, which step 6 needs,
  is assumption A1.
- A watchdog reset starts our bootloader, not STC's, because it does not set `SWBS`
  (section 2.2.8).

### Vector redirect

The 8051 vectors (0x0003 to 0x00AB, section 6.3) are in the bootloader, whose assembly table
holds `ljmp APP_BASE + vector` at each (`make firmware` passes `APP_BASE` from `flashmap.h` to the
assembler). The application is linked with `--code-loc APP_BASE`, which also moves SDCC's vector
table. Each interrupt costs 4 extra clocks (0.4 us). The bootloader uses
**no interrupts**: it polls the UART and clears the watchdog in its loop.

**No byte lost while it sends.** The UART holds one received byte, and the bootloader's boot status
(11 bytes) takes about 1 ms to send. Each received byte goes into a ring of `BOOT_RX_RING`
bytes, both in the main loop and while the bootloader waits for each sent byte to leave, so the
bytes that arrive during a boot status (at most 12) are kept, in order, and the main loop takes
them from the ring. Polling
during the send is enough, so the vector table and the application's interrupt cost stay as they
are. Bytes are still lost while it handles a command: Begin's erase (about 0.4 s), a Write (about
8 ms) and Finish's CRC check (about 60 ms) do not poll the UART, and an erase or write stalls the
CPU with the UART holding one byte. The ESP32 sends nothing then, because it waits for the reply.

### Application image

The update image is the whole application area (`APP_SIZE`), padded with 0xFF, ending in a
trailer (`TRAILER_SIZE`, fields `TR_*` in `flashmap.h`) written by `mkesphome.py`: the magic
`RPA1`, the length of the code the CRC covers, its CRC-16/CCITT, the firmware version, the build ID
("Toolchain and build"), and the valid mark: 0xFF in the update image; the bootloader writes 0x00
after checking the CRC.

"Valid" at start-up means the right magic and a valid mark of 0x00. A full CRC at every start-up
would take about 100 ms (A6), so the bootloader checks it once, after an update. The first-flash
image has the mark already set, because the STC programmer verifies its own writes.

### Update protocol

In update mode the bootloader sends a boot status every `BOOT_STATUS_PERIOD_MS`, so the ESP32 can
tell it from the application.

| Type | Direction | Payload | Meaning |
|---|---|---|---|
| `MSG_BOOT_STATUS` | MCU -> ESP32 | bootloader version (u8), application valid (u8), reason for update mode (`BOOT_REASON_*`), result of the last command (`BR_*`), next offset expected (u16) | Every `BOOT_STATUS_PERIOD_MS`, and in reply to each command |
| `MSG_BOOT_BEGIN` | ESP32 -> MCU | `BOOT_BEGIN_LEN` bytes: image size (u16, must be `APP_SIZE`) | Clear the old application's magic, so an erase cut short cannot leave it looking valid, then erase the application area, **trailer sector first** (about 0.4 s) |
| `MSG_BOOT_WRITE` | ESP32 -> MCU | `BOOT_WRITE_LEN` bytes: offset (u16) + `WRITE_CHUNK` bytes | Write, read back, reply. A repeat of a written chunk is accepted if it matches, so a lost reply can be retried. |
| `MSG_BOOT_FINISH` | ESP32 -> MCU | - | Check the trailer and the CRC over the image; if good, write the valid mark |
| `MSG_BOOT_RUN` | ESP32 -> MCU | - | Restart into the application (`IAP_CONTR = IAP_SWRST_APP`, section 2.2.2), if valid |

A frame of another type is dropped by the receiver and gets no reply. A command of the wrong length
is answered with `BR_BAD_FRAME` and changes nothing, also during an update. `BR_NO_COMMAND` means no
frame has been handled since update mode started. Without it, a fresh bootloader (OK, next offset 0)
would look like one that has just finished begin, so a lost begin could pass unnoticed until the
first write.

The ESP32 sends one frame at a time and waits for the reply. Writes go in increasing order. Any
error leaves the bootloader in update mode, and the ESP32 starts again with `begin`.

### Update flow

The ESP32 side, the MCU tool, is in [mcu-tool.md](mcu-tool.md).

1. `MSG_UPDATE`: the application turns the lamp off with the normal sequence and, once the relay
   drive is off, writes the hand-over bytes and restarts into update mode. A non-zero gate target
   received before that cancels the request.
2. Begin, one write per `WRITE_CHUNK`, finish, run: about 3 to 4 s (0.8 s of data, 0.4 s of
   erasing, 0.5 s of writing, plus turn-around).
3. The application starts with the lamp off, as after power-on. The main configuration restores
   the lamp once it is back on the ESP32.

Power loss at any point leaves the old application, the new one, or none; with none, the
bootloader stays in update mode and the update can be run again. Entered on request with a valid
application, update mode returns to the application `IDLE_RETURN_MS` after the last command. Each
of its own commands starts that time again, even one it answers with an error; frames of other
types are dropped at the type byte and do not.

A new bootloader, or a broken one, can only be written through STC's bootloader at a mains power
cycle, which the MCU tool catches.

### First flash and option bits

The first flash writes the whole image through STC's bootloader: the MCU tool from
`rpstd01_mcu_full_image.h`, or, with the box open ([../hardware/README.md, "With the box
open"](../hardware/README.md#with-the-box-open-not-tried): unplugged from mains), `stcgal` with
`build/mcu/full.ihx` (from the top folder, after `make firmware`). Each must write the factory
option bits back every time:

```sh
stcgal -P stc15 -p /dev/ttyUSB0 -t 11059 \
  -o reset_pin_enabled=false \
  -o por_reset_delay=long \
  -o bsl_pindetect_enabled=false \
  -o rstout_por_state=high \
  -o low_voltage_reset=true \
  -o low_voltage_threshold=3 \
  -o watchdog_por_enabled=false \
  build/mcu/full.ihx
```

**Keep these option bits**, or the box-closed route stops working; the MCU tool refuses to write a
chip on which the ones it checks differ ([../hardware/README.md, "Over Wi-Fi with the box
closed"](../hardware/README.md#over-wi-fi-with-the-box-closed-tested)).

## Testing

Everything but the register access builds with gcc, so `make test` runs the firmware's own code
on the host: unit tests per module, against a fake flash for the bootloader. The simulated MCU
(`esphome/tests/mcu_sim.c`) runs the real `app_loop()`, `boot_loop()` and start-up decision on
simulated time, mains, inputs, UART rings and flash (the Zo interrupt is not modelled: edges are
counted); the ESP32 side's tests run against it. A gate simulation (`tests/test_sim.c`) runs the
real `app_loop()` with a model of the PCA and the interrupts, and checks at every PCA count, at
every position of the 1 ms tick against the Zo edges, that the gate is never high at or after the
release point or across a Zo edge, that the relay never opens while the triac conducts, and that
nothing fires after a fault.

## Assumptions to confirm

| # | Assumption | Status | Matters for |
|---|---|---|---|
| A1 | Auxiliary RAM survives a watchdog reset | Software reset confirmed; watchdog reset not tried | Update mode after `BOOT_WDT_RESETS` crashes; the boot ID after a watchdog reset (if not, it changes only if the RAM comes back random) |
| A2 | Thermistor curve (100 kohm, B = 3950 assumed) | Warm the heatsink next to a thermometer | Over-temperature thresholds |
| A3 | Relay contacts closed `RELAY_SETTLE_MS` after the drive | Not measured | Turn-on |
| A4 | High-speed output flips the pin even if software also wrote it | Logic analyser | Gate timing |
| A5 | SCR and relay drive stay low enough during STC's bootloader | No visible effect during a flash with a lamp; relay drive not checked | First flash, every power-on |
| A6 | A full CRC at start-up would take about 100 ms | Time it | Choice of the valid mark |
| A7 | A status every `STATUS_PERIOD_MS` is enough for the add-on and buttons | Every gesture and add-on press recognised in testing | Link load |

## Sources

- STC15 series datasheet, English (2016): https://www.stcmicro.com/datasheet/STC15F2K60S2-en.pdf
- stcgal: https://github.com/grigorig/stcgal
- Tuya serial protocol:
  https://developer.tuya.com/en/docs/iot/tuya-cloud-universal-serial-port-access-protocol?id=K9hhi0xxtn9cb
