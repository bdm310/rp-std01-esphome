# MCU tool

The MCU tool is a separate ESPHome firmware for the RP-STD01's ESP32-C3. It installs and updates the
firmware of the dimmer's MCU (`firmware/`), bootloader included. It replaces the main configuration
([esphome-config-design.md](esphome-config-design.md)) on the ESP32 for as long as the work takes;
moving between the two is an ordinary ESPHome OTA update in either direction. The steps for users
are in [install.md](install.md).

It writes the MCU in one of two ways:

- **through our bootloader, over the link:** the application alone, at once, with no power cycle;
- **through STC's ROM bootloader, at a mains power-on:** bootloader and application, after the
  user switches the dimmer off and on.

It chooses by itself when it starts, and its web page has buttons for the exceptions. It is also
how a stock unit gets our firmware: it is uploaded from the factory Tasmota and installs the MCU
firmware at a power-on.

## Why a separate firmware

- **Only STC's bootloader can replace our bootloader.** Our bootloader never rewrites itself, and
  STC's runs only at a mains power-on, so a new bootloader needs code on the ESP32 that is ready
  at that moment.
- **The main configuration would carry code it rarely uses:** the update, its recovery and the MCU
  image are needed a few times in the life of a unit.
- **What it costs:** an application update takes two ESP32 OTA uploads (to the tool and back),
  about a minute in all, instead of one button; a bootloader update also needs someone at the
  breaker.

## Building

The tool is built per unit, from a device file that includes the same unit file (name and
secrets) as the unit's main device file: ESPHome's `name:` sets the mDNS and DHCP name and the
name `esphome upload` looks for, so the unit keeps its name and usually its address while the tool
runs. The tool has its own build folder (`build_path`), so building the two in turn does not
rebuild everything each time.

The tool has no `api:`, so Home Assistant shows the unit as unavailable while the tool runs,
instead of showing a different set of entities. Its OTA uses the main configuration's API key.

The tool carries the MCU firmware (`rpstd01_mcu_full_image.h`), and the main configuration expects
that same build (`MCU_IMAGE_*` in `rpstd01_fw_constants.h`), so both are built from the same
commit.

## When it is needed

When the main configuration shows **MCU Update Available**, or MCU State says "Update mode (use the
MCU tool)" or "MCU firmware needs the MCU tool" ([esphome-config-design.md, "The expected
build"](esphome-config-design.md#the-expected-build) and ["Protocol
check"](esphome-config-design.md#protocol-check)).

## The web page

| Entity | Type | Shows or does |
|---|---|---|
| MCU State | text | What the tool found, what it does or did, and what to do next |
| Application Update | text | The update through our bootloader: Idle, Requesting, Erasing, Writing 37 %, Checking, Starting, Done, Failed: ... |
| Application Update Progress | sensor, % | The same as a number |
| MCU Installed | text | What the MCU reported at the check (application version, build and bootloader version, or update mode, or no answer), and again after an update through our bootloader or when our bootloader comes back |
| MCU Carried by Tool | text | The firmware the tool carries: version, build, bootloader version, size and CRC-32 |
| MCU Install Armed | text | What is armed for the next power-on, the last button, the count of failed automatic attempts, and whether the MCU firmware is incomplete |
| Last MCU Install | text | The last install through STC's bootloader: refused (and why), failed (at which stage, and how many bytes were written), written but not answering, or OK with the application that answered |
| Update Application Now | button | Writes the carried application through our bootloader, also when it is current or newer; refused, and the page says why, while the MCU's bootloader is not the carried one |
| Install at Next Power-On | button | Arms an install through STC's bootloader by hand, also when everything is current; also clears a refusal and the count of failed attempts |
| Cancel Install | button | Disarms |

While the tool runs, nothing controls the lamp: the MCU keeps the state it had when the main
configuration stopped. An update through our bootloader turns the lamp off first.

## Power cycle

STC's bootloader runs only after a real power-off: the MCU's long power-on reset delay is a factory
option, and the tool keeps it.

Until ESP-IDF has marked the tool's image valid, a power cycle may start the previous firmware
instead (OTA rollback, if the unit's second-stage bootloader supports it; units migrated from
Tasmota keep Tasmota's, which does not). ESPHome's `safe_mode` marks it after
`SAFE_MODE_GOOD_AFTER_S`; the tool marks it as soon as Wi-Fi is up and the check has decided, and
until then MCU State starts with "Wait until this tool has run for ...". Marking it early is safe:
the install flag is one-shot ("Arming rules"), and `safe_mode` still catches boot loops.

## What it does at start

### At a power-on with an install armed

A C++ constructor in `rpstd01_tool_platform.cpp` runs before ESPHome starts, and before the other
constructors, feeding the RTC watchdog (the second-stage bootloader starts it, and ESP-IDF turns
it off only after the constructors). It acts only when the ESP32's reset reason is a power-on, and
only when NVS holds an armed install (or the incomplete flag, see "Arming rules"). It then:

1. clears the flag in NVS;
2. drives UART0 (GPIO20/21) directly and sends STC's sync byte until STC's bootloader answers with
   its status packet, for at most `MCU_CONNECT_US`;
3. checks the chip model, the internal clock and the option bits in the status packet
   ([../hardware/README.md, "Over Wi-Fi with the box
   closed"](../hardware/README.md#over-wi-fi-with-the-box-closed-tested)); anything else is
   refused, and nothing is written;
4. calibrates the MCU's clock to `SYSCLK_HZ`, changes to a faster baud rate, erases, writes the
   whole image (bootloader and application), and writes the option bytes back unchanged;
5. tells STC's bootloader to start the new firmware (the disconnect), and listens for the new
   application's info frame for `MCU_VERIFY_US`;
6. records the attempt in NVS: the stage reached, the results, the bytes written, and the
   application that answered.

The disconnect starts whatever is at 0x0000, so it is sent only when our bootloader is complete:
the image is written from 0x0000 up, with the application's trailer in the last block, so our
bootloader then starts, finds no valid application, and holds the gate and relay pins low in
update mode. Otherwise the MCU stays in STC's bootloader, with those pins on weak pull-ups, until
the power is switched off. If the reply to the baud rate switch was lost, STC's bootloader may
already have switched, so the tool sends the disconnect at the new rate first, then at the old one
after `RATE_GAP_US` (`stcgal` sends it only at the old rate, or not at all).

The constructor has to reach STC's bootloader while it still listens after the power-on. The
second-stage bootloader checks the application image before starting it, and that time grows with
the image, so the tool image is kept smaller than the main configuration's. A 1.5 MB Tasmota image
still won the race.

### The check

As soon as ESPHome has started, the tool asks the MCU for its info frame, again every
`CHECK_ASK_EVERY_MS`, for at most `CHECK_LISTEN_MS`. It decides at the first info frame or boot
status, or at the end of that time:

| The MCU | Seen as | The tool |
|---|---|---|
| Our application, carried build, carried bootloader | Info frame | Does nothing: "Everything is current: install the main configuration." |
| Our application, another build, carried bootloader | Info frame | **Updates the application through our bootloader at once** |
| Our application, bootloader older than carried | Info frame | **Arms an install through STC's bootloader**, asks for a power cycle |
| Our bootloader in update mode, not newer than carried, with a valid application | Boot status | Sends `MSG_BOOT_RUN` (up to `CHECK_RUN_TRIES` times) and decides by the application's info frame, as in the rows above and below. If it does not start, nothing is armed: its version is unknown |
| Our bootloader in update mode, carried version, no valid application | Boot status | **Writes the carried application at once** |
| Our bootloader in update mode, older version, no valid application | Boot status | Arms an install through STC's bootloader |
| Newer firmware than carried: a newer bootloader, or a newer application version | Info frame or boot status | Does nothing by itself (no downgrade). Install at Next Power-On forces an install; Update Application Now only while the bootloader is the carried one |
| An answer, but no info frame or boot status the tool reads (a release with frames of other lengths) | Frames it cannot read | Does nothing by itself (no downgrade): the page says the MCU runs firmware this tool does not understand. Install at Next Power-On forces an install |
| No answer, or not our protocol (factory firmware, no firmware) | Nothing valid | Arms an install through STC's bootloader |
| No answer, but the application this power-on installed answered just after the install | Nothing valid | Does nothing: a link fault (GPIO20/21), not a reason to write again. NVS is not changed |

A version's high byte is the link protocol, its low byte the release ([mcu-firmware-design.md,
"Toolchain and build"](mcu-firmware-design.md#toolchain-and-build)); a higher version of either
counts as newer. The tool reads frames as the main configuration does ([esphome-config-design.md,
"Protocol check"](esphome-config-design.md#protocol-check)), so a later release with frames of
other lengths is never taken for factory firmware.

The boot status does not say which application the bootloader holds, so a valid one is started
rather than written over: it may be newer than the carried one.

After an update through our bootloader the tool checks again with the new application's info
frame, so the page says whether everything is current. Otherwise frames after the decision do not
change what the page shows, with two exceptions, both on our bootloader at the carried version
with no update running:

- **A crash:** when the MCU was last seen current (at the check, or after an update), and our
  bootloader comes back with the reason "watchdog", the tool writes the carried application once.
  If it crashes again after that, the tool stops and says so: an application the watchdog keeps
  resetting is a firmware bug, and a power-cycle install would write the same application. Nothing
  is armed. Update Application Now still writes it.
- **The repair:** when an update through our bootloader failed and the bootloader reports no
  valid application, the tool starts the update again at once, at most `REPAIR_MAX` times per tool
  start. Once that is used up, it decides on the MCU as it is at that moment (our bootloader with
  no valid application) by the arming rules, as for an MCU that does not answer: an install
  through STC's bootloader is armed unless the rules hold it, and the page says why. Repeated
  failed writes over the link point at the link, and the STC install is the remaining way that
  needs no box opening. The arming rules bound it, and it needs someone at the breaker.

While it works through our bootloader the tool changes nothing in NVS: Cancel Install, an install
armed by hand, the count of failed attempts and the incomplete flag stay as they were. A check
that then finds everything current applies the usual rules (an automatic arm, the count and the
incomplete flag are cleared).

## The application update

The update through our bootloader is `AppUpdater` in `rpstd01_tool_update.h`, on top of the
receiver the main configuration uses too (`McuLink` in `rpstd01_link.h`). It sends nothing while
no update runs, and at most one frame per loop. Each step:

| State | Sends | Moves on when | Retries and limits |
|---|---|---|---|
| Requesting | update request, every `REQUEST_RESEND_MS` while our application's statuses come | the first boot status: Begin | fails `REQUEST_TIMEOUT_MS` after the start; fails at once if the application restarts after a request instead of entering update mode |
| Begin | begin, with `APP_SIZE` | boot status OK, next offset 0, application not valid: Write | `COMMAND_TRIES` |
| Write | write, the `WRITE_CHUNK` bytes at the boot status's next offset | boot status OK with a higher next offset: the next chunk, or Finish after the last | `WRITE_TRIES` per chunk; a next offset that is not a multiple of `WRITE_CHUNK` goes back to Begin ("MCU restarted during the update") |
| Finish | finish | boot status OK, application valid: Run | `COMMAND_TRIES` |
| Run | run while boot statuses still come; then an info request | info with the carried build: Done | `COMMAND_TRIES`, then fails; another build fails |

- **Retry timer:** a frame goes out again `RETRY_MS` after the last one when no boot status has
  shown the step's result. A lost reply needs no resend, because the bootloader repeats its last
  result and the next boot status shows the progress.
- **Stale errors:** a boot status the bootloader started sending before the command arrived can
  still carry the previous result. So an error result counts only after a loop that started at
  least `CROSS_MS` (the longest command plus a boot status, on the wire) after the first loop
  since the command was written. A real error is repeated with the next boot status,
  so the wait costs nothing.
- **Back to Begin:** a boot status with an error result (bad data, wrong offset, a failed image
  check), a next offset lower than the one written, or the tries of a step used up go back to
  Begin, at most `MAX_RESTARTS` times; then the update fails with the reason. "No command yet" is
  not an error in Begin (Begin has not arrived; the retry timer sends it again), and later means the
  bootloader restarted.
- **Fails at once:** in Begin, Write and Finish, a status from our application ("MCU left update
  mode"), or no boot status for a while ("MCU stopped answering"); in Run, no status of either
  kind after a try; at any step, a boot status with a bootloader version other than the carried
  one. The first boot status is checked before Begin is sent, so then nothing is erased.
- The tool sends no gate frame first: it never turned the lamp on, and the MCU turns the lamp off
  with its normal sequence before it enters update mode. Application frames go out only while the
  newest frame is our application's status, bootloader frames only to the bootloader.
- It writes the application area of the image it carries for STC's bootloader. That image has the
  valid mark set; the tool sends the mark as 0xFF, because our bootloader sets it only after its
  own CRC check.
- Started on the bootloader, the update begins with Begin; started on the application, with the
  request.

The tool's own state on top of this is two values: whether it has written the application for a
crash, and how many repairs it has started.

## Arming rules

The arming rules are in `mcuflash` in `rpstd01_mcuflash.h`. NVS holds the arm flag (the
CRC-32 of the image to install), the last button, the count of failed automatic attempts, the
incomplete flag, and the record of the last install.

- **Automatic arming** happens only after the check, and only for the states in the table above
  (and after a used-up repair). Cancel Install, `MAX_AUTO_FAILS` failed automatic attempts in a
  row, or a refusal hold it until Install at Next Power-On. A check that finds the MCU current (or
  newer) forgets Cancel Install.
- **One-shot flag:** the flag is cleared in NVS before an attempt starts, so a crash or a power cut
  cannot repeat it at every power-on.
- **Refusals:** another chip model, unexpected option bits, an external clock, or a locked chip.
  Nothing is written. A refusal holds until Install at Next Power-On, also for another tool build
  with the same record version. That attempt then replaces the record: refused again, it holds
  again; any other result ends the refusal. A refusal recorded with another record version is
  dropped with that record (see "Stale NVS values"), so the check may arm again; that attempt is
  refused again before anything is written.
- **Failed attempts** count only when STC's bootloader's status packet arrived and the attempt was
  not refused; a power-on too short for the bootloader does not count. An attempt cut short by a
  crash or a power cut counts. The count resets when the check finds the MCU current (or newer), and
  on Install at Next Power-On; an install by hand does not count.
- **Incomplete firmware:** a write that stopped after the erase leaves the MCU without working
  firmware. The tool then installs at every power-on, without the flag and past the count of
  failures, until a write completes; only Cancel Install (and a refusal) holds it. The page warns.
- **No downgrade by itself:** newer firmware on the MCU, or an application of unknown version, is
  never replaced unless a button says so.
- **Update through our bootloader:** only with the carried bootloader (another one takes an STC
  install; Update Application Now is refused then), and only one at a time. Until its
  result is known it keeps Cancel Install and the other NVS values (see "The check").
- **Stale NVS values** do no harm: a record of another version or size is no record (a refusal in
  it is dropped; the page shows Last MCU Install "None"), an arm flag for another image is dropped
  at the next power-on, and other button values read as none. NVS is the tool's own namespace
  (`rpstd01tool`).

Checked for loops: a refusal is always cleared by Install at Next Power-On, which arms
unconditionally; automatic attempts stop after `MAX_AUTO_FAILS` failures, and attempts that never
reach STC's bootloader write nothing. The one install that repeats at every power-on is the
incomplete one, until a write completes or Cancel Install; each repeat needs a power cycle, and
every repeat writes only what an incomplete MCU would otherwise run as a partial program.

## Files

- `rpstd01_stc_isp.h` is the programmer for STC's bootloader, ported from `stcgal`.
- `rpstd01_mcu_full_image.h` is written by `make firmware`: bootloader and application, and their
  CRC-32.
- The tool reads the link with `McuLink` from `rpstd01_link` (shared with the main configuration).

## Testing

The tool's logic has no ESPHome dependency, so `make test` runs it on the host: the check and the
arming rules over sequences of buttons, restarts and power cycles; the update through our
bootloader against the simulated MCU ([mcu-firmware-design.md,
"Testing"](mcu-firmware-design.md#testing)); and the STC programmer against the packets `stcgal`
sent for a fixed test image, recorded from `stcgal`'s own test trace. `make test-host` also runs
the tool on ESPHome's host platform (no power-on step, NVS in RAM) against the fake MCU.
