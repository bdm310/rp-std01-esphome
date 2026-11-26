# Martin Jerry RP-STD01 hardware notes

Notes on the RP-STD01 3-way dimmer kit (master unit). All measurements used 60 Hz mains and a
120 V incandescent lamp, on one unit unless a section says otherwise.

Sources of each fact:

- **traced**: checked on the board with a meter.
- **probed**: seen on the serial link between the ESP32 and the MCU.
- **photo**: read from the photos in this folder.
- **datasheet**: STC's English STC15 datasheet (2016), section 1.5 (STC15W401AS series, which
  includes the IAP15W413AS).
- **inferred**: an educated guess, with its confidence.

> **Safety.** This device switches mains voltage, and its low-voltage side is **not isolated**
> from mains: the add-on sense line joins it to the SG and LO wires through diodes. Treat
> everything, including the ESP32 header, as live whenever the device is on mains.

## Photos

| File | Shows |
|---|---|
| `control-front.jpg` | Control board, button and LED side |
| `control-back.jpg` | Control board, MCU and ESP32 side |
| `power-front.jpg` | Power board, component side |
| `power-back.jpg` | Power board, solder side |
| `remote-front.jpg` | Add-on switch, button side |
| `remote-back.jpg` | Add-on switch, wire side |

## Overview

Two boards joined by a 6-wire harness:

- **Control board:** an ESP32-C3 Wi-Fi module and an STC 8051 MCU, plus the buttons, LEDs and a
  thumbwheel potentiometer. Marked "RP-STD01 2024.12.18-Kx 10-00171".
- **Power board:** a mains power supply, the relay, the triac and its heatsink, a zero-cross
  detector, and the mains wires.

In stock form the MCU does all the real work and the ESP32 only talks to it over a serial link. Our
firmware keeps only timing and safety on the MCU
([../docs/mcu-firmware-design.md](../docs/mcu-firmware-design.md)).

## Control board

### Main parts

| Part | Details | Source |
|---|---|---|
| U2 MCU | STC **IAP15W413AS-35I-SOP28**, date code 2446. 8051 core, internal clock (no crystal). "IAP" means the firmware can rewrite all of its own flash. | photo |
| Wi-Fi module | DOIT **ESP-C3-12F** (ESP32-C3, 4 MB flash), FCC ID 2AL3B-ESP-C12F. The factory firmware uses DIO flash mode, so ours does too. | photo, probed |
| Q2 | SOT-23 transistor marked "Y1", likely an SS8050 NPN. Relay driver. | photo, traced |
| D12 | SOD-123 diode marked "T4", likely a 1N4148W | photo, traced |
| Thumbwheel pot | Read by the MCU. Scales the firing delay ("Stock firmware measurements"). | traced, tested |
| R38 pads | No resistor fitted. A two-wire lead runs to a thermistor on the triac heatsink. | traced |

### Buttons and LEDs

| Item | Connection | Source |
|---|---|---|
| WiFi button, "On/Off" on the board | MCU pin 2 (P2.7) | traced |
| Paddle UP | MCU pin 23 (P2.0) | traced |
| Paddle DOWN | MCU pin 18 (P3.3/INT1) | traced |
| RESET button | Shorts 3.3 V to GND, power-cycling both chips (below) | traced |
| LED1 to LED5 (brightness bar) | MCU pins 28, 27, 26, 25, 24 (P2.5 to P2.1) | traced |
| B2 (white LED by the WiFi button), the night light | MCU pin 22 (P3.7), which can output hardware PWM | traced |
| B3 (LED by the WiFi button) | MCU pin 1 (P2.6) | traced |

The LEDs connect to 3.3 V, so they are active low. The buttons are active low too: pressed
reads 0 (tested). The thumbwheel trims the lamp, not the night light.

**RESET button:** it shorts the 3.3 V rail to ground, a cold power-on reset for both chips. In
normal use the AMS1117 regulator limits the current. **Never press it while the board is powered
from anything else**, such as a USB-serial adapter: nothing would limit the current.

### MCU pin map

Pin names: **datasheet**. Uses: **traced**.

| Pin | Port and functions | Use |
|---|---|---|
| 1 | P2.6/CCP1_3 | LED B3 |
| 2 | P2.7/CCP2_3 | WiFi button |
| 3 | P1.0/ADC0/CCP1 | ? |
| 4 | P1.1/ADC1/CCP0 | Add-on sense: Ks through R17 (1 kohm), C3 (100 pF) to GND |
| 5 | P1.2/ADC2/SS/ECI/CMPO | Thermistor (heatsink temperature) |
| 6 | P1.3/ADC3/MOSI | Thumbwheel pot wiper |
| 7 | P1.4/ADC4/MISO | ? |
| 8 | P1.5/ADC5/SCLK | ? |
| 9 | P1.6/ADC6/RxD_3/XTAL2 | ? |
| 10 | P1.7/ADC7/TxD_3/XTAL1 | ? |
| 11 | P5.4/RST/CMP- | ? |
| 12 | VCC | 3.3 V |
| 13 | P5.5/CMP+ | ? |
| 14 | GND | GND |
| 15 | P3.0/RxD/INT4 | <- ESP32 TX0; header pin 9 |
| 16 | P3.1/TxD/T2 | -> ESP32 RX0; header pin 8 |
| 17 | P3.2/INT0 | Zo, zero-cross input from the power board |
| 18 | P3.3/INT1 | Paddle DOWN |
| 19 | P3.4/T0 | Relay drive: R7 (2.2 kohm) to the base of Q2 |
| 20 | P3.5/T0CLKO/CCP0_2 | SCR, triac gate drive to the power board |
| 21 | P3.6/INT2/RxD_2 | ? |
| 22 | P3.7/INT3/TxD_2/CCP2 | LED B2 |
| 23 | P2.0/RSTOUT_LOW | Paddle UP |
| 24 | P2.1/SCLK_2 | LED5 |
| 25 | P2.2/MISO_2 | LED4 |
| 26 | P2.3/MOSI_2 | LED3 |
| 27 | P2.4/ECI_3/SS_2 | LED2 |
| 28 | P2.5/CCP0_3 | LED1 |

P2.0 (paddle UP) has a programming option that drives it low after reset (RSTOUT_LOW). Leave it
off, so the pin stays an input.

### Harness

The 6-pin connector on the control board, in order. The label above it reads "ANC".

| Pin | Connects to | Meaning |
|---|---|---|
| SCR | MCU pin 20 | Triac gate drive, active high (traced, tested) |
| Zo | MCU pin 17 (INT0) | Zero-cross square wave (traced, tested) |
| Gn | GND | Ground (traced) |
| 3V | VCC | 3.3 V from the power board's AMS1117 (traced, photo) |
| Ks | R17 to MCU pin 4; R14 and R19, see below | Add-on sense line; its idle level follows the relay drive (traced, tested) |
| Re | Collector of Q2, and R45 | Relay coil, low side (traced) |

```
                    VCC (MCU pin 12)
                      |
                      +  D12 1N4148W (band at VCC)
                      ^
                      |
Re ---R45 470 ohm---- J ---R14 2.2k---- Ks ---R19 2k--- GND
|                                          |
|                                          +--R17 1k---+-- MCU pin 4 (P1.1/ADC1/CCP0)
|                                                      +-- C3 100pF -- GND
Q2 collector    Q2 base --R7 2.2k-- MCU pin 19    Q2 emitter -- GND
```

When MCU pin 19 is high, Q2 pulls Re low and the relay coil switches on. R17 and C3 filter only
very fast noise (0.1 us time constant).

**Ks on the harness** idles at about 3.4 V with Q2 off and 2.3 V with Q2 on, switching within 0.2 ms
of the drive, so it follows the relay **drive**, not the contacts (stock firmware, read on the
harness wire by the ESP32's ADC through a 10k/10k divider to GND; method in "Stock firmware
measurements"). This network alone cannot give those levels (D12 clamps J at about 3.9 V, so R14/R19
would hold Ks below 1.9 V), so the power board also drives Ks, in a way not known.

**Ks at MCU pin 4** (raw, of 1023, VCC reference, read by our MCU firmware with nothing else
connected to Ks) is well inside the ADC's range, unlike the harness levels, which are above the
3.3 V supply. Why they differ is not known. Each reading is steady within a few counts:

| Relay drive | Idle | Add-on UP held | Add-on DOWN held |
|---|---|---|---|
| Off | 556 (1.79 V) | 680 (2.19 V) | 840 (2.71 V) |
| On | about 62 | about 630 | about 820 |

While the relay drive is off, a small current leaks from the coil supply through the coil, R45 and
D12 into the 3.3 V rail: harmless at 5 V or less, about 9 mA with a 12 V, 400 ohm coil. The coil
voltage is not known.

**Thermistor:** pin 5 to ground, with a 10 kohm pull-up to 3.3 V (traced). It reads 933 of 1023 at
room temperature, which fits a 100 kohm NTC; hotter reads lower.

### 9-pin header (unpopulated)

Pin 1 is the square pad (the silkscreen "NC" is at the other end). Use it only with the device
disconnected from mains ("Safety", above).

| Pin | Connects to |
|---|---|
| 1 | 3.3 V |
| 2 | GND |
| 3 | ESP32 GPIO2 (boot strapping pin) |
| 4 | ESP32 EN (reset) |
| 5 | ESP32 GPIO9 (low at power-on enters the ESP32 download mode) |
| 6 | ESP32 GPIO10 |
| 7 | ESP32 GPIO8 (boot strapping pin) |
| 8 | ESP32 RX0 (GPIO20) and MCU pin 16 (TxD) |
| 9 | ESP32 TX0 (GPIO21) and MCU pin 15 (RxD) |

Traced; other connections may have been missed.

Download mode also needs GPIO2 and GPIO8 high, so disconnect anything that can hold them low
(for example Zo, or a divider to GND) before a USB-serial recovery of the ESP32. Factory Tasmota
leaves GPIO2, GPIO8 and GPIO10 unassigned.

## Power board

From the photos unless marked otherwise.

| Part | Details | Confidence |
|---|---|---|
| Mains wires | N = white, L = black, **TRA1 = red (SG on the board)**, **TRA2 = brown (LO on the board)**. TRA1/TRA2 are the case labels. | traced |
| Power supply | Chipown **PN8366** offline controller, a transformer, and an **AMS1117-3.3** regulator feeding the 3V harness pin | High |
| Relay | Black sealed relay between the toroid and the lower X2 capacitor. Part number not visible. | Medium-high |
| Triac | TO-220 part clamped to the metal bracket, with the thermistor under the same clip. Part number hidden. | High |
| Toroid | Interference-suppression choke in series with the load | High |
| Two yellow "TENTA" boxes | X2 safety capacitors (one is Cx1) | High |
| U1 | **PC817C** optocoupler beside R1. Probably the zero-cross detector driving Zo. | ~65% |
| Q4 and nearby small transistors | Probably the triac gate driver | Low |

LO is the dimmed output. SG carries the add-on's signal: the add-on talks to the master over a
signal wire, not as a classic 3-way switch (tested). No opto-triac was found, which fits a gate
driven directly from the low-voltage side. Gn reads high resistance to L and N unpowered, so the
low-voltage ground probably reaches mains through the relay, a diode or the triac.

## Add-on switch (remote)

From the photos unless marked otherwise. Board marked "RP-STD01-ADD 2024.12.12-Kx 10-00172".

The add-on is **passive**: no microcontroller, no power supply. Its only wires are TRA1/SG (red),
TRA2/LO (brown) and ground (green, to the metal strap).

| Part | Details |
|---|---|
| Paddle | UP and DOWN only (UP1 and DOWN1). K1 and K2 are fitted but change nothing and have no visible traces (traced). |
| LEDs | LEDP1 and ERR, where the master's B2/B3 are |
| Eight diodes marked "M7" | 1N4007-type rectifiers (1000 V, 1 A) |
| D9 | SOT-23, probably a diode pair |
| D26 | Small diode, in series with R40 |
| R13 | 12 ohm ("120") |
| R20 | 2.2 kohm ("222") |
| R40 | 200 kohm ("204"), next to the SG terminal; with D26, probably lights the LEDs (about 0.6 mA at 120 V) |
| C22 | Small ceramic capacitor |
| R15 (back) | Yellow disc between the SG and LO traces; the symbol suggests a varistor |

Measured between SG and LO (traced, diode mode, add-on disconnected):

| Button held | + on SG | + on LO | Reading |
|---|---|---|---|
| none | 0.54 V | open | One diode, SG to LO |
| UP | 0.54 V | 0.54 V | Back-to-back diodes |
| DOWN | 0 V | 0 V | SG shorted to LO |

On the master's power board (traced), SG goes to the anodes of two diodes, one to LO and one to Ks:

```
            add-on                             master power board          master control board
LO -+-[ idle: SG->LO diode ]-+
    +-[ UP:   LO->SG diode ]-+- SG --+-|>|-- LO
    +-[ DOWN: short        ]-+       +-|>|-- Ks --+- R19 2k -- GND
                                                   +- R14 2.2k -- J --+- D12 -- VCC (clamp)
                                                                      +- R45 470 ohm -- Re (Q2, relay)
```

So a press changes Ks by letting current in from LO through the add-on. **There is no current
limit:** no resistor on SG, only the two diodes. A tag on the wires reads "Do not connect to
neutral"; if SG or LO touched neutral, nothing obvious would limit the current. The power board is
treated as a black box.

At MCU pin 4 the presses read as in the table under "Harness" (our firmware). On the harness wire
they read as below (stock firmware, lamp on LO, thumbwheel at K about 4.83 ms, harness wire
through the 10k/10k divider, sampled at 10 kHz and 5 kHz; the steps are reliable, the
absolute values within a few percent). Both half-cycles read the same:

| State | Relay drive on | Relay drive off |
|---|---|---|
| Idle | about 2.36 V | about 3.41 V |
| Add-on UP held | about 3.57 V | about 3.68 V |
| Add-on DOWN held | about 3.97 V | about 4.02 V |

- The highest Ks seen was 4.3 V. The lamp did not glow during presses with it off.
- Ks steps to the press level within one sample, with no bounce, and settles back within 0.7 to
  0.9 ms. It follows the contact, not the mains. Taps lasted 88 to 156 ms.
- Single samples occasionally jump by about 0.4 V, so accept a level only after it holds for a few
  milliseconds.
- With the stock firmware, each add-on UP press moved the firing point about 0.88 ms, up to about
  0.28 x K, brighter than the master's paddle reaches. Holding a button turns the lamp on briefly,
  then off, as on the master; a DOWN press with the lamp off turns it on.

## Serial link and factory MCU firmware

Probed through the factory Tasmota firmware (13.0.0 "STD01 by MJ").

- **Protocol:** Tuya MCU serial protocol at **115200 baud, 8N1**, on ESP32 GPIO20 (RX) and GPIO21
  (TX). Factory Tasmota is set to 9600 baud (`SetOption97` off), so out of the box the ESP32
  cannot control the dimmer. `SetOption97 1` fixes it. If the factory settings are lost (for
  example after a Tasmota upgrade or reset), this Tasmota console line restores a working setup,
  baud rate fix included:

  ```
  Backlog Template {"NAME":"RP-STD01","GPIO":[1,1,0,1,1,1,1,1,0,1,0,0,0,0,0,0,0,0,1,1,2304,2272],"FLAG":0,"BASE":1}; Module 0; SetOption97 1; TuyaMCU 11,1; TuyaMCU 21,2; DimmerRange 0,100
  ```
- **Heartbeat:** the MCU replies `01`.
- **Product query (0x01):** `{"p":"rqkqghnesphc3ppj","v":"1.0.0","m":2}`, plus a stray `}`.
- **Working mode query (0x02):** an empty reply: the MCU runs its own LEDs and reset.
- **Datapoints:** only dp 1 (power, bool) and dp 2 (brightness, 1 to 100, echoed back). Nothing for
  the night light, thermistor or add-on.

**The factory MCU firmware cannot be backed up:** STC's bootloader has no read command and the chip
has no debug port.

## Programming the MCU

### Over Wi-Fi with the box closed (tested)

After a cold power-on the MCU runs STC's built-in bootloader before the user program and waits "tens
or hundreds of milliseconds" for a sync pattern on P3.0/P3.1 (datasheet section 16.2.1). Those pins
are the ESP32's serial link, and both chips share one supply, so a mains power cycle lets the ESP32
catch it. Sending `stcgal`'s sync byte (0x7F, 2400 baud, 8E1) every 10 ms from a static constructor
on the ESP32 got a valid status packet after about 20 to 22 sync bytes, about 213 to 218 ms after
power-on, at every try on both tested units, and the stock firmware then started as normal. The MCU
tool ([../docs/mcu-tool.md](../docs/mcu-tool.md)) uses this route. Both units gave the same model,
bootloader version and option bits (observed MSR bytes F7BB9FFDEC). The `stcgal` command with these
option bits is in [../docs/mcu-firmware-design.md, "First flash and option
bits"](../docs/mcu-firmware-design.md#first-flash-and-option-bits).

Decoded with `stcgal`'s packet layout:

| Field | Value |
|---|---|
| Model magic | 0xF562 = IAP15W413AS |
| Bootloader version | 7.2.5T (`stcgal` lists it as tested on the STC15W408AS) |
| Clock | internal RC, calibrated to 11.05 MHz |
| Power-on reset delay | long: an extra ~180 ms (datasheet section 2.2.4) |
| Bootloader pin check | off: P3.2/P3.3 (Zo/DOWN) need not be low |
| Reset pin | disabled: P5.4 is an I/O pin |
| P2.0 after power-on | high |
| Low-voltage reset | on, threshold setting 3 |
| Watchdog at power-on | off |

The long power-on delay is why the ESP32 wins the race: by the time the bootloader listens, the
ESP32 is already sending. **Keep the long delay on and the pin check off when programming**, or
this route stops working. The MCU tool refuses to write unless the model, the internal clock and
four of these are as above: the long power-on reset delay, the bootloader pin check off, the reset
pin disabled, and P2.0 high after power-on.

### With the box open (not tried)

The STC bootloader listens on header pins 9/8, shared with the ESP32's serial port. A likely setup:

- **Power:** unplug from mains and feed 3.3 V from a USB-serial adapter to header pins 1 (3.3 V)
  and 2 (GND).
- **Quiet the ESP32:** hold EN (header pin 4) low.
- **Connect:** adapter TX to header pin 9, RX to header pin 8.
- **Flash:** start `stcgal`, then switch the adapter's 3.3 V off and on. Do not use the RESET
  button ("Buttons and LEDs").

## Stock firmware measurements

Measured on one unit with the stock MCU firmware, 60 Hz mains and a 120 V incandescent lamp on LO,
instead of tracing the power board. A temporary Tasmota build on the unit's own ESP32 recorded the
signals, with jumpers from the harness to the 9-pin header:

- Zo to GPIO8 and SCR to GPIO10, every edge timestamped in an interrupt, up to a few
  microseconds late, so a shorter pulse could be recorded with the wrong level.
- Ks to GPIO2 through a 10k/10k divider to GND, sampled by the ADC at 20 kHz (10 kHz or 5 kHz
  in the add-on runs) and scaled back by 2.

Each recording lasted 1.5 to 6 s. The MCU was driven through Tasmota's TuyaMCU driver (dp 1,
dp 2) and by hand with the paddles on the master and the add-on. Unless a table says otherwise, the
thumbwheel was in its starting position, K about 4.6 ms (K is defined under "Thumbwheel" below).

Times are from the Zo edge that starts each half-cycle; unless noted, half-cycles that start on a
rising edge.

**Zo** is a square wave at mains frequency, high for about 8.29 ms and low for about 8.38 ms. Both
edges mark zero crossings; if the detector is symmetric, each is about 22 us from the crossing,
plus the optocoupler's delay. The level shows which half-cycle is in progress.

**Half-cycle timing.** In half-cycles that start on a falling edge, the stock firmware fires and
releases 40 to 50 us later (for example 4.32 against 4.36 ms). That matches Zo's imbalance, so it
probably times both halves from one edge. Gate pulses are a steady 8.34 ms apart.

**SCR** (active high) goes high at the firing point and stays high until about 7.4 ms, roughly
1 ms before the next crossing, in every half-cycle while the lamp is on.

| Setting | Firing point |
|---|---|
| Start and end of every fade | about 4.29 ms (about 0.93 K) |
| dp 2 = 20 | 4.32 ms |
| dp 2 = 50 | 3.75 to 3.81 ms |
| dp 2 = 80 | 3.30 ms |
| dp 2 = 100 | 2.99 ms |
| Brightest with the paddle | 2.15 ms (about 0.47 K) |

**Thumbwheel.** It scales the whole firing delay. With the wheel at each end:

| dp 2 | End A | End B |
|---|---|---|
| 1, 2 | sawtooth sweep (below) | not tested |
| 3 | 4.15 ms | 7.50 ms (bursts, below) |
| 10 | 4.08 ms | 7.31 ms |
| 20 | 3.90 ms | 7.06 ms |
| 50 | 3.45 ms | 6.21 ms |
| 100 | 2.73 ms | 4.79 ms |

Each value is the steady firing point, read 2.5 to 6 s after the dp 2 write.

All readings fit **firing delay ~ K x (1 - 0.35 x (dp - 3) / 97)** within about 0.03 ms (end B
at dp 2 = 100 is 0.09 ms early). K is about 4.17 ms at end A, 7.50 ms at end B, and 4.6 ms in the
wheel's starting position. Read by our firmware, the wheel spans raw 0 (end B, the dimmer bottom
end) to 1023 (end A).

- dp 2 = 100 fires at about 0.65 x K, so it never reaches full brightness.
- **At end B, dp 2 = 3 to 5 do not dim:** SCR goes high at about 7.5 ms, after the release point,
  and stays high for about 71 ms through about 8 crossings, then low for 12.5 ms. The lamp
  probably gets bursts of full power at about 12 Hz (inferred from SCR). A stock firmware bug.
- dp 2 = 1 or 2 starts a sawtooth: the firing point moves about 0.015 ms per half-cycle down to
  0.40 ms and jumps back (dp 2 = 1, every 2.1 s), or 0.023 ms up to about 4.1 ms and jumps to
  0.43 ms (dp 2 = 2, every 1.4 s). Use dp 2 = 3 as the minimum.

**Buttons.** A press of UP or DOWN changes the brightness one step. Holding UP or DOWN turns the
lamp on for about 0.7 s, then off.

**Serial control:**

- dp 1 switches the lamp; dp 2 (1 to 100) sets the brightness. dp 2 = 0 turns the lamp on at its
  last brightness.
- The MCU echoes each write it accepts, but occasionally drops one without replying (2 of about
  70 writes over all the sessions), including the dp 1 write Tasmota sends 2 ms after a dp 2
  write at every boot. There is no retry, so a controller must check for the echo.

### Stock timing

Our MCU firmware copies the gate drive, the relay timing and, as its default, the fade rate; the
ESPHome configuration sets its own Fade Limit, and copies the start at the dim end.

| Item | Value |
|---|---|
| Firing point range | 0.40 ms (dp 2 = 1 sweep) to about 4.3 ms at the usual thumbwheel setting. Never fire after the release point, as stock does at end B. |
| Gate drive | SCR held high from the firing point until about 7.4 ms |
| Fade rate | about 0.017 ms per half-cycle brightening, 0.025 ms dimming, whatever started it |
| Turning on | relay drive on, then 11.5 to 12.5 ms later start firing at about 0.93 x K and fade to the target |
| Turning off | fade to about 0.93 x K, stop firing, relay drive off about 1 ms later |
| Half-cycle reference | both halves apparently timed from one Zo edge; timing each from its own edge would differ by about 45 us |

## Open questions

1. **Thermistor:** its curve, and the stock over-temperature thresholds.
2. **Relay:** when the contacts close and open compared with the drive, and the coil voltage.
3. **Add-on:** whether it works with no load, and whether the idle Ks level shows
   that one is connected (2.36 V with one, 2.31 V in other sessions without, other conditions
   also changed).
4. **Night light:** what sets its brightness in stock form.
5. **SCR and relay drive during STC's bootloader:** after reset the pins have a weak pull-up (150
   to 270 uA, datasheet section 4.1), for the bootloader's short wait at every power-on and for
   the whole of a flash. During a 6.5 s flash with a lamp, nothing visible happened; whether the
   relay drive came on is not known.
6. **Unused MCU pins:** which of 3, 7 to 11, 13 and 21 are not connected.
7. **Stock firmware only:** whether dp 2 accepts values above 100, why the MCU drops writes, and
   what the lamp does at end B with dp 2 = 3 to 5.

## Sources

- Martin Jerry RP-series support: https://www.martinjerry.com/support/rp-series
- Amazon listing: https://www.amazon.com/dp/B0DT74K641
- Older ESP8266 STD01 on the ESPHome device database:
  https://devices.esphome.io/devices/Martin-Jerry-STD01
- STC15W408AS-35I-SOP28 pinout (third-party): https://wiki.adom.inc/p/pg_n3ex93aza9
