/*
 * RP-STD01 board constants. Times are in PCA counts (PCA_HZ) unless the name ends in _MS.
 */
#ifndef BOARD_H
#define BOARD_H

/* High byte: link protocol version, raised for any change an older ESP32 side would get wrong
   (then the low byte starts again at 0). Low byte: release. */
#define FW_VERSION 0x0001u

#define SYSCLK_HZ 11059200ul
#define PCA_HZ 921600ul
#if PCA_HZ * 12u != SYSCLK_HZ
#error "the PCA counts SYSclk / 12"
#endif
/* Rounded to the nearest count. PCA_HZ / 100 keeps the product within 32 bits. */
#define PCA_US(us) ((unsigned) (((us) * (PCA_HZ / 100u) + 5000u) / 10000u))
#if PCA_HZ % 100u
#error "PCA_US needs PCA_HZ to be a multiple of 100"
#endif

#define EDGE_MIN 6500u   /* closer edges are noise */
#define EDGE_MAX 11000u  /* a longer gap restarts the count of good edges */
#define PERIOD_MIN ((unsigned) (PCA_HZ / 65u))
#define PERIOD_MAX ((unsigned) (PCA_HZ / 45u))
#define LOCK_WINDOW 300u  /* once locked, edges more than this early are noise */
#define ZO_GOOD_EDGES 16u
#define ZO_LOST_MS 20u

#define RELEASE_BEFORE_EDGE PCA_US(933)
#define LATEST_BEFORE_EDGE ((unsigned) (RELEASE_BEFORE_EDGE + PCA_US(400)))
#define EARLIEST_FIRE PCA_US(500)
#define SCHED_MARGIN 100           /* a fire time closer than this to the edge is skipped */
#define RELEASE_OVERDUE ((int) PCA_US(1000)) /* past the release time with the gate still armed */

/* Default smoothing, counts per half-cycle (stock fade rates). */
#define LIMIT_BRIGHTER 16u
#define LIMIT_DIMMER 23u

#define RELAY_SETTLE_MS 12u
/* Whole ticks, the first of which may come at once: one more than the 1 ms that must pass. */
#define RELAY_OFF_DELAY_MS 2u
/* After a fault the relay opens like a normal turn-off, but at the latest this long after it: a
   pulse fired before the fault ends within one half-cycle, and Zo edges may have stopped. */
#define FAULT_RELAY_MS 16u
_Static_assert((FAULT_RELAY_MS - 1u) * PCA_HZ > PERIOD_MAX / 2u * 1000ul,
               "FAULT_RELAY_MS less its first tick must outlast the longest half-cycle");

/* Raw ADC, hotter reads lower. */
#define THERM_TRIP 729u     /* about 60 C */
#define THERM_CLEAR 801u    /* about 50 C */
#define THERM_OPEN_SHORT_LOW 20u
#define THERM_OPEN_SHORT_HIGH 1003u  /* above: colder than about -7 C, or the thermistor is open */

#define STATUS_PERIOD_MS 10u
#define INFO_PERIOD_MS 1000u
#define WDT_CLEAR_COUNT_MS 10000u
/* Whole 1 ms ticks without a byte before a partly received frame is dropped. */
#define APP_RX_TIMEOUT_MS 5u
#define BOOT_RX_TIMEOUT_MS 20u

#endif
