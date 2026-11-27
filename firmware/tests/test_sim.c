/*
 * Runs the real main loop (app_loop) and the interrupt logic against a model of the PCA module and a
 * 60 Hz or 50 Hz Zo signal, one PCA count per step, and checks the gate timing rules throughout.
 * This file is the hardware layer (hal.h) the main loop runs on.
 */
#include <string.h>

#include "app.h"
#include "board.h"
#include "flashmap.h"
#include "hal.h"
#include "protocol.h"
#include "sched.h"
#include "test.h"

#define HALF_60HZ (PCA_HZ / 120u)
#define HALF_50HZ (PCA_HZ / 100u)
#define ZO_SKEW 20u         /* Zo rises this long after a crossing and falls this long before one */
#define MS ((PCA_HZ + 500u) / 1000u)
#define ISR_LATENCY 2u /* the Zo interrupt reads the pin this many counts after the edge */
/* The main loop passes the new delay to the Zo interrupt this many counts after it read the edge
   count (gate_tick and safety run in between). */
#define FIRE_LAG 40u
#define SECONDS(x) ((uint32_t) ((x) * PCA_HZ))
/* The gate is never high this close before a zero crossing. */
#define GATE_LOW_BEFORE_CROSSING PCA_US(800)

static sched_t s;
static uint8_t pin, armed;
static uint16_t cmp;
static uint16_t fire_delay, fire_next;
static uint32_t fire_at;
static uint32_t half_c; /* true zero crossings at multiples of this */
static uint32_t sim_t;
static uint16_t sim_ms;
static uint16_t sim_therm;
static uint8_t sim_therm_ready;
static uint8_t relay_pin;
/* The triac conducts from a gate pulse until the next zero crossing. relay_breaks counts gate
   pulses with the relay drive off and turn-offs that drop the relay while the triac conducts. */
static uint8_t conducting;
static unsigned relay_breaks, old_delay_fires;
static unsigned fault_fires; /* ticks in FAULT that leave a delay for the Zo interrupt */
/* The gate high outside the firing window or across a true Zo edge, or in a state that does not
   fire. */
static unsigned gate_violations;
static uint8_t last_mains;
static uint8_t gate_stuck_low; /* the gate pin stays low at the next fire match: a gate fault */

static uint32_t zo_stop_from, zo_stop_to;
/* Mains itself: gone from mains_gone_from to mains_gone_to (nothing conducts, no Zo, and the gate
   rules do not apply), and from shift_at on, its crossings shift_by later. */
static uint32_t mains_gone_from, mains_gone_to, shift_at, shift_by;
/* Each falling Zo edge moves by up to this, pseudo-randomly, as measured on unit 1. */
static uint32_t zo_jitter;
static uint32_t last_violation_at;
/*
 * LED lamps on an across-the-switch Zo detector (led_load), as measured on unit 1:
 * - Zo holds its level while the relay is closed and the triac has not conducted for a half-cycle;
 * - for LED_FALL_LATE_FOR after the relay closes, falling edges come LED_FALL_LATE late;
 * - after the relay opens, rising edges come later and later, up to LED_RISE_LATE_MAX after
 *   LED_RISE_LATE_FOR, then on time again.
 */
#define LED_FALL_LATE PCA_US(1000)
#define LED_FALL_LATE_FOR SECONDS(0.5)
#define LED_RISE_LATE_MAX PCA_US(2500)
#define LED_RISE_LATE_FOR SECONDS(1.5)
static uint8_t led_load;
static uint32_t conducted_at, relay_changed_at;
static unsigned timer_starts; /* half-cycles the reference timer started */
static uint8_t most_missed;   /* falling half-cycles in a row without a falling edge, at most */
static uint32_t noise_from, noise_to;
static unsigned glitch_edges; /* Zo edges of noise glitches seen by the Zo interrupt */
static unsigned wdt_count_clears;
static uint16_t wdt_count_cleared_ms;

uint16_t hal_ms(void) { return sim_ms; }
void hal_wdt_clear(void) {}

void hal_take_inputs(hal_in_t *in) {
  in->edges = s.edges;
  in->half = s.phase.period >> 1;
  in->zo_valid = s.zo_valid;
  in->gate_fault = s.fault;
  s.fault = 0;
  in->zo_coasted = (s.phase.missed | s.phase.outside_phase) != 0;
  in->therm_avg = sim_therm;
  in->therm_ready = sim_therm_ready;
  in->wheel_avg = 512;
}

void hal_take_window(window_t *w) { window_reset(w); }

void hal_set_fire(uint16_t delay) {
  fire_next = delay;
  fire_at = sim_t + FIRE_LAG;
}

void hal_set_relay(uint8_t on) {
  /* No turn-off, normal or after a fault, may open the relay while the triac conducts. */
  if (relay_pin && !on && conducting)
    relay_breaks++;
  if (relay_pin != on)
    relay_changed_at = sim_t;
  relay_pin = on;
}

void hal_set_outputs(uint8_t leds, uint8_t night) {
  (void) leds;
  (void) night;
}
uint8_t hal_uart_read(uint8_t *b) {
  (void) b;
  return 0;
}
uint8_t hal_uart_send(const uint8_t *buf, uint8_t len) {
  (void) buf;
  (void) len;
  return 1;
}
void hal_enter_bootloader(void) { CHECK(0); }
uint8_t hal_reset_cause(void) { return RESET_POWER_ON; }
uint8_t hal_wdt_count(void) { return 0; }
void hal_clear_wdt_count(void) {
  wdt_count_clears++;
  wdt_count_cleared_ms = sim_ms;
}
uint8_t hal_boot_id(void) { return 0; }
uint16_t hal_fw_version(void) { return FW_VERSION; }
uint32_t hal_build_id(void) { return 0; }
uint8_t hal_boot_version(void) { return BOOT_VERSION; }

static void apply(uint8_t a, uint16_t t) {
  if (a == SA_OFF || a == SA_ARM) {
    pin = 0;
    armed = 0;
  }
  if (a == SA_ARM || a == SA_RELEASE) {
    armed = 1;
    cmp = t;
  }
}

/* Time on the mains' own clock, which its crossings are multiples of half_c on. */
static uint32_t mains_t(uint32_t t) { return t >= shift_at ? t - shift_by : t; }

static uint8_t mains_gone(uint32_t t) { return t >= mains_gone_from && t < mains_gone_to; }

/* Zo from the mains crossings: rising ZO_SKEW after one, falling ZO_SKEW before one, each edge
   later by rise_late or fall_late, the falling one also by up to zo_jitter either way. */
static uint8_t zo_shaped(uint32_t t, uint32_t rise_late, uint32_t fall_late) {
  uint32_t m = mains_t(t) + 2 * half_c - ZO_SKEW, pos = m % (2 * half_c);
  int64_t fall_at = (int64_t) half_c - 2 * ZO_SKEW + fall_late;
  if (zo_jitter) {
    uint32_t k = m / (2 * half_c);
    fall_at += (int64_t) (((k * 2654435761u) >> 8) % (2 * zo_jitter + 1)) - zo_jitter;
  }
  return pos >= rise_late && (int64_t) pos < fall_at ? 1 : 0;
}

/* The mains crossings as a level: high from one to the next, as Zo would be with no skew. */
static uint8_t zo_mains(uint32_t t) { return (mains_t(t) + 2 * half_c) % (2 * half_c) < half_c ? 1 : 0; }

/* Zo without noise. Holds its level while Zo is stopped. */
static uint8_t zo_true(uint32_t t) {
  static uint8_t held;
  uint32_t since = t - relay_changed_at, rise_late = 0, fall_late = 0;
  if ((t >= zo_stop_from && t < zo_stop_to) || mains_gone(t) ||
      (led_load && relay_pin && t - conducted_at > half_c))
    return held;
  if (led_load && relay_pin && since < LED_FALL_LATE_FOR)
    fall_late = LED_FALL_LATE;
  if (led_load && !relay_pin && relay_changed_at && since < LED_RISE_LATE_FOR)
    rise_late = (uint32_t) ((uint64_t) since * LED_RISE_LATE_MAX / LED_RISE_LATE_FOR);
  held = zo_shaped(t, rise_late, fall_late);
  return held;
}

/* Zo as the MCU sees it: a short glitch now and then inside the noise window. */
#define GLITCH_EVERY 4099u
#define GLITCH_COUNTS 3u
static uint8_t zo_level(uint32_t t) {
  uint8_t v = zo_true(t);
  if (t >= noise_from && t < noise_to && t % GLITCH_EVERY < GLITCH_COUNTS)
    return !v;
  return v;
}

/* Starts the application as after a reset, with the thermistor average ready unless the test says
   otherwise, mains on, no Zo stop and no noise. */
static void sim_init(uint32_t half) {
  half_c = half;
  zo_stop_from = zo_stop_to = 0;
  noise_from = noise_to = 0;
  led_load = 0;
  conducted_at = relay_changed_at = 0;
  mains_gone_from = mains_gone_to = 0;
  shift_at = UINT32_MAX;
  shift_by = 0;
  zo_jitter = 0;
  last_violation_at = 0;
  timer_starts = 0;
  most_missed = 0;
  sched_init(&s);
  pin = armed = 0;
  fire_delay = fire_next = 0;
  fire_at = 0;
  sim_t = 0;
  sim_ms = 0;
  sim_therm = THERM_ROOM;
  sim_therm_ready = 1;
  relay_pin = 0;
  conducting = 0;
  relay_breaks = old_delay_fires = 0;
  fault_fires = 0;
  gate_violations = 0;
  last_mains = 0;
  gate_stuck_low = 0;
  glitch_edges = 0;
  wdt_count_clears = 0;
  app_init();
}

/* One PCA count of the hardware: the delay hand-over, Zo edges, compare matches and the triac,
   and the gate timing rules. Returns 1 when the gate rose. */
static uint8_t sim_count(uint32_t t, uint8_t *last_level) {
  uint16_t x, now = (uint16_t) t;
  uint8_t level, a, rose = 0;
  uint32_t pos;
  sim_t = t;
  if (t == fire_at)
    fire_delay = fire_next;
  if (mains_t(t) % half_c == 0 || mains_gone(t))
    conducting = 0;
  if (conducting)
    conducted_at = t;
  if (zo_mains(t) != last_mains) {
    last_mains = !last_mains;
    if (pin && !mains_gone(t)) {
      gate_violations++;
      last_violation_at = t;
    }
  }
  level = zo_level(t);
  if (level != *last_level) {
    *last_level = level;
    if (level != zo_true(t))
      glitch_edges++;
    a = sched_edge(&s, now, zo_level(t + ISR_LATENCY), fire_delay, &x);
    apply(a, x);
  }
  if (armed && now == cmp) {
    if (!pin && gate_stuck_low)
      gate_stuck_low = 0;
    else
      pin ^= 1;
    if (pin) {
      rose = 1;
      conducting = 1;
      if (app_gate.state != ST_FIRING)
        old_delay_fires++;
      /* The relay contacts would carry the current while they open. */
      if (!relay_pin)
        relay_breaks++;
    }
    a = sched_match(&s, pin, &x);
    apply(a, x);
  }
  /* PCA module 1, the reference timer, as hal.c sets it. */
  if (phase_locked(&s.phase) && now == s.phase.next_at) {
    a = sched_edge(&s, now, PHASE_TIMER, fire_delay, &x);
    timer_starts += phase_locked(&s.phase);
    apply(a, x);
  }
  if (s.phase.missed > most_missed)
    most_missed = s.phase.missed;
  if (pin && !mains_gone(t)) {
    pos = mains_t(t) % half_c;
    /* Half-cycles start at the falling Zo edge, ZO_SKEW before the crossing, and half a period
       later. */
    if (pos < EARLIEST_FIRE - ZO_SKEW - zo_jitter || pos > half_c - GATE_LOW_BEFORE_CROSSING) {
      gate_violations++;
      last_violation_at = t;
    }
    if (app_gate.state != ST_FIRING && app_gate.state != ST_STOPPING && app_gate.state != ST_FAULT) {
      gate_violations++;
      last_violation_at = t;
    }
  }
  return rose;
}

/* The 1 ms tick interrupt's Zo and gate checks, then one pass of the main loop for that tick. */
static void sim_tick(uint32_t t) {
  if (sched_zo_lost(&s))
    sched_zo_reset(&s);
  apply(sched_tick_check(&s, pin, (uint16_t) t), 0);
  sim_ms++;
  app_loop();
  if (app_gate.state == ST_FAULT && app_gate.fire)
    fault_fires++;
}

typedef struct {
  double at;
  uint16_t target;
} step_t;

static const step_t script[] = {
    {0.2, 4000}, {2.0, 1500}, {4.0, 6500}, {5.0, 60000}, {6.0, 0},
    {7.0, 3000}, {9.5, 0},
};

static void run(uint32_t half) {
  uint32_t t, next_ms = MS, end = SECONDS(10.0);
  uint8_t last_level = 0;
  unsigned step = 0, pulses = 0, gate_faults = 0;
  uint8_t checked_fade = 0, saw_fault = 0;

  sim_init(half);
  zo_jitter = PCA_US(15);
  /* Longer than ZO_MISSED_MAX periods: Zo lost. */
  zo_stop_from = SECONDS(8.0);
  zo_stop_to = SECONDS(8.6);
  noise_from = SECONDS(1.0);
  noise_to = SECONDS(1.5);

  for (t = 0; t < end; t++) {
    pulses += sim_count(t, &last_level);

    if (t == next_ms) {
      next_ms += MS;
      while (step < sizeof script / sizeof script[0] && t >= SECONDS(script[step].at)) {
        gate_set_target(&app_gate, script[step].target, LIMIT_BRIGHTER, LIMIT_DIMMER);
        step++;
      }

      sim_tick(t);
      gate_faults += (app_status.faults & F_GATE) != 0;
      if (app_gate.state == ST_FAULT)
        saw_fault = 1;

      /* Steady firing at the target. */
      if (!checked_fade && t >= SECONDS(3.9)) {
        checked_fade = 1;
        CHECK_EQ(app_gate.state, ST_FIRING);
        CHECK_EQ(app_gate.current, 1500);
      }
      if (t == SECONDS(5.5) / MS * MS) {
        CHECK_EQ(app_gate.clamped_target, half - LATEST_BEFORE_EDGE);
      }
      if (t >= SECONDS(6.5) && t < SECONDS(6.6)) {
        CHECK_EQ(app_gate.state, ST_OFF);
        CHECK_EQ(relay_pin, 0);
      }
      if (t >= SECONDS(8.0) && t < SECONDS(8.0) + (ZO_MISSED_MAX - 1) * 2 * half)
        CHECK_EQ(app_gate.state, ST_FIRING);
      /* Missed periods count from the first falling half-cycle after the stop. */
      if (t >= SECONDS(8.0) + (ZO_MISSED_MAX + 2) * 2 * half + (FAULT_RELAY_MS + 2) * MS && t < SECONDS(8.6)) {
        CHECK_EQ(relay_pin, 0);
        CHECK_EQ(pin, 0);
      }
      /* Zo is back, but the lamp stays off until the ESP32 sends target 0. */
      if (t >= SECONDS(9.0) && t < SECONDS(9.4)) {
        CHECK_EQ(app_gate.state, ST_FAULT);
        CHECK_EQ(relay_pin, 0);
      }
    }
  }
  CHECK_EQ(gate_violations, 0);
  CHECK_EQ(gate_faults, 0);
  CHECK_EQ(relay_breaks, 0);
  CHECK(pulses > SECONDS(6.25) / half);
  CHECK(saw_fault);
  CHECK_EQ(app_gate.state, ST_OFF);
  CHECK(glitch_edges > 0);
}

/* At a steady delay, a falling half-cycle fires at its falling edge + delay, and a rising one half a
   period later: 2 * ZO_SKEW before its own rising edge + delay. */
static void test_fire_offsets(void) {
  const uint16_t delay = 4000;
  uint32_t t, last_fall = 0;
  uint8_t last_level = 0, was = 0;
  int falls = 0, rises = 0, ok_f = 0, ok_r = 0;

  sim_init(HALF_60HZ);
  fire_next = delay;
  fire_at = 1;
  for (t = 0; t < SECONDS(1.0); t++) {
    uint8_t lvl = zo_true(t);
    if (was && !lvl)
      last_fall = t;
    was = lvl;
    if (sim_count(t, &last_level) && t > SECONDS(0.5)) {
      if (t - last_fall < HALF_60HZ) {
        falls++;
        ok_f += t - last_fall == delay;
      } else {
        rises++;
        ok_r += t - last_fall == HALF_60HZ + delay;
      }
    }
  }
  CHECK(rises > 20 && falls > 20);
  CHECK_EQ(ok_f, falls);
  CHECK_EQ(ok_r, rises);
}

static void run_60hz(void) { run(HALF_60HZ); }
static void run_50hz(void) { run(HALF_50HZ); }

/* Zo noise from shortly before 'from' to the end of the run. */
static void noise_after(uint32_t from, uint32_t end) {
  noise_from = from - SECONDS(0.05);
  noise_to = end;
}

/*
 * Turns the lamp off at every position of the 1 ms tick against the Zo edges, including the
 * ticks that read the edge count just before an edge whose interrupt still sees the old delay.
 * The relay must never open while the triac conducts, also with Zo noise across the turn-off.
 */
static void turn_off_at_any_phase(uint8_t noisy) {
  uint32_t t, next_ms, off_at = SECONDS(0.4), end = SECONDS(0.45);
  uint16_t tick_phase;
  uint8_t last_level, off_sent;
  unsigned old_fires = 0, runs = 0, glitches = 0;

  for (tick_phase = 0; tick_phase < MS; tick_phase += 13) {
    sim_init(HALF_60HZ);
    if (noisy)
      noise_after(off_at, end);
    gate_set_target(&app_gate, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER);
    next_ms = tick_phase + MS;
    last_level = 0;
    off_sent = 0;
    for (t = 0; t < end; t++) {
      sim_count(t, &last_level);
      if (t == next_ms) {
        next_ms += MS;
        if (!off_sent && t >= off_at) {
          off_sent = 1;
          CHECK_EQ(app_gate.state, ST_FIRING);
          gate_set_target(&app_gate, 0, LIMIT_BRIGHTER, LIMIT_DIMMER);
        }
        sim_tick(t);
      }
    }
    CHECK_EQ(app_gate.state, ST_OFF);
    CHECK_EQ(relay_pin, 0);
    CHECK_EQ(relay_breaks, 0);
    CHECK_EQ(gate_violations, 0);
    old_fires += old_delay_fires;
    glitches += glitch_edges;
    runs++;
  }
  /* Some runs did fire once more with the old delay after the turn-off. */
  CHECK(old_fires > 0);
  CHECK(runs > 60);
  CHECK(!noisy || glitches > 0);
}

static void test_turn_off_at_any_phase(void) { turn_off_at_any_phase(0); }
static void test_turn_off_with_noise(void) { turn_off_at_any_phase(1); }

enum { FAULT_OVERTEMP, FAULT_OVERTEMP_ZO_STOP, FAULT_GATE };

/*
 * A fault while firing, at every position of the 1 ms tick against the Zo edges: the triac is
 * never fired by an edge after the delay was cleared, the relay never opens while it conducts,
 * and it opens within FAULT_RELAY_MS. FAULT_OVERTEMP_ZO_STOP also stops Zo at the fault (mains
 * stays), so only the time limit can open the relay. FAULT_GATE: the gate pin stays low at a fire
 * match, which the PCA interrupt reports as a gate fault.
 */
static void fault_at_any_phase(uint32_t half, uint8_t kind, uint8_t noisy) {
  uint32_t t, next_ms, fault_at = SECONDS(0.4), end = SECONDS(0.45);
  uint16_t tick_phase, fault_ms, open_ms;
  uint8_t last_level, injected, faults_shown;
  unsigned runs = 0, held = 0, old_fires = 0, glitches = 0;

  for (tick_phase = 0; tick_phase < MS; tick_phase += 13) {
    sim_init(half);
    if (noisy)
      noise_after(fault_at, end);
    gate_set_target(&app_gate, 6000, LIMIT_BRIGHTER, LIMIT_DIMMER);
    next_ms = tick_phase + MS;
    fault_ms = open_ms = 0;
    last_level = 0;
    injected = 0;
    faults_shown = 0;
    for (t = 0; t < end; t++) {
      sim_count(t, &last_level);
      if (t == next_ms) {
        next_ms += MS;
        if (!injected && t >= fault_at) {
          injected = 1;
          CHECK_EQ(app_gate.state, ST_FIRING);
          if (kind == FAULT_GATE) {
            gate_stuck_low = 1;
          } else {
            sim_therm = THERM_TRIP;
            if (kind == FAULT_OVERTEMP_ZO_STOP) {
              zo_stop_from = t;
              zo_stop_to = end;
            }
          }
        }
        sim_tick(t);
        faults_shown |= app_status.faults;
        if (!fault_ms && app_gate.state == ST_FAULT)
          fault_ms = sim_ms;
        if (fault_ms && !open_ms && !relay_pin)
          open_ms = sim_ms;
      }
    }
    CHECK_EQ(app_gate.state, ST_FAULT);
    CHECK_EQ(relay_pin, 0);
    CHECK_EQ(relay_breaks, 0);
    CHECK_EQ(fault_fires, 0);
    CHECK_EQ(gate_violations, 0);
    CHECK(faults_shown & (kind == FAULT_GATE ? F_GATE : F_OVERTEMP));
    /* At most the one pulse armed by an edge before the cleared delay reached the interrupt. */
    CHECK(old_delay_fires <= 1);
    old_fires += old_delay_fires;
    CHECK(fault_ms != 0 && open_ms != 0 && (uint16_t) (open_ms - fault_ms) <= FAULT_RELAY_MS);
    held += open_ms != fault_ms;
    glitches += glitch_edges;
    runs++;
  }
  CHECK_EQ(held, runs); /* the relay never opened in the tick that saw the fault */
  /* A gate fault comes from a fire match, well before the next edge, so only the others can meet
     an edge whose interrupt still has the old delay. */
  CHECK(kind == FAULT_GATE || old_fires > 0);
  CHECK(runs > 60);
  CHECK(!noisy || glitches > 0);
}

static void test_fault_at_any_phase(void) {
  fault_at_any_phase(HALF_60HZ, FAULT_OVERTEMP, 0);
  fault_at_any_phase(HALF_50HZ, FAULT_OVERTEMP, 0);
  /* The slowest mains: the time limit comes before the second edge. */
  fault_at_any_phase(PERIOD_MAX / 2u, FAULT_OVERTEMP, 0);
}
static void test_fault_zo_stop_at_any_phase(void) { fault_at_any_phase(HALF_60HZ, FAULT_OVERTEMP_ZO_STOP, 0); }

/* Zo stops while firing: the timer starts the half-cycles on for ZO_MISSED_MAX - 1 periods, then
   FAULT and F_ZO. */
static void test_zo_stop_while_firing(void) {
  uint32_t t, next_ms = MS, stop_at = SECONDS(0.4), end = SECONDS(0.8);
  uint8_t last_level = 0;
  unsigned pulses_while_stopped = 0;
  sim_init(HALF_60HZ);
  gate_set_target(&app_gate, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER);
  zo_stop_from = stop_at;
  zo_stop_to = end;
  for (t = 0; t < end; t++) {
    pulses_while_stopped += sim_count(t, &last_level) && t >= stop_at;
    if (t == next_ms) {
      next_ms += MS;
      sim_tick(t);
      if (t >= stop_at - SECONDS(0.1) && t < stop_at + (ZO_MISSED_MAX - 2) * 2 * HALF_60HZ)
        CHECK(app_gate.state == ST_FIRING && s.zo_valid);
      else if (t > stop_at + (ZO_MISSED_MAX + 1) * 2 * HALF_60HZ + MS)
        CHECK(app_gate.state == ST_FAULT && !s.zo_valid);
    }
  }
  CHECK(app_status.faults & F_ZO);
  CHECK_EQ(most_missed, ZO_MISSED_MAX - 1);
  CHECK(pulses_while_stopped >= 2 * (ZO_MISSED_MAX - 2));
  CHECK_EQ(relay_pin, 0);
  CHECK_EQ(relay_breaks, 0);
  CHECK_EQ(gate_violations, 0);
}

/* Zo stops while firing, for less than ZO_MISSED_MAX periods: the lamp fires on at the right times,
   at every length of the gap. */
static void test_zo_gap_while_firing(void) {
  uint32_t t, next_ms, stop_at = SECONDS(0.4), end = SECONDS(1.0), gap;
  uint8_t last_level;
  unsigned runs = 0;
  for (gap = MS; gap < (ZO_MISSED_MAX - 2) * 2 * HALF_60HZ; gap += 7 * MS) {
    sim_init(HALF_60HZ);
    gate_set_target(&app_gate, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER);
    zo_stop_from = stop_at;
    zo_stop_to = stop_at + gap;
    next_ms = MS;
    last_level = 0;
    for (t = 0; t < end; t++) {
      sim_count(t, &last_level);
      if (t == next_ms) {
        next_ms += MS;
        sim_tick(t);
        if (t > SECONDS(0.3))
          CHECK_EQ(app_gate.state, ST_FIRING);
      }
    }
    CHECK_EQ(s.phase.missed, 0);
    CHECK(s.zo_valid);
    CHECK_EQ(app_gate.current, 4000);
    CHECK_EQ(gate_violations, 0);
    CHECK_EQ(relay_breaks, 0);
    runs++;
  }
  CHECK(runs > 20);
}

/*
 * LED lamps (led_load): no Zo while the relay is closed and the triac idle, falling edges late after
 * the relay closes, rising edges drifting late after it opens. The lamp turns on for on_s and off for
 * off_s, again and again from 0.3 s, at every position of the tick against Zo, without a fault and
 * within every gate timing rule.
 */
static void led_load_on_off(double on_s, double off_s, double end_s) {
  uint32_t t, next_ms, end = SECONDS(end_s), cycle = SECONDS(on_s + off_s), into;
  uint16_t tick_phase;
  uint8_t last_level, on, most = 0;
  unsigned runs = 0, turn_ons, turn_offs;
  for (tick_phase = 0; tick_phase < MS; tick_phase += 37) {
    sim_init(HALF_60HZ);
    led_load = 1;
    zo_jitter = PCA_US(15);
    next_ms = tick_phase + MS;
    last_level = 0;
    on = 0;
    turn_ons = turn_offs = 0;
    for (t = 0; t < end; t++) {
      sim_count(t, &last_level);
      if (t != next_ms)
        continue;
      next_ms += MS;
      into = t >= SECONDS(0.3) ? (t - SECONDS(0.3)) % cycle : cycle;
      if (into < MS) {
        gate_set_target(&app_gate, 6000, LIMIT_BRIGHTER, LIMIT_DIMMER);
        on = 1;
        turn_ons++;
      } else if (on && into >= SECONDS(on_s)) {
        gate_set_target(&app_gate, 0, LIMIT_BRIGHTER, LIMIT_DIMMER);
        on = 0;
        turn_offs++;
      }
      sim_tick(t);
      CHECK(app_gate.state != ST_FAULT);
      if (on && into == SECONDS(0.1) / MS * MS)
        CHECK_EQ(app_gate.state, ST_FIRING);
    }
    CHECK(turn_ons > 1 && turn_offs > 1);
    CHECK_EQ(gate_violations, 0);
    CHECK_EQ(relay_breaks, 0);
    CHECK_EQ(fault_fires, 0);
    if (most_missed > most)
      most = most_missed;
    runs++;
  }
  CHECK(runs > 20);
  CHECK(most > 0); /* Zo was missing at times */
}

static void test_led_load_quick_on_off(void) { led_load_on_off(0.15, 0.15, 2.0); }
/* As tested on unit 1: on for seconds, and on again while the rising edges still drift. */
static void test_led_load_on_off_as_used(void) { led_load_on_off(2.0, 1.0, 7.0); }


/*
 * Mains goes while firing, and comes back a quarter period later (another phase, as after a
 * transfer switch). Nothing can conduct while it is gone. Once back, the gate may fire on the old
 * timing only until the lock lets go, within 70 ms (ZO_OUTSIDE_CAPTURE_MAX + 1 periods at 60 Hz); then FAULT,
 * and nothing conducts when the relay opens.
 */
static void test_mains_back_at_other_phase(void) {
  uint32_t t, next_ms = MS, gone = SECONDS(0.4), back = gone + 5 * 2 * HALF_60HZ, end = SECONDS(1.0);
  uint32_t fault_at = 0;
  uint8_t last_level = 0;
  sim_init(HALF_60HZ);
  gate_set_target(&app_gate, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER);
  mains_gone_from = gone;
  mains_gone_to = back;
  shift_at = back;
  shift_by = 2 * HALF_60HZ - HALF_60HZ / 2;
  for (t = 0; t < end; t++) {
    sim_count(t, &last_level);
    if (t == next_ms) {
      next_ms += MS;
      sim_tick(t);
      if (!fault_at && app_gate.state == ST_FAULT)
        fault_at = t;
    }
  }
  CHECK(fault_at > back);
  CHECK(fault_at <= back + SECONDS(0.07));
  CHECK(last_violation_at <= fault_at);
  CHECK(gate_violations == 0 || last_violation_at >= back);
  CHECK_EQ(relay_pin, 0);
  CHECK_EQ(relay_breaks, 0);
}

static void test_gate_fault_at_any_phase(void) {
  fault_at_any_phase(HALF_60HZ, FAULT_GATE, 0);
  fault_at_any_phase(HALF_50HZ, FAULT_GATE, 0);
}
static void test_fault_with_noise(void) {
  fault_at_any_phase(HALF_60HZ, FAULT_OVERTEMP, 1);
  fault_at_any_phase(HALF_60HZ, FAULT_GATE, 1);
}

enum { STARTUP_CLEAN, STARTUP_HOT, STARTUP_GATE_FAULT };

/*
 * The start-up FAULT: the thermistor average is not ready at first, and a non-zero target arrives
 * meanwhile. With nothing else wrong, the lamp starts with that target once the average is ready.
 * With overtemperature at that point (STARTUP_HOT), or a gate fault seen before it
 * (STARTUP_GATE_FAULT), the FAULT stays after the cause has gone, until a target of 0. Ready
 * before and after Zo is good, at every position of the tick against Zo.
 */
static void startup(uint16_t ready_ms, uint8_t kind) {
  uint32_t t, next_ms, end = SECONDS(0.6);
  uint16_t tick_phase;
  uint8_t last_level;
  unsigned runs = 0;

  for (tick_phase = 0; tick_phase < MS; tick_phase += 29) {
    sim_init(HALF_60HZ);
    sim_therm_ready = 0;
    next_ms = tick_phase + MS;
    last_level = 0;
    for (t = 0; t < end; t++) {
      sim_count(t, &last_level);
      if (t != next_ms)
        continue;
      next_ms += MS;
      if (sim_ms == 5)
        gate_set_target(&app_gate, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER);
      if (kind == STARTUP_GATE_FAULT && sim_ms == ready_ms / 2)
        s.fault++;
      if (sim_ms + 1 == ready_ms) {
        sim_therm_ready = 1;
        if (kind == STARTUP_HOT)
          sim_therm = THERM_TRIP;
      }
      if (kind == STARTUP_HOT && sim_ms == ready_ms + 50)
        sim_therm = THERM_ROOM;
      if (kind != STARTUP_CLEAN && sim_ms == ready_ms + 200) {
        CHECK_EQ(app_gate.state, ST_FAULT);
        gate_set_target(&app_gate, 0, LIMIT_BRIGHTER, LIMIT_DIMMER);
      }
      if (kind != STARTUP_CLEAN && sim_ms == ready_ms + 205)
        gate_set_target(&app_gate, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER);
      sim_tick(t);
      if (sim_ms < ready_ms) {
        CHECK_EQ(app_gate.state, ST_FAULT);
        CHECK_EQ(app_status.starting, 1);
      }
      if (sim_ms < (kind == STARTUP_CLEAN ? ready_ms : ready_ms + 200))
        CHECK_EQ(relay_pin, 0);
    }
    CHECK_EQ(app_gate.state, ST_FIRING);
    CHECK_EQ(app_gate.current, 4000);
    CHECK_EQ(app_status.starting, 0);
    CHECK_EQ(relay_breaks, 0);
    CHECK_EQ(fault_fires, 0);
    CHECK_EQ(gate_violations, 0);
    runs++;
  }
  CHECK(runs > 30);
}

static void test_wdt_count_cleared_once_after_running(void) {
  uint32_t ms;
  sim_init(HALF_60HZ);
  for (ms = 1; ms < 2u * WDT_CLEAR_COUNT_MS; ms++)
    sim_tick(ms * MS);
  CHECK_EQ(wdt_count_clears, 1);
  CHECK_EQ(wdt_count_cleared_ms, WDT_CLEAR_COUNT_MS);
}

static void test_startup_fault(void) {
  startup(80, STARTUP_CLEAN);
  startup(300, STARTUP_CLEAN);
  startup(80, STARTUP_HOT);
  startup(300, STARTUP_HOT);
  startup(80, STARTUP_GATE_FAULT);
  startup(300, STARTUP_GATE_FAULT);
}

int main(void) {
  RUN(run_60hz);
  RUN(run_50hz);
  RUN(test_turn_off_at_any_phase);
  RUN(test_turn_off_with_noise);
  RUN(test_fault_at_any_phase);
  RUN(test_fault_zo_stop_at_any_phase);
  RUN(test_zo_stop_while_firing);
  RUN(test_zo_gap_while_firing);
  RUN(test_led_load_quick_on_off);
  RUN(test_led_load_on_off_as_used);
  RUN(test_mains_back_at_other_phase);
  RUN(test_gate_fault_at_any_phase);
  RUN(test_fault_with_noise);
  RUN(test_startup_fault);
  RUN(test_fire_offsets);
  RUN(test_wdt_count_cleared_once_after_running);
  TEST_EXIT();
}
