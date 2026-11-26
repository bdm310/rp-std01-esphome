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
static uint8_t last_true;
static uint8_t gate_stuck_low; /* the gate pin stays low at the next fire match: a gate fault */

static uint32_t zo_stop_from, zo_stop_to;
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

/* Zo without noise. Holds its level while Zo is stopped. */
static uint8_t zo_true(uint32_t t) {
  static uint8_t held;
  uint32_t pos;
  if (t >= zo_stop_from && t < zo_stop_to)
    return held;
  pos = (t + 2 * half_c - ZO_SKEW) % (2 * half_c);
  held = pos < half_c - 2 * ZO_SKEW ? 1 : 0;
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
  last_true = 0;
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
  if (t % half_c == 0)
    conducting = 0;
  if (zo_true(t) != last_true) {
    last_true = !last_true;
    if (pin)
      gate_violations++;
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
  if (pin) {
    pos = t % half_c;
    if (pos < ZO_SKEW + EARLIEST_FIRE || pos > half_c - GATE_LOW_BEFORE_CROSSING)
      gate_violations++;
    if (app_gate.state != ST_FIRING && app_gate.state != ST_STOPPING && app_gate.state != ST_FAULT)
      gate_violations++;
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
  zo_stop_from = SECONDS(8.0);
  zo_stop_to = SECONDS(8.1);
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
      if (t >= SECONDS(8.05) && t < SECONDS(8.1)) {
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

static void test_fire_offsets(void) {
  /* At a steady delay, both half-cycles fire at reference + delay: rising half at edge +
     delay, falling half 2 * ZO_SKEW later than its own edge, as stock. */
  const uint16_t delay = 4000;
  uint32_t t;
  uint16_t x;
  uint8_t last = 0, lvl;
  int rises = 0, falls = 0, ok_r = 0, ok_f = 0;
  uint32_t edge_t = 0;
  uint8_t edge_lvl = 0, a;

  sim_init(HALF_60HZ);
  for (t = 0; t < SECONDS(1.0); t++) {
    uint16_t now = (uint16_t) t;
    lvl = zo_level(t);
    if (lvl != last) {
      last = lvl;
      a = sched_edge(&s, now, lvl, delay, &x);
      apply(a, x);
      edge_t = t;
      edge_lvl = lvl;
    }
    if (armed && now == cmp) {
      pin ^= 1;
      if (pin && t > SECONDS(0.5)) {
        if (edge_lvl) {
          rises++;
          ok_r += t - edge_t == delay;
        } else {
          falls++;
          ok_f += t - edge_t == delay + 2 * ZO_SKEW;
        }
      }
      a = sched_match(&s, pin, &x);
      apply(a, x);
    }
  }
  CHECK(rises > 20 && falls > 20);
  CHECK_EQ(ok_r, rises);
  CHECK_EQ(ok_f, falls);
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

/* Zo stops while firing: the lamp fires on for ZO_LOST_MS after the last edge, then FAULT and F_ZO
   within the next tick. */
static void test_zo_stop_while_firing(void) {
  uint32_t t, next_ms = MS, stop_at = SECONDS(0.4), end = SECONDS(0.5), last_edge = 0;
  uint8_t last_level = 0, edges = 0;
  sim_init(HALF_60HZ);
  gate_set_target(&app_gate, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER);
  zo_stop_from = stop_at;
  zo_stop_to = end;
  for (t = 0; t < end; t++) {
    sim_count(t, &last_level);
    if (s.edges != edges) {
      edges = s.edges;
      last_edge = t;
    }
    if (t == next_ms) {
      next_ms += MS;
      sim_tick(t);
      if (t < stop_at)
        CHECK(t < stop_at - SECONDS(0.1) || app_gate.state == ST_FIRING);
      else if (t < last_edge + ZO_LOST_MS * MS)
        CHECK(app_gate.state == ST_FIRING && s.zo_valid);
      else if (t > last_edge + (ZO_LOST_MS + 1) * MS)
        CHECK(app_gate.state == ST_FAULT && !s.zo_valid);
    }
  }
  CHECK(app_status.faults & F_ZO);
  CHECK_EQ(relay_pin, 0);
  CHECK_EQ(relay_breaks, 0);
  CHECK_EQ(gate_violations, 0);
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
  RUN(test_gate_fault_at_any_phase);
  RUN(test_fault_with_noise);
  RUN(test_startup_fault);
  RUN(test_fire_offsets);
  RUN(test_wdt_count_cleared_once_after_running);
  TEST_EXIT();
}
