#include "board.h"
#include "gate.h"
#include "protocol.h"
#include "test.h"

#define HALF 7680u

static gate_t g;
static uint16_t ms;
static uint8_t edges;

static void tick(uint8_t zo_ok, uint8_t faults) {
  ms++;
  gate_tick(&g, ms, edges, zo_ok ? HALF : 0, zo_ok, faults);
}

static void ticks(int n, uint8_t zo_ok, uint8_t faults) {
  while (n--)
    tick(zo_ok, faults);
}

static void start(void) {
  gate_init(&g);
  ms = 100;
  edges = 0;
}

static void test_off_until_target(void) {
  start();
  ticks(50, 1, 0);
  CHECK_EQ(g.state, ST_OFF);
  CHECK_EQ(g.relay, 0);
  CHECK_EQ(g.fire, 0);
}

static void test_turn_on_sequence(void) {
  unsigned i;
  start();
  gate_set_target(&g, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER);
  tick(1, 0);
  CHECK_EQ(g.state, ST_SETTLE);
  CHECK_EQ(g.relay, 1);
  CHECK_EQ(g.fire, 0);
  for (i = 1; i < RELAY_SETTLE_MS; i++)
    tick(1, 0);
  CHECK_EQ(g.state, ST_SETTLE);
  CHECK_EQ(g.fire, 0);
  tick(1, 0);
  CHECK_EQ(g.state, ST_FIRING);
  CHECK_EQ(g.fire, 4000);
}

static void test_needs_zo_to_start(void) {
  start();
  gate_set_target(&g, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER);
  ticks(50, 0, 0);
  CHECK_EQ(g.state, ST_OFF);
  CHECK_EQ(g.relay, 0);
}

static void on(uint16_t target) {
  start();
  gate_set_target(&g, target, LIMIT_BRIGHTER, LIMIT_DIMMER);
  ticks(RELAY_SETTLE_MS + 1, 1, 0);
  CHECK_EQ(g.state, ST_FIRING);
}

static void test_smoothing(void) {
  on(4000);
  gate_set_target(&g, 3000, LIMIT_BRIGHTER, LIMIT_DIMMER);
  tick(1, 0);
  gate_half_cycle(&g);
  CHECK_EQ(g.fire, 4000 - LIMIT_BRIGHTER);
  gate_half_cycle(&g);
  CHECK_EQ(g.fire, 4000 - 2 * LIMIT_BRIGHTER);
  gate_set_target(&g, 3990, LIMIT_BRIGHTER, LIMIT_DIMMER); /* dimmer, closer than one step */
  tick(1, 0);
  gate_half_cycle(&g);
  CHECK_EQ(g.fire, 3990);
  gate_set_target(&g, 5000, LIMIT_BRIGHTER, LIMIT_DIMMER);
  tick(1, 0);
  gate_half_cycle(&g);
  CHECK_EQ(g.fire, 3990 + 23);
  gate_set_target(&g, 2000, 0, 0); /* no limit */
  tick(1, 0);
  gate_half_cycle(&g);
  CHECK_EQ(g.fire, 2000);
}

static void test_clamped_target(void) {
  on(60000);
  CHECK_EQ(g.clamped_target, HALF - LATEST_BEFORE_EDGE);
  CHECK_EQ(g.fire, HALF - LATEST_BEFORE_EDGE);
}

static void test_turn_off_sequence(void) {
  on(4000);
  gate_set_target(&g, 0, LIMIT_BRIGHTER, LIMIT_DIMMER);
  tick(1, 0);
  CHECK_EQ(g.state, ST_STOPPING);
  CHECK_EQ(g.fire, 0);
  CHECK_EQ(g.relay, 1);
  ticks(5, 1, 0);
  CHECK_EQ(g.relay, 1);
  edges++;
  tick(1, 0);
  CHECK_EQ(g.relay, 1);
  ticks(5, 1, 0); /* one edge is not enough */
  CHECK_EQ(g.relay, 1);
  edges++;
  tick(1, 0);
  CHECK_EQ(g.relay, 1);
  tick(1, 0);
  CHECK_EQ(g.relay, 1);
  tick(1, 0);
  CHECK_EQ(g.state, ST_OFF);
  CHECK_EQ(g.relay, 0);
}

/* A Zo edge between the tick reading 'edges' and the main loop clearing the delay still fires
   with the old delay. The relay must stay on through that half-cycle. */
static void test_turn_off_edge_before_fire_cleared(void) {
  int i;
  on(4000);
  gate_set_target(&g, 0, LIMIT_BRIGHTER, LIMIT_DIMMER);
  tick(1, 0); /* reads edges, sets fire 0 ... */
  edges++;    /* ... but this edge armed the old delay before fire 0 reached the interrupt */
  for (i = 0; i < 8; i++) { /* the whole half-cycle that may be firing */
    tick(1, 0);
    CHECK_EQ(g.relay, 1);
  }
  edges++; /* the first edge seen with fire 0 */
  ticks(RELAY_OFF_DELAY_MS + 1, 1, 0);
  CHECK_EQ(g.state, ST_OFF);
  CHECK_EQ(g.relay, 0);
}

/* A fault clears the delay at once, but the relay opens like a normal turn-off: the half-cycle
   fired before the fault may still conduct. */
static void test_fault_from_firing(void) {
  on(4000);
  tick(1, F_OVERTEMP);
  CHECK_EQ(g.state, ST_FAULT);
  CHECK_EQ(g.fire, 0);
  CHECK_EQ(g.relay, 1);
  edges++;
  tick(1, F_OVERTEMP);
  CHECK_EQ(g.relay, 1);
  edges++; /* the second edge since the fault: nothing conducts any more */
  tick(1, F_OVERTEMP);
  CHECK_EQ(g.relay, 1);
  tick(1, F_OVERTEMP);
  CHECK_EQ(g.relay, 1);
  tick(1, F_OVERTEMP);
  CHECK_EQ(g.relay, 0);
  CHECK_EQ(g.fire, 0);
  ticks(10, 1, 0); /* cause gone, but no target 0 yet */
  CHECK_EQ(g.state, ST_FAULT);
  gate_set_target(&g, 0, LIMIT_BRIGHTER, LIMIT_DIMMER);
  tick(1, 0);
  CHECK_EQ(g.state, ST_OFF);
}

static void test_fault_relay_time_limit(void) {
  unsigned i;
  on(4000);
  tick(1, F_THERM);
  CHECK_EQ(g.state, ST_FAULT);
  CHECK_EQ(g.relay, 1);
  for (i = 1; i < FAULT_RELAY_MS; i++) {
    tick(1, F_THERM);
    CHECK_EQ(g.relay, 1);
    CHECK_EQ(g.fire, 0);
  }
  tick(1, F_THERM);
  CHECK_EQ(g.relay, 0);
}

/* The fault never fires the triac again, even if the cause goes and a new target comes while the
   relay is still on; it leaves FAULT only with the relay open. */
static void test_fault_never_fires(void) {
  unsigned i;
  on(4000);
  tick(1, F_GATE);
  gate_set_target(&g, 0, LIMIT_BRIGHTER, LIMIT_DIMMER);
  for (i = 0; i < 4; i++) {
    tick(1, 0);
    gate_half_cycle(&g);
    CHECK_EQ(g.state, ST_FAULT);
    CHECK_EQ(g.relay, 1);
    CHECK_EQ(g.fire, 0);
  }
  gate_set_target(&g, 5000, LIMIT_BRIGHTER, LIMIT_DIMMER);
  edges += 2;
  for (i = 0; i < FAULT_RELAY_MS; i++) {
    tick(1, 0);
    gate_half_cycle(&g);
    CHECK_EQ(g.fire, 0);
  }
  CHECK_EQ(g.relay, 0);
  CHECK_EQ(g.state, ST_FAULT); /* the 5000 came after the 0 */
}

static void test_fault_needs_cause_gone(void) {
  on(4000);
  tick(1, F_GATE);
  gate_set_target(&g, 0, LIMIT_BRIGHTER, LIMIT_DIMMER);
  ticks(FAULT_RELAY_MS, 1, F_OVERTEMP);
  CHECK_EQ(g.relay, 0);
  CHECK_EQ(g.state, ST_FAULT);
  tick(1, 0);
  CHECK_EQ(g.state, ST_OFF);
}

static void test_zo_lost_while_on(void) {
  on(4000);
  tick(0, 0);
  CHECK_EQ(g.state, ST_FAULT);
  CHECK_EQ(g.fire, 0);
  ticks(FAULT_RELAY_MS - 1, 0, 0);
  CHECK_EQ(g.relay, 1);
  tick(0, 0);
  CHECK_EQ(g.relay, 0);
}

static void test_zo_lost_while_settling_and_stopping(void) {
  start();
  gate_set_target(&g, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER);
  tick(1, 0);
  tick(0, 0);
  CHECK_EQ(g.state, ST_FAULT);
  CHECK_EQ(g.relay, 0); /* not fired yet: opens at once */

  on(4000);
  gate_set_target(&g, 0, LIMIT_BRIGHTER, LIMIT_DIMMER);
  tick(1, 0);
  edges++;
  tick(1, 0);
  tick(0, 0);
  CHECK_EQ(g.state, ST_FAULT);
  CHECK_EQ(g.relay, 1);
  edges++; /* the edge count started at the turn-off goes on */
  tick(0, 0);
  ticks(RELAY_OFF_DELAY_MS, 0, 0);
  CHECK_EQ(g.relay, 0);
}

static void test_fault_while_off_blocks_start(void) {
  start();
  tick(1, F_THERM);
  CHECK_EQ(g.state, ST_FAULT);
  gate_set_target(&g, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER);
  ticks(20, 1, F_THERM);
  CHECK_EQ(g.relay, 0);
  ticks(20, 1, 0); /* cleared, but the ESP32 must send 0 first */
  CHECK_EQ(g.state, ST_FAULT);
}

static void test_off_while_settling(void) {
  start();
  gate_set_target(&g, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER);
  tick(1, 0);
  gate_set_target(&g, 0, LIMIT_BRIGHTER, LIMIT_DIMMER);
  tick(1, 0);
  CHECK_EQ(g.state, ST_OFF);
  CHECK_EQ(g.relay, 0);
}

static void test_update_request(void) {
  on(4000);
  gate_request_update(&g);
  CHECK_EQ(g.target, 0);
  CHECK_EQ(g.limit_brighter, 16);
  tick(1, 0);
  CHECK_EQ(g.state, ST_STOPPING);
  CHECK_EQ(gate_update_ready(&g), 0); /* not while the relay is on */
  edges += 2;
  ticks(RELAY_OFF_DELAY_MS + 1, 1, 0);
  CHECK_EQ(g.state, ST_OFF);
  CHECK_EQ(gate_update_ready(&g), 1);
  gate_set_target(&g, 0, LIMIT_BRIGHTER, LIMIT_DIMMER); /* a repeated 0 keeps the request */
  CHECK_EQ(gate_update_ready(&g), 1);
}

/* A non-zero target after the request means the ESP32 gave it up. */
static void test_update_request_cancelled(void) {
  on(4000);
  gate_request_update(&g);
  tick(1, 0);
  gate_set_target(&g, 3000, LIMIT_BRIGHTER, LIMIT_DIMMER);
  edges += 2;
  ticks(RELAY_OFF_DELAY_MS + 1, 1, 0);
  CHECK_EQ(gate_update_ready(&g), 0);
  gate_set_target(&g, 0, LIMIT_BRIGHTER, LIMIT_DIMMER);
  ticks(30, 1, 0);
  CHECK_EQ(g.state, ST_OFF);
  CHECK_EQ(gate_update_ready(&g), 0);
}

/* In FAULT the bootloader waits for the relay to open. */
static void test_update_request_in_fault(void) {
  on(4000);
  gate_request_update(&g);
  tick(1, F_OVERTEMP);
  CHECK_EQ(g.state, ST_FAULT);
  CHECK_EQ(gate_update_ready(&g), 0);
  ticks(FAULT_RELAY_MS, 1, F_OVERTEMP);
  CHECK_EQ(g.relay, 0);
  CHECK_EQ(gate_update_ready(&g), 1);
  start();
  CHECK_EQ(gate_update_ready(&g), 0);
}

/* The start-up FAULT (thermistor average not ready) ends when the average is ready, also when a
   non-zero target arrived meanwhile, e.g. a resend in flight across the restart. */
static void test_startup_fault_ends_when_ready(void) {
  start();
  gate_set_target(&g, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER); /* before the first tick */
  ticks(30, 0, F_THERM);
  CHECK_EQ(g.state, ST_FAULT);
  gate_set_target(&g, 3000, LIMIT_BRIGHTER, LIMIT_DIMMER); /* during start-up */
  ticks(50, 1, F_THERM);
  CHECK_EQ(g.state, ST_FAULT);
  CHECK_EQ(g.relay, 0);
  CHECK_EQ(g.fire, 0);
  gate_startup_done(&g, 0);
  tick(1, 0);
  CHECK_EQ(g.state, ST_OFF);
  tick(1, 0);
  CHECK_EQ(g.state, ST_SETTLE); /* the target received is used, through the normal sequence */
  gate_startup_done(&g, 0);
  CHECK_EQ(g.state, ST_SETTLE);
}

/* Real faults keep their rule: a zero target is needed after a non-zero one. */
static void test_startup_fault_when_ready_needs_zero(void) {
  start();
  gate_set_target(&g, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER);
  ticks(30, 1, F_THERM);
  gate_startup_done(&g, F_OVERTEMP);
  ticks(20, 1, F_OVERTEMP);
  CHECK_EQ(g.state, ST_FAULT);
  ticks(20, 1, 0); /* the cause gone, but no zero target since */
  CHECK_EQ(g.state, ST_FAULT);
  gate_set_target(&g, 0, LIMIT_BRIGHTER, LIMIT_DIMMER);
  tick(1, 0);
  CHECK_EQ(g.state, ST_OFF);
}

static void test_startup_gate_fault_gone_when_ready_needs_zero(void) {
  start();
  gate_set_target(&g, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER);
  ticks(10, 1, F_THERM);
  tick(1, F_THERM | F_GATE);
  ticks(10, 1, F_THERM);
  gate_startup_done(&g, 0);
  ticks(20, 1, 0);
  CHECK_EQ(g.state, ST_FAULT);
  gate_set_target(&g, 0, LIMIT_BRIGHTER, LIMIT_DIMMER);
  tick(1, 0);
  CHECK_EQ(g.state, ST_OFF);
}

static void test_therm_fault_after_startup_is_real(void) {
  start();
  ticks(10, 1, F_THERM);
  gate_startup_done(&g, 0);
  ticks(10, 1, 0);
  CHECK_EQ(g.state, ST_OFF);
  gate_set_target(&g, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER);
  ticks(20, 1, F_THERM);
  CHECK_EQ(g.state, ST_FAULT);
  gate_set_target(&g, 4000, LIMIT_BRIGHTER, LIMIT_DIMMER);
  ticks(20, 1, 0);
  CHECK_EQ(g.state, ST_FAULT);
}

int main(void) {
  RUN(test_off_until_target);
  RUN(test_turn_on_sequence);
  RUN(test_needs_zo_to_start);
  RUN(test_smoothing);
  RUN(test_clamped_target);
  RUN(test_turn_off_sequence);
  RUN(test_turn_off_edge_before_fire_cleared);
  RUN(test_fault_from_firing);
  RUN(test_fault_relay_time_limit);
  RUN(test_fault_never_fires);
  RUN(test_fault_needs_cause_gone);
  RUN(test_zo_lost_while_on);
  RUN(test_zo_lost_while_settling_and_stopping);
  RUN(test_fault_while_off_blocks_start);
  RUN(test_off_while_settling);
  RUN(test_update_request);
  RUN(test_update_request_cancelled);
  RUN(test_update_request_in_fault);
  RUN(test_startup_fault_ends_when_ready);
  RUN(test_startup_fault_when_ready_needs_zero);
  RUN(test_startup_gate_fault_gone_when_ready_needs_zero);
  RUN(test_therm_fault_after_startup_is_real);
  TEST_EXIT();
}
