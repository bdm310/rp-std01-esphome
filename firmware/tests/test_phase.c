#include "board.h"
#include "phase.h"
#include "sched.h"
#include "test.h"

/* 60 Hz Zo as measured: high 8.29 ms, low 8.38 ms. */
#define HIGH_TICKS 7640u
#define LOW_TICKS 7720u
#define PERIOD (HIGH_TICKS + LOW_TICKS)

static uint16_t t;

/* Feeds n half-cycles starting with a rising edge at t. Returns the last result. */
static uint8_t feed(phase_t *p, int n, uint16_t *ref) {
  uint8_t r = 0;
  int i;
  for (i = 0; i < n; i++) {
    uint8_t rising = (i & 1) == 0;
    r = phase_edge(p, t, rising, ref);
    t += rising ? HIGH_TICKS : LOW_TICKS;
  }
  return r;
}

static void test_locks_and_measures_period(void) {
  phase_t p;
  uint16_t ref = 0;
  phase_init(&p);
  t = 1000;
  feed(&p, ZO_GOOD_EDGES, &ref);
  CHECK(!phase_valid(&p));
  feed(&p, 2, &ref);
  CHECK(phase_valid(&p));
  CHECK_EQ(p.period, PERIOD);
}

static void test_references(void) {
  phase_t p;
  uint16_t ref = 0, rise;
  phase_init(&p);
  t = 0xFF00; /* crosses the counter wrap */
  feed(&p, 40, &ref);
  rise = t;
  CHECK_EQ(phase_edge(&p, t, 1, &ref), PHASE_REF);
  CHECK_EQ(ref, rise);
  t += HIGH_TICKS;
  CHECK_EQ(phase_edge(&p, t, 0, &ref), PHASE_REF);
  /* The falling half is timed from the rising edge plus half a period, as stock, not from the
     falling edge itself. */
  CHECK_EQ((uint16_t) (ref - rise), PERIOD / 2);
  CHECK_EQ((uint16_t) (ref - t), PERIOD / 2 - HIGH_TICKS);
}

static void test_noise_ignored(void) {
  phase_t p;
  uint16_t ref = 0;
  phase_init(&p);
  t = 0;
  feed(&p, 30, &ref);
  CHECK(phase_valid(&p));
  /* A glitch soon after the last edge changes nothing. */
  CHECK_EQ(phase_edge(&p, t - LOW_TICKS + PCA_US(3000), 1, &ref), PHASE_IGNORED);
  CHECK(phase_valid(&p));
  feed(&p, 2, &ref);
  CHECK(phase_valid(&p));
}

static void test_missed_edge_and_gap(void) {
  phase_t p;
  uint16_t ref = 0;
  phase_init(&p);
  t = 0;
  feed(&p, 30, &ref);
  /* The rising edge at t is missed. The falling edge after it has the same level as the last
     accepted edge, so it counts as noise; the rising edge after that shows the gap. */
  t += HIGH_TICKS;
  CHECK_EQ(phase_edge(&p, t, 0, &ref), PHASE_IGNORED);
  t += LOW_TICKS;
  CHECK_EQ(phase_edge(&p, t, 1, &ref), PHASE_REF);
  CHECK(!phase_valid(&p));
  CHECK_EQ(p.good, 0);

  phase_init(&p);
  t = 0;
  feed(&p, 30, &ref);
  t += 30000; /* Zo stopped for a while */
  feed(&p, 1, &ref);
  CHECK(!phase_valid(&p));
}

static void test_late_glitch_when_locked(void) {
  phase_t p;
  uint16_t ref = 0;
  phase_init(&p);
  t = 0;
  feed(&p, 30, &ref);
  /* Last accepted edge was falling at t - LOW_TICKS. A glitch shortly before the next rising
     edge is too early once locked. */
  CHECK_EQ(phase_edge(&p, t - PCA_US(1000), 1, &ref), PHASE_IGNORED);
  CHECK(phase_valid(&p));
  CHECK_EQ(phase_edge(&p, t, 1, &ref), PHASE_REF);
  CHECK(phase_valid(&p));
}

static void test_period_tracks_slowly(void) {
  phase_t p;
  uint16_t ref = 0;
  int i;
  phase_init(&p);
  t = 0;
  feed(&p, 40, &ref);
  /* Mains 0.1% slow: the filter must reach it, not stop within a few counts. */
  for (i = 0; i < 400; i++) {
    phase_edge(&p, t, (i & 1) == 0, &ref);
    t += (i & 1) ? LOW_TICKS + 8 : HIGH_TICKS + 8;
  }
  CHECK(p.period >= PERIOD + 15 && p.period <= PERIOD + 17);
}

static void test_50hz(void) {
  /* 50 Hz, with the same 40-count skew between the halves as the 60 Hz figures. */
  phase_t p;
  uint16_t ref = 0, rise;
  int i;
  phase_init(&p);
  t = 0;
  for (i = 0; i < 40; i++) {
    phase_edge(&p, t, (i & 1) == 0, &ref);
    t += (i & 1) ? 9256 : 9176;
  }
  CHECK(phase_valid(&p));
  CHECK_EQ(p.period, 18432);
  rise = t;
  CHECK_EQ(phase_edge(&p, t, 1, &ref), PHASE_REF);
  CHECK_EQ(ref, rise);
  t += 9176;
  CHECK_EQ(phase_edge(&p, t, 0, &ref), PHASE_REF);
  CHECK_EQ((uint16_t) (ref - rise), 9216);
  CHECK_EQ(sched_clamp(65000, 9216), 9216 - LATEST_BEFORE_EDGE);
}

static void test_out_of_range_period(void) {
  phase_t p;
  uint16_t ref = 0;
  int i;
  phase_init(&p);
  t = 0;
  /* 40 Hz: edges 12.5 ms apart are rejected. */
  for (i = 0; i < 40; i++) {
    phase_edge(&p, t, (i & 1) == 0, &ref);
    t += 11520;
  }
  CHECK(!phase_valid(&p));
}

static void test_clamp(void) {
  uint16_t half = PERIOD / 2;
  CHECK_EQ(sched_clamp(0, half), 0);
  CHECK_EQ(sched_clamp(10, half), EARLIEST_FIRE);
  CHECK_EQ(sched_clamp(4000, half), 4000);
  CHECK_EQ(sched_clamp(65000, half), half - LATEST_BEFORE_EDGE);
  CHECK_EQ(sched_clamp(4000, 0), 0);
}

static void lock(sched_t *s) {
  int i;
  uint16_t x;
  sched_init(s);
  t = 0;
  for (i = 0; i < 40; i++) {
    uint8_t rising = (i & 1) == 0;
    sched_edge(s, t, rising, 0, &x);
    t += rising ? HIGH_TICKS : LOW_TICKS;
  }
}

static void test_gap_longer_than_counter_wrap(void) {
  sched_t s;
  uint16_t x;
  uint32_t gap = 2ul * 65536u + 9000u;
  uint32_t ms;
  uint16_t last_edge;
  unsigned i;
  lock(&s);
  CHECK(phase_valid(&s.phase));
  last_edge = t - LOW_TICKS;
  /* The 16-bit timestamp wraps twice during the gap, so the edge after it looks only a little
     late. */
  for (ms = 0; ms < gap * 1000u / PCA_HZ; ms++)
    if (sched_zo_lost(&s))
      sched_zo_reset(&s);
  t = (uint16_t) (last_edge + gap);
  for (i = 0; i < ZO_GOOD_EDGES; i++) {
    uint8_t rising = (i & 1) == 0;
    CHECK_EQ(sched_edge(&s, t, rising, 4000, &x), SA_OFF);
    CHECK(!phase_valid(&s.phase));
    t += rising ? HIGH_TICKS : LOW_TICKS;
  }
  CHECK_EQ(sched_edge(&s, t, 1, 4000, &x), SA_ARM);
}

static void test_zo_lost(void) {
  sched_t s;
  uint16_t x;
  unsigned ms;
  lock(&s);
  CHECK(s.zo_valid);
  for (ms = 1; ms <= ZO_LOST_MS; ms++)
    CHECK(!sched_zo_lost(&s));
  CHECK(sched_zo_lost(&s));
  sched_zo_reset(&s);
  CHECK(!s.zo_valid);
  CHECK(!phase_valid(&s.phase));
  CHECK(!sched_zo_lost(&s)); /* once */
  /* An edge starts the count again. */
  sched_edge(&s, t, 1, 0, &x);
  for (ms = 1; ms <= ZO_LOST_MS; ms++)
    CHECK(!sched_zo_lost(&s));
  CHECK(sched_zo_lost(&s));
}

static void test_zo_valid_follows_edges(void) {
  sched_t s;
  uint16_t x;
  unsigned i;
  sched_init(&s);
  t = 0;
  for (i = 0; i < 40; i++) {
    uint8_t rising = (i & 1) == 0;
    sched_edge(&s, t, rising, 0, &x);
    CHECK_EQ(s.zo_valid, phase_valid(&s.phase));
    t += rising ? HIGH_TICKS : LOW_TICKS;
  }
  CHECK(s.zo_valid);
  t += 2 * PERIOD; /* a gap */
  sched_edge(&s, t, 1, 0, &x);
  CHECK(!s.zo_valid);
}

static void test_schedule(void) {
  sched_t s;
  uint16_t fire = 0, rel = 0, edge;
  lock(&s);
  edge = t;
  CHECK_EQ(sched_edge(&s, edge, 1, 4000, &fire), SA_ARM);
  CHECK_EQ((uint16_t) (fire - edge), 4000);
  CHECK_EQ(s.gs, GS_WAIT_FIRE);
  CHECK_EQ(sched_match(&s, 1, &rel), SA_RELEASE);
  CHECK_EQ((uint16_t) (rel - edge), PERIOD / 2 - RELEASE_BEFORE_EDGE);
  CHECK_EQ(sched_match(&s, 0, &rel), SA_OFF);
  CHECK_EQ(s.gs, GS_IDLE);
}

static void test_schedule_no_delay_or_not_locked(void) {
  sched_t s;
  uint16_t x;
  lock(&s);
  CHECK_EQ(sched_edge(&s, t, 1, 0, &x), SA_OFF);
  sched_init(&s);
  CHECK_EQ(sched_edge(&s, 0, 1, 4000, &x), SA_OFF);
}

static void test_schedule_clamps_late_delay(void) {
  sched_t s;
  uint16_t fire = 0, edge;
  lock(&s);
  edge = t;
  CHECK_EQ(sched_edge(&s, edge, 1, 9000, &fire), SA_ARM);
  CHECK_EQ((uint16_t) (fire - edge), PERIOD / 2 - LATEST_BEFORE_EDGE);
}

static void test_match_without_pin_is_fault(void) {
  sched_t s;
  uint16_t x;
  lock(&s);
  sched_edge(&s, t, 1, 4000, &x);
  CHECK_EQ(sched_match(&s, 0, &x), SA_OFF);
  CHECK_EQ(s.fault, 1);
}

static void test_tick_check(void) {
  sched_t s;
  uint16_t x, edge;
  lock(&s);
  CHECK_EQ(sched_tick_check(&s, 0, t), SA_NONE);
  CHECK_EQ(sched_tick_check(&s, 1, t), SA_OFF); /* high while nothing is scheduled */
  CHECK_EQ(s.fault, 1);
  edge = t;
  sched_edge(&s, edge, 1, 4000, &x);
  CHECK_EQ(sched_tick_check(&s, 1, edge + 4001), SA_NONE); /* fired, release not yet handled */
  sched_match(&s, 1, &x);
  CHECK_EQ(sched_tick_check(&s, 1, edge + 5000), SA_NONE);
  CHECK_EQ(sched_tick_check(&s, 1, x + RELEASE_OVERDUE + 1), SA_OFF);
  CHECK_EQ(s.fault, 2);
}

int main(void) {
  RUN(test_locks_and_measures_period);
  RUN(test_references);
  RUN(test_noise_ignored);
  RUN(test_missed_edge_and_gap);
  RUN(test_late_glitch_when_locked);
  RUN(test_period_tracks_slowly);
  RUN(test_50hz);
  RUN(test_out_of_range_period);
  RUN(test_clamp);
  RUN(test_gap_longer_than_counter_wrap);
  RUN(test_zo_lost);
  RUN(test_zo_valid_follows_edges);
  RUN(test_schedule);
  RUN(test_schedule_no_delay_or_not_locked);
  RUN(test_schedule_clamps_late_delay);
  RUN(test_match_without_pin_is_fault);
  RUN(test_tick_check);
  TEST_EXIT();
}
