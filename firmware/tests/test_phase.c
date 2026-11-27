#include <string.h>

#include "board.h"
#include "phase.h"
#include "sched.h"
#include "test.h"

/* 60 Hz Zo as measured: high 8.29 ms, low 8.38 ms. A period starts with a falling edge. */
#define HIGH_TICKS 7640u
#define LOW_TICKS 7720u
#define PERIOD (HIGH_TICKS + LOW_TICKS)

#define NO_EDGE ((int32_t) 0x7FFFFFFF)
#define SECONDS_PCA(x) ((uint32_t) ((x) * PCA_HZ))

/*
 * A driver for sched_edge: Zo edges and the reference timer's matches in time order. Period n has
 * its nominal falling edge at base + n * period; fall(n) and rise(n) give each edge's lateness, or
 * NO_EDGE. Every half-cycle start is recorded with whether it was a falling one.
 */
static sched_t s;
static uint32_t base, period;
static unsigned n_next;
static uint32_t clock32;
static int32_t (*fall)(unsigned n);
static int32_t (*rise)(unsigned n);
static uint16_t delay;

#define MAX_STARTS 8192
static uint32_t start_at[MAX_STARTS];
static uint8_t start_falling[MAX_STARTS];
static unsigned starts, lost;
static uint32_t lost_at;

static int32_t on_time(unsigned n) {
  (void) n;
  return 0;
}

static void record(uint32_t at, uint8_t falling) {
  CHECK(starts < MAX_STARTS);
  if (starts < MAX_STARTS) {
    start_at[starts] = at;
    start_falling[starts] = falling;
  }
  starts++;
}

static void timer_until(uint32_t until) {
  uint16_t x;
  while (phase_locked(&s.phase)) {
    uint32_t at = clock32 + (uint32_t) (int32_t) (int16_t) (s.phase.next_at - (uint16_t) clock32);
    uint8_t falling = ((s.phase.flags & PF_NEXT_FALLING) != 0);
    if (at > until)
      return;
    clock32 = at;
    if (sched_edge(&s, (uint16_t) at, PHASE_TIMER, delay, &x) == SA_OFF && !phase_locked(&s.phase)) {
      lost++;
      lost_at = at;
    }
    else
      record(at, falling);
  }
}

static void edge(uint32_t at, uint8_t level) {
  uint16_t x;
  uint8_t was_locked = phase_locked(&s.phase);
  timer_until(at);
  clock32 = at;
  if (sched_edge(&s, (uint16_t) at, level, delay, &x) != SA_NONE) {
    if (!was_locked) {
      record(at, 1);
    } else if (!phase_locked(&s.phase)) {
      lost++;
      lost_at = at;
    }
  }
}

/* Feeds n periods of Zo. */
static void run(unsigned n) {
  unsigned end = n_next + n;
  for (; n_next < end; n_next++) {
    uint32_t nominal = base + n_next * period;
    int32_t f = fall(n_next), r = rise(n_next);
    if (f != NO_EDGE)
      edge(nominal + f, 0);
    if (r != NO_EDGE)
      edge(nominal + (period - HIGH_TICKS) + r, 1);
  }
  /* Up to a quarter period before the next falling edge, so that an early one comes after. */
  timer_until(base + n_next * period - period / 4);
}

static void start(uint32_t per, uint32_t first_fall) {
  sched_init(&s);
  base = first_fall;
  period = per;
  n_next = 0;
  clock32 = first_fall - 1;
  fall = rise = on_time;
  delay = 0;
  starts = lost = 0;
}

/* The nominal falling edge nearest to t. */
static int32_t fall_error(uint32_t t) {
  uint32_t k = (t - base + period / 2) / period;
  return (int32_t) (t - (base + k * period));
}

static void test_locks_and_measures_period(void) {
  start(PERIOD, 0xFF00); /* crosses the counter wrap */
  run(ZO_GOOD_PERIODS);
  CHECK(!phase_locked(&s.phase));
  CHECK_EQ(starts, 0);
  run(1);
  CHECK(phase_locked(&s.phase));
  CHECK(s.zo_valid);
  /* The edge that locked counts for its half-cycle. */
  CHECK_EQ(s.phase.missed, 0);
  CHECK_EQ(s.phase.period, PERIOD);
  /* The falling edge that locks starts the first half-cycle. */
  CHECK_EQ(start_falling[0], 1);
  CHECK_EQ(start_at[0], base + ZO_GOOD_PERIODS * period);
}

/* Once locked, falling half-cycles start at the falling edge and rising ones half a period later,
   from the timer. */
static void test_half_cycles_from_timer(void) {
  unsigned i;
  start(PERIOD, 1000);
  run(ZO_GOOD_PERIODS + 1);
  starts = 0;
  run(20);
  CHECK_EQ(starts, 40);
  for (i = 0; i < starts; i++) {
    if (start_falling[i])
      CHECK_EQ(fall_error(start_at[i]), 0);
    if (i == 0)
      continue;
    CHECK(start_falling[i] != start_falling[i - 1]);
    if (!start_falling[i])
      CHECK_EQ(start_at[i] - start_at[i - 1], PERIOD / 2);
  }
  CHECK_EQ(s.phase.missed, 0);
}

/* As after a turn-off with LED lamps: rising edges more than 2 ms late, then anywhere. */
static int32_t rise_drifting(unsigned n) { return (int32_t) ((n * 97u) % 3000u); }

static void test_rising_edges_not_used(void) {
  unsigned i;
  start(PERIOD, 1000);
  run(ZO_GOOD_PERIODS + 1);
  rise = rise_drifting;
  starts = 0;
  run(100);
  for (i = 0; i < starts; i++)
    if (start_falling[i])
      CHECK_EQ(fall_error(start_at[i]), 0);
  CHECK(phase_locked(&s.phase));
}

/* As after the relay closes with LED lamps: falling edges 1 ms late for half a second. */
static int32_t fall_disturbed(unsigned n) { return n >= 20 && n < 50 ? (int32_t) PCA_US(1000) : 0; }

static void test_falling_disturbance_ignored(void) {
  unsigned i;
  start(PERIOD, 1000);
  fall = fall_disturbed;
  run(ZO_GOOD_PERIODS + 1);
  starts = 0;
  run(200);
  /* Present, so not missed, but outside the phase window: the timing does not move. */
  for (i = 0; i < starts; i++)
    if (start_falling[i])
      CHECK_EQ(fall_error(start_at[i]), 0);
  CHECK_EQ(s.phase.missed, 0);
  CHECK_EQ(s.phase.period, PERIOD);
}

/* Falling edges 150 us late from period 20 on, inside the phase window: the timing follows them, by
   a small step each period. */
static int32_t offset;
static int32_t fall_offset(unsigned n) { return n >= 20 ? offset : 0; }

static void small_offset_followed(int32_t by) {
  unsigned i;
  int32_t prev = 0, e, step = (int32_t) (PLL_PHASE_WINDOW >> PLL_PHASE_SHIFT) + 1;
  start(PERIOD, 1000);
  offset = by;
  fall = fall_offset;
  run(ZO_GOOD_PERIODS + 1);
  starts = 0;
  run(600);
  for (i = 0; i < starts; i++) {
    if (!start_falling[i])
      continue;
    e = fall_error(start_at[i]);
    /* Toward the edges, a small step at a time. */
    CHECK(by > 0 ? e - prev >= -1 && e - prev <= step : prev - e >= -1 && prev - e <= step);
    prev = e;
  }
  CHECK(prev - by >= -3 && prev - by <= 3);
  CHECK_EQ(s.phase.missed, 0);
}

static void test_small_offset_followed(void) {
  small_offset_followed((int32_t) PCA_US(150));
  small_offset_followed(-(int32_t) PCA_US(150));
}

/* Falling edges a little early and late in turn, around their predicted times: none is missed. */
static int32_t fall_jitter(unsigned n) { return n & 1 ? 10 : -10; }

static void test_jitter_not_missed(void) {
  unsigned i;
  start(PERIOD, 1000);
  fall = fall_jitter;
  run(ZO_GOOD_PERIODS + 1);
  for (i = 0; i < 200; i++) {
    run(1);
    CHECK_EQ(s.phase.missed, 0);
  }
}

/* Falling edges further from their predicted time than ZO_CAPTURE are not used; fewer than
   ZO_OUTSIDE_CAPTURE_MAX in a row ride through, ZO_OUTSIDE_CAPTURE_MAX in a row (Zo back at another phase) drop the lock. */
static unsigned far_len;
static int32_t fall_far(unsigned n) { return n >= 20 && n < 20 + far_len ? (int32_t) ZO_CAPTURE + 50 : 0; }

static void test_outside_capture_ignored(void) {
  unsigned i;
  start(PERIOD, 1000);
  fall = fall_far;
  far_len = ZO_OUTSIDE_CAPTURE_MAX - 1;
  run(ZO_GOOD_PERIODS + 1);
  starts = 0;
  run(12);
  CHECK(s.phase.missed > 0);
  run(30);
  for (i = 0; i < starts; i++)
    if (start_falling[i])
      CHECK_EQ(fall_error(start_at[i]), 0);
  CHECK_EQ(s.phase.missed, 0);
  CHECK_EQ(lost, 0);

  start(PERIOD, 1000);
  fall = fall_far;
  far_len = ZO_OUTSIDE_CAPTURE_MAX;
  run(ZO_GOOD_PERIODS + 1);
  run(20);
  CHECK_EQ(lost, 1);
}

/* No falling edges for fewer than ZO_MISSED_MAX periods: the half-cycles go on, on time.
   ZO_MISSED_MAX and Zo is lost. */
static unsigned gap_from, gap_len;
static int32_t fall_gap(unsigned n) { return n >= gap_from && n < gap_from + gap_len ? NO_EDGE : 0; }

static void test_gap_rides_through(void) {
  unsigned i;
  start(PERIOD, 1000);
  fall = rise = fall_gap;
  gap_from = 20;
  gap_len = ZO_MISSED_MAX - 1;
  run(ZO_GOOD_PERIODS + 1);
  starts = 0;
  run(40);
  CHECK(phase_locked(&s.phase));
  CHECK_EQ(lost, 0);
  CHECK_EQ(starts, 80);
  for (i = 0; i < starts; i++)
    if (start_falling[i])
      CHECK_EQ(fall_error(start_at[i]), 0);

  start(PERIOD, 1000);
  fall = rise = fall_gap;
  gap_len = ZO_MISSED_MAX;
  run(ZO_GOOD_PERIODS + 1);
  run(40);
  CHECK_EQ(lost, 1);
}

/* Mains 0.1 % slow: the period follows, and the half-cycles keep to the falling edges. */
static void test_tracks_frequency(void) {
  unsigned i;
  start(PERIOD, 1000);
  run(ZO_GOOD_PERIODS + 1);
  base += n_next * period;
  n_next = 0;
  period = PERIOD + 15;
  starts = 0;
  run(600);
  CHECK(s.phase.period >= PERIOD + 14 && s.phase.period <= PERIOD + 16);
  for (i = starts - 100; i < starts; i++)
    if (start_falling[i])
      CHECK(fall_error(start_at[i]) >= -20 && fall_error(start_at[i]) <= 20);
}


/* Locking only on steady edges. One edge off by more than PLL_PERIOD_WINDOW during the run, at its
   start, at its end or inside it, delays the lock instead of skewing it: once locked, the period is
   right and the timing keeps to the edges. */
static unsigned odd_n;
static int32_t odd_by;
static int32_t fall_odd(unsigned n) { return n == odd_n ? odd_by : 0; }

static void lock_with_odd_edge(unsigned n, int32_t by) {
  unsigned i;
  start(PERIOD, 1000);
  odd_n = n;
  odd_by = by;
  fall = fall_odd;
  run(40);
  CHECK(phase_locked(&s.phase));
  CHECK(s.phase.period >= PERIOD - 1 && s.phase.period <= PERIOD + 1);
  starts = 0;
  run(3000);
  /* An edge within PLL_PERIOD_WINDOW of steady still locks, and its offset fades. */
  for (i = 0; i < starts; i++)
    if (start_falling[i])
      CHECK(fall_error(start_at[i]) >= -(int32_t) PLL_PERIOD_WINDOW && fall_error(start_at[i]) <= (int32_t) PLL_PERIOD_WINDOW);
  for (i = starts - 20; i < starts; i++)
    if (start_falling[i])
      CHECK(fall_error(start_at[i]) >= -2 && fall_error(start_at[i]) <= 2);
  CHECK_EQ(lost, 0);
}

static void test_lock_needs_steady_edges(void) {
  lock_with_odd_edge(0, -(int32_t) PCA_US(100));
  lock_with_odd_edge(0, (int32_t) PCA_US(150));
  lock_with_odd_edge(0, -(int32_t) PCA_US(150));
  lock_with_odd_edge(ZO_GOOD_PERIODS, -(int32_t) PCA_US(300));
  lock_with_odd_edge(2, -(int32_t) PCA_US(300));
  lock_with_odd_edge(ZO_GOOD_PERIODS, (int32_t) PCA_US(40));
}

/* Falling edges drift later by 15 us a period to 0.6 ms, then snap back. The timing follows the
   drift, as it would a lower frequency; after the snap the steady edges are inside the capture
   window but outside the phase window, and the lock lets go within ZO_OUTSIDE_PHASE_MAX periods, rather than
   firing on, 0.6 ms late. */
static int32_t fall_ramp(unsigned n) {
  return n >= 20 && n < 85 ? (int32_t) ((n - 20) * PCA_US(600) / 65) : 0;
}

static void test_drift_then_snap_lets_go(void) {
  start(PERIOD, 1000);
  fall = fall_ramp;
  run(85);
  CHECK(phase_locked(&s.phase));
  CHECK_EQ(lost, 0);
  run(60);
  CHECK_EQ(lost, 1);
  /* Not before the longest disturbance measured on unit 1 (0.5 s), and within 0.8 s. */
  CHECK(lost_at >= base + 85 * period + SECONDS_PCA(0.5));
  CHECK(lost_at <= base + 85 * period + SECONDS_PCA(0.8));
}

/* Zo stops for a few periods, and comes back a quarter period later (another phase, as after a
   transfer switch). The lock lets go within ZO_OUTSIDE_CAPTURE_MAX periods of its return. */
static int32_t fall_jump(unsigned n) {
  if (n >= 20 && n < 25)
    return NO_EDGE;
  return n >= 25 ? (int32_t) (PERIOD / 4) : 0;
}

static void test_phase_jump_lets_go(void) {
  start(PERIOD, 1000);
  fall = rise = fall_jump;
  run(25);
  CHECK(phase_locked(&s.phase));
  run(ZO_OUTSIDE_CAPTURE_MAX + 2);
  CHECK_EQ(lost, 1);
  CHECK(lost_at <= base + (25 + ZO_OUTSIDE_CAPTURE_MAX + 1) * period);
}

/* Mains 0.5 % slower at once, more than PLL_PERIOD_WINDOW: the period becomes the new one within
   ZO_FREQ_AGREE + 1 periods, the timing jumps to the edges, and the lock holds. */
static void test_frequency_step(void) {
  unsigned i;
  int32_t e, worst = 0;
  start(PERIOD, 1000);
  run(ZO_GOOD_PERIODS + 1);
  run(10);
  base += n_next * period;
  n_next = 0;
  period = PERIOD + PERIOD / 200;
  starts = 0;
  run(ZO_FREQ_AGREE + 2);
  CHECK(s.phase.period >= period - 1 && s.phase.period <= period + 1);
  for (i = 0; i < starts; i++) {
    if (!start_falling[i])
      continue;
    e = fall_error(start_at[i]);
    if (e < 0)
      e = -e;
    if (e > worst)
      worst = e;
  }
  /* Until then, the timing fell behind by at most what ZO_FREQ_AGREE + 1 periods add up to. */
  CHECK(worst <= (int32_t) ((ZO_FREQ_AGREE + 1) * (PERIOD / 200)));
  starts = 0;
  run(200);
  for (i = 0; i < starts; i++)
    if (start_falling[i])
      CHECK(fall_error(start_at[i]) >= -3 && fall_error(start_at[i]) <= 3);
  CHECK(phase_locked(&s.phase));
  CHECK_EQ(lost, 0);
}

static void test_50hz(void) {
  start(18432, 1000);
  run(ZO_GOOD_PERIODS + 1);
  CHECK(phase_locked(&s.phase));
  CHECK_EQ(s.phase.period, 18432);
  CHECK_EQ(sched_clamp(65000, 9216), 9216 - LATEST_BEFORE_EDGE);
}

static void test_out_of_range_period(void) {
  start(PCA_HZ / 40u, 1000); /* 40 Hz */
  run(40);
  CHECK(!phase_locked(&s.phase));
}

/* Before lock, a glitch's falling edge breaks the run of consistent periods. */
static int32_t fall_glitch(unsigned n) { return n == 5 ? (int32_t) PCA_US(3000) : 0; }

static void test_glitch_before_lock(void) {
  start(PERIOD, 1000);
  fall = fall_glitch;
  run(ZO_GOOD_PERIODS + 1);
  CHECK(!phase_locked(&s.phase));
  run(ZO_GOOD_PERIODS);
  CHECK(phase_locked(&s.phase));
}

static void test_clamp(void) {
  uint16_t half = PERIOD / 2;
  CHECK_EQ(sched_clamp(0, half), 0);
  CHECK_EQ(sched_clamp(10, half), EARLIEST_FIRE);
  CHECK_EQ(sched_clamp(4000, half), 4000);
  CHECK_EQ(sched_clamp(65000, half), half - LATEST_BEFORE_EDGE);
  CHECK_EQ(sched_clamp(4000, 0), 0);
}

static void locked(void) {
  start(PERIOD, 1000);
  run(ZO_GOOD_PERIODS + 1);
}

/* The timer's next match, as the interrupt handler would see it. */
static uint16_t next_start(uint8_t *falling) {
  *falling = ((s.phase.flags & PF_NEXT_FALLING) != 0);
  return s.phase.next_at;
}

static void test_gap_longer_than_counter_wrap(void) {
  uint16_t x;
  uint32_t ms;
  unsigned i;
  uint16_t t;
  locked();
  /* The timer stops (Zo lost), and then nothing for longer than the 16-bit counter wraps twice. */
  sched_zo_reset(&s);
  for (ms = 0; ms < (2ul * 65536u + 9000u) * 1000u / PCA_HZ; ms++)
    if (sched_zo_lost(&s))
      sched_zo_reset(&s);
  t = 1234;
  for (i = 0; i < ZO_GOOD_PERIODS; i++) {
    CHECK_EQ(sched_edge(&s, t, 0, 4000, &x), SA_NONE);
    CHECK(!phase_locked(&s.phase));
    t += PERIOD;
  }
  CHECK_EQ(sched_edge(&s, t, 0, 4000, &x), SA_ARM);
}

static void test_zo_lost(void) {
  uint16_t x;
  unsigned ms;
  locked();
  CHECK(s.zo_valid);
  sched_zo_reset(&s);
  CHECK(!s.zo_valid);
  CHECK(!phase_locked(&s.phase));
  for (ms = 1; ms <= ZO_LOST_MS; ms++)
    CHECK(!sched_zo_lost(&s));
  CHECK(sched_zo_lost(&s));
  CHECK(!sched_zo_lost(&s)); /* once */
  /* Before lock, an edge starts the count again. */
  sched_edge(&s, 0, 1, 0, &x);
  for (ms = 1; ms <= ZO_LOST_MS; ms++)
    CHECK(!sched_zo_lost(&s));
  CHECK(sched_zo_lost(&s));
}

static void test_schedule(void) {
  uint16_t fire = 0, rel = 0, at, next;
  uint8_t falling;
  locked();
  at = next_start(&falling);
  CHECK_EQ(sched_edge(&s, at, PHASE_TIMER, 4000, &fire), SA_ARM);
  next = s.phase.next_at;
  CHECK_EQ((uint16_t) (fire - at), 4000);
  CHECK_EQ(s.gs, GS_WAIT_FIRE);
  CHECK_EQ(sched_match(&s, 1, &rel), SA_RELEASE);
  /* Released before the next half-cycle starts. */
  CHECK_EQ((uint16_t) (next - rel), RELEASE_BEFORE_EDGE);
  CHECK_EQ(sched_match(&s, 0, &rel), SA_OFF);
  CHECK_EQ(s.gs, GS_IDLE);
}

static void test_schedule_no_delay_or_not_locked(void) {
  uint16_t x, at;
  uint8_t falling;
  locked();
  at = next_start(&falling);
  CHECK_EQ(sched_edge(&s, at, PHASE_TIMER, 0, &x), SA_OFF);
  sched_init(&s);
  CHECK_EQ(sched_edge(&s, 0, 0, 4000, &x), SA_NONE);
  CHECK_EQ(sched_edge(&s, 0, PHASE_TIMER, 4000, &x), SA_NONE);
}

static void test_schedule_clamps_late_delay(void) {
  uint16_t fire = 0, at;
  uint8_t falling;
  locked();
  at = next_start(&falling);
  CHECK_EQ(sched_edge(&s, at, PHASE_TIMER, 9000, &fire), SA_ARM);
  CHECK_EQ((uint16_t) (fire - at), PERIOD / 2 - LATEST_BEFORE_EDGE);
}

static void test_match_without_pin_is_fault(void) {
  uint16_t x, at;
  uint8_t falling;
  locked();
  at = next_start(&falling);
  sched_edge(&s, at, PHASE_TIMER, 4000, &x);
  CHECK_EQ(sched_match(&s, 0, &x), SA_OFF);
  CHECK_EQ(s.fault, 1);
}

static void test_tick_check(void) {
  uint16_t x, at;
  uint8_t falling;
  locked();
  at = next_start(&falling);
  CHECK_EQ(sched_tick_check(&s, 0, at), SA_NONE);
  CHECK_EQ(sched_tick_check(&s, 1, at), SA_OFF); /* high while nothing is scheduled */
  CHECK_EQ(s.fault, 1);
  sched_edge(&s, at, PHASE_TIMER, 4000, &x);
  CHECK_EQ(sched_tick_check(&s, 1, at + 4001), SA_NONE); /* fired, release not yet handled */
  sched_match(&s, 1, &x);
  CHECK_EQ(sched_tick_check(&s, 1, at + 5000), SA_NONE);
  CHECK_EQ(sched_tick_check(&s, 1, x + RELEASE_OVERDUE + 1), SA_OFF);
  CHECK_EQ(s.fault, 2);
}

int main(void) {
  RUN(test_locks_and_measures_period);
  RUN(test_half_cycles_from_timer);
  RUN(test_rising_edges_not_used);
  RUN(test_falling_disturbance_ignored);
  RUN(test_small_offset_followed);
  RUN(test_jitter_not_missed);
  RUN(test_outside_capture_ignored);
  RUN(test_gap_rides_through);
  RUN(test_tracks_frequency);
  RUN(test_lock_needs_steady_edges);
  RUN(test_drift_then_snap_lets_go);
  RUN(test_phase_jump_lets_go);
  RUN(test_frequency_step);
  RUN(test_50hz);
  RUN(test_out_of_range_period);
  RUN(test_glitch_before_lock);
  RUN(test_clamp);
  RUN(test_gap_longer_than_counter_wrap);
  RUN(test_zo_lost);
  RUN(test_schedule);
  RUN(test_schedule_no_delay_or_not_locked);
  RUN(test_schedule_clamps_late_delay);
  RUN(test_match_without_pin_is_fault);
  RUN(test_tick_check);
  TEST_EXIT();
}
