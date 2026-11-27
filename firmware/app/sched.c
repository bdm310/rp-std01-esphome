#include "sched.h"
#include "board.h"

void sched_init(sched_t IDATA *s) {
  phase_init(&s->phase);
  s->gs = GS_IDLE;
  s->release_at = 0;
  s->edges = 0;
  s->zo_valid = 0;
  s->fault = 0;
  s->tick_edges = 0;
  s->ticks_since_edge = 0;
}

uint16_t sched_clamp(uint16_t delay, uint16_t half) REENTRANT {
  uint16_t latest;
  if (delay == 0)
    return 0;
  if (half <= LATEST_BEFORE_EDGE + EARLIEST_FIRE)
    return 0;
  latest = half - LATEST_BEFORE_EDGE;
  if (delay < EARLIEST_FIRE)
    return EARLIEST_FIRE;
  if (delay > latest)
    return latest;
  return delay;
}

uint8_t sched_edge(sched_t IDATA *s, uint16_t now, uint8_t level, uint16_t delay, uint16_t IDATA *t) {
  uint16_t ref = 0, fire;
  uint8_t r = phase_edge(&s->phase, now, level, &ref);
  if (r == PHASE_IGNORED)
    return SA_NONE;
  if (r == PHASE_SEEN) {
    s->edges++;
    return SA_NONE;
  }
  s->gs = GS_IDLE;
  s->zo_valid = phase_locked(&s->phase);
  if (r == PHASE_LOST)
    return SA_OFF;
  s->edges++;
  /* Clamped again here: the delay may have been set against an older half-period. */
  delay = sched_clamp(delay, s->phase.period >> 1);
  if (delay == 0)
    return SA_OFF;
  fire = ref + delay;
  if ((int16_t) (fire - now) < SCHED_MARGIN)
    return SA_OFF;
  s->release_at = s->phase.next_at - RELEASE_BEFORE_EDGE;
  s->gs = GS_WAIT_FIRE;
  *t = fire;
  return SA_ARM;
}

uint8_t sched_match(sched_t IDATA *s, uint8_t pin_high, uint16_t IDATA *t) {
  if (s->gs == GS_WAIT_FIRE) {
    if (!pin_high) {
      s->fault++;
      s->gs = GS_IDLE;
      return SA_OFF;
    }
    s->gs = GS_WAIT_RELEASE;
    *t = s->release_at;
    return SA_RELEASE;
  }
  s->gs = GS_IDLE;
  return SA_OFF;
}

uint8_t sched_zo_lost(sched_t IDATA *s) {
  uint8_t edges = s->edges;
  if (edges != s->tick_edges) {
    s->tick_edges = edges;
    s->ticks_since_edge = 1;
    return 0;
  }
  return s->ticks_since_edge <= ZO_LOST_MS && ++s->ticks_since_edge > ZO_LOST_MS;
}

void sched_zo_reset(sched_t IDATA *s) {
  phase_init(&s->phase);
  s->zo_valid = 0;
}

uint8_t sched_tick_check(sched_t IDATA *s, uint8_t pin_high, uint16_t now) {
  if ((pin_high && s->gs == GS_IDLE) ||
      (s->gs != GS_IDLE && (int16_t) (now - s->release_at) > RELEASE_OVERDUE)) {
    s->fault++;
    s->gs = GS_IDLE;
    return SA_OFF;
  }
  return SA_NONE;
}
