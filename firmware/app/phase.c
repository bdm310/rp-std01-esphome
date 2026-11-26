#include "phase.h"
#include "board.h"

void phase_init(phase_t IDATA *p) REENTRANT {
  p->period = 0;
  p->period_x8 = 0;
  p->good = 0;
  p->level = 0;
  p->have_edge = 0;
  p->have_rise = 0;
}

uint8_t phase_valid(const phase_t IDATA *p) { return p->good >= ZO_GOOD_EDGES && p->period != 0; }

uint8_t phase_edge(phase_t IDATA *p, uint16_t ts, uint8_t level, uint16_t IDATA *ref) {
  uint16_t dt;
  if (p->have_edge) {
    dt = ts - p->last_edge;
    /* Noise: the same level again (a glitch shorter than the interrupt latency), too soon
       after the last edge, or, once locked, too early for the next crossing. A truly missed
       edge shows up as a gap at the next accepted one. */
    if (level == p->level || dt < EDGE_MIN ||
        (phase_valid(p) && dt < (p->period >> 1) - LOCK_WINDOW)) {
      return PHASE_IGNORED;
    }
    if (dt > EDGE_MAX)
      p->good = 0;
    else if (p->good != 0xFF)
      p->good++;
  }
  p->have_edge = 1;
  p->last_edge = ts;
  p->level = level;

  if (level) {
    if (p->have_rise) {
      dt = ts - p->last_rise;
      if (dt < PERIOD_MIN || dt > PERIOD_MAX) {
        p->good = 0;
      } else if (p->period == 0) {
        p->period_x8 = (uint32_t) dt << 3;
        p->period = dt;
      } else {
        /* Moves 1/8 of the way to each new value. Written without a signed difference, so it
           behaves the same with SDCC's 16-bit int. */
        uint16_t cur = (uint16_t) (p->period_x8 >> 3);
        if (dt >= cur)
          p->period_x8 += dt - cur;
        else
          p->period_x8 -= cur - dt;
        p->period = (uint16_t) ((p->period_x8 + 4) >> 3);
      }
    }
    p->have_rise = 1;
    p->last_rise = ts;
    if (p->period == 0)
      return PHASE_NO_REF;
    *ref = ts;
    return PHASE_REF;
  }

  if (!p->have_rise || p->period == 0)
    return PHASE_NO_REF;
  /* As stock: the falling-edge half-cycle is timed from the rising edge plus half a period. */
  *ref = p->last_rise + (p->period >> 1);
  return PHASE_REF;
}
