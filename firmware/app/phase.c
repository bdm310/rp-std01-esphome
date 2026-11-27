#include "phase.h"
#include "board.h"

void phase_init(phase_t IDATA *p) REENTRANT {
  p->period = 0;
  p->period_x256 = 0;
  p->good = 0;
  p->missed = 0;
  p->outside_phase = 0;
  p->outside_capture = 0;
  p->flags = 0;
}

/* |a - b| without signed arithmetic, so it behaves the same with SDCC's 16-bit int. */
#define DIFF(a, b) ((uint16_t) ((a) >= (b) ? (a) - (b) : (b) - (a)))

uint8_t phase_edge(phase_t IDATA *p, uint16_t ts, uint8_t level, uint16_t IDATA *ref) {
  uint16_t d;

  if (level == PHASE_TIMER) {
    if (!phase_locked(p))
      return PHASE_IGNORED;
    *ref = p->next_at;
    if (p->flags & PF_NEXT_FALLING) {
      p->flags &= ~PF_NEXT_FALLING;
      p->next_at = (uint16_t) ((p->fall_x256 + (p->period_x256 >> 1)) >> 8);
      return PHASE_START;
    }
    /* Half a period after the falling half-cycle began, its falling edge has come, early or late,
       if it is there at all, and the next one has not. */
    if (p->flags & PF_FALL_SEEN) {
      p->missed = 0;
      p->outside_capture = 0;
    } else {
      p->missed++;
      p->outside_capture = (p->flags & PF_OUTSIDE_CAPTURE_SEEN) ? p->outside_capture + 1 : 0;
      if (p->missed >= ZO_MISSED_MAX || p->outside_capture >= ZO_OUTSIDE_CAPTURE_MAX) {
        phase_init(p);
        return PHASE_LOST;
      }
    }
    p->flags = (p->flags & ~(PF_FALL_SEEN | PF_OUTSIDE_CAPTURE_SEEN)) | PF_NEXT_FALLING;
    p->fall_x256 += p->period_x256;
    p->next_at = (uint16_t) (p->fall_x256 >> 8);
    return PHASE_START;
  }

  if (level)
    return phase_locked(p) ? PHASE_IGNORED : PHASE_SEEN;
  d = ts - p->last_fall;
  if ((p->flags & PF_HAVE_FALL) && d < EDGE_MIN)
    return PHASE_IGNORED;

  if (!phase_locked(p)) {
    if (!(p->flags & PF_HAVE_FALL) || d < PERIOD_MIN || d > PERIOD_MAX) {
      p->good = 0;
    } else if (p->good != 0 && DIFF(d, p->last_dt) <= PLL_PERIOD_WINDOW) {
      p->good++;
      p->fall_x256 += d;
    } else {
      p->good = 1;
      p->fall_x256 = d;
    }
    p->last_dt = d;
    p->last_fall = ts;
    p->flags |= PF_HAVE_FALL;
    if (p->good < ZO_GOOD_PERIODS)
      return PHASE_SEEN;
    p->period_x256 = p->fall_x256 << (8 - ZO_GOOD_PERIODS_LOG2);
    p->period = (uint16_t) ((p->period_x256 + 128) >> 8);
    p->good = 0;
    p->missed = 0;
    p->outside_phase = 0;
    p->outside_capture = 0;
    p->flags = PF_LOCKED | PF_FALL_SEEN | PF_HAVE_FALL;
    p->fall_x256 = (uint32_t) ts << 8;
    p->next_at = (uint16_t) ((p->fall_x256 + (p->period_x256 >> 1)) >> 8);
    *ref = ts;
    return PHASE_START;
  }

  /* The error from the predicted falling edge. Before the rising half-cycle fall_x256 is the last
     falling reference, so a late edge is just after it; from then on it is the next one, so an
     early edge is just before it. */
  d = ts - (uint16_t) (p->fall_x256 >> 8);
  if (d <= PLL_PHASE_WINDOW) {
    p->fall_x256 += (uint32_t) d << (8 - PLL_PHASE_SHIFT);
    p->outside_phase = 0;
  } else if ((uint16_t) -d <= PLL_PHASE_WINDOW) {
    p->fall_x256 -= (uint32_t) (uint16_t) -d << (8 - PLL_PHASE_SHIFT);
    p->outside_phase = 0;
  } else if (d <= ZO_CAPTURE || (uint16_t) -d <= ZO_CAPTURE) {
    if (++p->outside_phase >= ZO_OUTSIDE_PHASE_MAX) {
      phase_init(p);
      return PHASE_LOST;
    }
  } else {
    p->flags |= PF_OUTSIDE_CAPTURE_SEEN;
    return PHASE_IGNORED;
  }
  p->flags |= PF_FALL_SEEN;

  d = ts - p->last_fall;
  if (d < PERIOD_MIN || d > PERIOD_MAX) {
    p->good = 0;
  } else if (DIFF(d, p->period) <= PLL_PERIOD_WINDOW) {
    if (((uint32_t) d << 8) >= p->period_x256)
      p->period_x256 += (((uint32_t) d << 8) - p->period_x256) >> PLL_PERIOD_SHIFT;
    else
      p->period_x256 -= (p->period_x256 - ((uint32_t) d << 8)) >> PLL_PERIOD_SHIFT;
    p->period = (uint16_t) ((p->period_x256 + 128) >> 8);
    p->good = 0;
  } else if (DIFF(d, p->last_dt) <= PLL_PERIOD_WINDOW) {
    if (++p->good >= ZO_FREQ_AGREE) {
      p->period_x256 = (uint32_t) d << 8;
      p->period = d;
      p->fall_x256 = (uint32_t) ts << 8;
      p->good = 0;
      p->outside_phase = 0;
    }
  } else {
    p->good = 0;
  }
  p->last_dt = d;
  p->last_fall = ts;
  return PHASE_IGNORED;
}
