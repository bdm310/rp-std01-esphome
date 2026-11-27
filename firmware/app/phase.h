/*
 * Mains timing: a phase-locked reference that starts each half-cycle, corrected by Zo's falling
 * edges. Used from the high-priority interrupts, and elsewhere only with interrupts disabled.
 * Times with _x256 have 8 fractional bits; their 16 integer bits wrap with the PCA counter.
 */
#ifndef PHASE_H
#define PHASE_H

#include "compiler.h"

typedef struct {
  /* Locked: the falling reference, the last one or, from the rising half-cycle on, the next.
     Locking: the sum of the run's periods. */
  uint32_t fall_x256;
  uint32_t period_x256;
  uint16_t period; /* 0 = unknown */
  uint16_t last_fall;
  uint16_t last_dt; /* the period that ended at last_fall */
  uint16_t next_at;
  /* Locking: periods in a row that agree. Locked: periods in a row that agree with each other but
     not with period. */
  uint8_t good;
  uint8_t missed;
  uint8_t outside_phase;
  uint8_t outside_capture;
  uint8_t flags;
} phase_t;

#define PF_LOCKED 0x01u
#define PF_NEXT_FALLING 0x02u
#define PF_FALL_SEEN 0x04u
#define PF_OUTSIDE_CAPTURE_SEEN 0x08u
#define PF_HAVE_FALL 0x10u

#define phase_locked(p) (((p)->flags & PF_LOCKED) != 0)

/* As phase_edge's level: the timer reached next_at. One entry point for both keeps the interrupt
   code's fixed RAM, of which the 8051 has little, to one set of locals. */
#define PHASE_TIMER 2u

#define PHASE_IGNORED 0u
#define PHASE_SEEN 1u  /* locking: a Zo edge, which shows Zo is there */
#define PHASE_START 2u /* a half-cycle starts: *ref holds its reference; next_at is the next one */
#define PHASE_LOST 3u  /* the lock was dropped */

void phase_init(phase_t IDATA *p) REENTRANT;
/* level: the Zo pin after an edge at ts, or PHASE_TIMER (ts not used). */
uint8_t phase_edge(phase_t IDATA *p, uint16_t ts, uint8_t level, uint16_t IDATA *ref);

#endif
