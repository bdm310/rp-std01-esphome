/*
 * Zo edge tracking: noise rejection, mains period, and the reference time of each half-cycle.
 * Used from the Zo interrupt, and elsewhere only with interrupts disabled.
 */
#ifndef PHASE_H
#define PHASE_H

#include "compiler.h"

typedef struct {
  uint16_t last_edge;
  uint16_t last_rise;
  uint16_t period; /* rising to rising, filtered; 0 = unknown */
  uint32_t period_x8; /* filter state: period with 3 fractional bits */
  uint8_t good;    /* consecutive edges with plausible spacing, saturating */
  uint8_t level;
  uint8_t have_edge;
  uint8_t have_rise;
} phase_t;

#define PHASE_IGNORED 0u /* noise: change nothing */
#define PHASE_NO_REF 1u  /* a real edge, but no reference time yet */
#define PHASE_REF 2u     /* a real edge, *ref holds the half-cycle's reference time */

void phase_init(phase_t IDATA *p) REENTRANT;
uint8_t phase_edge(phase_t IDATA *p, uint16_t ts, uint8_t level, uint16_t IDATA *ref);
uint8_t phase_valid(const phase_t IDATA *p);

#endif
