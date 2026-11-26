/*
 * Gate scheduling decisions for the interrupt handlers. The handlers in hal.c only carry out
 * the returned action on the registers, so this logic runs unchanged in the host simulation.
 *
 * sched_edge is called only from the Zo interrupt and sched_match only from the PCA interrupt (both
 * high priority, so they never interrupt each other). The 1 ms tick calls sched_zo_lost, which only
 * reads a byte the Zo interrupt writes, and sched_zo_reset and sched_tick_check with interrupts
 * disabled.
 */
#ifndef SCHED_H
#define SCHED_H

#include "compiler.h"
#include "phase.h"

#define GS_IDLE 0u
#define GS_WAIT_FIRE 1u
#define GS_WAIT_RELEASE 2u

#define SA_NONE 0u
#define SA_OFF 1u      /* gate low, PCA module 0 disarmed */
#define SA_ARM 2u      /* SA_OFF, then arm module 0 at *t */
#define SA_RELEASE 3u  /* write *t as the next compare value (the release) */

typedef struct {
  phase_t phase;
  uint8_t gs;
  uint16_t release_at;
  uint8_t edges;    /* accepted edges, wrapping: one per half-cycle */
  uint8_t zo_valid; /* phase_valid() as of the last edge, 0 once Zo is lost */
  uint8_t fault;    /* gate faults seen; cleared by the reader */
  uint8_t tick_edges;
  uint8_t ticks_since_edge; /* saturates just above ZO_LOST_MS */
} sched_t;

void sched_init(sched_t IDATA *s);
/* delay: firing delay in PCA counts, 0 = no firing. */
uint8_t sched_edge(sched_t IDATA *s, uint16_t now, uint8_t level, uint16_t delay, uint16_t IDATA *t);
uint8_t sched_match(sched_t IDATA *s, uint8_t pin_high, uint16_t IDATA *t);
/* 1 at the tick that ends ZO_LOST_MS without a Zo edge: then call sched_zo_reset. After a gap the
   edge timestamps cannot be trusted: a gap of a multiple of the 16-bit PCA counter's period looks
   like a normal half-cycle. */
uint8_t sched_zo_lost(sched_t IDATA *s);
void sched_zo_reset(sched_t IDATA *s);
uint8_t sched_tick_check(sched_t IDATA *s, uint8_t pin_high, uint16_t now);

/* Clamps a firing delay to the window for this half-period. Returns 0, no firing, for a delay of
   0, and for a half-period that is unknown (0) or too short for the window. Called from both the
   main loop and the Zo interrupt, so it is reentrant. */
uint16_t sched_clamp(uint16_t delay, uint16_t half) REENTRANT;

#endif
