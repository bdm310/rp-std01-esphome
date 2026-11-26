/*
 * Power state machine (main loop): links the relay drive to the firing delay, and smooths
 * delay changes.
 */
#ifndef GATE_H
#define GATE_H

#include "compiler.h"

typedef struct {
  uint8_t state;
  uint16_t target;
  uint16_t clamped_target;
  uint16_t current;   /* delay in use while firing */
  uint8_t limit_brighter;
  uint8_t limit_dimmer;
  /* FAULT may end: the target was 0 on entering FAULT or has been set to 0 since, or the start-up
     FAULT ended without a fault. */
  uint8_t zero_seen;
  uint8_t waiting_edge;
  uint8_t edge_mark;
  uint16_t t_mark;
  uint16_t fault_since_ms;
  uint8_t relay;
  uint16_t fire;      /* delay for the Zo interrupt; 0 = no firing */
  uint8_t update_requested;
  uint8_t startup;    /* the start-up FAULT may still end without a zero target */
} gate_t;

void gate_init(gate_t XDATA *g);
void gate_set_target(gate_t XDATA *g, uint16_t target, uint8_t limit_brighter, uint8_t limit_dimmer);
void gate_request_update(gate_t XDATA *g);
uint8_t gate_update_ready(gate_t XDATA *g);
/*
 * Called every 1 ms tick. edges: the scheduler's edge counter. zo_ok: Zo is present and
 * tracked. faults: fault bits that must stop the lamp (safety and gate faults).
 */
void gate_tick(gate_t XDATA *g, uint16_t now_ms, uint8_t edges, uint16_t half, uint8_t zo_ok,
               uint8_t faults);
/* Call once, at the first tick with the thermistor average ready, before gate_tick, with that
   tick's faults. */
void gate_startup_done(gate_t XDATA *g, uint8_t faults);
void gate_half_cycle(gate_t XDATA *g);

#endif
