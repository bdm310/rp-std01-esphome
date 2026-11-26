#include "gate.h"
#include "board.h"
#include "protocol.h"
#include "sched.h"

void gate_init(gate_t XDATA *g) {
  g->state = ST_OFF;
  g->target = 0;
  g->clamped_target = 0;
  g->current = 0;
  g->limit_brighter = LIMIT_BRIGHTER;
  g->limit_dimmer = LIMIT_DIMMER;
  g->zero_seen = 1;
  g->waiting_edge = 0;
  g->relay = 0;
  g->fire = 0;
  g->update_requested = 0;
  g->startup = 1;
}

void gate_startup_done(gate_t XDATA *g, uint8_t faults) {
  if (g->startup && !faults && g->state == ST_FAULT)
    g->zero_seen = 1;
  g->startup = 0;
}

void gate_set_target(gate_t XDATA *g, uint16_t target, uint8_t limit_brighter, uint8_t limit_dimmer) {
  g->target = target;
  g->limit_brighter = limit_brighter;
  g->limit_dimmer = limit_dimmer;
  /* Leaving FAULT needs a 0 sent after the last non-zero target. */
  if (target == 0) {
    g->zero_seen = 1;
  } else {
    /* The ESP32 sends a non-zero target after an update request only if it gave the request
       up; the lamp must then not stop for the bootloader at its next turn-off. */
    g->update_requested = 0;
    if (g->state == ST_FAULT)
      g->zero_seen = 0;
  }
}

void gate_request_update(gate_t XDATA *g) {
  gate_set_target(g, 0, g->limit_brighter, g->limit_dimmer);
  g->update_requested = 1;
}

/* The relay drive is off only in OFF, and in FAULT once its turn-off is done. */
uint8_t gate_update_ready(gate_t XDATA *g) { return g->update_requested && !g->relay; }

/* Clears the delay and starts waiting for the half-cycle that may still conduct to end. */
static void stop_firing(gate_t XDATA *g, uint8_t edges) {
  g->fire = 0;
  g->waiting_edge = 1;
  g->edge_mark = edges;
}

/* RELAY_OFF_DELAY_MS after the second Zo edge since stop_firing: the first edge may have fired
   with the old delay. */
static uint8_t relay_may_open(gate_t XDATA *g, uint16_t now_ms, uint8_t edges) {
  if (g->waiting_edge) {
    if ((uint8_t) (edges - g->edge_mark) >= 2) {
      g->waiting_edge = 0;
      g->t_mark = now_ms;
    }
    return 0;
  }
  return (uint16_t) (now_ms - g->t_mark) >= RELAY_OFF_DELAY_MS;
}

void gate_tick(gate_t XDATA *g, uint16_t now_ms, uint8_t edges, uint16_t half, uint8_t zo_ok,
               uint8_t faults) {
  g->clamped_target = sched_clamp(g->target, half);
  /* Before the thermistor average is ready the safety check reports only F_THERM: anything
     else is a real fault, and ends the start-up exception. */
  if (faults & ~F_THERM)
    g->startup = 0;

  if (g->state != ST_FAULT && (faults || (g->relay && !zo_ok))) {
    if (g->state == ST_FIRING) {
      stop_firing(g, edges);
    } else if (g->state != ST_STOPPING) {
      /* Before firing starts nothing conducts, so the relay opens at once. */
      g->relay = 0;
    }
    g->state = ST_FAULT;
    g->fire = 0;
    g->fault_since_ms = now_ms;
    g->zero_seen = g->target == 0;
    return;
  }

  switch (g->state) {
    case ST_OFF:
      if (g->clamped_target != 0 && zo_ok) {
        g->state = ST_SETTLE;
        g->relay = 1;
        g->t_mark = now_ms;
      }
      break;
    case ST_SETTLE:
      if (g->target == 0) {
        g->state = ST_OFF;
        g->relay = 0;
      } else if ((uint16_t) (now_ms - g->t_mark) >= RELAY_SETTLE_MS && g->clamped_target != 0) {
        g->state = ST_FIRING;
        g->current = g->clamped_target;
        g->fire = g->current;
      }
      break;
    case ST_FIRING:
      if (g->target == 0) {
        g->state = ST_STOPPING;
        stop_firing(g, edges);
      }
      break;
    case ST_STOPPING:
      if (relay_may_open(g, now_ms, edges)) {
        g->state = ST_OFF;
        g->relay = 0;
      }
      break;
    case ST_FAULT:
    default:
      g->fire = 0;
      if (g->relay &&
          (relay_may_open(g, now_ms, edges) || (uint16_t) (now_ms - g->fault_since_ms) >= FAULT_RELAY_MS))
        g->relay = 0;
      if (!g->relay && !faults && g->zero_seen)
        g->state = ST_OFF;
      break;
  }
}

void gate_half_cycle(gate_t XDATA *g) {
  uint16_t d;
  if (g->state != ST_FIRING || g->clamped_target == 0)
    return;
  if (g->current > g->clamped_target) {
    d = g->current - g->clamped_target;
    g->current =
        (g->limit_brighter && d > g->limit_brighter) ? g->current - g->limit_brighter : g->clamped_target;
  } else if (g->current < g->clamped_target) {
    d = g->clamped_target - g->current;
    g->current = (g->limit_dimmer && d > g->limit_dimmer) ? g->current + g->limit_dimmer : g->clamped_target;
  }
  g->fire = g->current;
}
