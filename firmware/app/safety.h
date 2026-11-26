/* Thermistor checks. Thresholds are build constants, never set over the link. */
#ifndef SAFETY_H
#define SAFETY_H

#include "compiler.h"

typedef struct {
  uint8_t hot;
} safety_t;

void safety_init(safety_t XDATA *s);
/* raw: averaged thermistor reading; ready: 0 until the first average exists. Returns F_* bits. */
uint8_t safety_therm(safety_t XDATA *s, uint16_t raw, uint8_t ready);

#endif
