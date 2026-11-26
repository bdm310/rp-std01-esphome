/*
 * Input statistics for one status window, and an 8-sample average for the slow ADC
 * channels. The *_add functions are called only from the 1 ms tick interrupt; the main loop
 * copies and resets a window with that interrupt disabled.
 */
#ifndef WINDOW_H
#define WINDOW_H

#include "compiler.h"

typedef struct {
  uint16_t ks_min;
  uint8_t btn_and;
  uint8_t btn_or;
  uint8_t btn_last;
  uint8_t relay_and;
  uint8_t relay_or;
} window_t;

typedef struct {
  uint16_t sum;
  uint8_t n;
  uint8_t ready;
  uint16_t value;
} avg8_t;

void window_reset(window_t IDATA *w);
void window_add_ks(window_t IDATA *w, uint16_t v, uint8_t relay);
void window_add_buttons(window_t IDATA *w, uint8_t bits);

void avg8_init(avg8_t IDATA *a);
void avg8_add(avg8_t IDATA *a, uint16_t v);

#endif
