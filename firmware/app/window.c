#include "window.h"
#include "protocol.h"

void window_reset(window_t IDATA *w) {
  w->ks_min = KS_NO_SAMPLE;
  w->btn_and = 0xFF;
  w->btn_or = 0;
  w->relay_and = 1;
  w->relay_or = 0;
}

void window_add_ks(window_t IDATA *w, uint16_t v, uint8_t relay) {
  if (v < w->ks_min)
    w->ks_min = v;
  w->relay_and &= relay;
  w->relay_or |= relay;
}

void window_add_buttons(window_t IDATA *w, uint8_t bits) {
  w->btn_and &= bits;
  w->btn_or |= bits;
  w->btn_last = bits;
}

void avg8_init(avg8_t IDATA *a) {
  a->sum = 0;
  a->n = 0;
  a->ready = 0;
  a->value = 0;
}

void avg8_add(avg8_t IDATA *a, uint16_t v) {
  a->sum += v;
  if (++a->n == 8) {
    a->value = a->sum >> 3;
    a->sum = 0;
    a->n = 0;
    a->ready = 1;
  }
}
