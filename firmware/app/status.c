#include "status.h"
#include "protocol.h"

static void put16(uint8_t XDATA *p, uint16_t v) {
  p[0] = (uint8_t) v;
  p[1] = (uint8_t) (v >> 8);
}

void status_build(uint8_t XDATA *out, const status_in_t XDATA *s, const window_t XDATA *w) {
  uint8_t changed = w->btn_and ^ w->btn_or, relay, stale;
  out[SF_STATE] = s->state;
  out[SF_FAULTS] = s->faults;
  out[SF_BUTTONS] = (w->btn_last & BTN_ALL) | ((changed & BTN_ALL) << BTN_CHANGED_SHIFT);
  put16(out + SF_TARGET, s->target);
  put16(out + SF_CURRENT, s->current);
  put16(out + SF_HALF, s->half);
  put16(out + SF_KS_MIN, w->ks_min);
  /* Ks stale: the samples saw both drive states, the drive switched after the last one, or there
     were none. */
  relay = s->relay ? 1 : 0;
  stale = w->relay_and != w->relay_or || w->relay_or != relay;
  out[SF_FLAGS] = (relay ? S_RELAY : 0) | (stale ? S_KS_STALE : 0) | (s->starting ? S_STARTING : 0) |
                  (s->coasted ? S_ZO_COASTED : 0);
  put16(out + SF_THERM, s->therm);
  put16(out + SF_WHEEL, s->wheel);
  out[SF_BOOT_ID] = s->boot_id;
  out[SF_LEDS] = s->leds;
  out[SF_NIGHT] = s->night;
}

void info_build(uint8_t XDATA *out, uint16_t version, uint32_t build, uint8_t reset_cause,
                uint8_t wdt_count, uint8_t boot_version, uint8_t boot_id) {
  put16(out, version);
  put16(out + 2, (uint16_t) build);
  put16(out + 4, (uint16_t) (build >> 16));
  out[6] = reset_cause;
  out[7] = wdt_count;
  out[8] = boot_version;
  out[9] = boot_id;
}
