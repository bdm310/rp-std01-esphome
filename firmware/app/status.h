#ifndef STATUS_H
#define STATUS_H

#include "compiler.h"
#include "window.h"

typedef struct {
  uint8_t state;
  uint8_t faults;
  uint16_t target;
  uint16_t current;
  uint16_t half;
  uint8_t relay;
  uint16_t therm;
  uint16_t wheel;
  uint8_t starting; /* thermistor average not ready yet */
  uint8_t boot_id;
  uint8_t leds;     /* outputs in use, as received in MSG_OUTPUTS */
  uint8_t night;
} status_in_t;

/* Writes STATUS_LEN bytes. */
void status_build(uint8_t XDATA *out, const status_in_t XDATA *s, const window_t XDATA *w);
/* Writes INFO_LEN bytes. */
void info_build(uint8_t XDATA *out, uint16_t version, uint32_t build, uint8_t reset_cause,
                uint8_t wdt_count, uint8_t boot_version, uint8_t boot_id);

#endif
