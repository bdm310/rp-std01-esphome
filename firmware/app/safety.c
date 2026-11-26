#include "safety.h"
#include "board.h"
#include "protocol.h"

void safety_init(safety_t XDATA *s) { s->hot = 0; }

uint8_t safety_therm(safety_t XDATA *s, uint16_t raw, uint8_t ready) {
  uint8_t f = 0;
  if (!ready)
    return F_THERM;
  if (raw < THERM_OPEN_SHORT_LOW || raw > THERM_OPEN_SHORT_HIGH)
    f |= F_THERM;
  if (raw <= THERM_TRIP)
    s->hot = 1;
  else if (raw >= THERM_CLEAR)
    s->hot = 0;
  if (s->hot)
    f |= F_OVERTEMP;
  return f;
}
