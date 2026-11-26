/* Link frames, for the application and the bootloader. */
#ifndef LINK_H
#define LINK_H

#include "compiler.h"

#if defined(__SDCC) && !defined(LINK_SINGLE)
#define LINK_SINGLE
#endif

typedef struct {
  uint8_t state;
  uint8_t idx;
  uint8_t crc_lo;
  uint16_t crc;
} link_parse_t;

typedef struct {
  uint8_t type;
  uint8_t len;
  uint8_t max;
  uint8_t XDATA *buf;
#ifndef LINK_SINGLE
  uint8_t first_type;
  uint8_t last_type;
  link_parse_t p;
#endif
} link_rx_t;

/* Accepts every frame type until link_rx_types narrows it. */
void link_rx_init(link_rx_t XDATA *rx, uint8_t XDATA *buf, uint8_t max);
/*
 * Accepts only frame types first to last. Any other type byte ends the frame at once, and the
 * search for a start byte goes on from that byte, so a false start is given up there instead of
 * swallowing the real frame that follows.
 */
void link_rx_types(link_rx_t XDATA *rx, uint8_t first, uint8_t last);
void link_rx_drop_partial(link_rx_t XDATA *rx);
/* Returns 1 when a complete, valid frame is in rx->type, rx->len, rx->buf. */
uint8_t link_rx_byte(link_rx_t XDATA *rx, uint8_t byte);

/* Writes a whole frame to out (len + FRAME_OVERHEAD bytes). Returns the frame length. */
uint8_t link_build(uint8_t XDATA *out, uint8_t type, const uint8_t XDATA *payload, uint8_t len);

#endif
