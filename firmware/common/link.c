/*
 * Frame receiver. The parser works on a working copy at a fixed address, w: on the 8051, fields at
 * a fixed address take far less code than fields behind a pointer, and keep SDCC from placing
 * locals in the application's scarce direct RAM.
 *
 * LINK_SINGLE, set for SDCC builds, which have one receiver: the parser state lives only in w, so
 * link_rx_t holds only what the caller uses; rx just passes the buffer and limit in, and the frame
 * type and length out. Copying the whole receiver in and out instead would cost about 0.1 ms per
 * byte on the MCU. Otherwise (host tests) w is loaded from rx and saved back on every call, so any
 * number of receivers work.
 */
#include "link.h"
#include "crc16.h"
#include "protocol.h"

enum { RX_START, RX_TYPE, RX_LEN, RX_DATA, RX_CRC_LO, RX_CRC_HI };

typedef struct {
  uint8_t type;
  uint8_t len;
  uint8_t max;
  uint8_t first_type;
  uint8_t last_type;
  uint8_t XDATA *buf;
  link_parse_t p;
} work_t;

static XDATA work_t w;
static XDATA uint8_t w_byte, w_good;

#ifdef LINK_SINGLE
#define load(rx)
static void save(link_rx_t XDATA *rx) {
  rx->type = w.type;
  rx->len = w.len;
}
#else
static void load(const link_rx_t *rx) {
  w.type = rx->type;
  w.len = rx->len;
  w.max = rx->max;
  w.first_type = rx->first_type;
  w.last_type = rx->last_type;
  w.buf = rx->buf;
  w.p = rx->p;
}
static void save(link_rx_t *rx) {
  rx->type = w.type;
  rx->len = w.len;
  rx->p = w.p;
}
#endif

void link_rx_init(link_rx_t XDATA *rx, uint8_t XDATA *buf, uint8_t max) {
  rx->buf = buf;
  rx->max = max;
#ifdef LINK_SINGLE
  w.buf = buf;
  w.max = max;
#endif
  link_rx_types(rx, 0x00, 0xFF);
  link_rx_drop_partial(rx);
}

void link_rx_types(link_rx_t XDATA *rx, uint8_t first, uint8_t last) {
#ifdef LINK_SINGLE
  (void) rx;
  w.first_type = first;
  w.last_type = last;
#else
  rx->first_type = first;
  rx->last_type = last;
#endif
}

void link_rx_drop_partial(link_rx_t XDATA *rx) {
  load(rx);
  w.p.state = RX_START;
  save(rx);
}

/* w_byte through the parser; w_good is 1 at the end of a good frame. */
static void step(void) {
  w_good = 0;
  switch (w.p.state) {
    case RX_START:
      if (w_byte == FRAME_START)
        w.p.state = RX_TYPE;
      return;
    case RX_TYPE:
      if (w_byte < w.first_type || w_byte > w.last_type)
        break;
      w.type = w_byte;
      w.p.crc = crc16_update(CRC16_INIT, w_byte);
      w.p.state = RX_LEN;
      return;
    case RX_LEN:
      if (w_byte > w.max)
        break;
      w.len = w_byte;
      w.p.idx = 0;
      w.p.crc = crc16_update(w.p.crc, w_byte);
      w.p.state = w_byte ? RX_DATA : RX_CRC_LO;
      return;
    case RX_DATA:
      w.buf[w.p.idx] = w_byte;
      w.p.crc = crc16_update(w.p.crc, w_byte);
      if (++w.p.idx == w.len)
        w.p.state = RX_CRC_LO;
      return;
    case RX_CRC_LO:
      w.p.crc_lo = w_byte;
      w.p.state = RX_CRC_HI;
      return;
    default:
      w_good = w.p.crc_lo == (uint8_t) w.p.crc && w_byte == (uint8_t) (w.p.crc >> 8);
      break;
  }
  w.p.state = !w_good && w_byte == FRAME_START ? RX_TYPE : RX_START;
}

uint8_t link_rx_byte(link_rx_t XDATA *rx, uint8_t byte) {
  load(rx);
  w_byte = byte;
  step();
  save(rx);
  return w_good;
}

uint8_t link_build(uint8_t XDATA *out, uint8_t type, const uint8_t XDATA *payload, uint8_t len) {
  uint16_t crc;
  uint8_t i;
  out[0] = FRAME_START;
  out[1] = type;
  out[2] = len;
  crc = crc16_update(CRC16_INIT, type);
  crc = crc16_update(crc, len);
  for (i = 0; i < len; i++) {
    out[3 + i] = payload[i];
    crc = crc16_update(crc, payload[i]);
  }
  out[3 + len] = (uint8_t) crc;
  out[4 + len] = (uint8_t) (crc >> 8);
  return len + FRAME_OVERHEAD;
}
