#include "update.h"
#include "crc16.h"
#include "flashmap.h"
#include "protocol.h"

#define MAGIC_LEN (sizeof APP_MAGIC - 1u)
_Static_assert(TR_MAGIC + MAGIC_LEN <= TR_CODE_LEN, "the magic overlaps the code length");
_Static_assert(!(APP_SIZE % WRITE_CHUNK) && TRAILER_SIZE <= WRITE_CHUNK,
               "the application area is whole chunks, and the trailer lies in the last");

#define NO_SUCH_LEN 0xFFu
_Static_assert(BOOT_MAX_PAYLOAD < NO_SUCH_LEN, "a payload length must differ from NO_SUCH_LEN");

static XDATA uint8_t zero = 0x00;

void upd_init(upd_t IDATA *u) {
  u->receiving = 0;
  u->next = 0;
  u->result = BR_NO_COMMAND;
}

static uint16_t read16(uint16_t addr) {
  return flash_read(addr) | ((uint16_t) flash_read(addr + 1) << 8);
}

static uint8_t magic_ok(void) {
  uint8_t i;
  for (i = 0; i < MAGIC_LEN; i++)
    if (flash_read(TRAILER_ADDR + TR_MAGIC + i) != APP_MAGIC[i])
      return 0;
  return 1;
}

/* Cannot wrap past 0xFFFF, whatever addr is. */
uint8_t in_app_area(uint16_t addr, uint8_t n) {
  return (uint16_t) (addr - APP_BASE) < APP_SIZE && n <= (uint16_t) (APP_END - addr);
}

uint8_t app_valid(void) { return magic_ok() && flash_read(TRAILER_ADDR + TR_VALID) == 0x00; }

static uint8_t begin(upd_t IDATA *u, const uint8_t XDATA *p) {
  uint16_t a;
  uint16_t trailer_sector = TRAILER_ADDR & ~(SECTOR_SIZE - 1);
  u->receiving = 0;
  if ((p[0] | ((uint16_t) p[1] << 8)) != APP_SIZE)
    return BR_BAD_SIZE;
  /* The old application is invalid from here on: its magic is cleared (programming only clears
     bits, so this also works on an erased sector), so that an erase cut short cannot leave it
     looking valid. Then the trailer's sector goes first. */
  for (a = TRAILER_ADDR + TR_MAGIC; a < TRAILER_ADDR + TR_MAGIC + MAGIC_LEN; a++)
    if (!flash_write(a, &zero, 1))
      return BR_VERIFY;
  if (!flash_erase(trailer_sector))
    return BR_VERIFY;
  for (a = APP_BASE; a < APP_END; a += SECTOR_SIZE) {
    wdt_kick();
    if (a != trailer_sector && !flash_erase(a))
      return BR_VERIFY;
  }
  u->receiving = 1;
  u->next = 0;
  return BR_OK;
}

static uint8_t write_chunk(upd_t IDATA *u, const uint8_t XDATA *p) {
  uint16_t off;
  uint8_t i;
  const uint8_t XDATA *data = p + 2;
  if (!u->receiving)
    return BR_BAD_STATE;
  off = p[0] | ((uint16_t) p[1] << 8);
  if ((off & (WRITE_CHUNK - 1)) || off > APP_SIZE - WRITE_CHUNK)
    return BR_BAD_OFFSET;
  if (off < u->next) {
    /* A repeat whose reply was lost. */
    for (i = 0; i < WRITE_CHUNK; i++)
      if (flash_read(APP_BASE + off + i) != data[i])
        return BR_MISMATCH;
    return BR_OK;
  }
  if (off != u->next)
    return BR_BAD_OFFSET;
  if (off == APP_SIZE - WRITE_CHUNK && data[WRITE_CHUNK - TRAILER_SIZE + TR_VALID] != 0xFF)
    return BR_BAD_IMAGE;
  if (!flash_write(APP_BASE + off, data, WRITE_CHUNK))
    return BR_VERIFY;
  for (i = 0; i < WRITE_CHUNK; i++)
    if (flash_read(APP_BASE + off + i) != data[i])
      return BR_VERIFY;
  u->next = off + WRITE_CHUNK;
  return BR_OK;
}

static uint8_t finish(upd_t IDATA *u) {
  uint16_t len, crc, a;
  if (!u->receiving || u->next != APP_SIZE)
    return BR_BAD_STATE;
  if (!magic_ok())
    return BR_BAD_IMAGE;
  len = read16(TRAILER_ADDR + TR_CODE_LEN);
  if (len == 0 || len > APP_SIZE - TRAILER_SIZE)
    return BR_BAD_IMAGE;
  crc = CRC16_INIT;
  for (a = 0; a < len; a++) {
    if (!(a & 0xFF))
      wdt_kick();
    crc = crc16_update(crc, flash_read(APP_BASE + a));
  }
  if (crc != read16(TRAILER_ADDR + TR_CRC))
    return BR_BAD_IMAGE;
  if (!flash_write(TRAILER_ADDR + TR_VALID, &zero, 1) || flash_read(TRAILER_ADDR + TR_VALID) != 0)
    return BR_VERIFY;
  u->receiving = 0;
  return BR_OK;
}

static uint8_t payload_len(uint8_t type) {
  switch (type) {
    case MSG_BOOT_BEGIN:
      return BOOT_BEGIN_LEN;
    case MSG_BOOT_WRITE:
      return BOOT_WRITE_LEN;
    case MSG_BOOT_FINISH:
    case MSG_BOOT_RUN:
      return 0;
    default:
      return NO_SUCH_LEN;
  }
}

uint8_t upd_handle(upd_t IDATA *u, uint8_t type, const uint8_t XDATA *p, uint8_t len) {
  if (len != payload_len(type)) {
    u->result = BR_BAD_FRAME;
    return UA_NONE;
  }
  switch (type) {
    case MSG_BOOT_BEGIN:
      u->result = begin(u, p);
      break;
    case MSG_BOOT_WRITE:
      u->result = write_chunk(u, p);
      break;
    case MSG_BOOT_FINISH:
      u->result = finish(u);
      break;
    case MSG_BOOT_RUN:
      if (!app_valid()) {
        u->result = BR_BAD_STATE;
        break;
      }
      u->result = BR_OK;
      return UA_RUN;
  }
  return UA_NONE;
}

void upd_status(const upd_t IDATA *u, uint8_t reason, uint8_t XDATA *out) {
  out[0] = BOOT_VERSION;
  out[1] = app_valid();
  out[2] = reason;
  out[3] = u->result;
  out[4] = (uint8_t) u->next;
  out[5] = (uint8_t) (u->next >> 8);
}
