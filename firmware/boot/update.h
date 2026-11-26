#ifndef UPDATE_H
#define UPDATE_H

#include "compiler.h"

uint8_t in_app_area(uint16_t addr, uint8_t n);
/* flash_erase and flash_write act only on the application area, and return 0 on failure. */
uint8_t flash_erase(uint16_t addr);
uint8_t flash_write(uint16_t addr, const uint8_t XDATA *data, uint8_t n);
uint8_t flash_read(uint16_t addr);
void wdt_kick(void);

typedef struct {
  uint8_t receiving;
  uint16_t next;   /* offset of the next chunk expected */
  uint8_t result;  /* BR_* */
} upd_t;

#define UA_NONE 0u
#define UA_RUN 1u

void upd_init(upd_t IDATA *u);
uint8_t app_valid(void);
/* Handles one frame from the ESP32. Returns UA_RUN when the application should be started. */
uint8_t upd_handle(upd_t IDATA *u, uint8_t type, const uint8_t XDATA *p, uint8_t len);
/* Writes BOOT_STATUS_LEN bytes. */
void upd_status(const upd_t IDATA *u, uint8_t reason, uint8_t XDATA *out);

#endif
