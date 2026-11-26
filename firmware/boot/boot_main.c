#include "boot.h"
#include "flashmap.h"
#include "protocol.h"
#include "stc15.h"
#include "update.h"

/* The update only writes the application area, so this byte never changes. */
const __code __at(BOOT_VERSION_ADDR) uint8_t boot_version = BOOT_VERSION;

void main(void) {
  uint8_t wdt = WDT_CONTR;
  uint8_t pcon = PCON;
  uint8_t reason;

  wdt_kick(); /* starts the watchdog and clears WDT_FLAG */
  boot_hal_iap_idle();
  if (pcon & PCON_POF)
    PCON = pcon & ~PCON_POF;

  reason = boot_reason(pcon & PCON_POF, wdt & WDT_FLAG);
  if (!reason)
    ((void (*)(void)) APP_BASE)();
  boot_hal_init();
  boot_init(reason);
  for (;;)
    boot_loop();
}
