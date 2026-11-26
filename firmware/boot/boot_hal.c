/* The bootloader's hardware layer (SDCC only). It uses no interrupts: the UART and a 1 ms timer
   are polled. */
#include "boot.h"
#include "flashmap.h"
#include "protocol.h"
#include "stc15.h"
#include "update.h"

/* Received bytes wait here, so none is lost while the bootloader sends: the UART holds only one. */
_Static_assert(BOOT_RX_RING > BOOT_STATUS_LEN + FRAME_OVERHEAD && !(BOOT_RX_RING & (BOOT_RX_RING - 1)),
               "BOOT_RX_RING: a power of 2 that holds what arrives while a boot status is sent, "
               "and one more");
static XDATA uint8_t rx_ring[BOOT_RX_RING];
static uint8_t rx_head, rx_tail;

void boot_hal_take(void) {
  if (RI) {
    RI = 0;
    if ((uint8_t) (rx_head - rx_tail) < BOOT_RX_RING)
      rx_ring[rx_head++ & (BOOT_RX_RING - 1)] = SBUF;
  }
}

uint8_t boot_hal_read(uint8_t DATA *b) {
  if (rx_tail == rx_head)
    return 0;
  *b = rx_ring[rx_tail++ & (BOOT_RX_RING - 1)];
  return 1;
}

void boot_hal_send(const uint8_t XDATA *frame, uint8_t n) {
  uint8_t i;
  for (i = 0; i < n; i++) {
    TI = 0;
    SBUF = frame[i];
    while (!TI)
      boot_hal_take();
  }
  TI = 0;
}

uint8_t boot_hal_tick(void) {
  if (!TF0)
    return 0;
  TF0 = 0;
  return 1;
}

void boot_hal_restart(void) {
  IAP_CONTR = IAP_SWRST_APP;
  for (;;) {
  }
}

void boot_hal_init(void) {
  P_SW1 = P_SW1_UART1_P3;
  SCON = SCON_8BIT_RX;
  T2L = (uint8_t) T2_RELOAD_BAUD;
  T2H = (uint8_t) (T2_RELOAD_BAUD >> 8);
  TMOD = TMOD_T0_RELOAD;
  TL0 = (uint8_t) T0_RELOAD_1MS;
  TH0 = (uint8_t) (T0_RELOAD_1MS >> 8);
  AUXR = AUXR_TIMERS;
  TR0 = 1;
}

void wdt_kick(void) { WDT_CONTR = WDT_RUN_CLEAR; }

void boot_hal_iap_idle(void) {
  IAP_CONTR = 0;
  IAP_CMD = 0;
  IAP_TRIG = 0;
  IAP_ADDRH = 0x80;
  IAP_ADDRL = 0;
}

static uint8_t iap_run(uint8_t cmd, uint16_t addr) {
  uint8_t ok;
  IAP_CONTR = IAP_ENABLE;
  IAP_CMD = cmd;
  IAP_ADDRL = (uint8_t) addr;
  IAP_ADDRH = (uint8_t) (addr >> 8);
  IAP_TRIG = 0x5A;
  IAP_TRIG = 0xA5;
  __asm__("nop");
  ok = !(IAP_CONTR & IAP_CMD_FAIL);
  boot_hal_iap_idle();
  return ok;
}

uint8_t flash_erase(uint16_t addr) {
  if (!in_app_area(addr, 1))
    return 0;
  return iap_run(IAP_CMD_ERASE, addr);
}

uint8_t flash_write(uint16_t addr, const uint8_t XDATA *data, uint8_t n) {
  uint8_t i;
  if (!in_app_area(addr, n))
    return 0;
  for (i = 0; i < n; i++) {
    IAP_DATA = data[i];
    if (!iap_run(IAP_CMD_WRITE, addr + i))
      return 0;
  }
  return 1;
}

uint8_t flash_read(uint16_t addr) { return *(const uint8_t CODE *) addr; }
