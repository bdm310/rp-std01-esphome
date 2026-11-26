#include "boot.h"
#include "board.h"
#include "flashmap.h"
#include "link.h"
#include "protocol.h"
#include "update.h"

#ifndef __SDCC
uint16_t boot_idle_return_ms = IDLE_RETURN_MS;
#endif

IDATA upd_t boot_upd;
uint8_t boot_update_reason;
volatile XDATA AT(HO_ADDR) uint8_t handover[HO_SIZE];
static XDATA link_rx_t rx;
static XDATA uint8_t rx_buf[BOOT_MAX_PAYLOAD];
static XDATA uint8_t payload[BOOT_STATUS_LEN];
static XDATA uint8_t frame[BOOT_STATUS_LEN + FRAME_OVERHEAD];
static uint16_t ms, last_status, last_rx, last_activity;

uint8_t boot_reason(uint8_t power_on, uint8_t watchdog) {
  if (handover[0] != HO_MAGIC0 || handover[1] != HO_MAGIC1) {
    handover[0] = HO_MAGIC0;
    handover[1] = HO_MAGIC1;
    handover[HO_REQUEST] = 0;
    handover[HO_WDT_COUNT] = 0;
  }
  if (power_on) {
    handover[HO_RESET_CAUSE] = RESET_POWER_ON;
    handover[HO_REQUEST] = 0;
    handover[HO_WDT_COUNT] = 0;
  } else if (watchdog) {
    handover[HO_RESET_CAUSE] = RESET_WATCHDOG;
    if (handover[HO_WDT_COUNT] != 0xFF)
      handover[HO_WDT_COUNT]++;
  } else {
    handover[HO_RESET_CAUSE] = RESET_OTHER;
  }

  if (handover[HO_REQUEST] == HO_REQUEST_UPDATE) {
    handover[HO_REQUEST] = 0;
    return BOOT_REASON_REQUEST;
  }
  if (!app_valid())
    return BOOT_REASON_INVALID;
  if (handover[HO_WDT_COUNT] >= BOOT_WDT_RESETS)
    return BOOT_REASON_WATCHDOG;
  return 0;
}

static void send_status(void) {
  upd_status(&boot_upd, boot_update_reason, payload);
  boot_hal_send(frame, link_build(frame, MSG_BOOT_STATUS, payload, BOOT_STATUS_LEN));
}

void boot_init(uint8_t why) {
  boot_update_reason = why;
  ms = last_status = last_rx = last_activity = 0;
  upd_init(&boot_upd);
  link_rx_init(&rx, rx_buf, BOOT_MAX_PAYLOAD);
  link_rx_types(&rx, MSG_BOOT_BEGIN, MSG_BOOT_RUN);
  send_status();
}

void boot_loop(void) {
  uint8_t b;
  wdt_kick();
  boot_hal_take();
  if (boot_hal_tick()) {
    ms++;
    if ((uint16_t) (ms - last_rx) > BOOT_RX_TIMEOUT_MS)
      link_rx_drop_partial(&rx);
    if ((uint16_t) (ms - last_status) >= BOOT_STATUS_PERIOD_MS) {
      last_status = ms;
      send_status();
    }
    if (boot_update_reason == BOOT_REASON_REQUEST && app_valid() &&
        (uint16_t) (ms - last_activity) >= BOOT_IDLE_RETURN_MS) {
      boot_hal_restart();
      return;
    }
  }
  if (boot_hal_read(&b)) {
    last_rx = ms;
    if (link_rx_byte(&rx, b)) {
      last_activity = ms;
      if (upd_handle(&boot_upd, rx.type, rx_buf, rx.len) == UA_RUN) {
        handover[HO_WDT_COUNT] = 0;
        send_status();
        boot_hal_restart();
        return;
      }
      if (rx.type == MSG_BOOT_FINISH && boot_upd.result == BR_OK)
        handover[HO_WDT_COUNT] = 0;
      send_status();
      last_status = ms;
    }
  }
}
