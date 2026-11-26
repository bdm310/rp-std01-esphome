#include <string.h>

#include "boot.h"
#include "flashmap.h"
#include "protocol.h"
#include "test.h"
#include "update.h"

static uint8_t flash[APP_END];

uint8_t flash_erase(uint16_t addr) {
  (void) addr;
  return 0;
}
uint8_t flash_write(uint16_t addr, const uint8_t *data, uint8_t n) {
  (void) addr;
  (void) data;
  (void) n;
  return 0;
}
uint8_t flash_read(uint16_t addr) { return flash[addr]; }
void wdt_kick(void) {}

void boot_hal_take(void) {}
uint8_t boot_hal_read(uint8_t *b) {
  (void) b;
  return 0;
}
void boot_hal_send(const uint8_t *frame, uint8_t n) {
  (void) frame;
  (void) n;
}
uint8_t boot_hal_tick(void) { return 0; }
void boot_hal_restart(void) {}

static void set_app_valid(uint8_t valid) {
  memset(flash, 0xFF, sizeof flash);
  memcpy(flash + TRAILER_ADDR + TR_MAGIC, APP_MAGIC, sizeof APP_MAGIC - 1);
  flash[TRAILER_ADDR + TR_VALID] = valid ? 0x00 : 0xFF;
}

/* Auxiliary RAM as after a power cut: anything. */
static void lose_handover(void) { memset((uint8_t *) handover, 0xEE, HO_SIZE); }

static void test_power_on(void) {
  set_app_valid(1);
  lose_handover();
  CHECK_EQ(boot_reason(1, 0), 0);
  CHECK_EQ(handover[0], HO_MAGIC0);
  CHECK_EQ(handover[1], HO_MAGIC1);
  CHECK_EQ(handover[HO_REQUEST], 0);
  CHECK_EQ(handover[HO_WDT_COUNT], 0);
  CHECK_EQ(handover[HO_RESET_CAUSE], RESET_POWER_ON);
}

static void test_power_on_clears_count_and_request(void) {
  set_app_valid(1);
  boot_reason(1, 0);
  handover[HO_REQUEST] = HO_REQUEST_UPDATE;
  handover[HO_WDT_COUNT] = BOOT_WDT_RESETS;
  CHECK_EQ(boot_reason(1, 0), 0);
  CHECK_EQ(handover[HO_REQUEST], 0);
  CHECK_EQ(handover[HO_WDT_COUNT], 0);
}

static void test_watchdog_resets_in_a_row(void) {
  unsigned i;
  set_app_valid(1);
  boot_reason(1, 0);
  for (i = 1; i < BOOT_WDT_RESETS; i++) {
    CHECK_EQ(boot_reason(0, 1), 0);
    CHECK_EQ(handover[HO_WDT_COUNT], i);
    CHECK_EQ(handover[HO_RESET_CAUSE], RESET_WATCHDOG);
  }
  CHECK_EQ(boot_reason(0, 1), BOOT_REASON_WATCHDOG);
  CHECK_EQ(boot_reason(0, 0), BOOT_REASON_WATCHDOG);
  CHECK_EQ(handover[HO_RESET_CAUSE], RESET_OTHER);
  handover[HO_WDT_COUNT] = 0xFF;
  CHECK_EQ(boot_reason(0, 1), BOOT_REASON_WATCHDOG);
  CHECK_EQ(handover[HO_WDT_COUNT], 0xFF);
}

static void test_watchdog_count_restarts_when_cleared(void) {
  set_app_valid(1);
  boot_reason(1, 0);
  boot_reason(0, 1);
  handover[HO_WDT_COUNT] = 0;
  CHECK_EQ(boot_reason(0, 1), 0);
  CHECK_EQ(handover[HO_WDT_COUNT], 1);
}

static void test_lost_handover_without_power_on(void) {
  set_app_valid(1);
  lose_handover();
  CHECK_EQ(boot_reason(0, 1), 0);
  CHECK_EQ(handover[HO_REQUEST], 0);
  CHECK_EQ(handover[HO_WDT_COUNT], 1);
}

static void test_request_consumed_once(void) {
  set_app_valid(1);
  boot_reason(1, 0);
  handover[HO_REQUEST] = HO_REQUEST_UPDATE;
  CHECK_EQ(boot_reason(0, 0), BOOT_REASON_REQUEST);
  CHECK_EQ(handover[HO_REQUEST], 0);
  CHECK_EQ(boot_reason(0, 0), 0);
}

static void test_invalid_app(void) {
  set_app_valid(0);
  CHECK_EQ(boot_reason(1, 0), BOOT_REASON_INVALID);
  handover[HO_REQUEST] = HO_REQUEST_UPDATE;
  CHECK_EQ(boot_reason(0, 0), BOOT_REASON_REQUEST);
}

int main(void) {
  RUN(test_power_on);
  RUN(test_power_on_clears_count_and_request);
  RUN(test_watchdog_resets_in_a_row);
  RUN(test_watchdog_count_restarts_when_cleared);
  RUN(test_lost_handover_without_power_on);
  RUN(test_request_consumed_once);
  RUN(test_invalid_app);
  TEST_EXIT();
}
