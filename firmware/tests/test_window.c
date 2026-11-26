#include "protocol.h"
#include "status.h"
#include "test.h"
#include "window.h"

static uint16_t get16(const uint8_t *p) { return p[0] | (p[1] << 8); }

static void test_ks_and_relay(void) {
  window_t w;
  status_in_t s = {0};
  uint8_t out[STATUS_LEN];
  window_reset(&w);
  window_add_ks(&w, 700, 1);
  window_add_ks(&w, 900, 1);
  window_add_ks(&w, 800, 0);
  status_build(out, &s, &w);
  CHECK_EQ(get16(out + 9), 700);
  CHECK_EQ(out[11] & S_KS_STALE, S_KS_STALE);
}

/* The relay switched after the window's last sample: the samples do not show the current drive. */
static void test_relay_switched_after_samples(void) {
  window_t w;
  status_in_t s = {0};
  uint8_t out[STATUS_LEN];
  window_reset(&w);
  window_add_ks(&w, 700, 1);
  window_add_ks(&w, 710, 1);
  s.relay = 0;
  status_build(out, &s, &w);
  CHECK_EQ(out[11], S_KS_STALE);
  s.relay = 1;
  status_build(out, &s, &w);
  CHECK_EQ(out[11], S_RELAY);
  window_reset(&w);
  window_add_ks(&w, 900, 0);
  status_build(out, &s, &w);
  CHECK_EQ(out[11], S_RELAY | S_KS_STALE);
  s.relay = 0;
  status_build(out, &s, &w);
  CHECK_EQ(out[11], 0);
}

static void test_empty_window(void) {
  window_t w;
  status_in_t s = {0};
  uint8_t out[STATUS_LEN];
  window_reset(&w);
  window_add_buttons(&w, 0);
  status_build(out, &s, &w);
  CHECK_EQ(get16(out + 9), 0xFFFF);
  CHECK_EQ(out[11], S_KS_STALE);
}

static void test_buttons(void) {
  window_t w;
  status_in_t s = {0};
  uint8_t out[STATUS_LEN];
  window_reset(&w);
  window_add_buttons(&w, BTN_ALL);
  window_add_buttons(&w, BTN_ALL & ~BTN_ONOFF);
  window_add_buttons(&w, BTN_ALL);
  status_build(out, &s, &w);
  CHECK_EQ(out[2] & BTN_ALL, BTN_ALL);
  CHECK_EQ(out[2] >> BTN_CHANGED_SHIFT, BTN_ONOFF);
}

static void test_fields(void) {
  window_t w;
  status_in_t s = {2, 0x10, 4000, 3990, 7680, 1, 512, 300, 0, 0x5C, 0x21, 200};
  uint8_t out[STATUS_LEN];
  window_reset(&w);
  window_add_ks(&w, 1, 1);
  window_add_buttons(&w, 0);
  status_build(out, &s, &w);
  CHECK_EQ(out[0], 2);
  CHECK_EQ(out[1], 0x10);
  CHECK_EQ(out[2], 0);
  CHECK_EQ(get16(out + 3), 4000);
  CHECK_EQ(get16(out + 5), 3990);
  CHECK_EQ(get16(out + 7), 7680);
  CHECK_EQ(get16(out + 9), 1);
  CHECK_EQ(out[11], 1);
  CHECK_EQ(get16(out + 12), 512);
  CHECK_EQ(get16(out + 14), 300);
  CHECK_EQ(out[16], 0x5C);
  CHECK_EQ(out[17], 0x21);
  CHECK_EQ(out[18], 200);
}

static void test_starting_boot_id_and_outputs(void) {
  window_t w;
  status_in_t s = {0};
  uint8_t out[STATUS_LEN];
  CHECK_EQ(STATUS_LEN, 19);
  window_reset(&w);
  window_add_ks(&w, 1, 1);
  s.relay = 1;
  s.starting = 1;
  s.boot_id = 0xC7;
  s.leds = 0x21;
  s.night = 200;
  status_build(out, &s, &w);
  CHECK_EQ(out[11], S_RELAY | S_STARTING);
  CHECK_EQ(out[16], 0xC7);
  CHECK_EQ(out[17], 0x21);
  CHECK_EQ(out[18], 200);
  s.starting = 0;
  s.relay = 0;
  status_build(out, &s, &w);
  CHECK_EQ(out[11], S_KS_STALE); /* the samples were taken with the drive on */
}

static void test_info(void) {
  uint8_t out[INFO_LEN];
  CHECK_EQ(INFO_LEN, 10);
  info_build(out, 0x0102, 0xA1B2C3D4ul, 2, 5, 7, 0x9E);
  CHECK_EQ(get16(out), 0x0102);
  CHECK_EQ(get16(out + 2), 0xC3D4);
  CHECK_EQ(get16(out + 4), 0xA1B2);
  CHECK_EQ(out[6], 2);
  CHECK_EQ(out[7], 5);
  CHECK_EQ(out[8], 7);
  CHECK_EQ(out[9], 0x9E);
}

static void test_avg8(void) {
  avg8_t a;
  int i;
  avg8_init(&a);
  for (i = 0; i < 7; i++)
    avg8_add(&a, 100);
  CHECK_EQ(a.ready, 0);
  avg8_add(&a, 108);
  CHECK_EQ(a.ready, 1);
  CHECK_EQ(a.value, 101);
}

int main(void) {
  RUN(test_ks_and_relay);
  RUN(test_relay_switched_after_samples);
  RUN(test_empty_window);
  RUN(test_buttons);
  RUN(test_fields);
  RUN(test_starting_boot_id_and_outputs);
  RUN(test_info);
  RUN(test_avg8);
  TEST_EXIT();
}
