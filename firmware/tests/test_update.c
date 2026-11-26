#include <string.h>

#include "crc16.h"
#include "flashmap.h"
#include "protocol.h"
#include "test.h"
#include "update.h"

/* Fake flash: erase sets a sector to 0xFF, write can only clear bits, as on the chip. */
static uint8_t flash[APP_END];
static int ops_left = -1; /* simulated power loss after this many erase/write operations */
static int powered = 1;
/* The power loss interrupts an erase, which then leaves the first half of the sector erased and
   the rest (the trailer, in the last sector) as it was. Otherwise an interrupted erase does
   nothing. */
static int partial_erase;
static int cut_now;

static int op(void) {
  cut_now = 0;
  if (!powered)
    return 0;
  if (ops_left == 0) {
    powered = 0;
    cut_now = 1;
    return 0;
  }
  if (ops_left > 0)
    ops_left--;
  return 1;
}

uint8_t flash_erase(uint16_t addr) {
  uint8_t *sector = flash + (addr & ~(SECTOR_SIZE - 1));
  if (!in_app_area(addr, 1))
    return 0;
  if (!op()) {
    if (cut_now && partial_erase)
      memset(sector, 0xFF, SECTOR_SIZE / 2);
    return 0;
  }
  memset(sector, 0xFF, SECTOR_SIZE);
  return 1;
}

uint8_t flash_write(uint16_t addr, const uint8_t *data, uint8_t n) {
  uint8_t i;
  if (!in_app_area(addr, n) || !op())
    return 0;
  for (i = 0; i < n; i++)
    flash[addr + i] &= data[i];
  return 1;
}

uint8_t flash_read(uint16_t addr) { return flash[addr]; }
void wdt_kick(void) {}

static uint8_t image[APP_SIZE];
static upd_t u;
static uint8_t frame[BOOT_WRITE_LEN];

static uint16_t image_len = 3000;

static void make_image(uint8_t seed) {
  uint16_t len = image_len, crc = CRC16_INIT, i;
  uint8_t *tr = image + APP_SIZE - TRAILER_SIZE;
  memset(image, 0xFF, sizeof image);
  for (i = 0; i < len; i++) {
    image[i] = (uint8_t) (i * 7 + seed);
    crc = crc16_update(crc, image[i]);
  }
  memcpy(tr + TR_MAGIC, APP_MAGIC, sizeof APP_MAGIC - 1);
  tr[TR_CODE_LEN] = (uint8_t) len;
  tr[TR_CODE_LEN + 1] = len >> 8;
  tr[TR_CRC] = (uint8_t) crc;
  tr[TR_CRC + 1] = crc >> 8;
  tr[TR_VERSION] = seed;
  tr[TR_VALID] = 0xFF;
}

static uint8_t send_begin(uint16_t size) {
  frame[0] = (uint8_t) size;
  frame[1] = size >> 8;
  upd_handle(&u, MSG_BOOT_BEGIN, frame, BOOT_BEGIN_LEN);
  return u.result;
}

static uint8_t send_write(uint16_t off) {
  frame[0] = (uint8_t) off;
  frame[1] = off >> 8;
  memcpy(frame + 2, image + off, WRITE_CHUNK);
  upd_handle(&u, MSG_BOOT_WRITE, frame, sizeof frame);
  return u.result;
}

static uint8_t send_finish(void) {
  upd_handle(&u, MSG_BOOT_FINISH, frame, 0);
  return u.result;
}

static int full_update(void) {
  uint16_t off;
  if (send_begin(APP_SIZE) != BR_OK)
    return 0;
  for (off = 0; off < APP_SIZE; off += WRITE_CHUNK)
    if (send_write(off) != BR_OK)
      return 0;
  return send_finish() == BR_OK;
}

static void reset_flash(void) {
  memset(flash, 0xFF, sizeof flash);
  ops_left = -1;
  powered = 1;
  partial_erase = 0;
  upd_init(&u);
}

static void test_blank_is_invalid(void) {
  reset_flash();
  CHECK(!app_valid());
}

static void test_normal_update(void) {
  reset_flash();
  make_image(1);
  CHECK(full_update());
  CHECK(app_valid());
  CHECK(memcmp(flash + APP_BASE, image, APP_SIZE - 1) == 0);
  CHECK_EQ(flash[APP_END - 1], 0x00);
  CHECK_EQ(upd_handle(&u, MSG_BOOT_RUN, frame, 0), UA_RUN);
}

static void test_begin_invalidates_old_app(void) {
  reset_flash();
  make_image(1);
  CHECK(full_update());
  CHECK_EQ(send_begin(APP_SIZE), BR_OK);
  CHECK(!app_valid());
  CHECK_EQ(upd_handle(&u, MSG_BOOT_RUN, frame, 0), UA_NONE);
  CHECK_EQ(u.result, BR_BAD_STATE);
}

static void test_wrong_size(void) {
  reset_flash();
  CHECK_EQ(send_begin(1234), BR_BAD_SIZE);
  CHECK_EQ(u.receiving, 0);
  upd_handle(&u, MSG_BOOT_BEGIN, frame, BOOT_BEGIN_LEN + 1);
  CHECK_EQ(u.result, BR_BAD_FRAME);
  CHECK_EQ(u.receiving, 0);
}

static void test_never_receiving_with_a_valid_app(void) {
  uint16_t off;
  reset_flash();
  make_image(1);
  CHECK(full_update());
  CHECK(app_valid());
  make_image(2);
  CHECK_EQ(send_begin(APP_SIZE), BR_OK);
  CHECK(u.receiving && !app_valid());
  for (off = 0; off < APP_SIZE; off += WRITE_CHUNK) {
    CHECK_EQ(send_write(off), BR_OK);
    CHECK(u.receiving && !app_valid());
  }
  CHECK_EQ(send_finish(), BR_OK);
  CHECK(!u.receiving && app_valid());
}

static void test_write_rules(void) {
  reset_flash();
  make_image(2);
  CHECK_EQ(send_write(0), BR_BAD_STATE); /* before begin */
  send_begin(APP_SIZE);
  CHECK_EQ(send_write(WRITE_CHUNK), BR_BAD_OFFSET); /* gap */
  CHECK_EQ(send_write(5), BR_BAD_OFFSET);           /* not aligned */
  CHECK_EQ(send_write(0), BR_OK);
  CHECK_EQ(send_write(0), BR_OK); /* repeat after a lost reply */
  image[0] ^= 1;
  CHECK_EQ(send_write(0), BR_MISMATCH);
  image[0] ^= 1;
  CHECK_EQ(send_write(APP_SIZE), BR_BAD_OFFSET); /* past the end */
  upd_handle(&u, MSG_BOOT_WRITE, frame, 10);
  CHECK_EQ(u.result, BR_BAD_FRAME);
  CHECK_EQ(send_finish(), BR_BAD_STATE); /* incomplete */
}

static uint8_t send_bad_length(uint8_t type, uint8_t len) {
  CHECK_EQ(upd_handle(&u, type, frame, len), UA_NONE);
  return u.result;
}

static void test_length_checked_first(void) {
  uint16_t off;
  reset_flash();
  make_image(1);
  CHECK(full_update());
  CHECK_EQ(send_bad_length(MSG_BOOT_WRITE, BOOT_WRITE_LEN - 1), BR_BAD_FRAME);
  CHECK_EQ(send_bad_length(MSG_BOOT_FINISH, 1), BR_BAD_FRAME);
  CHECK_EQ(send_bad_length(MSG_BOOT_RUN, 1), BR_BAD_FRAME);
  CHECK_EQ(send_bad_length(MSG_BOOT_RUN + 1, 0), BR_BAD_FRAME);
  CHECK(app_valid());

  make_image(2);
  CHECK_EQ(send_begin(APP_SIZE), BR_OK);
  CHECK_EQ(send_write(0), BR_OK);
  CHECK_EQ(send_bad_length(MSG_BOOT_BEGIN, BOOT_BEGIN_LEN + 1), BR_BAD_FRAME);
  CHECK_EQ(send_bad_length(MSG_BOOT_WRITE, BOOT_WRITE_LEN + 1), BR_BAD_FRAME);
  CHECK(u.receiving);
  CHECK_EQ(u.next, WRITE_CHUNK);
  for (off = WRITE_CHUNK; off < APP_SIZE; off += WRITE_CHUNK)
    CHECK_EQ(send_write(off), BR_OK);
  CHECK_EQ(send_finish(), BR_OK);
  CHECK(app_valid());
}

static void test_valid_mark_must_be_blank(void) {
  reset_flash();
  make_image(3);
  image[APP_SIZE - 1] = 0x00;
  CHECK(!full_update());
  CHECK_EQ(u.result, BR_BAD_IMAGE);
  CHECK(!app_valid());
}

static void test_bad_crc(void) {
  uint16_t off;
  reset_flash();
  make_image(4);
  send_begin(APP_SIZE);
  for (off = 0; off < APP_SIZE; off += WRITE_CHUNK) {
    if (off == 1024)
      image[1030] ^= 0x40; /* corrupted in transit, after the frame CRC */
    send_write(off);
  }
  CHECK_EQ(send_finish(), BR_BAD_IMAGE);
  CHECK(!app_valid());
}

static void test_bad_magic(void) {
  reset_flash();
  make_image(5);
  image[APP_SIZE - TRAILER_SIZE] = 'X';
  CHECK(!full_update());
  CHECK_EQ(u.result, BR_BAD_IMAGE);
}

static void test_power_loss_everywhere(void) {
  /* Power fails after every possible number of flash operations. The application is then
     either the complete old one, the complete new one, or invalid; never a mix. */
  int total, n;
  /* Count the operations of one update. */
  reset_flash();
  make_image(7);
  CHECK(full_update());
  ops_left = 1 << 30;
  make_image(6);
  upd_init(&u);
  CHECK(full_update());
  total = (1 << 30) - ops_left;

  for (n = 0; n <= total; n++) {
    reset_flash();
    make_image(7);
    CHECK(full_update()); /* old application */
    make_image(6);
    upd_init(&u); /* the bootloader restarts */
    ops_left = n;
    full_update();
    powered = 1;
    ops_left = -1;
    if (app_valid()) {
      uint8_t v = flash[TRAILER_ADDR + TR_VERSION];
      CHECK(v == 6 || v == 7);
      if (v == 6)
        CHECK(memcmp(flash + APP_BASE, image, APP_SIZE - 1) == 0);
    }
    /* A retry always succeeds. */
    upd_init(&u);
    CHECK(full_update());
    CHECK(app_valid());
  }
}

static void test_power_loss_during_first_erase(void) {
  /* An application that fills the last sector. An erase of the trailer's sector cut short must
     not leave the old trailer valid over partly erased code. */
  int n;
  uint8_t old[APP_SIZE];
  image_len = APP_SIZE - TRAILER_SIZE;
  for (n = 0; n < 8; n++) {
    reset_flash();
    make_image(7);
    memcpy(old, image, sizeof old);
    CHECK(full_update());
    make_image(6);
    upd_init(&u);
    partial_erase = 1;
    ops_left = n;
    full_update();
    powered = 1;
    ops_left = -1;
    partial_erase = 0;
    if (app_valid())
      CHECK(memcmp(flash + APP_BASE, old, APP_SIZE - 1) == 0);
    upd_init(&u);
    CHECK(full_update());
    CHECK(app_valid());
  }
  image_len = 3000;
}

static void test_in_app_area(void) {
  CHECK(in_app_area(APP_BASE, 1));
  CHECK(in_app_area(APP_BASE, 255));
  CHECK(in_app_area(APP_END - 1, 1));
  CHECK(in_app_area(APP_END - WRITE_CHUNK, WRITE_CHUNK));
  CHECK(!in_app_area(APP_BASE - 1, 1));
  CHECK(!in_app_area(APP_BASE - 1, 2));
  CHECK(!in_app_area(APP_END, 1));
  CHECK(!in_app_area(APP_END - 1, 2));
  CHECK(!in_app_area(0xFFFF, 255));
  CHECK(!in_app_area(0, 255));
}

static void test_status(void) {
  uint8_t out[BOOT_STATUS_LEN];
  reset_flash();
  make_image(1);
  /* A fresh bootloader differs from one that has just finished Begin. */
  upd_status(&u, BOOT_REASON_INVALID, out);
  CHECK_EQ(out[3], BR_NO_COMMAND);
  CHECK_EQ(out[4] | (out[5] << 8), 0);
  send_begin(APP_SIZE);
  upd_status(&u, BOOT_REASON_INVALID, out);
  CHECK_EQ(out[3], BR_OK);
  CHECK_EQ(out[4] | (out[5] << 8), 0);
  send_write(0);
  upd_status(&u, BOOT_REASON_REQUEST, out);
  CHECK_EQ(out[0], BOOT_VERSION);
  CHECK_EQ(out[1], 0);
  CHECK_EQ(out[2], BOOT_REASON_REQUEST);
  CHECK_EQ(out[3], BR_OK);
  CHECK_EQ(out[4] | (out[5] << 8), WRITE_CHUNK);
}

int main(void) {
  RUN(test_blank_is_invalid);
  RUN(test_normal_update);
  RUN(test_begin_invalidates_old_app);
  RUN(test_wrong_size);
  RUN(test_never_receiving_with_a_valid_app);
  RUN(test_write_rules);
  RUN(test_length_checked_first);
  RUN(test_valid_mark_must_be_blank);
  RUN(test_bad_crc);
  RUN(test_bad_magic);
  RUN(test_power_loss_everywhere);
  RUN(test_power_loss_during_first_erase);
  RUN(test_in_app_area);
  RUN(test_status);
  TEST_EXIT();
}
