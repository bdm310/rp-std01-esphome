#include <string.h>

#include "board.h"
#include "crc16.h"
#include "flashmap.h"
#include "link.h"
#include "protocol.h"
#include "test.h"

static uint8_t buf[APP_MAX_PAYLOAD];
static uint8_t boot_buf[BOOT_MAX_PAYLOAD];
static link_rx_t rx;

static void app_rx(void) {
  link_rx_init(&rx, buf, APP_MAX_PAYLOAD);
  link_rx_types(&rx, MSG_GATE, MSG_UPDATE);
}
static void boot_rx(void) {
  link_rx_init(&rx, boot_buf, BOOT_MAX_PAYLOAD);
  link_rx_types(&rx, MSG_BOOT_BEGIN, MSG_BOOT_RUN);
}

static int feed(const uint8_t *data, int n) {
  int frames = 0, i;
  for (i = 0; i < n; i++)
    frames += link_rx_byte(&rx, data[i]);
  return frames;
}

static void test_crc_check_value(void) {
  const char *s = "123456789";
  uint16_t crc = CRC16_INIT;
  while (*s)
    crc = crc16_update(crc, (uint8_t) *s++);
  CHECK_EQ(crc, 0x29B1);
}

static void test_round_trip(void) {
  uint8_t payload[4] = {0x34, 0x12, 16, 23};
  uint8_t frame[32];
  uint8_t n = link_build(frame, MSG_GATE, payload, 4);
  CHECK_EQ(n, 9);
  CHECK_EQ(frame[0], FRAME_START);
  app_rx();
  CHECK_EQ(feed(frame, n), 1);
  CHECK_EQ(rx.type, MSG_GATE);
  CHECK_EQ(rx.len, 4);
  CHECK(memcmp(buf, payload, 4) == 0);
}

/* The bootloader's receiver: a write frame at its limit, then one byte over it. */
static void test_boot_limit(void) {
  uint8_t payload[BOOT_MAX_PAYLOAD + 1];
  uint8_t frame[BOOT_MAX_PAYLOAD + 1 + FRAME_OVERHEAD];
  uint8_t n, i;
  for (i = 0; i < sizeof payload; i++)
    payload[i] = (uint8_t) (i * 37 + 0xA5);
  boot_rx();
  n = link_build(frame, MSG_BOOT_WRITE, payload, BOOT_MAX_PAYLOAD);
  CHECK_EQ(feed(frame, n), 1);
  CHECK_EQ(rx.len, BOOT_MAX_PAYLOAD);
  CHECK(memcmp(boot_buf, payload, BOOT_MAX_PAYLOAD) == 0);
  n = link_build(frame, MSG_BOOT_WRITE, payload, BOOT_MAX_PAYLOAD + 1);
  CHECK_EQ(feed(frame, n), 0);
}

static void test_empty_payload(void) {
  uint8_t frame[8];
  uint8_t n = link_build(frame, MSG_INFO_REQ, NULL, 0);
  app_rx();
  CHECK_EQ(n, 5);
  CHECK_EQ(feed(frame, n), 1);
  CHECK_EQ(rx.len, 0);
}

static void test_noise_and_joined_frames(void) {
  uint8_t payload[2] = {0x3F, 0x80};
  uint8_t stream[64];
  int n = 0;
  stream[n++] = 0x00;
  stream[n++] = 0x13;
  n += link_build(stream + n, MSG_OUTPUTS, payload, 2);
  n += link_build(stream + n, MSG_INFO_REQ, NULL, 0);
  app_rx();
  CHECK_EQ(feed(stream, n), 2);
  CHECK_EQ(rx.type, MSG_INFO_REQ);
}

static void test_bad_crc(void) {
  uint8_t payload[2] = {1, 2};
  uint8_t frame[16];
  uint8_t n = link_build(frame, MSG_OUTPUTS, payload, 2);
  app_rx();
  frame[4] ^= 0x01;
  CHECK_EQ(feed(frame, n), 0);
  /* The parser recovers for the next good frame. */
  frame[4] ^= 0x01;
  CHECK_EQ(feed(frame, n), 1);
}

static void test_too_long(void) {
  uint8_t frame[3] = {FRAME_START, MSG_GATE, APP_MAX_PAYLOAD + 1};
  uint8_t payload[4] = {0, 0, 0, 0};
  uint8_t good[16];
  uint8_t n = link_build(good, MSG_GATE, payload, 4);
  app_rx();
  CHECK_EQ(feed(frame, 3), 0);
  CHECK_EQ(feed(good, n), 1);
}

static void test_split_and_reset(void) {
  uint8_t payload[4] = {9, 8, 7, 6};
  uint8_t frame[16];
  uint8_t n = link_build(frame, MSG_GATE, payload, 4);
  app_rx();
  CHECK_EQ(feed(frame, 5), 0);
  link_rx_drop_partial(&rx); /* timeout in the middle of a frame */
  CHECK_EQ(feed(frame + 5, n - 5), 0);
  CHECK_EQ(feed(frame, n), 1);
}

/* A bad frame whose last byte is a start byte: the next frame starts there. */
static void test_bad_frame_ends_in_start(void) {
  uint8_t frame[16];
  uint8_t n = link_build(frame, MSG_INFO_REQ, NULL, 0);
  uint8_t junk[3] = {FRAME_START, MSG_GATE, 0xA5}; /* length 0xA5 is too long */
  app_rx();
  CHECK_EQ(feed(junk, 3), 0);
  CHECK_EQ(feed(frame + 1, n - 1), 1);
}

/* A false start inside a payload swallows the frames it overlaps; the frame after it arrives. */
static void test_false_start_in_payload(void) {
  uint8_t lost[4] = {0xA5, MSG_OUTPUTS, 0x02, 0x17}; /* false type MSG_OUTPUTS, length 2 */
  uint8_t gate[4] = {0x34, 0x12, 16, 23};
  uint8_t stream[64];
  int n = 0;
  n += link_build(stream + n, MSG_GATE, lost, 4);
  n += link_build(stream + n, MSG_GATE, gate, 4);
  app_rx();
  CHECK_EQ(feed(stream + 1, n - 1), 1);
  CHECK_EQ(rx.type, MSG_GATE);
  CHECK(memcmp(buf, gate, 4) == 0);
}

/* Frame number id of a random stream: its type, length and payload, with 0xA5 for every third
   payload byte. Byte 0 is the number itself. */
static uint8_t rnd_type(uint8_t id) { return (uint8_t) (MSG_GATE + id % (MSG_UPDATE - MSG_GATE + 1)); }
static uint8_t rnd_len(uint8_t id) { return (uint8_t) (1 + (id * 7u + 3u) % APP_MAX_PAYLOAD); }
static uint8_t rnd_byte(uint8_t id, int i) {
  if (i == 0)
    return id;
  return (id + i) % 3 ? (uint8_t) (id * 31 + i * 17) : 0xA5;
}

/* Good frames with random noise between them, many 0xA5 bytes in payloads and noise: no frame is
   made up or garbled, and most arrive. */
static void test_random_noise(void) {
  static uint8_t stream[4096];
  uint8_t payload[APP_MAX_PAYLOAD], len, id;
  uint32_t seed = 1;
  int round, n, frames, received, i, k, last;
  for (round = 0; round < 300; round++) {
    n = 0;
    for (frames = 0; frames < 150; frames++) {
      seed = seed * 1103515245u + 12345u;
      for (i = (seed >> 16) % 4; i > 0; i--) {
        seed = seed * 1103515245u + 12345u;
        stream[n++] = (seed >> 16) & 1 ? FRAME_START : (uint8_t) (seed >> 20);
      }
      id = (uint8_t) frames;
      len = rnd_len(id);
      for (k = 0; k < len; k++)
        payload[k] = rnd_byte(id, k);
      n += link_build(stream + n, rnd_type(id), payload, len);
    }
    app_rx();
    received = 0;
    last = -1;
    for (i = 0; i < n; i++) {
      if (!link_rx_byte(&rx, stream[i]))
        continue;
      id = buf[0];
      CHECK(id > last);
      last = id;
      CHECK_EQ(rx.type, rnd_type(id));
      CHECK_EQ(rx.len, rnd_len(id));
      for (k = 0; k < rx.len; k++)
        CHECK_EQ(buf[k], rnd_byte(id, k));
      received++;
    }
    CHECK(received > frames / 2);
  }
}

/* A start byte followed by an accepted type anywhere but at the frame's own start. */
static int has_false_start(const uint8_t *frame, uint8_t n, uint8_t first, uint8_t last) {
  uint8_t i;
  for (i = 1; i + 1 < n; i++)
    if (frame[i] == FRAME_START && frame[i + 1] >= first && frame[i + 1] <= last)
      return 1;
  return 0;
}

/*
 * Identical frames back to back with no gap, so the receive timeout never fires, after a desync at
 * every byte of the frame. The desync is each state the parser can be in there: searching, a false
 * start, a false start with an accepted type, or that plus any accepted length (a false frame that
 * takes the next bytes as its payload and CRC). Such a false frame ends at most max + 3 bytes into
 * the stream, and every frame that starts after that must arrive.
 */
static void resync_back_to_back(void (*setup)(void), uint8_t first, uint8_t last, const uint8_t *frame,
                                uint8_t n, uint8_t max) {
  static uint8_t stream[2048];
  uint8_t got[64];
  uint8_t desync[3] = {FRAME_START, first, 0};
  int false_frame_end = max + 3, frames = false_frame_end / n + 8;
  int offset, state, i, j, lost;
  CHECK(!has_false_start(frame, n, first, last));
  for (i = 0; i < frames; i++)
    memcpy(stream + i * n, frame, n);
  for (offset = 0; offset < n; offset++) {
    for (state = 0; state <= 3 + max; state++) {
      setup();
      desync[2] = (uint8_t) (state >= 3 ? state - 3 : 0);
      feed(desync, state < 3 ? state : 3);
      memset(got, 0, sizeof got);
      for (i = offset; i < frames * n; i++) {
        if (!link_rx_byte(&rx, stream[i]))
          continue;
        CHECK(rx.type == frame[1] && rx.len == frame[2] && memcmp(rx.buf, frame + 3, rx.len) == 0);
        got[i / n] = 1;
      }
      lost = 0;
      for (j = offset ? 1 : 0; j < frames; j++) {
        if (got[j])
          continue;
        lost++;
        CHECK(j * n - offset < false_frame_end);
      }
      CHECK(lost <= (false_frame_end + n - 1) / n);
    }
  }
}

/* A frame of the type whose payload starts with a 16-bit value chosen so the CRC's high byte, the
   frame's last byte, is a start byte. */
static uint8_t crc_ends_in_start(uint8_t *frame, uint8_t type, uint8_t *payload, uint8_t len, uint8_t first,
                                 uint8_t last) {
  uint32_t v;
  uint8_t n = 0;
  for (v = 0; v <= 0xFFFF; v++) {
    payload[0] = (uint8_t) v;
    payload[1] = (uint8_t) (v >> 8);
    n = link_build(frame, type, payload, len);
    if (frame[n - 1] == FRAME_START && !has_false_start(frame, n, first, last))
      break;
  }
  CHECK(v <= 0xFFFF);
  return n;
}

static void test_resync_app(void) {
  /* Gate target 256 with limits 220 and 0 ends in a start byte. Without the type check that byte
     is taken as a start and the next frame's start as its type, and no frame arrives again. */
  const uint8_t example[9] = {0xA5, 0x01, 0x04, 0x00, 0x01, 0xDC, 0x00, 0xCC, 0xA5};
  uint8_t gate[4] = {0x00, 0x01, 220, 0};
  uint8_t outputs[2] = {0x3F, 0x80};
  uint8_t update[UPDATE_LEN] = {UPDATE_KEY0, UPDATE_KEY1, UPDATE_KEY2, UPDATE_KEY3};
  uint8_t frame[16], n;

  n = link_build(frame, MSG_GATE, gate, 4);
  CHECK(n == sizeof example && memcmp(frame, example, n) == 0);
  resync_back_to_back(app_rx, MSG_GATE, MSG_UPDATE, example, sizeof example, APP_MAX_PAYLOAD);

  gate[2] = LIMIT_BRIGHTER;
  gate[3] = LIMIT_DIMMER;
  n = crc_ends_in_start(frame, MSG_GATE, gate, 4, MSG_GATE, MSG_UPDATE);
  resync_back_to_back(app_rx, MSG_GATE, MSG_UPDATE, frame, n, APP_MAX_PAYLOAD);
  n = link_build(frame, MSG_GATE, gate, 4);
  resync_back_to_back(app_rx, MSG_GATE, MSG_UPDATE, frame, n, APP_MAX_PAYLOAD);
  n = crc_ends_in_start(frame, MSG_OUTPUTS, outputs, 2, MSG_GATE, MSG_UPDATE);
  resync_back_to_back(app_rx, MSG_GATE, MSG_UPDATE, frame, n, APP_MAX_PAYLOAD);
  n = link_build(frame, MSG_INFO_REQ, NULL, 0);
  resync_back_to_back(app_rx, MSG_GATE, MSG_UPDATE, frame, n, APP_MAX_PAYLOAD);
  n = link_build(frame, MSG_UPDATE, update, 4);
  resync_back_to_back(app_rx, MSG_GATE, MSG_UPDATE, frame, n, APP_MAX_PAYLOAD);
}

static void test_resync_boot(void) {
  static uint8_t write[WRITE_CHUNK + 2];
  uint8_t begin[2] = {(uint8_t) APP_SIZE, (uint8_t) (APP_SIZE >> 8)};
  static uint8_t frame[BOOT_MAX_PAYLOAD + FRAME_OVERHEAD];
  uint8_t n;

  memset(write, 0xFF, sizeof write);
  n = crc_ends_in_start(frame, MSG_BOOT_WRITE, write, sizeof write, MSG_BOOT_BEGIN, MSG_BOOT_RUN);
  resync_back_to_back(boot_rx, MSG_BOOT_BEGIN, MSG_BOOT_RUN, frame, n, BOOT_MAX_PAYLOAD);
  n = link_build(frame, MSG_BOOT_BEGIN, begin, 2);
  resync_back_to_back(boot_rx, MSG_BOOT_BEGIN, MSG_BOOT_RUN, frame, n, BOOT_MAX_PAYLOAD);
  n = link_build(frame, MSG_BOOT_FINISH, NULL, 0);
  resync_back_to_back(boot_rx, MSG_BOOT_BEGIN, MSG_BOOT_RUN, frame, n, BOOT_MAX_PAYLOAD);
  n = link_build(frame, MSG_BOOT_RUN, NULL, 0);
  resync_back_to_back(boot_rx, MSG_BOOT_BEGIN, MSG_BOOT_RUN, frame, n, BOOT_MAX_PAYLOAD);
}

/*
 * The type check cannot help when a start byte inside the frame is followed by an accepted type:
 * gate target 0x02A5 with limit brighter 3 makes a false frame that ends inside the next frame,
 * just before that frame's own false start, so back to back with no gap no frame arrives. The
 * receive timeout covers this: the ESP32 writes at most one frame per loop, and the gap drops the
 * false frame before the next one comes.
 */
static void test_gap_ends_false_frame(void) {
  uint8_t gate[4] = {0xA5, 0x02, 3, LIMIT_DIMMER};
  uint8_t frame[16];
  uint8_t n = link_build(frame, MSG_GATE, gate, 4);
  int i, got = 0;
  app_rx();
  for (i = 0; i < 10; i++) {
    got += feed(frame + (i ? 0 : 1), n - (i ? 0 : 1));
    link_rx_drop_partial(&rx);
  }
  CHECK_EQ(got, 9);
}

int main(void) {
  RUN(test_crc_check_value);
  RUN(test_round_trip);
  RUN(test_boot_limit);
  RUN(test_empty_payload);
  RUN(test_noise_and_joined_frames);
  RUN(test_bad_crc);
  RUN(test_too_long);
  RUN(test_split_and_reset);
  RUN(test_bad_frame_ends_in_start);
  RUN(test_false_start_in_payload);
  RUN(test_random_noise);
  RUN(test_resync_app);
  RUN(test_resync_boot);
  RUN(test_gap_ends_false_frame);
  TEST_EXIT();
}
