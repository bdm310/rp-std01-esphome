// For the unit tests: frames built and parsed by the firmware's own link.c, a fake UART, and
// SimMcu, the simulated MCU of mcu_sim.h with wires to the ESP32 and fault injection. Include
// rpstd01_link.h (or a header that includes it) first.
#pragma once

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <utility>
#include <vector>

#include "esphome/components/rpstd01_tool/rpstd01_mcu_full_image.h"
#include "test.h"

// Firmware headers that do not include protocol.h, flashmap.h or board.h: their constants come from
// rpstd01_fw_constants.h, and the macros would clash with those names.
extern "C" {
#include "boot.h"
#include "crc16.h"
#include "link.h"
#include "mcu_sim.h"
}

using namespace rpstd01;

static uint32_t rng_state = 12345;
[[maybe_unused]] static uint32_t rnd() {
  rng_state = rng_state * 1103515245u + 12345u;
  return rng_state >> 8;
}

struct Frame {
  uint8_t type;
  std::vector<uint8_t> payload;
};

// Splits bytes into frames with the firmware's own parser, as the application does.
[[maybe_unused]] static std::vector<Frame> fw_parse(const std::vector<uint8_t> &bytes, uint8_t max = BOOT_MAX_PAYLOAD) {
  static uint8_t buf[256];
  link_rx_t rx;
  link_rx_init(&rx, buf, max);
  std::vector<Frame> out;
  for (uint8_t b : bytes)
    if (link_rx_byte(&rx, b))
      out.push_back({rx.type, std::vector<uint8_t>(buf, buf + rx.len)});
  return out;
}

[[maybe_unused]] static std::vector<uint8_t> fw_frame(uint8_t type, const uint8_t *payload, uint8_t len) {
  uint8_t out[300];
  uint8_t n = link_build(out, type, payload, len);
  return std::vector<uint8_t>(out, out + n);
}

// UART with a manual feed, for the protocol tests.
struct FakeUart {
  std::deque<uint8_t> rx;
  std::vector<uint8_t> tx;
  size_t available() { return rx.size(); }
  bool read_byte(uint8_t *b) {
    if (rx.empty())
      return false;
    *b = rx.front();
    rx.pop_front();
    return true;
  }
  void write_array(const uint8_t *d, size_t n) { tx.insert(tx.end(), d, d + n); }
  void push(const std::vector<uint8_t> &v) { rx.insert(rx.end(), v.begin(), v.end()); }
  std::vector<Frame> take_frames() {
    auto f = fw_parse(tx);
    tx.clear();
    return f;
  }
};

[[maybe_unused]] static std::vector<Frame> of_type(const std::vector<Frame> &frames, uint8_t type) {
  std::vector<Frame> out;
  for (auto &f : frames)
    if (f.type == type)
      out.push_back(f);
  return out;
}
[[maybe_unused]] static uint16_t g16(const std::vector<uint8_t> &p, size_t i) { return p[i] | (p[i + 1] << 8); }

static uint8_t *const flash = mcu_sim_flash;

// The application area of the committed MCU image as an update image (valid mark 0xFF).
static const std::vector<uint8_t> app_image = [] {
  std::vector<uint8_t> area(MCU_IMAGE + APP_BASE, MCU_IMAGE + APP_END);
  area[APP_SIZE - TRAILER_SIZE + TR_VALID] = 0xFF;
  return area;
}();

[[maybe_unused]] static uint32_t image_build_of(const uint8_t *img) {
  const uint8_t *t = img + APP_SIZE - TRAILER_SIZE + TR_BUILD;
  return t[0] | (t[1] << 8) | (t[2] << 16) | ((uint32_t) t[3] << 24);
}

// Installs app_image in the flash, marked valid, with the given build ID and release.
[[maybe_unused]] static void install(uint32_t build, uint16_t version = MCU_IMAGE_VERSION) {
  memset(flash, 0xFF, APP_END);
  memcpy(flash + APP_BASE, app_image.data(), APP_SIZE);
  uint8_t *t = flash + TRAILER_ADDR + TR_BUILD;
  t[0] = build;
  t[1] = build >> 8;
  t[2] = build >> 16;
  t[3] = build >> 24;
  flash[TRAILER_ADDR + TR_VERSION] = version;
  flash[TRAILER_ADDR + TR_VERSION + 1] = version >> 8;
  flash[TRAILER_ADDR + TR_VALID] = 0x00;
}

[[maybe_unused]] static bool flash_matches_image() {
  return memcmp(flash + APP_BASE, app_image.data(), APP_SIZE - 1) == 0 && flash[APP_END - 1] == 0x00;
}

// The simulated MCU with byte timing on both wires. The ESP32 side is esp_loop, called every
// ESP_LOOP_US with the time in ms, reading and writing uart. One at a time: it drives the one
// mcu_sim.
struct SimMcu {
  enum Mode { OFF = SIM_OFF, APP = SIM_APP, BOOT = SIM_BOOT };
  static constexpr uint64_t STEP_US = 100;
  static constexpr uint64_t ESP_LOOP_US = 16000;
  std::function<void(uint32_t)> esp_loop;
  uint64_t &us = mcu_sim.now_us;
  int &mode = mcu_sim.mode;
  gate_t &gate = app_gate;
  uint8_t &leds = app_status.leds;
  uint8_t &night = app_status.night;
  uint8_t &boot_id = mcu_sim.boot_id;
  int &aux_ram_survives_reset = mcu_sim.aux_ram_survives_reset;
  upd_t &upd = boot_upd;
  uint8_t &boot_reason = boot_update_reason;
  uint32_t &boot_bytes_dropped = mcu_sim.busy_dropped;

  struct EspUart {
    SimMcu *w;
    std::deque<uint8_t> rx;
    size_t available() { return rx.size(); }
    bool read_byte(uint8_t *b) {
      if (rx.empty())
        return false;
      *b = rx.front();
      rx.pop_front();
      return true;
    }
    void write_array(const uint8_t *d, size_t n) { w->esp_wrote(d, n); }
  } uart{this, {}};

  std::deque<std::pair<uint64_t, uint8_t>> to_esp_at;
  uint64_t to_mcu_free = 0;

  int drop_esp_type = -1, drop_esp_count = 0;  // drop whole ESP32 frames of this type
  int corrupt_write_at = -1;                   // flip a byte of this write (bad CRC)
  int tamper_write_at = -1, tamper_times = 0;  // flip a byte and fix the CRC
  int drop_boot_reply_after_write = -1;        // lose the bootloader's reply to this write
  int cut_power_at_write = -1;                 // power cut when this write arrives
  uint32_t cut_ms = 0;
  bool drop_next_boot_status = false;
  int boot_bad_frames = 0, writes_seen = 0, begins_seen = 0, update_reqs = 0;
  int esp_frames[256]{};

  static SimMcu *&current() {
    static SimMcu *w = nullptr;
    return w;
  }

  SimMcu() {
    current() = this;
    mcu_sim_init();
    mcu_sim.lost_ram_fixed = 1;  // the worst case: the boot ID repeats from one such start to the next
    mcu_sim.tx = [](const uint8_t *f, uint8_t n, uint64_t at) { current()->mcu_sent(f, n, at); };
    mcu_sim.frame_in = [](uint8_t type, const uint8_t *p, uint8_t len) { current()->mcu_got(type, p, len); };
  }
  ~SimMcu() {
    mcu_sim.tx = nullptr;
    mcu_sim.frame_in = nullptr;
    current() = nullptr;
  }

  void power_on() { mcu_sim_power_on(); }
  void watchdog_reset() { mcu_sim_reset(1); }
  void boot_start(uint8_t reason) { mcu_sim_update_mode(reason); }
  uint16_t app_version() const { return mcu_sim_app_version(); }

  void esp_wrote(const uint8_t *d, size_t n) {
    std::vector<uint8_t> f(d, d + n);
    uint8_t type = n > 1 ? d[1] : 0;
    esp_frames[type]++;
    if (type == drop_esp_type && drop_esp_count > 0) {
      drop_esp_count--;
      return;
    }
    if (type == MSG_BOOT_WRITE && n == FRAME_OVERHEAD + 2 + WRITE_CHUNK) {
      uint16_t off = d[3] | (d[4] << 8);
      if ((int) off == corrupt_write_at) {
        corrupt_write_at = -1;
        f[40] ^= 0x01;
      }
      if ((int) off == tamper_write_at && tamper_times > 0) {
        tamper_times--;
        f[40] ^= 0x01;
        build_frame(f.data(), f[1], f.data() + 3, f[2]);
      }
    }
    uint64_t t = std::max(us, to_mcu_free);
    for (uint8_t b : f) {
      t += MCU_SIM_BYTE_US;
      mcu_sim_rx(b, t);
    }
    to_mcu_free = t;
  }

  void mcu_got(uint8_t type, const uint8_t *p, uint8_t len) {
    if (mode == APP) {
      if (type == MSG_UPDATE && len == UPDATE_LEN && p[0] == UPDATE_KEY0 && p[1] == UPDATE_KEY1 && p[2] == UPDATE_KEY2 &&
          p[3] == UPDATE_KEY3)
        update_reqs++;
      return;
    }
    if (type < 0x10)
      boot_bad_frames++;
    if (type == MSG_BOOT_BEGIN)
      begins_seen++;
    if (type == MSG_BOOT_WRITE && len >= 2) {
      int off = p[0] | (p[1] << 8);
      writes_seen++;
      if (off == drop_boot_reply_after_write) {
        drop_boot_reply_after_write = -1;
        drop_next_boot_status = true;
      }
      if (off == cut_power_at_write) {
        cut_power_at_write = -1;
        mcu_sim_power_cut(cut_ms);
      }
    }
  }

  void mcu_sent(const uint8_t *f, uint8_t n, uint64_t at) {
    if (f[1] == MSG_BOOT_STATUS && drop_next_boot_status) {
      drop_next_boot_status = false;
      return;
    }
    for (uint8_t i = 0; i < n; i++)
      to_esp_at.push_back({at + (i + 1) * MCU_SIM_BYTE_US, f[i]});
  }

  void run(uint32_t ms) {
    uint64_t end = us + ms * 1000ull;
    while (us < end) {
      mcu_sim_run(us);
      while (!to_esp_at.empty() && to_esp_at.front().first <= us) {
        uart.rx.push_back(to_esp_at.front().second);
        to_esp_at.pop_front();
      }
      if (us % ESP_LOOP_US == 0 && esp_loop)
        esp_loop((uint32_t) (us / 1000));
      us += STEP_US;
    }
  }
};
