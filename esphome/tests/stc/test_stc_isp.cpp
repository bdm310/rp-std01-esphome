// A fake bootloader serves the responses recorded by gen_stc_vectors.py. The programmer must send
// exactly the packets stcgal sent, framed and checksummed as stcgal frames them.
// The fake bootloader only understands bytes sent at its own baud rate, so the tests also check
// where the disconnect lands when the reply to the baud rate switch is lost.

#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "esphome/components/rpstd01_tool/rpstd01_stc_isp.h"
#include "esphome/components/rpstd01_tool/rpstd01_tool.h"
#include "test.h"

using Bytes = std::vector<uint8_t>;
using rpstd01tool::MCU_MODEL;
using rpstd01tool::STC_TRANSFER_BAUD;
using rpstd01tool::MCU_USER_HZ;

static Bytes unhex(const std::string &s) {
  Bytes b;
  for (size_t i = 0; i + 1 < s.size(); i += 2)
    b.push_back((uint8_t) std::stoul(s.substr(i, 2), nullptr, 16));
  return b;
}

static Bytes frame_from_mcu(const Bytes &payload) {
  Bytes p = {0x46, 0xB9, 0x68, 0, 0};
  const size_t len = payload.size() + 8 - 2;  // the length field counts from 0x68 to the end
  p[3] = (uint8_t) (len >> 8);
  p[4] = (uint8_t) len;
  p.insert(p.end(), payload.begin(), payload.end());
  uint16_t sum = 0;
  for (size_t i = 2; i < p.size(); i++)
    sum += p[i];
  p.push_back((uint8_t) (sum >> 8));
  p.push_back((uint8_t) sum);
  p.push_back(0x16);
  return p;
}

struct Vectors {
  Bytes status;
  std::vector<Bytes> responses;
  std::vector<Bytes> writes;
};

class FakeBsl : public stcisp::Io {
 public:
  explicit FakeBsl(const Vectors &v) : v_(v) {}

  int corrupt_response = -1;   // flip a checksum bit in this response
  int silent_after = -1;       // stop answering after this many host packets
  int drop_response = -1;      // act on this host packet, but lose the reply
  int ignore_packet = -1;      // lose this host packet: no action, no reply
  bool reack = false;          // answer the first sync byte with 0x80, as some STC15A do
  Bytes noise;                 // sent after the first sync byte
  size_t answer_at_sync = 1;   // the status packet answers this sync byte
  std::vector<uint32_t> bauds;
  size_t packets = 0;
  size_t mismatches = 0;
  size_t sync_bytes = 0;
  size_t garbled = 0;          // bytes sent at a baud rate the bootloader was not using
  size_t misread = 0;          // bytes at the right rate, but sent while it was still busy
  bool got_disconnect = false;
  uint32_t disconnect_baud = 0;
  Bytes reack_raw;             // the host's re-acknowledge packet, as sent

  void set_baud(uint32_t baud) override {
    bauds.push_back(baud);
    host_baud_ = baud;
  }

  // Bytes sent at a rate the bootloader is not using are lost. A burst at a higher rate also
  // starts a byte in a bootloader at the handshake rate, which then receives for a whole character
  // time; a byte at the right rate that starts in that time is misread.
  void write(const uint8_t *data, size_t len) override {
    const uint32_t t0 = t_;
    t_ += len * 100;
    if (host_baud_ != bsl_baud_) {
      garbled += len;
      if (bsl_baud_ == stcisp::HANDSHAKE_BAUD && host_baud_ > bsl_baud_ && t0 >= busy_until_)
        busy_until_ = t0 + stcisp::HANDSHAKE_CHAR_US;
      return;
    }
    for (size_t i = 0; i < len; i++) {
      if (t0 + i * 100 < busy_until_) {
        misread++;
        byte_in((uint8_t) ~data[i]);
      } else {
        byte_in(data[i]);
      }
    }
  }
  bool available() override { return !rx_.empty(); }
  int read(uint32_t timeout_us) override {
    if (rx_.empty()) {
      t_ += timeout_us;
      return -1;
    }
    int c = rx_.front();
    rx_.pop_front();
    t_ += 10;
    return c;
  }
  uint32_t now_us() override { return t_ += 5; }
  void delay_us(uint32_t us) override { t_ += us; }

 private:
  void queue(const Bytes &payload, int index) {
    Bytes f = frame_from_mcu(payload);
    if (index == corrupt_response)
      f[f.size() - 2] ^= 1;
    rx_.insert(rx_.end(), f.begin(), f.end());
  }

  void byte_in(uint8_t c) {
    if (cur_.empty() && c != 0x46) {
      // A pulse byte between packets: 0x7F for sync, 0xFE during calibration.
      if (c == 0x7F) {
        sync_bytes++;
        if (sync_bytes == 1)
          rx_.insert(rx_.end(), noise.begin(), noise.end());
        if (reack && !reack_sent_ && !v_.status.empty()) {
          reack_sent_ = true;
          queue(Bytes{0x80}, -2);
        } else if (!status_sent_ && !v_.status.empty() && sync_bytes >= answer_at_sync &&
                   (!reack || !reack_raw.empty())) {
          status_sent_ = true;
          queue(v_.status, -2);
        }
      }
      if (c == 0xFE && held_ >= 0) {
        queue(v_.responses[held_], held_);
        held_ = -1;
      }
      return;
    }
    cur_.push_back(c);
    if (cur_.size() >= 5) {
      const size_t total = (size_t) ((cur_[3] << 8) | cur_[4]) + 2;
      if (cur_.size() == total) {
        packet(cur_);
        cur_.clear();
      }
    }
  }

  void packet(const Bytes &p) {
    CHECK(p[0] == 0x46 && p[1] == 0xB9 && p[2] == 0x6A && p.back() == 0x16);
    uint16_t sum = 0;
    for (size_t i = 2; i < p.size() - 3; i++)
      sum += p[i];
    CHECK(sum == ((p[p.size() - 3] << 8) | p[p.size() - 2]));
    const Bytes payload(p.begin() + 5, p.end() - 3);
    if (payload.size() == 1 && payload[0] == 0x82) {
      got_disconnect = true;
      disconnect_baud = bsl_baud_;
      return;
    }
    if (reack && payload == Bytes{0x80}) {
      reack_raw = p;
      return;
    }
    const size_t idx = packets++;
    if (idx >= v_.writes.size() || payload != v_.writes[idx]) {
      if (mismatches++ == 0)
        std::printf("  first difference at packet %zu\n", idx);
    }
    if (silent_after >= 0 && (int) packets > silent_after)
      return;
    if ((int) idx == ignore_packet)
      return;
    if (idx < v_.responses.size() && (int) idx != drop_response) {
      if (payload[0] == 0x00)
        held_ = (int) idx;  // calibration: answered after the next 0xFE pulse
      else
        queue(v_.responses[idx], (int) idx);
    }
    if (payload[0] == 0x01)
      bsl_baud_ = STC_TRANSFER_BAUD;  // the bootloader answers at the old rate, then switches
  }

  const Vectors &v_;
  std::deque<uint8_t> rx_;
  Bytes cur_;
  bool status_sent_ = false;
  bool reack_sent_ = false;
  int held_ = -1;
  uint32_t t_ = 0;
  uint32_t host_baud_ = 2400;
  uint32_t bsl_baud_ = 2400;
  uint32_t busy_until_ = 0;
};

static Vectors load(const char *path) {
  Vectors v;
  std::ifstream in(path);
  std::string kind, hex;
  while (in >> kind >> hex) {
    if (kind == "S")
      v.status = unhex(hex);
    else if (kind == "R")
      v.responses.push_back(unhex(hex));
    else if (kind == "W")
      v.writes.push_back(unhex(hex));
  }
  return v;
}

static void test_matches_stcgal(const Vectors &v, const Bytes &image) {
  FakeBsl io(v);
  stcisp::Programmer prog(io);
  CHECK(prog.connect(MCU_MODEL, 2000000, 10000) == stcisp::RESULT_OK);
  CHECK(prog.session().bsl_version == 0x72 && prog.session().bsl_stepping == 'T' && prog.session().bsl_minor == 5);
  CHECK(prog.session().factory_hz == 11059200);
  CHECK(prog.por_delay_long() && !prog.bsl_pindetect() && !prog.reset_pin_enabled());
  CHECK(prog.session().msr_len == 5);
  const stcisp::Result r = prog.program(image.data(), image.size(), MCU_USER_HZ, STC_TRANSFER_BAUD);
  CHECK(r == stcisp::RESULT_OK);
  prog.disconnect();
  CHECK(io.got_disconnect);
  CHECK(io.mismatches == 0);
  // stcgal's list ends with the disconnect packet, which FakeBsl counts separately.
  CHECK(v.writes.back() == Bytes{0x82});
  CHECK(io.packets == v.writes.size() - 1);
  CHECK(prog.session().stage == stcisp::STAGE_DONE);
  CHECK(prog.session().written == image.size());
  CHECK(prog.session().trim_hz == 11047200);
  CHECK(io.bauds.size() == 2 && io.bauds[0] == 2400 && io.bauds[1] == STC_TRANSFER_BAUD);
  CHECK(io.disconnect_baud == STC_TRANSFER_BAUD && io.garbled == 0);
  std::printf("  %zu packets identical to stcgal, %zu sync bytes\n", io.packets, io.sync_bytes);
}

static void test_wrong_model(const Vectors &v) {
  Vectors w = v;
  w.status[21] = 0x63;
  FakeBsl io(w);
  stcisp::Programmer prog(io);
  CHECK(prog.connect(MCU_MODEL, 2000000, 10000) == stcisp::RESULT_WRONG_MODEL);
  CHECK(io.packets == 0);  // nothing sent after the status packet
}

static void test_unexpected_options(const Vectors &v) {
  const struct {
    int byte;
    uint8_t mask;
  } cases[] = {{7, 0x80}, {7, 0x10}, {7, 0x08}, {12, 0x01}};
  for (const auto &c : cases) {
    Vectors w = v;
    w.status[c.byte] ^= c.mask;
    FakeBsl io(w);
    stcisp::Programmer prog(io);
    CHECK(prog.connect(MCU_MODEL, 2000000, 10000) == stcisp::RESULT_UNEXPECTED_OPTIONS);
    CHECK(io.packets == 0);
  }
}

static void test_no_reply() {
  Vectors empty;
  FakeBsl io(empty);
  stcisp::Programmer prog(io);
  // FakeBsl only answers 0x7F if it has a status packet; an empty one is never queued.
  CHECK(prog.connect(MCU_MODEL, 600000, 10000) == stcisp::RESULT_NO_REPLY);
}

// A stray byte must not stop the sync bytes: the bootloader only listens for them briefly.
static void test_noise_before_status(const Vectors &v, const Bytes &image) {
  FakeBsl io(v);
  io.noise = {0x00, 0xFF, 0x46, 0x00, 0x00};
  io.answer_at_sync = 3;
  stcisp::Programmer prog(io);
  const uint32_t t0 = io.now_us();
  CHECK(prog.connect(MCU_MODEL, 300000, 10000) == stcisp::RESULT_OK);
  CHECK(io.now_us() - t0 < 100000);
  CHECK(io.sync_bytes == 3);
  CHECK(prog.program(image.data(), image.size(), MCU_USER_HZ, STC_TRANSFER_BAUD) == stcisp::RESULT_OK);
  CHECK(io.mismatches == 0);
}

// A lone byte that looks like a frame start, then silence: the sync bytes must resume quickly.
static void test_lone_frame_start(const Vectors &v, const Bytes &image) {
  FakeBsl io(v);
  io.noise = {0x46};
  io.answer_at_sync = 2;
  stcisp::Programmer prog(io);
  const uint32_t t0 = io.now_us();
  CHECK(prog.connect(MCU_MODEL, 300000, 10000) == stcisp::RESULT_OK);
  CHECK(io.now_us() - t0 < 60000);
  CHECK(io.sync_bytes == 2);
  CHECK(prog.program(image.data(), image.size(), MCU_USER_HZ, STC_TRANSFER_BAUD) == stcisp::RESULT_OK);
  CHECK(io.mismatches == 0);
}

// Stc15AProtocol.get_status_packet: a 0x80 packet first, re-acknowledged as stcgal does.
static void test_reack(const Vectors &v, const Bytes &image) {
  FakeBsl io(v);
  io.reack = true;
  stcisp::Programmer prog(io);
  CHECK(prog.connect(MCU_MODEL, 2000000, 10000) == stcisp::RESULT_OK);
  // stcgal: PACKET_START + PACKET_HOST + 00 07 80 00 F1 + PACKET_END
  CHECK(io.reack_raw == (Bytes{0x46, 0xB9, 0x6A, 0x00, 0x07, 0x80, 0x00, 0xF1, 0x16}));
  CHECK(prog.session().bsl_version == 0x72 && prog.session().magic == MCU_MODEL);
  CHECK(prog.program(image.data(), image.size(), MCU_USER_HZ, STC_TRANSFER_BAUD) == stcisp::RESULT_OK);
  CHECK(io.mismatches == 0);
}

// The reply to the baud rate switch is lost after the bootloader switched: the disconnect must
// still reach it, at the new rate.
static void test_baud_reply_lost(const Vectors &v, const Bytes &image) {
  FakeBsl io(v);
  io.drop_response = 2;  // round 1, round 2, then the switch
  stcisp::Programmer prog(io);
  CHECK(prog.connect(MCU_MODEL, 2000000, 10000) == stcisp::RESULT_OK);
  CHECK(prog.program(image.data(), image.size(), MCU_USER_HZ, STC_TRANSFER_BAUD) == stcisp::RESULT_TIMEOUT);
  CHECK(prog.session().stage == stcisp::STAGE_BAUD);
  prog.disconnect();
  CHECK(io.got_disconnect && io.disconnect_baud == STC_TRANSFER_BAUD);
  CHECK(io.mismatches == 0 && io.packets == 3);
}

// The switch itself is lost, so the bootloader stays at 2400: the disconnect must reach it there.
static void test_baud_switch_lost(const Vectors &v, const Bytes &image) {
  FakeBsl io(v);
  io.ignore_packet = 2;
  stcisp::Programmer prog(io);
  CHECK(prog.connect(MCU_MODEL, 2000000, 10000) == stcisp::RESULT_OK);
  CHECK(prog.program(image.data(), image.size(), MCU_USER_HZ, STC_TRANSFER_BAUD) == stcisp::RESULT_TIMEOUT);
  CHECK(prog.session().stage == stcisp::STAGE_BAUD);
  prog.disconnect();
  CHECK(io.got_disconnect && io.disconnect_baud == 2400);
  CHECK(io.garbled == 9);  // the copy sent at the transfer rate
  CHECK(io.misread == 0);
}

// After a failure past the switch, the disconnect goes out once, at the new rate.
static void test_disconnect_after_switch(const Vectors &v, const Bytes &image) {
  FakeBsl io(v);
  io.silent_after = 3;  // no answer to the handshake
  stcisp::Programmer prog(io);
  CHECK(prog.connect(MCU_MODEL, 2000000, 10000) == stcisp::RESULT_OK);
  CHECK(prog.program(image.data(), image.size(), MCU_USER_HZ, STC_TRANSFER_BAUD) == stcisp::RESULT_TIMEOUT);
  CHECK(prog.session().stage == stcisp::STAGE_HANDSHAKE);
  prog.disconnect();
  CHECK(io.got_disconnect && io.disconnect_baud == STC_TRANSFER_BAUD);
  CHECK(io.bauds.size() == 2 && io.garbled == 0);
}

// A calibration reply claiming more trims than were sent.
static void test_calibration_too_long(const Vectors &v, const Bytes &image) {
  Vectors w = v;
  Bytes &r = w.responses[0];
  r.assign(2 + 2 * 20, 0x01);
  r[0] = 0x00;
  r[1] = 20;
  r[2 + 2 * 12] = 0xFF;  // only a pair past the 12 trims sent brackets the target
  FakeBsl io(w);
  stcisp::Programmer prog(io);
  CHECK(prog.connect(MCU_MODEL, 2000000, 10000) == stcisp::RESULT_OK);
  CHECK(prog.program(image.data(), image.size(), MCU_USER_HZ, STC_TRANSFER_BAUD) == stcisp::RESULT_CALIBRATION);
  CHECK(prog.session().stage == stcisp::STAGE_CALIBRATE);
  CHECK(io.packets == 1);
}

static void test_bad_checksum(const Vectors &v, const Bytes &image) {
  FakeBsl io(v);
  io.corrupt_response = 10;  // a write acknowledgement
  stcisp::Programmer prog(io);
  CHECK(prog.connect(MCU_MODEL, 2000000, 10000) == stcisp::RESULT_OK);
  CHECK(prog.program(image.data(), image.size(), MCU_USER_HZ, STC_TRANSFER_BAUD) == stcisp::RESULT_FRAMING);
  CHECK(prog.session().stage == stcisp::STAGE_WRITE);
}

static void test_silent(const Vectors &v, const Bytes &image) {
  FakeBsl io(v);
  io.silent_after = 5;  // answers up to the erase, then nothing
  stcisp::Programmer prog(io);
  CHECK(prog.connect(MCU_MODEL, 2000000, 10000) == stcisp::RESULT_OK);
  CHECK(prog.program(image.data(), image.size(), MCU_USER_HZ, STC_TRANSFER_BAUD) == stcisp::RESULT_TIMEOUT);
  CHECK(prog.session().stage == stcisp::STAGE_WRITE);
}

static void test_bad_image(const Vectors &v, const Bytes &image) {
  FakeBsl io(v);
  stcisp::Programmer prog(io);
  CHECK(prog.connect(MCU_MODEL, 2000000, 10000) == stcisp::RESULT_OK);
  CHECK(prog.program(image.data(), 100, MCU_USER_HZ, STC_TRANSFER_BAUD) == stcisp::RESULT_BAD_IMAGE);
  CHECK(io.packets == 0);
}

int main(int argc, char **argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s <vectors.txt> <image.bin>\n", argv[0]);
    return 2;
  }
  const Vectors v = load(argv[1]);
  std::ifstream f(argv[2], std::ios::binary);
  const Bytes image((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  CHECK(!v.status.empty() && !v.writes.empty() && !image.empty());

  std::printf("test_matches_stcgal\n");
  test_matches_stcgal(v, image);
  std::printf("test_wrong_model\n");
  test_wrong_model(v);
  std::printf("test_unexpected_options\n");
  test_unexpected_options(v);
  std::printf("test_no_reply\n");
  test_no_reply();
  std::printf("test_noise_before_status\n");
  test_noise_before_status(v, image);
  std::printf("test_lone_frame_start\n");
  test_lone_frame_start(v, image);
  std::printf("test_reack\n");
  test_reack(v, image);
  std::printf("test_baud_reply_lost\n");
  test_baud_reply_lost(v, image);
  std::printf("test_baud_switch_lost\n");
  test_baud_switch_lost(v, image);
  std::printf("test_disconnect_after_switch\n");
  test_disconnect_after_switch(v, image);
  std::printf("test_calibration_too_long\n");
  test_calibration_too_long(v, image);
  std::printf("test_bad_checksum\n");
  test_bad_checksum(v, image);
  std::printf("test_silent\n");
  test_silent(v, image);
  std::printf("test_bad_image\n");
  test_bad_image(v, image);

  TEST_EXIT();
}
