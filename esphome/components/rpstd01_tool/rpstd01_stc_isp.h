/*
  STC15 bootloader (ISP) programmer, ported from stcgal's Stc15Protocol: stcgal/protocols.py
  (StcBaseProtocol, Stc12BaseProtocol, Stc15AProtocol, Stc15Protocol) and frontend.py (program_mcu).
  Plain C++ with no platform code, so the host tests can check it against stcgal itself.

  Copyright (c) 2013-2015 Grigori Goronzy, MIT license: see LICENSES/stcgal-MIT.txt.
  SPDX-License-Identifier: GPL-3.0-or-later AND MIT
*/

#ifndef RPSTD01_STC_ISP_H
#define RPSTD01_STC_ISP_H

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace stcisp {

// Serial port and clock, provided by the platform. The port is always 8 data bits, even parity,
// one stop bit.
class Io {
 public:
  virtual void set_baud(uint32_t baud) = 0;
  virtual void write(const uint8_t *data, size_t len) = 0;  // returns once the bytes are sent
  virtual bool available() = 0;
  virtual int read(uint32_t timeout_us) = 0;                // a byte, or -1 on timeout
  virtual uint32_t now_us() = 0;
  virtual void delay_us(uint32_t us) = 0;
};

enum Stage : uint8_t {
  STAGE_SYNC = 0,
  STAGE_STATUS,
  STAGE_CALIBRATE,
  STAGE_BAUD,
  STAGE_HANDSHAKE,
  STAGE_ERASE,
  STAGE_WRITE,
  STAGE_FINISH,
  STAGE_OPTIONS,
  STAGE_DONE,
};

enum Result : uint8_t {
  RESULT_OK = 0,
  RESULT_NO_REPLY,        // no bootloader answer to the sync bytes
  RESULT_TIMEOUT,         // the bootloader stopped answering
  RESULT_FRAMING,         // a reply with a bad header, length or checksum
  RESULT_BAD_REPLY,       // a reply with the wrong command byte
  RESULT_WRONG_MODEL,     // not the MCU this image is for
  RESULT_UNEXPECTED_OPTIONS,  // an option bit the install depends on is not as expected; nothing was changed
  RESULT_EXTERNAL_CLOCK,  // not supported
  RESULT_CALIBRATION,
  RESULT_LOCKED,
  RESULT_BAD_IMAGE,       // empty, or not a multiple of IMAGE_UNIT
};

inline constexpr uint32_t HANDSHAKE_BAUD = 2400;
inline constexpr uint32_t HANDSHAKE_CHAR_US = 11 * 1000000 / HANDSHAKE_BAUD;  // 8E1
inline constexpr uint32_t PROGRAM_HZ = 22118400;  // stcgal's programming clock
inline constexpr size_t IMAGE_UNIT = 512;  // as stcgal pads its images
inline constexpr size_t BLOCK_SIZE = 64;
inline constexpr size_t OPTIONS_SIZE = 64;
inline constexpr size_t MAX_PACKET = 128;

struct Session {
  Stage stage;
  uint8_t status_len;
  uint16_t magic;
  uint8_t bsl_version;   // 0x72 for 7.2.x
  uint8_t bsl_stepping;  // a letter, 'T' for 7.2.5T
  uint8_t bsl_minor;
  uint32_t factory_hz;   // clock stored by the last programming, 0 if none
  uint8_t msr[5];        // option bytes, written back unchanged
  uint8_t msr_len;
  uint8_t user_trim[2];  // trim value and range for the user clock
  uint8_t prog_trim[2];  // the same for the programming clock
  uint32_t trim_hz;      // user clock reached by calibration
  uint32_t written;
};

class Programmer {
 public:
  explicit Programmer(Io &io) : io_(io) {}

  // Sends the sync byte every gap_us until the bootloader answers with its status
  // packet, for at most timeout_us. Checks the model, the internal clock and the option bits the
  // install depends on (accept_status). Changes nothing.
  Result connect(uint16_t expected_magic, uint32_t timeout_us, uint32_t gap_us) {
    session_.stage = STAGE_SYNC;
    io_.set_baud(HANDSHAKE_BAUD);
    const uint32_t start = io_.now_us();
    uint32_t last = start - gap_us;
    for (;;) {
      if ((uint32_t) (io_.now_us() - start) > timeout_us)
        return RESULT_NO_REPLY;
      if (io_.available()) {
        // Like stcgal: drop a byte that cannot start a packet and keep sending sync bytes.
        const int c = io_.read(0);
        if (c != 0x46 && c != 0x68)
          continue;
        session_.stage = STAGE_STATUS;
        size_t len = 0;
        // A short wait between bytes, so that a lone noise byte that looks like a frame start
        // does not hold back the sync bytes for long.
        Result r = read_packet(buf_, &len, 500000, true, c, STATUS_INTER_BYTE_US);
        if (r == RESULT_OK && len > 0 && buf_[0] == 0x80) {
          // Acknowledged, the bootloader sends its status again.
          const uint8_t ack[] = {0x80};
          write_packet(ack, sizeof(ack));
          pulse(0x7F, 500000);
          r = read_packet(buf_, &len, 500000, true, -1, STATUS_INTER_BYTE_US);
        }
        if (r == RESULT_OK && len >= 23 && buf_[0] == 0x50)
          return accept_status(len, expected_magic);
        session_.stage = STAGE_SYNC;
        continue;  // like stcgal: ignore anything else and keep sending sync bytes
      }
      if ((uint32_t) (io_.now_us() - last) >= gap_us) {
        const uint8_t sync = 0x7F;
        io_.write(&sync, 1);
        last = io_.now_us();
      }
    }
  }

  // Calibrates the clock to user_hz, switches to transfer_baud, erases, writes image (from
  // address 0), finishes, and writes the option bytes back unchanged with the new trim.
  Result program(const uint8_t *image, size_t len, uint32_t user_hz, uint32_t transfer_baud) {
    if (len == 0 || len % IMAGE_UNIT != 0)
      return RESULT_BAD_IMAGE;
    Result r = calibrate(user_hz);
    if (r != RESULT_OK)
      return r;
    if ((r = switch_baud(transfer_baud)) != RESULT_OK)
      return r;

    session_.stage = STAGE_HANDSHAKE;
    {
      const uint8_t p[] = {0x05, 0x00, 0x00, 0x5A, 0xA5};
      size_t n = 0;
      if ((r = exchange(p, bsl72() ? sizeof(p) : 1, &n, 2000000)) != RESULT_OK)
        return r;
      if (n == 1 && buf_[0] == 0x0F)
        return RESULT_LOCKED;
      if (n < 1 || buf_[0] != 0x05)
        return RESULT_BAD_REPLY;
    }

    session_.stage = STAGE_ERASE;
    {
      const uint8_t p[] = {0x03, 0x00, 0x00, 0x5A, 0xA5};
      size_t n = 0;
      if ((r = exchange(p, bsl72() ? sizeof(p) : 2, &n, 20000000)) != RESULT_OK)
        return r;
      if (n < 1 || buf_[0] != 0x03)
        return RESULT_BAD_REPLY;
    }

    session_.stage = STAGE_WRITE;
    for (size_t i = 0; i < len; i += BLOCK_SIZE) {
      uint8_t p[5 + BLOCK_SIZE];
      size_t k = 0;
      p[k++] = i == 0 ? 0x22 : 0x02;
      p[k++] = (uint8_t) (i >> 8);
      p[k++] = (uint8_t) i;
      if (bsl72()) {
        p[k++] = 0x5A;
        p[k++] = 0xA5;
      }
      memcpy(p + k, image + i, BLOCK_SIZE);
      k += BLOCK_SIZE;
      size_t n = 0;
      if ((r = exchange(p, k, &n, 2000000)) != RESULT_OK)
        return r;
      if (n < 2 || buf_[0] != 0x02 || buf_[1] != 0x54)
        return RESULT_BAD_REPLY;
      session_.written = i + BLOCK_SIZE;
    }

    if (bsl72()) {
      session_.stage = STAGE_FINISH;
      const uint8_t p[] = {0x07, 0x00, 0x00, 0x5A, 0xA5};
      size_t n = 0;
      if ((r = exchange(p, sizeof(p), &n, 2000000)) != RESULT_OK)
        return r;
      if (n < 2 || buf_[0] != 0x07 || buf_[1] != 0x54)
        return RESULT_BAD_REPLY;
    }

    session_.stage = STAGE_OPTIONS;
    {
      uint8_t p[5 + OPTIONS_SIZE];
      size_t k = 0;
      p[k++] = 0x04;
      p[k++] = 0x00;
      p[k++] = 0x00;
      if (bsl72()) {
        p[k++] = 0x5A;
        p[k++] = 0xA5;
      }
      build_options(p + k);
      k += OPTIONS_SIZE;
      size_t n = 0;
      if ((r = exchange(p, k, &n, 2000000)) != RESULT_OK)
        return r;
      if (n < 2 || buf_[0] != 0x04 || buf_[1] != 0x54)
        return RESULT_BAD_REPLY;
    }
    session_.stage = STAGE_DONE;
    return RESULT_OK;
  }

  // Leaves the bootloader and starts the user program (stcgal's disconnect). If the baud switch
  // reply was lost the bootloader may be at either rate: send at the new rate, wait RATE_GAP_US,
  // then the old.
  void disconnect() {
    const uint8_t p[] = {0x82};
    if (baud_unsure_) {
      io_.delay_us(BAUD_SWITCH_US);
      io_.set_baud(transfer_baud_);
      write_packet(p, sizeof(p));
      io_.set_baud(HANDSHAKE_BAUD);
      io_.delay_us(RATE_GAP_US);
      baud_unsure_ = false;
    }
    write_packet(p, sizeof(p));
  }

  bool por_delay_long() const { return session_.msr[2] & 0x80; }
  bool bsl_pindetect() const { return !(session_.msr[3] & 0x01); }
  bool reset_pin_enabled() const { return !(session_.msr[2] & 0x10); }
  bool rstout_high() const { return session_.msr[2] & 0x08; }
  bool internal_clock() const { return session_.msr[2] & 0x01; }
  bool low_voltage_reset() const { return !(session_.msr[1] & 0x40); }
  uint8_t low_voltage_threshold() const { return session_.msr[1] & 0x07; }
  bool watchdog_por() const { return !(session_.msr[0] & 0x20); }

  const Session &session() const { return session_; }

 private:
  bool bsl72() const { return session_.bsl_version >= 0x72; }

  static uint16_t be16(const uint8_t *p) { return (uint16_t) ((p[0] << 8) | p[1]); }

  Result accept_status(size_t len, uint16_t expected_magic) {
    session_.status_len = (uint8_t) len;
    session_.magic = be16(buf_ + 20);
    session_.bsl_version = buf_[17];
    session_.bsl_stepping = buf_[18];
    session_.bsl_minor = buf_[22] & 0x0F;
    session_.factory_hz = ((uint32_t) buf_[8] << 24) | ((uint32_t) buf_[9] << 16) |
                      ((uint32_t) buf_[10] << 8) | buf_[11];
    if (session_.factory_hz == 0xFFFFFFFFu)
      session_.factory_hz = 0;
    session_.msr[0] = buf_[5];
    session_.msr[1] = buf_[6];
    session_.msr[2] = buf_[7];
    session_.msr[3] = buf_[12];
    session_.msr_len = 4;
    if (len > 37) {
      session_.msr[4] = buf_[37];
      session_.msr_len = 5;
    }
    if (session_.magic != expected_magic)
      return RESULT_WRONG_MODEL;
    if (!(buf_[7] & 0x01))
      return RESULT_EXTERNAL_CLOCK;
    // The box-closed route depends on these; refuse to touch a chip where they differ.
    if (!por_delay_long() || bsl_pindetect() || reset_pin_enabled() || !rstout_high())
      return RESULT_UNEXPECTED_OPTIONS;
    return RESULT_OK;
  }

  // False if no pair brackets the target.
  static bool choose_range(const uint8_t *challenge, const uint8_t *resp, size_t resp_len,
                           double target, int32_t *trim, uint8_t *range) {
    const uint8_t calib_len = resp[1];
    const uint8_t *calib = resp + 2;
    if (calib_len > CALIB_MAX || resp_len - 2 < 2u * calib_len)
      return false;
    for (int i = 0; i + 1 < calib_len; i++) {
      const double a = be16(calib + 2 * i), b = be16(calib + 2 * i + 2);
      const double ta = challenge[2 * i], tb = challenge[2 * i + 2];
      if ((a <= target && b >= target) || (b <= target && a >= target)) {
        if (b == a)
          return false;
        const double m = (tb - ta) / (b - a);
        const double n = ta - m * a;
        const double t = std::nearbyint(m * target + n);  // ties to even, like Python's round()
        if (t > 65536 || t < 0)
          return false;
        *trim = (int32_t) t;
        *range = challenge[2 * i + 3];
        return true;
      }
    }
    return false;
  }

  static bool choose_trim(const uint8_t *challenge, const uint8_t *resp, size_t resp_len,
                          double target, uint8_t out[2], uint16_t *count_out) {
    const uint8_t calib_len = resp[1];
    const uint8_t *calib = resp + 2;
    if (calib_len > CALIB_MAX || resp_len - 2 < 2u * calib_len || calib_len == 0)
      return false;
    double best = 1e300;
    for (int i = 0; i < calib_len; i++) {
      const uint16_t count = be16(calib + 2 * i);
      const double d = std::fabs(count - target);
      if (d < best) {
        best = d;
        out[0] = challenge[2 * i];
        out[1] = challenge[2 * i + 1];
        *count_out = count;
      }
    }
    return true;
  }

  static uint8_t iap_wait(uint32_t hz) {
    if (hz < 1000000) return 0x87;
    if (hz < 2000000) return 0x86;
    if (hz < 3000000) return 0x85;
    if (hz < 6000000) return 0x84;
    if (hz < 12000000) return 0x83;
    if (hz < 20000000) return 0x82;
    if (hz < 24000000) return 0x81;
    return 0x80;
  }

  Result calibrate(uint32_t user_hz) {
    session_.stage = STAGE_CALIBRATE;
    if (user_hz == 0)
      user_hz = session_.factory_hz;
    if (user_hz == 0)
      return RESULT_CALIBRATION;
    const double target_user = std::nearbyint(user_hz / (HANDSHAKE_BAUD / 2.0));
    const double target_prog = std::nearbyint(PROGRAM_HZ / (HANDSHAKE_BAUD / 2.0));

    // Round 1: coarse trims across four ranges.
    static const uint8_t round1[] = {0x00, 12,   0x00, 0xC0, 0x80, 0xC0, 0xFF, 0xC0, 0x00,
                                     0x80, 0x80, 0x80, 0xFF, 0x80, 0x00, 0x40, 0x80, 0x40,
                                     0xFF, 0x40, 0x00, 0x00, 0x80, 0x00, 0xC0, 0x00};
    size_t n = 0;
    Result r = exchange_pulsed(round1, sizeof(round1), &n);
    if (r != RESULT_OK)
      return r;
    if (n < 2 || buf_[0] != 0x00)
      return RESULT_BAD_REPLY;
    int32_t ut, pt;
    uint8_t ur, pr;
    if (!choose_range(round1 + 2, buf_, n, target_user, &ut, &ur) ||
        !choose_range(round1 + 2, buf_, n, target_prog, &pt, &pr))
      return RESULT_CALIBRATION;

    uint8_t round2[2 + 24];
    size_t k = 0;
    round2[k++] = 0x00;
    round2[k++] = 12;
    for (int32_t i = ut - 3; i < ut + 3; i++) {
      round2[k++] = (uint8_t) i;
      round2[k++] = ur;
    }
    for (int32_t i = pt - 3; i < pt + 3; i++) {
      round2[k++] = (uint8_t) i;
      round2[k++] = pr;
    }
    if ((r = exchange_pulsed(round2, k, &n)) != RESULT_OK)
      return r;
    if (n < 2 || buf_[0] != 0x00)
      return RESULT_BAD_REPLY;
    uint16_t user_count = 0, prog_count = 0;
    if (!choose_trim(round2 + 2, buf_, n, target_user, session_.user_trim, &user_count) ||
        !choose_trim(round2 + 2, buf_, n, target_prog, session_.prog_trim, &prog_count))
      return RESULT_CALIBRATION;
    session_.trim_hz = (uint32_t) std::nearbyint(user_count * (HANDSHAKE_BAUD / 2.0));
    return RESULT_OK;
  }

  Result switch_baud(uint32_t transfer_baud) {
    session_.stage = STAGE_BAUD;
    const uint16_t brt = (uint16_t) (int32_t) (65536 - (double) PROGRAM_HZ / (transfer_baud * 4.0));
    const uint8_t p[] = {0x01,
                         session_.prog_trim[0],
                         session_.prog_trim[1],
                         (uint8_t) (brt >> 8),
                         (uint8_t) brt,
                         session_.user_trim[1],
                         session_.user_trim[0],
                         iap_wait(PROGRAM_HZ)};
    transfer_baud_ = transfer_baud;
    baud_unsure_ = true;
    size_t n = 0;
    const Result r = exchange(p, sizeof(p), &n, 2000000);
    if (r != RESULT_OK)
      return r;
    if (n < 1 || buf_[0] != 0x01)
      return RESULT_BAD_REPLY;
    io_.delay_us(BAUD_SWITCH_US);
    io_.set_baud(transfer_baud);
    baud_unsure_ = false;
    return RESULT_OK;
  }

  void build_options(uint8_t *o) const {
    const uint32_t f = session_.trim_hz;
    size_t k = 0;
    memset(o, 0xFF, OPTIONS_SIZE);
    k = 23;
    o[k++] = (uint8_t) (f >> 24);
    o[k++] = 0xFF;
    o[k++] = (uint8_t) (f >> 16);
    o[k++] = 0xFF;
    o[k++] = (uint8_t) (f >> 8);
    o[k++] = 0xFF;
    o[k++] = (uint8_t) f;
    o[k++] = 0xFF;
    o[k++] = session_.msr[3];
    k += 23;
    o[k++] = session_.msr_len > 4 ? session_.msr[4] : 0xFF;
    k += 3;
    o[k++] = session_.user_trim[0];
    o[k++] = (uint8_t) (session_.user_trim[1] + 0x3F);
    o[k++] = session_.msr[0];
    o[k++] = session_.msr[1];
    o[k++] = session_.msr[2];
  }

  void write_packet(const uint8_t *payload, size_t n) {
    uint8_t p[MAX_PACKET + 8];
    size_t k = 0;
    p[k++] = 0x46;
    p[k++] = 0xB9;
    p[k++] = 0x6A;
    p[k++] = (uint8_t) ((n + 6) >> 8);
    p[k++] = (uint8_t) (n + 6);
    memcpy(p + k, payload, n);
    k += n;
    uint16_t sum = 0;
    for (size_t i = 2; i < k; i++)
      sum += p[i];
    p[k++] = (uint8_t) (sum >> 8);
    p[k++] = (uint8_t) sum;
    p[k++] = 0x16;
    io_.write(p, k);
  }

  // The payload goes to out. first: the first byte if the caller has already read it, else -1.
  // inter_byte_us: the longest wait for each later byte.
  Result read_packet(uint8_t *out, size_t *len, uint32_t first_timeout_us, bool skip_noise, int first = -1,
                     uint32_t inter_byte_us = INTER_BYTE_US) {
    uint8_t p[MAX_PACKET + 8];
    int c = first;
    const uint32_t start = io_.now_us();
    while (c < 0) {
      const uint32_t spent = io_.now_us() - start;
      if (spent >= first_timeout_us)
        return RESULT_TIMEOUT;
      c = io_.read(first_timeout_us - spent);
      if (c < 0)
        return RESULT_TIMEOUT;
      if (c != 0x46 && c != 0x68 && skip_noise)
        c = -1;
    }
    size_t k;
    if (c == 0x68) {
      // Some bootloaders leave out the frame start before the status packet.
      p[0] = 0x46;
      p[1] = 0xB9;
      p[2] = 0x68;
      k = 3;
    } else {
      if (c != 0x46)
        return RESULT_FRAMING;
      p[0] = 0x46;
      if ((c = io_.read(inter_byte_us)) != 0xB9)
        return c < 0 ? RESULT_TIMEOUT : RESULT_FRAMING;
      p[1] = 0xB9;
      if ((c = io_.read(inter_byte_us)) != 0x68)
        return c < 0 ? RESULT_TIMEOUT : RESULT_FRAMING;
      p[2] = 0x68;
      k = 3;
    }
    for (int i = 0; i < 2; i++) {
      if ((c = io_.read(inter_byte_us)) < 0)
        return RESULT_TIMEOUT;
      p[k++] = (uint8_t) c;
    }
    const size_t plen = be16(p + 3);
    if (plen < 6 || plen + 2 > sizeof(p))
      return RESULT_FRAMING;
    while (k < plen + 2) {
      if ((c = io_.read(inter_byte_us)) < 0)
        return RESULT_TIMEOUT;
      p[k++] = (uint8_t) c;
    }
    if (p[k - 1] != 0x16)
      return RESULT_FRAMING;
    uint16_t sum = 0;
    for (size_t i = 2; i < k - 3; i++)
      sum += p[i];
    if (sum != be16(p + k - 3))
      return RESULT_FRAMING;
    *len = k - 8;
    memcpy(out, p + 5, *len);
    return RESULT_OK;
  }

  Result exchange(const uint8_t *p, size_t n, size_t *len, uint32_t timeout_us) {
    write_packet(p, n);
    return read_packet(buf_, len, timeout_us, false);
  }

  // Sends a packet, then the pulse byte every PULSE_GAP_US until the MCU answers (calibration).
  Result exchange_pulsed(const uint8_t *p, size_t n, size_t *len) {
    write_packet(p, n);
    if (!pulse(0xFE, 1000000))
      return RESULT_TIMEOUT;
    return read_packet(buf_, len, 2000000, false);
  }

  // True once a reply starts.
  bool pulse(uint8_t c, uint32_t timeout_us) {
    const uint32_t start = io_.now_us();
    for (;;) {
      io_.write(&c, 1);
      const uint32_t sent = io_.now_us();
      while ((uint32_t) (io_.now_us() - sent) < PULSE_GAP_US) {
        if (io_.available())
          return true;
      }
      if ((uint32_t) (io_.now_us() - start) > timeout_us)
        return false;
    }
  }

  static constexpr uint32_t INTER_BYTE_US = 500000;
  static constexpr uint32_t PULSE_GAP_US = 30000;
  // The status packet arrives in one piece.
  static constexpr uint32_t STATUS_INTER_BYTE_US = 20000;
  static_assert(STATUS_INTER_BYTE_US > 4 * HANDSHAKE_CHAR_US, "STATUS_INTER_BYTE_US");
  // Between the disconnect at the new and at the old rate, so the bootloader at the old rate sees
  // the line idle before the packet.
  static constexpr uint32_t RATE_GAP_US = 20000;
  static_assert(RATE_GAP_US > 4 * HANDSHAKE_CHAR_US, "RATE_GAP_US");
  // How long the bootloader takes to change its rate after it answered the switch, as stcgal waits:
  // nothing on the line shows when it is done.
  static constexpr uint32_t BAUD_SWITCH_US = 100000;
  static constexpr uint8_t CALIB_MAX = 12;  // trims sent per calibration round

  Io &io_;
  Session session_{};
  uint8_t buf_[MAX_PACKET];
  uint32_t transfer_baud_ = HANDSHAKE_BAUD;
  bool baud_unsure_ = false;  // the baud rate switch was sent, but not confirmed
};

}

#endif  // RPSTD01_STC_ISP_H
