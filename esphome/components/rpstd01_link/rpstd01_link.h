// RP-STD01: the link to our MCU firmware.
//
// Plain C++17 with no ESPHome dependency, so the unit tests can build it with g++.
//
// U, the UART type of every template that takes one: size_t available(); bool read_byte(uint8_t *);
// void write_array(const uint8_t *, size_t). ESPHome's uart::UARTComponent fits.
#pragma once

#include <algorithm>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "esphome/components/rpstd01_link/rpstd01_fw_constants.h"

namespace rpstd01 {

inline uint16_t crc16(const uint8_t *data, size_t len, uint16_t crc = CRC16_INIT);

struct Status {
  uint8_t state, faults, buttons;
  uint16_t target, current, half;  // PCA counts; half = 0 when Zo is not good
  uint16_t ks_min;
  uint8_t flags;
  uint16_t therm, wheel;
  // New at a start of the MCU's application, except after one that lost auxiliary RAM and found it
  // in a fixed state: "starting" shows that one.
  uint8_t boot_id;
  uint8_t leds, night;  // the outputs in use (MSG_OUTPUTS payload)
  bool starting() const { return (this->flags & S_STARTING) != 0; }
};
struct Info {
  uint16_t version;
  uint32_t build;
  uint8_t reset_cause;  // RESET_*
  uint8_t wdt_count;
  uint8_t boot_version;  // from the bootloader's last byte
  uint8_t boot_id;       // of the start that sent it, as in the status
};
struct BootStatus {
  uint8_t version, app_valid, reason, result;  // reason: BOOT_REASON_*, result: BR_*
  uint16_t next;
};

// The MCU's firmware against the firmware this release carries or expects. Never older by version
// alone: the version's high byte is the protocol, the low byte the release.
enum class FirmwareAge : uint8_t { CURRENT, APP_OLD, BOOT_OLD, NEWER };
inline FirmwareAge firmware_age(const Info &mcu, uint16_t version, uint32_t build, uint8_t boot_version);

inline bool decode_status(const uint8_t *p, uint8_t len, Status *out);
inline bool decode_info(const uint8_t *p, uint8_t len, Info *out);
inline bool decode_boot_status(const uint8_t *p, uint8_t len, BootStatus *out);
// Writes a whole frame into out (room for len + FRAME_OVERHEAD bytes); returns its length.
inline size_t build_frame(uint8_t *out, uint8_t type, const uint8_t *payload, uint8_t len);

// Frames from the MCU, taking only the types the MCU sends. Any length is taken: a release that
// changes a frame's length must still be seen to answer, so that the MCU tool does not take it for an
// MCU without our firmware. A frame that turns out bad is scanned again from the byte after its
// start, so a frame cut off by an MCU reset cannot hide the frames whose bytes it took.
class FrameParser {
 public:
  // Feeds one byte, and calls on_frame(type, payload, len) for each complete frame with a good CRC
  // that it completes. The payload is valid only during the call.
  template<typename F> void feed(uint8_t byte, F &&on_frame);
  // Gives up the frame in progress, and passes on any whole frame among the bytes after its start.
  template<typename F> void flush(F &&on_frame);
  bool idle() const { return this->used_ == 0; }

 protected:
  enum class Scan : uint8_t { NEED_MORE, BAD, GOOD };
  template<typename F> void take_frames_(F &&on_frame);
  Scan scan_() const;
  void drop_(size_t n);
  static bool type_known_(uint8_t type);

  uint8_t window_[UINT8_MAX + FRAME_OVERHEAD]{};
  size_t used_{0};
};

enum class LinkState : uint8_t { NO_RESPONSE, APPLICATION, BOOTLOADER };

// One frame from the MCU, as McuLink passes it to its owner after taking it in.
struct Received {
  uint8_t type;
  const uint8_t *payload;
  uint8_t len;
  bool decoded;    // status(), info() or boot_status() now holds it; false for a length this release cannot read
  uint8_t starts;  // starts of the MCU's application it showed
};

class McuLink {
 public:
  // A status within the last APP_SEEN_MS is the application; a boot status within the last
  // BOOT_SEEN_MS, and no status, our bootloader.
  LinkState link_state() const;
  bool have_status() const { return this->have_status_; }
  const Status &status() const { return this->status_; }
  bool have_info() const { return this->have_info_; }
  const Info &info() const { return this->info_; }
  // The info frame whose boot ID the status shows.
  bool have_current_info() const;
  const Info &current_info() const { return this->current_info_; }
  bool have_boot_status() const { return this->have_boot_; }
  const BootStatus &boot_status() const { return this->boot_; }
  // A new boot ID, or a status that shows "starting" again after one without it.
  uint32_t mcu_restarts() const { return this->restarts_; }
  // A frame of a length this release cannot read came within UNREADABLE_SEEN_MS: the MCU runs a
  // release that this one does not know.
  bool unreadable_frames() const;

 protected:
  // Calls on_frame(const Received &) for each complete frame. A part frame is given up after
  // RX_GAP_MS without a byte, so a frame cut off by an MCU reset cannot hold back the frames after
  // it.
  template<typename U, typename F> void receive_(U &uart, uint32_t now_ms, F &&on_frame);
  // Our bootloader takes only its own commands and drops anything else, and each command restarts
  // its IDLE_RETURN_MS idle time: nothing useful can go out to it but bootloader commands.
  bool in_bootloader_() const { return this->link_state() == LinkState::BOOTLOADER || this->last_seen_boot_; }
  uint32_t elapsed_(uint32_t since) const { return this->now_ - since; }

  uint32_t now_{0};
  bool have_status_{false}, have_info_{false}, have_current_info_{false}, have_boot_{false};
  Status status_{};
  Info info_{}, current_info_{};
  BootStatus boot_{};
  bool last_seen_boot_{false};  // the newest frame was a boot status

 private:
  void take_(Received *rx);
  uint8_t note_boot_id_(uint8_t boot_id);

  FrameParser parser_;
  uint32_t last_rx_ms_{0}, last_status_ms_{0}, last_boot_ms_{0}, last_unreadable_ms_{0};
  bool have_unreadable_{false};
  bool have_boot_id_{false};
  uint8_t boot_id_{0};  // from the newest status or info frame
  uint32_t restarts_{0};
};

inline constexpr uint8_t NO_SLOT = 0xFF;

// Sends the first frame that prepare(slot, frame) builds, trying slots in order: at most one
// application frame per loop, since the MCU's receive ring (HAL_RX_RING) holds only a few. Returns
// the slot sent, or NO_SLOT.
template<typename U, typename P> uint8_t send_first(U &uart, std::initializer_list<uint8_t> slots, P &&prepare) {
  uint8_t frame[APP_MAX_PAYLOAD + FRAME_OVERHEAD];
  for (uint8_t slot : slots) {
    if (size_t n = prepare(slot, frame)) {
      uart.write_array(frame, n);
      return slot;
    }
  }
  return NO_SLOT;
}

// Appends to a text buffer, and stops quietly when it is full.
struct TextOut {
  TextOut(char *buf, size_t size, size_t used = 0) : buf(buf), size(size), used(used) {
    if (used < size)
      buf[used] = '\0';
  }
  void add(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
  char *buf;
  size_t size;
  size_t used;
};

struct FirmwareText {
  char str[20];
};
inline FirmwareText firmware_text(uint16_t version, uint32_t build);

inline const char *reset_cause_name(uint8_t reset_cause);

// Publishes text on a text sensor (S: ESPHome's TextSensor) only when it changed, so unchanged text
// costs no std::string.
template<typename S> void publish_if_changed(S *sensor, const char *text) {
  if (!sensor->has_state() || sensor->state != text)
    sensor->publish_state(text);
}

namespace detail {

inline uint16_t get16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline void put16(uint8_t *p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}
inline uint32_t get32(const uint8_t *p) { return get16(p) | (static_cast<uint32_t>(get16(p + 2)) << 16); }
inline void put32(uint8_t *p, uint32_t v) {
  put16(p, static_cast<uint16_t>(v));
  put16(p + 2, static_cast<uint16_t>(v >> 16));
}

// A false frame start inside a status holds back the frames behind it until the bytes of the longest
// frame have come, at one status per STATUS_PERIOD_MS.
inline constexpr uint32_t FRAME_MAX_BYTES = UINT8_MAX + FRAME_OVERHEAD;
inline constexpr uint32_t STATUS_FRAME_BYTES = STATUS_LEN + FRAME_OVERHEAD;
inline constexpr uint32_t FALSE_START_HOLD_MS =
    ((FRAME_MAX_BYTES + STATUS_FRAME_BYTES - 1) / STATUS_FRAME_BYTES + 1) * STATUS_PERIOD_MS;
// ESPHome sets the ESP32's UART to pass received bytes on once 10 ms of them have come or the line
// is idle. A pause of the main loop needs no allowance: the link's time only moves on in receive_(),
// after it has read every byte that came in the pause.
inline constexpr uint32_t UART_DELIVERY_MS = 10;
inline constexpr uint32_t APP_SEEN_MS = FALSE_START_HOLD_MS + UART_DELIVERY_MS;
// Two boot statuses may be lost in a row.
inline constexpr uint32_t BOOT_SEEN_MS = 3 * BOOT_STATUS_PERIOD_MS;
inline constexpr uint32_t UNREADABLE_SEEN_MS = std::max(APP_SEEN_MS, BOOT_SEEN_MS);
inline constexpr uint32_t RX_GAP_MS = 2 * STATUS_PERIOD_MS;

}

template<typename U, typename F> void McuLink::receive_(U &uart, uint32_t now_ms, F &&on_frame) {
  this->now_ = now_ms;
  auto take = [this, &on_frame](uint8_t type, const uint8_t *payload, uint8_t len) {
    Received rx{type, payload, len, false, 0};
    this->take_(&rx);
    on_frame(rx);
  };
  if (uart.available() > 0) {
    this->last_rx_ms_ = now_ms;
  } else if (!this->parser_.idle() && this->elapsed_(this->last_rx_ms_) > detail::RX_GAP_MS) {
    this->parser_.flush(take);
  }
  while (uart.available() > 0) {
    uint8_t byte;
    if (!uart.read_byte(&byte))
      break;
    this->parser_.feed(byte, take);
  }
}

inline void McuLink::take_(Received *rx) {
  if (rx->type == MSG_STATUS) {
    Status s;
    rx->decoded = decode_status(rx->payload, rx->len, &s);
    if (rx->decoded) {
      // Only a new start sets "starting" after a status without it. A new boot ID is counted below.
      const bool started_again = this->have_status_ && !this->status_.starting() && s.starting() &&
                                 s.boot_id == this->status_.boot_id;
      rx->starts = this->note_boot_id_(s.boot_id) + (started_again ? 1 : 0);
      this->status_ = s;
      this->have_status_ = true;
      if (this->have_info_ && this->info_.boot_id == s.boot_id) {
        this->current_info_ = this->info_;
        this->have_current_info_ = true;
      }
      this->last_status_ms_ = this->now_;
      this->last_seen_boot_ = false;
    }
  } else if (rx->type == MSG_INFO) {
    Info info;
    rx->decoded = decode_info(rx->payload, rx->len, &info);
    if (rx->decoded) {
      rx->starts = this->note_boot_id_(info.boot_id);
      this->info_ = info;
      this->have_info_ = true;
      if (this->have_status_ && info.boot_id == this->status_.boot_id) {
        this->current_info_ = info;
        this->have_current_info_ = true;
      }
    }
  } else if (rx->type == MSG_BOOT_STATUS) {
    rx->decoded = decode_boot_status(rx->payload, rx->len, &this->boot_);
    if (rx->decoded) {
      this->have_boot_ = true;
      this->last_boot_ms_ = this->now_;
      this->last_seen_boot_ = true;
    }
  }
  if (!rx->decoded) {
    this->have_unreadable_ = true;
    this->last_unreadable_ms_ = this->now_;
  }
  this->restarts_ += rx->starts;
}

inline uint8_t McuLink::note_boot_id_(uint8_t boot_id) {
  const bool changed = this->have_boot_id_ && boot_id != this->boot_id_;
  this->have_boot_id_ = true;
  this->boot_id_ = boot_id;
  return changed ? 1 : 0;
}

inline LinkState McuLink::link_state() const {
  if (this->have_status_ && this->elapsed_(this->last_status_ms_) < detail::APP_SEEN_MS)
    return LinkState::APPLICATION;
  if (this->have_boot_ && this->elapsed_(this->last_boot_ms_) < detail::BOOT_SEEN_MS)
    return LinkState::BOOTLOADER;
  return LinkState::NO_RESPONSE;
}

inline bool McuLink::unreadable_frames() const {
  return this->have_unreadable_ && this->elapsed_(this->last_unreadable_ms_) < detail::UNREADABLE_SEEN_MS;
}

inline bool McuLink::have_current_info() const {
  return this->have_current_info_ && this->have_status_ && this->current_info_.boot_id == this->status_.boot_id;
}

inline void TextOut::add(const char *fmt, ...) {
  if (this->used + 1 >= this->size)
    return;
  va_list args;
  va_start(args, fmt);
  const int n = vsnprintf(this->buf + this->used, this->size - this->used, fmt, args);
  va_end(args);
  if (n > 0)
    this->used = (this->used + n < this->size) ? this->used + n : this->size - 1;
}

inline FirmwareText firmware_text(uint16_t version, uint32_t build) {
  FirmwareText t;
  snprintf(t.str, sizeof(t.str), "%04X build %08lX", version, static_cast<unsigned long>(build));
  return t;
}

inline FirmwareAge firmware_age(const Info &mcu, uint16_t version, uint32_t build, uint8_t boot_version) {
  if (mcu.boot_version > boot_version || mcu.version > version)
    return FirmwareAge::NEWER;
  if (mcu.boot_version < boot_version)
    return FirmwareAge::BOOT_OLD;
  return mcu.build == build ? FirmwareAge::CURRENT : FirmwareAge::APP_OLD;
}

inline const char *reset_cause_name(uint8_t reset_cause) {
  switch (reset_cause) {
    case RESET_POWER_ON:
      return "Power-on";
    case RESET_WATCHDOG:
      return "Watchdog";
    default:
      return "Other";
  }
}

inline uint16_t crc16(const uint8_t *data, size_t len, uint16_t crc) {
  for (size_t i = 0; i < len; i++) {
    crc ^= static_cast<uint16_t>(data[i] << 8);
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u) : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

inline bool decode_status(const uint8_t *p, uint8_t len, Status *out) {
  if (len != STATUS_LEN)
    return false;
  out->state = p[SF_STATE];
  out->faults = p[SF_FAULTS];
  out->buttons = p[SF_BUTTONS];
  out->target = detail::get16(p + SF_TARGET);
  out->current = detail::get16(p + SF_CURRENT);
  out->half = detail::get16(p + SF_HALF);
  out->ks_min = detail::get16(p + SF_KS_MIN);
  out->flags = p[SF_FLAGS];
  out->therm = detail::get16(p + SF_THERM);
  out->wheel = detail::get16(p + SF_WHEEL);
  out->boot_id = p[SF_BOOT_ID];
  out->leds = p[SF_LEDS];
  out->night = p[SF_NIGHT];
  return true;
}

inline bool decode_info(const uint8_t *p, uint8_t len, Info *out) {
  if (len != INFO_LEN)
    return false;
  out->version = detail::get16(p);
  out->build = detail::get32(p + 2);
  out->reset_cause = p[6];
  out->wdt_count = p[7];
  out->boot_version = p[8];
  out->boot_id = p[9];
  return true;
}

inline bool decode_boot_status(const uint8_t *p, uint8_t len, BootStatus *out) {
  if (len != BOOT_STATUS_LEN)
    return false;
  out->version = p[0];
  out->app_valid = p[1];
  out->reason = p[2];
  out->result = p[3];
  out->next = detail::get16(p + 4);
  return true;
}

inline size_t build_frame(uint8_t *out, uint8_t type, const uint8_t *payload, uint8_t len) {
  out[0] = FRAME_START;
  out[1] = type;
  out[2] = len;
  if (len)
    memmove(out + 3, payload, len);
  uint16_t crc = crc16(out + 1, len + 2u);
  out[3 + len] = static_cast<uint8_t>(crc);
  out[4 + len] = static_cast<uint8_t>(crc >> 8);
  return len + FRAME_OVERHEAD;
}

template<typename F> void FrameParser::feed(uint8_t byte, F &&on_frame) {
  this->window_[this->used_++] = byte;
  this->take_frames_(on_frame);
}

template<typename F> void FrameParser::flush(F &&on_frame) {
  while (!this->idle()) {
    this->drop_(1);
    this->take_frames_(on_frame);
  }
}

template<typename F> void FrameParser::take_frames_(F &&on_frame) {
  for (;;) {
    switch (this->scan_()) {
      case Scan::NEED_MORE:
        return;
      case Scan::BAD:
        this->drop_(1);
        break;
      case Scan::GOOD: {
        const uint8_t type = this->window_[1], len = this->window_[2];
        on_frame(type, static_cast<const uint8_t *>(this->window_ + 3), len);
        this->drop_(len + FRAME_OVERHEAD);
        break;
      }
    }
  }
}

inline FrameParser::Scan FrameParser::scan_() const {
  const uint8_t *w = this->window_;
  if (this->used_ == 0)
    return Scan::NEED_MORE;
  if (w[0] != FRAME_START)
    return Scan::BAD;
  if (this->used_ < 2)
    return Scan::NEED_MORE;
  if (!type_known_(w[1]))
    return Scan::BAD;
  if (this->used_ < 3)
    return Scan::NEED_MORE;
  const size_t len = w[2];
  if (this->used_ < len + FRAME_OVERHEAD)
    return Scan::NEED_MORE;
  const uint16_t crc = crc16(w + 1, len + 2);
  return detail::get16(w + 3 + len) == crc ? Scan::GOOD : Scan::BAD;
}

inline void FrameParser::drop_(size_t n) {
  this->used_ -= n;
  memmove(this->window_, this->window_ + n, this->used_);
}

inline bool FrameParser::type_known_(uint8_t type) {
  return type == MSG_STATUS || type == MSG_INFO || type == MSG_BOOT_STATUS;
}

}
