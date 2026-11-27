// RP-STD01 MCU tool: decides which way to write the MCU, and writes the page text.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "esphome/components/rpstd01_link/rpstd01_link.h"
#include "esphome/components/rpstd01_tool/rpstd01_mcuflash.h"
#include "esphome/components/rpstd01_tool/rpstd01_stc_isp.h"
#include "esphome/components/rpstd01_tool/rpstd01_tool_update.h"

namespace rpstd01tool {

inline constexpr uint16_t MCU_MODEL = 0xF562;  // IAP15W413AS
inline constexpr uint32_t MCU_USER_HZ = rpstd01::SYSCLK_HZ;
// The rate STC's bootloader erases and writes at: stcgal's, as in the stcgal sessions the tests replay.
inline constexpr uint32_t STC_TRANSFER_BAUD = 115200;
inline constexpr uint32_t MCU_CONNECT_US = 1500000;
inline constexpr uint32_t MCU_SYNC_GAP_US = 10000;
inline constexpr uint32_t MCU_VERIFY_US = 1500000;

inline constexpr uint32_t CHECK_ASK_EVERY_MS = 700;
inline constexpr uint32_t CHECK_LISTEN_MS = 3000;
// A boot status does not show the application's version: a valid application found in update mode
// is started, CHECK_ASK_EVERY_MS apart, until its info frame shows whether it is newer.
inline constexpr uint8_t CHECK_RUN_TRIES = 3;
// Updates started by the tool itself after one failed and left no valid application.
inline constexpr uint8_t REPAIR_MAX = 3;
// ESPHome's safe_mode marks the running image valid after this (boot_is_good_after's default).
inline constexpr uint32_t SAFE_MODE_GOOD_AFTER_S = 60;

// The image the tool carries: bootloader and application from address 0, for STC's bootloader. Its
// application area (app_area()) is what an update through our bootloader writes.
struct Carried {
  const uint8_t *image;
  uint32_t size;
  uint32_t crc32;
  uint16_t version;
  uint32_t build;
  uint8_t boot_version;
};

inline const uint8_t *app_area(const Carried &img) {
  return img.size >= static_cast<uint32_t>(rpstd01::APP_BASE) + rpstd01::APP_SIZE ? img.image + rpstd01::APP_BASE
                                                                                   : nullptr;
}

// The last install through STC's bootloader, kept in NVS.
struct IspRecord {
  uint8_t connect_result;  // stcisp::Result, or mcuflash::RESULT_UNFINISHED
  uint8_t program_result;  // stcisp::Result, or mcuflash::RESULT_NOT_STARTED
  uint8_t stage;
  uint8_t app_seen;  // the new application's info frame arrived
  uint8_t app_boot_version;
  uint16_t app_version;
  uint32_t written;
  uint32_t app_build;
};

// A stored record of another size or version is no record.
inline constexpr uint8_t ISP_RECORD_VERSION = 1;
static_assert(stcisp::RESULT_BAD_IMAGE == 10 && stcisp::STAGE_DONE == 9,
              "stored results and stages were renumbered: raise ISP_RECORD_VERSION, then these numbers");
inline constexpr size_t ISP_RECORD_SIZE = 16;

inline void encode_record(const IspRecord &r, uint8_t out[ISP_RECORD_SIZE]) {
  out[0] = ISP_RECORD_VERSION;
  out[1] = r.connect_result;
  out[2] = r.program_result;
  out[3] = r.stage;
  out[4] = r.app_seen;
  out[5] = r.app_boot_version;
  rpstd01::detail::put16(out + 6, r.app_version);
  rpstd01::detail::put32(out + 8, r.written);
  rpstd01::detail::put32(out + 12, r.app_build);
}

inline bool decode_record(const uint8_t *in, size_t len, IspRecord *r) {
  if (len != ISP_RECORD_SIZE || in[0] != ISP_RECORD_VERSION)
    return false;
  r->connect_result = in[1];
  r->program_result = in[2];
  r->stage = in[3];
  r->app_seen = in[4];
  r->app_boot_version = in[5];
  r->app_version = rpstd01::detail::get16(in + 6);
  r->written = rpstd01::detail::get32(in + 8);
  r->app_build = rpstd01::detail::get32(in + 12);
  return true;
}

inline uint8_t refusal(const IspRecord &r) { return mcuflash::refusal(r.connect_result, r.program_result); }

// NVS as the tool uses it: the arming Store (mcuflash::StoreIo) and the last install.
class ToolStore : public mcuflash::StoreIo {
 public:
  virtual bool load_record(IspRecord *rec) = 0;
  virtual bool save_record(const IspRecord &rec) = 0;  // saves and commits
};

// The serial port at power-on: STC's bootloader (stcisp::Io, 8E1), then our application's link.
class PowerOnIo : public stcisp::Io {
 public:
  virtual void begin() = 0;
  virtual void set_no_parity() = 0;  // after set_baud(): 8N1, as our application's link
};

// The power-on step: takes the one-shot flag, runs the install, listens for the new application, and
// records the result. Returns true if an attempt ran (*rec then holds it).
template<typename Prog = stcisp::Programmer>
bool power_on(ToolStore &store, PowerOnIo &io, const Carried &img, IspRecord *rec) {
  IspRecord last;
  mcuflash::Taken taken;
  if (!mcuflash::power_on_take(store, img.crc32, store.load_record(&last) ? refusal(last) : 0, &taken))
    return false;

  *rec = IspRecord{mcuflash::RESULT_UNFINISHED, mcuflash::RESULT_NOT_STARTED, 0, 0, 0, 0, 0, 0};
  // Saved now and again at the end, so an attempt that never finishes (a crash, or the power going
  // off again) still leaves a record.
  store.save_record(*rec);

  io.begin();
  Prog prog(io);
  rec->connect_result = prog.connect(MCU_MODEL, MCU_CONNECT_US, MCU_SYNC_GAP_US);
  if (rec->connect_result == stcisp::RESULT_OK)
    rec->program_result = prog.program(img.image, img.size, MCU_USER_HZ, STC_TRANSFER_BAUD);
  rec->stage = prog.session().stage;
  rec->written = prog.session().written;
  if (mcuflash::should_disconnect(prog.session().status_len > 0, rec->stage, rec->program_result, rec->written))
    prog.disconnect();
  // The new application sends its info frame at start.
  io.set_baud(rpstd01::LINK_BAUD);
  io.set_no_parity();
  rpstd01::Info app;
  if (rec->program_result == stcisp::RESULT_OK && mcuflash::listen_for_info(io, MCU_VERIFY_US, &app)) {
    rec->app_seen = 1;
    rec->app_version = app.version;
    rec->app_build = app.build;
    rec->app_boot_version = app.boot_version;
  }
  store.save_record(*rec);
  mcuflash::power_on_done(store, taken,
                          mcuflash::Attempt{rec->connect_result, rec->program_result, rec->stage, rec->written});
  return true;
}

// The MCU as the check saw it, or as it was when the tool last decided.
struct Seen {
  bool info;                 // our application's info frame
  rpstd01::Info app;         // valid if info
  bool boot_status;          // our bootloader's boot status (update mode), and no info frame
  rpstd01::BootStatus boot;  // valid if boot_status
  bool unreadable;           // answers in our protocol, but no info frame or boot status this tool reads
};

enum Verdict : uint8_t {
  VERDICT_CHECKING = 0,
  VERDICT_CURRENT,      // our application at the carried build, bootloader current
  VERDICT_APP_OLD,      // our application, another build; bootloader current
  VERDICT_BOOT_OLD,     // our application; bootloader older than carried
  VERDICT_UPDATE_MODE,  // our bootloader in update mode (boot status), not newer than carried, with no
                        // valid application or one that did not start when asked
  VERDICT_NO_ANSWER,    // nothing valid: factory firmware, no firmware, or a dead link
  VERDICT_NEWER,        // a newer bootloader, or a newer application release, than carried
  VERDICT_LINK_FAULT,   // no answer, but the application this power-on installed answered just after
  VERDICT_UNKNOWN_APP,  // answers, but no info frame or boot status this tool reads: a release it does not know
};

// Verdicts that do not arm an install by themselves: the MCU holds complete firmware that this tool
// must not replace on its own, or an application whose version is unknown. The button still works.
inline bool holds_auto_install(Verdict v, const Seen &s) {
  return v == VERDICT_CURRENT || v == VERDICT_NEWER || v == VERDICT_UNKNOWN_APP ||
         (v == VERDICT_UPDATE_MODE && s.boot.app_valid);
}

// ran: the record of the install this power-on made, or nullptr.
inline Verdict classify(const Seen &s, const Carried &img, const IspRecord *ran = nullptr) {
  if (s.info) {
    switch (rpstd01::firmware_age(s.app, img.version, img.build, img.boot_version)) {
      case rpstd01::FirmwareAge::NEWER:
        return VERDICT_NEWER;
      case rpstd01::FirmwareAge::BOOT_OLD:
        return VERDICT_BOOT_OLD;
      case rpstd01::FirmwareAge::APP_OLD:
        return VERDICT_APP_OLD;
      default:
        return VERDICT_CURRENT;
    }
  }
  if (s.boot_status)
    return s.boot.version > img.boot_version ? VERDICT_NEWER : VERDICT_UPDATE_MODE;
  if (s.unreadable)
    return VERDICT_UNKNOWN_APP;
  if (ran != nullptr && ran->program_result == stcisp::RESULT_OK && ran->app_seen && ran->app_build == img.build &&
      ran->app_boot_version >= img.boot_version)
    return VERDICT_LINK_FAULT;
  return VERDICT_NO_ANSWER;
}

// True if the verdict is handled through our bootloader over the link, at once and with no power
// cycle, rather than by an STC install at the next power-on. In update mode only an application
// that is not valid is replaced: a valid one may be newer than the carried one.
inline bool link_update_handles(Verdict v, const Seen &s, const Carried &img) {
  if (v == VERDICT_APP_OLD)
    return true;
  return v == VERDICT_UPDATE_MODE && s.boot_status && s.boot.version == img.boot_version && !s.boot.app_valid;
}

inline const char *result_name(uint8_t r) {
  static const char *const NAMES[] = {"ok",          "no reply",           "timeout",        "framing",     "bad reply",
                                      "wrong model", "unexpected options", "external clock", "calibration", "locked",
                                      "bad image"};
  static_assert(sizeof(NAMES) / sizeof(NAMES[0]) == stcisp::RESULT_BAD_IMAGE + 1, "a name for each stcisp::Result");
  if (r == mcuflash::RESULT_NOT_STARTED)
    return "not started";
  if (r == mcuflash::RESULT_UNFINISHED)
    return "did not finish";
  return r < sizeof(NAMES) / sizeof(NAMES[0]) ? NAMES[r] : "?";
}

inline const char *stage_name(uint8_t s) {
  static const char *const NAMES[] = {"sync",  "status", "calibrate", "baud",    "handshake",
                                      "erase", "write",  "finish",    "options", "done"};
  return s < sizeof(NAMES) / sizeof(NAMES[0]) ? NAMES[s] : "?";
}

using rpstd01::firmware_text;
using rpstd01::TextOut;

// The tool: the check at start, the application updates it starts, the buttons, and the page's text.
class Tool {
 public:
  void begin(ToolStore *store, const Carried &img, const IspRecord *ran) {
    this->store_ = store;
    this->img_ = img;
    this->ran_ = ran != nullptr;
    if (ran != nullptr)
      this->ran_rec_ = *ran;
    this->store_->load(&this->nvs_);
    if (const uint8_t *area = app_area(img))
      this->updater_.set_image(area, img.build, img.boot_version);
  }

  template<typename U> void loop(U &uart, uint32_t now_ms);

  // Update Application Now: writes the carried application through our bootloader, also over a
  // current or newer one. Refused when the check saw a bootloader other than the carried one: the
  // page then says that only an STC install replaces it. One the check did not see is checked by the
  // update itself before it erases anything.
  bool update_application() {
    if (!this->decided() || !this->updater_.has_image() || this->updater_.update_running())
      return false;
    this->update_refused_ = this->boot_known_() && !this->boot_current_();
    if (this->update_refused_) {
      RPSTD01TOOL_LOG("Update Application Now refused: the MCU's bootloader is not the carried one");
      return false;
    }
    RPSTD01TOOL_LOG("Update Application Now: writing the carried application through our bootloader");
    this->updater_.start_update();
    return true;
  }
  const AppUpdater &updater() const { return this->updater_; }

  // A button: arm or disarm. False if NVS could not be written.
  bool command(mcuflash::Command c) {
    const bool ok = mcuflash::run_command(*this->store_, c, this->img_.crc32);
    // Also after a failure, which may have written part of the change.
    if (this->decided())
      this->decide_();
    else
      this->store_->load(&this->nvs_);
    return ok;
  }

  // True while ESP-IDF has not yet marked this image valid: the page then asks to wait.
  void set_image_pending(bool pending) { this->image_pending_ = pending; }
  bool image_pending() const { return this->image_pending_; }

  uint8_t state() const { return this->state_; }
  bool decided() const { return this->state_ > mcuflash::CHECK_ASK; }
  Verdict verdict() const { return this->verdict_; }
  const mcuflash::Store &nvs() const { return this->nvs_; }
  // The last attempt, from this power-on or from NVS.
  bool last(IspRecord *rec) const {
    if (this->ran_) {
      *rec = this->ran_rec_;
      return true;
    }
    return this->store_->load_record(rec);
  }

  void state_text(char *buf, size_t size) const;
  void installed_text(char *buf, size_t size) const;
  void carried_text(char *buf, size_t size) const;
  void armed_text(char *buf, size_t size) const;
  void last_install_text(char *buf, size_t size) const;
  void update_text(char *buf, size_t size) const { this->updater_.update_text(buf, size); }
  float update_progress_percent() const { return this->updater_.update_progress_percent(); }

 protected:
  // The MCU as the shared receiver has it now: the info frame if one came, else the boot status
  // unless something newer answered.
  void take_snapshot_() {
    const bool info = this->updater_.have_info();
    const bool boot = !info && this->updater_.have_boot_status() && this->boot_status_newest_;
    const bool answers =
        this->updater_.link_state() == rpstd01::LinkState::APPLICATION || this->updater_.unreadable_frames();
    this->seen_ = Seen{info, this->updater_.info(), boot, this->updater_.boot_status(), !info && !boot && answers};
  }
  void take_boot_snapshot_() { this->seen_ = Seen{false, {}, true, this->updater_.boot_status(), false}; }

  bool app_to_start_() const {
    const rpstd01::BootStatus &bs = this->updater_.boot_status();
    return !this->updater_.have_info() && this->updater_.have_boot_status() && this->boot_status_newest_ &&
           bs.app_valid && bs.version <= this->img_.boot_version;
  }
  // Started by MSG_BOOT_RUN and answering in frames this tool reads, but without both its info frame
  // and a status yet. t: since the check started.
  bool app_starting_(uint32_t t) const {
    const bool info_and_status =
        this->updater_.have_info() && this->updater_.link_state() == rpstd01::LinkState::APPLICATION;
    return this->runs_ > 0 && !this->boot_status_newest_ && !this->updater_.unreadable_frames() && !info_and_status &&
           t - this->run_ms_ < CHECK_LISTEN_MS;
  }

  void decide_() {
    this->update_refused_ = false;
    this->verdict_ = classify(this->seen_, this->img_, this->ran_ ? &this->ran_rec_ : nullptr);
    // A link fault holds the install, but changes nothing in NVS: the count of failed automatic
    // attempts is not reset, so an application that answers once and then hangs still reaches the
    // limit. An update through our bootloader does the same until its result is known: a good result
    // checks again and finds the MCU current.
    this->nvs_kept_ = this->verdict_ == VERDICT_LINK_FAULT || this->link_handles_();
    if (this->nvs_kept_) {
      this->store_->load(&this->nvs_);
      this->refused_ = 0;
      this->state_ = mcuflash::CHECK_CURRENT;
      return;
    }
    IspRecord rec;
    this->refused_ = this->last(&rec) ? refusal(rec) : 0;
    this->state_ = mcuflash::check(*this->store_, holds_auto_install(this->verdict_, this->seen_), this->refused_,
                                   this->img_.crc32, &this->nvs_);
  }

  // Once the automatic repair is used up, an MCU with no valid application is left to an STC install.
  bool link_handles_() const {
    if (this->repair_used_up_() && this->seen_.boot_status && !this->seen_.boot.app_valid)
      return false;
    return this->updater_.has_image() && link_update_handles(this->verdict_, this->seen_, this->img_);
  }
  bool repair_used_up_() const { return this->repairs_ > REPAIR_MAX; }
  uint8_t boot_version_seen_() const {
    return this->seen_.info ? this->seen_.app.boot_version : this->seen_.boot.version;
  }
  bool boot_known_() const { return this->seen_.info || this->seen_.boot_status; }
  bool boot_current_() const { return this->boot_known_() && this->boot_version_seen_() == this->img_.boot_version; }

  void log_state_() const {
    char text[512];
    this->state_text(text, sizeof(text));
    RPSTD01TOOL_LOG("%s", text);
  }

  // After the check's decision: our bootloader's status, with no update running.
  void on_boot_status_() {
    const rpstd01::BootStatus &bs = this->updater_.boot_status();
    if (bs.version != this->img_.boot_version)
      return;
    if (this->verdict_ == VERDICT_CURRENT && bs.reason == rpstd01::BOOT_REASON_WATCHDOG) {
      this->take_boot_snapshot_();
      this->decide_();
      if (this->crash_rewritten_) {
        RPSTD01TOOL_LOG("The application crashed again after it was written: stopping");
        this->log_state_();
        return;
      }
      RPSTD01TOOL_LOG("The application was reset by the watchdog %u times: writing it once more",
                      rpstd01::BOOT_WDT_RESETS);
      this->crash_rewritten_ = true;
      this->updater_.start_update();
    } else if (!bs.app_valid && this->updater_.update_state() == UpdateState::FAILED &&
               this->state_ == mcuflash::CHECK_CURRENT) {
      this->take_boot_snapshot_();
      if (++this->repairs_ <= REPAIR_MAX) {
        RPSTD01TOOL_LOG("No valid application after the failed update: repair %u of %u", this->repairs_, REPAIR_MAX);
        this->updater_.start_update();
        return;
      }
      this->decide_();
      this->log_state_();
    }
  }

  void act_on_verdict_() {
    if (!this->link_handles_())
      return;
    RPSTD01TOOL_LOG("Updating the MCU's application through our bootloader");
    this->updater_.start_update();
  }

  bool link_text_(TextOut *out, const char *reason, bool armed) const;
  void current_text_(TextOut *out, const char *reason, bool armed) const;
  // Fixed text, or buf when the text holds a number.
  const char *reason_(char *buf, size_t size) const;

  ToolStore *store_{nullptr};
  Carried img_{};
  bool ran_{false};
  IspRecord ran_rec_{};
  mcuflash::Store nvs_{0, mcuflash::USER_NONE, 0, 0};
  AppUpdater updater_;
  Seen seen_{};
  bool started_{false};
  uint32_t since_ms_{0};
  uint32_t asks_{0};
  uint8_t runs_{0};
  uint32_t run_ms_{0};
  uint8_t state_{mcuflash::CHECK_ASK};
  Verdict verdict_{VERDICT_CHECKING};
  uint8_t refused_{0};
  bool image_pending_{false};
  UpdateState last_upd_{UpdateState::IDLE};
  bool crash_rewritten_{false};  // the carried application was written for a watchdog reset
  uint8_t repairs_{0};
  bool update_refused_{false};  // Update Application Now was refused for another bootloader
  bool nvs_kept_{false};        // the last decision left NVS as it was
  bool boot_status_newest_{false};
};

template<typename U> void Tool::loop(U &uart, uint32_t now_ms) {
  if (!this->started_) {
    this->started_ = true;
    this->since_ms_ = now_ms;
  }
  this->updater_.receive(uart, now_ms, [this](const rpstd01::Received &rx) {
    this->boot_status_newest_ = rx.type == rpstd01::MSG_BOOT_STATUS && rx.decoded;
    if (this->decided() && this->boot_status_newest_ && !this->updater_.update_running())
      this->on_boot_status_();
  });

  switch (this->state_) {
    case mcuflash::CHECK_ASK: {
      const uint32_t t = now_ms - this->since_ms_;
      if (this->app_to_start_()) {
        if (this->runs_ < CHECK_RUN_TRIES && (this->runs_ == 0 || t - this->run_ms_ >= CHECK_ASK_EVERY_MS)) {
          uint8_t frame[rpstd01::FRAME_OVERHEAD];
          if (this->runs_ == 0)
            RPSTD01TOOL_LOG("MCU: update mode with a valid application: starting it to read its version");
          uart.write_array(frame, rpstd01::build_frame(frame, rpstd01::MSG_BOOT_RUN, nullptr, 0));
          this->runs_++;
          this->run_ms_ = t;
        }
        if (this->runs_ < CHECK_RUN_TRIES || t - this->run_ms_ < CHECK_ASK_EVERY_MS)
          return;
      } else if (this->app_starting_(t) ||
                 (!this->updater_.have_info() && !this->updater_.have_boot_status() && t < CHECK_LISTEN_MS)) {
        if (t >= this->asks_ * CHECK_ASK_EVERY_MS) {
          uint8_t frame[rpstd01::FRAME_OVERHEAD];
          uart.write_array(frame, rpstd01::build_frame(frame, rpstd01::MSG_INFO_REQ, nullptr, 0));
          this->asks_++;
        }
        return;
      }
      this->take_snapshot_();
      this->decide_();
      if (this->seen_.info) {
        RPSTD01TOOL_LOG("MCU: application %s, bootloader v%u; carried %s, bootloader v%u",
                        firmware_text(this->seen_.app.version, this->seen_.app.build).str, this->seen_.app.boot_version,
                        firmware_text(this->img_.version, this->img_.build).str, this->img_.boot_version);
      } else if (this->seen_.boot_status) {
        RPSTD01TOOL_LOG("MCU: our bootloader v%u in update mode, application %s", this->seen_.boot.version,
                        this->seen_.boot.app_valid ? "valid" : "not valid");
      } else if (this->seen_.unreadable) {
        RPSTD01TOOL_LOG("MCU: answers, but with no info frame or boot status this tool reads");
      } else {
        RPSTD01TOOL_LOG("MCU: no answer in %u ms", (unsigned) CHECK_LISTEN_MS);
      }
      this->log_state_();
      this->act_on_verdict_();
      break;
    }
    default:
      break;
  }
  const UpdateState u = this->updater_.update_state();
  if (u != this->last_upd_) {
    this->last_upd_ = u;
    if (u == UpdateState::DONE) {
      this->take_snapshot_();
      this->decide_();
      this->log_state_();
    }
  }
  this->updater_.service(uart);
}

inline const char *Tool::reason_(char *buf, size_t size) const {
  switch (this->verdict_) {
    case VERDICT_CURRENT:
      return "The MCU runs the carried application and bootloader";
    case VERDICT_APP_OLD:
      return "The MCU's application is another build (its bootloader is current)";
    case VERDICT_BOOT_OLD:
      return "The MCU's bootloader is older than the carried one";
    case VERDICT_UPDATE_MODE:
      if (this->seen_.boot.version < this->img_.boot_version)
        return "The MCU is in update mode, with a bootloader older than the carried one";
      if (!this->seen_.boot.app_valid)
        return "The MCU is in our bootloader's update mode with no valid application";
      if (this->seen_.boot.reason == rpstd01::BOOT_REASON_WATCHDOG) {
        snprintf(buf, size,
                 "The MCU is in our bootloader's update mode: its application was reset by the watchdog %u times",
                 rpstd01::BOOT_WDT_RESETS);
        return buf;
      }
      return "The MCU is in our bootloader's update mode with a valid application";
    case VERDICT_NO_ANSWER:
      return "The MCU does not answer our protocol (factory firmware, or none)";
    case VERDICT_NEWER: {
      if (!this->seen_.info)
        return "The MCU's bootloader is newer than this tool's";
      const bool boot_newer = this->seen_.app.boot_version > this->img_.boot_version;
      const bool app_newer = this->seen_.app.version > this->img_.version;
      if (boot_newer && app_newer)
        return "The MCU's application and bootloader are newer than this tool's";
      if (boot_newer)
        return "The MCU's bootloader is newer than this tool's";
      if (this->seen_.app.boot_version == this->img_.boot_version)
        return "The MCU's application is newer than this tool's";
      return "The MCU's application is newer than this tool's, its bootloader older";
    }
    case VERDICT_LINK_FAULT:
      return "Link fault: the MCU answered after the install, but not now";
    case VERDICT_UNKNOWN_APP:
      return "The MCU runs firmware this tool does not understand: it answers, but with no info frame or boot status "
             "this tool reads";
    default:
      return "Checking";
  }
}

inline void Tool::state_text(char *buf, size_t size) const {
  static const char POWER_CYCLE[] =
      "press the RESET button. Wi-Fi then takes about 10 s longer than after an ordinary restart to come back.";
  static const char TO_INSTALL[] =
      "To install, press Install at Next Power-On, then press the RESET button.";
  char reason_buf[128];
  const char *reason = this->reason_(reason_buf, sizeof(reason_buf));
  TextOut out{buf, size};
  if (this->image_pending_)
    out.add("Wait until this tool has run for %u s before pressing RESET. ", (unsigned) SAFE_MODE_GOOD_AFTER_S);
  switch (this->state_) {
    case mcuflash::CHECK_ASK:
      out.add("Checking...");
      break;
    case mcuflash::CHECK_CURRENT: {
      const bool armed = this->nvs_.arm == this->img_.crc32;
      if (!this->link_text_(&out, reason, armed))
        this->current_text_(&out, reason, armed);
      if (armed && this->nvs_.user == mcuflash::USER_ARMED) {
        out.add("%sInstall at Next Power-On is set, so this tool's firmware is written at the next power-on; "
                "Cancel Install cancels that.",
                this->verdict_ == VERDICT_CURRENT ? "Everything is current. " : " ");
      } else if (armed && this->nvs_kept_) {
        out.add(" An install armed by an earlier check is still set, so this tool's firmware is written at the next "
                "power-on; Cancel Install clears it.");
      } else if (armed) {
        out.add("%sAn old install flag could not be cleared (NVS write failed), so this tool's firmware is "
                "written at the next power-on; Cancel Install clears it.",
                this->verdict_ == VERDICT_CURRENT ? "Everything is current. " : " ");
      }
      break;
    }
    case mcuflash::CHECK_ARMED:
      out.add("%s. Install armed: %s", reason, POWER_CYCLE);
      break;
    case mcuflash::CHECK_ARM_FAILED:
      out.add("%s, and the install could not be armed (NVS write failed). Restart to try again, or press Install "
              "at Next Power-On.",
              reason);
      break;
    case mcuflash::CHECK_REFUSED:
      out.add("%s, but this tool cannot install on this unit (%s). Nothing on the MCU was changed. Trying again "
              "gives the same refusal unless the hardware differs; Install at Next Power-On tries again.",
              reason, result_name(this->refused_));
      break;
    case mcuflash::CHECK_DISARMED:
      out.add("%s. Automatic install is off (Cancel Install). %s", reason, TO_INSTALL);
      break;
    case mcuflash::CHECK_GAVE_UP:
      out.add("%s. The last %u automatic attempts failed, so no more are made. %s", reason, mcuflash::MAX_AUTO_FAILS,
              TO_INSTALL);
      break;
    default:
      break;
  }
  if (this->state_ > mcuflash::CHECK_CURRENT && this->updater_.update_state() != UpdateState::IDLE) {
    char text[96];
    this->updater_.update_text(text, sizeof(text));
    out.add(" Application update through our bootloader: %s.", text);
    if (this->repair_used_up_())
      out.add(" The MCU has no valid application, and the automatic repair is used up.");
  }
  if (this->update_refused_) {
    out.add(" Update Application Now cannot write this MCU: the update over the link needs the carried bootloader "
            "(v%u), and the MCU has v%u. Only an install through STC's bootloader replaces it: press Install at Next "
            "Power-On, then press RESET.",
            this->img_.boot_version, this->boot_version_seen_());
  }
  if (this->nvs_.partial && this->state_ > mcuflash::CHECK_CURRENT)
    out.add(" Warning: the MCU firmware is incomplete; the dimmer does not work until it is installed.");
}

inline void Tool::current_text_(TextOut *out, const char *reason, bool armed) const {
  if (this->verdict_ == VERDICT_NEWER && armed) {
    out->add("%s.", reason);
  } else if (this->verdict_ == VERDICT_NEWER && this->boot_current_()) {
    out->add("%s, so nothing is armed. Update Application Now forces this tool's application; Install at Next "
             "Power-On forces its application and bootloader.",
             reason);
  } else if (this->verdict_ == VERDICT_NEWER) {
    out->add("%s, so nothing is armed. Install at Next Power-On forces this tool's application and bootloader.",
             reason);
  } else if (this->verdict_ == VERDICT_UNKNOWN_APP) {
    out->add("%s.", reason);
    if (!armed)
      out->add(" Nothing is armed. Install at Next Power-On forces an install of this tool's firmware.");
  } else if (this->verdict_ == VERDICT_UPDATE_MODE && this->seen_.boot.app_valid) {
    out->add("%s. It did not start when asked, so its version is unknown", reason);
    if (!armed)
      out->add(" and nothing is armed. Install at Next Power-On forces an install of this tool's firmware.");
    else
      out->add(".");
  } else if (this->verdict_ == VERDICT_LINK_FAULT) {
    out->add("%s. Nothing is armed: check the link (GPIO20/21), or restart the tool to check again.", reason);
  } else if (!armed) {
    out->add("Everything is current: install the main configuration.");
  }
}

// The state text while an update through our bootloader runs or after it ended; false if there is none.
inline bool Tool::link_text_(TextOut *out, const char *reason, bool armed) const {
  if (this->updater_.update_running()) {
    char text[96];
    this->updater_.update_text(text, sizeof(text));
    out->add("%s. Updating the application through our bootloader (no power cycle needed): %s.", reason, text);
    return true;
  }
  switch (this->updater_.update_state()) {
    case UpdateState::FAILED:
      out->add("%s. The application update through our bootloader failed: %s. Press Update Application Now to "
               "try again, or Install at Next Power-On to install through STC's bootloader at the next power-on.",
               reason, this->updater_.fail_reason());
      return true;
    case UpdateState::DONE:
      if (this->verdict_ == VERDICT_UPDATE_MODE) {
        // Our bootloader came back after a write that ended well: the application crashed again.
        out->add("%s. It crashed again after it was written, so the tool stops: the firmware needs a fix. Update "
                 "Application Now writes it again.",
                 reason);
        return true;
      }
      if (this->verdict_ != VERDICT_CURRENT || armed)
        return false;
      out->add("Application updated. Everything is current: install the main configuration.");
      return true;
    default:
      return false;
  }
}

inline void Tool::installed_text(char *buf, size_t size) const {
  const Seen &s = this->seen_;
  if (s.info) {
    snprintf(buf, size, "Application %s, bootloader v%u", firmware_text(s.app.version, s.app.build).str,
             s.app.boot_version);
  } else if (s.boot_status) {
    snprintf(buf, size, "Bootloader v%u in update mode, application %s", s.boot.version,
             s.boot.app_valid ? "valid" : "not valid");
  } else if (s.unreadable) {
    snprintf(buf, size, "Answers, but no info frame or boot status this tool reads");
  } else if (!this->decided()) {
    snprintf(buf, size, "Checking...");
  } else {
    snprintf(buf, size, "No answer at start");
  }
}

inline void Tool::carried_text(char *buf, size_t size) const {
  snprintf(buf, size, "Application %s, bootloader v%u (%u bytes, CRC-32 %08X)",
           firmware_text(this->img_.version, this->img_.build).str, this->img_.boot_version, (unsigned) this->img_.size,
           (unsigned) this->img_.crc32);
}

inline void Tool::armed_text(char *buf, size_t size) const {
  static const char *const USERS[] = {"none", "Cancel Install", "Install at Next Power-On"};
  const mcuflash::Store &s = this->nvs_;
  const char *armed = s.arm == 0 ? "Nothing" : s.arm == this->img_.crc32 ? "Install at next power-on" : "Stale flag";
  snprintf(buf, size, "%s. Last choice: %s. Failed automatic attempts: %u. Firmware incomplete: %s", armed,
           USERS[s.user < sizeof(USERS) / sizeof(USERS[0]) ? s.user : 0], s.fails, s.partial ? "yes" : "no");
}

inline void Tool::last_install_text(char *buf, size_t size) const {
  IspRecord r;
  TextOut out{buf, size};
  if (!this->last(&r)) {
    out.add("None");
    return;
  }
  if (const uint8_t refused = refusal(r)) {
    out.add("Install refused: %s. Nothing was changed", result_name(refused));
  } else if (r.connect_result != stcisp::RESULT_OK) {
    out.add("Install failed: %s", result_name(r.connect_result));
  } else if (r.program_result != stcisp::RESULT_OK) {
    out.add("Install failed at %s: %s, %u bytes written", stage_name(r.stage), result_name(r.program_result),
            (unsigned) r.written);
  } else if (r.app_seen) {
    out.add("Install OK: application %s, bootloader v%u", firmware_text(r.app_version, r.app_build).str,
            r.app_boot_version);
  } else {
    out.add("Install written, but the new firmware did not answer");
  }
  if (this->ran_)
    out.add(", at this power-on");
}

}
