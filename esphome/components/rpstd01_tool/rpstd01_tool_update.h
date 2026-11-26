// RP-STD01 MCU tool: the application update through our bootloader, over the link.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "esphome/components/rpstd01_link/rpstd01_link.h"

#ifndef RPSTD01TOOL_LOG
#if defined(ESP_PLATFORM) || defined(USE_HOST)
#include "esphome/core/log.h"
#define RPSTD01TOOL_LOG(...) ESP_LOGI("rpstd01tool", __VA_ARGS__)
#else
#define RPSTD01TOOL_LOG(...) \
  do { \
  } while (0)
#endif
#endif

namespace rpstd01tool {

enum class UpdateState : uint8_t { IDLE, REQUESTING, BEGIN, WRITING, FINISHING, RUNNING, DONE, FAILED };

namespace update_detail {

inline constexpr uint32_t REQUEST_RESEND_MS = 500, REQUEST_TIMEOUT_MS = 5000;
// Well above the bootloader's receive timeout and its longest command, the erase of the application
// area.
inline constexpr uint32_t RETRY_MS = 1000;
static_assert(RETRY_MS > 10 * rpstd01::BOOT_RX_TIMEOUT_MS, "RETRY_MS");
inline constexpr uint8_t MAX_RESTARTS = 2;
// Frames sent in one step before it starts again from Begin.
inline constexpr uint8_t WRITE_TRIES = 5, COMMAND_TRIES = 3;
// The writes run from WRITE_FROM to WRITE_FROM + WRITE_SPAN.
inline constexpr float PROGRESS_WRITE_FROM = 5.0f, PROGRESS_WRITE_SPAN = 90.0f, PROGRESS_STARTING = 98.0f;
// A boot status that the bootloader started to send before it had the whole command has fully
// arrived this long after the command was written: the longest command, then a boot status.
inline constexpr uint32_t CROSS_BYTES =
    rpstd01::BOOT_MAX_PAYLOAD + rpstd01::FRAME_OVERHEAD + rpstd01::BOOT_STATUS_LEN + rpstd01::FRAME_OVERHEAD;
inline constexpr uint32_t CROSS_MS =
    (CROSS_BYTES * rpstd01::LINK_BYTE_BITS * 1000 + rpstd01::LINK_BAUD - 1) / rpstd01::LINK_BAUD;
inline constexpr uint16_t VALID_MARK_OFFSET = rpstd01::APP_SIZE - rpstd01::TRAILER_SIZE + rpstd01::TR_VALID;

inline constexpr const char *OTHER_BOOTLOADER = "MCU's bootloader is not the one this application is for";

inline const char *boot_result_text(uint8_t result) {
  switch (result) {
    case rpstd01::BR_BAD_STATE:
      return "bootloader in the wrong state";
    case rpstd01::BR_BAD_SIZE:
      return "wrong image size";
    case rpstd01::BR_BAD_OFFSET:
      return "wrong write offset";
    case rpstd01::BR_VERIFY:
      return "flash write failed";
    case rpstd01::BR_MISMATCH:
      return "written data does not match";
    case rpstd01::BR_BAD_IMAGE:
      return "image check failed";
    case rpstd01::BR_BAD_FRAME:
      return "bootloader rejected a frame";
    case rpstd01::BR_NO_COMMAND:
      return "bootloader restarted";
    default:
      return "unknown bootloader error";
  }
}

}

class AppUpdater : public rpstd01::McuLink {
 public:
  // The application area to write (rpstd01::APP_SIZE bytes, from APP_BASE), with the build ID in its
  // trailer. Its valid mark is sent as 0xFF (the carried image has it set): the bootloader sets it
  // after checking the CRC. An update stops before it writes anything if the bootloader is not
  // boot_version, the one the application was built for.
  void set_image(const uint8_t *app_area, uint32_t build, uint8_t boot_version) {
    this->image_ = app_area;
    this->image_build_ = build;
    this->image_boot_version_ = boot_version;
  }
  bool has_image() const { return this->image_ != nullptr; }

  // Each loop: receive(), which passes each frame on to also(const rpstd01::Received &), then
  // service(), which sends at most one frame, and nothing while no update runs.
  template<typename U, typename F> void receive(U &uart, uint32_t now_ms, F &&also);
  template<typename U> void service(U &uart);
  template<typename U> void loop(U &uart, uint32_t now_ms) {
    this->receive(uart, now_ms, [](const rpstd01::Received &) {});
    this->service(uart);
  }

  // Through the application if it runs, else straight to Begin.
  void start_update();
  UpdateState update_state() const { return this->state_; }
  bool update_running() const {
    return this->state_ != UpdateState::IDLE && this->state_ != UpdateState::DONE &&
           this->state_ != UpdateState::FAILED;
  }
  float update_progress_percent() const;
  void update_text(char *buf, size_t size) const;
  const char *fail_reason() const { return this->fail_reason_; }

 protected:
  void note_loop_(uint32_t now_ms);
  void handle_frame_(const rpstd01::Received &rx);
  void handle_boot_reply_(const rpstd01::BootStatus &bs);
  void tick_();
  size_t prepare_(uint8_t *frame);
  size_t boot_command_(uint8_t *frame) const;
  void enter_(UpdateState state);
  void restart_(const char *reason);
  void fail_(const char *reason);
  bool due_(uint32_t every_ms) const { return this->tries_ == 0 || this->elapsed_(this->sent_ms_) >= every_ms; }

  const uint8_t *image_{nullptr};
  uint32_t image_build_{0};
  uint8_t image_boot_version_{0};
  UpdateState state_{UpdateState::IDLE};
  uint32_t start_ms_{0};
  uint32_t sent_ms_{0};  // of the step's last frame
  uint8_t tries_{0};     // frames sent in this step (for WRITING: for this chunk)
  uint8_t restarts_{0};  // returns to Begin
  uint16_t next_{0};
  const char *fail_reason_{""};
  // Whether a loop ran since the last command, the first such loop, and the loop before this one.
  bool looped_since_sent_{false};
  uint32_t first_loop_ms_{0}, previous_loop_ms_{0};
  bool status_is_reply_{false};
};

template<typename U, typename F> void AppUpdater::receive(U &uart, uint32_t now_ms, F &&also) {
  this->note_loop_(now_ms);
  this->receive_(uart, now_ms, [this, &also](const rpstd01::Received &rx) {
    this->handle_frame_(rx);
    also(rx);
  });
}

template<typename U> void AppUpdater::service(U &uart) {
  this->tick_();
  uint8_t frame[rpstd01::BOOT_MAX_PAYLOAD + rpstd01::FRAME_OVERHEAD];
  if (size_t n = this->prepare_(frame)) {
    uart.write_array(frame, n);
    this->tries_++;
    this->sent_ms_ = this->now_;
    this->looped_since_sent_ = false;
  }
}

// A boot status is taken for the reply only when it cannot have crossed the command: read in a loop
// after a loop that started CROSS_MS after the first loop since the command was written. The bootloader
// repeats its result, so a real reply is seen again soon. Of the last command's results, only an
// error or Begin's could pass for this command's reply.
inline void AppUpdater::note_loop_(uint32_t now_ms) {
  if (!this->looped_since_sent_)
    this->first_loop_ms_ = now_ms;
  this->status_is_reply_ = this->tries_ > 0 && this->looped_since_sent_ &&
                           this->previous_loop_ms_ - this->first_loop_ms_ >= update_detail::CROSS_MS;
  this->looped_since_sent_ = true;
  this->previous_loop_ms_ = now_ms;
}

inline void AppUpdater::handle_frame_(const rpstd01::Received &rx) {
  const bool boot = rx.type == rpstd01::MSG_BOOT_STATUS && rx.decoded;
  if (boot && this->update_running() && this->boot_.version != this->image_boot_version_) {
    this->fail_(update_detail::OTHER_BOOTLOADER);
    return;
  }
  switch (this->state_) {
    case UpdateState::REQUESTING:
      if (boot)
        this->enter_(UpdateState::BEGIN);
      else if (rx.starts && this->tries_)
        this->fail_("MCU restarted without entering update mode");
      break;
    case UpdateState::BEGIN:
    case UpdateState::WRITING:
    case UpdateState::FINISHING:
      if (boot)
        this->handle_boot_reply_(this->boot_);
      break;
    case UpdateState::RUNNING:
      if (boot) {
        this->handle_boot_reply_(this->boot_);
      } else if (rx.type == rpstd01::MSG_INFO && rx.decoded) {
        if (this->info_.build != this->image_build_) {
          this->fail_("MCU reports a different build after the update");
        } else {
          this->state_ = UpdateState::DONE;
          RPSTD01TOOL_LOG("Application update done");
        }
      }
      break;
    default:
      break;
  }
}

// A boot status that does not show the step's result is not taken for the reply: the bootloader
// also repeats its last result, and the retry timer covers a lost command or reply.
inline void AppUpdater::handle_boot_reply_(const rpstd01::BootStatus &bs) {
  using rpstd01::BR_OK;
  // "No command yet" before Begin is answered means Begin has not arrived; later, a restart.
  const bool waits_for_begin = this->state_ == UpdateState::BEGIN && bs.result == rpstd01::BR_NO_COMMAND;
  if (bs.result != BR_OK && !waits_for_begin) {
    if (this->status_is_reply_)
      this->restart_(update_detail::boot_result_text(bs.result));
    return;
  }
  switch (this->state_) {
    case UpdateState::BEGIN:
      if (bs.result == BR_OK && bs.next == 0 && !bs.app_valid && this->status_is_reply_) {
        this->next_ = 0;
        this->enter_(UpdateState::WRITING);
      }
      break;
    case UpdateState::WRITING:
      if (bs.next < this->next_ || bs.next % rpstd01::WRITE_CHUNK != 0) {
        this->restart_("MCU restarted during the update");
      } else if (bs.next > this->next_) {
        this->next_ = bs.next;
        this->enter_(this->next_ >= rpstd01::APP_SIZE ? UpdateState::FINISHING : UpdateState::WRITING);
      }
      break;
    case UpdateState::FINISHING:
      if (bs.app_valid)
        this->enter_(UpdateState::RUNNING);
      break;
    default:
      break;  // RUNNING: the reply to Run comes just before the application starts
  }
}

inline void AppUpdater::tick_() {
  using update_detail::RETRY_MS;
  switch (this->state_) {
    case UpdateState::REQUESTING:
      if (this->elapsed_(this->start_ms_) >= update_detail::REQUEST_TIMEOUT_MS)
        this->fail_("MCU did not enter update mode");
      return;
    case UpdateState::BEGIN:
    case UpdateState::WRITING:
    case UpdateState::FINISHING:
      if (!this->last_seen_boot_) {
        this->fail_("MCU left update mode");
        return;
      }
      if (this->link_state() == rpstd01::LinkState::NO_RESPONSE) {
        this->fail_("MCU stopped answering");
        return;
      }
      break;
    case UpdateState::RUNNING:
      if (this->link_state() == rpstd01::LinkState::NO_RESPONSE && this->due_(RETRY_MS)) {
        this->fail_("MCU did not start the new firmware");
        return;
      }
      break;
    default:
      return;
  }
  const uint8_t max_tries =
      this->state_ == UpdateState::WRITING ? update_detail::WRITE_TRIES : update_detail::COMMAND_TRIES;
  if (this->tries_ < max_tries || !this->due_(RETRY_MS))
    return;
  switch (this->state_) {
    case UpdateState::BEGIN:
      this->restart_("no reply to erase");
      break;
    case UpdateState::WRITING:
      this->restart_("no reply to write");
      break;
    case UpdateState::FINISHING:
      this->restart_("no reply to the image check");
      break;
    default:
      this->fail_("MCU did not start the new firmware");
      break;
  }
}

// Application frames go out only while the newest frame is our application's status: our bootloader
// drops anything but its own commands.
inline size_t AppUpdater::prepare_(uint8_t *frame) {
  using rpstd01::build_frame;
  const bool app = !this->last_seen_boot_ && this->link_state() == rpstd01::LinkState::APPLICATION;
  switch (this->state_) {
    case UpdateState::REQUESTING: {
      // No gate target 0 first: the tool never turned the lamp on, and the MCU turns it off by itself
      // before it enters update mode.
      if (!app || !this->due_(update_detail::REQUEST_RESEND_MS))
        return 0;
      static const uint8_t KEY[rpstd01::UPDATE_LEN] = {rpstd01::UPDATE_KEY0, rpstd01::UPDATE_KEY1, rpstd01::UPDATE_KEY2,
                                                       rpstd01::UPDATE_KEY3};
      return build_frame(frame, rpstd01::MSG_UPDATE, KEY, rpstd01::UPDATE_LEN);
    }
    case UpdateState::RUNNING:
      // Run again while our bootloader still answers; once the application runs, its info frame is
      // asked for, in case the one it sent at its start was lost.
      if (!this->due_(update_detail::RETRY_MS))
        return 0;
      if (this->last_seen_boot_)
        return this->boot_command_(frame);
      return app ? build_frame(frame, rpstd01::MSG_INFO_REQ, nullptr, 0) : 0;
    case UpdateState::BEGIN:
    case UpdateState::WRITING:
    case UpdateState::FINISHING:
      return this->due_(update_detail::RETRY_MS) ? this->boot_command_(frame) : 0;
    default:
      return 0;
  }
}

inline size_t AppUpdater::boot_command_(uint8_t *frame) const {
  using rpstd01::build_frame;
  static constexpr size_t OFFSET_LEN = rpstd01::BOOT_WRITE_LEN - rpstd01::WRITE_CHUNK;
  uint8_t payload[rpstd01::BOOT_WRITE_LEN];
  switch (this->state_) {
    case UpdateState::BEGIN:
      rpstd01::detail::put16(payload, rpstd01::APP_SIZE);
      return build_frame(frame, rpstd01::MSG_BOOT_BEGIN, payload, rpstd01::BOOT_BEGIN_LEN);
    case UpdateState::WRITING:
      rpstd01::detail::put16(payload, this->next_);
      memcpy(payload + OFFSET_LEN, this->image_ + this->next_, rpstd01::WRITE_CHUNK);
      if (this->next_ + rpstd01::WRITE_CHUNK > update_detail::VALID_MARK_OFFSET)
        payload[OFFSET_LEN + update_detail::VALID_MARK_OFFSET - this->next_] = 0xFF;
      return build_frame(frame, rpstd01::MSG_BOOT_WRITE, payload, rpstd01::BOOT_WRITE_LEN);
    case UpdateState::FINISHING:
      return build_frame(frame, rpstd01::MSG_BOOT_FINISH, nullptr, 0);
    default:
      return build_frame(frame, rpstd01::MSG_BOOT_RUN, nullptr, 0);
  }
}

inline void AppUpdater::enter_(UpdateState state) {
  this->state_ = state;
  this->tries_ = 0;
  this->status_is_reply_ = false;
}

inline void AppUpdater::restart_(const char *reason) {
  if (this->restarts_ >= update_detail::MAX_RESTARTS) {
    this->fail_(reason);
    return;
  }
  RPSTD01TOOL_LOG("Application update: %s; starting again from Begin", reason);
  this->restarts_++;
  this->next_ = 0;
  this->enter_(UpdateState::BEGIN);
}

inline void AppUpdater::fail_(const char *reason) {
  RPSTD01TOOL_LOG("Application update failed: %s", reason);
  this->state_ = UpdateState::FAILED;
  this->fail_reason_ = reason;
}

inline void AppUpdater::start_update() {
  if (this->image_ == nullptr || this->update_running())
    return;
  this->restarts_ = 0;
  this->next_ = 0;
  this->fail_reason_ = "";
  this->start_ms_ = this->now_;
  const bool in_bootloader = this->link_state() == rpstd01::LinkState::BOOTLOADER;
  if (in_bootloader && this->boot_.version != this->image_boot_version_) {
    this->fail_(update_detail::OTHER_BOOTLOADER);
    return;
  }
  this->enter_(in_bootloader ? UpdateState::BEGIN : UpdateState::REQUESTING);
}

inline float AppUpdater::update_progress_percent() const {
  switch (this->state_) {
    case UpdateState::WRITING:
    case UpdateState::FINISHING:
    case UpdateState::FAILED:
      if (this->next_ == 0)
        return 0.0f;
      return update_detail::PROGRESS_WRITE_FROM + update_detail::PROGRESS_WRITE_SPAN * this->next_ / rpstd01::APP_SIZE;
    case UpdateState::RUNNING:
      return update_detail::PROGRESS_STARTING;
    case UpdateState::DONE:
      return 100.0f;
    default:
      return 0.0f;
  }
}

inline void AppUpdater::update_text(char *buf, size_t size) const {
  if (size == 0)
    return;
  switch (this->state_) {
    case UpdateState::IDLE:
      snprintf(buf, size, "Idle");
      break;
    case UpdateState::REQUESTING:
      snprintf(buf, size, "Requesting");
      break;
    case UpdateState::BEGIN:
      snprintf(buf, size, "Erasing");
      break;
    case UpdateState::WRITING:
      snprintf(buf, size, "Writing %d %%", static_cast<int>(this->update_progress_percent()));
      break;
    case UpdateState::FINISHING:
      snprintf(buf, size, "Checking");
      break;
    case UpdateState::RUNNING:
      snprintf(buf, size, "Starting");
      break;
    case UpdateState::DONE:
      snprintf(buf, size, "Done");
      break;
    case UpdateState::FAILED:
      snprintf(buf, size, "Failed: %s", this->fail_reason_);
      break;
  }
}

}
