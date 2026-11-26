// The arming rules of the install through STC's bootloader, on a small model of the unit (NVS with
// one key per value, the last-attempt record, the MCU and the serial link) driven in the order the
// MCU tool calls them.

#include <cstdio>
#include <deque>
#include <vector>

#include "esphome/components/rpstd01_tool/rpstd01_mcuflash.h"
#include "test.h"

using namespace mcuflash;
using rpstd01::APP_BASE;
using rpstd01::APP_END;
using rpstd01::MSG_INFO;
using rpstd01::MSG_STATUS;
using rpstd01::STATUS_LEN;
using Bytes = std::vector<uint8_t>;

static_assert(rpstd01::TRAILER_ADDR >= APP_END - stcisp::BLOCK_SIZE,
              "the trailer that makes the application valid is in the last block written");

constexpr uint32_t OUR_BUILD = 0x84BFA001;
constexpr uint32_t STOCK = 0;         // the MCU runs the factory firmware
constexpr uint32_t BROKEN = 1;        // the MCU's firmware is incomplete

// NVS with one key per Store value. fail_after: the number of further writes that succeed
// (-1: all).
class FakeNvs : public StoreIo {
 public:
  Store s{0, USER_NONE, 0, 0};
  int fail_after = -1;
  int writes = 0;
  void load(Store *out) override { *out = s; }
  bool set(Key key, uint32_t v) override {
    if (fail_after == 0)
      return false;
    if (fail_after > 0)
      fail_after--;
    writes++;
    switch (key) {
      case KEY_ARM:
        s.arm = v;
        break;
      case KEY_USER:
        s.user = (uint8_t) v;
        break;
      case KEY_FAILS:
        s.fails = (uint8_t) v;
        break;
      case KEY_PARTIAL:
        s.partial = (uint8_t) v;
        break;
    }
    return true;
  }
  bool commit() override { return fail_after != 0; }
};

// What a power-on attempt ends with.
enum Outcome {
  OUT_OK,           // programmed, and our firmware runs
  OUT_NO_REPLY,     // the bootloader was not caught: a short power-on, or mains flicker
  OUT_CALIBRATION,  // fails before the erase, every time
  OUT_WRITE_EARLY,  // fails while writing our bootloader
  OUT_WRITE_LATE,   // fails while writing the application
  OUT_WRONG_MODEL,  // a refusal
  OUT_CRASH,        // the power goes off while writing
};

struct Unit {
  FakeNvs nvs;
  uint32_t image_crc = 0x11111111;
  uint32_t image_build = OUR_BUILD;
  uint32_t mcu = STOCK;   // what the MCU runs
  bool link_ok = true;    // the check can read the info frame
  bool have_last = false;  // the last-attempt record
  Attempt rec{};
  Check state = CHECK_ASK;
  Store now{};
  int attempts = 0;

  // The last attempt's refusal, as the tool reads it from its record.
  uint8_t refused() const { return have_last ? refusal(rec.connect_result, rec.program_result) : 0; }

  // The tool's power_on() at a mains power-on. True if an install ran.
  bool power_on(Outcome out) {
    Taken t;
    if (!power_on_take(nvs, image_crc, refused(), &t))
      return false;
    attempts++;
    have_last = true;
    rec = Attempt{RESULT_UNFINISHED, RESULT_NOT_STARTED, stcisp::STAGE_SYNC, 0};  // saved before the attempt
    if (out == OUT_CRASH) {
      mcu = BROKEN;
      return true;  // power_on_done never runs
    }
    rec.connect_result = stcisp::RESULT_OK;
    rec.stage = stcisp::STAGE_STATUS;
    if (out == OUT_NO_REPLY) {
      rec.connect_result = stcisp::RESULT_NO_REPLY;
      rec.stage = stcisp::STAGE_SYNC;
    } else if (out == OUT_WRONG_MODEL) {
      rec.connect_result = stcisp::RESULT_WRONG_MODEL;
    } else {
      switch (out) {
        case OUT_CALIBRATION:
          rec = Attempt{stcisp::RESULT_OK, stcisp::RESULT_CALIBRATION, stcisp::STAGE_CALIBRATE, 0};
          break;
        case OUT_WRITE_EARLY:
          rec = Attempt{stcisp::RESULT_OK, stcisp::RESULT_TIMEOUT, stcisp::STAGE_WRITE, 0x0400};
          mcu = BROKEN;
          break;
        case OUT_WRITE_LATE:
          rec = Attempt{stcisp::RESULT_OK, stcisp::RESULT_TIMEOUT, stcisp::STAGE_WRITE, 0x2000};
          mcu = BROKEN;
          break;
        default:
          rec = Attempt{stcisp::RESULT_OK, stcisp::RESULT_OK, stcisp::STAGE_DONE, APP_END};
          mcu = image_build;
          break;
      }
    }
    power_on_done(nvs, t, rec);
    return true;
  }

  // At the end of the tool's check after each start, and after each button.
  Check check() {
    const bool current = link_ok && mcu == image_build;
    state = mcuflash::check(nvs, current, refused(), image_crc, &now);
    return state;
  }

  // A button (Install at Next Power-On, Cancel Install). Returns false for "NVS write failed".
  bool cmd(Command c) {
    const bool ok = run_command(nvs, c, image_crc);
    check();
    return ok;
  }

  // An OTA update to another tool build, which restarts without a power-on.
  void ota(uint32_t new_image_crc, uint32_t new_image_build) {
    image_crc = new_image_crc;
    image_build = new_image_build;
  }
};

static void test_auto_arm_and_install() {
  Unit u;
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.nvs.s.arm == u.image_crc && u.nvs.s.user == USER_NONE);
  CHECK(u.check() == CHECK_ARMED);  // a restart changes nothing
  CHECK(u.power_on(OUT_OK));
  CHECK(u.nvs.s.arm == 0 && u.nvs.s.partial == 0);
  CHECK(u.nvs.s.fails == 1);  // until the check confirms the build
  CHECK(u.check() == CHECK_CURRENT);
  CHECK(u.nvs.s.fails == 0);
  CHECK(!u.power_on(OUT_OK));
  CHECK(u.check() == CHECK_CURRENT);
}

static void test_disarm_then_ota() {
  Unit u;
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.cmd(COMMAND_DISARM));
  CHECK(u.state == CHECK_DISARMED && u.nvs.s.arm == 0);
  u.ota(0x22222222, OUR_BUILD + 1);
  CHECK(u.check() == CHECK_DISARMED);
  CHECK(!u.power_on(OUT_OK));
  CHECK(u.check() == CHECK_DISARMED);
  CHECK(u.cmd(COMMAND_ARM));
  CHECK(u.state == CHECK_ARMED && u.nvs.s.arm == 0x22222222);
}

// While our build runs, Cancel Install only cancels an arm: a newer tool still updates.
static void test_disarm_while_current() {
  Unit u;
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_OK));
  CHECK(u.check() == CHECK_CURRENT);
  CHECK(u.cmd(COMMAND_ARM));
  CHECK(u.state == CHECK_CURRENT && u.now.arm == u.image_crc && u.now.user == USER_ARMED);
  CHECK(u.cmd(COMMAND_DISARM));
  CHECK(u.state == CHECK_CURRENT && u.nvs.s.arm == 0 && u.nvs.s.user == USER_NONE);
  u.ota(0x22222222, OUR_BUILD + 1);
  CHECK(u.check() == CHECK_ARMED);
}

static void test_refusal_then_user_arm() {
  Unit u;
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_WRONG_MODEL));
  CHECK(u.nvs.s.fails == 0);  // a refusal is not a failure
  CHECK(u.check() == CHECK_REFUSED);
  CHECK(u.nvs.s.arm == 0);
  CHECK(!u.power_on(OUT_OK));  // not armed again
  CHECK(u.check() == CHECK_REFUSED);
  CHECK(u.cmd(COMMAND_ARM));
  CHECK(u.state == CHECK_ARMED);
  CHECK(u.power_on(OUT_WRONG_MODEL));
  CHECK(u.check() == CHECK_REFUSED);
  CHECK(u.cmd(COMMAND_DISARM));
  CHECK(u.state == CHECK_REFUSED);
  CHECK(u.cmd(COMMAND_ARM));
  CHECK(u.state == CHECK_ARMED);
  CHECK(u.power_on(OUT_OK));
  CHECK(u.check() == CHECK_CURRENT && u.refused() == 0);
}

// A refusal holds for every tool build, until Install at Next Power-On, so it never adds up to
// "gave up" and is never retried by itself.
static void test_refusal_holds_across_tool_builds() {
  Unit u;
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_WRONG_MODEL));
  for (uint32_t build = 1; build <= 3; build++) {
    u.ota(u.image_crc + build, OUR_BUILD + build);
    CHECK(u.check() == CHECK_REFUSED);
    CHECK(!u.power_on(OUT_WRONG_MODEL));
  }
  CHECK(u.attempts == 1 && u.nvs.s.fails == 0);
  // A power-on the bootloader missed after Install at Next Power-On: the refusal is no longer the
  // last attempt, so automatic arming resumes, and a new refusal holds again.
  CHECK(u.cmd(COMMAND_ARM));
  CHECK(u.power_on(OUT_NO_REPLY));
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_WRONG_MODEL));
  CHECK(u.check() == CHECK_REFUSED);
}

// The count of failed automatic attempts survives a new tool build; Install at Next Power-On and
// a check that finds the MCU current reset it.
static void test_fails_reset() {
  Unit u;
  for (int i = 0; i < MAX_AUTO_FAILS; i++) {
    CHECK(u.check() == CHECK_ARMED);
    CHECK(u.power_on(OUT_CALIBRATION));
  }
  CHECK(u.check() == CHECK_GAVE_UP);
  u.ota(u.image_crc + 1, OUR_BUILD + 1);
  CHECK(u.check() == CHECK_GAVE_UP && u.nvs.s.fails == MAX_AUTO_FAILS);
  u.mcu = u.image_build;  // made current another way, for example over the link
  CHECK(u.check() == CHECK_CURRENT && u.nvs.s.fails == 0);
}

static void test_failed_flash_power_cycle() {
  Unit u;
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_WRITE_LATE));
  CHECK(u.nvs.s.fails == 1 && u.nvs.s.partial == 1);
  CHECK(u.check() == CHECK_ARMED);  // retried at the next power-on
  CHECK(u.power_on(OUT_OK));
  CHECK(u.nvs.s.partial == 0);
  CHECK(u.check() == CHECK_CURRENT);
  CHECK(u.nvs.s.fails == 0);
}

static void test_repeated_pre_erase_failures() {
  Unit u;
  for (int i = 0; i < MAX_AUTO_FAILS; i++) {
    CHECK(u.check() == CHECK_ARMED);
    CHECK(u.power_on(OUT_CALIBRATION));
    CHECK(u.nvs.s.fails == i + 1);
    CHECK(u.nvs.s.partial == 0);  // nothing was erased
  }
  CHECK(u.check() == CHECK_GAVE_UP);
  CHECK(u.nvs.s.arm == 0);
  CHECK(!u.power_on(OUT_CALIBRATION));
  CHECK(u.check() == CHECK_GAVE_UP);
  CHECK(u.attempts == MAX_AUTO_FAILS);
  CHECK(u.cmd(COMMAND_ARM));
  CHECK(u.state == CHECK_ARMED && u.nvs.s.fails == 0);
  CHECK(u.power_on(OUT_CALIBRATION));
  CHECK(u.nvs.s.fails == 0);  // the user's own attempt does not count
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_OK));
  CHECK(u.check() == CHECK_CURRENT && u.nvs.s.fails == 0);
}

// Power-ons too short for the bootloader do not count.
static void test_no_reply_not_counted() {
  Unit u;
  for (int i = 0; i < 2 * MAX_AUTO_FAILS; i++) {
    CHECK(u.check() == CHECK_ARMED);
    CHECK(u.power_on(OUT_NO_REPLY));
    CHECK(u.nvs.s.fails == 0);
  }
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_CALIBRATION));
  CHECK(u.nvs.s.fails == 1);
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_NO_REPLY));  // keeps the count, does not add to it
  CHECK(u.nvs.s.fails == 1);
}

// A power cut while writing counts as a failure, and leaves the firmware incomplete.
static void test_crash_counts() {
  Unit u;
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_CRASH));
  CHECK(u.nvs.s.fails == 1 && u.nvs.s.partial == 1);
}

// The flash reports success, but the check does not see our build: the count keeps going.
static void test_verify_check_disagree() {
  {
    Unit u;  // the check cannot read the MCU
    u.link_ok = false;
    for (int i = 0; i < MAX_AUTO_FAILS; i++) {
      CHECK(u.check() == CHECK_ARMED);
      CHECK(u.power_on(OUT_OK));
    }
    CHECK(u.check() == CHECK_GAVE_UP);
    CHECK(u.attempts == MAX_AUTO_FAILS);
  }
  {
    Unit u;  // the image's build differs from what the check expects
    u.image_build = OUR_BUILD + 1;
    for (int i = 0; i < MAX_AUTO_FAILS; i++) {
      CHECK(u.check() == CHECK_ARMED);
      u.image_build = OUR_BUILD;  // what the flash writes
      CHECK(u.power_on(OUT_OK));
      u.image_build = OUR_BUILD + 1;
    }
    CHECK(u.check() == CHECK_GAVE_UP);
  }
}

// A flash that stopped after the erase leaves a firmware STC's bootloader would run from the next
// power-on: the fail count must not hold the next attempt back.
static void test_incomplete_overrides_hold() {
  Unit u;
  for (int i = 0; i < MAX_AUTO_FAILS - 1; i++) {
    CHECK(u.check() == CHECK_ARMED);
    CHECK(u.power_on(OUT_CALIBRATION));
  }
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_WRITE_EARLY));
  CHECK(u.nvs.s.fails == MAX_AUTO_FAILS && u.nvs.s.partial == 1);
  CHECK(u.check() == CHECK_ARMED);  // not "gave up"
  CHECK(u.power_on(OUT_WRITE_EARLY));
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_NO_REPLY));
  CHECK(u.nvs.s.partial == 1);
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_OK));
  CHECK(u.nvs.s.partial == 0);
  CHECK(u.check() == CHECK_CURRENT);
}

static void test_incomplete_still_held_by_cancel_install() {
  Unit u;
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_CRASH));
  CHECK(u.cmd(COMMAND_DISARM));
  CHECK(u.state == CHECK_DISARMED && u.now.partial == 1);
}

// A flash cut short by the power going off is retried at the very next power-on, before any
// check has run: from that power-on, STC's bootloader would run the partial program.
static void test_power_cut_retries_next_power_on() {
  Unit u;
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_CRASH));
  CHECK(u.nvs.s.arm == 0 && u.nvs.s.partial == 1 && u.nvs.s.fails == 1);
  CHECK(u.power_on(OUT_CRASH));  // no check in between
  CHECK(u.nvs.s.fails == 2);     // automatic, so counted
  CHECK(u.power_on(OUT_OK));
  CHECK(u.nvs.s.partial == 0 && u.attempts == 3);
  CHECK(u.check() == CHECK_CURRENT);
  CHECK(!u.power_on(OUT_OK));  // complete: nothing more
}

static void test_power_cut_retry_after_install_by_hand_is_automatic_and_counts() {
  Unit u;
  CHECK(u.cmd(COMMAND_ARM));
  CHECK(u.power_on(OUT_CRASH));
  CHECK(u.nvs.s.fails == 0);
  CHECK(u.power_on(OUT_CRASH));
  CHECK(u.nvs.s.fails == 1);
}

static void test_power_cut_retry_not_stopped_by_stale_arm() {
  Unit u;
  u.nvs.s = {0x99999999, USER_NONE, 0, 1};
  CHECK(u.power_on(OUT_OK));
}

static void test_power_cut_retry_held_by_cancel_install() {
  Unit u;
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_CRASH));
  CHECK(u.cmd(COMMAND_DISARM));
  CHECK(!u.power_on(OUT_OK));
  CHECK(u.nvs.s.user == USER_DISARMED && u.nvs.s.partial == 1);
  CHECK(u.check() == CHECK_DISARMED);
  CHECK(!u.power_on(OUT_OK));
  CHECK(u.attempts == 1);
}

// Like the check: no attempt at every power-on, nor after a newer tool build; Install at Next
// Power-On ends the hold.
static void test_power_cut_retry_held_by_refusal() {
  Unit u;
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_CRASH));
  CHECK(u.power_on(OUT_WRONG_MODEL));
  CHECK(u.nvs.s.partial == 1);
  CHECK(!u.power_on(OUT_OK));
  CHECK(u.check() == CHECK_REFUSED);
  CHECK(!u.power_on(OUT_OK));
  CHECK(u.attempts == 2);
  u.ota(u.image_crc + 1, OUR_BUILD);
  CHECK(!u.power_on(OUT_OK));
  CHECK(u.cmd(COMMAND_ARM));
  CHECK(u.power_on(OUT_OK));
  CHECK(u.check() == CHECK_CURRENT);
}

// Stale NVS: an arm flag for another image, and unknown values.
static void test_old_nvs() {
  {
    Unit u;
    u.nvs.s = {0x99999999, 0, 0, 0};  // armed for another image, with a refusal on record
    u.have_last = true;
    u.rec = Attempt{stcisp::RESULT_WRONG_MODEL, 0xFF, stcisp::STAGE_STATUS, 0};
    CHECK(u.check() == CHECK_REFUSED);
    CHECK(!u.power_on(OUT_OK) && u.nvs.s.arm == 0);  // the stale flag is dropped at the power-on
  }
  {
    Unit u;
    u.nvs.s = {0x50524F42, 3, 0, 0};  // an arm flag for no image, and a button value out of range
    CHECK(!u.power_on(OUT_OK));
    CHECK(u.check() == CHECK_ARMED && u.nvs.s.arm == u.image_crc);
    u.nvs.s.user = 3;
    u.mcu = u.image_build;
    CHECK(u.check() == CHECK_CURRENT && u.nvs.s.user == USER_NONE);
  }
  {
    Unit u;
    u.nvs.s = {0x99999999, 0, 0, 0};
    CHECK(!u.power_on(OUT_OK));  // a stale arm is dropped, not run
    CHECK(u.nvs.s.arm == 0 && u.nvs.s.user == USER_NONE);
    CHECK(u.check() == CHECK_ARMED);
  }
  {
    Unit u;
    u.nvs.s = {u.image_crc, 0, 0, 0};
    CHECK(u.power_on(OUT_CALIBRATION));
    CHECK(u.nvs.s.fails == 1);
  }
}

static void test_nvs_write_failure() {
  Unit u;
  u.nvs.fail_after = 0;
  CHECK(u.check() == CHECK_ARM_FAILED);
  CHECK(u.nvs.s.arm == 0);
  CHECK(!u.cmd(COMMAND_ARM));
  CHECK(u.state == CHECK_ARM_FAILED);
  u.nvs.fail_after = -1;
  CHECK(u.check() == CHECK_ARMED);
}

static void test_arm_flag_that_cannot_be_cleared_at_power_on_runs_nothing() {
  Unit u;
  CHECK(u.check() == CHECK_ARMED);
  u.nvs.fail_after = 0;
  CHECK(!u.power_on(OUT_OK));
  CHECK(u.attempts == 0);
}

// A write that stops part way leaves the old arm flag, and the page shows what NVS holds.
static void test_partial_cancel_install_writes_user_value_only() {
  Unit u;
  CHECK(u.check() == CHECK_ARMED);
  u.nvs.fail_after = 1;
  CHECK(!u.cmd(COMMAND_DISARM));
  CHECK(u.nvs.s.user == USER_DISARMED && u.nvs.s.arm == u.image_crc);
  CHECK(u.state == CHECK_DISARMED);  // the user value alone holds the flash
  u.nvs.fail_after = -1;
  CHECK(!u.power_on(OUT_OK));
  CHECK(u.attempts == 0 && u.nvs.s.arm == 0 && u.nvs.s.user == USER_DISARMED);
  CHECK(u.check() == CHECK_DISARMED);
  CHECK(u.cmd(COMMAND_ARM));  // Install at Next Power-On still works
  CHECK(u.state == CHECK_ARMED);
  CHECK(u.power_on(OUT_OK));
  CHECK(u.check() == CHECK_CURRENT);
}

static void test_partial_cancel_install_repaired_by_check() {
  Unit u;
  CHECK(u.check() == CHECK_ARMED);
  u.nvs.fail_after = 1;
  CHECK(!u.cmd(COMMAND_DISARM));
  u.nvs.fail_after = -1;
  CHECK(u.check() == CHECK_DISARMED && u.nvs.s.arm == 0);
  CHECK(!u.power_on(OUT_OK));
}

static void test_partial_install_by_hand_after_giving_up_resets_count_only() {
  Unit u;
  for (int i = 0; i < MAX_AUTO_FAILS; i++) {
    CHECK(u.check() == CHECK_ARMED);
    CHECK(u.power_on(OUT_CALIBRATION));
  }
  CHECK(u.check() == CHECK_GAVE_UP);
  u.nvs.fail_after = 2;  // the user value and the fail count are written, the arm flag is not
  CHECK(!u.cmd(COMMAND_ARM));
  CHECK(u.nvs.s.arm == 0);
  CHECK(u.state == CHECK_ARM_FAILED);
  CHECK(u.now == u.nvs.s);
}

static void test_failed_clear_of_leftover_arm_not_shown_as_users() {
  Unit u;
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_OK));
  u.nvs.s.arm = u.image_crc;  // as left by a check that ran before the flash was confirmed
  u.nvs.fail_after = 0;
  CHECK(u.check() == CHECK_CURRENT);
  CHECK(u.now.arm == u.image_crc && u.now.user != USER_ARMED);
}

static void test_current_clears_leftover() {
  {
    Unit u;
    u.mcu = OUR_BUILD;
    u.nvs.s = {u.image_crc, USER_NONE, 2, 1};
    CHECK(u.check() == CHECK_CURRENT);
    CHECK(u.nvs.s.arm == 0 && u.nvs.s.fails == 0 && u.nvs.s.partial == 0);
  }
  {
    Unit u;
    u.mcu = OUR_BUILD;
    u.nvs.s = {0x99999999, USER_NONE, 0, 0};  // stale
    CHECK(u.check() == CHECK_CURRENT);
    CHECK(u.nvs.s.arm == 0);
  }
  {
    Unit u;
    u.mcu = OUR_BUILD;
    CHECK(u.cmd(COMMAND_ARM));  // the user's own arm stays
    CHECK(u.state == CHECK_CURRENT && u.nvs.s.arm == u.image_crc);
    CHECK(u.check() == CHECK_CURRENT && u.nvs.s.arm == u.image_crc);
  }
}

// The MCU is later given another build (for example from ESPHome): the tool arms again.
static void test_not_current_later() {
  Unit u;
  CHECK(u.check() == CHECK_ARMED);
  CHECK(u.power_on(OUT_OK));
  CHECK(u.check() == CHECK_CURRENT);
  u.mcu = OUR_BUILD + 7;
  CHECK(u.check() == CHECK_ARMED);
}

static void test_should_disconnect() {
  using namespace stcisp;
  const uint8_t none = 0xFF;
  CHECK(!should_disconnect(false, STAGE_SYNC, none, 0));
  CHECK(should_disconnect(true, STAGE_STATUS, none, 0));  // a refusal
  CHECK(should_disconnect(true, STAGE_CALIBRATE, RESULT_CALIBRATION, 0));
  CHECK(should_disconnect(true, STAGE_BAUD, RESULT_TIMEOUT, 0));
  CHECK(should_disconnect(true, STAGE_HANDSHAKE, RESULT_LOCKED, 0));
  CHECK(!should_disconnect(true, STAGE_ERASE, RESULT_TIMEOUT, 0));
  CHECK(!should_disconnect(true, STAGE_WRITE, RESULT_TIMEOUT, 0));
  CHECK(!should_disconnect(true, STAGE_WRITE, RESULT_TIMEOUT, APP_BASE - BLOCK_SIZE));
  CHECK(should_disconnect(true, STAGE_WRITE, RESULT_TIMEOUT, APP_BASE));
  CHECK(should_disconnect(true, STAGE_WRITE, RESULT_FRAMING, APP_END - BLOCK_SIZE));
  CHECK(should_disconnect(true, STAGE_FINISH, RESULT_TIMEOUT, APP_END));
  CHECK(should_disconnect(true, STAGE_OPTIONS, RESULT_BAD_REPLY, APP_END));
  CHECK(should_disconnect(true, STAGE_DONE, RESULT_OK, APP_END));
}

static Bytes link_frame(uint8_t type, const Bytes &payload) {
  Bytes f(payload.size() + rpstd01::FRAME_OVERHEAD);
  rpstd01::build_frame(f.data(), type, payload.data(), (uint8_t) payload.size());
  return f;
}

// Bytes arrive at the link's baud rate after the given delay.
class FakeLink : public stcisp::Io {
 public:
  static constexpr uint32_t BYTE_US =
      (rpstd01::LINK_BYTE_BITS * 1000000 + rpstd01::LINK_BAUD - 1) / rpstd01::LINK_BAUD;
  std::deque<std::pair<uint32_t, uint8_t>> rx;  // arrival time, byte
  void set_baud(uint32_t) override {}
  void write(const uint8_t *, size_t) override {}
  bool available() override { return !rx.empty() && rx.front().first <= t_; }
  int read(uint32_t timeout_us) override {
    if (rx.empty() || rx.front().first > t_ + timeout_us) {
      t_ += timeout_us;
      return -1;
    }
    if (rx.front().first > t_)
      t_ = rx.front().first;
    const int c = rx.front().second;
    rx.pop_front();
    return c;
  }
  uint32_t now_us() override { return t_; }
  void delay_us(uint32_t us) override { t_ += us; }
  void send(uint32_t at, const Bytes &b) {
    for (uint8_t c : b) {
      rx.emplace_back(at, c);
      at += BYTE_US;
    }
  }

 private:
  uint32_t t_ = 0;
};

static const Bytes INFO_PAYLOAD = {0x00, 0x01, 0x02, 0x1F, 0x22, 0xC8, 1, 0, 3, 1};

static void test_listen_for_info_stops_at_info_frame_after_noise_and_statuses() {
  FakeLink io;
  io.send(50000, {0x00, 0xA5, 0x13});
  io.send(100000, link_frame(MSG_STATUS, Bytes(STATUS_LEN, 0)));
  io.send(300000, link_frame(MSG_STATUS, Bytes(STATUS_LEN, 1)));
  io.send(400000, link_frame(MSG_INFO, INFO_PAYLOAD));
  io.send(450000, link_frame(MSG_STATUS, Bytes(STATUS_LEN, 2)));
  rpstd01::Info i{};
  CHECK(listen_for_info(io, 1500000, &i));
  CHECK(i.build == 0xC8221F02 && i.boot_version == 3);
  CHECK(io.now_us() < 410000);
  CHECK(io.rx.size() == STATUS_LEN + rpstd01::FRAME_OVERHEAD);  // the last status frame is left unread
}

static void test_listen_for_info_does_not_wait_past_timeout() {
  const uint32_t timeout_us = 1500000;
  FakeLink io;
  io.send(timeout_us + 100000, link_frame(MSG_INFO, INFO_PAYLOAD));
  rpstd01::Info i{};
  CHECK(!listen_for_info(io, timeout_us, &i));
  CHECK(io.now_us() >= timeout_us && io.now_us() < timeout_us + 20000);
}

int main() {
  RUN(test_auto_arm_and_install);
  RUN(test_disarm_then_ota);
  RUN(test_disarm_while_current);
  RUN(test_refusal_then_user_arm);
  RUN(test_refusal_holds_across_tool_builds);
  RUN(test_fails_reset);
  RUN(test_failed_flash_power_cycle);
  RUN(test_repeated_pre_erase_failures);
  RUN(test_no_reply_not_counted);
  RUN(test_crash_counts);
  RUN(test_verify_check_disagree);
  RUN(test_incomplete_overrides_hold);
  RUN(test_incomplete_still_held_by_cancel_install);
  RUN(test_power_cut_retries_next_power_on);
  RUN(test_power_cut_retry_after_install_by_hand_is_automatic_and_counts);
  RUN(test_power_cut_retry_not_stopped_by_stale_arm);
  RUN(test_power_cut_retry_held_by_cancel_install);
  RUN(test_power_cut_retry_held_by_refusal);
  RUN(test_old_nvs);
  RUN(test_nvs_write_failure);
  RUN(test_arm_flag_that_cannot_be_cleared_at_power_on_runs_nothing);
  RUN(test_partial_cancel_install_writes_user_value_only);
  RUN(test_partial_cancel_install_repaired_by_check);
  RUN(test_partial_install_by_hand_after_giving_up_resets_count_only);
  RUN(test_failed_clear_of_leftover_arm_not_shown_as_users);
  RUN(test_current_clears_leftover);
  RUN(test_not_current_later);
  RUN(test_should_disconnect);
  RUN(test_listen_for_info_stops_at_info_frame_after_noise_and_statuses);
  RUN(test_listen_for_info_does_not_wait_past_timeout);

  TEST_EXIT();
}
