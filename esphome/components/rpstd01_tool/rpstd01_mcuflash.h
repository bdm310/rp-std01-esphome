// RP-STD01 MCU tool: the arming rules for an install through STC's bootloader.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esphome/components/rpstd01_link/rpstd01_link.h"
#include "esphome/components/rpstd01_tool/rpstd01_stc_isp.h"

namespace mcuflash {

// An attempt's results beyond stcisp::Result.
inline constexpr uint8_t RESULT_UNFINISHED = 0xFE;
inline constexpr uint8_t RESULT_NOT_STARTED = 0xFF;

// Disconnect only when what runs at 0x0000 is complete: our bootloader keeps SCR and relay low.
// program_result: a stcisp::Result, or RESULT_NOT_STARTED.
inline bool should_disconnect(bool answered, uint8_t stage, uint8_t program_result, uint32_t written) {
  if (!answered)
    return false;
  return stage < stcisp::STAGE_ERASE || program_result == stcisp::RESULT_OK || written >= rpstd01::APP_BASE;
}

// The last button (Command), or what the last power-on did with it. Other stored values read as
// USER_NONE.
enum User : uint8_t { USER_NONE = 0, USER_DISARMED = 1, USER_ARMED = 2 };

enum Command : uint8_t { COMMAND_ARM, COMMAND_DISARM };

enum Check : uint8_t {
  CHECK_ASK = 0,
  CHECK_CURRENT,     // nothing to install through STC's bootloader
  CHECK_ARMED,
  CHECK_ARM_FAILED,  // an install is due, but the arm flag could not be saved
  CHECK_REFUSED,     // the last attempt found a chip this tool must not program; held until COMMAND_ARM
  CHECK_DISARMED,    // held by COMMAND_DISARM until COMMAND_ARM
  CHECK_GAVE_UP,     // MAX_AUTO_FAILS automatic attempts in a row failed; held until COMMAND_ARM
};

inline constexpr uint8_t MAX_AUTO_FAILS = 3;

// Kept in NVS. A missing key reads as 0.
struct Store {
  uint32_t arm;     // 0, or the CRC-32 of the image to install; any other value is stale
  uint8_t user;     // User
  uint8_t fails;    // automatic attempts in a row that failed
  uint8_t partial;  // 1: an install stopped after the erase, so the MCU's firmware is incomplete.
                    // Also set while an install runs, so a power cut leaves it set.
};

inline bool operator==(const Store &a, const Store &b) {
  return a.arm == b.arm && a.user == b.user && a.fails == b.fails && a.partial == b.partial;
}
inline bool operator!=(const Store &a, const Store &b) { return !(a == b); }

// What a power-on took from the Store, for attempt_done().
struct Taken {
  bool run;
  bool automatic;  // not armed by COMMAND_ARM
  uint8_t partial_before;
};

// The end of a power-on attempt, from the record the tool keeps (IspRecord).
struct Attempt {
  uint8_t connect_result;  // stcisp::Result, or RESULT_UNFINISHED
  uint8_t program_result;  // stcisp::Result, or RESULT_NOT_STARTED
  uint8_t stage;
  uint32_t written;
};

inline void command(Store &s, Command c, uint32_t image_crc) {
  if (c == COMMAND_ARM) {
    s.arm = image_crc;
    s.user = USER_ARMED;
    s.fails = 0;
  } else {
    s.arm = 0;
    s.user = USER_DISARMED;
  }
}

// Results that will not change at the next power-on: the chip is not one this tool can program.
// Returns the stcisp::Result, or 0.
inline uint8_t refusal(uint8_t connect_result, uint8_t program_result) {
  switch (connect_result) {
    case stcisp::RESULT_WRONG_MODEL:
    case stcisp::RESULT_UNEXPECTED_OPTIONS:
    case stcisp::RESULT_EXTERNAL_CLOCK:
      return connect_result;
    default:
      return (program_result == stcisp::RESULT_LOCKED) ? program_result : 0;
  }
}

// Clears the flag before the attempt; an automatic attempt counts as failed and the firmware as
// incomplete until attempt_done(). refused: refusal() of the last attempt.
inline bool take_arm(Store &s, uint32_t image_crc, uint8_t refused, Taken *t) {
  t->run = false;
  t->automatic = false;
  t->partial_before = s.partial;
  if (s.user == USER_DISARMED) {
    s.arm = 0;  // also a flag left by a COMMAND_DISARM whose write stopped part way
    return false;
  }
  if (s.arm && s.arm == image_crc) {
    t->run = true;
    t->automatic = s.user != USER_ARMED;
  } else if (s.partial && !refused) {
    t->run = true;
    t->automatic = true;
  } else if (!s.arm) {
    return false;
  }
  if (t->run) {
    if (t->automatic && s.fails < 0xFF)
      s.fails++;
    s.partial = 1;
  }
  s.arm = 0;
  s.user = USER_NONE;
  return t->run;
}

// Before the erase the firmware is as it was; from the erase on it is complete only if the install
// ended well.
inline void attempt_done(Store &s, const Taken &t, const Attempt &a) {
  if (!t.run)
    return;
  const bool tried = a.connect_result != stcisp::RESULT_NO_REPLY && !refusal(a.connect_result, a.program_result);
  if (t.automatic && !tried && s.fails > 0)
    s.fails--;
  if (a.stage < stcisp::STAGE_ERASE)
    s.partial = t.partial_before;
  else
    s.partial = (a.program_result != stcisp::RESULT_OK);
}

// current: nothing to install by itself. check() saves s.
inline Check decide(Store &s, bool current, uint8_t refused, uint32_t image_crc) {
  if (current) {
    if (s.arm && s.user != USER_ARMED)
      s.arm = 0;  // a leftover automatic arm would only write the same image again
    if (s.user != USER_ARMED)
      s.user = USER_NONE;
    s.fails = 0;
    s.partial = 0;
    return CHECK_CURRENT;
  }
  if (s.user == USER_DISARMED)
    s.arm = 0;
  if (s.arm == image_crc)
    return CHECK_ARMED;
  if (refused)
    return CHECK_REFUSED;
  if (s.user == USER_DISARMED)
    return CHECK_DISARMED;
  if (!s.partial && s.fails >= MAX_AUTO_FAILS && s.user != USER_ARMED)
    return CHECK_GAVE_UP;
  s.arm = image_crc;
  return CHECK_ARMED;
}

// write_ok: false if the Store from decide() could not be saved.
inline Check stored(Check c, bool write_ok) { return (c == CHECK_ARMED && !write_ok) ? CHECK_ARM_FAILED : c; }

class StoreIo {
 public:
  enum Key : uint8_t { KEY_ARM, KEY_USER, KEY_FAILS, KEY_PARTIAL };
  virtual void load(Store *s) = 0;  // a missing key, or no NVS, reads as 0
  virtual bool set(Key key, uint32_t value) = 0;
  virtual bool commit() = 0;
};

// Writes the keys that changed, the arm flag last: a write that stops part way then leaves the
// old arm flag, never a new one without the values that go with it. For COMMAND_DISARM, the user
// value written first is enough to hold the install (take_arm). On failure, s is reloaded with what
// NVS holds.
inline bool save(StoreIo &io, const Store &old, Store &s) {
  const bool ok = (s.user == old.user || io.set(StoreIo::KEY_USER, s.user)) &&
                  (s.fails == old.fails || io.set(StoreIo::KEY_FAILS, s.fails)) &&
                  (s.partial == old.partial || io.set(StoreIo::KEY_PARTIAL, s.partial)) &&
                  (s.arm == old.arm || io.set(StoreIo::KEY_ARM, s.arm)) && io.commit();
  if (!ok)
    io.load(&s);
  return ok;
}

// At power-on. Runs nothing if the cleared flag cannot be saved.
inline bool power_on_take(StoreIo &io, uint32_t image_crc, uint8_t refused, Taken *t) {
  Store s;
  io.load(&s);
  const Store old = s;
  take_arm(s, image_crc, refused, t);
  if (s != old && !save(io, old, s))
    t->run = false;
  return t->run;
}

inline void power_on_done(StoreIo &io, const Taken &t, const Attempt &a) {
  Store s;
  io.load(&s);
  const Store old = s;
  attempt_done(s, t, a);
  if (s != old)
    save(io, old, s);
}

inline Check check(StoreIo &io, bool current, uint8_t refused, uint32_t image_crc, Store *now) {
  Store s;
  io.load(&s);
  const Store old = s;
  const Check c = decide(s, current, refused, image_crc);
  const bool ok = (s == old) || save(io, old, s);
  *now = s;
  return stored(c, ok);
}

inline bool run_command(StoreIo &io, Command c, uint32_t image_crc) {
  Store s;
  io.load(&s);
  const Store old = s;
  command(s, c, image_crc);
  return (s == old) || save(io, old, s);
}

// After an install: listens (LINK_BAUD 8N1, set up by the caller) for up to timeout_us for the info
// frame our application sends at start, and stops as soon as it arrives.
inline bool listen_for_info(stcisp::Io &io, uint32_t timeout_us, rpstd01::Info *info) {
  rpstd01::FrameParser parser;
  bool got = false;
  const uint32_t start = io.now_us();
  while (!got && (uint32_t) (io.now_us() - start) < timeout_us) {
    const int c = io.read(10000);
    if (c < 0)
      continue;
    parser.feed((uint8_t) c, [&got, info](uint8_t type, const uint8_t *payload, uint8_t len) {
      got = got || (type == rpstd01::MSG_INFO && rpstd01::decode_info(payload, len, info));
    });
  }
  return got;
}

}
