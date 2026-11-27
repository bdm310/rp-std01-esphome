// The MCU tool, with an MCU that never answers the application update over the link.

#include <cstdio>
#include <deque>
#include <string>
#include <vector>

#define RPSTD01TOOL_NVS_FAKE
#include "esphome/components/rpstd01_tool/rpstd01_tool_nvs.h"
#include "test.h"

using namespace rpstd01tool;
using Bytes = std::vector<uint8_t>;

static const uint8_t IMAGE[rpstd01::APP_END] = {0};  // bootloader and application: an app_area() for the link
static constexpr uint32_t CRC = 0x0A74B9FA;
static constexpr uint32_t BUILD = 0xFEE2D341;
static constexpr uint32_t OLD_BUILD = 0x84BFA001;
static constexpr uint8_t BOOT = 3;
static constexpr uint16_t VERSION = 0x0102;
static const Carried IMG{IMAGE, sizeof(IMAGE), CRC, VERSION, BUILD, BOOT};

static Bytes frame(uint8_t type, const Bytes &payload) {
  Bytes f(payload.size() + 5);
  f.resize(rpstd01::build_frame(f.data(), type, payload.data(), (uint8_t) payload.size()));
  return f;
}

// Our application's status, with the boot ID of info_payload().
static Bytes status_payload() {
  Bytes p(rpstd01::STATUS_LEN, 0);
  p[rpstd01::SF_BOOT_ID] = 1;
  return p;
}

static Bytes info_payload(uint32_t build, uint8_t boot = BOOT, uint16_t version = VERSION) {
  return {(uint8_t) version, (uint8_t) (version >> 8), (uint8_t) build, (uint8_t) (build >> 8), (uint8_t) (build >> 16),
          (uint8_t) (build >> 24), 1, 0, boot, 1};
}

static Bytes boot_status(uint8_t version, uint8_t app_valid) {
  const uint8_t reason = app_valid ? rpstd01::BOOT_REASON_REQUEST : rpstd01::BOOT_REASON_INVALID;
  return {version, app_valid, reason, rpstd01::BR_OK, 0, 0};
}

static Bytes longer(Bytes payload) {
  payload.resize(payload.size() + 4, 0);
  return payload;
}

static Seen seen_info(uint32_t build, uint8_t boot) {
  Seen s{};
  Bytes p = info_payload(build, boot);
  s.info = rpstd01::decode_info(p.data(), (uint8_t) p.size(), &s.app);
  return s;
}

static void test_classify() {
  CHECK(classify(seen_info(BUILD, BOOT), IMG) == VERDICT_CURRENT);
  CHECK(classify(seen_info(BUILD, BOOT + 1), IMG) == VERDICT_NEWER);  // newer bootloader: no downgrade
  CHECK(classify(seen_info(OLD_BUILD, BOOT), IMG) == VERDICT_APP_OLD);
  CHECK(classify(seen_info(BUILD, BOOT - 1), IMG) == VERDICT_BOOT_OLD);
  CHECK(classify(seen_info(OLD_BUILD, BOOT - 1), IMG) == VERDICT_BOOT_OLD);  // both old: the bootloader decides
  Seen boot{};
  boot.boot_status = true;
  boot.boot.version = BOOT;
  CHECK(classify(boot, IMG) == VERDICT_UPDATE_MODE);
  boot.boot.version = BOOT - 1;
  CHECK(classify(boot, IMG) == VERDICT_UPDATE_MODE);
  boot.boot.version = BOOT + 1;  // a newer bootloader in update mode: no downgrade by itself
  CHECK(classify(boot, IMG) == VERDICT_NEWER);
  CHECK(classify(Seen{}, IMG) == VERDICT_NO_ANSWER);
  Seen statuses{};
  statuses.unreadable = true;
  CHECK(classify(statuses, IMG) == VERDICT_UNKNOWN_APP);
}

static void test_holds_auto_install() {
  for (Verdict v : {VERDICT_CURRENT, VERDICT_NEWER, VERDICT_UNKNOWN_APP})
    CHECK(holds_auto_install(v, Seen{}));
  for (Verdict v : {VERDICT_APP_OLD, VERDICT_BOOT_OLD, VERDICT_UPDATE_MODE, VERDICT_NO_ANSWER, VERDICT_LINK_FAULT})
    CHECK(!holds_auto_install(v, Seen{}));
  Seen valid_app{};
  valid_app.boot_status = true;
  valid_app.boot.app_valid = 1;  // its version is unknown: it may be newer
  CHECK(holds_auto_install(VERDICT_UPDATE_MODE, valid_app));
}

static void test_link_update_handles_app_and_invalid_app_at_carried_bootloader() {
  CHECK(link_update_handles(VERDICT_APP_OLD, seen_info(OLD_BUILD, BOOT), IMG));
  Seen boot{};
  boot.boot_status = true;
  boot.boot.version = BOOT;
  CHECK(link_update_handles(VERDICT_UPDATE_MODE, boot, IMG));
  boot.boot.app_valid = 1;
  CHECK(!link_update_handles(VERDICT_UPDATE_MODE, boot, IMG));
  boot.boot.app_valid = 0;
  boot.boot.version = BOOT - 1;
  CHECK(!link_update_handles(VERDICT_UPDATE_MODE, boot, IMG));
  for (Verdict v : {VERDICT_CURRENT, VERDICT_BOOT_OLD, VERDICT_NO_ANSWER, VERDICT_NEWER, VERDICT_LINK_FAULT})
    CHECK(!link_update_handles(v, seen_info(OLD_BUILD, BOOT), IMG));
}

static void test_app_area() {
  CHECK(app_area(IMG) == IMAGE + rpstd01::APP_BASE);
  CHECK(app_area(Carried{IMAGE, 512, CRC, 3, BUILD, BOOT}) == nullptr);
}

static IspRecord sample_record(uint32_t written) {
  IspRecord r{};
  r.connect_result = stcisp::RESULT_OK;
  r.program_result = stcisp::RESULT_OK;
  r.written = written;
  r.app_build = BUILD;
  return r;
}

static void put_record_blob(const char *key, const Bytes &blob) {
  nvs_handle_t h;
  nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
  nvs_set_blob(h, key, blob.data(), blob.size());
  nvs_commit(h);
  nvs_close(h);
}

// A record of another size, that a refusal would hold the install for if it were read.
static void put_old_record(size_t size) {
  Bytes old(size, 0);
  old[0] = ISP_RECORD_VERSION;
  old[1] = stcisp::RESULT_WRONG_MODEL;
  put_record_blob(NVS_RECORD, old);
}

static void test_nvs_store_survives_restart() {
  fake_nvs::reset();
  {
    NvsToolStore st(true);
    CHECK(st.is_open());
    mcuflash::Store s{1, 2, 3, 4};
    st.load(&s);
    CHECK(s == (mcuflash::Store{0, mcuflash::USER_NONE, 0, 0}));  // missing keys read as 0
    IspRecord r;
    CHECK(!st.load_record(&r));

    CHECK(st.set(mcuflash::StoreIo::KEY_ARM, CRC) && st.set(mcuflash::StoreIo::KEY_USER, mcuflash::USER_ARMED));
    CHECK(st.set(mcuflash::StoreIo::KEY_FAILS, 2) && st.set(mcuflash::StoreIo::KEY_PARTIAL, 1));
    CHECK(st.commit());
    CHECK(st.save_record(sample_record(100)));
    CHECK(st.save_record(sample_record(sizeof(IMAGE))));
  }
  fake_nvs::restart();
  {
    NvsToolStore st(false);
    mcuflash::Store s;
    st.load(&s);
    CHECK(s == (mcuflash::Store{CRC, mcuflash::USER_ARMED, 2, 1}));
    IspRecord r;
    CHECK(st.load_record(&r) && r.written == sizeof(IMAGE) && r.app_build == BUILD);
    CHECK(!st.set(mcuflash::StoreIo::KEY_ARM, 0));  // read only
  }
}

static void test_nvs_store_ignores_same_keys_in_other_namespace() {
  fake_nvs::reset();
  {
    NvsToolStore st(true);
    CHECK(st.set(mcuflash::StoreIo::KEY_ARM, CRC) && st.commit());
  }
  nvs_handle_t h;
  nvs_open("rpstd01", NVS_READWRITE, &h);
  nvs_set_u32(h, "isp_arm", 0x11111111);
  nvs_commit(h);
  nvs_close(h);
  NvsToolStore st(false);
  mcuflash::Store s;
  st.load(&s);
  CHECK(s.arm == CRC);
}

static void test_nvs_record_of_other_size_or_version_is_no_record() {
  fake_nvs::reset();
  IspRecord r;
  for (size_t size : {ISP_RECORD_SIZE - 1, ISP_RECORD_SIZE + 1, (size_t) 160}) {
    put_old_record(size);
    NvsToolStore st(false);
    CHECK(!st.load_record(&r));
  }
  uint8_t bytes[ISP_RECORD_SIZE];
  encode_record(sample_record(sizeof(IMAGE)), bytes);
  bytes[0] = ISP_RECORD_VERSION + 1;
  put_record_blob(NVS_RECORD, Bytes(bytes, bytes + sizeof(bytes)));
  NvsToolStore st(false);
  CHECK(!st.load_record(&r));
  bytes[0] = ISP_RECORD_VERSION;
  put_record_blob(NVS_RECORD, Bytes(bytes, bytes + sizeof(bytes)));
  CHECK(st.load_record(&r) && r.written == sizeof(IMAGE));
}

static void test_record_round_trip() {
  IspRecord r{stcisp::RESULT_OK, stcisp::RESULT_TIMEOUT, stcisp::STAGE_WRITE, 1, 4, 0x0102, 0x2000, 0xDEADBEEF};
  uint8_t bytes[ISP_RECORD_SIZE];
  encode_record(r, bytes);
  IspRecord d{};
  CHECK(decode_record(bytes, sizeof(bytes), &d));
  CHECK(d.connect_result == r.connect_result && d.program_result == r.program_result && d.stage == r.stage);
  CHECK(d.app_seen == r.app_seen && d.app_boot_version == r.app_boot_version && d.app_version == r.app_version);
  CHECK(d.written == r.written && d.app_build == r.app_build);
}

static void test_nvs_that_cannot_open_reads_empty_and_fails_to_arm() {
  fake_nvs::reset();
  fake_nvs::state().open_fails = true;
  NvsToolStore st(true);
  CHECK(!st.is_open());
  mcuflash::Store s{1, 1, 1, 1};
  st.load(&s);
  CHECK(s == (mcuflash::Store{0, 0, 0, 0}));
  CHECK(!st.set(mcuflash::StoreIo::KEY_ARM, 1) && !st.commit());
  CHECK(!st.save_record(sample_record(sizeof(IMAGE))));
  mcuflash::Store now;
  CHECK(mcuflash::check(st, false, 0, CRC, &now) == mcuflash::CHECK_ARM_FAILED);
}

static void test_nvs_failed_commit_reported_by_save() {
  fake_nvs::reset();
  fake_nvs::state().commit_fails = true;
  {
    NvsToolStore st(true);
    CHECK(!st.save_record(sample_record(sizeof(IMAGE))));
    CHECK(!mcuflash::run_command(st, mcuflash::COMMAND_ARM, CRC));
  }
  fake_nvs::reset();
}

// What the scripted programmer does at the next power-on.
enum Outcome {
  OUT_OK,          // programmed; the new application answers
  OUT_SILENT_APP,  // programmed; the new application does not answer
  OUT_NO_REPLY,    // the bootloader was not caught
  OUT_WRONG_MODEL, // refused before anything was changed
  OUT_CALIBRATION, // fails before the erase
  OUT_WRITE_LATE,  // fails while writing the application (after our bootloader)
};

struct Mcu {
  // LATER_APP and LATER_BOOTLOADER: a later release whose frames have other lengths.
  enum Kind { STOCK, APP, BOOTLOADER, BROKEN, STATUSES_ONLY, LATER_APP, LATER_BOOTLOADER } kind = STOCK;
  uint32_t build = 0;
  uint8_t boot = 0;
  uint8_t app_valid = 1;
  uint16_t version = VERSION;
  bool runs_on_request = true;  // MSG_BOOT_RUN starts a valid application
  Kind runs_as = APP;
  bool info_at_start = true;
};

static Mcu mcu;
static Outcome next_outcome = OUT_OK;
static uint32_t boot_status_from_ms = 0;  // the bootloader's status frames start then
static bool link_dead = false;            // the check's link carries nothing either way
static bool image_pending = false;        // as the ESP32 glue reports it
static int programs = 0, connects = 0, disconnects = 0;

class ScriptedProgrammer {
 public:
  explicit ScriptedProgrammer(stcisp::Io &io) : io_(io) {}
  uint8_t connect(uint16_t, uint32_t, uint32_t) {
    connects++;
    if (next_outcome == OUT_NO_REPLY)
      return stcisp::RESULT_NO_REPLY;
    session_.status_len = 40;
    session_.stage = stcisp::STAGE_STATUS;
    session_.magic = next_outcome == OUT_WRONG_MODEL ? 0xF4A1 : MCU_MODEL;
    return next_outcome == OUT_WRONG_MODEL ? stcisp::RESULT_WRONG_MODEL : stcisp::RESULT_OK;
  }
  uint8_t program(const uint8_t *, size_t, uint32_t, uint32_t) {
    programs++;
    switch (next_outcome) {
      case OUT_CALIBRATION:
        session_.stage = stcisp::STAGE_CALIBRATE;
        return stcisp::RESULT_CALIBRATION;
      case OUT_WRITE_LATE:
        session_.stage = stcisp::STAGE_WRITE;
        session_.written = 0x2000;
        mcu.kind = Mcu::BROKEN;
        return stcisp::RESULT_TIMEOUT;
      default:
        session_.stage = stcisp::STAGE_DONE;
        session_.written = sizeof(IMAGE);
        mcu = Mcu{next_outcome == OUT_OK ? Mcu::APP : Mcu::BROKEN, BUILD, BOOT};
        return stcisp::RESULT_OK;
    }
  }
  void disconnect() { disconnects++; }
  const stcisp::Session &session() const { return this->session_; }
  stcisp::Session session_{};

 private:
  stcisp::Io &io_;
};

// The power-on port: after programming, the new application's info frame arrives.
class FakePowerOnIo : public PowerOnIo {
 public:
  static constexpr uint32_t BYTE_US =
      (rpstd01::LINK_BYTE_BITS * 1000000 + rpstd01::LINK_BAUD - 1) / rpstd01::LINK_BAUD;
  void begin() override { this->begun = true; }
  void set_baud(uint32_t baud) override { this->baud = baud; }
  void set_no_parity() override {
    this->no_parity = true;
    if (mcu.kind == Mcu::APP && this->rx.empty()) {
      Bytes f = frame(rpstd01::MSG_INFO, info_payload(mcu.build, mcu.boot));
      this->rx.assign(f.begin(), f.end());
    }
  }
  void write(const uint8_t *, size_t) override {}
  bool available() override { return !this->rx.empty(); }
  int read(uint32_t timeout_us) override {
    if (this->rx.empty()) {
      this->t += timeout_us;
      return -1;
    }
    this->t += BYTE_US;
    const int c = this->rx.front();
    this->rx.pop_front();
    return c;
  }
  uint32_t now_us() override { return this->t; }
  void delay_us(uint32_t us) override { this->t += us; }
  std::deque<uint8_t> rx;
  uint32_t t = 0, baud = 0;
  bool begun = false, no_parity = false;
};

// The link as ESPHome's UART shows it to Tool::loop. The MCU answers info requests.
struct FakeUart {
  std::deque<uint8_t> rx;
  int requests = 0, runs = 0;
  std::vector<uint32_t> request_ms;
  uint32_t now = 0;
  size_t available() const { return this->rx.size(); }
  bool read_byte(uint8_t *b) {
    if (this->rx.empty())
      return false;
    *b = this->rx.front();
    this->rx.pop_front();
    return true;
  }
  void push(const Bytes &f) { this->rx.insert(this->rx.end(), f.begin(), f.end()); }
  // The tool writes one whole frame at a time.
  void write_array(const uint8_t *data, size_t len) {
    if (link_dead || len < 2)
      return;
    if (data[1] == rpstd01::MSG_BOOT_RUN) {
      this->runs++;
      if (mcu.kind == Mcu::BOOTLOADER && mcu.app_valid && mcu.runs_on_request) {
        mcu.kind = mcu.runs_as;
        if (mcu.kind == Mcu::APP && mcu.info_at_start)
          this->push(frame(rpstd01::MSG_INFO, info_payload(mcu.build, mcu.boot, mcu.version)));
        if (mcu.kind == Mcu::LATER_APP)
          this->push(frame(rpstd01::MSG_INFO, longer(info_payload(mcu.build, mcu.boot, mcu.version))));
      }
      return;
    }
    if (data[1] != rpstd01::MSG_INFO_REQ)
      return;
    this->requests++;
    this->request_ms.push_back(this->now);
    if (mcu.kind == Mcu::APP)
      this->push(frame(rpstd01::MSG_INFO, info_payload(mcu.build, mcu.boot, mcu.version)));
    if (mcu.kind == Mcu::LATER_APP)
      this->push(frame(rpstd01::MSG_INFO, longer(info_payload(BUILD))));
  }
};

struct Boot {
  uint8_t state;
  Verdict verdict;
  bool attempted;
  IspRecord rec;
  std::string text;
  int requests, runs;
  uint32_t decided_ms;
  mcuflash::Store nvs;
};

static NvsToolStore *tool_store = nullptr;
static Tool *last_tool = nullptr;

static constexpr uint32_t FIRST_LOOP_MS = 100, LOOP_MS = 16, START_END_MS = FIRST_LOOP_MS + 5000;

// Frames the MCU sends every period_ms, from from_ms on, in the loop that ends at now.
static int frames_in_loop(uint32_t now, uint32_t period_ms, uint32_t from_ms = 0) {
  int n = 0;
  for (uint32_t t = now - LOOP_MS + 1; t <= now; t++)
    n += t % period_ms == 0 && t >= from_ms;
  return n;
}

static void push_frames(FakeUart &uart, int n, uint8_t type, const Bytes &payload) {
  for (int i = 0; i < n; i++)
    uart.push(frame(type, payload));
}

// One start of the ESP32: the constructor (at a power-on), then the check.
template<typename Prog = ScriptedProgrammer> static Boot start(bool power_on_reset) {
  fake_nvs::restart();
  Boot b{};
  if (power_on_reset) {
    NvsToolStore st(true);
    FakePowerOnIo io;
    b.attempted = power_on<Prog>(st, io, IMG, &b.rec);
    if (b.attempted)
      CHECK(io.begun && io.no_parity && io.baud == rpstd01::LINK_BAUD);
    else
      CHECK(!io.begun);  // the UART is not touched without an attempt
  }
  delete last_tool;
  delete tool_store;
  tool_store = new NvsToolStore(true);
  last_tool = new Tool();
  last_tool->begin(tool_store, IMG, b.attempted ? &b.rec : nullptr);
  last_tool->set_image_pending(image_pending);
  FakeUart uart;
  for (uint32_t now = FIRST_LOOP_MS; now < START_END_MS; now += LOOP_MS) {
    uart.now = now;
    if (!link_dead) {
      const int statuses = frames_in_loop(now, rpstd01::STATUS_PERIOD_MS);
      const int boot_statuses = frames_in_loop(now, rpstd01::BOOT_STATUS_PERIOD_MS, boot_status_from_ms);
      if (mcu.kind == Mcu::BOOTLOADER)
        push_frames(uart, boot_statuses, rpstd01::MSG_BOOT_STATUS, boot_status(mcu.boot, mcu.app_valid));
      if (mcu.kind == Mcu::APP || mcu.kind == Mcu::STATUSES_ONLY)
        push_frames(uart, statuses, rpstd01::MSG_STATUS, status_payload());
      if (mcu.kind == Mcu::LATER_APP)
        push_frames(uart, statuses, rpstd01::MSG_STATUS, longer(status_payload()));
      if (mcu.kind == Mcu::LATER_BOOTLOADER)
        push_frames(uart, boot_statuses, rpstd01::MSG_BOOT_STATUS, longer(boot_status(BOOT + 1, 1)));
    }
    const uint8_t before = last_tool->state();
    last_tool->loop(uart, now);
    if (before <= mcuflash::CHECK_ASK && last_tool->decided())
      b.decided_ms = now;
  }
  b.state = last_tool->state();
  b.verdict = last_tool->verdict();
  char text[400];
  last_tool->state_text(text, sizeof(text));
  b.text = text;
  b.requests = uart.requests;
  b.runs = uart.runs;
  b.nvs = last_tool->nvs();
  return b;
}

static int requests_without_answer() { return (CHECK_LISTEN_MS + CHECK_ASK_EVERY_MS - 1) / CHECK_ASK_EVERY_MS; }

static bool has(const std::string &text, const char *part) { return text.find(part) != std::string::npos; }

static void fresh(Mcu m) {
  fake_nvs::reset();
  mcu = m;
  next_outcome = OUT_OK;
  programs = connects = disconnects = 0;
  boot_status_from_ms = 0;
  link_dead = false;
  image_pending = false;
}

// The factory MCU firmware: no answer, so an install is armed; the power cycle installs; the next
// check finds everything current.
static void test_first_install() {
  fresh(Mcu{});
  Boot b = start(true);  // a power-on with nothing armed
  CHECK(!b.attempted);
  CHECK(b.verdict == VERDICT_NO_ANSWER && b.state == mcuflash::CHECK_ARMED);
  CHECK(b.requests == requests_without_answer());
  const uint32_t no_answer_ms = FIRST_LOOP_MS + CHECK_LISTEN_MS;
  CHECK(b.decided_ms >= no_answer_ms && b.decided_ms < no_answer_ms + 2 * LOOP_MS);
  CHECK(b.nvs.arm == CRC);
  CHECK(has(b.text, "does not answer our protocol") && has(b.text, "press the RESET button"));

  b = start(false);  // a restart does not install
  CHECK(!b.attempted && b.state == mcuflash::CHECK_ARMED && programs == 0);

  b = start(true);
  CHECK(b.attempted && programs == 1 && disconnects == 1);
  CHECK(b.rec.program_result == stcisp::RESULT_OK);
  CHECK(b.rec.app_seen == 1 && b.rec.app_build == BUILD && b.rec.app_boot_version == BOOT);
  CHECK(b.verdict == VERDICT_CURRENT && b.state == mcuflash::CHECK_CURRENT);
  CHECK(b.nvs == (mcuflash::Store{0, mcuflash::USER_NONE, 0, 0}));
  CHECK(b.text == "Everything is current: install the main configuration.");
  CHECK(b.requests == 1 && b.decided_ms < FIRST_LOOP_MS + 2 * LOOP_MS);
  char text[200];
  last_tool->last_install_text(text, sizeof(text));
  CHECK(std::string(text) == "Install OK: application 0102 build FEE2D341, bootloader v3, at this power-on");

  b = start(true);  // stays current; nothing more is written
  CHECK(!b.attempted && programs == 1 && b.state == mcuflash::CHECK_CURRENT);
  last_tool->last_install_text(text, sizeof(text));
  CHECK(std::string(text) == "Install OK: application 0102 build FEE2D341, bootloader v3");
}

// Our application with an older bootloader, or our bootloader in update mode at an older version:
// each arms an install.
static void test_out_of_date_arms() {
  const struct {
    Mcu m;
    Verdict v;
    const char *text;
  } cases[] = {
      {Mcu{Mcu::APP, OLD_BUILD, BOOT - 1}, VERDICT_BOOT_OLD, "older than the carried one"},
      {Mcu{Mcu::APP, BUILD, BOOT - 1}, VERDICT_BOOT_OLD, "older than the carried one"},
      {Mcu{Mcu::BOOTLOADER, 0, BOOT - 1, 0}, VERDICT_UPDATE_MODE, "in update mode, with a bootloader older than"},
      {Mcu{Mcu::BOOTLOADER, OLD_BUILD, BOOT - 1}, VERDICT_BOOT_OLD, "older than the carried one"},
  };
  for (const auto &c : cases) {
    fresh(c.m);
    Boot b = start(false);
    CHECK(b.verdict == c.v && b.state == mcuflash::CHECK_ARMED && b.nvs.arm == CRC);
    CHECK(has(b.text, c.text));
    if (c.m.kind == Mcu::BOOTLOADER)
      CHECK(b.requests == 1 && b.runs == c.m.app_valid);  // the request before its boot status arrived
    b = start(true);
    CHECK(b.attempted && b.state == mcuflash::CHECK_CURRENT);
  }
}

// Only the application out of date, or our bootloader in update mode at the carried version: an
// update over the link at once; nothing is armed and no power cycle is asked for.
static void test_link_path() {
  const struct {
    Mcu m;
    const char *text;
    UpdateState upd;
  } cases[] = {
      {Mcu{Mcu::APP, OLD_BUILD, BOOT},
       "The MCU's application is another build (its bootloader is current). Updating the application through "
       "our bootloader (no power cycle needed): Requesting.",
       UpdateState::REQUESTING},
      {Mcu{Mcu::BOOTLOADER, 0, BOOT, 0},
       "The MCU is in our bootloader's update mode with no valid application. Updating the application through "
       "our bootloader (no power cycle needed): Writing 0 %.",
       UpdateState::WRITING},
  };
  for (const auto &c : cases) {
    fresh(c.m);
    Boot b = start(true);
    CHECK(!b.attempted && b.state == mcuflash::CHECK_CURRENT && b.nvs.arm == 0);
    CHECK(b.text == c.text);
    CHECK(last_tool->updater().update_state() == c.upd);
    CHECK(!has(b.text, "press the RESET button"));
  }
}

static void test_link_update_never_answered_fails_then_arm_by_hand() {
  fresh(Mcu{Mcu::APP, OLD_BUILD, BOOT});
  start(false);
  FakeUart uart;
  for (uint32_t now = START_END_MS; now < 8000; now += LOOP_MS)
    last_tool->loop(uart, now);
  char text[512];
  last_tool->state_text(text, sizeof(text));
  CHECK(has(text, "failed: MCU did not enter update mode. Press Update Application Now to try again, or Install at "
                  "Next Power-On"));
  CHECK(last_tool->nvs().arm == 0);
  CHECK(last_tool->command(mcuflash::COMMAND_ARM));
  last_tool->state_text(text, sizeof(text));
  CHECK(has(text, "Install at Next Power-On is set"));
}

// The arming rules' holds, through the buttons.
static void test_buttons() {
  fresh(Mcu{});
  Boot b = start(false);
  CHECK(b.state == mcuflash::CHECK_ARMED);

  CHECK(last_tool->command(mcuflash::COMMAND_DISARM));
  CHECK(last_tool->state() == mcuflash::CHECK_DISARMED && last_tool->nvs().arm == 0);
  b = start(true);
  CHECK(!b.attempted && b.state == mcuflash::CHECK_DISARMED && has(b.text, "Cancel Install"));

  CHECK(last_tool->command(mcuflash::COMMAND_ARM));
  CHECK(last_tool->state() == mcuflash::CHECK_ARMED && last_tool->nvs().user == mcuflash::USER_ARMED);
  b = start(true);
  CHECK(b.attempted && programs == 1 && b.state == mcuflash::CHECK_CURRENT);
}

static void test_install_by_hand_while_current_written_at_next_power_on() {
  fresh(Mcu{Mcu::APP, BUILD, BOOT});
  Boot b = start(false);
  CHECK(b.state == mcuflash::CHECK_CURRENT);
  CHECK(last_tool->command(mcuflash::COMMAND_ARM));
  CHECK(last_tool->state() == mcuflash::CHECK_CURRENT && last_tool->nvs().arm == CRC);
  char text[200];
  last_tool->state_text(text, sizeof(text));
  CHECK(has(text, "Install at Next Power-On is set"));
  b = start(true);
  CHECK(b.attempted && programs == 1 && b.state == mcuflash::CHECK_CURRENT && b.nvs.arm == 0);
}

// Three failed automatic attempts, then no more until Install at Next Power-On. Power-ons where the
// bootloader was not caught do not count.
static void test_gives_up() {
  fresh(Mcu{});
  start(false);
  next_outcome = OUT_NO_REPLY;
  Boot b = start(true);
  CHECK(b.attempted && b.rec.connect_result == stcisp::RESULT_NO_REPLY && disconnects == 0);
  CHECK(b.state == mcuflash::CHECK_ARMED && b.nvs.fails == 0);
  next_outcome = OUT_CALIBRATION;
  for (int i = 1; i <= 3; i++) {
    b = start(true);
    CHECK(b.attempted && b.nvs.fails == i && b.nvs.partial == 0);
    CHECK(b.state == (i < 3 ? mcuflash::CHECK_ARMED : mcuflash::CHECK_GAVE_UP));
  }
  CHECK(has(b.text, "last 3 automatic attempts failed"));
  b = start(true);
  CHECK(!b.attempted && b.state == mcuflash::CHECK_GAVE_UP);
  last_tool->command(mcuflash::COMMAND_ARM);
  CHECK(last_tool->state() == mcuflash::CHECK_ARMED);
  next_outcome = OUT_OK;
  b = start(true);
  CHECK(b.attempted && b.state == mcuflash::CHECK_CURRENT && b.nvs.fails == 0);
}

// A refusal holds until Install at Next Power-On.
static void test_refusal() {
  fresh(Mcu{});
  start(false);
  next_outcome = OUT_WRONG_MODEL;
  Boot b = start(true);
  CHECK(b.attempted && programs == 0 && disconnects == 1);
  CHECK(b.state == mcuflash::CHECK_REFUSED && b.nvs.arm == 0 && has(b.text, "wrong model"));
  b = start(true);
  CHECK(!b.attempted && b.state == mcuflash::CHECK_REFUSED);
  char text[200];
  last_tool->last_install_text(text, sizeof(text));
  CHECK(std::string(text) == "Install refused: wrong model. Nothing was changed");
  CHECK(last_tool->command(mcuflash::COMMAND_ARM));
  CHECK(last_tool->state() == mcuflash::CHECK_ARMED);
  next_outcome = OUT_OK;
  b = start(true);
  CHECK(b.attempted && b.state == mcuflash::CHECK_CURRENT);
}

// A failure after the erase leaves the firmware incomplete: every power-on installs again, even
// without the flag, until one completes. A written image whose application does not answer counts as
// failed until the check confirms it.
static void test_incomplete() {
  fresh(Mcu{});
  start(false);
  next_outcome = OUT_WRITE_LATE;
  Boot b = start(true);
  CHECK(b.attempted && disconnects == 1);  // our bootloader is complete, so the disconnect is sent
  CHECK(b.nvs.partial == 1 && b.state == mcuflash::CHECK_ARMED && has(b.text, "incomplete"));
  // As if the power went off again before the check could arm: the next power-on still installs.
  delete last_tool;
  last_tool = nullptr;
  delete tool_store;
  tool_store = nullptr;
  fake_nvs::restart();
  {
    NvsToolStore st(true);
    CHECK(st.set(mcuflash::StoreIo::KEY_ARM, 0) && st.commit());
  }
  next_outcome = OUT_SILENT_APP;
  b = start(true);
  CHECK(b.attempted);  // incomplete: installs without the flag
  CHECK(b.rec.program_result == stcisp::RESULT_OK && b.rec.app_seen == 0);
  char text[200];
  last_tool->last_install_text(text, sizeof(text));
  CHECK(std::string(text) == "Install written, but the new firmware did not answer, at this power-on");
  CHECK(b.state == mcuflash::CHECK_ARMED && b.nvs.partial == 0);
}

// Answers the sync bytes (2400 baud 8E1) with a status packet, or never.
static uint16_t stc_magic = MCU_MODEL;
static uint8_t stc_msr2 = 0x99;  // long power-on delay, internal clock, reset pin off, P2.0 high
static bool stc_silent = false;

class FakeStcIo : public FakePowerOnIo {
 public:
  void write(const uint8_t *data, size_t len) override {
    this->t += stcisp::HANDSHAKE_CHAR_US * len;
    if (stc_silent || !this->rx.empty() || this->answered)
      return;
    for (size_t i = 0; i < len; i++) {
      if (data[i] == 0x7F && ++this->syncs == 3) {
        uint8_t st[40] = {0x50};
        st[5] = 0xFF;
        st[6] = 0xFF;
        st[7] = stc_msr2;
        st[12] = 0xFF;  // pin check off
        st[17] = 0x72;
        st[18] = 'T';
        st[20] = stc_magic >> 8;
        st[21] = stc_magic & 0xFF;
        Bytes p = {0x46, 0xB9, 0x68, 0x00, (uint8_t) (sizeof(st) + 6)};
        p.insert(p.end(), st, st + sizeof(st));
        uint16_t sum = 0;
        for (size_t k = 2; k < p.size(); k++)
          sum += p[k];
        p.push_back(sum >> 8);
        p.push_back(sum & 0xFF);
        p.push_back(0x16);
        this->rx.assign(p.begin(), p.end());
        this->answered = true;
      }
    }
  }
  // Polling takes time, as on the ESP32.
  uint32_t now_us() override { return this->t += 10; }
  int syncs = 0;
  bool answered = false;
};

static bool run_real(IspRecord *rec) {
  NvsToolStore st(true);
  FakeStcIo io;
  return power_on<stcisp::Programmer>(st, io, IMG, rec);
}

static void test_real_programmer() {
  fresh(Mcu{});
  start(false);
  IspRecord rec;
  stc_silent = true;
  fake_nvs::restart();
  CHECK(run_real(&rec));
  CHECK(rec.connect_result == stcisp::RESULT_NO_REPLY && rec.program_result == mcuflash::RESULT_NOT_STARTED);

  stc_silent = false;
  stc_magic = 0xF4A1;
  start(false);
  fake_nvs::restart();
  CHECK(run_real(&rec));
  CHECK(rec.connect_result == stcisp::RESULT_WRONG_MODEL && rec.stage == stcisp::STAGE_STATUS);
  Boot b = start(false);
  CHECK(b.state == mcuflash::CHECK_REFUSED);

  stc_magic = MCU_MODEL;
  stc_msr2 = 0x19;  // short power-on delay
  CHECK(last_tool->command(mcuflash::COMMAND_ARM));
  fake_nvs::restart();
  CHECK(run_real(&rec));
  CHECK(rec.connect_result == stcisp::RESULT_UNEXPECTED_OPTIONS && rec.program_result == mcuflash::RESULT_NOT_STARTED);
  b = start(false);
  CHECK(b.state == mcuflash::CHECK_REFUSED);
  char text[400];
  last_tool->last_install_text(text, sizeof(text));
  CHECK(std::string(text) == "Install refused: unexpected options. Nothing was changed");
  stc_msr2 = 0x99;
}

static void test_texts() {
  fresh(Mcu{Mcu::APP, BUILD, BOOT});
  Boot b = start(false);
  char text[400];
  last_tool->installed_text(text, sizeof(text));
  CHECK(std::string(text) == "Application 0102 build FEE2D341, bootloader v3");
  last_tool->carried_text(text, sizeof(text));
  CHECK(std::string(text) == "Application 0102 build FEE2D341, bootloader v3 (" + std::to_string(sizeof(IMAGE)) +
                                 " bytes, CRC-32 0A74B9FA)");
  last_tool->armed_text(text, sizeof(text));
  CHECK(std::string(text) == "Nothing. Last choice: none. Failed automatic attempts: 0. Firmware incomplete: no");
  last_tool->last_install_text(text, sizeof(text));
  CHECK(std::string(text) == "None");
}

static void test_state_text_cut_to_small_buffer() {
  fresh(Mcu{Mcu::APP, BUILD, BOOT});
  start(false);
  char small[16];
  memset(small, 'x', sizeof(small));
  last_tool->state_text(small, 10);
  CHECK(strlen(small) == 9 && small[10] == 'x');
}

static void test_installed_text_no_answer() {
  fresh(Mcu{});
  start(false);
  char text[400];
  last_tool->installed_text(text, sizeof(text));
  CHECK(has(text, "No answer"));
}

// Runs the decided tool from from_ms to to_ms; the MCU sends what start() would.
static void run_more(uint32_t from_ms, uint32_t to_ms) {
  FakeUart uart;
  for (uint32_t now = from_ms; now < to_ms; now += LOOP_MS) {
    uart.now = now;
    if (mcu.kind == Mcu::BOOTLOADER)
      push_frames(uart, frames_in_loop(now, rpstd01::BOOT_STATUS_PERIOD_MS), rpstd01::MSG_BOOT_STATUS,
                  boot_status(mcu.boot, mcu.app_valid));
    last_tool->loop(uart, now);
  }
}

// An update through our bootloader changes nothing in NVS until its result is known.
static void test_link_start_keeps_cancel_install() {
  fresh(Mcu{Mcu::APP, OLD_BUILD, BOOT});
  Boot b = start(false);
  CHECK(last_tool->command(mcuflash::COMMAND_DISARM));
  CHECK(last_tool->state() == mcuflash::CHECK_CURRENT && last_tool->nvs().user == mcuflash::USER_DISARMED);
  b = start(false);
  CHECK(b.state == mcuflash::CHECK_CURRENT && b.nvs.user == mcuflash::USER_DISARMED);
}

static void test_link_start_keeps_install_armed_by_hand() {
  fresh(Mcu{Mcu::APP, OLD_BUILD, BOOT});
  start(false);
  CHECK(last_tool->command(mcuflash::COMMAND_ARM));
  Boot b = start(false);
  CHECK(b.nvs.arm == CRC && b.nvs.user == mcuflash::USER_ARMED);
  CHECK(has(b.text, "Install at Next Power-On is set"));
}

// An STC install cut off after the bootloader leaves our bootloader with no valid application.
// The link repairs it; when that fails too and the automatic repair is used up, the install is
// armed again and the incomplete firmware is shown.
static void test_link_repair_fails() {
  fresh(Mcu{});
  start(false);
  next_outcome = OUT_WRITE_LATE;
  Boot b = start(true);
  CHECK(b.attempted && b.nvs.partial == 1 && b.nvs.arm == CRC);
  mcu = Mcu{Mcu::BOOTLOADER, 0, BOOT, 0};
  b = start(false);
  CHECK(b.verdict == VERDICT_UPDATE_MODE && b.state == mcuflash::CHECK_CURRENT);
  CHECK(b.nvs.partial == 1 && b.nvs.arm == CRC);  // untouched while the link works
  CHECK(has(b.text, "with no valid application. Updating the application"));
  CHECK(!has(b.text, "incomplete"));
  run_more(START_END_MS, 200000);  // the fake MCU never answers the update: every repair fails
  CHECK(last_tool->state() == mcuflash::CHECK_ARMED);
  CHECK(last_tool->nvs().arm == CRC && last_tool->nvs().partial == 1);
  char text[512];
  last_tool->state_text(text, sizeof(text));
  CHECK(has(text, "Install armed") && has(text, "the automatic repair is used up"));
  CHECK(has(text, "Warning: the MCU firmware is incomplete"));
  next_outcome = OUT_OK;
  b = start(true);
  CHECK(b.attempted && b.state == mcuflash::CHECK_CURRENT);
}

// A boot status that arrives after the first request stops the requests and decides.
static void test_update_mode_late() {
  fresh(Mcu{Mcu::BOOTLOADER, 0, BOOT - 1, 0});
  const uint32_t first_request_ms = FIRST_LOOP_MS;
  boot_status_from_ms = first_request_ms + 100;
  const uint32_t first_status_ms =
      (boot_status_from_ms + rpstd01::BOOT_STATUS_PERIOD_MS - 1) / rpstd01::BOOT_STATUS_PERIOD_MS *
      rpstd01::BOOT_STATUS_PERIOD_MS;
  Boot b = start(false);
  CHECK(b.requests == 1 && b.verdict == VERDICT_UPDATE_MODE && b.state == mcuflash::CHECK_ARMED);
  CHECK(b.decided_ms >= first_status_ms && b.decided_ms < first_status_ms + 2 * LOOP_MS);
}

// A valid application in update mode is started first: its info frame decides, so a newer one is
// never written over.
static void test_update_mode_valid_application_started_first() {
  const struct {
    Mcu m;
    Verdict v;
    uint8_t state;
    UpdateState upd;
  } cases[] = {
      {Mcu{Mcu::BOOTLOADER, OLD_BUILD, BOOT}, VERDICT_APP_OLD, mcuflash::CHECK_CURRENT, UpdateState::REQUESTING},
      {Mcu{Mcu::BOOTLOADER, BUILD, BOOT}, VERDICT_CURRENT, mcuflash::CHECK_CURRENT, UpdateState::IDLE},
      {Mcu{Mcu::BOOTLOADER, OLD_BUILD, BOOT, 1, VERSION + 1}, VERDICT_NEWER, mcuflash::CHECK_CURRENT,
       UpdateState::IDLE},
      {Mcu{Mcu::BOOTLOADER, OLD_BUILD, BOOT - 1, 1, VERSION + 1}, VERDICT_NEWER, mcuflash::CHECK_CURRENT,
       UpdateState::IDLE},
      {Mcu{Mcu::BOOTLOADER, OLD_BUILD, BOOT - 1}, VERDICT_BOOT_OLD, mcuflash::CHECK_ARMED, UpdateState::IDLE},
  };
  for (const auto &c : cases) {
    fresh(c.m);
    Boot b = start(true);
    CHECK(!b.attempted && b.runs == 1);
    CHECK(b.verdict == c.v && b.state == c.state && last_tool->updater().update_state() == c.upd);
    CHECK((b.nvs.arm == CRC) == (c.state == mcuflash::CHECK_ARMED));
  }
}

// A valid application in update mode that does not start when asked: its version is unknown, so
// nothing is written or armed.
static void test_update_mode_valid_application_that_does_not_start_is_held() {
  Mcu m{Mcu::BOOTLOADER, 0, BOOT};
  m.runs_on_request = false;
  fresh(m);
  Boot b = start(true);
  CHECK(!b.attempted && b.runs == CHECK_RUN_TRIES);
  CHECK(b.verdict == VERDICT_UPDATE_MODE && b.state == mcuflash::CHECK_CURRENT && b.nvs.arm == 0);
  CHECK(last_tool->updater().update_state() == UpdateState::IDLE);
  CHECK(has(b.text, "with a valid application. It did not start when asked, so its version is unknown and nothing "
                    "is armed. Install at Next Power-On forces"));
  b = start(true);
  CHECK(!b.attempted && programs == 0);
}

// A valid application in update mode that starts when asked, but is a release whose frames this
// tool cannot read: it answers, so it is not taken for one that did not start, and nothing writes
// over it.
static void test_update_mode_later_application_started() {
  Mcu m{Mcu::BOOTLOADER, OLD_BUILD, BOOT};
  m.runs_as = Mcu::LATER_APP;
  fresh(m);
  Boot b = start(true);
  CHECK(!b.attempted && b.runs == 1);
  CHECK(b.verdict == VERDICT_UNKNOWN_APP && b.state == mcuflash::CHECK_CURRENT && b.nvs.arm == 0);
  CHECK(has(b.text, "does not understand") && !has(b.text, "did not start"));
  CHECK(last_tool->updater().update_state() == UpdateState::IDLE);
  b = start(true);
  CHECK(!b.attempted && programs == 0);
}

// A valid application in update mode that starts when asked, but whose info frame at its start is
// lost: it is asked for the info frame, not started again.
static void test_update_mode_application_started_info_lost() {
  Mcu m{Mcu::BOOTLOADER, OLD_BUILD, BOOT};
  m.info_at_start = false;
  fresh(m);
  Boot b = start(true);
  CHECK(!b.attempted && b.runs == 1 && b.requests >= 1);
  CHECK(b.verdict == VERDICT_APP_OLD && b.state == mcuflash::CHECK_CURRENT && b.nvs.arm == 0);
}

// Newer firmware on the MCU than the tool carries is never replaced by itself.
static void test_no_downgrade() {
  const Mcu newer[] = {
      Mcu{Mcu::APP, BUILD, BOOT + 1},      // newer bootloader, same application
      Mcu{Mcu::APP, OLD_BUILD, BOOT + 1},  // newer bootloader, another application
  };
  for (const Mcu &m : newer) {
    fresh(m);
    Boot b = start(true);
    CHECK(b.verdict == VERDICT_NEWER && b.state == mcuflash::CHECK_CURRENT && b.nvs.arm == 0);
    CHECK(has(b.text, "bootloader is newer than this tool's") && has(b.text, "Install at Next Power-On forces"));
    b = start(true);
    CHECK(!b.attempted && programs == 0);
  }
}

static void test_classify_by_release_and_protocol_with_carried_bootloader() {
  Seen s{};
  Bytes p = info_payload(OLD_BUILD);
  p[0] = (uint8_t) (VERSION + 1);  // a newer release
  CHECK(rpstd01::decode_info(p.data(), (uint8_t) p.size(), &s.app));
  s.info = true;
  CHECK(classify(s, IMG) == VERDICT_NEWER);
  p[0] = (uint8_t) (VERSION - 1);  // older release, other build: an update
  CHECK(rpstd01::decode_info(p.data(), (uint8_t) p.size(), &s.app));
  CHECK(classify(s, IMG) == VERDICT_APP_OLD);
  p[0] = 0x00;
  p[1] = (uint8_t) ((VERSION >> 8) + 1);  // a newer protocol
  CHECK(rpstd01::decode_info(p.data(), (uint8_t) p.size(), &s.app));
  CHECK(classify(s, IMG) == VERDICT_NEWER);
}

static void test_arm_button_forces_over_newer_firmware() {
  fresh(Mcu{Mcu::APP, BUILD, BOOT + 1});
  Boot b = start(false);
  CHECK(last_tool->command(mcuflash::COMMAND_ARM));
  CHECK(last_tool->state() == mcuflash::CHECK_CURRENT && last_tool->nvs().arm == CRC);
  char text[512];
  last_tool->state_text(text, sizeof(text));
  CHECK(has(text, "newer than this tool's") && has(text, "Install at Next Power-On is set"));
  b = start(true);
  CHECK(b.attempted && programs == 1 && b.verdict == VERDICT_CURRENT && b.nvs.arm == 0);
}

// While the image waits to be marked valid, the page asks to wait before a power cycle.
static void test_image_pending_text() {
  fresh(Mcu{});
  image_pending = true;
  Boot b = start(false);
  char wait[80];
  snprintf(wait, sizeof(wait), "Wait until this tool has run for %u s before pressing RESET. ",
           (unsigned) SAFE_MODE_GOOD_AFTER_S);
  CHECK(b.text.rfind(wait, 0) == 0);
  CHECK(has(b.text, "Install armed"));
  last_tool->set_image_pending(false);
  char text[512];
  last_tool->state_text(text, sizeof(text));
  CHECK(!has(text, "Wait until") && has(text, "Install armed"));
}

// A record of another size is no record: it neither arms nor holds anything by
// itself, and the page shows none.
static void test_old_record() {
  fresh(Mcu{Mcu::APP, BUILD, BOOT});
  put_old_record(160);
  Boot b = start(true);
  CHECK(!b.attempted && b.state == mcuflash::CHECK_CURRENT && b.nvs.arm == 0);
  char text[200];
  last_tool->last_install_text(text, sizeof(text));
  CHECK(std::string(text) == "None");
}

static void test_old_record_with_out_of_date_mcu_arms_as_without_record() {
  fresh(Mcu{Mcu::APP, BUILD, BOOT - 1});
  put_old_record(160);
  Boot b = start(false);
  CHECK(b.state == mcuflash::CHECK_ARMED);
  b = start(true);
  CHECK(b.attempted && b.state == mcuflash::CHECK_CURRENT);
}

// The wording for newer firmware names what is newer.
static void test_newer_text() {
  const struct {
    Mcu m;
    uint16_t version;
    const char *text;
  } cases[] = {
      {Mcu{Mcu::APP, BUILD, BOOT + 1}, VERSION, "The MCU's bootloader is newer than this tool's, so"},
      {Mcu{Mcu::APP, BUILD, BOOT + 1}, VERSION + 1, "The MCU's application and bootloader are newer"},
      {Mcu{Mcu::APP, BUILD, BOOT}, VERSION + 1, "The MCU's application is newer than this tool's, so"},
      {Mcu{Mcu::APP, BUILD, BOOT - 1}, VERSION + 1,
       "The MCU's application is newer than this tool's, its bootloader older"},
  };
  for (const auto &c : cases) {
    fresh(c.m);
    Tool t;
    NvsToolStore st(true);
    t.begin(&st, IMG, nullptr);
    FakeUart uart;
    Bytes p = info_payload(c.m.build, c.m.boot);
    p[0] = (uint8_t) c.version;
    p[1] = (uint8_t) (c.version >> 8);
    uart.push(frame(rpstd01::MSG_INFO, p));
    t.loop(uart, 0);
    t.loop(uart, 1000);
    char text[512];
    t.state_text(text, sizeof(text));
    CHECK(t.verdict() == VERDICT_NEWER && has(text, c.text));
    t.command(mcuflash::COMMAND_ARM);
    t.state_text(text, sizeof(text));
    CHECK(!has(text, "nothing is armed") && has(text, "Install at Next Power-On is set"));
  }
}

// The application this power-on installed answered, but the check hears nothing: a link fault,
// not a reason to install again.
static void test_link_fault_holds_but_counts_attempts() {
  fresh(Mcu{});
  start(false);
  link_dead = true;
  Boot b = start(true);
  CHECK(b.attempted && b.rec.app_seen && b.verdict == VERDICT_LINK_FAULT);
  CHECK(b.state == mcuflash::CHECK_CURRENT && b.nvs.arm == 0 && b.nvs.partial == 0 && b.nvs.fails == 1);
  CHECK(has(b.text, "Link fault: the MCU answered after the install, but not now"));
  b = start(false);  // a later start without the install's record: no answer, so it arms
  CHECK(b.verdict == VERDICT_NO_ANSWER && b.state == mcuflash::CHECK_ARMED && b.nvs.fails == 1);
  for (int i = 2; i <= 3; i++) {
    b = start(true);
    CHECK(b.attempted && b.verdict == VERDICT_LINK_FAULT && b.nvs.fails == i);
    b = start(true);
    CHECK(!b.attempted && b.verdict == VERDICT_NO_ANSWER);
  }
  CHECK(b.state == mcuflash::CHECK_GAVE_UP);
}

static void test_install_whose_app_never_answered_is_not_link_fault() {
  fresh(Mcu{});
  start(false);
  next_outcome = OUT_SILENT_APP;
  Boot b = start(true);
  CHECK(b.verdict == VERDICT_NO_ANSWER && b.state == mcuflash::CHECK_ARMED);
}

// Frames after the decision do not change what the page shows.
static void test_frozen_after_decision() {
  fresh(Mcu{});
  Boot b = start(false);
  CHECK(b.verdict == VERDICT_NO_ANSWER);
  FakeUart uart;
  uart.push(frame(rpstd01::MSG_INFO, info_payload(BUILD)));
  last_tool->loop(uart, 6000);
  char text[200];
  last_tool->installed_text(text, sizeof(text));
  CHECK(has(text, "No answer at start") && last_tool->verdict() == VERDICT_NO_ANSWER && uart.rx.empty());
}

// An MCU that answers, but with no info frame or boot status this tool reads (a later release with
// other frame lengths): nothing is armed, and an automatic arm left from an earlier check is
// cleared, so no downgrade happens by itself.
static void test_unknown_application() {
  for (Mcu::Kind kind : {Mcu::STATUSES_ONLY, Mcu::LATER_APP, Mcu::LATER_BOOTLOADER}) {
    fresh(Mcu{});
    Boot b = start(false);
    CHECK(b.verdict == VERDICT_NO_ANSWER && b.nvs.arm == CRC);
    mcu = Mcu{kind};
    b = start(false);
    CHECK(b.verdict == VERDICT_UNKNOWN_APP && b.state == mcuflash::CHECK_CURRENT && b.nvs.arm == 0);
    CHECK(b.requests == requests_without_answer());
    CHECK(has(b.text, "The MCU runs firmware this tool does not understand"));
    CHECK(has(b.text, "Nothing is armed. Install at Next Power-On forces an install"));
    char text[200];
    last_tool->installed_text(text, sizeof(text));
    CHECK(has(text, "no info frame or boot status this tool reads"));
    b = start(true);
    CHECK(!b.attempted && programs == 0);
    CHECK(last_tool->command(mcuflash::COMMAND_ARM));
    char armed[600];
    last_tool->state_text(armed, sizeof(armed));
    CHECK(has(armed, "reads. Install at Next Power-On is set") && !has(armed, "Nothing is armed"));
    b = start(true);
    CHECK(b.attempted && programs == 1 && b.verdict == VERDICT_CURRENT);
  }
}

// Update Application Now needs the carried bootloader: an older one is refused, and the page says
// that only an install through STC's bootloader replaces it.
static void test_update_now_old_bootloader() {
  const Mcu old[] = {Mcu{Mcu::APP, BUILD, BOOT - 1}, Mcu{Mcu::BOOTLOADER, 0, BOOT - 1, 0}};
  for (const Mcu &m : old) {
    fresh(m);
    Boot b = start(false);
    CHECK(b.state == mcuflash::CHECK_ARMED);
    CHECK(!last_tool->update_application());
    CHECK(last_tool->updater().update_state() == UpdateState::IDLE);
    char text[600];
    last_tool->state_text(text, sizeof(text));
    CHECK(has(text, "Update Application Now cannot write this MCU: the update over the link needs the carried "
                    "bootloader (v3), and the MCU has v2"));
    CHECK(last_tool->command(mcuflash::COMMAND_ARM));
    last_tool->state_text(text, sizeof(text));
    CHECK(!has(text, "cannot write this MCU"));
  }
}

// Nor over a newer one: the page then names only Install at Next Power-On as the way to force this
// tool's firmware.
static void test_update_now_newer_bootloader() {
  const Mcu newer[] = {
      Mcu{Mcu::APP, BUILD, BOOT + 1},
      Mcu{Mcu::APP, OLD_BUILD, BOOT + 1, 1, VERSION + 1},
      Mcu{Mcu::BOOTLOADER, 0, BOOT + 1, 0},
  };
  for (const Mcu &m : newer) {
    fresh(m);
    Boot b = start(true);
    CHECK(b.verdict == VERDICT_NEWER && b.nvs.arm == 0);
    CHECK(!has(b.text, "Update Application Now forces") && has(b.text, "Install at Next Power-On forces"));
    CHECK(!last_tool->update_application());
    CHECK(last_tool->updater().update_state() == UpdateState::IDLE);
    char text[600];
    last_tool->state_text(text, sizeof(text));
    CHECK(has(text, "Update Application Now cannot write this MCU: the update over the link needs the carried "
                    "bootloader (v3), and the MCU has v4"));
  }
}

// A newer application with the carried bootloader: Update Application Now forces this tool's.
static void test_update_now_over_newer_application_with_carried_bootloader() {
  fresh(Mcu{Mcu::APP, OLD_BUILD, BOOT, 1, VERSION + 1});
  Boot b = start(true);
  CHECK(b.verdict == VERDICT_NEWER && b.nvs.arm == 0 && has(b.text, "Update Application Now forces"));
  CHECK(last_tool->update_application());
  CHECK(last_tool->updater().update_state() == UpdateState::REQUESTING);
}

static void test_update_now_with_carried_bootloader_allowed() {
  fresh(Mcu{Mcu::APP, BUILD, BOOT});
  start(false);
  CHECK(last_tool->update_application());
  CHECK(last_tool->updater().update_state() == UpdateState::REQUESTING);
}

// A link update that fails with an automatic arm still in NVS (left as it was while the link
// worked): the page says the arm is from an earlier check, not that an NVS write failed.
static void test_kept_arm_text() {
  fresh(Mcu{});
  Boot b = start(false);
  CHECK(b.nvs.arm == CRC && b.nvs.user == mcuflash::USER_NONE);
  mcu = Mcu{Mcu::APP, OLD_BUILD, BOOT};
  b = start(false);
  CHECK(b.verdict == VERDICT_APP_OLD && b.state == mcuflash::CHECK_CURRENT && b.nvs.arm == CRC);
  run_more(START_END_MS, 8000);
  CHECK(last_tool->updater().update_state() == UpdateState::FAILED);
  char text[600];
  last_tool->state_text(text, sizeof(text));
  CHECK(has(text, "An install armed by an earlier check is still set"));
  CHECK(!has(text, "NVS write failed"));
}

// A crash between the sets of one save leaves what was set so far, as in ESP-IDF.
static void test_crash_mid_save() {
  fresh(Mcu{});
  Boot b = start(false);
  CHECK(b.state == mcuflash::CHECK_ARMED);
  // Cancel Install writes the user value first, then the arm flag. A crash in between...
  fake_nvs::state().writes_left = 1;
  CHECK(!last_tool->command(mcuflash::COMMAND_DISARM));
  fake_nvs::state().writes_left = -1;
  // ...still holds the install at the next power-on, and the check clears the flag.
  b = start(true);
  CHECK(!b.attempted && b.state == mcuflash::CHECK_DISARMED && b.nvs.arm == 0);
}

int main() {
  RUN(test_classify);
  RUN(test_holds_auto_install);
  RUN(test_link_update_handles_app_and_invalid_app_at_carried_bootloader);
  RUN(test_app_area);
  RUN(test_nvs_store_survives_restart);
  RUN(test_nvs_store_ignores_same_keys_in_other_namespace);
  RUN(test_nvs_record_of_other_size_or_version_is_no_record);
  RUN(test_record_round_trip);
  RUN(test_nvs_that_cannot_open_reads_empty_and_fails_to_arm);
  RUN(test_nvs_failed_commit_reported_by_save);
  RUN(test_first_install);
  RUN(test_out_of_date_arms);
  RUN(test_link_path);
  RUN(test_link_update_never_answered_fails_then_arm_by_hand);
  RUN(test_buttons);
  RUN(test_install_by_hand_while_current_written_at_next_power_on);
  RUN(test_gives_up);
  RUN(test_refusal);
  RUN(test_incomplete);
  RUN(test_real_programmer);
  RUN(test_texts);
  RUN(test_state_text_cut_to_small_buffer);
  RUN(test_installed_text_no_answer);
  RUN(test_link_start_keeps_cancel_install);
  RUN(test_link_start_keeps_install_armed_by_hand);
  RUN(test_link_repair_fails);
  RUN(test_update_mode_late);
  RUN(test_update_mode_valid_application_started_first);
  RUN(test_update_mode_valid_application_that_does_not_start_is_held);
  RUN(test_update_mode_later_application_started);
  RUN(test_update_mode_application_started_info_lost);
  RUN(test_no_downgrade);
  RUN(test_classify_by_release_and_protocol_with_carried_bootloader);
  RUN(test_arm_button_forces_over_newer_firmware);
  RUN(test_image_pending_text);
  RUN(test_old_record);
  RUN(test_old_record_with_out_of_date_mcu_arms_as_without_record);
  RUN(test_newer_text);
  RUN(test_link_fault_holds_but_counts_attempts);
  RUN(test_install_whose_app_never_answered_is_not_link_fault);
  RUN(test_frozen_after_decision);
  RUN(test_unknown_application);
  RUN(test_update_now_old_bootloader);
  RUN(test_update_now_newer_bootloader);
  RUN(test_update_now_over_newer_application_with_carried_bootloader);
  RUN(test_update_now_with_carried_bootloader_allowed);
  RUN(test_kept_arm_text);
  RUN(test_crash_mid_save);
  delete last_tool;
  delete tool_store;
  TEST_EXIT();
}
