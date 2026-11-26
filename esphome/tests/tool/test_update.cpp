// The MCU tool's application update through our bootloader, on the simulated MCU.

#define RPSTD01TOOL_NVS_FAKE
#include "esphome/components/rpstd01_tool/rpstd01_tool_nvs.h"
#include "../sim_wrapper.h"

using rpstd01tool::AppUpdater;
using rpstd01tool::UpdateState;
using rpstd01tool::update_detail::MAX_RESTARTS;
using rpstd01tool::update_detail::REQUEST_RESEND_MS;
using rpstd01tool::update_detail::REQUEST_TIMEOUT_MS;
using rpstd01tool::update_detail::RETRY_MS;

static constexpr int BEGINS_UNTIL_GIVING_UP = 1 + MAX_RESTARTS;
static constexpr size_t REQUESTS_UNTIL_TIMEOUT = REQUEST_TIMEOUT_MS / REQUEST_RESEND_MS;

static bool about_requests_until_timeout(size_t requests) {
  return requests + 1 >= REQUESTS_UNTIL_TIMEOUT && requests <= REQUESTS_UNTIL_TIMEOUT + 1;
}

// The application area as the tool carries it, valid mark set. The AppUpdater sends the mark as 0xFF.
static const uint8_t *const carried_area = MCU_IMAGE + APP_BASE;

struct UpdSim : SimMcu {
  AppUpdater link;
  UpdSim() {
    this->link.set_image(carried_area, image_build_of(app_image.data()), BOOT_VERSION);
    this->esp_loop = [this](uint32_t ms) { this->link.loop(this->uart, ms); };
  }
  bool run_until(UpdateState st, uint32_t max_ms) {
    for (uint32_t t = 0; t < max_ms; t += 10) {
      this->run(10);
      if (this->link.update_state() == st)
        return true;
    }
    return false;
  }
  std::string text() {
    char buf[96];
    this->link.update_text(buf, sizeof buf);
    return buf;
  }
};

static rpstd01tool::Carried carried() {
  return rpstd01tool::Carried{MCU_IMAGE,         sizeof(MCU_IMAGE), MCU_IMAGE_CRC32,
                              MCU_IMAGE_VERSION, MCU_IMAGE_BUILD,   MCU_IMAGE_BOOT_VERSION};
}

static void invalidate_app() { memset(flash + APP_END - SECTOR_SIZE, 0xFF, SECTOR_SIZE); }

static UpdSim *new_world(uint32_t installed_build, bool valid = true) {
  install(installed_build);
  if (!valid)
    invalidate_app();
  UpdSim *w = new UpdSim();
  w->us = 1000000;  // so that times never wrap below zero
  w->power_on();
  return w;
}

// The lamp as the main configuration left it: firing.
static void lamp_on(SimMcu *w) {
  gate_set_target(&w->gate, 3000, 48, 48);
  w->run(500);
}

static void test_carried_image() {
  rpstd01tool::Carried c = carried();
  CHECK(rpstd01tool::app_area(c) == carried_area);
  c.size = APP_END - 1;
  CHECK(rpstd01tool::app_area(c) == nullptr);
}

static void test_update_from_application() {
  UpdSim *w = new_world(0x12345678);
  lamp_on(w);
  CHECK_EQ(w->gate.state, ST_FIRING);
  w->run(500);
  CHECK(w->link.have_info());
  CHECK(w->esp_frames[MSG_INFO_REQ] == 0 && w->esp_frames[MSG_UPDATE] == 0);  // nothing sent while idle
  w->link.start_update();
  uint64_t started = w->us;
  CHECK(w->link.update_state() == UpdateState::REQUESTING);
  CHECK(w->text() == "Requesting");
  w->run(50);
  CHECK_EQ(w->gate.target, 0);  // the MCU turns the lamp off for the request by itself
  CHECK(w->run_until(UpdateState::WRITING, 3000));
  CHECK(w->text().rfind("Writing", 0) == 0);
  CHECK(w->run_until(UpdateState::DONE, 15000));
  CHECK(w->text() == "Done");
  printf("    update took %.1f s (simulated)\n", (w->us - started) / 1e6);
  CHECK_NEAR(w->link.update_progress_percent(), 100.0, 1e-6);
  CHECK(flash_matches_image());
  CHECK_EQ(w->begins_seen, 1);
  CHECK_EQ(w->writes_seen, APP_SIZE / WRITE_CHUNK);
  CHECK_EQ(w->update_reqs, 1);
  CHECK_EQ(w->boot_bytes_dropped, 0);  // never sent while the bootloader was busy
  CHECK_EQ(w->boot_bad_frames, 0);     // never sent application frames to the bootloader
  w->run(500);
  CHECK_EQ((int) w->link.link_state(), (int) LinkState::APPLICATION);
  w->link.start_update();
  CHECK(w->link.update_state() == UpdateState::REQUESTING);
  CHECK(w->run_until(UpdateState::DONE, 15000));
  delete w;
}

static void test_update_lost_reply_and_frames() {
  UpdSim *w = new_world(0x12345678);
  w->run(300);
  w->drop_boot_reply_after_write = 10 * WRITE_CHUNK;
  w->corrupt_write_at = 20 * WRITE_CHUNK;
  w->link.start_update();
  w->run(2000);
  w->drop_esp_type = MSG_BOOT_WRITE;  // one write lost on the way
  w->drop_esp_count = 1;
  CHECK(w->run_until(UpdateState::DONE, 20000));
  CHECK(flash_matches_image());
  CHECK_EQ(w->begins_seen, 1);
  // The lost reply needs no resend (the next boot status shows the progress); the corrupted and
  // the dropped write are each sent once more, after the retry time.
  CHECK_EQ(w->esp_frames[MSG_BOOT_WRITE], APP_SIZE / WRITE_CHUNK + 2);
  CHECK_EQ(w->boot_bad_frames, 0);
  delete w;
}

static void test_update_lost_command_sent_again() {
  for (uint8_t type : {MSG_BOOT_BEGIN, MSG_BOOT_FINISH, MSG_BOOT_RUN}) {
    UpdSim *w = new_world(0x12345678);
    w->run(300);
    w->drop_esp_type = type;
    w->drop_esp_count = 1;
    w->link.start_update();
    CHECK(w->run_until(UpdateState::DONE, 20000));
    CHECK(flash_matches_image());
    CHECK_EQ(w->begins_seen, 1);
    CHECK_EQ(w->esp_frames[type], 2);
    CHECK_EQ(w->boot_bytes_dropped, 0);
    delete w;
  }
}

static void test_update_every_write_lost_fails_then_new_start_works() {
  UpdSim *w = new_world(0x12345678);
  w->run(300);
  w->link.start_update();
  CHECK(w->run_until(UpdateState::WRITING, 3000));
  w->run(200);
  w->drop_esp_type = MSG_BOOT_WRITE;
  w->drop_esp_count = 1000;
  CHECK(w->run_until(UpdateState::FAILED, 30000));
  CHECK(w->text() == "Failed: no reply to write");
  CHECK_EQ(w->begins_seen, BEGINS_UNTIL_GIVING_UP);
  w->drop_esp_count = 0;
  w->run(2000);
  CHECK(w->link.update_state() == UpdateState::FAILED);  // the updater alone does not start again
  w->link.start_update();
  CHECK(w->link.update_state() == UpdateState::BEGIN);
  CHECK(w->run_until(UpdateState::DONE, 15000));
  CHECK(flash_matches_image());
  delete w;
}

static void test_update_tampered_write_restarts() {
  UpdSim *w = new_world(0x12345678);
  w->run(300);
  w->tamper_write_at = 5 * WRITE_CHUNK;
  w->tamper_times = 1;
  w->link.start_update();
  CHECK(w->run_until(UpdateState::DONE, 25000));
  CHECK_EQ(w->begins_seen, 2);
  CHECK(flash_matches_image());
  delete w;
}

static void test_update_tampered_every_time_fails_in_bootloader() {
  UpdSim *w = new_world(0x12345678);
  w->run(300);
  w->tamper_write_at = 5 * WRITE_CHUNK;
  w->tamper_times = 10;
  w->link.start_update();
  CHECK(w->run_until(UpdateState::FAILED, 40000));
  CHECK_EQ(w->begins_seen, BEGINS_UNTIL_GIVING_UP);
  CHECK(w->text() == "Failed: image check failed");
  CHECK(strcmp(w->link.fail_reason(), "image check failed") == 0);
  CHECK(w->mode == SimMcu::BOOT);
  CHECK_EQ((int) w->link.link_state(), (int) LinkState::BOOTLOADER);
  delete w;
}

static void test_update_long_power_cut_fails() {
  UpdSim *w = new_world(0x12345678);
  w->run(300);
  w->cut_power_at_write = 30 * WRITE_CHUNK;
  w->cut_ms = 4000;
  w->link.start_update();
  CHECK(w->run_until(UpdateState::FAILED, 10000));
  CHECK(w->text() == "Failed: MCU stopped answering");
  CHECK(w->mode == SimMcu::OFF);
  delete w;
}

static void test_update_short_power_cut_starts_again_from_begin() {
  UpdSim *w = new_world(0x12345678);
  w->run(300);
  w->cut_power_at_write = 40 * WRITE_CHUNK;
  w->cut_ms = 300;
  w->link.start_update();
  CHECK(w->run_until(UpdateState::DONE, 20000));
  CHECK_EQ(w->begins_seen, 2);
  CHECK(flash_matches_image());
  delete w;
}

static void test_update_from_bootloader_begins_at_once() {
  UpdSim *w = new_world(0x12345678, false);
  CHECK(w->mode == SimMcu::BOOT);
  w->run(1000);
  CHECK(w->link.update_state() == UpdateState::IDLE);
  CHECK(w->esp_frames[MSG_BOOT_BEGIN] == 0);  // nothing sent while idle
  w->link.start_update();
  CHECK(w->link.update_state() == UpdateState::BEGIN);
  CHECK(w->run_until(UpdateState::DONE, 15000));
  CHECK(flash_matches_image());
  w->run(200);
  CHECK_EQ((int) w->link.link_state(), (int) LinkState::APPLICATION);
  delete w;
}

static std::vector<uint8_t> status_frame() {
  uint8_t p[STATUS_LEN] = {0};
  return fw_frame(MSG_STATUS, p, STATUS_LEN);
}

static void run_updater(AppUpdater &l, FakeUart &u, uint32_t *t, uint32_t ms, bool with_status) {
  for (uint32_t end = *t + ms; *t < end; *t += 10) {
    if (with_status)
      u.push(status_frame());
    l.loop(u, *t);
  }
}

static void test_request_resent_until_timeout() {
  static uint8_t image[APP_SIZE];
  memset(image, 0xFF, sizeof image);
  AppUpdater l;
  FakeUart u;
  uint32_t t = 0;
  l.set_image(image, 2, BOOT_VERSION);
  run_updater(l, u, &t, 100, true);
  CHECK(u.take_frames().empty());
  l.start_update();
  run_updater(l, u, &t, REQUEST_TIMEOUT_MS - 100, true);
  CHECK(l.update_state() == UpdateState::REQUESTING);
  auto req = of_type(u.take_frames(), MSG_UPDATE);
  CHECK(about_requests_until_timeout(req.size()));
  CHECK(req.size() >= 1 && req[0].payload.size() == UPDATE_LEN && req[0].payload[0] == UPDATE_KEY0);
  run_updater(l, u, &t, 200, true);
  CHECK(l.update_state() == UpdateState::FAILED);
  CHECK(strcmp(l.fail_reason(), "MCU did not enter update mode") == 0);
}

static void test_request_without_any_status_times_out_sending_nothing() {
  static uint8_t image[APP_SIZE];
  memset(image, 0xFF, sizeof image);
  AppUpdater c;
  FakeUart v;
  uint32_t t = 0;
  c.set_image(image, 2, BOOT_VERSION);
  c.start_update();
  run_updater(c, v, &t, REQUEST_TIMEOUT_MS - 100, false);
  CHECK(c.update_state() == UpdateState::REQUESTING);
  run_updater(c, v, &t, 200, false);
  CHECK(c.update_state() == UpdateState::FAILED);
  CHECK(v.take_frames().empty());
}

// Retries go out on their own timer, never sooner, whatever the bootloader's statuses show.
static void test_retry_timer() {
  static uint8_t image[APP_SIZE];
  AppUpdater l;
  FakeUart u;
  l.set_image(image, 2, BOOT_VERSION);
  uint32_t t = 0;
  auto boot_status = [&](uint8_t result) {
    const uint8_t bs[BOOT_STATUS_LEN] = {BOOT_VERSION, 0, BOOT_REASON_INVALID, result, 0, 0};
    u.push(fw_frame(MSG_BOOT_STATUS, bs, BOOT_STATUS_LEN));
  };
  boot_status(BR_NO_COMMAND);
  l.loop(u, t);
  l.start_update();
  CHECK(l.update_state() == UpdateState::BEGIN);
  l.loop(u, t);
  CHECK_EQ(of_type(u.take_frames(), MSG_BOOT_BEGIN).size(), 1);
  // Statuses that still say "no command yet" are not a reply.
  for (t = 16; t < RETRY_MS; t += 16) {
    if (t % 496 == 0)
      boot_status(BR_NO_COMMAND);
    l.loop(u, t);
  }
  CHECK(u.take_frames().empty());
  for (; t < RETRY_MS + 100; t += 16)
    l.loop(u, t);
  CHECK_EQ(of_type(u.take_frames(), MSG_BOOT_BEGIN).size(), 1);
  CHECK(l.update_state() == UpdateState::BEGIN);
  boot_status(BR_BAD_STATE);
  l.loop(u, t);
  CHECK_EQ(of_type(u.take_frames(), MSG_BOOT_BEGIN).size(), 1);
  for (uint32_t end = t + detail::BOOT_SEEN_MS + 100; t < end; t += 16)
    l.loop(u, t);
  CHECK(l.update_state() == UpdateState::FAILED);
  CHECK(strcmp(l.fail_reason(), "MCU stopped answering") == 0);
}

// A boot status with an error result that crossed Begin on the wire (sent before the bootloader had
// Begin) is not Begin's reply: it costs no return to Begin. A later error still is a reply.
static void test_crossing_error_status() {
  static uint8_t image[APP_SIZE];
  AppUpdater l;
  FakeUart u;
  l.set_image(image, 2, BOOT_VERSION);
  auto boot_status = [&](uint8_t result) {
    const uint8_t bs[BOOT_STATUS_LEN] = {BOOT_VERSION, 0, BOOT_REASON_REQUEST, result, 0, 0};
    u.push(fw_frame(MSG_BOOT_STATUS, bs, BOOT_STATUS_LEN));
  };
  uint32_t t = 0;
  boot_status(BR_BAD_IMAGE);
  l.loop(u, t);
  l.start_update();
  CHECK(l.update_state() == UpdateState::BEGIN);
  for (int crossing = 0; crossing <= rpstd01tool::update_detail::MAX_RESTARTS; crossing++) {
    // Each Begin crosses a status that repeats the old error, read in the next loop or the one after.
    l.loop(u, t += 16);
    CHECK_EQ(of_type(u.take_frames(), MSG_BOOT_BEGIN).size(), 1);
    boot_status(BR_BAD_IMAGE);
    l.loop(u, t += 1);
    boot_status(BR_BAD_IMAGE);
    l.loop(u, t += 16);
    CHECK(l.update_state() == UpdateState::BEGIN);
    CHECK(u.take_frames().empty());
    t += RETRY_MS;
  }
  // Begin's real reply.
  boot_status(BR_OK);
  l.loop(u, t += 16);
  CHECK(l.update_state() == UpdateState::WRITING);
  CHECK_EQ(of_type(u.take_frames(), MSG_BOOT_WRITE).size(), 1);
  // An error read later than that is the write's reply: back to Begin.
  l.loop(u, t += 16);
  l.loop(u, t += 16);
  boot_status(BR_BAD_OFFSET);
  l.loop(u, t += 16);
  CHECK(l.update_state() == UpdateState::BEGIN);
  CHECK_EQ(of_type(u.take_frames(), MSG_BOOT_BEGIN).size(), 1);
}

// A bootloader other than the one the application is for, seen before or during an update: the
// update stops before anything is erased or written.
static void test_other_bootloader_never_written() {
  static uint8_t image[APP_SIZE];
  auto boot_status = [](FakeUart &u, uint8_t version) {
    const uint8_t bs[BOOT_STATUS_LEN] = {version, 1, BOOT_REASON_REQUEST, BR_NO_COMMAND, 0, 0};
    u.push(fw_frame(MSG_BOOT_STATUS, bs, BOOT_STATUS_LEN));
  };
  for (uint8_t other : {static_cast<uint8_t>(BOOT_VERSION - 1), static_cast<uint8_t>(BOOT_VERSION + 1)}) {
    AppUpdater idle;
    FakeUart u;
    idle.set_image(image, 2, BOOT_VERSION);
    boot_status(u, other);
    idle.loop(u, 0);
    idle.start_update();
    idle.loop(u, 16);
    CHECK(idle.update_state() == UpdateState::FAILED);
    CHECK(strcmp(idle.fail_reason(), rpstd01tool::update_detail::OTHER_BOOTLOADER) == 0);
    CHECK(u.take_frames().empty());

    AppUpdater requesting;
    FakeUart v;
    uint32_t t = 0;
    requesting.set_image(image, 2, BOOT_VERSION);
    run_updater(requesting, v, &t, 100, true);
    requesting.start_update();
    run_updater(requesting, v, &t, 100, true);
    CHECK(of_type(v.take_frames(), MSG_UPDATE).size() == 1);
    boot_status(v, other);
    run_updater(requesting, v, &t, RETRY_MS, false);
    CHECK(requesting.update_state() == UpdateState::FAILED);
    CHECK(v.take_frames().empty());
  }
}

// A boot status that still shows the last Begin's result (erased, nothing written) and crossed a new
// Begin on the wire is not its reply: no write goes out while the bootloader erases.
static void test_crossing_ok_status_to_begin() {
  static uint8_t image[APP_SIZE];
  AppUpdater l;
  FakeUart u;
  l.set_image(image, 2, BOOT_VERSION);
  auto erased = [&]() {
    const uint8_t bs[BOOT_STATUS_LEN] = {BOOT_VERSION, 0, BOOT_REASON_REQUEST, BR_OK, 0, 0};
    u.push(fw_frame(MSG_BOOT_STATUS, bs, BOOT_STATUS_LEN));
  };
  uint32_t t = 0;
  erased();
  l.loop(u, t);
  l.start_update();
  CHECK(l.update_state() == UpdateState::BEGIN);
  l.loop(u, t += 16);
  CHECK_EQ(of_type(u.take_frames(), MSG_BOOT_BEGIN).size(), 1);
  erased();
  l.loop(u, t += 1);
  erased();
  l.loop(u, t += 16);
  CHECK(l.update_state() == UpdateState::BEGIN);
  CHECK(u.take_frames().empty());
  for (uint32_t erase_end = t + 400; t < erase_end;)
    l.loop(u, t += 16);
  CHECK(u.take_frames().empty());
  erased();
  l.loop(u, t += 16);
  CHECK(l.update_state() == UpdateState::WRITING);
  CHECK_EQ(of_type(u.take_frames(), MSG_BOOT_WRITE).size(), 1);
}

// A frame that arrives while the bootloader sends its status is received.
static void test_boot_receives_while_sending() {
  install(0x12345678);
  invalidate_app();
  SimMcu *w = new SimMcu();
  w->us = 1000000;
  w->power_on();
  CHECK(w->mode == SimMcu::BOOT && !w->to_esp_at.empty());  // its first status is on the wire
  uint8_t size[BOOT_BEGIN_LEN];
  detail::put16(size, APP_SIZE);
  auto begin = fw_frame(MSG_BOOT_BEGIN, size, BOOT_BEGIN_LEN);
  w->esp_wrote(begin.data(), begin.size());
  w->run(1000);
  CHECK_EQ(w->begins_seen, 1);
  CHECK_EQ(w->upd.result, BR_OK);
  CHECK_EQ(w->boot_bytes_dropped, 0);
  delete w;
}

static void test_update_aux_ram_lost() {
  UpdSim *w = new_world(0x12345678);
  w->aux_ram_survives_reset = false;
  w->run(300);
  w->link.start_update();
  CHECK(w->run_until(UpdateState::FAILED, 6000));
  CHECK(w->text() == "Failed: MCU restarted without entering update mode");
  CHECK(w->mode == SimMcu::APP);
  delete w;
}

static void test_update_request_never_received_times_out() {
  UpdSim *w = new_world(0x12345678);
  w->run(300);
  w->drop_esp_type = MSG_UPDATE;
  w->drop_esp_count = 100;
  w->link.start_update();
  CHECK(w->run_until(UpdateState::FAILED, REQUEST_TIMEOUT_MS + 1000));
  CHECK(w->text() == "Failed: MCU did not enter update mode");
  CHECK(about_requests_until_timeout(w->esp_frames[MSG_UPDATE]));
  delete w;
}

// A fresh updater, as after an ESP32 restart, while the bootloader waits in update mode after the
// request: an update started then goes ahead from Begin, and Run goes out only at its end.
static void test_update_esp_restart_in_update_mode() {
  UpdSim *w = new_world(0x12345678);
  w->run(300);
  w->link.start_update();
  for (int i = 0; i < 300 && w->mode != SimMcu::BOOT; i++)
    w->run(10);
  CHECK_EQ(w->mode, SimMcu::BOOT);
  w->link = AppUpdater();
  w->link.set_image(carried_area, image_build_of(app_image.data()), BOOT_VERSION);
  w->run(1000);
  CHECK_EQ((int) w->link.link_state(), (int) LinkState::BOOTLOADER);
  w->link.start_update();
  CHECK(w->link.update_state() == UpdateState::BEGIN);
  CHECK(w->run_until(UpdateState::DONE, 15000));
  CHECK(flash_matches_image());
  CHECK_EQ(w->begins_seen, 1);
  CHECK_EQ(w->esp_frames[MSG_BOOT_RUN], 1);
  delete w;
}

// While the MCU still shows its start-up FAULT it jumps to the bootloader at once.
static void test_update_just_after_mcu_start() {
  UpdSim *w = new_world(0x12345678);
  w->run(200);
  lamp_on(w);
  CHECK_EQ(w->gate.state, ST_FIRING);
  w->watchdog_reset();
  w->run(20);
  w->link.start_update();
  CHECK(w->link.update_state() == UpdateState::REQUESTING);
  CHECK(w->run_until(UpdateState::DONE, 20000));
  CHECK(flash_matches_image());
  CHECK_EQ(w->update_reqs, 1);
  delete w;
}

// After an update the new start is counted once, the info frame every second not at all.
static void test_update_restarts() {
  UpdSim *w = new_world(0x12345678);
  w->run(500);
  uint32_t restarts = w->link.mcu_restarts();
  w->link.start_update();
  CHECK(w->run_until(UpdateState::DONE, 15000));
  CHECK(flash_matches_image());
  CHECK_EQ(w->link.info().version, MCU_IMAGE_VERSION);
  w->run(3500);
  CHECK_EQ(w->link.mcu_restarts(), restarts + 1);
  w->watchdog_reset();
  w->run(300);
  CHECK_EQ(w->link.mcu_restarts(), restarts + 2);
  delete w;
}

static void test_update_info_frames_while_requesting_do_not_fail_it() {
  UpdSim *w = new_world(0x12345678);
  // The first three requests are lost, so the application keeps running, and sending its info frame,
  // for 1.5 s after the first.
  w->run(300);
  w->drop_esp_type = MSG_UPDATE;
  w->drop_esp_count = 3;
  int infos = 0;
  w->esp_loop = [w, &infos](uint32_t ms) {
    w->link.receive(w->uart, ms, [&infos](const Received &rx) { infos += rx.type == MSG_INFO && rx.decoded; });
    w->link.service(w->uart);
  };
  w->link.start_update();
  for (int i = 0; i < 300 && w->link.update_state() == UpdateState::REQUESTING; i++)
    w->run(10);
  CHECK(infos >= 1);
  CHECK(w->link.update_state() != UpdateState::FAILED && w->link.update_state() != UpdateState::REQUESTING);
  CHECK(w->run_until(UpdateState::DONE, 15000));
  CHECK_EQ(w->esp_frames[MSG_UPDATE], 4);
  delete w;
}

static void test_update_aux_ram_lost_twice() {
  // Auxiliary RAM lost at a reset, but coming back in the same state each time: the second such
  // start has the same boot ID as the first. Its statuses show "starting" again, so the restart
  // is still seen, and both requests fail the same way.
  UpdSim *w = new_world(0x12345678);
  w->aux_ram_survives_reset = false;
  w->run(300);
  uint32_t restarts = w->link.mcu_restarts();
  for (int k = 0; k < 2; k++) {
    w->link.start_update();
    CHECK(w->run_until(UpdateState::FAILED, 6000));
    CHECK(w->text() == "Failed: MCU restarted without entering update mode");
    CHECK_EQ(w->boot_id, 0xEF);
    w->run(500);
  }
  CHECK_EQ(w->link.mcu_restarts(), restarts + 2);
  CHECK_EQ(w->update_reqs, 2);
  delete w;
}

using rpstd01tool::Tool;
using rpstd01tool::Verdict;

struct ToolSim : SimMcu {
  rpstd01tool::NvsToolStore *store;
  Tool tool;
  ToolSim() {
    fake_nvs::reset();
    this->store = new rpstd01tool::NvsToolStore(true);
    this->tool.begin(this->store, carried(), nullptr);
    this->esp_loop = [this](uint32_t ms) { this->tool.loop(this->uart, ms); };
  }
  ~ToolSim() { delete this->store; }
  std::string text() {
    char buf[512];
    this->tool.state_text(buf, sizeof buf);
    return buf;
  }
  bool run_until(UpdateState st, uint32_t max_ms) {
    for (uint32_t t = 0; t < max_ms; t += 10) {
      this->run(10);
      if (this->tool.updater().update_state() == st)
        return true;
    }
    return false;
  }
};

static bool has(const std::string &text, const char *part) { return text.find(part) != std::string::npos; }

static const char UPDATED[] = "Application updated. Everything is current: install the main configuration.";

// The ESP32 starts the tool with the MCU as set up by the caller. Runs until the check decided.
static ToolSim *start_tool(uint32_t installed_build, bool valid = true, int boot_reason = 0,
                           uint16_t version = MCU_IMAGE_VERSION) {
  install(installed_build, version);
  if (!valid)
    invalidate_app();
  ToolSim *w = new ToolSim();
  w->us = 1000000;
  w->power_on();
  if (boot_reason) {
    // In update mode before the tool starts: the application's start frames are long gone.
    w->to_esp_at.clear();
    w->boot_start((uint8_t) boot_reason);
  }
  for (int i = 0; i < 600 && !w->tool.decided(); i++)
    w->run(10);
  CHECK(w->tool.decided());
  return w;
}

static void test_tool_updates_old_application_over_link_at_once() {
  ToolSim *w = start_tool(0x12345678);
  CHECK(w->tool.verdict() == rpstd01tool::VERDICT_APP_OLD);
  CHECK(w->tool.state() == mcuflash::CHECK_CURRENT && w->tool.nvs().arm == 0);  // no STC install
  CHECK(w->tool.updater().update_state() == UpdateState::REQUESTING);
  uint64_t t0 = w->us;
  w->run(1500);
  CHECK(has(w->text(), "The MCU's application is another build (its bootloader is current). Updating the "
                       "application through our bootloader (no power cycle needed): "));
  CHECK(w->run_until(UpdateState::DONE, 10000));
  printf("    decided at %.1f s, update took %.1f s (simulated)\n", (t0 - 1000000) / 1e6, (w->us - t0) / 1e6);
  w->run(100);
  CHECK(flash_matches_image());
  CHECK_EQ(w->begins_seen, 1);
  CHECK_EQ(w->update_reqs, 1);
  CHECK(w->tool.verdict() == rpstd01tool::VERDICT_CURRENT);
  CHECK(w->text() == UPDATED);
  char text[200];
  w->tool.installed_text(text, sizeof text);
  CHECK(has(text, ("bootloader v" + std::to_string(BOOT_VERSION)).c_str()));
  delete w;
}

// Our bootloader in update mode with no valid application: the carried application is written at
// once.
static void test_tool_update_mode_invalid_application() {
  ToolSim *w = start_tool(0x12345678, false, BOOT_REASON_INVALID);
  CHECK(w->tool.verdict() == rpstd01tool::VERDICT_UPDATE_MODE);
  CHECK(w->tool.state() == mcuflash::CHECK_CURRENT && w->tool.nvs().arm == 0);
  CHECK(w->tool.updater().update_state() == UpdateState::BEGIN);
  CHECK(has(w->text(), "with no valid application. Updating the application"));
  CHECK(w->run_until(UpdateState::DONE, 10000));
  w->run(500);  // the reply to Run is the update's, not a new crash
  CHECK(flash_matches_image());
  CHECK_EQ(w->begins_seen, 1);
  CHECK_EQ(w->esp_frames[MSG_BOOT_RUN], 1);
  CHECK_EQ(w->update_reqs, 0);
  CHECK(w->tool.verdict() == rpstd01tool::VERDICT_CURRENT && w->tool.state() == mcuflash::CHECK_CURRENT);
  CHECK(w->text() == UPDATED);
  delete w;
}

// Our bootloader in update mode with a valid application, whatever the reason: the application is
// started, and its info frame decides, so a newer one is never written over.
static void test_tool_update_mode_valid_application_started_first() {
  const uint32_t carried_build = image_build_of(app_image.data());
  const struct {
    uint32_t build;
    uint16_t version;
    int reason;
    Verdict verdict;
  } cases[] = {
      {carried_build, MCU_IMAGE_VERSION, BOOT_REASON_WATCHDOG, rpstd01tool::VERDICT_CURRENT},
      {carried_build, MCU_IMAGE_VERSION, BOOT_REASON_REQUEST, rpstd01tool::VERDICT_CURRENT},
      {0x12345678, MCU_IMAGE_VERSION + 1, BOOT_REASON_REQUEST, rpstd01tool::VERDICT_NEWER},
      {0x12345678, MCU_IMAGE_VERSION + 1, BOOT_REASON_WATCHDOG, rpstd01tool::VERDICT_NEWER},
      {0x12345678, MCU_IMAGE_VERSION, BOOT_REASON_REQUEST, rpstd01tool::VERDICT_APP_OLD},
  };
  for (const auto &c : cases) {
    ToolSim *w = start_tool(c.build, true, c.reason, c.version);
    CHECK(w->tool.verdict() == c.verdict);
    CHECK(w->tool.state() == mcuflash::CHECK_CURRENT && w->tool.nvs().arm == 0);
    CHECK_EQ(w->esp_frames[MSG_BOOT_RUN], 1);
    if (c.verdict == rpstd01tool::VERDICT_APP_OLD) {
      CHECK(w->tool.updater().update_state() == UpdateState::REQUESTING);
      CHECK(w->run_until(UpdateState::DONE, 10000));
      w->run(100);
      CHECK(flash_matches_image());
      CHECK(w->text() == UPDATED);
    } else {
      w->run(3000);
      CHECK(w->mode == SimMcu::APP && w->tool.updater().update_state() == UpdateState::IDLE);
      CHECK_EQ(w->begins_seen, 0);
    }
    delete w;
  }
}

static void test_tool_update_now_writes_even_current_application() {
  ToolSim *w = start_tool(image_build_of(app_image.data()));
  CHECK(w->tool.verdict() == rpstd01tool::VERDICT_CURRENT);
  int frames = 0;
  for (int c : w->esp_frames)
    frames += c;
  w->run(3000);
  int after = 0;
  for (int c : w->esp_frames)
    after += c;
  CHECK_EQ(after, frames);  // nothing sent after the decision
  CHECK(w->tool.update_application());
  CHECK(!w->tool.update_application());  // one at a time
  CHECK(w->run_until(UpdateState::DONE, 10000));
  w->run(100);
  CHECK_EQ(w->begins_seen, 1);
  CHECK(w->text() == UPDATED);
  delete w;
}

static void test_tool_failed_update_names_the_buttons() {
  ToolSim *w = start_tool(image_build_of(app_image.data()));
  w->aux_ram_survives_reset = false;
  CHECK(w->tool.update_application());
  CHECK(w->run_until(UpdateState::FAILED, 10000));
  std::string text = w->text();
  CHECK(has(text, "The application update through our bootloader failed: MCU restarted without entering "
                  "update mode. Press Update Application Now"));
  CHECK(w->tool.state() == mcuflash::CHECK_CURRENT && w->tool.nvs().arm == 0);
  delete w;
}

static void test_tool_no_update_now_before_check_decided() {
  Tool t;
  fake_nvs::reset();
  rpstd01tool::NvsToolStore st(true);
  t.begin(&st, carried(), nullptr);
  CHECK(!t.update_application());
}

static void test_tool_no_answer_arms_stc_install_and_stays_quiet() {
  install(image_build_of(app_image.data()));
  ToolSim *w = new ToolSim();
  w->us = 1000000;
  for (int i = 0; i < 600 && !w->tool.decided(); i++)
    w->run(10);
  CHECK(w->tool.verdict() == rpstd01tool::VERDICT_NO_ANSWER && w->tool.state() == mcuflash::CHECK_ARMED);
  const int requests_without_answer =
      (rpstd01tool::CHECK_LISTEN_MS + rpstd01tool::CHECK_ASK_EVERY_MS - 1) / rpstd01tool::CHECK_ASK_EVERY_MS;
  CHECK_EQ(w->esp_frames[MSG_INFO_REQ], requests_without_answer);
  w->power_on();  // the MCU answers now: the tool does not act on it
  w->run(5000);
  CHECK_EQ(w->esp_frames[MSG_INFO_REQ], requests_without_answer);
  CHECK(w->tool.updater().update_state() == UpdateState::IDLE);
  CHECK(w->tool.update_application());
  CHECK(w->run_until(UpdateState::WRITING, 3000));
  w->run(500);
  std::string text = w->text();
  CHECK(has(text, "Install armed") && has(text, "Application update through our bootloader: Writing "));
  CHECK(w->run_until(UpdateState::DONE, 10000));
  w->run(100);
  CHECK(w->tool.verdict() == rpstd01tool::VERDICT_CURRENT);
  CHECK(w->tool.state() == mcuflash::CHECK_CURRENT && w->tool.nvs().arm == 0);
  CHECK(w->text() == UPDATED);
  delete w;
}

// The application crashes (3 watchdog resets) after a check that found it current: written once;
// crashing again after that, the tool stops and says so, and arms nothing.
static void test_tool_crash() {
  ToolSim *w = start_tool(image_build_of(app_image.data()));
  CHECK(w->tool.verdict() == rpstd01tool::VERDICT_CURRENT);
  w->boot_start(BOOT_REASON_WATCHDOG);
  CHECK(w->run_until(UpdateState::DONE, 10000));
  CHECK_EQ(w->begins_seen, 1);
  w->run(500);
  CHECK(w->text() == UPDATED);
  w->boot_start(BOOT_REASON_WATCHDOG);
  w->run(3000);
  CHECK_EQ(w->begins_seen, 1);
  CHECK(w->tool.verdict() == rpstd01tool::VERDICT_UPDATE_MODE);
  CHECK(w->tool.state() == mcuflash::CHECK_CURRENT && w->tool.nvs().arm == 0);
  CHECK(has(w->text(), "reset by the watchdog 3 times. It crashed again after it was written"));
  CHECK(w->tool.update_application());
  CHECK(w->run_until(UpdateState::DONE, 10000));
  w->run(500);
  CHECK(w->text() == UPDATED);
  w->boot_start(BOOT_REASON_WATCHDOG);
  w->run(3000);
  CHECK_EQ(w->begins_seen, 2);
  CHECK(has(w->text(), "It crashed again"));
  delete w;
}

// A crash found by the check: the application is started; when it crashes again it is written once.
static void test_tool_crash_found_by_check_started_then_written_once() {
  ToolSim *w = start_tool(image_build_of(app_image.data()), true, BOOT_REASON_WATCHDOG);
  CHECK(w->tool.verdict() == rpstd01tool::VERDICT_CURRENT);
  CHECK_EQ(w->begins_seen, 0);
  w->boot_start(BOOT_REASON_WATCHDOG);
  CHECK(w->run_until(UpdateState::DONE, 10000));
  w->run(500);
  w->boot_start(BOOT_REASON_WATCHDOG);
  w->run(3000);
  CHECK_EQ(w->begins_seen, 1);
  CHECK(has(w->text(), "It crashed again"));
  delete w;
}

static void test_tool_newer_firmware_not_written_on_crash() {
  ToolSim *w = start_tool(image_build_of(app_image.data()), true, 0, MCU_IMAGE_VERSION + 1);
  CHECK(w->tool.verdict() == rpstd01tool::VERDICT_NEWER);
  w->boot_start(BOOT_REASON_WATCHDOG);
  w->run(3000);
  CHECK_EQ(w->begins_seen, 0);
  CHECK(w->tool.updater().update_state() == UpdateState::IDLE);
  delete w;
}

// A power cut during an update leaves no valid application: when the bootloader is back, the tool
// repairs it at once.
static void test_tool_repair_after_power_cut() {
  ToolSim *w = start_tool(0x12345678);
  w->cut_power_at_write = 30 * WRITE_CHUNK;
  w->cut_ms = 4000;
  CHECK(w->run_until(UpdateState::FAILED, 10000));
  CHECK(has(w->text(), "failed: MCU stopped answering"));
  CHECK(w->run_until(UpdateState::BEGIN, 6000));
  CHECK(w->mode == SimMcu::BOOT && w->boot_reason == BOOT_REASON_INVALID);
  CHECK(w->run_until(UpdateState::DONE, 15000));
  w->run(100);
  CHECK(flash_matches_image());
  CHECK(w->text() == UPDATED);
  delete w;
}

// Update Application Now on a current MCU, every write lost: once the automatic repair is used up,
// the tool decides on the MCU as it is now (our bootloader, no valid application), not as the check
// saw it, and arms an install through STC's bootloader.
static void test_tool_repair_used_up() {
  ToolSim *w = start_tool(image_build_of(app_image.data()));
  CHECK(w->tool.verdict() == rpstd01tool::VERDICT_CURRENT);
  CHECK(w->tool.update_application());
  w->drop_esp_type = MSG_BOOT_WRITE;
  w->drop_esp_count = 1000000;
  for (int i = 0; i < 1200 && w->tool.state() == mcuflash::CHECK_CURRENT; i++)
    w->run(100);
  CHECK(w->mode == SimMcu::BOOT && !app_valid());
  const int begins_with_every_repair = (1 + rpstd01tool::REPAIR_MAX) * BEGINS_UNTIL_GIVING_UP;
  CHECK_EQ(w->begins_seen, begins_with_every_repair);
  CHECK(w->tool.verdict() == rpstd01tool::VERDICT_UPDATE_MODE);
  CHECK(w->tool.state() == mcuflash::CHECK_ARMED && w->tool.nvs().arm == MCU_IMAGE_CRC32);
  std::string text = w->text();
  CHECK(has(text, "with no valid application. Install armed"));
  CHECK(has(text, "Application update through our bootloader: Failed: "));
  CHECK(has(text, "the automatic repair is used up"));
  CHECK(!has(text, "Everything is current"));
  w->run(5000);
  CHECK_EQ(w->begins_seen, begins_with_every_repair);  // nothing more by itself
  CHECK(w->tool.command(mcuflash::COMMAND_DISARM));
  CHECK(w->tool.state() == mcuflash::CHECK_DISARMED);
  w->drop_esp_count = 0;
  CHECK(w->tool.update_application());
  CHECK(w->run_until(UpdateState::DONE, 10000));
  w->run(100);
  CHECK(w->tool.state() == mcuflash::CHECK_CURRENT && w->text() == UPDATED);
  delete w;
}

int main() {
  RUN(test_carried_image);
  RUN(test_update_from_application);
  RUN(test_update_lost_reply_and_frames);
  RUN(test_update_lost_command_sent_again);
  RUN(test_update_every_write_lost_fails_then_new_start_works);
  RUN(test_update_tampered_write_restarts);
  RUN(test_update_tampered_every_time_fails_in_bootloader);
  RUN(test_update_long_power_cut_fails);
  RUN(test_update_short_power_cut_starts_again_from_begin);
  RUN(test_update_from_bootloader_begins_at_once);
  RUN(test_request_resent_until_timeout);
  RUN(test_request_without_any_status_times_out_sending_nothing);
  RUN(test_retry_timer);
  RUN(test_crossing_error_status);
  RUN(test_crossing_ok_status_to_begin);
  RUN(test_other_bootloader_never_written);
  RUN(test_boot_receives_while_sending);
  RUN(test_update_aux_ram_lost);
  RUN(test_update_request_never_received_times_out);
  RUN(test_update_esp_restart_in_update_mode);
  RUN(test_update_just_after_mcu_start);
  RUN(test_update_restarts);
  RUN(test_update_info_frames_while_requesting_do_not_fail_it);
  RUN(test_update_aux_ram_lost_twice);
  RUN(test_tool_updates_old_application_over_link_at_once);
  RUN(test_tool_update_mode_invalid_application);
  RUN(test_tool_update_mode_valid_application_started_first);
  RUN(test_tool_update_now_writes_even_current_application);
  RUN(test_tool_failed_update_names_the_buttons);
  RUN(test_tool_no_update_now_before_check_decided);
  RUN(test_tool_no_answer_arms_stc_install_and_stays_quiet);
  RUN(test_tool_crash);
  RUN(test_tool_crash_found_by_check_started_then_written_once);
  RUN(test_tool_newer_firmware_not_written_on_crash);
  RUN(test_tool_repair_after_power_cut);
  RUN(test_tool_repair_used_up);
  TEST_EXIT();
}
