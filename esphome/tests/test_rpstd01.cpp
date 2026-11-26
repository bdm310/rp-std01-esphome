// The main configuration's link, checked against the MCU firmware's own C code.
#include "esphome/components/rpstd01/rpstd01.h"

#include <fstream>
#include <string>

#include "sim_wrapper.h"

static void test_crc_and_build() {
  for (int n = 0; n < 200; n++) {
    uint8_t data[140];
    uint8_t len = rnd() % 133;
    for (int i = 0; i < len; i++)
      data[i] = rnd();
    uint16_t c = CRC16_INIT;
    for (int i = 0; i < len; i++)
      c = crc16_update(c, data[i]);
    CHECK_EQ(crc16(data, len), c);
    uint8_t type = rnd();
    uint8_t mine[140];
    size_t m = build_frame(mine, type, data, len);
    auto ref = fw_frame(type, data, len);
    CHECK_EQ(m, ref.size());
    CHECK(memcmp(mine, ref.data(), m) == 0);
  }
  // build_frame works in place, as the payload may already sit at out + 3.
  uint8_t f[10] = {0, 0, 0, 1, 2, 3};
  build_frame(f, 0x42, f + 3, 3);
  uint8_t p[3] = {1, 2, 3};
  auto ref = fw_frame(0x42, p, 3);
  CHECK(memcmp(f, ref.data(), ref.size()) == 0);
}

static std::vector<Frame> feed_one(FrameParser &parser, uint8_t byte) {
  std::vector<Frame> out;
  parser.feed(byte, [&out](uint8_t type, const uint8_t *payload, uint8_t len) {
    out.push_back({type, std::vector<uint8_t>(payload, payload + len)});
  });
  return out;
}

// Frames from the MCU with noise and bad frames between them: no frame is made up or garbled, the
// good ones arrive in order, and most of them arrive.
static void test_parser() {
  static const uint8_t TYPES[] = {MSG_STATUS, MSG_INFO, MSG_BOOT_STATUS};
  size_t sent_total = 0, got_total = 0;
  for (int round = 0; round < 3000; round++) {
    std::vector<uint8_t> stream;
    std::vector<Frame> sent;
    for (int k = 0; k < 30; k++) {
      int kind = rnd() % 6;
      if (kind == 0) {
        int n = rnd() % 8;
        for (int i = 0; i < n; i++)
          stream.push_back(rnd() % 3 == 0 ? FRAME_START : rnd());
      } else {
        uint8_t type = TYPES[rnd() % 3];
        // Mostly the type's own length, but any length the firmware can build is a frame.
        const uint8_t own = type == MSG_STATUS ? STATUS_LEN : type == MSG_INFO ? INFO_LEN : BOOT_STATUS_LEN;
        uint8_t len = rnd() % 4 ? own : rnd() % (UINT8_MAX - FRAME_OVERHEAD + 1);
        std::vector<uint8_t> p(len);
        for (auto &b : p)
          b = rnd() % 3 == 0 ? FRAME_START : rnd();
        auto f = fw_frame(type, p.data(), len);
        if (kind == 1)
          f[3 + (len ? rnd() % len : 0)] ^= 0x10;  // bad CRC
        else
          sent.push_back({type, p});
        stream.insert(stream.end(), f.begin(), f.end());
      }
    }
    FrameParser parser;
    size_t next = 0;
    for (uint8_t b : stream) {
      for (const Frame &got : feed_one(parser, b)) {
        while (next < sent.size() && !(sent[next].type == got.type && sent[next].payload == got.payload))
          next++;
        CHECK(next < sent.size());
        next++;
        got_total++;
      }
    }
    sent_total += sent.size();
  }
  CHECK(got_total > sent_total * 3 / 4);
}

// The frames in bytes, followed by a pause.
static std::vector<Frame> esp_parse(const std::vector<uint8_t> &bytes) {
  FrameParser parser;
  std::vector<Frame> out;
  for (uint8_t b : bytes)
    for (Frame &f : feed_one(parser, b))
      out.push_back(std::move(f));
  parser.flush([&out](uint8_t type, const uint8_t *payload, uint8_t len) {
    out.push_back({type, std::vector<uint8_t>(payload, payload + len)});
  });
  return out;
}

// A false start of the longest length holds back the frames after it until the pause after them.
static void test_parser_resync_after_noise_in_frame() {
  Link link;
  FakeUart u;
  uint8_t st[STATUS_LEN] = {0};
  std::vector<uint8_t> all;
  for (int i = 0; i < 5; i++) {
    st[SF_LEDS] = i;
    auto f = fw_frame(MSG_STATUS, st, STATUS_LEN);
    all.insert(all.end(), f.begin(), f.end());
  }
  all.insert(all.begin() + 10, {0x00, FRAME_START, 0x81, 0xFF});
  for (size_t i = 0; i < all.size(); i += 7) {
    u.push(std::vector<uint8_t>(all.begin() + i, all.begin() + std::min(all.size(), i + 7)));
    link.loop(u, 1);
  }
  CHECK(!link.have_status());
  link.loop(u, 2 + detail::RX_GAP_MS);
  auto ref = esp_parse(all);
  CHECK_EQ(ref.size(), 4);
  CHECK_EQ(ref[0].payload[SF_LEDS], 1);
  CHECK_EQ(link.status().leds, 4);
}

static void test_parser_resync_after_cut_frame_with_false_headers() {
  Link link;
  FakeUart u;
  uint8_t st[STATUS_LEN] = {0};
  std::vector<uint8_t> cut;
  for (int i = 0; i < 20; i++) {
    st[SF_STATE] = ST_FAULT;
    st[SF_FAULTS] = 0x16;
    st[SF_BUTTONS] = BTN_ALL;
    detail::put16(st + SF_HALF, 7680);
    // An 0xA5 with 0x03 after it, and the wheel's low byte as a length: link.c alone never gets back in step.
    detail::put16(st + SF_THERM, 0x03A5);
    detail::put16(st + SF_WHEEL, 806);
    st[SF_LEDS] = i;
    auto f = fw_frame(MSG_STATUS, st, STATUS_LEN);
    if (i == 0)
      f.resize(11);
    cut.insert(cut.end(), f.begin(), f.end());
  }
  for (size_t i = 0; i < cut.size(); i += 27) {
    u.push(std::vector<uint8_t>(cut.begin() + i, cut.begin() + std::min(cut.size(), i + 27)));
    link.loop(u, 1);
  }
  auto got = esp_parse(cut);
  CHECK_EQ(got.size(), 19);
  CHECK_EQ(got.front().payload[SF_LEDS], 1);
  CHECK_EQ(link.status().leds, 19);
  CHECK(fw_parse(cut).empty());
}

// A reset cuts a status off and the new application's info frame follows within RX_GAP_MS: the
// info frame sits inside the bytes the cut status claims, and is still read.
static void test_parser_reads_frame_inside_cut_frame() {
  uint8_t st[STATUS_LEN] = {0};
  st[SF_BOOT_ID] = 9;
  const uint8_t info[INFO_LEN] = {1, 0, 2, 3, 4, 5, RESET_WATCHDOG, 1, BOOT_VERSION, 9};
  auto cut = fw_frame(MSG_STATUS, st, STATUS_LEN);
  cut.resize(11);
  const auto info_frame = fw_frame(MSG_INFO, info, INFO_LEN);
  const auto status_frame = fw_frame(MSG_STATUS, st, STATUS_LEN);
  std::vector<uint8_t> stream = cut;
  stream.insert(stream.end(), info_frame.begin(), info_frame.end());
  stream.insert(stream.end(), status_frame.begin(), status_frame.end());
  auto got = esp_parse(stream);
  CHECK(got.size() == 2 && got[0].type == MSG_INFO && got[1].type == MSG_STATUS);
  CHECK(!got.empty() && got[0].payload == std::vector<uint8_t>(info, info + INFO_LEN));

  Link link;
  FakeUart u;
  u.push(stream);
  link.loop(u, 1);
  CHECK(link.have_current_info());
  CHECK_EQ(link.current_info().boot_id, 9);
}

// True if the frame holds, after its start, an 0xA5 with a type the parser takes after it: joined
// there, the parser waits for the false frame's length before it scans those bytes again.
static bool has_false_start(const std::vector<uint8_t> &f) {
  for (size_t i = 1; i + 1 < f.size(); i++) {
    const uint8_t type = f[i + 1];
    if (f[i] == FRAME_START && (type == MSG_STATUS || type == MSG_INFO || type == MSG_BOOT_STATUS))
      return true;
  }
  return false;
}

static void test_parser_rejoins_repeated_status_ending_in_false_header() {
  const uint8_t night_light_a5_status[] = {0x02, 0x00, 0x07, 0xA0, 0x0F, 0xA0, 0x0F, 0x00, 0x1E, 0xFF,
                                           0x03, 0x01, 0xA5, 0x03, 0x03, 0x00, 0x00, 0x01, 0xA5};
  static_assert(sizeof night_light_a5_status == STATUS_LEN, "a whole status");
  const auto rf = fw_frame(MSG_STATUS, night_light_a5_status, STATUS_LEN);
  CHECK(rf[rf.size() - 2] == MSG_BOOT_STATUS && rf.back() == 0x6E);
  FrameParser p;
  int got = 0;
  for (int k = 0; k < 100; k++)
    for (size_t i = k == 0 ? rf.size() - 3 : 0; i < rf.size(); i++)
      got += feed_one(p, rf[i]).size();
  CHECK_EQ(got, 99);
}

// Statuses that stay alike, joined from any byte offset: the parser reads every whole one, and the
// first as soon as it is complete unless a false start holds it back, which it does for at most
// FALSE_START_HOLD_MS after the status before the join (one status per STATUS_PERIOD_MS).
static void test_parser_desync_sweep() {
  int statuses = 0, checked = 0, false_starts = 0;
  uint32_t longest_gap_ms = 0;
  uint8_t st[STATUS_LEN] = {0};
  for (int night : {165, 0, 30, 0x81, 0x90, 255}) {
    for (uint16_t wheel = 0; wheel < 1024; wheel++) {
      st[SF_STATE] = ST_FIRING;
      detail::put16(st + SF_TARGET, 3000);
      detail::put16(st + SF_THERM, 0x03A5);
      detail::put16(st + SF_WHEEL, wheel);
      st[SF_BOOT_ID] = 7;
      st[SF_LEDS] = 0x1F;
      st[SF_NIGHT] = night;
      const auto f = fw_frame(MSG_STATUS, st, STATUS_LEN);
      const size_t whole = (UINT8_MAX + FRAME_OVERHEAD) / f.size() + 2;
      const bool false_start = has_false_start(f);
      statuses++;
      false_starts += false_start;
      for (size_t offset = 1; offset < f.size(); offset++) {
        FrameParser p;
        std::vector<uint8_t> stream(f.begin() + offset, f.end());
        for (size_t k = 0; k < whole; k++)
          stream.insert(stream.end(), f.begin(), f.end());
        size_t first = 0, got = 0;
        for (size_t i = 0; i < stream.size(); i++) {
          for (const Frame &fr : feed_one(p, stream[i])) {
            CHECK(fr.type == MSG_STATUS && fr.payload == std::vector<uint8_t>(st, st + STATUS_LEN));
            got++;
            if (first == 0)
              first = i + 1;
          }
        }
        CHECK_EQ(got, whole);
        if (!false_start)
          CHECK_EQ(first, f.size() - offset + f.size());
        const uint32_t gap_ms = static_cast<uint32_t>((first - 1 + offset) / f.size() + 1) * STATUS_PERIOD_MS;
        longest_gap_ms = std::max(longest_gap_ms, gap_ms);
        checked++;
      }
    }
  }
  CHECK(false_starts > 0);
  CHECK(longest_gap_ms <= detail::FALSE_START_HOLD_MS);
  printf("    %d offsets checked, %d of %d statuses with a false start, longest gap %u ms\n", checked, false_starts,
         statuses, (unsigned) longest_gap_ms);
}

// Noise that starts a frame of the longest length between two statuses: the link still shows the
// application all the while, with one status per STATUS_PERIOD_MS read every millisecond.
static void test_false_start_of_longest_frame_keeps_application_seen() {
  Link link;
  FakeUart u;
  uint8_t st[STATUS_LEN] = {0};
  const auto f = fw_frame(MSG_STATUS, st, STATUS_LEN);
  const uint32_t noise_ms = 50 * STATUS_PERIOD_MS, end_ms = noise_ms + 2 * detail::APP_SEEN_MS;
  int not_application = 0;
  for (uint32_t ms = 0; ms < end_ms; ms++) {
    if (ms % STATUS_PERIOD_MS == 0)
      u.push(f);
    if (ms == noise_ms)
      u.push({FRAME_START, MSG_STATUS, UINT8_MAX});
    link.loop(u, ms);
    if (ms > STATUS_PERIOD_MS && link.link_state() != LinkState::APPLICATION)
      not_application++;
  }
  CHECK_EQ(not_application, 0);
}

static void test_decoders() {
  for (int n = 0; n < 200; n++) {
    status_in_t s;
    s.state = rnd() % 5;
    s.faults = rnd() & 0x1F;
    s.target = rnd();
    s.current = rnd();
    s.half = rnd();
    s.relay = rnd() % 2;
    s.therm = rnd() % 1024;
    s.wheel = rnd() % 1024;
    s.starting = rnd() % 2;
    s.boot_id = rnd();
    s.leds = rnd() & 0x3F;
    s.night = rnd();
    window_t w;
    window_reset(&w);
    int k = rnd() % 4;
    for (int i = 0; i < k; i++)
      window_add_ks(&w, rnd() % 1024, rnd() % 2);
    for (int i = 0; i < 3; i++)
      window_add_buttons(&w, rnd() & BTN_ALL);
    uint8_t out[STATUS_LEN];
    status_build(out, &s, &w);
    Status d;
    CHECK(decode_status(out, STATUS_LEN, &d));
    CHECK_EQ(d.state, s.state);
    CHECK_EQ(d.faults, s.faults);
    CHECK_EQ(d.buttons, out[2]);
    CHECK_EQ(d.buttons & BTN_ALL, w.btn_last & BTN_ALL);
    CHECK_EQ((d.buttons >> BTN_CHANGED_SHIFT) & BTN_ALL, (w.btn_and ^ w.btn_or) & BTN_ALL);
    CHECK_EQ(d.target, s.target);
    CHECK_EQ(d.current, s.current);
    CHECK_EQ(d.half, s.half);
    CHECK_EQ(d.ks_min, w.ks_min);
    if (!k)
      CHECK_EQ(d.ks_min, KS_NO_SAMPLE);
    CHECK_EQ(d.flags & 1, s.relay);
    CHECK_EQ(d.flags, out[11]);
    CHECK_EQ(d.therm, s.therm);
    CHECK_EQ(d.wheel, s.wheel);
    CHECK_EQ(d.starting(), s.starting);
    CHECK_EQ(d.boot_id, s.boot_id);
    CHECK_EQ(d.leds, s.leds);
    CHECK_EQ(d.night, s.night);
    CHECK(!decode_status(out, STATUS_LEN - 1, &d));
    CHECK(!decode_status(out, STATUS_LEN + 1, &d));
  }
  uint8_t out[INFO_LEN];
  info_build(out, 0x0102, 0xD75BF4E3u, 2, 7, 2, 0x81);
  Info i;
  CHECK(decode_info(out, INFO_LEN, &i));
  CHECK_EQ(i.boot_id, 0x81);
  CHECK_EQ(i.version, 0x0102);
  CHECK_EQ(i.build, 0xD75BF4E3u);
  CHECK_EQ(i.reset_cause, 2);
  CHECK_EQ(i.wdt_count, 7);
  CHECK_EQ(i.boot_version, 2);
  CHECK(!decode_info(out, INFO_LEN - 1, &i));
  CHECK(!decode_info(out, INFO_LEN + 1, &i));
  upd_t u;
  upd_init(&u);
  uint8_t bs_out[BOOT_STATUS_LEN];
  upd_status(&u, 3, bs_out);
  BootStatus bs;
  CHECK(decode_boot_status(bs_out, BOOT_STATUS_LEN, &bs));
  CHECK_EQ(bs.version, bs_out[0]);
  CHECK_EQ(bs.version, BOOT_VERSION);
  CHECK_EQ(bs.reason, 3);
  CHECK_EQ(bs.result, BR_NO_COMMAND);
  CHECK_EQ(bs.next, 0);
}

struct StatusSpec {
  uint8_t state = ST_OFF, faults = 0, buttons = BTN_ALL;
  uint16_t target = 0, half = 7680;
  uint16_t ks_min = KS_NO_SAMPLE;
  uint8_t relay = 0;
  uint16_t therm = 933, wheel = 806;
  bool starting = false;  // the MCU's start-up: thermistor average not ready
};

// A status, with the MCU's boot ID and the outputs it uses.
static std::vector<uint8_t> status_bytes(const StatusSpec &s, uint8_t boot_id = 1, uint8_t leds = 0,
                                         uint8_t night = 0) {
  uint8_t p[STATUS_LEN]{};
  p[SF_STATE] = s.state;
  p[SF_FAULTS] = s.faults;
  p[SF_BUTTONS] = s.buttons;
  detail::put16(p + SF_TARGET, s.target);
  detail::put16(p + SF_CURRENT, s.state == ST_FIRING ? s.target : 0);
  detail::put16(p + SF_HALF, s.half);
  detail::put16(p + SF_KS_MIN, s.ks_min);
  p[SF_FLAGS] = s.relay | (s.starting ? S_STARTING : 0);
  detail::put16(p + SF_THERM, s.therm);
  detail::put16(p + SF_WHEEL, s.wheel);
  p[SF_BOOT_ID] = boot_id;
  p[SF_LEDS] = leds;
  p[SF_NIGHT] = night;
  return fw_frame(MSG_STATUS, p, STATUS_LEN);
}

static std::vector<uint8_t> info_bytes(uint32_t build, uint8_t boot_id = 1, uint16_t version = FW_VERSION) {
  uint8_t p[INFO_LEN];
  info_build(p, version, build, 1, 0, BOOT_VERSION, boot_id);
  return fw_frame(MSG_INFO, p, INFO_LEN);
}

static std::vector<uint8_t> boot_status_bytes(uint8_t reason) {
  const uint8_t app_valid = 1;
  uint8_t p[BOOT_STATUS_LEN] = {BOOT_VERSION, app_valid, reason, BR_OK, 0, 0};
  return fw_frame(MSG_BOOT_STATUS, p, BOOT_STATUS_LEN);
}

// A number's initial_value in rp-std01-core.yaml.
static float core_initial_value(const std::string &id) {
  std::ifstream yaml("esphome/rp-std01-core.yaml");
  bool in_number = false;
  for (std::string line; std::getline(yaml, line);) {
    const size_t text = line.find_first_not_of(" -");
    if (text != std::string::npos && line.compare(text, std::string::npos, "id: " + id) == 0)
      in_number = true;
    else if (in_number && text != std::string::npos && line.compare(text, 15, "initial_value: ") == 0)
      return std::stof(line.substr(text + 15));
  }
  fprintf(stderr, "rp-std01-core.yaml: no initial_value for %s\n", id.c_str());
  exit(1);
}

// The settings the component applies from its numbers at setup.
static const float TEST_DIM_END = core_initial_value("dim_end") / 100, TEST_BRIGHT_END =
    core_initial_value("bright_end") / 100;
static const uint8_t TEST_FADE_LIMIT = static_cast<uint8_t>(core_initial_value("fade_limit"));
static const uint16_t ADDON_UP_OFF = core_initial_value("addon_up_off"),
                      ADDON_DOWN_OFF = core_initial_value("addon_down_off"),
                      ADDON_UP_ON = core_initial_value("addon_up_on"),
                      ADDON_DOWN_ON = core_initial_value("addon_down_on");
static void apply_test_settings(Link &link) {
  link.set_dim_end_fraction(TEST_DIM_END);
  link.set_bright_end_fraction(TEST_BRIGHT_END);
  link.set_fade_limit(TEST_FADE_LIMIT);
  link.set_addon_thresholds(ADDON_UP_OFF, ADDON_DOWN_OFF, ADDON_UP_ON, ADDON_DOWN_ON);
}

static bool b3_blinks(const Link &link) { return link.b3_on(true) && !link.b3_on(false); }

// Runs the link every 10 ms, with a status frame each time (if a spec is given), as the MCU
// would: its info frame with the first status and every second, its boot ID, and the
// outputs from the last outputs frame it received.
struct Bench {
  Link link;
  Bench() { apply_test_settings(this->link); }
  FakeUart u;
  uint32_t t = 0;
  std::vector<Frame> sent;
  bool echo = false;       // status reports the last target sent, as a well-behaved MCU would
  bool auto_adopt = true;  // call take_adopt() after the first status, as the YAML does
  bool auto_info = true;   // the MCU's info frames
  uint16_t version = FW_VERSION;
  bool adopted = false;
  uint16_t echo_target = 0;
  uint8_t boot_id = 1, leds = 0, night = 0;
  uint32_t build = 1;
  int drop_outputs = 0;  // outputs frames it loses
  bool info_sent = false;
  uint32_t info_t = 0;
  std::vector<uint8_t> status(const StatusSpec &s) {
    return status_bytes(s, this->boot_id, this->leds, this->night);
  }
  // The MCU restarts: a new boot ID, outputs off, and its info frame (sent before its first status).
  void restart() {
    this->boot_id++;
    this->leds = this->night = 0;
    this->info_sent = false;
  }
  void step(const StatusSpec *s) {
    if (s) {
      if (this->auto_info && (!this->info_sent || this->t - this->info_t >= 1000)) {
        this->u.push(info_bytes(this->build, this->boot_id, this->version));
        this->info_sent = true;
        this->info_t = this->t;
      }
      StatusSpec c = *s;
      if (this->echo)
        c.target = this->echo_target;
      this->u.push(this->status(c));
    }
    this->link.loop(this->u, this->t);
    if (this->auto_adopt && !this->adopted && this->link.have_status()) {
      // As the YAML: take_adopt() after the loop; the gate goes out on the next loop, which
      // is run here at once to keep the timeline simple.
      float lvl;
      this->link.take_adopt(&lvl);
      this->adopted = true;
      this->link.loop(this->u, this->t);
    }
    auto f = this->u.take_frames();
    for (auto &fr : f) {
      if (fr.type == MSG_GATE)
        this->echo_target = g16(fr.payload, 0);
      if (fr.type == MSG_OUTPUTS && fr.payload.size() == 2) {
        if (this->drop_outputs > 0) {
          this->drop_outputs--;
        } else {
          this->leds = fr.payload[0];
          this->night = fr.payload[1];
        }
      }
    }
    this->sent.insert(this->sent.end(), f.begin(), f.end());
    this->t += 10;
  }
  void run(uint32_t ms, const StatusSpec *s) {
    for (uint32_t end = this->t + ms; this->t < end;)
      this->step(s);
  }
  std::vector<Frame> take(uint8_t type) {
    auto r = of_type(this->sent, type);
    this->sent.clear();
    return r;
  }
};

static void test_no_gate_before_status() {
  Bench b;
  b.link.set_level(0.5f);
  b.run(500, nullptr);
  CHECK(b.take(MSG_GATE).empty());
  CHECK_EQ((int) b.link.link_state(), (int) LinkState::NO_RESPONSE);
  CHECK(b.link.problem());
  StatusSpec s;
  b.step(&s);  // the MCU's info frame and its first status
  auto g = b.take(MSG_GATE);
  CHECK_EQ(g.size(), 1);
  CHECK(g.size() == 1 && g[0].payload.size() == 4 && g16(g[0].payload, 0) > EARLIEST_FIRE);
  CHECK(g.size() == 1 && g[0].payload[2] == TEST_FADE_LIMIT && g[0].payload[3] == TEST_FADE_LIMIT);
  CHECK_EQ((int) b.link.link_state(), (int) LinkState::APPLICATION);
  CHECK(!b.link.problem());
}

static void test_not_driven_until_info_frame() {
  Bench c;
  StatusSpec s;
  c.auto_info = false;
  c.link.set_level(0.5f);
  c.link.set_leds(OUTPUTS_LED2, true);
  c.run(3005, &s);
  CHECK(c.sent.empty());
  CHECK(c.link.problem());
  CHECK(!c.link.have_current_info());
  c.u.push(info_bytes(1));
  c.step(&s);
  CHECK_EQ(c.take(MSG_GATE).size(), 1);
  c.step(&s);
  CHECK_EQ(c.take(MSG_OUTPUTS).size(), 1);
  CHECK(!c.link.problem());
}

// First status, info and outputs out of the way, so that only gate frames are left to send.
static void settle(Bench &b, const StatusSpec &s) {
  b.run(130, &s);
  b.sent.clear();
}

// The lamp already firing, so the real target goes out: returns that target, once the status shows it.
static uint16_t fire_at_full_level(Bench &b, StatusSpec &s) {
  settle(b, s);
  s.state = ST_FIRING;
  b.link.set_level(1.0f);
  b.step(&s);
  auto g = b.take(MSG_GATE);
  CHECK_EQ(g.size(), 1);
  s.target = g.empty() ? 0 : g16(g[0].payload, 0);
  b.run(50, &s);
  b.take(MSG_GATE);
  return s.target;
}

static void test_gate_resent_every_30ms_until_acknowledged_then_never() {
  Bench b;
  StatusSpec s;
  settle(b, s);
  s.state = ST_FIRING;
  b.link.set_level(1.0f);
  b.step(&s);
  auto g = b.take(MSG_GATE);
  CHECK_EQ(g.size(), 1);
  b.run(30, &s);
  CHECK_EQ(b.take(MSG_GATE).size(), 1);
  s.target = g.empty() ? 0 : g16(g[0].payload, 0);
  b.run(3000, &s);
  CHECK_EQ(b.take(MSG_GATE).size(), 0);
}

static void test_clamped_bit_with_same_target_is_not_acknowledgement() {
  Bench b;
  StatusSpec s;
  fire_at_full_level(b, s);
  s.faults = F_CLAMPED;
  b.run(60, &s);
  CHECK(b.take(MSG_GATE).size() >= 1);
}

static void test_mcu_reset_restarts_from_dim_end_until_firing() {
  Bench b;
  StatusSpec s;
  const uint16_t target = fire_at_full_level(b, s);
  s.state = ST_OFF;
  s.target = 0;
  b.run(40, &s);
  auto g = b.take(MSG_GATE);
  CHECK(!g.empty());
  uint16_t dim = g.empty() ? 0 : g16(g.back().payload, 0);
  CHECK(dim > target);
  CHECK_NEAR(power_share(dim, 7680), power_share(TEST_DIM_END * 7680, 7680), 0.001);
  s.state = ST_SETTLE;
  s.target = dim;
  b.run(50, &s);
  CHECK(b.take(MSG_GATE).empty());
  s.state = ST_FIRING;
  b.step(&s);
  g = b.take(MSG_GATE);
  CHECK(g.size() == 1 && g16(g[0].payload, 0) == target);
}

static void test_new_level_sent_at_once_newest_only() {
  Bench b;
  StatusSpec s;
  const uint16_t target = fire_at_full_level(b, s);
  b.link.set_level(0.3f);
  b.link.set_level(0.2f);
  b.step(&s);
  auto g = b.take(MSG_GATE);
  CHECK_EQ(g.size(), 1);
  CHECK(!g.empty() && g16(g[0].payload, 0) > target);
}

static void test_fade_limit_change_sent_at_once() {
  Bench b;
  StatusSpec s;
  fire_at_full_level(b, s);
  b.link.set_fade_limit(10);
  b.step(&s);
  auto g = b.take(MSG_GATE);
  CHECK(g.size() == 1 && g[0].payload[2] == 10 && g[0].payload[3] == 10);
}

static void test_ack_zo_not_good() {
  Bench b;
  StatusSpec s;
  s.half = 0;
  s.faults = F_ZO;
  b.step(&s);  // first status and info, level 0: target 0
  CHECK_EQ(b.take(MSG_GATE).size(), 1);
  b.run(900, &s);  // 0 acknowledged with half 0, no clamped bit
  CHECK_EQ(b.take(MSG_GATE).size(), 0);
  b.link.set_level(0.5f);
  b.step(&s);
  auto g = b.take(MSG_GATE);
  CHECK_EQ(g.size(), 1);
  uint16_t sent = g16(g[0].payload, 0);
  CHECK(sent != 0);
  // With half 0 the MCU clamps any target to 0 and sets the clamped bit: that is the acknowledgement.
  s.faults = F_ZO | F_CLAMPED;
  b.run(900, &s);
  CHECK_EQ(b.take(MSG_GATE).size(), 0);
  s.faults = F_ZO;
  b.run(40, &s);
  CHECK(b.take(MSG_GATE).size() >= 1);
  s.faults = 0;
  s.half = 7000;
  b.step(&s);
  g = b.take(MSG_GATE);
  CHECK_EQ(g.size(), 1);
  b.link.set_dim_end_fraction(0.99f);
  b.link.set_level(0.0001f);
  b.step(&s);
  g = b.take(MSG_GATE);
  CHECK(g.size() == 1 && g16(g[0].payload, 0) == 7000 - LATEST_BEFORE_EDGE - 8);
}

static void test_adopt_on() {
  for (uint16_t mcu_target : {2400, 3000, 4000}) {
    Bench b;
    b.auto_adopt = false;
    b.link.set_dim_end_from_wheel(false);
    b.link.set_soft_reboot(true);
    b.link.set_level(0.9f);  // stale restored value
    float lvl = -1;
    CHECK(b.link.take_adopt(&lvl) == Adopt::NONE);
    StatusSpec s;
    s.state = ST_FIRING;
    s.target = mcu_target;
    s.relay = S_RELAY;
    b.run(200, &s);
    CHECK(b.take(MSG_GATE).empty());
    CHECK(b.link.take_adopt(&lvl) == Adopt::ON);
    CHECK(lvl > 0 && lvl <= 1);
    CHECK(b.link.take_adopt(&lvl) == Adopt::NONE);
    b.link.set_level(lvl);
    b.step(&s);
    auto g = b.take(MSG_GATE);
    CHECK_EQ(g.size(), 1);
    CHECK(!g.empty() && abs((int) g16(g[0].payload, 0) - mcu_target) <= 2);
  }
}

static void test_adopt_off() {
  Bench b;
  b.auto_adopt = false;
  b.link.set_soft_reboot(true);
  b.link.set_level(0.9f);
  StatusSpec s;
  b.step(&s);
  CHECK(b.take(MSG_GATE).empty());
  float lvl = -1;
  CHECK(b.link.take_adopt(&lvl) == Adopt::OFF);
  CHECK_EQ(lvl, 0);
  b.step(&s);
  auto g = b.take(MSG_GATE);
  CHECK(g.size() == 1 && g16(g[0].payload, 0) == 0);
}

static void test_adopt_off_for_fault_and_stopping() {
  for (uint8_t st : {ST_FAULT, ST_STOPPING}) {
    Bench f;
    f.auto_adopt = false;
    f.link.set_soft_reboot(true);
    StatusSpec fs;
    fs.state = st;
    fs.faults = st == ST_FAULT ? F_OVERTEMP : 0;
    f.step(&fs);
    float lvl = -1;
    CHECK(f.link.take_adopt(&lvl) == Adopt::OFF);
  }
}

static void test_power_on_with_mcu_off_adopts_none_then_gates() {
  Bench p;
  p.auto_adopt = false;
  p.link.set_soft_reboot(false);
  p.link.set_level(0.5f);
  float lvl = -1;
  CHECK(p.link.take_adopt(&lvl) == Adopt::NONE);  // no status yet
  StatusSpec off;
  p.run(50, &off);
  CHECK(p.take(MSG_GATE).empty());
  CHECK(p.link.take_adopt(&lvl) == Adopt::NONE);
  p.step(&off);
  auto g = p.take(MSG_GATE);
  CHECK(g.size() == 1 && g16(g[0].payload, 0) != 0);
  CHECK(p.link.take_adopt(&lvl) == Adopt::NONE);
}

static void test_adopt_none_without_soft_reboot_setting() {
  Bench q;
  q.auto_adopt = false;
  StatusSpec off;
  q.step(&off);
  float lvl = -1;
  CHECK(q.link.take_adopt(&lvl) == Adopt::NONE);
}

// For example a brown-out the MCU survived.
static void test_power_on_with_mcu_firing_adopts_on() {
  for (uint8_t st : {ST_FIRING, ST_SETTLE}) {
    Bench r;
    r.auto_adopt = false;
    r.link.set_soft_reboot(false);
    r.link.set_dim_end_from_wheel(false);
    StatusSpec on;
    on.state = st;
    on.target = 3000;
    on.relay = S_RELAY;
    r.step(&on);
    CHECK(r.take(MSG_GATE).empty());
    float lvl = -1;
    CHECK(r.link.take_adopt(&lvl) == Adopt::ON);
    CHECK(lvl > 0 && lvl < 1);
    r.step(&on);
    auto g = r.take(MSG_GATE);
    if (st == ST_FIRING)
      CHECK(g.size() == 1 && abs((int) g16(g[0].payload, 0) - 3000) <= 2);
    else
      CHECK(g.size() == 1 && g16(g[0].payload, 0) > 3000);  // SETTLE: dim end until FIRING
  }
}

static void test_fault() {
  Bench b;
  StatusSpec s;
  b.link.set_level(0.5f);
  b.step(&s);
  CHECK(!b.link.take_fault_event());
  s.state = ST_FAULT;
  s.faults = F_OVERTEMP | F_ZO;
  b.run(100, &s);
  CHECK(b.link.take_fault_event());
  CHECK(!b.link.take_fault_event());
  auto g = b.take(MSG_GATE);
  CHECK(!g.empty() && g16(g.back().payload, 0) == 0);
  CHECK(b.link.problem());
  CHECK(strcmp(b.link.state_text(), "Fault") == 0);
  char buf[64];
  b.link.faults_text(buf, sizeof buf);
  CHECK(strcmp(buf, "over-temperature, mains lost") == 0);
  s.state = ST_OFF;
  s.faults = F_CLAMPED;
  b.run(50, &s);
  b.link.faults_text(buf, sizeof buf);
  CHECK(strcmp(buf, "none") == 0);
  s.state = ST_FAULT;
  b.run(50, &s);
  CHECK(b.link.take_fault_event());
}

static void test_startup_fault_not_reported_and_target_waits_for_off() {
  Bench b;
  b.link.set_level(0.5f);
  StatusSpec s;
  s.state = ST_FAULT;
  s.faults = F_THERM | F_ZO | F_CLAMPED;
  s.starting = true;
  b.run(80, &s);
  CHECK(!b.link.take_fault_event());
  CHECK(!b.link.fault_holds_off());
  CHECK(!b.link.problem());
  b.link.set_level(0.6f);  // a turn-on meanwhile raises nothing either
  b.step(&s);
  CHECK(!b.link.take_fault_event());
  auto g = b.take(MSG_GATE);
  CHECK(!g.empty() && g16(g.back().payload, 0) == 0);
  StatusSpec off;
  b.step(&off);
  g = b.take(MSG_GATE);
  CHECK(g.size() == 1 && g16(g[0].payload, 0) != 0);
  b.run(1000, &off);
  CHECK(!b.link.take_fault_event());
}

static void test_fault_reported_when_starting_clears() {
  Bench c;
  StatusSpec f;
  f.state = ST_FAULT;
  f.faults = F_THERM;
  f.starting = true;
  c.run(80, &f);
  CHECK(!c.link.take_fault_event());
  f.starting = false;
  c.step(&f);
  CHECK(c.link.take_fault_event());
  CHECK(c.link.fault_holds_off());
  CHECK(c.link.problem());
  c.run(100, &f);
  CHECK(!c.link.take_fault_event());
}

static void report_thermistor_fault(Bench &c, StatusSpec &f) {
  f.state = ST_FAULT;
  f.faults = F_THERM;
  c.run(100, &f);
  CHECK(c.link.take_fault_event());
}

static void test_restart_with_lost_starting_statuses_reports_same_fault_again() {
  Bench c;
  StatusSpec f;
  report_thermistor_fault(c, f);
  c.restart();
  c.step(&f);
  CHECK(c.link.take_fault_event());
  CHECK_EQ(c.link.mcu_restarts(), 1);
}

static void test_restart_into_same_fault_reported_when_starting_clears() {
  Bench c;
  StatusSpec f;
  report_thermistor_fault(c, f);
  c.run(1100, &f);
  CHECK(!c.link.take_fault_event());
  c.restart();
  f.starting = true;
  c.run(80, &f);
  CHECK(!c.link.take_fault_event());
  f.starting = false;
  c.step(&f);
  CHECK(c.link.take_fault_event());
}

static void test_startup_fault_after_update_mode_not_reported() {
  Bench h;
  StatusSpec off;
  h.run(500, &off);
  h.u.push(boot_status_bytes(BOOT_REASON_REQUEST));
  h.step(nullptr);
  h.restart();
  StatusSpec s;
  s.state = ST_FAULT;
  s.faults = F_THERM;
  s.starting = true;
  h.run(80, &s);
  h.run(50, &off);
  CHECK(!h.link.take_fault_event());
}

static void test_restart_counted_on_boot_id_change_only() {
  Bench b;
  StatusSpec s;
  settle(b, s);
  b.run(300, &s);
  b.run(50, nullptr);
  b.run(30, &s);
  b.u.push(info_bytes(1));
  b.run(30, &s);
  CHECK_EQ(b.link.mcu_restarts(), 0);
  b.restart();
  b.run(30, &s);
  CHECK_EQ(b.link.mcu_restarts(), 1);
  CHECK(of_type(b.sent, MSG_INFO_REQ).empty());
}

static void test_restart_counted_when_whole_start_was_lost() {
  Bench b;
  StatusSpec s;
  settle(b, s);
  b.boot_id += 3;
  b.info_sent = true;
  b.run(30, &s);
  CHECK_EQ(b.link.mcu_restarts(), 1);
  CHECK(of_type(b.sent, MSG_INFO_REQ).empty());
}

static void test_mcu_watchdog_restart_restores_outputs_without_fault_event() {
  Bench b;
  StatusSpec s;
  s.state = ST_FIRING;
  s.relay = S_RELAY;
  b.echo = true;
  b.link.set_level(0.6f);
  b.link.set_leds(OUTPUTS_LED3, true);
  b.link.set_night_light(0.4f);
  settle(b, s);
  CHECK(b.leds == OUTPUTS_LED3 && b.night == 102);
  b.run(500, &s);
  CHECK(b.sent.empty());
  uint16_t target = b.echo_target;
  b.restart();
  b.echo = false;
  StatusSpec start;
  start.state = ST_FAULT;
  start.faults = F_THERM | F_ZO | F_CLAMPED;
  start.starting = true;
  b.run(80, &start);
  CHECK(!b.link.take_fault_event());
  CHECK(b.leds == OUTPUTS_LED3 && b.night == 102);  // restored at the first status that showed them off
  StatusSpec off;
  b.run(50, &off);
  auto g = b.take(MSG_GATE);
  CHECK(!g.empty() && g16(g.back().payload, 0) > target);  // the dim end first, as at any turn-on
  CHECK_EQ(b.link.mcu_restarts(), 1);
}

static void test_mcu_reset_mid_status_reads_info_frame_behind() {
  Bench b;
  StatusSpec s;
  settle(b, s);
  b.link.set_level(0.6f);
  b.echo = true;
  s.state = ST_FIRING;
  b.run(500, &s);
  b.echo = false;
  // The part frame takes the bytes of the new start's info frame, which is still read.
  auto part = b.status(s);
  part.resize(11);
  b.u.push(part);
  b.restart();
  b.build = 2;
  StatusSpec start;
  start.state = ST_FAULT;
  start.faults = F_THERM | F_ZO | F_CLAMPED;
  start.starting = true;
  b.sent.clear();
  b.run(80, &start);
  CHECK(!b.link.take_fault_event());
  CHECK_EQ(b.link.mcu_restarts(), 1);
  CHECK(b.link.have_current_info());
  CHECK_EQ(b.link.current_info().build, 2u);
  StatusSpec off;
  b.run(50, &off);
  auto g = b.take(MSG_GATE);
  CHECK(!g.empty() && g16(g.back().payload, 0) != 0);
}

// The info frame arrives just before the first status of the start whose boot ID it carries.
static void start_with_info(Bench &b, uint32_t build) {
  b.auto_info = false;
  b.link.set_level(0.5f);
  b.u.push(info_bytes(build, b.boot_id));
  StatusSpec s;
  b.step(&s);
}

static void test_info_before_first_status_counts_for_it() {
  Bench b;
  start_with_info(b, 7);
  CHECK(b.link.have_current_info());
  CHECK_EQ(b.link.current_info().build, 7u);
  CHECK_EQ(b.take(MSG_GATE).size(), 1);
}

static void test_restart_with_lost_info_not_driven_until_its_info() {
  Bench b;
  start_with_info(b, 7);
  b.sent.clear();
  b.restart();
  StatusSpec s;
  b.run(500, &s);
  CHECK(!b.link.have_current_info());
  CHECK(b.link.problem());
  CHECK(b.sent.empty());
  b.u.push(info_bytes(8, b.boot_id));
  b.step(&s);
  CHECK(b.link.have_current_info());
  CHECK_EQ(b.link.current_info().build, 8u);
  CHECK(!b.link.problem());
  CHECK_EQ(b.take(MSG_GATE).size(), 1);
}

static void test_late_info_of_earlier_start_not_used() {
  Bench b;
  start_with_info(b, 7);
  const uint8_t earlier_boot_id = b.boot_id;
  b.restart();
  start_with_info(b, 8);
  b.u.push(info_bytes(9, earlier_boot_id));
  StatusSpec s;
  b.step(&s);
  CHECK_EQ(b.link.current_info().build, 8u);
  CHECK(b.link.have_current_info());
}

// The cut frame claims more bytes than ever follow it: only the reset after RX_GAP_MS without a byte
// ends it.
static void test_rx_gap_reset() {
  Bench b;
  StatusSpec s;
  settle(b, s);
  uint8_t big[120] = {0};
  auto part = fw_frame(MSG_STATUS, big, sizeof big);
  part.resize(10);
  b.u.push(part);
  b.run(2 * detail::RX_GAP_MS, nullptr);
  b.u.push(info_bytes(5));
  b.step(nullptr);
  CHECK_EQ(b.link.current_info().build, 5u);
  s.therm = 900;
  b.step(&s);
  CHECK_EQ(b.link.status().therm, 900);
}

static void test_shorter_rx_pause_keeps_arriving_frame() {
  Bench b;
  StatusSpec s;
  settle(b, s);
  s.therm = 901;
  auto st = status_bytes(s);
  b.u.push(std::vector<uint8_t>(st.begin(), st.begin() + 12));
  b.run(20, nullptr);
  b.u.push(std::vector<uint8_t>(st.begin() + 12, st.end()));
  b.step(nullptr);
  CHECK_EQ(b.link.status().therm, 901);
}

static void test_turn_on_during_fault_raises_no_event_and_waits_for_clear() {
  Bench b;
  StatusSpec s;
  b.echo = true;
  settle(b, s);
  s.state = ST_FAULT;
  s.faults = F_OVERTEMP;
  b.run(50, &s);
  CHECK(b.link.take_fault_event());
  b.link.set_level(0.0f);
  b.run(50, &s);
  b.link.set_level(0.5f);
  b.run(2000, &s);
  CHECK(!b.link.take_fault_event());
  CHECK(b.link.fault_holds_off());
  CHECK(b.echo_target == 0);
  s.state = ST_OFF;
  s.faults = 0;
  b.run(50, &s);
  CHECK(!b.link.fault_holds_off());
  CHECK(!b.link.take_fault_event());
  CHECK(b.echo_target != 0);
}

static void test_startup_fault_does_not_hold_off() {
  Bench c;
  StatusSpec t;
  t.state = ST_FAULT;
  t.faults = F_THERM;
  t.starting = true;
  c.run(50, &t);
  CHECK(!c.link.fault_holds_off());
  c.link.set_level(0.5f);
  c.run(50, &t);
  CHECK(!c.link.take_fault_event());
}

static void test_silent_mcu_slow_resend() {
  Bench b;
  StatusSpec s;
  settle(b, s);
  b.run(200, nullptr);
  b.sent.clear();
  b.link.set_level(0.5f);
  b.run(2000, nullptr);
  auto n = b.take(MSG_GATE).size();
  CHECK(n >= 2 && n <= 3);
  b.step(&s);
  b.run(40, &s);
  CHECK(b.take(MSG_GATE).size() >= 2);
}

static void test_outputs_sent_once_until_status_shows_them() {
  Bench b;
  StatusSpec s;
  settle(b, s);
  b.run(500, &s);
  CHECK(b.take(MSG_OUTPUTS).empty());  // all off, as the MCU starts
  b.link.set_leds(OUTPUTS_LED1, true);
  b.link.set_leds(OUTPUTS_B3, true);
  b.link.set_night_light(0.5f);
  b.step(&s);
  auto out = b.take(MSG_OUTPUTS);
  CHECK(out.size() == 1 && out[0].payload[0] == (OUTPUTS_LED1 | OUTPUTS_B3) && out[0].payload[1] == 128);
  b.run(2000, &s);
  CHECK(b.take(MSG_OUTPUTS).empty());
  b.link.set_leds(OUTPUTS_LED3, true);
  b.link.set_leds(OUTPUTS_LED3, false);
  b.run(100, &s);
  CHECK(b.take(MSG_OUTPUTS).empty());
}

static void test_lost_outputs_resent_every_30ms_until_shown() {
  Bench b;
  StatusSpec s;
  settle(b, s);
  b.drop_outputs = 2;
  b.link.set_leds(OUTPUTS_LED4, true);
  b.link.set_night_light(0.5f);
  const uint8_t leds = OUTPUTS_LED4;
  std::vector<uint32_t> times;
  uint32_t t0 = b.t;
  for (int i = 0; i < 20; i++) {
    uint32_t t = b.t;
    b.step(&s);
    for (auto &f : of_type(b.sent, MSG_OUTPUTS)) {
      times.push_back(t - t0);
      CHECK(f.payload[0] == leds && f.payload[1] == 128);
    }
    CHECK(of_type(b.sent, MSG_INFO_REQ).empty());
    b.sent.clear();
  }
  CHECK(times.size() == 3 && times[0] == 0 && times[1] == 30 && times[2] == 60);
  CHECK_EQ(b.leds, leds);
}

static void test_link_state_and_reports() {
  Bench b;
  StatusSpec s;
  s.half = 7680;
  s.therm = 933;
  b.run(50, &s);
  CHECK_NEAR(b.link.mains_hz(), 60.0, 0.01);
  CHECK_NEAR(b.link.therm_resistance_ohms(), 10000.0 * 933 / 90, 0.5);
  CHECK(strcmp(b.link.state_text(), "Off") == 0);
  CHECK(!b.link.problem());
  b.run(detail::APP_SEEN_MS, nullptr);
  CHECK_EQ((int) b.link.link_state(), (int) LinkState::NO_RESPONSE);
  CHECK(strcmp(b.link.state_text(), "No Response") == 0);
  b.u.push(boot_status_bytes(BOOT_REASON_WATCHDOG));
  b.step(nullptr);
  CHECK_EQ((int) b.link.link_state(), (int) LinkState::BOOTLOADER);
  CHECK(strcmp(b.link.state_text(), "Update mode (use the MCU tool)") == 0);
  CHECK(b.link.problem());
  b.run(1400, nullptr);
  CHECK_EQ((int) b.link.link_state(), (int) LinkState::BOOTLOADER);
  b.run(200, nullptr);
  CHECK_EQ((int) b.link.link_state(), (int) LinkState::NO_RESPONSE);
  s.half = 0;
  b.step(&s);
  CHECK(std::isnan(b.link.mains_hz()));
  // A shorted thermistor reads near 0: no resistance (the NTC would show about -273 C).
  s.therm = 0;
  b.step(&s);
  CHECK(std::isnan(b.link.therm_resistance_ohms()));
  s.therm = THERM_OPEN_SHORT_LOW - 1;
  b.step(&s);
  CHECK(std::isnan(b.link.therm_resistance_ohms()));
  s.therm = THERM_OPEN_SHORT_LOW;
  b.step(&s);
  CHECK(!std::isnan(b.link.therm_resistance_ohms()));
  // An open thermistor reads near the top, where the MCU's safety check also calls it open.
  s.therm = THERM_OPEN_SHORT_HIGH;
  b.step(&s);
  CHECK(!std::isnan(b.link.therm_resistance_ohms()));
  s.therm = THERM_OPEN_SHORT_HIGH + 1;
  b.step(&s);
  CHECK(std::isnan(b.link.therm_resistance_ohms()));
}

static void test_one_app_frame_per_loop_while_level_and_leds_change() {
  Bench b;
  StatusSpec s;
  s.state = ST_FIRING;
  b.echo = true;
  settle(b, s);
  int outputs = 0;
  for (int i = 0; i < 100; i++) {
    b.link.set_level(0.2f + 0.007f * i);
    b.link.set_leds(BAR_LED_BITS[i % BAR_LEDS], (i / 5) % 2 == 0);
    b.step(&s);
    CHECK(b.sent.size() <= 1);
    outputs += of_type(b.sent, MSG_OUTPUTS).size();
    b.sent.clear();
  }
  CHECK(outputs >= 15);
  b.run(300, &s);
  b.sent.clear();
  b.run(4000, &s);
  CHECK(b.sent.empty());
}

static void test_nan_settings() {
  Bench b;
  StatusSpec s;
  s.state = ST_FIRING;
  b.echo = true;
  settle(b, s);
  b.link.set_level(0.5f);
  b.step(&s);
  auto g = b.take(MSG_GATE);
  CHECK_EQ(g.size(), 1);
  uint16_t t = g.empty() ? 0 : g16(g[0].payload, 0);
  b.link.set_dim_end_fraction(NAN);
  b.link.set_bright_end_fraction(NAN);
  b.run(50, &s);
  CHECK(b.take(MSG_GATE).empty());
  b.link.set_level(0.5f);
  b.step(&s);
  CHECK(b.take(MSG_GATE).empty());
  CHECK_EQ(b.echo_target, t);
  b.link.set_level(NAN);
  b.step(&s);
  g = b.take(MSG_GATE);
  CHECK(g.size() == 1 && g16(g[0].payload, 0) == 0);
  b.link.set_level(INFINITY);
  b.step(&s);
  g = b.take(MSG_GATE);
  CHECK(g.size() == 1 && g16(g[0].payload, 0) > 0 && g16(g[0].payload, 0) < t);
  b.link.set_dim_end_fraction(INFINITY);  // clamped to 1, then by the latest firing point
  b.link.set_night_light(NAN);
  b.link.set_night_light(-INFINITY);
  b.run(50, &s);
}

static void test_curve() {
  CHECK_NEAR(power_share(0, 7680), 1.0, 1e-6);
  CHECK_NEAR(power_share(7680, 7680), 0.0, 1e-6);
  CHECK_NEAR(power_share(3840, 7680), 0.5, 1e-6);
  CHECK_NEAR(power_share(4240, 7680), 0.397, 0.001);
  CHECK_NEAR(power_share(TEST_BRIGHT_END * 4240, 7680), 0.899, 0.001);
  for (float p = 0.02f; p < 1.0f; p += 0.07f) {
    float d = delay_for_power(p, 7680);
    CHECK_NEAR(power_share(d, 7680), p, 1e-4);
  }
  for (float d = 300; d < 7400; d += 333)
    CHECK_NEAR(delay_for_power(power_share(d, 7680), 7680), d, 0.5);
}

static void settle_firing_with_fixed_dim_end(Bench &b, StatusSpec &s) {
  b.echo = true;
  b.link.set_dim_end_from_wheel(false);
  s.state = ST_FIRING;
  settle(b, s);
}

static void test_curve_ends_through_link() {
  Bench b;
  // K = 4.6 ms (4239 counts) as the dim end.
  b.link.set_dim_end_fraction(4.6e-3f * PCA_HZ / 7680.0f);
  StatusSpec s;
  settle_firing_with_fixed_dim_end(b, s);
  b.link.set_level(0.00001f);
  b.step(&s);
  auto g = b.take(MSG_GATE);
  CHECK(g.size() == 1 && abs((int) g16(g[0].payload, 0) - 4239) <= 1);
  CHECK_NEAR(power_share(g16(g[0].payload, 0), 7680), 0.397, 0.001);
  b.link.set_level(1.0f);
  b.step(&s);
  g = b.take(MSG_GATE);
  CHECK(g.size() == 1 && abs((int) g16(g[0].payload, 0) - (int) lroundf(TEST_BRIGHT_END * 4239.36f)) <= 1);
  CHECK_NEAR(power_share(g16(g[0].payload, 0), 7680), 0.899, 0.001);
  b.link.set_level(0.5f);
  b.step(&s);
  g = b.take(MSG_GATE);
  CHECK(g.size() == 1);
  CHECK_NEAR(power_share(g16(g[0].payload, 0), 7680), (0.397 + 0.899) / 2, 0.002);
}

static void test_curve_ends_clamped_to_firing_window() {
  Bench b;
  StatusSpec s;
  settle_firing_with_fixed_dim_end(b, s);
  b.link.set_dim_end_fraction(0.95f);
  b.link.set_bright_end_fraction(0.0f);
  b.link.set_level(0.00001f);
  b.step(&s);
  auto g = b.take(MSG_GATE);
  CHECK(g.size() == 1 && g16(g[0].payload, 0) == 7680 - LATEST_BEFORE_EDGE - detail::CLAMP_MARGIN);
  b.link.set_level(1.0f);
  b.step(&s);
  g = b.take(MSG_GATE);
  CHECK(g.size() == 1 && g16(g[0].payload, 0) == EARLIEST_FIRE);
}

static void test_small_half_period_change_not_recomputed() {
  Bench b;
  StatusSpec s;
  settle_firing_with_fixed_dim_end(b, s);
  b.link.set_level(0.5f);
  b.run(50, &s);
  b.take(MSG_GATE);
  s.half = 7682;
  b.run(50, &s);
  CHECK(b.take(MSG_GATE).empty());
}

static void test_wheel() {
  Bench b;
  b.echo = true;
  StatusSpec s;
  s.wheel = 100;
  s.state = ST_FIRING;
  settle(b, s);
  CHECK(std::isnan(b.link.wheel_percent()));  // uncalibrated
  b.link.set_level(0.00001f);
  b.step(&s);
  auto g = b.take(MSG_GATE);  // uncalibrated: the Dim End number
  const int number_dim_end = (int) std::lround(TEST_DIM_END * s.half);
  CHECK(g.size() == 1 && abs((int) g16(g[0].payload, 0) - number_dim_end) <= 1);
  b.link.set_wheel_calibration(100, 900);
  b.step(&s);
  g = b.take(MSG_GATE);
  CHECK(g.size() == 1 && abs((int) g16(g[0].payload, 0) - (int) std::lround(detail::K_A_MS * 1e-3f * PCA_HZ)) <= 1);
  CHECK_NEAR(b.link.wheel_percent(), 0.0, 1e-6);
  s.wheel = 103;  // inside the deadband
  b.run(50, &s);
  CHECK(b.take(MSG_GATE).empty());
  CHECK_NEAR(b.link.wheel_percent(), 0.0, 1e-6);
  s.wheel = 500;
  b.step(&s);
  CHECK_NEAR(b.link.wheel_percent(), 50.0, 1e-3);
  g = b.take(MSG_GATE);
  const float k_middle_ms = (detail::K_A_MS + detail::K_B_MS) / 2;
  CHECK(g.size() == 1 && abs((int) g16(g[0].payload, 0) - (int) std::lround(k_middle_ms * 1e-3f * PCA_HZ)) <= 1);
  s.wheel = 1000;  // beyond end B, clamped by the latest firing point
  b.step(&s);
  CHECK_NEAR(b.link.wheel_percent(), 100.0, 1e-6);
  g = b.take(MSG_GATE);
  CHECK(g.size() == 1 && g16(g[0].payload, 0) == 7680 - LATEST_BEFORE_EDGE - detail::CLAMP_MARGIN);
  b.link.set_dim_end_from_wheel(false);
  b.step(&s);
  g = b.take(MSG_GATE);
  CHECK(g.size() == 1 && abs((int) g16(g[0].payload, 0) - number_dim_end) <= 1);
  s.wheel = 300;
  b.step(&s);
  CHECK(b.take(MSG_GATE).empty());
  CHECK_NEAR(b.link.wheel_percent(), 25.0, 1e-3);
}

static std::vector<std::pair<int, bool>> events(Link &l) {
  std::vector<std::pair<int, bool>> out;
  Input in;
  bool st;
  while (l.pop_input_event(&in, &st))
    out.push_back({in, st});
  return out;
}

static uint8_t buttons_pressed(uint8_t pressed) { return BTN_ALL & ~pressed; }
static uint8_t changed_in_window(uint8_t buttons) { return buttons << BTN_CHANGED_SHIFT; }

static void test_buttons() {
  Bench b;
  StatusSpec s;
  s.buttons = BTN_ALL;
  b.step(&s);
  auto e = events(b.link);
  CHECK(e.size() == 5 && e[0] == std::make_pair((int) IN_ONOFF, true) && e[1] == std::make_pair((int) IN_UP, true) &&
        e[2] == std::make_pair((int) IN_DOWN, true) && e[3] == std::make_pair((int) IN_ADDON_UP, false) &&
        e[4] == std::make_pair((int) IN_ADDON_DOWN, false));
  s.buttons = buttons_pressed(BTN_ONOFF) | changed_in_window(BTN_ONOFF);
  b.step(&s);
  CHECK(events(b.link).empty());
  s.buttons = buttons_pressed(BTN_ONOFF);
  b.step(&s);
  e = events(b.link);
  CHECK(e.size() == 1 && e[0].first == IN_ONOFF && !e[0].second);
  b.run(30, &s);
  CHECK(events(b.link).empty());
  s.buttons = buttons_pressed(BTN_DOWN);
  b.step(&s);
  e = events(b.link);
  CHECK(e.size() == 2 && e[0] == std::make_pair((int) IN_ONOFF, true) && e[1] == std::make_pair((int) IN_DOWN, false));
}

static void test_button_backlog_every_frame_counts() {
  Bench b;
  StatusSpec s;
  s.buttons = buttons_pressed(BTN_DOWN);
  b.step(&s);
  events(b.link);
  s.buttons = buttons_pressed(BTN_UP | BTN_DOWN);
  auto f1 = status_bytes(s);
  s.buttons = buttons_pressed(BTN_DOWN);
  auto f2 = status_bytes(s);
  b.u.push(f1);
  b.u.push(f2);
  b.step(nullptr);
  auto e = events(b.link);
  CHECK(e.size() == 2 && e[0] == std::make_pair((int) IN_UP, false) && e[1] == std::make_pair((int) IN_UP, true));
}

static void test_button_low_at_first_frame_is_starting_level() {
  Bench c;
  StatusSpec s;
  s.buttons = buttons_pressed(BTN_UP) | changed_in_window(BTN_UP);
  c.step(&s);
  auto e = events(c.link);
  CHECK(e.size() == 5 && e[0] == std::make_pair((int) IN_ONOFF, true) && e[1] == std::make_pair((int) IN_UP, false) &&
        e[2] == std::make_pair((int) IN_DOWN, true));
  c.run(30, &s);
  CHECK(events(c.link).empty());
}

static void test_addon() {
  Bench b;
  StatusSpec s;
  auto ks = [&](uint16_t mn) { s.ks_min = mn; };
  ks(556);
  b.run(30, &s);
  auto e = events(b.link);  // starting states: buttons, then add-on idle
  CHECK(e.size() == 5 && e[3] == std::make_pair((int) IN_ADDON_UP, false) &&
        e[4] == std::make_pair((int) IN_ADDON_DOWN, false));
  ks(678);
  b.step(&s);
  CHECK(events(b.link).empty());
  b.step(&s);
  e = events(b.link);
  CHECK(e.size() == 1 && e[0] == std::make_pair((int) IN_ADDON_UP, true));
  ks(KS_NO_SAMPLE);
  b.step(&s);
  CHECK(events(b.link).empty());
  ks(838);
  b.step(&s);
  CHECK(events(b.link).empty());
  b.step(&s);
  e = events(b.link);
  CHECK(e.size() == 2 && e[0] == std::make_pair((int) IN_ADDON_UP, false) &&
        e[1] == std::make_pair((int) IN_ADDON_DOWN, true));
  ks(556);
  b.run(20, &s);
  e = events(b.link);
  CHECK(e.size() == 1 && e[0] == std::make_pair((int) IN_ADDON_DOWN, false));
  ks(678);
  b.step(&s);
  ks(556);
  b.run(30, &s);
  CHECK(events(b.link).empty());
  // A relay drive change blanks that window and the next three.
  s.relay = S_RELAY | S_KS_STALE;
  ks(838);
  b.step(&s);
  s.relay = S_RELAY;
  for (int i = 0; i < 3; i++)
    b.step(&s);
  CHECK(events(b.link).empty());
  b.run(20, &s);
  e = events(b.link);
  CHECK(e.size() == 1 && e[0] == std::make_pair((int) IN_ADDON_DOWN, true));
  // The drive changes after the window's last sample: S_RELAY changes without S_KS_STALE. The samples
  // (drive-off idle) are not judged with the drive-on thresholds.
  apply_test_settings(b.link);
  s.relay = 0;
  ks(556);
  b.run(60, &s);
  e = events(b.link);
  CHECK(e.size() == 1 && e[0] == std::make_pair((int) IN_ADDON_DOWN, false));
  s.relay = S_RELAY;
  b.step(&s);
  CHECK(events(b.link).empty());
  ks(62);
  for (int i = 0; i < 5; i++)
    b.step(&s);
  CHECK(events(b.link).empty());
  ks(838);
  b.run(20, &s);
  e = events(b.link);
  CHECK(e.size() == 1 && e[0] == std::make_pair((int) IN_ADDON_DOWN, true));
  b.link.set_addon_thresholds(ADDON_UP_OFF, ADDON_DOWN_OFF, 700, 900);
  b.run(20, &s);
  e = events(b.link);
  CHECK(e.size() == 2 && e[0] == std::make_pair((int) IN_ADDON_DOWN, false) &&
        e[1] == std::make_pair((int) IN_ADDON_UP, true));
}

static void test_addon_held_at_first_frame_reported_held() {
  Bench a;
  StatusSpec s;
  s.ks_min = 838;
  a.step(&s);
  auto e = events(a.link);
  CHECK(e.size() == 5 && e[3] == std::make_pair((int) IN_ADDON_UP, false) &&
        e[4] == std::make_pair((int) IN_ADDON_DOWN, true));
  a.step(&s);
  CHECK(events(a.link).empty());
}

static void test_addon_blanked_first_frame_reported_idle_then_classified() {
  Bench b;
  StatusSpec s;
  s.ks_min = 838;
  s.relay = S_RELAY | S_KS_STALE;
  b.step(&s);
  auto e = events(b.link);
  CHECK(e.size() == 5 && !e[3].second && !e[4].second);
  s.relay = S_RELAY;
  b.run(30, &s);
  CHECK(events(b.link).empty());
  b.run(20, &s);
  e = events(b.link);
  CHECK(e.size() == 1 && e[0] == std::make_pair((int) IN_ADDON_DOWN, true));
}

static void test_addon_first_frame_without_samples_idle() {
  Bench c;
  StatusSpec n;
  c.step(&n);
  auto e = events(c.link);
  CHECK(e.size() == 5 && !e[3].second && !e[4].second);
}

// The YAML's numbers, drive on.
static void test_addon_default_thresholds() {
  Bench d;
  StatusSpec u;
  u.relay = S_RELAY;
  u.ks_min = ADDON_UP_ON;
  d.step(&u);
  auto e = events(d.link);
  CHECK(e.size() == 5 && e[3].second && !e[4].second);
  Bench f;
  u.ks_min = ADDON_DOWN_ON;
  f.step(&u);
  e = events(f.link);
  CHECK(e.size() == 5 && !e[3].second && e[4].second);
}

static void test_event_queue_drops_oldest() {
  Bench b;
  StatusSpec s;
  s.buttons = BTN_ALL;
  b.step(&s);
  CHECK_EQ(events(b.link).size(), 5);
  auto onoff_released = [](int event) { return event % 2 == 1; };
  const int pushed = 41;
  for (int i = 0; i < pushed; i++) {
    s.buttons = onoff_released(i) ? BTN_ALL : buttons_pressed(BTN_ONOFF);
    b.u.push(status_bytes(s));
  }
  b.step(nullptr);
  auto e = events(b.link);
  CHECK_EQ(e.size(), Link::EVENT_QUEUE);
  const int first_kept = pushed - (int) e.size();
  CHECK(!e.empty() && e.front().second == onoff_released(first_kept) && e.back().second == onoff_released(pushed - 1));
}

static std::vector<uint8_t> info_bytes_v(uint32_t build, uint8_t boot_version) {
  uint8_t p[INFO_LEN];
  info_build(p, FW_VERSION, build, 1, 0, boot_version, 1);
  return fw_frame(MSG_INFO, p, INFO_LEN);
}

static void run_build(Bench &b, StatusSpec &s, uint32_t build) {
  b.build = build;
  b.run(30, &s);
  b.u.push(info_bytes(build));
  b.run(100, &s);
}

static void test_expected_build() {
  Bench b;
  StatusSpec s;
  run_build(b, s, 0x1234);
  CHECK(b.link.have_current_info());
  CHECK(!b.link.update_available());  // nothing expected yet
  b.link.set_expected(FW_VERSION, 0x1234, BOOT_VERSION);
  CHECK(!b.link.update_available());
  char buf[96];
  b.link.state_text_with_hint(buf, sizeof buf);
  CHECK(strcmp(buf, "Off") == 0);
  b.link.set_expected(FW_VERSION, 0x4321, BOOT_VERSION);
  CHECK(b.link.update_available());
  b.link.state_text_with_hint(buf, sizeof buf);
  CHECK(strcmp(buf, "Off (update available: use the MCU tool)") == 0);
  CHECK(strcmp(b.link.state_text(), "Off") == 0);
  CHECK(!b.link.problem());  // the lamp still works
}

static void test_older_bootloader_is_update() {
  Bench b;
  StatusSpec s;
  run_build(b, s, 0x1234);
  b.link.set_expected(FW_VERSION, 0x1234, BOOT_VERSION + 1);
  CHECK(b.link.update_available());
}

static void test_newer_bootloader_is_not_update() {
  Bench b;
  StatusSpec s;
  run_build(b, s, 0x1234);
  b.link.set_expected(FW_VERSION, 0x1234, BOOT_VERSION);
  b.u.push(info_bytes_v(0x1234, BOOT_VERSION + 1));
  b.run(20, &s);
  CHECK(!b.link.update_available() && b.link.firmware_newer());
}

// Another build of a newer release: the MCU tool leaves it as it is, so no update is shown.
static void test_newer_release_is_not_update() {
  Bench b;
  StatusSpec s;
  b.version = static_cast<uint16_t>(FW_VERSION + 1);
  run_build(b, s, 0x4321);
  b.u.push(info_bytes(0x4321, b.boot_id, b.version));
  b.run(20, &s);
  b.link.set_expected(FW_VERSION, 0x1234, BOOT_VERSION);
  CHECK(!b.link.update_available() && b.link.firmware_newer());
  char buf[96];
  b.link.state_text_with_hint(buf, sizeof buf);
  CHECK(strcmp(buf, "Off (MCU firmware newer than this configuration)") == 0);
  b.link.set_expected(static_cast<uint16_t>(FW_VERSION + 1), 0x1234, BOOT_VERSION);
  CHECK(b.link.update_available() && !b.link.firmware_newer());
}

struct Sink {
  std::vector<std::pair<int, bool>> inputs;
  std::vector<std::string> calls;
  void input(Input input, bool state) {
    this->inputs.push_back({input, state});
    this->calls.push_back("input");
  }
  void fault_off() { this->calls.push_back("fault_off"); }
  void adopt_on(float level) { this->calls.push_back("adopt_on " + std::to_string(lroundf(level * 100))); }
  void adopt_off() { this->calls.push_back("adopt_off"); }
};

// What the main configuration does each loop: the input events in order, then a FAULT that starts,
// then the first status's adopt, once.
static void test_dispatch() {
  Bench b;
  b.auto_adopt = false;
  StatusSpec s;
  s.state = ST_FAULT;
  s.therm = 0;
  b.run(10, &s);
  Sink sink;
  b.link.dispatch(sink);
  const std::vector<std::string> first = {"input", "input", "input", "input", "input", "fault_off"};
  CHECK_EQ(sink.inputs.size(), IN_COUNT);
  CHECK(sink.inputs[0] == std::make_pair((int) IN_ONOFF, true));
  CHECK(sink.calls == first);
  sink.calls.clear();
  b.link.dispatch(sink);
  CHECK(sink.calls.empty());

  Bench on;
  on.auto_adopt = false;
  on.link.set_soft_reboot(true);
  StatusSpec firing;
  firing.state = ST_FIRING;
  firing.target = 3000;
  on.run(10, &firing);
  Sink s2;
  on.link.dispatch(s2);
  CHECK(!s2.calls.empty() && s2.calls.back().rfind("adopt_on ", 0) == 0);
}

static void test_brightness_bar_leds() {
  CHECK_EQ(brightness_bar_leds(false, 1.0f), 0);
  CHECK_EQ(brightness_bar_leds(true, 0.0f), 1);
  CHECK_EQ(brightness_bar_leds(true, 0.01f), 1);
  CHECK_EQ(brightness_bar_leds(true, 0.2f), 1);
  CHECK_EQ(brightness_bar_leds(true, 0.21f), 2);
  CHECK_EQ(brightness_bar_leds(true, 0.6f), 3);
  CHECK_EQ(brightness_bar_leds(true, 1.0f), BAR_LEDS);
}

// B3 blinks unless the application runs; then it shows a FAULT that keeps the lamp off.
static void test_b3() {
  Bench b;
  CHECK(b3_blinks(b.link));
  StatusSpec s;
  b.run(50, &s);
  CHECK(!b.link.b3_on(true) && !b.link.b3_on(false));
  s.state = ST_FAULT;
  b.run(50, &s);
  CHECK(b.link.b3_on(true) && b.link.b3_on(false));
  b.run(detail::APP_SEEN_MS, nullptr);
  CHECK(b3_blinks(b.link));
}

static void test_reset_cause_text() {
  Bench b;
  char buf[96];
  CHECK(!b.link.reset_cause_text(buf, sizeof buf));
  StatusSpec s;
  b.run(50, &s);
  CHECK(b.link.reset_cause_text(buf, sizeof buf));
  char want[96];
  snprintf(want, sizeof want, "Power-on, watchdog count 0, bootloader %u", BOOT_VERSION);
  CHECK(strcmp(buf, want) == 0);
}

static void test_state_text_cut_to_small_buffer() {
  Bench b;
  StatusSpec s;
  run_build(b, s, 0x1234);
  b.link.set_expected(FW_VERSION, 0x4321, BOOT_VERSION);
  char small[8];
  memset(small, 'x', sizeof small);
  b.link.state_text_with_hint(small, 5);
  CHECK(strcmp(small, "Off ") == 0 && small[5] == 'x');
}

// The bootloader drops application frames, so the main configuration sends it nothing at all.
static void test_silent_in_update_mode() {
  Bench b;
  StatusSpec s;
  s.state = ST_FIRING;
  b.echo = true;
  b.link.set_level(0.5f);
  b.link.set_leds(OUTPUTS_LED2, true);
  settle(b, s);
  b.run(1500, &s);
  b.sent.clear();
  for (int i = 0; i < 20; i++) {
    b.u.push(boot_status_bytes(BOOT_REASON_REQUEST));
    b.run(500, nullptr);
    b.link.set_level(0.2f + 0.03f * i);
    b.link.set_leds(BAR_LED_BITS[i % BAR_LEDS], i % 2 == 0);
    b.link.set_night_light(0.05f * i);
  }
  CHECK(b.sent.empty());
  CHECK(strcmp(b.link.state_text(), "Update mode (use the MCU tool)") == 0);
  CHECK(b.link.problem());
  CHECK(!b.link.take_fault_event());
  b.run(3000, nullptr);
  CHECK(b.sent.empty());
  CHECK_EQ((int) b.link.link_state(), (int) LinkState::NO_RESPONSE);
  b.restart();
  s.state = ST_OFF;
  b.echo = false;
  b.run(100, &s);
  CHECK(!of_type(b.sent, MSG_GATE).empty());
  CHECK(!of_type(b.sent, MSG_OUTPUTS).empty());
  CHECK(of_type(b.sent, MSG_INFO_REQ).empty());
  CHECK(!b.link.problem());
  CHECK_EQ(b.link.mcu_restarts(), 1);
}

static void test_other_protocol_not_driven_and_names_the_tool() {
  Bench c;
  c.version = static_cast<uint16_t>(FW_VERSION + 0x0100 + 2);
  c.link.set_level(0.5f);
  c.link.set_leds(OUTPUTS_LED3, true);
  c.link.set_expected(FW_VERSION, 0x1234, BOOT_VERSION);
  StatusSpec s;
  c.run(2000, &s);
  CHECK(c.sent.empty());
  CHECK(!c.link.protocol_supported());
  CHECK(c.link.problem());
  CHECK_EQ((int) c.link.link_state(), (int) LinkState::APPLICATION);
  CHECK(!c.link.update_available() && c.link.firmware_newer());
  char buf[96];
  c.link.state_text_with_hint(buf, sizeof buf);
  CHECK(strcmp(buf, "MCU firmware needs the MCU tool") == 0);
}

// A later release whose frames have other lengths: the lamp is not driven, and MCU State names the
// tool instead of showing that the MCU does not answer.
static void test_frames_of_other_length_name_the_tool() {
  for (bool driven_before : {false, true}) {
    Bench e;
    e.link.set_level(0.5f);
    e.link.set_leds(OUTPUTS_LED2, true);
    StatusSpec s;
    if (driven_before) {
      e.run(100, &s);
      CHECK(!e.sent.empty() && !e.link.problem());
    }
    e.sent.clear();
    uint8_t other_status[STATUS_LEN + 3] = {0}, other_info[INFO_LEN + 2] = {0};
    for (int i = 0; i < 300; i++) {
      e.u.push(fw_frame(MSG_STATUS, other_status, sizeof other_status));
      if (i % 100 == 0)
        e.u.push(fw_frame(MSG_INFO, other_info, sizeof other_info));
      e.link.set_level(0.2f + 0.002f * i);
      e.run(10, nullptr);
    }
    CHECK(e.sent.empty());
    CHECK(!e.link.protocol_supported());
    CHECK(e.link.problem());
    CHECK(strcmp(e.link.state_text(), "MCU firmware needs the MCU tool") == 0);
    e.run(3000, nullptr);
    CHECK(strcmp(e.link.state_text(), "No Response") == 0);
  }
}

static void test_any_release_of_this_protocol_driven() {
  Bench d;
  d.auto_info = false;
  d.link.set_level(0.5f);
  d.link.set_leds(OUTPUTS_LED3, true);
  d.u.push(info_bytes(0x1234, 1, static_cast<uint16_t>(FW_VERSION + 6)));
  StatusSpec s;
  d.run(100, &s);
  CHECK(d.link.protocol_supported());
  CHECK(!of_type(d.sent, MSG_GATE).empty());
  CHECK(!of_type(d.sent, MSG_OUTPUTS).empty());
  CHECK(!d.link.problem());
}

static void test_update_mode_lost_info_waits_for_next_without_asking() {
  Bench b;
  StatusSpec s;
  s.state = ST_FIRING;
  b.echo = true;
  b.link.set_level(0.5f);
  settle(b, s);
  b.run(500, &s);
  b.sent.clear();
  b.u.push(boot_status_bytes(BOOT_REASON_REQUEST));
  b.run(500, nullptr);
  b.restart();
  b.info_sent = true;  // its info frame at start is lost
  b.info_t = b.t;
  b.echo = false;
  s.state = ST_OFF;
  b.run(500, &s);
  CHECK(b.sent.empty());
  CHECK(b.link.problem());
  b.run(600, &s);
  CHECK(of_type(b.sent, MSG_INFO_REQ).empty());
  CHECK(!b.take(MSG_GATE).empty());
  CHECK(!b.link.problem());
}

// The main configuration's link on the simulated MCU, with the YAML's take_adopt().
struct LinkSim : SimMcu {
  Link link;
  LinkSim() {
    apply_test_settings(this->link);
    this->esp_loop = [this](uint32_t ms) {
      this->link.loop(this->uart, ms);
      float lvl;
      this->link.take_adopt(&lvl);
    };
  }
};

static int frames_from_esp(const LinkSim &w) {
  int n = 0;
  for (int c : w.esp_frames)
    n += c;
  return n;
}

static void test_update_mode_returns_to_application_without_fault_event() {
  install(image_build_of(app_image.data()));
  LinkSim *w = new LinkSim();
  w->us = 1000000;
  w->power_on();
  w->link.set_level(0.5f);
  int events = 0;
  auto run = [&](uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += 10) {
      w->run(10);
      if (w->link.take_fault_event())
        events++;
    }
  };
  run(1000);
  CHECK_EQ(w->gate.state, ST_FIRING);
  uint32_t restarts = w->link.mcu_restarts();
  w->boot_start(BOOT_REASON_REQUEST);
  uint64_t entered = w->us;
  int frames = frames_from_esp(*w);
  run(2000);
  CHECK(w->mode == SimMcu::BOOT);
  char buf[96];
  w->link.state_text_with_hint(buf, sizeof buf);
  CHECK(strcmp(buf, "Update mode (use the MCU tool)") == 0);
  CHECK(w->link.problem());
  for (int i = 0; i < 7000 && w->mode == SimMcu::BOOT; i++)
    run(10);
  CHECK(w->mode == SimMcu::APP);
  const uint64_t idle_return_us = IDLE_RETURN_MS * 1000ull;
  CHECK(w->us - entered >= idle_return_us && w->us - entered < idle_return_us + 1000000ull);
  CHECK_EQ(frames_from_esp(*w), frames);  // nothing at all while in update mode
  CHECK_EQ(w->boot_bad_frames, 0);
  run(1500);
  CHECK_EQ(w->gate.state, ST_FIRING);
  CHECK(w->gate.target != 0);
  CHECK_EQ(events, 0);
  CHECK(!w->link.problem());
  CHECK_EQ(w->link.mcu_restarts(), restarts + 1);  // the application's start after update mode
  delete w;
}

static void test_sim_watchdog_reset_restores_lamp_and_outputs() {
  install(image_build_of(app_image.data()));
  LinkSim *w = new LinkSim();
  w->us = 1000000;
  w->power_on();
  w->link.set_level(0.5f);
  w->link.set_leds(OUTPUTS_LED1, true);
  const uint8_t leds = OUTPUTS_LED1;
  w->link.set_night_light(1.0f);
  int events = 0;
  auto run = [&](uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += 10) {
      w->run(10);
      if (w->link.take_fault_event())
        events++;
    }
  };
  run(1500);
  CHECK_EQ(w->gate.state, ST_FIRING);
  CHECK(w->leds == leds && w->night == 255);
  int frames = frames_from_esp(*w);
  run(2000);
  CHECK_EQ(frames_from_esp(*w), frames);  // nothing while everything matches
  w->watchdog_reset();
  CHECK(w->leds == 0 && w->gate.state != ST_FIRING);
  run(1000);
  CHECK_EQ(w->gate.state, ST_FIRING);
  CHECK(w->leds == leds && w->night == 255);
  CHECK_EQ(events, 0);
  CHECK_EQ(w->link.mcu_restarts(), 1);
  CHECK(!w->link.problem());
  delete w;
}

static void test_sim_restart_target_in_flight() {
  // The MCU restarts while a non-zero target is on its way, and that target arrives at the end of
  // its start-up, after the 0 the link sends when it sees the start-up FAULT. The start-up FAULT
  // still ends when the thermistor average is ready: no fault event, and the lamp comes back.
  install(image_build_of(app_image.data()));
  LinkSim *w = new LinkSim();
  w->us = 1000000;
  w->power_on();
  w->link.set_level(0.5f);
  int events = 0;
  auto run = [&](uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += 10) {
      w->run(10);
      if (w->link.take_fault_event())
        events++;
    }
  };
  run(1500);
  CHECK_EQ(w->gate.state, ST_FIRING);
  uint16_t target = w->gate.target;
  w->watchdog_reset();
  run(60);
  CHECK_EQ(w->gate.state, ST_FAULT);
  CHECK_EQ(w->gate.target, 0);  // the link's 0
  // The late copy of the old target, 2 ms before the average is ready: the link would send 0 again
  // (the status shows a target it did not send), but only 30 ms later.
  w->run(15);
  CHECK(w->gate.startup);
  gate_set_target(&w->gate, target, TEST_FADE_LIMIT, TEST_FADE_LIMIT);
  run(1000);
  CHECK_EQ(events, 0);
  CHECK_EQ(w->gate.state, ST_FIRING);
  delete w;
}

int main() {
  RUN(test_crc_and_build);
  RUN(test_parser);
  RUN(test_parser_resync_after_noise_in_frame);
  RUN(test_parser_resync_after_cut_frame_with_false_headers);
  RUN(test_parser_reads_frame_inside_cut_frame);
  RUN(test_parser_rejoins_repeated_status_ending_in_false_header);
  RUN(test_parser_desync_sweep);
  RUN(test_false_start_of_longest_frame_keeps_application_seen);
  RUN(test_decoders);
  RUN(test_no_gate_before_status);
  RUN(test_not_driven_until_info_frame);
  RUN(test_gate_resent_every_30ms_until_acknowledged_then_never);
  RUN(test_clamped_bit_with_same_target_is_not_acknowledgement);
  RUN(test_mcu_reset_restarts_from_dim_end_until_firing);
  RUN(test_new_level_sent_at_once_newest_only);
  RUN(test_fade_limit_change_sent_at_once);
  RUN(test_ack_zo_not_good);
  RUN(test_adopt_on);
  RUN(test_adopt_off);
  RUN(test_adopt_off_for_fault_and_stopping);
  RUN(test_power_on_with_mcu_off_adopts_none_then_gates);
  RUN(test_adopt_none_without_soft_reboot_setting);
  RUN(test_power_on_with_mcu_firing_adopts_on);
  RUN(test_fault);
  RUN(test_startup_fault_not_reported_and_target_waits_for_off);
  RUN(test_fault_reported_when_starting_clears);
  RUN(test_restart_with_lost_starting_statuses_reports_same_fault_again);
  RUN(test_restart_into_same_fault_reported_when_starting_clears);
  RUN(test_startup_fault_after_update_mode_not_reported);
  RUN(test_restart_counted_on_boot_id_change_only);
  RUN(test_restart_counted_when_whole_start_was_lost);
  RUN(test_mcu_watchdog_restart_restores_outputs_without_fault_event);
  RUN(test_mcu_reset_mid_status_reads_info_frame_behind);
  RUN(test_info_before_first_status_counts_for_it);
  RUN(test_restart_with_lost_info_not_driven_until_its_info);
  RUN(test_late_info_of_earlier_start_not_used);
  RUN(test_rx_gap_reset);
  RUN(test_shorter_rx_pause_keeps_arriving_frame);
  RUN(test_turn_on_during_fault_raises_no_event_and_waits_for_clear);
  RUN(test_startup_fault_does_not_hold_off);
  RUN(test_silent_mcu_slow_resend);
  RUN(test_outputs_sent_once_until_status_shows_them);
  RUN(test_lost_outputs_resent_every_30ms_until_shown);
  RUN(test_link_state_and_reports);
  RUN(test_one_app_frame_per_loop_while_level_and_leds_change);
  RUN(test_nan_settings);
  RUN(test_curve);
  RUN(test_curve_ends_through_link);
  RUN(test_curve_ends_clamped_to_firing_window);
  RUN(test_small_half_period_change_not_recomputed);
  RUN(test_wheel);
  RUN(test_buttons);
  RUN(test_button_backlog_every_frame_counts);
  RUN(test_button_low_at_first_frame_is_starting_level);
  RUN(test_addon);
  RUN(test_addon_held_at_first_frame_reported_held);
  RUN(test_addon_blanked_first_frame_reported_idle_then_classified);
  RUN(test_addon_first_frame_without_samples_idle);
  RUN(test_addon_default_thresholds);
  RUN(test_event_queue_drops_oldest);
  RUN(test_expected_build);
  RUN(test_older_bootloader_is_update);
  RUN(test_newer_bootloader_is_not_update);
  RUN(test_newer_release_is_not_update);
  RUN(test_dispatch);
  RUN(test_brightness_bar_leds);
  RUN(test_b3);
  RUN(test_reset_cause_text);
  RUN(test_state_text_cut_to_small_buffer);
  RUN(test_silent_in_update_mode);
  RUN(test_other_protocol_not_driven_and_names_the_tool);
  RUN(test_frames_of_other_length_name_the_tool);
  RUN(test_any_release_of_this_protocol_driven);
  RUN(test_update_mode_lost_info_waits_for_next_without_asking);
  RUN(test_update_mode_returns_to_application_without_fault_event);
  RUN(test_sim_watchdog_reset_restores_lamp_and_outputs);
  RUN(test_sim_restart_target_in_flight);
  TEST_EXIT();
}
