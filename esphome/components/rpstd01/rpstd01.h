// RP-STD01: the main configuration's side of the link to our MCU firmware.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "esphome/components/rpstd01_link/rpstd01_link.h"

namespace rpstd01 {

// Share of full power delivered by a triac fired at `delay` counts into a half-cycle of `half`
// counts, resistive load: P = 1 - t/pi + sin(2t)/(2pi), t = pi * delay / half.
inline float power_share(float delay, float half);
// Inverse of power_share by bisection; returns a delay in counts.
inline float delay_for_power(float power, float half);
// Lamp power for a level 0..1 between the dim-end and bright-end powers, in even steps of perceived
// brightness: lightness goes as the cube root of light (CIE L*), taking light as proportional to power.
inline float power_for_level(float level, float p_dim, float p_bright);
// Inverse of power_for_level; 1 when the ends are equal.
inline float level_for_power(float power, float p_dim, float p_bright);

enum Input : uint8_t { IN_ONOFF = 0, IN_UP = 1, IN_DOWN = 2, IN_ADDON_UP = 3, IN_ADDON_DOWN = 4, IN_COUNT = 5 };

// The brightness bar's LEDs in the outputs frame, from the bottom.
inline constexpr uint8_t BAR_LED_BITS[] = {OUTPUTS_LED1, OUTPUTS_LED2, OUTPUTS_LED3, OUTPUTS_LED4, OUTPUTS_LED5};
inline constexpr uint8_t BAR_LEDS = sizeof(BAR_LED_BITS);

// LEDs of the brightness bar lit for a lamp at brightness 0..1: at least one while it is on.
inline uint8_t brightness_bar_leds(bool on, float brightness);

enum class Adopt : uint8_t { NONE, ON, OFF };

class Link : public McuLink {
 public:
  // Sends at most one application frame per loop, and nothing while the MCU is in update mode.
  template<typename U> void loop(U &uart, uint32_t now_ms);

  // After gamma; 0 or NaN is off. While the MCU is OFF, STOPPING or SETTLE the link sends the dim-end
  // delay instead, so the MCU fades the lamp up from it.
  void set_level(float level);
  void set_soft_reboot(bool soft_reboot);  // the ESP32's reset was not a power-on
  // Once, at the first status: SETTLE or FIRING gives ON at the level that maps back to the MCU's
  // target, even after a power-on; otherwise OFF after a soft reboot and NONE after a power-on, so
  // that restore_mode applies. No gate frame goes out before it has been taken.
  Adopt take_adopt(float *level);
  // The start-up FAULT, while "starting", is not one.
  bool take_fault_event();
  bool fault_holds_off() const;
  // Hands sink (S) every input event as input(Input, bool), then a FAULT that starts as fault_off(),
  // then the first status's adopt as adopt_on(level) or adopt_off().
  template<typename S> void dispatch(S &sink);
  // Shows a FAULT that holds the lamp off while the application runs, and blinks otherwise.
  bool b3_on(bool blink_on) const;

  void set_dim_end_from_wheel(bool from_wheel);
  void set_dim_end_fraction(float of_half);        // NaN ignored
  void set_bright_end_fraction(float of_dim_end);  // NaN ignored
  void set_fade_limit(uint8_t counts);             // 0: none
  // raw_a == raw_b: uncalibrated, so the wheel cannot set the dim end.
  void set_wheel_calibration(uint16_t raw_a, uint16_t raw_b);

  void set_leds(uint8_t bits, bool on);
  void set_night_light(float level);

  void set_addon_thresholds(uint16_t up_off, uint16_t down_off, uint16_t up_on, uint16_t down_on);
  // In arrival order, a backlog after a pause included. The first status gives each input's starting
  // state, later ones only changes. Buttons: the stable pin level (true = released, active low),
  // which a window with its changed bit set does not change. Add-on: true = held, once two judged
  // windows in a row show it. When the queue is full the oldest event is dropped.
  static constexpr uint8_t EVENT_QUEUE = 32;
  bool pop_input_event(Input *input, bool *state);
  // NAN if uncalibrated or before the first status. Moves only by WHEEL_DEADBAND counts or more.
  float wheel_percent() const;

  float mains_hz() const;
  // Statuses with S_ZO_COASTED, since boot.
  uint32_t zo_coasted_windows() const { return this->zo_coasted_windows_; }
  float therm_resistance_ohms() const;  // NAN if no status, shorted or open
  const char *state_text() const;
  // With a hint while the application runs a supported protocol and is not the expected firmware.
  void state_text_with_hint(char *buf, size_t size) const;
  void faults_text(char *buf, size_t size) const;       // F_CLAMPED is not shown
  bool reset_cause_text(char *buf, size_t size) const;  // false before the current start's info frame
  // A reported FAULT, not the application (update mode included), no info frame yet, or an
  // unsupported protocol.
  bool problem() const;
  // False for another link protocol (the version's high byte), or while frames come that this
  // release cannot read.
  bool protocol_supported() const;

  void set_expected(uint16_t version, uint32_t build, uint8_t boot_version);
  // Another build, or an older bootloader, than expected. False until the info frame and the
  // expected build are known.
  bool update_available() const;
  // A newer application or bootloader than expected: the MCU tool leaves it as it is.
  bool firmware_newer() const;

 protected:
  struct Event {
    uint8_t input;
    bool state;
  };
  enum TxSlot : uint8_t { TX_GATE, TX_OUTPUTS };

  void handle_status_(const Status &s);
  void process_buttons_(const Status &s);
  void process_addon_(const Status &s);
  void push_event_(Input input, bool state);
  static bool reported_fault_(const Status &s) { return s.state == ST_FAULT && !s.starting(); }
  size_t prepare_tx_(uint8_t slot, uint8_t *frame);
  bool gate_allowed_() const;
  bool firmware_age_(FirmwareAge *age) const;
  uint16_t wanted_target_();
  void recompute_curve_();
  uint16_t clamp_target_(float delay) const;
  bool wheel_calibrated_() const { return this->wheel_raw_a_ != this->wheel_raw_b_; }
  float curve_half_now_() const;
  void curve_ends_(float half, float *p_dim, float *p_bright) const;

  bool fault_event_{false};
  // The last status showed a reported FAULT, and the MCU restarts seen up to it.
  bool status_fault_{false};
  uint32_t status_restarts_{0};
  uint32_t zo_coasted_windows_{0};
  bool addon_init_{false};
  uint8_t last_app_slot_{NO_SLOT};
  uint16_t dim_target_{0};

  // No gate or outputs frame goes out until both fractions are set; no add-on input is held until
  // the thresholds are.
  float dim_fraction_{NAN}, bright_fraction_{NAN};
  uint8_t fade_limit_{0};
  uint16_t addon_up_off_{UINT16_MAX}, addon_down_off_{UINT16_MAX}, addon_up_on_{UINT16_MAX},
      addon_down_on_{UINT16_MAX};

  float level_{0.0f};
  bool soft_reboot_{false}, adopt_taken_{false};
  bool from_wheel_{true};
  uint16_t wheel_raw_a_{0}, wheel_raw_b_{0};
  bool have_wheel_{false};
  uint16_t wheel_settled_{0};
  bool curve_dirty_{true};
  uint16_t curve_target_{0};
  float curve_half_{0.0f};
  uint16_t clamp_half_{0};
  uint16_t last_good_half_{0};

  bool gate_sent_{false}, gate_acked_{false};
  uint16_t sent_target_{0};
  uint8_t sent_fade_{0};
  uint32_t gate_tx_ms_{0};

  uint8_t led_bits_{0}, night_{0};
  bool outputs_sent_{false};
  uint8_t sent_leds_{0}, sent_night_{0};
  uint32_t outputs_tx_ms_{0};

  bool buttons_init_{false};
  uint8_t buttons_stable_{0};
  uint8_t addon_blank_{0}, addon_relay_{0};
  bool addon_up_{false}, addon_down_{false};
  // The last judged window's classification.
  bool addon_seen_up_{false}, addon_seen_down_{false};
  Event events_[EVENT_QUEUE]{};
  uint8_t event_head_{0}, event_count_{0};

  // The expected MCU firmware; expected_build_ 0 = not set.
  uint16_t expected_version_{0};
  uint32_t expected_build_{0};
  uint8_t expected_boot_version_{0};
};

namespace detail {

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

inline constexpr float PI_F = 3.14159265358979f;
inline constexpr float NOMINAL_MAINS_HZ = 60.0f;
inline constexpr float NOMINAL_HALF = PCA_HZ / (2.0f * NOMINAL_MAINS_HZ);
inline constexpr float K_A_MS = 4.17f, K_B_MS = 7.50f;
inline constexpr uint16_t CLAMP_MARGIN = 8;
inline constexpr uint16_t WHEEL_DEADBAND = 4;
inline constexpr int CURVE_BISECTION_STEPS = 20;
inline constexpr float CURVE_HALF_CHANGE = 2.0f;
// A gate or outputs frame that the status does not show yet is sent again after RESEND_MS, or
// after SILENT_RESEND_MS while the MCU does not answer.
inline constexpr uint32_t RESEND_MS = 30, SILENT_RESEND_MS = 1000;
inline constexpr uint8_t ADDON_BLANK_FRAMES = 3;
inline constexpr uint8_t SUPPORTED_PROTOCOL = FW_VERSION >> 8;
inline constexpr float THERM_PULLUP_OHMS = 10000.0f;
inline constexpr uint16_t ADC_MAX = 1023;

}

inline uint8_t brightness_bar_leds(bool on, float brightness) {
  if (!on)
    return 0;
  const int n = static_cast<int>(std::ceil(BAR_LEDS * brightness - 0.001f));
  return static_cast<uint8_t>(n < 1 ? 1 : (n > BAR_LEDS ? BAR_LEDS : n));
}

inline float power_share(float delay, float half) {
  if (half <= 0.0f)
    return 0.0f;
  float t = detail::PI_F * detail::clampf(delay / half, 0.0f, 1.0f);
  return 1.0f - t / detail::PI_F + std::sin(2.0f * t) / (2.0f * detail::PI_F);
}

inline float delay_for_power(float power, float half) {
  // power_share falls from 1 at delay 0 to 0 at delay half.
  float lo = 0.0f, hi = half;
  for (int i = 0; i < detail::CURVE_BISECTION_STEPS; i++) {
    float mid = 0.5f * (lo + hi);
    if (power_share(mid, half) > power)
      lo = mid;
    else
      hi = mid;
  }
  return 0.5f * (lo + hi);
}

inline float power_for_level(float level, float p_dim, float p_bright) {
  const float l_dim = std::cbrt(p_dim);
  const float lightness = l_dim + level * (std::cbrt(p_bright) - l_dim);
  return lightness * lightness * lightness;
}

inline float level_for_power(float power, float p_dim, float p_bright) {
  const float l_dim = std::cbrt(p_dim), l_bright = std::cbrt(p_bright);
  if (l_bright == l_dim)
    return 1.0f;
  return (std::cbrt(power) - l_dim) / (l_bright - l_dim);
}

template<typename U> void Link::loop(U &uart, uint32_t now_ms) {
  this->receive_(uart, now_ms, [this](const Received &rx) {
    if (rx.type == MSG_STATUS && rx.decoded)
      this->handle_status_(this->status_);
  });
  auto prepare = [this](uint8_t slot, uint8_t *frame) { return this->prepare_tx_(slot, frame); };
  // Right after a gate frame the outputs go first, so a long transition (a new target every loop)
  // cannot hold them back.
  this->last_app_slot_ = this->last_app_slot_ == TX_GATE ? send_first(uart, {TX_OUTPUTS, TX_GATE}, prepare)
                                                         : send_first(uart, {TX_GATE, TX_OUTPUTS}, prepare);
}

inline void Link::handle_status_(const Status &s) {
  // A FAULT starts when "starting" has cleared, and again after a restart, even if the last status
  // before the restart showed one too.
  const bool fault = reported_fault_(s);
  const bool restarted = this->mcu_restarts() != this->status_restarts_;
  if (fault && (restarted || !this->status_fault_))
    this->fault_event_ = true;
  this->status_fault_ = fault;
  this->status_restarts_ = this->mcu_restarts();
  if (s.flags & S_ZO_COASTED)
    this->zo_coasted_windows_++;

  if (this->gate_sent_) {
    bool clamped = (s.faults & F_CLAMPED) != 0;
    if (s.half == 0)
      this->gate_acked_ = (this->sent_target_ == 0 && !clamped) || (this->sent_target_ != 0 && clamped);
    else
      this->gate_acked_ = s.target == this->sent_target_ && !clamped;
  }

  if (s.half != 0) {
    if (std::fabs(static_cast<float>(s.half) - this->curve_half_) > detail::CURVE_HALF_CHANGE)
      this->curve_dirty_ = true;
    this->last_good_half_ = s.half;
  }
  if ((s.half == 0) != (this->clamp_half_ == 0))
    this->curve_dirty_ = true;

  if (!this->have_wheel_ ||
      (s.wheel > this->wheel_settled_ ? s.wheel - this->wheel_settled_ : this->wheel_settled_ - s.wheel) >=
          detail::WHEEL_DEADBAND) {
    this->have_wheel_ = true;
    this->wheel_settled_ = s.wheel;
    if (this->from_wheel_)
      this->curve_dirty_ = true;
  }

  this->process_buttons_(s);
  this->process_addon_(s);
}

inline void Link::process_buttons_(const Status &s) {
  struct Button {
    Input input;
    uint8_t bit;
  };
  static constexpr Button BUTTONS[] = {{IN_ONOFF, BTN_ONOFF}, {IN_UP, BTN_UP}, {IN_DOWN, BTN_DOWN}};
  uint8_t level = s.buttons & BTN_ALL, changed = (s.buttons >> BTN_CHANGED_SHIFT) & BTN_ALL;
  if (!this->buttons_init_) {
    this->buttons_init_ = true;
    this->buttons_stable_ = level;
    for (const Button &b : BUTTONS)
      this->push_event_(b.input, (level & b.bit) != 0);
    return;
  }
  for (const Button &b : BUTTONS) {
    if (!(changed & b.bit) && ((level ^ this->buttons_stable_) & b.bit)) {
      this->buttons_stable_ ^= b.bit;
      this->push_event_(b.input, (level & b.bit) != 0);
    }
  }
}

inline void Link::process_addon_(const Status &s) {
  bool init = !this->addon_init_;
  this->addon_init_ = true;
  // The changed bit covers only the window's samples: a drive that changed after the last sample
  // shows only in bit 0, and those samples must not be judged with the new drive's thresholds.
  bool drive_changed = !init && ((s.flags ^ this->addon_relay_) & S_RELAY);
  this->addon_relay_ = s.flags;
  bool skip = false;
  if ((s.flags & S_KS_STALE) || drive_changed) {
    this->addon_blank_ = detail::ADDON_BLANK_FRAMES;
    skip = true;
  } else if (this->addon_blank_) {
    this->addon_blank_--;
    skip = true;
  } else if (s.ks_min == KS_NO_SAMPLE) {
    skip = true;
  }
  bool up = false, down = false;
  if (!skip) {
    bool drive = (s.flags & S_RELAY) != 0;
    uint16_t up_min = drive ? this->addon_up_on_ : this->addon_up_off_;
    uint16_t down_min = drive ? this->addon_down_on_ : this->addon_down_off_;
    down = s.ks_min >= down_min;
    up = !down && s.ks_min >= up_min;
  }
  if (init) {
    this->addon_up_ = this->addon_seen_up_ = up;
    this->addon_down_ = this->addon_seen_down_ = down;
    this->push_event_(IN_ADDON_UP, up);
    this->push_event_(IN_ADDON_DOWN, down);
    return;
  }
  if (skip)
    return;
  const bool seen_up = this->addon_seen_up_, seen_down = this->addon_seen_down_;
  this->addon_seen_up_ = up;
  this->addon_seen_down_ = down;
  up = up == seen_up ? up : this->addon_up_;
  down = down == seen_down ? down : this->addon_down_;
  // Releases first, so a direct UP -> DOWN change never shows both held.
  if (!up && this->addon_up_) {
    this->addon_up_ = false;
    this->push_event_(IN_ADDON_UP, false);
  }
  if (!down && this->addon_down_) {
    this->addon_down_ = false;
    this->push_event_(IN_ADDON_DOWN, false);
  }
  if (up && !this->addon_up_) {
    this->addon_up_ = true;
    this->push_event_(IN_ADDON_UP, true);
  }
  if (down && !this->addon_down_) {
    this->addon_down_ = true;
    this->push_event_(IN_ADDON_DOWN, true);
  }
}

inline void Link::push_event_(Input input, bool state) {
  if (this->event_count_ == EVENT_QUEUE) {
    this->event_head_ = (this->event_head_ + 1) % EVENT_QUEUE;
    this->event_count_--;
  }
  uint8_t idx = (this->event_head_ + this->event_count_) % EVENT_QUEUE;
  this->events_[idx].input = input;
  this->events_[idx].state = state;
  this->event_count_++;
}

inline bool Link::pop_input_event(Input *input, bool *state) {
  if (this->event_count_ == 0)
    return false;
  *input = static_cast<Input>(this->events_[this->event_head_].input);
  *state = this->events_[this->event_head_].state;
  this->event_head_ = (this->event_head_ + 1) % EVENT_QUEUE;
  this->event_count_--;
  return true;
}

// No gate or outputs frame before the info frame: the protocol check needs it. Nothing to our
// bootloader either: it drops them, and the main configuration leaves it to return to the
// application after its idle time.
inline bool Link::gate_allowed_() const {
  return this->have_current_info() && this->adopt_taken_ && !this->in_bootloader_() && this->protocol_supported() &&
         !std::isnan(this->dim_fraction_) && !std::isnan(this->bright_fraction_);
}

inline size_t Link::prepare_tx_(uint8_t slot, uint8_t *frame) {
  uint8_t payload[APP_MAX_PAYLOAD];
  const uint32_t resend =
      this->link_state() == LinkState::NO_RESPONSE ? detail::SILENT_RESEND_MS : detail::RESEND_MS;
  switch (slot) {
    case TX_GATE: {
      if (!this->gate_allowed_())
        return 0;
      uint16_t want = this->wanted_target_();
      // The smoothing limits are not in the status: they go out with every gate frame.
      bool send = !this->gate_sent_ || want != this->sent_target_ || this->fade_limit_ != this->sent_fade_ ||
                  (!this->gate_acked_ && this->elapsed_(this->gate_tx_ms_) >= resend);
      if (!send)
        return 0;
      this->sent_target_ = want;
      this->sent_fade_ = this->fade_limit_;
      this->gate_sent_ = true;
      this->gate_acked_ = false;
      this->gate_tx_ms_ = this->now_;
      detail::put16(payload, want);
      payload[2] = this->fade_limit_;
      payload[3] = this->fade_limit_;
      return build_frame(frame, MSG_GATE, payload, GATE_LEN);
    }
    case TX_OUTPUTS: {
      if (!this->gate_allowed_())
        return 0;
      // The MCU starts with all outputs off, so this also restores them after an MCU restart.
      if (this->status_.leds == this->led_bits_ && this->status_.night == this->night_)
        return 0;
      bool changed = !this->outputs_sent_ || this->led_bits_ != this->sent_leds_ || this->night_ != this->sent_night_;
      if (!changed && this->elapsed_(this->outputs_tx_ms_) < resend)
        return 0;
      this->outputs_sent_ = true;
      this->sent_leds_ = this->led_bits_;
      this->sent_night_ = this->night_;
      this->outputs_tx_ms_ = this->now_;
      payload[0] = this->led_bits_;
      payload[1] = this->night_;
      return build_frame(frame, MSG_OUTPUTS, payload, OUTPUTS_LEN);
    }
    default:
      return 0;
  }
}

inline void Link::set_level(float level) {
  level = std::isnan(level) ? 0.0f : detail::clampf(level, 0.0f, 1.0f);
  if (level != this->level_) {
    this->level_ = level;
    this->curve_dirty_ = true;
  }
}

inline bool Link::fault_holds_off() const {
  return this->have_status_ && reported_fault_(this->status_) && !this->last_seen_boot_ &&
         this->link_state() == LinkState::APPLICATION;
}

inline void Link::set_soft_reboot(bool soft_reboot) { this->soft_reboot_ = soft_reboot; }

template<typename S> void Link::dispatch(S &sink) {
  Input input;
  bool state;
  while (this->pop_input_event(&input, &state))
    sink.input(input, state);
  if (this->take_fault_event())
    sink.fault_off();
  float level;
  switch (this->take_adopt(&level)) {
    case Adopt::ON:
      sink.adopt_on(level);
      break;
    case Adopt::OFF:
      sink.adopt_off();
      break;
    default:
      break;
  }
}

inline bool Link::b3_on(bool blink_on) const {
  return this->link_state() == LinkState::APPLICATION ? this->fault_holds_off() : blink_on;
}

inline bool Link::reset_cause_text(char *buf, size_t size) const {
  if (!this->have_current_info())
    return false;
  const Info &info = this->current_info_;
  snprintf(buf, size, "%s, watchdog count %u, bootloader %u", reset_cause_name(info.reset_cause), info.wdt_count,
           info.boot_version);
  return true;
}

inline float Link::curve_half_now_() const {
  return this->last_good_half_ ? static_cast<float>(this->last_good_half_) : detail::NOMINAL_HALF;
}

inline void Link::curve_ends_(float half, float *p_dim, float *p_bright) const {
  float d_dim;
  if (this->from_wheel_ && this->wheel_calibrated_() && this->have_wheel_) {
    float k_ms = detail::K_A_MS + (static_cast<float>(this->wheel_settled_) - this->wheel_raw_a_) *
                                      (detail::K_B_MS - detail::K_A_MS) /
                                      (static_cast<float>(this->wheel_raw_b_) - this->wheel_raw_a_);
    k_ms = detail::clampf(k_ms, detail::K_A_MS, detail::K_B_MS);
    d_dim = k_ms * 1e-3f * PCA_HZ;
  } else {
    d_dim = this->dim_fraction_ * half;
  }
  float d_bright = this->bright_fraction_ * d_dim;
  if (d_bright < EARLIEST_FIRE)
    d_bright = EARLIEST_FIRE;
  *p_dim = power_share(d_dim, half);
  *p_bright = power_share(d_bright, half);
}

inline uint16_t Link::clamp_target_(float delay) const {
  long r = std::lround(delay);
  if (this->clamp_half_ != 0) {
    long hi = static_cast<long>(this->clamp_half_) - LATEST_BEFORE_EDGE - detail::CLAMP_MARGIN;
    if (r > hi)
      r = hi;
    if (r < EARLIEST_FIRE)
      r = EARLIEST_FIRE;
  }
  if (r < 1)
    r = 1;
  if (r > 0xFFFF)
    r = 0xFFFF;
  return static_cast<uint16_t>(r);
}

inline void Link::recompute_curve_() {
  float half = this->curve_half_now_();
  float p_dim, p_bright;
  this->curve_ends_(half, &p_dim, &p_bright);
  this->clamp_half_ = this->have_status_ ? this->status_.half : 0;
  this->curve_target_ = this->clamp_target_(delay_for_power(power_for_level(this->level_, p_dim, p_bright), half));
  this->dim_target_ = this->clamp_target_(delay_for_power(p_dim, half));
  this->curve_half_ = half;
  this->curve_dirty_ = false;
}

inline uint16_t Link::wanted_target_() {
  if (this->level_ <= 0.0f || (this->have_status_ && this->status_.state == ST_FAULT))
    return 0;
  if (this->curve_dirty_)
    this->recompute_curve_();
  // The MCU starts firing at the target it holds when SETTLE ends, without a fade.
  if (this->have_status_ &&
      (this->status_.state == ST_OFF || this->status_.state == ST_SETTLE || this->status_.state == ST_STOPPING))
    return this->dim_target_;
  return this->curve_target_;
}

inline Adopt Link::take_adopt(float *level) {
  if (this->adopt_taken_ || !this->have_status_)
    return Adopt::NONE;
  this->adopt_taken_ = true;
  const Status &s = this->status_;
  if (s.state == ST_FIRING || s.state == ST_SETTLE) {
    float half = s.half ? static_cast<float>(s.half) : this->curve_half_now_();
    float p_dim, p_bright;
    this->curve_ends_(half, &p_dim, &p_bright);
    float lvl = detail::clampf(level_for_power(power_share(s.target, half), p_dim, p_bright), 0.001f, 1.0f);
    this->set_level(lvl);
    if (level != nullptr)
      *level = lvl;
    return Adopt::ON;
  }
  if (!this->soft_reboot_)
    return Adopt::NONE;
  this->set_level(0.0f);
  if (level != nullptr)
    *level = 0.0f;
  return Adopt::OFF;
}

inline bool Link::take_fault_event() {
  bool e = this->fault_event_;
  this->fault_event_ = false;
  return e;
}

inline void Link::set_dim_end_from_wheel(bool from_wheel) {
  this->from_wheel_ = from_wheel;
  this->curve_dirty_ = true;
}
inline void Link::set_dim_end_fraction(float of_half) {
  if (std::isnan(of_half))
    return;
  this->dim_fraction_ = detail::clampf(of_half, 0.0f, 1.0f);
  this->curve_dirty_ = true;
}
inline void Link::set_bright_end_fraction(float of_dim_end) {
  if (std::isnan(of_dim_end))
    return;
  this->bright_fraction_ = detail::clampf(of_dim_end, 0.0f, 1.0f);
  this->curve_dirty_ = true;
}
inline void Link::set_fade_limit(uint8_t counts) { this->fade_limit_ = counts; }
inline void Link::set_wheel_calibration(uint16_t raw_a, uint16_t raw_b) {
  this->wheel_raw_a_ = raw_a;
  this->wheel_raw_b_ = raw_b;
  this->curve_dirty_ = true;
}

inline float Link::wheel_percent() const {
  if (!this->wheel_calibrated_() || !this->have_wheel_)
    return NAN;
  float p = (static_cast<float>(this->wheel_settled_) - this->wheel_raw_a_) * 100.0f /
            (static_cast<float>(this->wheel_raw_b_) - this->wheel_raw_a_);
  return detail::clampf(p, 0.0f, 100.0f);
}

inline void Link::set_leds(uint8_t bits, bool on) {
  this->led_bits_ = on ? (this->led_bits_ | bits) : (this->led_bits_ & ~bits);
}

inline void Link::set_night_light(float level) {
  level = std::isnan(level) ? 0.0f : detail::clampf(level, 0.0f, 1.0f);
  this->night_ = static_cast<uint8_t>(std::lround(level * 255.0f));
}

inline void Link::set_addon_thresholds(uint16_t up_off, uint16_t down_off, uint16_t up_on, uint16_t down_on) {
  this->addon_up_off_ = up_off;
  this->addon_down_off_ = down_off;
  this->addon_up_on_ = up_on;
  this->addon_down_on_ = down_on;
}

inline float Link::mains_hz() const {
  if (!this->have_status_ || this->status_.half == 0)
    return NAN;
  return PCA_HZ / (2.0f * this->status_.half);
}

inline float Link::therm_resistance_ohms() const {
  if (!this->have_status_ || this->status_.therm < THERM_OPEN_SHORT_LOW || this->status_.therm > THERM_OPEN_SHORT_HIGH)
    return NAN;
  return detail::THERM_PULLUP_OHMS * this->status_.therm / (detail::ADC_MAX - this->status_.therm);
}

inline const char *Link::state_text() const {
  static const char NEEDS_TOOL[] = "MCU firmware needs the MCU tool";
  if (this->unreadable_frames())
    return NEEDS_TOOL;
  switch (this->link_state()) {
    case LinkState::BOOTLOADER:
      return "Update mode (use the MCU tool)";
    case LinkState::NO_RESPONSE:
      return "No Response";
    default:
      break;
  }
  if (!this->protocol_supported())
    return NEEDS_TOOL;
  switch (this->status_.state) {
    case ST_OFF:
      return "Off";
    case ST_SETTLE:
      return "Starting";
    case ST_FIRING:
      return "On";
    case ST_STOPPING:
      return "Stopping";
    case ST_FAULT:
      return "Fault";
    default:
      return "Unknown";
  }
}

inline void Link::faults_text(char *buf, size_t size) const {
  static const struct {
    uint8_t bit;
    const char *name;
  } NAMES[] = {{F_OVERTEMP, "over-temperature"}, {F_THERM, "thermistor"}, {F_ZO, "mains lost"}, {F_GATE, "gate fault"}};
  TextOut out{buf, size};
  const uint8_t faults = this->have_status_ ? this->status_.faults : 0;
  for (const auto &n : NAMES) {
    if (faults & n.bit)
      out.add("%s%s", out.used ? ", " : "", n.name);
  }
  if (out.used == 0)
    out.add("none");
}

inline void Link::state_text_with_hint(char *buf, size_t size) const {
  if (size == 0)
    return;
  const char *hint = "";
  if (this->link_state() == LinkState::APPLICATION && this->protocol_supported()) {
    if (this->update_available())
      hint = " (update available: use the MCU tool)";
    else if (this->firmware_newer())
      hint = " (MCU firmware newer than this configuration)";
  }
  snprintf(buf, size, "%s%s", this->state_text(), hint);
}

inline bool Link::protocol_supported() const {
  if (this->unreadable_frames())
    return false;
  if (!this->have_current_info())
    return true;
  return (this->current_info_.version >> 8) == detail::SUPPORTED_PROTOCOL;
}

inline bool Link::problem() const {
  if (this->link_state() != LinkState::APPLICATION || !this->have_current_info() || !this->protocol_supported())
    return true;
  return this->have_status_ && reported_fault_(this->status_);
}

inline void Link::set_expected(uint16_t version, uint32_t build, uint8_t boot_version) {
  this->expected_version_ = version;
  this->expected_build_ = build;
  this->expected_boot_version_ = boot_version;
}

inline bool Link::firmware_age_(FirmwareAge *age) const {
  if (!this->have_current_info() || this->expected_build_ == 0)
    return false;
  *age = firmware_age(this->current_info_, this->expected_version_, this->expected_build_,
                      this->expected_boot_version_);
  return true;
}

inline bool Link::update_available() const {
  FirmwareAge age;
  return this->firmware_age_(&age) && (age == FirmwareAge::APP_OLD || age == FirmwareAge::BOOT_OLD);
}

inline bool Link::firmware_newer() const {
  FirmwareAge age;
  return this->firmware_age_(&age) && age == FirmwareAge::NEWER;
}

}