#pragma once

#include <array>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/number/number.h"
#include "esphome/components/output/float_output.h"
#include "esphome/components/rpstd01/rpstd01.h"
#include "esphome/components/select/select.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32
#include <esp_system.h>
#endif

namespace rpstd01 {

class LinkComponent : public esphome::Component, public esphome::uart::UARTDevice {
 public:
  using Number = esphome::number::Number;
  using Select = esphome::select::Select;
  using TextSensor = esphome::text_sensor::TextSensor;
  using LightState = esphome::light::LightState;

  LinkComponent(uint16_t wheel_raw_a, uint16_t wheel_raw_b, Number *dim_end_percent, Number *bright_end_percent,
                Number *fade_limit_counts, Number *addon_up_off_ks, Number *addon_down_off_ks, Number *addon_up_on_ks,
                Number *addon_down_on_ks)
      : wheel_raw_a_(wheel_raw_a),
        wheel_raw_b_(wheel_raw_b),
        dim_end_percent_(dim_end_percent),
        bright_end_percent_(bright_end_percent),
        fade_limit_counts_(fade_limit_counts),
        addon_up_off_ks_(addon_up_off_ks),
        addon_down_off_ks_(addon_down_off_ks),
        addon_up_on_ks_(addon_up_on_ks),
        addon_down_on_ks_(addon_down_on_ks) {
    this->link_.set_expected(MCU_IMAGE_VERSION, MCU_IMAGE_BUILD, MCU_IMAGE_BOOT_VERSION);
    this->link_.set_soft_reboot(soft_reboot_());
  }

  // Each select with the option the code acts on, as the code generation checked it.
  void set_thumbwheel_use(Select *select, const char *dim_end) {
    this->thumbwheel_use_ = select;
    this->thumbwheel_dim_end_ = dim_end;
  }
  void set_brightness_bar(Select *select, const char *level) {
    this->brightness_bar_ = select;
    this->brightness_bar_level_ = level;
  }
  void set_night_light_mode(Select *select, const char *follows_lamp) {
    this->night_light_mode_ = select;
    this->night_light_follows_lamp_ = follows_lamp;
  }
  void set_light(LightState *light) { this->light_ = light; }
  void set_input(Input input, esphome::binary_sensor::BinarySensor *sensor) { this->inputs_[input] = sensor; }
  void set_text_sensors(TextSensor *state, TextSensor *faults, TextSensor *firmware, TextSensor *firmware_expected,
                        TextSensor *reset_cause) {
    this->state_ = state;
    this->faults_ = faults;
    this->firmware_ = firmware;
    this->firmware_expected_ = firmware_expected;
    this->reset_cause_ = reset_cause;
  }

  Link &link() { return this->link_; }

  // The brightness bar (LED1..LED5) for the lamp's state, if Brightness Bar shows the level.
  void update_brightness_bar() {
    const auto &v = this->light_->remote_values;
    const bool shown = this->brightness_bar_->current_option() == this->brightness_bar_level_;
    const uint8_t lit = brightness_bar_leds(shown && v.is_on(), v.get_brightness());
    for (uint8_t i = 0; i < BAR_LEDS; i++)
      this->link_.set_leds(BAR_LED_BITS[i], i < lit);
  }
  bool night_light_follows_lamp() const {
    return this->night_light_mode_->current_option() == this->night_light_follows_lamp_;
  }

  // After the numbers, selects and lights have restored their state.
  float get_setup_priority() const override { return esphome::setup_priority::DATA; }
  void setup() override {
    const auto apply = [this](auto &&...) { this->apply_settings_(); };
    this->thumbwheel_use_->add_on_state_callback(apply);
    for (Number *n : this->numbers_())
      n->add_on_state_callback(apply);
    this->apply_settings_();
    this->set_interval(PANEL_EVERY_MS, [this]() { this->publish_panel_(); });
  }

  void loop() override {
    this->link_.loop(*this, esphome::App.get_loop_component_start_time());
    this->link_.dispatch(*this);
    this->log_changes_();
  }
  void dump_config() override {
    ESP_LOGCONFIG(TAG,
                  "RP-STD01 link:\n"
                  "  Expected MCU firmware: %s, bootloader %u\n"
                  "  Thumbwheel raw A: %u, B: %u",
                  firmware_text(MCU_IMAGE_VERSION, MCU_IMAGE_BUILD).str, MCU_IMAGE_BOOT_VERSION, this->wheel_raw_a_,
                  this->wheel_raw_b_);
  }

  void input(Input input, bool state) { this->inputs_[input]->publish_state(state); }
  void fault_off() {
    char faults[64];
    this->link_.faults_text(faults, sizeof faults);
    const Status &st = this->link_.status();
    ESP_LOGW(TAG, "MCU fault (%s; target %u, current %u, half %u, flags 0x%02X), turning the light off", faults,
             st.target, st.current, st.half, st.flags);
    this->light_->turn_off().set_transition_length(0).perform();
  }
  void adopt_on(float level) { this->light_->turn_on().set_brightness(level).set_transition_length(0).perform(); }
  void adopt_off() { this->light_->turn_off().set_transition_length(0).perform(); }

 protected:
  static constexpr const char *TAG = "rpstd01";
  static constexpr uint32_t PANEL_EVERY_MS = 500;

  // A brown-out counts as a power-on: the MCU has most likely restarted too.
  static bool soft_reboot_() {
#ifdef USE_ESP32
    const esp_reset_reason_t reason = esp_reset_reason();
    return reason != ESP_RST_POWERON && reason != ESP_RST_BROWNOUT;
#else
    return false;
#endif
  }

  void publish_panel_() {
    const Link &link = this->link_;
    char buf[96];
    link.state_text_with_hint(buf, sizeof(buf));
    publish_if_changed(this->state_, buf);
    link.faults_text(buf, sizeof(buf));
    publish_if_changed(this->faults_, buf);
    publish_if_changed(this->firmware_expected_, firmware_text(MCU_IMAGE_VERSION, MCU_IMAGE_BUILD).str);
    if (link.have_current_info()) {
      const Info &info = link.current_info();
      publish_if_changed(this->firmware_, firmware_text(info.version, info.build).str);
    }
    if (link.reset_cause_text(buf, sizeof(buf)))
      publish_if_changed(this->reset_cause_, buf);
    this->blink_on_ = !this->blink_on_;
    this->link_.set_leds(OUTPUTS_B3, link.b3_on(this->blink_on_));
  }

  void log_changes_() {
    const Link &link = this->link_;
    const bool update_available = link.update_available(), newer = link.firmware_newer();
    if ((update_available || newer) && !this->age_logged_) {
      const Info &info = link.current_info();
      ESP_LOGW(TAG, "MCU firmware %s, bootloader %u is %s than the expected %s, bootloader %u: %s",
               firmware_text(info.version, info.build).str, info.boot_version, newer ? "newer" : "older",
               firmware_text(MCU_IMAGE_VERSION, MCU_IMAGE_BUILD).str, MCU_IMAGE_BOOT_VERSION,
               newer ? "update this configuration" : "update it with the MCU tool");
    }
    this->age_logged_ = update_available || newer;
    const bool update_mode = link.link_state() == LinkState::BOOTLOADER;
    if (update_mode && !this->update_mode_logged_)
      ESP_LOGW(TAG, "MCU in update mode: nothing is sent to it; use the MCU tool");
    this->update_mode_logged_ = update_mode;
    if (link.mcu_restarts() != this->restarts_logged_) {
      this->restarts_logged_ = link.mcu_restarts();
      ESP_LOGW(TAG, "MCU restarted (%u times since boot)", (unsigned) this->restarts_logged_);
    }
  }

  std::array<Number *, 7> numbers_() const {
    return {this->dim_end_percent_,   this->bright_end_percent_, this->fade_limit_counts_, this->addon_up_off_ks_,
            this->addon_down_off_ks_, this->addon_up_on_ks_,     this->addon_down_on_ks_};
  }

  void apply_settings_() {
    Link &link = this->link_;
    link.set_wheel_calibration(this->wheel_raw_a_, this->wheel_raw_b_);
    link.set_dim_end_from_wheel(!this->thumbwheel_use_->has_state() ||
                                this->thumbwheel_use_->current_option() == this->thumbwheel_dim_end_);
    if (this->dim_end_percent_->has_state())
      link.set_dim_end_fraction(this->dim_end_percent_->state / 100.0f);
    if (this->bright_end_percent_->has_state())
      link.set_bright_end_fraction(this->bright_end_percent_->state / 100.0f);
    if (this->fade_limit_counts_->has_state())
      link.set_fade_limit(static_cast<uint8_t>(this->fade_limit_counts_->state));
    if (this->addon_up_off_ks_->has_state() && this->addon_down_off_ks_->has_state() &&
        this->addon_up_on_ks_->has_state() && this->addon_down_on_ks_->has_state())
      link.set_addon_thresholds(this->addon_up_off_ks_->state, this->addon_down_off_ks_->state,
                                this->addon_up_on_ks_->state, this->addon_down_on_ks_->state);
  }

  Link link_;
  bool age_logged_{false}, update_mode_logged_{false};
  uint32_t restarts_logged_{0};
  bool blink_on_{false};
  uint16_t wheel_raw_a_, wheel_raw_b_;
  Number *dim_end_percent_, *bright_end_percent_, *fade_limit_counts_;
  Number *addon_up_off_ks_, *addon_down_off_ks_, *addon_up_on_ks_, *addon_down_on_ks_;
  Select *thumbwheel_use_{nullptr}, *brightness_bar_{nullptr}, *night_light_mode_{nullptr};
  const char *thumbwheel_dim_end_{""}, *brightness_bar_level_{""}, *night_light_follows_lamp_{""};
  LightState *light_{nullptr};
  std::array<esphome::binary_sensor::BinarySensor *, IN_COUNT> inputs_{};
  TextSensor *state_{nullptr}, *faults_{nullptr}, *firmware_{nullptr}, *firmware_expected_{nullptr},
      *reset_cause_{nullptr};
};

class LampOutput : public esphome::output::FloatOutput {
 public:
  explicit LampOutput(LinkComponent *parent) : parent_(parent) {}

 protected:
  void write_state(float state) override { this->parent_->link().set_level(state); }

  LinkComponent *parent_;
};

class NightLightOutput : public esphome::output::FloatOutput {
 public:
  explicit NightLightOutput(LinkComponent *parent) : parent_(parent) {}

 protected:
  void write_state(float state) override { this->parent_->link().set_night_light(state); }

  LinkComponent *parent_;
};

}
