#pragma once

#include "esphome/components/rpstd01_tool/rpstd01_tool_platform.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

namespace rpstd01tool {

class ToolComponent : public esphome::Component, public esphome::uart::UARTDevice {
 public:
  Tool &tool() { return this->tool_; }

  void setup() override {
    setup_tool(this->tool_);
    this->set_interval(CONFIRM_IMAGE, CONFIRM_IMAGE_EVERY_MS, [this]() {
      confirm_image_if_ready(this->tool_);
      if (!this->tool_.image_pending())
        this->cancel_interval(CONFIRM_IMAGE);
    });
  }
  void loop() override { this->tool_.loop(*this, esphome::App.get_loop_component_start_time()); }
  void dump_config() override {
    char text[160];
    this->tool_.carried_text(text, sizeof(text));
    ESP_LOGCONFIG("rpstd01tool", "RP-STD01 MCU tool:\n  Carried: %s", text);
  }
  float get_setup_priority() const override { return esphome::setup_priority::HARDWARE; }

 protected:
  static constexpr const char *CONFIRM_IMAGE = "confirm_image";
  static constexpr uint32_t CONFIRM_IMAGE_EVERY_MS = 1000;

  Tool tool_;
};

}
