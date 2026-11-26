// RP-STD01 MCU tool: the glue for the ESP32 (NVS, the power-on step, the OTA image state) and for
// the host-platform tests (no power-on step, NVS in RAM).
#pragma once

#include "esphome/components/rpstd01_tool/rpstd01_tool.h"

namespace rpstd01tool {

Carried carried_image();

// Starts the tool with this platform's store and the power-on step's record, if one ran.
void setup_tool(Tool &tool);

// Until the image is marked valid, a power cycle may roll back to the previous firmware on
// second-stage bootloaders that support it (Tasmota's does not). ESPHome's safe_mode marks it after
// SAFE_MODE_GOOD_AFTER_S; this marks it as soon as the network is up and the check has decided.
void confirm_image_if_ready(Tool &tool);

}
