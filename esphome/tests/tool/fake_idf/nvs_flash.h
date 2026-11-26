// Fake of ESP-IDF's nvs_flash.h for the tool's host test.
#pragma once
#include "nvs.h"
inline esp_err_t nvs_flash_init() { return ESP_OK; }
