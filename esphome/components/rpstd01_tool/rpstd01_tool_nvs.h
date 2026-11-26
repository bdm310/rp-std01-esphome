// RP-STD01 MCU tool: the tool's NVS store on ESP-IDF's NVS.
//
// ESPHome includes every component header, also on the host platform, which has no NVS.
// RPSTD01TOOL_NVS_FAKE builds it against a fake.
#pragma once

#if defined(ESP_PLATFORM) || defined(RPSTD01TOOL_NVS_FAKE)
#include "esphome/components/rpstd01_tool/rpstd01_tool.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace rpstd01tool {

inline constexpr const char *NVS_NAMESPACE = "rpstd01tool";
inline constexpr const char *NVS_ARM = "isp_arm";
inline constexpr const char *NVS_USER = "isp_user";
inline constexpr const char *NVS_FAILS = "isp_fails";
inline constexpr const char *NVS_PARTIAL = "isp_part";
inline constexpr const char *NVS_RECORD = "isp_record";

class NvsToolStore final : public ToolStore {
 public:
  explicit NvsToolStore(bool write) {
    this->open_ = nvs_open(NVS_NAMESPACE, write ? NVS_READWRITE : NVS_READONLY, &this->h_) == ESP_OK;
  }
  ~NvsToolStore() {
    if (this->open_)
      nvs_close(this->h_);
  }
  NvsToolStore(const NvsToolStore &) = delete;
  NvsToolStore &operator=(const NvsToolStore &) = delete;
  bool is_open() const { return this->open_; }

  void load(mcuflash::Store *s) override {
    *s = mcuflash::Store{0, mcuflash::USER_NONE, 0, 0};
    if (!this->open_)
      return;
    nvs_get_u32(this->h_, NVS_ARM, &s->arm);
    nvs_get_u8(this->h_, NVS_USER, &s->user);
    nvs_get_u8(this->h_, NVS_FAILS, &s->fails);
    nvs_get_u8(this->h_, NVS_PARTIAL, &s->partial);
  }
  bool set(Key key, uint32_t value) override {
    if (!this->open_)
      return false;
    switch (key) {
      case KEY_ARM:
        return nvs_set_u32(this->h_, NVS_ARM, value) == ESP_OK;
      case KEY_USER:
        return nvs_set_u8(this->h_, NVS_USER, (uint8_t) value) == ESP_OK;
      case KEY_FAILS:
        return nvs_set_u8(this->h_, NVS_FAILS, (uint8_t) value) == ESP_OK;
      case KEY_PARTIAL:
        return nvs_set_u8(this->h_, NVS_PARTIAL, (uint8_t) value) == ESP_OK;
    }
    return false;
  }
  bool commit() override { return this->open_ && nvs_commit(this->h_) == ESP_OK; }

  bool load_record(IspRecord *rec) override {
    uint8_t bytes[ISP_RECORD_SIZE];
    size_t size = 0;
    return this->open_ && nvs_get_blob(this->h_, NVS_RECORD, nullptr, &size) == ESP_OK && size == sizeof(bytes) &&
           nvs_get_blob(this->h_, NVS_RECORD, bytes, &size) == ESP_OK && decode_record(bytes, size, rec);
  }
  bool save_record(const IspRecord &rec) override {
    uint8_t bytes[ISP_RECORD_SIZE];
    encode_record(rec, bytes);
    return this->open_ && nvs_set_blob(this->h_, NVS_RECORD, bytes, sizeof(bytes)) == ESP_OK &&
           nvs_commit(this->h_) == ESP_OK;
  }

 protected:
  nvs_handle_t h_{0};
  bool open_{false};
};

}
#endif
