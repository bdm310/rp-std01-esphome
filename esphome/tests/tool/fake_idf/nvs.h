// A fake of ESP-IDF's nvs.h for the tool's host test: one namespace-qualified map, values stored as
// bytes, with failure injection. Only what the tool's NVS store uses. As in ESP-IDF, each set is kept at once
// (it survives a restart without a commit); nvs_commit only reports.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

using esp_err_t = int;
using nvs_handle_t = uint32_t;
enum nvs_open_mode_t { NVS_READONLY, NVS_READWRITE };
static constexpr esp_err_t ESP_OK = 0;
static constexpr esp_err_t ESP_FAIL = -1;
static constexpr esp_err_t ESP_ERR_NVS_NOT_FOUND = 0x1102;
static constexpr esp_err_t ESP_ERR_NVS_READ_ONLY = 0x1107;
static constexpr esp_err_t ESP_ERR_NVS_INVALID_LENGTH = 0x110c;

namespace fake_nvs {
struct State {
  std::map<std::string, std::vector<uint8_t>> values;
  std::map<nvs_handle_t, std::pair<std::string, nvs_open_mode_t>> handles;
  nvs_handle_t next = 1;
  bool open_fails = false;
  int writes_left = -1;  // further set calls that succeed (-1: all)
  bool commit_fails = false;
  int sets = 0, commits = 0;
};
inline State &state() {
  static State s;
  return s;
}
inline void reset() { state() = State{}; }
inline std::string key(nvs_handle_t h, const char *k) { return state().handles[h].first + "/" + k; }
inline esp_err_t set(nvs_handle_t h, const char *k, const void *v, size_t n) {
  State &s = state();
  if (!s.handles.count(h))
    return ESP_FAIL;
  if (s.handles[h].second != NVS_READWRITE)
    return ESP_ERR_NVS_READ_ONLY;
  if (s.writes_left == 0)
    return ESP_FAIL;
  if (s.writes_left > 0)
    s.writes_left--;
  s.sets++;
  const uint8_t *b = static_cast<const uint8_t *>(v);
  s.values[key(h, k)] = std::vector<uint8_t>(b, b + n);
  return ESP_OK;
}
inline const std::vector<uint8_t> *get(nvs_handle_t h, const char *k) {
  State &s = state();
  if (!s.handles.count(h))
    return nullptr;
  auto v = s.values.find(key(h, k));
  return v == s.values.end() ? nullptr : &v->second;
}
template<typename T> esp_err_t get_int(nvs_handle_t h, const char *k, T *out) {
  const std::vector<uint8_t> *v = get(h, k);
  if (v == nullptr)
    return ESP_ERR_NVS_NOT_FOUND;
  if (v->size() != sizeof(T))
    return ESP_FAIL;
  memcpy(out, v->data(), sizeof(T));
  return ESP_OK;
}
// A restart: open handles are gone, the values stay.
inline void restart() { state().handles.clear(); }
}

inline esp_err_t nvs_open(const char *ns, nvs_open_mode_t mode, nvs_handle_t *h) {
  fake_nvs::State &s = fake_nvs::state();
  if (s.open_fails)
    return ESP_FAIL;
  *h = s.next++;
  s.handles[*h] = {ns, mode};
  return ESP_OK;
}
inline void nvs_close(nvs_handle_t h) { fake_nvs::state().handles.erase(h); }
inline esp_err_t nvs_get_u8(nvs_handle_t h, const char *k, uint8_t *v) { return fake_nvs::get_int(h, k, v); }
inline esp_err_t nvs_get_u32(nvs_handle_t h, const char *k, uint32_t *v) { return fake_nvs::get_int(h, k, v); }
inline esp_err_t nvs_set_u8(nvs_handle_t h, const char *k, uint8_t v) { return fake_nvs::set(h, k, &v, 1); }
inline esp_err_t nvs_set_u32(nvs_handle_t h, const char *k, uint32_t v) { return fake_nvs::set(h, k, &v, 4); }
inline esp_err_t nvs_set_blob(nvs_handle_t h, const char *k, const void *v, size_t n) {
  return fake_nvs::set(h, k, v, n);
}
inline esp_err_t nvs_get_blob(nvs_handle_t h, const char *k, void *out, size_t *n) {
  const std::vector<uint8_t> *v = fake_nvs::get(h, k);
  if (v == nullptr)
    return ESP_ERR_NVS_NOT_FOUND;
  if (out == nullptr) {  // as in ESP-IDF: only the size
    *n = v->size();
    return ESP_OK;
  }
  if (*n < v->size()) {
    *n = v->size();
    return ESP_ERR_NVS_INVALID_LENGTH;
  }
  memcpy(out, v->data(), v->size());
  *n = v->size();
  return ESP_OK;
}
inline esp_err_t nvs_commit(nvs_handle_t) {
  fake_nvs::State &s = fake_nvs::state();
  if (s.commit_fails)
    return ESP_FAIL;
  s.commits++;
  return ESP_OK;
}
