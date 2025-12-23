#include "ble_client_hid.h"
#include "usages.h"

#ifdef USE_ESP32
#include <array>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

#include <esp_bt.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_common_api.h>
#include <esp_gattc_api.h>

namespace esphome {
namespace ble_client_hid {

static const char *const TAG = "ble_client_hid";

// -----------------------------------------------------------------------------
// Build-time options
// -----------------------------------------------------------------------------
#ifndef BLE_HID_DEBUG
#define BLE_HID_DEBUG 0
#endif

#ifndef BLE_HID_INCLUDE_FALLBACK_PAIR
// Set to 1 to include the old "62/63" fallback pair (useful before first cache build).
// Set to 0 to rely purely on discovery + persisted cache.
#define BLE_HID_INCLUDE_FALLBACK_PAIR 1
#endif

#if BLE_HID_DEBUG
#define DBG_LOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#define DBG_LOGW(...) ESP_LOGW(TAG, __VA_ARGS__)
#else
#define DBG_LOGI(...) \
  do {               \
  } while (0)
#define DBG_LOGW(...) \
  do {               \
  } while (0)
#endif

static constexpr uint16_t FALLBACK_INPUT_HANDLE = 62;
static constexpr uint16_t FALLBACK_CCC_HANDLE = 63;

// Multi-press timing (device-side interpretation)
static constexpr uint32_t MULTIPRESS_GAP_MS = 400;
static constexpr uint32_t LONG_PRESS_MS = 1500;

// CCC write guard (avoid spamming)
static constexpr uint32_t CCC_MIN_INTERVAL_MS = 5000;

// -----------------------------------------------------------------------------
// Button state (per instance) - only for "real buttons", not wheel events.
// -----------------------------------------------------------------------------
enum class ButtonId : uint8_t { UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3, NONE = 255 };

struct ButtonState {
  bool is_down{false};
  bool long_fired{false};
  uint8_t click_count{0};
};

struct InstanceButtons {
  std::array<ButtonState, 4> st{};
  ButtonId active_button{ButtonId::NONE};
};

static std::map<const BLEClientHID *, InstanceButtons> btn_state_by_instance;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static std::string hex4(uint16_t v) {
  std::ostringstream ss;
  ss << std::hex << std::nouppercase << std::setfill('0') << std::setw(4) << (unsigned) v;
  return ss.str();
}

static std::string bytes_hex(const uint8_t *data, size_t len, size_t max_len = 24) {
  std::ostringstream ss;
  ss << std::hex << std::nouppercase << std::setfill('0');
  const size_t n = (len > max_len) ? max_len : len;
  for (size_t i = 0; i < n; i++)
    ss << std::setw(2) << (unsigned) data[i];
  if (len > max_len)
    ss << "...";
  return ss.str();
}

static const char *button_name(ButtonId b) {
  switch (b) {
    case ButtonId::UP:
      return "up";
    case ButtonId::DOWN:
      return "down";
    case ButtonId::LEFT:
      return "left";
    case ButtonId::RIGHT:
      return "right";
    default:
      return "unknown";
  }
}

static ButtonId raw_to_button_press(uint16_t raw) {
  // Essence Remote observed:
  // 0x0006 = Up
  // 0x0001 = Down
  // 0x000B = Left
  // 0x000A = Right
  switch (raw) {
    case 0x0006:
      return ButtonId::UP;
    case 0x0001:
      return ButtonId::DOWN;
    case 0x000B:
      return ButtonId::LEFT;
    case 0x000A:
      return ButtonId::RIGHT;
    default:
      return ButtonId::NONE;
  }
}

// -----------------------------------------------------------------------------
// Notify pairs + per-instance BLE/CCC state
// -----------------------------------------------------------------------------
struct NotifyPair {
  uint16_t input_handle{0};  // characteristic value handle
  uint16_t ccc_handle{0};    // 0x2902 descriptor handle

  bool operator==(const NotifyPair &o) const { return input_handle == o.input_handle && ccc_handle == o.ccc_handle; }
};

struct CccState {
  bool enabled{false};
  uint32_t last_attempt_ms{0};
};

struct InstanceBleState {
  std::vector<NotifyPair> pairs;                 // cached + discovered (+ optional fallback)
  std::map<uint16_t, CccState> ccc_by_ccc;       // keyed by ccc_handle
  std::map<uint16_t, uint16_t> ccc_value_by_ccc; // desired: 0x0001 notify, 0x0002 indicate
  bool loaded_pairs{false};
  uint32_t last_notify_ms{0};

  // HID service range (0x1812), captured from SEARCH_RES_EVT
  bool have_hid_range{false};
  uint16_t hid_start{0};
  uint16_t hid_end{0};

  bool tried_ccc_both_bits{false};
};

static std::map<const BLEClientHID *, InstanceBleState> ble_state_by_instance;

static void add_pair_unique_(std::vector<NotifyPair> &v, const NotifyPair &p) {
  if (p.input_handle == 0 || p.ccc_handle == 0)
    return;
  for (auto &e : v) {
    if (e == p)
      return;
  }
  v.push_back(p);
}

static bool input_is_known_(BLEClientHID *self, uint16_t input_handle) {
  auto &st = ble_state_by_instance[self];
  for (auto &p : st.pairs) {
    if (p.input_handle == input_handle)
      return true;
  }
  return false;
}

// -----------------------------------------------------------------------------
// Persisted handle cache (Preferences / NVS) per remote MAC
// -----------------------------------------------------------------------------
static constexpr uint32_t HANDLE_CACHE_MAGIC = 0xB0E05A11;
static constexpr uint8_t HANDLE_CACHE_VERSION = 1;
static constexpr size_t HANDLE_CACHE_MAX_PAIRS = 6;

struct HandleCacheBlob {
  uint32_t magic{HANDLE_CACHE_MAGIC};
  uint8_t version{HANDLE_CACHE_VERSION};
  uint8_t count{0};
  uint16_t reserved{0};
  NotifyPair pairs[HANDLE_CACHE_MAX_PAIRS]{};
};

struct InstanceHandleCache {
  bool pref_init{false};
  bool loaded{false};
  ESPPreferenceObject pref;
  HandleCacheBlob blob{};
};

static std::map<const BLEClientHID *, InstanceHandleCache> handle_cache_by_instance;

static uint32_t fnv1a32_(const char *s) {
  uint32_t h = 2166136261u;
  while (s && *s) {
    h ^= static_cast<uint8_t>(*s++);
    h *= 16777619u;
  }
  return h;
}

static uint32_t make_pref_key_(BLEClientHID *self) {
  const char *mac = self->parent()->address_str();
  uint32_t h = fnv1a32_("ble_client_hid_handle_cache");
  h ^= fnv1a32_(mac ? mac : "");
  return h;
}

static void ensure_pref_(BLEClientHID *self) {
  auto &hc = handle_cache_by_instance[self];
  if (hc.pref_init)
    return;
  // NOTE: ESPHome requires template type here.
  hc.pref = esphome::global_preferences->make_preference<HandleCacheBlob>(make_pref_key_(self));
  hc.pref_init = true;
}

static void load_cached_pairs_(BLEClientHID *self) {
  auto &hc = handle_cache_by_instance[self];
  auto &st = ble_state_by_instance[self];
  if (hc.loaded)
    return;

  ensure_pref_(self);

  HandleCacheBlob tmp;
  bool ok = hc.pref.load(&tmp);

  st.pairs.clear();
  st.ccc_by_ccc.clear();
  st.ccc_value_by_ccc.clear();
  st.last_notify_ms = 0;
  st.tried_ccc_both_bits = false;

  if (ok && tmp.magic == HANDLE_CACHE_MAGIC && tmp.version == HANDLE_CACHE_VERSION) {
    hc.blob = tmp;
    for (uint8_t i = 0; i < hc.blob.count && i < HANDLE_CACHE_MAX_PAIRS; i++) {
      add_pair_unique_(st.pairs, hc.blob.pairs[i]);
    }
    // Default desired CCC value to NOTIFY for cached pairs unless discovery later overrides.
    for (auto &p : st.pairs) {
      if (p.ccc_handle != 0 && st.ccc_value_by_ccc.find(p.ccc_handle) == st.ccc_value_by_ccc.end()) {
        st.ccc_value_by_ccc[p.ccc_handle] = 0x0001;
      }
    }
    ESP_LOGI(TAG, "Handle cache loaded for %s: %u pair(s)", self->parent()->address_str(), hc.blob.count);
  } else {
    hc.blob = HandleCacheBlob{};
    hc.blob.magic = HANDLE_CACHE_MAGIC;
    hc.blob.version = HANDLE_CACHE_VERSION;
    hc.blob.count = 0;
    ESP_LOGI(TAG, "Handle cache empty for %s (first run)", self->parent()->address_str());
  }

#if BLE_HID_INCLUDE_FALLBACK_PAIR
  add_pair_unique_(st.pairs, NotifyPair{FALLBACK_INPUT_HANDLE, FALLBACK_CCC_HANDLE});
  if (FALLBACK_CCC_HANDLE != 0 && st.ccc_value_by_ccc.find(FALLBACK_CCC_HANDLE) == st.ccc_value_by_ccc.end()) {
    st.ccc_value_by_ccc[FALLBACK_CCC_HANDLE] = 0x0001;
  }
#endif

  hc.loaded = true;
  st.loaded_pairs = true;
}

static void save_cached_pairs_(BLEClientHID *self) {
  auto &hc = handle_cache_by_instance[self];
  auto &st = ble_state_by_instance[self];
  if (!hc.pref_init)
    return;

  HandleCacheBlob out{};
  out.magic = HANDLE_CACHE_MAGIC;
  out.version = HANDLE_CACHE_VERSION;

  uint8_t n = 0;
  for (auto &p : st.pairs) {
    if (n >= HANDLE_CACHE_MAX_PAIRS)
      break;
    if (p.input_handle == 0 || p.ccc_handle == 0)
      continue;
    out.pairs[n++] = p;
  }
  out.count = n;

  bool changed = (out.count != hc.blob.count);
  if (!changed) {
    for (uint8_t i = 0; i < out.count; i++) {
      if (!(out.pairs[i] == hc.blob.pairs[i])) {
        changed = true;
        break;
      }
    }
  }
  if (!changed)
    return;

  hc.blob = out;
  hc.pref.save(&hc.blob);
  ESP_LOGI(TAG, "Handle cache saved for %s: %u pair(s)", self->parent()->address_str(), hc.blob.count);
}

// -----------------------------------------------------------------------------
// CCC write + register-for-notify (guarded, per pair)
// -----------------------------------------------------------------------------
static void reset_ccc_state_(BLEClientHID *self) {
  auto &st = ble_state_by_instance[self];
  st.ccc_by_ccc.clear();
  st.last_notify_ms = 0;
  st.tried_ccc_both_bits = false;
}

static uint16_t desired_ccc_value_(BLEClientHID *self, uint16_t ccc_handle) {
  auto &st = ble_state_by_instance[self];
  auto it = st.ccc_value_by_ccc.find(ccc_handle);
  if (it != st.ccc_value_by_ccc.end())
    return it->second;
  return 0x0001;  // default notify
}

static void write_ccc_and_register_(BLEClientHID *self, const char *reason, bool force, uint16_t input_handle,
                                    uint16_t ccc_handle, uint16_t ccc_value_override, bool use_override) {
  auto &st = ble_state_by_instance[self];
  auto &cs = st.ccc_by_ccc[ccc_handle];

  const uint32_t now = esphome::millis();
  if (!force && cs.enabled && (now - cs.last_attempt_ms) < CCC_MIN_INTERVAL_MS) {
    return;
  }
  cs.last_attempt_ms = now;

  const uint16_t ccc_u16 = use_override ? ccc_value_override : desired_ccc_value_(self, ccc_handle);
  uint8_t ccc_value[2] = {static_cast<uint8_t>(ccc_u16 & 0xFF), static_cast<uint8_t>((ccc_u16 >> 8) & 0xFF)};

  esp_err_t r = esp_ble_gattc_write_char_descr(self->parent()->get_gattc_if(), self->parent()->get_conn_id(), ccc_handle,
                                              sizeof(ccc_value), ccc_value, ESP_GATT_WRITE_TYPE_RSP,
                                              ESP_GATT_AUTH_REQ_NONE);
  if (r == ESP_OK) {
    DBG_LOGI("CCC write ok (ccc=%u) val=0x%04x input=%u (%s)", ccc_handle, ccc_u16, input_handle, reason);
    cs.enabled = true;
  } else {
    DBG_LOGW("CCC write failed (ccc=%u) err=%d val=0x%04x input=%u (%s)", ccc_handle, (int) r, ccc_u16, input_handle,
             reason);
  }

  esp_err_t rn =
      esp_ble_gattc_register_for_notify(self->parent()->get_gattc_if(), self->parent()->get_remote_bda(), input_handle);
  if (rn != ESP_OK) {
    DBG_LOGW("register_for_notify failed for input=%u err=%d (%s)", input_handle, (int) rn, reason);
  }
}

static void enable_notifications_for_all_pairs_(BLEClientHID *self, const char *reason, bool force) {
  auto &st = ble_state_by_instance[self];
  for (auto &p : st.pairs) {
    write_ccc_and_register_(self, reason, force, p.input_handle, p.ccc_handle, 0, false);
  }
}

// One-time fallback: try CCC=0x0003 (notify+indicate bits) without changing desired mapping.
static void try_enable_ccc_both_bits_once_(BLEClientHID *self, const char *reason) {
  auto &st = ble_state_by_instance[self];
  if (st.tried_ccc_both_bits)
    return;
  st.tried_ccc_both_bits = true;

  for (auto &p : st.pairs) {
    write_ccc_and_register_(self, reason, true, p.input_handle, p.ccc_handle, 0x0003, true);
  }
}

// -----------------------------------------------------------------------------
// GATT DB discovery: find HID Report characteristic (0x2A4D) with NOTIFY/INDICATE
// and its CCC (0x2902), inside HID service range (0x1812).
// -----------------------------------------------------------------------------
static void discover_notify_pairs_(BLEClientHID *self, const char *reason) {
  auto &st = ble_state_by_instance[self];

  uint16_t start = 0x0001;
  uint16_t end = 0xFFFF;
  if (st.have_hid_range && st.hid_start != 0 && st.hid_end != 0 && st.hid_end >= st.hid_start) {
    start = st.hid_start;
    end = st.hid_end;
  }

  uint16_t count = 0;
  esp_err_t ec = esp_ble_gattc_get_attr_count(self->parent()->get_gattc_if(), self->parent()->get_conn_id(),
                                             ESP_GATT_DB_ALL, start, end, 0, &count);
  if (ec != ESP_OK || count == 0) {
    ESP_LOGW(TAG, "GATT DB: get_attr_count failed err=%d count=%u range=%u..%u (%s)", (int) ec, count, start, end,
             reason);
    return;
  }

  std::vector<esp_gattc_db_elem_t> db(count);
  uint16_t out_count = count;

  esp_err_t edb =
      esp_ble_gattc_get_db(self->parent()->get_gattc_if(), self->parent()->get_conn_id(), start, end, db.data(),
                           &out_count);
  if (edb != ESP_OK || out_count == 0) {
    ESP_LOGW(TAG, "GATT DB: get_db failed err=%d count=%u range=%u..%u (%s)", (int) edb, out_count, start, end, reason);
    return;
  }

  DBG_LOGI("DBG gattdb: %u attrs in range=%u..%u (%s)", out_count, start, end, reason);

  uint16_t cur_input = 0;
  uint16_t cur_ccc = 0;
  bool cur_notify = false;
  bool cur_indicate = false;
  uint16_t cur_props = 0;

  auto flush = [&]() {
    if (cur_input != 0 && cur_ccc != 0 && (cur_notify || cur_indicate)) {
      NotifyPair np{cur_input, cur_ccc};
      const size_t before = st.pairs.size();
      add_pair_unique_(st.pairs, np);

      // Choose desired CCC based on properties: notify preferred else indicate.
      const uint16_t want = cur_notify ? 0x0001 : 0x0002;
      st.ccc_value_by_ccc[np.ccc_handle] = want;

      if (st.pairs.size() != before) {
        // INFO on purpose (useful even without BLE_HID_DEBUG)
        ESP_LOGI(TAG, "HID notify candidate: input=%u ccc=%u mode=%s props=0x%02x", np.input_handle, np.ccc_handle,
                 (want == 0x0002 ? "indicate" : "notify"), cur_props);
      } else {
        DBG_LOGI("DBG: candidate already known: input=%u ccc=%u", np.input_handle, np.ccc_handle);
      }
    }

    cur_input = 0;
    cur_ccc = 0;
    cur_notify = false;
    cur_indicate = false;
    cur_props = 0;
  };

  for (uint16_t i = 0; i < out_count; i++) {
    auto &e = db[i];
    const bool uuid16 = (e.uuid.len == ESP_UUID_LEN_16);
    const uint16_t u16 = uuid16 ? e.uuid.uuid.uuid16 : 0;

    if (e.type == ESP_GATT_DB_CHARACTERISTIC) {
      flush();
      if (uuid16 && u16 == 0x2A4D) {
        cur_input = e.attribute_handle;
        cur_props = e.properties;
        cur_notify = (cur_props & ESP_GATT_CHAR_PROP_BIT_NOTIFY) != 0;
        cur_indicate = (cur_props & ESP_GATT_CHAR_PROP_BIT_INDICATE) != 0;
        DBG_LOGI("DBG report char: h=%u props=0x%02x%s%s", cur_input, cur_props, cur_notify ? " N" : "",
                 cur_indicate ? " I" : "");
      }
      continue;
    }

    if (e.type == ESP_GATT_DB_DESCRIPTOR) {
      if (cur_input != 0 && uuid16 && u16 == 0x2902) {
        cur_ccc = e.attribute_handle;
        DBG_LOGI("DBG CCC desc: ccc=%u for input=%u", cur_ccc, cur_input);
      }
      continue;
    }
  }

  flush();

  // If we discovered new pairs, persist them.
  save_cached_pairs_(self);
}

// -----------------------------------------------------------------------------
// Component implementation
// -----------------------------------------------------------------------------
void BLEClientHID::loop() {
  // No periodic work required.
}

void BLEClientHID::dump_config() {
  ESP_LOGCONFIG(TAG, "BLE Client HID (B&O Remote):");
  ESP_LOGCONFIG(TAG, " MAC address : %s", this->parent()->address_str());
  ESP_LOGCONFIG(TAG, " multi-press gap : %ums", (unsigned) MULTIPRESS_GAP_MS);
  ESP_LOGCONFIG(TAG, " long press : %ums", (unsigned) LONG_PRESS_MS);

#if BLE_HID_DEBUG
  ESP_LOGCONFIG(TAG, " debug : enabled");
#else
  ESP_LOGCONFIG(TAG, " debug : disabled");
#endif

#if BLE_HID_INCLUDE_FALLBACK_PAIR
  ESP_LOGCONFIG(TAG, " fallback pair : enabled (%u/%u)", FALLBACK_INPUT_HANDLE, FALLBACK_CCC_HANDLE);
#else
  ESP_LOGCONFIG(TAG, " fallback pair : disabled");
#endif
}

void BLEClientHID::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  (void) param;
  if (event == ESP_GAP_BLE_AUTH_CMPL_EVT) {
    // After auth, some remotes start accepting CCC writes reliably.
    load_cached_pairs_(this);
    enable_notifications_for_all_pairs_(this, "auth_complete", true);
  }
}

void BLEClientHID::read_client_characteristics() {}

void BLEClientHID::on_gatt_read_finished(GATTReadData *data) { (void) data; }

void BLEClientHID::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                       esp_ble_gattc_cb_param_t *param) {
  (void) gattc_if;

  switch (event) {
    case ESP_GATTC_CONNECT_EVT: {
      // Best-effort: request link encryption early.
      auto ret = esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
      if (ret) {
        ESP_LOGE(TAG, "[%d] [%s] esp_ble_set_encryption error, status=%d", this->parent()->get_connection_index(),
                 this->parent()->address_str(), ret);
      }
      break;
    }

    case ESP_GATTC_OPEN_EVT: {
      load_cached_pairs_(this);

      // Important for "first press after wake": try enabling quickly from cache.
      this->set_timeout("post_open_enable_fast", 80, [this]() {
        load_cached_pairs_(this);
        enable_notifications_for_all_pairs_(this, "post_open_fast", false);
      });

      // Retry after a short delay (lets the stack settle).
      this->set_timeout("post_open_enable", 600, [this]() {
        load_cached_pairs_(this);
        enable_notifications_for_all_pairs_(this, "post_open", false);
      });

      // One more retry, plus an optional CCC=0x0003 fallback if no traffic.
      this->set_timeout("ccc_retry", 2000, [this]() {
        load_cached_pairs_(this);
        enable_notifications_for_all_pairs_(this, "open_retry", false);

        auto &st = ble_state_by_instance[this];
        if (st.last_notify_ms == 0) {
          try_enable_ccc_both_bits_once_(this, "ccc_both_bits_fallback");
        }
      });

      this->node_state = espbt::ClientState::ESTABLISHED;
      break;
    }

    case ESP_GATTC_SEARCH_RES_EVT: {
      // Capture HID service handle range (0x1812).
      const auto &sr = param->search_res;

      // IMPORTANT: In ESP-IDF 5.5.x, sr.srvc_id is esp_gatt_id_t (NO ".id" member).
      if (sr.srvc_id.uuid.len == ESP_UUID_LEN_16 && sr.srvc_id.uuid.uuid.uuid16 == 0x1812) {
        auto &st = ble_state_by_instance[this];
        st.have_hid_range = true;
        st.hid_start = sr.start_handle;
        st.hid_end = sr.end_handle;

        DBG_LOGI("DBG HID service range: %u..%u", st.hid_start, st.hid_end);
      }
      break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      load_cached_pairs_(this);

      discover_notify_pairs_(this, "search_complete");
      enable_notifications_for_all_pairs_(this, "search_complete", false);
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "[%s] Disconnected!", this->parent()->address_str());
      this->status_set_warning("Disconnected");
      reset_ccc_state_(this);
      btn_state_by_instance[this] = InstanceButtons{};
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.conn_id != this->parent()->get_conn_id())
        break;

      auto &st = ble_state_by_instance[this];
      st.last_notify_ms = esphome::millis();

      const uint16_t h = param->notify.handle;
      const bool known = input_is_known_(this, h);

#if BLE_HID_DEBUG
      DBG_LOGI("DBG notify%s: handle=%u len=%u data=%s", known ? "" : "(unknown)", h,
               (unsigned) param->notify.value_len, bytes_hex(param->notify.value, param->notify.value_len).c_str());
#else
      (void) known;
#endif

      if (param->notify.value_len >= 2) {
        this->send_input_report_event(param);
      }

      break;
    }

    default:
      break;
  }
}

// -----------------------------------------------------------------------------
// Notify parsing + event emission
// -----------------------------------------------------------------------------
void BLEClientHID::send_input_report_event(esp_ble_gattc_cb_param_t *p_data) {
  if (p_data->notify.value_len < 2) {
    DBG_LOGW("HID notify too short: len=%u", (unsigned) p_data->notify.value_len);
    return;
  }

  uint16_t raw = ((uint16_t) p_data->notify.value[0] << 8) | (uint16_t) p_data->notify.value[1];
  const std::string raw_hex = hex4(raw);

  const char *remote = this->parent()->address_str();
  const std::string source = esphome::App.get_name();

  auto emit = [&](const std::string &action, int clicks, const std::string &raw_for_event) {
#ifdef USE_API
    std::map<std::string, std::string> data;
    data["action"] = action;
    data["raw"] = raw_for_event;
    data["clicks"] = std::to_string(clicks);
    data["remote"] = remote ? remote : "";
    data["source"] = source;
    this->fire_homeassistant_event("esphome.remote_action", data);
#endif

    if (this->last_event_usage_text_sensor != nullptr) {
      this->last_event_usage_text_sensor->publish_state(action);
    }
    if (this->last_event_value_sensor != nullptr) {
      this->last_event_value_sensor->publish_state(0.0f);
    }

    ESP_LOGI(TAG, "Remote action: %s remote=%s source=%s raw=%s clicks=%d", action.c_str(), remote ? remote : "",
             source.c_str(), raw_for_event.c_str(), clicks);
  };

  // Wheel events
  if (raw == 0x4000) {
    emit("rotate_right", -1, raw_hex);
    return;
  }
  if (raw == 0x8000) {
    emit("rotate_left", -1, raw_hex);
    return;
  }

  auto &inst = btn_state_by_instance[this];

  // Press (non-zero)
  ButtonId press_btn = raw_to_button_press(raw);
  if (press_btn != ButtonId::NONE) {
    inst.active_button = press_btn;
    auto &st = inst.st[(uint8_t) press_btn];
    st.is_down = true;
    st.long_fired = false;

    this->cancel_timeout(std::string("final_") + button_name(press_btn));
    emit(std::string(button_name(press_btn)) + "_pressed", -1, raw_hex);

    const std::string long_key = std::string("long_") + button_name(press_btn);
    this->cancel_timeout(long_key);

    this->set_timeout(long_key, LONG_PRESS_MS, [this, press_btn]() {
      auto &inst2 = btn_state_by_instance[this];
      auto &st2 = inst2.st[(uint8_t) press_btn];
      if (st2.is_down && !st2.long_fired) {
        st2.long_fired = true;
        st2.click_count = 0;

        const char *remote = this->parent()->address_str();
        const std::string source = esphome::App.get_name();
        std::string action = std::string(button_name(press_btn)) + "_long";

#ifdef USE_API
        std::map<std::string, std::string> data;
        data["action"] = action;
        data["raw"] = "";
        data["clicks"] = "-1";
        data["remote"] = remote ? remote : "";
        data["source"] = source;
        this->fire_homeassistant_event("esphome.remote_action", data);
#endif

        if (this->last_event_usage_text_sensor != nullptr) {
          this->last_event_usage_text_sensor->publish_state(action);
        }
        ESP_LOGI(TAG, "Remote action: %s remote=%s source=%s raw= clicks=-1", action.c_str(), remote ? remote : "",
                 source.c_str());
      }
    });

    return;
  }

  // Release (0x0000)
  if (raw == 0x0000) {
    if (inst.active_button == ButtonId::NONE)
      return;

    ButtonId rb = inst.active_button;
    inst.active_button = ButtonId::NONE;

    auto &st = inst.st[(uint8_t) rb];
    st.is_down = false;

    emit(std::string(button_name(rb)) + "_released", -1, raw_hex);

    if (st.long_fired) {
      st.long_fired = false;
      st.click_count = 0;
      return;
    }

    if (st.click_count < 3)
      st.click_count++;

    const std::string final_key = std::string("final_") + button_name(rb);
    this->cancel_timeout(final_key);

    this->set_timeout(final_key, MULTIPRESS_GAP_MS, [this, rb]() {
      auto &inst2 = btn_state_by_instance[this];
      auto &st2 = inst2.st[(uint8_t) rb];

      if (st2.is_down || st2.long_fired || st2.click_count == 0)
        return;

      std::string action;
      if (st2.click_count == 1)
        action = std::string(button_name(rb)) + "_single";
      else if (st2.click_count == 2)
        action = std::string(button_name(rb)) + "_double";
      else
        action = std::string(button_name(rb)) + "_triple";

      const char *remote = this->parent()->address_str();
      const std::string source = esphome::App.get_name();

#ifdef USE_API
      std::map<std::string, std::string> data;
      data["action"] = action;
      data["raw"] = "";
      data["clicks"] = std::to_string(st2.click_count);
      data["remote"] = remote ? remote : "";
      data["source"] = source;
      this->fire_homeassistant_event("esphome.remote_action", data);
#endif

      if (this->last_event_usage_text_sensor != nullptr) {
        this->last_event_usage_text_sensor->publish_state(action);
      }
      ESP_LOGI(TAG, "Remote action: %s remote=%s source=%s raw= clicks=%d", action.c_str(), remote ? remote : "",
               source.c_str(), st2.click_count);

      st2.click_count = 0;
    });

    return;
  }

  // Unknown raw - still emit for visibility
  emit(std::string("raw_") + raw_hex, -1, raw_hex);
}

// -----------------------------------------------------------------------------
// Registration helpers for sensors/text sensors (used by ESPHome YAML platforms)
// -----------------------------------------------------------------------------
void BLEClientHID::register_last_event_value_sensor(sensor::Sensor *last_event_value_sensor) {
  this->last_event_value_sensor = last_event_value_sensor;
}

void BLEClientHID::register_battery_sensor(sensor::Sensor *battery_sensor) { this->battery_sensor = battery_sensor; }

void BLEClientHID::register_last_event_usage_text_sensor(text_sensor::TextSensor *last_event_usage_text_sensor) {
  this->last_event_usage_text_sensor = last_event_usage_text_sensor;
}

// -----------------------------------------------------------------------------
// Unused API hooks (kept for compatibility with existing header / component)
// -----------------------------------------------------------------------------
void BLEClientHID::schedule_read_char(ble_client::BLECharacteristic *characteristic) { (void) characteristic; }

uint8_t *BLEClientHID::parse_characteristic_data(ble_client::BLEService *service, uint16_t uuid) {
  (void) service;
  (void) uuid;
  return nullptr;
}

void BLEClientHID::configure_hid_client() {
  // Not needed (handle-based / discovery-based).
}

}  // namespace ble_client_hid
}  // namespace esphome

#endif  // USE_ESP32
