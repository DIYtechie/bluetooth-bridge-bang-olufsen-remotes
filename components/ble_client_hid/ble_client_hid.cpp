#include "ble_client_hid.h"
#include "usages.h"

#ifdef USE_ESP32

#include <array>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ble_client_hid {

static const char *const TAG = "ble_client_hid";

// -----------------------------------------------------------------------------
// BeoSound Essence Remote: HID input/CCC handles (measured / known working).
// If you want to support other remotes, these may need to be discovered/configured.
// -----------------------------------------------------------------------------
static constexpr uint16_t HID_INPUT_HANDLE = 62;
static constexpr uint16_t HID_CCC_HANDLE = 63;

// -----------------------------------------------------------------------------
// Multi-press timing (device-side interpretation)
// -----------------------------------------------------------------------------
static constexpr uint32_t MULTIPRESS_GAP_MS = 400;   // single vs double vs triple
static constexpr uint32_t LONG_PRESS_MS = 1500;      // long press threshold

// -----------------------------------------------------------------------------
// CCC/notify re-enable guard: prevents CCC spam during normal operation.
// -----------------------------------------------------------------------------
static constexpr uint32_t CCC_MIN_INTERVAL_MS = 5000;  // min time between CCC attempts once enabled

struct CccState {
  bool enabled{false};
  uint32_t last_attempt_ms{0};
};

static std::map<const BLEClientHID *, CccState> ccc_state_by_instance;

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
  ButtonId active_button{ButtonId::NONE};  // which button is currently considered "down"
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
  // Press codes observed from the BeoSound Essence Remote:
  // 0x0006 = Up press
  // 0x0001 = Down press
  // 0x000B = Left press
  // 0x000A = Right press
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

static bool is_wheel_event(uint16_t raw) {
  // Wheel events observed from the BeoSound Essence Remote:
  // 0x4000 = rotate right
  // 0x8000 = rotate left
  return raw == 0x4000 || raw == 0x8000;
}

// -----------------------------------------------------------------------------
// CCC write + register-for-notify (guarded)
// -----------------------------------------------------------------------------
static void reset_ccc_state_(BLEClientHID *self) {
  auto &s = ccc_state_by_instance[self];
  s.enabled = false;
  s.last_attempt_ms = 0;
}

static void write_ccc_enable_notify_(BLEClientHID *self, const char *reason, bool force = false) {
  auto &s = ccc_state_by_instance[self];

  const uint32_t now = esphome::millis();

  // If CCC is already enabled, avoid re-writing too often (prevents spam during wheel rotation).
  if (!force && s.enabled && (now - s.last_attempt_ms) < CCC_MIN_INTERVAL_MS) {
    return;
  }

  s.last_attempt_ms = now;

  // 0x0001 = notifications enabled
  uint8_t ccc_value[2] = {0x01, 0x00};

  esp_err_t r = esp_ble_gattc_write_char_descr(self->parent()->get_gattc_if(), self->parent()->get_conn_id(),
                                              HID_CCC_HANDLE, sizeof(ccc_value), ccc_value, ESP_GATT_WRITE_TYPE_RSP,
                                              ESP_GATT_AUTH_REQ_NONE);

  if (r == ESP_OK) {
    ESP_LOGI(TAG, "CCC write done (handle %u) to enable notify (%s)", HID_CCC_HANDLE, reason);
    s.enabled = true;
  } else {
    ESP_LOGW(TAG, "CCC write failed (handle %u) err=%d (%s)", HID_CCC_HANDLE, (int) r, reason);
    // keep s.enabled as-is; next forced attempt may re-try
  }

  // Register-for-notify helps ensure callbacks are routed correctly in many setups.
  esp_err_t rn = esp_ble_gattc_register_for_notify(self->parent()->get_gattc_if(), self->parent()->get_remote_bda(),
                                                  HID_INPUT_HANDLE);

  if (rn != ESP_OK) {
    ESP_LOGW(TAG, "register_for_notify failed for handle %u err=%d (%s)", HID_INPUT_HANDLE, (int) rn, reason);
  } else {
    ESP_LOGI(TAG, "Fast notify registration started for HID handle %u (%s)", HID_INPUT_HANDLE, reason);
  }
}

// -----------------------------------------------------------------------------
// Component implementation
// -----------------------------------------------------------------------------
void BLEClientHID::loop() {
  // No periodic work required.
  // We react to BLE events (connect/auth/discovery/notify).
}

void BLEClientHID::dump_config() {
  ESP_LOGCONFIG(TAG, "BLE Client HID (BeoSound Essence Remote):");
  ESP_LOGCONFIG(TAG, "  MAC address      : %s", this->parent()->address_str().c_str());
  ESP_LOGCONFIG(TAG, "  HID input handle : %u", HID_INPUT_HANDLE);
  ESP_LOGCONFIG(TAG, "  HID CCC handle   : %u", HID_CCC_HANDLE);
  ESP_LOGCONFIG(TAG, "  multi-press gap  : %ums", (unsigned) MULTIPRESS_GAP_MS);
  ESP_LOGCONFIG(TAG, "  long press       : %ums", (unsigned) LONG_PRESS_MS);
}

void BLEClientHID::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  (void) param;

  // We primarily care about authentication completing (encryption/bonding),
  // because some devices require CCC to be written after auth.
  if (event == ESP_GAP_BLE_AUTH_CMPL_EVT) {
    ESP_LOGI(TAG, "GAP auth complete -> re-enabling notifications (CCC)");
    // Force is OK here (happens once per connection).
    write_ccc_enable_notify_(this, "auth_complete", true);
  }
}

void BLEClientHID::read_client_characteristics() {
  // Not used in this simplified, handle-based implementation.
}

void BLEClientHID::on_gatt_read_finished(GATTReadData *data) {
  (void) data;
  // Not used in this simplified, handle-based implementation.
}

void BLEClientHID::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                       esp_ble_gattc_cb_param_t *param) {
  (void) gattc_if;

  switch (event) {
    case ESP_GATTC_CONNECT_EVT: {
      // Trigger encryption; required by many HID devices.
      auto ret = esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
      if (ret) {
        ESP_LOGE(TAG, "[%d] [%s] esp_ble_set_encryption error, status=%d", this->parent()->get_connection_index(),
                 this->parent()->address_str().c_str(), ret);
      }
      break;
    }

    case ESP_GATTC_OPEN_EVT: {
      // Enable notifications as early as possible.
      write_ccc_enable_notify_(this, "open", true);

      // Optional one-shot retry shortly after open (covers timing edge cases),
      // guarded so it won't spam later.
      this->set_timeout("essence_ccc_retry", 500, [this]() { write_ccc_enable_notify_(this, "open_retry", false); });

      // Mark as established; we don't need full HID discovery for this remote.
      this->node_state = espbt::ClientState::ESTABLISHED;
      break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      // Service discovery completed (ESPHome BLE client may still do discovery).
      // Re-enable notifications once (guard prevents spam).
      ESP_LOGI(TAG, "Service discovery complete -> re-enabling notifications (CCC)");
      write_ccc_enable_notify_(this, "search_complete", false);
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "[%s] Disconnected!", this->parent()->address_str().c_str());
      this->status_set_warning("Disconnected");

      reset_ccc_state_(this);

      // Reset button state on disconnect.
      auto &inst = btn_state_by_instance[this];
      inst = InstanceButtons{};

      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.conn_id != this->parent()->get_conn_id())
        break;

      if (param->notify.handle != HID_INPUT_HANDLE) {
        // Ignore other notifications.
        break;
      }

      this->send_input_report_event(param);
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
    ESP_LOGW(TAG, "HID notify too short: len=%d", p_data->notify.value_len);
    return;
  }

  // Remote sends big-endian 16-bit values: [hi][lo]
  uint16_t raw = ((uint16_t) p_data->notify.value[0] << 8) | (uint16_t) p_data->notify.value[1];
  const std::string raw_hex = hex4(raw);

  // Helper to emit HA event + update debug sensors
  auto emit = [&](const std::string &action, int clicks) {
#ifdef USE_API
    // NOTE: ESPHome CustomAPIDevice only supports map<string,string> for event data.
    // Keeping it flat is the most compatible approach.
    std::map<std::string, std::string> data;
    data["payload"] = action;  // convenient for Node-RED / automations
    data["action"] = action;
    data["raw"] = raw_hex;
    data["clicks"] = std::to_string(clicks);
    this->fire_homeassistant_event("esphome.beosound_action", data);
#endif

    // Optional debug helpers (show last action + value)
    if (this->last_event_usage_text_sensor != nullptr) {
      this->last_event_usage_text_sensor->publish_state(action);
    }
    if (this->last_event_value_sensor != nullptr) {
      // For wheel/unknown: publish 0; for button press/release we can publish 1/0 if desired.
      // Here we publish 1 for actions that end with "_pressed" and 0 otherwise.
      float v = 0.0f;
      if (action.size() >= 8 && action.rfind("_pressed") == action.size() - 8)
        v = 1.0f;
      this->last_event_value_sensor->publish_state(v);
    }

    ESP_LOGI(TAG, "BeoSound action: %s raw=%s clicks=%d", action.c_str(), raw_hex.c_str(), clicks);
  };

  // ---------------------------------------------------------------------------
  // Wheel events (stateless) - do NOT affect button press/release/multipress.
  // ---------------------------------------------------------------------------
  if (raw == 0x4000) {
    emit("rotate_right", -1);
    return;
  }
  if (raw == 0x8000) {
    emit("rotate_left", -1);
    return;
  }

  // ---------------------------------------------------------------------------
  // Button events
  // Press codes are non-zero; release is 0x0000.
  // We maintain an "active_button" to bind releases to the right button.
  // ---------------------------------------------------------------------------
  auto &inst = btn_state_by_instance[this];

  // PRESS?
  ButtonId press_btn = raw_to_button_press(raw);
  if (press_btn != ButtonId::NONE) {
    // Mark active button and state
    inst.active_button = press_btn;
    auto &st = inst.st[(uint8_t) press_btn];
    st.is_down = true;
    st.long_fired = false;

    // Cancel any pending finalize from previous click bursts for this button
    this->cancel_timeout(std::string("essence_final_") + button_name(press_btn));

    // Emit pressed
    emit(std::string(button_name(press_btn)) + "_pressed", -1);

    // Start long-press timer (member access OK)
    const std::string long_key = std::string("essence_long_") + button_name(press_btn);
    this->cancel_timeout(long_key);
    this->set_timeout(long_key, LONG_PRESS_MS, [this, press_btn]() {
      auto &inst2 = btn_state_by_instance[this];
      auto &st2 = inst2.st[(uint8_t) press_btn];
      if (st2.is_down && !st2.long_fired) {
        st2.long_fired = true;
        st2.click_count = 0;
        // emit long
        std::string action = std::string(button_name(press_btn)) + "_long";
        // raw for long doesn't matter much; we keep last seen raw from notify in outer scope,
        // but this lambda runs later, so we don't reuse raw here.
        // Use action only; raw debug will still show last raw received on next notify.
#ifdef USE_API
        std::map<std::string, std::string> data;
        data["payload"] = action;
        data["action"] = action;
        data["raw"] = "";  // unknown at timer time
        data["clicks"] = "-1";
        this->fire_homeassistant_event("esphome.beosound_action", data);
#endif
        if (this->last_event_usage_text_sensor != nullptr) {
          this->last_event_usage_text_sensor->publish_state(action);
        }
        ESP_LOGI(TAG, "BeoSound action: %s raw=<timer> clicks=-1", action.c_str());
      }
    });

    return;
  }

  // RELEASE?
  if (raw == 0x0000) {
    // Only treat as release if we have an active button down.
    if (inst.active_button == ButtonId::NONE) {
      // Some devices may emit 0x0000 without a preceding press; ignore.
      return;
    }

    ButtonId rb = inst.active_button;
    inst.active_button = ButtonId::NONE;

    auto &st = inst.st[(uint8_t) rb];
    st.is_down = false;

    // Emit released
    emit(std::string(button_name(rb)) + "_released", -1);

    // If long fired, do not count clicks.
    if (st.long_fired) {
      st.long_fired = false;
      st.click_count = 0;
      return;
    }

    // Count clicks (max 3)
    if (st.click_count < 3)
      st.click_count++;

    // Start/Restart finalize timer
    const std::string final_key = std::string("essence_final_") + button_name(rb);
    this->cancel_timeout(final_key);
    this->set_timeout(final_key, MULTIPRESS_GAP_MS, [this, rb]() {
      auto &inst2 = btn_state_by_instance[this];
      auto &st2 = inst2.st[(uint8_t) rb];

      if (st2.is_down || st2.long_fired || st2.click_count == 0)
        return;

      std::string action;
      if (st2.click_count == 1) {
        action = std::string(button_name(rb)) + "_single";
      } else if (st2.click_count == 2) {
        action = std::string(button_name(rb)) + "_double";
      } else {
        action = std::string(button_name(rb)) + "_triple";
      }

      // emit finalize event
#ifdef USE_API
      std::map<std::string, std::string> data;
      data["payload"] = action;
      data["action"] = action;
      data["raw"] = "";  // timer context
      data["clicks"] = std::to_string(st2.click_count);
      this->fire_homeassistant_event("esphome.beosound_action", data);
#endif
      if (this->last_event_usage_text_sensor != nullptr) {
        this->last_event_usage_text_sensor->publish_state(action);
      }
      ESP_LOGI(TAG, "BeoSound action: %s raw=<timer> clicks=%d", action.c_str(), st2.click_count);

      st2.click_count = 0;
    });

    return;
  }

  // Unknown non-zero non-wheel code: emit a raw action (optional)
  emit(std::string("raw_") + raw_hex, -1);
}

// -----------------------------------------------------------------------------
// Registration helpers for sensors/text sensors (used by ESPHome YAML platforms)
// -----------------------------------------------------------------------------
void BLEClientHID::register_last_event_value_sensor(sensor::Sensor *last_event_value_sensor) {
  this->last_event_value_sensor = last_event_value_sensor;
}

void BLEClientHID::register_battery_sensor(sensor::Sensor *battery_sensor) {
  this->battery_sensor = battery_sensor;
}

void BLEClientHID::register_last_event_usage_text_sensor(text_sensor::TextSensor *last_event_usage_text_sensor) {
  this->last_event_usage_text_sensor = last_event_usage_text_sensor;
}

// -----------------------------------------------------------------------------
// Unused API hooks (kept for compatibility with existing header / component)
// -----------------------------------------------------------------------------
void BLEClientHID::schedule_read_char(ble_client::BLECharacteristic *characteristic) {
  (void) characteristic;
}

uint8_t *BLEClientHID::parse_characteristic_data(ble_client::BLEService *service, uint16_t uuid) {
  (void) service;
  (void) uuid;
  return nullptr;
}

void BLEClientHID::configure_hid_client() {
  // Not needed in handle-based mode.
}

}  // namespace ble_client_hid
}  // namespace esphome

#endif  // USE_ESP32
