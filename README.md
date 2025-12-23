Bluetooth bridge for Bang & Olufsen remotes
=================================================
An ESPHome external component that connects an ESP32 to **Bang & Olufsen remotes** over Bluetooth Low Energy (BLE HID) and emits clean, automation-friendly **Home Assistant events**.

This project is built for *smooth, low-latency control* (think: dimming lights, adjusting volume, scrubbing scenes) with a remote that feels genuinely premium: **fast, precise, and consistent**.

> **Credit / upstream:** This project is a fork and extension of an existing ESPHome BLE HID client component. Huge thanks to the original author(s) and contributors for making a solid base to build on. This fork keeps the same overall approach (ESPHome BLE client + HID notifications) but adds B&O-remote-specific fixes and decoding to make it reliable and automation-friendly.

Feature highlights
------------------

- **Low latency, “feels native”**: BeoSound Essence Remote is a genuinely premium remote with precise output. Volume/brightness control feels smooth and immediate.
- **Wheel events** suitable for dimming/volume.
- **Action events** for buttons (press/release + single/double/triple/long).
- **ESPHome → Home Assistant events** emitted directly from the ESP for easy automation.

Why this fork exists
--------------------

B&O remotes (notably the **BeoSound Essence Remote**) are extremely responsive, but they also **sleep** aggressively. A common issue with the upstream HID approach is that the **first button press after the remote wakes** can be missed. In practice, this happens when the remote wakes and sends its first HID report **before notifications are fully enabled** on the ESP32 side (a race between “remote sends report” and “ESP has finished CCC/notify subscription”).

This fork focuses on *capturing the very first report* reliably by:

- **Enabling notifications as early as possible** (CCC write) when the BLE connection opens.
- **Re-enabling notifications after pairing/auth completes** (some remotes / stacks reset CCC after encryption/auth).
- **Re-enabling notifications after service discovery** (to cover stack timing differences and reconnection paths).
- Keeping the logic lightweight so it doesn’t add latency or destabilize BLE.

In other words: upstream typically enables notifications at one “normal” point in the connection flow; this fork deliberately **covers multiple points in the lifecycle** to avoid the “first press lost on wake” race condition.

What’s different from upstream
------------------------------

This fork is intentionally targeted at Bang & Olufsen remotes and adds three major things:

1) **B&O-specific decoding (“translators”)**
- The remote sends HID input reports that need to be interpreted into human-friendly actions.
- This fork includes specific mappings for button presses and wheel direction, producing actions like:
  - `up_pressed`, `up_released`, `up_single`, `up_double`, `up_triple`, `up_long`
  - `rotate_left`, `rotate_right`

2) **Multi-press & long-press logic computed on the ESP**
- The remote does not necessarily send dedicated “double click” or “triple click” commands.
- This fork implements **timers and state tracking on-device** to generate multi-click and long-press events:
  - Single / double / triple click detection
  - Long press detection
- That means one physical button can control multiple functions in your automations without extra software layers.

3) **More robust notification subscription timing**
- As described above, the fork adds additional hooks and retries around CCC/notify enabling to avoid missing the first report when the remote wakes.

## Supported devices

✅ **Verified**
- Bang & Olufsen **BeoSound Essence Remote**

🧪 **Planned / Community testing welcome**
- Other B&O BLE remotes (if they expose HID notifications similarly)

If you try a different remote, please open an issue and include:
- ESPHome logs (INFO/WARN around connection + notifications)
- The BLE MAC address format you used
- What actions you see (if any)

## Requirements

### Hardware
- Any ESP32 capable of BLE client mode  
  - ✅ ESP32-C3 (tested)  
  - ✅ ESP32 DevKit (tested)

### Firmware / framework
- **ESP-IDF is required**
  - This project is intended to run with the ESP-IDF framework for BLE stability and compatibility.

### Home Assistant integration
- ESPHome `api:` must be enabled
- **`homeassistant_services: true` must be enabled**
  - Required to emit Home Assistant events from the device firmware

### The bluetooth MAC adress of your remote
To find a Bluetooth remote’s MAC address on a Mac, first pair the remote with your Mac. Then open System Information (Option-click the Apple menu → System Information), go to Bluetooth, locate the remote in the device list, and read the Address field—this is the remote’s MAC address. Afterwards, remove it by choosing Forget This Device for the remote in Bluetooth settings.

On Windows PC, pair the remote, then open Control Panel → Devices and Printers, right-click the device → Properties and look for Bluetooth address (or similar). 

On Android, pair the remote, then go to Settings → Connected devices / Bluetooth, tap the gear/info icon next to the device, and look for the device address (if shown); if it isn’t displayed in the Bluetooth UI, you can typically see it using a Bluetooth scanner app that lists paired devices and their addresses.

## Installation (ESPHome external_components)

In your ESPHome YAML:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/DIYtechie/bluetooth-bridge-bang-olufsen-remotes
      ref: master
    components: [ ble_client_hid ]
```

> If you prefer pinning to a stable version later, replace `ref: master` with a tag like `ref: v0.1.0`.


## Example YAML (BeoSound Essence Remote)

Example (copy/paste):

```yaml
substitutions:
  # Device identity shown in ESPHome + Home Assistant
  device_name: "b-o-remote-bridge"
  friendly_name: "Remote Bridge"

  # Remote 1 (enabled by default)
  remote_1_mac: "84:EB:18:07:DD:20" # <- Replace with your remote's BLE MAC address

  # Remote 2 (optional - enable by uncommenting below + the ble_client + ble_client_hid blocks)
  #remote_2_mac: "84:EB:18:07:DD:26" # <- Replace with your 2nd remote's BLE MAC address

esphome:
  name: ${device_name}
  friendly_name: ${friendly_name}
  comment: >
    ESPHome BLE client for Bang & Olufsen Remotes (HID over BLE).
    Parses button + wheel actions and emits Home Assistant events.

esp32:
  board: esp32-c3-devkitm-1 # <- Adjust to match your hardware.
  framework:
    type: esp-idf # This project REQUIRES the ESP-IDF framework for BLE stability/compatibility.

logger:
  # Keep INFO for component logs, but reduce BLE stack noise.
  level: INFO
  logs:
    esp32_ble: ERROR
    esp32_ble_client: ERROR
    ble_client_hid: INFO
    hid_parser: WARN

api:
  # Recommended for best practice. Does not meaningfully affect event latency.
  encryption:
    key: !secret api_encryption_key

  # REQUIRED: needed for fire_homeassistant_event() in the custom component.
  homeassistant_services: true

  # Optional: avoid rebooting if Home Assistant is temporarily unavailable.
  reboot_timeout: 0s

ota:
  - platform: esphome
    password: !secret ota_password

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

  # Optional: can improve reliability on some setups (less Wi-Fi power saving jitter).
  power_save_mode: none

# NOTE:
# This bridge can connect to multiple remotes at the same time.
# Set esp32_ble.max_connections to match the number of remotes you enable below (1-3 recommended).
esp32_ble:
  # Keep the BLE stack lightweight and quiet.
  disable_bt_logs: true
  max_connections: 1          # <-- Set this to the number of remotes you use
  max_notifications: 64

external_components:
  - source:
      type: git
      url: https://github.com/DIYtechie/bluetooth-bridge-bang-olufsen-remotes
      ref: master
    components: [ ble_client_hid ]

esp32_ble_tracker:
  scan_parameters:
    active: false
    interval: 320ms
    window: 60ms

# ----------------------------
# BLE clients (2 remotes)
# ----------------------------
ble_client:
  # Remote 1
  - id: remote_1
    mac_address: ${remote_1_mac}
    auto_connect: true
    on_connect:
      then:
        - logger.log: "${friendly_name} remote_1 connected"
    on_disconnect:
      then:
        - logger.log: "${friendly_name} remote_1 disconnected"

  # Remote 2 (optional)
  # - id: remote_2
  #   mac_address: ${remote_2_mac}
  #   auto_connect: true
  #   on_connect:
  #     then:
  #       - logger.log: "${friendly_name} remote_2 connected"
  #   on_disconnect:
  #     then:
  #       - logger.log: "${friendly_name} remote_2 disconnected"

ble_client_hid:
  # HID bridge for Remote 1
  - id: remote_1_hid
    ble_client_id: remote_1

  # HID bridge for Remote 2 (optional)
  # - id: remote_2_hid
  #   ble_client_id: remote_2


```

---

## Home Assistant events

This component emits a Home Assistant event for each action:

- **Event type:** `esphome.beosound_action`

### Typical actions

Buttons:
- `up_pressed`, `up_released`, `up_single`, `up_double`, `up_triple`, `up_long`
- `down_pressed`, ...
- `left_*`, `right_*`

Wheel:
- `rotate_left`
- `rotate_right`

### Event data

The event includes:
- action name
- raw HID value (hex)
- click count (for single/double/triple; wheel uses `-1`)

> Tip: In Node-RED, it’s common to route on `event_type` + `event.action`, and optionally rate-limit wheel events (e.g. 50ms) if your downstream devices can’t keep up.

---

## Pairing / Resetting the remote

If you reset the remote or it stops sending events:
1. Ensure the ESP32 is running and scanning/connecting.
2. Put the remote into pairing mode (varies by model/firmware) (on BeoSound Essense remote hold down the up (empty circle) button until you see the green light above the circle).
3. Watch ESPHome logs to confirm:
   - BLE connection opens
   - Notifications are enabled (CCC write)

> If pairing mode steps differ across versions, open an issue with your remote revision and what worked.

## Troubleshooting

### “CCC write done … enable notify”
This is expected: it enables BLE HID notifications so the remote can push button/wheel reports.

### BLE warnings like “get_descr_by_char_handle error” or “unexpected GAP event”
Some ESP-IDF BLE stacks can produce noisy warnings during discovery/auth.  
If events are working reliably, you can usually ignore them or reduce log verbosity.

### No events, but connection succeeds
- Confirm you're using **ESP-IDF**
- Confirm `homeassistant_services: true` under `api:`
- Confirm the MAC address is correct
- Try slightly more aggressive scan parameters (`interval/window`) if you miss the first press after wakeup


## Roadmap (tentative)

- Add more verified B&O remote profiles (community testing)
- Improve docs + diagrams
- Publish a stable tagged release once the interface is finalized
- Add optional wheel rate limiting on-device (configurable)


## License

Same license as upstream (see `LICENSE`).
