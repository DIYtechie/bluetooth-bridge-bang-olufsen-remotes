#pragma once

// This header is intentionally empty. Older versions exposed large HID usage
// lookup tables here, but the current component parses and emits numeric usage
// values directly. Keeping those tables as global std::map instances made
// memory-constrained ESP32 builds vulnerable to boot-time bad_alloc failures.
