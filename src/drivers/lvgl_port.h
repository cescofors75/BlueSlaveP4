// =============================================================================
// lvgl_port.h — LVGL display/touch integration for ESP32-P4
// Zero-copy double-buffer, vsync sync, FreeRTOS task
// =============================================================================
#pragma once

#include <lvgl.h>

// Initialize LVGL display driver + touch input device.
// Call after display_init() returns successfully.
void lvgl_port_init(void);

// No-op (LVGL now runs in dedicated FreeRTOS task)
void lvgl_port_update(void);

// Get the touch input device (for screen callbacks)
lv_indev_t* lvgl_port_get_touch_indev(void);

// Return the strongest currently active touch mapped to MIDI velocity (40..127).
// Returns 0 when no finger is currently active.
uint8_t lvgl_port_get_touch_velocity(void);

// Return the MIDI velocity (40..127) of the active touch point closest to (x,y)
// within the given pixel radius. Lets per-key callbacks read the velocity of
// the specific finger that pressed them instead of the global maximum.
// Returns 0 when no touch is inside the radius.
uint8_t lvgl_port_get_touch_velocity_at(lv_coord_t x, lv_coord_t y, lv_coord_t radius);

// Thread safety — wrap LVGL API calls from outside the LVGL task
bool lvgl_port_lock(int timeout_ms);
void lvgl_port_unlock(void);

// Start LVGL task (call after UI setup is complete)
void lvgl_port_task_start(void);
