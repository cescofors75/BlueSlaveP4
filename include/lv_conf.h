// =============================================================================
// BlueSlaveP4 - lv_conf.h
// LVGL 9.2.x for Guition ESP32-P4 JC1060P470C (1024×600 MIPI-DSI)
//
// Minimal config: any option not set here falls back to the LVGL default in
// lv_conf_internal.h. Only project-relevant knobs are overridden.
// =============================================================================
#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

// =============================================================================
// COLOR
// =============================================================================
#define LV_COLOR_DEPTH     16    // RGB565 — display color format set in code

// =============================================================================
// MEMORY — custom allocator routes every LVGL allocation to PSRAM (32MB OPI).
// LVGL 9 dropped LV_MEM_CUSTOM; the equivalent is LV_STDLIB_CUSTOM, which lets
// us implement lv_malloc_core/lv_realloc_core/lv_free_core ourselves. Those
// live in src/drivers/lvgl_mem_psram.c and call heap_caps_*(MALLOC_CAP_SPIRAM),
// so LVGL shares the same dynamic PSRAM heap as the MIDI/audio code (no fixed
// pool, no cap — same behaviour as the old LV_MEM_CUSTOM setup).
// =============================================================================
#define LV_USE_STDLIB_MALLOC    LV_STDLIB_CUSTOM
#define LV_USE_STDLIB_STRING    LV_STDLIB_CLIB     // std memcpy/memset (was LV_MEMCPY_MEMSET_STD)
#define LV_USE_STDLIB_SPRINTF   LV_STDLIB_BUILTIN

// =============================================================================
// HAL — refresh period. Tick source is provided in code via lv_tick_set_cb()
// (LVGL 9 removed LV_TICK_CUSTOM); see lvgl_port_init().
// =============================================================================
#define LV_DEF_REFR_PERIOD 8     // ~120Hz refresh cap
#define LV_DPI_DEF         130

// =============================================================================
// OS — LVGL runs single-threaded inside a dedicated FreeRTOS task guarded by
// our own mutex (lvgl_port_lock/unlock), so LVGL's internal locking stays off.
// =============================================================================
#define LV_USE_OS   LV_OS_NONE

// =============================================================================
// PERFORMANCE / MONITORS
// =============================================================================
#define LV_USE_SYSMON         0
#define LV_USE_PERF_MONITOR   0
#define LV_USE_MEM_MONITOR    0

// =============================================================================
// FONTS
// =============================================================================
#define LV_FONT_MONTSERRAT_10  1
#define LV_FONT_MONTSERRAT_12  1
#define LV_FONT_MONTSERRAT_14  1
#define LV_FONT_MONTSERRAT_16  1
#define LV_FONT_MONTSERRAT_18  1
#define LV_FONT_MONTSERRAT_20  1
#define LV_FONT_MONTSERRAT_22  1
#define LV_FONT_MONTSERRAT_24  1
#define LV_FONT_MONTSERRAT_28  1
#define LV_FONT_MONTSERRAT_32  1
#define LV_FONT_MONTSERRAT_40  1
#define LV_FONT_DEFAULT &lv_font_montserrat_14

// =============================================================================
// WIDGETS  (LVGL 9 renames: BTN→BUTTON, BTNMATRIX→BUTTONMATRIX, IMG→IMAGE)
// =============================================================================
#define LV_USE_ARC          1
#define LV_USE_BAR          1
#define LV_USE_BUTTON       1
#define LV_USE_BUTTONMATRIX 1
#define LV_USE_CANVAS       0
#define LV_USE_CHECKBOX     1
#define LV_USE_DROPDOWN     1
#define LV_USE_IMAGE        1
#define LV_USE_LABEL        1
#define LV_USE_LINE         1
#define LV_USE_ROLLER       0
#define LV_USE_SLIDER       1
#define LV_USE_SWITCH       1
#define LV_USE_TABLE        1

#define LV_USE_LED          1
#define LV_USE_LIST         1
#define LV_USE_MSGBOX       1
#define LV_USE_SPINNER      1

// =============================================================================
// LAYOUT
// =============================================================================
#define LV_USE_FLEX       1
#define LV_USE_GRID       1

// =============================================================================
// THEME
// =============================================================================
#define LV_USE_THEME_DEFAULT    1
#define LV_THEME_DEFAULT_DARK   1

// =============================================================================
// DEBUG
// =============================================================================
#define LV_USE_ASSERT_NULL      1
#define LV_USE_ASSERT_MALLOC    1   // was LV_USE_ASSERT_MEM in LVGL 8
#define LV_USE_LOG              0

#endif // LV_CONF_H
