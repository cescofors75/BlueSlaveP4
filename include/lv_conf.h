// =============================================================================
// BlueSlaveP4 - lv_conf.h
// LVGL 8.4.x for Guition ESP32-P4 JC1060P470C (1024×600 MIPI-DSI)
// =============================================================================
#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

// =============================================================================
// COLOR
// =============================================================================
#define LV_COLOR_DEPTH     16
#define LV_COLOR_16_SWAP   0

// =============================================================================
// MEMORY — use PSRAM (32MB OPI) for LVGL allocations
// =============================================================================
#define LV_MEM_CUSTOM      1
#if LV_MEM_CUSTOM
    #define LV_MEM_CUSTOM_INCLUDE   <esp_heap_caps.h>
    #define LV_MEM_CUSTOM_ALLOC(size)     heap_caps_malloc(size, MALLOC_CAP_SPIRAM)
    #define LV_MEM_CUSTOM_FREE            heap_caps_free
    #define LV_MEM_CUSTOM_REALLOC(p,size) heap_caps_realloc(p, size, MALLOC_CAP_SPIRAM)
#endif
#define LV_MEMCPY_MEMSET_STD  1

// =============================================================================
// DISPLAY
// =============================================================================
#define LV_HOR_RES_MAX     1024
#define LV_VER_RES_MAX     1024
#define LV_DPI_DEF         130

// =============================================================================
// PERFORMANCE — P4 has 400MHz RISC-V, can push harder
// =============================================================================
#define LV_DISP_DEF_REFR_PERIOD 8    // ~120Hz LVGL tick
#define LV_INDEV_DEF_READ_PERIOD 10  // 100Hz touch
#define LV_USE_PERF_MONITOR   0
#define LV_USE_MEM_MONITOR    0

// =============================================================================
// TICK
// =============================================================================
#define LV_TICK_CUSTOM     1
#if LV_TICK_CUSTOM
    #define LV_TICK_CUSTOM_INCLUDE  <Arduino.h>
    #define LV_TICK_CUSTOM_SYS_TIME_EXPR  (millis())
#endif

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
#define LV_FONT_MONTSERRAT_30  0
#define LV_FONT_MONTSERRAT_32  1
#define LV_FONT_MONTSERRAT_36  0
#define LV_FONT_MONTSERRAT_40  1
#define LV_FONT_MONTSERRAT_48  0
#define LV_FONT_DEFAULT &lv_font_montserrat_14

// =============================================================================
// WIDGETS
// =============================================================================
#define LV_USE_ARC        1
#define LV_USE_BAR        1
#define LV_USE_BTN        1
#define LV_USE_BTNMATRIX  1
#define LV_USE_CANVAS     0
#define LV_USE_CHECKBOX   1
#define LV_USE_DROPDOWN   1
#define LV_USE_IMG        1
#define LV_USE_LABEL      1
#define LV_USE_LINE       1
#define LV_USE_ROLLER     0
#define LV_USE_SLIDER     1
#define LV_USE_SWITCH     1
#define LV_USE_TEXTAREA   0
#define LV_USE_TABLE      1

// Extra widgets
#define LV_USE_ANIMIMG    0
#define LV_USE_CALENDAR   0
#define LV_USE_CHART      0
#define LV_USE_COLORWHEEL 0
#define LV_USE_IMGBTN     0
#define LV_USE_KEYBOARD   0
#define LV_USE_LED        1
#define LV_USE_LIST       1
#define LV_USE_MENU       0
#define LV_USE_METER      0
#define LV_USE_MSGBOX     1
#define LV_USE_SPAN       0
#define LV_USE_SPINBOX    0
#define LV_USE_SPINNER    1
#define LV_USE_TABVIEW    0
#define LV_USE_TILEVIEW   0
#define LV_USE_WIN        0

// =============================================================================
// LAYOUT
// =============================================================================
#define LV_USE_FLEX       1
#define LV_USE_GRID       1

// =============================================================================
// MISC
// =============================================================================
#define LV_USE_THEME_DEFAULT    1
#define LV_USE_THEME_BASIC      1
#define LV_THEME_DEFAULT_DARK   1
#define LV_USE_ASSERT_NULL      1
#define LV_USE_ASSERT_MEM       1
#define LV_USE_LOG              0

#endif // LV_CONF_H
