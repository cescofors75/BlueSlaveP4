// =============================================================================
// ui_theme.h — RED808 theme system for P4 (mirror of S3 themes)
// =============================================================================
#pragma once

#include <lvgl.h>

enum VisualTheme : uint8_t {
    THEME_RED808 = 0,
    THEME_OCEAN,
    THEME_NEON,
    THEME_SUNSET,
    THEME_RAINBOW,
    THEME_GREYSCALE,
    THEME_COUNT
};

struct ThemeColors {
    uint32_t bg, panel, surface, border;
    uint32_t text, text_dim;
    uint32_t accent, accent2;
    uint32_t success, warning, error, info, cyan;
    uint32_t track_colors[16];
    const char* name;
};

extern const ThemeColors theme_presets[THEME_COUNT];
extern VisualTheme currentTheme;

lv_color_t theme_bg();
lv_color_t theme_panel();
lv_color_t theme_surface();
lv_color_t theme_border();
lv_color_t theme_text();
lv_color_t theme_text_dim();
lv_color_t theme_accent();
lv_color_t theme_accent2();
lv_color_t theme_success();
lv_color_t theme_warning();
lv_color_t theme_error();
lv_color_t theme_info();
lv_color_t theme_cyan();

void ui_theme_apply(VisualTheme theme);

#define RED808_BG       theme_bg()
#define RED808_PANEL    theme_panel()
#define RED808_SURFACE  theme_surface()
#define RED808_BORDER   theme_border()
#define RED808_TEXT     theme_text()
#define RED808_TEXT_DIM theme_text_dim()
#define RED808_ACCENT   theme_accent()
#define RED808_ACCENT2  theme_accent2()
#define RED808_SUCCESS  theme_success()
#define RED808_WARNING  theme_warning()
#define RED808_ERROR    theme_error()
#define RED808_INFO     theme_info()
#define RED808_CYAN     theme_cyan()
