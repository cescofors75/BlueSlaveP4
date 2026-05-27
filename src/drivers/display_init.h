// =============================================================================
// display_init.h — MIPI-DSI display driver for Guition JC1060P470C
// =============================================================================
#pragma once

#include <esp_lcd_panel_ops.h>

// Initialize the MIPI-DSI display (JD9165BA 1024×600)
// Returns the panel handle, or NULL on failure.
esp_lcd_panel_handle_t display_init(void);

// Get the panel handle (for LVGL flush callback)
esp_lcd_panel_handle_t display_get_panel(void);

// Backlight control
void display_backlight(bool on);
