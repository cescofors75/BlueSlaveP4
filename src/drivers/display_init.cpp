// =============================================================================
// display_init.cpp — MIPI-DSI display driver for Guition JC1060P470C
// JD9165BA controller, 1024×600, 2 MIPI-DSI lanes
// =============================================================================

#include "display_init.h"
#include "esp_lcd_jd9165.h"
#include "../include/config.h"
#include <Arduino.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_mipi_dsi.h>
#include <esp_ldo_regulator.h>

// MIPI-DSI PHY requires 2.5V power from on-chip LDO channel 3
#define MIPI_DSI_PHY_PWR_LDO_CHAN       3
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500
#define LCD_BIT_PER_PIXEL               16

static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;

void display_backlight(bool on) {
    pinMode(LCD_BL_GPIO, OUTPUT);
    digitalWrite(LCD_BL_GPIO, on ? HIGH : LOW);
}

esp_lcd_panel_handle_t display_get_panel(void) {
    return panel_handle;
}

esp_lcd_panel_handle_t display_init(void) {
    P4_LOG_PRINTLN("[Display] Init MIPI-DSI JD9165BA 1024x600...");

    // 1. Backlight off during init
    display_backlight(false);

    // 2. Power on MIPI-DSI PHY via on-chip LDO
    P4_LOG_PRINTLN("[Display] Enabling DSI PHY LDO...");
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo_mipi_phy));

    // 3. Create MIPI-DSI bus (also initializes DSI PHY)
    P4_LOG_PRINTLN("[Display] Creating DSI bus (2 lanes @ 750 Mbps)...");
    esp_lcd_dsi_bus_handle_t dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t bus_cfg = JD9165_PANEL_BUS_DSI_2CH_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_cfg, &dsi_bus));

    // 4. Create DBI panel IO (for sending DCS commands to JD9165BA)
    P4_LOG_PRINTLN("[Display] Creating DBI command IO...");
    esp_lcd_dbi_io_config_t dbi_cfg = JD9165_PANEL_IO_DBI_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(dsi_bus, &dbi_cfg, &io_handle));

    // 5. Create JD9165 panel (includes DPI video mode config)
    P4_LOG_PRINTLN("[Display] Creating JD9165 panel...");
    esp_lcd_dpi_panel_config_t dpi_cfg =
        JD9165_1024_600_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
    dpi_cfg.num_fbs = 2;   // Double buffering for zero-copy LVGL

    jd9165_vendor_config_t vendor_config = {
        .init_cmds = NULL,
        .init_cmds_size = 0,
        .mipi_config = {
            .dsi_bus = dsi_bus,
            .dpi_config = &dpi_cfg,
        },
    };
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_RST_GPIO,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_jd9165(io_handle, &panel_cfg, &panel_handle));

    // 6. Reset + Init (sends DCS commands + starts DPI video)
    P4_LOG_PRINTLN("[Display] Panel reset + init...");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    // 7. Backlight on
    display_backlight(true);

    P4_LOG_PRINTF("[Display] Ready: %dx%d MIPI-DSI\n", LCD_H_RES, LCD_V_RES);
    return panel_handle;
}
