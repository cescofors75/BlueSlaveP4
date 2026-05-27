// =============================================================================
// lvgl_port.cpp — LVGL display/touch integration for ESP32-P4
// Guition JC1060P470C: 1024×600 MIPI-DSI + GT911 touch
// Zero-copy double-buffer, dual-semaphore vsync sync, FreeRTOS task
// =============================================================================

#include "lvgl_port.h"
#include "display_init.h"
#include "../include/config.h"
#include "../ui/ui_screens.h"
#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_mipi_dsi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

// =============================================================================
// TASK + SYNC PRIMITIVES
// =============================================================================
static SemaphoreHandle_t lvgl_mutex    = NULL;
static SemaphoreHandle_t sem_vsync_end = NULL;  // vsync acked our swap
static SemaphoreHandle_t sem_gui_ready = NULL;  // swap pending, wait for vsync
static volatile bool task_started = false;

static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;

// Multitouch: 5 input devices (one per GT911 touch point)
#define MAX_TOUCH_POINTS 5
static lv_indev_drv_t touch_drvs[MAX_TOUCH_POINTS];
static lv_indev_t* touch_indevs[MAX_TOUCH_POINTS];

static struct {
    volatile lv_point_t point;
    volatile lv_indev_state_t state;
    volatile uint8_t area;   // GT911 touch size byte — proxy for finger pressure/area
} touch_data[MAX_TOUCH_POINTS];

// =============================================================================
// VSYNC ISR — dual-semaphore handshake (Espressif pattern)
// Fires every frame refresh (~60Hz). Only unblocks flush() when a swap
// is genuinely pending (sem_gui_ready given by flush AFTER draw_bitmap).
// =============================================================================
static bool IRAM_ATTR dpi_on_refresh_done(esp_lcd_panel_handle_t panel,
                                           esp_lcd_dpi_panel_event_data_t *edata,
                                           void *user_ctx) {
    (void)panel; (void)edata; (void)user_ctx;
    BaseType_t woken = pdFALSE;
    if (sem_gui_ready && xSemaphoreTakeFromISR(sem_gui_ready, &woken) == pdTRUE) {
        xSemaphoreGiveFromISR(sem_vsync_end, &woken);
    }
    return woken == pdTRUE;
}

// Full-screen buffer pixel count (for draw_buf init)
#define LVGL_BUF_PIXELS   (LCD_H_RES * LCD_V_RES)

// =============================================================================
// DISPLAY FLUSH \u2014 zero-copy swap + vsync-gated completion
// draw_bitmap with internal FB pointer \u2192 pointer swap (no memcpy!)
// In direct_mode + partial refresh, LVGL may call this multiple times per
// frame (one per dirty area). Only the LAST call actually swaps + waits vsync.
// =============================================================================
static void disp_flush_cb(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* color_p) {
    (void)area;
    // Intermediate dirty regions \u2014 LVGL is still composing the frame.
    // In direct_mode the whole FB pointer is valid, so we just ack.
    if (!lv_disp_flush_is_last(drv)) {
        lv_disp_flush_ready(drv);
        return;
    }

    esp_lcd_panel_handle_t panel = display_get_panel();

    // Step 1: swap active FB (DPI recognises internal pointer \u2192 zero-copy)
    esp_lcd_panel_draw_bitmap(panel, 0, 0, LCD_H_RES, LCD_V_RES, color_p);

    // Step 2: arm handshake \u2014 must come AFTER draw_bitmap
    xSemaphoreGive(sem_gui_ready);

    // Step 3: wait for next frame refresh (vsync ACK)
    xSemaphoreTake(sem_vsync_end, pdMS_TO_TICKS(500));

    lv_disp_flush_ready(drv);
}

// =============================================================================
// GT911 TOUCH READ
// =============================================================================
// GT911 TOUCH — init + polling (runs on separate Core 0 task)
// =============================================================================
static bool gt911_initialized = false;

static void gt911_init(void) {
    Wire.begin(TOUCH_I2C_SDA, TOUCH_I2C_SCL, 400000);

    pinMode(TOUCH_INT_GPIO, OUTPUT);
    pinMode(TOUCH_RST_GPIO, OUTPUT);
    digitalWrite(TOUCH_INT_GPIO, LOW);
    digitalWrite(TOUCH_RST_GPIO, LOW);
    delay(10);
    digitalWrite(TOUCH_RST_GPIO, HIGH);
    delay(50);
    pinMode(TOUCH_INT_GPIO, INPUT);
    delay(50);

    Wire.beginTransmission(TOUCH_I2C_ADDR);
    if (Wire.endTransmission() == 0) {
        gt911_initialized = true;
        P4_LOG_PRINTLN("[Touch] GT911 detected at 0x5D");
    } else {
        P4_LOG_PRINTLN("[Touch] GT911 NOT found!");
    }
}

static void gt911_poll_all(void) {
    if (!gt911_initialized) {
        for (int i = 0; i < MAX_TOUCH_POINTS; i++) {
            touch_data[i].state = LV_INDEV_STATE_REL;
            touch_data[i].area  = 0;
        }
        return;
    }

    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(0x81); Wire.write(0x4E);
    if (Wire.endTransmission(false) != 0) return;
    if (Wire.requestFrom((uint8_t)TOUCH_I2C_ADDR, (uint8_t)1) != 1) return;
    uint8_t status = Wire.read();

    uint8_t touches = status & 0x0F;
    bool buf_ready = (status & 0x80);

    // Keep last valid frame when the controller hasn't published a new one yet.
    // This avoids short REL glitches that can cut sustained piano notes.
    if (!buf_ready) return;

    if (touches == 0 || touches > MAX_TOUCH_POINTS) {
        for (int i = 0; i < MAX_TOUCH_POINTS; i++) {
            touch_data[i].state = LV_INDEV_STATE_REL;
            touch_data[i].area  = 0;
        }
        Wire.beginTransmission(TOUCH_I2C_ADDR);
        Wire.write(0x81); Wire.write(0x4E); Wire.write((uint8_t)0);
        Wire.endTransmission();
        return;
    }

    int readLen = touches * 8;
    uint8_t buf[MAX_TOUCH_POINTS * 8];
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(0x81); Wire.write(0x50);
    bool ok = (Wire.endTransmission(false) == 0);
    if (ok) ok = (Wire.requestFrom((uint8_t)TOUCH_I2C_ADDR, (uint8_t)readLen) == readLen);
    if (ok) {
        for (int i = 0; i < MAX_TOUCH_POINTS; i++) {
            touch_data[i].state = LV_INDEV_STATE_REL;
            touch_data[i].area  = 0;
        }
        for (int i = 0; i < readLen; i++) buf[i] = Wire.read();
        for (int i = 0; i < touches; i++) {
            uint16_t x    = buf[i*8]   | ((uint16_t)buf[i*8+1] << 8);
            uint16_t y    = buf[i*8+2] | ((uint16_t)buf[i*8+3] << 8);
            uint16_t size = buf[i*8+4] | ((uint16_t)buf[i*8+5] << 8);
            touch_data[i].point.x = (x < LCD_H_RES) ? (lv_coord_t)x : (lv_coord_t)(LCD_H_RES - 1);
            touch_data[i].point.y = (y < LCD_V_RES) ? (lv_coord_t)y : (lv_coord_t)(LCD_V_RES - 1);
            touch_data[i].area  = (size > 255) ? 255 : (uint8_t)size;
            touch_data[i].state = LV_INDEV_STATE_PR;
        }
    }

    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(0x81); Wire.write(0x4E); Wire.write((uint8_t)0);
    Wire.endTransmission();
}

// Touch FreeRTOS task — Core 0, polls I2C independently of LVGL rendering.
// Aggregates up to 5 touch points into a per-pad pressed[] + velocity[] frame
// and delegates press/release/repeat logic to ui_screens (ui_pad_frame_update).
static void touch_task(void* arg) {
    (void)arg;
    while (true) {
        gt911_poll_all();

        bool    pressed[16]  = {};
        uint8_t velocity[16] = {};
        uint8_t cell_x[16];
        uint8_t cell_y[16];
        for (int p = 0; p < 16; p++) {
            cell_x[p] = 64;
            cell_y[p] = 64;
        }

        for (int i = 0; i < MAX_TOUCH_POINTS; i++) {
            if (touch_data[i].state != LV_INDEV_STATE_PR) continue;
            uint8_t lx = 64;
            uint8_t ly = 64;
            int pad = ui_pad_from_xy((uint16_t)touch_data[i].point.x,
                                     (uint16_t)touch_data[i].point.y,
                                     &lx, &ly);
            if (pad < 0) continue;
            pressed[pad] = true;
            // Map GT911 area byte (0..255) to MIDI velocity (40..127).
            // Very light contacts (area==0) get a sensible floor so the sample
            // still plays clearly. Hard presses approach 127.
            uint8_t a = touch_data[i].area;
            uint8_t v = a ? (uint8_t)(40 + ((uint32_t)a * 87) / 255) : 100;
            if (v > 127) v = 127;
            if (v > velocity[pad]) {
                velocity[pad] = v;
                cell_x[pad] = lx;
                cell_y[pad] = ly;
            }
        }

        ui_pad_frame_update(pressed, velocity, cell_x, cell_y);

        vTaskDelay(pdMS_TO_TICKS(5));   // 200Hz touch polling
    }
}

// LVGL touch callback — instant read from cache (zero I2C overhead)
static void touch_read_cb(lv_indev_drv_t* drv, lv_indev_data_t* data) {
    int idx = (int)(intptr_t)drv->user_data;
    data->point.x = touch_data[idx].point.x;
    data->point.y = touch_data[idx].point.y;
    data->state   = touch_data[idx].state;
}

// =============================================================================
// LVGL FREERTOS TASK — Core 0, priority 5
// Core 1 reserved for Arduino loop() (WiFi/UDP/UART)
// =============================================================================
static void lvgl_task(void* arg) {
    (void)arg;
    while (!task_started) vTaskDelay(pdMS_TO_TICKS(10));

    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        if (lvgl_port_lock(5)) {
            lv_timer_handler();
            ui_update_current_screen();
            lvgl_port_unlock();
        }
        // 8ms (125Hz tick) lets pad flashes appear on the very next vsync
        // after a touch. The actual paint cost is now tiny (partial refresh),
        // so the CPU spends most of the 8ms idle waiting for vsync.
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(8));
    }
}

// =============================================================================
// INIT
// =============================================================================
void lvgl_port_init(void) {
    P4_LOG_PRINTLN("[LVGL] Initializing (zero-copy + vsync + dual-task)...");

    lvgl_mutex    = xSemaphoreCreateMutex();
    sem_vsync_end = xSemaphoreCreateBinary();
    sem_gui_ready = xSemaphoreCreateBinary();
    if (!lvgl_mutex || !sem_vsync_end || !sem_gui_ready) {
        P4_LOG_PRINTLN("[LVGL] Failed to create synchronization primitives");
        return;
    }

    lv_init();

    // Register vsync callback on DPI panel
    esp_lcd_panel_handle_t panel = display_get_panel();
    esp_lcd_dpi_panel_event_callbacks_t cbs = {};
    cbs.on_refresh_done = dpi_on_refresh_done;
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(panel, &cbs, NULL));

    // Zero-copy: get the DPI panel's own PSRAM framebuffers
    void* fb0 = NULL;
    void* fb1 = NULL;
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(panel, 2, &fb0, &fb1));
    P4_LOG_PRINTF("[LVGL] DPI framebuffers: fb0=%p fb1=%p\n", fb0, fb1);

    lv_disp_draw_buf_init(&draw_buf, fb0, fb1, LVGL_BUF_PIXELS);

    // Display driver — zero-copy with dual PSRAM framebuffers + vsync.
    // direct_mode=1 + 2 FBs + full_refresh=0 → LVGL renders only dirty areas
    // and internally merges the previous frame's invalid area so both FBs stay
    // consistent (standard LVGL 8.x dual-buffer direct_mode pattern).
    // Cost per frame drops from 1.2 MB (full 1024×600) to just the dirty rect.
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res      = LCD_H_RES;
    disp_drv.ver_res      = LCD_V_RES;
    disp_drv.flush_cb     = disp_flush_cb;
    disp_drv.draw_buf     = &draw_buf;
    disp_drv.direct_mode  = 1;
    disp_drv.full_refresh = 0;
    lv_disp_drv_register(&disp_drv);

    // Touch — init GT911, then spawn polling task on Core 0
    // Higher priority than LVGL render (5) so pad taps are detected immediately
    // even when LVGL is flushing a frame.
    gt911_init();
    BaseType_t touch_ok = xTaskCreatePinnedToCore(touch_task, "touch", 4096, NULL, 6, NULL, 0);
    if (touch_ok != pdPASS) {
        P4_LOG_PRINTLN("[LVGL] Failed to create touch task");
    }

    // Register 5 LVGL input devices (read from cached touch_data — instant)
    for (int i = 0; i < MAX_TOUCH_POINTS; i++) {
        lv_indev_drv_init(&touch_drvs[i]);
        touch_drvs[i].type = LV_INDEV_TYPE_POINTER;
        touch_drvs[i].read_cb = touch_read_cb;
        touch_drvs[i].user_data = (void*)(intptr_t)i;
        touch_indevs[i] = lv_indev_drv_register(&touch_drvs[i]);
    }

    // LVGL rendering task — Core 0, priority 5
    // Core 1 stays free for Arduino loop (WiFi/UDP/UART)
    BaseType_t lvgl_ok = xTaskCreatePinnedToCore(lvgl_task, "lvgl", 16384, NULL, 5, NULL, 0);
    if (lvgl_ok != pdPASS) {
        P4_LOG_PRINTLN("[LVGL] Failed to create render task");
    }

    P4_LOG_PRINTF("[LVGL] Ready: %dx%d, zero-copy, vsync, touch+lvgl@Core0, wifi@Core1\n",
                  LCD_H_RES, LCD_V_RES);
}

void lvgl_port_update(void) {
    // No-op: LVGL now runs in dedicated FreeRTOS task
}

bool lvgl_port_lock(int timeout_ms) {
    if (!lvgl_mutex) return false;
    return xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void lvgl_port_unlock(void) {
    if (lvgl_mutex) xSemaphoreGive(lvgl_mutex);
}

void lvgl_port_task_start(void) {
    task_started = true;
}

lv_indev_t* lvgl_port_get_touch_indev(void) {
    return touch_indevs[0];
}

uint8_t lvgl_port_get_touch_velocity(void) {
    uint8_t best = 0;
    for (int i = 0; i < MAX_TOUCH_POINTS; i++) {
        if (touch_data[i].state != LV_INDEV_STATE_PR) continue;
        uint8_t a = touch_data[i].area;
        uint8_t v = a ? (uint8_t)(40 + ((uint32_t)a * 87) / 255) : 100;
        if (v > 127) v = 127;
        if (v > best) best = v;
    }
    return best;
}
