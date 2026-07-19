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
#include <atomic>
#if P4_ENABLE_PERF_LOG
#include <esp_heap_caps.h>
#endif

// =============================================================================
// TASK + SYNC PRIMITIVES
// =============================================================================
static SemaphoreHandle_t lvgl_mutex    = NULL;
static SemaphoreHandle_t sem_vsync_end = NULL;  // vsync acked our swap
static SemaphoreHandle_t sem_gui_ready = NULL;  // swap pending, wait for vsync
static volatile bool task_started = false;
static std::atomic<uint32_t> s_refresh_seq{0};
static_assert(std::atomic<uint32_t>::is_always_lock_free,
              "VSYNC counter must remain ISR-safe");

#if P4_ENABLE_PERF_LOG
// Perf counters — plain volatile ints are fine here: this is a diagnostic
// aid (occasional off-by-one from a torn read doesn't matter), not state
// the app depends on. touch_poll counter lives next to touch_task below.
static volatile uint32_t s_frame_count = 0;         // physical panel flips
static volatile uint32_t s_timer_handler_count = 0; // lv_timer_handler() calls
#endif

static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;

// Multitouch: 5 input devices (one per GT911 touch point)
#define MAX_TOUCH_POINTS 5
static lv_indev_drv_t touch_drvs[MAX_TOUCH_POINTS];
static lv_indev_t* touch_indevs[MAX_TOUCH_POINTS];

struct TouchPointState {
    lv_point_t point;
    lv_indev_state_t state;
    uint8_t area;   // GT911 touch size byte — proxy for finger pressure/area
};
static TouchPointState touch_data[MAX_TOUCH_POINTS] = {};
static portMUX_TYPE s_touch_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_touch_last_frame_ms = 0;

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
    s_refresh_seq.fetch_add(1, std::memory_order_relaxed);
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

#if P4_ENABLE_PERF_LOG
    s_frame_count++;
#endif

    // Step 1: swap active FB (DPI recognises internal pointer \u2192 zero-copy)
    esp_lcd_panel_draw_bitmap(panel, 0, 0, LCD_H_RES, LCD_V_RES, color_p);

    // Step 2: arm handshake \u2014 must come AFTER draw_bitmap
    // Drain tokens left by a timed-out older flush before arming this swap.
    while (xSemaphoreTake(sem_vsync_end, 0) == pdTRUE) {}
    while (xSemaphoreTake(sem_gui_ready, 0) == pdTRUE) {}
    uint32_t refresh_before = s_refresh_seq.load(std::memory_order_acquire);
    xSemaphoreGive(sem_gui_ready);

    // Step 3: wait for a refresh newer than this framebuffer swap. The
    // sequence check prevents a stale semaphore token acknowledging it.
    TickType_t wait_start = xTaskGetTickCount();
    const TickType_t wait_limit = pdMS_TO_TICKS(500);
    bool refreshed = false;
    do {
        TickType_t now = xTaskGetTickCount();
        TickType_t elapsed = now - wait_start;
        TickType_t remaining = (elapsed < wait_limit) ? (wait_limit - elapsed) : 0;
        if (xSemaphoreTake(sem_vsync_end, remaining) != pdTRUE) break;
        refreshed = (s_refresh_seq.load(std::memory_order_acquire) != refresh_before);
    } while (!refreshed);
    if (!refreshed) {
        xSemaphoreTake(sem_gui_ready, 0);
        P4_LOG_PRINTLN("[LVGL] Vsync timeout");
    }

    lv_disp_flush_ready(drv);
}

// =============================================================================
// GT911 TOUCH READ
// =============================================================================
// GT911 TOUCH — init + polling (runs on separate Core 0 task)
// =============================================================================
static bool gt911_initialized = false;

static void touch_publish(const TouchPointState next[MAX_TOUCH_POINTS]) {
    portENTER_CRITICAL(&s_touch_mux);
    memcpy(touch_data, next, sizeof(touch_data));
    portEXIT_CRITICAL(&s_touch_mux);
}

static void touch_release_all(void) {
    TouchPointState next[MAX_TOUCH_POINTS] = {};
    for (int i = 0; i < MAX_TOUCH_POINTS; i++) next[i].state = LV_INDEV_STATE_REL;
    touch_publish(next);
}

static void touch_release_if_stale(void) {
    if (s_touch_last_frame_ms != 0 &&
        (uint32_t)(millis() - s_touch_last_frame_ms) >= 1000) {
        touch_release_all();
        s_touch_last_frame_ms = millis();
    }
}

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
        touch_release_all();
        return;
    }

    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(0x81); Wire.write(0x4E);
    if (Wire.endTransmission(false) != 0) { touch_release_if_stale(); return; }
    if (Wire.requestFrom((uint8_t)TOUCH_I2C_ADDR, (uint8_t)1) != 1) {
        touch_release_if_stale();
        return;
    }
    uint8_t status = Wire.read();

    uint8_t touches = status & 0x0F;
    bool buf_ready = (status & 0x80);

    // Keep last valid frame when the controller hasn't published a new one yet.
    // This avoids short REL glitches that can cut sustained piano notes.
    if (!buf_ready) { touch_release_if_stale(); return; }

    if (touches == 0 || touches > MAX_TOUCH_POINTS) {
        touch_release_all();
        s_touch_last_frame_ms = millis();
        Wire.beginTransmission(TOUCH_I2C_ADDR);
        Wire.write(0x81); Wire.write(0x4E); Wire.write((uint8_t)0);
        Wire.endTransmission();
        return;
    }

    int readLen = touches * 8;
    uint8_t buf[MAX_TOUCH_POINTS * 8];
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    // Start at 0x814F so each 8-byte record includes its stable tracking ID.
    Wire.write(0x81); Wire.write(0x4F);
    bool ok = (Wire.endTransmission(false) == 0);
    if (ok) ok = (Wire.requestFrom((uint8_t)TOUCH_I2C_ADDR, (uint8_t)readLen) == readLen);
    if (ok) {
        TouchPointState next[MAX_TOUCH_POINTS] = {};
        for (int i = 0; i < MAX_TOUCH_POINTS; i++) next[i].state = LV_INDEV_STATE_REL;
        for (int i = 0; i < readLen; i++) buf[i] = Wire.read();
        for (int i = 0; i < touches; i++) {
            uint8_t track_id = buf[i*8] & 0x0F;
            int idx = (track_id < MAX_TOUCH_POINTS) ? track_id : i;
            uint16_t x    = buf[i*8+1] | ((uint16_t)buf[i*8+2] << 8);
            uint16_t y    = buf[i*8+3] | ((uint16_t)buf[i*8+4] << 8);
            uint16_t size = buf[i*8+5] | ((uint16_t)buf[i*8+6] << 8);
            next[idx].point.x = (x < LCD_H_RES) ? (lv_coord_t)x : (lv_coord_t)(LCD_H_RES - 1);
            next[idx].point.y = (y < LCD_V_RES) ? (lv_coord_t)y : (lv_coord_t)(LCD_V_RES - 1);
            next[idx].area  = (size > 255) ? 255 : (uint8_t)size;
            next[idx].state = LV_INDEV_STATE_PR;
        }
        touch_publish(next);
        s_touch_last_frame_ms = millis();
    } else {
        touch_release_if_stale();
    }

    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(0x81); Wire.write(0x4E); Wire.write((uint8_t)0);
    Wire.endTransmission();
}

// Adaptive touch poll rate: full 200Hz while a finger is down (or was down
// in the last TOUCH_WARM_MS — covers fast taps/drum rolls), backing off to
// TOUCH_POLL_IDLE_MS the rest of the time. The GT911 has an INT line that
// could gate this exactly, but its trigger polarity/mode isn't verified on
// this board and touch is the primary input for a live instrument — a wrong
// polarity guess would silently stop all touch input. This timing-based
// backoff needs no assumption about GT911 interrupt behavior: idle backoff
// only ever adds up to TOUCH_POLL_IDLE_MS of latency to the FIRST touch
// after a pause, then jumps straight back to 200Hz for the rest of the
// gesture, so it can't degrade in-progress tracking (drags/rolls) or make
// missed touches — only worst case is behaving exactly like before.
static constexpr uint32_t TOUCH_POLL_ACTIVE_MS = 5;    // 200Hz while warm
static constexpr uint32_t TOUCH_POLL_IDLE_MS   = 20;   // 50Hz once cold
static constexpr uint32_t TOUCH_WARM_MS        = 250;  // stay warm this long after last touch

// millis() of the last real finger-down, updated from the touch task (the one
// choke point for all GT911 input). Read by the QR screensaver's idle check.
static volatile uint32_t s_last_touch_ms = 0;
uint32_t lvgl_port_last_touch_ms(void) { return s_last_touch_ms; }

#if P4_ENABLE_PERF_LOG
static volatile uint32_t s_touch_poll_count = 0;
#endif

// Touch FreeRTOS task — Core 0, polls I2C independently of LVGL rendering.
// Aggregates up to 5 touch points into a per-pad pressed[] + velocity[] frame
// and delegates press/release/repeat logic to ui_screens (ui_pad_frame_update).
static void touch_task(void* arg) {
    (void)arg;
    uint32_t last_active_ms = 0;
    while (true) {
        gt911_poll_all();
#if P4_ENABLE_PERF_LOG
        s_touch_poll_count++;
#endif

        TouchPointState points[MAX_TOUCH_POINTS];
        portENTER_CRITICAL(&s_touch_mux);
        memcpy(points, touch_data, sizeof(points));
        portEXIT_CRITICAL(&s_touch_mux);

        bool    pressed[16]  = {};
        uint8_t velocity[16] = {};
        uint8_t cell_x[16];
        uint8_t cell_y[16];
        for (int p = 0; p < 16; p++) {
            cell_x[p] = 64;
            cell_y[p] = 64;
        }

        bool any_touch = false;
        for (int i = 0; i < MAX_TOUCH_POINTS; i++) {
            if (points[i].state != LV_INDEV_STATE_PR) continue;
            any_touch = true;
            uint8_t lx = 64;
            uint8_t ly = 64;
            int pad = ui_pad_from_xy((uint16_t)points[i].point.x,
                                     (uint16_t)points[i].point.y,
                                     &lx, &ly);
            if (pad < 0) continue;
            pressed[pad] = true;
            // Map GT911 area byte (0..255) to MIDI velocity (40..127).
            // Very light contacts (area==0) get a sensible floor so the sample
            // still plays clearly. Hard presses approach 127.
            uint8_t a = points[i].area;
            uint8_t v = a ? (uint8_t)(40 + ((uint32_t)a * 87) / 255) : 100;
            if (v > 127) v = 127;
            if (v > velocity[pad]) {
                velocity[pad] = v;
                cell_x[pad] = lx;
                cell_y[pad] = ly;
            }
        }

        ui_pad_frame_update(pressed, velocity, cell_x, cell_y);

        uint32_t now = millis();
        if (any_touch) last_active_ms = now;
        if (any_touch) s_last_touch_ms = now;
        bool warm = (now - last_active_ms) < TOUCH_WARM_MS;
        vTaskDelay(pdMS_TO_TICKS(warm ? TOUCH_POLL_ACTIVE_MS : TOUCH_POLL_IDLE_MS));
    }
}

// LVGL touch callback — instant read from cache (zero I2C overhead)
static void touch_read_cb(lv_indev_drv_t* drv, lv_indev_data_t* data) {
    int idx = (int)(intptr_t)drv->user_data;
    portENTER_CRITICAL(&s_touch_mux);
    TouchPointState point = touch_data[idx];
    portEXIT_CRITICAL(&s_touch_mux);
    data->point = point.point;
    data->state = point.state;
}

#if P4_ENABLE_PERF_LOG
// Prints once every PERF_LOG_INTERVAL_MS: real panel FPS (frames actually
// flushed, i.e. keeping up with vsync — the number that matters), LVGL task
// iteration rate, touch poll rate (confirms the idle backoff is working),
// and free heap split internal/PSRAM (watches for fragmentation over long
// sessions). Deliberately NOT per-task CPU%: that needs
// configGENERATE_RUN_TIME_STATS, which the prebuilt Arduino core's sdkconfig
// doesn't enable (same limitation noted for the watchdog config in
// main.cpp) — these counters use only always-available Arduino/heap APIs.
static constexpr uint32_t PERF_LOG_INTERVAL_MS = 2000;

static void perf_log_tick(void) {
    static uint32_t last_report_ms = 0;
    uint32_t now = millis();
    if (now - last_report_ms < PERF_LOG_INTERVAL_MS) return;
    uint32_t elapsed_ms = now - last_report_ms;
    last_report_ms = now;

    uint32_t frames = s_frame_count; s_frame_count = 0;
    uint32_t ticks   = s_timer_handler_count; s_timer_handler_count = 0;
    uint32_t touches = s_touch_poll_count; s_touch_poll_count = 0;

    float fps        = frames * 1000.0f / elapsed_ms;
    float tick_rate   = ticks * 1000.0f / elapsed_ms;
    float touch_rate  = touches * 1000.0f / elapsed_ms;

    Serial.printf("[PERF] fps=%.1f lvgl_tick=%.1f/s touch_poll=%.1f/s "
                  "heap_int=%uK heap_psram=%uK\n",
                  fps, tick_rate, touch_rate,
                  (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                  (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
}
#endif

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
        // The panel refreshes at 60 Hz and touch/pad delivery has its own
        // 200 Hz task. Running LVGL at 125 Hz added wakeups without producing
        // extra visible frames; align UI work with the physical display.
#if P4_ENABLE_PERF_LOG
        s_timer_handler_count++;
        perf_log_tick();
#endif
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(16));
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
    TouchPointState points[MAX_TOUCH_POINTS];
    portENTER_CRITICAL(&s_touch_mux);
    memcpy(points, touch_data, sizeof(points));
    portEXIT_CRITICAL(&s_touch_mux);
    uint8_t best = 0;
    for (int i = 0; i < MAX_TOUCH_POINTS; i++) {
        if (points[i].state != LV_INDEV_STATE_PR) continue;
        uint8_t a = points[i].area;
        uint8_t v = a ? (uint8_t)(40 + ((uint32_t)a * 87) / 255) : 100;
        if (v > 127) v = 127;
        if (v > best) best = v;
    }
    return best;
}
