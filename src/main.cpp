// =============================================================================
// BlueSlaveP4 — main.cpp
// RED808 V6 Visual Beast — ESP32-P4 + MIPI-DSI 7" Display
// Entry point: display init → LVGL → WiFi/UDP → UI
// P4 connects DIRECTLY to Master via ESP32-C6 WiFi (SDIO ESP-Hosted)
// =============================================================================

#include <Arduino.h>
#include <SPIFFS.h>
#include <esp_task_wdt.h>
#include "../include/config.h"
#include "drivers/display_init.h"
#include "drivers/lvgl_port.h"
#include "uart_handler.h"
#include "udp_handler.h"
#include "ui/ui_screens.h"
#include "ui/ui_theme.h"
#include "dsp_task.h"

#if P4_USB_CDC_ENABLED && !P4_STANDALONE_MASTER_ONLY
#include "usb_cdc_handler.h"
#endif

void setup() {
    // 1. Debug serial (only waits if debug logging is enabled)
    Serial.begin(115200);
#if P4_ENABLE_DEBUG_LOG
    delay(500);
    Serial.println("\n=== RED808 P4 — Visual Beast ===");
    Serial.println("Guition JC1060P470C | ESP32-P4 + C6 WiFi");
#elif P4_ENABLE_FX_SYNC_LOG
    delay(500);
    Serial.println("\n[FX][DBG] P4 FX sync log enabled @115200");
    Serial.println("[FX][DBG] Waiting for masterFx/state_sync.fx/http FX packets");
#endif

    // 1b. Task watchdog: the prebuilt Arduino core ships with the IDF
    // defaults baked in (5 s timeout + panic reset) — sdkconfig.defaults is
    // NOT applied when framework = arduino uses precompiled libs. Reconfigure
    // at runtime instead: 30 s absorbs transient bursts (screen transitions,
    // MIDI load) and no panic keeps a genuine stall from hard-resetting the
    // instrument mid-session (it logs the culprit task instead).
    {
        esp_task_wdt_config_t wdt_cfg = {};
        wdt_cfg.timeout_ms = 30000;
        wdt_cfg.idle_core_mask = (1 << 0);   // keep watching Core 0 idle task
        wdt_cfg.trigger_panic = false;
        esp_task_wdt_reconfigure(&wdt_cfg);
    }

    // 2. Initialize MIPI-DSI display + backlight
    P4_LOG_PRINTLN("[INIT] Display...");
    display_init();
    P4_LOG_PRINTLN("[INIT] Display OK");

    // 3. Initialize LVGL (display driver + GT911 touch)
    P4_LOG_PRINTLN("[INIT] LVGL port...");
    lvgl_port_init();
    P4_LOG_PRINTLN("[INIT] LVGL OK");

    // 4. Apply default theme
    ui_theme_apply(THEME_OCEAN);

    // 5. Mount SPIFFS BEFORE creating screens: create_live_screen() restores
    // the persisted XTRA pad state from SPIFFS, so the FS must be up first.
    // Also used for MEM MIDI storage (/mid/*.mid). Non-fatal if absent.
    P4_LOG_PRINTLN("[INIT] Mounting SPIFFS...");
    if (SPIFFS.begin(false, "/spiffs", 10)) {
#if P4_ENABLE_DEBUG_LOG
        Serial.printf("[INIT] SPIFFS OK — %u / %u bytes used\n",
                      (unsigned)SPIFFS.usedBytes(), (unsigned)SPIFFS.totalBytes());
#endif
    } else {
        P4_LOG_PRINTLN("[INIT] SPIFFS mount failed (run uploadfs once)");
    }

    // 5b. Create UI screens (boot → live → seq → fx → vol → settings → perf)
    P4_LOG_PRINTLN("[INIT] UI screens...");
    ui_create_all_screens();
    P4_LOG_PRINTLN("[INIT] UI screens OK");

    // 6. Start WiFi/UDP connection to Master (via C6 SDIO)
    P4_LOG_PRINTLN("[INIT] WiFi/UDP to Master...");
    udp_handler_init();

    // 7. Start UART1 (optional S3 connection)
    P4_LOG_PRINTLN("[INIT] UART bridge to S3 (optional)...");
    uart_handler_init();

#if P4_USB_CDC_ENABLED && !P4_STANDALONE_MASTER_ONLY
    // 8. Start USB Host CDC (S3 via USB-C OTG port)
    P4_LOG_PRINTLN("[INIT] USB-C Host for S3...");
    usb_cdc_init();
#elif P4_STANDALONE_MASTER_ONLY
    P4_LOG_PRINTLN("[INIT] Standalone mode: AUX/S3 host disabled");
#endif

    // 9. Start DSP processing task (Core 0)
    P4_LOG_PRINTLN("[INIT] DSP task...");
    dsp_task_init();

    P4_LOG_PRINTLN("=== P4 Ready — Connecting to Master ===");

    // 10. Start LVGL rendering task (must be AFTER UI creation)
    lvgl_port_task_start();
}

void loop() {
    // Drain pad event queue FIRST — lowest latency pad→Master path (Core 1, no mutex)
    ui_process_pad_queue();

    // Send deferred UI control commands outside the LVGL task so button
    // feedback paints immediately even if WiFi/UDP stalls briefly.
    ui_process_control_queue();

    // Process WiFi/UDP from Master (primary connection)
    udp_handler_process();

    // Process UART packets from S3 (optional secondary)
    uart_handler_process();

    // Drain deferred MIDI→Master UDP burst (staged by MSG_PATTERN_PUSH)
    uart_handler_tick_pending_push();

#if P4_USB_CDC_ENABLED && !P4_STANDALONE_MASTER_ONLY
    // Try to connect/reconnect to S3 USB CDC device
    usb_cdc_process();
#endif

    // LVGL screen updates and rendering are handled by the dedicated LVGL task.
}
