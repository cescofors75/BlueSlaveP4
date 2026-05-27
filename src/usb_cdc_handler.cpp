// =============================================================================
// usb_cdc_handler.cpp — USB Host CDC-ACM (P4 reads S3 HWCDC via USB-OTG)
// ESP32-P4 acts as USB Host on its OTG port.
// ESP32-S3 presents as composite JTAG+CDC device (VID 0x303A, PID 0x1001).
// CDC-ACM interface is typically interface 2 on ESP32-S3 HWCDC composite.
// =============================================================================

#include "usb_cdc_handler.h"
#include "../include/config.h"
#include <Arduino.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/portmacro.h>
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

// ESP32-S3 HWCDC identifiers (from waveshare_s3_lcd7.json hwids)
#define S3_USB_VID  0x303A
#define S3_USB_PID  0x1001

// Ring buffer for received USB data
#define USB_RX_BUF_SIZE 1024
static uint8_t  usbRxBuf[USB_RX_BUF_SIZE];
static volatile int usbRxHead = 0;
static volatile int usbRxTail = 0;
static portMUX_TYPE s_usb_rx_mux = portMUX_INITIALIZER_UNLOCKED;

// CDC device handle
static cdc_acm_dev_hdl_t s_cdc_dev = NULL;
static volatile bool s_usb_connected = false;
static SemaphoreHandle_t s_cdc_tx_mutex = NULL;  // Protects s_cdc_dev during TX

// TX queue — decouples loop()/Core1 from USB blocking on Core0
// usb_cdc_write() enqueues (non-blocking); usb_cdc_tx_task drains from Core0
#define CDC_TX_QUEUE_DEPTH 16
#define CDC_TX_MAX_LEN     64  // basic pkt=5, extended pattern payload=32 + hdr/checksum
typedef struct { uint8_t data[CDC_TX_MAX_LEN]; uint8_t len; } CdcTxPkt;
static QueueHandle_t s_tx_queue = NULL;
static volatile bool s_usb_init_ok = false;
static unsigned long s_last_open_attempt = 0;
static const unsigned long OPEN_RETRY_MS = 2000;
static int s_open_attempts = 0;

// Diagnostic counters (persist across connections)
static std::atomic<int> s_connect_count{0};
static std::atomic<int> s_disconnect_count{0};
static std::atomic<int> s_dev_detect_count{0};
static std::atomic<int> s_last_open_err{0};
static std::atomic<int> s_last_dtr_err{0};
static std::atomic<uint32_t> s_rx_drop_count{0};
static char s_status_buf[128] = "not-init";

// =============================================================================
// CALLBACKS
// =============================================================================

// Called by CDC driver when data arrives from S3
static bool on_cdc_rx(const uint8_t *data, size_t data_len, void *user_arg) {
    portENTER_CRITICAL(&s_usb_rx_mux);
    for (size_t i = 0; i < data_len; i++) {
        int next = (usbRxHead + 1) % USB_RX_BUF_SIZE;
        if (next != usbRxTail) {  // drop if full
            usbRxBuf[usbRxHead] = data[i];
            usbRxHead = next;
        } else {
            s_rx_drop_count.fetch_add(1, std::memory_order_relaxed);
        }
    }
    portEXIT_CRITICAL(&s_usb_rx_mux);
    return true;
}

// Called on device events (disconnect, error)
static void on_cdc_event(const cdc_acm_host_dev_event_data_t *event, void *user_arg) {
    char buf[128];
    switch (event->type) {
        case CDC_ACM_HOST_DEVICE_DISCONNECTED:
            {
            int disconnect_count = s_disconnect_count.fetch_add(1, std::memory_order_relaxed) + 1;
            snprintf(buf, sizeof(buf), "[USB-CDC] DISCONNECTED (#%d) at %lums\n",
                     disconnect_count, millis());
            P4_LOG_PRINT(buf);
            // Acquire TX mutex to synchronize with any in-progress usb_cdc_write().
            // IMPORTANT: Do NOT call cdc_acm_host_close() from within this callback.
            // Calling it here causes re-entry into the CDC driver (which is still
            // processing this event) and triggers an assertion/crash in the USB host
            // library. The driver auto-invalidates the handle after the callback returns.
            // usb_cdc_process() will call cdc_acm_host_open() to reconnect.
            if (s_cdc_tx_mutex) xSemaphoreTake(s_cdc_tx_mutex, pdMS_TO_TICKS(50));
            s_cdc_dev = NULL;
            s_usb_connected = false;
            if (s_cdc_tx_mutex) xSemaphoreGive(s_cdc_tx_mutex);
            }
            break;
        case CDC_ACM_HOST_ERROR:
            snprintf(buf, sizeof(buf), "[USB-CDC] ERROR: %d at %lums\n",
                     event->data.error, millis());
            P4_LOG_PRINT(buf);
            break;
        default:
            snprintf(buf, sizeof(buf), "[USB-CDC] Event type=%d at %lums\n",
                     event->type, millis());
            P4_LOG_PRINT(buf);
            break;
    }
}

// Called when ANY new USB device is detected by the host
static void on_new_dev(usb_device_handle_t usb_dev) {
    s_dev_detect_count.fetch_add(1, std::memory_order_relaxed);
    // Use snprintf to build one message at a time (avoids race conditions on Serial)
    char buf[256];

    const usb_device_desc_t *desc;
    if (usb_host_get_device_descriptor(usb_dev, &desc) == ESP_OK) {
        snprintf(buf, sizeof(buf),
            "[USB-CDC] NEW DEV VID=0x%04X PID=0x%04X class=%d sub=%d configs=%d bcdUSB=0x%04X\n",
            desc->idVendor, desc->idProduct, desc->bDeviceClass, desc->bDeviceSubClass,
            desc->bNumConfigurations, desc->bcdUSB);
        P4_LOG_PRINT(buf);
    } else {
        P4_LOG_PRINTLN("[USB-CDC] NEW DEVICE: (no descriptor)");
    }

    // Dump config descriptor to find the CDC interface index
    const usb_config_desc_t *config_desc;
    if (usb_host_get_active_config_descriptor(usb_dev, &config_desc) == ESP_OK) {
        snprintf(buf, sizeof(buf), "[USB-CDC]   num_ifaces=%d total_len=%d\n",
                 config_desc->bNumInterfaces, config_desc->wTotalLength);
        P4_LOG_PRINT(buf);
        // Walk interface descriptors
        int offset = 0;
        const uint8_t *p = (const uint8_t *)config_desc;
        while (offset < config_desc->wTotalLength) {
            uint8_t len = p[offset];
            uint8_t type = p[offset + 1];
            if (len == 0) break;
            if (type == 0x04) { // INTERFACE descriptor
                snprintf(buf, sizeof(buf), "[USB-CDC]   IF#%d: class=0x%02X sub=0x%02X proto=0x%02X eps=%d\n",
                         p[offset + 2], p[offset + 5], p[offset + 6], p[offset + 7], p[offset + 4]);
                P4_LOG_PRINT(buf);
            }
            offset += len;
        }
    }
}

// =============================================================================
// USB CDC TX TASK — Core 0, priority 3
// Runs below USB host / LVGL tasks (pri 5) but drains the TX queue
// whenever Core 0 has free cycles. This prevents loop()/Core1 from ever
// blocking on cdc_acm_host_data_tx_blocking(), which was the root cause
// of the LVGL-9 crash (USB TX timeout → corrupted CDC driver state).
// =============================================================================
static void usb_cdc_tx_task(void* arg) {
    CdcTxPkt pkt;
    while (true) {
        if (xQueueReceive(s_tx_queue, &pkt, pdMS_TO_TICKS(20)) == pdTRUE) {
            // Take mutex to serialise with on_cdc_event disconnect handler
            if (xSemaphoreTake(s_cdc_tx_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (s_cdc_dev && s_usb_connected) {
                    // 50 ms timeout is generous — this task is not holding any
                    // shared mutex that other tasks need, so blocking here is safe
                    cdc_acm_host_data_tx_blocking(s_cdc_dev, pkt.data, pkt.len, 50);
                }
                xSemaphoreGive(s_cdc_tx_mutex);
            }
        }
    }
}

// =============================================================================
// USB HOST DAEMON TASK
// =============================================================================
static void usb_host_lib_task(void *arg) {
    P4_LOG_PRINTLN("[USB-CDC] Host lib task started");
    for (;;) {
        uint32_t event_flags;
        esp_err_t err = usb_host_lib_handle_events(pdMS_TO_TICKS(1000), &event_flags);
        if (err == ESP_OK) {
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
                P4_LOG_PRINTLN("[USB-CDC] No clients event");
                usb_host_device_free_all();
            }
        }
    }
}

// =============================================================================
// INIT
// =============================================================================
void usb_cdc_init(void) {
    P4_LOG_PRINTLN("[USB-CDC] === Initializing USB Host ===");

    // Create TX mutex (guards s_cdc_dev access between write and disconnect)
    if (!s_cdc_tx_mutex) {
        s_cdc_tx_mutex = xSemaphoreCreateMutex();
    }

    // Create TX queue — non-blocking enqueue from loop()/Core1
    if (!s_tx_queue) {
        s_tx_queue = xQueueCreate(CDC_TX_QUEUE_DEPTH, sizeof(CdcTxPkt));
        if (!s_tx_queue) {
            P4_LOG_PRINTLN("[USB-CDC] Failed to create TX queue!");
            return;
        }
    }

    // 1. Install USB Host Library (drives OTG-HS controller)
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        P4_LOG_PRINTF("[USB-CDC] usb_host_install FAILED: 0x%x (%s)\n", err, esp_err_to_name(err));
        return;
    }
    P4_LOG_PRINTLN("[USB-CDC] usb_host_install OK");

    // 2. Daemon task — handles USB Host Library events (enumeration, etc.)
    BaseType_t ok = xTaskCreatePinnedToCore(
        usb_host_lib_task, "usb_host", 4096, NULL, 5, NULL, 0);
    if (ok != pdPASS) {
        P4_LOG_PRINTLN("[USB-CDC] Failed to create usb_host task!");
        return;
    }
    P4_LOG_PRINTLN("[USB-CDC] Daemon task created");

    // 3b. TX task — Core 0, priority 3 (below USB host/LVGL at 5)
    // Drains s_tx_queue when Core 0 has free cycles; never blocks loop()
    ok = xTaskCreatePinnedToCore(
        usb_cdc_tx_task, "usb_tx", 2048, NULL, 3, NULL, 0);
    if (ok != pdPASS) {
        P4_LOG_PRINTLN("[USB-CDC] Failed to create usb_tx task!");
    } else {
        P4_LOG_PRINTLN("[USB-CDC] TX task created (Core0, pri3)");
    }

    // 3. Install CDC-ACM class driver with new_dev callback for diagnostics
    const cdc_acm_host_driver_config_t drv_cfg = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = 5,
        .xCoreID = 0,
        .new_dev_cb = on_new_dev,
    };
    err = cdc_acm_host_install(&drv_cfg);
    if (err != ESP_OK) {
        P4_LOG_PRINTF("[USB-CDC] cdc_acm_host_install FAILED: 0x%x (%s)\n", err, esp_err_to_name(err));
        return;
    }
    P4_LOG_PRINTLN("[USB-CDC] CDC-ACM driver installed");

    s_usb_init_ok = true;
    P4_LOG_PRINTLN("[USB-CDC] === Host ready — plug S3 into USB-OTG port ===");
}

// Helper: finalize connection after successful open
static void finalize_connection(const char* method, bool assert_dtr) {
    s_usb_connected = true;
    int connect_count = s_connect_count.fetch_add(1, std::memory_order_relaxed) + 1;

    char buf[128];
    snprintf(buf, sizeof(buf), "[USB-CDC] CONNECTED #%d via %s at %lums\n",
             connect_count, method, millis());
    P4_LOG_PRINT(buf);

    // Native ESP32-S3 HWCDC only starts behaving like an open serial port when
    // the host asserts DTR. Keep vendor-specific bridges conservative because
    // CH343-style auto-reset circuits may wire DTR/RTS to EN/BOOT.
    esp_err_t err = cdc_acm_host_set_control_line_state(s_cdc_dev, assert_dtr, false);
    s_last_dtr_err.store((int)err, std::memory_order_relaxed);
    snprintf(buf, sizeof(buf), "[USB-CDC] DTR=%d RTS=0: 0x%x (%s)\n", assert_dtr ? 1 : 0, err, esp_err_to_name(err));
    P4_LOG_PRINT(buf);

    // Set line coding (informational for USB CDC)
    cdc_acm_line_coding_t lc = {
        .dwDTERate   = 921600,
        .bCharFormat = 0,  // 1 stop bit
        .bParityType = 0,  // none
        .bDataBits   = 8,
    };
    err = cdc_acm_host_line_coding_set(s_cdc_dev, &lc);
    snprintf(buf, sizeof(buf), "[USB-CDC] LineCoding: 0x%x (%s)\n", err, esp_err_to_name(err));
    P4_LOG_PRINT(buf);

    // Update status buffer
    snprintf(s_status_buf, sizeof(s_status_buf), "OK via %s", method);
}

// =============================================================================
// PROCESS — call from loop(); tries to open S3 when not yet connected
// =============================================================================
void usb_cdc_process(void) {
    if (!s_usb_init_ok) return;
    if (s_usb_connected) return;  // already connected, nothing to do

    unsigned long now = millis();
    if (now - s_last_open_attempt < OPEN_RETRY_MS) return;
    s_last_open_attempt = now;
    s_open_attempts++;

    const cdc_acm_host_device_config_t dev_config = {
        .connection_timeout_ms = 500,
        .out_buffer_size = 256,
        .in_buffer_size  = 512,
        .event_cb  = on_cdc_event,
        .data_cb   = on_cdc_rx,
        .user_arg  = NULL,
    };

    // Strategy: across S3 firmware variants (TinyUSB CDC and USB-JTAG+CDC),
    // interface numbering can change. Probe a wider interface range and avoid
    // vendor-specific fallback by default because it can bind to non-CDC
    // interfaces and produce a "connected but no data" state.
    esp_err_t err;

    const uint8_t ifaces[] = {0, 1, 2, 3, 4, 5, 6, 7};
    for (uint8_t iface : ifaces) {
        err = cdc_acm_host_open(S3_USB_VID, S3_USB_PID, iface,
                                &dev_config, &s_cdc_dev);
        s_last_open_err.store((int)err, std::memory_order_relaxed);
        if (err == ESP_OK && s_cdc_dev) {
            char method[40];
            snprintf(method, sizeof(method), "VID/PID iface=%u", iface);
            finalize_connection(method, true);
            return;
        }

        err = cdc_acm_host_open(CDC_HOST_ANY_VID, CDC_HOST_ANY_PID, iface,
                                &dev_config, &s_cdc_dev);
        s_last_open_err.store((int)err, std::memory_order_relaxed);
        if (err == ESP_OK && s_cdc_dev) {
            char method[32];
            snprintf(method, sizeof(method), "wildcard iface=%u", iface);
            finalize_connection(method, true);
            return;
        }
    }

#if defined(P4_USB_CDC_ALLOW_VENDOR_FALLBACK) && P4_USB_CDC_ALLOW_VENDOR_FALLBACK
    // Optional fallback for unusual USB-UART bridges. Disabled by default
    // because it may attach to non-CDC interfaces on composite devices.
    for (uint8_t iface : ifaces) {
        err = cdc_acm_host_open_vendor_specific(CDC_HOST_ANY_VID, CDC_HOST_ANY_PID, iface,
                                                &dev_config, &s_cdc_dev);
        s_last_open_err.store((int)err, std::memory_order_relaxed);
        if (err == ESP_OK && s_cdc_dev) {
            char method[40];
            snprintf(method, sizeof(method), "vendor iface=%u", iface);
            finalize_connection(method, false);
            return;
        }
    }
#endif

    // Log periodically (every 5th attempt = every 10s)
    if (s_open_attempts % 5 == 1) {
        char buf[128];
        snprintf(buf, sizeof(buf), "[USB-CDC] No S3 (attempt #%d)\n", s_open_attempts);
        P4_LOG_PRINT(buf);
        usb_host_lib_info_t info;
        if (usb_host_lib_info(&info) == ESP_OK) {
            snprintf(buf, sizeof(buf), "[USB-CDC] Host: %d devs, %d clients\n",
                     info.num_devices, info.num_clients);
            P4_LOG_PRINT(buf);
        }
    }
}

// =============================================================================
// PUBLIC API
// =============================================================================

bool usb_cdc_connected(void) {
    return s_usb_connected;
}

int usb_cdc_available(void) {
    portENTER_CRITICAL(&s_usb_rx_mux);
    int h = usbRxHead;
    int t = usbRxTail;
    int available = (h - t + USB_RX_BUF_SIZE) % USB_RX_BUF_SIZE;
    portEXIT_CRITICAL(&s_usb_rx_mux);
    return available;
}

int usb_cdc_read(void) {
    portENTER_CRITICAL(&s_usb_rx_mux);
    if (usbRxHead == usbRxTail) {
        portEXIT_CRITICAL(&s_usb_rx_mux);
        return -1;
    }
    uint8_t b = usbRxBuf[usbRxTail];
    usbRxTail = (usbRxTail + 1) % USB_RX_BUF_SIZE;
    portEXIT_CRITICAL(&s_usb_rx_mux);
    return b;
}

size_t usb_cdc_write(const uint8_t* data, size_t len) {
    // Non-blocking enqueue — loop()/Core1 NEVER waits on USB hardware.
    // The actual cdc_acm_host_data_tx_blocking() runs from usb_cdc_tx_task
    // on Core0, where the USB host lib task also lives, eliminating the
    // cross-core blocking that was causing TX timeouts and CDC driver crashes
    // under LVGL 9's heavier Core0 load.
    if (!s_usb_connected || !data || len == 0 || len > CDC_TX_MAX_LEN) return 0;
    if (!s_tx_queue) return 0;
    CdcTxPkt pkt;
    memcpy(pkt.data, data, len);
    pkt.len = (uint8_t)len;
    // xQueueSend with 0 timeout: drop silently if queue full (pad events are
    // best-effort visual sync; dropped frames don't affect audio)
    return (xQueueSend(s_tx_queue, &pkt, 0) == pdTRUE) ? len : 0;
}

const char* usb_cdc_status_str(void) {
    // Update status buffer with current counters
    snprintf(s_status_buf, sizeof(s_status_buf),
             "init=%d det=%d conn=%d/%d att=%d dtr=0x%x drop=%lu",
             (int)s_usb_init_ok,
             s_dev_detect_count.load(std::memory_order_relaxed),
             s_connect_count.load(std::memory_order_relaxed),
             s_disconnect_count.load(std::memory_order_relaxed),
             s_open_attempts,
             s_last_dtr_err.load(std::memory_order_relaxed),
             (unsigned long)s_rx_drop_count.load(std::memory_order_relaxed));
    return s_status_buf;
}

void usb_cdc_get_diag(int* detected, int* connects, int* disconnects, int* attempts) {
    if (detected) *detected = s_dev_detect_count.load(std::memory_order_relaxed);
    if (connects) *connects = s_connect_count.load(std::memory_order_relaxed);
    if (disconnects) *disconnects = s_disconnect_count.load(std::memory_order_relaxed);
    if (attempts) *attempts = s_open_attempts;
}
