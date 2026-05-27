// =============================================================================
// dsp_task.cpp — DSP processing task for ESP32-P4
// Runs on Core 0, produces spectrum data consumed by UI on Core 1
// =============================================================================

#include "dsp_task.h"
#include "../include/config.h"
#include <Arduino.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>

// =============================================================================
// CONFIGURATION
// =============================================================================
static constexpr int DSP_TASK_STACK   = 4096;
static constexpr int DSP_TASK_PRIO    = 3;       // below LVGL (5), above idle
static constexpr int DSP_CORE         = 0;       // Core 0 — keep Core 1 for UI
static constexpr int DSP_TICK_MS      = 16;      // ~60 Hz processing
static constexpr int SPECTRUM_BARS    = 16;
static constexpr uint8_t DECAY_RATE   = 6;       // amplitude units per tick

// =============================================================================
// SHARED STATE
// =============================================================================
static SpectrumData s_spectrum = {};
static portMUX_TYPE s_spectrum_mux = portMUX_INITIALIZER_UNLOCKED;
static std::atomic<uint32_t> s_bpm_q16{(uint32_t)(120.0f * 65536.0f)};

// Per-pad trigger energy — written from any core (UI/UART callbacks) and
// consumed on the DSP core. std::atomic<uint8_t> is lock-free on ESP32
// (single-byte aligned) and guarantees a consistent read/modify/write.
static std::atomic<uint8_t> s_padEnergy[16] = {};

// =============================================================================
// PUBLIC API
// =============================================================================
void dsp_get_spectrum(SpectrumData* out) {
    if (!out) return;
    portENTER_CRITICAL(&s_spectrum_mux);
    *out = s_spectrum;
    portEXIT_CRITICAL(&s_spectrum_mux);
}

void dsp_notify_pad(uint8_t pad, uint8_t velocity) {
    if (pad < 16) {
        // Store unconditionally — max() would need CAS loop and isn't needed:
        // the DSP task reads-and-clears, so the latest hit wins.
        s_padEnergy[pad].store(velocity, std::memory_order_relaxed);
    }
}

void dsp_set_bpm(float bpm) {
    s_bpm_q16.store((uint32_t)(bpm * 65536.0f), std::memory_order_relaxed);
}

// =============================================================================
// DSP TASK — runs continuously on Core 0
// =============================================================================
static void dsp_task_func(void* /*arg*/) {
    for (;;) {
        bool changed = false;

        portENTER_CRITICAL(&s_spectrum_mux);
        for (int i = 0; i < SPECTRUM_BARS; i++) {
            // Atomic read-and-clear of the trigger energy.
            uint8_t energy = s_padEnergy[i].exchange(0, std::memory_order_relaxed);
            if (energy > 0) {
                if (energy > s_spectrum.bars[i]) {
                    s_spectrum.bars[i] = energy;
                    changed = true;
                }
            }

            // Exponential-ish decay
            if (s_spectrum.bars[i] > DECAY_RATE) {
                s_spectrum.bars[i] -= DECAY_RATE;
                changed = true;
            } else if (s_spectrum.bars[i] > 0) {
                s_spectrum.bars[i] = 0;
                changed = true;
            }
        }

        s_spectrum.dirty = changed;
        portEXIT_CRITICAL(&s_spectrum_mux);

        vTaskDelay(pdMS_TO_TICKS(DSP_TICK_MS));
    }
}

// =============================================================================
// INIT
// =============================================================================
void dsp_task_init() {
    BaseType_t ok = xTaskCreatePinnedToCore(
        dsp_task_func,
        "dsp",
        DSP_TASK_STACK,
        nullptr,
        DSP_TASK_PRIO,
        nullptr,
        DSP_CORE
    );
    if (ok == pdPASS) {
        P4_LOG_PRINTLN("[DSP] Task started on Core 0");
    } else {
        P4_LOG_PRINTLN("[DSP] Failed to create task");
    }
}
