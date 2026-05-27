// =============================================================================
// dsp_task.h — DSP processing task scaffold for ESP32-P4
// Pinned FreeRTOS task on Core 0 for spectrum analysis & visual effects
// =============================================================================
#pragma once

#include <cstdint>

// Spectrum bar data (16 bars, one per track)
struct SpectrumData {
    uint8_t bars[16];       // 0-255 amplitude per track
    bool    dirty;          // true when new data available for UI
};

// Copy current spectrum snapshot (thread-safe read from UI)
void dsp_get_spectrum(SpectrumData* out);

// Notify DSP task that a pad was triggered (call from any core)
void dsp_notify_pad(uint8_t pad, uint8_t velocity);

// Notify DSP task of current BPM (for pulse sync)
void dsp_set_bpm(float bpm);

// Start the DSP task (call once from setup)
void dsp_task_init();
