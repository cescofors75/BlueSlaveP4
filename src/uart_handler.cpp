// =============================================================================
// uart_handler.cpp — P4 shared app state + local pattern cache + UDP pattern push
//
// The legacy UART/USB-C transport to the ESP32-S3 has been removed. The P4 is
// master-only over UDP. This file keeps the shared P4State, a local pattern
// cache (for instant pattern restore) and the deferred pattern push that
// streams a loaded pattern to the Master over UDP.
// =============================================================================

#include "uart_handler.h"
#include "udp_handler.h"
#include "../include/config.h"
#include <Arduino.h>

// P4 local state — single source of truth for UI rendering.
// All FX start muted (OFF) so the UI matches the DSP default state. If
// enc_muted/pot_muted defaulted to false the FX buttons would show "ON" even
// though the DSP never received an enable command.
P4State p4 = {
    .enc_muted       = {true, true, true},   // FLANGE/DELAY/REVERB start OFF
    .pot_muted       = {true, true, true},   // FOLD/CRUSH/PHASER start OFF
    .filter_type     = 0,                    // OFF
    .cutoff_hz       = 20000,                // neutral/open (treated as OFF in UI)
    .resonance_x10   = 10,                   // Q=1.0 neutral
    .distortion_pct  = 0,                    // OFF
    .bitcrush_bits   = 16,                   // 16-bit = bypass
    .sample_rate_hz  = 32000,                // neutral for UI SRATE mapping
};

// P4 SD browse state (populated locally from SD_MMC).
P4SdState p4sd = {};

// Pending melody state pushed by the Master over UDP (consumed by main loop).
PendingMelodyFromS3 g_pending_melody_from_s3 = {};

// -----------------------------------------------------------------------------
// Local master pattern cache — lets pattern switches restore instantly.
// -----------------------------------------------------------------------------
static bool s_master_step_cache[16][16][16] = {};
static bool s_master_step_cache_valid[16] = {};

// -----------------------------------------------------------------------------
// Deferred pattern push to the Master over UDP. The first push for an unknown
// slot sends a full clear+active refresh; later pushes use the per-slot cache
// and only send changed steps, so sequencer page/swing edits don't flood UDP.
// -----------------------------------------------------------------------------
enum PendingPushPhase {
    PP_IDLE = 0,
    PP_SELECT,
    PP_RESET_MIX,
    PP_DELTA,
    PP_CLEAR,
    PP_ACTIVE,
    PP_START,
};
struct PendingPush {
    PendingPushPhase phase;
    uint8_t  slot;
    int      idx;          // index within current phase
    uint32_t next_ms;      // earliest millis() to send next packet
    bool     step_bits[16][16];  // snapshot of steps to broadcast
};
static PendingPush s_push = {PP_IDLE, 0, 0, 0, {{false}}};

// Tunables — small delays keep WiFi/stack happy without stalling the loop.
static constexpr int PP_PACKETS_PER_TICK = 12;  // packets drained per loop()
static constexpr uint32_t PP_INTER_MS    = 1;   // pacing between bursts

static void stage_pattern_push(uint8_t slot, const bool steps[16][16]) {
    s_push.phase   = PP_SELECT;
    s_push.slot    = slot;
    s_push.idx     = 0;
    s_push.next_ms = millis();
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < 16; s++)
            s_push.step_bits[t][s] = steps[t][s];
}

static void remember_master_pattern(uint8_t slot, const bool steps[16][16]) {
    if (slot > 15) return;
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < 16; s++)
            s_master_step_cache[slot][t][s] = steps[t][s];
    s_master_step_cache_valid[slot] = true;
}

bool uart_restore_cached_pattern(uint8_t slot) {
    if (slot > 15 || !s_master_step_cache_valid[slot]) return false;
    p4.current_pattern = slot;
    for (int t = 0; t < 16; t++) {
        for (int s = 0; s < 16; s++) {
            p4.steps[t][s] = s_master_step_cache[slot][t][s];
        }
    }
    return true;
}

// =============================================================================
// INIT — set default P4 state
// =============================================================================
void uart_handler_init(void) {
    p4.bpm_int = Config::DEFAULT_BPM;
    p4.master_volume = 75;
    p4.seq_volume = 75;
    p4.live_volume = 75;
    p4.cutoff_hz = 20000;
    p4.resonance_x10 = 10;
    p4.bitcrush_bits = 16;
    p4.sample_rate_hz = 32000;
    for (int i = 0; i < 16; i++) p4.track_volume[i] = 75;

    p4sd.midi_load_result = -2;  // idle

    p4.s3_connected = false;
    p4.s3_wifi_connected = false;
    p4.last_heartbeat_ms = 0;
}

// =============================================================================
// PATTERN CACHE — remember a pattern's steps locally (no S3 transport anymore)
// =============================================================================
void uart_send_pattern_to_s3(int pattern, const bool steps[16][16]) {
    remember_master_pattern((uint8_t)constrain(pattern, 0, 15), steps);
}

// Retained no-op: the Master is now the sole tempo source.
void uart_lock_tempo(uint32_t duration_ms) {
    (void)duration_ms;
}

// =============================================================================
// PATTERN PUSH TO MASTER (UDP) — used by the MEM MIDI loader / sequencer
// =============================================================================
void uart_stage_pattern_push_from_steps(uint8_t slot, const bool steps[16][16]) {
    if (slot > 15) return;

    // 1) Update local UI state immediately
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < 16; s++)
            p4.steps[t][s] = steps[t][s];
    p4.current_pattern = slot;

    // 2) Skip staging if no Wi-Fi link to Master — UI is still correct.
    if (!udp_wifi_connected()) return;

    // 3) Arm deferred drain
    stage_pattern_push(slot, steps);
}

// Drain a few UDP packets per main-loop tick so MIDI loads don't stall the UI.
// Safe no-op when idle. Called from main loop().
void uart_handler_tick_pending_push(void) {
    if (s_push.phase == PP_IDLE) return;
    if (!udp_wifi_connected()) {
        s_push.phase = PP_IDLE;
        return;
    }
    uint32_t now = millis();
    if ((int32_t)(now - s_push.next_ms) < 0) return;

    int budget = PP_PACKETS_PER_TICK;

    while (budget > 0 && s_push.phase != PP_IDLE) {
        switch (s_push.phase) {
            case PP_SELECT:
                udp_send_select_pattern(s_push.slot);
                s_push.phase = s_master_step_cache_valid[s_push.slot] ? PP_DELTA : PP_RESET_MIX;
                s_push.idx   = 0;
                budget--;
                break;

            case PP_RESET_MIX: {
                int trk = s_push.idx & 0x0F;
                if (s_push.idx < 16) {
                    p4.track_solo[trk] = false;
                    udp_send_solo(trk, false);
                } else {
                    p4.track_muted[trk] = false;
                    udp_send_mute(trk, false);
                }
                s_push.idx++;
                budget--;
                if (s_push.idx >= 32) {
                    s_push.phase = PP_CLEAR;
                    s_push.idx   = 0;
                }
                break;
            }

            case PP_DELTA: {
                bool sent = false;
                while (s_push.idx < 256) {
                    int t = s_push.idx / 16;
                    int st = s_push.idx % 16;
                    s_push.idx++;
                    bool next = s_push.step_bits[t][st];
                    if (s_master_step_cache[s_push.slot][t][st] != next) {
                        udp_send_set_step(t, st, next);
                        budget--;
                        sent = true;
                        break;
                    }
                }
                if (!sent || s_push.idx >= 256) {
                    remember_master_pattern(s_push.slot, s_push.step_bits);
                    s_push.phase = PP_START;
                }
                break;
            }

            case PP_CLEAR: {
                // 256 clear packets (16 tracks × 16 steps)
                int t = s_push.idx / 16;
                int st = s_push.idx % 16;
                udp_send_set_step(t, st, false);
                s_push.idx++;
                budget--;
                if (s_push.idx >= 256) {
                    s_push.phase = PP_ACTIVE;
                    s_push.idx   = 0;
                }
                break;
            }

            case PP_ACTIVE: {
                // Scan forward to next active step
                bool sent = false;
                while (s_push.idx < 256) {
                    int t = s_push.idx / 16;
                    int st = s_push.idx % 16;
                    s_push.idx++;
                    if (s_push.step_bits[t][st]) {
                        udp_send_set_step(t, st, true);
                        budget--;
                        sent = true;
                        break;
                    }
                }
                if (!sent || s_push.idx >= 256) {
                    if (s_push.idx >= 256) {
                        remember_master_pattern(s_push.slot, s_push.step_bits);
                        s_push.phase = PP_START;
                    }
                }
                break;
            }

            case PP_START:
                if (p4.is_playing) {
                    udp_send_start();
                }
                s_push.phase = PP_IDLE;
                budget--;
                break;

            default:
                s_push.phase = PP_IDLE;
                break;
        }
    }

    s_push.next_ms = millis() + PP_INTER_MS;
}
