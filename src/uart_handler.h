// =============================================================================
// uart_handler.h — P4 shared app state + local pattern cache + UDP pattern push
//
// The legacy UART/USB-C link to the ESP32-S3 has been removed; the P4 talks to
// the RED808 Master over UDP only. This module now just owns the shared P4State
// and the helpers that stage pattern data to the Master.
// =============================================================================
#pragma once

#include <stdint.h>

// Initialize default P4 state (call once at boot).
void uart_handler_init(void);

// Drain any pending pattern push to the Master a few UDP packets at a time so
// it never blocks the main loop. No-op when idle. Call every loop().
void uart_handler_tick_pending_push(void);

// Stage a pattern push to the Master from a raw 16×16 step grid (MEM MIDI
// loader / sequencer). Updates p4.steps and p4.current_pattern, then streams
// the pattern to the Master over UDP.
void uart_stage_pattern_push_from_steps(uint8_t slot, const bool steps[16][16]);

// Retained for the MIDI-load tempo path. No-op now that the Master is the only
// tempo source (kept so callers don't need to change).
void uart_lock_tempo(uint32_t duration_ms);

// Cache the given pattern's steps locally so pattern switches restore instantly.
// Despite the legacy name this no longer transmits to any S3.
void uart_send_pattern_to_s3(int pattern, const bool steps[16][16]);

// Restore a previously cached pattern into p4.steps. Returns false if the slot
// has no cached data.
bool uart_restore_cached_pattern(uint8_t slot);

// =============================================================================
// P4 LOCAL STATE — single source of truth for the UI
// =============================================================================

// Pending melody state pushed by the Master over UDP (consumed by the main
// loop under lvgl_port_lock). Kept under the legacy field/global name.
struct PendingMelodyFromS3 {
    uint8_t engine = 3;
    uint8_t octave = 4;
    uint8_t rec    = 0;
    uint8_t pad    = 0;
    volatile bool pending = false;
};
extern PendingMelodyFromS3 g_pending_melody_from_s3;

struct P4State {
    // System
    int  bpm_int;           // 40-240
    int  bpm_frac;          // 0-9
    int  current_pattern;   // 0-15
    int  current_step;      // 0-15
    bool is_playing;
    bool wifi_connected;
    bool master_connected;
    int  theme;             // 0-5
    int  master_volume;     // 0-150
    int  seq_volume;        // 0-150
    int  live_volume;       // 0-150

    // Encoders (0-127)
    uint8_t enc_value[3];
    bool    enc_muted[3];

    // Pots (MIDI 0-127 raw)
    uint8_t pot_value[4];
    bool    pot_muted[3];   // P2/P3/P4 mute states

    // FX extended
    int  filter_type;       // 0-4
    int  cutoff_hz;         // 20-20000
    int  resonance_x10;    // 10-100
    int  distortion_pct;   // 0-100
    int  bitcrush_bits;    // 4-16
    int  sample_rate_hz;   // 1000-44100
    int  fx_resp_mode;     // 0-1

    // Tracks
    bool track_muted[16];
    bool track_solo[16];
    int  track_volume[16];  // 0-100

    // Master feedback
    char kit_name[32];
    bool sample_loaded[24];
    char sample_name[24][32];

    // Pattern step data
    bool steps[16][16];     // [track][step]

    // Pad triggers (flash feedback)
    uint8_t pad_velocity[16];
    unsigned long pad_flash_until[16];

    // Screen
    int  current_screen;    // Screen enum value

    // Connection
    unsigned long last_heartbeat_ms;
    bool s3_connected;
    bool s3_wifi_connected;
};

extern P4State p4;

// =============================================================================
// SD BROWSE STATE — populated locally from the P4's own SD_MMC card
// =============================================================================
#define P4_SD_MAX_ENTRIES 64

struct P4SdEntry {
    char name[48];
    bool is_dir;
    bool is_midi;
};

struct P4SdState {
    bool mounted;
    char path[128];
    char selected_file[64];
    int  selected_pad;
    bool selected_is_midi;   // true when selected entry is a .mid file
    // MIDI load result status (for UI feedback):
    //   -2 = idle / no request in flight
    //   -1 = request in flight
    //    0..N = loaded OK into pattern slot N
    //  0x7F = parse/load failed
    int8_t midi_load_result;
    P4SdEntry entries[P4_SD_MAX_ENTRIES];
    int  entry_count;
    bool list_complete;
    volatile bool needs_refresh;
};

extern P4SdState p4sd;
