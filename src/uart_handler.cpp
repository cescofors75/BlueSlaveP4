// =============================================================================
// uart_handler.cpp — UART binary protocol receiver (P4 side)
// =============================================================================

#include "uart_handler.h"
#include "udp_handler.h"
#include "dsp_task.h"
#include "ui/ui_screens.h"
#include "../include/config.h"
#include <Arduino.h>
#include <atomic>

#if P4_USB_CDC_ENABLED
#include "usb_cdc_handler.h"
#endif

// P4 local state — single source of truth for UI rendering
// All FX start muted (OFF) so the UI is consistent with the DSP default state.
// If enc_muted/pot_muted were false by default the FX buttons would show "ON"
// even though the DSP never received an enable command (→ reverb ON but no sound).
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

// P4 SD remote browse state
P4SdState p4sd = {};

// Link statistics
UartStats uart_stats = {};

// UART instance
static HardwareSerial UartS3(UART_S3_PORT);

// Independent receive parsers per transport. UART and USB can be active at the
// same time during bring-up, so they must not share framing state.
struct RxParser {
    uint8_t buf[UART_RX_BUF];
    int head;
};
static RxParser s_uart_rx = {{0}, 0};
#if P4_USB_CDC_ENABLED
static RxParser s_usb_rx = {{0}, 0};
#endif

// Link source tracking for robust AUX status and TX routing.
// 0 = UART, 1 = USB CDC, -1 = unknown.
static int s_hb_source = -1;
static uint8_t s_hb_streak = 0;

// Tempo lock: absolute millis() until which incoming BPM updates from S3 are
// ignored (set by uart_lock_tempo() after applying a MIDI-file tempo).
static uint32_t s_tempo_lock_until_ms = 0;

static bool tempo_lock_active(void) {
    if (s_tempo_lock_until_ms == 0) return false;
    if ((int32_t)(millis() - s_tempo_lock_until_ms) < 0) return true;
    s_tempo_lock_until_ms = 0;
    return false;
}

// v2.9 — Track S3's current melody engine/octave so TCMD_MELODY_REC can
// forward the correct engine+octave to master in melodyRecToggle.
static uint8_t s_s3_mel_engine = 3;
static uint8_t s_s3_mel_octave = 4;
static bool    s_s3_mel_rec = false;
static uint8_t s_s3_mel_pad = 0;
static uint8_t s_s3_preview_engine = 3;
static uint32_t s_s3_preview_note_off_due_ms = 0;

// Ignore short-term S3 reflections of local mute/solo writes.
static const uint32_t TRACK_ECHO_GUARD_MS = 350;
static uint32_t s_last_mute_tx_ms[16] = {};
static uint32_t s_last_solo_tx_ms[16] = {};
static bool s_last_mute_tx_val[16] = {};
static bool s_last_solo_tx_val[16] = {};

// Packed cross-core melody snapshot. Bit 31 is the pending marker.
static std::atomic<uint32_t> s_pending_melody{0};

void p4_publish_pending_melody(uint8_t engine, uint8_t octave,
                               bool rec, uint8_t pad) {
    uint32_t packed = 0x80000000u |
                      ((uint32_t)engine) |
                      ((uint32_t)octave << 8) |
                      ((uint32_t)(rec ? 1u : 0u) << 16) |
                      ((uint32_t)pad << 17);
    s_pending_melody.store(packed, std::memory_order_release);
}

bool p4_consume_pending_melody(uint8_t* engine, uint8_t* octave,
                               bool* rec, uint8_t* pad) {
    uint32_t packed = s_pending_melody.exchange(0, std::memory_order_acq_rel);
    if ((packed & 0x80000000u) == 0) return false;
    if (engine) *engine = (uint8_t)(packed & 0xFFu);
    if (octave) *octave = (uint8_t)((packed >> 8) & 0xFFu);
    if (rec) *rec = ((packed >> 16) & 0x01u) != 0;
    if (pad) *pad = (uint8_t)((packed >> 17) & 0xFFu);
    return true;
}

// -----------------------------------------------------------------------------
// Deferred pattern push to Master. The first push for an unknown slot sends a
// full clear+active refresh; later pushes use a per-slot cache and only send
// changed steps, so sequencer page/swing edits don't flood UDP.
// -----------------------------------------------------------------------------
enum PendingPushPhase {
    PP_IDLE = 0,
    PP_STAGING,
    PP_SELECT,
    PP_RESET_MIX,
    PP_DELTA,
    PP_CLEAR,
    PP_ACTIVE,
    PP_START,
};
struct PendingPush {
    // phase is the cross-core publish flag: staged from the LVGL task
    // (Core 0), drained by loop() (Core 1). Atomic with release/acquire so
    // the drainer never sees PP_SELECT before the snapshot below is complete.
    std::atomic<PendingPushPhase> phase;
    uint8_t  slot;
    int      idx;          // index within current phase
    uint32_t next_ms;      // earliest millis() to send next packet
    bool     step_bits[16][16];  // snapshot of steps to broadcast
};
static PendingPush s_push = {PP_IDLE, 0, 0, 0, {{false}}};
struct QueuedPush {
    uint8_t slot;
    bool step_bits[16][16];
};
static QueuedPush s_queued_push = {};
static bool s_queued_push_valid = false;
static portMUX_TYPE s_push_queue_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_master_step_cache[16][16][16] = {};
static bool s_master_step_cache_valid[16] = {};

// Tunables — small delays keep WiFi/stack happy without stalling the loop.
static constexpr int PP_PACKETS_PER_TICK = 12;  // packets drained per loop()
static constexpr uint32_t PP_INTER_MS    = 1;   // pacing between bursts

static void stage_pattern_push(uint8_t slot, const bool steps[16][16]) {
    PendingPushPhase expected = PP_IDLE;
    if (s_push.phase.compare_exchange_strong(expected, PP_STAGING,
                                              std::memory_order_acq_rel)) {
        // We exclusively own the active slot while phase=PP_STAGING.
        // This new request is newer than any replacement queued just before
        // the previous transfer became idle.
        portENTER_CRITICAL(&s_push_queue_mux);
        s_queued_push_valid = false;
        portEXIT_CRITICAL(&s_push_queue_mux);
        s_push.slot    = slot;
        s_push.idx     = 0;
        s_push.next_ms = millis();
        for (int t = 0; t < 16; t++)
            for (int s = 0; s < 16; s++)
                s_push.step_bits[t][s] = steps[t][s];
        s_push.phase.store(PP_SELECT, std::memory_order_release);
        return;
    }

    // A transfer is active. Preserve one latest-wins replacement instead of
    // overwriting fields that Core 1 is currently draining.
    portENTER_CRITICAL(&s_push_queue_mux);
    s_queued_push.slot = slot;
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < 16; s++)
            s_queued_push.step_bits[t][s] = steps[t][s];
    s_queued_push_valid = true;
    portEXIT_CRITICAL(&s_push_queue_mux);
}

static bool activate_queued_push(void) {
    PendingPushPhase expected = PP_IDLE;
    if (!s_push.phase.compare_exchange_strong(expected, PP_STAGING,
                                               std::memory_order_acq_rel)) {
        return false;
    }
    bool found = false;
    portENTER_CRITICAL(&s_push_queue_mux);
    if (s_queued_push_valid) {
        s_push.slot = s_queued_push.slot;
        for (int t = 0; t < 16; t++)
            for (int s = 0; s < 16; s++)
                s_push.step_bits[t][s] = s_queued_push.step_bits[t][s];
        s_queued_push_valid = false;
        found = true;
    }
    portEXIT_CRITICAL(&s_push_queue_mux);
    if (!found) {
        s_push.phase.store(PP_IDLE, std::memory_order_release);
        return false;
    }
    s_push.idx = 0;
    s_push.next_ms = millis();
    s_push.phase.store(PP_SELECT, std::memory_order_release);
    return true;
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
// INIT
// =============================================================================
void uart_handler_init(void) {
    // Set defaults
    p4.bpm_int = Config::DEFAULT_BPM;
    p4.master_volume = 75;
    p4.seq_volume = 75;
    p4.live_volume = 75;
    p4.cutoff_hz = 20000;
    p4.resonance_x10 = 10;
    p4.bitcrush_bits = 16;
    p4.sample_rate_hz = 32000;
    for (int i = 0; i < 16; i++) p4.track_volume[i] = 75;

    // SD / MIDI load state
    p4sd.midi_load_result = -2;  // idle

    p4.s3_connected = false;
    p4.s3_wifi_connected = false;
    p4.last_heartbeat_ms = 0;

#if P4_STANDALONE_MASTER_ONLY
    P4_LOG_PRINTLN("[UART] Standalone mode: AUX/S3 transport disabled");
    return;
#endif

    UartS3.begin(UART_BAUD_RATE, SERIAL_8N1, UART_S3_RX_PIN, UART_S3_TX_PIN);
    UartS3.setRxBufferSize(UART_RX_BUF);

    P4_LOG_PRINTF("[UART] Init port %d: TX=%d RX=%d @ %d baud\n",
                  UART_S3_PORT, UART_S3_TX_PIN, UART_S3_RX_PIN, UART_BAUD_RATE);
}

// =============================================================================
// SEND TO S3
// =============================================================================
void uart_send_to_s3(uint8_t type, uint8_t id, uint8_t value) {
#if P4_STANDALONE_MASTER_ONLY
    (void)type;
    (void)id;
    (void)value;
    return;
#else
    UartBasicPacket pkt;
    uart_build_basic(&pkt, type, id, value);

    if (type == MSG_TRACK) {
        uint8_t sub = id & 0xF0;
        uint8_t trk = id & 0x0F;
        if (trk < 16) {
            uint32_t now = millis();
            if (sub == TRK_MUTE_BIT) {
                s_last_mute_tx_ms[trk] = now;
                s_last_mute_tx_val[trk] = (value != 0);
            } else if (sub == TRK_SOLO_BIT) {
                s_last_solo_tx_ms[trk] = now;
                s_last_solo_tx_val[trk] = (value != 0);
            }
        }
    }

#if P4_USB_CDC_ENABLED
    // Prefer USB if connected, fall back to UART
    if (usb_cdc_connected()) {
        usb_cdc_write((uint8_t*)&pkt, sizeof(pkt));
    } else if (s_hb_source == 1) {
        // Last healthy link was USB. Avoid silently blackholing commands to
        // UART when there is no physical UART bridge to S3.
        return;
    } else {
        UartS3.write((uint8_t*)&pkt, sizeof(pkt));
    }
#else
    UartS3.write((uint8_t*)&pkt, sizeof(pkt));
#endif
    uart_stats.tx_packets++;
#endif
}

// =============================================================================
// SEND PATTERN DATA TO S3 (extended packet)
// =============================================================================
void uart_send_pattern_to_s3(int pattern, const bool steps[16][16]) {
    remember_master_pattern((uint8_t)constrain(pattern, 0, 15), steps);

#if P4_STANDALONE_MASTER_ONLY
    (void)pattern;
    (void)steps;
    return;
#endif

#if P4_USB_CDC_ENABLED
    bool has_usb_s3 = usb_cdc_connected();
#else
    bool has_usb_s3 = false;
#endif
    if (!has_usb_s3 && !p4.s3_connected) return;

    // Pack 16 tracks × 16 steps into 32 bytes (2 bytes/track, big-endian, bit per step)
    uint8_t packed[32];
    for (int t = 0; t < 16; t++) {
        uint16_t bits = 0;
        for (int s = 0; s < 16; s++) { if (steps[t][s]) bits |= (1u << s); }
        packed[t * 2]     = (bits >> 8) & 0xFF;
        packed[t * 2 + 1] = bits & 0xFF;
    }
    // Build extended header: 0xAB, type, id(pattern), len_h, len_l
    const uint16_t plen = 32;
    uint8_t hdr[UART_EXT_HEADER_LEN];
    hdr[0] = UART_START_EXTENDED;
    hdr[1] = MSG_PATTERN_DATA;
    hdr[2] = (uint8_t)constrain(pattern, 0, 15);
    hdr[3] = (plen >> 8) & 0xFF;
    hdr[4] = plen & 0xFF;
    uint8_t cs = 0;
    for (int i = 0; i < UART_EXT_HEADER_LEN; i++) cs += hdr[i];
    for (int i = 0; i < 32; i++) cs += packed[i];

#if P4_USB_CDC_ENABLED
    if (has_usb_s3) {
        usb_cdc_write(hdr, UART_EXT_HEADER_LEN);
        usb_cdc_write(packed, 32);
        usb_cdc_write(&cs, 1);
        return;
    }
#endif
    UartS3.write(hdr, UART_EXT_HEADER_LEN);
    UartS3.write(packed, 32);
    UartS3.write(cs);
}

bool uart_s3_alive(void) {
#if P4_STANDALONE_MASTER_ONLY
    return false;
#else
    return p4.s3_connected;
#endif
}

// =============================================================================
// SD REMOTE BROWSE — send commands to S3
// =============================================================================
void uart_send_sd_mount(void) {
    p4sd.entry_count = 0;
    p4sd.list_complete = false;
    uart_send_to_s3(MSG_TOUCH_CMD, TCMD_SD_MOUNT, 0);
}

void uart_send_sd_select(uint8_t index) {
    p4sd.entry_count = 0;
    p4sd.list_complete = false;
    uart_send_to_s3(MSG_TOUCH_CMD, TCMD_SD_SELECT, index);
}

void uart_send_sd_back(void) {
    p4sd.entry_count = 0;
    p4sd.list_complete = false;
    uart_send_to_s3(MSG_TOUCH_CMD, TCMD_SD_BACK, 0);
}

void uart_send_sd_load(uint8_t pad) {
    uart_send_to_s3(MSG_TOUCH_CMD, TCMD_SD_LOAD, pad);
}

void uart_send_sd_load_midi(uint8_t slot) {
    // Mark load as in-flight so the UI shows a waiting state until the
    // S3 responds with SD_RESP_LOAD_OK (success slot) or 0xFF (fail).
    p4sd.midi_load_result = -1;
    p4sd.needs_refresh = true;
    uart_send_to_s3(MSG_TOUCH_CMD, TCMD_SD_LOAD_MIDI, slot);
}

// =============================================================================
// PROCESS BASIC PACKET
// =============================================================================
static void process_basic(const UartBasicPacket* pkt, bool from_usb) {
    uint8_t type = pkt->type;
    uint8_t id   = pkt->id;
    uint8_t val  = pkt->value;

    switch (type) {
        case MSG_ENCODER:
            if (id < 3) {
#if !P4_ENABLE_LEGACY_UART_FX_CONTROLS
                if (id == 1 || id == 2) break;
#endif
                p4.enc_value[id] = val;
                // Relay encoder FX values to Master (P4 is the WiFi gateway)
                if (udp_wifi_connected()) {
                    udp_send_fx_enc(id, val, p4.enc_muted[id]);
                }
            }
            break;

        case MSG_PAD:
            if (id < 16) {
                p4.pad_velocity[id] = val;
                p4.pad_flash_until[id] = millis() + 120;
                dsp_notify_pad(id, val);
                // Always relay S3 pad triggers to Master via P4 WiFi.
                // S3 is UART-only bridge (sendLivePadTrigger is a stub — no direct
                // WiFi path from S3 to Master). Do NOT gate on p4.s3_wifi_connected.
                if (udp_wifi_connected()) {
                    udp_send_trigger(id, val);
                }
            }
            break;

        case MSG_POT:
            if (id < 4) {
#if !P4_ENABLE_LEGACY_UART_FX_CONTROLS
                if (id == 2) break;
#endif
                p4.pot_value[id] = val;
                // Relay pot FX macros to Master:
                //   S3 pot 0 = Master volume (NOT an FX, ignored here — handled elsewhere)
                //   S3 pot 1 = Crush macro   → udp_send_fx_pot(1)
                //   S3 pot 2 = Phaser macro  → udp_send_fx_pot(2)
                //   S3 pot 3 = Fold macro    → udp_send_fx_pot(0)
                if (udp_wifi_connected()) {
                    if (id == 1) {
                        udp_send_fx_pot(1, val, p4.pot_muted[1]);
                    } else if (id == 2) {
                        udp_send_fx_pot(2, val, p4.pot_muted[2]);
                    } else if (id == 3) {
                        udp_send_fx_pot(0, val, p4.pot_muted[0]);
                    }
                }
            }
            break;

        case MSG_SYSTEM:
            switch (id) {
                case SYS_BPM_INT:
                    // Tempo lock window: ignore S3-initiated BPM updates for
                    // a short time after we applied a MIDI file's tempo, so
                    // the S3's stale cached BPM doesn't overwrite it.
                    if (tempo_lock_active()) break;
                    p4.bpm_int = val;
                    break;
                case SYS_BPM_FRAC:
                    if (tempo_lock_active()) break;
                    p4.bpm_frac = val;
                    // Relay BPM to Master (frac arrives after int)
                    if (udp_wifi_connected()) {
                        float bpm = p4.bpm_int + p4.bpm_frac * 0.1f;
                        udp_send_tempo(bpm);
                    }
                    break;
                case SYS_PATTERN:
                    p4.current_pattern = val;
                    if (uart_restore_cached_pattern(val)) {
                        uart_send_pattern_to_s3(val, p4.steps);
                    }
                    // Relay pattern selection to Master and request the full
                    // pattern payload to guarantee both P4 and S3 load it.
                    if (udp_wifi_connected()) {
                        udp_send_select_pattern(val);
                        udp_send_get_pattern(val);
                    }
                    break;
                case SYS_PLAY_STATE:
                    {
                    bool next_play = (val != 0);
                    if (udp_wifi_connected() && next_play != p4.is_playing) {
                        if (next_play) udp_send_start();
                        else udp_send_stop();
                    }
                    p4.is_playing = next_play;
                    // Snap local step counter when playback starts/stops
                    // so the fallback clock / UI label reset cleanly.
                    if (!p4.is_playing) p4.current_step = 0;
                    }
                    break;
                case SYS_STEP:
                    // P4 is authoritative clock; ignore external SYS_STEP
                    // updates from S3 to prevent clock fights.
                    (void)val;
                    break;
                case SYS_WIFI_STATE:  p4.s3_wifi_connected = (val != 0);   break;
                case SYS_MASTER_CONN: p4.master_connected = (val != 0);    break;
                case SYS_THEME:       p4.theme = val;                      break;
                // Relay volumes only on change — same P4↔S3↔Master loop
                // topology that required TRACK_ECHO_GUARD_MS for mute/solo.
                case SYS_VOLUME:
                    if (p4.master_volume != val) {
                        p4.master_volume = val;
                        if (udp_wifi_connected()) udp_send_set_volume(val);
                    }
                    break;
                case SYS_SEQ_VOL:
                    if (p4.seq_volume != val) {
                        p4.seq_volume = val;
                        if (udp_wifi_connected()) udp_send_set_seq_volume(val);
                    }
                    break;
                case SYS_LIVE_VOL:
                    if (p4.live_volume != val) {
                        p4.live_volume = val;
                        if (udp_wifi_connected()) udp_send_set_live_volume(val);
                    }
                    break;
                case SYS_HEARTBEAT:
                    {
                    uint32_t now_ms = millis();
                    int src = from_usb ? 1 : 0;
                    if (s_hb_source != src || (now_ms - p4.last_heartbeat_ms) > 1500) {
                        s_hb_streak = 0;
                    }
                    s_hb_source = src;
                    if (s_hb_streak < 255) s_hb_streak++;
                    p4.last_heartbeat_ms = now_ms;
                    // Require at least 2 coherent heartbeats from the same
                    // transport before declaring AUX ON.
                    p4.s3_connected = (s_hb_streak >= 2);
                    // Echo heartbeat back so S3 knows P4 is alive
                    uart_send_to_s3(MSG_SYSTEM, SYS_HEARTBEAT, 0x01);
                    }
                    break;
            }
            break;

        case MSG_FX:
            switch (id) {
                case FX_ENC0_MUTE:
                    p4.enc_muted[0] = (val != 0);
                    if (udp_wifi_connected()) udp_send_fx_enc(0, p4.enc_value[0], p4.enc_muted[0]);
                    break;
                case FX_ENC1_MUTE:
#if P4_ENABLE_LEGACY_UART_FX_CONTROLS
                    p4.enc_muted[1] = (val != 0);
                    if (udp_wifi_connected()) udp_send_fx_enc(1, p4.enc_value[1], p4.enc_muted[1]);
#endif
                    break;
                case FX_ENC2_MUTE:
#if P4_ENABLE_LEGACY_UART_FX_CONTROLS
                    p4.enc_muted[2] = (val != 0);
                    if (udp_wifi_connected()) udp_send_fx_enc(2, p4.enc_value[2], p4.enc_muted[2]);
#endif
                    break;
                case FX_POT0_MUTE:
                    p4.pot_muted[0] = (val != 0);
                    if (udp_wifi_connected()) udp_send_fx_pot(0, p4.pot_value[3], p4.pot_muted[0]);
                    break;
                case FX_POT1_MUTE:
                    p4.pot_muted[1] = (val != 0);
                    if (udp_wifi_connected()) udp_send_fx_pot(1, p4.pot_value[1], p4.pot_muted[1]);
                    break;
                case FX_POT2_MUTE:
#if P4_ENABLE_LEGACY_UART_FX_CONTROLS
                    p4.pot_muted[2] = (val != 0);
                    if (udp_wifi_connected()) udp_send_fx_pot(2, p4.pot_value[2], p4.pot_muted[2]);
#endif
                    break;
                case FX_FILTER_TYPE:  p4.filter_type = val;                break;
                // 16-bit values arrive as two packets (H then L). Accumulate
                // into a staging variable and commit to p4.* only when the
                // low byte arrives so UI never reads a half-updated value.
                // Staging is per-transport: UART and USB can both be live
                // during bring-up, and interleaved H/L pairs across feeds
                // would corrupt the committed value.
                case FX_CUTOFF_H: {
                    static int s_cutoff_staging[2] = {20000, 20000};
                    int src = from_usb ? 1 : 0;
                    s_cutoff_staging[src] = (s_cutoff_staging[src] & 0x00FF) | (val << 8);
                    // Expose staging until L arrives — keeps high byte visible.
                    p4.cutoff_hz = s_cutoff_staging[src];
                    break;
                }
                case FX_CUTOFF_L: {
                    // Commit: take last high byte from current value.
                    p4.cutoff_hz = (p4.cutoff_hz & 0xFF00) | val;
                    break;
                }
                case FX_RESONANCE:    p4.resonance_x10 = val;             break;
                case FX_DISTORTION:   p4.distortion_pct = val;            break;
                case FX_BITCRUSH:     p4.bitcrush_bits = val;             break;
                case FX_SAMPLERATE_H: {
                    static int s_sr_staging[2] = {32000, 32000};
                    int src = from_usb ? 1 : 0;
                    s_sr_staging[src] = (s_sr_staging[src] & 0x00FF) | (val << 8);
                    p4.sample_rate_hz = s_sr_staging[src];
                    break;
                }
                case FX_SAMPLERATE_L: {
                    p4.sample_rate_hz = (p4.sample_rate_hz & 0xFF00) | val;
                    break;
                }
                case FX_RESP_MODE:    p4.fx_resp_mode = val;              break;
            }
            break;

        case MSG_TRACK: {
            uint8_t sub = id & 0xF0;
            uint8_t trk = id & 0x0F;
            if (trk < 16) {
                switch (sub) {
                    case TRK_MUTE_BIT: {
                        bool incoming = (val != 0);
                        uint32_t now = millis();
                        if ((now - s_last_mute_tx_ms[trk] < TRACK_ECHO_GUARD_MS)) {
                            break;
                        }
                        if (p4.track_muted[trk] == incoming) break;
                        p4.track_muted[trk] = incoming;
                        if (udp_wifi_connected()) udp_send_mute(trk, incoming);
                        break;
                    }
                    case TRK_SOLO_BIT: {
                        bool incoming = (val != 0);
                        uint32_t now = millis();
                        if ((now - s_last_solo_tx_ms[trk] < TRACK_ECHO_GUARD_MS)) {
                            break;
                        }
                        if (p4.track_solo[trk] == incoming) break;
                        p4.track_solo[trk] = incoming;
                        if (udp_wifi_connected()) udp_send_solo(trk, incoming);
                        break;
                    }
                    case TRK_VOLUME:
                        p4.track_volume[trk] = val;
                        if (udp_wifi_connected()) udp_send_set_track_volume(trk, val);
                        break;
                }
            }
            break;
        }

        case MSG_SCREEN:
            if (id == SCR_NAVIGATE) p4.current_screen = val;
            break;

        case MSG_TOUCH_CMD:
            // S3 relays a step toggle when it has no WiFi (P4 forwards to Master)
            if (id == TCMD_STEP_TOGGLE) {
                if (udp_master_connected()) {
                    // With master online, step authority is setStep/pattern_sync.
                    // Ignore relay toggles to prevent double inversion in multi-P4.
                    break;
                }
                int trk = (val >> 4) & 0xF;
                int stp = val & 0xF;
                if (trk < 16 && stp < 16) {
                    p4.steps[trk][stp] = !p4.steps[trk][stp];
                    if (udp_wifi_connected())
                        udp_send_set_step(trk, stp, p4.steps[trk][stp]);
                }
            }
            else if (id == TCMD_SYNC_PADS) {
                ui_live_set_sync_p4(val != 0);
            }
            // v2.9 — melody commands from S3: apply DIRECTLY to piano UI (like pad sync)
            // Also forward to master via UDP so web/other UIs stay in sync.
            else if (id == TCMD_MELODY_ENGINE) {
                s_s3_mel_engine = val;
                p4_publish_pending_melody(s_s3_mel_engine, s_s3_mel_octave,
                                           s_s3_mel_rec, s_s3_mel_pad);
                if (udp_wifi_connected()) udp_send_melody_set_engine(val);
            }
            else if (id == TCMD_MELODY_OCTAVE) {
                s_s3_mel_octave = val;
                p4_publish_pending_melody(s_s3_mel_engine, s_s3_mel_octave,
                                           s_s3_mel_rec, s_s3_mel_pad);
                if (udp_wifi_connected()) udp_send_melody_set_octave(val);
            }
            else if (id == TCMD_MELODY_REC) {
                s_s3_mel_rec = (val != 0);
                p4_publish_pending_melody(s_s3_mel_engine, s_s3_mel_octave,
                                           s_s3_mel_rec, s_s3_mel_pad);
                if (udp_wifi_connected())
                    udp_send_melody_rec_toggle(val != 0, s_s3_mel_engine, s_s3_mel_octave);
            }
            else if (id == TCMD_MELODY_CLEAR) {
                if (udp_wifi_connected()) udp_send_melody_clear();
            }
            else if (id == TCMD_MELODY_PAD) {
                s_s3_mel_pad = val;
                p4_publish_pending_melody(s_s3_mel_engine, s_s3_mel_octave,
                                           s_s3_mel_rec, s_s3_mel_pad);
                if (udp_wifi_connected()) udp_send_melody_set_pad(val);
            }
            else if (id == TCMD_MELODY_NOTE) {
                if (val <= 127 && udp_wifi_connected()) {
                    if (s_s3_preview_note_off_due_ms != 0 && s_s3_preview_engine != s_s3_mel_engine) {
                        udp_send_synth_note_off(s_s3_preview_engine, 0);
                    }
                    udp_send_synth_note_on_ex(s_s3_mel_engine, val, 110, false, false);
                    s_s3_preview_engine = s_s3_mel_engine;
                    s_s3_preview_note_off_due_ms = millis() + 220;
                }
            }
            else if (id == TCMD_PIANO_NOTE_ON) {
                if (val <= 127 && udp_wifi_connected()) {
                    if (s_s3_preview_note_off_due_ms != 0) {
                        udp_send_synth_note_off(s_s3_preview_engine, 0);
                        s_s3_preview_note_off_due_ms = 0;
                    }
                    udp_send_synth_note_on_ex(s_s3_mel_engine, val, 110, false, false);
                    s_s3_preview_engine = s_s3_mel_engine;
                }
            }
            else if (id == TCMD_MELODY_NOTE_OFF) {
                if (udp_wifi_connected()) udp_send_synth_note_off(s_s3_mel_engine, 0);
                if (s_s3_preview_engine == s_s3_mel_engine) s_s3_preview_note_off_due_ms = 0;
            }
            break;
    }
}

// =============================================================================
// PROCESS EXTENDED PACKET (pattern data, etc.)
// =============================================================================
static void process_extended(uint8_t type, uint8_t id, const uint8_t* payload, int len) {
    if (type == MSG_PATTERN_DATA && len == 32) {
        // 32 bytes = 256 bits = 16 tracks × 16 steps (big-endian: MSB first per track)
        for (int track = 0; track < 16; track++) {
            uint16_t row = ((uint16_t)payload[track * 2] << 8) | payload[track * 2 + 1];
            for (int step = 0; step < 16; step++) {
                p4.steps[track][step] = (row >> step) & 1;
            }
        }
        if (id < 16) remember_master_pattern(id, p4.steps);
        // NOTE: do NOT forward the whole pattern as N×setStep to Master.
        // The Master already owns the canonical pattern and pushes it via
        // `pattern_sync` (see udp_handler.cpp). Re-broadcasting 256 UDP
        // packets per pattern change floods the link and can overflow the
        // socket. selectPattern (SYS_PATTERN) is enough for Master to sync.
    }
    else if (type == MSG_PATTERN_PUSH && len == 32) {
        // S3 just parsed a MIDI file from SD and handed us a freshly-loaded
        // pattern. The Master does NOT yet know about this pattern — P4 is
        // the owner of the Master UDP channel for this path.
        //
        // Flow:
        //   1. Decode the 32-byte packed payload into p4.steps[][].
        //   2. Select that pattern slot on Master.
        //   3. Clear the slot (setStep active:false for all 16 tracks × 16 steps).
        //   4. Send setStep active:true for every hit.
        //   5. If transport is already running, re-send start after the
        //      final setStep so Master begins from a complete pattern.
        int slot = (int)id;
        if (slot < 0 || slot > 15) return;

        // 1) decode into local UI state (authoritative for P4 display)
        for (int track = 0; track < 16; track++) {
            uint16_t row = ((uint16_t)payload[track * 2] << 8) | payload[track * 2 + 1];
            for (int step = 0; step < 16; step++) {
                p4.steps[track][step] = (row >> step) & 1;
            }
        }
        p4.current_pattern = slot;

        if (!udp_wifi_connected()) {
            // No master link — UI is updated but master won't play the new
            // pattern until WiFi is available again.
            return;
        }

        // 2) Stage a deferred push to Master. The heavy UDP burst is drained
        //    a few packets per main-loop tick by uart_handler_tick_pending_push()
        //    so LVGL / touch / pad queue never stall.
        stage_pattern_push((uint8_t)slot, p4.steps);
    }
    else if (type == MSG_MELODY_DATA && id == MEL_DATA_ASSIGN && len == 35) {
        uint8_t pad = payload[0];
        uint8_t engine = payload[1];
        uint8_t octave = payload[2];
        if (pad >= 16 || engine < 3 || engine > 6 || octave < 1 || octave > 7) return;

        bool grid[16][12] = {{false}};
        for (int c = 0; c < 16; c++) {
            uint16_t bits = ((uint16_t)payload[3 + c * 2] << 8) | payload[4 + c * 2];
            for (int r = 0; r < 12; r++) {
                grid[c][r] = ((bits >> r) & 1u) != 0;
            }
        }
        if (udp_wifi_connected()) {
            udp_send_melody_assign(pad, engine, octave, grid);
        }
    }
    else if (type == MSG_SYNTH_DATA) {
        if (id == SYNTH_DATA_PARAM && len == 7) {
            uint8_t engine = payload[0];
            uint8_t instrument = payload[1];
            uint8_t paramId = payload[2];
            float value = 0.0f;
            memcpy(&value, &payload[3], sizeof(value));
            if (engine >= 3 && engine <= 8 && udp_wifi_connected()) {
                udp_send_synth_param(engine, instrument, paramId, value);
            }
        }
        else if (id == SYNTH_DATA_PRESET && len == 2) {
            uint8_t engine = payload[0];
            uint8_t preset = payload[1];
            if (engine >= 3 && engine <= 8 && preset < 4 && udp_wifi_connected()) {
                udp_send_synth_preset(engine, preset);
            }
        }
    }
    else if (type == MSG_SD_DATA) {
        switch (id) {
            case SD_RESP_STATUS:
                if (len >= 1) p4sd.mounted = (payload[0] != 0);
                p4sd.needs_refresh = true;
                break;
            case SD_RESP_ENTRY:
                if (len >= 3 && p4sd.entry_count < P4_SD_MAX_ENTRIES) {
                    int idx = p4sd.entry_count++;
                    p4sd.entries[idx].is_dir  = (payload[1] == 'D');
                    p4sd.entries[idx].is_midi = (payload[1] == 'M');
                    int nameLen = len - 2;
                    if (nameLen > 47) nameLen = 47;
                    memcpy(p4sd.entries[idx].name, &payload[2], nameLen);
                    p4sd.entries[idx].name[nameLen] = '\0';
                }
                break;
            case SD_RESP_LIST_END:
                p4sd.list_complete = true;
                p4sd.needs_refresh = true;
                break;
            case SD_RESP_PATH:
                if (len > 0 && len < (int)sizeof(p4sd.path) - 1) {
                    memcpy(p4sd.path, payload, len);
                    p4sd.path[len] = '\0';
                }
                break;
            case SD_RESP_SELECTED:
                if (len > 0 && len < (int)sizeof(p4sd.selected_file) - 1) {
                    memcpy(p4sd.selected_file, payload, len);
                    p4sd.selected_file[len] = '\0';
                    p4sd.needs_refresh = true;
                }
                break;
            case SD_RESP_LOAD_OK:
                p4sd.needs_refresh = true;
                // Update UI feedback from LVGL context (p4sd state read in sd_refresh_ui)
                // The payload[0]==0xFF signals MIDI parse failure.
                // IMPORTANT: do NOT overwrite p4sd.selected_pad — that is the
                // user-selected drum pad for WAV sample load. Use a dedicated
                // status field so the two flows don't interfere.
                if (len >= 1 && p4sd.selected_is_midi) {
                    p4sd.midi_load_result = (payload[0] == 0xFF) ? 0x7F : (int8_t)payload[0];
                }
                break;
        }
    }
}

// =============================================================================
// MAIN PROCESS — call from loop()
// =============================================================================
static int s_totalDiscarded = 0;
static int s_processed = 0;

// Resync after a corrupted frame: instead of discarding the whole window
// (which can swallow the genuine start byte of the next packet after a single
// dropped byte at 921600 baud), shift the buffer to the next 0xAA/0xAB
// candidate. Worst case it's a payload byte and the next checksum fails again.
static void parser_resync(RxParser& parser) {
    int next = -1;
    for (int i = 1; i < parser.head; i++) {
        if (parser.buf[i] == UART_START_BASIC || parser.buf[i] == UART_START_EXTENDED) {
            next = i;
            break;
        }
    }
    if (next < 0) {
        parser.head = 0;
        return;
    }
    memmove(parser.buf, parser.buf + next, parser.head - next);
    parser.head -= next;
}

// Drop n consumed bytes from the front of the window, keeping the rest.
static void parser_consume(RxParser& parser, int n) {
    if (n >= parser.head) {
        parser.head = 0;
        return;
    }
    memmove(parser.buf, parser.buf + n, parser.head - n);
    parser.head -= n;
}

// Feed one byte into one transport's protocol parser.
static void feed_byte(RxParser& parser, uint8_t b, bool from_usb) {
    bool link_active = p4.s3_connected;
#if P4_USB_CDC_ENABLED
    if (from_usb) link_active = usb_cdc_connected() || p4.s3_connected;
#endif

    // Look for start byte
    if (parser.head == 0) {
        if (b != UART_START_BASIC && b != UART_START_EXTENDED) {
            s_totalDiscarded++;
            if (link_active) {
                uart_stats.rx_framing_errors++;
            }
            if (link_active && s_totalDiscarded <= 50) {
                P4_LOG_PRINTF("[UART-DISC] 0x%02X\n", b);
            }
            return;  // discard, wait for start byte
        }
    }

    // Guard against buffer overflow (defensive — the parse loop below keeps
    // head below the largest accepted frame size).
    if (parser.head >= (int)sizeof(parser.buf)) {
        parser.head = 0;
        return;
    }

    parser.buf[parser.head] = b;
    parser.head++;

    // Parse as many complete frames as the window holds. After a resync the
    // window can already contain one or more complete packets, so loop until
    // the leading frame is incomplete. parser_resync() guarantees buf[0] is a
    // start byte whenever head > 0.
    for (;;) {
        if (parser.head == 0) return;

        // Basic packet?
        if (parser.buf[0] == UART_START_BASIC) {
            if (parser.head < UART_BASIC_LEN) return;
            UartBasicPacket* pkt = (UartBasicPacket*)parser.buf;
            bool is_heartbeat = (pkt->type == MSG_SYSTEM && pkt->id == SYS_HEARTBEAT);
            if (!p4.s3_connected && !is_heartbeat) {
                parser_consume(parser, UART_BASIC_LEN);
                continue;
            }
            if (uart_validate_packet(pkt)) {
                process_basic(pkt, from_usb);
                uart_stats.rx_packets++;
                s_processed++;
                parser_consume(parser, UART_BASIC_LEN);
            } else {
                if (link_active || is_heartbeat) uart_stats.rx_checksum_errors++;
                parser_resync(parser);
            }
            continue;
        }

        // Extended packet?
        if (parser.buf[0] == UART_START_EXTENDED) {
            if (parser.head < UART_EXT_HEADER_LEN) return;
            UartExtendedHeader* hdr = (UartExtendedHeader*)parser.buf;
            int payload_len = ((int)hdr->len_h << 8) | hdr->len_l;
            int total = UART_EXT_HEADER_LEN + payload_len + 1; // +1 for checksum

            // Reject oversized payload up front (also catches garbage in len bytes)
            if (payload_len > UART_EXT_MAX_PAYLOAD || total > (int)sizeof(parser.buf)) {
                if (link_active) uart_stats.rx_checksum_errors++;
                parser_resync(parser);
                continue;
            }
            if (parser.head < total) return;

            // Validate checksum (sum of all bytes)
            uint8_t sum = 0;
            for (int i = 0; i < total - 1; i++) sum += parser.buf[i];
            if (sum == parser.buf[total - 1]) {
                process_extended(hdr->type, hdr->id,
                                 &parser.buf[UART_EXT_HEADER_LEN], payload_len);
                uart_stats.rx_packets++;
                s_processed++;
                parser_consume(parser, total);
            } else {
                if (link_active) uart_stats.rx_checksum_errors++;
                parser_resync(parser);
            }
            continue;
        }

        // head > 0 without a start byte at [0] — shouldn't happen, resync.
        parser_resync(parser);
        if (parser.head > 0 &&
            parser.buf[0] != UART_START_BASIC && parser.buf[0] != UART_START_EXTENDED) {
            parser.head = 0;
            return;
        }
    }
}

int uart_handler_process(void) {
#if P4_STANDALONE_MASTER_ONLY
    p4.s3_connected = false;
    return 0;
#endif

    s_processed = 0;

    // Check heartbeat timeout
    if (p4.s3_connected && (millis() - p4.last_heartbeat_ms) > Config::HEARTBEAT_TIMEOUT_MS) {
        p4.s3_connected = false;
        s_hb_streak = 0;
        s_hb_source = -1;
        // Reset framing-error display throttle so [UART-DISC] logs resume
        // for diagnosing the recovery window.
        s_totalDiscarded = 0;
        P4_LOG_PRINTLN("[UART] S3 heartbeat lost!");
    }

    // Proactive heartbeat: even when S3 is silent, poke the link every 1s so
    // a partially-alive S3 (e.g. display thread busy) gets a chance to reply
    // and we keep the USB/UART endpoint warm. Cheap (1 byte per second).
    {
        static unsigned long lastTx = 0;
        unsigned long nowMs = millis();
        if (nowMs - lastTx >= 1000) {
            lastTx = nowMs;
            uart_send_to_s3(MSG_SYSTEM, SYS_HEARTBEAT, 0x01);
        }
    }

    // Debug: periodic status
    {
        static unsigned long lastDbg = 0;
        if (millis() - lastDbg >= 3000) {
            lastDbg = millis();
            int avail = UartS3.available();
#if P4_USB_CDC_ENABLED
            int usbAvail = usb_cdc_available();
            P4_LOG_PRINTF("[UART-DBG] uart=%d usb=%d s3=%d disc=%d hb=%lums | USB: %s\n",
                          avail, usbAvail, p4.s3_connected, s_totalDiscarded,
                          millis() - p4.last_heartbeat_ms,
                          usb_cdc_status_str());
#else
            P4_LOG_PRINTF("[UART-DBG] avail=%d s3_conn=%d disc=%d last_hb=%lums ago\n",
                          avail, p4.s3_connected, s_totalDiscarded,
                          millis() - p4.last_heartbeat_ms);
#endif
        }
    }

    // Read from UART
    while (UartS3.available()) {
        feed_byte(s_uart_rx, (uint8_t)UartS3.read(), false);
    }

#if P4_USB_CDC_ENABLED
    // Read from USB CDC (S3 via USB-C)
    while (usb_cdc_available()) {
        int b = usb_cdc_read();
        if (b >= 0) feed_byte(s_usb_rx, (uint8_t)b, true);
    }
#endif

    return s_processed;
}

void uart_lock_tempo(uint32_t duration_ms) {
    s_tempo_lock_until_ms = millis() + duration_ms;
}

// Stage a pattern push to the Master from a raw 16×16 step grid.
// Used by the MEM MIDI loader on P4 — keeps P4 as the sole owner of the
// master UDP channel regardless of whether the MIDI came from SD (via S3)
// or from local SPIFFS.
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
    if (s_s3_preview_note_off_due_ms != 0 && (int32_t)(millis() - s_s3_preview_note_off_due_ms) >= 0) {
        if (udp_wifi_connected()) udp_send_synth_note_off(s_s3_preview_engine, 0);
        s_s3_preview_note_off_due_ms = 0;
    }

    PendingPushPhase phase = s_push.phase.load(std::memory_order_acquire);
    if (phase == PP_IDLE) {
        if (!activate_queued_push()) return;
    } else if (phase == PP_STAGING) {
        return;
    }
    if (!udp_wifi_connected()) {
        s_push.phase = PP_IDLE;
        portENTER_CRITICAL(&s_push_queue_mux);
        s_queued_push_valid = false;
        portEXIT_CRITICAL(&s_push_queue_mux);
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
