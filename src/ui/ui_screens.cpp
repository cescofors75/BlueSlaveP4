// =============================================================================
// ui_screens.cpp — P4 UI screens (data-driven from UART state)
// All screen rendering reads from P4State (p4.*) — no direct hardware access.
// =============================================================================

#include "ui_screens.h"
#include "ui_theme.h"
#include "../drivers/lvgl_port.h"
#include "../udp_handler.h"
#include "../uart_handler.h"
#include "../dsp_task.h"
#include "../mem_midi_loader.h"
#include "../sample_edit.h"
#include "config.h"
#include "../../../shared/synth_params.h"
#include <Arduino.h>
#include <SD_MMC.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <atomic>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

// ── IntelliSense fallbacks ───────────────────────────────────────────────────
// The real values are provided via -D flags in platformio.ini and via
// config.h / lv_conf.h. These fallbacks only kick in for editor analysis
// when IntelliSense can't resolve the build system's include/define graph.
#ifndef LCD_H_RES
#define LCD_H_RES 1024
#endif
#ifndef LCD_V_RES
#define LCD_V_RES 600
#endif
#ifndef UI_H
#define UI_H LCD_V_RES
#endif
#if defined(__INTELLISENSE__)
LV_FONT_DECLARE(lv_font_montserrat_10)
#endif

// ── Pad event queue: touch_task (Core 0) → loop (Core 1) ──
// Each entry packs (velocity << 8) | pad to carry MPC-style velocity all the
// way to UDP without adding a parallel array. Decouples UDP/UART send from
// the LVGL mutex.
static volatile uint16_t s_pad_q[32];
static std::atomic<uint8_t> s_pad_qh{0};
static std::atomic<uint8_t> s_pad_qt{0};
static std::atomic<uint32_t> s_pad_q_drops{0};

static std::atomic<uint16_t> s_ctrl_mute_dirty{0};
static std::atomic<uint16_t> s_ctrl_mute_values{0};
static std::atomic<bool>     s_ctrl_mute_mask_pending{false};
static std::atomic<uint16_t> s_ctrl_mute_mask{0};
static std::atomic<bool>     s_ctrl_solo_mask_pending{false};
static std::atomic<uint16_t> s_ctrl_solo_mask{0};

// Touch debounce tuned for GT911 + multi-indev setup.
static const uint32_t MUTE_DEBOUNCE_TRACK_MS = 180;
static const uint32_t MUTE_DEBOUNCE_GLOBAL_MS = 60;
static const uint32_t SOLO_DEBOUNCE_TRACK_MS = 220;
static const uint32_t SOLO_DEBOUNCE_GLOBAL_MS = 70;

// Direct touch bypass: flag used by touch_task to early-out when not on LIVE
static std::atomic<bool> g_live_screen_active{false};

// The pad ring has a single consumer (Core 1 loop) but TWO producers, both on
// Core 0: the GT911 touch task (prio 6) and LVGL event callbacks (prio 5, e.g.
// pad-instrument assignment). Because the touch task can preempt the LVGL task
// mid-enqueue, the head update is not safe as a plain SPSC store. Serialize the
// producers with a spinlock; the consumer still needs no lock because head is
// published (release) only after the slot is written.
static portMUX_TYPE s_pad_q_mux = portMUX_INITIALIZER_UNLOCKED;
static inline void enqueue_pad_event(uint8_t pad, uint8_t velocity) {
    portENTER_CRITICAL(&s_pad_q_mux);
    uint8_t h = s_pad_qh.load(std::memory_order_relaxed);
    uint8_t t = s_pad_qt.load(std::memory_order_acquire);
    if ((uint8_t)(h - t) >= 32) {
        s_pad_q_drops.fetch_add(1, std::memory_order_relaxed);
        portEXIT_CRITICAL(&s_pad_q_mux);
        return;
    }
    s_pad_q[h & 0x1F] = (uint16_t)((velocity << 8) | pad);
    s_pad_qh.store(h + 1, std::memory_order_release);
    portEXIT_CRITICAL(&s_pad_q_mux);
}

static inline void enqueue_mute_control(uint8_t track, bool muted) {
    if (track >= 16) return;
    uint16_t bit = (uint16_t)(1U << track);
    uint16_t values = s_ctrl_mute_values.load(std::memory_order_relaxed);
    values = muted ? (uint16_t)(values | bit) : (uint16_t)(values & ~bit);
    s_ctrl_mute_values.store(values, std::memory_order_release);
    s_ctrl_mute_dirty.fetch_or(bit, std::memory_order_release);
}

static inline void enqueue_mute_mask_control(uint16_t mask) {
    s_ctrl_mute_dirty.store(0, std::memory_order_release);
    s_ctrl_mute_mask.store(mask, std::memory_order_release);
    s_ctrl_mute_mask_pending.store(true, std::memory_order_release);
}

static inline void enqueue_solo_mask_control(uint16_t mask) {
    s_ctrl_solo_mask.store(mask, std::memory_order_release);
    s_ctrl_solo_mask_pending.store(true, std::memory_order_release);
}

// =============================================================================
// MPC-STYLE PLAYBACK STATE — note repeat, 16 levels, velocity fade
// =============================================================================
// Note repeat: subdivisions per beat = {1, 2, 4, 8, 3, 6} for
// 1/4, 1/8, 1/16, 1/32, 1/8T, 1/16T respectively. Index 0 = OFF.
static const uint8_t     NR_SUBDIV_PER_BEAT[7] = {0, 1, 2, 4, 8, 3, 6};
static const char* const NR_LABEL[7] = {"NR\nOFF", "NR\n1/4", "NR\n1/8",
                                         "NR\n1/16", "NR\n1/32",
                                         "NR\n1/8T", "NR\n1/16T"};
static volatile uint8_t s_nr_idx = 0;                  // 0 = OFF

// 16 Levels: all 16 pads become 16 velocities of a single source pad
static volatile bool    s_16l_active   = false;
static volatile uint8_t s_16l_src_pad  = 0;            // last non-16L tap

// Per-pad state, written by touch_task (Core 0) and read by update_live_screen
// (Core 0 LVGL task). Single writer / single reader per field → no locks.
static volatile bool          s_pad_held[16] = {};
static volatile unsigned long s_pad_repeat_next_ms[16] = {};
static volatile uint8_t       s_pad_held_velocity[16] = {};
static volatile uint8_t       s_pad_hold_x[16] = {};
static volatile uint8_t       s_pad_hold_y[16] = {};
static volatile unsigned long s_pad_hold_start_ms[16] = {};
static volatile uint8_t       s_pad_roll_phase[16] = {};

static const unsigned long PAD_TREMOLO_HOLD_MS = 165;
static const unsigned long PAD_TREMOLO_FAST_MS = 42;
static const unsigned long PAD_TREMOLO_SLOW_MS = 235;

// Velocity fade visualisation: each press stores (velocity, start_ms) and
// update_live_screen interpolates an exponential decay over FADE_MS to drive
// the pad background opacity. Quantised into 8 brightness bands so LVGL only
// re-invalidates a pad rect when the band actually changes (keeps partial
// refresh cheap even with 16 pads decaying at once).
static const int FADE_MS = 320;
static volatile uint8_t       s_pad_flash_vel[16] = {};
static volatile unsigned long s_pad_flash_start_ms[16] = {};

static inline void ui_pad_flash_start(uint8_t pad, uint8_t velocity) {
    if (pad >= 16) return;
    s_pad_flash_vel[pad]      = velocity ? velocity : 1;
    s_pad_flash_start_ms[pad] = millis();
}

static inline uint8_t ui_live_pad_velocity(void);

static unsigned long ui_nr_interval_ms(void) {
    // Use current tempo from P4State (UART-synced). BPM can be 0 briefly at
    // boot; clamp to 40..300 for safety.
    extern struct P4State p4;
    int bpm_x10 = p4.bpm_int * 10 + p4.bpm_frac;
    if (bpm_x10 < 400)  bpm_x10 = 1200;
    if (bpm_x10 > 3000) bpm_x10 = 3000;
    uint8_t idx = s_nr_idx;
    if (idx == 0 || idx >= (sizeof(NR_SUBDIV_PER_BEAT) / sizeof(NR_SUBDIV_PER_BEAT[0]))) return 0;
    uint32_t div = NR_SUBDIV_PER_BEAT[idx];
    // interval = 60000 ms / (bpm * div). bpm_x10 is BPM*10, so:
    //   ms = 600000 / (bpm_x10 * div)
    unsigned long ms = 600000UL / ((unsigned long)bpm_x10 * div);
    if (ms < 15) ms = 15;   // safety floor (~66 Hz max retrigger)
    return ms;
}

static unsigned long ui_pad_tremolo_interval_ms(uint8_t pad, unsigned long nr_interval) {
    if (nr_interval) return nr_interval;
    if (pad >= 16) return 0;
    uint8_t x = s_pad_hold_x[pad];
    if (x > 127) x = 127;
    return PAD_TREMOLO_SLOW_MS - (((PAD_TREMOLO_SLOW_MS - PAD_TREMOLO_FAST_MS) * (unsigned long)x) / 127UL);
}

static uint8_t ui_pad_tremolo_velocity(uint8_t pad, unsigned long now_ms) {
    if (pad >= 16) return 100;
    uint8_t y = s_pad_hold_y[pad];
    if (y > 127) y = 127;
    uint16_t base = ui_live_pad_velocity();
    uint16_t amp = 34 + (((uint16_t)(127 - y) * 93U) / 127U);
    uint16_t vel = (base * amp) / 127U;

    unsigned long held_ms = now_ms - s_pad_hold_start_ms[pad];
    if (held_ms < 420UL) {
        vel = (vel * (64U + (uint16_t)((held_ms * 63UL) / 420UL))) / 127U;
    }

    static const int8_t wobble[8] = {0, 5, 9, 5, 0, -4, -7, -4};
    uint8_t phase = s_pad_roll_phase[pad];
    uint8_t depth = 2 + (uint8_t)(((uint16_t)(127 - y) * 10U) / 127U);
    int16_t shaped = (int16_t)vel + (int16_t)((wobble[phase & 0x07] * (int8_t)depth) / 4);
    if (shaped < 8) shaped = 8;
    if (shaped > 127) shaped = 127;
    return (uint8_t)shaped;
}


// Screen objects
lv_obj_t* scr_boot = NULL;
lv_obj_t* scr_live = NULL;
lv_obj_t* scr_sequencer = NULL;
lv_obj_t* scr_fx = NULL;
lv_obj_t* scr_volumes = NULL;
lv_obj_t* scr_sdcard = NULL;
lv_obj_t* scr_performance = NULL;
lv_obj_t* scr_piano = NULL;       /* v2.6 — PIANO live keyboard */
lv_obj_t* scr_piano_params = NULL; /* v2.7 — synth engine parameter editor */

// Header widgets
static lv_obj_t* header_bar = NULL;
static lv_obj_t* hdr_bpm_label = NULL;
static lv_obj_t* hdr_pattern_label = NULL;
static lv_obj_t* hdr_play_btn = NULL;
static lv_obj_t* hdr_play_label = NULL;
static lv_obj_t* hdr_pattern_minus_btn = NULL;
static lv_obj_t* hdr_pattern_plus_btn = NULL;
static lv_obj_t* hdr_wifi_label = NULL;
static lv_obj_t* hdr_s3_label = NULL;
static lv_obj_t* hdr_step_dots[16] = {};

// Current active screen index + history for back navigation
static int active_screen = 0;
static int prev_active_screen = 0;

// Track names
static const char* trackNames[] = {
    "BD", "SD", "CH", "OH", "CP", "CB", "RS", "CL",
    "MA", "CY", "HT", "LT", "MC", "MT", "HC", "LC"
};

static int ui_layout_w(void) {
    return LCD_H_RES;
}

static int ui_layout_h(void) {
    // The current panel path renders 1024x600 without framebuffer rotation.
    // Keep the live layout inside the visible 600px height.
    return (UI_H > LCD_V_RES) ? LCD_V_RES : UI_H;
}

static inline lv_color_t ui_track_color(int track) {
    return lv_color_hex(theme_presets[currentTheme].track_colors[track & 0x0F]);
}

static inline bool ui_track_color_is_light(int track) {
    uint32_t c = theme_presets[currentTheme].track_colors[track & 0x0F];
    uint8_t r = (uint8_t)((c >> 16) & 0xFF);
    uint8_t g = (uint8_t)((c >> 8) & 0xFF);
    uint8_t b = (uint8_t)(c & 0xFF);
    return ((uint16_t)r + (uint16_t)g + (uint16_t)b) > 560;
}

static inline lv_color_t ui_track_label_color(int track, bool lit) {
    return (lit && ui_track_color_is_light(track)) ? RED808_BG : ui_track_color(track);
}

static inline uint32_t ui_tremolo_neon_hex(uint8_t x, uint8_t y) {
    uint8_t amp = (uint8_t)(127U - (y > 127 ? 127 : y));
    if (x < 32)  return (amp > 84) ? 0xFF3324 : 0xC9271B;
    if (x < 64)  return (amp > 72) ? 0xFF6A2A : 0xE86820;
    if (x < 96)  return (amp > 60) ? 0xFFD052 : 0xF5BC31;
    return (amp > 48) ? 0xFFF7E8 : 0xF7EAD7;
}

// =============================================================================
// HELPER: Section shell (styled container)
// =============================================================================
lv_obj_t* create_section_shell(lv_obj_t* parent, int x, int y, int w, int h) {
    lv_obj_t* obj = lv_obj_create(parent);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_style_bg_color(obj, RED808_PANEL, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_80, 0);
    lv_obj_set_style_border_width(obj, 1, 0);
    lv_obj_set_style_border_color(obj, RED808_BORDER, 0);
    lv_obj_set_style_radius(obj, 8, 0);
    lv_obj_set_style_pad_all(obj, 14, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    return obj;
}

static bool ui_use_udp_transport(void) {
    return p4.wifi_connected || p4.master_connected;
}

static bool ui_master_link_display_on(void) {
    static bool shown = false;
    static bool pending = false;
    static uint32_t pending_since = 0;

    bool raw = p4.wifi_connected || p4.master_connected;
    uint32_t now = millis();
    uint32_t settle_ms = raw ? 350UL : 2200UL;

    if (raw == shown) {
        pending = raw;
        pending_since = now;
        return shown;
    }

    if (raw != pending) {
        pending = raw;
        pending_since = now;
        return shown;
    }

    if ((uint32_t)(now - pending_since) >= settle_ms) shown = raw;
    return shown;
}

static void apply_control_button_style(lv_obj_t* button, lv_color_t accent,
                                       bool filled, int radius) {
    if (!button) return;
    lv_obj_set_style_radius(button, radius > 8 ? 8 : radius, 0);
    lv_obj_set_style_bg_color(button, filled ? accent : RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_color(button, filled ? RED808_SURFACE : RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_dir(button, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(button, 2, 0);
    lv_obj_set_style_border_color(button, accent, 0);
    lv_obj_set_style_border_opa(button, filled ? LV_OPA_COVER : LV_OPA_70, 0);
    lv_obj_set_style_shadow_width(button, 0, 0);
    lv_obj_set_style_bg_color(button, accent, LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(button, (lv_opa_t)216, LV_STATE_PRESSED);
    lv_obj_clear_flag(button, LV_OBJ_FLAG_SCROLLABLE);
}

static lv_obj_t* create_header_button(lv_obj_t* parent, int x, int y, int w, int h,
                                      const char* text, lv_color_t bg, lv_color_t border) {
    lv_obj_t* btn = lv_btn_create(parent);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_pos(btn, x, y);
    apply_control_button_style(btn, border, true, 12);
    lv_obj_set_style_bg_color(btn, bg, 0);

    lv_obj_t* label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(label, RED808_TEXT, 0);
    lv_obj_center(label);
    return btn;
}

static lv_obj_t* s_ui_toast = NULL;
static lv_obj_t* s_ui_toast_label = NULL;
static uint32_t s_ui_toast_until_ms = 0;

// Full-screen busy overlay used to give feedback during a blocking operation
// (e.g. uploading a WAV to the Master, which runs synchronously on the UI
// thread). It also blocks navigation so the user can't wander off mid-upload
// and lose the result toast.
static lv_obj_t* s_busy_overlay = NULL;
static lv_obj_t* s_busy_lbl     = NULL;
static bool      s_ui_busy      = false;

static void ui_toast_hide(void) {
    if (s_ui_toast) lv_obj_add_flag(s_ui_toast, LV_OBJ_FLAG_HIDDEN);
    s_ui_toast_until_ms = 0;
}

static void ui_toast_update(void) {
    if (s_ui_toast && s_ui_toast_until_ms && millis() > s_ui_toast_until_ms) {
        ui_toast_hide();
    }
}

static void ui_show_toast(const char* text, lv_color_t accent) {
    // Parent to the top layer (shared across all screens) so a toast raised
    // just before — or during — a screen change still shows instead of being
    // destroyed with the previous screen.
    lv_obj_t* parent = lv_layer_top();
    if (!parent) return;

    if (!s_ui_toast || lv_obj_get_parent(s_ui_toast) != parent) {
        // Toast belongs to a different (still-alive) screen — delete it so we
        // don't leave orphan toast+label objects behind on every navigation.
        if (s_ui_toast) {
            lv_obj_del(s_ui_toast);
            s_ui_toast = NULL;
            s_ui_toast_label = NULL;
        }
        s_ui_toast = lv_obj_create(parent);
        lv_obj_set_size(s_ui_toast, 420, 62);
        lv_obj_align(s_ui_toast, LV_ALIGN_TOP_MID, 0, 54);
        lv_obj_set_style_radius(s_ui_toast, 8, 0);
        lv_obj_set_style_bg_color(s_ui_toast, RED808_PANEL, 0);
        lv_obj_set_style_bg_opa(s_ui_toast, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(s_ui_toast, 2, 0);
        lv_obj_set_style_shadow_width(s_ui_toast, 18, 0);
        lv_obj_set_style_shadow_color(s_ui_toast, lv_color_hex(0x000000), 0);
        lv_obj_set_style_pad_hor(s_ui_toast, 18, 0);
        lv_obj_set_style_pad_ver(s_ui_toast, 12, 0);
        lv_obj_clear_flag(s_ui_toast, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(s_ui_toast, LV_OBJ_FLAG_CLICKABLE);

        s_ui_toast_label = lv_label_create(s_ui_toast);
        lv_obj_set_width(s_ui_toast_label, 384);
        lv_label_set_long_mode(s_ui_toast_label, LV_LABEL_LONG_DOT);
        lv_obj_set_style_text_font(s_ui_toast_label, &lv_font_montserrat_18, 0);
        lv_obj_center(s_ui_toast_label);
    }

    lv_label_set_text(s_ui_toast_label, text);
    lv_obj_set_style_border_color(s_ui_toast, accent, 0);
    lv_obj_set_style_text_color(s_ui_toast_label, RED808_TEXT, 0);
    lv_obj_move_foreground(s_ui_toast);
    lv_obj_clear_flag(s_ui_toast, LV_OBJ_FLAG_HIDDEN);
    s_ui_toast_until_ms = millis() + 1800U;
}

// Show a modal "busy" overlay and paint it immediately. Because the work that
// follows blocks the LVGL thread, we force one synchronous redraw here so the
// message is actually on screen before we freeze.
static void ui_busy_show(const char* text) {
    lv_obj_t* top = lv_layer_top();
    if (!top) return;
    if (!s_busy_overlay) {
        s_busy_overlay = lv_obj_create(top);
        lv_obj_remove_style_all(s_busy_overlay);
        lv_obj_set_size(s_busy_overlay, LV_PCT(100), LV_PCT(100));
        lv_obj_set_style_bg_color(s_busy_overlay, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(s_busy_overlay, LV_OPA_70, 0);
        lv_obj_add_flag(s_busy_overlay, LV_OBJ_FLAG_CLICKABLE);  // eat touches
        lv_obj_clear_flag(s_busy_overlay, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* card = lv_obj_create(s_busy_overlay);
        lv_obj_set_size(card, 480, 130);
        lv_obj_center(card);
        lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
        lv_obj_set_style_border_color(card, RED808_CYAN, 0);
        lv_obj_set_style_border_width(card, 2, 0);
        lv_obj_set_style_radius(card, 10, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        s_busy_lbl = lv_label_create(card);
        lv_obj_set_width(s_busy_lbl, 440);
        lv_label_set_long_mode(s_busy_lbl, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_align(s_busy_lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(s_busy_lbl, &lv_font_montserrat_18, 0);
        lv_obj_set_style_text_color(s_busy_lbl, RED808_TEXT, 0);
        lv_obj_center(s_busy_lbl);
    }
    if (s_busy_lbl) lv_label_set_text(s_busy_lbl, text);
    lv_obj_clear_flag(s_busy_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_busy_overlay);
    s_ui_busy = true;
    lv_refr_now(NULL);   // paint before the caller blocks the UI thread
}

static void ui_busy_hide(void) {
    s_ui_busy = false;
    if (s_busy_overlay) lv_obj_add_flag(s_busy_overlay, LV_OBJ_FLAG_HIDDEN);
}

static void seq_pattern_modal_show(int pattern);
static void seq_pattern_modal_hide(void);
static void seq_pattern_modal_mark_loaded(void);

static void header_play_cb(lv_event_t* e) {
    LV_UNUSED(e);
    // Debounce — LVGL can double-fire on a sloppy tap, causing visible
    // start/stop/start flicker and duplicate UDP bursts.
    static uint32_t last_ms = 0;
    uint32_t now = millis();
    if (now - last_ms < 250) return;
    last_ms = now;

    bool next_play = !p4.is_playing;
    // P4 is the UDP owner for its own play/pause toggle — send once.
    if (next_play) udp_send_start();
    else           udp_send_stop();
    // Notify S3 of the new state so its UI & isPlaying mirror us WITHOUT
    // triggering another UDP start/stop from S3 (that duplication is what
    // made the two sequencers fall out of sync on every P4 tap).
    uart_send_to_s3(MSG_SYSTEM, SYS_PLAY_STATE, next_play ? 1 : 0);
    // P4 owns step clock: reset phase explicitly on every transport toggle.
    uart_send_to_s3(MSG_SYSTEM, SYS_STEP, 0);
    p4.current_step = 0;
    p4.is_playing = next_play;
}

static void header_clear_track_isolation(void) {
    if (!udp_wifi_connected()) return;
    for (int track = 0; track < 16; track++) {
        bool was_solo  = p4.track_solo[track];
        bool was_muted = p4.track_muted[track];
        p4.track_solo[track]  = false;
        p4.track_muted[track] = false;
        // Only send UDP if state was actually non-zero — avoids flooding
        // the socket buffer with 32 no-op packets before selectPattern.
        if (was_solo)  udp_send_solo(track, false);
        if (was_muted) udp_send_mute(track, false);
    }
}

static void header_pattern_cb(lv_event_t* e) {
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    int next_pattern = p4.current_pattern + delta;
    if (next_pattern < 0) next_pattern = Config::MAX_PATTERNS - 1;
    if (next_pattern >= Config::MAX_PATTERNS) next_pattern = 0;

    p4.current_pattern = next_pattern;
    ui_sequencer_sync_from_current_pattern();
    if ((active_screen == 3 || lv_scr_act() == scr_sequencer) &&
            (udp_wifi_connected() || udp_master_connected())) {
        seq_pattern_modal_show(next_pattern);
    } else {
        seq_pattern_modal_hide();
    }
#if !P4_STANDALONE_MASTER_ONLY
    // Avoid overwriting the selected pattern with stale S3 cache while
    // UDP master sync is active.
    if (!udp_wifi_connected() && !udp_master_connected()) {
        uart_restore_cached_pattern((uint8_t)next_pattern);
    }
#endif
    header_clear_track_isolation();
    // Always push selection/request to Master; sendJson() already no-ops when
    // UDP is not started, and this avoids stale UI transport flags blocking
    // pattern changes.
    udp_send_select_pattern(next_pattern);
    udp_send_get_pattern(next_pattern);
    if (p4.s3_connected) {
        // S3 owns local/demo patterns. Ask it to push the selected grid back
        // through MSG_PATTERN_PUSH only when the slot has real local data.
        uart_send_to_s3(MSG_TOUCH_CMD, TCMD_PATTERN_SEL, (uint8_t)next_pattern);
    }
}

// =============================================================================
// BACK BUTTON — replaces the old header bar (floating top-left corner)
// =============================================================================
void ui_create_header(lv_obj_t* parent) {
    // Nullify all header widget pointers — not used anymore
    header_bar = NULL;
    hdr_bpm_label = NULL; hdr_pattern_label = NULL;
    hdr_play_btn = NULL; hdr_play_label = NULL;
    hdr_pattern_minus_btn = NULL; hdr_pattern_plus_btn = NULL;
    hdr_wifi_label = NULL; hdr_s3_label = NULL;
    for (int i = 0; i < 16; i++) hdr_step_dots[i] = NULL;

    // Small floating back button (top-left)
    lv_obj_t* back_btn = lv_btn_create(parent);
    lv_obj_set_size(back_btn, 44, 34);
    lv_obj_set_pos(back_btn, 8, 8);
    apply_control_button_style(back_btn, RED808_BORDER, false, 8);
    lv_obj_add_event_cb(back_btn, [](lv_event_t* e) {
        LV_UNUSED(e);
        if (active_screen != 2) ui_navigate_to(2);
    }, LV_EVENT_CLICKED, NULL);
    lv_obj_t* back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_font(back_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(back_lbl, RED808_TEXT, 0);
    lv_obj_center(back_lbl);
}

void ui_update_header(void) {
    static int prev_bpm = -1, prev_frac = -1, prev_pat = -1;
    static bool prev_play = false, prev_wifi = false, prev_s3 = false;

    if (p4.bpm_int != prev_bpm || p4.bpm_frac != prev_frac) {
        prev_bpm = p4.bpm_int;
        prev_frac = p4.bpm_frac;
        if (hdr_bpm_label) lv_label_set_text_fmt(hdr_bpm_label, "%d.%d", p4.bpm_int, p4.bpm_frac);
    }

    if (p4.current_pattern != prev_pat) {
        prev_pat = p4.current_pattern;
        if (hdr_pattern_label) lv_label_set_text_fmt(hdr_pattern_label, "P%02d", p4.current_pattern + 1);
    }

    if (p4.is_playing != prev_play) {
        prev_play = p4.is_playing;
        if (hdr_play_btn && hdr_play_label) {
            lv_label_set_text(hdr_play_label, p4.is_playing ? "PAUSE" : "PLAY");
            lv_obj_set_style_bg_color(hdr_play_btn, p4.is_playing ? RED808_SUCCESS : RED808_ACCENT, 0);
            lv_obj_set_style_border_color(hdr_play_btn, p4.is_playing ? RED808_CYAN : RED808_ACCENT2, 0);
        }
    }

    if (p4.wifi_connected != prev_wifi) {
        prev_wifi = p4.wifi_connected;
        if (hdr_wifi_label) {
            lv_label_set_text(hdr_wifi_label, p4.wifi_connected ? "NET OK" : "NET OFF");
            lv_obj_set_style_text_color(hdr_wifi_label,
                p4.wifi_connected ? RED808_SUCCESS : RED808_ERROR, 0);
        }
    }

    if (p4.s3_connected != prev_s3) {
        prev_s3 = p4.s3_connected;
        if (hdr_s3_label) {
            lv_label_set_text(hdr_s3_label, p4.s3_connected ? "AUX ON" : "AUX OFF");
            lv_obj_set_style_text_color(hdr_s3_label,
                p4.s3_connected ? RED808_INFO : RED808_TEXT_DIM, 0);
        }
    }
}

// =============================================================================
// BOOT SCREEN
// =============================================================================
// Radial gradient descriptor for the boot halo. Must outlive the object since
// LVGL stores a pointer to it (not a copy), so keep it at file scope.
static lv_grad_dsc_t s_boot_halo_grad;

static void create_boot_screen(void) {
    scr_boot = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_boot, RED808_BG, 0);
    lv_obj_clear_flag(scr_boot, LV_OBJ_FLAG_SCROLLABLE);

    // ── Radial glow halo behind the logo (LVGL 9 radial gradient + additive
    // blend). Bright cyan at the centre fading to transparent at the edge; the
    // additive blend makes it read as emitted light over the dark background.
    {
        const int R = 190;  // halo radius (object is 2R x 2R)
        s_boot_halo_grad.dir = LV_GRAD_DIR_RADIAL;
        s_boot_halo_grad.extend = LV_GRAD_EXTEND_PAD;
        s_boot_halo_grad.stops_count = 2;
        s_boot_halo_grad.stops[0].color = RED808_CYAN;
        s_boot_halo_grad.stops[0].opa   = LV_OPA_70;  // soft centre keeps title readable
        s_boot_halo_grad.stops[0].frac  = 0;     // centre: brightest
        s_boot_halo_grad.stops[1].color = RED808_CYAN;
        s_boot_halo_grad.stops[1].opa   = LV_OPA_TRANSP;
        s_boot_halo_grad.stops[1].frac  = 255;   // edge: transparent
        // Concentric radial: focal circle radius 0 at centre, end circle radius R.
        s_boot_halo_grad.params.radial.focal.x = R;
        s_boot_halo_grad.params.radial.focal.y = R;
        s_boot_halo_grad.params.radial.focal_extent.x = R;
        s_boot_halo_grad.params.radial.focal_extent.y = R;
        s_boot_halo_grad.params.radial.end.x = R;
        s_boot_halo_grad.params.radial.end.y = R;
        s_boot_halo_grad.params.radial.end_extent.x = 2 * R;  // distance R from centre
        s_boot_halo_grad.params.radial.end_extent.y = R;

        lv_obj_t* halo = lv_obj_create(scr_boot);
        lv_obj_remove_style_all(halo);
        lv_obj_set_size(halo, 2 * R, 2 * R);
        lv_obj_align(halo, LV_ALIGN_CENTER, 0, -10);
        lv_obj_set_style_radius(halo, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_opa(halo, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_grad(halo, &s_boot_halo_grad, 0);
        lv_obj_set_style_blend_mode(halo, LV_BLEND_MODE_ADDITIVE, 0);
        lv_obj_clear_flag(halo, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(halo, LV_OBJ_FLAG_CLICKABLE);
    }

    lv_obj_t* title = lv_label_create(scr_boot);
    lv_label_set_text(title, "BLUE808 SLAVE P4");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_40, 0);
    lv_obj_set_style_text_color(title, RED808_CYAN, 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -40);

    lv_obj_t* sub = lv_label_create(scr_boot);
    lv_label_set_text(sub, "Connecting to RED808 Master...");
    lv_obj_set_style_text_font(sub, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(sub, RED808_TEXT_DIM, 0);
    lv_obj_align(sub, LV_ALIGN_CENTER, 0, 20);

    lv_obj_t* spinner = lv_spinner_create(scr_boot);
    lv_spinner_set_anim_params(spinner, 1000, 60);
    lv_obj_set_size(spinner, 60, 60);
    lv_obj_align(spinner, LV_ALIGN_CENTER, 0, 80);
}

// =============================================================================
// LIVE GRID SCREEN — 8×4 full-screen grid (4×4 pads + 4×4 controls)
// =============================================================================
static lv_obj_t* live_pad_btns[16] = {};
static lv_obj_t* live_pad_labels[16] = {};
static lv_obj_t* live_pad_num_labels[16] = {};
static lv_obj_t* live_pad_state_labels[16] = {};
static lv_obj_t* live_pad_inst_labels[16] = {};
static lv_obj_t* live_pad_accent_strips[16] = {};
static lv_obj_t* live_pad_glow[16] = {};       // additive-blend glow overlay per pad
static lv_obj_t* live_spectrum_bars[16] = {};  // spectrum bar per pad (bottom of pad)
static lv_obj_t* live_home_panels[24] = {};
static int       live_home_panel_count = 0;

// Pad layout mode: 0=16 normal, 1=16 FS, 2=8 FS, 3=4 FS, 4=2 FS, 5=1 FS
static int        s_pad_mode      = 0;
static lv_obj_t*  s_pad_back_btn  = NULL;
static lv_obj_t*  s_pad_mode_modal = NULL;

// Right-side control widgets (dynamic updates)
static lv_obj_t* grid_play_btn = NULL;
static lv_obj_t* grid_play_lbl = NULL;
static lv_obj_t* grid_bpm_lbl = NULL;
static lv_obj_t* grid_home_vol_lbl = NULL;
static lv_obj_t* grid_home_status_cell = NULL;  // STATUS cell — tinted by master link
static lv_obj_t* grid_pat_lbl = NULL;
static lv_obj_t* grid_step_lbl = NULL;
static lv_obj_t* grid_step_dots[16] = {};
static lv_obj_t* grid_nr_btn  = NULL;   // Note Repeat toggle + subdivision cycler
static lv_obj_t* grid_nr_lbl  = NULL;
static lv_obj_t* grid_16l_btn = NULL;   // 16 Levels toggle
static lv_obj_t* grid_16l_lbl = NULL;
// Link status indicators (replaces the old "LIVE" badge)
static lv_obj_t* grid_mstr_dot = NULL;  // Master (UDP to ESP32-C6 AP) link
static lv_obj_t* grid_mstr_lbl = NULL;
static lv_obj_t* grid_aux_dot  = NULL;  // Aux (UART to ESP32-S3) link
static lv_obj_t* grid_aux_lbl  = NULL;
static lv_obj_t* grid_vol_lbl = NULL;
static lv_obj_t* grid_pad_prev_btn = NULL;
static lv_obj_t* grid_pad_next_btn = NULL;
static lv_obj_t* grid_pad_lbl = NULL;
static lv_obj_t* grid_inst_prev_btn = NULL;
static lv_obj_t* grid_inst_next_btn = NULL;
static lv_obj_t* grid_inst_lbl = NULL;
static lv_obj_t* grid_inst_edit_btn = NULL;
static lv_obj_t* grid_xtra_btns[4] = {};
static lv_obj_t* grid_xtra_lbls[4] = {};
static lv_obj_t* grid_xtra_change_btns[4] = {};
static lv_obj_t* grid_xtra_delete_btns[4] = {};
static lv_obj_t* grid_xtra_meta_lbls[4] = {};
static lv_obj_t* grid_xtra_slot_lbls[4] = {};
static int s_xtra_pending_slot = -1;

// --- XTRA sampler editor (waveform + trim + fades) -------------------------
static int        s_xtra_edit_slot       = 0;    // slot currently in the editor
static int        s_xtra_loaded_slot      = -1;   // slot whose sample is in sample_edit
static bool       s_xtra_applied[4]       = {false, false, false, false}; // baked+uploaded with current edits
static lv_obj_t*  s_xtra_wave_panel       = NULL;
static lv_obj_t*  s_xtra_wave_line        = NULL;
static lv_obj_t*  s_xtra_trim_a_line      = NULL; // trim-start marker
static lv_obj_t*  s_xtra_trim_b_line      = NULL; // trim-end marker
static lv_obj_t*  s_xtra_playhead_line    = NULL; // moving playback cursor over the waveform
static unsigned long s_xtra_play_start_ms = 0;    // 0 = not playing
static unsigned long s_xtra_play_dur_ms   = 0;    // trimmed length in ms
static float      s_xtra_play_a           = 0.f;  // playhead sweep start (0..1 of waveform)
static float      s_xtra_play_b           = 1.f;  // playhead sweep end
static bool       s_xtra_loop_on          = false;// re-trigger the sample at the end of each pass
static lv_obj_t*  s_xtra_loop_btn         = NULL;
static lv_obj_t*  s_xtra_info_lbl         = NULL; // filename / sr / dur
static lv_obj_t*  s_xtra_trim_lbl         = NULL; // "TRIM 12%..88%"
static lv_obj_t*  s_xtra_fade_lbl         = NULL; // "FADE IN 20ms  OUT 50ms"
static constexpr int XTRA_WAVE_COLS = 120;
static lv_point_precise_t s_xtra_wave_pts[XTRA_WAVE_COLS * 2 + 2];
static float      s_xtra_env[XTRA_WAVE_COLS];
static lv_obj_t* s_pad_inst_modal = NULL;
static lv_obj_t* s_pad_inst_modal_pad_lbl = NULL;
static lv_obj_t* s_pad_inst_modal_inst_lbl = NULL;
static lv_obj_t* s_pad_inst_modal_pad_btns[16] = {};
static lv_obj_t* s_pad_inst_modal_inst_btns[9] = {};
static lv_obj_t* s_pad_inst_modal_kit_btns[3][5] = {};   // [engine 0=808/1=909/2=505][preset 0..4]
static lv_obj_t* s_pad_inst_modal_kit_lbl_eng[3] = {};   // labels "808"/"909"/"505"

static const char* PAD_INST_NAMES[9] = {
    "Sampler", "808", "909", "505", "303", "WT", "FM2", "SH101", "GuitarT"
};
static const char* PAD_INST_SHORT[9] = {
    "SMP", "808", "909", "505", "303", "WT", "FM2", "SH1", "GTR"
};
static uint8_t s_pad_inst_sel[16] = {0};
// Selección pendiente del modal PAD SOUND — no se aplica al master hasta
// pulsar PREVIEW (suena) o ASIGNAR (suena y cierra).
static uint8_t s_pad_inst_pending[16] = {0};
// Kit (preset 0..4) por pad. Solo aplica cuando el instrumento del pad es un
// drum engine (808/909/505). El "pending" es lo que el usuario marca en el
// modal antes de PREVIEW/ASIGNAR; el "assigned" es lo confirmado por pad.
static uint8_t s_pad_kit_pending[16] = {0};
static uint8_t s_pad_kit_assigned[16] = {0};
// Último kit enviado a la Daisy por engine drum (0=808, 1=909, 2=505) para
// deduplicar envíos: solo se manda CMD_SYNTH_PRESET si difiere.
static int8_t s_engine_kit_last_applied[3] = {-1,-1,-1};
static volatile uint8_t s_pad_inst_focus_pad = 0;
static unsigned long s_pad_inst_local_ms[16] = {};
static const unsigned long PAD_INST_OWNERSHIP_MS = 1800;

// Number of synth engines exposed to the master (0..7: 808,909,505,303,WT,
// SH101,FM2,GUITAR). Used for all-notes-off sweeps so they cover every engine.
static constexpr int SYNTH_ENGINE_COUNT = 8;

// Deferred master→UI track-engine sync. ui_pad_sound_sync_track_engines() runs
// on Core 1 (UDP) and must NOT touch LVGL objects directly. It now only stages
// the engine array here; the LVGL task applies it under the lock in
// ui_update_current_screen(). Avoids a cross-core data race on the object tree.
static int8_t s_pending_track_engines[16] = {};
static std::atomic<bool> s_pending_track_engines_valid{false};

// Deferred "leaving LIVE" all-notes-off. Set from the touch task (Core 0) and
// drained on Core 1 in ui_process_pad_queue() so UDP is only ever sent from the
// main loop, never from the 200 Hz touch task.
static std::atomic<bool> s_pending_live_allnotesoff{false};

static void pad_inst_modal_refresh(void);
static bool pad_inst_unload_daisy_sample(uint8_t pad);
static int pp_engine_idx_from_code(uint8_t engine);
static uint8_t xtra_slot_engine_code(int slot);

static constexpr int XTRA_PARAM_MAX = 21;

struct XtraPadSlot {
    bool used;
    uint8_t pad;
    char name[24];
    uint8_t synth_engine_idx;
    uint8_t preset_idx;
    bool synth_mode;
    // Sampler editor (v: XTRA sampler-only). src_path is the WAV/MP3 on SD that
    // the slot was loaded from; trim_*/fade_* are baked into the uploaded WAV.
    char     src_path[128];
    float    trim_start;     // 0..1
    float    trim_end;       // 0..1
    uint16_t fade_in_ms;
    uint16_t fade_out_ms;
};

static XtraPadSlot s_xtra_slots[4] = {};
static const char* XTRA_PADS_STATE_FILE = "/xtra_pads.txt";
static const char* XTRA_PADS_PARAMS_FILE = "/xtra_params.txt";
static bool s_sd_for_xtra = false;
static uint8_t xtra_backing_pad_for_slot(int slot);
static bool s_xtra_touch_active[4] = {};
static bool s_xtra_hold_latched[4] = {};
static int s_xtra_last_note[4] = {-1, -1, -1, -1};
static lv_coord_t s_xtra_last_lx[4] = {};
static lv_coord_t s_xtra_last_ly[4] = {};
static uint32_t s_xtra_touch_start_ms[4] = {};
static uint32_t s_xtra_sampler_next_ms[4] = {};
static uint32_t s_xtra_xy_last_send_ms[4] = {};
static float s_xtra_param_values[4][XTRA_PARAM_MAX] = {};
static bool s_xtra_param_valid[4] = {};

static const uint8_t XTRA_SYNTH_ENGINE_CODES[8] = {0, 1, 2, 3, 4, 5, 6, 7};
static const char* XTRA_SYNTH_ENGINE_NAMES[8] = {"808", "909", "505", "303", "WT", "SH101", "FM2", "GUITAR"};
static const char* XTRA_PRESET_LABELS[3] = {"A", "B", "C"};
static const uint8_t XTRA_DRUM_INSTRUMENTS[3][3][4] = {
    { {0, 1, 2, 5}, {0, 3, 6, 7}, {0, 4, 8, 9} },
    { {0, 1, 2, 5}, {0, 3, 4, 6}, {1, 4, 7, 8} },
    { {0, 1, 2, 3}, {1, 3, 4, 6}, {0, 2, 5, 7} }
};
static const uint8_t XTRA_MELODIC_BASE_NOTES[3][4] = {
    {48, 52, 55, 60},
    {36, 43, 48, 55},
    {60, 64, 67, 72}
};

static inline lv_color_t xtra_slot_color(int slot) {
    return lv_color_hex(theme_presets[currentTheme].track_colors[slot & 0x0F]);
}

static void xtra_apply_visual_state(int slot, bool active, lv_coord_t lx, lv_coord_t ly) {
    if (slot < 0 || slot >= 4 || !grid_xtra_btns[slot]) return;
    lv_obj_t* obj = grid_xtra_btns[slot];
    lv_color_t accent = xtra_slot_color(slot);
    if (!active) {
        lv_obj_set_style_shadow_width(obj, 0, 0);
        lv_obj_set_style_shadow_opa(obj, LV_OPA_0, 0);
        lv_obj_set_style_outline_width(obj, 0, 0);
        lv_obj_set_style_outline_opa(obj, LV_OPA_0, 0);
        lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(obj, 2, 0);
        lv_obj_set_style_border_color(obj, theme_accent2(), 0);
        if (grid_xtra_meta_lbls[slot]) lv_obj_set_style_text_color(grid_xtra_meta_lbls[slot], theme_text(), 0);
        if (grid_xtra_slot_lbls[slot]) lv_obj_set_style_text_color(grid_xtra_slot_lbls[slot], theme_text_dim(), 0);
        return;
    }

    lv_coord_t w = lv_obj_get_width(obj);
    lv_coord_t h = lv_obj_get_height(obj);
    lv_coord_t dx = abs((int)lx - (int)(w / 2));
    lv_coord_t dy = abs((int)ly - (int)(h / 2));
    uint16_t travel = (uint16_t)constrain(dx + dy, 0, 240);
    lv_opa_t fill_opa = (lv_opa_t)(190 + travel / 4);
    if (fill_opa > LV_OPA_COVER) fill_opa = LV_OPA_COVER;
    lv_coord_t outline_w = (lv_coord_t)(2 + travel / 30);
    lv_coord_t shadow_w = (lv_coord_t)(18 + travel / 8);
    lv_opa_t glow_opa = (lv_opa_t)(160 + travel / 3);
    if (glow_opa > LV_OPA_COVER) glow_opa = LV_OPA_COVER;

    lv_obj_set_style_bg_color(obj, accent, 0);
    lv_obj_set_style_bg_grad_color(obj, RED808_SURFACE, 0);
    lv_obj_set_style_bg_opa(obj, fill_opa, 0);
    lv_obj_set_style_border_width(obj, 3, 0);
    lv_obj_set_style_border_color(obj, accent, 0);
    lv_obj_set_style_border_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_outline_width(obj, outline_w, 0);
    lv_obj_set_style_outline_pad(obj, 1, 0);
    lv_obj_set_style_outline_color(obj, accent, 0);
    lv_obj_set_style_outline_opa(obj, glow_opa, 0);
    lv_obj_set_style_shadow_width(obj, shadow_w, 0);
    lv_obj_set_style_shadow_color(obj, accent, 0);
    lv_obj_set_style_shadow_opa(obj, glow_opa, 0);
    if (grid_xtra_meta_lbls[slot]) lv_obj_set_style_text_color(grid_xtra_meta_lbls[slot], lv_color_white(), 0);
    if (grid_xtra_slot_lbls[slot]) lv_obj_set_style_text_color(grid_xtra_slot_lbls[slot], lv_color_white(), 0);
}

static void xtra_save_param_state(void);

static int xtra_slot_pp_engine_idx(int slot) {
    if (slot < 0 || slot >= 4 || !s_xtra_slots[slot].synth_mode) return -1;
    return pp_engine_idx_from_code(xtra_slot_engine_code(slot));
}

static void xtra_reset_slot_params(int slot) {
    if (slot < 0 || slot >= 4) return;
    memset(s_xtra_param_values[slot], 0, sizeof(s_xtra_param_values[slot]));
    s_xtra_param_valid[slot] = false;
    int eng_idx = xtra_slot_pp_engine_idx(slot);
    if (eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) return;
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    for (uint8_t i = 0; i < eng->param_count && i < XTRA_PARAM_MAX; i++) {
        s_xtra_param_values[slot][i] = eng->params[i].vdef;
    }
    int preset_idx = constrain((int)s_xtra_slots[slot].preset_idx, 0, (int)eng->preset_count - 1);
    if (preset_idx >= 0 && preset_idx < eng->preset_count) {
        const SynthPreset* pr = &eng->presets[preset_idx];
        for (uint8_t pv = 0; pv < pr->count; pv++) {
            for (uint8_t i = 0; i < eng->param_count && i < XTRA_PARAM_MAX; i++) {
                if (eng->params[i].param_id == pr->values[pv].param_id) {
                    s_xtra_param_values[slot][i] = pr->values[pv].value;
                    break;
                }
            }
        }
    }
    s_xtra_param_valid[slot] = true;
}

static void xtra_capture_editor_state(int slot);
static void xtra_load_editor_state(int slot);

static void xtra_send_slot_param_snapshot(int slot) {
    if (slot < 0 || slot >= 4 || !s_xtra_slots[slot].synth_mode || !ui_use_udp_transport()) return;
    int eng_idx = xtra_slot_pp_engine_idx(slot);
    if (eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) return;
    if (!s_xtra_param_valid[slot]) xtra_reset_slot_params(slot);
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    // Map the A/B/C slot index into the engine's real preset range — sending an
    // index the engine doesn't have leaves it with no preset loaded (= silent).
    uint8_t preset = (eng->preset_count > 0)
                     ? (uint8_t)(s_xtra_slots[slot].preset_idx % eng->preset_count)
                     : 0;
    udp_send_synth_preset(eng->engine, preset);
    for (uint8_t i = 0; i < eng->param_count && i < XTRA_PARAM_MAX; i++) {
        udp_send_synth_param(eng->engine, 0, eng->params[i].param_id, s_xtra_param_values[slot][i]);
    }
}

static void xtra_load_param_state(void) {
    for (int i = 0; i < 4; i++) xtra_reset_slot_params(i);
    File f = SPIFFS.open(XTRA_PADS_PARAMS_FILE, FILE_READ);
    if (!f) return;
    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;
        char buf[512];
        line.toCharArray(buf, sizeof(buf));
        char* ctx = nullptr;
        char* tok = strtok_r(buf, ",", &ctx);
        if (!tok) continue;
        int slot = atoi(tok);
        tok = strtok_r(nullptr, ",", &ctx);
        if (!tok) continue;
        int engine = atoi(tok);
        tok = strtok_r(nullptr, ",", &ctx);
        if (!tok) continue;
        int count = atoi(tok);
        if (slot < 0 || slot >= 4) continue;
        if (engine != (int)xtra_slot_engine_code(slot)) continue;
        int eng_idx = xtra_slot_pp_engine_idx(slot);
        if (eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) continue;
        const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
        int limit = constrain(count, 0, (int)eng->param_count);
        for (int i = 0; i < limit && i < XTRA_PARAM_MAX; i++) {
            tok = strtok_r(nullptr, ",", &ctx);
            if (!tok) break;
            s_xtra_param_values[slot][i] = (float)atof(tok);
            s_xtra_param_valid[slot] = true;
        }
    }
    f.close();
}

static void xtra_save_param_state(void) {
    File f = SPIFFS.open(XTRA_PADS_PARAMS_FILE, FILE_WRITE);
    if (!f) return;
    for (int slot = 0; slot < 4; slot++) {
        int eng_idx = xtra_slot_pp_engine_idx(slot);
        if (!s_xtra_param_valid[slot] || eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) continue;
        const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
        f.printf("%d,%u,%u", slot, (unsigned)eng->engine, (unsigned)eng->param_count);
        for (uint8_t i = 0; i < eng->param_count && i < XTRA_PARAM_MAX; i++) {
            f.printf(",%.5f", s_xtra_param_values[slot][i]);
        }
        f.print('\n');
    }
    f.close();
}

static const char* xtra_slot_mode_label(int slot) {
    if (slot < 0 || slot >= 4) return "SMP";
    if (!s_xtra_slots[slot].synth_mode) return "SMP";
    return XTRA_SYNTH_ENGINE_NAMES[s_xtra_slots[slot].synth_engine_idx & 0x07];
}

static void xtra_apply_default_slots(void) {
    // XTRA is now sampler-only: start with empty pads ready to load a WAV/MP3.
    for (int i = 0; i < 4; i++) {
        s_xtra_slots[i] = XtraPadSlot{};
        s_xtra_slots[i].used = false;
        s_xtra_slots[i].pad = xtra_backing_pad_for_slot(i);
        s_xtra_slots[i].synth_mode = false;
        s_xtra_slots[i].synth_engine_idx = 0;
        s_xtra_slots[i].preset_idx = 0;
        s_xtra_slots[i].src_path[0] = '\0';
        s_xtra_slots[i].trim_start = 0.0f;
        s_xtra_slots[i].trim_end = 1.0f;
        s_xtra_slots[i].fade_in_ms = 0;
        s_xtra_slots[i].fade_out_ms = 0;
        s_xtra_slots[i].name[0] = '\0';
    }
}

static uint8_t xtra_slot_engine_code(int slot) {
    return XTRA_SYNTH_ENGINE_CODES[s_xtra_slots[slot].synth_engine_idx & 0x07];
}

static bool xtra_slot_is_drum(int slot) {
    return s_xtra_slots[slot].synth_mode && s_xtra_slots[slot].synth_engine_idx < 3;
}

static void xtra_slot_refresh_name(int slot) {
    if (slot < 0 || slot >= 4) return;
    if (!s_xtra_slots[slot].synth_mode) {
        if (s_xtra_slots[slot].name[0] == '\0') {
            strncpy(s_xtra_slots[slot].name, "SAMPLER", sizeof(s_xtra_slots[slot].name) - 1);
            s_xtra_slots[slot].name[sizeof(s_xtra_slots[slot].name) - 1] = '\0';
        }
        return;
    }
    snprintf(s_xtra_slots[slot].name, sizeof(s_xtra_slots[slot].name), "%s %s",
             XTRA_SYNTH_ENGINE_NAMES[s_xtra_slots[slot].synth_engine_idx & 0x07],
             XTRA_PRESET_LABELS[s_xtra_slots[slot].preset_idx % 3]);
}

static void xtra_apply_preset(int slot) {
    if (slot < 0 || slot >= 4 || !ui_use_udp_transport() || !s_xtra_slots[slot].synth_mode) return;
    uint8_t engine = xtra_slot_engine_code(slot);
    uint8_t preset = s_xtra_slots[slot].preset_idx % 3;
    // Drum engines (808/909/505) always expose 3 kits; melodic engines may have
    // fewer presets — clamp into the real range so we never send a phantom
    // index that the synth ignores (a common "preset doesn't sound" cause).
    if (!xtra_slot_is_drum(slot)) {
        int eng_idx = xtra_slot_pp_engine_idx(slot);
        if (eng_idx >= 0 && eng_idx < SP_ENGINE_COUNT) {
            int pc = (int)SP_ENGINES[eng_idx].preset_count;
            if (pc > 0) preset = (uint8_t)(preset % pc);
        }
    }
    udp_send_synth_preset(engine, preset);
}

static void xtra_send_note_on(int slot, int note, uint8_t velocity) {
    uint8_t engine = xtra_slot_engine_code(slot);
    udp_send_synth_note_on_ex(engine, (uint8_t)constrain(note, 24, 96), velocity, false, false);
    s_xtra_last_note[slot] = constrain(note, 24, 96);
}

static void xtra_send_note_off(int slot) {
    if (slot < 0 || slot >= 4 || s_xtra_last_note[slot] < 0) return;
    uint8_t engine = xtra_slot_engine_code(slot);
    udp_send_synth_note_off_ex(engine, 0, (uint8_t)s_xtra_last_note[slot]);
    s_xtra_last_note[slot] = -1;
}

static void xtra_apply_xy_modulation(int slot, uint8_t engine, float xNorm, float yNorm) {
    uint32_t now = millis();
    if ((now - s_xtra_xy_last_send_ms[slot]) < 14) return;
    s_xtra_xy_last_send_ms[slot] = now;

    switch (engine) {
        case 3: {
            float bend = (xNorm - 0.5f) * 12.0f;
            float cutoff = 180.0f + yNorm * 4200.0f;
            float envMod = 0.18f + yNorm * 0.82f;
            udp_send_synth_param(engine, 0, 14, bend);
            udp_send_synth_param(engine, 0, 0, cutoff);
            udp_send_synth_param(engine, 0, 2, envMod);
            break;
        }
        case 4: {
            float wavePos = xNorm * 7.0f;
            float cutoff = 1500.0f + yNorm * 12000.0f;
            float volume = 0.40f + yNorm * 0.45f;
            udp_send_synth_param(engine, 0, 0, wavePos);
            udp_send_synth_param(engine, 0, 4, cutoff);
            udp_send_synth_param(engine, 0, 3, volume);
            break;
        }
        case 5: {
            float pwm = 0.1f + xNorm * 0.8f;
            float cutoff = 120.0f + yNorm * 12000.0f;
            float res = 0.12f + yNorm * 0.70f;
            udp_send_synth_param(engine, 0, 1, pwm);
            udp_send_synth_param(engine, 0, 4, cutoff);
            udp_send_synth_param(engine, 0, 5, res);
            break;
        }
        case 6: {
            float ratio = 0.5f + xNorm * 7.5f;
            float detune = (xNorm - 0.5f) * 50.0f;
            float fmIndex = yNorm * 12.0f;
            float feedback = yNorm;
            udp_send_synth_param(engine, 0, 8, ratio);
            udp_send_synth_param(engine, 0, 12, detune);
            udp_send_synth_param(engine, 0, 9, fmIndex);
            udp_send_synth_param(engine, 0, 10, feedback);
            break;
        }
        case 7: {
            float mStruct = xNorm;
            float sStruct = 1.0f - xNorm * 0.85f;
            float mDamp = 0.08f + yNorm * 0.86f;
            float sBright = 0.10f + yNorm * 0.90f;
            udp_send_synth_param(engine, 0, 1, mStruct);
            udp_send_synth_param(engine, 0, 6, sStruct);
            udp_send_synth_param(engine, 0, 3, mDamp);
            udp_send_synth_param(engine, 0, 7, sBright);
            break;
        }
        default:
            break;
    }
}

static void xtra_trigger_slot(int slot, int lx, int ly, bool initialPress) {
    if (slot < 0 || slot >= 4 || !udp_wifi_connected()) return;
    int w = grid_xtra_btns[slot] ? lv_obj_get_width(grid_xtra_btns[slot]) : 1;
    int h = grid_xtra_btns[slot] ? lv_obj_get_height(grid_xtra_btns[slot]) : 1;
    float xNorm = (float)constrain(lx, 0, w) / (float)(w > 0 ? w : 1);
    float yNorm = 1.0f - (float)constrain(ly, 0, h) / (float)(h > 0 ? h : 1);
    uint8_t velocity = (uint8_t)constrain((int)(40.0f + yNorm * 87.0f + 0.5f), 20, 127);
    if (!s_xtra_slots[slot].synth_mode) {
        if (initialPress) {
            udp_send_trigger(s_xtra_slots[slot].pad, velocity);
            s_xtra_sampler_next_ms[slot] = millis() + (uint32_t)(70.0f + (1.0f - xNorm) * 190.0f);
        } else {
            uint32_t now = millis();
            if (now >= s_xtra_sampler_next_ms[slot]) {
                udp_send_trigger(s_xtra_slots[slot].pad, velocity);
                s_xtra_sampler_next_ms[slot] = now + (uint32_t)(70.0f + (1.0f - xNorm) * 190.0f);
            }
        }
        return;
    }
    if (initialPress && !xtra_slot_is_drum(slot)) xtra_send_slot_param_snapshot(slot);
    else xtra_apply_preset(slot);
    if (xtra_slot_is_drum(slot)) {
        if (!initialPress) return;
        uint8_t engineIdx = s_xtra_slots[slot].synth_engine_idx;
        uint8_t instrument = XTRA_DRUM_INSTRUMENTS[engineIdx][s_xtra_slots[slot].preset_idx % 3][slot];
        udp_send_synth_trigger(xtra_slot_engine_code(slot), instrument, velocity);
        return;
    }

    int note = XTRA_MELODIC_BASE_NOTES[s_xtra_slots[slot].preset_idx % 3][slot] + (int)((xNorm - 0.5f) * 12.0f + (xNorm >= 0.5f ? 0.5f : -0.5f));
    if (initialPress) {
        xtra_send_note_on(slot, note, velocity);
    } else if (note != s_xtra_last_note[slot]) {
        xtra_send_note_off(slot);
        xtra_send_note_on(slot, note, velocity);
    }

    uint8_t engine = xtra_slot_engine_code(slot);
    xtra_apply_xy_modulation(slot, engine, xNorm, yNorm);
}

static bool xtra_local_touch(lv_event_t* e, lv_coord_t* lx, lv_coord_t* ly) {
    if (!e || !lx || !ly) return false;
    lv_obj_t* obj = (lv_obj_t*)lv_event_get_target(e);
    lv_indev_t* indev = lv_indev_get_act();
    if (!obj || !indev) return false;
    lv_point_t p;
    lv_indev_get_point(indev, &p);
    lv_area_t a;
    lv_obj_get_coords(obj, &a);
    *lx = (lv_coord_t)(p.x - a.x1);
    *ly = (lv_coord_t)(p.y - a.y1);
    return true;
}

static void xtra_pad_touch_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot >= 4 || !grid_xtra_btns[slot]) return;
    lv_event_code_t code = lv_event_get_code(e);
    lv_coord_t lx = 0, ly = 0;
    xtra_local_touch(e, &lx, &ly);
    if (code == LV_EVENT_PRESSED) {
        s_xtra_touch_active[slot] = true;
        s_xtra_hold_latched[slot] = false;
        s_xtra_touch_start_ms[slot] = millis();
        s_xtra_last_lx[slot] = lx;
        s_xtra_last_ly[slot] = ly;
        xtra_trigger_slot(slot, lx, ly, true);
        xtra_apply_visual_state(slot, true, lx, ly);
    } else if (code == LV_EVENT_PRESSING) {
        if (!s_xtra_touch_active[slot]) return;
        uint32_t now = millis();
        if (!s_xtra_hold_latched[slot] && (now - s_xtra_touch_start_ms[slot]) >= 28) {
            s_xtra_hold_latched[slot] = true;
        }
        lv_coord_t dx = abs((int)lx - (int)s_xtra_last_lx[slot]);
        lv_coord_t dy = abs((int)ly - (int)s_xtra_last_ly[slot]);
        if (dx >= 3 || dy >= 3 || s_xtra_hold_latched[slot]) {
            s_xtra_last_lx[slot] = lx;
            s_xtra_last_ly[slot] = ly;
            xtra_trigger_slot(slot, lx, ly, false);
        }
        xtra_apply_visual_state(slot, true, lx, ly);
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        s_xtra_touch_active[slot] = false;
        s_xtra_hold_latched[slot] = false;
        s_xtra_touch_start_ms[slot] = 0;
        s_xtra_sampler_next_ms[slot] = 0;
        if (!xtra_slot_is_drum(slot)) xtra_send_note_off(slot);
        xtra_apply_visual_state(slot, false, 0, 0);
    }
}

static void xtra_refresh_panel(void);
static void xtra_edit_cb(lv_event_t* e);
static void xtra_editor_select(int slot);   // sampler editor: load+show a slot

static uint8_t xtra_backing_pad_for_slot(int slot) {
    if (slot < 0) slot = 0;
    if (slot > 3) slot = 3;
    return (uint8_t)(16 + slot);
}

static void xtra_begin_load_for_slot(int slot) {
    if (slot < 0 || slot >= 4) return;
    s_xtra_pending_slot = slot;
    s_sd_for_xtra = true;
    p4sd.selected_is_midi = false;
    p4sd.selected_pad = xtra_backing_pad_for_slot(slot);
    ui_show_toast("XTRA: elige WAV y LOAD", RED808_CYAN);
    ui_navigate_to(9);
}

static void trim_wav_extension(char* name) {
    if (!name) return;
    size_t n = strlen(name);
    if (n > 4) {
        const char* ext = name + n - 4;
        if (strcasecmp(ext, ".wav") == 0) {
            name[n - 4] = '\0';
        }
    }
}

static void xtra_save_state(void) {
    File f = SPIFFS.open(XTRA_PADS_STATE_FILE, FILE_WRITE);
    if (!f) return;
    for (int i = 0; i < 4; i++) {
        const XtraPadSlot& s = s_xtra_slots[i];
        // Legacy 6 fields + sampler editor fields (src_path,trim,fade). The name
        // and path never contain commas (filenames), so CSV stays unambiguous.
        f.printf("%d,%u,%s,%u,%u,%u,%s,%.4f,%.4f,%u,%u\n",
                 s.used ? 1 : 0, (unsigned)s.pad, s.name[0] ? s.name : "-",
                 s.synth_mode ? 1U : 0U, (unsigned)s.synth_engine_idx, (unsigned)s.preset_idx,
                 s.src_path[0] ? s.src_path : "-",
                 s.trim_start, s.trim_end, (unsigned)s.fade_in_ms, (unsigned)s.fade_out_ms);
    }
    f.close();
    xtra_save_param_state();
}

static void xtra_load_state(void) {
    memset(s_xtra_slots, 0, sizeof(s_xtra_slots));
    xtra_apply_default_slots();
    File f = SPIFFS.open(XTRA_PADS_STATE_FILE, FILE_READ);
    if (!f) return;
    int idx = 0;
    while (f.available() && idx < 4) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) {
            idx++;
            continue;
        }
        int used = 0;
        unsigned pad = 0;
        char name[24] = {0};
        char src[128] = {0};
        unsigned synth_mode = 0, synth_engine_idx = 0, preset_idx = 0;
        float trim_a = 0.0f, trim_b = 1.0f;
        unsigned fin = 0, fout = 0;
        int parsed = sscanf(line.c_str(),
                            "%d,%u,%23[^,],%u,%u,%u,%127[^,],%f,%f,%u,%u",
                            &used, &pad, name, &synth_mode, &synth_engine_idx, &preset_idx,
                            src, &trim_a, &trim_b, &fin, &fout);
        if (parsed >= 2) {
            s_xtra_slots[idx].used = (used != 0);
            // Enforce fixed XTRA backing slots (16..19) regardless of legacy file values.
            s_xtra_slots[idx].pad = xtra_backing_pad_for_slot(idx);
            // Sampler-only now: ignore any legacy synth flags from old files.
            s_xtra_slots[idx].synth_mode = false;
            s_xtra_slots[idx].synth_engine_idx = 0;
            s_xtra_slots[idx].preset_idx = 0;
            if (parsed >= 3 && strcmp(name, "-") != 0) {
                strncpy(s_xtra_slots[idx].name, name, sizeof(s_xtra_slots[idx].name) - 1);
                s_xtra_slots[idx].name[sizeof(s_xtra_slots[idx].name) - 1] = '\0';
                trim_wav_extension(s_xtra_slots[idx].name);
            }
            if (parsed >= 7 && strcmp(src, "-") != 0) {
                strncpy(s_xtra_slots[idx].src_path, src, sizeof(s_xtra_slots[idx].src_path) - 1);
                s_xtra_slots[idx].src_path[sizeof(s_xtra_slots[idx].src_path) - 1] = '\0';
            }
            s_xtra_slots[idx].trim_start  = (parsed >= 8)  ? constrain(trim_a, 0.0f, 1.0f) : 0.0f;
            s_xtra_slots[idx].trim_end    = (parsed >= 9)  ? constrain(trim_b, 0.0f, 1.0f) : 1.0f;
            if (s_xtra_slots[idx].trim_end <= s_xtra_slots[idx].trim_start) s_xtra_slots[idx].trim_end = 1.0f;
            s_xtra_slots[idx].fade_in_ms  = (parsed >= 10) ? (uint16_t)fin  : 0;
            s_xtra_slots[idx].fade_out_ms = (parsed >= 11) ? (uint16_t)fout : 0;
        }
        idx++;
    }
    f.close();
    xtra_load_param_state();
}

static void xtra_refresh_panel(void) {
    for (int i = 0; i < 4; i++) {
        if (!grid_xtra_btns[i] || !grid_xtra_lbls[i]) continue;
        lv_color_t accent = xtra_slot_color(i);
        bool selected = (i == s_xtra_edit_slot);
        if (s_xtra_slots[i].used && s_xtra_slots[i].src_path[0]) {
            lv_obj_set_style_bg_color(grid_xtra_btns[i], accent, 0);
            lv_obj_set_style_bg_opa(grid_xtra_btns[i], selected ? LV_OPA_COVER : LV_OPA_80, 0);
            lv_label_set_text(grid_xtra_lbls[i],
                              s_xtra_slots[i].name[0] ? s_xtra_slots[i].name : "XTRA");
            if (grid_xtra_meta_lbls[i]) lv_label_set_text(grid_xtra_meta_lbls[i], "SAMPLER  ·  TAP = TRIG");
        } else {
            lv_obj_set_style_bg_color(grid_xtra_btns[i], theme_surface(), 0);
            lv_obj_set_style_bg_opa(grid_xtra_btns[i], LV_OPA_80, 0);
            lv_label_set_text(grid_xtra_lbls[i], "+ LOAD");
            if (grid_xtra_meta_lbls[i]) lv_label_set_text(grid_xtra_meta_lbls[i], "WAV / MP3");
        }
        // The selected slot gets a bright border so the editor target is clear.
        lv_obj_set_style_border_color(grid_xtra_btns[i], selected ? theme_accent2() : theme_border(), 0);
        lv_obj_set_style_border_width(grid_xtra_btns[i], selected ? 3 : 2, 0);
        if (grid_xtra_slot_lbls[i]) lv_label_set_text_fmt(grid_xtra_slot_lbls[i], "S%02d", i + 1);
        if (grid_xtra_delete_btns[i]) {
            lv_obj_set_style_border_color(grid_xtra_delete_btns[i], accent, 0);
            lv_obj_t* lbl = lv_obj_get_child(grid_xtra_delete_btns[i], 0);
            if (lbl) lv_label_set_text(lbl, "LOAD");
        }
    }
}

static void xtra_change_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot >= 4) return;
    s_xtra_slots[slot].used = true;
    if (!s_xtra_slots[slot].synth_mode) {
        s_xtra_slots[slot].synth_mode = true;
        s_xtra_slots[slot].synth_engine_idx = 0;
        s_xtra_slots[slot].preset_idx = 0;
    } else if (s_xtra_slots[slot].synth_engine_idx >= 7) {
        s_xtra_slots[slot].synth_mode = false;
    } else {
        s_xtra_slots[slot].synth_engine_idx = (uint8_t)(s_xtra_slots[slot].synth_engine_idx + 1);
        s_xtra_slots[slot].preset_idx = 0;
    }
    xtra_reset_slot_params(slot);
    xtra_slot_refresh_name(slot);
    xtra_save_state();
    xtra_refresh_panel();
}

static void xtra_delete_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot >= 4) return;
    s_xtra_slots[slot].used = true;
    if (s_xtra_slots[slot].synth_mode) {
        s_xtra_slots[slot].preset_idx = (uint8_t)((s_xtra_slots[slot].preset_idx + 1) % 3);
        xtra_reset_slot_params(slot);
        xtra_slot_refresh_name(slot);
        ui_show_toast("Preset XTRA", theme_warning());
    } else {
        xtra_begin_load_for_slot(slot);
        return;
    }
    xtra_slot_refresh_name(slot);
    xtra_save_state();
    xtra_refresh_panel();
}

static void xtra_slot_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot >= 4) return;
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (s_xtra_slots[slot].synth_mode) ui_show_toast("Hold para XY", theme_success());
    else ui_show_toast("Sampler XTRA listo", theme_success());
}

// Devuelve idx 0..2 si el instrumento es engine drum (808/909/505), -1 si no.
static int8_t pad_inst_drum_engine_idx(uint8_t inst_idx) {
    if (inst_idx == 1) return 0; // 808
    if (inst_idx == 2) return 1; // 909
    if (inst_idx == 3) return 2; // 505
    return -1;
}

static int8_t pad_inst_engine_code(uint8_t inst_idx) {
    switch (inst_idx) {
        case 0: return -1; // Sampler
        case 1: return 0;  // 808
        case 2: return 1;  // 909
        case 3: return 2;  // 505
        case 4: return 3;  // 303
        case 5: return 4; // WT
        case 6: return 6; // FM2
        case 7: return 5; // SH101
        case 8: return 7; // GuitarT
        default: return -1;
    }
}

static uint8_t pad_inst_idx_from_engine_code(int8_t engine) {
    switch (engine) {
        case -1: return 0; // Sampler
        case 0: return 1;  // 808
        case 1: return 2;  // 909
        case 2: return 3;  // 505
        case 3: return 4;  // 303
        case 4: return 5;  // WT
        case 6: return 6;  // FM2
        case 5: return 7;  // SH101
        case 7: return 8;  // GuitarT
        default: return 0;
    }
}

static void pad_inst_refresh_pad_badge(uint8_t pad) {
    if (pad > 15 || !live_pad_inst_labels[pad]) return;
    uint8_t inst = s_pad_inst_sel[pad];
    if (inst > 7) inst = 0;
    lv_label_set_text(live_pad_inst_labels[pad], PAD_INST_SHORT[inst]);
}

static void pad_inst_refresh_controls(void) {
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    uint8_t inst = s_pad_inst_sel[pad];
    if (inst > 7) inst = 0;
    if (grid_pad_lbl) lv_label_set_text_fmt(grid_pad_lbl, "PAD %02d", (int)pad + 1);
    if (grid_inst_lbl) lv_label_set_text(grid_inst_lbl, PAD_INST_NAMES[inst]);
}

static void pad_inst_apply_to_master(uint8_t pad) {
    if (pad > 15) return;
    uint8_t inst = s_pad_inst_sel[pad];
    if (inst > 7) inst = 0;
    int8_t engine = pad_inst_engine_code(inst);
    s_pad_inst_local_ms[pad] = millis();
    if (udp_wifi_connected() || udp_master_connected()) {
        udp_send_set_track_engine(pad, engine);
        // Audible feedback to confirm assignment. Hand the trigger over to the
        // pad queue so the melodic note-off scheduler also runs (avoids 303
        // hanging after the assignment-confirmation tap).
        enqueue_pad_event(pad, 110);
    }
}

// Core 1 (UDP) entry point: stage the engines and let the LVGL task apply them.
// MUST NOT touch LVGL objects here — that would race with the render task.
void ui_pad_sound_sync_track_engines(const int8_t engines[16]) {
    if (!engines) return;
    for (int i = 0; i < 16; i++) s_pending_track_engines[i] = engines[i];
    s_pending_track_engines_valid.store(true, std::memory_order_release);
}

// Applied on the LVGL task (under lvgl_port_lock) from ui_update_current_screen.
static void apply_pad_sound_track_engines(const int8_t engines[16]) {
    if (!engines) return;
    unsigned long nowMs = millis();
    bool anyChanged = false;
    for (int pad = 0; pad < 16; pad++) {
        if (nowMs - s_pad_inst_local_ms[pad] < PAD_INST_OWNERSHIP_MS) {
            continue;
        }
        uint8_t incomingInst = pad_inst_idx_from_engine_code(engines[pad]);
        uint8_t oldAssigned = s_pad_inst_sel[pad];
        if (oldAssigned == incomingInst) {
            continue;
        }
        s_pad_inst_sel[pad] = incomingInst;
        if (s_pad_inst_pending[pad] == oldAssigned) {
            // Keep pending in lockstep only when user isn't editing that pad.
            s_pad_inst_pending[pad] = incomingInst;
        }
        pad_inst_refresh_pad_badge((uint8_t)pad);
        anyChanged = true;
    }
    if (anyChanged) {
        pad_inst_refresh_controls();
        if (s_pad_inst_modal) {
            pad_inst_modal_refresh();
        }
    }
}

// Sync Pads LEDs — pads illuminate automatically with sequencer
static lv_obj_t* grid_sync_btn = NULL;
static bool sync_pads_active = false;  // OFF by default (synced with S3)

// Ripple effect — pool of expanding ring objects
static constexpr int RIPPLE_POOL = 4;
static constexpr int RIPPLE_FRAMES = 12;      // animation steps at ~60Hz = 200ms
static constexpr int RIPPLE_MAX_R = 80;        // max radius in pixels
struct RippleState {
    lv_obj_t* obj = nullptr;
    int frame = 0;         // 0 = inactive
    lv_color_t color;
    lv_coord_t cx, cy;     // center position (absolute on scr_live)
};
static RippleState ripples[RIPPLE_POOL];

static void ripple_spawn(int pad) {
    // DISABLED — ripple overlay forced LVGL to invalidate a large expanding
    // area every frame for 200ms per tap. On the live screen this stacks up
    // when tapping fast and drowns the render task. The pad border already
    // flashes on press, which is enough feedback.
    (void)pad;
    return;
    if (pad < 0 || pad >= 16 || !live_pad_btns[pad] || !scr_live) return;
    // Calculate pad center in screen coordinates
    lv_coord_t px = lv_obj_get_x(live_pad_btns[pad]);
    lv_coord_t py = lv_obj_get_y(live_pad_btns[pad]);
    lv_coord_t pw = lv_obj_get_width(live_pad_btns[pad]);
    lv_coord_t ph = lv_obj_get_height(live_pad_btns[pad]);
    lv_coord_t cx = px + pw / 2;
    lv_coord_t cy = py + ph / 2;
    lv_color_t tc = lv_color_hex(theme_presets[currentTheme].track_colors[pad]);

    // Find free or oldest ripple slot
    int slot = 0;
    for (int i = 0; i < RIPPLE_POOL; i++) {
        if (ripples[i].frame == 0) { slot = i; break; }
        if (ripples[i].frame > ripples[slot].frame) slot = i;
    }

    RippleState& r = ripples[slot];
    r.frame = 1;
    r.color = tc;
    r.cx = cx;
    r.cy = cy;

    if (!r.obj) {
        r.obj = lv_obj_create(scr_live);
        lv_obj_clear_flag(r.obj, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(r.obj, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_bg_opa(r.obj, LV_OPA_0, 0);
        lv_obj_set_style_shadow_width(r.obj, 0, 0);
    }
    // Reset visual
    lv_obj_set_size(r.obj, 10, 10);
    lv_obj_set_pos(r.obj, cx - 5, cy - 5);
    lv_obj_set_style_radius(r.obj, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(r.obj, 3, 0);
    lv_obj_set_style_border_color(r.obj, tc, 0);
    lv_obj_set_style_border_opa(r.obj, LV_OPA_COVER, 0);
    lv_obj_clear_flag(r.obj, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(r.obj);
}

// Called each frame from update_live_screen to animate active ripples
static void ripple_update(void) {
    for (int i = 0; i < RIPPLE_POOL; i++) {
        RippleState& r = ripples[i];
        if (r.frame == 0 || !r.obj) continue;

        r.frame++;
        if (r.frame > RIPPLE_FRAMES) {
            // Animation done — hide and recycle
            r.frame = 0;
            lv_obj_add_flag(r.obj, LV_OBJ_FLAG_HIDDEN);
            continue;
        }

        float t = (float)(r.frame - 1) / (float)RIPPLE_FRAMES;  // 0..1
        int sz = 10 + (int)(t * RIPPLE_MAX_R * 2);
        lv_obj_set_size(r.obj, sz, sz);
        lv_obj_set_pos(r.obj, r.cx - sz / 2, r.cy - sz / 2);
        lv_obj_set_style_border_opa(r.obj, (lv_opa_t)(255 * (1.0f - t)), 0);
        // Border thins as it expands
        lv_obj_set_style_border_width(r.obj, (lv_coord_t)(3 * (1.0f - t * 0.7f)), 0);
    }
}

static void pad_touch_cb(lv_event_t* e) {
    // Safety fallback for the LVGL button event. In practice the GT911 direct
    // path (ui_pad_frame_update) already serviced the press at 200Hz; this
    // only fires if LVGL somehow received a touch the cache did not classify
    // as a pad (shouldn't happen with matching geometry). No-op is safe — the
    // real velocity-aware tap handling lives in ui_pad_frame_update().
    LV_UNUSED(e);
}

static void grid_nav_cb(lv_event_t* e) {
    int screen_id = (int)(intptr_t)lv_event_get_user_data(e);
    ui_navigate_to(screen_id);
}

static void live_step_nav_cb(lv_event_t* e) {
    LV_UNUSED(e);
    ui_navigate_to(3);
}

static void grid_sync_cb(lv_event_t* e) {
    LV_UNUSED(e);
    sync_pads_active = !sync_pads_active;
    if (grid_sync_btn) {
        lv_obj_set_style_bg_color(grid_sync_btn,
            sync_pads_active ? RED808_SUCCESS : RED808_SURFACE, 0);
        lv_obj_set_style_border_color(grid_sync_btn,
            sync_pads_active ? RED808_CYAN : RED808_BORDER, 0);
        lv_obj_t* lbl = lv_obj_get_child(grid_sync_btn, 0);
        if (lbl) lv_label_set_text(lbl, sync_pads_active ? "SYNC\nON" : "SYNC\nOFF");
    }
    // Sync state to S3
    uart_send_to_s3(MSG_TOUCH_CMD, TCMD_SYNC_PADS, sync_pads_active ? 1 : 0);
}

// Cycle Note Repeat: OFF → 1/4 → 1/8 → 1/16 → 1/32 → 1/8T → 1/16T → OFF
static void grid_nr_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t idx = (uint8_t)((s_nr_idx + 1) % 7);
    s_nr_idx = idx;
    if (grid_nr_btn) {
        bool on = (idx != 0);
        lv_obj_set_style_bg_color(grid_nr_btn,
            on ? RED808_ACCENT : RED808_SURFACE, 0);
        lv_obj_set_style_border_color(grid_nr_btn,
            on ? RED808_ACCENT2 : RED808_BORDER, 0);
        if (grid_nr_lbl) lv_label_set_text(grid_nr_lbl, NR_LABEL[idx]);
    }
}

// Toggle 16 Levels — all pads play the last-tapped sample at 16 velocities
static void grid_16l_cb(lv_event_t* e) {
    LV_UNUSED(e);
    bool on = !s_16l_active;
    s_16l_active = on;
    if (grid_16l_btn) {
        lv_obj_set_style_bg_color(grid_16l_btn,
            on ? RED808_CYAN : RED808_SURFACE, 0);
        lv_obj_set_style_border_color(grid_16l_btn,
            on ? RED808_ACCENT : RED808_BORDER, 0);
        if (grid_16l_lbl) {
            if (on) {
                lv_label_set_text_fmt(grid_16l_lbl, "16 LVL\nSRC %d", s_16l_src_pad + 1);
            } else {
                lv_label_set_text(grid_16l_lbl, "16 LVL\nOFF");
            }
        }
    }
}

// Called when S3 sends sync toggle — update UI without re-sending
void ui_live_set_sync_p4(bool on) {
    sync_pads_active = on;
    if (grid_sync_btn) {
        lv_obj_set_style_bg_color(grid_sync_btn,
            on ? RED808_SUCCESS : RED808_SURFACE, 0);
        lv_obj_set_style_border_color(grid_sync_btn,
            on ? RED808_CYAN : RED808_BORDER, 0);
        lv_obj_t* lbl = lv_obj_get_child(grid_sync_btn, 0);
        if (lbl) lv_label_set_text(lbl, on ? "SYNC\nON" : "SYNC\nOFF");
    }
}

static void grid_theme_cb(lv_event_t* e) {
    LV_UNUSED(e);
    int next = ((int)currentTheme + 1) % THEME_COUNT;
    p4.theme = next;
    ui_theme_apply((VisualTheme)next);
    // Sync theme to S3
    uart_send_to_s3(MSG_TOUCH_CMD, TCMD_THEME_NEXT, (uint8_t)next);
}

static void grid_master_vol_step_cb(lv_event_t* e) {
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    int next = constrain((int)p4.master_volume + delta, 0, Config::MAX_VOLUME);
    if (next == p4.master_volume) return;
    p4.master_volume = (uint8_t)next;
    if (udp_wifi_connected() || udp_master_connected()) udp_send_set_volume(next);
}

static void grid_bpm_step_cb(lv_event_t* e) {
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    int next = constrain((int)p4.bpm_int + delta, 40, 240);
    if (next == p4.bpm_int) return;
    p4.bpm_int = (uint16_t)next;
    p4.bpm_frac = 0;
    if (udp_wifi_connected() || udp_master_connected()) udp_send_tempo((float)next);
}

static void grid_pad_prev_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t pad = s_pad_inst_focus_pad;
    pad = (pad == 0) ? 15 : (uint8_t)(pad - 1);
    s_pad_inst_focus_pad = pad;
    pad_inst_refresh_controls();
}

static void grid_pad_next_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t pad = s_pad_inst_focus_pad;
    pad = (pad >= 15) ? 0 : (uint8_t)(pad + 1);
    s_pad_inst_focus_pad = pad;
    pad_inst_refresh_controls();
}

static void grid_inst_prev_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    uint8_t inst = s_pad_inst_sel[pad];
    inst = (inst == 0) ? 7 : (uint8_t)(inst - 1);
    s_pad_inst_sel[pad] = inst;
    pad_inst_refresh_pad_badge(pad);
    pad_inst_refresh_controls();
    pad_inst_apply_to_master(pad);
}

static void grid_inst_next_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    uint8_t inst = s_pad_inst_sel[pad];
    inst = (inst >= 7) ? 0 : (uint8_t)(inst + 1);
    s_pad_inst_sel[pad] = inst;
    pad_inst_refresh_pad_badge(pad);
    pad_inst_refresh_controls();
    pad_inst_apply_to_master(pad);
}

static void pad_inst_modal_refresh(void) {
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    uint8_t inst_assigned = s_pad_inst_sel[pad];
    if (inst_assigned > 7) inst_assigned = 0;
    uint8_t inst_pending = s_pad_inst_pending[pad];
    if (inst_pending > 7) inst_pending = 0;
    bool dirty = (inst_pending != inst_assigned);
    if (s_pad_inst_modal_pad_lbl)
        lv_label_set_text_fmt(s_pad_inst_modal_pad_lbl, "PAD %02d", (int)pad + 1);
    if (s_pad_inst_modal_inst_lbl) {
        if (dirty)
            lv_label_set_text_fmt(s_pad_inst_modal_inst_lbl, "%s  >  %s",
                                  PAD_INST_NAMES[inst_assigned], PAD_INST_NAMES[inst_pending]);
        else
            lv_label_set_text(s_pad_inst_modal_inst_lbl, PAD_INST_NAMES[inst_assigned]);
        lv_obj_set_style_text_color(s_pad_inst_modal_inst_lbl,
            dirty ? RED808_WARNING : RED808_TEXT, 0);
    }
    for (int i = 0; i < 16; i++) {
        lv_obj_t* btn = s_pad_inst_modal_pad_btns[i];
        if (!btn) continue;
        bool active = (i == (int)pad);
        lv_obj_set_style_bg_color(btn, active ? RED808_ACCENT2 : RED808_SURFACE, 0);
        lv_obj_set_style_border_color(btn, active ? RED808_CYAN : RED808_BORDER, 0);
        lv_obj_set_style_bg_opa(btn, active ? LV_OPA_COVER : LV_OPA_80, 0);
        lv_obj_set_style_border_width(btn, active ? 2 : 1, 0);
    }
    for (int i = 0; i < 8; i++) {
        lv_obj_t* btn = s_pad_inst_modal_inst_btns[i];
        if (!btn) continue;
        bool is_pending = (i == (int)inst_pending);
        bool is_assigned = (i == (int)inst_assigned);
        lv_color_t bg = RED808_SURFACE;
        lv_color_t bd = RED808_BORDER;
        lv_opa_t op = LV_OPA_80;
        int bw = 1;
        if (is_pending && dirty) {
            bg = RED808_WARNING;  // amarillo: seleccionado, pendiente de asignar
            bd = RED808_CYAN;
            op = LV_OPA_COVER;
            bw = 3;
        } else if (is_assigned) {
            bg = RED808_ACCENT;
            bd = RED808_CYAN;
            op = LV_OPA_COVER;
            bw = 2;
        }
        lv_obj_set_style_bg_color(btn, bg, 0);
        lv_obj_set_style_border_color(btn, bd, 0);
        lv_obj_set_style_bg_opa(btn, op, 0);
        lv_obj_set_style_border_width(btn, bw, 0);
    }

    // Kit chips: la fila del engine drum pendiente queda activa y resalta el
    // kit pendiente del pad activo. Las otras filas quedan atenuadas.
    int8_t drum_eng = pad_inst_drum_engine_idx(inst_pending);
    uint8_t pad_kit = s_pad_kit_pending[pad];
    if (pad_kit > 4) pad_kit = 0;
    for (int eng = 0; eng < 3; eng++) {
        bool row_active = (eng == drum_eng);
        if (s_pad_inst_modal_kit_lbl_eng[eng]) {
            lv_obj_set_style_text_color(s_pad_inst_modal_kit_lbl_eng[eng],
                row_active ? RED808_ACCENT : RED808_TEXT_DIM, 0);
        }
        for (int p = 0; p < 5; p++) {
            lv_obj_t* kb = s_pad_inst_modal_kit_btns[eng][p];
            if (!kb) continue;
            bool is_pending_kit = row_active && (p == (int)pad_kit);
            lv_color_t bg = RED808_SURFACE;
            lv_color_t bd = RED808_BORDER;
            lv_opa_t op = row_active ? LV_OPA_80 : LV_OPA_30;
            int bw = 1;
            if (is_pending_kit) {
                bg = (p == 4) ? RED808_SUCCESS : RED808_ACCENT;
                bd = RED808_CYAN;
                op = LV_OPA_COVER;
                bw = 3;
            } else if (p == 4 && row_active) {
                // Pure: mantén sello visual cuando la fila está activa
                bd = RED808_CYAN;
                bw = 2;
            }
            lv_obj_set_style_bg_color(kb, bg, 0);
            lv_obj_set_style_border_color(kb, bd, 0);
            lv_obj_set_style_bg_opa(kb, op, 0);
            lv_obj_set_style_border_width(kb, bw, 0);
            if (row_active) {
                lv_obj_add_flag(kb, LV_OBJ_FLAG_CLICKABLE);
            } else {
                lv_obj_clear_flag(kb, LV_OBJ_FLAG_CLICKABLE);
            }
        }
    }
}

static void pad_inst_modal_close_cb(lv_event_t* e) {
    if (e && lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (s_pad_inst_modal) {
        lv_obj_del(s_pad_inst_modal);
        s_pad_inst_modal = NULL;
        s_pad_inst_modal_pad_lbl = NULL;
        s_pad_inst_modal_inst_lbl = NULL;
        for (int i = 0; i < 16; i++) s_pad_inst_modal_pad_btns[i] = NULL;
        for (int i = 0; i < 8; i++) s_pad_inst_modal_inst_btns[i] = NULL;
        for (int e2 = 0; e2 < 3; e2++) {
            s_pad_inst_modal_kit_lbl_eng[e2] = NULL;
            for (int p = 0; p < 5; p++) s_pad_inst_modal_kit_btns[e2][p] = NULL;
        }
    }
}

// fwd decl: defined later; used inside the PAD SOUND modal builder
static void grid_pad_kit_select_cb(lv_event_t* e);

static void pad_inst_modal_pick_pad_cb(lv_event_t* e) {
    int pad = (int)(intptr_t)lv_event_get_user_data(e);
    if (pad < 0 || pad > 15) return;
    s_pad_inst_focus_pad = (uint8_t)pad;
    // Al cambiar de pad, descarta pendiente del anterior y empieza con
    // la asignación actual del nuevo.
    s_pad_inst_pending[pad] = s_pad_inst_sel[pad];
    pad_inst_refresh_controls();
    pad_inst_modal_refresh();
}

static void pad_inst_modal_pad_cb(lv_event_t* e) {
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    uint8_t pad = s_pad_inst_focus_pad;
    pad = (uint8_t)((pad + delta + 16) % 16);
    s_pad_inst_focus_pad = pad;
    pad_inst_refresh_controls();
    pad_inst_modal_refresh();
}

static void pad_inst_modal_inst_cb(lv_event_t* e) {
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    uint8_t inst = s_pad_inst_sel[pad];
    inst = (uint8_t)((inst + delta + 8) % 8);
    s_pad_inst_sel[pad] = inst;
    pad_inst_refresh_pad_badge(pad);
    pad_inst_refresh_controls();
    pad_inst_apply_to_master(pad);
    pad_inst_modal_refresh();
}

static bool pad_inst_unload_daisy_sample(uint8_t pad) {
    // NOTE: this runs on the LVGL task under lvgl_mutex, so any blocking here
    // freezes the whole UI and starves touch. Keep the timeouts tight; a missed
    // unload is harmless (it just falls back to the previous sample).
    WiFiClient client;
    if (!client.connect(IPAddress(192, 168, 4, 1), 80, 400)) return false;
    client.printf("POST /api/unloadDaisy?pad=%d HTTP/1.1\r\n", pad);
    client.print("Host: 192.168.4.1\r\n");
    client.print("Content-Length: 0\r\n");
    client.print("Connection: close\r\n\r\n");
    uint32_t start = millis();
    while (!client.available() && client.connected() && (millis() - start) < 400) delay(5);
    String statusLine = client.readStringUntil('\n');
    statusLine.trim();
    client.stop();
    return statusLine.startsWith("HTTP/1.1 200") || statusLine.startsWith("HTTP/1.0 200");
}

static void pad_inst_sampler_original_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    if (!udp_wifi_connected() && !udp_master_connected()) {
        ui_show_toast("Master no conectado", RED808_WARNING);
        return;
    }
    bool ok = pad_inst_unload_daisy_sample(pad);
    if (!ok) {
        ui_show_toast("No se pudo restaurar sample", RED808_WARNING);
        return;
    }
    s_pad_inst_sel[pad] = 0;
    pad_inst_refresh_pad_badge(pad);
    pad_inst_refresh_controls();
    pad_inst_apply_to_master(pad);
    pad_inst_modal_refresh();
    ui_show_toast("Sampler original restaurado", RED808_SUCCESS);
}

static void pad_inst_modal_pick_inst_cb(lv_event_t* e) {
    int inst = (int)(intptr_t)lv_event_get_user_data(e);
    if (inst < 0 || inst > 7) return;
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    // Solo marca la selección como pendiente. La asignación al master se hace
    // al pulsar PREVIEW (escuchar) o ASIGNAR (confirmar).
    s_pad_inst_pending[pad] = (uint8_t)inst;
    pad_inst_modal_refresh();
}

// Aplica al master la selección pendiente del pad (instrumento + kit si drum).
// Devuelve true si se cambió algo.
static bool pad_inst_commit_pending(uint8_t pad) {
    if (pad > 15) return false;
    bool changed = false;
    uint8_t inst = s_pad_inst_pending[pad];
    if (inst > 7) inst = 0;
    int8_t drum = pad_inst_drum_engine_idx(inst);
    // Cambio de kit por pad. El envío a la Daisy se deduplica con el último
    // kit aplicado al engine: solo se manda CMD_SYNTH_PRESET si difiere del
    // último. (En tiempo real, ui_process_pad_queue también hace switch).
    if (drum >= 0) {
        uint8_t kit = s_pad_kit_pending[pad];
        if (kit > 4) kit = 0;
        if (s_pad_kit_assigned[pad] != kit) {
            s_pad_kit_assigned[pad] = kit;
            changed = true;
        }
        if (udp_wifi_connected() && s_engine_kit_last_applied[drum] != (int8_t)kit) {
            udp_send_synth_preset((uint8_t)drum, kit);
            s_engine_kit_last_applied[drum] = (int8_t)kit;
            changed = true;
        }
    }
    // Cambio de instrumento (engine asignado al pad)
    if (s_pad_inst_sel[pad] != inst) {
        s_pad_inst_sel[pad] = inst;
        pad_inst_refresh_pad_badge(pad);
        pad_inst_refresh_controls();
        pad_inst_apply_to_master(pad);  // udp_send_set_track_engine + trigger
        changed = true;
    }
    return changed;
}

// PREVIEW: aplica la selección pendiente y dispara el pad para escuchar.
static void pad_inst_modal_preview_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    if (!udp_wifi_connected() && !udp_master_connected()) {
        ui_show_toast("Master no conectado", RED808_WARNING);
        return;
    }
    bool changed = pad_inst_commit_pending(pad);
    if (!changed) {
        // Sin cambios: dispara igualmente para volver a oír
        udp_send_trigger(pad, 110);
    }
    pad_inst_modal_refresh();
}

// ASIGNAR: confirma la selección pendiente y cierra el modal.
static void pad_inst_modal_assign_cb(lv_event_t* e) {
    LV_UNUSED(e);
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    if (!udp_wifi_connected() && !udp_master_connected()) {
        ui_show_toast("Master no conectado", RED808_WARNING);
        return;
    }
    pad_inst_commit_pending(pad);
    ui_show_toast("Asignado", RED808_SUCCESS);
    pad_inst_modal_close_cb(NULL);
}

static void grid_pad_inst_popup_cb(lv_event_t* e) {
    if (lv_event_get_target(e) != lv_event_get_current_target(e)) return;
    if (s_pad_inst_modal) {
        pad_inst_modal_close_cb(NULL);
        return;
    }

    // Inicializa la selección pendiente con la asignación actual de cada pad
    for (int i = 0; i < 16; i++) {
        s_pad_inst_pending[i] = s_pad_inst_sel[i];
        s_pad_kit_pending[i]  = s_pad_kit_assigned[i];
    }

    s_pad_inst_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_pad_inst_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(s_pad_inst_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_pad_inst_modal, LV_OPA_70, 0);
    lv_obj_set_style_border_width(s_pad_inst_modal, 0, 0);
    lv_obj_set_style_pad_all(s_pad_inst_modal, 0, 0);
    lv_obj_clear_flag(s_pad_inst_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_pad_inst_modal, pad_inst_modal_close_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(s_pad_inst_modal);
    lv_obj_set_size(card, 880, 500);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(card, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, RED808_CYAN, 0);
    lv_obj_set_style_radius(card, 18, 0);
    lv_obj_set_style_pad_all(card, 14, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text(title, "PAD INSTRUMENT SELECT");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(title, RED808_CYAN, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 2);

    lv_obj_t* sub = lv_label_create(card);
    lv_label_set_text(sub, "Seleccion directa por instrumento");
    lv_obj_set_style_text_font(sub, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sub, RED808_TEXT_DIM, 0);
    lv_obj_align(sub, LV_ALIGN_TOP_MID, 0, 32);

    s_pad_inst_modal_pad_lbl = lv_label_create(card);
    lv_obj_set_style_text_font(s_pad_inst_modal_pad_lbl, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(s_pad_inst_modal_pad_lbl, RED808_ACCENT, 0);
    lv_obj_align(s_pad_inst_modal_pad_lbl, LV_ALIGN_TOP_MID, 0, 64);

    s_pad_inst_modal_inst_lbl = lv_label_create(card);
    lv_obj_set_style_text_font(s_pad_inst_modal_inst_lbl, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(s_pad_inst_modal_inst_lbl, RED808_TEXT, 0);
    lv_obj_align(s_pad_inst_modal_inst_lbl, LV_ALIGN_TOP_MID, 0, 98);

    const int pad_grid_x0 = 32;
    const int pad_grid_y0 = 138;
    const int pad_btn_w = 56;
    const int pad_btn_h = 30;
    const int pad_gap_x = 8;
    const int pad_gap_y = 8;
    for (int i = 0; i < 16; i++) {
        lv_obj_t* pb = lv_btn_create(card);
        s_pad_inst_modal_pad_btns[i] = pb;
        int col = i % 4;
        int row = i / 4;
        lv_obj_set_size(pb, pad_btn_w, pad_btn_h);
        lv_obj_set_pos(pb, pad_grid_x0 + col * (pad_btn_w + pad_gap_x), pad_grid_y0 + row * (pad_btn_h + pad_gap_y));
        apply_control_button_style(pb, RED808_BORDER, false, 8);
        lv_obj_t* pl = lv_label_create(pb);
        lv_label_set_text_fmt(pl, "%02d", i + 1);
        lv_obj_set_style_text_font(pl, &lv_font_montserrat_12, 0);
        lv_obj_center(pl);
        lv_obj_add_event_cb(pb, pad_inst_modal_pick_pad_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    }

    lv_obj_t* pad_hdr = lv_label_create(card);
    lv_label_set_text(pad_hdr, "PAD");
    lv_obj_set_style_text_font(pad_hdr, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(pad_hdr, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(pad_hdr, pad_grid_x0, pad_grid_y0 - 20);

    const int grid_x0 = 326;
    const int grid_y0 = 138;
    const int grid_cols = 4;
    const int btn_w = 104;
    const int btn_h = 38;
    const int gap_x = 8;
    const int gap_y = 8;

    lv_obj_t* inst_hdr = lv_label_create(card);
    lv_label_set_text(inst_hdr, "INSTRUMENT");
    lv_obj_set_style_text_font(inst_hdr, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(inst_hdr, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(inst_hdr, grid_x0, grid_y0 - 20);

    for (int i = 0; i < 8; i++) {
        lv_obj_t* ib = lv_btn_create(card);
        s_pad_inst_modal_inst_btns[i] = ib;
        int col = i % grid_cols;
        int row = i / grid_cols;
        lv_obj_set_size(ib, btn_w, btn_h);
        lv_obj_set_pos(ib, grid_x0 + col * (btn_w + gap_x), grid_y0 + row * (btn_h + gap_y));
        apply_control_button_style(ib, RED808_BORDER, false, 10);
        lv_obj_t* il = lv_label_create(ib);
        lv_label_set_text(il, PAD_INST_NAMES[i]);
        lv_obj_set_style_text_font(il, &lv_font_montserrat_14, 0);
        lv_obj_center(il);
        lv_obj_add_event_cb(ib, pad_inst_modal_pick_inst_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    }

    /* === DRUM KITS — preset selector (engines × presets, Pure incluido) === */
    const int kits_y0 = grid_y0 + 2 * (btn_h + gap_y) + 18;  // tras el grid 2x4 de instruments

    lv_obj_t* kit_hdr = lv_label_create(card);
    lv_label_set_text(kit_hdr, "DRUM KITS");
    lv_obj_set_style_text_font(kit_hdr, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(kit_hdr, RED808_CYAN, 0);
    lv_obj_set_pos(kit_hdr, grid_x0, kits_y0 - 18);

    static const char* engine_names[3]      = { "808", "909", "505" };
    static const char* preset_names_808[5] = { "Classic", "HipHop", "Techno",  "Latin",      "Pure 808" };
    static const char* preset_names_909[5] = { "Classic", "Techno", "House",   "Industrial", "Pure 909" };
    static const char* preset_names_505[5] = { "Classic", "NewWav", "Electro", "LoFi HH",    "Pure 505" };
    const char* const* preset_names[3] = { preset_names_808, preset_names_909, preset_names_505 };

    const int eng_lbl_w = 56;
    const int kit_btn_h = 36;
    const int kit_gap_x = 6;
    const int kit_gap_y = 6;
    const int kit_btn_w = (4 * btn_w + 3 * gap_x - eng_lbl_w - kit_gap_x - 4 * kit_gap_x) / 5;

    for (int eng = 0; eng < 3; eng++) {
        int row_y = kits_y0 + eng * (kit_btn_h + kit_gap_y);

        lv_obj_t* eng_lbl = lv_label_create(card);
        lv_label_set_text(eng_lbl, engine_names[eng]);
        lv_obj_set_style_text_font(eng_lbl, &lv_font_montserrat_18, 0);
        lv_obj_set_style_text_color(eng_lbl, RED808_ACCENT, 0);
        lv_obj_set_pos(eng_lbl, grid_x0, row_y + 6);
        s_pad_inst_modal_kit_lbl_eng[eng] = eng_lbl;

        for (int p = 0; p < 5; p++) {
            bool is_pure = (p == 4);
            lv_obj_t* kb = lv_btn_create(card);
            s_pad_inst_modal_kit_btns[eng][p] = kb;
            lv_obj_set_size(kb, kit_btn_w, kit_btn_h);
            lv_obj_set_pos(kb,
                grid_x0 + eng_lbl_w + kit_gap_x + p * (kit_btn_w + kit_gap_x),
                row_y);
            apply_control_button_style(kb, RED808_BORDER, false, 8);
            if (is_pure) {
                lv_obj_set_style_border_color(kb, RED808_CYAN, 0);
                lv_obj_set_style_border_width(kb, 2, 0);
            }
            lv_obj_t* kl = lv_label_create(kb);
            lv_label_set_text(kl, preset_names[eng][p]);
            lv_obj_set_style_text_font(kl, &lv_font_montserrat_12, 0);
            lv_obj_set_style_text_color(kl, RED808_TEXT, 0);
            lv_obj_center(kl);
            uint32_t key = ((uint32_t)eng << 8) | (uint32_t)p;
            lv_obj_add_event_cb(kb, grid_pad_kit_select_cb, LV_EVENT_CLICKED,
                                (void*)(intptr_t)key);
        }
    }

    lv_obj_t* original_btn = lv_btn_create(card);
    lv_obj_set_size(original_btn, 180, 44);
    lv_obj_align(original_btn, LV_ALIGN_BOTTOM_LEFT, 18, -14);
    apply_control_button_style(original_btn, RED808_ACCENT2, false, 10);
    lv_obj_t* original_lbl = lv_label_create(original_btn);
    lv_label_set_text(original_lbl, "SAMPLER ORIG.");
    lv_obj_set_style_text_font(original_lbl, &lv_font_montserrat_14, 0);
    lv_obj_center(original_lbl);
    lv_obj_add_event_cb(original_btn, pad_inst_sampler_original_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* preview_btn = lv_btn_create(card);
    lv_obj_set_size(preview_btn, 160, 44);
    lv_obj_align(preview_btn, LV_ALIGN_BOTTOM_MID, -90, -14);
    apply_control_button_style(preview_btn, RED808_SURFACE, false, 10);
    lv_obj_set_style_border_color(preview_btn, RED808_CYAN, 0);
    lv_obj_set_style_border_width(preview_btn, 2, 0);
    lv_obj_t* preview_lbl = lv_label_create(preview_btn);
    lv_label_set_text(preview_lbl, LV_SYMBOL_PLAY "  PREVIEW");
    lv_obj_set_style_text_font(preview_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(preview_lbl, RED808_TEXT, 0);
    lv_obj_center(preview_lbl);
    lv_obj_add_event_cb(preview_btn, pad_inst_modal_preview_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* assign_btn = lv_btn_create(card);
    lv_obj_set_size(assign_btn, 160, 44);
    lv_obj_align(assign_btn, LV_ALIGN_BOTTOM_MID, 90, -14);
    apply_control_button_style(assign_btn, RED808_SUCCESS, false, 10);
    lv_obj_set_style_border_color(assign_btn, RED808_CYAN, 0);
    lv_obj_set_style_border_width(assign_btn, 2, 0);
    lv_obj_t* assign_lbl = lv_label_create(assign_btn);
    lv_label_set_text(assign_lbl, LV_SYMBOL_OK "  ASIGNAR");
    lv_obj_set_style_text_font(assign_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(assign_lbl, RED808_TEXT, 0);
    lv_obj_center(assign_lbl);
    lv_obj_add_event_cb(assign_btn, pad_inst_modal_assign_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* close_btn = lv_btn_create(card);
    lv_obj_set_size(close_btn, 110, 44);
    lv_obj_align(close_btn, LV_ALIGN_BOTTOM_RIGHT, -18, -14);
    apply_control_button_style(close_btn, RED808_BORDER, false, 10);
    lv_obj_t* close_lbl = lv_label_create(close_btn);
    lv_label_set_text(close_lbl, "CERRAR");
    lv_obj_center(close_lbl);
    lv_obj_add_event_cb(close_btn, pad_inst_modal_close_cb, LV_EVENT_CLICKED, NULL);

    pad_inst_modal_refresh();
}

// Helper: styled control button
static lv_obj_t* create_ctrl_btn(lv_obj_t* parent, int x, int y, int w, int h,
                                  const char* text, lv_color_t border_color,
                                  const lv_font_t* font) {
    lv_obj_t* btn = lv_btn_create(parent);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_style_radius(btn, 14, 0);
    lv_obj_set_style_bg_color(btn, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_color(btn, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(btn, 2, 0);
    lv_obj_set_style_border_color(btn, border_color, 0);
    lv_obj_set_style_shadow_width(btn, 0, 0);
    lv_obj_set_style_bg_color(btn, border_color, LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(btn, LV_OPA_80, LV_STATE_PRESSED);

    lv_obj_t* label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, RED808_TEXT, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(label);
    return btn;
}

// Helper: info display cell (non-clickable)
static lv_obj_t* create_info_cell(lv_obj_t* parent, int x, int y, int w, int h,
                                   const char* title, const char* value,
                                   lv_color_t value_color, lv_obj_t** value_out) {
    lv_obj_t* panel = lv_obj_create(parent);
    lv_obj_set_size(panel, w, h);
    lv_obj_set_pos(panel, x, y);
    lv_obj_set_style_radius(panel, 14, 0);
    lv_obj_set_style_bg_color(panel, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(panel, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(panel, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_border_color(panel, RED808_BORDER, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* t = lv_label_create(panel);
    lv_label_set_text(t, title);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(t, RED808_TEXT_DIM, 0);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 4);

    lv_obj_t* v = lv_label_create(panel);
    lv_label_set_text(v, value);
    lv_obj_set_style_text_font(v, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(v, value_color, 0);
    lv_obj_align(v, LV_ALIGN_CENTER, 0, 10);

    if (value_out) *value_out = v;
    return panel;
}

// =============================================================================
// PAD LAYOUT — resize/reposition live_pad_btns for 6 display modes
// =============================================================================
static void apply_pad_layout(int mode) {
    s_pad_mode = mode;
    const int M = 8, G = 4, SCR_W = 1024, SCR_H = 600;
    int cols, count, pw, ph;
    switch (mode) {
        default:
        case 0: cols=4; count=16; pw=122;                ph=143;                break;
        case 1: cols=4; count=16; pw=(SCR_W-2*M-3*G)/4;  ph=(SCR_H-2*M-3*G)/4; break;
        case 2: cols=4; count=8;  pw=(SCR_W-2*M-3*G)/4;  ph=(SCR_H-2*M-1*G)/2; break;
        case 3: cols=2; count=4;  pw=(SCR_W-2*M-1*G)/2;  ph=(SCR_H-2*M-1*G)/2; break;
        case 4: cols=2; count=2;  pw=(SCR_W-2*M-1*G)/2;  ph=(SCR_H-2*M);        break;
        case 5: cols=1; count=1;  pw=(SCR_W-2*M);         ph=(SCR_H-2*M);        break;
    }

    bool fullscreen = (mode != 0);
    for (int i = 0; i < live_home_panel_count && i < (int)(sizeof(live_home_panels) / sizeof(live_home_panels[0])); i++) {
        if (!live_home_panels[i]) continue;
        if (fullscreen) lv_obj_add_flag(live_home_panels[i], LV_OBJ_FLAG_HIDDEN);
        else            lv_obj_clear_flag(live_home_panels[i], LV_OBJ_FLAG_HIDDEN);
    }

    for (int i = 0; i < 16; i++) {
        if (!live_pad_btns[i]) continue;
        if (i < count) {
            int c = i % cols, r = i / cols;
            lv_obj_set_size(live_pad_btns[i], pw, ph);
            lv_obj_set_pos(live_pad_btns[i], M + c*(pw+G), M + r*(ph+G));
            if (live_pad_glow[i]) {
                lv_obj_set_size(live_pad_glow[i], pw, ph);  // keep glow full-pad
                lv_obj_center(live_pad_glow[i]);
            }
            lv_obj_clear_flag(live_pad_btns[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(live_pad_btns[i]);
        } else {
            lv_obj_add_flag(live_pad_btns[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (s_pad_back_btn) {
        if (mode == 0) lv_obj_add_flag(s_pad_back_btn, LV_OBJ_FLAG_HIDDEN);
        else { lv_obj_clear_flag(s_pad_back_btn, LV_OBJ_FLAG_HIDDEN); lv_obj_move_foreground(s_pad_back_btn); }
    }
}

static void pad_mode_select_cb(lv_event_t* e) {
    int mode = (int)(intptr_t)lv_event_get_user_data(e);
    if (s_pad_mode_modal) { lv_obj_del(s_pad_mode_modal); s_pad_mode_modal = NULL; }
    apply_pad_layout(mode);
    if (lv_scr_act() != scr_live) ui_navigate_to(2);
}

// Kit chip click — marca el preset como pendiente para el PAD ACTIVO.
static void grid_pad_kit_select_cb(lv_event_t* e) {
    uint32_t key = (uint32_t)(intptr_t)lv_event_get_user_data(e);
    uint8_t engine = (uint8_t)(key >> 8);   // 0=808, 1=909, 2=505
    uint8_t preset = (uint8_t)(key & 0xFF); // 0..4
    if (engine > 2 || preset > 4) return;
    uint8_t pad = s_pad_inst_focus_pad;
    if (pad > 15) pad = 15;
    // Solo permite cambiar el kit si el inst pendiente del pad coincide con
    // el engine de la fila pulsada. (El refresh ya las desactiva visualmente,
    // pero defendemos por si acaso.)
    int8_t drum = pad_inst_drum_engine_idx(s_pad_inst_pending[pad]);
    if (drum != (int8_t)engine) return;
    s_pad_kit_pending[pad] = preset;
    pad_inst_modal_refresh();
}

// PAD MODE modal — solo selector de visualización (1/2/4/8/16 pads)
static void grid_pad_mode_cb(lv_event_t* e) {
    if (s_pad_mode_modal) { lv_obj_del(s_pad_mode_modal); s_pad_mode_modal = NULL; return; }

    s_pad_mode_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_pad_mode_modal, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(s_pad_mode_modal, 0, 0);
    lv_obj_set_style_bg_color(s_pad_mode_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_pad_mode_modal, LV_OPA_70, 0);
    lv_obj_set_style_border_width(s_pad_mode_modal, 0, 0);
    lv_obj_set_style_radius(s_pad_mode_modal, 0, 0);
    lv_obj_set_style_pad_all(s_pad_mode_modal, 0, 0);
    lv_obj_clear_flag(s_pad_mode_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_pad_mode_modal, [](lv_event_t*){
        if (s_pad_mode_modal) { lv_obj_del(s_pad_mode_modal); s_pad_mode_modal = NULL; }
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t* card = lv_obj_create(s_pad_mode_modal);
    lv_obj_set_size(card, 360, 400);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(card, RED808_ACCENT2, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_pad_all(card, 14, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* title = lv_label_create(card);
    lv_label_set_text(title, "PAD GRID");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(title, RED808_ACCENT2, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 2);

    static const char* mode_labels[] = {
        "16 PADS", "16 PADS  FULLSCREEN",
        "8 PADS  FULLSCREEN", "4 PADS  FULLSCREEN",
        "2 PADS  FULLSCREEN", "1 PAD  FULLSCREEN"
    };
    for (int i = 0; i < 6; i++) {
        bool sel = (i == s_pad_mode);
        lv_obj_t* btn = lv_btn_create(card);
        lv_obj_set_size(btn, 320, 44);
        lv_obj_set_pos(btn, 6, 44 + i*52);
        apply_control_button_style(btn, sel ? RED808_ACCENT2 : RED808_SURFACE, sel, 10);
        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text(lbl, mode_labels[i]);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(lbl, RED808_TEXT, 0);
        lv_obj_center(lbl);
        lv_obj_add_event_cb(btn, pad_mode_select_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    }
}

static void create_live_screen(void) {
    scr_live = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_live, RED808_BG, 0);
    lv_obj_clear_flag(scr_live, LV_OBJ_FLAG_SCROLLABLE);

    // 8×4 full-screen grid: 1024×600
    // Left 4 cols = pads, Right 4 cols = controls
    const int M = 8, G = 4, CG = 8, CW = 122, CH = 143;
    #define COL_X(c) ((c) < 4 ? (M + (c)*(CW+G)) : (M + 4*(CW+G) + CG + ((c)-4)*(CW+G)))
    #define ROW_Y(r) (M + (r)*(CH+G))

    // Vertical separator
    lv_obj_t* sep = lv_obj_create(scr_live);
    lv_obj_set_size(sep, 2, LCD_V_RES - 2*M);
    lv_obj_set_pos(sep, M + 4*(CW+G) + CG/2 - 1, M);
    lv_obj_set_style_bg_color(sep, RED808_BORDER, 0);
    lv_obj_set_style_bg_opa(sep, LV_OPA_40, 0);
    lv_obj_set_style_border_width(sep, 0, 0);
    lv_obj_set_style_radius(sep, 1, 0);
    lv_obj_clear_flag(sep, LV_OBJ_FLAG_SCROLLABLE);
    live_home_panel_count = 0;
    live_home_panels[live_home_panel_count++] = sep;

    // === LEFT 4×4: Drum Pads (Neon Ring Style) ===
    for (int i = 0; i < 16; i++) {
        int c = i % 4, r = i / 4;
        lv_color_t tc = ui_track_color(i);

        live_pad_btns[i] = lv_btn_create(scr_live);
        lv_obj_set_size(live_pad_btns[i], CW, CH);
        lv_obj_set_pos(live_pad_btns[i], COL_X(c), ROW_Y(r));
        lv_obj_set_style_radius(live_pad_btns[i], 12, 0);
        lv_obj_set_style_bg_color(live_pad_btns[i], RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_color(live_pad_btns[i], RED808_PANEL, 0);
        lv_obj_set_style_bg_grad_dir(live_pad_btns[i], LV_GRAD_DIR_NONE, 0);  // solid: gradients are costly to repaint while hammering
        lv_obj_set_style_bg_opa(live_pad_btns[i], LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(live_pad_btns[i], 3, 0);
        lv_obj_set_style_border_color(live_pad_btns[i], tc, 0);
        lv_obj_set_style_outline_width(live_pad_btns[i], 0, 0);
        lv_obj_set_style_shadow_width(live_pad_btns[i], 0, 0);
        lv_obj_set_style_pad_all(live_pad_btns[i], 0, 0);
        lv_obj_clear_flag(live_pad_btns[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(live_pad_btns[i], pad_touch_cb, LV_EVENT_PRESSED, (void*)(intptr_t)i);

        // Hit-flash overlay — bottom-most child so labels stay on top.
        // NOTE: this used LV_BLEND_MODE_ADDITIVE, but any non-NORMAL blend mode
        // makes LVGL render the object into a temporary layer and composite it
        // (lv_obj_style.c: layer_type SIMPLE). With 16 pads repainting while
        // hammering, that collapsed the render to ~8-12 FPS. Using NORMAL blend
        // with a WHITE overlay keeps the "incandescent" flash on hits with no
        // layer cost — update_live_screen drives the opacity from the band.
        live_pad_glow[i] = lv_obj_create(live_pad_btns[i]);
        lv_obj_remove_style_all(live_pad_glow[i]);
        lv_obj_set_size(live_pad_glow[i], CW, CH);
        lv_obj_center(live_pad_glow[i]);
        lv_obj_set_style_radius(live_pad_glow[i], 12, 0);
        lv_obj_set_style_bg_color(live_pad_glow[i], lv_color_white(), 0);
        lv_obj_set_style_bg_opa(live_pad_glow[i], LV_OPA_TRANSP, 0);
        lv_obj_set_style_blend_mode(live_pad_glow[i], LV_BLEND_MODE_NORMAL, 0);
        lv_obj_clear_flag(live_pad_glow[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(live_pad_glow[i], LV_OBJ_FLAG_CLICKABLE);

        live_pad_accent_strips[i] = lv_obj_create(live_pad_btns[i]);
        lv_obj_set_size(live_pad_accent_strips[i], 6, CH - 24);
        lv_obj_align(live_pad_accent_strips[i], LV_ALIGN_LEFT_MID, 8, 0);
        lv_obj_set_style_radius(live_pad_accent_strips[i], 3, 0);
        lv_obj_set_style_bg_color(live_pad_accent_strips[i], tc, 0);
        lv_obj_set_style_bg_opa(live_pad_accent_strips[i], LV_OPA_70, 0);
        lv_obj_set_style_border_width(live_pad_accent_strips[i], 0, 0);
        lv_obj_clear_flag(live_pad_accent_strips[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(live_pad_accent_strips[i], LV_OBJ_FLAG_CLICKABLE);

        live_pad_num_labels[i] = lv_label_create(live_pad_btns[i]);
        lv_label_set_text_fmt(live_pad_num_labels[i], "%02d", i + 1);
        lv_obj_set_style_text_font(live_pad_num_labels[i], &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(live_pad_num_labels[i], RED808_TEXT_DIM, 0);
        lv_obj_align(live_pad_num_labels[i], LV_ALIGN_TOP_LEFT, 10, 8);

        live_pad_state_labels[i] = lv_label_create(live_pad_btns[i]);
        lv_label_set_text(live_pad_state_labels[i], "");
        lv_obj_set_style_text_font(live_pad_state_labels[i], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(live_pad_state_labels[i], RED808_TEXT_DIM, 0);
        lv_obj_align(live_pad_state_labels[i], LV_ALIGN_TOP_RIGHT, -8, 9);

        live_pad_inst_labels[i] = lv_label_create(live_pad_btns[i]);
        lv_label_set_text(live_pad_inst_labels[i], PAD_INST_SHORT[s_pad_inst_sel[i] & 0x07]);
        lv_obj_set_style_text_font(live_pad_inst_labels[i], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(live_pad_inst_labels[i], tc, 0);
        lv_obj_align(live_pad_inst_labels[i], LV_ALIGN_BOTTOM_RIGHT, -8, -7);

        live_pad_labels[i] = lv_label_create(live_pad_btns[i]);
        lv_label_set_text(live_pad_labels[i], trackNames[i]);
        lv_obj_set_style_text_font(live_pad_labels[i], &lv_font_montserrat_28, 0);
        lv_obj_set_style_text_color(live_pad_labels[i], tc, 0);
        lv_obj_center(live_pad_labels[i]);

        // Spectrum bar — thin horizontal bar at bottom of pad, grows upward
        live_spectrum_bars[i] = lv_obj_create(live_pad_btns[i]);
        lv_obj_set_size(live_spectrum_bars[i], CW - 20, 0);  // starts at 0 height
        lv_obj_align(live_spectrum_bars[i], LV_ALIGN_BOTTOM_MID, 0, -4);
        lv_obj_set_style_bg_color(live_spectrum_bars[i], tc, 0);
        lv_obj_set_style_bg_opa(live_spectrum_bars[i], LV_OPA_60, 0);
        lv_obj_set_style_border_width(live_spectrum_bars[i], 0, 0);
        lv_obj_set_style_radius(live_spectrum_bars[i], 3, 0);
        lv_obj_clear_flag(live_spectrum_bars[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(live_spectrum_bars[i], LV_OBJ_FLAG_CLICKABLE);
    }

    // === RIGHT 4×4: Controls ===

    // --- Row 0: Transport ---
    // [4,0] PLAY / PAUSE
    grid_play_btn = lv_btn_create(scr_live);
    lv_obj_set_size(grid_play_btn, CW, CH);
    lv_obj_set_pos(grid_play_btn, COL_X(4), ROW_Y(0));
    apply_control_button_style(grid_play_btn, RED808_ACCENT2, true, 12);
    lv_obj_set_style_bg_color(grid_play_btn, RED808_ACCENT, 0);
    lv_obj_add_event_cb(grid_play_btn, header_play_cb, LV_EVENT_CLICKED, NULL);
    live_home_panels[live_home_panel_count++] = grid_play_btn;
    grid_play_lbl = lv_label_create(grid_play_btn);
    lv_label_set_text(grid_play_lbl, LV_SYMBOL_PLAY "\nPLAY");
    lv_obj_set_style_text_font(grid_play_lbl, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(grid_play_lbl, RED808_TEXT, 0);
    lv_obj_set_style_text_align(grid_play_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(grid_play_lbl);

    // [5,0] PAT -
    lv_obj_t* b;
    b = create_ctrl_btn(scr_live, COL_X(5), ROW_Y(0), CW, CH,
                         LV_SYMBOL_LEFT "\nPAT", RED808_WARNING, &lv_font_montserrat_20);
    lv_obj_add_event_cb(b, header_pattern_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    live_home_panels[live_home_panel_count++] = b;

    // [6,0] PAT +
    b = create_ctrl_btn(scr_live, COL_X(6), ROW_Y(0), CW, CH,
                         "PAT\n" LV_SYMBOL_RIGHT, RED808_WARNING, &lv_font_montserrat_20);
    lv_obj_add_event_cb(b, header_pattern_cb, LV_EVENT_CLICKED, (void*)(intptr_t)1);
    live_home_panels[live_home_panel_count++] = b;

    // [7,0] Home status cell — pattern + link health
    lv_obj_t* home_cell = create_info_cell(scr_live, COL_X(7), ROW_Y(0), CW, CH,
                                           "STATUS", "P01", RED808_WARNING, &grid_bpm_lbl);
    grid_home_status_cell = home_cell;
    {
        // Initial subtle link tint so the cell is correct on first paint and
        // after a theme reload (the per-frame updater only re-tints on change).
        lv_color_t sc0 = ui_master_link_display_on() ? RED808_SUCCESS : RED808_WARNING;
        lv_obj_set_style_bg_color(home_cell, sc0, 0);
        lv_obj_set_style_bg_grad_color(home_cell, RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_dir(home_cell, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_opa(home_cell, LV_OPA_30, 0);
        lv_obj_set_style_border_color(home_cell, sc0, 0);
        lv_obj_set_style_border_opa(home_cell, LV_OPA_60, 0);
        lv_obj_set_style_border_width(home_cell, 2, 0);
    }
    grid_home_vol_lbl = lv_label_create(home_cell);
    lv_label_set_text(grid_home_vol_lbl, "MSTR --  AUX --");
    lv_obj_set_style_text_font(grid_home_vol_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(grid_home_vol_lbl, RED808_CYAN, 0);
    lv_obj_align(grid_home_vol_lbl, LV_ALIGN_BOTTOM_MID, 0, -10);
    live_home_panels[live_home_panel_count++] = home_cell;

    // --- Row 1: STEP + Navigation ---
    // [5..7,1] Screen navigation
    static const char* nav_texts[] = {"FX", "MIXER", "PIANO"};
    static const int   nav_screens[] = {8, 7, 10};
    lv_color_t         nav_colors[] = {RED808_INFO, RED808_SUCCESS, RED808_ACCENT};
    for (int i = 0; i < 3; i++) {
        b = create_ctrl_btn(scr_live, COL_X(5 + i), ROW_Y(1), CW, CH,
                             nav_texts[i], nav_colors[i], &lv_font_montserrat_20);
        lv_obj_add_event_cb(b, grid_nav_cb, LV_EVENT_CLICKED, (void*)(intptr_t)nav_screens[i]);
        live_home_panels[live_home_panel_count++] = b;
    }

    // --- Row 2: XTRA / PAD SOUND / PAD GRID / SYNC ---
    // [4,2] XTRA PADS
    b = create_ctrl_btn(scr_live, COL_X(4), ROW_Y(2), CW, CH,
                         "XTRA\nPADS", RED808_INFO, &lv_font_montserrat_20);
    lv_obj_add_event_cb(b, grid_nav_cb, LV_EVENT_CLICKED, (void*)(intptr_t)6);
    live_home_panels[live_home_panel_count++] = b;

    // [6,2] PAD GRID — selector de visualización (1/2/4/8/16 pads)
    grid_nr_btn = create_ctrl_btn(scr_live, COL_X(6), ROW_Y(2), CW, CH,
                                  "PAD\nGRID", RED808_ACCENT2,
                                  &lv_font_montserrat_18);
    lv_obj_set_style_border_color(grid_nr_btn, RED808_ACCENT2, 0);
    grid_nr_lbl = lv_obj_get_child(grid_nr_btn, 0);
    lv_obj_add_event_cb(grid_nr_btn, grid_pad_mode_cb, LV_EVENT_CLICKED, NULL);
    live_home_panels[live_home_panel_count++] = grid_nr_btn;

    // [7,2] SYNC PADS ON/OFF
    grid_sync_btn = create_ctrl_btn(scr_live, COL_X(7), ROW_Y(2), CW, CH,
                                    sync_pads_active ? "SYNC\nON" : "SYNC\nOFF",
                                    sync_pads_active ? RED808_SUCCESS : RED808_SURFACE,
                                    &lv_font_montserrat_20);
    lv_obj_set_style_border_color(grid_sync_btn,
        sync_pads_active ? RED808_CYAN : RED808_BORDER, 0);
    lv_obj_add_event_cb(grid_sync_btn, grid_sync_cb, LV_EVENT_CLICKED, NULL);
    live_home_panels[live_home_panel_count++] = grid_sync_btn;
    grid_16l_btn = NULL;
    grid_16l_lbl = NULL;

    // --- Row 3: Info Displays ---
    // [4,3] MASTER volume control
    lv_obj_t* vol_panel = lv_obj_create(scr_live);
    lv_obj_set_size(vol_panel, CW, CH);
    lv_obj_set_pos(vol_panel, COL_X(4), ROW_Y(3));
    lv_obj_set_style_radius(vol_panel, 14, 0);
    lv_obj_set_style_bg_color(vol_panel, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(vol_panel, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(vol_panel, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(vol_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(vol_panel, 1, 0);
    lv_obj_set_style_border_color(vol_panel, RED808_BORDER, 0);
    lv_obj_set_style_pad_all(vol_panel, 6, 0);
    lv_obj_clear_flag(vol_panel, LV_OBJ_FLAG_SCROLLABLE);
    live_home_panels[live_home_panel_count++] = vol_panel;

    lv_obj_t* vol_title = lv_label_create(vol_panel);
    lv_label_set_text(vol_title, "MASTER VOL");
    lv_obj_set_style_text_font(vol_title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(vol_title, RED808_TEXT_DIM, 0);
    lv_obj_align(vol_title, LV_ALIGN_TOP_MID, 0, 2);

    lv_obj_t* vol_minus = lv_btn_create(vol_panel);
    lv_obj_set_size(vol_minus, 50, 40);
    lv_obj_set_pos(vol_minus, 4, CH - 54);
    apply_control_button_style(vol_minus, RED808_WARNING, false, 10);
    lv_obj_set_style_transform_zoom(vol_minus, 230, LV_STATE_PRESSED);
    lv_obj_t* vol_minus_lbl = lv_label_create(vol_minus);
    lv_label_set_text(vol_minus_lbl, "-");
    lv_obj_set_style_text_font(vol_minus_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(vol_minus_lbl);
    lv_obj_add_event_cb(vol_minus, grid_master_vol_step_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    lv_obj_add_event_cb(vol_minus, grid_master_vol_step_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void*)(intptr_t)-1);

    lv_obj_t* vol_plus = lv_btn_create(vol_panel);
    lv_obj_set_size(vol_plus, 50, 40);
    lv_obj_set_pos(vol_plus, CW - 62, CH - 54);
    apply_control_button_style(vol_plus, RED808_SUCCESS, false, 10);
    lv_obj_set_style_transform_zoom(vol_plus, 230, LV_STATE_PRESSED);
    lv_obj_t* vol_plus_lbl = lv_label_create(vol_plus);
    lv_label_set_text(vol_plus_lbl, "+");
    lv_obj_set_style_text_font(vol_plus_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(vol_plus_lbl);
    lv_obj_add_event_cb(vol_plus, grid_master_vol_step_cb, LV_EVENT_CLICKED, (void*)(intptr_t)1);
    lv_obj_add_event_cb(vol_plus, grid_master_vol_step_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void*)(intptr_t)1);

    grid_vol_lbl = lv_label_create(vol_panel);
    lv_label_set_text(grid_vol_lbl, "75");
    lv_obj_set_style_text_font(grid_vol_lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(grid_vol_lbl, RED808_ACCENT, 0);
    lv_obj_align(grid_vol_lbl, LV_ALIGN_CENTER, 0, -12);

    // [4,1] STEP ACTIVO + mini secuencer
    lv_obj_t* step_panel = lv_obj_create(scr_live);
    lv_obj_set_size(step_panel, CW, CH);
    lv_obj_set_pos(step_panel, COL_X(4), ROW_Y(1));
    lv_obj_set_style_radius(step_panel, 14, 0);
    lv_obj_set_style_bg_color(step_panel, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(step_panel, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(step_panel, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(step_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(step_panel, 1, 0);
    lv_obj_set_style_border_color(step_panel, RED808_CYAN, 0);
    lv_obj_set_style_pad_all(step_panel, 6, 0);
    lv_obj_clear_flag(step_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(step_panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(step_panel, live_step_nav_cb, LV_EVENT_CLICKED, NULL);
    live_home_panels[live_home_panel_count++] = step_panel;

    lv_obj_t* step_title = lv_label_create(step_panel);
    lv_label_set_text(step_title, "STEP ACTIVO");
    lv_obj_set_style_text_font(step_title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(step_title, RED808_TEXT_DIM, 0);
    lv_obj_align(step_title, LV_ALIGN_TOP_MID, 0, 2);

    grid_step_lbl = lv_label_create(step_panel);
    lv_label_set_text(grid_step_lbl, "--");
    lv_obj_set_style_text_font(grid_step_lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(grid_step_lbl, RED808_CYAN, 0);
    lv_obj_align(grid_step_lbl, LV_ALIGN_TOP_MID, 0, 20);

    const int step_dot_cols = 8;
    const int step_dot_size = 9;
    const int step_dot_gap_x = 13;
    const int step_dot_gap_y = 14;
    const int step_grid_w = (step_dot_cols - 1) * step_dot_gap_x + step_dot_size;
    const int step_start_x = (CW - step_grid_w) / 2;
    const int step_start_y = 74;
    for (int i = 0; i < 16; i++) {
        grid_step_dots[i] = lv_obj_create(step_panel);
        lv_obj_set_size(grid_step_dots[i], step_dot_size, step_dot_size);
        lv_obj_set_pos(grid_step_dots[i],
                       step_start_x + (i % step_dot_cols) * step_dot_gap_x,
                       step_start_y + (i / step_dot_cols) * step_dot_gap_y);
        lv_obj_set_style_radius(grid_step_dots[i], 4, 0);
        lv_obj_set_style_bg_color(grid_step_dots[i], RED808_BORDER, 0);
        lv_obj_set_style_bg_opa(grid_step_dots[i], LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(grid_step_dots[i], 0, 0);
        lv_obj_clear_flag(grid_step_dots[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(grid_step_dots[i], LV_OBJ_FLAG_CLICKABLE);
    }

    // [5,2] PAD SOUND
    lv_obj_t* inst_panel = lv_obj_create(scr_live);
    lv_obj_set_size(inst_panel, CW, CH);
    lv_obj_set_pos(inst_panel, COL_X(5), ROW_Y(2));
    lv_obj_set_style_radius(inst_panel, 14, 0);
    lv_obj_set_style_bg_color(inst_panel, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(inst_panel, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(inst_panel, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(inst_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(inst_panel, 2, 0);
    lv_obj_set_style_border_color(inst_panel, RED808_ACCENT2, 0);
    lv_obj_set_style_pad_all(inst_panel, 8, 0);
    lv_obj_clear_flag(inst_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(inst_panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(inst_panel, grid_pad_inst_popup_cb, LV_EVENT_CLICKED, NULL);
    live_home_panels[live_home_panel_count++] = inst_panel;

    grid_pad_prev_btn = NULL;
    grid_pad_next_btn = NULL;
    grid_pad_lbl = NULL;
    grid_inst_prev_btn = NULL;
    grid_inst_next_btn = NULL;
    grid_inst_lbl = NULL;

    grid_inst_edit_btn = lv_label_create(inst_panel);
    lv_label_set_text(grid_inst_edit_btn, "PAD\nSOUND");
    lv_obj_set_width(grid_inst_edit_btn, CW - 18);
    lv_obj_set_style_text_font(grid_inst_edit_btn, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(grid_inst_edit_btn, RED808_TEXT, 0);
    lv_obj_set_style_text_align(grid_inst_edit_btn, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(grid_inst_edit_btn);
    lv_obj_add_flag(grid_inst_edit_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(grid_inst_edit_btn, grid_pad_inst_popup_cb, LV_EVENT_CLICKED, NULL);

    pad_inst_refresh_controls();

    // [5,3] SD CARD
    b = create_ctrl_btn(scr_live, COL_X(5), ROW_Y(3), CW, CH,
                         LV_SYMBOL_DRIVE "\nSD", RED808_CYAN, &lv_font_montserrat_18);
    lv_obj_add_event_cb(b, grid_nav_cb, LV_EVENT_CLICKED, (void*)(intptr_t)9);
    live_home_panels[live_home_panel_count++] = b;

    // [6,3] THEME
    b = create_ctrl_btn(scr_live, COL_X(6), ROW_Y(3), CW, CH,
                         "THEME\n" LV_SYMBOL_RIGHT, RED808_ACCENT2, &lv_font_montserrat_18);
    lv_obj_add_event_cb(b, grid_theme_cb, LV_EVENT_CLICKED, NULL);
    live_home_panels[live_home_panel_count++] = b;

    // [7,3] BPM control
    lv_obj_t* bpm_panel = lv_obj_create(scr_live);
    lv_obj_set_size(bpm_panel, CW, CH);
    lv_obj_set_pos(bpm_panel, COL_X(7), ROW_Y(3));
    lv_obj_set_style_radius(bpm_panel, 14, 0);
    lv_obj_set_style_bg_color(bpm_panel, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_color(bpm_panel, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_dir(bpm_panel, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(bpm_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(bpm_panel, 2, 0);
    lv_obj_set_style_border_color(bpm_panel, RED808_BORDER, 0);
    lv_obj_set_style_pad_all(bpm_panel, 6, 0);
    lv_obj_clear_flag(bpm_panel, LV_OBJ_FLAG_SCROLLABLE);
    live_home_panels[live_home_panel_count++] = bpm_panel;

    lv_obj_t* bpm_title = lv_label_create(bpm_panel);
    lv_label_set_text(bpm_title, "BPM");
    lv_obj_set_style_text_font(bpm_title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(bpm_title, RED808_TEXT_DIM, 0);
    lv_obj_align(bpm_title, LV_ALIGN_TOP_MID, 0, 2);

    lv_obj_t* bpm_minus = lv_btn_create(bpm_panel);
    lv_obj_set_size(bpm_minus, 50, 40);
    lv_obj_set_pos(bpm_minus, 4, CH - 54);
    apply_control_button_style(bpm_minus, RED808_WARNING, false, 10);
    lv_obj_set_style_transform_zoom(bpm_minus, 230, LV_STATE_PRESSED);
    lv_obj_t* bpm_minus_lbl = lv_label_create(bpm_minus);
    lv_label_set_text(bpm_minus_lbl, "-");
    lv_obj_set_style_text_font(bpm_minus_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(bpm_minus_lbl);
    lv_obj_add_event_cb(bpm_minus, grid_bpm_step_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    lv_obj_add_event_cb(bpm_minus, grid_bpm_step_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void*)(intptr_t)-1);

    lv_obj_t* bpm_plus = lv_btn_create(bpm_panel);
    lv_obj_set_size(bpm_plus, 50, 40);
    lv_obj_set_pos(bpm_plus, CW - 62, CH - 54);
    apply_control_button_style(bpm_plus, RED808_SUCCESS, false, 10);
    lv_obj_set_style_transform_zoom(bpm_plus, 230, LV_STATE_PRESSED);
    lv_obj_t* bpm_plus_lbl = lv_label_create(bpm_plus);
    lv_label_set_text(bpm_plus_lbl, "+");
    lv_obj_set_style_text_font(bpm_plus_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(bpm_plus_lbl);
    lv_obj_add_event_cb(bpm_plus, grid_bpm_step_cb, LV_EVENT_CLICKED, (void*)(intptr_t)1);
    lv_obj_add_event_cb(bpm_plus, grid_bpm_step_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void*)(intptr_t)1);

    grid_pat_lbl = lv_label_create(bpm_panel);
    lv_label_set_text(grid_pat_lbl, "120");
    lv_obj_set_style_text_font(grid_pat_lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(grid_pat_lbl, RED808_CYAN, 0);
    lv_obj_align(grid_pat_lbl, LV_ALIGN_CENTER, 0, -12);

    // Floating back button — shown only in FS pad modes (mode 1-5)
    s_pad_back_btn = lv_btn_create(scr_live);
    lv_obj_set_size(s_pad_back_btn, 72, 36);
    lv_obj_set_pos(s_pad_back_btn, 1024 - 8 - 72, 8);
    apply_control_button_style(s_pad_back_btn, RED808_BORDER, true, 8);
    lv_obj_set_style_bg_color(s_pad_back_btn, RED808_SURFACE, 0);
    lv_obj_set_style_bg_opa(s_pad_back_btn, LV_OPA_90, 0);
    lv_obj_t* back_lbl2 = lv_label_create(s_pad_back_btn);
    lv_label_set_text(back_lbl2, LV_SYMBOL_HOME);
    lv_obj_set_style_text_font(back_lbl2, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(back_lbl2, RED808_TEXT, 0);
    lv_obj_center(back_lbl2);
    lv_obj_add_flag(s_pad_back_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_pad_back_btn, [](lv_event_t*){ apply_pad_layout(0); }, LV_EVENT_CLICKED, NULL);

    #undef COL_X
    #undef ROW_Y
}

static void update_live_screen(void) {
    unsigned long now = millis();

    // ── MPC-style pad fade: velocity-weighted exponential decay ──
    // Each pad maps its current "brightness" to one of 8 bands (0 = idle,
    // 7 = peak hit). LVGL only re-invalidates a pad when its band actually
    // changes, keeping partial refresh cheap even when all 16 pads fire.
    static int prev_sync_step = -1;
    bool step_changed = (p4.current_step != prev_sync_step);
    if (step_changed) prev_sync_step = p4.current_step;
    static uint8_t pad_prev_band[16] = {};
    static bool pad_prev_muted_for_flash[16] = {};
    for (int i = 0; i < 16; i++) {
        if (!live_pad_btns[i]) continue;

        bool muted = p4.track_muted[i];
        bool muted_changed = (muted != pad_prev_muted_for_flash[i]);
        if (muted_changed) {
            pad_prev_muted_for_flash[i] = muted;
            pad_prev_band[i] = 0xFF;
        }

        uint8_t band = 0;
        uint8_t vel  = muted ? 0 : s_pad_flash_vel[i];
        if (muted) s_pad_flash_vel[i] = 0;
        if (vel) {
            unsigned long el = now - s_pad_flash_start_ms[i];
            if (el >= (unsigned long)FADE_MS) {
                s_pad_flash_vel[i] = 0;   // fade complete → back to idle
            } else {
                float t   = (float)el / (float)FADE_MS;           // 0..1
                float env = expf(-3.2f * t);                       // 1→~0.04
                float b   = (vel / 127.0f) * env * 7.999f;
                band = (uint8_t)b;
                if (band > 7) band = 7;
            }
        }
        // Sequencer sync floor: if this pad is active on the current step,
        // render at mid brightness so the groove is always visible.
        if (!muted && sync_pads_active && p4.is_playing && p4.steps[i][p4.current_step]) {
            if (band < 4) band = 4;
        }
        if (band == pad_prev_band[i]) continue;
        pad_prev_band[i] = band;

        lv_color_t tc = ui_track_color(i);

        // White hit-flash overlay tracks the band: harder hit = brighter flash.
        // Capped well below full opacity so the pad brightens without washing
        // out the labels. Stays white (set at creation) — NORMAL blend, no layer.
        if (live_pad_glow[i]) {
            lv_opa_t g = (muted || band == 0)
                             ? LV_OPA_TRANSP
                             : (lv_opa_t)(25 + (band * 130) / 7);  // band7 → ~155
            lv_obj_set_style_bg_opa(live_pad_glow[i], g, 0);
        }

        if (muted) {
            lv_obj_set_style_bg_color(live_pad_btns[i], RED808_SURFACE, 0);
            lv_obj_set_style_bg_grad_color(live_pad_btns[i], RED808_PANEL, 0);
            lv_obj_set_style_bg_opa(live_pad_btns[i], LV_OPA_70, 0);
            lv_obj_set_style_border_width(live_pad_btns[i], 3, 0);
            lv_obj_set_style_border_color(live_pad_btns[i], RED808_TEXT_DIM, 0);
            lv_obj_set_style_border_opa(live_pad_btns[i], LV_OPA_50, 0);
            if (live_pad_accent_strips[i]) {
                lv_obj_set_style_bg_color(live_pad_accent_strips[i], RED808_TEXT_DIM, 0);
                lv_obj_set_style_bg_opa(live_pad_accent_strips[i], LV_OPA_40, 0);
            }
            if (live_pad_labels[i])
                lv_obj_set_style_text_color(live_pad_labels[i], RED808_TEXT_DIM, 0);
            if (live_pad_num_labels[i])
                lv_obj_set_style_text_color(live_pad_num_labels[i], RED808_TEXT_DIM, 0);
            continue;
        }

        if (band == 0) {
            // Idle: dark surface + colored ring
            lv_obj_set_style_bg_color(live_pad_btns[i], RED808_SURFACE, 0);
            lv_obj_set_style_bg_grad_color(live_pad_btns[i], RED808_PANEL, 0);
            lv_obj_set_style_bg_opa(live_pad_btns[i], LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(live_pad_btns[i], 3, 0);
            lv_obj_set_style_border_color(live_pad_btns[i], tc, 0);
            lv_obj_set_style_border_opa(live_pad_btns[i], LV_OPA_COVER, 0);
            if (live_pad_accent_strips[i])
                lv_obj_set_style_bg_opa(live_pad_accent_strips[i], LV_OPA_70, 0);
            if (live_pad_labels[i])
                lv_obj_set_style_text_color(live_pad_labels[i], ui_track_label_color(i, false), 0);
            if (live_pad_num_labels[i])
                lv_obj_set_style_text_color(live_pad_num_labels[i], RED808_TEXT_DIM, 0);
        } else {
            // Lit: SOLID OPAQUE colour lerped surface→track by band. The old
            // path used a gradient + partial opacity (per-pixel interpolation +
            // alpha blend over the screen), which ×16 pads repainting while
            // hammering was the real FPS killer — not the glow. A flat opaque
            // fill is the cheapest draw op there is.
            uint8_t mix = (uint8_t)(36 + (band * 219) / 7);  // band1→~67, band7→255 (full tc)
            lv_obj_set_style_bg_color(live_pad_btns[i], lv_color_mix(tc, RED808_SURFACE, mix), 0);
            lv_obj_set_style_bg_opa(live_pad_btns[i], LV_OPA_COVER, 0);
            // Border widens on hard hits (band 6-7) and stays colored for soft
            lv_coord_t bw = (band >= 6) ? 4 : 3;
            lv_obj_set_style_border_width(live_pad_btns[i], bw, 0);
            lv_color_t bc = (band >= 6) ? lv_color_white() : tc;
            lv_obj_set_style_border_color(live_pad_btns[i], bc, 0);
            lv_obj_set_style_border_opa(live_pad_btns[i], LV_OPA_COVER, 0);
            if (live_pad_labels[i])
                lv_obj_set_style_text_color(live_pad_labels[i], ui_track_label_color(i, band >= 5), 0);
            if (live_pad_num_labels[i])
                lv_obj_set_style_text_color(live_pad_num_labels[i],
                    (band >= 5 && ui_track_color_is_light(i)) ? RED808_BG : (band >= 5 ? lv_color_white() : RED808_TEXT), 0);
            if (live_pad_accent_strips[i])
                lv_obj_set_style_bg_opa(live_pad_accent_strips[i], band >= 5 ? LV_OPA_COVER : LV_OPA_80, 0);
        }
    }

    // Per-pad performance state: keep mute/solo/current-step readable on the
    // pad itself so the live surface works as an instrument, not just a grid.
    static bool prev_muted[16] = {};
    static bool prev_solo[16] = {};
    static bool prev_step_lit[16] = {};
    static bool prev_live_playing = false;
    for (int i = 0; i < 16; i++) {
        if (!live_pad_btns[i]) continue;
        bool muted = p4.track_muted[i];
        bool solo = p4.track_solo[i];
        bool step_lit = !muted && p4.is_playing && p4.steps[i][p4.current_step];
        if (muted == prev_muted[i] && solo == prev_solo[i] &&
            step_lit == prev_step_lit[i] && p4.is_playing == prev_live_playing) {
            continue;
        }
        prev_muted[i] = muted;
        prev_solo[i] = solo;
        prev_step_lit[i] = step_lit;

        lv_color_t tc = ui_track_color(i);
        if (live_pad_state_labels[i]) {
            if (solo) {
                lv_label_set_text(live_pad_state_labels[i], "SOLO");
                lv_obj_set_style_text_color(live_pad_state_labels[i], RED808_WARNING, 0);
            } else if (muted) {
                lv_label_set_text(live_pad_state_labels[i], "MUTE");
                lv_obj_set_style_text_color(live_pad_state_labels[i], RED808_ERROR, 0);
            } else {
                lv_label_set_text(live_pad_state_labels[i], "");
            }
        }
        lv_obj_set_style_border_color(live_pad_btns[i], muted ? RED808_TEXT_DIM : (solo ? RED808_WARNING : tc), 0);
        lv_obj_set_style_border_opa(live_pad_btns[i], muted ? LV_OPA_50 : LV_OPA_COVER, 0);
        if (live_pad_labels[i] && muted) lv_obj_set_style_text_color(live_pad_labels[i], RED808_TEXT_DIM, 0);
        if (live_pad_accent_strips[i]) {
            lv_obj_set_style_bg_color(live_pad_accent_strips[i], solo ? RED808_WARNING : (muted ? RED808_TEXT_DIM : tc), 0);
            lv_obj_set_style_bg_opa(live_pad_accent_strips[i], muted ? LV_OPA_40 : LV_OPA_80, 0);
        }
    }
    prev_live_playing = p4.is_playing;

    // Play button state
    static bool gp_prev_play = false;
    if (grid_play_btn && grid_play_lbl && p4.is_playing != gp_prev_play) {
        gp_prev_play = p4.is_playing;
        lv_label_set_text(grid_play_lbl, p4.is_playing
            ? LV_SYMBOL_PAUSE "\nPAUSE" : LV_SYMBOL_PLAY "\nPLAY");
        lv_obj_set_style_bg_color(grid_play_btn,
            p4.is_playing ? RED808_SUCCESS : RED808_ACCENT, 0);
        lv_obj_set_style_border_color(grid_play_btn,
            p4.is_playing ? RED808_CYAN : RED808_ACCENT2, 0);
    }

    // Top status cell: pattern + link state (MSTR/AUX)
    static int gp_prev_pat_top = -1;
    if (grid_bpm_lbl && p4.current_pattern != gp_prev_pat_top) {
        gp_prev_pat_top = p4.current_pattern;
        lv_label_set_text_fmt(grid_bpm_lbl, "P%02d", p4.current_pattern + 1);
    }

    static int8_t gp_prev_mstr_top = -1;
    static int8_t gp_prev_aux_top = -1;
    bool mstr_on = ui_master_link_display_on();
    bool aux_on  = p4.s3_connected;
    if (grid_home_vol_lbl && ((int8_t)mstr_on != gp_prev_mstr_top || (int8_t)aux_on != gp_prev_aux_top)) {
        gp_prev_mstr_top = (int8_t)mstr_on;
        gp_prev_aux_top = (int8_t)aux_on;
        lv_label_set_text_fmt(grid_home_vol_lbl, "MSTR %s  AUX %s",
                              mstr_on ? "OK" : "--", aux_on ? "OK" : "--");
        lv_obj_set_style_text_color(grid_home_vol_lbl,
            (mstr_on && aux_on) ? RED808_SUCCESS : (mstr_on || aux_on) ? RED808_CYAN : RED808_TEXT_DIM, 0);

        // Subtle, theme-matched background wash on the STATUS cell: green when
        // the Master link is up, orange when it's down.
        if (grid_home_status_cell) {
            lv_color_t sc = mstr_on ? RED808_SUCCESS : RED808_WARNING;
            lv_obj_set_style_bg_color(grid_home_status_cell, sc, 0);
            lv_obj_set_style_bg_grad_color(grid_home_status_cell, RED808_SURFACE, 0);
            lv_obj_set_style_bg_grad_dir(grid_home_status_cell, LV_GRAD_DIR_VER, 0);
            lv_obj_set_style_bg_opa(grid_home_status_cell, LV_OPA_30, 0);
            lv_obj_set_style_border_color(grid_home_status_cell, sc, 0);
            lv_obj_set_style_border_opa(grid_home_status_cell, LV_OPA_60, 0);
            lv_obj_set_style_border_width(grid_home_status_cell, 2, 0);
        }
    }

    // Dedicated HOME controls labels
    static int gp_prev_home_master = -1;
    static lv_obj_t* gp_prev_home_master_lbl = NULL;
    if (grid_vol_lbl && (p4.master_volume != gp_prev_home_master || grid_vol_lbl != gp_prev_home_master_lbl)) {
        gp_prev_home_master = p4.master_volume;
        gp_prev_home_master_lbl = grid_vol_lbl;
        lv_label_set_text_fmt(grid_vol_lbl, "%d", p4.master_volume);
        lv_obj_clear_flag(grid_vol_lbl, LV_OBJ_FLAG_HIDDEN);
    }

    static int gp_prev_home_bpm = -1;
    static lv_obj_t* gp_prev_home_bpm_lbl = NULL;
    if (grid_pat_lbl && (p4.bpm_int != gp_prev_home_bpm || grid_pat_lbl != gp_prev_home_bpm_lbl)) {
        gp_prev_home_bpm = p4.bpm_int;
        gp_prev_home_bpm_lbl = grid_pat_lbl;
        lv_label_set_text_fmt(grid_pat_lbl, "%d", p4.bpm_int);
        lv_obj_clear_flag(grid_pat_lbl, LV_OBJ_FLAG_HIDDEN);
    }

    // Step — only show the running step while playing; show "--" when paused
    // or disconnected so the counter doesn't appear to run on the home screen.
    static int gp_prev_step = -2;   // -2 = never set, -1 = currently showing "--"
    if (grid_step_lbl) {
        if (!p4.is_playing) {
            if (gp_prev_step != -1) {
                gp_prev_step = -1;
                lv_label_set_text(grid_step_lbl, "--");
            }
        } else if (p4.current_step != gp_prev_step) {
            gp_prev_step = p4.current_step;
            lv_label_set_text_fmt(grid_step_lbl, "%02d", p4.current_step + 1);
        }
    }

    static int live_prev_step_dot = -2;
    if (live_prev_step_dot != gp_prev_step) {
        live_prev_step_dot = gp_prev_step;
        for (int i = 0; i < 16; i++) {
            if (!grid_step_dots[i]) continue;
            bool active_dot = p4.is_playing && i == p4.current_step;
            bool has_hits = p4.is_playing;
            if (has_hits) {
                has_hits = false;
                for (int track = 0; track < 16; track++) {
                    if (p4.steps[track][i]) { has_hits = true; break; }
                }
            }
            lv_obj_set_style_bg_color(grid_step_dots[i], active_dot ? RED808_WARNING : (has_hits ? RED808_CYAN : RED808_BORDER), 0);
            lv_obj_set_style_bg_opa(grid_step_dots[i], active_dot ? LV_OPA_COVER : (has_hits ? LV_OPA_70 : LV_OPA_40), 0);
        }
    }

    // 16 Levels source pad label tracking — keeps the right-side button in
    // sync with whichever pad the player last tapped outside of 16L mode.
    static uint8_t prev_16l_src = 255;
    if (grid_16l_lbl && s_16l_active) {
        uint8_t cur = s_16l_src_pad;
        if (cur != prev_16l_src) {
            prev_16l_src = cur;
            lv_label_set_text_fmt(grid_16l_lbl, "16 LVL\nSRC %d", cur + 1);
        }
    } else {
        prev_16l_src = 255;
    }

    // Tremolo hold: neon ring responds to X(rate) and Y(amplitude), no marker dot.
    static bool prev_trem_visible[16] = {};
    static uint8_t prev_trem_x[16] = {};
    static uint8_t prev_trem_y[16] = {};
    static uint8_t prev_trem_phase[16] = {};
    for (int i = 0; i < 16; i++) {
        if (!live_pad_btns[i]) continue;
        bool visible = s_pad_held[i] && !p4.track_muted[i] &&
                       !lv_obj_has_flag(live_pad_btns[i], LV_OBJ_FLAG_HIDDEN);
        uint8_t x = s_pad_hold_x[i];
        uint8_t y = s_pad_hold_y[i];
        uint8_t phase = s_pad_roll_phase[i] & 0x07;
        if (!visible) {
            if (prev_trem_visible[i]) {
                lv_obj_set_style_outline_width(live_pad_btns[i], 0, 0);
                lv_obj_set_style_shadow_width(live_pad_btns[i], 0, 0);
                lv_obj_set_style_shadow_opa(live_pad_btns[i], LV_OPA_0, 0);
                pad_prev_band[i] = 0xFF;
            }
            prev_trem_visible[i] = false;
            continue;
        }

        if (visible == prev_trem_visible[i] && x == prev_trem_x[i] &&
            y == prev_trem_y[i] && phase == prev_trem_phase[i]) {
            continue;
        }
        prev_trem_visible[i] = true;
        prev_trem_x[i] = x;
        prev_trem_y[i] = y;
        prev_trem_phase[i] = phase;

        uint8_t amp = (uint8_t)(127U - (y > 127 ? 127 : y));
        uint8_t speed = x > 127 ? 127 : x;
        uint8_t pulse = (phase <= 4) ? (uint8_t)(phase * 16U) : (uint8_t)((8U - phase) * 16U);
        lv_color_t neon = lv_color_hex(ui_tremolo_neon_hex(speed, y));
        lv_opa_t fill_opa = (lv_opa_t)(78 + ((uint16_t)amp * 112U) / 127U + pulse / 3U);
        if (fill_opa > LV_OPA_COVER) fill_opa = LV_OPA_COVER;
        lv_coord_t border_w = (lv_coord_t)(4 + ((uint16_t)speed * 3U) / 127U);
        lv_coord_t outline_w = (lv_coord_t)(2 + ((uint16_t)amp * 4U) / 127U);
        lv_coord_t shadow_w = (lv_coord_t)(10 + ((uint16_t)amp * 18U) / 127U + ((uint16_t)speed * 10U) / 127U);
        lv_opa_t glow_opa = (lv_opa_t)(70 + ((uint16_t)amp * 145U) / 127U + pulse / 2U);
        if (glow_opa > LV_OPA_COVER) glow_opa = LV_OPA_COVER;

        lv_obj_set_style_bg_color(live_pad_btns[i], neon, 0);
        lv_obj_set_style_bg_grad_color(live_pad_btns[i], RED808_SURFACE, 0);
        lv_obj_set_style_bg_opa(live_pad_btns[i], fill_opa, 0);
        lv_obj_set_style_border_width(live_pad_btns[i], border_w, 0);
        lv_obj_set_style_border_color(live_pad_btns[i], neon, 0);
        lv_obj_set_style_border_opa(live_pad_btns[i], LV_OPA_COVER, 0);
        lv_obj_set_style_outline_width(live_pad_btns[i], outline_w, 0);
        lv_obj_set_style_outline_pad(live_pad_btns[i], 1, 0);
        lv_obj_set_style_outline_color(live_pad_btns[i], neon, 0);
        lv_obj_set_style_outline_opa(live_pad_btns[i], glow_opa, 0);
        lv_obj_set_style_shadow_width(live_pad_btns[i], shadow_w, 0);
        lv_obj_set_style_shadow_color(live_pad_btns[i], neon, 0);
        lv_obj_set_style_shadow_opa(live_pad_btns[i], glow_opa, 0);
        if (live_pad_labels[i])
            lv_obj_set_style_text_color(live_pad_labels[i], ui_track_color_is_light(i) ? RED808_BG : lv_color_white(), 0);
        if (live_pad_num_labels[i])
            lv_obj_set_style_text_color(live_pad_num_labels[i], ui_track_color_is_light(i) ? RED808_BG : RED808_TEXT, 0);
        if (live_pad_accent_strips[i]) {
            lv_obj_set_style_bg_color(live_pad_accent_strips[i], neon, 0);
            lv_obj_set_style_bg_opa(live_pad_accent_strips[i], LV_OPA_COVER, 0);
        }
    }

    // Spectrum bars — read from DSP task
    {
        SpectrumData sp;
        dsp_get_spectrum(&sp);
        static uint8_t prev_bar_h[16] = {};
        const int MAX_BAR_H = 60;  // max bar height in pixels (pad is 143px)
        for (int i = 0; i < 16; i++) {
            if (!live_spectrum_bars[i]) continue;
            uint8_t h_px = (uint8_t)((sp.bars[i] * MAX_BAR_H) / 255);
            if (h_px == prev_bar_h[i]) continue;
            prev_bar_h[i] = h_px;
            lv_obj_set_height(live_spectrum_bars[i], h_px);
            lv_obj_align(live_spectrum_bars[i], LV_ALIGN_BOTTOM_MID, 0, -4);
            lv_obj_set_style_bg_opa(live_spectrum_bars[i],
                h_px > 0 ? LV_OPA_60 : LV_OPA_0, 0);
        }
    }

    // Ripple animation
    ripple_update();
}

// =============================================================================
// FX LAB SCREEN — dynamic grid with 12 available cards and view modes 3/6/9/12
// =============================================================================
enum FxCardKind : uint8_t {
    FX_CARD_FLANGE = 0,
    FX_CARD_DELAY,
    FX_CARD_REVERB,
    FX_CARD_FOLD,
    FX_CARD_CRUSH,
    FX_CARD_PHASER,
    FX_CARD_CUTOFF,
    FX_CARD_RESO,
    FX_CARD_DRIVE,
    FX_CARD_BITS,
    FX_CARD_SRATE,
    FX_CARD_FILTER,
};

static constexpr int FX_CARD_COUNT = 12;
static constexpr int FX_VIEW_MODE_COUNT = 3;
static constexpr int FX_PAGE_DOT_COUNT = 4;
static const int fx_view_modes[FX_VIEW_MODE_COUNT] = {3, 6, 12};

static int fx_page = 0;
static int fx_view_mode = 0;

static lv_obj_t* fx_cards[FX_CARD_COUNT]       = {};
static lv_obj_t* fx_arcs[FX_CARD_COUNT]        = {};
// Conical-gradient knob faces (one descriptor per FX card; kept at file scope
// because LVGL stores a pointer to it, not a copy).
static lv_grad_dsc_t s_fx_knob_grad[FX_CARD_COUNT];
static lv_obj_t* fx_value_labels[FX_CARD_COUNT]= {};
static lv_obj_t* fx_name_labels[FX_CARD_COUNT] = {};
static lv_obj_t* fx_src_labels[FX_CARD_COUNT]  = {};
static lv_obj_t* fx_toggle_btns[FX_CARD_COUNT] = {};
static lv_obj_t* fx_pct_labels[FX_CARD_COUNT]  = {};
static lv_obj_t* fx_page_dot[FX_PAGE_DOT_COUNT]= {};
static lv_obj_t* fx_page_lbl                   = NULL;
static lv_obj_t* fx_view_btn                   = NULL;
static lv_obj_t* fx_view_lbl                   = NULL;
static bool s_fx_ui_syncing = false;
static uint32_t s_fx_toggle_last_ms[FX_CARD_COUNT] = {};
static uint32_t s_fx_any_toggle_last_ms = 0;          // global across all FX buttons
static float    s_fx_arc_anim[FX_CARD_COUNT] = {};    // file-scope for snap access
static uint32_t s_fx_arc_user_ms[FX_CARD_COUNT] = {}; // last user-touch timestamp
static uint8_t  s_fx_last_active_u7[FX_CARD_COUNT] = {64, 64, 64, 64, 64, 64, 96, 64, 64, 64, 64, 64};

static const char* fx_names[FX_CARD_COUNT] = {
    "FLANGE", "DELAY", "REVERB", "FOLD", "CRUSH", "PHASER",
    "CUTOFF", "RESO", "DRIVE", "BITS", "SRATE", "FILTER"
};

static const uint32_t fx_colors[FX_CARD_COUNT] = {
    0xC9271B, 0xE86820, 0xF5BC31, 0xF2552F, 0xFF8C2A, 0xF7EAD7,
    0x27B0D0, 0x31D2A1, 0xF2466B, 0xD18A2B, 0x4CA8FF, 0xA17BFF
};

static const char* fx_src[FX_CARD_COUNT] = {
    "ENC 1", "ENC 2", "ENC 3", "MACRO", "MACRO", "MACRO",
    "FILTER", "FILTER", "MASTER", "CRUSH", "CRUSH", "FILTER"
};

static lv_color_t fx_safe_text_color(uint32_t hexColor) {
    uint8_t r = (uint8_t)((hexColor >> 16) & 0xFF);
    uint8_t g = (uint8_t)((hexColor >> 8) & 0xFF);
    uint8_t b = (uint8_t)(hexColor & 0xFF);
    int luminance = (int)(0.299f * (float)r + 0.587f * (float)g + 0.114f * (float)b);
    return (luminance > 190) ? RED808_TEXT : lv_color_hex(hexColor);
}

static bool fx_card_has_onoff(int cell) {
    return cell >= 0 && cell < FX_CARD_COUNT;
}

static int fx_card_current_value_u7(int cell) {
    switch (cell) {
        case FX_CARD_FLANGE: return p4.enc_value[0];
        case FX_CARD_DELAY:  return p4.enc_value[1];
        case FX_CARD_REVERB: return p4.enc_value[2];
        case FX_CARD_FOLD:   return p4.pot_value[3];
        case FX_CARD_CRUSH: {
            float norm = (float)(16 - constrain(p4.bitcrush_bits, 8, 16)) / 8.0f;
            return constrain((int)(norm * 127.0f + 0.5f), 0, 127);
        }
        case FX_CARD_PHASER: return p4.pot_value[2];
        case FX_CARD_CUTOFF: {
            float norm = (float)(constrain(p4.cutoff_hz, 20, 20000) - 20) / 19980.0f;
            return constrain((int)(norm * 127.0f + 0.5f), 0, 127);
        }
        case FX_CARD_RESO: {
            float norm = (float)(constrain(p4.resonance_x10, 10, 100) - 10) / 90.0f;
            return constrain((int)(norm * 127.0f + 0.5f), 0, 127);
        }
        case FX_CARD_DRIVE: {
            float norm = (float)constrain(p4.distortion_pct, 0, 100) / 100.0f;
            return constrain((int)(norm * 127.0f + 0.5f), 0, 127);
        }
        case FX_CARD_BITS: {
            // Keep UI mapping aligned with CRUSH macro pot#1 command path.
            float norm = (float)(16 - constrain(p4.bitcrush_bits, 8, 16)) / 8.0f;
            return constrain((int)(norm * 127.0f + 0.5f), 0, 127);
        }
        case FX_CARD_SRATE: {
            // Keep UI mapping aligned with CRUSH macro pot#1 command path.
            float norm = (float)(32000 - constrain(p4.sample_rate_hz, 9000, 32000)) / 22000.0f;
            return constrain((int)(norm * 127.0f + 0.5f), 0, 127);
        }
        case FX_CARD_FILTER: {
            float norm = (float)constrain(p4.filter_type, 0, 4) / 4.0f;
            return constrain((int)(norm * 127.0f + 0.5f), 0, 127);
        }
    }
    return 0;
}

static bool fx_card_is_muted(int cell) {
    switch (cell) {
        case FX_CARD_FLANGE: return p4.enc_muted[0];
        case FX_CARD_DELAY:  return p4.enc_muted[1];
        case FX_CARD_REVERB: return p4.enc_muted[2];
        case FX_CARD_FOLD:   return p4.pot_muted[0];
        case FX_CARD_CRUSH:  return p4.bitcrush_bits >= 16;
        case FX_CARD_PHASER: return p4.pot_muted[2];
        case FX_CARD_CUTOFF: return p4.cutoff_hz >= 19950;
        case FX_CARD_RESO:   return p4.resonance_x10 <= 11;
        case FX_CARD_DRIVE:  return p4.distortion_pct <= 0;
        case FX_CARD_BITS:   return p4.bitcrush_bits >= 16;
        case FX_CARD_SRATE:  return (p4.sample_rate_hz <= 0) || (p4.sample_rate_hz >= 31800);
        case FX_CARD_FILTER: return p4.filter_type == 0;
    }
    return false;
}

static const char* fx_card_button_text(int cell, bool muted) {
    if (fx_card_has_onoff(cell)) return muted ? "OFF" : "ON";
    return muted ? "OFF" : "ON";
}

static int fx_card_neutral_u7(int cell) {
    switch (cell) {
        case FX_CARD_CRUSH:  return 0;
        case FX_CARD_CUTOFF: return 127;
        case FX_CARD_RESO:   return 0;
        case FX_CARD_DRIVE:  return 0;
        case FX_CARD_BITS:   return 0;
        case FX_CARD_SRATE:  return 0;
        case FX_CARD_FILTER: return 0;
        default:             return 0;
    }
}

static void fx_card_send_value(int cell, int u7) {
    int neutral_u7 = fx_card_neutral_u7(cell);
    if (u7 != neutral_u7) {
        s_fx_last_active_u7[cell] = (uint8_t)u7;
    }

    switch (cell) {
        case FX_CARD_FLANGE:
        case FX_CARD_DELAY:
        case FX_CARD_REVERB:
            p4.enc_value[cell] = (uint8_t)u7;
            if (udp_wifi_connected()) udp_send_fx_enc(cell, p4.enc_value[cell], p4.enc_muted[cell]);
            break;
        case FX_CARD_FOLD:
            p4.pot_value[3] = (uint8_t)u7;
            if (udp_wifi_connected()) udp_send_fx_pot(0, p4.pot_value[3], p4.pot_muted[0]);
            break;
        case FX_CARD_CRUSH:
            p4.pot_value[1] = (uint8_t)u7;
            p4.bitcrush_bits = constrain((int)(16.0f - ((float)u7 / 127.0f) * 8.0f + 0.5f), 8, 16);
            if (udp_wifi_connected()) udp_send_set_bitcrush(p4.bitcrush_bits);
            break;
        case FX_CARD_PHASER:
            p4.pot_value[2] = (uint8_t)u7;
            if (udp_wifi_connected()) udp_send_fx_pot(2, p4.pot_value[2], p4.pot_muted[2]);
            break;
        case FX_CARD_CUTOFF: {
            int hz = constrain((int)(20.0f + ((float)u7 / 127.0f) * 19980.0f + 0.5f), 20, 20000);
            p4.cutoff_hz = hz;
            if (udp_wifi_connected()) udp_send_set_filter_cutoff(hz);
            break;
        }
        case FX_CARD_RESO: {
            int resonanceX10 = constrain((int)(10.0f + ((float)u7 / 127.0f) * 90.0f + 0.5f), 10, 100);
            p4.resonance_x10 = resonanceX10;
            if (udp_wifi_connected()) udp_send_set_filter_resonance((float)resonanceX10 / 10.0f);
            break;
        }
        case FX_CARD_DRIVE: {
            int drive = constrain((int)(((float)u7 / 127.0f) * 100.0f + 0.5f), 0, 100);
            p4.distortion_pct = drive;
            if (udp_wifi_connected()) udp_send_set_distortion((float)drive / 100.0f);
            break;
        }
        case FX_CARD_BITS: {
            // Match udp_send_fx_pot(pot=1): 16..8 bits across u7 range.
            int bits = constrain((int)(16.0f - ((float)u7 / 127.0f) * 8.0f + 0.5f), 8, 16);
            p4.bitcrush_bits = bits;
            p4.pot_value[1] = (uint8_t)u7;
            if (udp_wifi_connected()) udp_send_set_bitcrush(bits);
            break;
        }
        case FX_CARD_SRATE: {
            // Match udp_send_fx_pot(pot=1): 32000..9000 Hz across u7 range.
            int sr = constrain((int)(32000.0f - ((float)u7 / 127.0f) * 22000.0f + 0.5f), 9000, 32000);
            p4.sample_rate_hz = sr;
            if (udp_wifi_connected()) udp_send_set_sample_rate(sr);
            break;
        }
        case FX_CARD_FILTER: {
            int type = constrain((int)((float)u7 / 127.0f * 4.0f + 0.5f), 0, 4);
            p4.filter_type = type;
            if (udp_wifi_connected()) udp_send_set_filter(type);
            break;
        }
    }
    g_fx_screen_dirty = true;
}

static void fx_card_reset(int cell) {
    switch (cell) {
        case FX_CARD_CRUSH: fx_card_send_value(cell, 0); break;
        case FX_CARD_CUTOFF: fx_card_send_value(cell, 127); break;
        case FX_CARD_RESO: fx_card_send_value(cell, 0); break;
        case FX_CARD_DRIVE: fx_card_send_value(cell, 0); break;
        case FX_CARD_BITS: fx_card_send_value(cell, 0); break;
        case FX_CARD_SRATE: fx_card_send_value(cell, 0); break;
        case FX_CARD_FILTER: fx_card_send_value(cell, 0); break;
        default: break;
    }
}

static int fx_page_count(void) {
    int perPage = fx_view_modes[constrain(fx_view_mode, 0, FX_VIEW_MODE_COUNT - 1)];
    return (FX_CARD_COUNT + perPage - 1) / perPage;
}

static void fx_apply_layout(void) {
    int perPage = fx_view_modes[constrain(fx_view_mode, 0, FX_VIEW_MODE_COUNT - 1)];
    int pageCount = fx_page_count();
    if (fx_page >= pageCount) fx_page = pageCount - 1;
    if (fx_page < 0) fx_page = 0;

    if (fx_page_lbl) lv_label_set_text_fmt(fx_page_lbl, "%d / %d", fx_page + 1, pageCount);
    if (fx_view_lbl) lv_label_set_text_fmt(fx_view_lbl, "VIEW %d", perPage);

    for (int p = 0; p < FX_PAGE_DOT_COUNT; p++) {
        if (!fx_page_dot[p]) continue;
        bool visible = p < pageCount;
        if (visible) lv_obj_clear_flag(fx_page_dot[p], LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(fx_page_dot[p], LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_opa(fx_page_dot[p], p == fx_page ? LV_OPA_COVER : LV_OPA_30, 0);
    }

    int start = fx_page * perPage;
    int visibleCount = constrain(FX_CARD_COUNT - start, 0, perPage);
    int cols = visibleCount >= 12 ? 4 : (visibleCount >= 9 ? 3 : 3);
    int rows = (visibleCount + cols - 1) / cols;
    if (rows < 1) rows = 1;

    const int topY = 52;
    const int bottomPad = 8;
    const int sidePad = 12;
    const int gap = 10;
    int gridH = LCD_V_RES - topY - bottomPad;
    int cardW = (LCD_H_RES - sidePad * 2 - gap * (cols - 1)) / cols;
    int cardH = (gridH - gap * (rows - 1)) / rows;
    bool compact12 = (visibleCount >= 12);
    bool compact6 = (!compact12 && visibleCount >= 6);
    int arcSize = compact12
        ? constrain((cardW < cardH ? cardW : cardH) - 96, 72, 130)
        : (compact6
            ? constrain((cardW < cardH ? cardW : cardH) - 84, 96, 190)
            : constrain((cardW < cardH ? cardW : cardH) - 72, 120, 290));

    const lv_font_t* titleFont = compact12 ? &lv_font_montserrat_12 : (compact6 ? &lv_font_montserrat_16 : &lv_font_montserrat_22);
    const lv_font_t* valueFont = compact12 ? &lv_font_montserrat_20 : (compact6 ? &lv_font_montserrat_28 : &lv_font_montserrat_40);
    const lv_font_t* srcFont = compact12 ? &lv_font_montserrat_10 : (compact6 ? &lv_font_montserrat_10 : &lv_font_montserrat_12);
    const lv_font_t* toggleFont = compact12 ? &lv_font_montserrat_12 : (compact6 ? &lv_font_montserrat_12 : &lv_font_montserrat_16);

    int nameY = compact12 ? 4 : (compact6 ? 8 : 14);
    int srcY = compact12 ? 20 : (compact6 ? 28 : 42);
    int centerY = compact12 ? -4 : (compact6 ? -8 : -18);
    int pctY = compact12 ? 10 : (compact6 ? 4 : -2);

    for (int cell = 0; cell < FX_CARD_COUNT; cell++) {
        if (!fx_cards[cell]) continue;
        bool visible = (cell >= start && cell < start + visibleCount);
        if (!visible) {
            lv_obj_add_flag(fx_cards[cell], LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_clear_flag(fx_cards[cell], LV_OBJ_FLAG_HIDDEN);
        int local = cell - start;
        int col = local % cols;
        int row = local / cols;
        int x = sidePad + col * (cardW + gap);
        int y = topY + row * (cardH + gap);
        lv_obj_set_pos(fx_cards[cell], x, y);
        lv_obj_set_size(fx_cards[cell], cardW, cardH);

        if (fx_name_labels[cell]) {
            lv_obj_set_width(fx_name_labels[cell], cardW);
            lv_obj_set_style_text_font(fx_name_labels[cell], titleFont, 0);
            lv_obj_align(fx_name_labels[cell], LV_ALIGN_TOP_MID, 0, nameY);
        }
        if (fx_arcs[cell]) {
            lv_obj_set_size(fx_arcs[cell], arcSize, arcSize);
            lv_obj_align(fx_arcs[cell], LV_ALIGN_CENTER, 0, centerY);
        }
        if (fx_value_labels[cell]) {
            lv_obj_set_width(fx_value_labels[cell], cardW);
            lv_obj_set_style_text_font(fx_value_labels[cell], valueFont, 0);
            lv_obj_align(fx_value_labels[cell], LV_ALIGN_CENTER, 0, centerY);
        }
        if (fx_src_labels[cell]) {
            lv_obj_set_width(fx_src_labels[cell], cardW);
            lv_obj_set_style_text_font(fx_src_labels[cell], srcFont, 0);
            lv_obj_align(fx_src_labels[cell], LV_ALIGN_TOP_MID, 0, srcY);
            if (compact12) lv_obj_add_flag(fx_src_labels[cell], LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(fx_src_labels[cell], LV_OBJ_FLAG_HIDDEN);
        }
        if (fx_toggle_btns[cell]) {
            lv_obj_set_size(fx_toggle_btns[cell], compact12 ? 72 : (compact6 ? 86 : 100), compact12 ? 24 : (compact6 ? 32 : 38));
            lv_obj_align(fx_toggle_btns[cell], LV_ALIGN_BOTTOM_MID, 0, compact12 ? -6 : (compact6 ? -10 : -14));
            lv_obj_t* lbl = lv_obj_get_child(fx_toggle_btns[cell], 0);
            if (lbl) lv_obj_set_style_text_font(lbl, toggleFont, 0);
        }
        if (fx_pct_labels[cell]) {
            lv_obj_align(fx_pct_labels[cell], LV_ALIGN_CENTER, arcSize / 4, pctY);
            if (compact12) lv_obj_add_flag(fx_pct_labels[cell], LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(fx_pct_labels[cell], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

// Callback: toggle FX mute on card click
static void fx_toggle_cb(lv_event_t* e) {
    int cell = (int)(intptr_t)lv_event_get_user_data(e);
    if (cell < 0 || cell >= FX_CARD_COUNT) return;
    uint32_t now = millis();
    // Per-button debounce (700ms) + global cross-button cooldown (200ms).
    // GT911 on P4 with LVGL can fire duplicate CLICKED events within <300ms;
    // the global guard prevents two adjacent buttons from both triggering on
    // a sloppy wide tap.
    if (now - s_fx_toggle_last_ms[cell] < 700) return;
    if (now - s_fx_any_toggle_last_ms  < 200) return;
    s_fx_toggle_last_ms[cell] = now;
    s_fx_any_toggle_last_ms   = now;
    if (fx_card_has_onoff(cell)) {
        if (cell < 3) {
            bool unmuting = p4.enc_muted[cell];
            p4.enc_muted[cell] = !p4.enc_muted[cell];
            // Delay/Reverb/Flange need a non-zero value when enabling, otherwise
            // active=false in UDP path and it looks ON but sounds OFF.
            if (unmuting && p4.enc_value[cell] == 0) {
                p4.enc_value[cell] = 48;
                s_fx_arc_anim[cell] = 48.0f;
            }
            if (udp_wifi_connected()) udp_send_fx_enc(cell, p4.enc_value[cell], p4.enc_muted[cell]);
        } else {
            if (cell < 6) {
                int pot_idx = cell - 3;
                p4.pot_muted[pot_idx] = !p4.pot_muted[pot_idx];
                if (udp_wifi_connected()) {
                    if (pot_idx == 0) udp_send_fx_pot(0, p4.pot_value[3], p4.pot_muted[0]);
                    else if (pot_idx == 1) {
                        // CRUSH card in touch UI controls bitcrush only (no chained SRATE/DIST).
                        if (p4.pot_muted[1]) {
                            p4.bitcrush_bits = 16;
                            udp_send_set_bitcrush(16);
                        } else {
                            if (p4.bitcrush_bits >= 16) p4.bitcrush_bits = 12;
                            udp_send_set_bitcrush(p4.bitcrush_bits);
                        }
                    }
                    else udp_send_fx_pot(2, p4.pot_value[2], p4.pot_muted[2]);
                }
            } else {
                bool muted = fx_card_is_muted(cell);
                if (muted) {
                    int val = (int)s_fx_last_active_u7[cell];
                    int neutral_u7 = fx_card_neutral_u7(cell);
                    if (val == neutral_u7) {
                        val = (cell == FX_CARD_CUTOFF) ? 96 : 64;
                    }
                    fx_card_send_value(cell, val);
                    s_fx_arc_anim[cell] = (float)val;
                } else {
                    int neutral_u7 = fx_card_neutral_u7(cell);
                    fx_card_send_value(cell, neutral_u7);
                    s_fx_arc_anim[cell] = (float)neutral_u7;
                }
            }
        }
    } else {
        fx_card_reset(cell);
    }
    g_fx_screen_dirty = true;
}

static void fx_arc_cb(lv_event_t* e) {
    if (s_fx_ui_syncing) return;
    int cell = (int)(intptr_t)lv_event_get_user_data(e);
    if (cell < 0 || cell >= FX_CARD_COUNT) return;
    lv_obj_t* arc = (lv_obj_t*)lv_event_get_target(e);
    int val = lv_arc_get_value(arc);
    // Snap the lerp animation immediately to avoid the animation overwriting the
    // value the user just set (e.g. set 50, sees 22 because lerp was still at 0).
    s_fx_arc_anim[cell] = (float)val;
    s_fx_arc_user_ms[cell] = millis();   // own this arc for 800ms
    fx_card_send_value(cell, val);
}

static void fx_page_cb(lv_event_t* e) {
    int dir = (int)(intptr_t)lv_event_get_user_data(e);
    int pages = fx_page_count();
    fx_page = (fx_page + dir + pages) % pages;
    fx_apply_layout();
}

static void fx_view_cb(lv_event_t* e) {
    LV_UNUSED(e);
    fx_view_mode = (fx_view_mode + 1) % FX_VIEW_MODE_COUNT;
    fx_page = 0;
    fx_apply_layout();
}

static void create_fx_screen(void) {
    scr_fx = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_fx, RED808_BG, 0);
    lv_obj_clear_flag(scr_fx, LV_OBJ_FLAG_SCROLLABLE);
    ui_create_header(scr_fx);

    // ── Title row ──
    lv_obj_t* title = lv_label_create(scr_fx);
    lv_label_set_text(title, LV_SYMBOL_AUDIO "  FX LAB");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(title, RED808_ACCENT, 0);
    lv_obj_set_pos(title, 60, 10);

    for (int cell = 0; cell < FX_CARD_COUNT; cell++) {
        // Card container
        lv_obj_t* card = lv_obj_create(scr_fx);
        fx_cards[cell] = card;
        lv_obj_set_size(card, 320, 260);
        lv_obj_set_pos(card, 0, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
        // Themed card background
        lv_obj_set_style_bg_color(card, RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_color(card, RED808_PANEL, 0);
        lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(card, 8, 0);
        lv_obj_set_style_border_width(card, 2, 0);
        lv_obj_set_style_border_color(card, lv_color_hex(fx_colors[cell]), 0);
        lv_obj_set_style_border_opa(card, LV_OPA_40, 0);
        // Outer neon glow
        lv_obj_set_style_outline_width(card, 4, 0);
        lv_obj_set_style_outline_color(card, lv_color_hex(fx_colors[cell]), 0);
        lv_obj_set_style_outline_opa(card, LV_OPA_20, 0);
        lv_obj_set_style_outline_pad(card, 2, 0);
        lv_obj_set_style_shadow_width(card, 0, 0);
        lv_obj_set_style_pad_all(card, 0, 0);
        lv_obj_set_style_bg_color(card, lv_color_hex(fx_colors[cell]), LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(card, (lv_opa_t)216, LV_STATE_PRESSED);
        // Card is just a visual container; toggle is on its dedicated button
        // (see ON/OFF below). Otherwise the arc drag bubbled up into a card
        // click and silently muted the FX.
        lv_obj_clear_flag(card, LV_OBJ_FLAG_CLICKABLE);

        // FX Name — top center, neon style
        fx_name_labels[cell] = lv_label_create(card);
        lv_label_set_text(fx_name_labels[cell], fx_names[cell]);
        lv_obj_set_style_text_font(fx_name_labels[cell], &lv_font_montserrat_22, 0);
        lv_obj_set_style_text_color(fx_name_labels[cell], lv_color_hex(fx_colors[cell]), 0);
        lv_obj_set_width(fx_name_labels[cell], 320);
        lv_obj_set_style_text_align(fx_name_labels[cell], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(fx_name_labels[cell], LV_ALIGN_TOP_MID, 0, 14);

        // Source tag — subtle under name
        fx_src_labels[cell] = lv_label_create(card);
        lv_label_set_text(fx_src_labels[cell], fx_src[cell]);
        lv_obj_set_style_text_font(fx_src_labels[cell], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(fx_src_labels[cell], RED808_TEXT_DIM, 0);
        lv_obj_set_width(fx_src_labels[cell], 320);
        lv_obj_set_style_text_align(fx_src_labels[cell], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(fx_src_labels[cell], LV_ALIGN_TOP_MID, 0, 42);

        // ── Conical-gradient knob face (rotary look) — sits inside the arc ──
        // Symmetric dark→colour→dark sweep (3 stops) so there's no hard seam;
        // the bright lobe peaks at the top (270° in LVGL's angle convention).
        {
            lv_grad_dsc_t& kg = s_fx_knob_grad[cell];
            lv_color_t fxc = lv_color_hex(fx_colors[cell]);
            const int KD = 152;                 // knob-face diameter
            kg.dir = LV_GRAD_DIR_CONICAL;
            kg.extend = LV_GRAD_EXTEND_PAD;
            kg.stops_count = 3;
            kg.stops[0].color = RED808_SURFACE; kg.stops[0].opa = LV_OPA_COVER; kg.stops[0].frac = 0;
            kg.stops[1].color = fxc;            kg.stops[1].opa = LV_OPA_COVER; kg.stops[1].frac = 191; // ~270°→top
            kg.stops[2].color = RED808_SURFACE; kg.stops[2].opa = LV_OPA_COVER; kg.stops[2].frac = 255;
            kg.params.conical.center.x = KD / 2;
            kg.params.conical.center.y = KD / 2;
            kg.params.conical.start_angle = 0;
            kg.params.conical.end_angle   = 3600;   // full sweep (0.1° units)

            lv_obj_t* knob_face = lv_obj_create(card);
            lv_obj_remove_style_all(knob_face);
            lv_obj_set_size(knob_face, KD, KD);
            lv_obj_align(knob_face, LV_ALIGN_CENTER, 0, -18);   // concentric with the arc
            lv_obj_set_style_radius(knob_face, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_bg_opa(knob_face, LV_OPA_COVER, 0);
            lv_obj_set_style_bg_grad(knob_face, &kg, 0);
            lv_obj_set_style_border_width(knob_face, 2, 0);
            lv_obj_set_style_border_color(knob_face, RED808_BG, 0);
            lv_obj_clear_flag(knob_face, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_clear_flag(knob_face, LV_OBJ_FLAG_CLICKABLE);
        }

        // ── BIG ARC (neon circle indicator) ──
        fx_arcs[cell] = lv_arc_create(card);
        lv_obj_set_size(fx_arcs[cell], 220, 220);
        lv_obj_align(fx_arcs[cell], LV_ALIGN_CENTER, 0, -18);
        lv_arc_set_rotation(fx_arcs[cell], 135);
        lv_arc_set_bg_angles(fx_arcs[cell], 0, 270);
        lv_arc_set_range(fx_arcs[cell], 0, 127);
        lv_arc_set_value(fx_arcs[cell], 0);
        lv_obj_add_event_cb(fx_arcs[cell], fx_arc_cb, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)cell);
        // Track (background ring) — dim, theme-aware
        lv_obj_set_style_arc_width(fx_arcs[cell], 14, LV_PART_MAIN);
        lv_obj_set_style_arc_color(fx_arcs[cell], RED808_BORDER, LV_PART_MAIN);
        lv_obj_set_style_arc_opa(fx_arcs[cell], LV_OPA_COVER, LV_PART_MAIN);
        // Indicator (filled arc) — neon glow
        lv_obj_set_style_arc_width(fx_arcs[cell], 20, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(fx_arcs[cell], lv_color_hex(fx_colors[cell]), LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(fx_arcs[cell], lv_color_white(), LV_PART_KNOB);
        lv_obj_set_style_bg_opa(fx_arcs[cell], LV_OPA_COVER, LV_PART_KNOB);
        lv_obj_set_style_pad_all(fx_arcs[cell], 5, LV_PART_KNOB);
        lv_obj_set_style_radius(fx_arcs[cell], LV_RADIUS_CIRCLE, LV_PART_KNOB);
        lv_obj_set_style_shadow_color(fx_arcs[cell], lv_color_hex(fx_colors[cell]), LV_PART_KNOB);
        lv_obj_set_style_shadow_width(fx_arcs[cell], 10, LV_PART_KNOB);
        lv_obj_set_style_shadow_opa(fx_arcs[cell], LV_OPA_50, LV_PART_KNOB);

        // Value label — center of arc (big neon number)
        fx_value_labels[cell] = lv_label_create(card);
        lv_label_set_text(fx_value_labels[cell], "000");
        lv_obj_set_style_text_font(fx_value_labels[cell], &lv_font_montserrat_40, 0);
        lv_obj_set_style_text_color(fx_value_labels[cell], RED808_TEXT, 0);
        lv_obj_set_width(fx_value_labels[cell], 320);
        lv_obj_set_style_text_align(fx_value_labels[cell], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(fx_value_labels[cell], LV_ALIGN_CENTER, 0, -18);

        // Percentage sub-label
        fx_pct_labels[cell] = lv_label_create(card);
        lv_label_set_text(fx_pct_labels[cell], "%");
        lv_obj_set_style_text_font(fx_pct_labels[cell], &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(fx_pct_labels[cell], RED808_TEXT_DIM, 0);
        lv_obj_align(fx_pct_labels[cell], LV_ALIGN_CENTER, 55, -2);

        // ── ON/OFF Toggle Button ──
        fx_toggle_btns[cell] = lv_btn_create(card);
        lv_obj_set_size(fx_toggle_btns[cell], 100, 38);
        lv_obj_align(fx_toggle_btns[cell], LV_ALIGN_BOTTOM_MID, 0, -14);
        apply_control_button_style(fx_toggle_btns[cell], lv_color_hex(fx_colors[cell]), false, 8);
        lv_obj_set_style_bg_color(fx_toggle_btns[cell], lv_color_hex(fx_colors[cell]), 0);
        lv_obj_set_style_bg_opa(fx_toggle_btns[cell], LV_OPA_20, 0);
        lv_obj_set_style_border_opa(fx_toggle_btns[cell], LV_OPA_80, 0);
        lv_obj_set_style_shadow_width(fx_toggle_btns[cell], 12, 0);
        lv_obj_set_style_shadow_color(fx_toggle_btns[cell], lv_color_hex(fx_colors[cell]), 0);
        lv_obj_set_style_shadow_opa(fx_toggle_btns[cell], LV_OPA_40, 0);
        lv_obj_add_flag(fx_toggle_btns[cell], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(fx_toggle_btns[cell], fx_toggle_cb, LV_EVENT_CLICKED, (void*)(intptr_t)cell);
        lv_obj_t* tog_lbl = lv_label_create(fx_toggle_btns[cell]);
        lv_label_set_text(tog_lbl, fx_card_button_text(cell, fx_card_is_muted(cell)));
        lv_obj_set_style_text_font(tog_lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(tog_lbl, fx_safe_text_color(fx_colors[cell]), 0);
        lv_obj_center(tog_lbl);
    }

    // Page controls
    const int page_ctrl_y = 8;
    const int page_ctrl_w = 46;
    const int page_ctrl_gap = 6;
    const int page_lbl_w = 46;
    const int page_dots_w = 48;
    const int page_group_w = page_ctrl_w * 2 + page_ctrl_gap + page_lbl_w + 12 + page_dots_w + 88;
    const int page_group_x = LCD_H_RES - 24 - page_group_w;

    lv_obj_t* prev_btn = lv_btn_create(scr_fx);
    lv_obj_set_size(prev_btn, 46, 34);
    lv_obj_set_pos(prev_btn, page_group_x, page_ctrl_y);
    apply_control_button_style(prev_btn, RED808_CYAN, false, 8);
    lv_obj_t* prev_lbl = lv_label_create(prev_btn);
    lv_label_set_text(prev_lbl, LV_SYMBOL_LEFT);
    lv_obj_center(prev_lbl);
    lv_obj_add_event_cb(prev_btn, fx_page_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);

    lv_obj_t* next_btn = lv_btn_create(scr_fx);
    lv_obj_set_size(next_btn, 46, 34);
    lv_obj_set_pos(next_btn, page_group_x + page_ctrl_w + page_ctrl_gap, page_ctrl_y);
    apply_control_button_style(next_btn, RED808_CYAN, false, 8);
    lv_obj_t* next_lbl = lv_label_create(next_btn);
    lv_label_set_text(next_lbl, LV_SYMBOL_RIGHT);
    lv_obj_center(next_lbl);
    lv_obj_add_event_cb(next_btn, fx_page_cb, LV_EVENT_CLICKED, (void*)(intptr_t)1);

    fx_page_lbl = lv_label_create(scr_fx);
    lv_label_set_text(fx_page_lbl, "1 / 4");
    lv_obj_set_style_text_font(fx_page_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(fx_page_lbl, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(fx_page_lbl, page_group_x + page_ctrl_w * 2 + page_ctrl_gap * 2, 16);

    for (int p = 0; p < FX_PAGE_DOT_COUNT; p++) {
        fx_page_dot[p] = lv_obj_create(scr_fx);
        lv_obj_set_size(fx_page_dot[p], 8, 8);
        lv_obj_set_pos(fx_page_dot[p], page_group_x + page_ctrl_w * 2 + page_ctrl_gap * 2 + page_lbl_w + 6 + p * 14, 22);
        lv_obj_set_style_radius(fx_page_dot[p], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(fx_page_dot[p], RED808_CYAN, 0);
        lv_obj_set_style_bg_opa(fx_page_dot[p], p == 0 ? LV_OPA_COVER : LV_OPA_30, 0);
        lv_obj_set_style_border_width(fx_page_dot[p], 0, 0);
        lv_obj_clear_flag(fx_page_dot[p], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(fx_page_dot[p], LV_OBJ_FLAG_CLICKABLE);
    }

    fx_view_btn = lv_btn_create(scr_fx);
    lv_obj_set_size(fx_view_btn, 80, 34);
    lv_obj_set_pos(fx_view_btn, page_group_x + page_group_w - 80, page_ctrl_y);
    apply_control_button_style(fx_view_btn, RED808_WARNING, false, 8);
    lv_obj_add_event_cb(fx_view_btn, fx_view_cb, LV_EVENT_CLICKED, NULL);
    fx_view_lbl = lv_label_create(fx_view_btn);
    lv_label_set_text(fx_view_lbl, "VIEW 3");
    lv_obj_set_style_text_font(fx_view_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(fx_view_lbl);

    fx_apply_layout();
}

static void update_fx_screen(void) {
    static uint16_t prev_key[FX_CARD_COUNT];
    static bool prev_init = false;
    if (!prev_init) {
        prev_init = true;
        for (int i = 0; i < FX_CARD_COUNT; i++) prev_key[i] = 0xFFFF;
    }

    uint32_t now = millis();

    for (int cell = 0; cell < FX_CARD_COUNT; cell++) {
        int val = fx_card_current_value_u7(cell);
        bool muted = fx_card_is_muted(cell);
        int display_val = muted ? 0 : val;

        // If the user just touched this arc, hold the lerp for 800ms so the
        // animation does NOT overwrite what they set (root cause of "set 50 → shows 22").
        bool user_owns = (now - s_fx_arc_user_ms[cell]) < 800;
        if (!user_owns) {
            s_fx_arc_anim[cell] += ((float)display_val - s_fx_arc_anim[cell]) * 0.40f;
        }
        int anim_val = (int)(s_fx_arc_anim[cell] + 0.5f);

        // Key: tracks mute + target (for expensive style ops)
        uint16_t key = (uint16_t)((muted ? 0x100 : 0) | (display_val & 0xFF));
        bool still_animating = (fabsf(s_fx_arc_anim[cell] - (float)display_val) > 0.4f);
        bool key_changed = (key != prev_key[cell]);

        if (!still_animating && !key_changed) continue;

        int pct = (int)((float)anim_val / 127.0f * 100.0f + 0.5f);

        s_fx_ui_syncing = true;
        if (fx_arcs[cell])
            lv_arc_set_value(fx_arcs[cell], anim_val);
        s_fx_ui_syncing = false;

        if (fx_value_labels[cell])
            lv_label_set_text_fmt(fx_value_labels[cell], "%d", pct);

        // Expensive style ops only when the logical key changes (not every lerp tick)
        if (key_changed) {
            prev_key[cell] = key;

            // Update card border glow intensity based on value
            lv_obj_t* card = fx_arcs[cell] ? lv_obj_get_parent(fx_arcs[cell]) : NULL;
            if (card && !muted && val > 0) {
                lv_obj_set_style_border_opa(card, LV_OPA_90, 0);
                lv_obj_set_style_outline_opa(card, LV_OPA_50, 0);
            } else if (card) {
                lv_obj_set_style_border_opa(card, muted ? LV_OPA_20 : LV_OPA_40, 0);
                lv_obj_set_style_outline_opa(card, LV_OPA_10, 0);
            }

            // Update toggle button
            if (fx_toggle_btns[cell]) {
                lv_obj_t* lbl = lv_obj_get_child(fx_toggle_btns[cell], 0);
                if (lbl) lv_label_set_text(lbl, fx_card_button_text(cell, muted));
                lv_color_t tc = fx_safe_text_color(fx_colors[cell]);
                lv_obj_set_style_bg_opa(fx_toggle_btns[cell], muted ? LV_OPA_10 : LV_OPA_20, 0);
                lv_obj_set_style_shadow_opa(fx_toggle_btns[cell], muted ? LV_OPA_0 : LV_OPA_40, 0);
                if (lbl) lv_obj_set_style_text_color(lbl, muted ? RED808_TEXT_DIM : tc, 0);
            }

            // Update arc indicator color (dim if muted)
            if (fx_arcs[cell]) {
                lv_obj_set_style_arc_opa(fx_arcs[cell], muted ? LV_OPA_20 : LV_OPA_COVER, LV_PART_INDICATOR);
            }
        }
    }
}

// =============================================================================
// =============================================================================
// SEQUENCER SCREEN — Studio-grade 16-track × 16-step grid (1024×600)
// =============================================================================
static lv_obj_t* seq_step_btns[16][16]  = {};
static lv_obj_t* seq_track_labels[16]   = {};
static lv_obj_t* seq_mute_btns[16]      = {};
static lv_obj_t* seq_solo_btns[16]      = {};
static lv_obj_t* seq_solo_labels[16]    = {};
static lv_obj_t* seq_ruler_labels[16]   = {};  // beat/step number ruler
static lv_obj_t* seq_beat_bg[4]         = {};  // beat group shading panels
static lv_obj_t* seq_playhead_line      = NULL; // glowing vertical playhead
static lv_obj_t* seq_status_step_lbl    = NULL; // bottom "STEP 05 / 16"
static lv_obj_t* seq_status_pat_lbl     = NULL; // bottom "PATTERN 01"
static lv_obj_t* seq_status_bpm_lbl     = NULL; // bottom "BPM 120.0"
static int        seq_step_x[16]        = {};  // precomputed step column X

// ── Multi-bar pagination (populated by MEM-MIDI load with raw grid) ────────
static bool       seq_raw_grid[16][64]  = {};   // up to 4 bars of raw steps
static int        seq_raw_len           = 16;   // 16/32/48/64
static int        seq_page              = 0;    // 0..3
static bool       seq_force_refresh_cells = false; // force full cell repaint on next update_sequencer_screen
static lv_obj_t*  seq_page_btns[4]      = {};
static lv_obj_t*  seq_page_lbls[4]      = {};
static bool       seq_page_styles_dirty = false;
static bool       seq_groove_base[16][16] = {};
static bool       seq_groove_base_valid = false;
// Sequencer-local header buttons
static lv_obj_t*  seq_hdr_play_btn      = NULL;
static lv_obj_t*  seq_hdr_play_lbl      = NULL;
static lv_obj_t*  seq_hdr_pat_lbl       = NULL;
static lv_obj_t*  seq_ctrl_lbl          = NULL;
static int        seq_ctrl_swing        = 0;
static int        seq_ctrl_drive        = 0;
static lv_obj_t*  seq_pattern_modal     = NULL;
static lv_obj_t*  seq_pattern_modal_lbl = NULL;
static lv_obj_t*  seq_pattern_modal_spin = NULL;
static int        seq_pattern_wait_pat  = -1;
static uint32_t   seq_pattern_wait_ms   = 0;
static bool       seq_pattern_waiting   = false;
static uint32_t   seq_pattern_payload_revision = 0;
static uint32_t   seq_pattern_wait_base_revision = 0;

// Last-loaded MIDI info — kept so the info button in the sequencer header
// can re-open the summary modal on demand.
static char   seq_last_midi_name[64]   = "";
static int    seq_last_midi_slot       = 0;   // 1-based pattern slot
static int    seq_last_midi_steps      = 0;   // total active hits
static int    seq_last_midi_raw_len    = 0;   // 16/32/48/64
static float  seq_last_midi_bpm        = 0.0f;
static int    seq_last_midi_tracks     = 0;
static bool   seq_last_midi_valid      = false;

static void seq_apply_page_styles(void);
static void seq_copy_page_to_p4(int page);
static void show_midi_load_summary(const char* title, int slot,
                                   int steps, int raw_len, float bpm,
                                   int tracks_used);

static void seq_pattern_modal_hide(void) {
    if (!seq_pattern_modal) return;
    lv_obj_del(seq_pattern_modal);
    seq_pattern_modal = NULL;
    seq_pattern_modal_lbl = NULL;
    seq_pattern_modal_spin = NULL;
    seq_pattern_wait_pat = -1;
    seq_pattern_wait_ms = 0;
    seq_pattern_waiting = false;
    seq_pattern_wait_base_revision = 0;
}

static void seq_pattern_modal_show(int pattern) {
    seq_pattern_modal_hide();
    if (!scr_sequencer) return;

    seq_pattern_modal = lv_obj_create(scr_sequencer);
    lv_obj_set_size(seq_pattern_modal, 330, 120);
    lv_obj_align(seq_pattern_modal, LV_ALIGN_TOP_RIGHT, -16, 48);
    lv_obj_set_style_radius(seq_pattern_modal, 14, 0);
    lv_obj_set_style_bg_color(seq_pattern_modal, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(seq_pattern_modal, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_dir(seq_pattern_modal, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(seq_pattern_modal, LV_OPA_90, 0);
    lv_obj_set_style_border_width(seq_pattern_modal, 2, 0);
    lv_obj_set_style_border_color(seq_pattern_modal, RED808_CYAN, 0);
    lv_obj_set_style_pad_all(seq_pattern_modal, 10, 0);
    lv_obj_clear_flag(seq_pattern_modal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(seq_pattern_modal, LV_OBJ_FLAG_CLICKABLE);

    seq_pattern_modal_spin = lv_spinner_create(seq_pattern_modal);
    lv_spinner_set_anim_params(seq_pattern_modal_spin, 1000, 60);
    lv_obj_set_size(seq_pattern_modal_spin, 32, 32);
    lv_obj_align(seq_pattern_modal_spin, LV_ALIGN_LEFT_MID, 6, 0);
    lv_obj_set_style_arc_color(seq_pattern_modal_spin, RED808_CYAN, LV_PART_INDICATOR);

    lv_obj_t* t = lv_label_create(seq_pattern_modal);
    lv_label_set_text_fmt(t, "CARGANDO P%02d", pattern + 1);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(t, RED808_ACCENT, 0);
    lv_obj_align(t, LV_ALIGN_TOP_LEFT, 50, 8);

    seq_pattern_modal_lbl = lv_label_create(seq_pattern_modal);
    lv_label_set_text(seq_pattern_modal_lbl, "Esperando pattern_sync...");
    lv_obj_set_style_text_font(seq_pattern_modal_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(seq_pattern_modal_lbl, RED808_TEXT_DIM, 0);
    lv_obj_align(seq_pattern_modal_lbl, LV_ALIGN_TOP_LEFT, 50, 40);

    seq_pattern_wait_pat = pattern;
    seq_pattern_wait_ms = millis();
    seq_pattern_waiting = true;
    seq_pattern_wait_base_revision = seq_pattern_payload_revision;
}

static void seq_pattern_modal_mark_loaded(void) {
    if (!seq_pattern_modal) return;
    if (seq_pattern_modal_spin) {
        lv_obj_add_flag(seq_pattern_modal_spin, LV_OBJ_FLAG_HIDDEN);
    }
    if (seq_pattern_modal_lbl) {
        lv_label_set_text(seq_pattern_modal_lbl, "Pattern cargado");
        lv_obj_set_style_text_color(seq_pattern_modal_lbl, RED808_SUCCESS, 0);
    }
    lv_obj_set_style_border_color(seq_pattern_modal, RED808_SUCCESS, 0);
    seq_pattern_waiting = false;
    seq_pattern_wait_base_revision = 0;
    seq_pattern_wait_ms = millis();
}

// Layout — landscape 1024×600 (LCD native, LVGL canvas)
static const int SEQ_RULER_Y    = 44;   // ruler starts below header
static const int SEQ_RULER_H    = 14;   // ruler height
static const int SEQ_GRID_Y     = 58;   // first track row Y
static const int SEQ_TRACK_H    = 32;   // track row height
static const int SEQ_TRACK_GAP  = 1;    // gap between rows
static const int SEQ_STRIPE_W   = 4;    // left color accent stripe
static const int SEQ_NAME_X     = 4;    // track name button X
static const int SEQ_NAME_W     = 54;   // track name button width
static const int SEQ_GRID_X     = 62;   // step grid start X
static const int SEQ_CELL_W     = 54;   // step cell width
static const int SEQ_BEAT_GAP   = 5;    // gap between beat groups (every 4 steps)
static const int SEQ_CELL_GAP   = 1;    // gap between cells within a beat
static const int SEQ_SOLO_X     = 960;  // solo button X
static const int SEQ_SOLO_W     = 32;   // solo button width
static const int SEQ_STATUS_Y   = 586;  // bottom status bar Y
static const int SEQ_STATUS_H   = 14;   // bottom status bar height

static void seq_step_cb(lv_event_t* e) {
    int data = (int)(intptr_t)lv_event_get_user_data(e);
    int track = (data >> 8) & 0xFF;
    int step  = data & 0xFF;
    if (track < 16 && step < 16) {
        bool next = !p4.steps[track][step];
        p4.steps[track][step] = next;
        seq_groove_base_valid = false;
        if (ui_use_udp_transport()) udp_send_set_step(track, step, next);
        // Push updated pattern to S3 (so S3 pad-sync sees the change)
        uart_send_pattern_to_s3(p4.current_pattern, p4.steps);
        // Mirror into raw multi-bar grid so manual edits persist across pages.
        int idx = seq_page * 16 + step;
        if (idx < 64) seq_raw_grid[track][idx] = next;
    }
}

static void seq_mute_cb(lv_event_t* e) {
    int track = (int)(intptr_t)lv_event_get_user_data(e);
    if (track < 16) {
        // Debounce per-track and global window: 5 LVGL indevs are registered
        // for multi-touch and a single tap can fire CLICKED twice from
        // neighbour slots.
        static uint32_t last_ms[16] = {};
        static uint32_t last_any_ms = 0;
        uint32_t now = millis();
        if (now - last_ms[track] < MUTE_DEBOUNCE_TRACK_MS) {
            P4_LOG_PRINTF("[MUTE] DROPPED dup t=%d dt=%lu\n", track, (unsigned long)(now - last_ms[track]));
            return;
        }
        if (now - last_any_ms < MUTE_DEBOUNCE_GLOBAL_MS) {
            P4_LOG_PRINTF("[MUTE] DROPPED global t=%d dt=%lu\n", track, (unsigned long)(now - last_any_ms));
            return;
        }
        last_ms[track] = now;
        last_any_ms = now;

        bool next = !p4.track_muted[track];
        p4.track_muted[track] = next;
        char tb[48];
        snprintf(tb, sizeof(tb), "MUTE T%d %s",
                 track + 1, next ? "ON" : "OFF");
        ui_show_toast(tb, next ? RED808_ERROR : RED808_SUCCESS);
        if (ui_use_udp_transport()) enqueue_mute_control((uint8_t)track, next);
        // Relay mute to S3 so the sequencer trigger engine honors it.
        uart_send_to_s3(MSG_TRACK, TRK_MUTE_BIT | (track & 0x0F), next ? 1 : 0);
    }
}

// Saved mute state before a solo was engaged — restored on un-solo so the
// user's prior mute selections aren't lost.
static bool seq_saved_mute[16] = {};
static bool seq_solo_engaged   = false;

static void seq_solo_cb(lv_event_t* e) {
    int track = (int)(intptr_t)lv_event_get_user_data(e);
    if (track >= 16) return;
    static uint32_t last_ms[16] = {};
    static uint32_t last_any_ms = 0;
    uint32_t nowDbg = millis();
    if (nowDbg - last_ms[track] < SOLO_DEBOUNCE_TRACK_MS) {
        P4_LOG_PRINTF("[SOLO] DROPPED dup t=%d dt=%lu\n", track, (unsigned long)(nowDbg - last_ms[track]));
        return;
    }
    if (nowDbg - last_any_ms < SOLO_DEBOUNCE_GLOBAL_MS) {
        P4_LOG_PRINTF("[SOLO] DROPPED global t=%d dt=%lu\n", track, (unsigned long)(nowDbg - last_any_ms));
        return;
    }
    last_ms[track] = nowDbg;
    last_any_ms = nowDbg;
    bool wasSolo = p4.track_solo[track];

    // Visible feedback so we can confirm the callback fires.
    char toastBuf[64];
    snprintf(toastBuf, sizeof(toastBuf), "SOLO T%d %s",
             track + 1, wasSolo ? "OFF" : "ON");
    ui_show_toast(toastBuf, wasSolo ? RED808_BORDER : RED808_ACCENT);

    if (wasSolo) {
        // Un-solo: clear solo flag and UNMUTE ALL tracks so the user can
        // hear the full pattern again with one tap. (Previous behaviour
        // restored a "saved" mute state, but the master used to auto-mute
        // tracks on engine assignment, leaving phantom mutes after solo.)
        p4.track_solo[track] = false;
        seq_solo_engaged = false;
        for (int t = 0; t < 16; t++) {
            p4.track_muted[t] = false;
            seq_saved_mute[t] = false;
            uart_send_to_s3(MSG_TRACK, TRK_MUTE_BIT | (t & 0x0F), 0);
        }
        // Single atomic UDP packet — no flicker from partial state_sync.
        if (ui_use_udp_transport()) {
            enqueue_mute_mask_control(0);
            enqueue_solo_mask_control(0);
        }
    } else {
        // Engage solo: if first time, remember current mute state.
        if (!seq_solo_engaged) {
            for (int t = 0; t < 16; t++) seq_saved_mute[t] = p4.track_muted[t];
            seq_solo_engaged = true;
        }
        // Exclusive: clear other solos, mute all non-solo tracks.
        for (int i = 0; i < 16; i++) p4.track_solo[i] = false;
        p4.track_solo[track] = true;
        uint16_t muteMask = 0;
        for (int t = 0; t < 16; t++) {
            bool shouldMute = (t != track);
            p4.track_muted[t] = shouldMute;
            if (shouldMute) muteMask |= (1u << t);
            uart_send_to_s3(MSG_TRACK, TRK_MUTE_BIT | (t & 0x0F),
                            shouldMute ? 1 : 0);
        }
        if (ui_use_udp_transport()) {
            enqueue_mute_mask_control(muteMask);
            enqueue_solo_mask_control((uint16_t)(1u << track));
        }
    }
}

// ── Pagination helpers ─────────────────────────────────────────────────────
// Copy the 16 steps of the given raw-grid page into p4.steps, stage the
// resulting bar to the Master, and sync to S3. Used when the user taps P1..P4.
static void seq_copy_page_to_p4(int page) {
    if (page < 0 || page > 3) return;
    int base = page * 16;
    int num_pages = (seq_raw_len + 15) / 16;
    if (num_pages < 1) num_pages = 1;
    if (page >= num_pages) return;
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < 16; s++)
            p4.steps[t][s] = seq_raw_grid[t][base + s];
    seq_page = page;
    seq_groove_base_valid = false;
    uart_stage_pattern_push_from_steps((uint8_t)p4.current_pattern, p4.steps);
    uart_send_pattern_to_s3(p4.current_pattern, p4.steps);
}

// Refresh page button highlighting + enable/disable based on seq_raw_len.
static void seq_apply_page_styles(void) {
    int num_pages = (seq_raw_len + 15) / 16;
    if (num_pages < 1) num_pages = 1;
    for (int p = 0; p < 4; p++) {
        if (!seq_page_btns[p]) continue;
        bool enabled = (p < num_pages);
        bool active  = enabled && (p == seq_page);
        lv_obj_set_style_bg_color(seq_page_btns[p],
            active ? RED808_ACCENT : (enabled ? RED808_SURFACE : lv_color_hex(0x080808)), 0);
        lv_obj_set_style_bg_opa(seq_page_btns[p],
            enabled ? LV_OPA_COVER : LV_OPA_30, 0);
        lv_obj_set_style_border_color(seq_page_btns[p],
            active ? RED808_CYAN : RED808_BORDER, 0);
        if (seq_page_lbls[p]) {
            lv_obj_set_style_text_color(seq_page_lbls[p],
                active ? lv_color_white() : (enabled ? RED808_TEXT : RED808_TEXT_DIM), 0);
        }
        if (enabled) lv_obj_clear_state(seq_page_btns[p], LV_STATE_DISABLED);
        else         lv_obj_add_state  (seq_page_btns[p], LV_STATE_DISABLED);
    }
}

static void seq_page_cb(lv_event_t* e) {
    int page = (int)(intptr_t)lv_event_get_user_data(e);
    int num_pages = (seq_raw_len + 15) / 16;
    if (page < 0 || page >= num_pages) return;
    if (page == seq_page) return;
    seq_copy_page_to_p4(page);
    seq_apply_page_styles();
}

static void seq_update_mpc_ctrl_label(void) {
    if (!seq_ctrl_lbl) return;
    lv_label_set_text_fmt(seq_ctrl_lbl, "SW%02d DR%02d", seq_ctrl_swing, seq_ctrl_drive);
}

static void seq_mpc_preset_cb(lv_event_t* e) {
    (void)e;
    seq_ctrl_swing = 0;
    seq_ctrl_drive = 55;
    p4.distortion_pct = seq_ctrl_drive;
    if (ui_use_udp_transport()) {
        udp_send_set_distortion((float)seq_ctrl_drive / 100.0f);
    }
    seq_update_mpc_ctrl_label();
}

static void seq_swing_delta_cb(lv_event_t* e) {
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    seq_ctrl_swing += (delta > 0) ? 1 : -1;
    if (seq_ctrl_swing < 0) seq_ctrl_swing = 0;
    if (seq_ctrl_swing > 100) seq_ctrl_swing = 100;
    seq_update_mpc_ctrl_label();
}

static void seq_drive_delta_cb(lv_event_t* e) {
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    if (delta > 0) {
        uart_send_to_s3(MSG_TOUCH_CMD, TCMD_DRIVE_UP, 0);
        seq_ctrl_drive += 12;
    } else {
        uart_send_to_s3(MSG_TOUCH_CMD, TCMD_DRIVE_DOWN, 0);
        seq_ctrl_drive -= 12;
    }
    if (seq_ctrl_drive < 0) seq_ctrl_drive = 0;
    if (seq_ctrl_drive > 100) seq_ctrl_drive = 100;
    p4.distortion_pct = seq_ctrl_drive;
    if (ui_use_udp_transport()) {
        udp_send_set_distortion((float)seq_ctrl_drive / 100.0f);
    }
    seq_update_mpc_ctrl_label();
}

// Called by MEM-MIDI loader after filling seq_raw_grid.
static void seq_install_raw_and_show_page0(int raw_len) {
    seq_raw_len = (raw_len < 16) ? 16 : (raw_len > 64 ? 64 : raw_len);
    seq_page = 0;
    seq_copy_page_to_p4(0);
    seq_apply_page_styles();
}

void ui_sequencer_sync_from_current_pattern(void) {
    seq_raw_len = 16;
    seq_page = 0;
    seq_groove_base_valid = false;
    seq_force_refresh_cells = true;
    seq_page_styles_dirty = true;
}

void ui_sequencer_load_external_pattern(const bool steps[16][64], int raw_len) {
    seq_raw_len = (raw_len < 16) ? 16 : (raw_len > 64 ? 64 : raw_len);
    seq_page = 0;
    seq_groove_base_valid = false;
    for (int t = 0; t < 16; t++) {
        for (int s = 0; s < 64; s++) {
            seq_raw_grid[t][s] = (s < seq_raw_len) ? steps[t][s] : false;
        }
        for (int s = 0; s < 16; s++) {
            p4.steps[t][s] = seq_raw_grid[t][s];
        }
    }
    seq_force_refresh_cells = true;  // force full cell repaint — prev_cell_key may be stale
    seq_page_styles_dirty = true;
    seq_pattern_payload_revision++;
}

static void create_sequencer_screen(void) {
    scr_sequencer = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_sequencer, RED808_BG, 0);
    lv_obj_clear_flag(scr_sequencer, LV_OBJ_FLAG_SCROLLABLE);
    ui_create_header(scr_sequencer);

    // ── Sequencer-local header controls: Play/Pause + Pattern -/+ ──
    {
        const int HY = 6;   // top padding
        const int HH = 36;  // button height
        int hx = 60;

        // PLAY / PAUSE
        seq_hdr_play_btn = lv_btn_create(scr_sequencer);
        lv_obj_set_size(seq_hdr_play_btn, 90, HH);
        lv_obj_set_pos(seq_hdr_play_btn, hx, HY);
        apply_control_button_style(seq_hdr_play_btn,
            p4.is_playing ? RED808_CYAN : RED808_ACCENT2, true, 8);
        lv_obj_set_style_bg_color(seq_hdr_play_btn,
            p4.is_playing ? RED808_SUCCESS : RED808_ACCENT, 0);
        lv_obj_add_event_cb(seq_hdr_play_btn, header_play_cb, LV_EVENT_CLICKED, NULL);
        seq_hdr_play_lbl = lv_label_create(seq_hdr_play_btn);
        lv_label_set_text(seq_hdr_play_lbl, p4.is_playing ? "PAUSE" : "PLAY");
        lv_obj_set_style_text_font(seq_hdr_play_lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(seq_hdr_play_lbl, lv_color_white(), 0);
        lv_obj_center(seq_hdr_play_lbl);
        hx += 90 + 6;

        // PATTERN -
        lv_obj_t* pm = lv_btn_create(scr_sequencer);
        lv_obj_set_size(pm, 44, HH);
        lv_obj_set_pos(pm, hx, HY);
        apply_control_button_style(pm, RED808_WARNING, false, 8);
        lv_obj_add_event_cb(pm, header_pattern_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
        lv_obj_t* pml = lv_label_create(pm);
        lv_label_set_text(pml, LV_SYMBOL_MINUS);
        lv_obj_set_style_text_font(pml, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(pml, RED808_TEXT, 0);
        lv_obj_center(pml);
        hx += 44 + 4;

        // PATTERN label
        seq_hdr_pat_lbl = lv_label_create(scr_sequencer);
        lv_obj_set_size(seq_hdr_pat_lbl, 70, HH);
        lv_obj_set_pos(seq_hdr_pat_lbl, hx, HY);
        lv_label_set_text_fmt(seq_hdr_pat_lbl, "P%02d", p4.current_pattern + 1);
        lv_obj_set_style_text_font(seq_hdr_pat_lbl, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(seq_hdr_pat_lbl, RED808_ACCENT, 0);
        lv_obj_set_style_text_align(seq_hdr_pat_lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_pad_top(seq_hdr_pat_lbl, 6, 0);
        hx += 70 + 4;

        // PATTERN +
        lv_obj_t* pp = lv_btn_create(scr_sequencer);
        lv_obj_set_size(pp, 44, HH);
        lv_obj_set_pos(pp, hx, HY);
        apply_control_button_style(pp, RED808_WARNING, false, 8);
        lv_obj_add_event_cb(pp, header_pattern_cb, LV_EVENT_CLICKED, (void*)(intptr_t)1);
        lv_obj_t* ppl = lv_label_create(pp);
        lv_label_set_text(ppl, LV_SYMBOL_PLUS);
        lv_obj_set_style_text_font(ppl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(ppl, RED808_TEXT, 0);
        lv_obj_center(ppl);
        hx += 44 + 12;

        // UNMUTE ALL — clears all mutes and solos in one tap.
        lv_obj_t* uall = lv_btn_create(scr_sequencer);
        lv_obj_set_size(uall, 130, HH);
        lv_obj_set_pos(uall, hx, HY);
        apply_control_button_style(uall, RED808_SUCCESS, false, 8);
        lv_obj_set_style_border_color(uall, RED808_CYAN, 0);
        lv_obj_set_style_border_width(uall, 2, 0);
        lv_obj_add_event_cb(uall, [](lv_event_t* /*e*/){
            seq_solo_engaged = false;
            for (int t = 0; t < 16; t++) {
                p4.track_muted[t]  = false;
                p4.track_solo[t]   = false;
                seq_saved_mute[t]  = false;
                uart_send_to_s3(MSG_TRACK, TRK_MUTE_BIT | (t & 0x0F), 0);
                uart_send_to_s3(MSG_TRACK, TRK_SOLO_BIT | (t & 0x0F), 0);
            }
            // Single atomic UDP packet for all 16 tracks (no flicker).
            if (ui_use_udp_transport()) {
                udp_send_mute_mask(0);
                udp_send_solo_mask(0);
            }
            ui_show_toast("Unmute ALL", RED808_SUCCESS);
        }, LV_EVENT_CLICKED, NULL);
        lv_obj_t* uall_lbl = lv_label_create(uall);
        lv_label_set_text(uall_lbl, "UNMUTE ALL");
        lv_obj_set_style_text_font(uall_lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(uall_lbl, lv_color_white(), 0);
        lv_obj_center(uall_lbl);
    }

    // ── Precompute step X positions ──
    {
        int xOff = SEQ_GRID_X;
        for (int s = 0; s < 16; s++) {
            if (s > 0 && (s % 4) == 0) xOff += SEQ_BEAT_GAP;
            seq_step_x[s] = xOff;
            xOff += SEQ_CELL_W + SEQ_CELL_GAP;
        }
    }
    const int grid_bottom = SEQ_GRID_Y + 16 * (SEQ_TRACK_H + SEQ_TRACK_GAP) - SEQ_TRACK_GAP;

    // ── Beat group shading panels (drawn first = behind everything) ──
    // 4 groups × 4 steps each, alternating subtle bg
    const uint32_t beat_shade[2] = { 0x0B0B0B, 0x0F0F0F };
    for (int b = 0; b < 4; b++) {
        int bx = seq_step_x[b * 4];
        int bw = seq_step_x[b * 4 + 3] + SEQ_CELL_W - bx + 2;
        seq_beat_bg[b] = lv_obj_create(scr_sequencer);
        lv_obj_set_pos(seq_beat_bg[b], bx - 1, SEQ_RULER_Y);
        lv_obj_set_size(seq_beat_bg[b], bw, grid_bottom - SEQ_RULER_Y + SEQ_STATUS_H);
        lv_obj_set_style_bg_color(seq_beat_bg[b], lv_color_hex(beat_shade[b & 1]), 0);
        lv_obj_set_style_bg_opa(seq_beat_bg[b], LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(seq_beat_bg[b], 0, 0);
        lv_obj_set_style_radius(seq_beat_bg[b], 0, 0);
        lv_obj_clear_flag(seq_beat_bg[b], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(seq_beat_bg[b], LV_OBJ_FLAG_CLICKABLE);
    }

    // ── Beat separator lines between groups ──
    for (int b = 1; b < 4; b++) {
        lv_obj_t* sep = lv_obj_create(scr_sequencer);
        lv_obj_set_pos(sep, seq_step_x[b * 4] - 3, SEQ_RULER_Y);
        lv_obj_set_size(sep, 1, grid_bottom - SEQ_RULER_Y);
        lv_obj_set_style_bg_color(sep, lv_color_hex(0x2A2A2A), 0);
        lv_obj_set_style_bg_opa(sep, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(sep, 0, 0);
        lv_obj_set_style_radius(sep, 0, 0);
        lv_obj_clear_flag(sep, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(sep, LV_OBJ_FLAG_CLICKABLE);
    }

    // ── Step ruler: beat numbers 1–16 ──
    for (int s = 0; s < 16; s++) {
        bool is_beat_start = (s % 4 == 0);
        seq_ruler_labels[s] = lv_label_create(scr_sequencer);
        lv_label_set_text_fmt(seq_ruler_labels[s], "%d", s + 1);
        lv_obj_set_style_text_font(seq_ruler_labels[s],
            is_beat_start ? &lv_font_montserrat_14 : &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(seq_ruler_labels[s],
            is_beat_start ? RED808_ACCENT : RED808_TEXT_DIM, 0);
        lv_obj_set_width(seq_ruler_labels[s], SEQ_CELL_W);
        lv_obj_set_pos(seq_ruler_labels[s], seq_step_x[s], SEQ_RULER_Y);
        lv_obj_set_style_text_align(seq_ruler_labels[s], LV_TEXT_ALIGN_CENTER, 0);
    }

    // ── Ruler bottom separator line ──
    {
        lv_obj_t* rl = lv_obj_create(scr_sequencer);
        lv_obj_set_pos(rl, SEQ_GRID_X - 2, SEQ_GRID_Y - 2);
        lv_obj_set_size(rl, seq_step_x[15] + SEQ_CELL_W - SEQ_GRID_X + 4, 1);
        lv_obj_set_style_bg_color(rl, lv_color_hex(0x2A2A2A), 0);
        lv_obj_set_style_bg_opa(rl, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(rl, 0, 0);
        lv_obj_set_style_radius(rl, 0, 0);
        lv_obj_clear_flag(rl, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(rl, LV_OBJ_FLAG_CLICKABLE);
    }

    // ── Track rows ──
    for (int t = 0; t < 16; t++) {
        int rowY = SEQ_GRID_Y + t * (SEQ_TRACK_H + SEQ_TRACK_GAP);
        lv_color_t tc = lv_color_hex(theme_presets[currentTheme].track_colors[t]);

        // Alternating row background for legibility
        lv_obj_t* row_bg = lv_obj_create(scr_sequencer);
        lv_obj_set_pos(row_bg, 0, rowY);
        lv_obj_set_size(row_bg, LCD_H_RES, SEQ_TRACK_H);
        lv_obj_set_style_bg_color(row_bg, lv_color_hex(t & 1 ? 0x0E0E0E : 0x0A0A0A), 0);
        lv_obj_set_style_bg_opa(row_bg, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(row_bg, 0, 0);
        lv_obj_set_style_radius(row_bg, 0, 0);
        lv_obj_clear_flag(row_bg, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(row_bg, LV_OBJ_FLAG_CLICKABLE);

        // Left color accent stripe
        lv_obj_t* stripe = lv_obj_create(scr_sequencer);
        lv_obj_set_pos(stripe, 0, rowY);
        lv_obj_set_size(stripe, SEQ_STRIPE_W, SEQ_TRACK_H);
        lv_obj_set_style_bg_color(stripe, tc, 0);
        lv_obj_set_style_bg_opa(stripe, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(stripe, 0, 0);
        lv_obj_set_style_radius(stripe, 0, 0);
        lv_obj_clear_flag(stripe, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(stripe, LV_OBJ_FLAG_CLICKABLE);

        // Track name + number button (tap = mute toggle)
        seq_mute_btns[t] = lv_btn_create(scr_sequencer);
        lv_obj_set_size(seq_mute_btns[t], SEQ_NAME_W, SEQ_TRACK_H);
        lv_obj_set_pos(seq_mute_btns[t], SEQ_NAME_X, rowY);
        apply_control_button_style(seq_mute_btns[t], tc, false, 4);
        lv_obj_set_style_bg_opa(seq_mute_btns[t], (lv_opa_t)140, 0);
        lv_obj_set_style_border_opa(seq_mute_btns[t], LV_OPA_60, 0);
        lv_obj_set_style_pad_all(seq_mute_btns[t], 0, 0);
        lv_obj_add_event_cb(seq_mute_btns[t], seq_mute_cb, LV_EVENT_CLICKED, (void*)(intptr_t)t);

        // Track label: "01\nBD" — number above, name below
        seq_track_labels[t] = lv_label_create(seq_mute_btns[t]);
        lv_label_set_text_fmt(seq_track_labels[t], "%02d\n%s", t + 1, trackNames[t]);
        lv_obj_set_style_text_font(seq_track_labels[t], &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(seq_track_labels[t], tc, 0);
        lv_obj_set_style_text_align(seq_track_labels[t], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_line_space(seq_track_labels[t], 1, 0);
        lv_obj_center(seq_track_labels[t]);

        // ── 16 step cells ──
        for (int s = 0; s < 16; s++) {
            bool act = p4.steps[t][s];
            seq_step_btns[t][s] = lv_obj_create(scr_sequencer);
            lv_obj_set_size(seq_step_btns[t][s], SEQ_CELL_W, SEQ_TRACK_H);
            lv_obj_set_pos(seq_step_btns[t][s], seq_step_x[s], rowY);
            lv_obj_set_style_radius(seq_step_btns[t][s], 4, 0);
            lv_obj_set_style_bg_color(seq_step_btns[t][s], act ? tc : RED808_SURFACE, 0);
            lv_obj_set_style_bg_opa(seq_step_btns[t][s], act ? LV_OPA_80 : LV_OPA_40, 0);
            lv_obj_set_style_border_width(seq_step_btns[t][s], 1, 0);
            lv_obj_set_style_border_color(seq_step_btns[t][s], act ? tc : lv_color_hex(0x1E1E1E), 0);
            lv_obj_set_style_shadow_width(seq_step_btns[t][s], act ? 8 : 0, 0);
            lv_obj_set_style_shadow_color(seq_step_btns[t][s], tc, 0);
            lv_obj_set_style_shadow_opa(seq_step_btns[t][s], LV_OPA_60, 0);
            lv_obj_clear_flag(seq_step_btns[t][s], LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(seq_step_btns[t][s], LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(seq_step_btns[t][s], seq_step_cb, LV_EVENT_CLICKED,
                                (void*)(intptr_t)((t << 8) | s));

            // Bottom accent line inside active cells (velocity-like feel)
            if (act) {
                lv_obj_t* accent = lv_obj_create(seq_step_btns[t][s]);
                lv_obj_set_size(accent, SEQ_CELL_W - 8, 3);
                lv_obj_align(accent, LV_ALIGN_BOTTOM_MID, 0, -3);
                lv_obj_set_style_bg_color(accent, lv_color_white(), 0);
                lv_obj_set_style_bg_opa(accent, LV_OPA_40, 0);
                lv_obj_set_style_radius(accent, 2, 0);
                lv_obj_set_style_border_width(accent, 0, 0);
                lv_obj_clear_flag(accent, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_clear_flag(accent, LV_OBJ_FLAG_CLICKABLE);
            }
        }

        // ── Solo button ──
        seq_solo_btns[t] = lv_btn_create(scr_sequencer);
        lv_obj_set_size(seq_solo_btns[t], SEQ_SOLO_W, SEQ_TRACK_H);
        lv_obj_set_pos(seq_solo_btns[t], SEQ_SOLO_X, rowY);
        apply_control_button_style(seq_solo_btns[t], tc, false, 4);
        lv_obj_set_style_pad_all(seq_solo_btns[t], 0, 0);
        lv_obj_add_event_cb(seq_solo_btns[t], seq_solo_cb, LV_EVENT_CLICKED, (void*)(intptr_t)t);
        seq_solo_labels[t] = lv_label_create(seq_solo_btns[t]);
        lv_label_set_text(seq_solo_labels[t], "S");
        lv_obj_set_style_text_font(seq_solo_labels[t], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(seq_solo_labels[t], RED808_TEXT_DIM, 0);
        lv_obj_center(seq_solo_labels[t]);
    }   // end for(t)

    // ── Glowing vertical playhead (spans all rows, created last = on top) ──
    seq_playhead_line = lv_obj_create(scr_sequencer);
    lv_obj_set_pos(seq_playhead_line, seq_step_x[0], SEQ_RULER_Y);
    lv_obj_set_size(seq_playhead_line, SEQ_CELL_W, grid_bottom - SEQ_RULER_Y);
    lv_obj_set_style_radius(seq_playhead_line, 0, 0);
    lv_obj_set_style_bg_color(seq_playhead_line, RED808_WARNING, 0);
    lv_obj_set_style_bg_opa(seq_playhead_line, LV_OPA_20, 0);
    lv_obj_set_style_border_width(seq_playhead_line, 0, 0);
    lv_obj_set_style_border_color(seq_playhead_line, RED808_WARNING, 0);
    lv_obj_set_style_shadow_width(seq_playhead_line, 28, 0);
    lv_obj_set_style_shadow_color(seq_playhead_line, RED808_WARNING, 0);
    lv_obj_set_style_shadow_opa(seq_playhead_line, LV_OPA_50, 0);
    lv_obj_clear_flag(seq_playhead_line, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(seq_playhead_line, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(seq_playhead_line, LV_OBJ_FLAG_HIDDEN);

    // ── Status bar (bottom strip) ──
    {
        lv_obj_t* bar = lv_obj_create(scr_sequencer);
        lv_obj_set_pos(bar, 0, SEQ_STATUS_Y);
        lv_obj_set_size(bar, LCD_H_RES, SEQ_STATUS_H);
        lv_obj_set_style_bg_color(bar, lv_color_hex(0x060606), 0);
        lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(bar, 0, 0);
        lv_obj_set_style_radius(bar, 0, 0);
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_CLICKABLE);

        seq_status_pat_lbl = lv_label_create(scr_sequencer);
        lv_label_set_text_fmt(seq_status_pat_lbl, "PATTERN %02d", p4.current_pattern + 1);
        lv_obj_set_style_text_font(seq_status_pat_lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(seq_status_pat_lbl, RED808_TEXT_DIM, 0);
        lv_obj_set_pos(seq_status_pat_lbl, 70, SEQ_STATUS_Y + 2);

        seq_status_step_lbl = lv_label_create(scr_sequencer);
        lv_label_set_text(seq_status_step_lbl, "STEP -- / 16");
        lv_obj_set_style_text_font(seq_status_step_lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(seq_status_step_lbl, RED808_ACCENT, 0);
        lv_obj_set_width(seq_status_step_lbl, 120);
        lv_obj_set_style_text_align(seq_status_step_lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(seq_status_step_lbl, LCD_H_RES / 2 - 60, SEQ_STATUS_Y + 2);

        lv_obj_t* bpm_lbl = lv_label_create(scr_sequencer);
        lv_label_set_text_fmt(bpm_lbl, "BPM %d.%d", p4.bpm_int, p4.bpm_frac);
        lv_obj_set_style_text_font(bpm_lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(bpm_lbl, RED808_INFO, 0);
        lv_obj_set_pos(bpm_lbl, LCD_H_RES - 90, SEQ_STATUS_Y + 2);
        seq_status_bpm_lbl = bpm_lbl;
    }

    seq_update_mpc_ctrl_label();
}   // end create_sequencer_screen

static void update_sequencer_screen(void) {
    int step = p4.current_step;
    bool playing = p4.is_playing;
    unsigned long now = millis();

    // Pattern loading modal lifecycle: waiting -> loaded or timeout.
    if (seq_pattern_modal) {
        if (seq_pattern_waiting) {
            if (seq_pattern_payload_revision != seq_pattern_wait_base_revision) {
                seq_pattern_modal_mark_loaded();
            } else if ((now - seq_pattern_wait_ms) > 3500) {
                if (seq_pattern_modal_spin) {
                    lv_obj_add_flag(seq_pattern_modal_spin, LV_OBJ_FLAG_HIDDEN);
                }
                if (seq_pattern_modal_lbl) {
                    lv_label_set_text(seq_pattern_modal_lbl, "Timeout: no llego pattern_sync");
                    lv_obj_set_style_text_color(seq_pattern_modal_lbl, RED808_ERROR, 0);
                }
                lv_obj_set_style_border_color(seq_pattern_modal, RED808_ERROR, 0);
                seq_pattern_waiting = false;
                seq_pattern_wait_ms = now;
                seq_pattern_wait_base_revision = 0;
            }
        } else if ((now - seq_pattern_wait_ms) > 700) {
            seq_pattern_modal_hide();
        }
    }

    // Recovery probe: re-request current pattern only when we changed
    // pattern locally and have not yet received a payload for it. This
    // avoids hammering the master when the new pattern is genuinely empty.
    static unsigned long last_probe_ms      = 0;
    static int           last_probe_pattern = -1;
    static uint32_t      last_probe_rev     = 0;
    if (last_probe_pattern != p4.current_pattern) {
        last_probe_pattern = p4.current_pattern;
        last_probe_rev     = seq_pattern_payload_revision;
        last_probe_ms      = now - 400;  // probe almost immediately
    }
    if (seq_pattern_payload_revision == last_probe_rev &&
        (now - last_probe_ms >= 500)) {
        last_probe_ms = now;
        udp_send_get_pattern(p4.current_pattern);
    }

    // ── Dirty tracking state (persistent across calls) ──
    // Cell visual key: bit2=active, bit1=is_cur(col), bit0=muted
    // Track key:       bit1=soloed, bit0=muted
    // 0xFF = uninitialized/force-refresh
    static uint8_t prev_cell_key[16][16];
    static uint8_t prev_trk_key[16];
    static int     prev_seq_theme = -1;
    static bool    seq_dt_init = false;
    if (!seq_dt_init) {
        seq_dt_init = true;
        memset(prev_cell_key, 0xFF, sizeof(prev_cell_key));
        memset(prev_trk_key,  0xFF, sizeof(prev_trk_key));
    }
    // Theme change invalidates all color-dependent styles
    if (currentTheme != prev_seq_theme) {
        prev_seq_theme = currentTheme;
        memset(prev_cell_key, 0xFF, sizeof(prev_cell_key));
        memset(prev_trk_key,  0xFF, sizeof(prev_trk_key));
    }
    // External pattern loaded: force full cell repaint regardless of dirty state
    if (seq_force_refresh_cells) {
        seq_force_refresh_cells = false;
        memset(prev_cell_key, 0xFF, sizeof(prev_cell_key));
        memset(prev_trk_key,  0xFF, sizeof(prev_trk_key));
    }
    if (seq_page_styles_dirty) {
        seq_page_styles_dirty = false;
        seq_apply_page_styles();
    }

    // ── Auto-advance pages on bar wrap (song mode) ──
    // When a MEM-MIDI with multiple bars is loaded (seq_raw_len > 16), we
    // walk the pages in a loop: each time the Master's step wraps from 15→0
    // we push the next bar into the Master's single pattern slot.
    static int prev_step = -1;
    if (playing && seq_raw_len > 16) {
        int num_pages = (seq_raw_len + 15) / 16;
        if (num_pages > 1 && prev_step >= 0 && step == 0 && prev_step == 15) {
            int next_page = (seq_page + 1) % num_pages;
            seq_copy_page_to_p4(next_page);
            seq_apply_page_styles();
            // p4.steps changed — force full cell refresh on next frame
            memset(prev_cell_key, 0xFF, sizeof(prev_cell_key));
        }
    }
    prev_step = step;

    // ── Move / show glowing playhead line — disabled: the full-height overlay
    // sweeps over muted rows and reads as mute flicker. Per-cell current-step
    // highlighting below remains active for unmuted tracks only.
    static int  prev_ph_step    = -2;
    static bool prev_ph_playing = false;
    if (seq_playhead_line && (step != prev_ph_step || playing != prev_ph_playing)) {
        prev_ph_step    = step;
        prev_ph_playing = playing;
        lv_obj_add_flag(seq_playhead_line, LV_OBJ_FLAG_HIDDEN);
    }

    // ── Status bar step counter — only when changed ──
    static int  prev_stp_step    = -2;
    static bool prev_stp_playing = false;
    if (seq_status_step_lbl && (step != prev_stp_step || playing != prev_stp_playing)) {
        prev_stp_step    = step;
        prev_stp_playing = playing;
        if (playing && step >= 0 && step < 16) {
            lv_label_set_text_fmt(seq_status_step_lbl, "STEP %02d / 16", step + 1);
        } else {
            lv_label_set_text(seq_status_step_lbl, "STEP -- / 16");
        }
    }
    static int prev_stp_pat = -1;
    if (seq_status_pat_lbl && p4.current_pattern != prev_stp_pat) {
        prev_stp_pat = p4.current_pattern;
        lv_label_set_text_fmt(seq_status_pat_lbl, "PATTERN %02d", p4.current_pattern + 1);
    }
    static int prev_stp_bpm_int = -1, prev_stp_bpm_frac = -1;
    if (seq_status_bpm_lbl &&
            (p4.bpm_int != prev_stp_bpm_int || p4.bpm_frac != prev_stp_bpm_frac)) {
        prev_stp_bpm_int  = p4.bpm_int;
        prev_stp_bpm_frac = p4.bpm_frac;
        lv_label_set_text_fmt(seq_status_bpm_lbl, "BPM %d.%d", p4.bpm_int, p4.bpm_frac);
    }

    // ── Sequencer header play/pause + pattern — only when changed ──
    static bool prev_hdr_playing = false;
    if (seq_hdr_play_btn && seq_hdr_play_lbl && playing != prev_hdr_playing) {
        prev_hdr_playing = playing;
        lv_label_set_text(seq_hdr_play_lbl, playing ? "PAUSE" : "PLAY");
        lv_obj_set_style_bg_color(seq_hdr_play_btn,
            playing ? RED808_SUCCESS : RED808_ACCENT, 0);
        lv_obj_set_style_border_color(seq_hdr_play_btn,
            playing ? RED808_CYAN : RED808_ACCENT2, 0);
    }
    static int prev_hdr_pat = -1;
    if (seq_hdr_pat_lbl && p4.current_pattern != prev_hdr_pat) {
        prev_hdr_pat = p4.current_pattern;
        lv_label_set_text_fmt(seq_hdr_pat_lbl, "P%02d", p4.current_pattern + 1);
    }

    // ── Per-track updates — dirty tracking ──
    // Only call lv_obj_set_style_* when visual state actually changes.
    // During playback the cursor column changes every step (16 cells update).
    // Static patterns: only toggled cells update (1 cell per tap).
    // This reduces 1280 style-calls/frame → ~2-32 calls/frame typical.
    for (int t = 0; t < 16; t++) {
        bool muted  = p4.track_muted[t];
        bool soloed = p4.track_solo[t];
        uint8_t trk_key = (uint8_t)((soloed ? 2 : 0) | (muted ? 1 : 0));
        lv_color_t tc = lv_color_hex(theme_presets[currentTheme].track_colors[t]);

        if (trk_key != prev_trk_key[t]) {
            prev_trk_key[t] = trk_key;

            if (seq_mute_btns[t]) {
                if (muted) {
                    lv_obj_set_style_bg_color(seq_mute_btns[t], RED808_ERROR, 0);
                    lv_obj_set_style_bg_opa(seq_mute_btns[t], LV_OPA_90, 0);
                    lv_obj_set_style_border_color(seq_mute_btns[t], RED808_ERROR, 0);
                } else {
                    lv_obj_set_style_bg_color(seq_mute_btns[t], RED808_SURFACE, 0);
                    lv_obj_set_style_bg_opa(seq_mute_btns[t], LV_OPA_50, 0);
                    lv_obj_set_style_border_color(seq_mute_btns[t], tc, 0);
                }
            }
            if (seq_track_labels[t]) {
                lv_obj_set_style_text_color(seq_track_labels[t],
                    muted ? lv_color_white() : tc, 0);
            }
            if (seq_solo_btns[t]) {
                lv_obj_set_style_bg_color(seq_solo_btns[t], soloed ? tc : RED808_SURFACE, 0);
                lv_obj_set_style_border_color(seq_solo_btns[t], soloed ? tc : RED808_BORDER, 0);
                lv_obj_set_style_shadow_width(seq_solo_btns[t], soloed ? 14 : 0, 0);
                lv_obj_set_style_shadow_color(seq_solo_btns[t], tc, 0);
            }
            if (seq_solo_labels[t]) {
                lv_obj_set_style_text_color(seq_solo_labels[t],
                    soloed ? lv_color_black() : RED808_TEXT_DIM, 0);
            }
            // Mute change affects all cells in this track — invalidate their keys
            for (int s = 0; s < 16; s++) prev_cell_key[t][s] = 0xFF;
        }

        // Step cells — skip if visual key unchanged
        for (int s = 0; s < 16; s++) {
            if (!seq_step_btns[t][s]) continue;
            bool active = p4.steps[t][s];
            bool is_cur = !muted && playing && (step == s);
            uint8_t cell_key = (uint8_t)((active ? 4 : 0) | (is_cur ? 2 : 0) | (muted ? 1 : 0));
            if (cell_key == prev_cell_key[t][s]) continue;
            prev_cell_key[t][s] = cell_key;

            lv_color_t bg;
            lv_opa_t opa;
            lv_color_t border;
            int shadow_w;

            if (is_cur && active) {
                bg = lv_color_white();
                opa = LV_OPA_COVER;
                border = tc;
                shadow_w = 20;
            } else if (is_cur) {
                bg = lv_color_hex(0x262626);
                opa = LV_OPA_COVER;
                border = RED808_WARNING;
                shadow_w = 0;
            } else if (active) {
                bg = tc;
                opa = muted ? LV_OPA_20 : LV_OPA_80;
                border = tc;
                shadow_w = muted ? 0 : 8;
            } else {
                bg = RED808_SURFACE;
                opa = LV_OPA_40;
                border = lv_color_hex(0x1E1E1E);
                shadow_w = 0;
            }

            lv_obj_set_style_bg_color(seq_step_btns[t][s], bg, 0);
            lv_obj_set_style_bg_opa(seq_step_btns[t][s], opa, 0);
            lv_obj_set_style_border_color(seq_step_btns[t][s], border, 0);
            lv_obj_set_style_shadow_width(seq_step_btns[t][s], shadow_w, 0);
            lv_obj_set_style_shadow_color(seq_step_btns[t][s],
                is_cur ? RED808_WARNING : tc, 0);
        }
    }
}

// =============================================================================
// MIXER SCREEN — 16 track faders in a single row
// =============================================================================
static lv_obj_t* vol_sliders[16] = {};
static lv_obj_t* vol_labels[16] = {};
static lv_obj_t* vol_name_labels[16] = {};
static lv_obj_t* vol_mute_dots[16] = {};
static lv_obj_t* vol_strip_panels[16] = {};
static lv_obj_t* vol_vu_bars[16] = {};   // per-channel VU meter (driven by dsp spectrum)
static lv_obj_t* vol_vu_caps[16] = {};   // peak-hold cap per VU meter
static lv_obj_t* mix_master_slider = NULL;
static lv_obj_t* mix_seq_slider = NULL;
static lv_obj_t* mix_live_slider = NULL;
static lv_obj_t* mix_bpm_slider = NULL;
static lv_obj_t* mix_master_lbl = NULL;
static lv_obj_t* mix_seq_lbl = NULL;
static lv_obj_t* mix_live_lbl = NULL;
static lv_obj_t* mix_bpm_lbl = NULL;

static void mix_global_slider_cb(lv_event_t* e) {
    int which = (int)(intptr_t)lv_event_get_user_data(e);
    lv_obj_t* slider = (lv_obj_t*)lv_event_get_target(e);
    int val = lv_slider_get_value(slider);
    switch (which) {
        case 0:
            p4.master_volume = val;
            udp_send_set_volume(val);
            break;
        case 1:
            p4.seq_volume = val;
            udp_send_set_seq_volume(val);
            break;
        case 2:
            p4.live_volume = val;
            udp_send_set_live_volume(val);
            break;
        case 3:
            p4.bpm_int = val;
            p4.bpm_frac = 0;
            udp_send_tempo((float)val);
            break;
        default:
            break;
    }
}

static void vol_slider_cb(lv_event_t* e) {
    int trk = (int)(intptr_t)lv_event_get_user_data(e);
    if (trk < 0 || trk >= 16) return;
    lv_obj_t* slider = (lv_obj_t*)lv_event_get_target(e);
    int val = lv_slider_get_value(slider);
    p4.track_volume[trk] = val;
    udp_send_set_track_volume(trk, val);
    uart_send_to_s3(MSG_TRACK, TRK_VOLUME | (trk & 0x0F), (uint8_t)val);
}

static void create_volumes_screen(void) {
    scr_volumes = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_volumes, RED808_BG, 0);
    lv_obj_clear_flag(scr_volumes, LV_OBJ_FLAG_SCROLLABLE);
    ui_create_header(scr_volumes);

    // Layout in actual LVGL canvas coordinates (landscape 1024×600)
    const int LW = LCD_H_RES;   // 1024 — full display width
    const int LH = LCD_V_RES;   // 600  — full display height

    // --- Header bar: prominent title + grouped MAIN / BPM controls ----------
    lv_obj_t* title = lv_label_create(scr_volumes);
    lv_label_set_text(title, LV_SYMBOL_VOLUME_MAX "  MIXER");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(title, RED808_ACCENT, 0);
    lv_obj_set_pos(title, 62, 12);

    // Global controls: MAIN + BPM only, each grouped inside its own panel so
    // the header reads as a coherent strip instead of bare floating sliders.
    const int global_y = 26;
    const int slider_w = 250;
    const int label_w  = 64;
    const int value_w  = 46;
    const int block_w  = label_w + slider_w + value_w + 16;   // ~376
    const int gap      = 30;
    const int global_x = 210;   // leaves room for the title on the left
    // Two blocks: 210 .. 210+376=586 (block1), 616 .. 992 (block2) — fits 1024.
    struct GlobalCtl { const char* name; int value; int max; lv_color_t color; lv_obj_t** slider; lv_obj_t** label; };
    GlobalCtl globals[] = {
        {"MAIN", p4.master_volume, Config::MAX_VOLUME, RED808_CYAN,   &mix_master_slider, &mix_master_lbl},
        {"BPM",  p4.bpm_int,       240,                RED808_ACCENT, &mix_bpm_slider,    &mix_bpm_lbl},
    };
    mix_seq_slider = NULL;
    mix_seq_lbl = NULL;
    mix_live_slider = NULL;
    mix_live_lbl = NULL;
    for (int i = 0; i < 2; i++) {
        int x = global_x + i * (block_w + gap);

        // Grouping panel (behind the controls)
        lv_obj_t* panel = lv_obj_create(scr_volumes);
        lv_obj_set_size(panel, block_w + 8, 56);
        lv_obj_set_pos(panel, x - 10, global_y - 8);
        lv_obj_set_style_radius(panel, 10, 0);
        lv_obj_set_style_bg_color(panel, RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_color(panel, RED808_PANEL, 0);
        lv_obj_set_style_bg_grad_dir(panel, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_opa(panel, LV_OPA_50, 0);
        lv_obj_set_style_border_width(panel, 1, 0);
        lv_obj_set_style_border_color(panel, globals[i].color, 0);
        lv_obj_set_style_border_opa(panel, LV_OPA_50, 0);
        lv_obj_set_style_pad_all(panel, 0, 0);
        lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(panel, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_t* name = lv_label_create(scr_volumes);
        lv_label_set_text(name, globals[i].name);
        lv_obj_set_style_text_font(name, &lv_font_montserrat_18, 0);
        lv_obj_set_style_text_color(name, globals[i].color, 0);
        lv_obj_set_pos(name, x, global_y + 12);

        *globals[i].label = lv_label_create(scr_volumes);
        lv_label_set_text_fmt(*globals[i].label, "%d", globals[i].value);
        lv_obj_set_style_text_font(*globals[i].label, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(*globals[i].label, RED808_TEXT, 0);
        lv_obj_set_pos(*globals[i].label, x + label_w + slider_w + 10, global_y + 8);

        *globals[i].slider = lv_slider_create(scr_volumes);
        lv_obj_set_size(*globals[i].slider, slider_w, 16);
        lv_obj_set_pos(*globals[i].slider, x + label_w, global_y + 16);
        lv_slider_set_range(*globals[i].slider, i == 1 ? 40 : 0, globals[i].max);
        lv_slider_set_value(*globals[i].slider, globals[i].value, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(*globals[i].slider, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(*globals[i].slider, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_bg_color(*globals[i].slider, globals[i].color, LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(*globals[i].slider, LV_OPA_COVER, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(*globals[i].slider, lv_color_white(), LV_PART_KNOB);
        lv_obj_set_style_bg_opa(*globals[i].slider, LV_OPA_COVER, LV_PART_KNOB);
        lv_obj_set_style_pad_all(*globals[i].slider, 9, LV_PART_KNOB);
        lv_obj_set_style_radius(*globals[i].slider, LV_RADIUS_CIRCLE, LV_PART_KNOB);
        lv_obj_add_event_cb(*globals[i].slider, mix_global_slider_cb, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)(i == 0 ? 0 : 3));
    }

    // Single row of 16 strips filling the full display width
    int margin  = 10;
    int strip_gap = 4;
    int total_w = LW - 2 * margin;
    int strip_w = (total_w - 15 * strip_gap) / 16;   // ~56px each
    int y_top   = 100;
    int y_bottom = LH - 8;
    int strip_h  = y_bottom - y_top;            // ~508px
    int name_h   = 14;
    int value_h  = 14;
    int mute_h   = 10;
    int slider_h = strip_h - name_h - value_h - mute_h - 18;

    for (int i = 0; i < 16; i++) {
        int x = margin + i * (strip_w + strip_gap);
        int cx = x + strip_w / 2;
        lv_color_t tc = lv_color_hex(theme_presets[currentTheme].track_colors[i]);

        // Strip panel background
        vol_strip_panels[i] = lv_obj_create(scr_volumes);
        lv_obj_set_size(vol_strip_panels[i], strip_w, strip_h);
        lv_obj_set_pos(vol_strip_panels[i], x, y_top);
        lv_obj_set_style_bg_color(vol_strip_panels[i], RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_color(vol_strip_panels[i], RED808_PANEL, 0);
        lv_obj_set_style_bg_grad_dir(vol_strip_panels[i], LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_opa(vol_strip_panels[i], LV_OPA_40, 0);
        lv_obj_set_style_radius(vol_strip_panels[i], 8, 0);
        lv_obj_set_style_border_width(vol_strip_panels[i], 1, 0);
        lv_obj_set_style_border_color(vol_strip_panels[i], tc, 0);
        lv_obj_set_style_border_opa(vol_strip_panels[i], LV_OPA_40, 0);
        lv_obj_clear_flag(vol_strip_panels[i], LV_OBJ_FLAG_SCROLLABLE);

        // Track name
        vol_name_labels[i] = lv_label_create(scr_volumes);
        lv_label_set_text(vol_name_labels[i], trackNames[i]);
        lv_obj_set_style_text_font(vol_name_labels[i], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(vol_name_labels[i], tc, 0);
        lv_obj_set_style_text_align(vol_name_labels[i], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(vol_name_labels[i], x, y_top + 2);
        lv_obj_set_width(vol_name_labels[i], strip_w);

        // Vertical slider (fader)
        int y_sl = y_top + name_h + 4;
        vol_sliders[i] = lv_slider_create(scr_volumes);
        lv_obj_set_size(vol_sliders[i], 8, slider_h);
        lv_obj_set_pos(vol_sliders[i], cx - 4, y_sl);
        lv_slider_set_range(vol_sliders[i], 0, Config::MAX_VOLUME);
        lv_slider_set_value(vol_sliders[i], p4.track_volume[i], LV_ANIM_OFF);
        lv_obj_set_style_bg_color(vol_sliders[i], lv_color_hex(0x2A2A2A), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(vol_sliders[i], LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_radius(vol_sliders[i], 4, LV_PART_MAIN);
        lv_obj_set_style_bg_color(vol_sliders[i], tc, LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(vol_sliders[i], LV_OPA_COVER, LV_PART_INDICATOR);
        lv_obj_set_style_radius(vol_sliders[i], 4, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(vol_sliders[i], lv_color_white(), LV_PART_KNOB);
        lv_obj_set_style_bg_opa(vol_sliders[i], LV_OPA_COVER, LV_PART_KNOB);
        lv_obj_set_style_pad_all(vol_sliders[i], 5, LV_PART_KNOB);
        lv_obj_set_style_radius(vol_sliders[i], LV_RADIUS_CIRCLE, LV_PART_KNOB);
        lv_obj_set_style_shadow_color(vol_sliders[i], tc, LV_PART_KNOB);
        lv_obj_set_style_shadow_width(vol_sliders[i], 8, LV_PART_KNOB);
        lv_obj_set_style_shadow_opa(vol_sliders[i], LV_OPA_60, LV_PART_KNOB);
        lv_obj_set_style_border_color(vol_sliders[i], tc, LV_PART_KNOB);
        lv_obj_set_style_border_width(vol_sliders[i], 2, LV_PART_KNOB);
        lv_obj_add_event_cb(vol_sliders[i], vol_slider_cb, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)i);

        // VU meter — thin vertical bar just right of the fader. Driven by the
        // dsp spectrum (synthetic today; real once audio FFT lands). lv_bar is
        // vertical automatically because height > width, and fills bottom-up.
        vol_vu_bars[i] = lv_bar_create(scr_volumes);
        lv_obj_set_size(vol_vu_bars[i], 6, slider_h);
        lv_obj_set_pos(vol_vu_bars[i], cx + 10, y_sl);
        lv_bar_set_range(vol_vu_bars[i], 0, 255);
        lv_bar_set_value(vol_vu_bars[i], 0, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(vol_vu_bars[i], lv_color_hex(0x1E1E1E), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(vol_vu_bars[i], LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_radius(vol_vu_bars[i], 2, LV_PART_MAIN);
        lv_obj_set_style_bg_color(vol_vu_bars[i], RED808_SUCCESS, LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(vol_vu_bars[i], LV_OPA_COVER, LV_PART_INDICATOR);
        lv_obj_set_style_radius(vol_vu_bars[i], 2, LV_PART_INDICATOR);
        lv_obj_clear_flag(vol_vu_bars[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(vol_vu_bars[i], LV_OBJ_FLAG_SCROLLABLE);

        // Peak-hold cap — thin mark that holds the loudest recent level and
        // falls slowly (classic VU). Positioned each frame by update_volumes_screen.
        vol_vu_caps[i] = lv_obj_create(scr_volumes);
        lv_obj_set_size(vol_vu_caps[i], 6, 2);
        lv_obj_set_pos(vol_vu_caps[i], cx + 10, y_sl + slider_h - 2);
        lv_obj_set_style_bg_color(vol_vu_caps[i], lv_color_white(), 0);
        lv_obj_set_style_bg_opa(vol_vu_caps[i], LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(vol_vu_caps[i], 0, 0);
        lv_obj_set_style_radius(vol_vu_caps[i], 0, 0);
        lv_obj_clear_flag(vol_vu_caps[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(vol_vu_caps[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(vol_vu_caps[i], LV_OBJ_FLAG_HIDDEN);

        // Color bar at bottom of slider
        lv_obj_t* color_bar = lv_obj_create(scr_volumes);
        lv_obj_set_size(color_bar, strip_w - 6, 3);
        lv_obj_set_pos(color_bar, x + 3, y_sl + slider_h + 2);
        lv_obj_set_style_bg_color(color_bar, tc, 0);
        lv_obj_set_style_bg_opa(color_bar, LV_OPA_80, 0);
        lv_obj_set_style_radius(color_bar, 1, 0);
        lv_obj_set_style_border_width(color_bar, 0, 0);
        lv_obj_clear_flag(color_bar, LV_OBJ_FLAG_SCROLLABLE);

        // Value label
        int y_val = y_sl + slider_h + 8;
        vol_labels[i] = lv_label_create(scr_volumes);
        lv_label_set_text_fmt(vol_labels[i], "%d", p4.track_volume[i]);
        lv_obj_set_style_text_font(vol_labels[i], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(vol_labels[i], RED808_TEXT, 0);
        lv_obj_set_style_text_align(vol_labels[i], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(vol_labels[i], x, y_val);
        lv_obj_set_width(vol_labels[i], strip_w);

        // Mute dot
        vol_mute_dots[i] = lv_obj_create(scr_volumes);
        lv_obj_set_size(vol_mute_dots[i], 8, 8);
        lv_obj_set_pos(vol_mute_dots[i], cx - 4, y_val + value_h + 2);
        lv_obj_set_style_radius(vol_mute_dots[i], 4, 0);
        lv_obj_set_style_bg_color(vol_mute_dots[i], RED808_SUCCESS, 0);
        lv_obj_set_style_bg_opa(vol_mute_dots[i], LV_OPA_COVER, 0);
        lv_obj_set_style_shadow_color(vol_mute_dots[i], RED808_SUCCESS, 0);
        lv_obj_set_style_shadow_width(vol_mute_dots[i], 6, 0);
        lv_obj_set_style_shadow_opa(vol_mute_dots[i], LV_OPA_50, 0);
        lv_obj_set_style_border_width(vol_mute_dots[i], 0, 0);
        lv_obj_clear_flag(vol_mute_dots[i], LV_OBJ_FLAG_SCROLLABLE);
    }
}

static void update_volumes_screen(void) {
    static int  prev_volume[16] = {-1, -1, -1, -1, -1, -1, -1, -1,
                                   -1, -1, -1, -1, -1, -1, -1, -1};
    static bool prev_muted[16] = {};
    static bool prev_init = false;
    static int prev_master = -1;
    static int prev_bpm = -1;

    if (mix_master_slider && p4.master_volume != prev_master) {
        prev_master = p4.master_volume;
        lv_slider_set_value(mix_master_slider, p4.master_volume, LV_ANIM_OFF);
        if (mix_master_lbl) lv_label_set_text_fmt(mix_master_lbl, "%d", p4.master_volume);
    }
    if (mix_bpm_slider && p4.bpm_int != prev_bpm) {
        prev_bpm = p4.bpm_int;
        lv_slider_set_value(mix_bpm_slider, p4.bpm_int, LV_ANIM_OFF);
        if (mix_bpm_lbl) lv_label_set_text_fmt(mix_bpm_lbl, "%d", p4.bpm_int);
    }

    // VU meters — driven by the dsp spectrum. 3 colour zones (green/amber/red),
    // plus a peak-hold cap that catches the loudest recent level and falls slow.
    // Bar value/colour only repaint on change; the cap follows the peak.
    {
        static uint8_t prev_vu[16] = {};
        static uint8_t vu_peak[16] = {};
        SpectrumData sp;
        dsp_get_spectrum(&sp);
        for (int i = 0; i < 16; i++) {
            if (!vol_vu_bars[i]) continue;
            uint8_t lvl = p4.track_muted[i] ? 0 : sp.bars[i];

            // Peak hold with slow fall.
            if (lvl > vu_peak[i])      vu_peak[i] = lvl;
            else if (vu_peak[i] >= 3)  vu_peak[i] -= 3;
            else                       vu_peak[i] = 0;

            if (lvl != prev_vu[i]) {
                prev_vu[i] = lvl;
                lv_bar_set_value(vol_vu_bars[i], lvl, LV_ANIM_OFF);
                lv_obj_set_style_bg_color(vol_vu_bars[i],
                    (lvl > 210) ? RED808_ERROR : (lvl > 140) ? RED808_WARNING : RED808_SUCCESS,
                    LV_PART_INDICATOR);
            }

            if (vol_vu_caps[i]) {
                if (vu_peak[i] == 0) {
                    lv_obj_add_flag(vol_vu_caps[i], LV_OBJ_FLAG_HIDDEN);
                } else {
                    int by = lv_obj_get_y(vol_vu_bars[i]);
                    int bh = lv_obj_get_height(vol_vu_bars[i]);
                    int cy = by + bh - (vu_peak[i] * bh) / 255 - 2;
                    if (cy < by) cy = by;
                    lv_obj_clear_flag(vol_vu_caps[i], LV_OBJ_FLAG_HIDDEN);
                    lv_obj_set_y(vol_vu_caps[i], cy);
                    lv_obj_set_style_bg_color(vol_vu_caps[i],
                        (vu_peak[i] > 210) ? RED808_ERROR : (vu_peak[i] > 140) ? RED808_WARNING : lv_color_white(), 0);
                }
            }
        }
    }

    for (int i = 0; i < 16; i++) {
        bool volume_changed = p4.track_volume[i] != prev_volume[i];
        bool mute_changed = !prev_init || p4.track_muted[i] != prev_muted[i];
        if (!volume_changed && !mute_changed) continue;

        prev_volume[i] = p4.track_volume[i];
        prev_muted[i] = p4.track_muted[i];

        if (volume_changed) {
            if (vol_sliders[i]) lv_slider_set_value(vol_sliders[i], p4.track_volume[i], LV_ANIM_OFF);
            if (vol_labels[i]) lv_label_set_text_fmt(vol_labels[i], "%d", p4.track_volume[i]);
        }
        if (vol_mute_dots[i]) {
            lv_obj_set_style_bg_color(vol_mute_dots[i],
                p4.track_muted[i] ? RED808_ERROR : RED808_SUCCESS, 0);
            lv_obj_set_style_shadow_color(vol_mute_dots[i],
                p4.track_muted[i] ? RED808_ERROR : RED808_SUCCESS, 0);
        }
        if (vol_strip_panels[i]) {
            lv_obj_set_style_border_color(vol_strip_panels[i],
                p4.track_muted[i] ? RED808_ERROR :
                lv_color_hex(theme_presets[currentTheme].track_colors[i]), 0);
            lv_obj_set_style_border_opa(vol_strip_panels[i],
                p4.track_muted[i] ? LV_OPA_80 : LV_OPA_40, 0);
            lv_obj_set_style_bg_opa(vol_strip_panels[i],
                p4.track_muted[i] ? LV_OPA_20 : LV_OPA_40, 0);
        }
        if (vol_name_labels[i]) {
            lv_obj_set_style_text_color(vol_name_labels[i],
                p4.track_muted[i] ? RED808_TEXT_DIM :
                lv_color_hex(theme_presets[currentTheme].track_colors[i]), 0);
        }
    }
    prev_init = true;
}

// =============================================================================
// SD CARD SCREEN — browse P4 local SD card or P4 internal MEM MIDI storage
// =============================================================================

// SD screen widgets
static lv_obj_t* sd_left_panel  = NULL;
static lv_obj_t* sd_right_panel = NULL;
static lv_obj_t* sd_status_lbl  = NULL;
static lv_obj_t* sd_path_lbl    = NULL;
static lv_obj_t* sd_file_list   = NULL;
static lv_obj_t* sd_selected_lbl = NULL;
static lv_obj_t* sd_assign_lbl = NULL;
static lv_obj_t* sd_pad_btns[16] = {};
static lv_obj_t* sd_load_btn    = NULL;
static lv_obj_t* sd_load_lbl    = NULL;
static lv_obj_t* sd_preview_btn = NULL;
static lv_obj_t* sd_preview_lbl = NULL;
// MIDI section
static lv_obj_t* sd_wav_section       = NULL;
static lv_obj_t* sd_midi_section      = NULL;
static lv_obj_t* sd_midi_pat_btns[10] = {};
static lv_obj_t* sd_midi_load_btn     = NULL;
static lv_obj_t* sd_midi_info_lbl     = NULL;
static lv_obj_t* sd_midi_status_lbl   = NULL;
static int        sd_midi_target_slot  = 6;   // default: P07
static bool       sd_is_midi_mode      = false;
// 0 = PRO (merge all channels, dense drum sequencer feel)
// 1 = STD (GM drum channel 9 only, closer to a standard MIDI player)
static int        sd_midi_import_mode  = 0;
static lv_obj_t*  sd_midi_mode_pro_btn = NULL;
static lv_obj_t*  sd_midi_mode_std_btn = NULL;

// Forward declarations
static void sd_refresh_ui(void);
static void sd_switch_panel_mode(bool midi_mode);
static void sd_midi_pat_btn_cb(lv_event_t* e);
static void sd_midi_load_btn_cb(lv_event_t* e);
static void sd_refresh_source(void);
static void show_midi_load_summary(const char* title, int slot,
                                   int steps, int raw_len, float bpm,
                                   int tracks_used);

// ── Sources: 0 = P4 local SD, 1 = MEM MIDI in P4 SPIFFS ─────────────────────
static int        sd_source            = 0;
static lv_obj_t*  sd_src_sd_btn        = NULL;
static lv_obj_t*  sd_src_mem_btn       = NULL;
static char       sd_mem_files[64][48] = {};
static int        sd_mem_count         = 0;
static int        sd_mem_selected      = -1;
static bool       sd_local_mounted     = false;

static const char* sd_basename(const char* path) {
    const char* base = strrchr(path, '/');
    return base ? base + 1 : path;
}

static bool sd_local_try_mount(void) {
    if (sd_local_mounted) return true;
    if (SD_MMC.begin("/sdcard", false, false)) {
        sd_local_mounted = true;
        return true;
    }
    SD_MMC.end();
    if (SD_MMC.begin("/sdcard", true, false)) {
        sd_local_mounted = true;
        return true;
    }
    sd_local_mounted = false;
    return false;
}

static void sd_local_reset_selection(void) {
    p4sd.selected_file[0] = '\0';
    p4sd.selected_is_midi = false;
    p4sd.midi_load_result = -2;
}

static void sd_local_refresh_listing(bool reset_selection) {
    if (reset_selection) sd_local_reset_selection();
    p4sd.entry_count = 0;
    p4sd.list_complete = false;

    if (!sd_local_try_mount()) {
        p4sd.mounted = false;
        if (p4sd.path[0] == '\0') strcpy(p4sd.path, "/");
        p4sd.list_complete = true;
        p4sd.needs_refresh = true;
        return;
    }

    p4sd.mounted = true;
    if (p4sd.path[0] == '\0') strcpy(p4sd.path, "/");

    File dir = SD_MMC.open(p4sd.path);
    if (!dir || !dir.isDirectory()) {
        strcpy(p4sd.path, "/");
        dir = SD_MMC.open("/");
    }
    if (!dir || !dir.isDirectory()) {
        p4sd.mounted = false;
        p4sd.list_complete = true;
        p4sd.needs_refresh = true;
        return;
    }

    File entry = dir.openNextFile();
    while (entry && p4sd.entry_count < P4_SD_MAX_ENTRIES) {
        const char* base = sd_basename(entry.name());
        bool is_dir = entry.isDirectory();
        if (base[0] == '\0' || base[0] == '.') {
            entry.close();
            entry = dir.openNextFile();
            continue;
        }
        bool is_midi = false;
        if (!is_dir) {
            size_t nlen = strlen(base);
            bool is_wav = (nlen > 4 && strcasecmp(base + nlen - 4, ".wav") == 0);
            is_midi = (nlen > 4 && strcasecmp(base + nlen - 4, ".mid") == 0);
            if (!is_wav && !is_midi) {
                entry.close();
                entry = dir.openNextFile();
                continue;
            }
        }
        P4SdEntry& out = p4sd.entries[p4sd.entry_count++];
        strncpy(out.name, base, sizeof(out.name) - 1);
        out.name[sizeof(out.name) - 1] = '\0';
        out.is_dir = is_dir;
        out.is_midi = is_midi;
        entry.close();
        entry = dir.openNextFile();
    }
    dir.close();
    p4sd.list_complete = true;
    p4sd.needs_refresh = true;
}

static void sd_local_select(int idx) {
    if (idx < 0 || idx >= p4sd.entry_count) return;
    const P4SdEntry& entry = p4sd.entries[idx];
    if (entry.is_dir) {
        char next_path[128];
        if (strcmp(p4sd.path, "/") == 0) {
            snprintf(next_path, sizeof(next_path), "/%s", entry.name);
        } else {
            snprintf(next_path, sizeof(next_path), "%s/%s", p4sd.path, entry.name);
        }
        strncpy(p4sd.path, next_path, sizeof(p4sd.path) - 1);
        p4sd.path[sizeof(p4sd.path) - 1] = '\0';
        sd_local_refresh_listing(true);
    } else {
        strncpy(p4sd.selected_file, entry.name, sizeof(p4sd.selected_file) - 1);
        p4sd.selected_file[sizeof(p4sd.selected_file) - 1] = '\0';
        p4sd.selected_is_midi = entry.is_midi;
        p4sd.midi_load_result = -2;
        p4sd.needs_refresh = true;
    }
}

static void sd_local_back(void) {
    if (strcmp(p4sd.path, "/") == 0 || p4sd.path[0] == '\0') {
        sd_local_refresh_listing(true);
        return;
    }
    char* last = strrchr(p4sd.path, '/');
    if (last && last != p4sd.path) *last = '\0';
    else strcpy(p4sd.path, "/");
    sd_local_refresh_listing(true);
}

static void sd_mem_refresh_list(void) {
    sd_mem_count = mem_midi::list_midi_files("/mid", sd_mem_files, 64);
    if (sd_mem_selected >= sd_mem_count) sd_mem_selected = -1;
}

static void sd_refresh_source(void) {
    if (s_sd_for_xtra) {
        sd_source = 0;
    }
    if (sd_source == 1) {
        sd_mem_refresh_list();
        sd_switch_panel_mode(true);
    } else {
        if (p4sd.path[0] == '\0') strcpy(p4sd.path, "/");
        sd_local_refresh_listing(false);
    }
    sd_refresh_ui();
}

static void sd_mem_file_btn_cb(lv_event_t* e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= sd_mem_count) return;
    sd_mem_selected = idx;
    if (sd_midi_info_lbl) lv_label_set_text(sd_midi_info_lbl, sd_mem_files[idx]);
    if (sd_midi_load_btn) lv_obj_clear_state(sd_midi_load_btn, LV_STATE_DISABLED);
    if (sd_midi_status_lbl) lv_label_set_text(sd_midi_status_lbl, "");
    sd_refresh_ui();
}

static void sd_source_btn_cb(lv_event_t* e) {
    int src = (int)(intptr_t)lv_event_get_user_data(e);
    if (s_sd_for_xtra && src != 0) {
        ui_show_toast("XTRA usa solo SD WAV", RED808_WARNING);
        return;
    }
    if (src == sd_source) return;
    sd_source = src;
    sd_mem_selected = -1;
    if (sd_midi_info_lbl) lv_label_set_text(sd_midi_info_lbl, "");
    if (sd_midi_status_lbl) lv_label_set_text(sd_midi_status_lbl, "");
    if (sd_midi_load_btn) lv_obj_add_state(sd_midi_load_btn, LV_STATE_DISABLED);
    if (src == 1) {
        sd_mem_refresh_list();
        sd_switch_panel_mode(true);   // MEM is MIDI-only
    } else {
        if (p4sd.path[0] == '\0') strcpy(p4sd.path, "/");
        sd_local_refresh_listing(true);
        sd_switch_panel_mode(false);  // default local SD view
    }
    if (sd_src_sd_btn) {
        lv_obj_set_style_bg_color(sd_src_sd_btn,
            src == 0 ? RED808_CYAN : lv_color_hex(0x1A2A3A), 0);
    }
    if (sd_src_mem_btn) {
        lv_obj_set_style_bg_color(sd_src_mem_btn,
            src == 1 ? RED808_WARNING : lv_color_hex(0x1A2A3A), 0);
    }
    sd_refresh_ui();
}

static void sd_switch_panel_mode(bool midi_mode) {
    sd_is_midi_mode = midi_mode;
    if (sd_wav_section) {
        if (midi_mode) lv_obj_add_flag(sd_wav_section, LV_OBJ_FLAG_HIDDEN);
        else           lv_obj_clear_flag(sd_wav_section, LV_OBJ_FLAG_HIDDEN);
    }
    if (sd_midi_section) {
        if (midi_mode) lv_obj_clear_flag(sd_midi_section, LV_OBJ_FLAG_HIDDEN);
        else           lv_obj_add_flag(sd_midi_section, LV_OBJ_FLAG_HIDDEN);
    }
}

static void sd_file_btn_cb(lv_event_t* e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= p4sd.entry_count) return;
    const P4SdEntry& entry = p4sd.entries[idx];
    if (!entry.is_dir) {
        // Track selection type immediately (before S3 response arrives)
        p4sd.selected_is_midi = entry.is_midi;
        sd_switch_panel_mode(entry.is_midi);
        if (entry.is_midi && sd_midi_info_lbl) {
            lv_label_set_text(sd_midi_info_lbl, entry.name);
        }
        if (entry.is_midi && sd_midi_load_btn) {
            lv_obj_clear_state(sd_midi_load_btn, LV_STATE_DISABLED);
        }
        if (entry.is_midi && sd_midi_status_lbl) {
            lv_label_set_text(sd_midi_status_lbl, "");
        }
        if (entry.is_midi) {
            // Reset any stale load result when picking a new file
            p4sd.midi_load_result = -2;
        }
    }
    sd_local_select(idx);
}

static void sd_back_btn_cb(lv_event_t* e) {
    (void)e;
    p4sd.selected_is_midi = false;
    sd_switch_panel_mode(false);
    sd_local_back();
}

static void sd_pad_btn_cb(lv_event_t* e) {
    if (s_sd_for_xtra) return;
    int pad = (int)(intptr_t)lv_event_get_user_data(e);
    if (pad < 0 || pad >= 16) return;
    p4sd.selected_pad = pad;
    // Update pad button highlights
    for (int i = 0; i < 16; i++) {
        if (sd_pad_btns[i]) {
            lv_obj_set_style_bg_color(sd_pad_btns[i],
                i == pad ? RED808_ACCENT : lv_color_hex(0x222233), 0);
        }
    }
}

static void sd_midi_pat_btn_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot > 5) return;
    sd_midi_target_slot = slot;
    for (int i = 0; i < 6; i++) {
        if (!sd_midi_pat_btns[i]) continue;
        bool sel = (i == slot);
        lv_obj_set_style_bg_color(sd_midi_pat_btns[i],
            sel ? RED808_ACCENT : lv_color_hex(0x1A2A3A), 0);
        lv_obj_set_style_border_color(sd_midi_pat_btns[i],
            sel ? RED808_CYAN : lv_color_hex(0x334455), 0);
    }
}

// ── MIDI load summary modal ─────────────────────────────────────────────────
// Shown after a successful MEM-MIDI load. Displays filename, BPM, step count
// and unique tracks. OK button dismisses and navigates to the sequencer.
static lv_obj_t* midi_summary_modal = NULL;

static void midi_summary_ok_cb(lv_event_t* e) {
    (void)e;
    if (midi_summary_modal) {
        lv_obj_del(midi_summary_modal);
        midi_summary_modal = NULL;
    }
    ui_navigate_to(3);   // screen 3 = SEQUENCER
}

static void show_midi_load_summary(const char* title, int slot,
                                   int steps, int raw_len, float bpm,
                                   int tracks_used) {
    // Remember last summary so the sequencer info button can re-open it.
    snprintf(seq_last_midi_name, sizeof(seq_last_midi_name), "%s",
             title ? title : "(unknown)");
    seq_last_midi_slot    = slot;
    seq_last_midi_steps   = steps;
    seq_last_midi_raw_len = raw_len;
    seq_last_midi_bpm     = bpm;
    seq_last_midi_tracks  = tracks_used;
    seq_last_midi_valid   = true;

    if (midi_summary_modal) {
        lv_obj_del(midi_summary_modal);
        midi_summary_modal = NULL;
    }
    lv_obj_t* scr = lv_layer_top();
    midi_summary_modal = lv_obj_create(scr);
    lv_obj_remove_style_all(midi_summary_modal);
    lv_obj_set_size(midi_summary_modal, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(midi_summary_modal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(midi_summary_modal, LV_OPA_60, 0);
    lv_obj_add_flag(midi_summary_modal, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(midi_summary_modal, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* card = lv_obj_create(midi_summary_modal);
    lv_obj_set_size(card, 520, 320);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, RED808_PANEL, 0);
    lv_obj_set_style_border_color(card, RED808_ACCENT, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_radius(card, 10, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* t = lv_label_create(card);
    lv_label_set_text(t, LV_SYMBOL_OK "  MIDI Loaded");
    lv_obj_set_style_text_color(t, RED808_SUCCESS, 0);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_24, 0);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_t* fn = lv_label_create(card);
    char fnbuf[96];
    snprintf(fnbuf, sizeof(fnbuf), "File: %s", title ? title : "(unknown)");
    lv_label_set_text(fn, fnbuf);
    lv_obj_set_style_text_color(fn, RED808_TEXT, 0);
    lv_obj_set_style_text_font(fn, &lv_font_montserrat_18, 0);
    lv_obj_align(fn, LV_ALIGN_TOP_LEFT, 20, 58);

    lv_obj_t* l1 = lv_label_create(card);
    char b1[160];
    snprintf(b1, sizeof(b1),
        "Pattern slot:  P%02d\n"
        "Tempo:         %.1f BPM\n"
        "Steps (bar1):  %d\n"
        "Raw length:    %d steps (%d bars)\n"
        "Tracks used:   %d / 16",
        slot, bpm, steps, raw_len, raw_len / 16, tracks_used);
    lv_label_set_text(l1, b1);
    lv_obj_set_style_text_color(l1, RED808_TEXT, 0);
    lv_obj_set_style_text_font(l1, &lv_font_montserrat_18, 0);
    lv_obj_align(l1, LV_ALIGN_TOP_LEFT, 20, 96);

    // If STD import found no drum track and fell back to mapping every channel
    // onto the drum pads, the grid is only an approximation — say so plainly so
    // the result doesn't look like it was "invented".
    if (mem_midi::last_load_used_channel_fallback()) {
        lv_obj_set_style_border_color(card, RED808_WARNING, 0);
        lv_label_set_text(t, LV_SYMBOL_WARNING "  MIDI Loaded (approx)");
        lv_obj_set_style_text_color(t, RED808_WARNING, 0);
        lv_obj_t* warn = lv_label_create(card);
        lv_label_set_text(warn,
            LV_SYMBOL_WARNING "  Sin pista de bateria (canal 10):\n"
            "se han mapeado las notas melodicas a los pads.");
        lv_obj_set_style_text_color(warn, RED808_WARNING, 0);
        lv_obj_set_style_text_font(warn, &lv_font_montserrat_16, 0);
        lv_obj_align(warn, LV_ALIGN_TOP_LEFT, 20, 200);
        ui_show_toast("MIDI sin bateria: mapeo aproximado", RED808_WARNING);
    }

    lv_obj_t* ok = lv_btn_create(card);
    lv_obj_set_size(ok, 240, 56);
    lv_obj_align(ok, LV_ALIGN_BOTTOM_MID, 0, -18);
    lv_obj_set_style_bg_color(ok, RED808_ACCENT, 0);
    lv_obj_set_style_radius(ok, 8, 0);
    lv_obj_add_event_cb(ok, midi_summary_ok_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* okl = lv_label_create(ok);
    lv_label_set_text(okl, LV_SYMBOL_OK "  Go to Sequencer");
    lv_obj_set_style_text_color(okl, lv_color_white(), 0);
    lv_obj_set_style_text_font(okl, &lv_font_montserrat_18, 0);
    lv_obj_center(okl);
}

static void sd_midi_load_btn_cb(lv_event_t* e) {
    (void)e;
    // MEM branch — parse from P4 SPIFFS and stage directly to Master.
    if (sd_source == 1) {
        if (sd_mem_selected < 0 || sd_mem_selected >= sd_mem_count) return;
        char path[80];
        snprintf(path, sizeof(path), "/mid/%s", sd_mem_files[sd_mem_selected]);
        if (sd_midi_status_lbl) {
            lv_label_set_text(sd_midi_status_lbl, "Loading MEM MIDI...");
            lv_obj_set_style_text_color(sd_midi_status_lbl, RED808_WARNING, 0);
        }
        // Parse RAW (up to 64 steps) so pagination works and the whole tune
        // actually plays via page auto-advance.
        char name[16];
        int  steps_found = 0, raw_len = 0;
        float bpm = 0;
        bool ok = mem_midi::load_pattern_raw(path, seq_raw_grid, name, sizeof(name),
                                             &steps_found, &bpm, &raw_len,
                                             sd_midi_import_mode);
        if (!ok) {
            if (sd_midi_status_lbl) {
                lv_label_set_text(sd_midi_status_lbl, "MEM parse failed");
                lv_obj_set_style_text_color(sd_midi_status_lbl, RED808_ACCENT, 0);
            }
            return;
        }
        // Install raw grid + switch to page 0 (this also stages bar 1 to Master
        // and mirrors to S3).
        p4.current_pattern = sd_midi_target_slot;
        seq_install_raw_and_show_page0(raw_len);

        if (sd_midi_status_lbl) {
            char buf[64];
            snprintf(buf, sizeof(buf),
                "Loaded \xe2\x86\x92 Pat %02d (%d hits, %d bars)",
                sd_midi_target_slot + 1, steps_found, raw_len / 16);
            lv_label_set_text(sd_midi_status_lbl, buf);
            lv_obj_set_style_text_color(sd_midi_status_lbl, RED808_SUCCESS, 0);
        }
        // Apply MIDI tempo to local state + Master (if present in the SMF)
        if (bpm >= 40.0f && bpm <= 240.0f) {
            p4.bpm_int  = (int)bpm;
            p4.bpm_frac = (int)((bpm - p4.bpm_int) * 10);
            // Lock tempo so the S3's stale cached BPM (sent via UART on
            // reconnect / on its own periodic updates) doesn't overwrite
            // the MIDI tempo we just applied.
            uart_lock_tempo(3000);
            if (ui_use_udp_transport()) udp_send_tempo(bpm);
        }
        // Count unique tracks for summary (across full raw grid)
        int tracks_used = 0;
        for (int t = 0; t < 16; t++)
            for (int s = 0; s < raw_len; s++)
                if (seq_raw_grid[t][s]) { tracks_used++; break; }
        show_midi_load_summary(sd_mem_files[sd_mem_selected],
                               sd_midi_target_slot + 1,
                               steps_found, raw_len, bpm, tracks_used);
        return;
    }

    // Local SD branch — parse from P4 SD_MMC and stage directly to Master.
    if (p4sd.selected_file[0] == '\0') return;
    if (sd_midi_status_lbl) {
        lv_label_set_text(sd_midi_status_lbl, "Loading SD MIDI...");
        lv_obj_set_style_text_color(sd_midi_status_lbl, RED808_WARNING, 0);
    }
    char path[192];
    if (strcmp(p4sd.path, "/") == 0)
        snprintf(path, sizeof(path), "/%s", p4sd.selected_file);
    else
        snprintf(path, sizeof(path), "%s/%s", p4sd.path, p4sd.selected_file);

    char name[16];
    int steps_found = 0, raw_len = 0;
    float bpm = 0;
    bool ok = mem_midi::load_pattern_raw_from_fs(SD_MMC, path, seq_raw_grid,
                                                 name, sizeof(name),
                                                 &steps_found, &bpm, &raw_len,
                                                 sd_midi_import_mode);
    if (!ok) {
        if (sd_midi_status_lbl) {
            lv_label_set_text(sd_midi_status_lbl, "SD MIDI parse failed");
            lv_obj_set_style_text_color(sd_midi_status_lbl, RED808_ACCENT, 0);
        }
        return;
    }

    p4.current_pattern = sd_midi_target_slot;
    seq_install_raw_and_show_page0(raw_len);

    if (sd_midi_status_lbl) {
        char buf[64];
        snprintf(buf, sizeof(buf),
            "Loaded SD -> Pat %02d (%d hits, %d bars)",
            sd_midi_target_slot + 1, steps_found, raw_len / 16);
        lv_label_set_text(sd_midi_status_lbl, buf);
        lv_obj_set_style_text_color(sd_midi_status_lbl, RED808_SUCCESS, 0);
    }
    if (bpm >= 40.0f && bpm <= 240.0f) {
        p4.bpm_int  = (int)bpm;
        p4.bpm_frac = (int)((bpm - p4.bpm_int) * 10);
        uart_lock_tempo(3000);
        if (ui_use_udp_transport()) udp_send_tempo(bpm);
    }
    int tracks_used = 0;
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < raw_len; s++)
            if (seq_raw_grid[t][s]) { tracks_used++; break; }
    show_midi_load_summary(p4sd.selected_file,
                           sd_midi_target_slot + 1,
                           steps_found, raw_len, bpm, tracks_used);
}

static bool sd_upload_selected_wav(bool closeAfterSuccess, bool triggerAfterUpload) {
    if (p4sd.selected_file[0] == '\0' || p4sd.selected_is_midi) return false;
    if (s_ui_busy) return false;   // an upload is already running

    int xtraSlot = -1;
    if (s_xtra_pending_slot >= 0 && s_xtra_pending_slot < 4) {
        xtraSlot = s_xtra_pending_slot;
        p4sd.selected_pad = xtra_backing_pad_for_slot(xtraSlot);
    }

    if (sd_status_lbl) {
        lv_label_set_text_fmt(sd_status_lbl, "%s PAD %02d...",
                              triggerAfterUpload ? "PREVIEW" : "UPLOAD",
                              p4sd.selected_pad + 1);
        lv_obj_set_style_text_color(sd_status_lbl, RED808_CYAN, 0);
    }

    ui_busy_show(triggerAfterUpload ? "Preparando preview en el Master..."
                                    : "Subiendo WAV al Master...\nEspera, no cambies de pantalla.");

    char path[192];
    if (strcmp(p4sd.path, "/") == 0) snprintf(path, sizeof(path), "/%s", p4sd.selected_file);
    else snprintf(path, sizeof(path), "%s/%s", p4sd.path, p4sd.selected_file);

    File sample = SD_MMC.open(path, FILE_READ);
    if (!sample) {
        ui_busy_hide();
        ui_show_toast("No se puede abrir el WAV", RED808_WARNING);
        if (sd_status_lbl) lv_label_set_text(sd_status_lbl, "OPEN FAILED");
        return false;
    }

    size_t sample_size = sample.size();
    if (sample_size == 0 || sample_size > 4U * 1024U * 1024U) {
        sample.close();
        ui_busy_hide();
        ui_show_toast("WAV no valido o demasiado grande", RED808_WARNING);
        if (sd_status_lbl) lv_label_set_text(sd_status_lbl, "WAV INVALID");
        return false;
    }

    WiFiClient client;
    client.setTimeout(10000);
    if (!client.connect(IPAddress(192, 168, 4, 1), 80)) {
        sample.close();
        ui_busy_hide();
        ui_show_toast("Master no conectado", RED808_WARNING);
        if (sd_status_lbl) lv_label_set_text(sd_status_lbl, "MASTER OFFLINE");
        return false;
    }

    const char* boundary = "----RED808P4Upload";
    char file_head[192];
    snprintf(file_head, sizeof(file_head),
             "--%s\r\nContent-Disposition: form-data; name=\"file\"; filename=\"%s\"\r\nContent-Type: audio/wav\r\n\r\n",
             boundary, p4sd.selected_file);
    char file_tail[40];
    snprintf(file_tail, sizeof(file_tail), "\r\n--%s--\r\n", boundary);
    size_t content_len = strlen(file_head) + sample_size + strlen(file_tail);

    client.printf("POST /api/uploadDaisy?pad=%d HTTP/1.1\r\n", p4sd.selected_pad);
    client.print("Host: 192.168.4.1\r\n");
    client.print("Connection: close\r\n");
    client.printf("Content-Type: multipart/form-data; boundary=%s\r\n", boundary);
    client.printf("Content-Length: %u\r\n\r\n", (unsigned)content_len);
    client.print(file_head);

    uint8_t buf[2048];
    bool write_ok = true;
    while (sample.available()) {
        size_t n = sample.read(buf, sizeof(buf));
        if (n == 0) break;
        if (client.write(buf, n) != n) {
            write_ok = false;
            break;
        }
        yield();
    }
    sample.close();
    if (write_ok) client.print(file_tail);

    int status = 0;
    unsigned long wait_start = millis();
    while (millis() - wait_start < 5000) {
        if (client.available()) {
            String line = client.readStringUntil('\n');
            if (line.startsWith("HTTP/1.1 ")) status = line.substring(9, 12).toInt();
            break;
        }
        if (!client.connected()) break;
        delay(2);
    }
    client.stop();

    if (!(write_ok && status == 200)) {
        ui_busy_hide();
        char msg[64];
        if (!write_ok) snprintf(msg, sizeof(msg), "Upload cortado");
        else snprintf(msg, sizeof(msg), "Upload fallido HTTP %d", status);
        ui_show_toast(msg, RED808_WARNING);
        if (sd_status_lbl) {
            lv_label_set_text(sd_status_lbl, msg);
            lv_obj_set_style_text_color(sd_status_lbl, RED808_WARNING, 0);
        }
        return false;
    }

    ui_busy_hide();

    if (triggerAfterUpload && udp_wifi_connected()) {
        udp_send_trigger(p4sd.selected_pad, 110);
    }

    char msg[72];
    snprintf(msg, sizeof(msg), "%s PAD %02d",
             triggerAfterUpload ? "Preview listo en" : "Sample cargado en Daisy",
             p4sd.selected_pad + 1);
    ui_show_toast(msg, RED808_SUCCESS);
    if (sd_status_lbl) {
        lv_label_set_text(sd_status_lbl, msg);
        lv_obj_set_style_text_color(sd_status_lbl, RED808_SUCCESS, 0);
    }

    if (xtraSlot >= 0 && xtraSlot < 4) {
        XtraPadSlot& slot = s_xtra_slots[xtraSlot];
        slot.used = true;
        slot.pad = xtra_backing_pad_for_slot(xtraSlot);
        slot.synth_mode = false;
        strncpy(slot.name, p4sd.selected_file, sizeof(slot.name) - 1);
        slot.name[sizeof(slot.name) - 1] = '\0';
        trim_wav_extension(slot.name);
        xtra_save_state();
        xtra_refresh_panel();
        if (closeAfterSuccess) {
            s_xtra_pending_slot = -1;
            s_sd_for_xtra = false;
            ui_show_toast("XTRA cargado", RED808_SUCCESS);
            ui_navigate_to(6);
        }
    }
    return true;
}

static void sd_preview_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    (void)sd_upload_selected_wav(false, true);
}

static void sd_load_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    // XTRA sampler flow: don't upload the raw file. Remember the source path and
    // open the editor — trim/fade get baked in and uploaded on APPLY.
    if (s_sd_for_xtra && s_xtra_pending_slot >= 0 && s_xtra_pending_slot < 4 &&
        p4sd.selected_file[0] != '\0' && !p4sd.selected_is_midi) {
        int slot = s_xtra_pending_slot;
        XtraPadSlot& s = s_xtra_slots[slot];
        if (strcmp(p4sd.path, "/") == 0)
            snprintf(s.src_path, sizeof(s.src_path), "/%s", p4sd.selected_file);
        else
            snprintf(s.src_path, sizeof(s.src_path), "%s/%s", p4sd.path, p4sd.selected_file);
        strncpy(s.name, p4sd.selected_file, sizeof(s.name) - 1);
        s.name[sizeof(s.name) - 1] = '\0';
        trim_wav_extension(s.name);
        s.used = true;
        s.synth_mode = false;
        s.trim_start = 0.0f; s.trim_end = 1.0f;
        s.fade_in_ms = 0; s.fade_out_ms = 0;
        s.pad = xtra_backing_pad_for_slot(slot);
        s_xtra_loaded_slot = -1;        // force reload in the editor
        s_xtra_applied[slot] = false;   // not baked/uploaded yet
        s_xtra_pending_slot = -1;
        s_sd_for_xtra = false;
        xtra_save_state();
        ui_navigate_to(6);
        xtra_editor_select(slot);
        ui_show_toast("Sample cargado: ajusta y pulsa APPLY", RED808_SUCCESS);
        return;
    }
    (void)sd_upload_selected_wav(true, false);
}

static void sd_refresh_ui(void) {
    if (!sd_file_list) return;

    if (sd_assign_lbl) {
        lv_label_set_text(sd_assign_lbl, s_sd_for_xtra ? "XTRA SLOT TARGET" : "ASSIGN TO PAD");
    }
    for (int i = 0; i < 16; i++) {
        if (!sd_pad_btns[i]) continue;
        if (s_sd_for_xtra) lv_obj_add_flag(sd_pad_btns[i], LV_OBJ_FLAG_HIDDEN);
        else lv_obj_clear_flag(sd_pad_btns[i], LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_clean(sd_file_list);

    if (sd_load_lbl) {
        lv_label_set_text(sd_load_lbl,
            s_sd_for_xtra ? LV_SYMBOL_UPLOAD "  LOAD TO XTRA" : LV_SYMBOL_UPLOAD "  LOAD TO PAD");
    }
    if (sd_preview_lbl) {
        lv_label_set_text(sd_preview_lbl,
            s_sd_for_xtra ? LV_SYMBOL_PLAY "  PREVIEW XTRA" : LV_SYMBOL_PLAY "  PREVIEW PAD");
    }
    if (sd_preview_btn) {
        if (p4sd.mounted && p4sd.selected_file[0] && !p4sd.selected_is_midi)
            lv_obj_clear_state(sd_preview_btn, LV_STATE_DISABLED);
        else
            lv_obj_add_state(sd_preview_btn, LV_STATE_DISABLED);
    }

    // ── MEM branch: list P4's own /mid/*.mid from SPIFFS ────────────────
    if (sd_source == 1) {
        if (sd_status_lbl) {
            char sbuf[24];
            snprintf(sbuf, sizeof(sbuf), "MEM %d MIDI", sd_mem_count);
            lv_label_set_text(sd_status_lbl, sbuf);
            lv_obj_set_style_text_color(sd_status_lbl, RED808_WARNING, 0);
        }
        if (sd_path_lbl) lv_label_set_text(sd_path_lbl, "/mid (flash)");

        // Enable/disable LOAD button based on MEM selection
        if (sd_midi_load_btn) {
            if (sd_mem_selected >= 0)
                lv_obj_clear_state(sd_midi_load_btn, LV_STATE_DISABLED);
            else
                lv_obj_add_state(sd_midi_load_btn, LV_STATE_DISABLED);
        }

        if (sd_mem_count <= 0) {
            lv_obj_t* lbl = lv_label_create(sd_file_list);
            lv_label_set_text(lbl, "No MEM MIDI files");
            lv_obj_set_style_text_color(lbl, RED808_TEXT_DIM, 0);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
            return;
        }

        for (int i = 0; i < sd_mem_count; i++) {
            lv_obj_t* btn = lv_btn_create(sd_file_list);
            lv_obj_set_size(btn, 580, 44);
            lv_obj_set_style_radius(btn, 6, 0);
            bool sel = (i == sd_mem_selected);
            lv_obj_set_style_bg_color(btn,
                sel ? RED808_ACCENT : lv_color_hex(0x3A2A00), 0);
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x886622), LV_STATE_PRESSED);

            lv_obj_t* lbl = lv_label_create(btn);
            char display[64];
            snprintf(display, sizeof(display),
                LV_SYMBOL_FILE "  %s [MEM]", sd_mem_files[i]);
            lv_label_set_text(lbl, display);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
            lv_obj_set_style_text_color(lbl,
                sel ? lv_color_white() : RED808_WARNING, 0);
            lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 12, 0);

            lv_obj_add_event_cb(btn, sd_mem_file_btn_cb, LV_EVENT_CLICKED,
                                (void*)(intptr_t)i);
        }
        return;
    }

    // ── SD branch (local P4 SD_MMC) ─────────────────────────────────────
    // Update status
    if (sd_status_lbl) {
        lv_label_set_text(sd_status_lbl, p4sd.mounted ? "READY" : "NO SD CARD");
        lv_obj_set_style_text_color(sd_status_lbl,
            p4sd.mounted ? RED808_SUCCESS : RED808_WARNING, 0);
    }
    // Update path
    if (sd_path_lbl) lv_label_set_text(sd_path_lbl, p4sd.path);

    // Update selected file
    if (sd_selected_lbl) {
        if (p4sd.selected_file[0])
            lv_label_set_text(sd_selected_lbl, p4sd.selected_file);
        else
            lv_label_set_text(sd_selected_lbl, "");
    }
    // Enable/disable LOAD button
    if (sd_load_btn) {
        if (p4sd.mounted && p4sd.selected_file[0] && !p4sd.selected_is_midi)
            lv_obj_clear_state(sd_load_btn, LV_STATE_DISABLED);
        else
            lv_obj_add_state(sd_load_btn, LV_STATE_DISABLED);
    }
    if (sd_midi_load_btn) {
        if (p4sd.selected_file[0] && p4sd.selected_is_midi)
            lv_obj_clear_state(sd_midi_load_btn, LV_STATE_DISABLED);
        else
            lv_obj_add_state(sd_midi_load_btn, LV_STATE_DISABLED);
    }
    if (!p4sd.mounted) {
        lv_obj_t* lbl = lv_label_create(sd_file_list);
        lv_label_set_text(lbl, "SD NOT MOUNTED");
        lv_obj_set_style_text_color(lbl, RED808_ACCENT, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
        return;
    }

    // "Back" button if not root
    if (strcmp(p4sd.path, "/") != 0 && p4sd.path[0] != '\0') {
        lv_obj_t* back_btn = lv_btn_create(sd_file_list);
        lv_obj_set_size(back_btn, 580, 44);
        lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x333344), 0);
        lv_obj_set_style_radius(back_btn, 6, 0);
        lv_obj_t* back_lbl = lv_label_create(back_btn);
        lv_label_set_text(back_lbl, LV_SYMBOL_LEFT "  .. (back)");
        lv_obj_set_style_text_font(back_lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(back_lbl, lv_color_hex(0xCCCCCC), 0);
        lv_obj_center(back_lbl);
        lv_obj_add_event_cb(back_btn, sd_back_btn_cb, LV_EVENT_CLICKED, NULL);
    }

    // File/directory entries
    for (int i = 0; i < p4sd.entry_count; i++) {
        lv_obj_t* btn = lv_btn_create(sd_file_list);
        lv_obj_set_size(btn, 580, 44);
        lv_obj_set_style_radius(btn, 6, 0);

        bool is_dir  = p4sd.entries[i].is_dir;
        bool is_midi = p4sd.entries[i].is_midi;
        lv_obj_set_style_bg_color(btn,
            is_dir  ? lv_color_hex(0x1A3A5C) :
            is_midi ? lv_color_hex(0x3A2A00) : lv_color_hex(0x1A2A1A), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x446688), LV_STATE_PRESSED);

        lv_obj_t* lbl = lv_label_create(btn);
        char display[64];
        snprintf(display, sizeof(display),
            is_dir  ? LV_SYMBOL_DIRECTORY "  %s" :
            is_midi ? LV_SYMBOL_FILE "  %s [MIDI]" : LV_SYMBOL_AUDIO "  %s",
            p4sd.entries[i].name);
        lv_label_set_text(lbl, display);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(lbl,
            is_dir  ? RED808_CYAN :
            is_midi ? RED808_WARNING : RED808_SUCCESS, 0);
        lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 12, 0);

        lv_obj_add_event_cb(btn, sd_file_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    }

    if (p4sd.list_complete && p4sd.entry_count == 0) {
        lv_obj_t* lbl = lv_label_create(sd_file_list);
        lv_label_set_text(lbl, "No files found (.wav / .mid)");
        lv_obj_set_style_text_color(lbl, RED808_TEXT_DIM, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
    }
}

static void create_sdcard_screen(void) {
    scr_sdcard = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_sdcard, RED808_BG, 0);
    lv_obj_clear_flag(scr_sdcard, LV_OBJ_FLAG_SCROLLABLE);

    // Landscape layout: 1024×600
    // Left panel (file browser): 620px wide
    // Right panel (pad assign):  380px wide
    const int TOP    = 8;
    const int PANEL_H = LCD_V_RES - TOP - 8;  // ~584px
    const int LEFT_W  = 620;
    const int RIGHT_W = LCD_H_RES - LEFT_W - 16;  // ~392px
    const int GAP     = 8;

    // ── Left Panel: file browser ──
    sd_left_panel = lv_obj_create(scr_sdcard);
    lv_obj_set_size(sd_left_panel, LEFT_W, PANEL_H);
    lv_obj_set_pos(sd_left_panel, 4, TOP);
    lv_obj_set_style_bg_color(sd_left_panel, lv_color_hex(0x0D1520), 0);
    lv_obj_set_style_bg_opa(sd_left_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(sd_left_panel, RED808_INFO, 0);
    lv_obj_set_style_border_width(sd_left_panel, 1, 0);
    lv_obj_set_style_radius(sd_left_panel, 8, 0);
    lv_obj_set_style_pad_all(sd_left_panel, 8, 0);
    lv_obj_clear_flag(sd_left_panel, LV_OBJ_FLAG_SCROLLABLE);

    // Title
    lv_obj_t* title_lbl = lv_label_create(sd_left_panel);
    lv_label_set_text(title_lbl, LV_SYMBOL_DRIVE "  SD CARD BROWSER");
    lv_obj_set_style_text_font(title_lbl, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title_lbl, RED808_CYAN, 0);
    lv_obj_set_pos(title_lbl, 8, 4);

    // Source toggle: [SD] [MEM]
    {
        int btn_w = 70, btn_h = 28;
        int bx = 240, by = 4;
        sd_src_sd_btn = lv_btn_create(sd_left_panel);
        lv_obj_set_size(sd_src_sd_btn, btn_w, btn_h);
        lv_obj_set_pos(sd_src_sd_btn, bx, by);
        lv_obj_set_style_bg_color(sd_src_sd_btn, RED808_CYAN, 0);
        lv_obj_set_style_radius(sd_src_sd_btn, 6, 0);
        lv_obj_set_style_border_width(sd_src_sd_btn, 1, 0);
        lv_obj_set_style_border_color(sd_src_sd_btn, lv_color_hex(0x334455), 0);
        lv_obj_t* l1 = lv_label_create(sd_src_sd_btn);
        lv_label_set_text(l1, "SD");
        lv_obj_set_style_text_font(l1, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(l1, lv_color_black(), 0);
        lv_obj_center(l1);
        lv_obj_add_event_cb(sd_src_sd_btn, sd_source_btn_cb,
                            LV_EVENT_CLICKED, (void*)(intptr_t)0);

        sd_src_mem_btn = lv_btn_create(sd_left_panel);
        lv_obj_set_size(sd_src_mem_btn, btn_w, btn_h);
        lv_obj_set_pos(sd_src_mem_btn, bx + btn_w + 6, by);
        lv_obj_set_style_bg_color(sd_src_mem_btn, lv_color_hex(0x1A2A3A), 0);
        lv_obj_set_style_radius(sd_src_mem_btn, 6, 0);
        lv_obj_set_style_border_width(sd_src_mem_btn, 1, 0);
        lv_obj_set_style_border_color(sd_src_mem_btn, lv_color_hex(0x334455), 0);
        lv_obj_t* l2 = lv_label_create(sd_src_mem_btn);
        lv_label_set_text(l2, "MEM");
        lv_obj_set_style_text_font(l2, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(l2, RED808_WARNING, 0);
        lv_obj_center(l2);
        lv_obj_add_event_cb(sd_src_mem_btn, sd_source_btn_cb,
                            LV_EVENT_CLICKED, (void*)(intptr_t)1);
    }

    // Status label
    sd_status_lbl = lv_label_create(sd_left_panel);
    lv_label_set_text(sd_status_lbl, "CONNECTING...");
    lv_obj_set_style_text_font(sd_status_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sd_status_lbl, RED808_WARNING, 0);
    lv_obj_set_pos(sd_status_lbl, 420, 8);

    // Path label
    sd_path_lbl = lv_label_create(sd_left_panel);
    lv_label_set_text(sd_path_lbl, "/");
    lv_obj_set_style_text_font(sd_path_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sd_path_lbl, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(sd_path_lbl, 8, 30);

    // Scrollable file list
    sd_file_list = lv_obj_create(sd_left_panel);
    lv_obj_set_size(sd_file_list, LEFT_W - 24, PANEL_H - 72);
    lv_obj_set_pos(sd_file_list, 4, 54);
    lv_obj_set_style_bg_opa(sd_file_list, LV_OPA_0, 0);
    lv_obj_set_style_border_width(sd_file_list, 0, 0);
    lv_obj_set_style_pad_row(sd_file_list, 4, 0);
    lv_obj_set_style_pad_all(sd_file_list, 2, 0);
    lv_obj_set_flex_flow(sd_file_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(sd_file_list, LV_DIR_VER);
    lv_obj_add_flag(sd_file_list, LV_OBJ_FLAG_SCROLLABLE);

    // ── Right Panel: WAV pad assign + MIDI pattern slot ──
    sd_right_panel = lv_obj_create(scr_sdcard);
    lv_obj_set_size(sd_right_panel, RIGHT_W, PANEL_H);
    lv_obj_set_pos(sd_right_panel, LEFT_W + GAP, TOP);
    lv_obj_set_style_bg_color(sd_right_panel, lv_color_hex(0x0D1520), 0);
    lv_obj_set_style_bg_opa(sd_right_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(sd_right_panel, RED808_ACCENT, 0);
    lv_obj_set_style_border_width(sd_right_panel, 1, 0);
    lv_obj_set_style_radius(sd_right_panel, 8, 0);
    lv_obj_set_style_pad_all(sd_right_panel, 8, 0);
    lv_obj_clear_flag(sd_right_panel, LV_OBJ_FLAG_SCROLLABLE);

    // ── WAV section (default visible) ─────────────────────────────────────
    sd_wav_section = lv_obj_create(sd_right_panel);
    lv_obj_set_size(sd_wav_section, LV_PCT(100), LV_PCT(100));
    lv_obj_set_pos(sd_wav_section, 0, 0);
    lv_obj_set_style_bg_opa(sd_wav_section, LV_OPA_0, 0);
    lv_obj_set_style_border_width(sd_wav_section, 0, 0);
    lv_obj_set_style_pad_all(sd_wav_section, 0, 0);
    lv_obj_clear_flag(sd_wav_section, LV_OBJ_FLAG_SCROLLABLE);

    // "ASSIGN TO PAD" title
    sd_assign_lbl = lv_label_create(sd_wav_section);
    lv_label_set_text(sd_assign_lbl, "ASSIGN TO PAD");
    lv_obj_set_style_text_font(sd_assign_lbl, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(sd_assign_lbl, RED808_ACCENT, 0);
    lv_obj_set_pos(sd_assign_lbl, 8, 4);

    // 4x4 pad grid — fit within RIGHT_W
    int pad_gap = 6;
    int pad_w = (RIGHT_W - 32 - 3 * pad_gap) / 4;
    int pad_h = 56;
    int px_start = 8, py_start = 36;
    for (int i = 0; i < 16; i++) {
        int col = i % 4;
        int row = i / 4;
        int px = px_start + col * (pad_w + pad_gap);
        int py = py_start + row * (pad_h + pad_gap);

        lv_obj_t* btn = lv_btn_create(sd_wav_section);
        lv_obj_set_size(btn, pad_w, pad_h);
        lv_obj_set_pos(btn, px, py);
        lv_obj_set_style_bg_color(btn, i == 0 ? RED808_ACCENT : lv_color_hex(0x222233), 0);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_border_color(btn, lv_color_hex(0x444466), 0);

        lv_obj_t* num_lbl = lv_label_create(btn);
        char num_str[12];
        snprintf(num_str, sizeof(num_str), "%s\n%d", trackNames[i], i);
        lv_label_set_text(num_lbl, num_str);
        lv_obj_set_style_text_font(num_lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(num_lbl, lv_color_hex(theme_presets[currentTheme].track_colors[i]), 0);
        lv_obj_set_style_text_align(num_lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_center(num_lbl);

        sd_pad_btns[i] = btn;
        lv_obj_add_event_cb(btn, sd_pad_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    }

    // Selected file label
    sd_selected_lbl = lv_label_create(sd_wav_section);
    lv_label_set_text(sd_selected_lbl, "");
    lv_obj_set_width(sd_selected_lbl, RIGHT_W - 24);
    lv_obj_set_style_text_font(sd_selected_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sd_selected_lbl, RED808_SUCCESS, 0);
    lv_obj_set_style_text_align(sd_selected_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(sd_selected_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(sd_selected_lbl, 8, 300);

    sd_preview_btn = lv_btn_create(sd_wav_section);
    lv_obj_set_size(sd_preview_btn, RIGHT_W - 24, 48);
    lv_obj_set_pos(sd_preview_btn, 8, 308);
    lv_obj_set_style_bg_color(sd_preview_btn, RED808_SURFACE, 0);
    lv_obj_set_style_bg_color(sd_preview_btn, lv_color_hex(0x223344), LV_STATE_DISABLED);
    lv_obj_set_style_border_color(sd_preview_btn, RED808_CYAN, 0);
    lv_obj_set_style_border_width(sd_preview_btn, 2, 0);
    lv_obj_set_style_radius(sd_preview_btn, 10, 0);
    lv_obj_add_state(sd_preview_btn, LV_STATE_DISABLED);
    sd_preview_lbl = lv_label_create(sd_preview_btn);
    lv_label_set_text(sd_preview_lbl, LV_SYMBOL_PLAY "  PREVIEW PAD");
    lv_obj_set_style_text_font(sd_preview_lbl, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(sd_preview_lbl, lv_color_white(), 0);
    lv_obj_center(sd_preview_lbl);
    lv_obj_add_event_cb(sd_preview_btn, sd_preview_btn_cb, LV_EVENT_CLICKED, NULL);

    // LOAD WAV button
    sd_load_btn = lv_btn_create(sd_wav_section);
    lv_obj_set_size(sd_load_btn, RIGHT_W - 24, 60);
    lv_obj_set_pos(sd_load_btn, 8, 362);
    lv_obj_set_style_bg_color(sd_load_btn, RED808_ACCENT, 0);
    lv_obj_set_style_bg_color(sd_load_btn, lv_color_hex(0x882200), LV_STATE_DISABLED);
    lv_obj_set_style_radius(sd_load_btn, 10, 0);
    lv_obj_add_state(sd_load_btn, LV_STATE_DISABLED);

    sd_load_lbl = lv_label_create(sd_load_btn);
    lv_label_set_text(sd_load_lbl, LV_SYMBOL_UPLOAD "  LOAD TO PAD");
    lv_obj_set_style_text_font(sd_load_lbl, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(sd_load_lbl, lv_color_white(), 0);
    lv_obj_center(sd_load_lbl);
    lv_obj_add_event_cb(sd_load_btn, sd_load_btn_cb, LV_EVENT_CLICKED, NULL);

    // ── MIDI section (hidden by default) ──────────────────────────────────
    sd_midi_section = lv_obj_create(sd_right_panel);
    lv_obj_set_size(sd_midi_section, LV_PCT(100), LV_PCT(100));
    lv_obj_set_pos(sd_midi_section, 0, 0);
    lv_obj_set_style_bg_opa(sd_midi_section, LV_OPA_0, 0);
    lv_obj_set_style_border_width(sd_midi_section, 0, 0);
    lv_obj_set_style_pad_all(sd_midi_section, 0, 0);
    lv_obj_clear_flag(sd_midi_section, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(sd_midi_section, LV_OBJ_FLAG_HIDDEN);

    // MIDI title
    lv_obj_t* midi_title = lv_label_create(sd_midi_section);
    lv_label_set_text(midi_title, LV_SYMBOL_AUDIO "  LOAD MIDI TO PATTERN");
    lv_obj_set_style_text_font(midi_title, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(midi_title, RED808_WARNING, 0);
    lv_obj_set_pos(midi_title, 8, 4);

    // File info label
    sd_midi_info_lbl = lv_label_create(sd_midi_section);
    lv_label_set_text(sd_midi_info_lbl, "");
    lv_obj_set_style_text_font(sd_midi_info_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sd_midi_info_lbl, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(sd_midi_info_lbl, 8, 30);
    lv_obj_set_width(sd_midi_info_lbl, RIGHT_W - 24);
    lv_label_set_long_mode(sd_midi_info_lbl, LV_LABEL_LONG_DOT);

    // "SELECT TARGET SLOT:"
    lv_obj_t* slot_lbl = lv_label_create(sd_midi_section);
    lv_label_set_text(slot_lbl, "SELECT TARGET PATTERN SLOT:");
    lv_obj_set_style_text_font(slot_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(slot_lbl, RED808_CYAN, 0);
    lv_obj_set_pos(slot_lbl, 8, 54);

    // Pattern slot grid 2×3 (P01-P06 = slots 0-5, valid master patterns)
    {
        int mp_btn_w = (RIGHT_W - 16 - 10) / 2;  // 2 cols, gap=10
        int mp_btn_h = 52;
        int mp_gap   = 6;
        int mp_x0 = 0, mp_y0 = 76;
        for (int i = 0; i < 6; i++) {
            int col = i % 2, row = i / 2;
            int slot_id = i;
            int bx = mp_x0 + col * (mp_btn_w + 10);
            int by = mp_y0 + row * (mp_btn_h + mp_gap);

            lv_obj_t* btn = lv_btn_create(sd_midi_section);
            lv_obj_set_size(btn, mp_btn_w, mp_btn_h);
            lv_obj_set_pos(btn, bx, by);
            lv_obj_set_style_bg_color(btn,
                slot_id == sd_midi_target_slot ? RED808_ACCENT : lv_color_hex(0x1A2A3A), 0);
            lv_obj_set_style_border_width(btn, 2, 0);
            lv_obj_set_style_border_color(btn,
                slot_id == sd_midi_target_slot ? RED808_CYAN : lv_color_hex(0x334455), 0);
            lv_obj_set_style_radius(btn, 8, 0);

            lv_obj_t* bl = lv_label_create(btn);
            char bname[8];
            snprintf(bname, sizeof(bname), "P%02d", slot_id + 1);
            lv_label_set_text(bl, bname);
            lv_obj_set_style_text_font(bl, &lv_font_montserrat_20, 0);
            lv_obj_set_style_text_color(bl,
                slot_id == sd_midi_target_slot ? lv_color_white() : RED808_TEXT_DIM, 0);
            lv_obj_center(bl);

            sd_midi_pat_btns[i] = btn;
            lv_obj_add_event_cb(btn, sd_midi_pat_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)slot_id);
        }
    }

    // Import-mode selector: PRO / STD
    {
        lv_obj_t* ml = lv_label_create(sd_midi_section);
        lv_label_set_text(ml, "IMPORT MODE:");
        lv_obj_set_style_text_font(ml, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(ml, RED808_CYAN, 0);
        lv_obj_set_pos(ml, 8, 258);

        auto make_mode_btn = [](lv_obj_t* parent, int x, int y, int w, int h,
                                const char* title, const char* subtitle,
                                bool active) -> lv_obj_t* {
            lv_obj_t* btn = lv_btn_create(parent);
            lv_obj_set_size(btn, w, h);
            lv_obj_set_pos(btn, x, y);
            lv_obj_set_style_radius(btn, 8, 0);
            lv_obj_set_style_border_width(btn, 2, 0);
            lv_obj_set_style_bg_color(btn,
                active ? RED808_ACCENT : lv_color_hex(0x1A2A3A), 0);
            lv_obj_set_style_border_color(btn,
                active ? RED808_CYAN : lv_color_hex(0x334455), 0);
            lv_obj_t* t = lv_label_create(btn);
            lv_label_set_text(t, title);
            lv_obj_set_style_text_font(t, &lv_font_montserrat_18, 0);
            lv_obj_set_style_text_color(t,
                active ? lv_color_white() : RED808_TEXT, 0);
            lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 6);
            lv_obj_t* s = lv_label_create(btn);
            lv_label_set_text(s, subtitle);
            lv_obj_set_style_text_font(s, &lv_font_montserrat_10, 0);
            lv_obj_set_style_text_color(s,
                active ? lv_color_white() : RED808_TEXT_DIM, 0);
            lv_obj_align(s, LV_ALIGN_BOTTOM_MID, 0, -4);
            return btn;
        };

        int mb_w = (RIGHT_W - 16 - 10) / 2;
        int mb_h = 50;
        sd_midi_mode_pro_btn = make_mode_btn(sd_midi_section,
            0, 282, mb_w, mb_h, "PRO", "all channels",
            sd_midi_import_mode == 0);
        sd_midi_mode_std_btn = make_mode_btn(sd_midi_section,
            mb_w + 10, 282, mb_w, mb_h, "STD", "GM drums only",
            sd_midi_import_mode == 1);

        lv_obj_add_event_cb(sd_midi_mode_pro_btn, [](lv_event_t*){
            sd_midi_import_mode = 0;
            if (sd_midi_mode_pro_btn) {
                lv_obj_set_style_bg_color(sd_midi_mode_pro_btn, RED808_ACCENT, 0);
                lv_obj_set_style_border_color(sd_midi_mode_pro_btn, RED808_CYAN, 0);
            }
            if (sd_midi_mode_std_btn) {
                lv_obj_set_style_bg_color(sd_midi_mode_std_btn, lv_color_hex(0x1A2A3A), 0);
                lv_obj_set_style_border_color(sd_midi_mode_std_btn, lv_color_hex(0x334455), 0);
            }
        }, LV_EVENT_CLICKED, NULL);
        lv_obj_add_event_cb(sd_midi_mode_std_btn, [](lv_event_t*){
            sd_midi_import_mode = 1;
            if (sd_midi_mode_std_btn) {
                lv_obj_set_style_bg_color(sd_midi_mode_std_btn, RED808_ACCENT, 0);
                lv_obj_set_style_border_color(sd_midi_mode_std_btn, RED808_CYAN, 0);
            }
            if (sd_midi_mode_pro_btn) {
                lv_obj_set_style_bg_color(sd_midi_mode_pro_btn, lv_color_hex(0x1A2A3A), 0);
                lv_obj_set_style_border_color(sd_midi_mode_pro_btn, lv_color_hex(0x334455), 0);
            }
        }, LV_EVENT_CLICKED, NULL);
    }

    // LOAD MIDI PATTERN button
    sd_midi_load_btn = lv_btn_create(sd_midi_section);
    lv_obj_set_size(sd_midi_load_btn, RIGHT_W - 24, 56);
    lv_obj_set_pos(sd_midi_load_btn, 8, 362);
    lv_obj_set_style_bg_color(sd_midi_load_btn, RED808_WARNING, 0);
    lv_obj_set_style_bg_color(sd_midi_load_btn, lv_color_hex(0x554400), LV_STATE_DISABLED);
    lv_obj_set_style_radius(sd_midi_load_btn, 10, 0);
    lv_obj_add_state(sd_midi_load_btn, LV_STATE_DISABLED);
    lv_obj_add_event_cb(sd_midi_load_btn, sd_midi_load_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* midi_load_lbl = lv_label_create(sd_midi_load_btn);
    lv_label_set_text(midi_load_lbl, LV_SYMBOL_DOWNLOAD "  LOAD MIDI PATTERN");
    lv_obj_set_style_text_font(midi_load_lbl, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(midi_load_lbl, lv_color_black(), 0);
    lv_obj_center(midi_load_lbl);

    // Status label
    sd_midi_status_lbl = lv_label_create(sd_midi_section);
    lv_label_set_text(sd_midi_status_lbl, "");
    lv_obj_set_style_text_font(sd_midi_status_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sd_midi_status_lbl, RED808_SUCCESS, 0);
    lv_obj_set_pos(sd_midi_status_lbl, 8, 426);
    lv_obj_set_width(sd_midi_status_lbl, RIGHT_W - 24);
    lv_label_set_long_mode(sd_midi_status_lbl, LV_LABEL_LONG_WRAP);

    // BACK button (return to live)
    lv_obj_t* back_btn = lv_btn_create(sd_right_panel);
    lv_obj_set_size(back_btn, RIGHT_W - 24, 50);
    lv_obj_set_pos(back_btn, 8, PANEL_H - 74);
    lv_obj_set_style_bg_color(back_btn, RED808_SURFACE, 0);
    lv_obj_set_style_radius(back_btn, 8, 0);
    lv_obj_set_style_border_width(back_btn, 1, 0);
    lv_obj_set_style_border_color(back_btn, RED808_BORDER, 0);
    lv_obj_t* back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT "  BACK TO LIVE");
    lv_obj_set_style_text_font(back_lbl, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(back_lbl, RED808_TEXT, 0);
    lv_obj_center(back_lbl);
    lv_obj_add_event_cb(back_btn, [](lv_event_t* e) {
        (void)e;
        ui_navigate_to(2);  // SCREEN_LIVE
    }, LV_EVENT_CLICKED, NULL);
}

// =============================================================================
// PIANO SCREEN — live keyboard (12 / 24 keys) → synth via UDP
// Engines: 3=303, 4=WTosc, 5=SH101, 6=FM2Op, 7=PHYS (GTR)
// =============================================================================
static int  s_piano_engine_idx = 0;          // index into PIANO_ENGINES
static int  s_piano_octave     = 4;          // C4..C5
static bool s_piano_two_oct    = false;      // false=12 keys, true=24 keys
static uint8_t s_piano_velocity = 100;
static int  s_piano_held_note  = -1;         // last note-on still ringing
static bool s_piano_note_active[128] = {false};
static bool s_piano_rec_active = false;      // v2.7 — record to S3 melody screen
// v2.8 — local mirror of recorded notes so P4 can ASSIGN to a pad without
// round-tripping through S3. Same shape as S3: 16 steps × 12 pitch-classes.
static bool s_piano_rec_grid[16][12] = {{false}};
static uint8_t s_piano_rec_notes[16][12] = {{0}};
static int  s_piano_rec_step  = 0;
static int  s_piano_assign_pad = 0;          // 0..15
static uint8_t s_piano_rec_engine = 3;
static uint8_t s_piano_rec_octave = 4;
static bool s_piano_rec_has_notes = false;

static constexpr int PIANO_ENGINE_COUNT = 4;
static const uint8_t PIANO_ENGINES[PIANO_ENGINE_COUNT]      = {3, 4, 5, 6};
static const char*   PIANO_ENGINE_LABELS[PIANO_ENGINE_COUNT] = {"303", "WT", "SH101", "FM2"};

static lv_obj_t* s_piano_engine_btns[PIANO_ENGINE_COUNT]   = {NULL, NULL, NULL, NULL};
static lv_obj_t* s_piano_octave_lbl       = NULL;
static lv_obj_t* s_piano_keys24_btn       = NULL;
static lv_obj_t* s_piano_keys24_lbl       = NULL;
static lv_obj_t* s_piano_status_lbl       = NULL;
static lv_obj_t* s_piano_keys_container   = NULL;
static lv_obj_t* s_piano_expr_bar         = NULL;
static lv_obj_t* s_piano_rec_btn          = NULL;  // v2.7
static lv_obj_t* s_piano_rec_lbl          = NULL;  // v2.7
static lv_obj_t* s_piano_pad_lbl          = NULL;  // v2.8
static lv_obj_t* s_piano_glide_btn        = NULL;
static lv_obj_t* s_piano_glide_lbl        = NULL;
static lv_obj_t* s_piano_bend_btn         = NULL;
static lv_obj_t* s_piano_bend_lbl         = NULL;
static lv_obj_t* s_piano_eng_preset_btn   = NULL;  // quick synth-preset cycler per engine
static lv_obj_t* s_piano_eng_preset_lbl   = NULL;

// Piano gesture state (v3.0): hold+drag for glide and pitch bend.
static bool      s_piano_glide_enabled     = false;
static bool      s_piano_bend_enabled      = false;
static int       s_piano_bend_range_st     = 2;     // default +/-2 semitones
static bool      s_piano_gesture_active    = false;
static int16_t   s_piano_touch_start_x     = 0;
static int16_t   s_piano_touch_start_y     = 0;
static uint8_t   s_piano_touch_base_note   = 60;
static int       s_piano_last_slide_note   = -1;
static float     s_piano_last_bend_st      = 0.0f;
static uint32_t  s_piano_last_bend_send_ms = 0;
static float     s_piano_expr_base_cutoff  = 8000.0f;
static float     s_piano_expr_last_cutoff  = 8000.0f;
static float     s_piano_expr_base_volume  = 0.75f;
static float     s_piano_expr_last_volume  = 0.75f;
static float     s_piano_expr_last_amount  = 0.0f;
static uint32_t  s_piano_expr_last_send_ms = 0;
static bool      s_piano_release_pending   = false;
static uint32_t  s_piano_release_due_ms    = 0;
static lv_obj_t* s_piano_key_obj_by_note[128] = {NULL};
static constexpr uint32_t PIANO_RELEASE_DEBOUNCE_MS = 24;

// Forward declarations for melody grid (defined after key handlers)
static void piano_grid_refresh_cell(int col, int row);
static void piano_grid_refresh_all(void);
static bool piano_vertical_expression_active(void);
static void piano_apply_vertical_expression(int16_t ly);
static void piano_reset_vertical_expression(void);
static void piano_update_status_note(uint8_t midi_note);
static void piano_update_expression_status(void);
static void piano_update_expression_bar(void);
static void piano_sync_active_engine_state(void);
static void piano_engine_preset_cb(lv_event_t* e);
static void piano_refresh_engine_preset_label(void);

static inline bool piano_pc_is_black(uint8_t pc) {
    return (pc == 1) || (pc == 3) || (pc == 6) || (pc == 8) || (pc == 10);
}

static const char* piano_note_name(uint8_t midi) {
    static const char* NAMES[12] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
    static char buf[8];
    int oct = (midi / 12) - 1;
    snprintf(buf, sizeof(buf), "%s%d", NAMES[midi % 12], oct);
    return buf;
}

static inline uint8_t piano_engine_code(void) {
    return PIANO_ENGINES[s_piano_engine_idx];
}

static void piano_set_key_visual(lv_obj_t* btn, uint8_t note, bool pressed) {
    if (!btn) return;
    bool is_black = piano_pc_is_black(note % 12);
    lv_obj_set_style_bg_color(btn,
        pressed ? RED808_ACCENT : (is_black ? lv_color_hex(0x101010) : lv_color_hex(0xF0F0E8)), 0);
}

static void piano_set_note_active(uint8_t note, bool pressed) {
    if (note > 127) return;
    s_piano_note_active[note] = pressed;
    if (s_piano_key_obj_by_note[note]) {
        piano_set_key_visual(s_piano_key_obj_by_note[note], note, pressed);
    }
}

static bool piano_poly_mode_active(void) {
    // All engines play polyphonically. Glide and pitch-bend are inherently
    // monophonic gesture modes, so when either is on we fall back to the
    // single-note (slide/bend) path instead of stacking notes.
    return !s_piano_glide_enabled && !s_piano_bend_enabled;
}

static void piano_send_note_off_specific(uint8_t midi_note) {
    if (!ui_use_udp_transport()) return;
    // Release only this note so the other held notes keep ringing (polyphony).
    udp_send_synth_note_off_ex(piano_engine_code(), 0, midi_note);
}

static void piano_send_engine_all_notes_off(uint8_t engine) {
    if (!ui_use_udp_transport()) return;
    udp_send_synth_note_off(engine, 0xFF);
}

static void piano_send_panic_melodic(void) {
    if (!ui_use_udp_transport()) return;
    for (int i = 0; i < PIANO_ENGINE_COUNT; i++) {
        piano_send_engine_all_notes_off(PIANO_ENGINES[i]);
    }
#if P4_ENABLE_DEBUG_LOG
    Serial.printf("[P4 piano] panic melodic engines=%d\n", PIANO_ENGINE_COUNT);
#endif
}

static void piano_reset_bend(void) {
    if (!ui_use_udp_transport()) return;
    uint8_t engine = piano_engine_code();
    if (engine == 3) {
        udp_send_synth_param(engine, 0, 14, 0.0f);   // PitchBend
    } else if (engine == 6) {
        udp_send_synth_param(engine, 0, 12, 0.0f);   // Detune cents
    }
    s_piano_last_bend_st = 0.0f;
}

static void piano_apply_glide_setting(void) {
    if (!ui_use_udp_transport()) return;
    uint8_t engine = piano_engine_code();
    if (engine == 3) {
        udp_send_synth_param(engine, 0, 5, s_piano_glide_enabled ? 0.12f : 0.01f);
    } else if (engine == 5) {
        udp_send_synth_param(engine, 0, 17, s_piano_glide_enabled ? 0.35f : 0.0f);
    }
}

static void piano_send_off(void);
static void piano_send_on(uint8_t midi_note, bool legato);

static void piano_refresh_gesture_controls(void) {
    if (s_piano_glide_lbl) {
        lv_label_set_text(s_piano_glide_lbl, s_piano_glide_enabled ? "GLIDE ON" : "GLIDE OFF");
    }
    if (s_piano_glide_btn) {
        lv_obj_set_style_border_color(s_piano_glide_btn,
            s_piano_glide_enabled ? lv_color_hex(0x00E5FF) : RED808_BORDER, 0);
        lv_obj_set_style_border_width(s_piano_glide_btn, s_piano_glide_enabled ? 2 : 1, 0);
    }

    if (s_piano_bend_lbl) {
        lv_label_set_text(s_piano_bend_lbl, s_piano_bend_enabled ? "BEND ON" : "BEND OFF");
    }
    if (s_piano_bend_btn) {
        lv_obj_set_style_border_color(s_piano_bend_btn,
            s_piano_bend_enabled ? lv_color_hex(0x7CFF6B) : RED808_BORDER, 0);
        lv_obj_set_style_border_width(s_piano_bend_btn, s_piano_bend_enabled ? 2 : 1, 0);
    }
}

static bool piano_get_local_touch(int16_t* lx, int16_t* ly) {
    if (!s_piano_keys_container || !lx || !ly) return false;
    lv_indev_t* indev = lv_indev_get_act();
    if (!indev) return false;
    lv_point_t p;
    lv_indev_get_point(indev, &p);
    lv_area_t a;
    lv_obj_get_coords(s_piano_keys_container, &a);
    *lx = (int16_t)(p.x - a.x1);
    *ly = (int16_t)(p.y - a.y1);
    return true;
}

static bool piano_note_from_local_xy(int16_t lx, int16_t ly, uint8_t* note_out) {
    if (!s_piano_keys_container || !note_out) return false;
    int container_w = lv_obj_get_width(s_piano_keys_container);
    int container_h = lv_obj_get_height(s_piano_keys_container);
    if (container_w < 80 || container_h < 80) return false;
    if (lx < 0 || lx >= container_w || ly < 0 || ly >= container_h) return false;

    int num_octaves = s_piano_two_oct ? 2 : 1;
    int num_white = num_octaves * 7;
    if (num_white < 1) num_white = 1;
    int white_w = container_w / num_white;
    if (white_w < 4) white_w = 4;
    int white_h = container_h;
    int black_w = (white_w * 6) / 10;
    int black_h = (white_h * 6) / 10;

    static const uint8_t WHITE_PCS[7] = {0, 2, 4, 5, 7, 9, 11};
    static const uint8_t BLACK_PCS[5] = {1, 3, 6, 8, 10};
    static const uint8_t BLACK_AFTER_WHITE[5] = {0, 1, 3, 4, 5};

    int base_midi = (s_piano_octave + 1) * 12;

    if (ly < black_h) {
        for (int oct = 0; oct < num_octaves; oct++) {
            for (int b = 0; b < 5; b++) {
                int wpos = BLACK_AFTER_WHITE[b];
                int x0 = (oct * 7 + wpos) * white_w + white_w - black_w / 2;
                int x1 = x0 + black_w;
                if (lx >= x0 && lx < x1) {
                    int midi = base_midi + oct * 12 + BLACK_PCS[b];
                    if (midi >= 0 && midi <= 127) {
                        *note_out = (uint8_t)midi;
                        return true;
                    }
                }
            }
        }
    }

    int white_idx = lx / white_w;
    if (white_idx < 0) white_idx = 0;
    if (white_idx >= num_white) white_idx = num_white - 1;
    int oct = white_idx / 7;
    int w = white_idx % 7;
    int midi = base_midi + oct * 12 + WHITE_PCS[w];
    if (midi < 0 || midi > 127) return false;
    *note_out = (uint8_t)midi;
    return true;
}

static bool piano_touch_inside_keys(void) {
    int16_t lx = 0, ly = 0;
    if (!piano_get_local_touch(&lx, &ly)) return false;
    uint8_t dummy = 0;
    return piano_note_from_local_xy(lx, ly, &dummy);
}

static void piano_send_bend(float bend_st) {
    if (!ui_use_udp_transport()) return;
    uint8_t engine = piano_engine_code();
    uint32_t now = millis();
    if ((now - s_piano_last_bend_send_ms) < 20 && fabsf(bend_st - s_piano_last_bend_st) < 0.08f) return;

    if (engine == 3) {
        if (bend_st < -12.0f) bend_st = -12.0f;
        if (bend_st > 12.0f) bend_st = 12.0f;
        udp_send_synth_param(engine, 0, 14, bend_st);
        s_piano_last_bend_st = bend_st;
        s_piano_last_bend_send_ms = now;
    } else if (engine == 6) {
        float det = bend_st * 25.0f;  // map +/-2st gesture to +/-50ct detune
        if (det < -50.0f) det = -50.0f;
        if (det > 50.0f) det = 50.0f;
        udp_send_synth_param(engine, 0, 12, det);
        s_piano_last_bend_st = bend_st;
        s_piano_last_bend_send_ms = now;
    }
}

static void piano_send_off(void) {
    if (ui_use_udp_transport()) {
        uint8_t engine = piano_engine_code();
        bool any = false;
        // Release every note currently held so chords don't hang on octave /
        // engine changes or when leaving the screen.
        for (int n = 0; n < 128; n++) {
            if (s_piano_note_active[n]) {
                udp_send_synth_note_off_ex(engine, 0, (uint8_t)n);
                any = true;
            }
        }
        if (!any && s_piano_held_note >= 0) {
            udp_send_synth_note_off_ex(engine, 0, (uint8_t)s_piano_held_note);
        }
    }
    s_piano_release_pending = false;
    s_piano_release_due_ms = 0;
    s_piano_held_note = -1;
    memset(s_piano_note_active, 0, sizeof(s_piano_note_active));
    piano_reset_vertical_expression();
    piano_reset_bend();
    for (int note = 0; note < 128; note++) {
        if (s_piano_key_obj_by_note[note]) {
            piano_set_key_visual(s_piano_key_obj_by_note[note], (uint8_t)note, false);
        }
    }
    if (s_piano_status_lbl) lv_label_set_text(s_piano_status_lbl, "—");
}

static void piano_schedule_release(void) {
    if (s_piano_held_note < 0) return;
    s_piano_release_pending = true;
    s_piano_release_due_ms = millis() + PIANO_RELEASE_DEBOUNCE_MS;
}

static void piano_cancel_release(void) {
    s_piano_release_pending = false;
    s_piano_release_due_ms = 0;
}

static void piano_send_on(uint8_t midi_note, bool legato) {
    piano_cancel_release();
    bool poly_mode = piano_poly_mode_active();
    if (!legato) {
        if (!poly_mode) piano_send_off();
    } else if (s_piano_held_note < 0) {
        legato = false;
    }
    // Per-finger velocity: prefer the touch closest to the finger that
    // triggered this key event, so chords get independent attack levels
    // instead of all notes sharing the global maximum.
    uint8_t attack_velocity = 0;
    lv_indev_t* act_indev = lv_indev_get_act();
    if (act_indev) {
        lv_point_t p;
        lv_indev_get_point(act_indev, &p);
        attack_velocity = lvgl_port_get_touch_velocity_at(p.x, p.y, 80);
    }
    if (attack_velocity == 0) attack_velocity = lvgl_port_get_touch_velocity();
    if (attack_velocity == 0) attack_velocity = s_piano_velocity;
#if P4_ENABLE_DEBUG_LOG
    Serial.printf("[P4 piano] note=%u vel=%u rec=%d transport=%d legato=%d\n", midi_note, (unsigned)attack_velocity, (int)s_piano_rec_active, (int)ui_use_udp_transport(), (int)legato);
#endif
    piano_apply_glide_setting();
    if (ui_use_udp_transport()) {
        udp_send_synth_note_on_ex(piano_engine_code(),
                                   midi_note, attack_velocity, false, legato && s_piano_glide_enabled);
        if (s_piano_rec_active) {
#if P4_ENABLE_DEBUG_LOG
            Serial.printf("[P4 piano] -> melodyRecNote eng=%u note=%u\n", piano_engine_code(), midi_note);
#endif
            udp_send_melody_rec_note(piano_engine_code(), midi_note);
        }
    }
    if (s_piano_rec_active) {
        uart_send_to_s3(MSG_TOUCH_CMD, TCMD_MELODY_NOTE, midi_note);
        int pc = midi_note % 12;
        int row = -1;
        for (int r = 0; r < 12; r++) {
            if ((11 - r) == pc) { row = r; break; }
        }
        if (row >= 0) {
            int col = s_piano_rec_step;
            if (col < 0 || col >= 16) col = 0;
            s_piano_rec_grid[col][row] = true;
            s_piano_rec_notes[col][row] = midi_note;
            s_piano_rec_has_notes = true;
            piano_grid_refresh_cell(col, row);
            s_piano_rec_step = (col + 1) % 16;
        }
    }
    s_piano_held_note = (int)midi_note;
    s_piano_last_slide_note = (int)midi_note;
    piano_set_note_active(midi_note, true);
    piano_update_status_note(midi_note);
}

static void piano_handle_pressing(void) {
    if (!s_piano_gesture_active || s_piano_held_note < 0) return;
    int16_t lx = 0, ly = 0;
    if (!piano_get_local_touch(&lx, &ly)) return;

    if (piano_vertical_expression_active()) {
        piano_apply_vertical_expression(ly);
    }

    uint8_t touch_note = 0;
    if (s_piano_glide_enabled && piano_note_from_local_xy(lx, ly, &touch_note)) {
        if ((int)touch_note != s_piano_last_slide_note) {
            piano_send_on(touch_note, true);
            s_piano_touch_start_x = lx;
            s_piano_touch_base_note = touch_note;
            s_piano_last_bend_st = 0.0f;
        }
    }

    if (!s_piano_bend_enabled) return;

    int container_w = lv_obj_get_width(s_piano_keys_container);
    int num_white = (s_piano_two_oct ? 2 : 1) * 7;
    if (num_white < 1) num_white = 1;
    int white_w = container_w / num_white;
    if (white_w < 10) white_w = 10;
    float sens_px = (float)(white_w * 2);  // full bend across ~2 white keys
    float bend_st = ((float)(lx - s_piano_touch_start_x) / sens_px) * (float)s_piano_bend_range_st;
    if (bend_st < -(float)s_piano_bend_range_st) bend_st = -(float)s_piano_bend_range_st;
    if (bend_st > (float)s_piano_bend_range_st) bend_st = (float)s_piano_bend_range_st;

    uint8_t engine = piano_engine_code();
    if (engine == 3 || engine == 6) {
        piano_send_bend(bend_st);
    } else {
        int bend_steps = (int)lroundf(bend_st);
        int note = (int)s_piano_touch_base_note + bend_steps;
        if (note < 0) note = 0;
        if (note > 127) note = 127;
        if (note != s_piano_held_note) {
            piano_send_on((uint8_t)note, true);
        }
    }
}

static void piano_key_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    uint8_t note = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    bool poly_mode = piano_poly_mode_active();
    if (code == LV_EVENT_PRESSED) {
        bool from_glide = !poly_mode && s_piano_gesture_active && s_piano_held_note >= 0;
        piano_cancel_release();
        int16_t lx = 0, ly = 0;
        piano_get_local_touch(&lx, &ly);
        if (!poly_mode) {
            s_piano_gesture_active = true;
            s_piano_touch_start_x = lx;
            s_piano_touch_start_y = ly;
            s_piano_touch_base_note = note;
            s_piano_last_slide_note = note;
            s_piano_last_bend_st = 0.0f;
            s_piano_last_bend_send_ms = 0;
        }
        if (piano_vertical_expression_active()) {
            s_piano_touch_start_y = ly;
            s_piano_expr_base_cutoff = s_piano_expr_last_cutoff;
            s_piano_expr_base_volume = s_piano_expr_last_volume;
        }
        piano_send_on(note, from_glide);
    } else if (code == LV_EVENT_PRESSING) {
        piano_cancel_release();
        if (!poly_mode) piano_handle_pressing();
    } else if (code == LV_EVENT_PRESS_LOST) {
        if (poly_mode) {
            piano_set_note_active(note, false);
            piano_send_note_off_specific(note);
            if (!piano_touch_inside_keys()) {
                piano_reset_vertical_expression();
            }
            return;
        }
        if (piano_touch_inside_keys()) {
            // Keep gesture alive while sliding across adjacent keys.
            piano_cancel_release();
            piano_handle_pressing();
            return;
        }
        s_piano_gesture_active = false;
        piano_schedule_release();
    } else if (code == LV_EVENT_RELEASED) {
        if (poly_mode) {
            piano_set_note_active(note, false);
            piano_send_note_off_specific(note);
            if (!piano_touch_inside_keys()) {
                piano_reset_vertical_expression();
            }
            return;
        }
        s_piano_gesture_active = false;
        piano_schedule_release();
    }
}

// v2.9 — Envía el estado piano completo a S3 via UART (4 paquetes consecutivos).
// Garantiza que el receptor aplique un estado coherente sin mezclar defaults.
static void piano_uart_broadcast_state(void) {
    uart_send_to_s3(MSG_TOUCH_CMD, TCMD_MELODY_ENGINE, PIANO_ENGINES[s_piano_engine_idx]);
    uart_send_to_s3(MSG_TOUCH_CMD, TCMD_MELODY_OCTAVE, (uint8_t)s_piano_octave);
    uart_send_to_s3(MSG_TOUCH_CMD, TCMD_MELODY_REC,    s_piano_rec_active ? 1 : 0);
    uart_send_to_s3(MSG_TOUCH_CMD, TCMD_MELODY_PAD,    (uint8_t)s_piano_assign_pad);
}

static void piano_refresh_engine_chips(void) {
    for (int i = 0; i < PIANO_ENGINE_COUNT; i++) {
        if (!s_piano_engine_btns[i]) continue;
        bool sel = (i == s_piano_engine_idx);
        lv_obj_set_style_bg_color(s_piano_engine_btns[i],
            sel ? RED808_ACCENT : RED808_SURFACE, 0);
        lv_obj_set_style_border_color(s_piano_engine_btns[i],
            sel ? RED808_ACCENT2 : RED808_BORDER, 0);
    }
    if (s_piano_status_lbl) {
        lv_label_set_text_fmt(s_piano_status_lbl, "ENG %s",
                              PIANO_ENGINE_LABELS[s_piano_engine_idx]);
    }
}

static void piano_engine_btn_cb(lv_event_t* e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= PIANO_ENGINE_COUNT) return;
#if P4_ENABLE_DEBUG_LOG
    uint8_t old_engine = piano_engine_code();
#endif
    piano_send_off();
    piano_send_panic_melodic();
    s_piano_engine_idx = idx;
#if P4_ENABLE_DEBUG_LOG
    Serial.printf("[P4 piano] engine %u -> %u\n", old_engine, piano_engine_code());
#endif
    piano_refresh_engine_chips();
    piano_refresh_engine_preset_label();
    // v2.9 — broadcast engine selection through master
    if (ui_use_udp_transport()) {
        udp_send_melody_set_engine(PIANO_ENGINES[s_piano_engine_idx]);
    }
    piano_sync_active_engine_state();
    // v2.9 — sync directly to S3 melody screen via UART (4 campos completos)
    piano_uart_broadcast_state();
}

static void piano_rebuild_keys(void);

static void piano_octave_btn_cb(lv_event_t* e) {
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    int next = s_piano_octave + delta;
    if (next < 1) next = 1;
    if (next > 7) next = 7;
    if (next == s_piano_octave) return;
    piano_send_off();
    s_piano_octave = next;
    if (s_piano_octave_lbl)
        lv_label_set_text_fmt(s_piano_octave_lbl, "OCT %d", s_piano_octave);
    piano_rebuild_keys();
    // v2.9 — broadcast octave through master
    if (ui_use_udp_transport()) udp_send_melody_set_octave((uint8_t)s_piano_octave);
    // v2.9 — sync directly to S3 melody screen via UART (4 campos completos)
    piano_uart_broadcast_state();
}

static void piano_keys24_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    piano_send_off();
    s_piano_two_oct = !s_piano_two_oct;
    if (s_piano_keys24_lbl)
        lv_label_set_text(s_piano_keys24_lbl, s_piano_two_oct ? "24 K" : "12 K");
    piano_rebuild_keys();
}

static void piano_glide_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    s_piano_glide_enabled = !s_piano_glide_enabled;
    // Avoid zipper/clicks: if a note is currently held, apply on next note-on.
    if (s_piano_held_note < 0) {
        piano_apply_glide_setting();
    }
    piano_refresh_gesture_controls();
}

static void piano_bend_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    s_piano_bend_enabled = !s_piano_bend_enabled;
    // Avoid abrupt pitch jump while holding a note.
    if (!s_piano_bend_enabled && s_piano_held_note < 0) {
        piano_reset_bend();
    }
    piano_refresh_gesture_controls();
}

static void piano_rec_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    static uint32_t s_last_p4_rec_toggle_ms = 0;
    uint32_t now = millis();
    if ((now - s_last_p4_rec_toggle_ms) < 280U) return;
    s_last_p4_rec_toggle_ms = now;
    s_piano_rec_active = !s_piano_rec_active;
    if (s_piano_rec_active) {
        // v2.8 — fresh take: clear local mirror and rewind step cursor
        memset(s_piano_rec_grid, 0, sizeof(s_piano_rec_grid));
        memset(s_piano_rec_notes, 0, sizeof(s_piano_rec_notes));
        s_piano_rec_step = 0;
        s_piano_rec_engine = PIANO_ENGINES[s_piano_engine_idx];
        s_piano_rec_octave = (uint8_t)s_piano_octave;
        s_piano_rec_has_notes = false;
        piano_grid_refresh_all();
    }
    // v2.9 — tell master so all slaves mirror REC state and grid clear
    if (ui_use_udp_transport()) {
        udp_send_melody_rec_toggle(s_piano_rec_active,
                                   PIANO_ENGINES[s_piano_engine_idx],
                                   (uint8_t)s_piano_octave);
    }
    // v2.9 — sync directly to S3 melody screen via UART (4 campos completos)
    piano_uart_broadcast_state();
    if (s_piano_rec_btn) {
        lv_obj_set_style_border_color(s_piano_rec_btn,
            s_piano_rec_active ? lv_color_hex(0xFF3030) : RED808_BORDER, 0);
        lv_obj_set_style_border_width(s_piano_rec_btn, s_piano_rec_active ? 3 : 1, 0);
    }
    if (s_piano_rec_lbl) {
        lv_obj_set_style_text_color(s_piano_rec_lbl,
            s_piano_rec_active ? lv_color_hex(0xFF3030) : RED808_TEXT, 0);
        lv_label_set_text(s_piano_rec_lbl, s_piano_rec_active ? "● REC" : "○ REC");
    }
}

// v2.8 — pad +/- chips: cycle assign target across pads 1..16
static void piano_pad_btn_cb(lv_event_t* e) {
    int delta = (int)(intptr_t)lv_event_get_user_data(e);
    s_piano_assign_pad = (s_piano_assign_pad + delta + 16) % 16;
    if (s_piano_pad_lbl) {
        lv_label_set_text_fmt(s_piano_pad_lbl, "PAD %d", s_piano_assign_pad + 1);
    }
    // v2.9 — broadcast pad selection through master
    if (ui_use_udp_transport()) udp_send_melody_set_pad((uint8_t)s_piano_assign_pad);
    // v2.9 — sync directly to S3 melody screen via UART (4 campos completos)
    piano_uart_broadcast_state();
}

// =============================================================================
// PIANO MELODY GRID — 16 steps × 12 pitch-classes editor with preview
// (P3/P4 Prioridad 4: visual partitura, edición, presets, play, BPM/velocity)
// =============================================================================
static lv_obj_t* s_piano_grid_container = NULL;
static lv_obj_t* s_piano_grid_btns[16][12] = {{NULL}};
static lv_obj_t* s_piano_play_btn = NULL;
static lv_obj_t* s_piano_play_lbl = NULL;
static bool      s_piano_play_active   = false;
static int       s_piano_play_step     = 0;
static uint32_t  s_piano_play_next_ms  = 0;
static uint32_t  s_piano_play_off_due_ms = 0;
static int       s_piano_play_held_note = -1;

// Sequencer playback follows the project's global tempo (p4.bpm_int) — there is
// no separate piano BPM control.
static inline float piano_play_bpm(void) {
    return (p4.bpm_int >= 40) ? (float)p4.bpm_int : 120.0f;
}

// Row 0 = B (top), Row 11 = C (bottom). Matches existing recording mapping.
static const uint8_t PIANO_ROW_TO_PC[12] = {11,10,9,8,7,6,5,4,3,2,1,0};

static uint8_t piano_midi_for_grid_cell(int col, int row, uint8_t fallback_octave) {
    if (col >= 0 && col < 16 && row >= 0 && row < 12 && s_piano_rec_notes[col][row] > 0) {
        return s_piano_rec_notes[col][row];
    }
    int pc = (row >= 0 && row < 12) ? PIANO_ROW_TO_PC[row] : 0;
    int midi = ((int)fallback_octave + 1) * 12 + pc;
    if (midi < 0) midi = 0;
    if (midi > 127) midi = 127;
    return (uint8_t)midi;
}

static void piano_grid_refresh_cell(int col, int row) {
    if (col < 0 || col >= 16 || row < 0 || row >= 12) return;
    lv_obj_t* b = s_piano_grid_btns[col][row];
    if (!b) return;
    bool on      = s_piano_rec_grid[col][row];
    bool playing = (s_piano_play_active && col == s_piano_play_step);
    bool black   = piano_pc_is_black(PIANO_ROW_TO_PC[row]);
    uint32_t color;
    if (on && playing)       color = 0xFFD000;       // yellow — note + playhead
    else if (on)             color = 0x00E5FF;       // cyan — armed note
    else if (playing)        color = 0x303030;       // dim grey — playhead
    else                     color = black ? 0x101820 : 0x222A30;
    lv_obj_set_style_bg_color(b, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(b, on ? LV_OPA_COVER : (playing ? LV_OPA_70 : LV_OPA_60), 0);
}

static void piano_grid_refresh_all(void) {
    for (int c = 0; c < 16; c++)
        for (int r = 0; r < 12; r++)
            piano_grid_refresh_cell(c, r);
}

void piano_grid_visual_refresh_external(void) {
    piano_grid_refresh_all();
}

static void piano_grid_cell_cb(lv_event_t* e) {
    int packed = (int)(intptr_t)lv_event_get_user_data(e);
    int col = (packed >> 8) & 0xFF;
    int row = packed & 0xFF;
    if (col < 0 || col >= 16 || row < 0 || row >= 12) return;
    s_piano_rec_grid[col][row] = !s_piano_rec_grid[col][row];
    s_piano_rec_notes[col][row] = s_piano_rec_grid[col][row]
        ? piano_midi_for_grid_cell(col, row, (uint8_t)s_piano_octave)
        : 0;
    if (s_piano_rec_grid[col][row]) {
        s_piano_rec_engine = PIANO_ENGINES[s_piano_engine_idx];
        s_piano_rec_octave = (uint8_t)s_piano_octave;
        s_piano_rec_has_notes = true;
    }
    piano_grid_refresh_cell(col, row);
}

static void piano_grid_clear(void) {
    memset(s_piano_rec_grid, 0, sizeof(s_piano_rec_grid));
    memset(s_piano_rec_notes, 0, sizeof(s_piano_rec_notes));
    s_piano_rec_step = 0;
    s_piano_rec_has_notes = false;
    piano_grid_refresh_all();
}

static void piano_apply_preset(int idx) {
    memset(s_piano_rec_grid, 0, sizeof(s_piano_rec_grid));
    memset(s_piano_rec_notes, 0, sizeof(s_piano_rec_notes));
    s_piano_rec_engine = PIANO_ENGINES[s_piano_engine_idx];
    s_piano_rec_octave = (uint8_t)s_piano_octave;
    s_piano_rec_has_notes = true;
    // (col, pc) pairs. pc 0=C ... 11=B.
    static const uint8_t PRESET_BASS[][2] = {
        {0,9},{2,9},{4,9},{6,12 % 12},{8,9},{10,7},{12,9},{14,7}
    };
    static const uint8_t PRESET_ARP[][2] = {
        {0,0},{2,4},{4,7},{6,0},{8,4},{10,7},{12,0},{14,7}
    };
    static const uint8_t PRESET_SCALE[][2] = {
        {0,0},{1,2},{2,4},{3,5},{4,7},{5,9},{6,11},{7,0},
        {8,11},{9,9},{10,7},{11,5},{12,4},{13,2},{14,0},{15,7}
    };
    const uint8_t (*p)[2] = NULL;
    int len = 0;
    switch (idx) {
        case 0: p = PRESET_BASS;  len = sizeof(PRESET_BASS)/2;  break;
        case 1: p = PRESET_ARP;   len = sizeof(PRESET_ARP)/2;   break;
        case 2: p = PRESET_SCALE; len = sizeof(PRESET_SCALE)/2; break;
        default: return;
    }
    for (int i = 0; i < len; i++) {
        int col = p[i][0];
        int pc  = p[i][1] % 12;
        for (int r = 0; r < 12; r++) {
            if (PIANO_ROW_TO_PC[r] == (uint8_t)pc) {
                if (col >= 0 && col < 16) {
                    s_piano_rec_grid[col][r] = true;
                    s_piano_rec_notes[col][r] = piano_midi_for_grid_cell(col, r, s_piano_rec_octave);
                }
                break;
            }
        }
    }
    piano_grid_refresh_all();
}

static void piano_preset_btn_cb(lv_event_t* e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    piano_apply_preset(idx);
}

static void piano_clear_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    piano_grid_clear();
    if (ui_use_udp_transport()) udp_send_melody_clear();
}

static void piano_play_step_off(void) {
    if (s_piano_play_held_note < 0) return;
    if (ui_use_udp_transport()) {
        if (PIANO_ENGINES[s_piano_engine_idx] == 3) {
            udp_send_synth303_note_off();
        } else {
            udp_send_synth_note_off(PIANO_ENGINES[s_piano_engine_idx], 0);
        }
    }
    s_piano_play_held_note = -1;
}

static void piano_play_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    s_piano_play_active     = !s_piano_play_active;
    s_piano_play_step       = -1;   // first tick advances to 0
    s_piano_play_next_ms    = millis();
    s_piano_play_off_due_ms = 0;
    piano_play_step_off();
    if (s_piano_play_lbl)
        lv_label_set_text(s_piano_play_lbl, s_piano_play_active ? "■ STOP" : "▶ PLAY");
    if (s_piano_play_btn) {
        lv_obj_set_style_border_color(s_piano_play_btn,
            s_piano_play_active ? lv_color_hex(0x7CFF6B) : RED808_BORDER, 0);
        lv_obj_set_style_border_width(s_piano_play_btn, s_piano_play_active ? 3 : 1, 0);
    }
    if (!s_piano_play_active) piano_grid_refresh_all();
}

void update_piano_screen(void) {
    uint32_t now = millis();
    if (s_piano_release_pending && s_piano_held_note >= 0) {
        if (piano_touch_inside_keys()) {
            piano_cancel_release();
        } else if ((int32_t)(now - s_piano_release_due_ms) >= 0) {
            piano_send_off();
        }
    }
    if (!s_piano_play_active) return;
    uint32_t step_ms = (uint32_t)(60000.0f / piano_play_bpm() / 4.0f); // 16th note
    if (step_ms < 30) step_ms = 30;

    // Note-off scheduling for the currently held note
    if (s_piano_play_held_note >= 0 && now >= s_piano_play_off_due_ms) {
        piano_play_step_off();
    }

    if ((int32_t)(now - s_piano_play_next_ms) < 0) return;
    int prev = s_piano_play_step;
    int next = (prev + 1);
    if (next < 0 || next >= 16) next = 0;
    s_piano_play_step = next;

    if (prev >= 0 && prev < 16)
        for (int r = 0; r < 12; r++) piano_grid_refresh_cell(prev, r);
    for (int r = 0; r < 12; r++) piano_grid_refresh_cell(next, r);

    // Fire the lowest-row hit cell for this step (mono playback)
    for (int r = 11; r >= 0; r--) {
        if (s_piano_rec_grid[next][r]) {
            int midi = piano_midi_for_grid_cell(next, r, (uint8_t)s_piano_octave);
            piano_play_step_off();
            if (ui_use_udp_transport()) {
                udp_send_synth_note_on_ex(PIANO_ENGINES[s_piano_engine_idx],
                                          (uint8_t)midi, s_piano_velocity, false, false);
            }
            s_piano_play_held_note   = midi;
            s_piano_play_off_due_ms  = now + (step_ms * 4 / 5);
            break;
        }
    }
    s_piano_play_next_ms = now + step_ms;
}

// v2.8 — push the locally recorded grid to master as a melodyAssign packet
static void piano_assign_btn_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (!ui_use_udp_transport()) return;
    uint8_t assign_engine = s_piano_rec_has_notes ? s_piano_rec_engine : PIANO_ENGINES[s_piano_engine_idx];
    uint8_t assign_octave = s_piano_rec_has_notes ? s_piano_rec_octave : (uint8_t)s_piano_octave;
    udp_send_melody_assign((uint8_t)s_piano_assign_pad,
                           assign_engine,
                           assign_octave,
                           s_piano_rec_grid,
                           s_piano_rec_notes);
    if (s_piano_status_lbl) {
        lv_label_set_text_fmt(s_piano_status_lbl, "→ PAD %d", s_piano_assign_pad + 1);
    }
    char msg[64];
    snprintf(msg, sizeof(msg), "Melodia asignada a PAD %02d", s_piano_assign_pad + 1);
    ui_show_toast(msg, RED808_SUCCESS);
}

// v2.9 — Apply melody_sync payload from master (engine/octave/rec/pad).
// Must be called from within lvgl_port_lock.
void piano_apply_melody_sync(uint8_t engine, uint8_t octave, bool rec, uint8_t pad) {
    // Map engine value 3..7 → index 0..4
    int new_idx = s_piano_engine_idx;
    for (int i = 0; i < PIANO_ENGINE_COUNT; i++) {
        if (PIANO_ENGINES[i] == engine) { new_idx = i; break; }
    }
    bool octave_changed = ((int)octave != s_piano_octave && octave >= 1 && octave <= 7);
    s_piano_engine_idx = new_idx;
    if (octave_changed) s_piano_octave = (int)octave;
    if (pad < 16) s_piano_assign_pad = (int)pad;
    s_piano_rec_active = rec;

    piano_refresh_engine_chips();
    // Refresh octave label
    if (s_piano_octave_lbl)
        lv_label_set_text_fmt(s_piano_octave_lbl, "OCT %d", s_piano_octave);
    // Rebuild key layout only when octave changes (expensive but necessary)
    if (octave_changed) piano_rebuild_keys();
    // Refresh REC button visual
    if (s_piano_rec_btn) {
        lv_obj_set_style_border_color(s_piano_rec_btn,
            rec ? lv_color_hex(0xFF3030) : RED808_BORDER, 0);
        lv_obj_set_style_border_width(s_piano_rec_btn, rec ? 3 : 1, 0);
    }
    if (s_piano_rec_lbl) {
        lv_obj_set_style_text_color(s_piano_rec_lbl,
            rec ? lv_color_hex(0xFF3030) : RED808_TEXT, 0);
        lv_label_set_text(s_piano_rec_lbl, rec ? "● REC" : "○ REC");
    }
    // Refresh pad label
    if (s_piano_pad_lbl)
        lv_label_set_text_fmt(s_piano_pad_lbl, "PAD %d", s_piano_assign_pad + 1);
    // Refresh status label
    if (s_piano_status_lbl) {
        lv_label_set_text_fmt(s_piano_status_lbl, "ENG %s  OCT %d  PAD %d",
                              PIANO_ENGINE_LABELS[new_idx],
                              s_piano_octave, s_piano_assign_pad + 1);
    }
}

static lv_obj_t* piano_make_chip(lv_obj_t* parent, int x, int y, int w, int h,
                                 const char* text) {
    lv_obj_t* btn = lv_btn_create(parent);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_pos(btn, x, y);
    apply_control_button_style(btn, RED808_BORDER, false, 8);
    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(lbl, RED808_TEXT, 0);
    lv_obj_center(lbl);
    return btn;
}

static void piano_rebuild_keys(void) {
    if (!s_piano_keys_container) return;
    lv_obj_update_layout(s_piano_keys_container);
    lv_obj_clean(s_piano_keys_container);
    memset(s_piano_key_obj_by_note, 0, sizeof(s_piano_key_obj_by_note));
    memset(s_piano_note_active, 0, sizeof(s_piano_note_active));

    int container_w = lv_obj_get_width(s_piano_keys_container);
    int container_h = lv_obj_get_height(s_piano_keys_container);
    if (container_w < 100) container_w = ui_layout_w();
    if (container_h < 100) container_h = ui_layout_h() - 152;
    if (container_h < 160) container_h = 160;
    int num_octaves = s_piano_two_oct ? 2 : 1;
    int num_white   = num_octaves * 7;
    if (num_white < 1) num_white = 1;
    int white_w     = container_w / num_white;
    int white_h     = container_h;
    int black_w     = (white_w * 6) / 10;
    int black_h     = (white_h * 6) / 10;

    static const uint8_t WHITE_PCS[7]         = {0, 2, 4, 5, 7, 9, 11};
    static const uint8_t BLACK_PCS[5]         = {1, 3, 6, 8, 10};
    /* Black key sits between white index N and N+1 (within an octave) */
    static const uint8_t BLACK_AFTER_WHITE[5] = {0, 1, 3, 4, 5};

    int base_midi = (s_piano_octave + 1) * 12;   // C(s_piano_octave) MIDI

    /* White keys */
    for (int oct = 0; oct < num_octaves; oct++) {
        for (int w = 0; w < 7; w++) {
            int x = (oct * 7 + w) * white_w;
            uint8_t midi = base_midi + oct * 12 + WHITE_PCS[w];
            lv_obj_t* k = lv_btn_create(s_piano_keys_container);
            lv_obj_set_pos(k, x, 0);
            lv_obj_set_size(k, white_w - 2, white_h - 2);
            lv_obj_set_style_radius(k, 6, 0);
            lv_obj_set_style_bg_color(k, lv_color_hex(0xF0F0E8), 0);
            lv_obj_set_style_bg_grad_color(k, lv_color_hex(0xB8B8AE), 0);
            lv_obj_set_style_bg_grad_dir(k, LV_GRAD_DIR_VER, 0);
            lv_obj_set_style_bg_opa(k, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(k, 1, 0);
            lv_obj_set_style_border_color(k, lv_color_hex(0x404040), 0);
            lv_obj_set_style_shadow_width(k, 0, 0);
            lv_obj_set_style_bg_color(k, RED808_ACCENT, LV_STATE_PRESSED);
            lv_obj_clear_flag(k, LV_OBJ_FLAG_SCROLLABLE);
            s_piano_key_obj_by_note[midi] = k;
            lv_obj_add_event_cb(k, piano_key_event_cb, LV_EVENT_PRESSED, (void*)(uintptr_t)midi);
            lv_obj_add_event_cb(k, piano_key_event_cb, LV_EVENT_PRESSING, (void*)(uintptr_t)midi);
            lv_obj_add_event_cb(k, piano_key_event_cb, LV_EVENT_RELEASED, (void*)(uintptr_t)midi);
            lv_obj_add_event_cb(k, piano_key_event_cb, LV_EVENT_PRESS_LOST, (void*)(uintptr_t)midi);
            if (w == 0) {
                lv_obj_t* lbl = lv_label_create(k);
                lv_label_set_text_fmt(lbl, "C%d", s_piano_octave + oct);
                lv_obj_set_style_text_color(lbl, lv_color_hex(0x303030), 0);
                lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
                lv_obj_align(lbl, LV_ALIGN_BOTTOM_MID, 0, -4);
            }
        }
    }
    /* Black keys (drawn last so they appear on top) */
    for (int oct = 0; oct < num_octaves; oct++) {
        for (int b = 0; b < 5; b++) {
            int wpos = BLACK_AFTER_WHITE[b];
            int x = (oct * 7 + wpos) * white_w + white_w - black_w / 2;
            uint8_t midi = base_midi + oct * 12 + BLACK_PCS[b];
            lv_obj_t* k = lv_btn_create(s_piano_keys_container);
            lv_obj_set_pos(k, x, 0);
            lv_obj_set_size(k, black_w, black_h);
            lv_obj_set_style_radius(k, 4, 0);
            lv_obj_set_style_bg_color(k, lv_color_hex(0x101010), 0);
            lv_obj_set_style_bg_grad_color(k, lv_color_hex(0x303030), 0);
            lv_obj_set_style_bg_grad_dir(k, LV_GRAD_DIR_VER, 0);
            lv_obj_set_style_bg_opa(k, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(k, 1, 0);
            lv_obj_set_style_border_color(k, lv_color_hex(0x000000), 0);
            lv_obj_set_style_shadow_width(k, 0, 0);
            lv_obj_set_style_bg_color(k, RED808_ACCENT, LV_STATE_PRESSED);
            lv_obj_clear_flag(k, LV_OBJ_FLAG_SCROLLABLE);
            s_piano_key_obj_by_note[midi] = k;
            lv_obj_add_event_cb(k, piano_key_event_cb, LV_EVENT_PRESSED, (void*)(uintptr_t)midi);
            lv_obj_add_event_cb(k, piano_key_event_cb, LV_EVENT_PRESSING, (void*)(uintptr_t)midi);
            lv_obj_add_event_cb(k, piano_key_event_cb, LV_EVENT_RELEASED, (void*)(uintptr_t)midi);
            lv_obj_add_event_cb(k, piano_key_event_cb, LV_EVENT_PRESS_LOST, (void*)(uintptr_t)midi);
        }
    }
}

static void create_piano_screen(void) {
    int W = ui_layout_w();
    int H = ui_layout_h();
    scr_piano = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_piano, RED808_BG, 0);
    lv_obj_clear_flag(scr_piano, LV_OBJ_FLAG_SCROLLABLE);

    /* Floating back button (top-left, lands back on LIVE) */
    ui_create_header(scr_piano);

    /* Title */
    lv_obj_t* title = lv_label_create(scr_piano);
    lv_label_set_text(title, "PIANO");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, RED808_TEXT, 0);
    lv_obj_set_pos(title, 64, 12);

    /* Engine selector buttons (top-right) */
    int eng_w = 80, eng_h = 36, eng_gap = 6;
    int eng_x_start = W - (eng_w + eng_gap) * PIANO_ENGINE_COUNT - 12;
    for (int i = 0; i < PIANO_ENGINE_COUNT; i++) {
        lv_obj_t* btn = piano_make_chip(scr_piano,
            eng_x_start + i * (eng_w + eng_gap), 8,
            eng_w, eng_h, PIANO_ENGINE_LABELS[i]);
        if (i == s_piano_engine_idx) {
            lv_obj_set_style_bg_color(btn, RED808_ACCENT, 0);
            lv_obj_set_style_border_color(btn, RED808_ACCENT2, 0);
        }
        lv_obj_add_event_cb(btn, piano_engine_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        s_piano_engine_btns[i] = btn;
    }

    /* Row 1 (y=56) — octave / keys / gesture / engine-preset / params / rec */
    int row_y = 56;
    lv_obj_t* oct_minus = piano_make_chip(scr_piano, 12, row_y, 58, 36, "-");
    lv_obj_add_event_cb(oct_minus, piano_octave_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    s_piano_octave_lbl = lv_label_create(scr_piano);
    lv_label_set_text_fmt(s_piano_octave_lbl, "OCT %d", s_piano_octave);
    lv_obj_set_style_text_font(s_piano_octave_lbl, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(s_piano_octave_lbl, RED808_TEXT, 0);
    lv_obj_set_pos(s_piano_octave_lbl, 78, row_y + 8);
    lv_obj_t* oct_plus = piano_make_chip(scr_piano, 134, row_y, 58, 36, "+");
    lv_obj_add_event_cb(oct_plus, piano_octave_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)+1);

    s_piano_keys24_btn = piano_make_chip(scr_piano, 200, row_y, 70, 36,
                                          s_piano_two_oct ? "24 K" : "12 K");
    s_piano_keys24_lbl = lv_obj_get_child(s_piano_keys24_btn, 0);
    lv_obj_add_event_cb(s_piano_keys24_btn, piano_keys24_btn_cb, LV_EVENT_CLICKED, NULL);

    /* Gesture controls (glide / bend) — bend is fixed at ±2 semitones */
    s_piano_glide_btn = piano_make_chip(scr_piano, 280, row_y, 96, 36,
                                        s_piano_glide_enabled ? "GLIDE ON" : "GLIDE OFF");
    s_piano_glide_lbl = lv_obj_get_child(s_piano_glide_btn, 0);
    lv_obj_add_event_cb(s_piano_glide_btn, piano_glide_btn_cb, LV_EVENT_CLICKED, NULL);

    s_piano_bend_btn = piano_make_chip(scr_piano, 384, row_y, 92, 36,
                                       s_piano_bend_enabled ? "BEND ON" : "BEND OFF");
    s_piano_bend_lbl = lv_obj_get_child(s_piano_bend_btn, 0);
    lv_obj_add_event_cb(s_piano_bend_btn, piano_bend_btn_cb, LV_EVENT_CLICKED, NULL);
    piano_refresh_gesture_controls();

    /* Quick synth-preset cycler for the ACTIVE engine — no need to open PARAMS */
    s_piano_eng_preset_btn = piano_make_chip(scr_piano, 488, row_y, 184, 36, "PRESET");
    lv_obj_set_style_border_color(s_piano_eng_preset_btn, lv_color_hex(0xFFB300), 0);
    s_piano_eng_preset_lbl = lv_obj_get_child(s_piano_eng_preset_btn, 0);
    if (s_piano_eng_preset_lbl)
        lv_obj_set_style_text_color(s_piano_eng_preset_lbl, lv_color_hex(0xFFB300), 0);
    lv_obj_add_event_cb(s_piano_eng_preset_btn, piano_engine_preset_cb, LV_EVENT_CLICKED, NULL);
    piano_refresh_engine_preset_label();

    /* PARAMS button → synth parameter editor screen (id 11) */
    {
        lv_obj_t* pbtn = piano_make_chip(scr_piano, 680, row_y, 92, 36, "PARAMS");
        lv_obj_set_style_border_color(pbtn, lv_color_hex(0x00E5FF), 0);
        lv_obj_t* plbl = lv_obj_get_child(pbtn, 0);
        if (plbl) lv_obj_set_style_text_color(plbl, lv_color_hex(0x00E5FF), 0);
        lv_obj_add_event_cb(pbtn, [](lv_event_t* e){ (void)e; ui_navigate_to(11); },
                             LV_EVENT_CLICKED, NULL);
    }

    /* v2.7 — REC toggle: sends melodyRecNote to master while active */
    s_piano_rec_btn = piano_make_chip(scr_piano, 780, row_y, 104, 36, "○ REC");
    s_piano_rec_lbl = lv_obj_get_child(s_piano_rec_btn, 0);
    lv_obj_add_event_cb(s_piano_rec_btn, piano_rec_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(s_piano_rec_btn, LV_OBJ_FLAG_PRESS_LOCK);
    if (s_piano_rec_active) {
        lv_obj_set_style_border_color(s_piano_rec_btn, lv_color_hex(0xFF3030), 0);
        lv_obj_set_style_border_width(s_piano_rec_btn, 3, 0);
        if (s_piano_rec_lbl) {
            lv_obj_set_style_text_color(s_piano_rec_lbl, lv_color_hex(0xFF3030), 0);
            lv_label_set_text(s_piano_rec_lbl, "● REC");
        }
    }

    s_piano_status_lbl = lv_label_create(scr_piano);
    lv_label_set_text(s_piano_status_lbl, "—");
    lv_obj_set_style_text_font(s_piano_status_lbl, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(s_piano_status_lbl, RED808_ACCENT, 0);
    lv_obj_set_pos(s_piano_status_lbl, 898, row_y + 8);

    lv_obj_t* expr_track = lv_obj_create(scr_piano);
    lv_obj_set_size(expr_track, 14, H - 168);
    lv_obj_set_pos(expr_track, W - 24, 150);
    lv_obj_clear_flag(expr_track, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(expr_track, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_radius(expr_track, 7, 0);
    lv_obj_set_style_bg_color(expr_track, RED808_SURFACE, 0);
    lv_obj_set_style_border_color(expr_track, RED808_BORDER, 0);
    lv_obj_set_style_border_width(expr_track, 1, 0);
    lv_obj_set_style_pad_all(expr_track, 0, 0);

    s_piano_expr_bar = lv_obj_create(expr_track);
    lv_obj_set_size(s_piano_expr_bar, 10, 8);
    lv_obj_clear_flag(s_piano_expr_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_piano_expr_bar, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_radius(s_piano_expr_bar, 5, 0);
    lv_obj_set_style_bg_color(s_piano_expr_bar, lv_color_hex(0x00E5FF), 0);
    lv_obj_set_style_bg_grad_color(s_piano_expr_bar, lv_color_hex(0x7CFFB2), 0);
    lv_obj_set_style_bg_grad_dir(s_piano_expr_bar, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(s_piano_expr_bar, 0, 0);
    piano_update_expression_bar();

    /* v2.8/v2.9 — compact melody row: pad assign + presets + transport */
    int row_y2 = 104;
    int x_cursor = 12;
    lv_obj_t* pad_minus = piano_make_chip(scr_piano, x_cursor, row_y2, 54, 36, "PAD-");
    lv_obj_set_style_border_color(pad_minus, lv_color_hex(0xFF1493), 0);
    lv_obj_add_event_cb(pad_minus, piano_pad_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
    x_cursor += 54 + 4;
    s_piano_pad_lbl = lv_label_create(scr_piano);
    lv_label_set_text_fmt(s_piano_pad_lbl, "PAD %d", s_piano_assign_pad + 1);
    lv_obj_set_style_text_font(s_piano_pad_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_piano_pad_lbl, lv_color_hex(0xFF1493), 0);
    lv_obj_set_pos(s_piano_pad_lbl, x_cursor, row_y2 + 9);
    x_cursor += 70 + 4;
    lv_obj_t* pad_plus = piano_make_chip(scr_piano, x_cursor, row_y2, 54, 36, "PAD+");
    lv_obj_set_style_border_color(pad_plus, lv_color_hex(0xFF1493), 0);
    lv_obj_add_event_cb(pad_plus, piano_pad_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)+1);
    x_cursor += 54 + 6;
    {
        lv_obj_t* assign = piano_make_chip(scr_piano, x_cursor, row_y2, 90, 36, "ASSIGN");
        lv_obj_set_style_border_color(assign, lv_color_hex(0xFF1493), 0);
        lv_obj_t* l = lv_obj_get_child(assign, 0);
        if (l) lv_obj_set_style_text_color(l, lv_color_hex(0xFF1493), 0);
        lv_obj_add_event_cb(assign, piano_assign_btn_cb, LV_EVENT_CLICKED, NULL);
    }
    x_cursor += 90 + 10;

    /* Melody-grid pattern presets + clear + transport. Velocity comes from the
       key press (touch area) and tempo follows the global BPM, so neither has a
       button here. */
    {
        struct PresetCfg { const char* lbl; int idx; uint32_t color; };
        const PresetCfg presets[3] = {
            { "BASS",  0, 0xFFE066 },
            { "ARP",   1, 0x7CFF6B },
            { "SCALE", 2, 0x00E5FF },
        };
        for (int i = 0; i < 3; i++) {
            lv_obj_t* b = piano_make_chip(scr_piano, x_cursor, row_y2, 64, 36, presets[i].lbl);
            lv_obj_set_style_border_color(b, lv_color_hex(presets[i].color), 0);
            lv_obj_t* l = lv_obj_get_child(b, 0);
            if (l) lv_obj_set_style_text_color(l, lv_color_hex(presets[i].color), 0);
            lv_obj_add_event_cb(b, piano_preset_btn_cb, LV_EVENT_CLICKED, (void*)(intptr_t)presets[i].idx);
            x_cursor += 64 + 4;
        }
        lv_obj_t* clr = piano_make_chip(scr_piano, x_cursor, row_y2, 64, 36, "CLEAR");
        lv_obj_set_style_border_color(clr, lv_color_hex(0xFF6060), 0);
        lv_obj_add_event_cb(clr, piano_clear_btn_cb, LV_EVENT_CLICKED, NULL);
        x_cursor += 64 + 8;

        s_piano_play_btn = piano_make_chip(scr_piano, x_cursor, row_y2, 96, 36, "PLAY");
        s_piano_play_lbl = lv_obj_get_child(s_piano_play_btn, 0);
        lv_obj_set_style_border_color(s_piano_play_btn, lv_color_hex(0x7CFF6B), 0);
        if (s_piano_play_lbl) lv_obj_set_style_text_color(s_piano_play_lbl, lv_color_hex(0x7CFF6B), 0);
        lv_obj_add_event_cb(s_piano_play_btn, piano_play_btn_cb, LV_EVENT_CLICKED, NULL);
    }

    /* Melody grid container: 16 cols × 12 rows pitch grid */
    int grid_y = 148;
    int grid_h = 184;
    s_piano_grid_container = lv_obj_create(scr_piano);
    lv_obj_set_pos(s_piano_grid_container, 0, grid_y);
    lv_obj_set_size(s_piano_grid_container, W, grid_h);
    lv_obj_set_style_bg_color(s_piano_grid_container, RED808_PANEL, 0);
    lv_obj_set_style_bg_opa(s_piano_grid_container, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_piano_grid_container, 0, 0);
    lv_obj_set_style_pad_all(s_piano_grid_container, 0, 0);
    lv_obj_clear_flag(s_piano_grid_container, LV_OBJ_FLAG_SCROLLABLE);
    {
        int gx0 = 8;
        int gy0 = 4;
        int gw  = W - 16;
        int gh  = grid_h - 8;
        int cw  = gw / 16;
        int ch  = gh / 12;
        for (int c = 0; c < 16; c++) {
            for (int r = 0; r < 12; r++) {
                lv_obj_t* cell = lv_btn_create(s_piano_grid_container);
                lv_obj_set_pos(cell, gx0 + c * cw + 1, gy0 + r * ch + 1);
                lv_obj_set_size(cell, cw - 2, ch - 2);
                lv_obj_set_style_radius(cell, 3, 0);
                lv_obj_set_style_border_width(cell, 1, 0);
                lv_obj_set_style_border_color(cell, RED808_BORDER, 0);
                lv_obj_set_style_shadow_width(cell, 0, 0);
                int packed = (c << 8) | r;
                lv_obj_add_event_cb(cell, piano_grid_cell_cb, LV_EVENT_CLICKED, (void*)(intptr_t)packed);
                s_piano_grid_btns[c][r] = cell;
            }
        }
        piano_grid_refresh_all();
    }

    /* Keys area (bottom half) */
    int keys_y = grid_y + grid_h + 8;
    int keys_h = H - keys_y - 8;
    s_piano_keys_container = lv_obj_create(scr_piano);
    lv_obj_set_pos(s_piano_keys_container, 0, keys_y);
    lv_obj_set_size(s_piano_keys_container, W, keys_h);
    lv_obj_set_style_bg_color(s_piano_keys_container, RED808_PANEL, 0);
    lv_obj_set_style_bg_opa(s_piano_keys_container, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_piano_keys_container, 0, 0);
    lv_obj_set_style_pad_all(s_piano_keys_container, 0, 0);
    lv_obj_clear_flag(s_piano_keys_container, LV_OBJ_FLAG_SCROLLABLE);

    piano_rebuild_keys();
}

// =============================================================================
// PIANO PARAMS SCREEN — synth engine parameter editor (303/WT/SH101/FM2)
// v2.7 — mirrors web synth-editor with engine tabs + presets + sliders.
// =============================================================================
#include "../../../shared/synth_params.h"

#define PP_GRID_COLS_P4   3
#define PP_GRID_ROWS_P4   7
#define PP_MAX_PARAMS_P4  21

static int       s_pp_engine_idx = 0;
static int       s_pp_preset_idx[SP_ENGINE_COUNT] = { -1, -1, -1, -1, -1 };
static float     s_pp_values[SP_ENGINE_COUNT][PP_MAX_PARAMS_P4] = {};
static lv_obj_t* s_pp_engine_btns[SP_ENGINE_COUNT] = {};
static lv_obj_t* s_pp_preset_btns[4] = {};
static lv_obj_t* s_pp_param_panel    = NULL;
static lv_obj_t* s_pp_sliders[PP_MAX_PARAMS_P4] = {};
static lv_obj_t* s_pp_val_lbls[PP_MAX_PARAMS_P4] = {};
static lv_obj_t* s_pp_title_lbl = NULL;
static lv_obj_t* s_pp_wave_card = NULL;
static lv_obj_t* s_pp_wave_line = NULL;
static lv_obj_t* s_pp_wave_lbl = NULL;
static lv_point_precise_t s_pp_wave_points[96] = {};
static bool      s_pp_from_xtra = false;
static int       s_pp_xtra_slot = -1;

static void xtra_capture_editor_state(int slot) {
    if (slot < 0 || slot >= 4) return;
    int eng_idx = xtra_slot_pp_engine_idx(slot);
    if (eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) return;
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    for (uint8_t i = 0; i < eng->param_count && i < XTRA_PARAM_MAX; i++) {
        s_xtra_param_values[slot][i] = s_pp_values[eng_idx][i];
    }
    s_xtra_param_valid[slot] = true;
    xtra_save_param_state();
}

static void xtra_load_editor_state(int slot) {
    if (slot < 0 || slot >= 4) return;
    int eng_idx = xtra_slot_pp_engine_idx(slot);
    if (eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) return;
    if (!s_xtra_param_valid[slot]) xtra_reset_slot_params(slot);
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    for (uint8_t i = 0; i < eng->param_count && i < XTRA_PARAM_MAX; i++) {
        s_pp_values[eng_idx][i] = s_xtra_param_values[slot][i];
    }
}

static float pp_param_value_or_default(int eng_idx, uint8_t param_id, float fallback) {
    if (eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) return fallback;
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    for (uint8_t i = 0; i < eng->param_count && i < PP_MAX_PARAMS_P4; i++) {
        if (eng->params[i].param_id == param_id) return s_pp_values[eng_idx][i];
    }
    return fallback;
}

static void pp_refresh_wave_preview(void) {
    if (!s_pp_wave_line || !s_pp_wave_lbl) return;
    const int count = (int)(sizeof(s_pp_wave_points) / sizeof(s_pp_wave_points[0]));
    const int width = 300;
    const int height = 68;
    const int mid = height / 2;
    const uint8_t engine = SP_ENGINES[s_pp_engine_idx].engine;
    const float cutoffNorm = constrain(pp_param_value_or_default(s_pp_engine_idx, 4, 8000.0f) / 18000.0f, 0.05f, 1.0f);
    const float modA = constrain(pp_param_value_or_default(s_pp_engine_idx, 1, 0.5f), 0.0f, 1.0f);
    const float modB = constrain(pp_param_value_or_default(s_pp_engine_idx, 9, 0.25f), 0.0f, 12.0f);
    const float shape = constrain(pp_param_value_or_default(s_pp_engine_idx, 0, 0.0f), 0.0f, 8.0f);

    for (int i = 0; i < count; i++) {
        float t = (float)i / (float)(count - 1);
        float y = 0.0f;
        switch (engine) {
            case SP_ENGINE_303: {
                float phase = t + shape * 0.03f;
                float saw = 2.0f * (phase - floorf(phase + 0.5f));
                float sq = (sinf(2.0f * PI * phase) >= 0.0f) ? 1.0f : -1.0f;
                float mix = constrain(pp_param_value_or_default(s_pp_engine_idx, 6, 0.0f), 0.0f, 1.0f);
                y = saw * (1.0f - mix) + sq * mix * (0.65f + modA * 0.35f);
                break;
            }
            case SP_ENGINE_WT:
                y = 0.58f * sinf(2.0f * PI * t) + 0.24f * sinf(4.0f * PI * t + shape * 0.4f) + 0.18f * sinf(6.0f * PI * t + shape * 0.85f);
                break;
            case SP_ENGINE_SH101: {
                float pwm = 0.1f + modA * 0.8f;
                float phase = t + shape * 0.02f;
                y = (fmodf(phase, 1.0f) < pwm) ? 1.0f : -1.0f;
                y *= 0.75f + cutoffNorm * 0.25f;
                break;
            }
            case SP_ENGINE_FM2OP: {
                float ratio = pp_param_value_or_default(s_pp_engine_idx, 8, 2.0f);
                float idx = constrain(modB / 12.0f, 0.0f, 1.0f) * 8.0f;
                y = sinf(2.0f * PI * t + idx * sinf(2.0f * PI * t * ratio));
                break;
            }
            case SP_ENGINE_PHYS: {
                float bright = constrain(pp_param_value_or_default(s_pp_engine_idx, 7, 0.64f), 0.0f, 1.0f);
                float damp = constrain(pp_param_value_or_default(s_pp_engine_idx, 3, 0.78f), 0.0f, 1.0f);
                float env = expf(-t * (1.0f + damp * 5.0f));
                y = env * (sinf(2.0f * PI * t * (1.2f + bright * 2.8f)) + 0.35f * sinf(2.0f * PI * t * (4.0f + bright * 6.0f)));
                break;
            }
            default:
                y = sinf(2.0f * PI * t);
                break;
        }
        y *= 0.85f;
        s_pp_wave_points[i].x = (lv_coord_t)((i * width) / (count - 1));
        s_pp_wave_points[i].y = (lv_coord_t)(mid - y * (mid - 8));
    }

    lv_line_set_points(s_pp_wave_line, s_pp_wave_points, count);
    if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
        lv_label_set_text_fmt(s_pp_wave_lbl, "Preview XTRA · slot %d · engine %s", s_pp_xtra_slot + 1, SP_ENGINES[s_pp_engine_idx].label);
    } else {
        lv_label_set_text_fmt(s_pp_wave_lbl, "Preview synth · engine %s", SP_ENGINES[s_pp_engine_idx].label);
    }
}

static int pp_engine_idx_from_code(uint8_t engine) {
    for (int i = 0; i < SP_ENGINE_COUNT; i++) {
        if (SP_ENGINES[i].engine == engine) return i;
    }
    return -1;
}

static uint8_t xtra_engine_idx_from_pp_engine(int pp_idx) {
    if (pp_idx < 0 || pp_idx >= SP_ENGINE_COUNT) return 3;
    switch (SP_ENGINES[pp_idx].engine) {
        case 3: return 3;
        case 4: return 4;
        case 5: return 5;
        case 6: return 6;
        case 7: return 7;
        default: return 3;
    }
}

static lv_color_t pp_engine_color(int idx) {
    static const uint32_t colors[SP_ENGINE_COUNT] = {
        0x00E5FF, 0xFF1493, 0xFFE066, 0x7CFF6B, 0xFF8C42
    };
    if (idx < 0 || idx >= SP_ENGINE_COUNT) return RED808_CYAN;
    return lv_color_hex(colors[idx]);
}

static inline int pp_f2i(float v, float vmin, float vmax) {
    if (vmax <= vmin) return 0;
    float t = (v - vmin) / (vmax - vmin);
    if (t < 0) t = 0; if (t > 1) t = 1;
    return (int)(t * 1000.0f + 0.5f);
}
static inline float pp_i2f(int i, float vmin, float vmax) {
    return vmin + (vmax - vmin) * (float)i / 1000.0f;
}

static void pp_format_value(char* buf, size_t bufsz, const SynthParamDef* p, float v) {
    if (p->step_int) {
        snprintf(buf, bufsz, "%d%s%s", (int)(v + 0.5f), p->unit[0] ? " " : "", p->unit);
    } else if (p->vmax >= 100.f) {
        snprintf(buf, bufsz, "%.0f%s%s", v, p->unit[0] ? " " : "", p->unit);
    } else if (p->vmax >= 10.f) {
        snprintf(buf, bufsz, "%.2f%s%s", v, p->unit[0] ? " " : "", p->unit);
    } else {
        snprintf(buf, bufsz, "%.3f%s%s", v, p->unit[0] ? " " : "", p->unit);
    }
}

static void pp_init_engine_defaults(int eng_idx) {
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    for (uint8_t i = 0; i < eng->param_count && i < PP_MAX_PARAMS_P4; i++) {
        s_pp_values[eng_idx][i] = eng->params[i].vdef;
    }
}

static void piano_sync_active_engine_state(void) {
    if (!ui_use_udp_transport()) return;
    int eng_idx = s_piano_engine_idx;
    if (eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) return;
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    int preset_idx = s_pp_preset_idx[eng_idx];
    uint8_t packets = 0;
    if (preset_idx >= 0 && preset_idx < eng->preset_count) {
        udp_send_synth_preset(eng->engine, (uint8_t)preset_idx);
        packets++;
    }
    piano_apply_glide_setting();
    if (eng->engine == SP_ENGINE_WT) {
        piano_reset_vertical_expression();
        packets += 2;
    }
    (void)packets;
#if P4_ENABLE_DEBUG_LOG
    Serial.printf("[P4 piano] sync eng=%u preset=%d packets~%u\n",
                  eng->engine, preset_idx, packets);
#endif
}

static bool piano_vertical_expression_active(void) {
    return piano_engine_code() == SP_ENGINE_WT && !s_piano_glide_enabled && !s_piano_bend_enabled;
}

static float piano_get_wt_cutoff_base(void) {
    const SynthEngineDef* eng = &SP_ENGINES[1];
    for (uint8_t i = 0; i < eng->param_count && i < PP_MAX_PARAMS_P4; i++) {
        if (eng->params[i].param_id == 4) {
            float v = s_pp_values[1][i];
            if (v >= eng->params[i].vmin && v <= eng->params[i].vmax) return v;
            return eng->params[i].vdef;
        }
    }
    return 8000.0f;
}

static float piano_get_wt_volume_base(void) {
    const SynthEngineDef* eng = &SP_ENGINES[1];
    for (uint8_t i = 0; i < eng->param_count && i < PP_MAX_PARAMS_P4; i++) {
        if (eng->params[i].param_id == 3) {
            float v = s_pp_values[1][i];
            if (v >= eng->params[i].vmin && v <= eng->params[i].vmax) return v;
            return eng->params[i].vdef;
        }
    }
    return 0.75f;
}

static void piano_update_status_note(uint8_t midi_note) {
    if (!s_piano_status_lbl) return;
    lv_label_set_text_fmt(s_piano_status_lbl, "%s · %s",
                          PIANO_ENGINE_LABELS[s_piano_engine_idx],
                          piano_note_name(midi_note));
}

static void piano_update_expression_status(void) {
    if (!s_piano_status_lbl) return;
    if (s_piano_held_note < 0) {
        lv_label_set_text(s_piano_status_lbl, "—");
        return;
    }
    int amount = (int)lroundf(s_piano_expr_last_amount * 100.0f);
    lv_label_set_text_fmt(s_piano_status_lbl, "%s · %s · EXP %d%%",
                          PIANO_ENGINE_LABELS[s_piano_engine_idx],
                          piano_note_name((uint8_t)s_piano_held_note),
                          amount);
}

static void piano_update_expression_bar(void) {
    if (!s_piano_expr_bar) return;
    int container_h = s_piano_keys_container ? lv_obj_get_height(s_piano_keys_container) : 0;
    if (container_h < 120) container_h = ui_layout_h() - 152;
    if (container_h < 120) container_h = 120;
    int fill_h = (int)(s_piano_expr_last_amount * (float)(container_h - 8) + 0.5f);
    if (fill_h < 8) fill_h = 8;
    lv_obj_set_height(s_piano_expr_bar, fill_h);
    lv_obj_align(s_piano_expr_bar, LV_ALIGN_BOTTOM_MID, 0, -2);
    lv_obj_set_style_bg_opa(s_piano_expr_bar,
                            s_piano_expr_last_amount > 0.01f ? LV_OPA_COVER : LV_OPA_30,
                            0);
}

static void piano_apply_vertical_expression(int16_t ly) {
    if (!ui_use_udp_transport() || !piano_vertical_expression_active()) return;
    const float ceil_cutoff = 18000.0f;
    const float max_volume_boost = 0.18f;
    const float dead_zone_px = 5.0f;
    const float full_scale_px = 80.0f;
    float dy = (float)(s_piano_touch_start_y - ly);
    float target = s_piano_expr_base_cutoff;
    float target_volume = s_piano_expr_base_volume;
    if (dy > dead_zone_px) {
        float t = (dy - dead_zone_px) / (full_scale_px - dead_zone_px);
        if (t > 1.0f) t = 1.0f;
        s_piano_expr_last_amount = t;
        target = s_piano_expr_base_cutoff + t * (ceil_cutoff - s_piano_expr_base_cutoff);
        target_volume = s_piano_expr_base_volume + t * max_volume_boost;
        if (target_volume > 1.0f) target_volume = 1.0f;
    } else {
        s_piano_expr_last_amount = 0.0f;
    }
    uint32_t now = millis();
    if ((now - s_piano_expr_last_send_ms) < 10 &&
        fabsf(target - s_piano_expr_last_cutoff) < 80.0f &&
        fabsf(target_volume - s_piano_expr_last_volume) < 0.015f) {
        return;
    }
    s_piano_expr_last_cutoff = target;
    s_piano_expr_last_volume = target_volume;
    s_piano_expr_last_send_ms = now;
    udp_send_synth_param(SP_ENGINE_WT, 0, 4, target);
    udp_send_synth_param(SP_ENGINE_WT, 0, 3, target_volume);
    piano_update_expression_status();
    piano_update_expression_bar();
}

static void piano_reset_vertical_expression(void) {
    if (!ui_use_udp_transport()) return;
    float base = piano_get_wt_cutoff_base();
    float base_volume = piano_get_wt_volume_base();
    s_piano_expr_base_cutoff = base;
    s_piano_expr_last_cutoff = base;
    s_piano_expr_base_volume = base_volume;
    s_piano_expr_last_volume = base_volume;
    s_piano_expr_last_amount = 0.0f;
    s_piano_expr_last_send_ms = 0;
    if (piano_engine_code() == SP_ENGINE_WT) {
        udp_send_synth_param(SP_ENGINE_WT, 0, 4, base);
        udp_send_synth_param(SP_ENGINE_WT, 0, 3, base_volume);
    }
    if (s_piano_held_note >= 0) {
        piano_update_status_note((uint8_t)s_piano_held_note);
    }
    piano_update_expression_bar();
}

static void pp_apply_preset_local(int eng_idx, int preset_idx) {
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    if (preset_idx < 0 || preset_idx >= eng->preset_count) return;
    pp_init_engine_defaults(eng_idx);
    const SynthPreset* pr = &eng->presets[preset_idx];
    for (uint8_t pv = 0; pv < pr->count; pv++) {
        for (uint8_t i = 0; i < eng->param_count; i++) {
            if (eng->params[i].param_id == pr->values[pv].param_id) {
                s_pp_values[eng_idx][i] = pr->values[pv].value;
                break;
            }
        }
    }
}

static void pp_slider_event_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    const SynthEngineDef* eng = &SP_ENGINES[s_pp_engine_idx];
    if (slot < 0 || slot >= (int)eng->param_count) return;
    const SynthParamDef* p = &eng->params[slot];
    lv_obj_t* sl = (lv_obj_t*)lv_event_get_target(e);
    int iv = lv_slider_get_value(sl);
    float fv = pp_i2f(iv, p->vmin, p->vmax);
    if (p->step_int) fv = (float)((int)(fv + 0.5f));
    s_pp_values[s_pp_engine_idx][slot] = fv;
    if (s_pp_val_lbls[slot]) {
        char buf[24];
        pp_format_value(buf, sizeof(buf), p, fv);
        lv_label_set_text(s_pp_val_lbls[slot], buf);
    }
    if (ui_use_udp_transport()) {
        udp_send_synth_param(eng->engine, 0, p->param_id, fv);
    }
    if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
        xtra_capture_editor_state(s_pp_xtra_slot);
    }
    pp_refresh_wave_preview();
}

// v2.9 — Hold-to-increment with cell background tracking the value.
// Press = step once. Hold = keep stepping (~10/s). RST badge resets.
static lv_obj_t* s_pp_cell_bars[PP_MAX_PARAMS_P4] = {};

static inline lv_color_t pp_value_color(float t) {
    if (t < 0) t = 0; if (t > 1) t = 1;
    uint8_t r = (uint8_t)(0x0A + t * (0xFF - 0x0A));
    uint8_t g = (uint8_t)(0x18 + t * (0x14 - 0x18));
    uint8_t b = (uint8_t)(0x40 + t * (0x93 - 0x40));
    return lv_color_make(r, g, b);
}

static void pp_cell_redraw_value(int slot) {
    const SynthEngineDef* eng = &SP_ENGINES[s_pp_engine_idx];
    if (slot < 0 || slot >= (int)eng->param_count) return;
    const SynthParamDef* p = &eng->params[slot];
    float fv = s_pp_values[s_pp_engine_idx][slot];
    float t = (fv - p->vmin) / (p->vmax - p->vmin);
    if (t < 0) t = 0; if (t > 1) t = 1;
    if (s_pp_val_lbls[slot]) {
        char buf[24];
        pp_format_value(buf, sizeof(buf), p, fv);
        lv_label_set_text(s_pp_val_lbls[slot], buf);
    }
    if (s_pp_cell_bars[slot]) {
        int full_w = (int)(intptr_t)lv_obj_get_user_data(s_pp_cell_bars[slot]);
        int w = (int)(full_w * t + 0.5f);
        if (w < 4) w = 4;
        lv_obj_set_width(s_pp_cell_bars[slot], w);
        uint8_t r = (uint8_t)(0x00 + t * (0xFF - 0x00));
        uint8_t g = (uint8_t)(0xE5 - t * (0xE5 - 0x14));
        uint8_t b = (uint8_t)(0xFF - t * (0xFF - 0x93));
        lv_obj_set_style_bg_color(s_pp_cell_bars[slot], lv_color_make(r, g, b), 0);
    }
    if (s_pp_sliders[slot]) {
        lv_color_t bg = pp_value_color(t);
        lv_obj_set_style_bg_color(s_pp_sliders[slot], bg, 0);
        uint8_t r = (uint8_t)(0x00 + t * (0xFF - 0x00));
        uint8_t g = (uint8_t)(0xE5 - t * (0xE5 - 0x14));
        uint8_t b = (uint8_t)(0xFF - t * (0xFF - 0x93));
        lv_obj_set_style_border_color(s_pp_sliders[slot], lv_color_make(r, g, b), 0);
    }
}

static void pp_cell_step(int slot) {
    const SynthEngineDef* eng = &SP_ENGINES[s_pp_engine_idx];
    if (slot < 0 || slot >= (int)eng->param_count) return;
    const SynthParamDef* p = &eng->params[slot];
    float fv = s_pp_values[s_pp_engine_idx][slot];
    float range = p->vmax - p->vmin;
    if (range <= 0.f) return;
    float step;
    if (p->step_int && range <= 20.f) {
        step = 1.f;
    } else if (p->step_int) {
        step = range * 0.04f;
    } else {
        step = range * 0.03f;
    }
    fv += step;
    if (fv > p->vmax + 1e-3f) fv = p->vmin;
    if (p->step_int) fv = (float)((int)(fv + 0.5f));
    s_pp_values[s_pp_engine_idx][slot] = fv;
    pp_cell_redraw_value(slot);
    if (ui_use_udp_transport()) {
        udp_send_synth_param(eng->engine, 0, p->param_id, fv);
    }
    if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
        xtra_capture_editor_state(s_pp_xtra_slot);
    }
    pp_refresh_wave_preview();
}

static void pp_cell_press_cb(lv_event_t* e) {
    pp_cell_step((int)(intptr_t)lv_event_get_user_data(e));
}

static void pp_cell_long_repeat_cb(lv_event_t* e) {
    pp_cell_step((int)(intptr_t)lv_event_get_user_data(e));
}

static void pp_cell_reset_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    const SynthEngineDef* eng = &SP_ENGINES[s_pp_engine_idx];
    if (slot < 0 || slot >= (int)eng->param_count) return;
    const SynthParamDef* p = &eng->params[slot];
    s_pp_values[s_pp_engine_idx][slot] = p->vdef;
    pp_cell_redraw_value(slot);
    if (ui_use_udp_transport()) {
        udp_send_synth_param(eng->engine, 0, p->param_id, p->vdef);
    }
    if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
        xtra_capture_editor_state(s_pp_xtra_slot);
    }
    pp_refresh_wave_preview();
}

static void pp_update_engine_chips(void);
static void pp_update_preset_chips(void);

static void pp_rebuild_param_grid(void) {
    if (!s_pp_param_panel) return;
    lv_obj_clean(s_pp_param_panel);
    for (int i = 0; i < PP_MAX_PARAMS_P4; i++) {
        s_pp_sliders[i] = NULL;
        s_pp_val_lbls[i] = NULL;
        s_pp_cell_bars[i] = NULL;
    }
    const SynthEngineDef* eng = &SP_ENGINES[s_pp_engine_idx];
    int count = eng->param_count;

    // Use the screen geometry directly — lv_obj_get_width() can return 0 when
    // called before LVGL has laid out the panel after creation.
    int W = ui_layout_w();
    int H = ui_layout_h();
    int panel_w = W - 24;
    int panel_h = H - 152 - 12;

    int cols = PP_GRID_COLS_P4;
    int rows = (count + cols - 1) / cols;
    if (rows < 1) rows = 1;
    int gap = 6;
    int pad = 6;
    int cell_w = (panel_w - 2 * pad - (cols - 1) * gap) / cols;
    int cell_h = (panel_h - 2 * pad - (rows - 1) * gap) / rows;
    if (cell_h < 52) cell_h = 52;

    // Hint label on top of panel telling user the interaction model.
    // We place this OUTSIDE the cells so it doesn't fight for vertical space.

    for (int i = 0; i < count && i < PP_MAX_PARAMS_P4; i++) {
        int col = i % cols;
        int row = i / cols;
        int x = pad + col * (cell_w + gap);
        int y = pad + row * (cell_h + gap);
        const SynthParamDef* p = &eng->params[i];

        // Whole-cell button — gives big touch target, works for portrait LCD.
        lv_obj_t* cell = lv_btn_create(s_pp_param_panel);
        lv_obj_set_size(cell, cell_w, cell_h);
        lv_obj_set_pos(cell, x, y);
        lv_obj_set_style_radius(cell, 8, 0);
        lv_obj_set_style_bg_color(cell, RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_color(cell, RED808_PANEL, 0);
        lv_obj_set_style_bg_grad_dir(cell, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_opa(cell, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(cell, pp_engine_color(s_pp_engine_idx), 0);
        lv_obj_set_style_border_opa(cell, (lv_opa_t)115, 0);
        lv_obj_set_style_border_width(cell, 2, 0);
        lv_obj_set_style_shadow_width(cell, 0, 0);
        lv_obj_set_style_pad_all(cell, 0, 0);
        lv_obj_set_style_bg_color(cell, pp_engine_color(s_pp_engine_idx), LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(cell, (lv_opa_t)216, LV_STATE_PRESSED);
        lv_obj_clear_flag(cell, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* top_line = lv_obj_create(cell);
        lv_obj_set_size(top_line, cell_w - 12, 3);
        lv_obj_align(top_line, LV_ALIGN_TOP_MID, 0, 5);
        lv_obj_set_style_radius(top_line, 2, 0);
        lv_obj_set_style_bg_color(top_line, pp_engine_color(s_pp_engine_idx), 0);
        lv_obj_set_style_bg_opa(top_line, LV_OPA_70, 0);
        lv_obj_set_style_border_width(top_line, 0, 0);
        lv_obj_clear_flag(top_line, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(top_line, LV_OBJ_FLAG_CLICKABLE);

        // Background fill bar (full cell width = max). Positioned at bottom,
        // grows upward as a horizontal strip showing fill.
        int bar_full_w = cell_w - 14;
        int bar_h = 8;
        lv_obj_t* bar = lv_obj_create(cell);
        lv_obj_set_size(bar, bar_full_w, bar_h);
        lv_obj_set_pos(bar, 7, cell_h - bar_h - 6);
        lv_obj_set_style_radius(bar, bar_h / 2, 0);
        lv_obj_set_style_bg_color(bar, lv_color_hex(0x00E5FF), 0);
        lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(bar, 0, 0);
        lv_obj_set_style_shadow_width(bar, 0, 0);
        lv_obj_set_style_pad_all(bar, 0, 0);
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_user_data(bar, (void*)(intptr_t)bar_full_w);
        s_pp_cell_bars[i] = bar;

        // Param name (top-left, big)
        lv_obj_t* nlbl = lv_label_create(cell);
        lv_label_set_text(nlbl, p->name);
        lv_obj_set_style_text_font(nlbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(nlbl, RED808_TEXT, 0);
        lv_obj_align(nlbl, LV_ALIGN_TOP_LEFT, 10, 11);

        // Value text (centered, very big)
        lv_obj_t* vlbl = lv_label_create(cell);
        lv_obj_set_style_text_font(vlbl, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(vlbl, lv_color_hex(0xFFE066), 0);
        lv_obj_align(vlbl, LV_ALIGN_CENTER, 0, 4);
        s_pp_val_lbls[i] = vlbl;

        // RST badge top-right — child button consumes its own click so the
        // surrounding cell only sees press/hold events.
        lv_obj_t* rst = lv_btn_create(cell);
        lv_obj_set_size(rst, 42, 22);
        lv_obj_align(rst, LV_ALIGN_TOP_RIGHT, -6, 9);
        lv_obj_set_style_radius(rst, 6, 0);
        lv_obj_set_style_bg_color(rst, RED808_BG, 0);
        lv_obj_set_style_bg_grad_color(rst, RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_dir(rst, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_border_color(rst, pp_engine_color(s_pp_engine_idx), 0);
        lv_obj_set_style_border_opa(rst, LV_OPA_60, 0);
        lv_obj_set_style_border_width(rst, 1, 0);
        lv_obj_set_style_shadow_width(rst, 0, 0);
        lv_obj_t* rl = lv_label_create(rst);
        lv_label_set_text(rl, "RST");
        lv_obj_set_style_text_font(rl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(rl, RED808_TEXT, 0);
        lv_obj_center(rl);
        lv_obj_add_event_cb(rst, pp_cell_reset_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);

        lv_obj_add_event_cb(cell, pp_cell_press_cb, LV_EVENT_PRESSED, (void*)(intptr_t)i);
        lv_obj_add_event_cb(cell, pp_cell_long_repeat_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void*)(intptr_t)i);
        s_pp_sliders[i] = cell;  // reuse the array slot for cleanup

        pp_cell_redraw_value(i);
    }
}

static void pp_refresh_view(void) {
    if (s_pp_title_lbl) {
        char tbuf[64];
        if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
            snprintf(tbuf, sizeof(tbuf), "XTRA EDIT · S%d · %s", s_pp_xtra_slot + 1,
                     SP_ENGINES[s_pp_engine_idx].long_name);
        } else {
            snprintf(tbuf, sizeof(tbuf), "SYNTH LAB · %s", SP_ENGINES[s_pp_engine_idx].long_name);
        }
        lv_label_set_text(s_pp_title_lbl, tbuf);
        lv_obj_set_style_text_color(s_pp_title_lbl, pp_engine_color(s_pp_engine_idx), 0);
    }
    pp_update_engine_chips();
    pp_update_preset_chips();
    pp_refresh_wave_preview();
    pp_rebuild_param_grid();
}

static void pp_update_engine_chips(void) {
    for (int i = 0; i < SP_ENGINE_COUNT; i++) {
        if (!s_pp_engine_btns[i]) continue;
        bool active = (i == s_pp_engine_idx);
        lv_color_t color = pp_engine_color(i);
        lv_obj_set_style_bg_color(s_pp_engine_btns[i], active ? color : RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_color(s_pp_engine_btns[i], active ? RED808_SURFACE : RED808_PANEL, 0);
        lv_obj_set_style_border_color(s_pp_engine_btns[i], active ? color : RED808_BORDER, 0);
        lv_obj_set_style_border_opa(s_pp_engine_btns[i], active ? LV_OPA_COVER : LV_OPA_70, 0);
        lv_obj_t* lbl = lv_obj_get_child(s_pp_engine_btns[i], 0);
        if (lbl) lv_obj_set_style_text_color(lbl, active ? RED808_BG : color, 0);
    }
}

static void pp_update_preset_chips(void) {
    const SynthEngineDef* eng = &SP_ENGINES[s_pp_engine_idx];
    for (int i = 0; i < 4; i++) {
        if (!s_pp_preset_btns[i]) continue;
        bool active = (i == s_pp_preset_idx[s_pp_engine_idx]);
        lv_color_t color = pp_engine_color(s_pp_engine_idx);
        lv_obj_set_style_bg_color(s_pp_preset_btns[i], active ? color : RED808_SURFACE, 0);
        lv_obj_set_style_bg_grad_color(s_pp_preset_btns[i], active ? RED808_SURFACE : RED808_PANEL, 0);
        lv_obj_set_style_border_color(s_pp_preset_btns[i],
                                      active ? color : RED808_BORDER, 0);
        lv_obj_set_style_border_opa(s_pp_preset_btns[i], active ? LV_OPA_COVER : LV_OPA_70, 0);
        lv_obj_t* lbl = lv_obj_get_child(s_pp_preset_btns[i], 0);
        if (lbl) {
            const char* nm = (i < eng->preset_count) ? eng->presets[i].name : "—";
            lv_label_set_text(lbl, nm);
            lv_obj_set_style_text_color(lbl, active ? RED808_BG : RED808_TEXT, 0);
        }
    }
}

// Reflect the active engine's selected preset on the piano-screen PRESET chip.
static void piano_refresh_engine_preset_label(void) {
    if (!s_piano_eng_preset_lbl) return;
    int eng_idx = s_piano_engine_idx;
    if (eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) {
        lv_label_set_text(s_piano_eng_preset_lbl, "PRESET");
        return;
    }
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    int pi = s_pp_preset_idx[eng_idx];
    if (pi >= 0 && pi < eng->preset_count) {
        lv_label_set_text_fmt(s_piano_eng_preset_lbl, "%s: %s", eng->label, eng->presets[pi].name);
    } else {
        lv_label_set_text_fmt(s_piano_eng_preset_lbl, "%s: INIT", eng->label);
    }
}

// Cycle through the active engine's factory presets and apply immediately,
// without entering the PARAMS editor. Mirrors pp_preset_cb's send path.
static void piano_engine_preset_cb(lv_event_t* e) {
    LV_UNUSED(e);
    int eng_idx = s_piano_engine_idx;
    if (eng_idx < 0 || eng_idx >= SP_ENGINE_COUNT) return;
    const SynthEngineDef* eng = &SP_ENGINES[eng_idx];
    if (eng->preset_count <= 0) return;
    int next = s_pp_preset_idx[eng_idx] + 1;
    if (next >= eng->preset_count) next = 0;
    s_pp_preset_idx[eng_idx] = next;
    pp_apply_preset_local(eng_idx, next);
    pp_update_preset_chips();   // keep PARAMS chips in sync if that screen is built
    piano_refresh_engine_preset_label();
    if (ui_use_udp_transport()) {
        piano_send_panic_melodic();
        udp_send_synth_preset(eng->engine, (uint8_t)next);
    }
}

static void pp_engine_cb(lv_event_t* e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= SP_ENGINE_COUNT) return;
    s_pp_engine_idx = idx;
    if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
        s_xtra_slots[s_pp_xtra_slot].synth_mode = true;
        s_xtra_slots[s_pp_xtra_slot].synth_engine_idx = xtra_engine_idx_from_pp_engine(idx);
        xtra_reset_slot_params(s_pp_xtra_slot);
        xtra_slot_refresh_name(s_pp_xtra_slot);
        xtra_save_state();
        xtra_refresh_panel();
    }
    pp_refresh_view();
}

static void pp_preset_cb(lv_event_t* e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    const SynthEngineDef* eng = &SP_ENGINES[s_pp_engine_idx];
    if (idx < 0 || idx >= eng->preset_count) return;
    pp_apply_preset_local(s_pp_engine_idx, idx);
    s_pp_preset_idx[s_pp_engine_idx] = idx;
    pp_update_preset_chips();
    pp_rebuild_param_grid();
    if (ui_use_udp_transport()) {
        piano_send_panic_melodic();
        udp_send_synth_preset(eng->engine, (uint8_t)idx);
#if P4_ENABLE_DEBUG_LOG
        Serial.printf("[P4 params] preset eng=%u preset=%d packets=6\n", eng->engine, idx);
#endif
    }
    if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
        s_xtra_slots[s_pp_xtra_slot].preset_idx = (uint8_t)idx;
        s_xtra_slots[s_pp_xtra_slot].synth_mode = true;
        s_xtra_slots[s_pp_xtra_slot].synth_engine_idx = xtra_engine_idx_from_pp_engine(s_pp_engine_idx);
        xtra_capture_editor_state(s_pp_xtra_slot);
        xtra_slot_refresh_name(s_pp_xtra_slot);
        xtra_save_state();
        xtra_refresh_panel();
    }
}

static void pp_init_cb(lv_event_t* e) {
    (void)e;
    pp_init_engine_defaults(s_pp_engine_idx);
    s_pp_preset_idx[s_pp_engine_idx] = -1;
    pp_update_preset_chips();
    pp_rebuild_param_grid();
    if (ui_use_udp_transport()) {
        const SynthEngineDef* eng = &SP_ENGINES[s_pp_engine_idx];
        for (uint8_t i = 0; i < eng->param_count; i++) {
            udp_send_synth_param(eng->engine, 0, eng->params[i].param_id,
                                 s_pp_values[s_pp_engine_idx][i]);
        }
    }
    if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
        xtra_capture_editor_state(s_pp_xtra_slot);
    }
}

static void pp_back_cb(lv_event_t* e) {
    (void)e;
    if (s_pp_from_xtra) {
        s_pp_from_xtra = false;
        s_pp_xtra_slot = -1;
        ui_navigate_to(6);
        return;
    }
    ui_navigate_to(10);  // back to PIANO
}

static void xtra_edit_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot >= 4) return;
    if (!s_xtra_slots[slot].synth_mode) {
        ui_show_toast("Sampler: usa LOAD WAV", theme_warning());
        return;
    }
    if (xtra_slot_is_drum(slot)) {
        ui_show_toast("Editor XTRA: melodic only", theme_warning());
        return;
    }
    int pp_idx = pp_engine_idx_from_code(xtra_slot_engine_code(slot));
    if (pp_idx < 0 || pp_idx >= SP_ENGINE_COUNT) {
        ui_show_toast("Engine sin editor", theme_warning());
        return;
    }
    s_pp_from_xtra = true;
    s_pp_xtra_slot = slot;
    s_pp_engine_idx = pp_idx;
    int preset_idx = constrain((int)s_xtra_slots[slot].preset_idx, 0, 2);
    xtra_load_editor_state(slot);
    s_pp_preset_idx[pp_idx] = preset_idx;
    ui_navigate_to(11);
}

static void create_piano_params_screen(void) {
    int W = ui_layout_w();
    int H = ui_layout_h();

    for (int e = 0; e < SP_ENGINE_COUNT; e++) {
        pp_init_engine_defaults(e);
    }

    scr_piano_params = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_piano_params, RED808_BG, 0);
    lv_obj_clear_flag(scr_piano_params, LV_OBJ_FLAG_SCROLLABLE);
    ui_create_header(scr_piano_params);

    // Title
    s_pp_title_lbl = lv_label_create(scr_piano_params);
    char tbuf[64];
    snprintf(tbuf, sizeof(tbuf), "SYNTH LAB · %s", SP_ENGINES[s_pp_engine_idx].long_name);
    lv_label_set_text(s_pp_title_lbl, tbuf);
    lv_obj_set_style_text_font(s_pp_title_lbl, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(s_pp_title_lbl, pp_engine_color(s_pp_engine_idx), 0);
    lv_obj_set_pos(s_pp_title_lbl, 64, 12);

    // BACK button (top right)
    {
        lv_obj_t* b = lv_btn_create(scr_piano_params);
        lv_obj_set_size(b, 80, 36);
        lv_obj_set_pos(b, W - 92, 8);
        apply_control_button_style(b, RED808_ERROR, false, 8);
        lv_obj_t* l = lv_label_create(b);
        lv_label_set_text(l, LV_SYMBOL_LEFT " BACK");
        lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(l, RED808_ERROR, 0);
        lv_obj_center(l);
        lv_obj_add_event_cb(b, pp_back_cb, LV_EVENT_CLICKED, NULL);
    }

    // Engine tabs row (y=56)
    const int tab_y = 56, tab_h = 38, tab_gap = 6;
    int tab_w = (W - 24 - (SP_ENGINE_COUNT - 1) * tab_gap) / SP_ENGINE_COUNT;
    for (int i = 0; i < SP_ENGINE_COUNT; i++) {
        lv_obj_t* b = lv_btn_create(scr_piano_params);
        lv_obj_set_size(b, tab_w, tab_h);
        lv_obj_set_pos(b, 12 + i * (tab_w + tab_gap), tab_y);
        apply_control_button_style(b, pp_engine_color(i), i == s_pp_engine_idx, 8);
        lv_obj_t* lbl = lv_label_create(b);
        lv_label_set_text(lbl, SP_ENGINES[i].label);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
        lv_obj_center(lbl);
        lv_obj_add_event_cb(b, pp_engine_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        s_pp_engine_btns[i] = b;
    }

    // Preset chips row (y=104) — 4 presets + INIT
    const int p_y = 104, p_h = 38, p_gap = 6;
    int p_w = (W - 24 - 4 * p_gap - 80) / 4;  // reserve 80 for INIT
    for (int i = 0; i < 4; i++) {
        lv_obj_t* b = lv_btn_create(scr_piano_params);
        lv_obj_set_size(b, p_w, p_h);
        lv_obj_set_pos(b, 12 + i * (p_w + p_gap), p_y);
        apply_control_button_style(b, lv_color_hex(0xFF1493), false, 8);
        lv_obj_t* lbl = lv_label_create(b);
        lv_label_set_text(lbl, SP_ENGINES[s_pp_engine_idx].presets[i].name);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_center(lbl);
        lv_obj_add_event_cb(b, pp_preset_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        s_pp_preset_btns[i] = b;
    }
    {
        lv_obj_t* b = lv_btn_create(scr_piano_params);
        lv_obj_set_size(b, 76, p_h);
        lv_obj_set_pos(b, W - 88, p_y);
        apply_control_button_style(b, RED808_INFO, false, 8);
        lv_obj_t* l = lv_label_create(b);
        lv_label_set_text(l, "INIT");
        lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(l, RED808_INFO, 0);
        lv_obj_center(l);
        lv_obj_add_event_cb(b, pp_init_cb, LV_EVENT_CLICKED, NULL);
    }

    s_pp_wave_card = lv_obj_create(scr_piano_params);
    lv_obj_set_size(s_pp_wave_card, W - 24, 82);
    lv_obj_set_pos(s_pp_wave_card, 12, 152);
    lv_obj_set_style_radius(s_pp_wave_card, 10, 0);
    lv_obj_set_style_bg_color(s_pp_wave_card, RED808_SURFACE, 0);
    lv_obj_set_style_bg_grad_color(s_pp_wave_card, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_dir(s_pp_wave_card, LV_GRAD_DIR_HOR, 0);
    lv_obj_set_style_border_color(s_pp_wave_card, RED808_BORDER, 0);
    lv_obj_set_style_border_width(s_pp_wave_card, 1, 0);
    lv_obj_set_style_pad_all(s_pp_wave_card, 0, 0);
    lv_obj_clear_flag(s_pp_wave_card, LV_OBJ_FLAG_SCROLLABLE);

    s_pp_wave_lbl = lv_label_create(s_pp_wave_card);
    lv_label_set_text(s_pp_wave_lbl, "Preview synth");
    lv_obj_set_style_text_font(s_pp_wave_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_pp_wave_lbl, RED808_TEXT_DIM, 0);
    lv_obj_set_pos(s_pp_wave_lbl, 12, 8);

    s_pp_wave_line = lv_line_create(s_pp_wave_card);
    lv_obj_set_size(s_pp_wave_line, 300, 68);
    lv_obj_align(s_pp_wave_line, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_obj_set_style_line_color(s_pp_wave_line, lv_color_hex(0x00E5FF), 0);
    lv_obj_set_style_line_width(s_pp_wave_line, 3, 0);
    lv_obj_set_style_line_rounded(s_pp_wave_line, true, 0);

    // Param panel
    int panel_y = 242;
    int panel_h = H - panel_y - 12;
    s_pp_param_panel = lv_obj_create(scr_piano_params);
    lv_obj_set_size(s_pp_param_panel, W - 24, panel_h);
    lv_obj_set_pos(s_pp_param_panel, 12, panel_y);
    lv_obj_set_style_radius(s_pp_param_panel, 8, 0);
    lv_obj_set_style_bg_color(s_pp_param_panel, RED808_PANEL, 0);
    lv_obj_set_style_bg_grad_color(s_pp_param_panel, RED808_BG, 0);
    lv_obj_set_style_bg_grad_dir(s_pp_param_panel, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_color(s_pp_param_panel, RED808_BORDER, 0);
    lv_obj_set_style_border_width(s_pp_param_panel, 1, 0);
    lv_obj_set_style_pad_all(s_pp_param_panel, 0, 0);
    lv_obj_clear_flag(s_pp_param_panel, LV_OBJ_FLAG_SCROLLABLE);

    pp_refresh_view();
}

// =============================================================================
// PERFORMANCE SCREEN (placeholder)
// =============================================================================
// --- XTRA sampler editor: waveform render + controls ----------------------
static void xtra_editor_render(void) {
    if (s_xtra_edit_slot < 0 || s_xtra_edit_slot >= 4) return;
    XtraPadSlot& s = s_xtra_slots[s_xtra_edit_slot];
    const sample_edit::SampleInfo& si = sample_edit::info();
    bool have = si.loaded && (s_xtra_loaded_slot == s_xtra_edit_slot) && si.frames > 0;

    if (s_xtra_wave_line) {
        if (have) sample_edit::envelope(s_xtra_env, XTRA_WAVE_COLS);
        else for (int c = 0; c < XTRA_WAVE_COLS; c++) s_xtra_env[c] = 0.f;
        int w = lv_obj_get_width(s_xtra_wave_line);
        int h = lv_obj_get_height(s_xtra_wave_line);
        if (w < 10) w = 960; if (h < 10) h = 146;
        int mid = h / 2;
        for (int c = 0; c < XTRA_WAVE_COLS; c++) {
            int x = c * w / XTRA_WAVE_COLS;
            int amp = (int)(s_xtra_env[c] * (mid - 2));
            s_xtra_wave_pts[c].x = x;                     s_xtra_wave_pts[c].y = mid - amp;
            s_xtra_wave_pts[XTRA_WAVE_COLS * 2 - 1 - c].x = x;
            s_xtra_wave_pts[XTRA_WAVE_COLS * 2 - 1 - c].y = mid + amp;
        }
        lv_line_set_points(s_xtra_wave_line, s_xtra_wave_pts, XTRA_WAVE_COLS * 2);
        if (s_xtra_trim_a_line) lv_obj_set_x(s_xtra_trim_a_line, (int)(s.trim_start * w));
        if (s_xtra_trim_b_line) lv_obj_set_x(s_xtra_trim_b_line, (int)(s.trim_end * w) - 2);
    }

    if (s_xtra_info_lbl) {
        if (!s.src_path[0]) {
            lv_label_set_text(s_xtra_info_lbl, "Slot vacio — pulsa LOAD para elegir un WAV/MP3");
        } else if (!have) {
            lv_label_set_text_fmt(s_xtra_info_lbl, "%s — no se pudo leer la onda", s.name);
        } else {
            float dur = (float)si.frames / (float)(si.sample_rate ? si.sample_rate : 1);
            lv_label_set_text_fmt(s_xtra_info_lbl, "S%02d  %s  ·  %s %luHz %dch  ·  %.2fs%s",
                s_xtra_edit_slot + 1, s.name, si.is_mp3 ? "MP3" : "WAV",
                (unsigned long)si.sample_rate, (int)si.channels, dur,
                si.truncated ? "  (recortado)" : "");
        }
    }
    if (s_xtra_trim_lbl)
        lv_label_set_text_fmt(s_xtra_trim_lbl, "TRIM  %d%% .. %d%%",
            (int)(s.trim_start * 100), (int)(s.trim_end * 100));
    if (s_xtra_fade_lbl)
        lv_label_set_text_fmt(s_xtra_fade_lbl, "FADE IN %ums   OUT %ums",
            (unsigned)s.fade_in_ms, (unsigned)s.fade_out_ms);
}

static void xtra_editor_select(int slot) {
    if (slot < 0 || slot >= 4) return;
    s_xtra_edit_slot = slot;
    XtraPadSlot& s = s_xtra_slots[slot];
    if (s.src_path[0] && s_xtra_loaded_slot != slot) {
        ui_busy_show("Leyendo sample...");
        bool ok = sample_edit::load(SD_MMC, s.src_path);
        ui_busy_hide();
        s_xtra_loaded_slot = ok ? slot : -1;
    } else if (!s.src_path[0]) {
        s_xtra_loaded_slot = -1;
        sample_edit::unload();
    }
    xtra_refresh_panel();
    xtra_editor_render();
}

// HTTP upload of an already-baked WAV file on SD to a Master pad.
static bool xtra_http_upload_path(const char* sd_path, const char* postname, int pad) {
    File sample = SD_MMC.open(sd_path, FILE_READ);
    if (!sample) return false;
    size_t sample_size = sample.size();
    if (sample_size == 0) { sample.close(); return false; }

    WiFiClient client;
    client.setTimeout(10000);
    if (!client.connect(IPAddress(192, 168, 4, 1), 80)) { sample.close(); return false; }

    const char* boundary = "----RED808P4Upload";
    char file_head[192];
    snprintf(file_head, sizeof(file_head),
             "--%s\r\nContent-Disposition: form-data; name=\"file\"; filename=\"%s\"\r\nContent-Type: audio/wav\r\n\r\n",
             boundary, postname);
    char file_tail[40];
    snprintf(file_tail, sizeof(file_tail), "\r\n--%s--\r\n", boundary);
    size_t content_len = strlen(file_head) + sample_size + strlen(file_tail);

    client.printf("POST /api/uploadDaisy?pad=%d HTTP/1.1\r\n", pad);
    client.print("Host: 192.168.4.1\r\n");
    client.print("Connection: close\r\n");
    client.printf("Content-Type: multipart/form-data; boundary=%s\r\n", boundary);
    client.printf("Content-Length: %u\r\n\r\n", (unsigned)content_len);
    client.print(file_head);

    uint8_t buf[2048];
    bool write_ok = true;
    while (sample.available()) {
        size_t n = sample.read(buf, sizeof(buf));
        if (n == 0) break;
        if (client.write(buf, n) != n) { write_ok = false; break; }
        yield();
    }
    sample.close();
    if (write_ok) client.print(file_tail);

    int status = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 5000) {
        if (client.available()) {
            String line = client.readStringUntil('\n');
            if (line.startsWith("HTTP/1.1 ")) status = line.substring(9, 12).toInt();
            break;
        }
        if (!client.connected()) break;
        delay(2);
    }
    client.stop();
    return write_ok && status == 200;
}

static void xtra_pad_select_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot >= 4) return;
    xtra_editor_select(slot);
    // Tap triggers the sample only once it's been applied to the Master; while
    // it has unbaked edits, point the user at PLAY (which bakes + uploads).
    if (!s_xtra_slots[slot].src_path[0]) return;
    if (s_xtra_applied[slot] && udp_wifi_connected())
        udp_send_trigger(s_xtra_slots[slot].pad, 110);
    else if (!s_xtra_applied[slot])
        ui_show_toast("Pulsa PLAY para oir los cambios", theme_warning());
}

static void xtra_load_cb(lv_event_t* e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot >= 4) return;
    s_xtra_edit_slot = slot;
    xtra_begin_load_for_slot(slot);   // → SD browser; LOAD there assigns the file
}

static void xtra_edit_adj_cb(lv_event_t* e) {
    int code = (int)(intptr_t)lv_event_get_user_data(e);
    int ctrl = code / 2;
    int dir  = (code % 2) ? +1 : -1;
    XtraPadSlot& s = s_xtra_slots[s_xtra_edit_slot];
    switch (ctrl) {
        case 0: s.trim_start = constrain(s.trim_start + dir * 0.02f, 0.0f, s.trim_end - 0.02f); break;
        case 1: s.trim_end   = constrain(s.trim_end   + dir * 0.02f, s.trim_start + 0.02f, 1.0f); break;
        case 2: { int v = (int)s.fade_in_ms  + dir * 10; v = constrain(v, 0, 2000); s.fade_in_ms  = (uint16_t)v; } break;
        case 3: { int v = (int)s.fade_out_ms + dir * 10; v = constrain(v, 0, 2000); s.fade_out_ms = (uint16_t)v; } break;
    }
    s_xtra_applied[s_xtra_edit_slot] = false;   // Master no longer matches the editor
    xtra_save_state();
    xtra_editor_render();
}

// Bake the current edits and upload to the slot's backing pad. Returns true on
// success (and clears the dirty flag). Shows its own busy overlay / error toast.
static bool xtra_apply_current(void) {
    int slot = s_xtra_edit_slot;
    XtraPadSlot& s = s_xtra_slots[slot];
    if (!s.src_path[0]) { ui_show_toast("Slot vacio", theme_warning()); return false; }
    if (s_xtra_loaded_slot != slot) {
        ui_busy_show("Leyendo sample...");
        bool ok = sample_edit::load(SD_MMC, s.src_path);
        ui_busy_hide();
        if (!ok) { ui_show_toast("No se pudo leer el sample", RED808_WARNING); return false; }
        s_xtra_loaded_slot = slot;
    }
    char outp[40];
    snprintf(outp, sizeof(outp), "/xtra_bake_%d.wav", slot);
    ui_busy_show("Procesando y subiendo...\nEspera, no cambies de pantalla.");
    size_t n = sample_edit::bake_wav(SD_MMC, outp, s.trim_start, s.trim_end,
                                     s.fade_in_ms, s.fade_out_ms);
    if (n == 0) { ui_busy_hide(); ui_show_toast("Error al procesar el sample", RED808_WARNING); return false; }
    char postname[40];
    snprintf(postname, sizeof(postname), "%s.wav", s.name[0] ? s.name : "xtra");
    bool ok = xtra_http_upload_path(outp, postname, s.pad);
    ui_busy_hide();
    if (!ok) { ui_show_toast("Subida fallida (Master?)", RED808_WARNING); return false; }
    s.used = true;
    s_xtra_applied[slot] = true;
    xtra_save_state();
    xtra_refresh_panel();
    return true;
}

// PLAY / PREVIEW: there is no local audio on the P4, so to preview the trim and
// fades we (re)bake + upload whenever the slot's edits aren't on the Master yet,
// then trigger.
static void xtra_play_cb(lv_event_t* e) {
    LV_UNUSED(e);
    XtraPadSlot& s = s_xtra_slots[s_xtra_edit_slot];
    if (!s.src_path[0]) { ui_show_toast("Slot vacio — pulsa LOAD", theme_warning()); return; }
    if (!s_xtra_applied[s_xtra_edit_slot]) {
        if (!xtra_apply_current()) return;   // upload failed; error already shown
        delay(120);
    }
    if (udp_wifi_connected()) udp_send_trigger(s.pad, 110);
    else ui_show_toast("Master no conectado", RED808_WARNING);

    // Kick off the estimated playhead over the waveform (timed from the sample's
    // trimmed length — the P4 has no real position feedback from the Master).
    s_xtra_play_start_ms = 0;
    const sample_edit::SampleInfo& psi = sample_edit::info();
    if (psi.loaded && s_xtra_loaded_slot == s_xtra_edit_slot && psi.frames > 0 && psi.sample_rate > 0) {
        float a = s.trim_start, b = s.trim_end;
        if (b < a) { float t = a; a = b; b = t; }
        float total = (float)psi.frames / (float)psi.sample_rate;     // seconds
        unsigned long dur = (unsigned long)((b - a) * total * 1000.0f);
        if (dur < 120) dur = 120;   // floor so LOOP can't flood the Master with re-triggers
        if (dur > 0) {
            s_xtra_play_a = a;
            s_xtra_play_b = b;
            s_xtra_play_dur_ms = dur;
            s_xtra_play_start_ms = millis();
        }
    }
}

// Move the playback cursor over the waveform. Called each frame while the XTRA
// screen is active; sweeps s_xtra_play_a→s_xtra_play_b over s_xtra_play_dur_ms.
static void update_xtra_playhead(void) {
    if (!s_xtra_playhead_line) return;
    if (s_xtra_play_start_ms == 0) {
        lv_obj_add_flag(s_xtra_playhead_line, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    unsigned long el = millis() - s_xtra_play_start_ms;
    if (el >= s_xtra_play_dur_ms) {
        if (s_xtra_loop_on && udp_wifi_connected() &&
            s_xtra_edit_slot >= 0 && s_xtra_edit_slot < 4) {
            udp_send_trigger(s_xtra_slots[s_xtra_edit_slot].pad, 110);  // LOOP: re-fire the sample
            s_xtra_play_start_ms = millis();
            el = 0;
        } else {
            s_xtra_play_start_ms = 0;
            lv_obj_add_flag(s_xtra_playhead_line, LV_OBJ_FLAG_HIDDEN);
            return;
        }
    }
    float prog = (float)el / (float)s_xtra_play_dur_ms;              // 0..1
    float pos  = s_xtra_play_a + prog * (s_xtra_play_b - s_xtra_play_a);
    int w = s_xtra_wave_line ? lv_obj_get_width(s_xtra_wave_line) : 960;
    if (w < 10) w = 960;
    lv_obj_clear_flag(s_xtra_playhead_line, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_x(s_xtra_playhead_line, (int)(pos * w));
}

static void xtra_refresh_loop_btn(void) {
    if (!s_xtra_loop_btn) return;
    lv_obj_t* l = lv_obj_get_child(s_xtra_loop_btn, 0);
    if (l) {
        lv_label_set_text(l, s_xtra_loop_on ? LV_SYMBOL_LOOP " ON" : LV_SYMBOL_LOOP " OFF");
        lv_obj_set_style_text_color(l, s_xtra_loop_on ? theme_success() : theme_text(), 0);
    }
}

static void xtra_loop_cb(lv_event_t* e) {
    (void)e;
    s_xtra_loop_on = !s_xtra_loop_on;
    if (s_xtra_loop_on) xtra_play_cb(NULL);   // start now; update_xtra_playhead keeps re-firing
    xtra_refresh_loop_btn();
}

static void xtra_apply_cb(lv_event_t* e) {
    LV_UNUSED(e);
    if (xtra_apply_current()) {
        int pad = s_xtra_slots[s_xtra_edit_slot].pad;
        delay(120);   // let the Master finish ingesting before we trigger
        if (udp_wifi_connected()) udp_send_trigger(pad, 110);
        char msg[48];
        snprintf(msg, sizeof(msg), "Subido y disparado · pad %d", pad);
        ui_show_toast(msg, RED808_SUCCESS);
    }
}

static lv_obj_t* xtra_make_btn(lv_obj_t* par, int x, int y, int w, int h,
                               const char* txt, lv_color_t col,
                               lv_event_cb_t cb, int code) {
    lv_obj_t* b = lv_btn_create(par);
    lv_obj_set_size(b, w, h);
    lv_obj_set_pos(b, x, y);
    apply_control_button_style(b, col, false, 8);
    lv_obj_t* l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_16, 0);
    lv_obj_center(l);
    if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, (void*)(intptr_t)code);
    return b;
}

// =============================================================================
// DRONE LAYER — sustained tuned note(s) held under the XTRA sampler to reframe
// a dry sample emotionally (Fred again.. style: "same words, different meaning"
// just by changing the harmonic bed). Pure control: holds notes on a Master
// synth engine over UDP (no local audio). Lives on the XTRA screen; trigger the
// XTRA sample pads on top and ride the root to reframe it live.
// =============================================================================
static const uint8_t DRONE_ENGINES[]      = {4, 5, 6, 3};   // WT, SH101, FM2, 303 (sustaining)
static const char*   DRONE_ENGINE_NAMES[] = {"WT", "SH101", "FM2", "303"};
static constexpr int DRONE_ENGINE_COUNT   = 4;
// Chord shapes built from a single root → harmonic beds. maj↔min is the big
// emotional flip (bright↔sad); sus4 = unresolved tension; 7ths add cinematic
// colour. This is what makes the reframe actually audible.
static const char*   DRONE_MODE_NAMES[]   = {"ROOT", "5th", "maj", "min", "sus4", "min7", "maj7"};
static constexpr int DRONE_MODE_COUNT     = 7;

static bool    s_drone_on       = false;
static int     s_drone_eng_idx  = 0;
static int     s_drone_root     = 48;    // C3
static int     s_drone_mode     = 2;     // maj — a full chord bed is far more noticeable than one note
static uint8_t s_drone_level    = 110;   // note velocity (louder = more present under the sample)
static uint8_t s_drone_notes[4] = {0};
static int     s_drone_note_cnt = 0;

static lv_obj_t* s_drone_toggle_btn = NULL;
static lv_obj_t* s_drone_status_lbl = NULL;

static void drone_note_name(int note, char* out, int n) {
    static const char* NN[12] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
    if (note < 0) note = 0;
    if (note > 127) note = 127;
    lv_snprintf(out, n, "%s%d", NN[note % 12], note / 12 - 1);
}

static int drone_build_notes(int root, int mode, int* out) {
    out[0] = root;
    switch (mode) {
        case 1: out[1] = root + 7;                                          return 2;  // 5th (power)
        case 2: out[1] = root + 4; out[2] = root + 7;                       return 3;  // major triad
        case 3: out[1] = root + 3; out[2] = root + 7;                       return 3;  // minor triad
        case 4: out[1] = root + 5; out[2] = root + 7;                       return 3;  // sus4
        case 5: out[1] = root + 3; out[2] = root + 7; out[3] = root + 10;   return 4;  // min7
        case 6: out[1] = root + 4; out[2] = root + 7; out[3] = root + 11;   return 4;  // maj7
        default:                                                            return 1;  // root only
    }
}

static void drone_all_off(void) {
    if (ui_use_udp_transport()) {
        uint8_t eng = DRONE_ENGINES[s_drone_eng_idx];
        for (int i = 0; i < s_drone_note_cnt; i++)
            udp_send_synth_note_off_ex(eng, 0, s_drone_notes[i]);
    }
    s_drone_note_cnt = 0;
}

static void drone_refresh_ui(void) {
    if (s_drone_toggle_btn) {
        lv_obj_t* l = lv_obj_get_child(s_drone_toggle_btn, 0);
        if (l) lv_label_set_text(l, s_drone_on ? "DRONE ON" : "DRONE OFF");
    }
    if (s_drone_status_lbl) {
        char nm[8];
        drone_note_name(s_drone_root, nm, sizeof(nm));
        lv_label_set_text_fmt(s_drone_status_lbl, "%s   %s   %s   vel %d",
                              nm, DRONE_ENGINE_NAMES[s_drone_eng_idx],
                              DRONE_MODE_NAMES[s_drone_mode], s_drone_level);
        lv_obj_set_style_text_color(s_drone_status_lbl,
                                    s_drone_on ? theme_accent2() : theme_text_dim(), 0);
    }
}

static void drone_revoice(void) {
    drone_all_off();
    if (s_drone_on && ui_use_udp_transport()) {
        uint8_t eng = DRONE_ENGINES[s_drone_eng_idx];
        int notes[3];
        int n = drone_build_notes(s_drone_root, s_drone_mode, notes);
        for (int i = 0; i < n; i++) {
            int v = notes[i];
            if (v < 0) v = 0;
            if (v > 127) v = 127;
            udp_send_synth_note_on_ex(eng, (uint8_t)v, s_drone_level, false, false);
            s_drone_notes[i] = (uint8_t)v;
        }
        s_drone_note_cnt = n;
    }
    drone_refresh_ui();
}

static void drone_force_off(void) {
    drone_all_off();
    s_drone_on = false;
    drone_refresh_ui();
}

static void drone_toggle_cb(lv_event_t* e) {
    (void)e;
    s_drone_on = !s_drone_on;
    drone_revoice();
}

static void drone_root_cb(lv_event_t* e) {
    int code = (int)(intptr_t)lv_event_get_user_data(e);   // 0:-1  1:+1  2:-12  3:+12
    int d = (code == 0) ? -1 : (code == 1) ? +1 : (code == 2) ? -12 : +12;
    s_drone_root += d;
    if (s_drone_root < 12)  s_drone_root = 12;
    if (s_drone_root > 108) s_drone_root = 108;
    drone_revoice();
}

static void drone_engine_cb(lv_event_t* e) {
    (void)e;
    drone_all_off();   // release notes on the OLD engine before switching
    s_drone_eng_idx = (s_drone_eng_idx + 1) % DRONE_ENGINE_COUNT;
    drone_revoice();
}

static void drone_mode_cb(lv_event_t* e) {
    (void)e;
    s_drone_mode = (s_drone_mode + 1) % DRONE_MODE_COUNT;
    drone_revoice();
}

static void drone_level_cb(lv_event_t* e) {
    int code = (int)(intptr_t)lv_event_get_user_data(e);   // 0:- 1:+
    int v = (int)s_drone_level + (code ? +8 : -8);
    if (v < 24)  v = 24;
    if (v > 127) v = 127;
    s_drone_level = (uint8_t)v;
    drone_revoice();   // retrigger at the new velocity
}

static void create_performance_screen(void) {
    scr_performance = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_performance, RED808_BG, 0);
    lv_obj_clear_flag(scr_performance, LV_OBJ_FLAG_SCROLLABLE);
    ui_create_header(scr_performance);

    int W = ui_layout_w();

    lv_obj_t* title = lv_label_create(scr_performance);
    lv_label_set_text(title, "XTRA SAMPLER");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, theme_accent2(), 0);
    lv_obj_set_pos(title, 64, 12);

    lv_obj_t* sub = lv_label_create(scr_performance);
    lv_label_set_text(sub, "Carga WAV/MP3 · onda · trim · fade in/out · APPLY sube al Master");
    lv_obj_set_style_text_font(sub, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sub, theme_text_dim(), 0);
    lv_obj_set_pos(sub, 280, 20);

    // --- 4 sampler pads (tap = select + trigger) + per-pad LOAD ------------
    const int x0 = 20, gap = 12, py = 64, ch = 92;
    const int cw = (W - 2 * x0 - 3 * gap) / 4;
    grid_xtra_change_btns[0] = grid_xtra_change_btns[1] = NULL;
    grid_xtra_change_btns[2] = grid_xtra_change_btns[3] = NULL;
    for (int i = 0; i < 4; i++) {
        int x = x0 + i * (cw + gap);
        grid_xtra_btns[i] = lv_btn_create(scr_performance);
        lv_obj_set_size(grid_xtra_btns[i], cw, ch);
        lv_obj_set_pos(grid_xtra_btns[i], x, py);
        lv_obj_set_style_radius(grid_xtra_btns[i], 12, 0);
        lv_obj_set_style_border_width(grid_xtra_btns[i], 2, 0);
        lv_obj_set_style_bg_opa(grid_xtra_btns[i], LV_OPA_80, 0);
        lv_obj_add_event_cb(grid_xtra_btns[i], xtra_pad_select_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);

        grid_xtra_lbls[i] = lv_label_create(grid_xtra_btns[i]);
        lv_label_set_text(grid_xtra_lbls[i], "+ LOAD");
        lv_obj_set_width(grid_xtra_lbls[i], cw - 14);
        lv_obj_set_style_text_font(grid_xtra_lbls[i], &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_align(grid_xtra_lbls[i], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(grid_xtra_lbls[i], LV_ALIGN_CENTER, 0, -8);

        grid_xtra_slot_lbls[i] = lv_label_create(grid_xtra_btns[i]);
        lv_label_set_text_fmt(grid_xtra_slot_lbls[i], "S%02d", i + 1);
        lv_obj_set_style_text_font(grid_xtra_slot_lbls[i], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(grid_xtra_slot_lbls[i], theme_text_dim(), 0);
        lv_obj_align(grid_xtra_slot_lbls[i], LV_ALIGN_TOP_LEFT, 8, 6);

        grid_xtra_meta_lbls[i] = lv_label_create(grid_xtra_btns[i]);
        lv_label_set_text(grid_xtra_meta_lbls[i], "WAV / MP3");
        lv_obj_set_style_text_font(grid_xtra_meta_lbls[i], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(grid_xtra_meta_lbls[i], theme_text(), 0);
        lv_obj_align(grid_xtra_meta_lbls[i], LV_ALIGN_BOTTOM_MID, 0, -8);

        grid_xtra_delete_btns[i] = xtra_make_btn(scr_performance, x, py + ch + 4, cw, 30,
                                                 "LOAD", theme_warning(), xtra_load_cb, i);
    }

    // --- Waveform panel ----------------------------------------------------
    const int wy = py + ch + 44;          // ~200
    const int wpw = W - 2 * x0;           // ~984
    const int wph = 150;
    s_xtra_wave_panel = lv_obj_create(scr_performance);
    lv_obj_set_size(s_xtra_wave_panel, wpw, wph);
    lv_obj_set_pos(s_xtra_wave_panel, x0, wy);
    lv_obj_set_style_radius(s_xtra_wave_panel, 10, 0);
    lv_obj_set_style_bg_color(s_xtra_wave_panel, RED808_PANEL, 0);
    lv_obj_set_style_bg_opa(s_xtra_wave_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_xtra_wave_panel, 1, 0);
    lv_obj_set_style_border_color(s_xtra_wave_panel, RED808_BORDER, 0);
    lv_obj_set_style_pad_all(s_xtra_wave_panel, 0, 0);
    lv_obj_clear_flag(s_xtra_wave_panel, LV_OBJ_FLAG_SCROLLABLE);

    s_xtra_trim_a_line = lv_obj_create(s_xtra_wave_panel);
    lv_obj_set_size(s_xtra_trim_a_line, 2, wph - 4);
    lv_obj_set_pos(s_xtra_trim_a_line, 0, 2);
    lv_obj_set_style_bg_color(s_xtra_trim_a_line, RED808_SUCCESS, 0);
    lv_obj_set_style_border_width(s_xtra_trim_a_line, 0, 0);
    lv_obj_clear_flag(s_xtra_trim_a_line, LV_OBJ_FLAG_CLICKABLE);

    s_xtra_trim_b_line = lv_obj_create(s_xtra_wave_panel);
    lv_obj_set_size(s_xtra_trim_b_line, 2, wph - 4);
    lv_obj_set_pos(s_xtra_trim_b_line, wpw - 6, 2);
    lv_obj_set_style_bg_color(s_xtra_trim_b_line, RED808_WARNING, 0);
    lv_obj_set_style_border_width(s_xtra_trim_b_line, 0, 0);
    lv_obj_clear_flag(s_xtra_trim_b_line, LV_OBJ_FLAG_CLICKABLE);

    s_xtra_wave_line = lv_line_create(s_xtra_wave_panel);
    lv_obj_set_size(s_xtra_wave_line, wpw - 4, wph - 4);
    lv_obj_align(s_xtra_wave_line, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_line_color(s_xtra_wave_line, lv_color_hex(0x00E5FF), 0);
    lv_obj_set_style_line_width(s_xtra_wave_line, 2, 0);
    lv_obj_set_style_line_rounded(s_xtra_wave_line, true, 0);

    // Playback cursor — moves left→right over the waveform while a sample plays.
    // Created last so it sits ON TOP of the wave line. The P4 doesn't get real
    // position from the Master, so update_xtra_playhead() sweeps it by time over
    // the sample's trimmed length. Hidden until PLAY.
    s_xtra_playhead_line = lv_obj_create(s_xtra_wave_panel);
    lv_obj_set_size(s_xtra_playhead_line, 2, wph - 4);
    lv_obj_set_pos(s_xtra_playhead_line, 0, 2);
    lv_obj_set_style_bg_color(s_xtra_playhead_line, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(s_xtra_playhead_line, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_xtra_playhead_line, 0, 0);
    lv_obj_clear_flag(s_xtra_playhead_line, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_xtra_playhead_line, LV_OBJ_FLAG_HIDDEN);

    // --- Info + controls ---------------------------------------------------
    s_xtra_info_lbl = lv_label_create(scr_performance);
    lv_label_set_text(s_xtra_info_lbl, "Slot vacio");
    lv_obj_set_style_text_font(s_xtra_info_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_xtra_info_lbl, theme_text_dim(), 0);
    lv_obj_set_pos(s_xtra_info_lbl, x0, wy + wph + 6);

    int ty = wy + wph + 30;               // trim controls row
    xtra_make_btn(scr_performance, x0,        ty, 64, 38, "S " LV_SYMBOL_MINUS, RED808_SUCCESS, xtra_edit_adj_cb, 0);
    xtra_make_btn(scr_performance, x0 + 68,   ty, 64, 38, "S " LV_SYMBOL_PLUS,  RED808_SUCCESS, xtra_edit_adj_cb, 1);
    s_xtra_trim_lbl = lv_label_create(scr_performance);
    lv_label_set_text(s_xtra_trim_lbl, "TRIM 0% .. 100%");
    lv_obj_set_style_text_font(s_xtra_trim_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_xtra_trim_lbl, RED808_TEXT, 0);
    lv_obj_set_pos(s_xtra_trim_lbl, x0 + 150, ty + 8);
    xtra_make_btn(scr_performance, x0 + 360,  ty, 64, 38, "E " LV_SYMBOL_MINUS, RED808_WARNING, xtra_edit_adj_cb, 2);
    xtra_make_btn(scr_performance, x0 + 428,  ty, 64, 38, "E " LV_SYMBOL_PLUS,  RED808_WARNING, xtra_edit_adj_cb, 3);

    int fy = ty + 48;                     // fade controls row
    xtra_make_btn(scr_performance, x0,        fy, 64, 38, "IN " LV_SYMBOL_MINUS,  RED808_CYAN, xtra_edit_adj_cb, 4);
    xtra_make_btn(scr_performance, x0 + 68,   fy, 64, 38, "IN " LV_SYMBOL_PLUS,   RED808_CYAN, xtra_edit_adj_cb, 5);
    s_xtra_fade_lbl = lv_label_create(scr_performance);
    lv_label_set_text(s_xtra_fade_lbl, "FADE IN 0ms   OUT 0ms");
    lv_obj_set_style_text_font(s_xtra_fade_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_xtra_fade_lbl, RED808_TEXT, 0);
    lv_obj_set_pos(s_xtra_fade_lbl, x0 + 150, fy + 8);
    xtra_make_btn(scr_performance, x0 + 360,  fy, 70, 38, "OUT " LV_SYMBOL_MINUS, RED808_CYAN, xtra_edit_adj_cb, 6);
    xtra_make_btn(scr_performance, x0 + 434,  fy, 70, 38, "OUT " LV_SYMBOL_PLUS,  RED808_CYAN, xtra_edit_adj_cb, 7);

    int ay = fy + 50;                     // action row — LEFT column: PLAY / LOOP / APPLY
                                          // (so it doesn't run under the DRONE panel)
    xtra_make_btn(scr_performance, x0,        ay, 140, 52, LV_SYMBOL_PLAY "  PLAY", RED808_CYAN, xtra_play_cb, 0);
    s_xtra_loop_btn = xtra_make_btn(scr_performance, x0 + 150, ay, 100, 52,
                                    LV_SYMBOL_LOOP " OFF", theme_text(), xtra_loop_cb, 0);
    xtra_make_btn(scr_performance, x0 + 260,  ay, 250, 52, LV_SYMBOL_UPLOAD "  APPLY", theme_accent2(), xtra_apply_cb, 0);
    xtra_refresh_loop_btn();

    // --- DRONE panel (right half, below the waveform; trim/fade live on left) -
    {
        const int bx = 540, by = 356;
        lv_obj_t* dp = lv_obj_create(scr_performance);
        lv_obj_set_size(dp, W - bx - x0, 174);   // right column, aligned with the action row bottom
        lv_obj_set_pos(dp, bx, by);
        lv_obj_set_style_radius(dp, 10, 0);
        lv_obj_set_style_bg_color(dp, RED808_PANEL, 0);
        lv_obj_set_style_bg_opa(dp, LV_OPA_40, 0);
        lv_obj_set_style_border_width(dp, 1, 0);
        lv_obj_set_style_border_color(dp, theme_accent2(), 0);
        lv_obj_set_style_border_opa(dp, LV_OPA_50, 0);
        lv_obj_set_style_pad_all(dp, 0, 0);
        lv_obj_clear_flag(dp, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* dt = lv_label_create(scr_performance);
        lv_label_set_text(dt, LV_SYMBOL_AUDIO "  DRONE");
        lv_obj_set_style_text_font(dt, &lv_font_montserrat_18, 0);
        lv_obj_set_style_text_color(dt, theme_accent2(), 0);
        lv_obj_set_pos(dt, bx + 14, by + 12);

        s_drone_toggle_btn = xtra_make_btn(scr_performance, bx + 300, by + 8, 130, 36,
                                           "DRONE OFF", theme_accent2(), drone_toggle_cb, 0);

        s_drone_status_lbl = lv_label_create(scr_performance);
        lv_obj_set_style_text_font(s_drone_status_lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(s_drone_status_lbl, bx + 14, by + 50);

        int ry = by + 78;    // root / octave / velocity row
        xtra_make_btn(scr_performance, bx + 14,  ry, 60, 34, LV_SYMBOL_LEFT,  RED808_SUCCESS, drone_root_cb, 0);
        xtra_make_btn(scr_performance, bx + 80,  ry, 60, 34, LV_SYMBOL_RIGHT, RED808_SUCCESS, drone_root_cb, 1);
        xtra_make_btn(scr_performance, bx + 150, ry, 70, 34, "OCT-", RED808_CYAN,    drone_root_cb,  2);
        xtra_make_btn(scr_performance, bx + 226, ry, 70, 34, "OCT+", RED808_CYAN,    drone_root_cb,  3);
        xtra_make_btn(scr_performance, bx + 320, ry, 56, 34, "VEL-", RED808_WARNING, drone_level_cb, 0);
        xtra_make_btn(scr_performance, bx + 380, ry, 56, 34, "VEL+", RED808_WARNING, drone_level_cb, 1);

        int ey = by + 120;   // engine / mode row
        xtra_make_btn(scr_performance, bx + 14,  ey, 200, 36, "ENGINE", theme_text(), drone_engine_cb, 0);
        xtra_make_btn(scr_performance, bx + 226, ey, 210, 36, "MODE",   theme_text(), drone_mode_cb,   0);

        drone_refresh_ui();
    }

    xtra_load_state();
    s_xtra_edit_slot = 0;
    s_xtra_loaded_slot = -1;
    xtra_refresh_panel();
    xtra_editor_render();
}

// =============================================================================
// PUBLIC API
// =============================================================================
void ui_create_all_screens(void) {
    create_boot_screen();
    create_live_screen();
    create_sequencer_screen();
    create_fx_screen();
    create_volumes_screen();
    create_sdcard_screen();
    create_performance_screen();
    create_piano_screen();      /* v2.6 */
    create_piano_params_screen(); /* v2.7 — synth params editor */

    // Start on boot screen
    lv_scr_load(scr_boot);
    active_screen = 0;
}

// =============================================================================
// THEME RELOAD — delete and recreate all themed screens with new colors
// =============================================================================
static void ui_reload_themed_screens(void) {
    int saved_screen = active_screen;

    // Stop the drone before its panel (on scr_performance) is torn down.
    drone_all_off();
    s_drone_on = false;

    // Block the touch task from hit-testing live pads while we delete and
    // recreate screens. Without this, touch_task (Core 0, no LVGL lock) can
    // dereference live_pad_btns[] pointers that are freed below before they are
    // nulled. ui_navigate_to() at the end restores this flag.
    g_live_screen_active.store(false, std::memory_order_release);

    // Navigate to boot temporarily so we can safely delete active screens
    lv_scr_load(scr_boot);

    // Delete all themed screens (nullify pointers before delete to avoid stale refs)
    if (scr_live)        { lv_obj_del(scr_live);        scr_live        = NULL; }
    if (scr_sequencer)   { lv_obj_del(scr_sequencer);   scr_sequencer   = NULL; }
    if (scr_fx)          { lv_obj_del(scr_fx);          scr_fx          = NULL; }
    if (scr_volumes)     { lv_obj_del(scr_volumes);     scr_volumes     = NULL; }
    if (scr_sdcard)      { lv_obj_del(scr_sdcard);      scr_sdcard      = NULL; }
    if (scr_performance) { lv_obj_del(scr_performance); scr_performance = NULL; }

    // Clear widget pointers (prevent stale access in update functions)
    s_drone_toggle_btn = NULL; s_drone_status_lbl = NULL;
    s_xtra_wave_panel = NULL; s_xtra_wave_line = NULL;
    s_xtra_trim_a_line = NULL; s_xtra_trim_b_line = NULL;
    s_xtra_playhead_line = NULL; s_xtra_play_start_ms = 0;
    s_xtra_loop_btn = NULL; s_xtra_loop_on = false;
    s_xtra_info_lbl = NULL; s_xtra_trim_lbl = NULL; s_xtra_fade_lbl = NULL;
    for (int i = 0; i < 4; i++) { grid_xtra_btns[i] = NULL; grid_xtra_delete_btns[i] = NULL; }
    header_bar = NULL; hdr_bpm_label = NULL; hdr_pattern_label = NULL;
    hdr_play_btn = NULL; hdr_play_label = NULL;
    hdr_pattern_minus_btn = NULL; hdr_pattern_plus_btn = NULL;
    hdr_wifi_label = NULL; hdr_s3_label = NULL;
    for (int i = 0; i < 16; i++) hdr_step_dots[i] = NULL;
    for (int i = 0; i < 16; i++) {
        live_pad_btns[i] = NULL; live_pad_labels[i] = NULL;
        live_pad_num_labels[i] = NULL; live_pad_state_labels[i] = NULL;
        live_pad_inst_labels[i] = NULL;
        live_pad_accent_strips[i] = NULL;
        live_spectrum_bars[i] = NULL;
        grid_step_dots[i] = NULL;
    }
    // Invalidate ripple pool — objects are children of scr_live (already deleted)
    for (int i = 0; i < RIPPLE_POOL; i++) {
        ripples[i].obj = nullptr;
        ripples[i].frame = 0;
    }
    grid_play_btn = NULL; grid_play_lbl = NULL; grid_bpm_lbl = NULL;
    grid_pat_lbl = NULL; grid_step_lbl = NULL;
    grid_nr_btn = NULL; grid_nr_lbl = NULL;
    grid_16l_btn = NULL; grid_16l_lbl = NULL;
    grid_mstr_dot = NULL; grid_mstr_lbl = NULL;
    grid_aux_dot  = NULL; grid_aux_lbl  = NULL;
    grid_vol_lbl = NULL; grid_sync_btn = NULL;
    grid_home_vol_lbl = NULL; grid_home_status_cell = NULL;
    grid_pad_prev_btn = NULL;
    grid_pad_next_btn = NULL;
    grid_pad_lbl = NULL;
    grid_inst_prev_btn = NULL;
    grid_inst_next_btn = NULL;
    grid_inst_lbl = NULL;
    grid_inst_edit_btn = NULL;
    s_pad_inst_modal = NULL;
    s_pad_inst_modal_pad_lbl = NULL;
    s_pad_inst_modal_inst_lbl = NULL;
    for (int i = 0; i < 8; i++) s_pad_inst_modal_inst_btns[i] = NULL;
    for (int e2 = 0; e2 < 3; e2++) {
        s_pad_inst_modal_kit_lbl_eng[e2] = NULL;
        for (int p = 0; p < 5; p++) s_pad_inst_modal_kit_btns[e2][p] = NULL;
    }
    for (int i = 0; i < FX_CARD_COUNT; i++) {
        fx_cards[i] = NULL; fx_arcs[i] = NULL; fx_value_labels[i] = NULL;
        fx_name_labels[i] = NULL; fx_src_labels[i] = NULL; fx_toggle_btns[i] = NULL;
        fx_pct_labels[i] = NULL;
    }
    for (int i = 0; i < FX_PAGE_DOT_COUNT; i++) fx_page_dot[i] = NULL;
    fx_page_lbl = NULL;
    fx_view_btn = NULL;
    fx_view_lbl = NULL;
    fx_page = 0;
    fx_view_mode = 0;
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) seq_step_btns[i][j] = NULL;
        seq_track_labels[i] = NULL; seq_mute_btns[i] = NULL;
        seq_solo_btns[i] = NULL;
        seq_solo_labels[i] = NULL;
        seq_ruler_labels[i] = NULL;
    }
    for (int b = 0; b < 4; b++) seq_beat_bg[b] = NULL;
    seq_playhead_line = NULL;
    seq_status_step_lbl = NULL;
    seq_status_pat_lbl = NULL;
    seq_status_bpm_lbl = NULL;
    seq_ctrl_lbl = NULL;
    seq_pattern_modal = NULL;
    seq_pattern_modal_lbl = NULL;
    seq_pattern_modal_spin = NULL;
    seq_pattern_wait_pat = -1;
    seq_pattern_wait_ms = 0;
    seq_pattern_waiting = false;
    for (int i = 0; i < 16; i++) {
        vol_sliders[i] = NULL; vol_labels[i] = NULL;
        vol_name_labels[i] = NULL; vol_mute_dots[i] = NULL;
        vol_strip_panels[i] = NULL; vol_vu_bars[i] = NULL; vol_vu_caps[i] = NULL;
    }

    // Toast lives as a child of whatever screen was active — those are being
    // deleted, so drop the pointers to avoid a use-after-free on the next toast.
    s_ui_toast = NULL; s_ui_toast_label = NULL; s_ui_toast_until_ms = 0;

    // Clear SD screen widgets
    sd_left_panel = NULL; sd_right_panel = NULL; sd_status_lbl = NULL;
    sd_path_lbl = NULL; sd_file_list = NULL; sd_selected_lbl = NULL;
    sd_load_btn = NULL; sd_load_lbl = NULL;
    for (int i = 0; i < 16; i++) sd_pad_btns[i] = NULL;
    sd_wav_section = NULL; sd_midi_section = NULL;
    sd_midi_info_lbl = NULL; sd_midi_status_lbl = NULL; sd_midi_load_btn = NULL;
    for (int i = 0; i < 6; i++) sd_midi_pat_btns[i] = NULL;
    sd_is_midi_mode = false;

    // Lazy theme reload: DON'T recreate every screen here (that rebuilt ~8 full
    // screens and was the visible lag on theme change). We only delete + null
    // them above; the active screen is rebuilt by the ui_navigate_to() at the
    // end, and the rest are recreated on demand by ui_ensure_screen() the first
    // time they're opened.
    /* piano + piano-params are deleted/nulled here too so they get a fresh,
       on-demand rebuild with the new theme colours. */
    if (scr_piano) { lv_obj_del(scr_piano); scr_piano = NULL; }
    s_piano_keys_container = NULL; s_piano_octave_lbl = NULL;
    s_piano_keys24_btn = NULL; s_piano_keys24_lbl = NULL; s_piano_status_lbl = NULL;
    s_piano_expr_bar = NULL;
    s_piano_rec_btn = NULL; s_piano_rec_lbl = NULL;
    s_piano_eng_preset_btn = NULL; s_piano_eng_preset_lbl = NULL;
    for (int i = 0; i < PIANO_ENGINE_COUNT; i++) s_piano_engine_btns[i] = NULL;
    if (scr_piano_params) { lv_obj_del(scr_piano_params); scr_piano_params = NULL; }
    s_pp_param_panel = NULL; s_pp_title_lbl = NULL;
    for (int i = 0; i < SP_ENGINE_COUNT; i++) s_pp_engine_btns[i] = NULL;
    for (int i = 0; i < 4; i++) s_pp_preset_btns[i] = NULL;
    for (int i = 0; i < PP_MAX_PARAMS_P4; i++) {
        s_pp_sliders[i] = NULL; s_pp_val_lbls[i] = NULL;
    }

    // Restore navigation (go to live if was on unknown screen)
    int nav_to = (saved_screen == 9) ? 9 : 2;  // stay in sdcard if we were there
    if (saved_screen == 6) nav_to = 6;
    if (saved_screen == 3) nav_to = 3;
    if (saved_screen == 7) nav_to = 7;
    if (saved_screen == 8) nav_to = 8;
    if (saved_screen == 10) nav_to = 10;   /* PIANO */
    ui_navigate_to(nav_to);
}

// Lazily (re)create a themed screen if it was torn down — e.g. after a theme
// change, which now rebuilds only the active screen and defers the rest to the
// first time they're navigated to. A no-op in normal operation (all screens are
// built eagerly at boot), so it only does work right after a theme switch.
static void ui_ensure_screen(int screen_id) {
    switch (screen_id) {
        case 2:  if (!scr_live)         create_live_screen();         break;
        case 3:  if (!scr_sequencer)    create_sequencer_screen();    break;
        case 6:  if (!scr_performance)  create_performance_screen();  break;
        case 7:  if (!scr_volumes)      create_volumes_screen();      break;
        case 8:  if (!scr_fx)           create_fx_screen();           break;
        case 9:  if (!scr_sdcard)       create_sdcard_screen();       break;
        case 10: if (!scr_piano)        create_piano_screen();        break;
        case 11: if (!scr_piano_params) create_piano_params_screen(); break;
        default: break;
    }
}

void ui_navigate_to(int screen_id) {
    // Don't let the user leave the screen while a blocking op (e.g. WAV upload)
    // is running — the result toast would otherwise land on the wrong screen.
    if (s_ui_busy) return;

    // Build the target screen on demand if a theme reload deferred it.
    ui_ensure_screen(screen_id);

    lv_obj_t* targets[] = {
        scr_boot, NULL, scr_live, scr_sequencer, NULL,
        NULL, scr_performance, scr_volumes, scr_fx, scr_sdcard,
        scr_piano,        /* 10 = PIANO (replaces stubbed performance slot) */
        scr_piano_params  /* 11 = PIANO PARAMS (synth editor) */
    };
    int count = sizeof(targets) / sizeof(targets[0]);
    if (screen_id >= 0 && screen_id < count && targets[screen_id]) {
        if (screen_id == 11) {
            if (s_pp_from_xtra && s_pp_xtra_slot >= 0 && s_pp_xtra_slot < 4) {
                pp_refresh_view();
            } else if (s_piano_engine_idx >= 0 && s_piano_engine_idx < SP_ENGINE_COUNT) {
                s_pp_engine_idx = s_piano_engine_idx;
                pp_refresh_view();
                piano_sync_active_engine_state();
            }
        }
        if (screen_id == 10) {
            if (active_screen == 11 && s_pp_engine_idx >= 0 && s_pp_engine_idx < PIANO_ENGINE_COUNT) {
                s_piano_engine_idx = s_pp_engine_idx;
                piano_refresh_engine_chips();
                if (ui_use_udp_transport()) {
                    udp_send_melody_set_engine(PIANO_ENGINES[s_piano_engine_idx]);
                }
                piano_uart_broadcast_state();
            }
            piano_sync_active_engine_state();
        }
        if (screen_id != 9) s_sd_for_xtra = false;
        // Leaving the XTRA screen: drop the drone so it doesn't get stuck (the
        // synth all-notes-off below would silence it anyway; keep state in sync)
        // and stop/hide the playhead so it doesn't freeze mid-sweep.
        if (active_screen == 6 && screen_id != 6) {
            if (s_drone_on) drone_force_off();
            s_xtra_loop_on = false; xtra_refresh_loop_btn();
            s_xtra_play_start_ms = 0;
            if (s_xtra_playhead_line) lv_obj_add_flag(s_xtra_playhead_line, LV_OBJ_FLAG_HIDDEN);
        }
        bool keep_piano_preview = s_piano_play_active &&
            ((active_screen == 10 && screen_id == 11) || (active_screen == 11 && screen_id == 10));
        // Before leaving most screens, stop active synths to prevent stuck notes.
        // Keep the local Melody preview alive while moving between PIANO and PARAMS.
        if (udp_wifi_connected() && !keep_piano_preview) {
            for (int eng = 0; eng < SYNTH_ENGINE_COUNT; eng++) {
                udp_send_synth_note_off(eng, 0);  // engine, track=0
            }
        }
        
        lv_scr_load_anim(targets[screen_id], LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, false);
        prev_active_screen = active_screen;
        active_screen = screen_id;
        if (screen_id == 3) {
            // Entering sequencer: enforce fresh pattern pull from Master.
            udp_send_select_pattern(p4.current_pattern);
            udp_send_get_pattern(p4.current_pattern);
        }
        // Refresh current storage source when entering SD screen
        if (screen_id == 9) sd_refresh_source();
    }
    // Enable/disable direct touch bypass for live pads
    g_live_screen_active.store(screen_id == 2, std::memory_order_release);
}

// =============================================================================
// PAD QUEUE DRAIN — called from loop() on Core 1 (outside LVGL mutex)
// =============================================================================
// Pending note-off for melodic engines (303/WT/FM2/SH101) so a pad-tap does
// not leave the synth voice ringing forever. Index by pad 0..15.
static uint32_t s_pad_noteoff_at[16]    = {0};
static int8_t   s_pad_noteoff_engine[16] = {-1, -1, -1, -1, -1, -1, -1, -1,
                                           -1, -1, -1, -1, -1, -1, -1, -1};
static uint8_t  s_pad_noteoff_note[16]   = {0};

void ui_process_control_queue(void) {
    if (s_ctrl_mute_mask_pending.exchange(false, std::memory_order_acquire)) {
        uint16_t mask = s_ctrl_mute_mask.load(std::memory_order_acquire);
        udp_send_mute_mask(mask);
    }

    if (s_ctrl_solo_mask_pending.exchange(false, std::memory_order_acquire)) {
        uint16_t mask = s_ctrl_solo_mask.load(std::memory_order_acquire);
        udp_send_solo_mask(mask);
    }

    uint16_t dirty = s_ctrl_mute_dirty.exchange(0, std::memory_order_acquire);
    if (dirty) {
        uint16_t values = s_ctrl_mute_values.load(std::memory_order_acquire);
        for (uint8_t track = 0; track < 16; track++) {
            uint16_t bit = (uint16_t)(1U << track);
            if (dirty & bit) {
                udp_send_mute(track, (values & bit) != 0);
            }
        }
    }
}

void ui_process_pad_queue(void) {
    uint32_t now_ms = millis();
    uint8_t t = s_pad_qt.load(std::memory_order_relaxed);
    uint8_t h = s_pad_qh.load(std::memory_order_acquire);
    while (t != h) {
        uint16_t ev = s_pad_q[t & 0x1F];
        t++;
        uint8_t pad      = (uint8_t)(ev & 0xFF);
        uint8_t velocity = (uint8_t)((ev >> 8) & 0xFF);
        if (!velocity) velocity = 100;   // defensive floor
        // Feed DSP spectrum with real velocity
        dsp_notify_pad(pad, velocity);
        // Notify S3 via UART (fast, 5 bytes). Legacy TCMD_PAD_TAP ignores
        // velocity; we still send it for forward compatibility.
        uart_send_to_s3(MSG_TOUCH_CMD, TCMD_PAD_TAP, pad);
        // Then send UDP to master with MPC-style velocity
        if (p4.wifi_connected || p4.master_connected) {
            int8_t engine = (pad < 16) ? pad_inst_engine_code(s_pad_inst_sel[pad]) : -1;
            bool melodic  = (engine >= 3 && engine <= 6);  // 303/WT/FM2/SH101

            // Kit per-pad: si el pad usa un engine drum (808/909/505) y su
            // kit asignado difiere del último aplicado a ese engine en la
            // Daisy, manda CMD_SYNTH_PRESET justo antes del trigger. Esto
            // permite que dos pads del mismo engine suenen con kits distintos
            // (a costa de un cambio de preset por golpe cuando alternan).
            if (pad < 16) {
                int8_t drum = pad_inst_drum_engine_idx(s_pad_inst_sel[pad]);
                if (drum >= 0) {
                    uint8_t kit = s_pad_kit_assigned[pad];
                    if (kit > 4) kit = 0;
                    if (s_engine_kit_last_applied[drum] != (int8_t)kit) {
                        udp_send_synth_preset((uint8_t)drum, kit);
                        s_engine_kit_last_applied[drum] = (int8_t)kit;
                    }
                }
            }

            if (melodic) {
                // Melodic synth engines don't sound from a bare trigger(pad):
                // they need an explicit note, exactly like the XTRA pads. Map
                // the 16 pads chromatically and schedule the matching note-off.
                int note = constrain(48 + (int)pad, 24, 96);
                udp_send_synth_note_on_ex((uint8_t)engine, (uint8_t)note, velocity, false, false);
                s_pad_noteoff_engine[pad] = engine;
                s_pad_noteoff_note[pad]   = (uint8_t)note;
                s_pad_noteoff_at[pad]     = now_ms + 220;
            } else {
                udp_send_trigger(pad, velocity);
            }
        }
        // Mirror to legacy binary flash timer so screens that still read
        // p4.pad_flash_until (e.g. sequencer sync highlight) keep working.
        p4.pad_flash_until[pad] = millis() + 80;
    }
    s_pad_qt.store(t, std::memory_order_relaxed);

    // Deferred "leaving LIVE" all-notes-off, staged by the touch task. Sent here
    // on Core 1 so UDP is never driven from the 200 Hz touch task.
    if (s_pending_live_allnotesoff.exchange(false, std::memory_order_acquire)) {
        if (udp_wifi_connected()) {
            for (int eng = 0; eng < SYNTH_ENGINE_COUNT; eng++) {
                udp_send_synth_note_off(eng, 0);
            }
        }
    }

    // Drain pending melodic note-offs.
    for (int pad = 0; pad < 16; pad++) {
        if (!s_pad_noteoff_at[pad]) continue;
        if ((int32_t)(now_ms - s_pad_noteoff_at[pad]) < 0) continue;
        int8_t engine = s_pad_noteoff_engine[pad];
        uint8_t note  = s_pad_noteoff_note[pad];
        s_pad_noteoff_at[pad]     = 0;
        s_pad_noteoff_engine[pad] = -1;
        if (!(p4.wifi_connected || p4.master_connected)) continue;
        // Release the note we played, matching the XTRA melodic note-off.
        if (engine >= 3 && engine <= 6) {
            udp_send_synth_note_off_ex((uint8_t)engine, 0, note);
        }
    }
}

// =============================================================================
// LIVE PAD HIT GEOMETRY — shared between LVGL layout and GT911 touch_task
// =============================================================================
// Pad grid geometry (must match create_live_screen layout below).
// M=8 (margin), CW/CH = pad size, SX/SY = stride (pad + gap).
static constexpr int LIVE_M  = 8;
static constexpr int LIVE_CW = 122;
static constexpr int LIVE_CH = 143;
static constexpr int LIVE_SX = 126;
static constexpr int LIVE_SY = 147;

int ui_pad_from_xy(uint16_t x, uint16_t y, uint8_t* cell_x, uint8_t* cell_y) {
    if (cell_x) *cell_x = 64;
    if (cell_y) *cell_y = 64;
    if (!g_live_screen_active.load(std::memory_order_acquire)) return -1;
    // Dynamic hit-test against current pad geometry (PAD MODE aware).
    for (int i = 0; i < 16; i++) {
        lv_obj_t* b = live_pad_btns[i];
        if (!b) continue;
        if (lv_obj_has_flag(b, LV_OBJ_FLAG_HIDDEN)) continue;
        lv_coord_t px = lv_obj_get_x(b);
        lv_coord_t py = lv_obj_get_y(b);
        lv_coord_t pw = lv_obj_get_width(b);
        lv_coord_t ph = lv_obj_get_height(b);
        if ((int)x >= px && (int)x < (px + pw) && (int)y >= py && (int)y < (py + ph)) {
            int dx = (int)x - (int)px;
            int dy = (int)y - (int)py;
            int denom_x = (pw > 1) ? (pw - 1) : 1;
            int denom_y = (ph > 1) ? (ph - 1) : 1;
            if (cell_x) *cell_x = (uint8_t)constrain((dx * 127) / denom_x, 0, 127);
            if (cell_y) *cell_y = (uint8_t)constrain((dy * 127) / denom_y, 0, 127);
            return i;
        }
    }

    // Legacy fallback geometry (default 4x4 layout)
    if (x < LIVE_M || x >= (LIVE_M + 4 * LIVE_SX)) return -1;
    if (y < LIVE_M || y >= (LIVE_M + 4 * LIVE_SY)) return -1;
    int col  = (x - LIVE_M) / LIVE_SX;
    int row  = (y - LIVE_M) / LIVE_SY;
    int x_in = (x - LIVE_M) % LIVE_SX;
    int y_in = (y - LIVE_M) % LIVE_SY;
    if (x_in >= LIVE_CW || y_in >= LIVE_CH) return -1;
    if (col >= 4 || row >= 4) return -1;
    if (cell_x) *cell_x = (uint8_t)constrain((x_in * 127) / (LIVE_CW - 1), 0, 127);
    if (cell_y) *cell_y = (uint8_t)constrain((y_in * 127) / (LIVE_CH - 1), 0, 127);
    return row * 4 + col;
}

static inline uint8_t ui_live_pad_velocity(void) {
    int volume = constrain(p4.live_volume, 0, Config::MAX_VOLUME);
    return (uint8_t)map(volume, 0, Config::MAX_VOLUME, 32, 127);
}

// =============================================================================
// PAD FRAME UPDATE — called from GT911 touch_task (Core 0, 200Hz)
// Rising edge → enqueue event (with 16 Levels remapping if active) and arm
// note-repeat timer. Falling edge → cancel repeat. Held → fire repeats on
// schedule using the current tempo & subdivision.
// =============================================================================
void ui_pad_frame_update(const bool pressed[16], const uint8_t velocity[16],
                         const uint8_t cell_x[16], const uint8_t cell_y[16]) {
    (void)velocity;
    static bool prev_live_active = true;

    // Si el modal PAD SOUND está abierto, ignora los pads físicos:
    // la pantalla está cubierta y cualquier toque debe ir a los
    // botones del modal, no al pad físico que hay debajo.
    if (s_pad_inst_modal) {
        for (int p = 0; p < 16; p++) {
            s_pad_held[p] = false;
            s_pad_repeat_next_ms[p] = 0;
            s_pad_hold_start_ms[p] = 0;
            s_pad_roll_phase[p] = 0;
        }
        return;
    }

    if (!g_live_screen_active.load(std::memory_order_acquire)) {
        // Leaving LIVE screen: release all held pads to Master so they don't sustain.
        if (prev_live_active) {  // only on transition OUT of LIVE
            // This runs on the 200 Hz touch task (Core 0). Do NOT send UDP from
            // here — defer the all-notes-off to Core 1 (ui_process_pad_queue),
            // so the WiFiUDP socket is only ever driven from the main loop.
            s_pending_live_allnotesoff.store(true, std::memory_order_release);
            prev_live_active = false;
        }
        
        // Clear state so we don't fire phantom repeats when leaving LIVE
        for (int p = 0; p < 16; p++) {
            s_pad_held[p] = false;
            s_pad_repeat_next_ms[p] = 0;
            s_pad_hold_start_ms[p] = 0;
            s_pad_roll_phase[p] = 0;
        }
        return;
    }
    
    // Entering LIVE screen: reset transition flag
    // Entering/in LIVE screen: reset transition flag for next time we leave
    prev_live_active = true;

    unsigned long now = millis();
    unsigned long nr_interval = ui_nr_interval_ms();    // 0 if NR off

    for (int p = 0; p < 16; p++) {
        bool was_held = s_pad_held[p];
        bool is_held  = pressed[p];
        if (is_held) {
            s_pad_hold_x[p] = cell_x ? cell_x[p] : 64;
            s_pad_hold_y[p] = cell_y ? cell_y[p] : 64;
        }

        if (is_held && !was_held) {
            // ── Rising edge: real finger-down ──
            uint8_t vel = ui_live_pad_velocity();
            s_pad_held_velocity[p] = vel;
            s_pad_hold_start_ms[p] = now;
            s_pad_roll_phase[p] = 0;

            uint8_t send_pad = p;
            uint8_t send_vel = vel;
            if (s_16l_active) {
                // Remap to 16 velocities of the stored source pad
                send_pad = s_16l_src_pad;
                send_vel = (uint8_t)(((p + 1) * 127) / 16);  // 7..127
                if (send_vel < 8) send_vel = 8;
            } else {
                s_16l_src_pad = (uint8_t)p;   // remember for future 16L
            }
            s_pad_inst_focus_pad = (uint8_t)p;
            if (send_pad < 16 && !p4.track_muted[send_pad]) {
                enqueue_pad_event(send_pad, send_vel);
                ui_pad_flash_start(p, vel);
            } else {
                s_pad_flash_vel[p] = 0;
            }

            unsigned long tremolo_interval = ui_pad_tremolo_interval_ms((uint8_t)p, nr_interval);
            s_pad_repeat_next_ms[p] = tremolo_interval
                ? (now + (nr_interval ? tremolo_interval : PAD_TREMOLO_HOLD_MS)) : 0;
        } else if (!is_held && was_held) {
            // ── Falling edge: finger lifted ──
            s_pad_repeat_next_ms[p] = 0;
            s_pad_hold_start_ms[p] = 0;
            s_pad_roll_phase[p] = 0;
        } else if (is_held && s_pad_repeat_next_ms[p]
                   && now >= s_pad_repeat_next_ms[p]) {
            // ── Held + tremolo/note-repeat tick ──
            unsigned long tremolo_interval = ui_pad_tremolo_interval_ms((uint8_t)p, nr_interval);
            if (!tremolo_interval) {
                s_pad_repeat_next_ms[p] = 0;
                s_pad_held[p] = is_held;
                continue;
            }
            s_pad_roll_phase[p] = (uint8_t)(s_pad_roll_phase[p] + 1);
            uint8_t vel = ui_pad_tremolo_velocity((uint8_t)p, now);
            uint8_t send_pad = p;
            uint8_t send_vel = vel;
            if (s_16l_active) {
                send_pad = s_16l_src_pad;
                send_vel = (uint8_t)(((p + 1) * 127) / 16);
                if (send_vel < 8) send_vel = 8;
            }
            if (send_pad < 16 && !p4.track_muted[send_pad]) {
                enqueue_pad_event(send_pad, send_vel);
                ui_pad_flash_start(p, vel);
            } else {
                s_pad_flash_vel[p] = 0;
            }
            // Schedule next tick; if we fell behind, catch up without drifting
            // into the far past (e.g. after a blocked frame).
            unsigned long next = s_pad_repeat_next_ms[p] + tremolo_interval;
            if (next <= now) next = now + tremolo_interval;
            s_pad_repeat_next_ms[p] = next;
        }

        s_pad_held[p] = is_held;
    }
}

void ui_update_current_screen(void) {
    unsigned long now = millis();
    static unsigned long boot_enter_ms = 0;
    ui_toast_update();

    // Apply deferred master→UI track-engine sync staged from Core 1 (UDP).
    // Done here so the LVGL object writes happen on the render task under lock.
    if (s_pending_track_engines_valid.exchange(false, std::memory_order_acquire)) {
        apply_pad_sound_track_engines(s_pending_track_engines);
    }

    // Apply deferred melody state from S3/master (engine/octave/rec/pad). This
    // path was previously dead — the pending flag was set but never consumed,
    // so the P4 piano UI never reflected remote melody changes.
    if (g_pending_melody_from_s3.pending) {
        g_pending_melody_from_s3.pending = false;
        piano_apply_melody_sync(g_pending_melody_from_s3.engine,
                                g_pending_melody_from_s3.octave,
                                g_pending_melody_from_s3.rec != 0,
                                g_pending_melody_from_s3.pad);
    }

    // Auto-navigate from boot to live when Master or optional S3 connects
    if (active_screen == 0) {
        if (boot_enter_ms == 0) boot_enter_ms = now;
        if (p4.master_connected || p4.s3_connected || (now - boot_enter_ms) > 5000UL) {
            ui_navigate_to(2);  // SCREEN_LIVE
        }
    } else {
        boot_enter_ms = 0;
    }

    // Theme change — recreate all screens with new palette
    static int prev_theme = -1;
    if (p4.theme != prev_theme && prev_theme != -1) {
        prev_theme = p4.theme;
        ui_theme_apply((VisualTheme)p4.theme);
        ui_reload_themed_screens();
        return;  // screens recreated; update functions have fresh state
    }
    if (prev_theme == -1) prev_theme = p4.theme;

    // Navigate if S3 sends screen command
    static int prev_screen = -1;
    if (p4.current_screen != prev_screen) {
        int requested = p4.current_screen;
        prev_screen = requested;
        // Ignore remote BOOT requests once UI has left boot screen.
        // S3 can transiently report SCREEN_BOOT during startup sync.
        if (!(requested == 0 && active_screen != 0)) {
            ui_navigate_to(requested);
        }
    }

    ui_update_header();

    // Force fx_screen repaint IMMEDIATELY when UDP receives new FX values.
    // Must be BEFORE the period throttle so dirty updates aren't delayed up to 33ms.
    if (g_fx_screen_dirty) {
        g_fx_screen_dirty = false;
        update_fx_screen();
    }

    // Per-screen pacing. LIVE and STEPS need 60Hz for pad fades/playhead.
    // Static editors do not: most interaction is handled by event callbacks.
    static unsigned long last_active_update_ms = 0;
    uint32_t period_ms = 33;
    lv_obj_t* active = lv_scr_act();
    if (active == scr_live) period_ms = 16;
    else if (active == scr_sequencer) period_ms = 8;
    else if (active == scr_sdcard) period_ms = p4sd.needs_refresh ? 16 : 100;
    else if (active == scr_piano) period_ms = 16;
    else if (active == scr_piano_params) period_ms = 50;
    else if (active == scr_fx) period_ms = 16;
    if (now - last_active_update_ms < period_ms) return;
    last_active_update_ms = now;

    // Update active screen content
    if (active == scr_live) update_live_screen();
    else if (active == scr_sequencer) update_sequencer_screen();
    else if (active == scr_fx) update_fx_screen();
    else if (active == scr_volumes) update_volumes_screen();
    else if (active == scr_performance) update_xtra_playhead();
    else if (active == scr_piano || active == scr_piano_params) update_piano_screen();
    else if (active == scr_sdcard && p4sd.needs_refresh) {
        p4sd.needs_refresh = false;
        sd_refresh_ui();
    }
}
