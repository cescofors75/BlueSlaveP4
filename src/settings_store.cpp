// =============================================================================
// settings_store.cpp — persisted user settings (NVS via Preferences)
// =============================================================================
#include "settings_store.h"
#include "uart_handler.h"
#include "ui/ui_theme.h"
#include "../include/config.h"
#include <Arduino.h>
#include <Preferences.h>

static Preferences s_prefs;

// Values staged by settings_load(), applied by settings_apply().
struct PersistedSettings {
    uint8_t theme         = THEME_OCEAN;
    uint8_t master_volume = 75;
    uint8_t seq_volume    = 75;
    uint8_t live_volume   = 75;
    uint8_t bpm_int       = Config::DEFAULT_BPM;
    uint8_t bpm_frac      = 0;
    uint8_t pattern       = 0;
    uint8_t track_volume[16] = {75, 75, 75, 75, 75, 75, 75, 75,
                                75, 75, 75, 75, 75, 75, 75, 75};
};
static PersistedSettings s_loaded;
static bool s_nvs_ok = false;

void settings_load(void) {
    s_nvs_ok = s_prefs.begin("p4cfg", false);
    if (!s_nvs_ok) {
        P4_LOG_PRINTLN("[CFG] NVS open failed — using defaults");
        return;
    }
    s_loaded.theme         = s_prefs.getUChar("theme", s_loaded.theme);
    s_loaded.master_volume = s_prefs.getUChar("mvol",  s_loaded.master_volume);
    s_loaded.seq_volume    = s_prefs.getUChar("svol",  s_loaded.seq_volume);
    s_loaded.live_volume   = s_prefs.getUChar("lvol",  s_loaded.live_volume);
    s_loaded.bpm_int       = s_prefs.getUChar("bpmi",  s_loaded.bpm_int);
    s_loaded.bpm_frac      = s_prefs.getUChar("bpmf",  s_loaded.bpm_frac);
    s_loaded.pattern       = s_prefs.getUChar("pat",   s_loaded.pattern);
    s_prefs.getBytes("tvol", s_loaded.track_volume, sizeof(s_loaded.track_volume));

    // Sanitize — a corrupt blob must not produce out-of-range state.
    if (s_loaded.theme >= THEME_COUNT) s_loaded.theme = THEME_OCEAN;
    if (s_loaded.master_volume > 150) s_loaded.master_volume = 75;
    if (s_loaded.seq_volume > 150)    s_loaded.seq_volume = 75;
    if (s_loaded.live_volume > 150)   s_loaded.live_volume = 75;
    if (s_loaded.bpm_int < 40 || s_loaded.bpm_int > 240) s_loaded.bpm_int = Config::DEFAULT_BPM;
    if (s_loaded.bpm_frac > 9)  s_loaded.bpm_frac = 0;
    if (s_loaded.pattern > 15)  s_loaded.pattern = 0;
    for (int i = 0; i < 16; i++) {
        if (s_loaded.track_volume[i] > 150) s_loaded.track_volume[i] = 75;
    }
    P4_LOG_PRINTF("[CFG] loaded theme=%u bpm=%u.%u pat=%u\n",
                  s_loaded.theme, s_loaded.bpm_int, s_loaded.bpm_frac, s_loaded.pattern);
}

int settings_boot_theme(void) {
    return s_loaded.theme;
}

void settings_apply(void) {
    p4.theme           = s_loaded.theme;
    p4.master_volume   = s_loaded.master_volume;
    p4.seq_volume      = s_loaded.seq_volume;
    p4.live_volume     = s_loaded.live_volume;
    p4.bpm_int         = s_loaded.bpm_int;
    p4.bpm_frac        = s_loaded.bpm_frac;
    p4.current_pattern = s_loaded.pattern;
    for (int i = 0; i < 16; i++) p4.track_volume[i] = s_loaded.track_volume[i];
}

void settings_tick(void) {
    if (!s_nvs_ok) return;

    static uint32_t last_check_ms = 0;
    static uint32_t last_change_ms = 0;
    static bool dirty = false;
    static PersistedSettings snap = s_loaded;   // last persisted/observed state

    uint32_t now = millis();
    if (now - last_check_ms < 1000) {
        // Flush path still runs below even between checks.
    } else {
        last_check_ms = now;
        PersistedSettings cur;
        cur.theme         = (uint8_t)constrain(p4.theme, 0, THEME_COUNT - 1);
        cur.master_volume = (uint8_t)constrain(p4.master_volume, 0, 150);
        cur.seq_volume    = (uint8_t)constrain(p4.seq_volume, 0, 150);
        cur.live_volume   = (uint8_t)constrain(p4.live_volume, 0, 150);
        cur.bpm_int       = (uint8_t)constrain(p4.bpm_int, 40, 240);
        cur.bpm_frac      = (uint8_t)constrain(p4.bpm_frac, 0, 9);
        cur.pattern       = (uint8_t)constrain(p4.current_pattern, 0, 15);
        for (int i = 0; i < 16; i++)
            cur.track_volume[i] = (uint8_t)constrain(p4.track_volume[i], 0, 150);

        if (memcmp(&cur, &snap, sizeof(cur)) != 0) {
            snap = cur;
            dirty = true;
            last_change_ms = now;
        }
    }

    // Debounce: persist 3 s after the last observed change. NVS skips writes
    // whose value is unchanged, so flash wear stays minimal.
    if (dirty && (now - last_change_ms >= 3000)) {
        dirty = false;
        s_prefs.putUChar("theme", snap.theme);
        s_prefs.putUChar("mvol",  snap.master_volume);
        s_prefs.putUChar("svol",  snap.seq_volume);
        s_prefs.putUChar("lvol",  snap.live_volume);
        s_prefs.putUChar("bpmi",  snap.bpm_int);
        s_prefs.putUChar("bpmf",  snap.bpm_frac);
        s_prefs.putUChar("pat",   snap.pattern);
        s_prefs.putBytes("tvol",  snap.track_volume, sizeof(snap.track_volume));
        P4_LOG_PRINTF("[CFG] persisted theme=%u bpm=%u.%u pat=%u\n",
                      snap.theme, snap.bpm_int, snap.bpm_frac, snap.pattern);
    }
}
