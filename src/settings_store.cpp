// =============================================================================
// settings_store.cpp — persisted user settings (NVS via Preferences)
// =============================================================================
#include "settings_store.h"
#include "uart_handler.h"
#include "ui/ui_theme.h"
#include "../include/config.h"
#include <Arduino.h>
#include <Preferences.h>
#include <atomic>
#include <freertos/task.h>

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
static bool s_needs_migration = false;

static constexpr uint32_t SETTINGS_MAGIC = 0x50344346u; // "P4CF"
static constexpr uint16_t SETTINGS_VERSION = 1;
struct PersistedBlob {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_size;
    PersistedSettings settings;
    uint32_t checksum;
};

static uint32_t settings_checksum(const PersistedSettings& settings) {
    const uint8_t* bytes = (const uint8_t*)&settings;
    uint32_t hash = 2166136261u; // FNV-1a
    for (size_t i = 0; i < sizeof(settings); i++) {
        hash ^= bytes[i];
        hash *= 16777619u;
    }
    return hash;
}

static PersistedBlob s_write_blob = {};
static std::atomic<uint8_t> s_write_state{0}; // 0 idle, 1 writing, 2 done
static bool s_write_ok = false;

static void settings_write_task(void* arg) {
    (void)arg;
    s_write_ok = (s_prefs.putBytes("blob", &s_write_blob,
                                   sizeof(s_write_blob)) == sizeof(s_write_blob));
    s_write_state.store(2, std::memory_order_release);
    vTaskDelete(NULL);
}

void settings_load(void) {
    s_nvs_ok = s_prefs.begin("p4cfg", false);
    if (!s_nvs_ok) {
        P4_LOG_PRINTLN("[CFG] NVS open failed — using defaults");
        return;
    }
    bool loaded_blob = false;
    if (s_prefs.getBytesLength("blob") == sizeof(PersistedBlob)) {
        PersistedBlob blob = {};
        if (s_prefs.getBytes("blob", &blob, sizeof(blob)) == sizeof(blob) &&
            blob.magic == SETTINGS_MAGIC &&
            blob.version == SETTINGS_VERSION &&
            blob.payload_size == sizeof(PersistedSettings) &&
            blob.checksum == settings_checksum(blob.settings)) {
            s_loaded = blob.settings;
            loaded_blob = true;
        }
    }

    if (!loaded_blob) {
        // Backward-compatible one-time import from the old multi-key layout.
        s_loaded.theme         = s_prefs.getUChar("theme", s_loaded.theme);
        s_loaded.master_volume = s_prefs.getUChar("mvol",  s_loaded.master_volume);
        s_loaded.seq_volume    = s_prefs.getUChar("svol",  s_loaded.seq_volume);
        s_loaded.live_volume   = s_prefs.getUChar("lvol",  s_loaded.live_volume);
        s_loaded.bpm_int       = s_prefs.getUChar("bpmi",  s_loaded.bpm_int);
        s_loaded.bpm_frac      = s_prefs.getUChar("bpmf",  s_loaded.bpm_frac);
        s_loaded.pattern       = s_prefs.getUChar("pat",   s_loaded.pattern);
        s_prefs.getBytes("tvol", s_loaded.track_volume, sizeof(s_loaded.track_volume));
        s_needs_migration = true;
    }

    // Sanitize — a corrupt or obsolete blob must not produce out-of-range
    // state. Persist repaired values through the migration path.
    bool repaired = false;
    if (s_loaded.theme >= THEME_COUNT) { s_loaded.theme = THEME_OCEAN; repaired = true; }
    if (s_loaded.master_volume > 150) { s_loaded.master_volume = 75; repaired = true; }
    if (s_loaded.seq_volume > 150)    { s_loaded.seq_volume = 75; repaired = true; }
    if (s_loaded.live_volume > 150)   { s_loaded.live_volume = 75; repaired = true; }
    if (s_loaded.bpm_int < 40 || s_loaded.bpm_int > 240) {
        s_loaded.bpm_int = Config::DEFAULT_BPM;
        repaired = true;
    }
    if (s_loaded.bpm_frac > 9)  { s_loaded.bpm_frac = 0; repaired = true; }
    if (s_loaded.pattern > 15)  { s_loaded.pattern = 0; repaired = true; }
    for (int i = 0; i < 16; i++) {
        if (s_loaded.track_volume[i] > 150) {
            s_loaded.track_volume[i] = 75;
            repaired = true;
        }
    }
    if (repaired) s_needs_migration = true;
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
    static bool dirty = s_needs_migration;
    static PersistedSettings snap = s_loaded;   // last persisted/observed state

    uint32_t now = millis();

    if (s_write_state.load(std::memory_order_acquire) == 2) {
        bool matches_latest = memcmp(&s_write_blob.settings, &snap, sizeof(snap)) == 0;
        if (s_write_ok) {
            s_loaded = s_write_blob.settings;
            s_needs_migration = false;
            if (matches_latest) dirty = false;
            P4_LOG_PRINTF("[CFG] persisted theme=%u bpm=%u.%u pat=%u\n",
                          s_write_blob.settings.theme,
                          s_write_blob.settings.bpm_int,
                          s_write_blob.settings.bpm_frac,
                          s_write_blob.settings.pattern);
        } else {
            dirty = true;
            last_change_ms = now;
            P4_LOG_PRINTLN("[CFG] persist failed; retry scheduled");
        }
        s_write_state.store(0, std::memory_order_release);
    }

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

    // Debounce: persist 3 s after the last observed change. One versioned
    // blob replaces eight independent NVS writes, preventing partial state
    // after power loss and reducing flash journal churn.
    if (dirty && (now - last_change_ms >= 3000) &&
        s_write_state.load(std::memory_order_acquire) == 0) {
        s_write_blob = {};
        s_write_blob.magic = SETTINGS_MAGIC;
        s_write_blob.version = SETTINGS_VERSION;
        s_write_blob.payload_size = sizeof(PersistedSettings);
        s_write_blob.settings = snap;
        s_write_blob.checksum = settings_checksum(s_write_blob.settings);
        s_write_state.store(1, std::memory_order_release);
        if (xTaskCreatePinnedToCore(settings_write_task, "cfgwrite", 3072,
                                    NULL, 1, NULL, 1) != pdPASS) {
            s_write_state.store(0, std::memory_order_release);
            last_change_ms = now;
            P4_LOG_PRINTLN("[CFG] failed to create writer task");
        }
    }
}
