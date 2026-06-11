// =============================================================================
// settings_store.h — persisted user settings (NVS)
// Theme, volumes, BPM and last pattern survive reboots. The master remains
// authoritative when connected (its state_sync overwrites these on arrival);
// persistence matters for boot defaults and standalone sessions.
// =============================================================================
#pragma once

// Read NVS. Call once in setup() BEFORE ui_theme_apply()/ui_create_all_screens().
// Only stages the values; theme is exposed via settings_boot_theme().
void settings_load(void);

// Persisted theme to apply at boot (falls back to THEME_OCEAN).
int settings_boot_theme(void);

// Overwrite the p4 runtime defaults with the persisted values. Call AFTER
// uart_handler_init() (which seeds the hardcoded defaults).
void settings_apply(void);

// Debounced persistence: snapshots p4 once per second and writes to NVS 3 s
// after the last change. Call from loop() (Core 1 — NVS writes block ~ms).
void settings_tick(void);
