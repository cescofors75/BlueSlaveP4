// =============================================================================
// udp_handler.h — WiFi/UDP communication to Master via ESP32-C6
// P4 connects directly to Master AP (RED808) through SDIO ESP-Hosted
// =============================================================================
#pragma once

#include <stdint.h>
#include "../../shared/raydrone_protocol.h"

// Initialize WiFi + UDP
void udp_handler_init(void);

// Process WiFi state + incoming UDP packets — call from main loop
void udp_handler_process(void);

// Connection state
bool udp_wifi_connected(void);
bool udp_master_connected(void);

// Absolute playing step (0..63) of the master pattern. p4.current_step is
// folded to 0..15 for the 16-step widgets; multi-bar pad sync needs this.
int udp_current_step_raw(void);

// =============================================================================
// Commands to Master (P4 → Master)
// =============================================================================
void udp_send_trigger(uint8_t pad, uint8_t velocity);
void udp_send_start(void);
void udp_send_stop(void);
void udp_send_tempo(float bpm);
void udp_send_select_pattern(int index);
void udp_send_get_pattern(int pattern);
void udp_send_set_step(int track, int step, bool active);
void udp_send_set_step_velocity(int track, int step, int velocity);
void udp_send_mute(int track, bool muted);
void udp_send_set_volume(int value);
void udp_send_set_seq_volume(int value);
void udp_send_set_live_volume(int value);
void udp_send_set_track_volume(int track, int volume);
void udp_send_set_track_engine(int track, int engine);

// FX commands — legacy filter
void udp_send_set_filter(int type);
void udp_send_set_filter_cutoff(int hz);
void udp_send_set_filter_resonance(float val);
void udp_send_set_distortion(float val);
void udp_send_set_bitcrush(int bits);
void udp_send_set_sample_rate(int rateHz);

// FX live commands (encoder/pot → Master)
// enc_id: 0=Flanger, 1=Delay, 2=Reverb
void udp_send_fx_enc(int enc_id, uint8_t value, bool muted);
// pot_id: 0=Fold macro, 1=Crush macro, 2=Phaser macro
void udp_send_fx_pot(int pot_id, uint8_t value, bool muted);

// RayDrone uses partial JSON edits; S3 merges them into one atomic Daisy payload.
void udp_raydrone_toggle_active(void);
void udp_raydrone_cycle_material(void);
void udp_set_raydrone_value(uint8_t update_mask, uint8_t value,
                            bool transmit);

// Solo
void udp_send_solo(int track, bool soloed);

// Atomic batch updates (single UDP packet sets all 16 tracks at once)
void udp_send_mute_mask(uint16_t mask);
void udp_send_solo_mask(uint16_t mask);

// Cross-core FX repaint notification. Producers may call the marker from any
// task; the LVGL task consumes it atomically so a simultaneous update is never
// lost while the flag is being cleared.
void udp_mark_fx_screen_dirty(void);
bool udp_consume_fx_screen_dirty(void);

// Synth (melody) — engine: 3=303, 4=WTosc, 5=SH101, 6=FM2Op
void udp_send_synth_note_on_ex(uint8_t engine, uint8_t note, uint8_t velocity,
                                bool accent, bool slide);
void udp_send_synth_note_off(uint8_t engine, uint8_t track);
void udp_send_synth_note_off_ex(uint8_t engine, uint8_t track, uint8_t note);
void udp_send_synth303_note_off(void);
void udp_send_synth_trigger(uint8_t engine, uint8_t instrument, uint8_t velocity);

// Synth parameter editor
void udp_send_synth_param(uint8_t engine, uint8_t instrument, uint8_t paramId, float value);
void udp_send_synth_preset(uint8_t engine, uint8_t preset);

// Non-destructive sample editor on Master. Values are normalized 0..1 and
// operate on the already-loaded pad sample.
void udp_send_trim_sample(uint8_t pad, float trim_start, float trim_end);

// v2.7 — record-mode note for the S3 melody screen (master forwards to all slaves)
void udp_send_melody_rec_note(uint8_t engine, uint8_t note);

// v2.8 — assign a recorded 16×12 melody grid to a pad (master forwards)
// pad: 0..15, engine: 3..6, octave: 1..7,
// grid[col][row]: row 0 = B (highest), row 11 = C (lowest)
void udp_send_melody_assign(uint8_t pad, uint8_t engine, uint8_t octave,
                            const bool grid[16][12],
                            const uint8_t notes[16][12] = nullptr,
                            uint8_t gate_percent = 55);

// v2.9 — master-authoritative melody state commands
void udp_send_melody_rec_toggle(bool active, uint8_t engine, uint8_t octave);
void udp_send_melody_set_pad(uint8_t pad);
void udp_send_melody_set_engine(uint8_t engine);
void udp_send_melody_set_octave(uint8_t octave);
void udp_send_melody_clear(void);
void udp_send_melody_assign_pad(uint8_t pad, uint8_t engine, uint8_t octave);

// Request full sync from Master
void udp_request_master_sync(void);
