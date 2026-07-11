// =============================================================================
// mem_midi_loader.cpp — SMF drum parser for P4 (reads from SPIFFS)
// =============================================================================
// Minimal SMF parser adapted from BlueSlaveV2/src/sd_midi_loader.cpp. The main
// differences:
//   * File source is SPIFFS (P4 internal flash) instead of SD_MMC.
//   * Output is a single-bar 16×16 step grid; multi-bar MIDIs are folded by
//     OR-ing bars 2..N onto bar 1 to match the fixed 16-step Master protocol.
//   * Event buffer lives in PSRAM (P4 has 32 MB OPI).
// =============================================================================

#include "mem_midi_loader.h"
#include <Arduino.h>
#include <SPIFFS.h>
#include <FS.h>
#include <cstring>
#include <esp_heap_caps.h>

// ---------------------------------------------------------------------------
// GM drum note → our track index (same table as the S3 parser).
// 0xFF means note is ignored.
// ---------------------------------------------------------------------------
static const uint8_t NOTE_MAP[47] = {
    /*35*/ 0,  /*36*/ 0,  /*37*/ 6,  /*38*/ 1,  /*39*/ 4,  /*40*/ 1,
    /*41*/ 11, /*42*/ 2,  /*43*/ 11, /*44*/ 2,  /*45*/ 11, /*46*/ 3,
    /*47*/ 13, /*48*/ 10, /*49*/ 9,  /*50*/ 10, /*51*/ 9,  /*52*/ 9,
    /*53*/ 9,  /*54*/ 8,  /*55*/ 9,  /*56*/ 5,  /*57*/ 9,  /*58*/ 7,
    /*59*/ 9,  /*60*/ 14, /*61*/ 15, /*62*/ 14, /*63*/ 14, /*64*/ 15,
    /*65*/ 10, /*66*/ 11, /*67*/ 7,  /*68*/ 7,  /*69*/ 8,  /*70*/ 8,
    /*71*/ 7,  /*72*/ 7,  /*73*/ 8,  /*74*/ 8,  /*75*/ 7,  /*76*/ 7,
    /*77*/ 7,  /*78*/ 12, /*79*/ 12, /*80*/ 7,  /*81*/ 7,
};

static inline uint8_t gm_note_to_track(uint8_t note) {
    if (note >= 35 && note <= 81) return NOTE_MAP[note - 35];
    return 0xFF;
}

// Per-channel mapping for MELODIC (non-ch10) notes. Each MIDI channel is
// assigned its OWN kit slot, so different instruments in the SMF stay
// perceptually separated in the drum grid. ch9 (drums) is handled by the
// GM table above; ch0..15 excluding 9 map to 15 distinct non-kick slots.
//   Typical GM layout: ch0=lead, ch1=bass, ch2=keys, ch3=strings, ...
static const uint8_t CH_TO_TRACK[16] = {
    /*ch0*/  1,   // SD — lead/melody
    /*ch1*/ 11,   // LT — bass (low tom = deep percussion)
    /*ch2*/  2,   // CH — keys/rhythm
    /*ch3*/  3,   // OH — strings/pads
    /*ch4*/  4,   // CP — guitar
    /*ch5*/  5,   // CB — brass
    /*ch6*/  6,   // RS
    /*ch7*/  7,   // CL
    /*ch8*/  8,   // MA
    /*ch9*/  0,   // (handled by GM table, fallback=BD)
    /*ch10*/ 9,   // CY
    /*ch11*/10,   // HT
    /*ch12*/13,   // MT
    /*ch13*/12,   // MC
    /*ch14*/14,   // HC
    /*ch15*/15,   // LC
};

static inline uint8_t channel_to_track(uint8_t ch) {
    return CH_TO_TRACK[ch & 0x0F];
}

// ---------------------------------------------------------------------------
// File-scope reader state (single-threaded, user-triggered only)
// ---------------------------------------------------------------------------
static File          s_f;
static bool          s_err = false;
static unsigned long s_parse_start_ms = 0;
static uint32_t      s_read_limit = 0;
static constexpr unsigned long MIDI_PARSE_TIMEOUT_MS = 5000;

static inline bool parse_timed_out() {
    return (millis() - s_parse_start_ms) > MIDI_PARSE_TIMEOUT_MS;
}

static uint8_t readU8() {
    uint8_t b = 0;
    if (s_err || (uint32_t)s_f.position() >= s_read_limit ||
        s_f.read(&b, 1) != 1) {
        s_err = true;
    }
    return b;
}
static uint16_t readU16() { uint8_t a = readU8(), b2 = readU8(); return (uint16_t)((a << 8) | b2); }
static uint32_t readU32() { uint16_t a = readU16(), b2 = readU16(); return ((uint32_t)a << 16) | b2; }

static uint32_t readVLQ() {
    uint32_t v = 0;
    for (int i = 0; i < 4 && !s_err; i++) {
        uint8_t b = readU8();
        v = (v << 7) | (b & 0x7F);
        if (!(b & 0x80)) return v;
    }
    s_err = true;
    return v;
}

static void skipN(uint32_t n) {
    if (n == 0 || s_err) return;
    uint32_t pos = (uint32_t)s_f.position();
    if (pos > s_read_limit || n > s_read_limit - pos || !s_f.seek(pos + n)) {
        s_err = true;
    }
}

// ---------------------------------------------------------------------------
// Event buffer in PSRAM
// ---------------------------------------------------------------------------
static constexpr int MIDI_EVENT_BUF_INITIAL = 4096;
static constexpr int MIDI_EVENT_BUF_MAX = 32768;
struct RawEvt { uint32_t tick; uint8_t trk; };
static RawEvt* s_evbuf = nullptr;
static int     s_evcount = 0;
static int     s_evcap = 0;
static bool    s_evbuf_overflow = false;

static RawEvt* alloc_event_buffer(int capacity) {
    RawEvt* p = (RawEvt*)heap_caps_malloc(sizeof(RawEvt) * capacity,
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) p = (RawEvt*)malloc(sizeof(RawEvt) * capacity);
    return p;
}

static bool ensure_evbuf_capacity(int needed = MIDI_EVENT_BUF_INITIAL) {
    if (needed <= s_evcap && s_evbuf) return true;
    int next_cap = s_evcap > 0 ? s_evcap : MIDI_EVENT_BUF_INITIAL;
    while (next_cap < needed && next_cap < MIDI_EVENT_BUF_MAX) next_cap *= 2;
    if (next_cap > MIDI_EVENT_BUF_MAX) next_cap = MIDI_EVENT_BUF_MAX;
    if (next_cap < needed) return false;

    RawEvt* next = alloc_event_buffer(next_cap);
    if (!next) return false;
    if (s_evbuf && s_evcount > 0) {
        memcpy(next, s_evbuf, sizeof(RawEvt) * s_evcount);
    }
    if (s_evbuf) heap_caps_free(s_evbuf);
    s_evbuf = next;
    s_evcap = next_cap;
    return true;
}

static bool append_event(uint32_t tick, uint8_t track) {
    if (s_evcount >= s_evcap && !ensure_evbuf_capacity(s_evcount + 1)) {
        s_evbuf_overflow = true;
        s_err = true;
        log_e("[MEM-MIDI] event limit reached (%d)", MIDI_EVENT_BUF_MAX);
        return false;
    }
    s_evbuf[s_evcount++] = {tick, track};
    return true;
}

// Parse one MTrk chunk — identical logic to the S3 parser.
static void parseTrack(uint32_t len, int tpq, uint32_t* tempo_us_out, int midi_channel) {
    (void)tpq;
    uint32_t start = (uint32_t)s_f.position();
    uint32_t file_limit = s_read_limit;
    if (len > file_limit - start) {
        s_err = true;
        return;
    }
    uint32_t end = start + len;
    s_read_limit = end;
    uint32_t tick = 0;
    uint8_t  status = 0;

    while (!s_err && (uint32_t)s_f.position() < end) {
        if (parse_timed_out()) { s_err = true; break; }
        uint32_t delta = readVLQ();
        if (UINT32_MAX - tick < delta) { s_err = true; break; }
        tick += delta;
        if (s_err) break;

        uint8_t b = readU8();

        if (b == 0xFF) {
            uint8_t meta_type = readU8();
            uint32_t ml = readVLQ();
            if (meta_type == 0x2F) {
                skipN(ml);
                break;
            } else if (meta_type == 0x51 && ml == 3 && tempo_us_out && *tempo_us_out == 0) {
                uint32_t t = (uint32_t)readU8() << 16;
                t |= (uint32_t)readU8() << 8;
                t |= (uint32_t)readU8();
                if (t > 0) *tempo_us_out = t;
            } else {
                skipN(ml);
            }
            continue;
        }
        if (b == 0xF0 || b == 0xF7) { skipN(readVLQ()); continue; }
        if (b >= 0xF8 && b <= 0xFE) continue;

        uint8_t d1;
        if (b & 0x80) { status = b; d1 = readU8(); }
        else {
            if (status == 0) { s_err = true; break; }
            d1 = b;
        }

        uint8_t ev = status & 0xF0;
        uint8_t ch = status & 0x0F;

        if (ev == 0x90 || ev == 0x80) {
            uint8_t vel = readU8();
            // Velocity gate: drop very soft notes (ghost / filler) so the
            // folded 16-step grid doesn't end up solid-on for dense tracks.
            bool is_on  = (ev == 0x90 && vel >= 16);
            uint8_t trk = 0xFF;
            if (is_on) {
                if (ch == 9) {
                    if (midi_channel == 9 || midi_channel == -1 || midi_channel == -2)
                        trk = gm_note_to_track(d1);
                } else {
                    if (midi_channel == -1 || midi_channel == -2 || midi_channel == (int)ch)
                        trk = channel_to_track(ch);
                }
            }
            if (trk != 0xFF) append_event(tick, trk);
        } else if (ev == 0xA0 || ev == 0xB0 || ev == 0xE0) {
            readU8();
        }
        // 0xC0, 0xD0: single data byte already consumed in d1
    }

    s_read_limit = file_limit;
    if (!s_err && !s_f.seek(end)) s_err = true;
}

// Fold collected events onto a 16×16 grid. Returns the raw length (16/32/48/64)
// BEFORE folding — useful for UI diagnostics ("fold 64→16").
static int foldEventsTo16(int tpq, bool steps[16][16]) {
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < 16; s++) steps[t][s] = false;

    if (s_evcount == 0 || tpq <= 0) return 16;

    uint32_t bar_ticks = (uint32_t)tpq * 4u;

    uint32_t min_tick = s_evbuf[0].tick;
    uint32_t max_tick = s_evbuf[0].tick;
    for (int i = 1; i < s_evcount; i++) {
        if (s_evbuf[i].tick < min_tick) min_tick = s_evbuf[i].tick;
        if (s_evbuf[i].tick > max_tick) max_tick = s_evbuf[i].tick;
    }

    // Snap start to the containing bar
    uint32_t bar_start = (min_tick / bar_ticks) * bar_ticks;

    // Raw length = number of bars × 16, capped at 64
    uint32_t span_ticks = max_tick - bar_start;
    int num_bars = (int)((span_ticks / bar_ticks) + 1);
    if (num_bars < 1) num_bars = 1;
    if (num_bars > 4) num_bars = 4;
    int raw_len = num_bars * 16;

    for (int i = 0; i < s_evcount; i++) {
        if (s_evbuf[i].tick < bar_start) continue;
        uint32_t rel  = s_evbuf[i].tick - bar_start;
        uint32_t step = (uint32_t)(((uint64_t)rel * 16u) / bar_ticks);   // 16th-note steps within raw
        if ((int)step < raw_len) {
            // Fold: step % 16 brings bar 2/3/4 hits onto bar 1
            int folded = (int)(step % 16);
            int trk = s_evbuf[i].trk;
            if (trk >= 0 && trk < 16) steps[trk][folded] = true;
        }
    }

    return raw_len;
}

// Same as foldEventsTo16 but WITHOUT folding — fills a 16×64 grid. Caller
// can then show any single 16-step page at a time. Returns raw_len.
static int expandEventsTo64(int tpq, bool raw[16][64]) {
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < 64; s++) raw[t][s] = false;

    if (s_evcount == 0 || tpq <= 0) return 16;

    uint32_t bar_ticks = (uint32_t)tpq * 4u;

    uint32_t min_tick = s_evbuf[0].tick;
    uint32_t max_tick = s_evbuf[0].tick;
    for (int i = 1; i < s_evcount; i++) {
        if (s_evbuf[i].tick < min_tick) min_tick = s_evbuf[i].tick;
        if (s_evbuf[i].tick > max_tick) max_tick = s_evbuf[i].tick;
    }

    uint32_t bar_start = (min_tick / bar_ticks) * bar_ticks;
    uint32_t span_ticks = max_tick - bar_start;
    int num_bars = (int)((span_ticks / bar_ticks) + 1);
    if (num_bars < 1) num_bars = 1;
    if (num_bars > 4) num_bars = 4;
    int raw_len = num_bars * 16;

    for (int i = 0; i < s_evcount; i++) {
        if (s_evbuf[i].tick < bar_start) continue;
        uint32_t rel  = s_evbuf[i].tick - bar_start;
        uint32_t step = (uint32_t)(((uint64_t)rel * 16u) / bar_ticks);
        if ((int)step < raw_len) {
            int trk = s_evbuf[i].trk;
            if (trk >= 0 && trk < 16) raw[trk][step] = true;
        }
    }
    return raw_len;
}

// ---------------------------------------------------------------------------
// parseFile — open the SMF and feed all tracks through parseTrack.
// Returns tpq (>0) on success, 0 on error. Fills tempo_us if found.
// ---------------------------------------------------------------------------
static uint16_t parseFile(fs::FS& storage, const char* path, int midi_channel, uint32_t* tempo_us_out) {
    s_f = storage.open(path, "r");
    s_err = false;
    s_parse_start_ms = millis();
    if (!s_f) return 0;
    size_t file_size = s_f.size();
    if (file_size < 14 || file_size > UINT32_MAX) {
        s_f.close();
        return 0;
    }
    s_read_limit = (uint32_t)file_size;

    uint16_t result_tpq = 0;
    char magic[4];
    bool ok = (s_f.read((uint8_t*)magic, 4) == 4) && (strncmp(magic, "MThd", 4) == 0);

    if (ok) {
        uint32_t hdr_len = readU32();
        uint16_t fmt     = readU16();
        uint16_t ntracks = readU16();
        uint16_t tpq     = readU16();
        if (!s_err && hdr_len >= 6 && fmt <= 1 && ntracks > 0 && !(tpq & 0x8000)) {
            if (tpq == 0) tpq = 96;
            if (hdr_len > 6) skipN(hdr_len - 6);

            for (int t = 0; t < ntracks && !s_err; t++) {
                if (parse_timed_out()) { s_err = true; break; }
                char tmagic[4];
                if (s_f.read((uint8_t*)tmagic, 4) != 4) break;
                if (strncmp(tmagic, "MTrk", 4) != 0) break;
                uint32_t tlen = readU32();
                if (s_err) break;
                uint32_t pos = (uint32_t)s_f.position();
                if (tlen > s_read_limit - pos) { s_err = true; break; }
                parseTrack(tlen, (int)tpq, tempo_us_out, midi_channel);
            }
            result_tpq = tpq;
        }
    }

    if (s_f) s_f.close();
    return s_err ? 0 : result_tpq;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
namespace mem_midi {

bool load_pattern(const char* path,
                  bool steps[16][16],
                  char* name_out,
                  int name_max,
                  int* steps_found_out,
                  float* bpm_out,
                  int* raw_len_out) {
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < 16; s++) steps[t][s] = false;

    const char* base = strrchr(path, '/');
    base = base ? base + 1 : path;
    if (name_out && name_max > 0) {
        strncpy(name_out, base, name_max - 1);
        name_out[name_max - 1] = '\0';
        char* dot = strrchr(name_out, '.');
        if (dot) *dot = '\0';
        if ((int)strlen(name_out) > 8) name_out[8] = '\0';
    }

    uint32_t tempo_us = 0;
    s_evcount = 0;
    s_evbuf_overflow = false;
    if (!ensure_evbuf_capacity()) {
        log_e("[MEM-MIDI] cannot allocate event buffer");
        return false;
    }

    uint16_t tpq = parseFile(SPIFFS, path, -2, &tempo_us);
    if (tpq == 0) return false;

    int ev_primary = s_evcount;

    // Fallback: nothing parsed at all → retry with -1 (equivalent here, kept for safety)
    if (s_evcount == 0) {
        s_evcount = 0;
        s_evbuf_overflow = false;
        tpq = parseFile(SPIFFS, path, -1, &tempo_us);
        if (tpq == 0) return false;
    }

    int raw_len = foldEventsTo16((int)tpq, steps);
    if (raw_len_out) *raw_len_out = raw_len;

    int total = 0;
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < 16; s++)
            if (steps[t][s]) total++;
    if (steps_found_out) *steps_found_out = total;

    if (bpm_out)
        *bpm_out = (tempo_us > 0) ? (60000000.0f / (float)tempo_us) : 0.0f;

    int tracks_used = 0;
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < 16; s++)
            if (steps[t][s]) { tracks_used++; break; }

    log_i("[MEM-MIDI] %s: tpq=%u evPrimary=%d evFinal=%d tracksUsed=%d rawLen=%d tempo_us=%u",
          path, (unsigned)tpq, ev_primary, s_evcount, tracks_used, raw_len, (unsigned)tempo_us);

    return total > 0;
}

bool load_pattern_raw(const char* path,
                      bool raw_steps[16][64],
                      char* name_out,
                      int name_max,
                      int* steps_found_out,
                      float* bpm_out,
                      int* raw_len_out,
                      int mode) {
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < 64; s++) raw_steps[t][s] = false;

    const char* base = strrchr(path, '/');
    base = base ? base + 1 : path;
    if (name_out && name_max > 0) {
        strncpy(name_out, base, name_max - 1);
        name_out[name_max - 1] = '\0';
        char* dot = strrchr(name_out, '.');
        if (dot) *dot = '\0';
        if ((int)strlen(name_out) > 8) name_out[8] = '\0';
    }

    uint32_t tempo_us = 0;
    s_evcount = 0;
    s_evbuf_overflow = false;
    if (!ensure_evbuf_capacity()) {
        log_e("[MEM-MIDI] cannot allocate event buffer");
        return false;
    }

    // Mode 0 (PRO): merge all channels with per-channel kit mapping.
    // Mode 1 (STD): only GM channel 9 (drum kit). If that yields zero
    //               events, fall back to all-channels so the user still
    //               gets something audible.
    int primary_filter = (mode == 1) ? 9 : -2;
    uint16_t tpq = parseFile(SPIFFS, path, primary_filter, &tempo_us);
    if (tpq == 0) return false;
    if (s_evcount == 0) {
        s_evcount = 0;
        s_evbuf_overflow = false;
        // Fallback: -1 (legacy any-channel, same path as load_pattern)
        tpq = parseFile(SPIFFS, path, -1, &tempo_us);
        if (tpq == 0) return false;
    }

    int raw_len = expandEventsTo64((int)tpq, raw_steps);
    if (raw_len_out) *raw_len_out = raw_len;

    int total = 0;
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < raw_len; s++)
            if (raw_steps[t][s]) total++;
    if (steps_found_out) *steps_found_out = total;

    if (bpm_out)
        *bpm_out = (tempo_us > 0) ? (60000000.0f / (float)tempo_us) : 0.0f;

    log_i("[MEM-MIDI-RAW] %s: rawLen=%d total=%d", path, raw_len, total);
    return total > 0;
}

int list_midi_files(const char* dir, char names[][48], int cap) {
    return list_midi_files_from_fs(SPIFFS, dir, names, cap);
}

bool load_pattern_raw_from_fs(fs::FS& storage,
                              const char* path,
                              bool raw_steps[16][64],
                              char* name_out,
                              int name_max,
                              int* steps_found_out,
                              float* bpm_out,
                              int* raw_len_out,
                              int mode) {
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < 64; s++) raw_steps[t][s] = false;

    const char* base = strrchr(path, '/');
    base = base ? base + 1 : path;
    if (name_out && name_max > 0) {
        strncpy(name_out, base, name_max - 1);
        name_out[name_max - 1] = '\0';
        char* dot = strrchr(name_out, '.');
        if (dot) *dot = '\0';
        if ((int)strlen(name_out) > 8) name_out[8] = '\0';
    }

    uint32_t tempo_us = 0;
    s_evcount = 0;
    s_evbuf_overflow = false;
    if (!ensure_evbuf_capacity()) {
        log_e("[MEM-MIDI] cannot allocate event buffer");
        return false;
    }

    int primary_filter = (mode == 1) ? 9 : -2;
    uint16_t tpq = parseFile(storage, path, primary_filter, &tempo_us);
    if (tpq == 0) return false;
    if (s_evcount == 0) {
        s_evcount = 0;
        s_evbuf_overflow = false;
        tpq = parseFile(storage, path, -1, &tempo_us);
        if (tpq == 0) return false;
    }

    int raw_len = expandEventsTo64((int)tpq, raw_steps);
    if (raw_len_out) *raw_len_out = raw_len;

    int total = 0;
    for (int t = 0; t < 16; t++)
        for (int s = 0; s < raw_len; s++)
            if (raw_steps[t][s]) total++;
    if (steps_found_out) *steps_found_out = total;

    if (bpm_out)
        *bpm_out = (tempo_us > 0) ? (60000000.0f / (float)tempo_us) : 0.0f;

    log_i("[MEM-MIDI-RAW-FS] %s: rawLen=%d total=%d", path, raw_len, total);
    return total > 0;
}

int list_midi_files_from_fs(fs::FS& storage, const char* dir, char names[][48], int cap) {
    int count = 0;
    File d = storage.open(dir);
    if (!d || !d.isDirectory()) {
        File root = storage.open("/");
        if (!root) return 0;
        File f = root.openNextFile();
        size_t prefix_len = strlen(dir);
        while (f && count < cap) {
            const char* full = f.name();   // e.g. "/mid/song.mid"
            const char* fname = (full[0] == '/') ? full : full;
            if (strncmp(fname, dir, prefix_len) == 0 ||
                (prefix_len > 1 && fname[0] != '/' && strncmp(fname, dir + 1, prefix_len - 1) == 0)) {
                const char* base = strrchr(fname, '/');
                base = base ? base + 1 : fname;
                int blen = (int)strlen(base);
                if (blen >= 4 && (strcasecmp(base + blen - 4, ".mid") == 0 ||
                                   strcasecmp(base + blen - 4, ".MID") == 0)) {
                    strncpy(names[count], base, 47);
                    names[count][47] = '\0';
                    count++;
                }
            }
            f.close();
            f = root.openNextFile();
        }
        if (f) f.close();
        root.close();
        for (int i = 1; i < count; i++) {
            char key[48];
            memcpy(key, names[i], sizeof(key));
            int j = i - 1;
            while (j >= 0 && strcasecmp(names[j], key) > 0) {
                memcpy(names[j + 1], names[j], sizeof(names[j + 1]));
                j--;
            }
            memcpy(names[j + 1], key, sizeof(names[j + 1]));
        }
        return count;
    }

    File f = d.openNextFile();
    while (f && count < cap) {
        if (!f.isDirectory()) {
            const char* full = f.name();
            const char* base = strrchr(full, '/');
            base = base ? base + 1 : full;
            int blen = (int)strlen(base);
            if (blen >= 4 && (strcasecmp(base + blen - 4, ".mid") == 0 ||
                               strcasecmp(base + blen - 4, ".MID") == 0)) {
                strncpy(names[count], base, 47);
                names[count][47] = '\0';
                count++;
            }
        }
        f.close();
        f = d.openNextFile();
    }
    if (f) f.close();
    d.close();
    for (int i = 1; i < count; i++) {
        char key[48];
        memcpy(key, names[i], sizeof(key));
        int j = i - 1;
        while (j >= 0 && strcasecmp(names[j], key) > 0) {
            memcpy(names[j + 1], names[j], sizeof(names[j + 1]));
            j--;
        }
        memcpy(names[j + 1], key, sizeof(names[j + 1]));
    }
    return count;
}

}  // namespace mem_midi
