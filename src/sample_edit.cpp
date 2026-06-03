// =============================================================================
// sample_edit.cpp — see sample_edit.h
// =============================================================================
#include "sample_edit.h"
#include <Arduino.h>
#include <cstring>
#include <strings.h>
#include <cstdlib>
#include <cmath>
#include "esp_heap_caps.h"

// MP3 support is optional: only compiled in when the Helix decoder library is
// present (add `pschatzmann/arduino-libhelix` to lib_deps). Without it, WAV
// still works and MP3 loads fail gracefully.
#if __has_include("MP3DecoderHelix.h")
  #include "MP3DecoderHelix.h"
  #define SE_HAVE_MP3 1
#else
  #define SE_HAVE_MP3 0
#endif

#if SE_HAVE_MP3
using namespace libhelix;   // brings in MP3DecoderHelix + MP3FrameInfo
#endif

namespace sample_edit {

// Hard cap on decoded length so a huge file can't exhaust PSRAM. ~1.5M frames
// is ~31s mono / ~15s stereo at 48 kHz — generous for one-shot pads.
static constexpr uint32_t MAX_FRAMES = 1500000u;

static int16_t*   s_pcm   = nullptr;   // interleaved int16, PSRAM
static uint32_t   s_cap   = 0;         // capacity in frames
static SampleInfo s_info  = {};

static void reset_info() {
    s_info.loaded = false;
    s_info.sample_rate = 0;
    s_info.channels = 0;
    s_info.frames = 0;
    s_info.truncated = false;
    s_info.is_mp3 = false;
}

void unload() {
    if (s_pcm) { heap_caps_free(s_pcm); s_pcm = nullptr; }
    s_cap = 0;
    reset_info();
}

static bool alloc_frames(uint32_t frames, uint8_t channels) {
    if (s_pcm) { heap_caps_free(s_pcm); s_pcm = nullptr; s_cap = 0; }
    if (frames == 0 || channels == 0 || channels > 2) return false;
    size_t bytes = (size_t)frames * channels * sizeof(int16_t);
    s_pcm = (int16_t*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_pcm) s_pcm = (int16_t*)malloc(bytes);   // fallback to internal RAM
    if (!s_pcm) return false;
    s_cap = frames;
    return true;
}

// --------------------------------------------------------------------------
// Little-endian readers
// --------------------------------------------------------------------------
static uint32_t rd_u32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint16_t rd_u16(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// --------------------------------------------------------------------------
// WAV (RIFF/PCM) loader — supports 8/16/24/32-bit int and 32-bit float, 1-2 ch
// --------------------------------------------------------------------------
static bool load_wav(File& f) {
    uint8_t hdr[12];
    if (f.read(hdr, 12) != 12) return false;
    if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) return false;

    uint16_t fmt = 0, channels = 0, bits = 0;
    uint32_t sample_rate = 0;
    bool have_fmt = false;
    uint32_t data_off = 0, data_len = 0;

    // Walk chunks
    uint8_t ch[8];
    while (f.read(ch, 8) == 8) {
        uint32_t id0 = rd_u32(ch);
        uint32_t csz = rd_u32(ch + 4);
        if (memcmp(ch, "fmt ", 4) == 0) {
            uint8_t fb[40];
            uint32_t n = csz < sizeof(fb) ? csz : sizeof(fb);
            if (f.read(fb, n) != (int)n) return false;
            fmt         = rd_u16(fb);
            channels    = rd_u16(fb + 2);
            sample_rate = rd_u32(fb + 4);
            bits        = rd_u16(fb + 14);
            have_fmt = true;
            if (csz > n) f.seek(f.position() + (csz - n));
        } else if (memcmp(ch, "data", 4) == 0) {
            data_off = f.position();
            data_len = csz;
            break;
        } else {
            f.seek(f.position() + csz + (csz & 1));   // chunks are word-aligned
        }
    }
    if (!have_fmt || data_off == 0 || data_len == 0) return false;
    if (channels < 1 || channels > 2) return false;
    if (sample_rate < 4000 || sample_rate > 96000) return false;

    int bytes_per_sample = bits / 8;
    if (bytes_per_sample < 1 || bytes_per_sample > 4) return false;
    bool is_float = (fmt == 3);

    uint32_t total_samples = data_len / bytes_per_sample;     // across all channels
    uint32_t frames = total_samples / channels;
    if (frames == 0) return false;
    bool truncated = false;
    if (frames > MAX_FRAMES) { frames = MAX_FRAMES; truncated = true; }

    if (!alloc_frames(frames, (uint8_t)channels)) return false;

    f.seek(data_off);
    const uint32_t total = frames * channels;
    static const int CHUNK = 2048;
    uint8_t raw[CHUNK * 4];
    uint32_t done = 0;
    while (done < total) {
        uint32_t want = total - done;
        if (want > CHUNK) want = CHUNK;
        int got = f.read(raw, want * bytes_per_sample);
        if (got <= 0) break;
        int n = got / bytes_per_sample;
        for (int i = 0; i < n; i++) {
            const uint8_t* p = raw + (size_t)i * bytes_per_sample;
            int32_t v = 0;
            switch (bytes_per_sample) {
                case 1: v = ((int32_t)p[0] - 128) << 8; break;             // 8-bit unsigned
                case 2: v = (int16_t)rd_u16(p); break;                     // 16-bit signed
                case 3: {                                                  // 24-bit signed
                    int32_t s = (p[0]) | (p[1] << 8) | (p[2] << 16);
                    if (s & 0x800000) s |= ~0xFFFFFF;
                    v = s >> 8;
                    break;
                }
                case 4:
                    if (is_float) {
                        float fv; memcpy(&fv, p, 4);
                        if (fv > 1.f) fv = 1.f; if (fv < -1.f) fv = -1.f;
                        v = (int32_t)(fv * 32767.0f);
                    } else {
                        int32_t s = (int32_t)rd_u32(p);
                        v = s >> 16;
                    }
                    break;
            }
            if (v > 32767) v = 32767; if (v < -32768) v = -32768;
            s_pcm[done + i] = (int16_t)v;
        }
        done += n;
    }

    s_info.loaded = true;
    s_info.sample_rate = sample_rate;
    s_info.channels = (uint8_t)channels;
    s_info.frames = done / channels;
    s_info.truncated = truncated;
    s_info.is_mp3 = false;
    return s_info.frames > 0;
}

// --------------------------------------------------------------------------
// MP3 loader (Helix). Decodes the whole file to the PSRAM buffer.
// --------------------------------------------------------------------------
#if SE_HAVE_MP3
static uint32_t s_mp3_written = 0;   // frames written so far

static void mp3_data_cb(MP3FrameInfo& fi, int16_t* pcm, size_t len, void* /*ref*/) {
    if (len == 0 || fi.nChans < 1 || fi.nChans > 2) return;
    if (!s_info.loaded) {
        s_info.sample_rate = (uint32_t)fi.samprate;
        s_info.channels    = (uint8_t)fi.nChans;
        s_info.loaded      = true;   // marks "header known"; frames set at end
    }
    uint8_t chs = s_info.channels;
    uint32_t frames_in = (uint32_t)(len / chs);
    if (s_mp3_written + frames_in > s_cap) {
        frames_in = (s_mp3_written < s_cap) ? (s_cap - s_mp3_written) : 0;
        s_info.truncated = true;
    }
    if (frames_in == 0) return;
    memcpy(s_pcm + (size_t)s_mp3_written * chs, pcm, (size_t)frames_in * chs * sizeof(int16_t));
    s_mp3_written += frames_in;
}

static bool load_mp3(File& f) {
    // Pre-allocate the cap buffer (stereo) since the length is unknown up front.
    if (!alloc_frames(MAX_FRAMES, 2)) return false;
    s_mp3_written = 0;
    reset_info();
    s_info.is_mp3 = true;

    MP3DecoderHelix mp3(mp3_data_cb);
    mp3.begin();
    uint8_t buf[1024];
    while (f.available() && s_mp3_written < s_cap) {
        int n = f.read(buf, sizeof(buf));
        if (n <= 0) break;
        mp3.write(buf, n);
    }
    mp3.end();

    if (!s_info.loaded || s_mp3_written == 0 || s_info.channels == 0) { unload(); return false; }
    s_info.frames = s_mp3_written;
    s_info.loaded = true;
    return true;
}
#endif  // SE_HAVE_MP3

// --------------------------------------------------------------------------
// Public API
// --------------------------------------------------------------------------
static bool ends_with_ci(const char* s, const char* suffix) {
    size_t ls = strlen(s), lf = strlen(suffix);
    if (ls < lf) return false;
    return strcasecmp(s + ls - lf, suffix) == 0;
}

bool load(fs::FS& storage, const char* path) {
    unload();
    if (!path) return false;
    File f = storage.open(path, FILE_READ);
    if (!f) return false;

    bool ok = false;
    if (ends_with_ci(path, ".mp3")) {
#if SE_HAVE_MP3
        ok = load_mp3(f);
#else
        ok = false;   // MP3 decoder not compiled in
#endif
    } else {
        ok = load_wav(f);
    }
    f.close();
    if (!ok) unload();
    return ok;
}

const SampleInfo& info() { return s_info; }

void envelope(float* out, int cols) {
    if (!out || cols <= 0) return;
    if (!s_info.loaded || !s_pcm || s_info.frames == 0) {
        for (int c = 0; c < cols; c++) out[c] = 0.f;
        return;
    }
    uint8_t ch = s_info.channels;
    uint32_t frames = s_info.frames;
    for (int c = 0; c < cols; c++) {
        uint32_t a = (uint32_t)(((uint64_t)c * frames) / cols);
        uint32_t b = (uint32_t)(((uint64_t)(c + 1) * frames) / cols);
        if (b <= a) b = a + 1;
        if (b > frames) b = frames;
        int32_t peak = 0;
        // Subsample wide columns so this stays cheap.
        uint32_t step = (b - a) / 256; if (step == 0) step = 1;
        for (uint32_t i = a; i < b; i += step) {
            const int16_t* fr = s_pcm + (size_t)i * ch;
            for (uint8_t k = 0; k < ch; k++) {
                int32_t v = fr[k]; if (v < 0) v = -v;
                if (v > peak) peak = v;
            }
        }
        out[c] = (float)peak / 32768.0f;
    }
}

static void wr_u32(uint8_t* p, uint32_t v) { p[0]=v; p[1]=v>>8; p[2]=v>>16; p[3]=v>>24; }
static void wr_u16(uint8_t* p, uint16_t v) { p[0]=v; p[1]=v>>8; }

size_t bake_wav(fs::FS& storage, const char* out_path,
                float trim_start, float trim_end,
                uint32_t fade_in_ms, uint32_t fade_out_ms) {
    if (!s_info.loaded || !s_pcm || s_info.frames == 0 || !out_path) return 0;

    if (trim_start < 0.f) trim_start = 0.f;
    if (trim_end > 1.f) trim_end = 1.f;
    if (trim_end <= trim_start) return 0;

    uint8_t  ch = s_info.channels;
    uint32_t sr = s_info.sample_rate;
    uint32_t frames = s_info.frames;
    uint32_t start = (uint32_t)(trim_start * frames);
    uint32_t end   = (uint32_t)(trim_end   * frames);
    if (end > frames) end = frames;
    if (end <= start) return 0;
    uint32_t region = end - start;

    uint32_t fin  = (uint64_t)fade_in_ms  * sr / 1000u;
    uint32_t fout = (uint64_t)fade_out_ms * sr / 1000u;
    if (fin  > region) fin  = region;
    if (fout > region) fout = region;

    uint32_t data_bytes = region * ch * 2u;

    File out = storage.open(out_path, FILE_WRITE);
    if (!out) return 0;

    uint8_t h[44];
    memcpy(h, "RIFF", 4);
    wr_u32(h + 4, 36 + data_bytes);
    memcpy(h + 8, "WAVE", 4);
    memcpy(h + 12, "fmt ", 4);
    wr_u32(h + 16, 16);                 // PCM fmt chunk size
    wr_u16(h + 20, 1);                  // PCM
    wr_u16(h + 22, ch);
    wr_u32(h + 24, sr);
    wr_u32(h + 28, sr * ch * 2u);       // byte rate
    wr_u16(h + 32, (uint16_t)(ch * 2)); // block align
    wr_u16(h + 34, 16);                 // bits
    memcpy(h + 36, "data", 4);
    wr_u32(h + 40, data_bytes);
    out.write(h, 44);

    // Stream the region with fades, buffering to keep writes chunky.
    static const int BUF_FRAMES = 512;
    int16_t buf[BUF_FRAMES * 2];
    int bi = 0;
    for (uint32_t i = start; i < end; i++) {
        uint32_t rel = i - start;
        float g = 1.0f;
        if (fin  > 0 && rel < fin)               g *= (float)rel / (float)fin;
        if (fout > 0 && (end - 1 - i) < fout)     g *= (float)(end - 1 - i) / (float)fout;
        const int16_t* fr = s_pcm + (size_t)i * ch;
        for (uint8_t k = 0; k < ch; k++) {
            int32_t v = (int32_t)lrintf(fr[k] * g);
            if (v > 32767) v = 32767; if (v < -32768) v = -32768;
            buf[bi++] = (int16_t)v;
        }
        if (bi >= BUF_FRAMES * ch) {
            out.write((uint8_t*)buf, bi * 2);
            bi = 0;
        }
    }
    if (bi > 0) out.write((uint8_t*)buf, bi * 2);
    size_t total = 44 + data_bytes;
    out.close();
    return total;
}

}  // namespace sample_edit
