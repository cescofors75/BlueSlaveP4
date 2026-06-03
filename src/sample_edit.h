// =============================================================================
// sample_edit.h — Load a WAV/MP3 sample into PSRAM, draw its envelope, and bake
// a trimmed + faded 16-bit PCM WAV ready to upload to the Master.
// =============================================================================
// The P4 is only a control surface: the Master plays the audio and its upload
// API (/api/uploadDaisy) takes a raw WAV with no trim/fade parameters. So all
// editing is done here — we decode the source to interleaved int16 PCM, let the
// UI pick trim points + fades, and re-encode a fresh WAV that already has the
// edits baked in. WAV (PCM 8/16/24-bit) is parsed natively; MP3 is decoded with
// the Helix decoder when the libhelix library is available.
// =============================================================================
#pragma once
#include <cstdint>
#include <cstddef>
#include <FS.h>

namespace sample_edit {

struct SampleInfo {
    bool     loaded;
    uint32_t sample_rate;   // Hz
    uint8_t  channels;      // 1 or 2
    uint32_t frames;        // total frames (samples per channel) in the buffer
    bool     truncated;     // true if the source was longer than the frame cap
    bool     is_mp3;
};

// Load a .wav or .mp3 from `storage` into the internal PSRAM PCM buffer.
// Returns false on failure (bad file, unsupported format, OOM, MP3 without
// decoder). Any previously loaded sample is freed first.
bool load(fs::FS& storage, const char* path);

// Info about the currently loaded sample.
const SampleInfo& info();

// Write `cols` peak-amplitude values (0..1) spanning the WHOLE loaded sample
// into `out`. Safe to call with no sample loaded (fills zeros).
void envelope(float* out, int cols);

// Bake the region [trim_start, trim_end] (fractions 0..1 of the sample) with a
// linear fade-in / fade-out (milliseconds) into a 16-bit PCM WAV written to
// `out_path` on `storage`. Returns the number of bytes written, or 0 on error.
size_t bake_wav(fs::FS& storage, const char* out_path,
                float trim_start, float trim_end,
                uint32_t fade_in_ms, uint32_t fade_out_ms);

// Free the PSRAM buffer (call when leaving the editor to reclaim memory).
void unload();

}  // namespace sample_edit
