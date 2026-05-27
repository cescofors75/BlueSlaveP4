// =============================================================================
// uart_protocol.h — Binary UART protocol S3 ↔ P4
// Shared between BlueSlaveV2 (S3) and BlueSlaveP4 (P4)
// =============================================================================
#pragma once

#include <stdint.h>

// =============================================================================
// TRANSPORT
// =============================================================================
#define UART_BAUD_RATE      921600   // High speed for low latency
#define UART_BASIC_LEN      5       // Fixed-length basic packet
#define UART_START_BASIC    0xAA    // Basic 5-byte packet marker
#define UART_START_EXTENDED 0xAB    // Variable-length packet marker
#define UART_EXT_HEADER_LEN 5      // Header before payload

// Maximum payload size for an extended packet. Validated on RX to drop
// malformed/oversized frames early. Must accommodate:
//   - MSG_PATTERN_DATA: 16 tracks * 64 steps = 1024 bits = 128 bytes (current = 32B for 16 steps)
//   - MSG_SD_DATA entries: index(1) + type(1) + name(32) ~= 34B
//   - MSG_SYNTH_DATA: synth param/preset relay payloads <= 7B
// 128 leaves comfortable headroom; do NOT exceed UART_RX_BUF / 2.
#define UART_EXT_MAX_PAYLOAD 128

// =============================================================================
// MESSAGE TYPES (Byte 1)
// =============================================================================
#define MSG_ENCODER     0x01    // Rotary encoder change
#define MSG_PAD         0x02    // Pad trigger / button press
#define MSG_POT         0x03    // Analog potentiometer
#define MSG_SYSTEM      0x04    // System state (BPM, pattern, play)
#define MSG_FX          0x05    // FX state / mute
#define MSG_TRACK       0x06    // Per-track info (mute, volume)
#define MSG_SCREEN      0x07    // Screen navigation
#define MSG_TOUCH_CMD   0x08    // P4→S3: touch-initiated command
#define MSG_DIAG        0x09    // RESERVED (unused)
#define MSG_PATTERN_DATA 0x0A   // Extended: pattern step data (display only, master already has it)
#define MSG_SD_DATA     0x0B    // S3→P4: SD card data (extended packets)
#define MSG_PATTERN_PUSH 0x0C   // Extended: freshly-loaded pattern (MIDI) — P4 must push it to Master via UDP
#define MSG_MELODY_DATA 0x0D    // Extended: melody grid assign data
#define MSG_SYNTH_DATA  0x0E    // Extended: synth params/presets (S3→P4→Master UDP)

// =============================================================================
// ENCODER IDs (MSG_ENCODER, Byte 2)
// =============================================================================
#define ENC_FLANGER     0x00    // FX lane 0 — Flanger mix 0-127
#define ENC_DELAY       0x01    // FX lane 1 — Delay mix 0-127
#define ENC_REVERB      0x02    // FX lane 2 — Reverb mix 0-127
#define ENC_BPM         0x03    // DFRobot encoder #1 — BPM (delta)

// =============================================================================
// POT IDs (MSG_POT, Byte 2)
// =============================================================================
#define POT_VOLUME      0x00    // P1: Master volume (MIDI 0-127)
#define POT_UNUSED      0x01    // P2: (disabled — cutoff removed)
#define POT_RESONANCE   0x02    // P3: Filter resonance (MIDI 0-127)
#define POT_DRIVE       0x03    // P4: Distortion drive (MIDI 0-127)

// =============================================================================
// SYSTEM IDs (MSG_SYSTEM, Byte 2)
// =============================================================================
#define SYS_BPM_INT     0x00    // Value = BPM integer part (40-240)
#define SYS_BPM_FRAC    0x01    // Value = BPM fractional ×10 (0-9)
#define SYS_PATTERN     0x02    // Value = current pattern index (0-15)
#define SYS_PLAY_STATE  0x03    // Value = 0=stop, 1=play
#define SYS_STEP        0x04    // Value = current step (0-15)
#define SYS_WIFI_STATE  0x05    // Value = 0=disconnected, 1=connected
#define SYS_MASTER_CONN 0x06    // Value = 0=no master, 1=master ok
#define SYS_THEME       0x07    // Value = theme index (0-5)
#define SYS_VOLUME      0x08    // Value = master volume (0-150)
#define SYS_SEQ_VOL     0x09    // Value = sequencer volume (0-150)
#define SYS_LIVE_VOL    0x0A    // Value = live pads volume (0-150)
#define SYS_HEARTBEAT   0x0F    // Keepalive (value = uptime seconds & 0xFF)

// =============================================================================
// FX IDs (MSG_FX, Byte 2)
// =============================================================================
#define FX_ENC0_MUTE    0x00    // Encoder 0 (Flanger) mute: value 0/1
#define FX_ENC1_MUTE    0x01    // Encoder 1 (Delay) mute:  value 0/1
#define FX_ENC2_MUTE    0x02    // Encoder 2 (Reverb) mute:  value 0/1
#define FX_POT0_MUTE    0x03    // Pot P2 mute: value 0/1
#define FX_POT1_MUTE    0x04    // Pot P3 (Res) mute: value 0/1
#define FX_POT2_MUTE    0x05    // Pot P4 (Drive) mute: value 0/1
#define FX_FILTER_TYPE  0x06    // Filter type (0=Off,1=LP,2=HP,3=BP,4=Notch)
#define FX_CUTOFF_H     0x07    // Cutoff Hz high byte
#define FX_CUTOFF_L     0x08    // Cutoff Hz low byte
#define FX_RESONANCE    0x09    // Resonance ×10 (10-100)
#define FX_DISTORTION   0x0A    // Distortion % (0-100)
#define FX_BITCRUSH     0x0B    // BitCrush bits (4,8,12,16)
#define FX_SAMPLERATE_H 0x0C    // SampleRate Hz high byte
#define FX_SAMPLERATE_L 0x0D    // SampleRate Hz low byte
#define FX_RESP_MODE    0x0E    // FX response mode (0=precision,1=live)

// =============================================================================
// TRACK IDs (MSG_TRACK, Byte 2)
// Sub-command in high nibble, track in low nibble
// =============================================================================
#define TRK_MUTE_BIT    0x00    // 0x0T: track T mute — value 0/1
#define TRK_SOLO_BIT    0x10    // 0x1T: track T solo — value 0/1
#define TRK_VOLUME      0x20    // 0x2T: track T volume — value 0-100

// =============================================================================
// SCREEN IDs (MSG_SCREEN, Byte 2)
// =============================================================================
#define SCR_NAVIGATE     0x00   // Value = target Screen enum value

// =============================================================================
// PAD IDs (MSG_PAD, Byte 2 = pad 0-15, Byte 3 = velocity 0-127)
// =============================================================================
// Byte 2 = pad index (0-15), Byte 3 = value (velocity or 0=release)

// =============================================================================
// TOUCH COMMAND IDs — P4→S3 (MSG_TOUCH_CMD, Byte 2)
// =============================================================================
#define TCMD_PAD_TAP       0x00 // Byte 3 = pad index (velocity fixed 127)
#define TCMD_PLAY_TOGGLE   0x01 // Byte 3 = unused
#define TCMD_PATTERN_SEL   0x02 // Byte 3 = pattern index
#define TCMD_FX_TOGGLE     0x03 // RESERVED (unused)
#define TCMD_THEME_NEXT    0x04 // Byte 3 = unused
#define TCMD_SCREEN_NAV    0x05 // Byte 3 = screen enum
#define TCMD_STEP_TOGGLE   0x06 // S3→P4: toggle step. id=TCMD_STEP_TOGGLE, value=(track<<4)|step
#define TCMD_SD_MOUNT      0x07 // P4→S3: mount SD + list root dir
#define TCMD_SD_SELECT     0x08 // P4→S3: select entry N (0-based index)
#define TCMD_SD_BACK       0x09 // P4→S3: go to parent directory
#define TCMD_SD_LOAD       0x0A // P4→S3: load selected file to pad N
#define TCMD_SYNC_PADS     0x0B  // Bidirectional: toggle pad LED sync mode
#define TCMD_SD_LOAD_MIDI  0x0C  // P4→S3: load selected MIDI file into pattern slot N (value=slot 6-15)
#define TCMD_RESERVED_0D   0x0D
#define TCMD_RESERVED_0E   0x0E
#define TCMD_RESERVED_0F   0x0F
#define TCMD_DRIVE_UP      0x10  // P4→S3: increase master drive
#define TCMD_DRIVE_DOWN    0x11  // P4→S3: decrease master drive
// v2.9 — Melody sync over UART (bidirectional S3↔P4)
#define TCMD_MELODY_ENGINE  0x12  // value = engine code (3-6)
#define TCMD_MELODY_OCTAVE  0x13  // value = octave (1-7)
#define TCMD_MELODY_REC     0x14  // value = 0=off, 1=on
#define TCMD_MELODY_CLEAR   0x15  // value = 0 (grid clear, S3→P4 only)
#define TCMD_MELODY_PAD     0x16  // value = pad index (0-15)
#define TCMD_MELODY_NOTE    0x17  // value = MIDI note (0-127, P4→S3 rec)
#define TCMD_MELODY_NOTE_OFF 0x18 // value = MIDI note (0-127, live piano release)
#define TCMD_PIANO_NOTE_ON  0x19  // value = MIDI note (0-127, live piano press)

// =============================================================================
// MELODY DATA sub-IDs (MSG_MELODY_DATA extended, Byte 2)
// =============================================================================
#define MEL_DATA_ASSIGN     0x00  // payload: pad,engine,octave + 32-byte 16x12 grid bits

// =============================================================================
// SYNTH DATA sub-IDs (MSG_SYNTH_DATA extended, Byte 2)
// =============================================================================
#define SYNTH_DATA_PARAM    0x00  // payload: engine,instrument,paramId,float32 value
#define SYNTH_DATA_PRESET   0x01  // payload: engine,preset

// =============================================================================
// SD DATA sub-IDs (MSG_SD_DATA extended, Byte 2)
// =============================================================================
#define SD_RESP_STATUS     0x00 // payload: [mounted(1)]
#define SD_RESP_ENTRY      0x01 // payload: [index(1), type(1='D'/'F'), name...]
#define SD_RESP_LIST_END   0x02 // payload: [total_count(1)]
#define SD_RESP_PATH       0x03 // payload: [path string...]
#define SD_RESP_SELECTED   0x04 // payload: [filename string...]
#define SD_RESP_LOAD_OK    0x05 // payload: [pad(1)]

// =============================================================================
// PACKET STRUCTURES
// =============================================================================

// Basic 5-byte packet (vast majority of messages)
typedef struct __attribute__((packed)) {
    uint8_t start;      // UART_START_BASIC (0xAA)
    uint8_t type;       // MSG_*
    uint8_t id;         // sub-ID (encoder/pot/system/fx/track/pad index)
    uint8_t value;      // 0-255
    uint8_t checksum;   // (start + type + id + value) & 0xFF
} UartBasicPacket;

// Extended variable-length packet header (for pattern data, bulk state)
typedef struct __attribute__((packed)) {
    uint8_t start;      // UART_START_EXTENDED (0xAB)
    uint8_t type;       // MSG_*
    uint8_t id;         // sub-ID
    uint8_t len_h;      // payload length high byte
    uint8_t len_l;      // payload length low byte
    // Followed by: [payload bytes ...] [checksum: sum of all bytes & 0xFF]
} UartExtendedHeader;

// =============================================================================
// HELPER FUNCTIONS (inline)
// =============================================================================

static inline uint8_t uart_checksum_basic(uint8_t start, uint8_t type, uint8_t id, uint8_t value) {
    return (uint8_t)((start + type + id + value) & 0xFF);
}

static inline void uart_build_basic(UartBasicPacket* pkt, uint8_t type, uint8_t id, uint8_t value) {
    pkt->start = UART_START_BASIC;
    pkt->type = type;
    pkt->id = id;
    pkt->value = value;
    pkt->checksum = uart_checksum_basic(UART_START_BASIC, type, id, value);
}

static inline bool uart_validate_basic(const UartBasicPacket* pkt) {
    return (pkt->start == UART_START_BASIC) &&
           (pkt->checksum == uart_checksum_basic(pkt->start, pkt->type, pkt->id, pkt->value));
}

// Strict validator: checksum + type + per-type id/value range checks.
// Returns false for any out-of-spec field so corrupt-but-checksum-valid
// packets (rare but possible with single-bit flips on multiple fields)
// are dropped before they reach the state machine.
static inline bool uart_validate_packet(const UartBasicPacket* pkt) {
    if (!uart_validate_basic(pkt)) return false;
    switch (pkt->type) {
        case MSG_ENCODER:
            return pkt->id <= ENC_BPM;                  // 0..3
        case MSG_PAD:
            return pkt->id < 16;                        // 0..15, value=velocity 0..127 also OK >=128 release
        case MSG_POT:
            return pkt->id <= POT_DRIVE && pkt->value <= 127;
        case MSG_SYSTEM:
            switch (pkt->id) {
                case SYS_PLAY_STATE:    return pkt->value <= 1;
                case SYS_WIFI_STATE:    return pkt->value <= 1;
                case SYS_MASTER_CONN:   return pkt->value <= 1;
                case SYS_THEME:         return pkt->value <= 5;
                case SYS_PATTERN:       return pkt->value < 16;
                case SYS_STEP:          return pkt->value < 64;
                case SYS_BPM_INT:       return pkt->value >= 40 && pkt->value <= 240;
                case SYS_BPM_FRAC:      return pkt->value <= 9;
                case SYS_VOLUME:
                case SYS_SEQ_VOL:
                case SYS_LIVE_VOL:      return pkt->value <= 150;
                case SYS_HEARTBEAT:     return true;
                default: return false;
            }
        case MSG_FX:
            return pkt->id <= FX_RESP_MODE;
        case MSG_TRACK: {
            uint8_t sub = pkt->id & 0xF0;
            uint8_t trk = pkt->id & 0x0F;
            if (trk >= 16) return false;
            if (sub == TRK_MUTE_BIT || sub == TRK_SOLO_BIT) return pkt->value <= 1;
            if (sub == TRK_VOLUME)                          return pkt->value <= 150;
            return false;
        }
        case MSG_SCREEN:
            return pkt->id == SCR_NAVIGATE;
        case MSG_TOUCH_CMD:
            return pkt->id <= TCMD_PIANO_NOTE_ON;
        case MSG_PATTERN_DATA:
        case MSG_SD_DATA:
        case MSG_SYNTH_DATA:
            return false;                                // not a basic packet
        default:
            return false;                                // unknown type
    }
}
