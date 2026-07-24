// =============================================================================
// udp_handler.cpp — WiFi/UDP communication to Master via ESP32-C6
// P4 connects directly to Master AP (RED808) through SDIO ESP-Hosted
// =============================================================================

#include "udp_handler.h"
#include "uart_handler.h"   // for P4State p4
#include "ui/ui_screens.h"
#include "../include/config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include <atomic>
#include <freertos/queue.h>

// =============================================================================
// CONFIGURATION
// =============================================================================
static const char*     WIFI_SSID      = P4_WIFI_SSID;
static const char*     WIFI_PASS      = P4_WIFI_PASS;
static const IPAddress MASTER_IP(192, 168, 4, 1);
static const uint16_t  UDP_PORT       = 8888;

// Timing
// 15 s retry: a full WiFi.begin() restart aborts an association already in
// progress; on a congested AP auth+DHCP can exceed 5 s, so a short retry
// could keep killing attempts that were about to succeed (autoReconnect
// already handles the common cases).
static const unsigned long WIFI_RETRY_MS    = 15000;
static const unsigned long MASTER_TIMEOUT_MS = 3000;
static const unsigned long SYNC_REQUEST_MS  = 500;

// =============================================================================
// STATE
// =============================================================================
static WiFiUDP udp;
// atomic: read by the HTTP worker task, written by loop()
static std::atomic<bool> wifiConnected{false};
static std::atomic<bool> udpStarted{false};
static bool masterAlive        = false;
static unsigned long lastWifiAttempt  = 0;
static unsigned long lastMasterPacket = 0;
static unsigned long lastSyncRequest  = 0;
static bool sessionCleanSent = false;
// Per-track timestamps of last LOCAL mute/solo change. Used to suppress the
// state_sync broadcast from the master clobbering user input within a short
// window (the broadcast can race with our own UDP commands).
static unsigned long lastLocalMuteMs[16] = {};
static unsigned long lastLocalSoloMs[16] = {};
static unsigned long lastLocalTrackVolMs[16] = {};
// Timestamp of the last LOCAL pattern selection. The master's periodic
// state_sync (and the HTTP state poll) can still report the previous pattern
// for a cycle or two after we switch, which would otherwise yank the user back
// to the old pattern. Treat a recent local selection as authoritative.
static unsigned long lastLocalPatternMs = 0;
// Master broadcasts state_sync about every 2 s. Keep a slightly larger local
// ownership window so near-race packets do not pull FX/track values back right
// after a local touch interaction.
static const unsigned long LOCAL_OWNERSHIP_MS = 4000;
static const unsigned long LOCAL_STEP_OWNERSHIP_MS = 700;
static int pendingPatternRequest = -1;
static unsigned long pendingPatternLastTxMs = 0;
static uint8_t pendingPatternRetries = 0;
// HTTP is only a fallback when the UDP get_pattern reply is lost. Rate-limit it
// and keep the request timeout short so a stalled master can't freeze the loop
// (and starve pad/control sends) for long.
static unsigned long lastPatternHttpMs = 0;
static const unsigned long PATTERN_HTTP_FALLBACK_MS = 500;
static const int          MASTER_HTTP_TIMEOUT_MS    = 350;
static bool patternRowGrid[16][64] = {};
static uint16_t patternRowMask = 0;
static int patternRowPattern = -1;
static int patternRowStepCount = 16;
static unsigned long lastLocalStepMs[16][64] = {};
static unsigned long lastMasterStepSyncMs = 0;
static unsigned long lastRemoteMasterFxLogMs = 0;
static unsigned long lastRemoteStateSyncFxLogMs = 0;
static unsigned long lastRemoteHttpFxLogMs = 0;
static const unsigned long FX_SYNC_LOG_MIN_MS = 250;
// Dirty flag: set by UDP/UI producers, atomically consumed by the LVGL task.
static std::atomic<bool> s_fx_screen_dirty{false};

void udp_mark_fx_screen_dirty(void) {
    s_fx_screen_dirty.store(true, std::memory_order_release);
}

bool udp_consume_fx_screen_dirty(void) {
    return s_fx_screen_dirty.exchange(false, std::memory_order_acq_rel);
}

// JSON parse buffer
static char rxBuf[8192];

static int clamp_int(int value, int lo, int hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

enum FxOwnershipIndex : int {
    FX_OWN_FILTER_TYPE = 0,
    FX_OWN_CUTOFF,
    FX_OWN_RESONANCE,
    FX_OWN_DISTORTION,
    FX_OWN_BITCRUSH,
    FX_OWN_SAMPLE_RATE,
    FX_OWN_DELAY,
    FX_OWN_REVERB,
    FX_OWN_PHASER,
    FX_OWN_COUNT,
};
static unsigned long lastLocalFxMs[FX_OWN_COUNT] = {};
static constexpr int RAYDRONE_FIELD_COUNT = 8;
static constexpr uint32_t RAYDRONE_RETRY_MS = 250;

struct RaydronePendingRequest {
    bool active;
    uint8_t updateMask;
    uint32_t requestId;
    uint32_t lastTxMs;
    RaydroneConfigPayload state;
};

struct RaydroneTxWork {
    uint8_t updateMask;
    uint32_t requestId;
    RaydroneConfigPayload state;
};

static RaydronePendingRequest s_raydrone_pending[RAYDRONE_FIELD_COUNT] = {};
static uint32_t s_raydrone_request_counter = 0;
static portMUX_TYPE s_raydrone_ownership_mux = portMUX_INITIALIZER_UNLOCKED;
// Serializes the short "snapshot pending -> enqueue" section with local
// invalidation. Without it, a retry copied just before a new gesture could be
// enqueued after the newer request and make the master apply stale state last.
// Pending/request ownership itself is always guarded by the portMUX above.
static SemaphoreHandle_t s_raydrone_tx_mutex = NULL;

static inline bool is_track_volume_owned_recent(int track, unsigned long nowMs) {
    if (track < 0 || track >= 16) return false;
    unsigned long stamp = lastLocalTrackVolMs[track];
    return stamp != 0 && (nowMs - stamp) < LOCAL_OWNERSHIP_MS;
}

static inline bool is_pattern_owned_recent(unsigned long nowMs) {
    return lastLocalPatternMs != 0 && (nowMs - lastLocalPatternMs) < LOCAL_OWNERSHIP_MS;
}

// Arbitraje del patrón que reporta el master (state_sync UDP y poll HTTP).
// La ventana de ownership local tapa el lag del master justo tras un cambio
// nuestro, pero no puede ganar para siempre — con 2 slaves conectados dejaba
// el número de patrón "haciendo el tonto". Devuelve true si hay que aplicar:
//  - sin ownership activo → aplicar siempre;
//  - el master confirma nuestro patrón → soltar la ventana ya, así el próximo
//    cambio del OTRO slave entra al momento en vez de ignorarse 4 s;
//  - el master insiste 2 veces seguidas en otro patrón → nuestro selectPattern
//    se perdió por el camino (UDP) o el otro slave cambió: manda el master.
static bool master_pattern_report_should_apply(int pat) {
    static uint8_t disagree = 0;
    if (!is_pattern_owned_recent(millis())) { disagree = 0; return true; }
    if (pat == p4.current_pattern)  { lastLocalPatternMs = 0; disagree = 0; return true; }
    if (++disagree >= 2)            { lastLocalPatternMs = 0; disagree = 0; return true; }
    return false;
}

static inline bool is_fx_owned_recent(FxOwnershipIndex idx, unsigned long nowMs) {
    unsigned long stamp = lastLocalFxMs[(int)idx];
    return stamp != 0 && (nowMs - stamp) < LOCAL_OWNERSHIP_MS;
}

static inline void mark_local_track_volume(int track) {
    if (track < 0 || track >= 16) return;
    lastLocalTrackVolMs[track] = millis();
}

static inline void mark_local_fx(FxOwnershipIndex idx) {
    lastLocalFxMs[(int)idx] = millis();
}

static int raydrone_field_index(uint8_t updateMask) {
    for (int i = 0; i < RAYDRONE_FIELD_COUNT; ++i) {
        if (updateMask == (uint8_t)(1u << i)) return i;
    }
    return -1;
}

// Caller holds s_raydrone_ownership_mux. RayDrone ownership is request based,
// not time based: only the matching ACK may release the field.
static bool is_raydrone_locally_owned(uint8_t updateMask) {
    const int index = raydrone_field_index(updateMask);
    return index >= 0 && s_raydrone_pending[index].active;
}

// Caller holds s_raydrone_ownership_mux.
static bool begin_raydrone_request_locked(
    uint8_t updateMask, const RaydroneConfigPayload& state, uint32_t nowMs,
    RaydroneTxWork& work) {
    const int index = raydrone_field_index(updateMask);
    if (index < 0) return false;

    // Zero is reserved as "no request". Wrap safely after UINT32_MAX.
    do {
        ++s_raydrone_request_counter;
    } while (s_raydrone_request_counter == 0);

    RaydronePendingRequest& pending = s_raydrone_pending[index];
    pending.active = true;
    pending.updateMask = updateMask;
    pending.requestId = s_raydrone_request_counter;
    pending.lastTxMs = nowMs;
    pending.state = state;

    work.updateMask = updateMask;
    work.requestId = pending.requestId;
    work.state = state;
    return true;
}

static inline void forward_fx_filter_type_to_s3(int type) {
    uart_send_to_s3(MSG_FX, FX_FILTER_TYPE, (uint8_t)clamp_int(type, 0, 4));
}

static inline void forward_fx_cutoff_to_s3(int cutoffHz) {
    int v = clamp_int(cutoffHz, 20, 20000);
    uart_send_to_s3(MSG_FX, FX_CUTOFF_H, (uint8_t)((v >> 8) & 0xFF));
    uart_send_to_s3(MSG_FX, FX_CUTOFF_L, (uint8_t)(v & 0xFF));
}

static inline void forward_fx_resonance_to_s3(int resonanceX10) {
    uart_send_to_s3(MSG_FX, FX_RESONANCE, (uint8_t)clamp_int(resonanceX10, 10, 100));
}

static inline void forward_fx_distortion_to_s3(int distortionPct) {
    uart_send_to_s3(MSG_FX, FX_DISTORTION, (uint8_t)clamp_int(distortionPct, 0, 100));
}

static inline void forward_fx_bitcrush_to_s3(int bits) {
    uart_send_to_s3(MSG_FX, FX_BITCRUSH, (uint8_t)clamp_int(bits, 4, 16));
}

static inline void forward_fx_samplerate_to_s3(int sampleRateHz) {
    int v = (sampleRateHz <= 0) ? 0 : clamp_int(sampleRateHz, 9000, 32000);
    uart_send_to_s3(MSG_FX, FX_SAMPLERATE_H, (uint8_t)((v >> 8) & 0xFF));
    uart_send_to_s3(MSG_FX, FX_SAMPLERATE_L, (uint8_t)(v & 0xFF));
}

static float clamp_float(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static int scale_unit_to_u7(float value) {
    float unit = clamp_float(value, 0.0f, 1.0f);
    return clamp_int((int)(unit * 127.0f + 0.5f), 0, 127);
}

static uint8_t delay_mix_to_u7(JsonVariantConst value, uint8_t fallback) {
    if (value.isNull()) return fallback;
    float mixPct = value.as<float>();
    if (mixPct <= 1.0f) mixPct *= 100.0f;
    float norm = (mixPct - 8.0f) / 50.0f;
    return (uint8_t)scale_unit_to_u7(norm);
}

static uint8_t reverb_mix_to_u7(JsonVariantConst value, uint8_t fallback) {
    if (value.isNull()) return fallback;
    float mix = value.as<float>();
    if (mix > 1.0f) mix /= 100.0f;
    float norm = (mix - 0.06f) / 0.42f;
    return (uint8_t)scale_unit_to_u7(norm);
}

static uint8_t phaser_depth_to_u7(JsonVariantConst value, uint8_t fallback) {
    if (value.isNull()) return fallback;
    float depthPct = value.as<float>();
    if (depthPct <= 1.0f) depthPct *= 100.0f;
    float norm = (depthPct - 28.0f) / 60.0f;
    return (uint8_t)scale_unit_to_u7(norm);
}

static float fx_scalar_to_unit(JsonVariantConst value, float fallbackUnit) {
    if (value.isNull()) return clamp_float(fallbackUnit, 0.0f, 1.0f);
    float raw = value.as<float>();
    if (raw > 1.0f) raw /= 100.0f;
    return clamp_float(raw, 0.0f, 1.0f);
}

static bool should_log_fx_sync(unsigned long& lastLogMs) {
#if P4_ENABLE_FX_SYNC_LOG
    unsigned long nowMs = millis();
    if ((nowMs - lastLogMs) < FX_SYNC_LOG_MIN_MS) return false;
    lastLogMs = nowMs;
    return true;
#else
    (void)lastLogMs;
    return false;
#endif
}

static void log_remote_macro_fx_state(const char* source, JsonObjectConst fx, unsigned long& lastLogMs) {
#if P4_ENABLE_FX_SYNC_LOG
    if (!should_log_fx_sync(lastLogMs)) return;
    P4_FX_LOG_PRINTF("[FX][RX][%s] fields dA=%d dM=%d rA=%d rM=%d pA=%d pD=%d -> D act=%d mix=%u R act=%d mix=%u P act=%d depth=%u\n",
                      source,
                      fx["delayActive"].isNull() ? 0 : 1,
                      fx["delayMix"].isNull() ? 0 : 1,
                      fx["reverbActive"].isNull() ? 0 : 1,
                      fx["reverbMix"].isNull() ? 0 : 1,
                      fx["phaserActive"].isNull() ? 0 : 1,
                      fx["phaserDepth"].isNull() ? 0 : 1,
                      p4.enc_muted[1] ? 0 : 1,
                      (unsigned)p4.enc_value[1],
                      p4.enc_muted[2] ? 0 : 1,
                      (unsigned)p4.enc_value[2],
                      p4.pot_muted[2] ? 0 : 1,
                      (unsigned)p4.pot_value[2]);
#else
    (void)source;
    (void)fx;
    (void)lastLogMs;
#endif
}

static void log_remote_master_fx_param(const char* param, JsonVariantConst value, bool handled) {
#if P4_ENABLE_FX_SYNC_LOG
    if (!should_log_fx_sync(lastRemoteMasterFxLogMs)) return;
    float raw = value.isNull() ? 0.0f : value.as<float>();
    P4_FX_LOG_PRINTF("[FX][RX][masterFx] param=%s raw=%.3f handled=%d -> D act=%d mix=%u R act=%d mix=%u P act=%d depth=%u\n",
                      param ? param : "",
                      raw,
                      handled ? 1 : 0,
                      p4.enc_muted[1] ? 0 : 1,
                      (unsigned)p4.enc_value[1],
                      p4.enc_muted[2] ? 0 : 1,
                      (unsigned)p4.enc_value[2],
                      p4.pot_muted[2] ? 0 : 1,
                      (unsigned)p4.pot_value[2]);
#else
    (void)param;
    (void)value;
    (void)handled;
#endif
}

static void apply_remote_macro_fx_state(JsonObjectConst fx, const char* source, unsigned long& lastLogMs) {
    unsigned long nowMs = millis();

    bool delayActive = fx["delayActive"].isNull() ? !p4.enc_muted[1] : (fx["delayActive"].as<bool>());
    bool reverbActive = fx["reverbActive"].isNull() ? !p4.enc_muted[2] : (fx["reverbActive"].as<bool>());
    bool phaserActive = fx["phaserActive"].isNull() ? !p4.pot_muted[2] : (fx["phaserActive"].as<bool>());

    if (!is_fx_owned_recent(FX_OWN_DELAY, nowMs)) {
        p4.enc_muted[1] = !delayActive;
    }
    if (!is_fx_owned_recent(FX_OWN_REVERB, nowMs)) {
        p4.enc_muted[2] = !reverbActive;
    }
    if (!is_fx_owned_recent(FX_OWN_PHASER, nowMs)) {
        p4.pot_muted[2] = !phaserActive;
    }

    if (!is_fx_owned_recent(FX_OWN_DELAY, nowMs) && !fx["delayMix"].isNull()) {
        p4.enc_value[1] = delay_mix_to_u7(fx["delayMix"], p4.enc_value[1]);
    }
    if (!is_fx_owned_recent(FX_OWN_REVERB, nowMs) && !fx["reverbMix"].isNull()) {
        p4.enc_value[2] = reverb_mix_to_u7(fx["reverbMix"], p4.enc_value[2]);
    }
    if (!is_fx_owned_recent(FX_OWN_PHASER, nowMs) && !fx["phaserDepth"].isNull()) {
        p4.pot_value[2] = phaser_depth_to_u7(fx["phaserDepth"], p4.pot_value[2]);
    }

    udp_mark_fx_screen_dirty();
    log_remote_macro_fx_state(source, fx, lastLogMs);
}

static void apply_remote_raydrone_state(JsonObjectConst raydrone) {
    if (raydrone.isNull()) return;

    int version = raydrone["version"] | RAYDRONE_CONFIG_VERSION;
    if (version != RAYDRONE_CONFIG_VERSION) {
        P4_LOG_PRINTF("[RAYDRONE] unsupported state version %d\n", version);
        return;
    }

    // A requestId identifies an ACK/reject for one locally-owned field. It is
    // deliberately handled separately from unsolicited snapshots: a stale ACK
    // must neither release ownership nor modify local state.
    if (!raydrone["requestId"].isNull()) {
        if (raydrone["updateMask"].isNull() || raydrone["ok"].isNull()) {
            P4_LOG_PRINTLN("[RAYDRONE] malformed ACK");
            return;
        }

        const uint8_t ackMask = raydrone["updateMask"].as<uint8_t>();
        const int ackIndex = raydrone_field_index(ackMask);
        const uint32_t requestId = raydrone["requestId"].as<uint32_t>();
        const bool ok = raydrone["ok"].as<bool>();
        if (ackIndex < 0 || requestId == 0) {
            P4_LOG_PRINTLN("[RAYDRONE] invalid ACK identity");
            return;
        }

        RaydroneConfigPayload authoritative = {};
        authoritative.version = RAYDRONE_CONFIG_VERSION;
        bool hasAuthoritativeField = false;
        switch (ackMask) {
            case RAYDRONE_UPDATE_ACTIVE:
                if (!raydrone["active"].isNull()) {
                    authoritative.flags = raydrone["active"].as<bool>()
                                              ? RAYDRONE_FLAG_ACTIVE
                                              : 0;
                    hasAuthoritativeField = true;
                } else if (!raydrone["flags"].isNull()) {
                    authoritative.flags =
                        raydrone["flags"].as<uint8_t>() & RAYDRONE_FLAG_ACTIVE;
                    hasAuthoritativeField = true;
                }
                break;
            case RAYDRONE_UPDATE_MATERIAL:
                if (!raydrone["material"].isNull()) {
                    authoritative.material = (uint8_t)clamp_int(
                        raydrone["material"].as<int>(), 0,
                        RAYDRONE_MATERIAL_COUNT - 1);
                    hasAuthoritativeField = true;
                }
                break;
            case RAYDRONE_UPDATE_CHARACTER:
                if (!raydrone["character"].isNull()) {
                    authoritative.character = (uint8_t)clamp_int(
                        raydrone["character"].as<int>(), 0, RAYDRONE_CHARACTER_SAFE_MAX);
                    hasAuthoritativeField = true;
                }
                break;
            case RAYDRONE_UPDATE_MOTION:
                if (!raydrone["motion"].isNull()) {
                    authoritative.motion = (uint8_t)clamp_int(
                        raydrone["motion"].as<int>(), 0, 100);
                    hasAuthoritativeField = true;
                }
                break;
            case RAYDRONE_UPDATE_SPACE:
                if (!raydrone["space"].isNull()) {
                    authoritative.space = (uint8_t)clamp_int(
                        raydrone["space"].as<int>(), 0, 100);
                    hasAuthoritativeField = true;
                }
                break;
            case RAYDRONE_UPDATE_VOLUME:
                if (!raydrone["volume"].isNull()) {
                    authoritative.volume = (uint8_t)clamp_int(
                        raydrone["volume"].as<int>(), 0, 100);
                    hasAuthoritativeField = true;
                }
                break;
            case RAYDRONE_UPDATE_MIX:
                if (!raydrone["mix"].isNull()) {
                    authoritative.mix = (uint8_t)clamp_int(
                        raydrone["mix"].as<int>(), 0, 100);
                    hasAuthoritativeField = true;
                }
                break;
            case RAYDRONE_UPDATE_EVOLUTION:
                if (!raydrone["evolution"].isNull()) {
                    authoritative.evolution = (uint8_t)clamp_int(
                        raydrone["evolution"].as<int>(), 0, 100);
                    hasAuthoritativeField = true;
                }
                break;
            default:
                break;
        }
        if (!hasAuthoritativeField) {
            P4_LOG_PRINTLN("[RAYDRONE] ACK missing authoritative field");
            return;
        }

        bool matched = false;
        portENTER_CRITICAL(&s_raydrone_ownership_mux);
        RaydronePendingRequest& pending = s_raydrone_pending[ackIndex];
        if (pending.active && pending.updateMask == ackMask &&
            pending.requestId == requestId) {
            matched = true;
            pending = {};
            // Both accepted and rejected commands use the master's returned
            // value. A reject therefore rolls the optimistic UI value back.
            p4_raydrone_update(authoritative, ackMask);
        }
        portEXIT_CRITICAL(&s_raydrone_ownership_mux);

        if (!matched) {
            P4_LOG_PRINTF("[RAYDRONE] stale ACK requestId=%lu ignored\n",
                          (unsigned long)requestId);
            return;
        }
        if (!ok) {
            P4_LOG_PRINTF("[RAYDRONE] requestId=%lu rejected; state restored\n",
                          (unsigned long)requestId);
        }
        udp_mark_fx_screen_dirty();
        return;
    }

    portENTER_CRITICAL(&s_raydrone_ownership_mux);
    const RaydroneConfigPayload current = p4_raydrone_snapshot();
    RaydroneConfigPayload patch = current;
    uint8_t updateMask = 0;

    auto accept_field = [&](uint8_t field) {
        return !is_raydrone_locally_owned(field);
    };

    if (!raydrone["flags"].isNull() || !raydrone["active"].isNull()) {
        bool active = !raydrone["active"].isNull()
                          ? raydrone["active"].as<bool>()
                          : ((raydrone["flags"].as<uint8_t>()
                              & RAYDRONE_FLAG_ACTIVE) != 0);
        if (accept_field(RAYDRONE_UPDATE_ACTIVE)) {
            patch.flags = active ? RAYDRONE_FLAG_ACTIVE : 0;
            updateMask |= RAYDRONE_UPDATE_ACTIVE;
        }
    }
    if (!raydrone["material"].isNull()) {
        const int incoming = clamp_int(
            raydrone["material"].as<int>(), 0, RAYDRONE_MATERIAL_COUNT - 1);
        if (accept_field(RAYDRONE_UPDATE_MATERIAL)) {
            patch.material = (uint8_t)incoming;
            updateMask |= RAYDRONE_UPDATE_MATERIAL;
        }
    }
    if (!raydrone["character"].isNull()) {
        const int incoming =
            clamp_int(raydrone["character"].as<int>(), 0, RAYDRONE_CHARACTER_SAFE_MAX);
        if (accept_field(RAYDRONE_UPDATE_CHARACTER)) {
            patch.character = (uint8_t)incoming;
            updateMask |= RAYDRONE_UPDATE_CHARACTER;
        }
    }
    if (!raydrone["motion"].isNull()) {
        const int incoming =
            clamp_int(raydrone["motion"].as<int>(), 0, 100);
        if (accept_field(RAYDRONE_UPDATE_MOTION)) {
            patch.motion = (uint8_t)incoming;
            updateMask |= RAYDRONE_UPDATE_MOTION;
        }
    }
    if (!raydrone["space"].isNull()) {
        const int incoming =
            clamp_int(raydrone["space"].as<int>(), 0, 100);
        if (accept_field(RAYDRONE_UPDATE_SPACE)) {
            patch.space = (uint8_t)incoming;
            updateMask |= RAYDRONE_UPDATE_SPACE;
        }
    }
    if (!raydrone["volume"].isNull()) {
        const int incoming =
            clamp_int(raydrone["volume"].as<int>(), 0, 100);
        if (accept_field(RAYDRONE_UPDATE_VOLUME)) {
            patch.volume = (uint8_t)incoming;
            updateMask |= RAYDRONE_UPDATE_VOLUME;
        }
    }
    if (!raydrone["mix"].isNull()) {
        const int incoming =
            clamp_int(raydrone["mix"].as<int>(), 0, 100);
        if (accept_field(RAYDRONE_UPDATE_MIX)) {
            patch.mix = (uint8_t)incoming;
            updateMask |= RAYDRONE_UPDATE_MIX;
        }
    }
    if (!raydrone["evolution"].isNull()) {
        const int incoming = clamp_int(
            raydrone["evolution"].as<int>(), 0, 100);
        if (accept_field(RAYDRONE_UPDATE_EVOLUTION)) {
            patch.evolution = (uint8_t)incoming;
            updateMask |= RAYDRONE_UPDATE_EVOLUTION;
        }
    }

    if (updateMask != 0) {
        p4_raydrone_update(patch, updateMask);
    }
    portEXIT_CRITICAL(&s_raydrone_ownership_mux);
    if (updateMask != 0) udp_mark_fx_screen_dirty();
}

static void apply_remote_master_fx_param(const char* param, JsonVariantConst value) {
    if (!param || value.isNull()) {
        log_remote_master_fx_param(param, value, false);
        return;
    }

    bool handled = true;
    unsigned long nowMs = millis();
    if (strcmp(param, "delayActive") == 0) {
        if (!is_fx_owned_recent(FX_OWN_DELAY, nowMs)) p4.enc_muted[1] = !value.as<bool>();
    } else if (strcmp(param, "delayMix") == 0) {
        if (!is_fx_owned_recent(FX_OWN_DELAY, nowMs)) p4.enc_value[1] = delay_mix_to_u7(value, p4.enc_value[1]);
    } else if (strcmp(param, "reverbActive") == 0) {
        if (!is_fx_owned_recent(FX_OWN_REVERB, nowMs)) p4.enc_muted[2] = !value.as<bool>();
    } else if (strcmp(param, "reverbMix") == 0) {
        if (!is_fx_owned_recent(FX_OWN_REVERB, nowMs)) p4.enc_value[2] = reverb_mix_to_u7(value, p4.enc_value[2]);
    } else if (strcmp(param, "phaserActive") == 0) {
        if (!is_fx_owned_recent(FX_OWN_PHASER, nowMs)) p4.pot_muted[2] = !value.as<bool>();
    } else if (strcmp(param, "phaserDepth") == 0) {
        if (!is_fx_owned_recent(FX_OWN_PHASER, nowMs)) p4.pot_value[2] = phaser_depth_to_u7(value, p4.pot_value[2]);
    } else if (strcmp(param, "bitCrush") == 0) {
        if (!is_fx_owned_recent(FX_OWN_BITCRUSH, nowMs)) {
            p4.bitcrush_bits = clamp_int(value.as<int>(), 8, 16);
            forward_fx_bitcrush_to_s3(p4.bitcrush_bits);
        }
    } else if (strcmp(param, "sampleRate") == 0) {
        if (!is_fx_owned_recent(FX_OWN_SAMPLE_RATE, nowMs)) {
            int sr = value.as<int>();
            p4.sample_rate_hz = (sr <= 0) ? 0 : clamp_int(sr, 9000, 32000);
            forward_fx_samplerate_to_s3(p4.sample_rate_hz);
        }
    } else if (strcmp(param, "distortion") == 0) {
        if (!is_fx_owned_recent(FX_OWN_DISTORTION, nowMs)) {
            float amount = value.as<float>();
            if (amount > 1.0f) amount /= 100.0f;
            p4.distortion_pct = clamp_int((int)(amount * 100.0f + 0.5f), 0, 100);
            forward_fx_distortion_to_s3(p4.distortion_pct);
        }
    } else {
        handled = false;
    }

    if (handled) udp_mark_fx_screen_dirty();
    log_remote_master_fx_param(param, value, handled);
}

static bool pattern_grid_has_data(void) {
    for (int t = 0; t < 16; t++) {
        for (int s = 0; s < 16; s++) {
            if (p4.steps[t][s]) return true;
        }
    }
    return false;
}

static int hex_nibble_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

static void decode_pattern_row_hex(const char* row, bool* dest, int stepCount) {
    if (!row || !dest) return;
    for (int s = 0; s < 64; s++) dest[s] = false;
    int nibbles = (stepCount + 3) / 4;
    if (nibbles > 16) nibbles = 16;
    for (int nib = 0; row[nib] && nib < nibbles; nib++) {
        int value = hex_nibble_value(row[nib]);
        if (value < 0) continue;
        int baseStep = nib * 4;
        for (int bit = 0; bit < 4; bit++) {
            int step = baseStep + bit;
            if (step >= stepCount || step >= 64) break;
            dest[step] = ((value >> bit) & 0x01) != 0;
        }
    }
}

static void apply_step_ownership_window(bool steps[16][64], int stepCount) {
    unsigned long nowMs = millis();
    int maxStep = clamp_int(stepCount, 16, 64);
    // p4.steps only tracks the first 16 steps (page 1) — clamp the source
    // read separately so steps 16..63 never index past p4.steps[t][16..].
    int p4MaxStep = (maxStep < 16) ? maxStep : 16;
    for (int t = 0; t < 16; t++) {
        for (int s = 0; s < p4MaxStep; s++) {
            unsigned long stamp = lastLocalStepMs[t][s];
            if (stamp != 0 && nowMs - stamp < LOCAL_STEP_OWNERSHIP_MS) {
                steps[t][s] = p4.steps[t][s];
            }
        }
    }
}

static inline void mark_local_step_edit(int track, int step) {
    if (track < 0 || track >= 16) return;
    if (step < 0 || step >= 64) return;
    lastLocalStepMs[track][step] = millis();
}

// Paso absoluto (0..63) del patrón sonando en el master. p4.current_step
// va plegado a 0..15 para los widgets de 16 pasos; los pads en modo sync
// necesitan el paso real para iluminar el compás 2+ de patrones largos.
static int s_step_raw = 0;

int udp_current_step_raw(void) { return s_step_raw; }

static inline void apply_master_step_sync(int step) {
    s_step_raw = clamp_int(step, 0, 63);
    int normalized = s_step_raw % 16;
    p4.current_step = normalized;
    lastMasterStepSyncMs = millis();
    // Keep a phase-locked local clock and slightly compensate network lag
    // so UI step follows audio closer on remote-master sessions.
    static uint32_t s_stepPhaseUs = 0;
    float bpm = p4.bpm_int + p4.bpm_frac * 0.1f;
    if (bpm < 40.0f) bpm = 120.0f;
    uint32_t stepIntervalUs = (uint32_t)(60000000.0f / bpm / 4.0f);
    // Tempo-aware compensation: bias a bit earlier so ACTIVE STEP UI aligns
    // better with perceived audio onset on WiFi sessions.
    uint32_t compUs = (stepIntervalUs * 45U) / 100U;
    const uint32_t COMP_MIN_US = 24000;
    const uint32_t COMP_MAX_US = 90000;
    if (compUs < COMP_MIN_US) compUs = COMP_MIN_US;
    if (compUs > COMP_MAX_US) compUs = COMP_MAX_US;
    if (compUs > (stepIntervalUs * 3U) / 5U) compUs = (stepIntervalUs * 3U) / 5U;
    s_stepPhaseUs = micros() - compUs;
    // Share phase with local runner (function-static in run_local_step_clock).
    extern uint32_t udp_step_phase_us_bridge;
    extern bool udp_step_phase_valid_bridge;
    udp_step_phase_us_bridge = s_stepPhaseUs;
    udp_step_phase_valid_bridge = true;
    uart_send_to_s3(MSG_SYSTEM, SYS_STEP, (uint8_t)normalized);
}

// Bridge vars used between apply_master_step_sync() and run_local_step_clock().
uint32_t udp_step_phase_us_bridge = 0;
bool udp_step_phase_valid_bridge = false;

// =============================================================================
// HTTP FETCH WORKER
// =============================================================================
// HTTPClient calls block for up to ~900 ms each (setTimeout), and the sync
// retry path used to chain two of them. Running that inside
// udp_handler_process() froze pads, UART forwarding and the local step clock
// for whole seconds. A low-priority worker task now performs the blocking
// transfer only; the JSON is parsed and applied back on the loop() task
// (http_consume_results), so all p4/UI state keeps a single writer thread.
enum HttpFetchKind : uint8_t { HTTP_FETCH_NONE = 0, HTTP_FETCH_PATTERN, HTTP_FETCH_STATE };

static std::atomic<int>     s_http_req_pattern{-1};   // pattern index, -1 = none (latest wins)
static std::atomic<bool>    s_http_req_state{false};
static std::atomic<uint8_t> s_http_resp_kind{HTTP_FETCH_NONE};
static int  s_http_resp_pattern = 0;     // written by worker before resp_kind
static char s_http_resp_buf[12288];      // body handoff worker → loop()

static void request_pattern_http(int pattern) {
    s_http_req_pattern.store(pattern, std::memory_order_release);
}

static void request_master_state_http(void) {
    s_http_req_state.store(true, std::memory_order_release);
}

class FixedBufferStream final : public Stream {
public:
    FixedBufferStream(char* buffer, size_t capacity)
        : buffer_(buffer), capacity_(capacity) {}

    size_t write(uint8_t value) override {
        return write(&value, 1);
    }

    size_t write(const uint8_t* data, size_t length) override {
        if (!data || length > capacity_ - size_) {
            overflow_ = true;
            return 0;
        }
        memcpy(buffer_ + size_, data, length);
        size_ += length;
        return length;
    }

    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
    void flush() override {}

    size_t size() const { return size_; }
    bool overflowed() const { return overflow_; }

private:
    char* buffer_;
    size_t capacity_;
    size_t size_ = 0;
    bool overflow_ = false;
};

// Worker task context. Returns true with the body in s_http_resp_buf.
static bool http_fetch_to_buf(const char* url) {
    HTTPClient http;
    http.setTimeout(MASTER_HTTP_TIMEOUT_MS);
    http.setConnectTimeout(MASTER_HTTP_TIMEOUT_MS);
    if (!http.begin(url)) return false;

    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        http.end();
        return false;
    }

    int declared_len = http.getSize();
    if (declared_len >= (int)sizeof(s_http_resp_buf)) {
        P4_LOG_PRINTF("[HTTP] response too large (%d bytes)\n", declared_len);
        http.end();
        return false;
    }

    // HTTPClient decodes both Content-Length and chunked transfer encoding.
    // The fixed Stream avoids the temporary heap String used by getString().
    FixedBufferStream sink(s_http_resp_buf, sizeof(s_http_resp_buf) - 1);
    int written = http.writeToStream(&sink);
    http.end();
    size_t total = sink.size();
    if (written < 0 || sink.overflowed() || total == 0 ||
        (declared_len >= 0 && total != (size_t)declared_len)) {
        P4_LOG_PRINTF("[HTTP] response incomplete (%u/%d bytes)\n",
                      (unsigned)total, declared_len);
        return false;
    }
    s_http_resp_buf[total] = '\0';
    return true;
}

static void http_fetch_task(void* arg) {
    (void)arg;
    for (;;) {
        // Wait until loop() consumed the previous response.
        if (s_http_resp_kind.load(std::memory_order_acquire) != HTTP_FETCH_NONE) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (!wifiConnected) {
            // Drop requests while offline; sync/watchdog paths re-request.
            s_http_req_state.store(false, std::memory_order_relaxed);
            s_http_req_pattern.store(-1, std::memory_order_relaxed);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (s_http_req_state.exchange(false, std::memory_order_acq_rel)) {
            if (http_fetch_to_buf("http://192.168.4.1/api/p4State")) {
                s_http_resp_kind.store(HTTP_FETCH_STATE, std::memory_order_release);
            }
            continue;
        }
        int pat = s_http_req_pattern.exchange(-1, std::memory_order_acq_rel);
        if (pat >= 0) {
            char url[96];
            snprintf(url, sizeof(url), "http://192.168.4.1/api/getPattern?index=%d", pat);
            if (http_fetch_to_buf(url)) {
                s_http_resp_pattern = pat;
                s_http_resp_kind.store(HTTP_FETCH_PATTERN, std::memory_order_release);
            }
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// loop() task context — parse + apply a pattern body fetched by the worker.
static bool apply_pattern_http_payload(int pattern, const char* payload) {
    // Staleness guard, mirroring the UDP pattern_sync path: a GET already in
    // flight for pattern A can land after the user moved on to pattern B
    // (the request atomics only coalesce *queued* requests). Apply only if
    // the body matches what we asked for or what is currently displayed.
    if (pattern != pendingPatternRequest && pattern != p4.current_pattern) {
        P4_LOG_PRINTF("[HTTP][PAT] stale response pat=%d cur=%d waiting=%d (ignored)\n",
                      pattern, p4.current_pattern, pendingPatternRequest);
        return false;
    }
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (err) {
        P4_LOG_PRINTF("[HTTP][PAT] parse error: %s\n", err.c_str());
        return false;
    }

    bool raw_steps[16][64] = {};
    int raw_len = 16;
    for (int track = 0; track < 16; track++) {
        char key[4];
        snprintf(key, sizeof(key), "%d", track);
        JsonArray row = doc[key];
        if (!row) continue;
        int step = 0;
        for (JsonVariant val : row) {
            if (step >= 64) break;
            raw_steps[track][step] = val.as<bool>();
            step++;
        }
        if (step > raw_len) raw_len = step;
    }

    p4.current_pattern = clamp_int(pattern, 0, Config::MAX_PATTERNS - 1);
    ui_sequencer_load_external_pattern(raw_steps, clamp_int(raw_len, 16, 64));
    pendingPatternRequest = -1;
    pendingPatternRetries = 0;
    pendingPatternLastTxMs = 0;
    masterAlive = true;
    p4.master_connected = true;
    lastMasterPacket = millis();
    uart_send_pattern_to_s3(p4.current_pattern, p4.steps);
    P4_LOG_PRINTF("[HTTP][PAT] loaded pattern %d len=%d\n", pattern, raw_len);
    return true;
}

// loop() task context — parse + apply a master-state body fetched by the worker.
static bool apply_master_state_http_payload(const char* payload) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (err) {
        P4_LOG_PRINTF("[HTTP][STATE] parse error: %s\n", err.c_str());
        return false;
    }

    // A 200 from the master proves it's reachable — keep the reconnect path's
    // liveness signal now that the pattern grid is no longer fetched eagerly.
    // Refresh lastMasterPacket too, otherwise the loop's timeout check would
    // flip masterAlive straight back to false on the next iteration.
    masterAlive = true;
    p4.master_connected = true;
    lastMasterPacket = millis();

    int pat = clamp_int(doc["pattern"] | p4.current_pattern, 0, Config::MAX_PATTERNS - 1);
    // Don't let a periodic state poll overwrite a just-made local pattern pick.
    if (master_pattern_report_should_apply(pat)) {
        p4.current_pattern = pat;
        uart_send_to_s3(MSG_SYSTEM, SYS_PATTERN, (uint8_t)pat);
    }

    bool playing = doc["playing"] | p4.is_playing;
    p4.is_playing = playing;
    uart_send_to_s3(MSG_SYSTEM, SYS_PLAY_STATE, playing ? 1 : 0);

    float bpm = clamp_float(doc["tempo"] | (float)(p4.bpm_int + p4.bpm_frac * 0.1f), 40.0f, 240.0f);
    p4.bpm_int = (int)bpm;
    p4.bpm_frac = (int)((bpm - p4.bpm_int) * 10.0f);
    uart_send_to_s3(MSG_SYSTEM, SYS_BPM_INT, (uint8_t)p4.bpm_int);
    uart_send_to_s3(MSG_SYSTEM, SYS_BPM_FRAC, (uint8_t)p4.bpm_frac);

    int masterVol = clamp_int(doc["masterVolume"] | p4.master_volume, 0, 150);
    p4.master_volume = masterVol;
    p4.seq_volume = clamp_int(doc["sequencerVolume"] | p4.seq_volume, 0, 150);
    p4.live_volume = clamp_int(doc["liveVolume"] | p4.live_volume, 0, 150);
    uart_send_to_s3(MSG_SYSTEM, SYS_VOLUME, (uint8_t)masterVol);
    uart_send_to_s3(MSG_SYSTEM, SYS_SEQ_VOL, (uint8_t)p4.seq_volume);
    uart_send_to_s3(MSG_SYSTEM, SYS_LIVE_VOL, (uint8_t)p4.live_volume);

    JsonArray mute = doc["mute"];
    if (mute) {
        unsigned long nowMs = millis();
        int track = 0;
        for (JsonVariant value : mute) {
            if (track >= 16) break;
            if (lastLocalMuteMs[track] != 0 &&
                nowMs - lastLocalMuteMs[track] < LOCAL_OWNERSHIP_MS) {
                bool incoming = value.as<bool>();
                if (incoming != p4.track_muted[track]) {
                    P4_LOG_PRINTF("[SYNC][MUTE] keep-local t=%d incoming=%d local=%d age=%lu\n",
                                  track, (int)incoming, (int)p4.track_muted[track],
                                  (unsigned long)(nowMs - lastLocalMuteMs[track]));
                }
                track++;
                continue;
            }
            bool v = value.as<bool>();
            if (p4.track_muted[track] != v) {
                P4_LOG_PRINTF("[SYNC][MUTE] apply-master t=%d old=%d new=%d age=%lu\n",
                              track, (int)p4.track_muted[track], (int)v,
                              (unsigned long)(nowMs - lastLocalMuteMs[track]));
                p4.track_muted[track] = v;
                uart_send_to_s3(MSG_TRACK, TRK_MUTE_BIT | (track & 0x0F), v ? 1 : 0);
            }
            track++;
        }
    }

    JsonArray solo = doc["solo"];
    if (solo) {
        unsigned long nowMs = millis();
        int track = 0;
        for (JsonVariant value : solo) {
            if (track >= 16) break;
            if (lastLocalSoloMs[track] != 0 &&
                nowMs - lastLocalSoloMs[track] < LOCAL_OWNERSHIP_MS) {
                bool incoming = value.as<bool>();
                if (incoming != p4.track_solo[track]) {
                    P4_LOG_PRINTF("[SYNC][SOLO] keep-local t=%d incoming=%d local=%d age=%lu\n",
                                  track, (int)incoming, (int)p4.track_solo[track],
                                  (unsigned long)(nowMs - lastLocalSoloMs[track]));
                }
                track++;
                continue;
            }
            bool v = value.as<bool>();
            if (p4.track_solo[track] != v) {
                P4_LOG_PRINTF("[SYNC][SOLO] apply-master t=%d old=%d new=%d age=%lu\n",
                              track, (int)p4.track_solo[track], (int)v,
                              (unsigned long)(nowMs - lastLocalSoloMs[track]));
                p4.track_solo[track] = v;
                uart_send_to_s3(MSG_TRACK, TRK_SOLO_BIT | (track & 0x0F), v ? 1 : 0);
            }
            track++;
        }
    }

    JsonArray volumes = doc["trackVolumes"];
    if (volumes) {
        unsigned long nowMs = millis();
        int track = 0;
        for (JsonVariant value : volumes) {
            if (track >= 16) break;
            if (is_track_volume_owned_recent(track, nowMs)) {
                track++;
                continue;
            }
            int vol = clamp_int(value.as<int>(), 0, 150);
            p4.track_volume[track] = vol;
            uart_send_to_s3(MSG_TRACK, TRK_VOLUME | (track & 0x0F), (uint8_t)vol);
            track++;
        }
    }

    JsonArray trackSynthEngines = doc["trackSynthEngines"];
    if (trackSynthEngines) {
        int8_t engines[16];
        for (int t = 0; t < 16; t++) engines[t] = -1;
        int track = 0;
        for (JsonVariant value : trackSynthEngines) {
            if (track >= 16) break;
            engines[track] = (int8_t)clamp_int(value.as<int>(), -1, 7);  // 7 = GuitarT
            track++;
        }
        ui_pad_sound_sync_track_engines(engines);
    }

    JsonObject fx = doc["fx"];
    if (fx) {
        unsigned long nowMs = millis();
        if (!is_fx_owned_recent(FX_OWN_FILTER_TYPE, nowMs)) {
            p4.filter_type = clamp_int(fx["filterType"] | p4.filter_type, 0, 4);
            forward_fx_filter_type_to_s3(p4.filter_type);
        }
        if (!is_fx_owned_recent(FX_OWN_CUTOFF, nowMs)) {
            p4.cutoff_hz = clamp_int(fx["filterCutoff"] | p4.cutoff_hz, 20, 20000);
            forward_fx_cutoff_to_s3(p4.cutoff_hz);
        }
        if (!is_fx_owned_recent(FX_OWN_RESONANCE, nowMs)) {
            float resonance = clamp_float(fx["filterResonance"] | ((float)p4.resonance_x10 / 10.0f), 1.0f, 10.0f);
            p4.resonance_x10 = (int)(resonance * 10.0f);
            forward_fx_resonance_to_s3(p4.resonance_x10);
        }
        if (!is_fx_owned_recent(FX_OWN_DISTORTION, nowMs)) {
            float distortion = clamp_float(fx["distortion"] | ((float)p4.distortion_pct / 100.0f), 0.0f, 1.0f);
            p4.distortion_pct = (int)(distortion * 100.0f);
            forward_fx_distortion_to_s3(p4.distortion_pct);
        }
        if (!is_fx_owned_recent(FX_OWN_BITCRUSH, nowMs)) {
            p4.bitcrush_bits = clamp_int(fx["bitCrush"] | p4.bitcrush_bits, 4, 16);
            forward_fx_bitcrush_to_s3(p4.bitcrush_bits);
        }
        if (!is_fx_owned_recent(FX_OWN_SAMPLE_RATE, nowMs)) {
            int sr = fx["sampleRate"] | p4.sample_rate_hz;
            p4.sample_rate_hz = (sr <= 0) ? 0 : clamp_int(sr, 9000, 32000);
            forward_fx_samplerate_to_s3(p4.sample_rate_hz);
        }
        apply_remote_macro_fx_state(fx, "http", lastRemoteHttpFxLogMs);
        apply_remote_raydrone_state(fx["raydrone"].as<JsonObjectConst>());
    }

    const char* kit = doc["kit"] | "";
    strncpy(p4.kit_name, kit, sizeof(p4.kit_name) - 1);
    p4.kit_name[sizeof(p4.kit_name) - 1] = '\0';

    masterAlive = true;
    p4.master_connected = true;
    lastMasterPacket = millis();  // without this the 3 s timeout re-fires at once
    P4_LOG_PRINTF("[HTTP][STATE] loaded pat=%d bpm=%.1f\n", pat, bpm);
    return true;
}

// Called from udp_handler_process() — applies at most one fetched body per
// loop iteration, on the same thread that owns p4/UI state.
static void http_consume_results(void) {
    uint8_t kind = s_http_resp_kind.load(std::memory_order_acquire);
    if (kind == HTTP_FETCH_NONE) return;
    if (kind == HTTP_FETCH_PATTERN) {
        apply_pattern_http_payload(s_http_resp_pattern, s_http_resp_buf);
    } else if (kind == HTTP_FETCH_STATE) {
        apply_master_state_http_payload(s_http_resp_buf);
    }
    s_http_resp_kind.store(HTTP_FETCH_NONE, std::memory_order_release);
}

// =============================================================================
// SEND HELPERS
// =============================================================================
// sendJson is reached from loop(), LVGL and touch tasks. WiFiUDP itself is not
// safe for concurrent beginPacket/write/endPacket sequences, and waiting on a
// mutex here used to stall LVGL for up to 50 ms. Producers now enqueue into a
// short staging buffer; udp_handler_process() is the sole WiFiUDP TX owner.
static constexpr size_t UDP_TX_MAX_LEN = 768;
static constexpr int UDP_TX_QUEUE_DEPTH = 24;
struct UdpTxPacket {
    uint16_t len;
    char data[UDP_TX_MAX_LEN];
};
static QueueHandle_t s_udp_tx_queue = NULL;
static SemaphoreHandle_t s_udp_tx_stage_mutex = NULL;
static UdpTxPacket s_udp_tx_stage;
static std::atomic<uint32_t> s_udp_tx_drops{0};

static void sendJson(const char* json) {
    if (!udpStarted || !json || !s_udp_tx_queue || !s_udp_tx_stage_mutex) return;
    size_t len = strnlen(json, UDP_TX_MAX_LEN);
    if (len == 0 || len >= UDP_TX_MAX_LEN) {
        s_udp_tx_drops.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    if (xSemaphoreTake(s_udp_tx_stage_mutex, pdMS_TO_TICKS(2)) != pdTRUE) {
        s_udp_tx_drops.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    s_udp_tx_stage.len = (uint16_t)len;
    memcpy(s_udp_tx_stage.data, json, len);
    bool queued = xQueueSend(s_udp_tx_queue, &s_udp_tx_stage, 0) == pdTRUE;
    xSemaphoreGive(s_udp_tx_stage_mutex);
    if (!queued) s_udp_tx_drops.fetch_add(1, std::memory_order_relaxed);
}

static void udp_drain_tx_queue(int budget) {
    if (!udpStarted || !s_udp_tx_queue) return;
    static UdpTxPacket pkt;
    while (budget-- > 0 && xQueueReceive(s_udp_tx_queue, &pkt, 0) == pdTRUE) {
        if (!udp.beginPacket(MASTER_IP, UDP_PORT)) continue;
        udp.write((const uint8_t*)pkt.data, pkt.len);
        udp.endPacket();
    }
}

static void sendCmd(const char* cmd) {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"%s\"}", cmd);
    sendJson(buf);
}

// =============================================================================
// PUBLIC SEND API
// =============================================================================
void udp_send_trigger(uint8_t pad, uint8_t velocity) {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"trigger\",\"pad\":%d,\"vel\":%d}", pad, velocity);
    sendJson(buf);
}

void udp_send_synth_note_on_ex(uint8_t engine, uint8_t note, uint8_t velocity,
                                bool accent, bool slide) {
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"synthNoteOnEx\",\"engine\":%u,\"note\":%u,\"velocity\":%u,"
             "\"accent\":%s,\"slide\":%s}",
             (unsigned)engine, (unsigned)note, (unsigned)velocity,
             accent ? "true" : "false", slide ? "true" : "false");
    sendJson(buf);
}

void udp_send_synth_note_off(uint8_t engine, uint8_t track) {
    char buf[80];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"synthNoteOff\",\"engine\":%u,\"track\":%u}",
             (unsigned)engine, (unsigned)track);
    sendJson(buf);
}

void udp_send_synth_note_off_ex(uint8_t engine, uint8_t track, uint8_t note) {
    char buf[96];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"synthNoteOff\",\"engine\":%u,\"track\":%u,\"note\":%u}",
             (unsigned)engine, (unsigned)track, (unsigned)note);
    sendJson(buf);
}

void udp_send_synth_trigger(uint8_t engine, uint8_t instrument, uint8_t velocity) {
    char buf[112];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"synthTrigger\",\"engine\":%u,\"instrument\":%u,\"velocity\":%u}",
             (unsigned)engine, (unsigned)instrument, (unsigned)velocity);
    sendJson(buf);
}

void udp_send_synth303_note_off(void) {
    sendJson("{\"cmd\":\"synth303NoteOff\"}");
}

void udp_send_synth_param(uint8_t engine, uint8_t instrument, uint8_t paramId, float value) {
    char buf[160];
    if (engine == 3) {
        // TB-303 uses dedicated command
        snprintf(buf, sizeof(buf),
                 "{\"cmd\":\"synth303Param\",\"paramId\":%u,\"value\":%.4f}",
                 (unsigned)paramId, value);
    } else {
        snprintf(buf, sizeof(buf),
                 "{\"cmd\":\"synthParam\",\"engine\":%u,\"instrument\":%u,\"paramId\":%u,\"value\":%.4f}",
                 (unsigned)engine, (unsigned)instrument, (unsigned)paramId, value);
    }
    sendJson(buf);
}

void udp_send_synth_preset(uint8_t engine, uint8_t preset) {
    char buf[96];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"synthPreset\",\"engine\":%u,\"preset\":%u}",
             (unsigned)engine, (unsigned)preset);
    sendJson(buf);
}

void udp_send_trim_sample(uint8_t pad, float trim_start, float trim_end) {
    if (trim_start < 0.0f) trim_start = 0.0f;
    if (trim_start > 0.98f) trim_start = 0.98f;
    if (trim_end < trim_start + 0.01f) trim_end = trim_start + 0.01f;
    if (trim_end > 1.0f) trim_end = 1.0f;
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"trimSample\",\"pad\":%u,\"trimStart\":%.4f,\"trimEnd\":%.4f}",
             (unsigned)pad, trim_start, trim_end);
    sendJson(buf);
}

void udp_send_melody_rec_note(uint8_t engine, uint8_t note) {
    char buf[96];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"melodyRecNote\",\"engine\":%u,\"note\":%u}",
             (unsigned)engine, (unsigned)note);
    sendJson(buf);
}

// v2.9 — Master now keeps melody state. Tell it REC is toggled and which
// engine/octave we're using so it can mirror the change to all slaves.
void udp_send_melody_rec_toggle(bool active, uint8_t engine, uint8_t octave) {
    char buf[96];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"melodyRecToggle\",\"active\":%d,\"engine\":%u,\"octave\":%u}",
             active ? 1 : 0, (unsigned)engine, (unsigned)octave);
    sendJson(buf);
}

void udp_send_melody_set_pad(uint8_t pad) {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"melodySetPad\",\"pad\":%u}", (unsigned)pad);
    sendJson(buf);
}

void udp_send_melody_set_engine(uint8_t engine) {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"melodySetEngine\",\"engine\":%u}", (unsigned)engine);
    sendJson(buf);
}

void udp_send_melody_set_octave(uint8_t octave) {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"melodySetOctave\",\"octave\":%u}", (unsigned)octave);
    sendJson(buf);
}

void udp_send_melody_clear(void) {
    sendJson("{\"cmd\":\"melodyClear\"}");
}

void udp_send_melody_assign_pad(uint8_t pad, uint8_t engine, uint8_t octave) {
    char buf[96];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"melodyAssign\",\"pad\":%u,\"engine\":%u,\"octave\":%u}",
             (unsigned)pad, (unsigned)engine, (unsigned)octave);
    sendJson(buf);
}

void udp_send_melody_assign(uint8_t pad, uint8_t engine, uint8_t octave,
                            const bool grid[16][12],
                            const uint8_t notes[16][12],
                            uint8_t gate_percent) {
    // Mirror the JSON shape produced by S3's ui_screens.cpp mel_assign_cb:
    //   {"cmd":"melodyAssign","pad":N,"engine":E,"octave":O,
    //    "steps":[[midi,midi,...], ... 16 columns]}
    char buf[768];
    gate_percent = (uint8_t)constrain((int)gate_percent, 10, 100);
    int n = snprintf(buf, sizeof(buf),
                     "{\"cmd\":\"melodyAssign\",\"pad\":%u,\"engine\":%u,"
                     "\"octave\":%u,\"gate\":%u,\"steps\":[",
                     (unsigned)pad, (unsigned)engine, (unsigned)octave,
                     (unsigned)gate_percent);
    if (n < 0 || n >= (int)sizeof(buf)) return;
    // S3 mapping: row r -> pc = (11 - r), midi = (octave + 1) * 12 + pc
    for (int c = 0; c < 16; c++) {
        if (n >= (int)sizeof(buf) - 8) return;
        if (c > 0) buf[n++] = ',';
        buf[n++] = '[';
        bool first = true;
        for (int r = 0; r < 12; r++) {
            if (!grid[c][r]) continue;
            int midi = notes ? notes[c][r] : 0;
            if (midi <= 0) {
                int pc = 11 - r;
                midi = ((int)octave + 1) * 12 + pc;
            }
            if (midi < 0) midi = 0;
            if (midi > 127) midi = 127;
            int w = snprintf(buf + n, sizeof(buf) - n,
                             "%s%d", first ? "" : ",", midi);
            if (w < 0 || w >= (int)(sizeof(buf) - n)) return;
            n += w;
            first = false;
        }
        if (n >= (int)sizeof(buf) - 4) return;
        buf[n++] = ']';
    }
    if (n >= (int)sizeof(buf) - 3) return;
    buf[n++] = ']';
    buf[n++] = '}';
    buf[n] = 0;
    sendJson(buf);
}

void udp_send_start(void) { sendCmd("start"); }
void udp_send_stop(void)  { sendCmd("stop"); }

void udp_send_tempo(float bpm) {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"tempo\",\"value\":%.1f}", bpm);
    sendJson(buf);
}

void udp_send_select_pattern(int index) {
    lastLocalPatternMs = millis();
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"selectPattern\",\"index\":%d}", index);
    P4_LOG_PRINTF("[UDP][PAT] TX selectPattern idx=%d\n", index);
    sendJson(buf);
}

void udp_send_get_pattern(int pattern) {
    char buf[64];
    if (pendingPatternRequest != pattern) {
        pendingPatternRequest = pattern;
        pendingPatternRetries = 0;
    }
    snprintf(buf, sizeof(buf), "{\"cmd\":\"get_pattern\",\"pattern\":%d}", pattern);
    pendingPatternLastTxMs = millis();
    if (pendingPatternRetries < 255) pendingPatternRetries++;
    P4_LOG_PRINTF("[UDP][PAT] TX get_pattern idx=%d\n", pattern);
    sendJson(buf);
    // Keep the request path non-blocking: worker fetch is async and consumed
    // on loop() by http_consume_results().
    request_pattern_http(pattern);
}

void udp_send_set_step(int track, int step, bool active) {
    mark_local_step_edit(track, step);
    char buf[96];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"setStep\",\"track\":%d,\"step\":%d,\"active\":%s}",
             track, step, active ? "true" : "false");
    sendJson(buf);
}

void udp_send_set_step_velocity(int track, int step, int velocity) {
    char buf[96];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"setStepVelocity\",\"track\":%d,\"step\":%d,\"velocity\":%d}",
             track, step, constrain(velocity, 1, 127));
    sendJson(buf);
}

void udp_send_mute(int track, bool muted) {
    if (track >= 0 && track < 16) lastLocalMuteMs[track] = millis();
    char buf[64];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"mute\",\"track\":%d,\"value\":%s}",
             track, muted ? "true" : "false");
    sendJson(buf);
}

void udp_send_solo(int track, bool soloed) {
    if (track >= 0 && track < 16) {
        // Solo touches every track's solo + mute state on the master side.
        // Mark all 16 entries to suppress racing state_sync overwrites.
        unsigned long now = millis();
        for (int t = 0; t < 16; t++) {
            lastLocalSoloMs[t] = now;
            lastLocalMuteMs[t] = now;
        }
    }
    char buf[64];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"solo\",\"track\":%d,\"value\":%s}",
             track, soloed ? "true" : "false");
    sendJson(buf);
}

void udp_send_mute_mask(uint16_t mask) {
    unsigned long now = millis();
    for (int t = 0; t < 16; t++) lastLocalMuteMs[t] = now;
    char buf[64];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"setMuteMask\",\"mask\":%u}", (unsigned)mask);
    sendJson(buf);
}

void udp_send_solo_mask(uint16_t mask) {
    unsigned long now = millis();
    for (int t = 0; t < 16; t++) {
        lastLocalSoloMs[t] = now;
        lastLocalMuteMs[t] = now;
    }
    char buf[64];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"setSoloMask\",\"mask\":%u}", (unsigned)mask);
    sendJson(buf);
}

void udp_send_set_volume(int value) {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"setVolume\",\"value\":%d}", value);
    sendJson(buf);
}

void udp_send_set_seq_volume(int value) {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"setSequencerVolume\",\"value\":%d}", value);
    sendJson(buf);
}

void udp_send_set_live_volume(int value) {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"setLiveVolume\",\"value\":%d}", value);
    sendJson(buf);
}

void udp_send_set_track_volume(int track, int volume) {
    mark_local_track_volume(track);
    char buf[96];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"setTrackVolume\",\"track\":%d,\"volume\":%d}",
             track, volume);
    sendJson(buf);
}

void udp_send_set_track_engine(int track, int engine) {
    if (track < 0) track = 0;
    if (track > 15) track = 15;
    if (engine < -1) engine = -1;
    if (engine > 6) engine = 6;
    char buf[96];
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"setTrackSynthEngine\",\"track\":%d,\"engine\":%d}",
             track, engine);
    sendJson(buf);
}

void udp_send_set_filter(int type) {
    mark_local_fx(FX_OWN_FILTER_TYPE);
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"setFilter\",\"type\":%d}", type);
    sendJson(buf);
}

void udp_send_set_filter_cutoff(int hz) {
    mark_local_fx(FX_OWN_CUTOFF);
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"setFilterCutoff\",\"value\":%d}", hz);
    sendJson(buf);
}

void udp_send_set_filter_resonance(float val) {
    mark_local_fx(FX_OWN_RESONANCE);
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"setFilterResonance\",\"value\":%.1f}", val);
    sendJson(buf);
}

void udp_send_set_distortion(float val) {
    mark_local_fx(FX_OWN_DISTORTION);
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"setDistortion\",\"value\":%.2f}", val);
    sendJson(buf);
}

void udp_send_set_bitcrush(int bits) {
    mark_local_fx(FX_OWN_BITCRUSH);
    int clamped = clamp_int(bits, 8, 16);
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"setBitCrush\",\"value\":%d}", clamped);
    sendJson(buf);
}

void udp_send_set_sample_rate(int rateHz) {
    mark_local_fx(FX_OWN_SAMPLE_RATE);
    // 0 is accepted by master as bypass/off; non-zero uses audible SRATE range.
    int clamped = (rateHz <= 0) ? 0 : clamp_int(rateHz, 9000, 32000);
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"setSampleRate\",\"value\":%d}", clamped);
    sendJson(buf);
}

static bool apply_local_raydrone(
    const RaydroneConfigPayload& patch, uint8_t updateMask,
    RaydroneTxWork& work) {
    portENTER_CRITICAL(&s_raydrone_ownership_mux);
    const RaydroneConfigPayload state =
        p4_raydrone_update(patch, updateMask);
    const bool started = begin_raydrone_request_locked(
        updateMask, state, millis(), work);
    portEXIT_CRITICAL(&s_raydrone_ownership_mux);
    if (started) udp_mark_fx_screen_dirty();
    return started;
}

static void send_raydrone_field(uint8_t updateMask,
                                uint32_t requestId,
                                const RaydroneConfigPayload& state) {
    char buf[160];
    switch (updateMask) {
        case RAYDRONE_UPDATE_ACTIVE:
            snprintf(buf, sizeof(buf),
                     "{\"cmd\":\"setRaydrone\",\"version\":%u,\"updateMask\":%u,"
                     "\"requestId\":%lu,\"active\":%s}",
                     (unsigned)state.version,
                     (unsigned)updateMask, (unsigned long)requestId,
                     (state.flags & RAYDRONE_FLAG_ACTIVE) ? "true" : "false");
            break;
        case RAYDRONE_UPDATE_MATERIAL:
            snprintf(buf, sizeof(buf),
                     "{\"cmd\":\"setRaydrone\",\"version\":%u,\"updateMask\":%u,"
                     "\"requestId\":%lu,\"material\":%u}",
                     (unsigned)state.version, (unsigned)updateMask,
                     (unsigned long)requestId, (unsigned)state.material);
            break;
        case RAYDRONE_UPDATE_CHARACTER:
            snprintf(buf, sizeof(buf),
                     "{\"cmd\":\"setRaydrone\",\"version\":%u,\"updateMask\":%u,"
                     "\"requestId\":%lu,\"character\":%u}",
                     (unsigned)state.version, (unsigned)updateMask,
                     (unsigned long)requestId, (unsigned)state.character);
            break;
        case RAYDRONE_UPDATE_MOTION:
            snprintf(buf, sizeof(buf),
                     "{\"cmd\":\"setRaydrone\",\"version\":%u,\"updateMask\":%u,"
                     "\"requestId\":%lu,\"motion\":%u}",
                     (unsigned)state.version, (unsigned)updateMask,
                     (unsigned long)requestId, (unsigned)state.motion);
            break;
        case RAYDRONE_UPDATE_SPACE:
            snprintf(buf, sizeof(buf),
                     "{\"cmd\":\"setRaydrone\",\"version\":%u,\"updateMask\":%u,"
                     "\"requestId\":%lu,\"space\":%u}",
                     (unsigned)state.version, (unsigned)updateMask,
                     (unsigned long)requestId, (unsigned)state.space);
            break;
        case RAYDRONE_UPDATE_VOLUME:
            snprintf(buf, sizeof(buf),
                     "{\"cmd\":\"setRaydrone\",\"version\":%u,\"updateMask\":%u,"
                     "\"requestId\":%lu,\"volume\":%u}",
                     (unsigned)state.version, (unsigned)updateMask,
                     (unsigned long)requestId, (unsigned)state.volume);
            break;
        case RAYDRONE_UPDATE_MIX:
            snprintf(buf, sizeof(buf),
                     "{\"cmd\":\"setRaydrone\",\"version\":%u,\"updateMask\":%u,"
                     "\"requestId\":%lu,\"mix\":%u}",
                     (unsigned)state.version, (unsigned)updateMask,
                     (unsigned long)requestId, (unsigned)state.mix);
            break;
        case RAYDRONE_UPDATE_EVOLUTION:
            snprintf(buf, sizeof(buf),
                     "{\"cmd\":\"setRaydrone\",\"version\":%u,\"updateMask\":%u,"
                     "\"requestId\":%lu,\"evolution\":%u}",
                     (unsigned)state.version, (unsigned)updateMask,
                     (unsigned long)requestId, (unsigned)state.evolution);
            break;
        default:
            return;
    }
    sendJson(buf);
}

static void retry_pending_raydrone(uint32_t nowMs, bool force) {
    if (!udpStarted || !s_raydrone_tx_mutex) return;
    const TickType_t waitTicks = force ? pdMS_TO_TICKS(5) : 0;
    if (xSemaphoreTake(s_raydrone_tx_mutex, waitTicks) != pdTRUE) return;

    RaydroneTxWork work[RAYDRONE_FIELD_COUNT] = {};
    int workCount = 0;
    portENTER_CRITICAL(&s_raydrone_ownership_mux);
    for (int i = 0; i < RAYDRONE_FIELD_COUNT; ++i) {
        RaydronePendingRequest& pending = s_raydrone_pending[i];
        if (!pending.active) continue;
        if (!force &&
            (uint32_t)(nowMs - pending.lastTxMs) < RAYDRONE_RETRY_MS) {
            continue;
        }

        pending.lastTxMs = nowMs;
        RaydroneTxWork& tx = work[workCount++];
        tx.updateMask = pending.updateMask;
        tx.requestId = pending.requestId;
        tx.state = pending.state;
    }
    portEXIT_CRITICAL(&s_raydrone_ownership_mux);

    for (int i = 0; i < workCount; ++i) {
        send_raydrone_field(
            work[i].updateMask, work[i].requestId, work[i].state);
    }
    xSemaphoreGive(s_raydrone_tx_mutex);
}

void udp_raydrone_toggle_active(void) {
    const bool txLocked = s_raydrone_tx_mutex &&
        xSemaphoreTake(s_raydrone_tx_mutex, portMAX_DELAY) == pdTRUE;
    RaydroneTxWork work = {};
    portENTER_CRITICAL(&s_raydrone_ownership_mux);
    RaydroneConfigPayload patch = p4_raydrone_snapshot();
    patch.flags =
        (patch.flags & RAYDRONE_FLAG_ACTIVE) ? 0 : RAYDRONE_FLAG_ACTIVE;
    const RaydroneConfigPayload state =
        p4_raydrone_update(patch, RAYDRONE_UPDATE_ACTIVE);
    const bool started = begin_raydrone_request_locked(
        RAYDRONE_UPDATE_ACTIVE, state, millis(), work);
    portEXIT_CRITICAL(&s_raydrone_ownership_mux);
    if (started) {
        udp_mark_fx_screen_dirty();
        send_raydrone_field(work.updateMask, work.requestId, work.state);
    }
    if (txLocked) xSemaphoreGive(s_raydrone_tx_mutex);
}

void udp_raydrone_cycle_material(void) {
    const bool txLocked = s_raydrone_tx_mutex &&
        xSemaphoreTake(s_raydrone_tx_mutex, portMAX_DELAY) == pdTRUE;
    RaydroneTxWork work = {};
    portENTER_CRITICAL(&s_raydrone_ownership_mux);
    RaydroneConfigPayload patch = p4_raydrone_snapshot();
    patch.material =
        (uint8_t)((patch.material + 1u) % RAYDRONE_MATERIAL_COUNT);
    const RaydroneConfigPayload state =
        p4_raydrone_update(patch, RAYDRONE_UPDATE_MATERIAL);
    const bool started = begin_raydrone_request_locked(
        RAYDRONE_UPDATE_MATERIAL, state, millis(), work);
    portEXIT_CRITICAL(&s_raydrone_ownership_mux);
    if (started) {
        udp_mark_fx_screen_dirty();
        send_raydrone_field(work.updateMask, work.requestId, work.state);
    }
    if (txLocked) xSemaphoreGive(s_raydrone_tx_mutex);
}

void udp_set_raydrone_value(uint8_t updateMask, uint8_t value,
                            bool transmit) {
    RaydroneConfigPayload patch = {};
    switch (updateMask) {
        case RAYDRONE_UPDATE_CHARACTER: patch.character = value; break;
        case RAYDRONE_UPDATE_MOTION:    patch.motion = value; break;
        case RAYDRONE_UPDATE_SPACE:     patch.space = value; break;
        case RAYDRONE_UPDATE_VOLUME:    patch.volume = value; break;
        case RAYDRONE_UPDATE_MIX:       patch.mix = value; break;
        case RAYDRONE_UPDATE_EVOLUTION: patch.evolution = value; break;
        default: return;
    }
    const bool txLocked = s_raydrone_tx_mutex &&
        xSemaphoreTake(s_raydrone_tx_mutex, portMAX_DELAY) == pdTRUE;
    RaydroneTxWork work = {};
    const bool started = apply_local_raydrone(patch, updateMask, work);
    if (started && transmit) {
        send_raydrone_field(work.updateMask, work.requestId, work.state);
    }
    if (txLocked) xSemaphoreGive(s_raydrone_tx_mutex);
}

// =============================================================================
// FX LIVE COMMANDS — enc/pot values → Master FX engine
// Ranges are intentionally conservative: audible first, no harsh clipping.
// Params resent periodically to survive UDP packet drops.
// =============================================================================
static bool s_fx_enc_last_active[3] = {};
static bool s_fx_enc_latch_valid[3] = {};
static uint32_t s_fx_enc_last_full_ms[3] = {};

void udp_send_fx_enc(int enc_id, uint8_t value, bool muted) {
    if (!udpStarted) return;
    if (enc_id < 0 || enc_id > 2) return;

    char buf[96];
    bool active = (!muted && value > 0);
    float norm = (float)value / 127.0f;
    uint32_t now = millis();
    bool activeChanged = !s_fx_enc_latch_valid[enc_id] ||
                         s_fx_enc_last_active[enc_id] != active;
    // Mix/depth follows the gesture at the UI throttle rate. The larger
    // parameter snapshot is only needed on activation and periodically as a
    // UDP-loss repair, not for every single arc increment.
    bool fullSend = active && (activeChanged ||
                    (uint32_t)(now - s_fx_enc_last_full_ms[enc_id]) >= 250);
    s_fx_enc_latch_valid[enc_id] = true;
    s_fx_enc_last_active[enc_id] = active;
    if (fullSend) s_fx_enc_last_full_ms[enc_id] = now;

    P4_LOG_PRINTF("[FX] enc%d val=%d muted=%d active=%d norm=%.2f full=%d\n",
                  enc_id, value, muted, active, norm, fullSend);

    switch (enc_id) {
        case 0: // Flanger — stronger modulation than chorus, with capped feedback/mix.
            if (activeChanged || fullSend) {
                snprintf(buf, sizeof(buf), "{\"cmd\":\"setFlangerActive\",\"value\":%d}", active ? 1 : 0);
                sendJson(buf);
            }
            if (active) {
                int rate_pct = clamp_int((int)(8.0f + norm * 44.0f + 0.5f), 8, 52);
                int depth_pct = clamp_int((int)(18.0f + norm * 62.0f + 0.5f), 18, 80);
                int feedback_pct = clamp_int((int)(8.0f + norm * 30.0f + 0.5f), 8, 38);
                int mix_pct = clamp_int((int)(10.0f + norm * 42.0f + 0.5f), 10, 52);
                if (fullSend) {
                    snprintf(buf, sizeof(buf), "{\"cmd\":\"setFlangerRate\",\"value\":%d}", rate_pct);
                    sendJson(buf);
                    snprintf(buf, sizeof(buf), "{\"cmd\":\"setFlangerDepth\",\"value\":%d}", depth_pct);
                    sendJson(buf);
                    snprintf(buf, sizeof(buf), "{\"cmd\":\"setFlangerFeedback\",\"value\":%d}", feedback_pct);
                    sendJson(buf);
                }
                snprintf(buf, sizeof(buf), "{\"cmd\":\"setFlangerMix\",\"value\":%d}", mix_pct);
                sendJson(buf);
            }
            break;
        case 1: // Delay — unmistakable echo effect
            mark_local_fx(FX_OWN_DELAY);
            if (activeChanged || fullSend) {
                snprintf(buf, sizeof(buf), "{\"cmd\":\"setDelayActive\",\"value\":%d}", active ? 1 : 0);
                sendJson(buf);
            }
            if (active) {
                int delay_ms = clamp_int((int)(90.0f + norm * 650.0f + 0.5f), 60, 900);
                int fb_pct = clamp_int((int)(14.0f + norm * 56.0f + 0.5f), 0, 85);
                int mix_pct = clamp_int((int)(8.0f + norm * 50.0f + 0.5f), 0, 100);
                if (fullSend) {
                    snprintf(buf, sizeof(buf), "{\"cmd\":\"setDelayTime\",\"value\":%d}", delay_ms);
                    sendJson(buf);
                    snprintf(buf, sizeof(buf), "{\"cmd\":\"setDelayFeedback\",\"value\":%d}", fb_pct);
                    sendJson(buf);
                    sendJson("{\"cmd\":\"setDelayStereo\",\"mode\":1}");
                }
                snprintf(buf, sizeof(buf), "{\"cmd\":\"setDelayMix\",\"value\":%d}", mix_pct);
                sendJson(buf);
            }
            break;
        case 2: // Reverb — unmistakable room/hall effect
            mark_local_fx(FX_OWN_REVERB);
            if (activeChanged || fullSend) {
                snprintf(buf, sizeof(buf), "{\"cmd\":\"setReverbActive\",\"value\":%d}", active ? 1 : 0);
                sendJson(buf);
            }
            if (active) {
                float feedback = 0.28f + norm * 0.48f;
                int lp_hz = clamp_int((int)(2200.0f + norm * 6800.0f + 0.5f), 1800, 12000);
                float early_mix = 0.08f + norm * 0.18f;
                float mix = 0.06f + norm * 0.42f;
                if (fullSend) {
                    snprintf(buf, sizeof(buf), "{\"cmd\":\"setReverbFeedback\",\"value\":%.3f}", feedback);
                    sendJson(buf);
                    snprintf(buf, sizeof(buf), "{\"cmd\":\"setReverbLpFreq\",\"value\":%d}", lp_hz);
                    sendJson(buf);
                    sendJson("{\"cmd\":\"setEarlyRefActive\",\"active\":1}");
                    snprintf(buf, sizeof(buf), "{\"cmd\":\"setEarlyRefMix\",\"mix\":%d}", clamp_int((int)(early_mix * 100.0f + 0.5f), 0, 100));
                    sendJson(buf);
                }
                snprintf(buf, sizeof(buf), "{\"cmd\":\"setReverbMix\",\"value\":%.3f}", mix);
                sendJson(buf);
            }
            break;
    }
}

void udp_send_fx_pot(int pot_id, uint8_t value, bool muted) {
    if (!udpStarted) return;
    char buf[96];
    float norm = (float)value / 127.0f;
    switch (pot_id) {
        case 0: {  // FOLD macro: mild wavefolder drive, reset to 1.0 when muted.
            if (muted) {
                sendJson("{\"cmd\":\"setWavefolderGain\",\"value\":1.0}");
                break;
            }
            float gain = 1.0f + norm * 2.25f;
            snprintf(buf, sizeof(buf), "{\"cmd\":\"setWavefolderGain\",\"value\":%.2f}", gain);
            sendJson(buf);
            break;
        }
        case 1: {  // CRUSH macro: bit depth and sample-rate reduction, no gain boost.
            if (muted || value == 0) {
                mark_local_fx(FX_OWN_BITCRUSH);
                mark_local_fx(FX_OWN_SAMPLE_RATE);
                mark_local_fx(FX_OWN_DISTORTION);
                sendJson("{\"cmd\":\"setBitCrush\",\"value\":16}");
                sendJson("{\"cmd\":\"setSampleRate\",\"value\":0}");
                sendJson("{\"cmd\":\"setDistortion\",\"value\":0.0}");
                break;
            }
            int bits = clamp_int((int)(16.0f - norm * 8.0f + 0.5f), 8, 16);
            int sr = clamp_int((int)(32000.0f - norm * 22000.0f + 0.5f), 9000, 32000);
            float dist = norm * 0.18f;
            mark_local_fx(FX_OWN_BITCRUSH);
            snprintf(buf, sizeof(buf), "{\"cmd\":\"setBitCrush\",\"value\":%d}", bits);
            sendJson(buf);
            mark_local_fx(FX_OWN_SAMPLE_RATE);
            snprintf(buf, sizeof(buf), "{\"cmd\":\"setSampleRate\",\"value\":%d}", sr);
            sendJson(buf);
            mark_local_fx(FX_OWN_DISTORTION);
            snprintf(buf, sizeof(buf), "{\"cmd\":\"setDistortion\",\"value\":%.3f}", dist);
            sendJson(buf);
            break;
        }
        case 2: {  // PHASER macro: audible sweep without the tremolo/limiter gain issues.
            mark_local_fx(FX_OWN_PHASER);
            bool active = !muted;
            snprintf(buf, sizeof(buf), "{\"cmd\":\"setPhaserActive\",\"value\":%d}", active ? 1 : 0);
            sendJson(buf);
            if (active) {
                int rate_pct = clamp_int((int)(10.0f + norm * 55.0f + 0.5f), 10, 65);
                int depth_pct = clamp_int((int)(28.0f + norm * 60.0f + 0.5f), 28, 88);
                int feedback_pct = clamp_int((int)(8.0f + norm * 34.0f + 0.5f), 8, 42);
                snprintf(buf, sizeof(buf), "{\"cmd\":\"setPhaserRate\",\"value\":%d}", rate_pct);
                sendJson(buf);
                snprintf(buf, sizeof(buf), "{\"cmd\":\"setPhaserDepth\",\"value\":%d}", depth_pct);
                sendJson(buf);
                snprintf(buf, sizeof(buf), "{\"cmd\":\"setPhaserFeedback\",\"value\":%d}", feedback_pct);
                sendJson(buf);
            }
            break;
        }
    }
}

void udp_reset_fx_latch(void) {
    for (int i = 0; i < 3; i++) {
        s_fx_enc_latch_valid[i] = false;
        s_fx_enc_last_active[i] = false;
        s_fx_enc_last_full_ms[i] = 0;
    }
}

// =============================================================================
// SYNC REQUEST — handshake + request initial data
// =============================================================================
void udp_request_master_sync(void) {
    P4_LOG_PRINTLN("[UDP] Requesting Master sync...");
    sendJson("{\"cmd\":\"hello\",\"device\":\"P4_DISPLAY\"}");

    if (!sessionCleanSent) {
        // One-time boot/session init only. Do NOT repeat this on every
        // reconnect/timeout because that force-clears mute/solo visuals.
        for (int track = 0; track < 16; track++) {
            p4.track_solo[track] = false;
            p4.track_muted[track] = false;
            uart_send_to_s3(MSG_TRACK, TRK_MUTE_BIT | (track & 0x0F), 0);
            uart_send_to_s3(MSG_TRACK, TRK_SOLO_BIT | (track & 0x0F), 0);
        }
        sessionCleanSent = true;
    }

    request_master_state_http();
    udp_send_get_pattern(p4.current_pattern);
    sendJson("{\"cmd\":\"getTrackVolumes\"}");

    lastSyncRequest = millis();
}

// =============================================================================
// PARSE INCOMING JSON
// =============================================================================
// Dispatches one already-parsed command object. Batch packets call this per
// element directly, so oversized elements are no longer truncated through an
// intermediate re-serialize buffer.
static void processJsonVariant(JsonVariant doc) {
    // Mark master alive
    lastMasterPacket = millis();
    if (!masterAlive) {
        masterAlive = true;
        p4.master_connected = true;
        P4_LOG_PRINTLN("[UDP] Master connected!");
    }

    const char* eventType = doc["type"] | "";
    if (strcmp(eventType, "trackVolumeSet") == 0) {
        int track = doc["track"] | -1;
        if (track >= 0 && track < 16) {
            unsigned long nowMs = millis();
            int vol = clamp_int(doc["volume"] | 100, 0, 150);
            if (!is_track_volume_owned_recent(track, nowMs)) {
                p4.track_volume[track] = vol;
            }
        }
        return;
    }
    if (strcmp(eventType, "songPattern") == 0) {
        int pat = clamp_int(doc["pattern"] | p4.current_pattern, 0, Config::MAX_PATTERNS - 1);
        p4.current_pattern = pat;
        p4.current_step = 0;
        s_step_raw = 0;
        uart_send_to_s3(MSG_SYSTEM, SYS_PATTERN, (uint8_t)pat);
        uart_send_to_s3(MSG_SYSTEM, SYS_STEP, 0);
        udp_send_get_pattern(pat);
        return;
    }
    if (strcmp(eventType, "masterFx") == 0) {
        apply_remote_master_fx_param(doc["param"] | "", doc["value"]);
        return;
    }
    if (strcmp(eventType, "raydrone") == 0) {
        apply_remote_raydrone_state(doc.as<JsonObjectConst>());
        return;
    }

    const char* cmd = doc["cmd"];
    if (!cmd) return;

    if (strcmp(cmd, "state_sync") == 0) {
        int pat = clamp_int(doc["pattern"] | p4.current_pattern, 0, Config::MAX_PATTERNS - 1);
        // Suppress a stale master pattern right after a local switch. The
        // periodic state_sync can still carry the previous pattern for a cycle
        // or two; applying it would snap the UI back to the old pattern.
        if (master_pattern_report_should_apply(pat)) {
            bool patternChanged = (pat != p4.current_pattern);
            p4.current_pattern = pat;
            uart_send_to_s3(MSG_SYSTEM, SYS_PATTERN, (uint8_t)pat);
            // Only re-fetch grid when pattern actually changed. Re-fetching on
            // every state_sync (every 2 s) caused full sequencer repaints and
            // visible flicker that fought against local mute/solo edits.
            if (patternChanged) {
                udp_send_get_pattern(pat);
            }
        }

        bool playing = doc["playing"] | p4.is_playing;
        p4.is_playing = playing;
        uart_send_to_s3(MSG_SYSTEM, SYS_PLAY_STATE, playing ? 1 : 0);

        if (doc["step"].is<int>()) {
            apply_master_step_sync(doc["step"].as<int>());
        }

        float bpm = clamp_float(doc["tempo"] | (float)(p4.bpm_int + p4.bpm_frac * 0.1f), 40.0f, 240.0f);
        p4.bpm_int = (int)bpm;
        p4.bpm_frac = (int)((bpm - p4.bpm_int) * 10.0f);
        uart_send_to_s3(MSG_SYSTEM, SYS_BPM_INT, (uint8_t)p4.bpm_int);
        uart_send_to_s3(MSG_SYSTEM, SYS_BPM_FRAC, (uint8_t)p4.bpm_frac);

        int masterVol = clamp_int(doc["masterVolume"] | p4.master_volume, 0, 150);
        p4.master_volume = masterVol;
        p4.seq_volume = clamp_int(doc["sequencerVolume"] | p4.seq_volume, 0, 150);
        p4.live_volume = clamp_int(doc["liveVolume"] | p4.live_volume, 0, 150);
        uart_send_to_s3(MSG_SYSTEM, SYS_VOLUME, (uint8_t)masterVol);
        uart_send_to_s3(MSG_SYSTEM, SYS_SEQ_VOL, (uint8_t)p4.seq_volume);
        uart_send_to_s3(MSG_SYSTEM, SYS_LIVE_VOL, (uint8_t)p4.live_volume);

        // P4 owns mute/solo visuals once the session is running. Applying the
        // periodic 2 s state_sync mix arrays here causes visible flicker when
        // they race with local touch edits or delayed master-side state. The
        // initial HTTP sync still seeds these values on connect/reconnect.

        JsonArray volumes = doc["trackVolumes"];
        if (volumes) {
            unsigned long nowMs = millis();
            int track = 0;
            for (JsonVariant value : volumes) {
                if (track >= 16) break;
                if (is_track_volume_owned_recent(track, nowMs)) {
                    track++;
                    continue;
                }
                int vol = clamp_int(value.as<int>(), 0, 150);
                if (p4.track_volume[track] != vol) {
                    p4.track_volume[track] = vol;
                    uart_send_to_s3(MSG_TRACK, TRK_VOLUME | (track & 0x0F), (uint8_t)vol);
                }
                track++;
            }
        }

        JsonArray trackSynthEngines = doc["trackSynthEngines"];
        if (trackSynthEngines) {
            int8_t engines[16];
            for (int t = 0; t < 16; t++) engines[t] = -1;
            int track = 0;
            for (JsonVariant value : trackSynthEngines) {
                if (track >= 16) break;
                engines[track] = (int8_t)clamp_int(value.as<int>(), -1, 7);  // 7 = GuitarT
                track++;
            }
            ui_pad_sound_sync_track_engines(engines);
        }

        JsonObject fx = doc["fx"];
        if (fx) {
            unsigned long nowMs = millis();
            if (!is_fx_owned_recent(FX_OWN_FILTER_TYPE, nowMs)) {
                p4.filter_type = clamp_int(fx["filterType"] | p4.filter_type, 0, 4);
                forward_fx_filter_type_to_s3(p4.filter_type);
            }
            if (!is_fx_owned_recent(FX_OWN_CUTOFF, nowMs)) {
                p4.cutoff_hz = clamp_int(fx["filterCutoff"] | p4.cutoff_hz, 20, 20000);
                forward_fx_cutoff_to_s3(p4.cutoff_hz);
            }
            if (!is_fx_owned_recent(FX_OWN_RESONANCE, nowMs)) {
                float resonance = clamp_float(fx["filterResonance"] | ((float)p4.resonance_x10 / 10.0f), 1.0f, 10.0f);
                p4.resonance_x10 = (int)(resonance * 10.0f);
                forward_fx_resonance_to_s3(p4.resonance_x10);
            }
            if (!is_fx_owned_recent(FX_OWN_DISTORTION, nowMs)) {
                float distortion = clamp_float(fx["distortion"] | ((float)p4.distortion_pct / 100.0f), 0.0f, 1.0f);
                p4.distortion_pct = (int)(distortion * 100.0f);
                forward_fx_distortion_to_s3(p4.distortion_pct);
            }
            if (!is_fx_owned_recent(FX_OWN_BITCRUSH, nowMs)) {
                p4.bitcrush_bits = clamp_int(fx["bitCrush"] | p4.bitcrush_bits, 4, 16);
                forward_fx_bitcrush_to_s3(p4.bitcrush_bits);
            }
            if (!is_fx_owned_recent(FX_OWN_SAMPLE_RATE, nowMs)) {
                int sr = fx["sampleRate"] | p4.sample_rate_hz;
                p4.sample_rate_hz = (sr <= 0) ? 0 : clamp_int(sr, 9000, 32000);
                forward_fx_samplerate_to_s3(p4.sample_rate_hz);
            }
            apply_remote_macro_fx_state(fx, "state_sync", lastRemoteStateSyncFxLogMs);
            apply_remote_raydrone_state(fx["raydrone"].as<JsonObjectConst>());
        }

        const char* kit = doc["kit"] | "";
        strncpy(p4.kit_name, kit, sizeof(p4.kit_name) - 1);
        p4.kit_name[sizeof(p4.kit_name) - 1] = '\0';
        memset(p4.sample_loaded, 0, sizeof(p4.sample_loaded));
        JsonArray samples = doc["samples"];
        if (samples) {
            for (JsonObject sample : samples) {
                int pad = sample["pad"] | -1;
                if (pad < 0 || pad >= 24) continue;
                p4.sample_loaded[pad] = sample["loaded"] | false;
                const char* name = sample["name"] | "";
                strncpy(p4.sample_name[pad], name, sizeof(p4.sample_name[pad]) - 1);
                p4.sample_name[pad][sizeof(p4.sample_name[pad]) - 1] = '\0';
            }
        }
        return;
    }

    // ----- Pattern sync -----
    if (strcmp(cmd, "pattern_sync") == 0) {
        int pat = clamp_int(doc["pattern"] | p4.current_pattern, 0, Config::MAX_PATTERNS - 1);
        bool requested_pattern = (pendingPatternRequest == pat);
        bool matches_current   = (pat == p4.current_pattern);
        bool active_hint       = doc["active"] | false;
        // A just-made local pattern switch wins over an active_hint broadcast
        // that still references the previous pattern (master lag), so we don't
        // snap back to the old pattern right after the user changes it.
        bool owned_stale = is_pattern_owned_recent(millis()) &&
                           !requested_pattern && (pat != p4.current_pattern);
        // Accept the payload if (a) it matches our pending request, (b) it
        // matches the pattern we are currently displaying, or (c) the master
        // marked the broadcast as the new active pattern. Otherwise ignore.
        if ((!requested_pattern && !matches_current && !active_hint) || owned_stale) {
            P4_LOG_PRINTF("[UDP][PAT] RX stale pattern_sync pat=%d cur=%d waiting=%d (ignored)\n",
                          pat, p4.current_pattern, pendingPatternRequest);
            return;
        }
        int raw_len = clamp_int(doc["stepCount"] | 16, 16, 64);
        bool raw_steps[16][64] = {};
        P4_LOG_PRINTF("[UDP][PAT] RX pattern_sync pat=%d raw=%d requested=%d\n",
                      pat, raw_len, requested_pattern ? 1 : 0);
        JsonArray rows = doc["rows"];
        JsonArray data = doc["data"];
        if (rows) {
            p4.current_pattern = pat;
            int track = 0;
            for (JsonVariant rowVal : rows) {
                if (track >= 16) break;
                const char* row = rowVal | "";
                for (int nib = 0; row[nib] && nib < 16; nib++) {
                    int value = hex_nibble_value(row[nib]);
                    if (value < 0) continue;
                    int baseStep = nib * 4;
                    for (int bit = 0; bit < 4; bit++) {
                        int step = baseStep + bit;
                        if (step >= 64) break;
                        raw_steps[track][step] = ((value >> bit) & 0x01) != 0;
                    }
                }
                track++;
            }
            apply_step_ownership_window(raw_steps, raw_len);
            ui_sequencer_load_external_pattern(raw_steps, raw_len);
        } else if (data) {
            bool nestedRows = false;
            if (!data.isNull() && data.size() > 0) {
                nestedRows = data[0].is<JsonArray>();
            }

            // Check if incoming payload is all-empty (supports 2D and flat).
            bool incomingEmpty = true;
            if (nestedRows) {
                for (JsonArray row : data) {
                    for (JsonVariant val : row) {
                        if (val.as<int>() != 0) { incomingEmpty = false; break; }
                    }
                    if (!incomingEmpty) break;
                }
            } else {
                for (JsonVariant val : data) {
                    if (val.as<int>() != 0) { incomingEmpty = false; break; }
                }
            }

            if (incomingEmpty) {
                P4_LOG_PRINTF("[UDP][PAT] RX empty payload pat=%d (applied)\n", pat);
            }

            p4.current_pattern = pat;

            if (nestedRows) {
                int track = 0;
                for (JsonArray row : data) {
                    if (track >= 16) break;
                    int step = 0;
                    for (JsonVariant val : row) {
                        if (step >= 64) break;
                        raw_steps[track][step] = (val.as<int>() != 0);
                        step++;
                    }
                    track++;
                }
            } else {
                // Flat payload fallback: track-major [t0s0..t0s63, t1s0..]
                int idx = 0;
                for (JsonVariant val : data) {
                    if (idx >= 1024) break;
                    int track = idx / 64;
                    int step  = idx % 64;
                    if (track >= 16) break;
                    raw_steps[track][step] = (val.as<int>() != 0);
                    idx++;
                }
            }
            apply_step_ownership_window(raw_steps, raw_len);
            ui_sequencer_load_external_pattern(raw_steps, raw_len);
        } else {
            p4.current_pattern = pat;
            ui_sequencer_sync_from_current_pattern();
        }
        pendingPatternRequest = -1;
        pendingPatternRetries = 0;
        pendingPatternLastTxMs = 0;
        // Forward pattern data to S3 so it can sync its sequencer + pad-sync
        uart_send_pattern_to_s3(pat, p4.steps);
    }
    else if (strcmp(cmd, "pattern_row") == 0) {
        int pat = clamp_int(doc["pattern"] | p4.current_pattern, 0, Config::MAX_PATTERNS - 1);
        int track = clamp_int(doc["track"] | -1, -1, 15);
        int raw_len = clamp_int(doc["stepCount"] | 16, 16, 64);
        const char* row = doc["row"] | "";
        bool requested_pattern = (pendingPatternRequest == pat);
        bool matches_current = (pat == p4.current_pattern);
        if (track < 0 || (!requested_pattern && !matches_current)) {
            return;
        }
        if (patternRowPattern != pat || patternRowStepCount != raw_len) {
            memset(patternRowGrid, 0, sizeof(patternRowGrid));
            patternRowMask = 0;
            patternRowPattern = pat;
            patternRowStepCount = raw_len;
        }
        decode_pattern_row_hex(row, patternRowGrid[track], raw_len);
        patternRowMask |= (uint16_t)(1U << track);
        p4.current_pattern = pat;
        if (patternRowMask == 0xFFFFU) {
            apply_step_ownership_window(patternRowGrid, raw_len);
            ui_sequencer_load_external_pattern(patternRowGrid, raw_len);
            pendingPatternRequest = -1;
            pendingPatternRetries = 0;
            pendingPatternLastTxMs = 0;
            patternRowMask = 0;
            uart_send_pattern_to_s3(pat, p4.steps);
        }
    }
    // ----- Play state -----
    else if (strcmp(cmd, "play_state") == 0) {
        p4.is_playing = doc["playing"] | false;
        p4.current_step = 0;
        s_step_raw = 0;
        uart_send_to_s3(MSG_SYSTEM, SYS_PLAY_STATE, p4.is_playing ? 1 : 0);
        uart_send_to_s3(MSG_SYSTEM, SYS_STEP, 0);
    }
    else if (strcmp(cmd, "start") == 0) {
        p4.is_playing = true;
        p4.current_step = 0;
        s_step_raw = 0;
        uart_send_to_s3(MSG_SYSTEM, SYS_PLAY_STATE, 1);
        uart_send_to_s3(MSG_SYSTEM, SYS_STEP, 0);
    }
    else if (strcmp(cmd, "stop") == 0) {
        p4.is_playing = false;
        p4.current_step = 0;
        s_step_raw = 0;
        uart_send_to_s3(MSG_SYSTEM, SYS_PLAY_STATE, 0);
        uart_send_to_s3(MSG_SYSTEM, SYS_STEP, 0);
    }
    // ----- Tempo -----
    else if (strcmp(cmd, "tempo_sync") == 0 || strcmp(cmd, "tempo") == 0) {
        float bpm = clamp_float(doc["value"] | 120.0f, 40.0f, 240.0f);
        p4.bpm_int = (int)bpm;
        p4.bpm_frac = (int)((bpm - p4.bpm_int) * 10);
    }
    // ----- Step update (visual tick from Master) -----
    else if (strcmp(cmd, "step_update") == 0 || strcmp(cmd, "step_sync") == 0) {
        if (doc["step"].is<int>()) {
            apply_master_step_sync(doc["step"].as<int>());
        }
    }
    // ----- Volume -----
    // Relay to S3 only on change: mute/solo needed TRACK_ECHO_GUARD_MS because
    // S3 reflections ping-ponged through the P4↔S3↔Master loop; the volume
    // path has the same topology, so at least skip no-op relays.
    else if (strcmp(cmd, "volume_sync") == 0 || strcmp(cmd, "master_volume_sync") == 0 ||
             strcmp(cmd, "volume_master_sync") == 0 || strcmp(cmd, "setVolume") == 0) {
        int v = clamp_int(doc["value"] | 75, 0, 150);
        if (v != p4.master_volume || v != p4.seq_volume || v != p4.live_volume) {
            p4.master_volume = v;
            p4.seq_volume = v;
            p4.live_volume = v;
            uart_send_to_s3(MSG_SYSTEM, SYS_VOLUME, (uint8_t)v);
            uart_send_to_s3(MSG_SYSTEM, SYS_SEQ_VOL, (uint8_t)v);
            uart_send_to_s3(MSG_SYSTEM, SYS_LIVE_VOL, (uint8_t)v);
        }
    }
    else if (strcmp(cmd, "volume_seq_sync") == 0 || strcmp(cmd, "setSequencerVolume") == 0) {
        int v = clamp_int(doc["value"] | 75, 0, 150);
        if (v != p4.seq_volume) {
            p4.seq_volume = v;
            uart_send_to_s3(MSG_SYSTEM, SYS_SEQ_VOL, (uint8_t)v);
        }
    }
    else if (strcmp(cmd, "volume_live_sync") == 0 || strcmp(cmd, "setLiveVolume") == 0) {
        int v = clamp_int(doc["value"] | 75, 0, 150);
        if (v != p4.live_volume) {
            p4.live_volume = v;
            uart_send_to_s3(MSG_SYSTEM, SYS_LIVE_VOL, (uint8_t)v);
        }
    }
    // ----- Track volumes -----
    else if (strcmp(cmd, "trackVolumes") == 0 || strcmp(cmd, "track_volumes") == 0 ||
             strcmp(cmd, "track_volume_sync") == 0 || strcmp(cmd, "getTrackVolumes") == 0) {
        JsonArray arr;
        if (doc["values"].is<JsonArray>())       arr = doc["values"].as<JsonArray>();
        else if (doc["volumes"].is<JsonArray>()) arr = doc["volumes"].as<JsonArray>();
        else if (doc["data"].is<JsonArray>())    arr = doc["data"].as<JsonArray>();
        if (arr) {
            unsigned long nowMs = millis();
            int i = 0;
            for (JsonVariant v : arr) {
                if (i >= 16) break;
                if (is_track_volume_owned_recent(i, nowMs)) {
                    i++;
                    continue;
                }
                p4.track_volume[i] = clamp_int(v.as<int>(), 0, 150);
                uart_send_to_s3(MSG_TRACK, TRK_VOLUME | (i & 0x0F), (uint8_t)p4.track_volume[i]);
                i++;
            }
        }
    }
    else if (strcmp(cmd, "trackVolume") == 0 || strcmp(cmd, "getTrackVolume") == 0) {
        int trk = doc["track"] | 0;
        int vol = clamp_int(doc["volume"] | doc["value"] | 75, 0, 150);
        if (trk >= 0 && trk < 16) {
            unsigned long nowMs = millis();
            if (is_track_volume_owned_recent(trk, nowMs)) {
                return;
            }
            p4.track_volume[trk] = vol;
            uart_send_to_s3(MSG_TRACK, TRK_VOLUME | (trk & 0x0F), (uint8_t)vol);
        }
    }
    // ----- FX -----
    else if (strcmp(cmd, "setFilter") == 0) {
        unsigned long nowMs = millis();
        if (!is_fx_owned_recent(FX_OWN_FILTER_TYPE, nowMs)) {
            p4.filter_type = clamp_int(doc["type"] | doc["value"] | 0, 0, 4);
            forward_fx_filter_type_to_s3(p4.filter_type);
        }
    }
    else if (strcmp(cmd, "setFilterCutoff") == 0) {
        unsigned long nowMs = millis();
        if (!is_fx_owned_recent(FX_OWN_CUTOFF, nowMs)) {
            p4.cutoff_hz = clamp_int(doc["value"] | 20000, 20, 20000);
            forward_fx_cutoff_to_s3(p4.cutoff_hz);
        }
    }
    else if (strcmp(cmd, "setFilterResonance") == 0) {
        unsigned long nowMs = millis();
        if (!is_fx_owned_recent(FX_OWN_RESONANCE, nowMs)) {
            float r = clamp_float(doc["value"] | 1.0f, 1.0f, 10.0f);
            p4.resonance_x10 = (int)(r * 10);
            forward_fx_resonance_to_s3(p4.resonance_x10);
        }
    }
    else if (strcmp(cmd, "setBitCrush") == 0) {
        unsigned long nowMs = millis();
        if (!is_fx_owned_recent(FX_OWN_BITCRUSH, nowMs)) {
            p4.bitcrush_bits = clamp_int(doc["value"] | 16, 4, 16);
            forward_fx_bitcrush_to_s3(p4.bitcrush_bits);
        }
    }
    else if (strcmp(cmd, "setDistortion") == 0) {
        unsigned long nowMs = millis();
        if (!is_fx_owned_recent(FX_OWN_DISTORTION, nowMs)) {
            float d = clamp_float(doc["value"] | 0.0f, 0.0f, 1.0f);
            p4.distortion_pct = (int)(d * 100.0f);
            forward_fx_distortion_to_s3(p4.distortion_pct);
        }
    }
    else if (strcmp(cmd, "setSampleRate") == 0) {
        unsigned long nowMs = millis();
        if (!is_fx_owned_recent(FX_OWN_SAMPLE_RATE, nowMs)) {
            // Same convention as the HTTP/state_sync/local paths: <=0 = bypass,
            // otherwise clamp to the crush range. (1000-44100 here turned a
            // master "bypass" into full crush at 9 kHz after S3 re-clamping.)
            int sr = doc["value"] | p4.sample_rate_hz;
            p4.sample_rate_hz = (sr <= 0) ? 0 : clamp_int(sr, 9000, 32000);
            forward_fx_samplerate_to_s3(p4.sample_rate_hz);
        }
    }
    // ----- Pattern selection -----
    else if (strcmp(cmd, "selectPattern") == 0 || strcmp(cmd, "pattern_select") == 0 ||
             strcmp(cmd, "current_pattern") == 0) {
        int idx = clamp_int(doc["index"] | doc["pattern"] | 0, 0, Config::MAX_PATTERNS - 1);
        if (idx != p4.current_pattern) {
            p4.current_pattern = idx;
            extern void ui_sequencer_sync_from_current_pattern(void);
            ui_sequencer_sync_from_current_pattern();
            udp_send_get_pattern(idx);
        } else if (!pattern_grid_has_data()) {
            // Refresh if we stayed on the same pattern but local grid is empty.
            udp_send_get_pattern(idx);
        }
    }
    // v2.9 — master-authoritative melody state: defer P4 piano UI apply
    // to the LVGL task. UDP/Core1 must never block on the LVGL mutex.
    else if (strcmp(cmd, "melody_sync") == 0) {
        uint8_t eng = (uint8_t)(doc["engine"] | 3);
        uint8_t oct = (uint8_t)(doc["octave"] | 4);
        bool    rec = doc["rec"] | false;
        uint8_t pad = (uint8_t)(doc["pad"]    | 0);
        p4_publish_pending_melody(eng, oct, rec, pad);
        // v2.9 — forward melody state to S3 so its melody screen stays in sync.
        // S3 has no WiFi; it receives these via UART and applies under LVGL lock.
        uart_send_to_s3(MSG_TOUCH_CMD, TCMD_MELODY_ENGINE, eng);
        uart_send_to_s3(MSG_TOUCH_CMD, TCMD_MELODY_OCTAVE, oct);
        uart_send_to_s3(MSG_TOUCH_CMD, TCMD_MELODY_REC,    rec ? 1 : 0);
        uart_send_to_s3(MSG_TOUCH_CMD, TCMD_MELODY_PAD,    pad);
    }
}

static void processJson(const char* json, int len) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json, len);
    if (err) {
        P4_LOG_PRINTF("[UDP] JSON parse error: %s\n", err.c_str());
        return;
    }
    processJsonVariant(doc.as<JsonVariant>());
}

// =============================================================================
// WiFi MANAGEMENT
// =============================================================================
static void startWiFi(void) {
    P4_LOG_PRINTLN("[WiFi] Connecting to RED808...");
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    lastWifiAttempt = millis();
}

static void onWiFiConnected(void) {
    wifiConnected = true;
    p4.wifi_connected = true;
    IPAddress local_ip = WiFi.localIP();
    P4_LOG_PRINTF("[WiFi] Connected! IP: %u.%u.%u.%u\n",
                  local_ip[0], local_ip[1], local_ip[2], local_ip[3]);

    // Start UDP
    if (udp.begin(UDP_PORT)) {
        if (s_udp_tx_queue) xQueueReset(s_udp_tx_queue);
        udpStarted = true;
        P4_LOG_PRINTF("[UDP] Listening on port %d\n", UDP_PORT);
        udp_request_master_sync();
        retry_pending_raydrone(millis(), true);
    } else {
        P4_LOG_PRINTLN("[UDP] Failed to start!");
    }
}

static void onWiFiDisconnected(void) {
    if (wifiConnected) {
        P4_LOG_PRINTLN("[WiFi] Disconnected!");
        wifiConnected = false;
        udpStarted = false;
        masterAlive = false;
        p4.wifi_connected = false;
        p4.master_connected = false;
        if (s_udp_tx_queue) xQueueReset(s_udp_tx_queue);
        // Clear FX latches so they are re-sent after reconnect (LP filter, etc.)
        extern void udp_reset_fx_latch(void);
        udp_reset_fx_latch();
    }
}

// =============================================================================
// INIT
// =============================================================================
void udp_handler_init(void) {
    P4_LOG_PRINTLN("[UDP] Init WiFi/UDP handler");
    s_udp_tx_stage_mutex = xSemaphoreCreateMutex();
    s_raydrone_tx_mutex = xSemaphoreCreateMutex();
    // Internal RAM keeps the queue usable while flash/NVS temporarily changes
    // external-memory cache state. Depth 24 still covers the largest command
    // burst (a complete deferred pattern) plus interactive input.
    s_udp_tx_queue = xQueueCreate(UDP_TX_QUEUE_DEPTH, sizeof(UdpTxPacket));
    if (!s_udp_tx_stage_mutex || !s_raydrone_tx_mutex || !s_udp_tx_queue) {
        P4_LOG_PRINTLN("[UDP] Failed to create TX queue");
    }
    startWiFi();

    // HTTP worker — Core 1 (with loop()), low priority. It spends nearly all
    // its time blocked on the socket or sleeping, so it doesn't starve loop().
    BaseType_t ok = xTaskCreatePinnedToCore(http_fetch_task, "httpfetch",
                                            8192, NULL, 1, NULL, 1);
    if (ok != pdPASS) {
        P4_LOG_PRINTLN("[UDP] Failed to create HTTP fetch task");
    }
}

// =============================================================================
// PUBLIC GETTERS
// =============================================================================
bool udp_wifi_connected(void)   { return wifiConnected; }
bool udp_master_connected(void) { return masterAlive; }

// =============================================================================
// LOCAL STEP CLOCK (authoritative)
// P4 is the sequencer authority. It advances step locally and mirrors every
// step to S3 over UART (SYS_STEP), independent of WiFi/Master transport.
// =============================================================================
static void run_local_step_clock(unsigned long now) {
    static uint32_t lastStepUs = 0;
    static bool prev_playing = false;
    if (udp_step_phase_valid_bridge) {
        lastStepUs = udp_step_phase_us_bridge;
        udp_step_phase_valid_bridge = false;
    }

    // While master step sync packets are fresh, do not locally advance the
    // sequencer clock. This avoids double-advancing SYS_STEP and early hits.
    if (masterAlive && (now - lastMasterStepSyncMs) < 1200) {
        prev_playing = p4.is_playing;
        return;
    }

    if (!p4.is_playing) {
        prev_playing = false;
        return;
    }
    // Edge false→true: snap the step clock to now to avoid catch-up bursts.
    if (!prev_playing) {
        lastStepUs = micros();
        prev_playing = true;
        uart_send_to_s3(MSG_SYSTEM, SYS_STEP, (uint8_t)p4.current_step);
    }
    float bpm = p4.bpm_int + p4.bpm_frac * 0.1f;
    if (bpm < 40) bpm = 120;
    uint32_t stepIntervalUs = (uint32_t)(60000000.0f / bpm / 4.0f); // 16ths
    uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastStepUs) >= stepIntervalUs) {
        do {
            lastStepUs += stepIntervalUs;
            p4.current_step = (p4.current_step + 1) % 16;
            // 192 = mcm(16,32,48,64): el módulo por longitud real que hace
            // la UI sigue en fase para cualquier largo de patrón.
            s_step_raw = (s_step_raw + 1) % 192;
        } while ((uint32_t)(nowUs - lastStepUs) >= stepIntervalUs);
        uart_send_to_s3(MSG_SYSTEM, SYS_STEP, (uint8_t)p4.current_step);
    }
}

// =============================================================================
// PROCESS — call from main loop
// =============================================================================
void udp_handler_process(void) {
    unsigned long now = millis();

    // --- WiFi state management ---
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifiConnected) {
            onWiFiConnected();
        }
    } else {
        if (wifiConnected) {
            onWiFiDisconnected();
        }
        // Retry WiFi
        if (now - lastWifiAttempt > WIFI_RETRY_MS) {
            startWiFi();
        }
        // Keep authoritative clock running even without WiFi.
        run_local_step_clock(now);
        return;
    }

    // --- Apply HTTP bodies fetched by the worker task (non-blocking) ---
    http_consume_results();

    // Retain the exact requestId across retries. The master can therefore
    // deduplicate a lost ACK without reapplying the user action.
    retry_pending_raydrone(now, false);

    // --- Drain all cross-task outgoing commands from one WiFiUDP owner ---
    // 24 entries cover a full deferred-pattern burst plus interactive input.
    udp_drain_tx_queue(UDP_TX_QUEUE_DEPTH);

    // --- Master timeout ---
    if (masterAlive && (now - lastMasterPacket > MASTER_TIMEOUT_MS)) {
        masterAlive = false;
        p4.master_connected = false;
        P4_LOG_PRINTLN("[UDP] Master timeout!");
    }

    // --- Re-request sync if no response ---
    if (udpStarted && !masterAlive) {
        if (now - lastSyncRequest > SYNC_REQUEST_MS) {
            udp_request_master_sync();
        }
    }

    // --- Periodic heartbeat: keep P4 in master's udpClients while alive ---
    // Without this, P4 is removed after UDP_CLIENT_TIMEOUT (30 s) of inactivity
    // and stops receiving melody_sync broadcasts.
    {
        static unsigned long lastPingMs = 0;
        if (udpStarted && masterAlive && (now - lastPingMs >= 5000)) {
            lastPingMs = now;
            sendCmd("ping");
        }
    }

    // --- Pattern request watchdog ---
    // If the selected pattern wasn't confirmed via pattern_sync, re-request it
    // at a controlled rate so UI/audio don't stay stuck on old steps.
    // IMPORTANTE: solo RE-PEDIR datos (get_pattern), NUNCA re-enviar
    // selectPattern desde aquí. El reselect convertía un watchdog de datos en
    // una orden al master: si la web cambiaba de patrón y el pattern_sync al
    // P4 se retrasaba/perdía, el P4 devolvía al master a su patrón pendiente
    // (guerra de selección; la web "hacía cosas raras" con el P4 encendido).
    // La selección solo debe salir del P4 cuando el usuario toca SU pantalla.
    if (udpStarted && masterAlive && pendingPatternRequest >= 0) {
        if (now - pendingPatternLastTxMs >= 250 && pendingPatternRetries < 12) {
            int target = pendingPatternRequest;
            udp_send_get_pattern(target);
            // UDP reply for the pending pattern is overdue (>=250 ms): fall back
            // to a rate-limited, short-timeout HTTP fetch so the grid still
            // loads if the UDP packet was dropped. The worker request is
            // coalesced and consumed on loop() by http_consume_results().
            if (now - lastPatternHttpMs >= PATTERN_HTTP_FALLBACK_MS) {
                lastPatternHttpMs = now;
                request_pattern_http(target);
            }
        } else if (pendingPatternRetries >= 12) {
            P4_LOG_PRINTF("[UDP][PAT] timeout waiting pattern_sync idx=%d\n",
                          pendingPatternRequest);
            pendingPatternRequest = -1;
            pendingPatternRetries = 0;
            pendingPatternLastTxMs = 0;
        }
    }

    // --- Receive UDP packets (bounded) ---
    // Cap packets per loop iteration to avoid starving ui_process_pad_queue()
    // under master-sync floods. Pad taps have priority over sync processing.
    int packetSize = udp.parsePacket();
    int pkt_budget = 4;
    while (packetSize > 0 && pkt_budget-- > 0) {
        int len = udp.read(rxBuf, sizeof(rxBuf) - 1);
        // Only the master may drive P4 state: the AP PSK is shared, so any
        // joined device could otherwise spoof tempo/mute/FX and be treated
        // as the master (lastMasterPacket/masterAlive).
        if (len > 0 && udp.remoteIP() == MASTER_IP) {
            rxBuf[len] = '\0';

            // Handle JSON array (batch) or single object
            if (rxBuf[0] == '[') {
                // Batch: parse array of commands and dispatch each element
                // directly (re-serializing into a fixed buffer truncated
                // elements bigger than the buffer and doubled the work).
                JsonDocument batchDoc;
                DeserializationError err = deserializeJson(batchDoc, rxBuf, len);
                if (!err && batchDoc.is<JsonArray>()) {
                    for (JsonVariant item : batchDoc.as<JsonArray>()) {
                        processJsonVariant(item);
                    }
                }
            } else {
                processJson(rxBuf, len);
            }
        }

        // Drain pad queue between packets — ensures taps go out even under
        // heavy inbound UDP traffic from the master.
        extern void ui_process_pad_queue(void);
        ui_process_pad_queue();

        packetSize = udp.parsePacket();
    }

    // --- Local step clock (authoritative) ---
    run_local_step_clock(now);

    // Commands generated while processing inbound packets should not wait for
    // the next Arduino loop iteration.
    udp_drain_tx_queue(UDP_TX_QUEUE_DEPTH);
}
