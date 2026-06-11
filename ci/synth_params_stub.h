// =============================================================================
// CI COMPILE STUB — synth_params.h
// The real file lives in the XboxBLE superproject (shared/synth_params.h)
// OUTSIDE this repo. This stub only mirrors the TYPES and symbols the P4
// firmware compiles against so CI can build the firmware standalone.
// Param/preset DATA here is placeholder — never flash a binary built with it.
// The workflow copies this file to ../shared/synth_params.h before building.
// =============================================================================
#pragma once
#include <stdint.h>

// Engine wire codes (match master/S3 firmware)
#define SP_ENGINE_303    3
#define SP_ENGINE_WT     4
#define SP_ENGINE_SH101  5
#define SP_ENGINE_FM2OP  6
#define SP_ENGINE_PHYS   7

#define SP_ENGINE_COUNT  5

struct SynthParamValue {
    uint8_t param_id;
    float   value;
};

struct SynthPreset {
    const char*     name;
    uint8_t         count;
    SynthParamValue values[8];
};

struct SynthParamDef {
    uint8_t     param_id;
    const char* name;
    const char* unit;
    float       vmin;
    float       vmax;
    float       vdef;
    bool        step_int;
};

struct SynthEngineDef {
    uint8_t              engine;
    const char*          label;
    const char*          long_name;
    uint8_t              param_count;
    const SynthParamDef* params;
    uint8_t              preset_count;
    const SynthPreset*   presets;
};

static const SynthParamDef SP_PARAMS_303[] = {
    {0, "WAVE",   "",   0.f,     1.f,     0.f,    true },
    {1, "CUTOFF", "Hz", 100.f,   8000.f,  1200.f, false},
    {2, "RESO",   "",   0.f,     1.f,     0.5f,   false},
    {3, "ENVMOD", "",   0.f,     1.f,     0.6f,   false},
    {4, "DECAY",  "ms", 30.f,    2000.f,  300.f,  false},
    {5, "ACCENT", "",   0.f,     1.f,     0.7f,   false},
};

static const SynthParamDef SP_PARAMS_WT[] = {
    {0, "WAVE",   "",   0.f,     7.f,     0.f,    true },
    {1, "ATTACK", "ms", 1.f,     2000.f,  5.f,    false},
    {2, "RELEASE","ms", 10.f,    4000.f,  300.f,  false},
    {3, "VOLUME", "",   0.f,     1.f,     0.75f,  false},
    {4, "CUTOFF", "Hz", 100.f,   16000.f, 8000.f, false},
    {5, "RESO",   "",   0.f,     1.f,     0.2f,   false},
};

static const SynthParamDef SP_PARAMS_SH101[] = {
    {0, "SAW",    "",   0.f,     1.f,     1.f,    false},
    {1, "PULSE",  "",   0.f,     1.f,     0.f,    false},
    {2, "SUB",    "",   0.f,     1.f,     0.3f,   false},
    {3, "CUTOFF", "Hz", 100.f,   12000.f, 4000.f, false},
    {4, "RESO",   "",   0.f,     1.f,     0.3f,   false},
    {5, "DECAY",  "ms", 30.f,    3000.f,  400.f,  false},
};

static const SynthParamDef SP_PARAMS_FM2[] = {
    {0, "RATIO",  "",   0.5f,    8.f,     2.f,    false},
    {1, "INDEX",  "",   0.f,     10.f,    3.f,    false},
    {2, "ATTACK", "ms", 1.f,     2000.f,  5.f,    false},
    {3, "RELEASE","ms", 10.f,    4000.f,  250.f,  false},
};

static const SynthParamDef SP_PARAMS_PHYS[] = {
    {0, "DAMP",   "",   0.f,     1.f,     0.5f,   false},
    {1, "BRIGHT", "",   0.f,     1.f,     0.6f,   false},
    {2, "POS",    "",   0.f,     1.f,     0.2f,   false},
};

static const SynthPreset SP_PRESETS_GENERIC[] = {
    {"INIT", 1, {{0, 0.f}}},
    {"SOFT", 1, {{0, 0.f}}},
    {"HARD", 1, {{0, 1.f}}},
};

static const SynthEngineDef SP_ENGINES[SP_ENGINE_COUNT] = {
    { SP_ENGINE_303,   "303",  "ACID 303",      6, SP_PARAMS_303,   3, SP_PRESETS_GENERIC },
    { SP_ENGINE_WT,    "WT",   "WAVETABLE",     6, SP_PARAMS_WT,    3, SP_PRESETS_GENERIC },
    { SP_ENGINE_SH101, "SH1",  "SH-101",        6, SP_PARAMS_SH101, 3, SP_PRESETS_GENERIC },
    { SP_ENGINE_FM2OP, "FM2",  "FM 2-OP",       4, SP_PARAMS_FM2,   3, SP_PRESETS_GENERIC },
    { SP_ENGINE_PHYS,  "GTR",  "GUITAR PHYS",   3, SP_PARAMS_PHYS,  3, SP_PRESETS_GENERIC },
};
