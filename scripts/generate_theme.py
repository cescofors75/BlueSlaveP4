#!/usr/bin/env python3
"""Generate the RED808 THEME — one electro-funk song told in 20 patterns.

Writes 20 Standard MIDI Files (format 0, 118 BPM, PPQ 96) into data/mid/ so
the P4 can load them from SPIFFS (MEM source in the SD screen) and push them
to the master's pattern slots. Drums live on GM channel 10 and follow the
NOTE_MAP table in src/mem_midi_loader.cpp; the two piano patterns carry a
real piano line on channel 1 (program 0) — the P4's PRO parser folds it onto
the grid as rhythm, and any standard MIDI player renders it as actual piano.

Song arc (16 steps per bar, piano patterns are 2 bars):
  01 INTRO  02 PULSE  03 FUNKA  04 GHOST  05 RIDE   06 BREAK  07 VERSE
  08 TOMS   09 LIFT   10 PIANO  11 KEYS   12 DROP   13 DROPB  14 PERC
  15 DUB    16 RISE   17 PEAK   18 FADE   19 FUNKB  20 OUTRO

Usage:  python scripts/generate_theme.py
"""
import os
import struct

PPQ      = 96
SIXT     = PPQ // 4          # one 16th note = 24 ticks
BPM      = 118.0
OUT_DIR  = os.path.join(os.path.dirname(__file__), "..", "data", "mid")

# GM drum notes chosen so each hit lands on the intended P4 track
# (see NOTE_MAP in src/mem_midi_loader.cpp).
BD, SD, CH, OH = 36, 38, 42, 46
CP, CB, RS, CL = 39, 56, 37, 75
MA, CY, RIDE   = 70, 49, 51          # CY 49 crash / 51 ride -> track 9
HT, LT, MT     = 48, 45, 47
MC, HC, LC     = 78, 63, 64          # MC only maps from cuica 78/79

ACC, NRM, GST = 116, 92, 52          # accent / normal / ghost velocities

def vlq(n):
    out = [n & 0x7F]
    n >>= 7
    while n:
        out.insert(0, 0x80 | (n & 0x7F))
        n >>= 7
    return bytes(out)

def write_smf(path, events, extra_setup=b""):
    """events: list of (tick, status, data1, data2). Sorted & delta-encoded."""
    tempo = int(60_000_000 / BPM)
    trk = bytearray()
    trk += b"\x00\xff\x51\x03" + struct.pack(">I", tempo)[1:]
    trk += extra_setup
    last = 0
    for tick, status, d1, d2 in sorted(events, key=lambda e: (e[0], e[1] & 0xF0 != 0x80)):
        trk += vlq(tick - last) + bytes([status, d1, d2])
        last = tick
    trk += b"\x00\xff\x2f\x00"
    with open(path, "wb") as f:
        f.write(b"MThd" + struct.pack(">IHHH", 6, 0, 1, PPQ))
        f.write(b"MTrk" + struct.pack(">I", len(trk)) + bytes(trk))

def drums(grid):
    """grid: {note: [step or (step, vel), ...]} -> event list (channel 10)."""
    ev = []
    for note, steps in grid.items():
        for s in steps:
            step, vel = s if isinstance(s, tuple) else (s, NRM)
            t = step * SIXT
            ev.append((t, 0x99, note, vel))            # ch10 note on
            ev.append((t + SIXT // 2, 0x89, note, 0))  # note off
    return ev

def piano(notes):
    """notes: [(step, midinote, len_steps, vel)] on channel 1 (melodic)."""
    ev = []
    for step, note, ln, vel in notes:
        t = step * SIXT
        ev.append((t, 0x90, note, vel))
        ev.append((t + ln * SIXT - 4, 0x80, note, 0))
    return ev

PIANO_SETUP = b"\x00\xc0\x00"        # ch1 program 0: Acoustic Grand

EIGHTS   = [0, 2, 4, 6, 8, 10, 12, 14]
OFFBEATS = [2, 6, 10, 14]
G        = lambda steps: [(s, GST) for s in steps]   # ghost layer

PATTERNS = {
    # ── Act I: arranque ─────────────────────────────────────────────────
    "T01INTRO": {BD: [(0, ACC), 8], MA: G(OFFBEATS), CH: [(4, GST), (12, GST)],
                 CY: [(0, GST)]},
    "T02PULSE": {BD: [(0, ACC), 6, 8], CP: [4, 12], CH: G([1, 3, 5, 7, 9, 11, 13, 15]),
                 MA: G(OFFBEATS)},
    "T03FUNKA": {BD: [(0, ACC), 6, 10], SD: [4, 12], CH: EIGHTS, OH: [7, 15],
                 CB: G([5, 13])},
    "T04GHOST": {BD: [(0, ACC), 6, 10], SD: [4, 12] + G([3, 7, 11, 15]),
                 CH: OFFBEATS, MA: G(EIGHTS)},
    "T05RIDE":  {BD: [(0, ACC), 6, 8, 10], SD: [4, 12], RIDE: EIGHTS,
                 OH: [7], CB: G([13])},
    "T06BREAK": {BD: [(0, ACC), 8], SD: [4, (12, ACC)], OH: [7],
                 HT: [12], MT: [13], LT: [14, (15, ACC)]},
    # ── Act II: el tema se asienta ──────────────────────────────────────
    "T07VERSE": {BD: [(0, ACC), 3, 8, 11], SD: [4, 12], CH: EIGHTS,
                 RS: G([6, 14])},
    "T08TOMS":  {BD: [(0, ACC), 8], SD: [4, 12], HT: [2], MT: [6],
                 LT: [10, 14], MA: G(EIGHTS)},
    "T09LIFT":  {BD: [0, 4, 8, 12], SD: [8, 10, 12, 13, (14, ACC), (15, ACC)],
                 CH: list(range(16)), CY: [(0, GST)]},
    # ── Act III: el piano — WAU ─────────────────────────────────────────
    "T10PIANO": {"len": 32,
                 "drums": {BD: [(0, ACC), 8, 16, 24], CH: G([4, 12, 20, 28]),
                           MA: G([2, 10, 18, 26])},
                 "piano": [(0, 64, 2, 100), (2, 67, 1, 88), (3, 69, 3, 104),
                           (6, 72, 2, 96), (8, 69, 2, 88), (11, 67, 1, 84),
                           (12, 64, 2, 92), (14, 62, 2, 80),
                           (16, 60, 2, 96), (18, 62, 1, 84), (19, 64, 3, 100),
                           (22, 67, 2, 88), (24, 69, 2, 104), (26, 72, 2, 96),
                           (28, 67, 2, 84), (30, 62, 2, 76)]},
    "T11KEYS":  {"len": 32,
                 "drums": {BD: [(0, ACC), 8, 16, 24], CP: [4, 12, 20, 28],
                           CH: G(OFFBEATS + [18, 22, 26, 30])},
                 # Am7 / Fmaj7 stabs answering the melody
                 "piano": [(s, n, 2, v)
                           for s, v in [(0, 100), (6, 88), (10, 92)]
                           for n in (57, 60, 64, 67)] +
                          [(s, n, 2, v)
                           for s, v in [(16, 100), (22, 88), (26, 92)]
                           for n in (53, 57, 60, 64)]},
    # ── Act IV: drop y fiesta ───────────────────────────────────────────
    "T12DROP":  {BD: [(0, ACC), 6, 8, 14], SD: [4, 12], CP: [(4, ACC), (12, ACC)],
                 OH: [2, 10], CH: [0, 4, 6, 8, 12, 14], CB: [7], CY: [(0, ACC)]},
    "T13DROPB": {BD: [(0, ACC), 6, 8, 14], SD: [4, 12], OH: [2, 10],
                 CH: [0, 4, 6, 8, 12, 14], RS: G([3, 11]), LT: [(15, ACC)]},
    "T14PERC":  {BD: [(0, ACC), 8], HC: [0, 3, 6, 10, 13], LC: [2, 7, 11, 14],
                 MC: [(9, GST)], CB: [5, 13], MA: G(EIGHTS)},
    "T15DUB":   {BD: [(0, ACC), 10], RS: [4, 12], OH: [7], CB: G([14]),
                 MA: G(OFFBEATS)},
    "T16RISE":  {BD: [0, 4, 8, 12], CP: [8, 12, 14, (15, ACC)],
                 HT: [12, 13], MT: [14, 15], CH: list(range(16)), CY: [(0, GST), 8]},
    "T17PEAK":  {BD: [(0, ACC), 4, 8, 12], SD: [4, 12], OH: OFFBEATS,
                 CB: [3, 11], CL: G([5, 13]), CY: [(0, ACC)]},
    # ── Act V: aterrizaje ───────────────────────────────────────────────
    "T18FADE":  {BD: [(0, ACC), 8], CH: OFFBEATS, SD: [12], MA: G(EIGHTS),
                 OH: [(7, GST)]},
    "T19FUNKB": {BD: [(0, ACC), 5, 10], SD: [4, 12], CL: [3, 10], CH: EIGHTS,
                 OH: [7, 15], CB: [(13, GST)]},
    "T20OUTRO": {"len": 16,
                 "drums": {BD: [(0, ACC)], CY: [(0, GST)], MA: G(OFFBEATS),
                           CH: G([4, 12]), RS: [(8, GST)]},
                 "piano": [(0, n, 8, 72) for n in (57, 60, 64)]},  # Am tail
}

def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    for name, spec in PATTERNS.items():
        if "drums" in spec:
            ev = drums(spec["drums"]) + piano(spec.get("piano", []))
            setup = PIANO_SETUP if spec.get("piano") else b""
        else:
            ev, setup = drums(spec), b""
        path = os.path.join(OUT_DIR, name + ".mid")
        write_smf(path, ev, setup)
        print(f"{name}.mid  ({os.path.getsize(path)} bytes)")

if __name__ == "__main__":
    main()
