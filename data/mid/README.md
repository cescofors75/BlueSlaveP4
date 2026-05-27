# MEM MIDIs — P4 internal flash (SPIFFS)

Place Standard MIDI Files here to be packed into the P4's SPIFFS partition.

## Upload

VS Code → Command Palette → `Tasks: Run Task` → **PlatformIO: Upload Filesystem P4**

or from terminal:

```powershell
Set-Location BlueSlaveP4
C:\Users\cesco\.platformio\penv\Scripts\platformio.exe run --target uploadfs --environment esp32p4 --upload-port COM21
```

## Rules

- Files must end in `.mid` or `.MID`.
- Keep names short (≤ 8 chars before `.mid`) — only the basename is shown in UI.
- Standard MIDI File format 0 or 1; drums on channel 10 are preferred (auto-detected).
- Multi-bar patterns (up to 4 bars of 4/4) are folded onto a single 16-step grid.

## Inspect before uploading

```powershell
python scripts/inspect_mid.py BlueSlaveP4/data/mid
```

Shows the exact 16×16 grid P4 will display.
