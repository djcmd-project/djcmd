# djcmd — Terminal DJ Application

```
░▒▓███████▓▒░       ░▒▓█▓▒░░▒▓██████▓▒░░▒▓██████████████▓▒░░▒▓███████▓▒░
░▒▓█▓▒░░▒▓█▓▒░      ░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░
░▒▓█▓▒░░▒▓█▓▒░      ░▒▓█▓▒░▒▓█▓▒░      ░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░
░▒▓█▓▒░░▒▓█▓▒░      ░▒▓█▓▒░▒▓█▓▒░      ░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░
░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░      ░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░
░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░
░▒▓███████▓▒░ ░▒▓██████▓▒░ ░▒▓██████▓▒░░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░▒▓███████▓▒░
```

A full-featured, zero-dependency terminal DJ application for Linux.
Built for and optimised on the **IBM PowerPC 7447A (G4, 32-bit)** — PowerBook G4 Late 2005 running Arch Linux PPC in a fullscreen TTY.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ DECK A ▶ 01:32.410  ↺ LOOP 2.13s  BAR: 12 BEAT:3  DECK B  ■  00:00.000   │
│ some_track.flac  BPM:128.0  +0.0%  G   other.mp3  BPM:140.0  +0.5%       │
│ [DECK A — scrolling waveform — [~~~~2.13s~~~~] loop region ————————|——]   │
│ [DECK B — scrolling waveform ——————————————————————————————————————|——]   │
│  XFADE A [..|.:....] B  (45%)                                             │
├─ BROWSER [NAME] ▼  TAB=next panel ─────────────────────────────────────────┤
│  FLAC  128.0  some_track.flac                                               │
│  MP3   140.0  other.mp3                                                     │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Contents

1. [Features](#features)
2. [Requirements](#requirements)
3. [Build](#build)
4. [Quick Start](#quick-start)
5. [Views & Navigation](#views--navigation)
6. [Key Bindings](#key-bindings)
7. [MIDI Controller](#midi-controller)
8. [Supported Controllers](#supported-controllers)
9. [Effects Engine](#effects-engine)
10. [Performance Pads](#performance-pads)
11. [Loops](#loops)
12. [BPM & Beat Detection](#bpm--beat-detection)
13. [Waveform Display](#waveform-display)
14. [Themes](#themes)
15. [Options Menu](#options-menu)
16. [Waveform Cache (Sidecar Files)](#waveform-cache-sidecar-files)
17. [Playlist](#playlist)
18. [MusicBrainz Tag Lookup](#musicbrainz-tag-lookup)
19. [Customisation — djcmd_config.h](#customisation--djcmd_configh)
20. [Architecture](#architecture)
21. [PowerPC Notes](#powerpc-notes)
22. [Troubleshooting](#troubleshooting)
23. [Licence](#licence)

---

## Features

| Category | Details |
|---|---|
| **Decks** | 2 or 4 simultaneous decks (toggle live with `T`); NS7III 2 physical platters map to all 4 via layer switching |
| **Formats** | WAV · MP3 · FLAC — decoded at load time, adaptive sample rate |
| **Audio output** | **Dual-output (Main + Headphone)**; ALSA, auto-detected sample rate, 16-bit stereo |
| **CUE / PFL** | Per-deck headphone monitoring toggles (`5`–`8`) |
| **Phase Meter** | **Visual drift indicator** between Master and Active deck for manual beatmatching |
| **Crossfader** | Equal-power fade; horizontal visualizer in status bar |
| **Pitch/Speed** | Keyboard: free-step ±0.5%/±5%; MIDI fader: per-deck range ±8%/±25%/±50% |
| **Key lock** | WSOLA time-stretching — change tempo without affecting pitch |
| **3-Band EQ** | Biquad low / mid / high per deck |
| **Filter** | Per-deck low/high-pass filter (MIDI CC) |
| **Effects** | 10 effect types on 2 slots per deck + 1 master: Echo, Ping-Pong, Reverb, Flanger, Chorus, Phaser, Distortion, Bitcrusher, Gate, Widener + master compressor |
| **Jog wheels** | Motorised (NS7III absolute encoder) and standard relative encoder support |
| **Vinyl mode** | Motorised platters only — playhead detaches from motor until vinyl/rim is touched; toggle in ESC → SYNC tab |
| **Scratch** | Touch-sensor scratch mode vs nudge mode per deck |
| **Loops** | Manual (in/out), autoloop (1/2/4/8 bar beat-aligned), roll, waveform overlay |
| **Performance pads** | 8 pads per deck — HOTCUE / AUTOROLL / ROLL modes with LED colour feedback |
| **CUE transport** | Standard CDJ/Serato CUE button: set, jump, hold-to-preview |
| **Hot cues** | 8 cue points per deck — imported from Mixxx, saved in sidecar |
| **Sync lock** | BPM + phase sync: slave decks phase-lock to a master |
| **Gang mode** | Pitch/stop/play to multiple decks simultaneously |
| **Library** | File browser · Playlist · Mixxx library — always visible in 2-deck mode |
| **Auto-gain** | RMS normalisation to a configurable dBFS target |
| **Waveform** | Scrolling 3-band waveform with loop region overlay, beat ruler, cue markers |
| **Waveform cache** | `.djcmd` sidecar files — instant load on repeat plays |
| **BPM** | Exact values from Mixxx SQLite DB; onset-detection fallback |
| **MIDI learn** | In-app bind/unbind/save — no config file editing needed |
| **Controller maps** | Drop `.map` files into `~/.config/djcmd/maps/` — auto-installed on connect |
| **Themes** | 10 built-in themes; add custom themes in `djcmd_config.h` |
| **TTY compatible** | Runs on real TTYs — uses ACS block chars instead of Unicode |
| **CPU arch** | Tuned for PowerPC 7447A G4 (`-mcpu=7450`); runs on any Linux |

---

## Requirements

### Build dependencies

```
cc                C99 compiler (any; Makefile uses cc — powerpc-linux-gnu-gcc for cross-compile)
libasound2-dev    ALSA (audio + MIDI)
libncurses-dev    terminal UI
libsqlite3-dev    Mixxx library import
libm              (glibc — always present)
libpthread        (glibc — always present)
curl              downloads single-header libs at build time
```

### Runtime

```
Linux kernel ≥ 2.6 with ALSA
ALSA-compatible audio device (one for Main, one optional for Headphones)
Optional: MIDI DJ controller (raw ALSA MIDI)
Optional: curl (for MusicBrainz tag lookup)
Optional: Mixxx with analysed tracks (~/.mixxx/mixxxdb.sqlite)
```

### Arch Linux (PPC or x86)

```bash
sudo pacman -S base-devel alsa-lib ncurses sqlite curl
```

### Debian / Ubuntu

```bash
sudo apt install gcc libasound2-dev libncurses-dev libsqlite3-dev curl
```

---

## Build

```bash
make deps          # fetch minimp3.h and dr_flac.h (once)
make powerpc       # build for G4 PowerPC (default)
make x86_64        # build for x86_64
sudo make install  # optional: install to /usr/local/bin/djcmd
make run           # build and launch immediately
make clean         # remove binary and objects
make check-deps    # verify build dependencies
```

---

## Quick Start

```bash
./djcmd                    # uses ~/.config/djcmd/library if set, else cwd
./djcmd /music             # open browser at a specific path
```

1. Split view opens — decks on top, file browser on the bottom.
2. Navigate with `j`/`k`; enter directories with `ENTER`; go up with `BACKSPACE`.
3. Press `ENTER` or `!` to load the selected track into Deck A.
4. Press `SPACE` to play.
5. Press `5` to route Deck A to Headphones (CUE/PFL).
6. Press `TAB` to cycle the bottom panel: Browser → Playlist → Library.
7. Press `2` to switch to Deck B, load another track, press `SPACE`.
8. Use `<`/`>` to crossfade between decks (watch the XF bar in the status line).
9. Press `ESC` to open the options menu.

---

## Views & Navigation

### Split View (default)

- Bottom panel always visible in 2-deck mode
- **`TAB`** cycles: Browser → Playlist → Library
- Divider shows active panel and `TAB=next panel` hint
- In **4-deck mode**, `TAB` toggles split view on/off (screen space is tight)

### Help View (`?`)

Inline key reference. Scroll with `j`/`k` or `PgDn`/`PgUp`. Press `?` to close.

### Options Overlay (`ESC`)

Tabs: **INFO · AUDIO · DISPLAY · SYNC · THEME · MIDI · OUT · FX**.
- **MIDI Tab** is now categorized (Mixer, Transport, Loops, etc.) for easier mapping.
- Navigate with `←`/`→`, scroll list with `j`/`k`, adjust with `-`/`+`.

---

## Key Bindings

### Deck Selection & CUE

| Key | Action |
|---|---|
| `1` `2` `3` `4` | Select Deck A / B / C / D |
| `5` `6` `7` `8` | Toggle Headphone Cue (PFL) for Deck A / B / C / D |
| `T` | Toggle 2-deck / 4-deck mode |

### Playback

| Key | Action |
|---|---|
| `SPACE` | Play / Pause |
| `s` | Stop |
| `r` | Restart from beginning |
| `l` | Toggle loop |
| `b` | Re-analyse waveform and BPM |
| `B` | Enter BPM manually |
| `H` | Cycle BPM display: ×1 → ×2 → ÷2 |
| `K` | Toggle key lock (WSOLA time-stretch) |

### Pitch / Speed

| Key | Action |
|---|---|
| `e` / `d` | Pitch +0.5% / −0.5% |
| `E` / `D` | Pitch +5% / −5% |
| `0` | Reset pitch to 0% |
| `V` | Cycle pitch fader range: ±8% → ±25% → ±50% |

### Beat Seek & Nudge

| Key | Action |
|---|---|
| `←` / `→` | Seek ±1 beat |
| `Shift+←` / `Shift+→` | Seek ±8 beats |
| `]` / `[` | Pitch bend forward / back (decaying) |
| `}` / `{` | Shift beat grid ±1 beat |
| `)` / `(` | Shift beat grid ±¼ beat (fine) |
| `Z` | Reset beat grid to frame 0 |

### Cue Points

| Key | Action |
|---|---|
| `F5`–`F8` | Set cue 1–4 |
| `F9`–`F12` | Jump to cue 1–4 |

### Volume, EQ, Crossfader

| Key | Action |
|---|---|
| `+` / `-` | Deck volume up / down |
| `A` | Toggle auto-gain |
| `m` / `n` | Master volume up / down |
| `q` / `a` | EQ Low +/− |
| `w` / `x` | EQ Mid +/− |
| `t` / `g` | EQ High +/− |
| `<` / `>` | Crossfader left / right (watch XF bar in status) |

### Sync & Gang

| Key | Action |
|---|---|
| `M` | Set active deck as sync master |
| `y` | Toggle sync slave |
| `G` | Toggle gang mode |
| `F1`–`F4` | Toggle deck A–D in/out of gang |

---

## MIDI Controller

### MIDI Learn

1. Press `ESC` → navigate to the **MIDI** tab
2. Use `j`/`k` to navigate the **Categorized** action list, press `L` to learn
3. Move the knob or press the button on your controller
4. Press `S` to save

---

## Visual Aids

### Phase Meter
When a deck is not the Sync Master, a Phase Meter appears in the deck header:
`[ <<<<:     ]` -> Slave is lagging (needs forward nudge)
`[     :>>>> ]` -> Slave is rushing (needs backward nudge)
`[     |     ]` -> Perfect phase alignment

### Crossfader Visualizer
A real-time fader position bar in the status line:
`XF:[A..|.:....B]`

---

## Licence

GNU GPL v3 — worthwhile for protecting your work while keeping it open for the community.

```
djcmd is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

djcmd is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
```
