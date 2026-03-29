# djcmd -- Terminal DJ Application

```
░▒▓███████▓▒░       ░▒▓█▓▒░░▒▓██████▓▒░░▒▓██████████████▓▒░░▒▓███████▓▒░
░▒▓█▓▒░░▒▓█▓▒░      ░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░
░▒▓█▓▒░░▒▓█▓▒░      ░▒▓█▓▒░▒▓█▓▒░      ░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░
░▒▓█▓▒░░▒▓█▓▒░      ░▒▓█▓▒░▒▓█▓▒░      ░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░
░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░      ░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░
░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░
░▒▓███████▓▒░ ░▒▓██████▓▒░ ░▒▓██████▓▒░░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░▒▓███████▓▒░
```
(conceptualized by daedalao)

A full-featured, minimal-dependency terminal DJ application for Linux.
Built for and optimised on the **IBM PowerPC 7447A (G4, 32-bit)** -- PowerBook 5,8 G4 Late 2005 running **Arch Linux POWER** in a fullscreen TTY.

Inspired by **cmus**, **btop** and **MIXXX**.

> **Note:** **Big Endian (PowerPC) is a first-class citizen.** Unlike most modern projects, djcmd is developed and tested primarily on PowerPC hardware to ensure correct byte-order handling and performance on legacy G4/G5 systems.

> **Note:** Windows support is currently **experimental and untested**. The primary development target is Linux. See the `platform/windows` branch for the Windows-specific build files.

Entirely vibe coded. Good luck.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ DECK A ▶ 01:32.410  [ 8A ]  -  DECK B  ■  00:00.000  [ 5A ]  -              │
│ some_heavy_techno.flac         other_track.mp3                              │
│ [▒▒▒▒▒▓▓▓▓█████████|█████████▓▓▓▓▒▒▒▒▒]                                     │
│ [▒▒▒▒▒▓▓▓▓█████|████████████▓▓▓▓▒▒▒▒▒]                                     │
│           XFADE: A [....|....] B (50%)                  [  #  |  #  ] L     │
├─ BROWSER [NAME] ▼  TAB=next panel                       [ #   | #   ] F ──  ┤
│  FLAC  128.0  8A    some_heavy_techno.flac                                  │
│  MP3   140.0  5A    other_track.mp3                                         │
└─────────────────────────────────────────────────────────────────── AUTO:ON ──┘
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
18. [Crates (Quick Jump)](#crates-quick-jump)
19. [MusicBrainz Tag Lookup](#musicbrainz-tag-lookup)
20. [Customisation -- djcmd_config.h](#customisation--djcmd_configh)
21. [Architecture](#architecture)
22. [PowerPC Notes](#powerpc-notes)
23. [Troubleshooting](#troubleshooting)
24. [Licence](#licence)

---

## Features

| Category | Details |
|---|---|
| **Decks** | 2 or 4 simultaneous decks (toggle live with `T`); height-aware 4-deck view shows warning if terminal is too short |
| **Formats** | WAV · MP3 · FLAC -- decoded at load time, adaptive sample rate |
| **Audio output** | **Dual-output (Main + Headphone)**; ALSA, auto-detected sample rate, 16-bit stereo |
| **Instant Doubles** | Loading a track already playing elsewhere **clones playback state** (pos, pitch, loops) exactly |
| **CUE / PFL** | Per-deck headphone monitoring toggles (`5`-`8`) |
| **VU Meters** | **Master VU** bar + **Lightweight Per-Deck LEDs** in header (`-`/`#`/`!` for levels/clipping) |
| **Musical Key** | **Camelot/Note keys** imported from Mixxx and displayed in deck headers |
| **Phase Meter** | **Visual beat-alignment blocks** at the bottom (L=Leader, F=Follower) for manual beatmatching |
| **Crossfader** | Compact equal-power fade; visualizer in the bottom cockpit area |
| **Pitch/Speed** | Keyboard: free-step ±0.5%/±5%; MIDI fader: per-deck range ±8%/±25%/±50% |
| **Key lock** | WSOLA time-stretching -- change tempo without affecting pitch |
| **3-Band EQ** | Biquad low / mid / high per deck |
| **Filter** | Per-deck low/high-pass filter (MIDI CC) |
| **Effects** | 10 effect types on 2 slots per deck + 1 master; **Persistent settings** when toggled on/off |
| **Jog wheels** | Motorised (NS7III absolute encoder) and standard relative encoder support |
| **Vinyl mode** | Motorised platters only -- playhead detaches from motor until vinyl/rim is touched; toggle in ESC → SYNC tab |
| **Scratch** | Touch-sensor scratch mode vs nudge mode per deck |
| **Loops** | Manual (in/out), autoloop (1/2/4/8 bar beat-aligned), roll, waveform overlay |
| **Performance pads** | 8 pads per deck -- HOTCUE / AUTOROLL / ROLL modes with LED colour feedback |
| **CUE transport** | Standard CDJ/Serato CUE button: set, jump, hold-to-preview |
| **Hot cues** | 8 cue points per deck -- imported from Mixxx, saved in sidecar |
| **Sync lock** | BPM + phase sync: follower decks phase-lock to a leader |
| **Gang mode** | Pitch/stop/play to multiple decks simultaneously |
| **Library** | File browser · Playlist · Mixxx library -- **Fixed blank pages** in short terminal windows |
| **Auto-gain** | RMS normalisation to a configurable dBFS target |
| **Waveform** | Scrolling 3-band waveform with loop region overlay, beat ruler, cue markers |
| **Waveform cache** | `.djcmd` sidecar files -- instant load on repeat plays |
| **BPM** | Exact values from Mixxx SQLite DB; onset-detection fallback |
| **MIDI learn** | In-app bind/unbind/save -- no config file editing needed |
| **Controller maps** | Drop `.map` files into `~/.config/djcmd/maps/` -- auto-installed on connect |
| **Themes** | 10 built-in themes; add custom themes in `djcmd_config.h` |
| **TTY compatible** | Runs on real TTYs -- uses ACS block chars instead of Unicode |
| **CPU arch** | Tuned for PowerPC 7447A G4 (`-mcpu=7450`); runs on any Linux |

---

## Requirements

### Build dependencies

```
cc                C99 compiler (any; Makefile uses cc -- powerpc-linux-gnu-gcc for cross-compile)
libasound2-dev    ALSA (audio + MIDI)
libncurses-dev    terminal UI
libsqlite3-dev    Mixxx library import
libm              (glibc -- always present)
libpthread        (glibc -- always present)
curl              downloads single-header libs at build time
```

### Runtime

#### Linux
```
Linux kernel ≥ 2.6 with ALSA
ALSA-compatible audio device (one for Main, one optional for Headphones)
Optional: MIDI DJ controller (raw ALSA MIDI)
Optional: curl (for MusicBrainz tag lookup)
Optional: Mixxx with analysed tracks (~/.mixxx/mixxxdb.sqlite)
```

#### Windows (Experimental / Untested)
- Windows 10/11
- PortAudio
- PDCurses
- WinMM
- pthreads-win32
- SQLite3
- MinGW-w64 (for building)

### Arch Linux POWER (or x86)

```bash
sudo pacman -S base-devel alsa-lib ncurses sqlite curl
```

### Debian / Ubuntu

```bash
sudo apt install gcc libasound2-dev libncurses-dev libsqlite3-dev curl
```

---

## Build

### Linux

```bash
make deps          # fetch minimp3.h and dr_flac.h (once)
make powerpc       # build for G4 (Arch Linux POWER) default
make x86_64        # build for x86_64
sudo make install  # optional: install to /usr/local/bin/djcmd
make run           # build and launch immediately
make clean         # remove binary and objects
make check-deps    # verify build dependencies
```

### Windows (Experimental)

Switch to the `platform/windows` branch to find the Windows-specific build files:

```bash
git checkout platform/windows
# Build instructions are in the Windows/ directory on that branch.
```

---

## Quick Start

```bash
./djcmd                    # uses ~/.config/djcmd/library if set, else cwd
./djcmd /music             # open browser at a specific path
```

1. Split view opens -- decks on top, file browser on the bottom.
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
- **`C`** opens the **Crate Jump** input for quick directory switching (see [Crates](#crates-quick-jump))
- **Height Check:** If the terminal is too short for 4-deck view or library panels, a warning is shown.

### Help View (`?`)

Inline key reference. Scroll with `j`/`k` or `PgDn`/`PgUp`. Press `?` or `ESC` to close.

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
| `Shift+↑` / `Shift+↓` | Seek ±16 beats |
| `]` / `[` | Pitch bend forward / back (decaying) |
| `}` / `{` | Shift beat grid ±1 beat |
| `)` / `(` | Shift beat grid ±¼ beat (fine) |
| `Z` | Reset beat grid to frame 0 |

### Cue Points

| Key | Action |
|---|---|
| `F5`-`F8` | Set cue 1-4 |
| `F9`-`F12` | Jump to cue 1-4 |

### Volume, EQ, Crossfader

| Key | Action |
|---|---|
| `+` / `-` | Deck volume up / down |
| `A` | Toggle auto-gain |
| `C` | Open Crate Jump (quick directory switching) |
| `m` / `n` | Master volume up / down |
| `q` / `a` | EQ Low +/− |
| `w` / `x` | EQ Mid +/− |
| `t` / `g` | EQ High +/− |
| `<` / `>` | Crossfader left / right (watch XF bar in status) |

### Sync & Gang

| Key | Action |
|---|---|
| `M` | Set active deck as sync leader |
| `y` | Toggle sync follower |
| `G` | Toggle gang mode |
| `F1`-`F4` | Toggle deck A-D in/out of gang |

### System

| Key | Action |
|---|---|
| `TAB` | Cycle panel: Browser → Playlist → Library |
| `Ctrl+A` | **Toggle Library Autoplay** (Status shown in footer) |
| `?` | Toggle help view |
| `Q` | Quit (requires confirmation) |

---

## Performance Features

### Instant Doubles
If you load a track that is already playing on another deck, djcmd will **instantly clone** the position, pitch, and loop state. This allows for seamless "hand-offs" between decks or creative doubling for scratching.

### Musical Key Display
If tracks are imported from a Mixxx database, the **Musical Key** (Camelot or Note format) is extracted and displayed in the deck header. This facilitates harmonic mixing.

### Lightweight VU Meters
Each deck header features a single-character LED-style VU meter that changes color based on signal level:
- `-` (Dim Green): Low signal
- `#` (Bright Green): Nominal signal
- `!` (Blinking Red): Clipping (> 0.95 amplitude)

---

## MIDI Controller

### MIDI Learn

1. Press `ESC` → navigate to the **MIDI** tab
2. Use `j`/`k` to navigate the **Categorized** action list, press `L` to learn
3. Move the knob or press the button on your controller
4. Press `S` to save

---

## Supported Controllers

### Numark NS7 III

Fully built-in. Map hardcoded in `ns7iii_map.h`, auto-written to `~/.config/djcmd/ns7_iii.map` on first connect. Delete that file to regenerate.

**Hardware:** 2 physical platters, 4 virtual decks via layer switching.

#### Layer Switching

The NS7III has a left platter and a right platter. djcmd maps each to one of four software decks:

| Physical side | Default (layer 1) | Layer 2 | How to switch |
|---|---|---|---|
| Left platter  | Deck A | Deck C | Press deck selector **[1]** (back to A) or **[3]** (to C) |
| Right platter | Deck B | Deck D | Press deck selector **[2]** (back to B) or **[4]** (to D) |

Playback on the previous deck continues when you switch layers -- only new jog/button input routes to the newly selected deck. Enable 4-deck mode first (`T` key or `ESC → DISPLAY`) before switching to Deck C or D.

The motor stays on the physical platter channel throughout; djcmd transfers motor state between decks automatically at layer-switch time.

#### Crossed displays (Work in Progress)

**Note: Display support is currently non-functional and in-process.**

The two center displays are wired in a crossed configuration:
- **Display Right** ALSA card → intended for **Deck A** (left platter)
- **Display Left** ALSA card → intended for **Deck B** (right platter)

This is handled automatically in the code, but pending hardware-specific implementation for the NS7III's specific display protocol.

Includes: motorised jog wheels, all 4 decks, 8 performance pads per side, HOTCUE/AUTOROLL/ROLL modes, FX buttons/knobs (2 slots + master), EQ, filter, volume, pitch, crossfader, loop controls, SHIFT, CUE transport, PITCH CENTER, full LED feedback (play blink, loop, cues, FX, pads, deck selectors).

### Numark Mixtrack 3 / Mixtrack Pro 3

Map provided as `mixtrack_3.map` -- identical MIDI layout, Pro 3 adds a soundcard.

Install: `cp mixtrack_3.map ~/.config/djcmd/maps/`

Includes: play, CUE, SHIFT, sync, EQ, filter, volume, pitch fader, pitch bend buttons, jog wheel + touch sensor, touch strip → FX wet, FX buttons 1-3, beat knob, manual loop (in/out/toggle/half), 4 hot cue pads (bottom row), 4 autoloop pads (top row), all LEDs.

Not yet mapped: sampler pads, TAP BPM, headphone routing.

### Any Other Controller

Drop `<device_name>.map` into `~/.config/djcmd/maps/`. Device name shown in **MIDI tab**. Use MIDI Learn to build a map from scratch.

---

## Effects Engine

10 effect types on 2 slots per deck + 1 master:

| Effect | Param 1 | Param 2 | Param 3 |
|---|---|---|---|
| Echo | Time | Feedback | -- |
| Ping-Pong | Time | Feedback | -- |
| Reverb | Room size | Damping | Width |
| Flanger | Delay | Depth | Rate |
| Chorus | Delay | Depth | Rate |
| Phaser | Rate | Depth | -- |
| Distortion | Drive | Tone | -- |
| Bitcrusher | Bit depth | Sample rate reduction | -- |
| Gate | Threshold | Attack | Release |
| Widener | Width | -- | -- |

Plus master **compressor/limiter** on the mix bus.

**Controls:** `fx_btn_1/2` cycle effect types, `SHIFT+fx_btn` = off, `fx_knob_1/2/3` = params, `fx_wet` = dry/wet. Defaults: slot 0 = Echo, slot 1 = Reverb (both wet=0, silent until activated).

Adjustable live in **ESC → FX tab**.

---

## Performance Pads

8 pads per deck, three modes via the PAD MODE button:

**HOTCUE** -- pads 1-8 set/jump to hot cues. SHIFT+pad deletes. Cues saved in `.djcmd` sidecar, imported from Mixxx. Coloured LEDs per pad.

**AUTOROLL** -- pads 1-4 engage beat-aligned loops (1/2/4/8 bars). Press again to toggle off. PARAM L/R halve/double. Pad LEDs blink while active.

**ROLL** -- hold a pad for a temporary loop, release returns to the roll start position.

---

## Loops

1. **Loop In** -- set start point
2. **Loop Out** -- set end + engage (press again to exit)
3. **Loop toggle** -- arm/disarm without resetting points
4. **Loop ½ / ×2** -- halve or double length

The waveform shows the loop region as `[~~~~2.13s~~~~]` on the beat ruler with `[`/`]` boundary markers on the waveform body, and `↺ LOOP 2.13s` in the panel header.

---

## BPM & Beat Detection

**Mixxx database (primary):** reads BPM, beat grid, and hot cues from `~/.mixxx/mixxxdb.sqlite`. Exact beat grids, no analysis delay. Add and analyse tracks in Mixxx first.

**Onset detection (fallback):** spectral-flux onset detection for tracks not in Mixxx. BPM defaults to 120.0. Use `]`/`[` to nudge the grid, or `b` to re-trigger analysis.

---

## Waveform Display

Scrolling waveform centred on the playhead. **Red** = kick (low band), **green** = snare/melody (mid), **blue/white** = hats (high). 256-colour terminals get full RGB cube blending; 8-colour TTYs get three-colour mode.

Overlays: playhead `|`, beat ruler with bar numbers, loop region `[~~~~]`, cue markers `C1`-`C8`.

---

## Visual Aids

### Phase Meter
A CDJ-style "dancing blocks" meter is right-justified at the bottom of the screen:
- **L (Leader)** row: Shows the reference beat positions.
- **F (Follower)** row: Shows where the synced deck's beats are relative to the leader.
- When the blocks align vertically at the center `|`, the tracks are in perfect phase.

### Crossfader Visualizer
An original-style fader position bar centered in the bottom cockpit:
`XFADE A [....|....] B`

---

## Themes

10 built-in themes, selectable live from `ESC → THEME`: Default (cyan), Amber, Green Phosphor, Red Sector, Ice. Add custom themes in `djcmd_config.h`.

---

## Options Menu

Press **`ESC`** to open. Tabs: **INFO · AUDIO · DISPLAY · SYNC · THEME · MIDI · OUT · FX**.

Navigate tabs with `←`/`→`; navigate rows with `j`/`k`; toggle or adjust with `LEFT`/`RIGHT` (or `-`/`+`).

### SYNC Tab

| Option | Default | Description |
|---|---|---|
| Quantize play | ON | SPACE on a synced deck waits for bar-1 of leader before starting |
| Smart BPM range | ON | Folds BPM by octaves before sync to prevent 90→180 jumps |
| Auto leader handoff | ON | If the leader deck is reloaded, the playing deck becomes leader |
| **Library Autoplay** | **OFF** | **Continuous Mix Mode**: Alternates between Deck A and B. When a track reaches its final 8 bars, the next track is automatically loaded and started on the opposite deck with an automated crossfade. |
| Key lock default | OFF | New tracks load with key lock (WSOLA time-stretch) pre-enabled |
| **Vinyl mode** | **ON** | **Motorised platters only** -- see below |

#### Vinyl Mode (motorised platters -- NS7III)

| State | Behaviour |
|---|---|
| **ON** (default) | The playhead runs independently at the pitch-fader speed. Touching the vinyl surface or platter rim engages 1:1 scratch control. Releasing snaps back to normal speed instantly, with no spinup wow. This matches how CDJ vinyl mode works. |
| **OFF** | The platter velocity always drives the audio (DVS / timecode-vinyl style). Motor speed fluctuations affect pitch; stopping the platter stops the audio even without a detected touch. |

Vinyl mode only takes effect when `g_motor_running` is true for a deck (i.e. the NS7III motor is spinning). Relative-encoder jog wheels are unaffected.

---

## Waveform Cache (Sidecar Files)

`.djcmd` sidecar written next to each audio file after first load (approx. 12kb per track). Subsequent loads are near-instant. Press `b` to force rebuild, or delete the `.djcmd` file.

---

## Playlist

In-session ordered list. `p` to add, `TAB` to view, `DEL` to remove, `Ctrl+X` to clear. Not saved to disk.

---

## Crates (Quick Jump)

`crates.txt` allows you to define directory aliases for near-instant navigation.

**Setup:**
Create a file named `crates.txt` in `~/.config/djcmd/` (recommended) or in your current working directory. Each line should follow this format:
```
<alias> <absolute_path>
```
Example:
```
techno /home/user/music/techno
house  /home/user/music/house
```

**Usage:**
1. Press **`C`** while in the browser.
2. A prompt appears: `Jump to: _` along with a list of your available aliases.
3. Type the alias (e.g., `house`) and press **`ENTER`**.
4. The browser will instantly jump to that directory.

---

## MusicBrainz Tag Lookup

Press `i` on a selected track. Requires `curl`. Results in a floating overlay; any key to dismiss.

---

## Customisation -- djcmd\_config.h

```c
#define CFG_PCM_DEVICE       "default"   // ALSA PCM device
#define CFG_SAMPLE_RATE      44100       // overridden at runtime if device differs
#define CFG_PERIOD_FRAMES    512         // lower = less latency, more CPU
#define CFG_WFM_ROWS         10          // waveform height in rows
#define CFG_WFM_VISIBLE_SECS 4.0f
#define CFG_DEFAULT_THEME    0
```

Edit and run `make`. No other files need changing for typical customisation.

---

## Architecture

### Threads

```
main()
 ├── load_worker    track load + waveform analysis + Mixxx DB import
 │                  double-buffer swap: frees old PCM outside audio lock (no dropout)
 ├── audio_thread   ALSA write loop
 │     wsola/read_pitched → EQ → FX slots → crossfade → master FX → soft-clip → ALSA
 ├── midi_thread    snd_rawmidi_read() → handle_midi()
 └── ui_thread      ncurses redraw (state-tracked LED refresh) + handle_key()
```

### Source Files

```
djcmd.c          main application (~15 000 lines)
djcmd_config.h   user-configurable constants, keybinds, themes
ns7iii_map.h     NS7III MIDI map (compiled in, auto-written on connect)
audiofile.h/.c   WAV / MP3 / FLAC decoder
Makefile         Arch Linux POWER-tuned build system
mixtrack_3.map   Mixtrack 3 / Pro 3 map (place in ~/.config/djcmd/maps/)
ns7_iii.map      NS7III map reference (auto-generated at ~/.config/djcmd/)
```

---

## PowerPC Notes

**Target:** PowerBook G4 (Late 2005) -- IBM PowerPC 7447A @ 1.67 GHz, 2 GB RAM, Arch Linux POWER.

Flags: `-mcpu=7450 -mtune=7450 -O2 -ffast-math -funroll-loops -fomit-frame-pointer`

**Performance Tip:** If you experience audio dropouts during 4-deck mixing or heavy FX use, enable **ECO Mode** in the `ESC → AUDIO` tab. This optimizes the time-stretching search algorithm to significantly reduce CPU load with minimal impact on audio quality.

AltiVec omitted -- `vec_ld` misalignment causes silent data corruption on stack buffers. Double precision used for the playback pointer to avoid float mantissa exhaustion on long tracks.

---

## Troubleshooting

**No audio:** `aplay -l` to list devices. Select the device from `ESC → AUDIO tab`, or set `CFG_PCM_DEVICE` in `djcmd_config.h` and rebuild.

**Note:** If the "default" ALSA device is missing or busy on startup, djcmd will now automatically try to fall back to the first available hardware device found on the system. If no devices can be opened, the app will still launch so you can select one manually in the options menu.

**No MIDI:** `amidi -l` to list devices. Select from `ESC → MIDI tab`. djcmd auto-detects on startup -- if nothing is found, open the MIDI tab and select manually.

**Controller map not loading:** check the MIDI tab for the exact sanitised device name. Delete `~/.config/djcmd/<name>.map` to force re-install from `maps/`.

**NS7III Deck C/D controls not working:** enable 4-deck mode first (`T` key). Then press deck selector [3] (left platter → Deck C) or [4] (right platter → Deck D).

**NS7III play/motor missing on Deck B:** the runtime map at `~/.config/djcmd/ns7_iii.map` may be stale from an older version. Delete it to regenerate: `rm ~/.config/djcmd/ns7_iii.map`.

**BPM shows 120.0:** track not in Mixxx. Add, analyse, reload. Use `]`/`[` to nudge the grid.

**Audio stutter on load:** should not occur with the double-buffer swap. If it persists, increase `CFG_PERIOD_FRAMES` to 1024 and rebuild.

**Waveform wrong:** press `b` on the loaded deck, or delete the `.djcmd` sidecar.

**make deps fails (no network):** copy `minimp3.h` and `dr_flac.h` manually from GitHub.

---

## Credits & Acknowledgements

- **cmus** — [cmus/cmus](https://github.com/cmus/cmus). Inspired the lightweight, keyboard-driven terminal interface and library management.
- **btop** — [aristocratos/btop](https://github.com/aristocratos/btop). Inspired the high-performance, visually rich terminal UI and layout.
- **MIXXX** — [mixxxdj/mixxx](https://github.com/mixxxdj/mixxx). The gold standard for open-source DJ software; inspired the deck logic, beat-syncing, and library database compatibility.
- **Arch Linux POWER** — The primary development and target platform. Special thanks to the maintainers of the [Arch Linux POWER](https://archlinuxpower.org/) project for keeping the G4/G5 machines alive.
- **minimp3.h** — [lieff/minimp3](https://github.com/lieff/minimp3) (Public Domain / CC0).
- **dr_flac.h** — [mackron/dr_libs](https://github.com/mackron/dr_libs) (Public Domain / MIT-0).
- **libasound2** — ALSA project.
- **ncurses** — GNU project.
- **sqlite3** — Public Domain.

---

## Licence

Licensed under the GNU General Public License v3.0.

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
