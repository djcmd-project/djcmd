/*
 * djcmd_config.h — User configuration for djcmd
 * Copyright (C) 2025  djcmd contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ─────────────────────────────────────────────────────────────────────
 * This is the single file to edit for most customisations.
 * After any change, rebuild with:  make
 *
 * Sections:
 *   Audio Hardware     — ALSA device, sample rate, buffer sizes
 *   Default Volumes    — master/deck volume, auto-gain settings
 *   Waveform Display   — bins, kick detection, height curve, visible range
 *   Sync & Beat        — nudge amount and decay rate
 *   Key Bindings       — rebind any key without touching djcmd.c
 *   MIDI CC Mappings   — CC numbers for volume, pitch, EQ, crossfader
 *   MIDI Note Mappings — note numbers for play/cue/sync triggers
 *   Filesystem         — browser limits, sidecar extension, config dir
 *   Themes             — define custom colour themes (see bottom of file)
 *
 * Key name reference for bindings:
 *   Printable:    'a'  ' '  '!'  etc.
 *   Arrow keys:   KEY_UP  KEY_DOWN  KEY_LEFT  KEY_RIGHT
 *   Shifted:      KEY_SLEFT  KEY_SRIGHT
 *   Function:     KEY_F(1) … KEY_F(12)
 *   Navigation:   KEY_NPAGE (PgDn)  KEY_PPAGE (PgUp)
 *                 KEY_BACKSPACE  KEY_ENTER  KEY_DC (Delete)
 *   Special:      '\t' (Tab)  '\n' (Enter)  27 (Escape)  24 (Ctrl+X)
 */

#ifndef DJCMD_CONFIG_H
#define DJCMD_CONFIG_H

/* Maximum number of simultaneous decks */
#define MAX_TRACKS 4

/* ──────────────────────────────────────────────
   Audio Hardware
   ──────────────────────────────────────────────
   CFG_PCM_DEVICE:    ALSA PCM output device.
                      Run `aplay -l` to list devices.
                      Examples: "default", "hw:0,0", "plughw:1,0"
   CFG_PERIOD_FRAMES: Frames per ALSA write.  Lower = less latency, more CPU.
                      Typical values: 256, 512, 1024, 2048.
   CFG_BUFFER_PERIODS: Number of periods in the ALSA ring buffer.
                       Increase if you hear xruns (crackling).
   ────────────────────────────────────────────── */
#define CFG_PCM_DEVICE "default" /* ALSA PCM device */
#define CFG_PCM_HEADPHONE "hw:0,0" /* Headphone device (e.g. built-in) */
#define CFG_SAMPLE_RATE 44100
#define CFG_CHANNELS 2
#define CFG_PERIOD_FRAMES 512 /* frames per ALSA write (latency) */
#define CFG_BUFFER_PERIODS 4 /* ALSA ring buffer depth */

/* ──────────────────────────────────────────────
   Default Volumes & Gain
   ──────────────────────────────────────────────
   CFG_DEFAULT_MASTER_VOL:  Master output volume 0–150 (100 = unity).
   CFG_DEFAULT_HEADPHONE_VOL: Headphone output volume 0–150.
   CFG_DEFAULT_DECK_VOL:    Per-deck fader level 0.0–1.5 (1.0 = unity).
   CFG_AUTO_GAIN_ENABLED:   1 = auto-gain on by default for newly loaded tracks.
   CFG_AUTO_GAIN_TARGET:    RMS target in dBFS.  Typical values: −14 (loud),
                            −18 (broadcast standard), −23 (EBU R128).
   These are startup defaults.  All values are adjustable live in the
   Options menu (ESC → AUDIO tab).
   ────────────────────────────────────────────── */
#define CFG_DEFAULT_MASTER_VOL 100 /* 0–100 */
#define CFG_DEFAULT_HEADPHONE_VOL 100 /* 0–100 */
#define CFG_DEFAULT_DECK_VOL 1.0f /* 0.0–1.0 */
#define CFG_AUTO_GAIN_ENABLED 0 /* 1 = auto-gain on by default */
#define CFG_AUTO_GAIN_TARGET -14.0f /* dBFS target for normalisation */

/* ──────────────────────────────────────────────
   Waveform Display
   ──────────────────────────────────────────────
   CFG_WFM_ROWS:         Terminal rows used per scrolling waveform panel.
   CFG_WFM_VISIBLE_SECS: Seconds of audio shown left+right of the playhead.
                         Smaller = more zoom, larger = more context.
   CFG_WFM_OVERVIEW_BINS: Number of analysis bins computed over the full track.
                          Higher = finer detail, slightly longer analysis time.
                          Sane values: 2048, 4096 (default), 8192.
   ────────────────────────────────────────────── */
#define CFG_WFM_ROWS 10 /* terminal rows per waveform panel */
#define CFG_WFM_VISIBLE_SECS 4.0f /* seconds visible left+right of playhead */
#define CFG_WFM_OVERVIEW_BINS 4096 /* pre-computed overview resolution */
/* (was 2048 — doubled for accuracy) */

/* Kick drum detection threshold.
 * When low_band / (mid_band + high_band + 0.01) > this value,
 * the column is rendered in bright red regardless of other bands. */
#define CFG_KICK_THRESHOLD 2.5f

/* Waveform height mapping: "disp = pow(amp, CFG_WFM_HEIGHT_GAMMA)"
 * 0.5 = sqrt (lifts quiet passages), 1.0 = linear, >1 = compress peaks */
#define CFG_WFM_HEIGHT_GAMMA 0.5f

/* ──────────────────────────────────────────────
   Sync & Beat
   ──────────────────────────────────────────────
   CFG_NUDGE_AMOUNT: Pitch offset applied per ] or [ keypress.
                     0.02 = 2% — one nudge shifts the deck ~2 BPM momentarily.
   CFG_NUDGE_DECAY:  Multiplicative decay applied each audio period (~11ms).
                     0.975 = ~320ms half-life.  Slow enough that repeated
                     keypresses accumulate under key-repeat; decays fully in
                     ~2s after release.  Lower = faster decay (was 0.94).
   CFG_NUDGE_CAP:    Maximum nudge magnitude as a multiple of NUDGE_AMOUNT.
                     16 × 0.02 = 32% max (~5 semitones at full stack).
                     Normal rapid pressing plateaus well below the cap.
   ────────────────────────────────────────────── */
#define CFG_NUDGE_AMOUNT 0.02f /* pitch offset per nudge event    */
#define CFG_NUDGE_DECAY 0.972f /* decay per audio period (~11ms)  */
#define CFG_JOG_DEAD_BAND 0.005f
#define CFG_NUDGE_CAP 16 /* max = NUDGE_AMOUNT × this       */

/* ──────────────────────────────────────────────
   Key Bindings
   Edit the right-hand value to rebind any key.
   Use a literal character like 'x' or a ncurses
   constant like KEY_F(1).  Key names:
     KEY_UP KEY_DOWN KEY_LEFT KEY_RIGHT
     KEY_NPAGE (PgDn) KEY_PPAGE (PgUp)
     KEY_BACKSPACE KEY_ENTER KEY_DC (Del)
     KEY_F(1)..KEY_F(12)
     '\t' (Tab)  '\n' (Enter)  27 (Esc)
   ────────────────────────────────────────────── */

/* Deck selection */
#define KEY_DECK_A '1'
#define KEY_DECK_B '2'
#define KEY_DECK_C '3'
#define KEY_DECK_D '4'

/* Playback */
#define KEY_PLAY_PAUSE ' '
#define KEY_STOP 's'
#define CFG_KEY_RESTART 'r' /* renamed — KEY_RESTART is reserved by ncurses */
#define KEY_LOOP_TOGGLE 'l'
#define KEY_REANALYSE 'b'

/* Sync */
#define KEY_SYNC_MASTER 'M'
#define KEY_SYNC_SLAVE 'y'
#define KEY_BEAT_NUDGE_FWD ']'
#define KEY_BEAT_NUDGE_BACK '['
#define KEY_GRID_SHIFT_FWD '}'
#define KEY_GRID_SHIFT_BACK '{'

/* Gang mode */
#define KEY_GANG_TOGGLE 'G'

/* Cue points */
#define KEY_CUE_SET_1 KEY_F(5)
#define KEY_CUE_SET_2 KEY_F(6)
#define KEY_CUE_SET_3 KEY_F(7)
#define KEY_CUE_SET_4 KEY_F(8)
#define KEY_CUE_JUMP_1 KEY_F(9)
#define KEY_CUE_JUMP_2 KEY_F(10)
#define KEY_CUE_JUMP_3 KEY_F(11)
#define KEY_CUE_JUMP_4 KEY_F(12)

/* Headphone Cue (PFL) toggle per deck */
#define KEY_HEADPHONE_CUE_A '5'
#define KEY_HEADPHONE_CUE_B '6'
#define KEY_HEADPHONE_CUE_C '7'
#define KEY_HEADPHONE_CUE_D '8'

/* Pitch */
#define KEY_PITCH_UP_FINE 'e'
#define KEY_PITCH_DOWN_FINE 'd'
#define KEY_PITCH_UP_COARSE 'E'
#define KEY_PITCH_DOWN_COARSE 'D'
#define KEY_PITCH_RESET '0'
/* V = cycle pitch fader range for active deck: ±8% → ±25% → ±50% → ±8%
 * Affects the MIDI pitch fader span only; keyboard e/d/E/D steps are unchanged. */
#define KEY_PITCH_RANGE 'V'

/* Volume */
#define KEY_VOL_UP '+'
#define KEY_VOL_DOWN '-'
#define KEY_AUTOGAIN 'A'
#define KEY_MASTER_UP 'm'
#define KEY_MASTER_DOWN 'n'

/* EQ */
#define KEY_EQ_LOW_UP 'q'
#define KEY_EQ_LOW_DOWN 'a'
#define KEY_EQ_MID_UP 'w'
#define KEY_EQ_MID_DOWN 'x' /* NOTE: x is free since playlist uses DEL */
#define KEY_EQ_HIGH_UP 't'
#define KEY_EQ_HIGH_DOWN 'g'

/* Crossfader */
#define KEY_XFADE_LEFT '<'
#define KEY_XFADE_RIGHT '>'

/* Browser / Playlist navigation */
#define KEY_NAV_DOWN 'j'
#define KEY_NAV_UP 'k'
#define KEY_BROWSER_ADD_PLAYLIST 'p'
#define KEY_BROWSER_TAG_LOOKUP 'i'
#define KEY_PLAYLIST_TOGGLE 'P'
#define KEY_PLAYLIST_REMOVE KEY_DC /* Delete key */
#define KEY_PLAYLIST_CLEAR 24 /* Ctrl+X */

/* Load to deck from browser */
#define KEY_LOAD_DECK_A '!'
#define KEY_LOAD_DECK_B '@'
#define KEY_LOAD_DECK_C '#'
#define KEY_LOAD_DECK_D '$'

/* View switching */
#define KEY_VIEW_BROWSER '\t' /* Tab */
#define KEY_VIEW_HELP '?'
#define KEY_VIEW_OPTIONS 27 /* Escape */
#define KEY_BROWSER_HOME '~'
#define KEY_BROWSER_ROOT '\\'

/* Seek (beat-based) */
/* KEY_LEFT / KEY_RIGHT = ±1 beat, KEY_SLEFT / KEY_SRIGHT = ±8 beats */

/* System */
#define KEY_TOGGLE_DECKS 'T'
#define KEY_QUIT 'Q'

/* ──────────────────────────────────────────────
   MIDI CC Mappings
   ──────────────────────────────────────────────
   djcmd reads raw ALSA MIDI.  The first available MIDI input device is
   opened automatically.  Pass a device string as the second argument to
   override:  ./djcmd /music hw:1,0,0
   Use `amidi -l` or `aconnect -l` to list available devices.

   Set any value to -1 to disable that mapping.

   CC base offsets — deck A=base, B=base+1, C=base+2, D=base+3:
     MIDI_CC_VOLUME_BASE  7   →  CC 7  (deck A), CC 8  (B), CC 9  (C), CC 10 (D)
     MIDI_CC_PITCH_BASE  20   →  CC 20 (A), 21 (B), 22 (C), 23 (D)
                                 Maps 0–127 to pitch range (default ±8%, set with V
                                 key or pitch_range_a/b/c/d MIDI button).
                                 Center (63–64) always = 0%, 2% dead zone applied.
     MIDI_CC_EQ_LOW_BASE  30  →  CC 30–33 per deck
     MIDI_CC_EQ_MID_BASE  34  →  CC 34–37 per deck
     MIDI_CC_EQ_HIGH_BASE 38  →  CC 38–41 per deck
     MIDI_CC_CROSSFADER   60  →  CC 60, any channel, 0–127 → A–B
                                 Has 10% dead zone at centre; never boosts above unity.
     MIDI_CC_MASTER_VOL   61  →  CC 61, any channel, 0–127 → 0–150%

   Infinite encoder (relative CC) actions — use the MIDI Learn tab to bind:
     jog_spin_a/b/c/d    : Jog wheel spin (center-64: >64=fwd, <64=rev)
                           Touch sensor: jog_touch_a/b/c/d (Note On/Off)
                           Top surface = scratch mode, edge = nudge mode
     pitch_bend_a/b/c/d  : Per-deck pitch bend encoder (same encoding as jog)
                           Produces a decaying nudge, same as ] / [ keyboard keys
     lib_encoder         : Library/browser scroll encoder (center-64 relative)

   Note On button actions — bind via MIDI Learn:
     play_a/b/c/d        : Play/pause per deck
     lib_select          : Load highlighted browser entry to active deck
     lib_load_a/b/c/d    : Load highlighted entry to specific deck (A/B/C/D)
     pitch_range_a/b/c/d : Cycle pitch fader range ±8%→±25%→±50%→±8% per deck
     sync_a/b/c/d        : Toggle sync slave per deck
     cue_1/2/3/4         : Jump to cue point on active deck
     nudge_fwd/back      : Pitch nudge forward/backward (active deck)
     loop                : Toggle loop on active deck

   Note On mappings (any channel; note velocity > 0 triggers):
     MIDI_NOTE_PLAY_BASE  36  →  notes 36–39 = play/pause deck A–D
     MIDI_NOTE_CUE_BASE   40  →  notes 40–43 = jump to cue 1–4 on active deck
     MIDI_NOTE_SYNC_BASE  44  →  notes 44–47 = toggle sync slave on deck A–D

   For deeper remapping (custom behaviour, velocity sensitivity, etc.),
   edit handle_midi() in djcmd.c.
   ────────────────────────────────────────────── */

#define CFG_MIDI_CHANNEL 0 /* 0-indexed (channel 1 = 0) */

/* Per-deck CC base offsets: A=base, B=base+1, C=base+2, D=base+3 */
#define MIDI_CC_VOLUME_BASE 7
#define MIDI_CC_PITCH_BASE 20
#define MIDI_CC_EQ_LOW_BASE 30
#define MIDI_CC_EQ_MID_BASE 34
#define MIDI_CC_EQ_HIGH_BASE 38
#define MIDI_CC_CROSSFADER 60
#define MIDI_CC_MASTER_VOL 61

/* MIDI Note On base offsets */
#define MIDI_NOTE_PLAY_BASE 36 /* 36–39 → play/pause deck A–D */
#define MIDI_NOTE_CUE_BASE 40 /* 40–43 → cue jump 1–4 (active deck) */
#define MIDI_NOTE_SYNC_BASE 44 /* 44–47 → sync slave toggle deck A–D */

/* ──────────────────────────────────────────────
   Filesystem
   ──────────────────────────────────────────────
   CFG_FB_MAX_ENTRIES: Maximum number of entries shown in the file browser
                       per directory scan.
   CFG_FB_PATH_MAX:    Maximum length of a directory path string.
   CFG_PL_MAX:         Maximum playlist entries (in-session, not saved to disk).
   CFG_CONFIG_DIR:     Subdirectory of $HOME for djcmd config files.
   CFG_SIDECAR_EXT:    Extension appended to audio files for waveform cache.
                       e.g. track.flac → track.flac.djcmd
   ────────────────────────────────────────────── */
#define CFG_FB_MAX_ENTRIES 2048
#define CFG_FB_PATH_MAX 1024
#define CFG_PL_MAX 512
#define CFG_CONFIG_DIR ".config/djcmd"
#define CFG_LIBRARY_FILE "library" /* default music path */
#define CFG_SIDECAR_EXT ".djcmd" /* waveform cache extension */

/* ──────────────────────────────────────────────
   Themes
   ─────────────────────────────────────────────

   Each theme defines foreground/background colours for the six
   named UI colour pairs.  Use standard ncurses colour constants:
     COLOR_BLACK   COLOR_RED     COLOR_GREEN   COLOR_YELLOW
     COLOR_BLUE    COLOR_MAGENTA COLOR_CYAN    COLOR_WHITE
   Use -1 for "terminal default" background.

   To create your own theme, copy one of the entries below, change
   the name string and colour values, increment THEME_COUNT, and
   set CFG_DEFAULT_THEME to its index.

   Fields per theme:
     name          — shown in the options menu
     header_fg/bg  — section labels, dividers, deck headers
     active_fg/bg  — selected item / active deck highlight
     vu_fg/bg      — VU meters, EQ bars, pitch bars
     status_fg/bg  — bottom status bar
     hot_fg/bg     — beat markers, alerts, sync lock
     wfm8_lo/mid/hi — 8-colour waveform: quiet/mid/loud band colour
   ────────────────────────────────────────────── */

#define THEME_COUNT 10

typedef struct {
	const char *name;
	/* UI pairs */
	short header_fg, header_bg;
	short active_fg, active_bg;
	short vu_fg, vu_bg;
	short status_fg, status_bg;
	short hot_fg, hot_bg;
	/* 8-colour waveform gradient colours */
	short wfm8_lo; /* quiet / bass-dominant  */
	short wfm8_mid; /* moderate / mixed       */
	short wfm8_hi; /* loud / kick-dominant   */
} ThemeDef;

/* ── Built-in themes ── */
/* Index 0: Default (cyan on black — original djcmd look) */
#define THEME_DEFAULT                                  \
	{ "Default",   COLOR_CYAN,  -1, /* header   */ \
	  COLOR_BLACK, COLOR_CYAN, /* active   */      \
	  COLOR_GREEN, -1, /* vu       */              \
	  COLOR_BLACK, COLOR_WHITE, /* status   */     \
	  COLOR_RED,   -1, /* hot      */              \
	  COLOR_BLUE,  COLOR_CYAN,  COLOR_WHITE }

/* Index 1: Amber (warm amber-on-black, classic terminal look) */
#define THEME_AMBER                                      \
	{ "Amber",	COLOR_YELLOW, -1, /* header   */ \
	  COLOR_BLACK,	COLOR_YELLOW, /* active   */     \
	  COLOR_YELLOW, -1, /* vu       */               \
	  COLOR_BLACK,	COLOR_YELLOW, /* status   */     \
	  COLOR_RED,	-1, /* hot      */               \
	  COLOR_RED,	COLOR_YELLOW, COLOR_WHITE }

/* Index 2: Green Phosphor (green CRT — easy on the eyes in dark rooms) */
#define THEME_PHOSPHOR                                      \
	{ "Green Phosphor", COLOR_GREEN, -1, /* header   */ \
	  COLOR_BLACK,	    COLOR_GREEN, /* active   */     \
	  COLOR_GREEN,	    -1, /* vu       */              \
	  COLOR_BLACK,	    COLOR_GREEN, /* status   */     \
	  COLOR_YELLOW,	    -1, /* hot      */              \
	  COLOR_GREEN,	    COLOR_CYAN,	 COLOR_WHITE }

/* Index 3: Red Sector (dark red — Serato-inspired, high contrast kicks) */
#define THEME_RED_SECTOR                          \
	{ "Red Sector", COLOR_RED,                \
	  -1, /* header   */                      \
	  COLOR_WHITE,	COLOR_RED, /* active   */ \
	  COLOR_RED,	-1, /* vu       */        \
	  COLOR_WHITE,	COLOR_RED, /* status   */ \
	  COLOR_YELLOW, -1, /* hot      */        \
	  COLOR_RED,	COLOR_MAGENTA,            \
	  COLOR_YELLOW }

/* Index 4: Ice (blue/white — clean and bright) */
#define THEME_ICE                                        \
	{ "Ice",	 COLOR_CYAN,  -1, /* header   */ \
	  COLOR_BLACK,	 COLOR_WHITE, /* active   */     \
	  COLOR_CYAN,	 -1, /* vu       */              \
	  COLOR_BLACK,	 COLOR_BLUE, /* status   */      \
	  COLOR_MAGENTA, -1, /* hot      */              \
	  COLOR_BLUE,	 COLOR_CYAN,  COLOR_MAGENTA }

/* Index 5: Synthwave (neon magenta/cyan) */
#define THEME_SYNTHWAVE                                     \
	{ "Synthwave", COLOR_MAGENTA, -1, /* header   */    \
	  COLOR_BLACK, COLOR_MAGENTA, /* active   */        \
	  COLOR_CYAN,  -1, /* vu       */                   \
	  COLOR_BLACK, COLOR_WHITE, /* status   */          \
	  COLOR_RED,   -1, /* hot      */                   \
	  COLOR_MAGENTA, COLOR_CYAN, COLOR_WHITE }

/* Index 6: Midnight (deep blue/grey) */
#define THEME_MIDNIGHT                                      \
	{ "Midnight", COLOR_BLUE,  -1, /* header   */       \
	  COLOR_WHITE, COLOR_BLUE, /* active   */           \
	  COLOR_CYAN,  -1, /* vu       */                   \
	  COLOR_BLACK, COLOR_BLUE, /* status   */           \
	  COLOR_YELLOW, -1, /* hot      */                  \
	  COLOR_BLUE,  COLOR_CYAN, COLOR_WHITE }

/* Index 7: Solar (yellow/orange high-energy) */
#define THEME_SOLAR                                         \
	{ "Solar",     COLOR_YELLOW, -1, /* header   */     \
	  COLOR_BLACK, COLOR_YELLOW, /* active   */         \
	  COLOR_RED,   -1, /* vu       */                   \
	  COLOR_BLACK, COLOR_WHITE, /* status   */          \
	  COLOR_WHITE, -1, /* hot      */                   \
	  COLOR_RED,   COLOR_YELLOW, COLOR_WHITE }

/* Index 8: Deep Sea (cyan/blue) */
#define THEME_DEEPSEA                                       \
	{ "Deep Sea",  COLOR_CYAN,  -1, /* header   */      \
	  COLOR_BLACK, COLOR_CYAN, /* active   */           \
	  COLOR_BLUE,  -1, /* vu       */                   \
	  COLOR_WHITE, COLOR_BLUE, /* status   */           \
	  COLOR_GREEN, -1, /* hot      */                   \
	  COLOR_BLUE,  COLOR_CYAN, COLOR_GREEN }

/* Index 9: Vampire (blood red/black) */
#define THEME_VAMPIRE                                       \
	{ "Vampire",   COLOR_RED,   -1, /* header   */      \
	  COLOR_BLACK, COLOR_RED,   /* active   */          \
	  COLOR_WHITE, -1, /* vu       */                   \
	  COLOR_BLACK, COLOR_RED,   /* status   */          \
	  COLOR_YELLOW, -1, /* hot      */                  \
	  COLOR_RED,   COLOR_BLACK, COLOR_WHITE }

/* Default theme index (0–THEME_COUNT-1) */
#define CFG_DEFAULT_THEME 0

/* ── NS7III Motor Control ────────────────────────────────────────────────────
 * The NS7III has 2 physical platter motors (left = ch2 = Deck A,
 * right = ch3 = Deck B).  When a layer switch routes the left platter to
 * Deck C (or right to Deck D), the motor still runs on the same physical
 * channel — only the software deck receiving jog/pitch data changes.
 * CFG_MOTOR_CH_C=0 and CFG_MOTOR_CH_D=0 disable motors for decks C and D;
 * motor start/stop is handled by the physical platter's channel (A or B)
 * via motor_handoff() at layer-switch time.
 *
 * Confirmed via motor_sweep probe tool.  All CCs sent on g_midi_out.
 * Channel numbers are 1-indexed (Ch2 = Deck A left platter, Ch3 = Deck B right platter).
 *
 * CONFIRMED CC MAP (Ch2/Ch3 per deck):
 *   CC#65=127  START motor
 *   CC#66=127  STOP motor (clean stop; other values = varying brake)
 *   CC#69=any  FORWARD direction trigger (send AFTER CC#65 start)
 *   CC#70=1    REVERSE direction trigger (any nonzero = reverse)
 *   CC#68=any  BRAKE register (any value slows/stops — do NOT send during run)
 *   CC#73=0-100 SPEED RAMP (0→100 over 18 steps at 20Hz for smooth spinup)
 *   CC#74=0-5  GLOBAL sawtooth drive on Ch1 (sent during ramp only)
 *   CC#75=0    GLOBAL enable on Ch1 (one-shot before first start)
 *
 * START/STOP TIME:
 *   Serato "start time" / "stop time" knobs map to CC#73 ramp speed.
 *   CFG_MOTOR_RAMP_STEPS: number of 20Hz steps to reach full speed (18 = ~0.9s).
 *   Increase for slower start (more steps), decrease for faster (fewer steps).
 *   CFG_MOTOR_BRAKE_VAL: value sent to CC#68 on stop for timed braking.
 *   0=instant, 64=medium brake, values 1-127 give varying brake duration.
 *   (CC#66=127 always gives a clean immediate stop regardless of brake val.)
 *
 * Set CFG_MOTOR_ENABLED 0 to disable all motor output. */
#define CFG_MOTOR_ENABLED 1
#define CFG_MOTOR_START_CC 65 /* CC to start motor (val 127)        */
#define CFG_MOTOR_STOP_CC 66 /* CC to stop motor  (val 127)        */
#define CFG_MOTOR_FWD_CC 69 /* FORWARD trigger (send after start) */
#define CFG_MOTOR_REV_CC 70 /* REVERSE trigger (nonzero = rev)    */
#define CFG_MOTOR_RAMP_CC 73 /* Speed ramp 0→100 during spinup     */
#define CFG_MOTOR_START_VAL 127
#define CFG_MOTOR_STOP_VAL 0
#define CFG_MOTOR_CH_A 2 /* deck A left platter  — 1-indexed   */
#define CFG_MOTOR_CH_B 3 /* deck B right platter — 1-indexed   */
#define CFG_MOTOR_CH_C 4 /* deck C left platter  (layer 2)     */
#define CFG_MOTOR_CH_D 5 /* deck D right platter (layer 2)     */


/* Start/stop time (ramp speed):
 * RAMP_STEPS: steps at 20Hz to reach full speed. 18=~0.9s, 36=~1.8s, 9=~0.45s
 * BRAKE_VAL:  CC#68 value sent before CC#66 stop for gradual braking.
 *             0=no brake (instant stop via CC#66 only), 1-126=brake, 127=hard stop.
 *             Note: CC#68 at most values reduces speed before the CC#66 stop. */
#define CFG_MOTOR_RAMP_STEPS 18 /* spinup steps (18 = ~0.9s "instant")*/
#define CFG_MOTOR_BRAKE_VAL 0 /* 0=instant stop, >0=brake before stop */

#endif /* DJCMD_CONFIG_H */
