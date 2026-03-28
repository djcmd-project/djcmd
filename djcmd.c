/*
 * djcmd — Command-line DJ application
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
 * ── Third-party components compiled into this binary ──────────────────
 * minimp3.h  — CC0 (public domain)  — https://github.com/lieff/minimp3
 * dr_flac.h  — public domain / MIT-0 — https://github.com/mackron/dr_libs
 * Runtime-linked libraries and their licenses:
 *   libasound   LGPL-2.1+   https://www.alsa-project.org
 *   libncurses  MIT/X11     https://invisible-island.net/ncurses/
 *   libsqlite3  public domain  https://www.sqlite.org
 *   libpthread  LGPL-2.1+   (glibc)
 *   libm        LGPL-2.1+   (glibc)
 * ──────────────────────────────────────────────────────────────────────
 *
 * Optimised for IBM PowerPC 7447A 32-bit (PowerBook G4 Late 2005).
 * See README.md for full build instructions.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <ncurses.h> /* Arch ships wide-char support in the main ncurses */

/* SIMD headers */
#if defined(__SSE2__)
  #include <emmintrin.h>
#elif defined(__ALTIVEC__)
  #include <altivec.h>
  #undef vector
  #define vfloat __vector float
#endif
#include <wchar.h>
#include <locale.h>
#include <alsa/asoundlib.h>
#include <ctype.h>
#include "audiofile.h"
#include <sqlite3.h> /* Mixxx library import */
#include <time.h>
#include "djcmd_config.h" /* user-configurable settings */
#include "ns7iii_map.h" /* NS7III default MIDI map     */
#include "djcmd_audio.h"   /* audio structures and stretcher */
#include "djcmd_fx.h"   /* effects engine               */
#include "djcmd_help.h" /* help view + UI color pairs   */

/* ──────────────────────────────────────────────
   Configuration & Constants  (see djcmd_config.h to change these)
   ────────────────────────────────────────────── */
#define SAMPLE_RATE CFG_SAMPLE_RATE
#define CHANNELS CFG_CHANNELS
#define PERIOD_FRAMES CFG_PERIOD_FRAMES
#define BUFFER_PERIODS CFG_BUFFER_PERIODS
#define MAX_FILENAME 512
#define PCM_DEVICE CFG_PCM_DEVICE

#define WFM_ROWS CFG_WFM_ROWS
#define WFM_DECKS 2
#define WFM_VISIBLE_SECS CFG_WFM_VISIBLE_SECS
#define WFM_OVERVIEW_BINS CFG_WFM_OVERVIEW_BINS

#define NUDGE_DECAY CFG_NUDGE_DECAY
#define NUDGE_AMOUNT CFG_NUDGE_AMOUNT

#define FB_MAX_ENTRIES CFG_FB_MAX_ENTRIES
#define FB_PATH_MAX CFG_FB_PATH_MAX
#define PL_MAX CFG_PL_MAX

/* ──────────────────────────────────────────────
   Forward declarations
   ────────────────────────────────────────────── */
/* Audio / motor */
static void motor_set(int deck, int on);
static void motor_sync_pitch(int deck);
static void sync_apply(int slave_idx);
/* LED output */
static void led_on(const char *name);
static void deck_leds_refresh(void);
static void led_off(const char *name);
/* Pad / FX LEDs */
static void pad_leds_refresh(int deck);
static void pad_mode_leds_refresh(int deck);
static void fx_leds_refresh(int deck);
/* Waveform */
static void wfm_compute_band_max(Track *t);
static void wfm_normalize_bands(float lo_raw, float mi_raw, float hi_raw,
				float *lo_n, float *mi_n, float *hi_n,
				const float band_max[3]);
static int wfm_pair_256(float ch_r, float ch_g, float ch_b);
/* File browser */
static void fb_selected_path(char *out, size_t max);
static void fb_enter_dir(const char *name);
/* UI */
static void draw_quit_modal(void);
static void options_read_cpuinfo(char *out, int max);
/* MIDI / PCM device management */
static void midi_map_load(void);
static void midi_map_save(void);
static void settings_save(void);
static void midi_map_write_generic_defaults(void);
static int midi_enumerate_devices(void);
static void midi_map_name_from_device(const char *dev_name, char *out,
				      size_t max);
static int pcm_enumerate_devices(void);
static void pcm_open_device(int dev_idx);
/* Crates */
static void crates_load(void);
static void crate_jump(const char *alias);
/* BPM / Grid */
static void tap_bpm(int deck);
static void snap_grid(int deck);

typedef struct {
	/* 3-band EQ state (simple biquad) — separate L and R delay lines */
	float lp_x1l, lp_x2l, lp_y1l, lp_y2l; /* low-pass  left  */
	float lp_x1r, lp_x2r, lp_y1r, lp_y2r; /* low-pass  right */
	float bp_x1l, bp_x2l, bp_y1l, bp_y2l; /* band-pass left  */
	float bp_x1r, bp_x2r, bp_y1r, bp_y2r; /* band-pass right */
	float hp_x1l, hp_x2l, hp_y1l, hp_y2l; /* high-pass left  */
	float hp_x1r, hp_x2r, hp_y1r, hp_y2r; /* high-pass right */
	/* Variable filter (LP or HP depending on knob position) */
	float fi_x1l, fi_x2l, fi_y1l, fi_y2l; /* filter left  */
	float fi_x1r, fi_x2r, fi_y1r, fi_y2r; /* filter right */
	float fi_b[3], fi_a[3]; /* current filter coefficients */
	float fi_last; /* last filter value — recompute only on change */
} EQState;

/* ──────────────────────────────────────────────
   Globals
   ────────────────────────────────────────────── */
static Track g_tracks[MAX_TRACKS];
static EQState g_eq[MAX_TRACKS];
static int g_num_tracks = 2; /* 2 or 4 */
static float g_crossfader = 0.5f; /* 0=A, 1=B */
static float g_cf_curve = 0.5f; /* 0.0=slow fade, 0.5=linear/eq, 1.0=fast cut */
static int g_master_vol = 100; /* 0–100 */
static volatile int g_running = 1;
static PVState g_pv[MAX_TRACKS];
static WSOLAState g_wsola[MAX_TRACKS];

/* Crates — directory aliases */
#define MAX_CRATES 64
typedef struct {
	char alias[32];
	char path[FB_PATH_MAX];
} Crate;
static Crate g_crates[MAX_CRATES];
static int g_ncrate = 0;
static int g_crate_jump_active = 0; /* 1 if 'C' pressed, waiting for alias */
static char g_crate_input[32] = "";

/* TAP BPM state */
#define MAX_TAPS 8
static int64_t g_tap_ms[MAX_TRACKS][MAX_TAPS];
static int g_tap_idx[MAX_TRACKS] = { 0 };
static int g_tap_count[MAX_TRACKS] = { 0 };
/* g_pv_cos/sin moved to djcmd_audio.c */

/* Actual sample rate negotiated with the ALSA hardware device.
 * Set after pcm_open(); used for all time↔frame conversions and
 * pushed to audiofile via af_set_target_rate() so loads resample
 * to the hardware rate rather than the compiled-in default. */
unsigned int g_actual_sample_rate =
	CFG_SAMPLE_RATE; /* one stretcher per deck */
static snd_pcm_t *g_pcm = NULL;
static snd_pcm_t *g_pcm_hp = NULL; /* Headphone device */
static char g_pcm_hp_dev_str[64] = CFG_PCM_HEADPHONE;
static int g_hp_vol = CFG_DEFAULT_HEADPHONE_VOL;
static snd_rawmidi_t *g_midi_in = NULL;
static snd_rawmidi_t *g_midi_out =
	NULL; /* MIDI output handle for motor/LED control */

/* ── NS7III Display globals — parked in ns7iii_displaysub.h ─────────────
 * Disabled: display thread caused CPU spikes and audio dropouts.
 * Re-enable once the Numark handshake (0x50/0x52/0x53/0x55) is solved.
 * See ns7iii_displaysub.h for full context and re-enable instructions.
 *
 * static snd_rawmidi_t *g_disp_a    = NULL;
 * static snd_rawmidi_t *g_disp_b    = NULL;
 * static pthread_mutex_t g_disp_mutex = PTHREAD_MUTEX_INITIALIZER;
 * static uint16_t g_disp_seq_a = 0;
 * static uint16_t g_disp_seq_b = 0;
 */
/* Per-deck motor running state — tracks whether we've sent start so we
 * don't spam redundant CC messages on every play/pause toggle. */
static int g_motor_running[MAX_TRACKS] = { 0 };
/* Deferred motor start: set by motor_handoff() so the motor_thread waits one
 * 50ms tick after sending stop before sending start on the same channel. */
static volatile int g_motor_pending_start[MAX_TRACKS] = { 0, 0, 0, 0 };
/* Mutex protecting MIDI output: motor thread and main thread both call
 * midi_send_cc; without a lock their 3-byte writes can interleave and
 * produce malformed messages that the hardware ignores. */
static pthread_mutex_t g_midi_out_mutex = PTHREAD_MUTEX_INITIALIZER;
static int64_t g_motor_settle_until[MAX_TRACKS] = { 0 };
/* Dedicated high-res motor stream - Global Scope */
volatile float g_motor_vel[MAX_TRACKS] = {
	0
}; /* written MIDI thread, read audio thread */
/* Slip motor override: when 1, motor is held off even if deck is playing.
 * Toggled by the SLIP MODE button. Motor restarts when toggled back off
 * if the deck is still playing. */
static int g_slip_motor_off[MAX_TRACKS] = { 0 };

/* ── NS7III motor protocol constants ────────────────────────────────────
 * Confirmed from cold-init-load-play-stop pcap:
 *   CC#75=0 Ch1 = global one-shot enable
 *   CC#65=127 ChN = motor START (per-deck)
 *   CC#66=127 ChN = motor STOP  (per-deck)
 *   CC#73    ChN = ramp 0→100 over 18 steps at 20Hz (spinup)
 *   CC#74    Ch1 = global sawtooth 0→5 ascending, runs during ramp only
 *   CC#105   ChN = pitch-at-center echo =64, sent with each ramp step
 * See motor_thread() and motor_set() for full sequence.
 */
#define MOTOR_ENABLE_CC 75 /* one-shot global enable, value=0, Ch1 */

/* ── Pad mode state ─────────────────────────────────────────────────────
 * Each physical side has its own pad mode and shift state.
 * PAD_MODE_HOTCUE  : pads 1-8 = hot cue assign/jump; SHIFT+pad = delete
 * PAD_MODE_AUTOLOOP: pads 1-4 = quantised beat loops (1/2/4/8 bars)
 *                    SHIFT+AUTO/ROLL button → PAD_MODE_ROLL
 * PAD_MODE_ROLL    : hold pad = temporary loop-roll, release = resume
 */
#define PAD_MODE_HOTCUE 0
#define PAD_MODE_AUTOLOOP 1
#define PAD_MODE_ROLL 2
#define PAD_MODE_MANUALLOOP 3 /* pads: loop_in/out/halve/double/reloop */

/* Master cue point per deck (standard CUE button — distinct from hot cues)
 * cue_default_set: 1 if a master cue point has been placed
 * cue_default_pos: frame position of master cue point */
static int g_cue_default_set[MAX_TRACKS] = { 0 };
static uint32_t g_cue_default_pos[MAX_TRACKS] = { 0 };
static int g_cue_default_held[MAX_TRACKS] = { 0 }; /* 1 while CUE button held */

/* Bleep (momentary slip+reverse): save position on press, restore on release */
static int g_bleep_held[MAX_TRACKS] = { 0 };
static uint32_t g_bleep_save_pos[MAX_TRACKS] = { 0 };

/* Filter toggle: 1 = filter sweep knob is engaged, 0 = bypassed (flat) */
static int g_filter_on[MAX_TRACKS] = { 0 };

static int g_pad_mode[MAX_TRACKS] = { 0 }; /* PAD_MODE_* per deck      */
static int g_pad_shift[MAX_TRACKS] = { 0 }; /* shift held per deck side */
static float g_autoloop_bars[MAX_TRACKS] = { 1.0f, 1.0f, 1.0f,
					     1.0f }; /* current autoloop size */
static uint32_t g_roll_resume_pos[MAX_TRACKS] = {
	0
}; /* saved position for roll release */
static int g_roll_active[MAX_TRACKS] = {
	0
}; /* pad index (1-4) or 0 if none   */
/* NS7III physical side routing — 2 platters, 4 virtual decks via layers.
 *
 * The NS7III has two physical platters.  djcmd supports 4 software decks (A–D).
 * Each hardware side routes to one software deck at a time; the deck selector
 * buttons switch the layer:
 *
 *   Side 0 = LEFT  platter (MIDI ch2, deck_sel buttons [1] and [3])
 *            default → Deck A (index 0)   layer 2 → Deck C (index 2)
 *   Side 1 = RIGHT platter (MIDI ch3, deck_sel buttons [2] and [4])
 *            default → Deck B (index 1)   layer 2 → Deck D (index 3)
 *
 * Press [1]/[3] on the controller to switch the left platter between A and C.
 * Press [2]/[4] to switch the right platter between B and D.
 * side_restack() rebinds the per-platter MIDI actions; g_side_deck[] tracks
 * which virtual deck each physical side currently routes to.
 *
 * Note on NS7III display orientation: the two center displays are "crossed" —
 *   Display Right ALSA card (dev_id=0x20) shows Deck A (left platter).
 *   Display Left  ALSA card (dev_id=0x10) shows Deck B (right platter).
 * This is confirmed by cold-init pcap captures of Serato sending to the hardware.
 */
static int g_side_deck[2] = { 0, 1 }; /* side→active deck index */
static uint8_t g_midi_last_status = 0;
static uint8_t g_midi_last_d1 = 0;
static uint8_t g_midi_last_d2 = 0;
static char g_midi_dev_str[64] = "";

/* ── Motor probe state ───────────────────────────────────────────────────
 * Interactive tool in the MIDI tab for discovering motor control messages.
 * Sends CC on a user-selected channel and logs the result. */
static int g_motor_probe_ch = 1;
static int g_motor_probe_cc = 65;
static int g_motor_probe_val = 0; /* value to send for stop probe         */
static int g_motor_probe_type = 0; /* 0=CC, 1=NoteOn, 2=NoteOff            */
static char g_motor_probe_log[256] = "Press M in MIDI tab to open motor probe";
static int g_motor_probe_open = 0;

/* ── MIDI monitor state ──────────────────────────────────────────────────
 * Captures raw incoming messages for the live monitor panel (J in MIDI IN tab).
 * Ring buffer of the last 16 messages, written from handle_midi before dispatch. */
#define MIDI_MON_SIZE 16
typedef struct {
	uint8_t status, d1, d2;
	int matched_act; /* MidiAction index, or -1 if unbound */
} MidiMonEntry;
static MidiMonEntry g_midi_mon_buf[MIDI_MON_SIZE];
static int g_midi_mon_head = 0; /* next write index (ring) */
static int g_midi_mon_count = 0; /* entries filled so far, max MIDI_MON_SIZE */
static int g_midi_mon_open = 0; /* 1 = panel visible in MIDI IN tab */

/* ── MIDI device list ────────────────────────────────────────────────────
 * Populated at startup and on demand.  Used for the device-picker in the
 * MIDI options tab so the user can choose which device to open when
 * multiple are present (e.g. NS7III screens vs NS7III controller). */
#define MIDI_MAX_DEVICES 16
typedef struct {
	char dev[64]; /* ALSA device string, e.g. "hw:1,0,0"   */
	char name[128]; /* human-readable name from rawmidi_info  */
} MidiDevEntry;
static MidiDevEntry g_midi_devlist[MIDI_MAX_DEVICES];
static int g_midi_ndevices = 0; /* count found          */
static int g_midi_dev_sel = 0; /* selected row in picker */

/* ── PCM (audio output) device list ─────────────────────────────────────
 * Populated at startup and on demand (R key in AUDIO tab).
 * Lets the user pick the audio output device from the options menu. */
#define PCM_MAX_DEVICES 16
typedef struct {
	char dev[64]; /* ALSA device string, e.g. "hw:0,0" or "default" */
	char name[128]; /* human-readable name                             */
} PcmDevEntry;
static PcmDevEntry g_pcm_devlist[PCM_MAX_DEVICES];
static int g_pcm_ndevices = 0;
static int g_pcm_dev_sel = 0;
static char g_pcm_dev_str[64] = CFG_PCM_DEVICE;
	/* active device */ /* e.g. "hw:1,0,0"           */

/* ── MIDI binding system ─────────────────────────────────────────────
 * Each binding maps a (status_byte, cc/note_number) pair → an action.
 * Status byte encodes both message type and channel:
 *   0xB0+ch = CC on channel ch,  0x90+ch = Note On on channel ch
 * The action enum covers everything handle_midi used to do via
 * hardcoded switch cases, plus new actions from the loaded map file.
 * ────────────────────────────────────────────────────────────────── */
typedef enum {
	/* per-deck faders — action + deck index gives the full binding */
	MACT_NONE = 0,
	MACT_DECK_VOL_A,
	MACT_DECK_VOL_B,
	MACT_DECK_VOL_C,
	MACT_DECK_VOL_D,
	MACT_DECK_PITCH_A,
	MACT_DECK_PITCH_B,
	MACT_DECK_PITCH_C,
	MACT_DECK_PITCH_D,
	/* 14-bit pitch fader LSB — bind to CC#33 same channel as pitch MSB.
     * Stored and combined with the MSB on next pitch message for higher resolution. */
	MACT_PITCH_LSB_A,
	MACT_PITCH_LSB_B,
	MACT_PITCH_LSB_C,
	MACT_PITCH_LSB_D,
	MACT_EQ_LOW_A,
	MACT_EQ_LOW_B,
	MACT_EQ_LOW_C,
	MACT_EQ_LOW_D,
	MACT_EQ_MID_A,
	MACT_EQ_MID_B,
	MACT_EQ_MID_C,
	MACT_EQ_MID_D,
	MACT_EQ_HIGH_A,
	MACT_EQ_HIGH_B,
	MACT_EQ_HIGH_C,
	MACT_EQ_HIGH_D,
	MACT_GAIN_A,
	MACT_GAIN_B,
	MACT_GAIN_C,
	MACT_GAIN_D,
	/* Filter knob per channel — absolute CC, 0=low-pass full, 64=flat, 127=high-pass full */
	MACT_FILTER_A,
	MACT_FILTER_B,
	MACT_FILTER_C,
	MACT_FILTER_D,
	/* Filter toggle button — Note On, toggles filter engage/bypass per deck.
     * When off the knob position is remembered but the filter is bypassed (flat). */
	MACT_FILTER_TOGGLE_A,
	MACT_FILTER_TOGGLE_B,
	MACT_FILTER_TOGGLE_C,
	MACT_FILTER_TOGGLE_D,
	MACT_CROSSFADER,
	MACT_MASTER_VOL,
	MACT_BOOTH_VOL, /* booth/monitor output level — CC */
	/* buttons / note-on triggers */
	MACT_PLAY_A,
	MACT_PLAY_B,
	MACT_PLAY_C,
	MACT_PLAY_D,
	/* Headphone Cue (PFL) toggle per deck */
	MACT_CUE_ACTIVE_A,
	MACT_CUE_ACTIVE_B,
	MACT_CUE_ACTIVE_C,
	MACT_CUE_ACTIVE_D,
	/* Hot cue set (records current position) and delete, 4 cues per deck */
	MACT_CUE_SET_1,
	MACT_CUE_SET_2,
	MACT_CUE_SET_3,
	MACT_CUE_SET_4,
	MACT_CUE_JUMP_1,
	MACT_CUE_JUMP_2,
	MACT_CUE_JUMP_3,
	MACT_CUE_JUMP_4,
	MACT_CUE_DELETE_1,
	MACT_CUE_DELETE_2,
	MACT_CUE_DELETE_3,
	MACT_CUE_DELETE_4,
	MACT_SYNC_SLAVE_A,
	MACT_SYNC_SLAVE_B,
	MACT_SYNC_SLAVE_C,
	MACT_SYNC_SLAVE_D,
	MACT_NUDGE_FWD,
	MACT_NUDGE_BACK,
	MACT_NUDGE_FWD_B,
	MACT_NUDGE_BACK_B, /* deck B nudge — routes to g_side_deck[1] regardless of active track */
	MACT_LOOP_TOGGLE,
	MACT_LOOP_IN_A,
	MACT_LOOP_IN_B,
	MACT_LOOP_IN_C,
	MACT_LOOP_IN_D,
	MACT_LOOP_OUT_A,
	MACT_LOOP_OUT_B,
	MACT_LOOP_OUT_C,
	MACT_LOOP_OUT_D,
	MACT_LOOP_DOUBLE_A,
	MACT_LOOP_DOUBLE_B,
	MACT_LOOP_DOUBLE_C,
	MACT_LOOP_DOUBLE_D,
	MACT_LOOP_HALF_A,
	MACT_LOOP_HALF_B,
	MACT_LOOP_HALF_C,
	MACT_LOOP_HALF_D,
	/* Per-deck toggles */
	MACT_KEY_LOCK_A,
	MACT_KEY_LOCK_B,
	MACT_KEY_LOCK_C,
	MACT_KEY_LOCK_D,
	MACT_SLIP_MODE_A,
	MACT_SLIP_MODE_B,
	MACT_SLIP_MODE_C,
	MACT_SLIP_MODE_D,
	MACT_REVERSE_A,
	MACT_REVERSE_B,
	MACT_REVERSE_C,
	MACT_REVERSE_D,
	/* Bleep — momentary slip+reverse (Note On = engage, Note Off = release + snap back).
     * While held: audio plays in reverse; on release position snaps back to where
     * it was when bleep was pressed, resuming forward playback. */
	MACT_BLEEP_A,
	MACT_BLEEP_B,
	MACT_BLEEP_C,
	MACT_BLEEP_D,
	/* Strip search — absolute CC: 0=track start, 127=track end (position seek) */
	MACT_STRIP_A,
	MACT_STRIP_B,
	MACT_STRIP_C,
	MACT_STRIP_D,
	/* Jog wheel — one pair per deck.
     * TOUCH: Note On = top touched (scratch mode), Note Off = released (nudge)
     * SPIN:  CC with relative value — two sub-modes:
     *   Absolute (0-127): center=64, >64=fwd, <64=back  (legacy potentiometer)
     *   Relative/encoder (2's complement or signed offset):
     *     64=no motion, 65-127=CW (fwd), 1-63=CCW (rev)  — high-res infinite encoder */
	MACT_JOG_TOUCH_A,
	MACT_JOG_TOUCH_B,
	MACT_JOG_TOUCH_C,
	MACT_JOG_TOUCH_D,
	MACT_JOG_SPIN_A,
	MACT_JOG_SPIN_B,
	MACT_JOG_SPIN_C,
	MACT_JOG_SPIN_D,
	/* Jog wheel pitch-bend signal — 14-bit 0xE0 message, motor-running velocity stream.
     * Separate from JOG_SPIN so both CC and pitch-bend can be bound simultaneously.
     * Map file: jog_pb_a  E1  0   (status = 0xE0|ch, data1 always 0) */
	MACT_JOG_PB_A,
	MACT_JOG_PB_B,
	MACT_JOG_PB_C,
	MACT_JOG_PB_D,
	/* Library / browser navigation encoder — infinite relative encoder
     * Same relative CC encoding as jog spin: 64=center, >64=down, <64=up */
	MACT_LIB_ENCODER,
	/* Touch sensor on library encoder knob (Note On/Off).
     * On NS7III there is no hardware touch note for the browse knob;
     * view switching is driven by scroll activity instead.  Other controllers
     * that DO send a touch note can bind it here for explicit control. */
	MACT_LIB_ENCODER_TOUCH,
	/* Library select/load buttons — mirrors keyboard ! @ # $ */
	MACT_LIB_SELECT, /* press = enter directory (files: do nothing) */
	MACT_LIB_BACK, /* press = go up one directory (like BACKSPACE) */
	MACT_LIB_FWD, /* press = enter highlighted directory (like ENTER on dir) */
	MACT_LIB_LOAD_A, /* load to deck A */
	MACT_LIB_LOAD_B, /* load to deck B */
	MACT_LIB_LOAD_C, /* load to deck C */
	MACT_LIB_LOAD_D, /* load to deck D */
	/* Panel switchers — switch the library pane shown in split view.
	 * Works in both 2-deck and 4-deck view (forces split view if needed). */
	MACT_PANEL_FILES,    /* switch to file browser (g_panel=0) */
	MACT_PANEL_LIBRARY,  /* switch to library/crates pane (g_panel=2) */
	/* Pitch range cycle — Note On cycles ±8% → ±25% → ±50% → ±8% */
	MACT_PITCH_RANGE_A,
	MACT_PITCH_RANGE_B,
	MACT_PITCH_RANGE_C,
	MACT_PITCH_RANGE_D,
	/* Per-deck MIDI pitch bend (CC, relative encoder, same encoding as jog spin)
     * Applied as a temporary nudge that decays, identical to ] / [ keyboard nudge */
	MACT_PITCH_BEND_A,
	MACT_PITCH_BEND_B,
	MACT_PITCH_BEND_C,
	MACT_PITCH_BEND_D,
	/* Motor control — independent of playback.
     * Toggle: bind to deck select button (single press on/off).
     * On/Off: separate bindings for explicit control. */
	MACT_MOTOR_TOGGLE_A,
	MACT_MOTOR_TOGGLE_B,
	MACT_MOTOR_TOGGLE_C,
	MACT_MOTOR_TOGGLE_D,
	MACT_MOTOR_ON_A,
	MACT_MOTOR_ON_B,
	MACT_MOTOR_ON_C,
	MACT_MOTOR_ON_D,
	MACT_MOTOR_OFF_A,
	MACT_MOTOR_OFF_B,
	MACT_MOTOR_OFF_C,
	MACT_MOTOR_OFF_D,
	/* Deck select buttons — [1][3] left side, [2][4] right side.
     * Pressing selects which software deck the physical side controls.
     * LED for active deck lights up, inactive dims. */
	MACT_DECK_SEL_1,
	MACT_DECK_SEL_2,
	MACT_DECK_SEL_3,
	MACT_DECK_SEL_4,
	/* SHIFT button — sets shift modifier state for pads/FX */
	MACT_SHIFT_A,
	MACT_SHIFT_B,
	/* Pitch center button — resets pitch to 0% */
	MACT_PITCH_CENTER_A,
	MACT_PITCH_CENTER_B,
	/* CUE button — standard CDJ/Serato cue behaviour */
	MACT_CUE_DEFAULT_A,
	MACT_CUE_DEFAULT_B,
	/* Pad mode select buttons — one per side (deck A/B) */
	MACT_PAD_MODE_CUES_A,
	MACT_PAD_MODE_CUES_B,
	MACT_PAD_MODE_AUTOROLL_A,
	MACT_PAD_MODE_AUTOROLL_B,
	/* Manual loop pad mode — pads 1-5 become loop_in/out/halve/double/reloop */
	MACT_PAD_MODE_MANUAL_A,
	MACT_PAD_MODE_MANUAL_B,
	/* Performance pads 1-8 per deck (Note On = press, Note Off = release) */
	MACT_PAD_1_A,
	MACT_PAD_2_A,
	MACT_PAD_3_A,
	MACT_PAD_4_A,
	MACT_PAD_5_A,
	MACT_PAD_6_A,
	MACT_PAD_7_A,
	MACT_PAD_8_A,
	MACT_PAD_1_B,
	MACT_PAD_2_B,
	MACT_PAD_3_B,
	MACT_PAD_4_B,
	MACT_PAD_5_B,
	MACT_PAD_6_B,
	MACT_PAD_7_B,
	MACT_PAD_8_B,
	/* Parameter left/right buttons — resize autoloop while in autoloop/roll mode */
	MACT_PARAM_LEFT_A,
	MACT_PARAM_RIGHT_A,
	MACT_PARAM_LEFT_B,
	MACT_PARAM_RIGHT_B,
	/* FX controls — per deck side */
	/* Buttons: cycle effect type in slot 0 (btn1), slot 1 (btn2), master (btn3) */
	MACT_FX_BTN_1_A,
	MACT_FX_BTN_2_A,
	MACT_FX_BTN_3_A,
	MACT_FX_BTN_1_B,
	MACT_FX_BTN_2_B,
	MACT_FX_BTN_3_B,
	/* Knobs: params 0-2 for currently-selected FX slot */
	MACT_FX_KNOB_1_A,
	MACT_FX_KNOB_2_A,
	MACT_FX_KNOB_3_A,
	MACT_FX_KNOB_1_B,
	MACT_FX_KNOB_2_B,
	MACT_FX_KNOB_3_B,
	/* Beat/depth knob: wet/dry for currently-selected slot */
	MACT_FX_WET_A,
	MACT_FX_WET_B,
	MACT_CF_CURVE, /* crossfader curve adjustment — CC */
	MACT_TAP_BPM_A,
	MACT_TAP_BPM_B,
	MACT_GRID_SNAP_A,
	MACT_GRID_SNAP_B,
	MACT_COUNT /* keep last */
} MidiAction;

/* Human-readable name for each action (used in UI and map file) */
static const char *g_mact_names[MACT_COUNT] = {
	"none",
	"vol_a",
	"vol_b",
	"vol_c",
	"vol_d",
	"pitch_a",
	"pitch_b",
	"pitch_c",
	"pitch_d",
	"pitch_lsb_a",
	"pitch_lsb_b",
	"pitch_lsb_c",
	"pitch_lsb_d",
	"eq_low_a",
	"eq_low_b",
	"eq_low_c",
	"eq_low_d",
	"eq_mid_a",
	"eq_mid_b",
	"eq_mid_c",
	"eq_mid_d",
	"eq_high_a",
	"eq_high_b",
	"eq_high_c",
	"eq_high_d",
	"gain_a",
	"gain_b",
	"gain_c",
	"gain_d",
	"filter_a",
	"filter_b",
	"filter_c",
	"filter_d",
	"filter_toggle_a",
	"filter_toggle_b",
	"filter_toggle_c",
	"filter_toggle_d",
	"crossfader",
	"master_vol",
	"booth_vol",
	"play_a",
	"play_b",
	"play_c",
	"play_d",
	"cue_active_a",
	"cue_active_b",
	"cue_active_c",
	"cue_active_d",
	"cue_set_1",
	"cue_set_2",
	"cue_set_3",
	"cue_set_4",
	"cue_1",
	"cue_2",
	"cue_3",
	"cue_4",
	"cue_del_1",
	"cue_del_2",
	"cue_del_3",
	"cue_del_4",
	"sync_a",
	"sync_b",
	"sync_c",
	"sync_d",
	"nudge_fwd",
	"nudge_back",
	"nudge_fwd_b",
	"nudge_back_b",
	"loop",
	"loop_in_a",
	"loop_in_b",
	"loop_in_c",
	"loop_in_d",
	"loop_out_a",
	"loop_out_b",
	"loop_out_c",
	"loop_out_d",
	"loop_double_a",
	"loop_double_b",
	"loop_double_c",
	"loop_double_d",
	"loop_half_a",
	"loop_half_b",
	"loop_half_c",
	"loop_half_d",
	"key_lock_a",
	"key_lock_b",
	"key_lock_c",
	"key_lock_d",
	"slip_a",
	"slip_b",
	"slip_c",
	"slip_d",
	"reverse_a",
	"reverse_b",
	"reverse_c",
	"reverse_d",
	"bleep_a",
	"bleep_b",
	"bleep_c",
	"bleep_d",
	"strip_a",
	"strip_b",
	"strip_c",
	"strip_d",
	"jog_touch_a",
	"jog_touch_b",
	"jog_touch_c",
	"jog_touch_d",
	"jog_spin_a",
	"jog_spin_b",
	"jog_spin_c",
	"jog_spin_d",
	"jog_pb_a",
	"jog_pb_b",
	"jog_pb_c",
	"jog_pb_d",
	"lib_encoder",
	"lib_encoder_touch",
	"lib_select",
	"lib_back",
	"lib_fwd",
	"lib_load_a",
	"lib_load_b",
	"lib_load_c",
	"lib_load_d",
	"panel_files",
	"panel_library",
	"pitch_range_a",
	"pitch_range_b",
	"pitch_range_c",
	"pitch_range_d",
	"pitch_bend_a",
	"pitch_bend_b",
	"pitch_bend_c",
	"pitch_bend_d",
	"motor_toggle_a",
	"motor_toggle_b",
	"motor_toggle_c",
	"motor_toggle_d",
	"motor_on_a",
	"motor_on_b",
	"motor_on_c",
	"motor_on_d",
	"motor_off_a",
	"motor_off_b",
	"motor_off_c",
	"motor_off_d",
	"deck_sel_1",
	"deck_sel_2",
	"deck_sel_3",
	"deck_sel_4",
	"shift_a",
	"shift_b",
	"pitch_center_a",
	"pitch_center_b",
	"cue_default_a",
	"cue_default_b",
	"pad_mode_cues_a",
	"pad_mode_cues_b",
	"pad_mode_autoroll_a",
	"pad_mode_autoroll_b",
	"pad_mode_manual_a",
	"pad_mode_manual_b",
	"pad_1_a",
	"pad_2_a",
	"pad_3_a",
	"pad_4_a",
	"pad_5_a",
	"pad_6_a",
	"pad_7_a",
	"pad_8_a",
	"pad_1_b",
	"pad_2_b",
	"pad_3_b",
	"pad_4_b",
	"pad_5_b",
	"pad_6_b",
	"pad_7_b",
	"pad_8_b",
	"param_left_a",
	"param_right_a",
	"param_left_b",
	"param_right_b",
	"fx_btn_1_a",
	"fx_btn_2_a",
	"fx_btn_3_a",
	"fx_btn_1_b",
	"fx_btn_2_b",
	"fx_btn_3_b",
	"fx_knob_1_a",
	"fx_knob_2_a",
	"fx_knob_3_a",
	"fx_knob_1_b",
	"fx_knob_2_b",
	"fx_knob_3_b",
	"fx_wet_a",
	"fx_wet_b",
	"cf_curve",
	"tap_bpm_a",
	"tap_bpm_b",
	"grid_snap_a",
	"grid_snap_b",
};

typedef struct {
	uint8_t status; /* 0 = unbound */
	uint8_t data1; /* CC number or note number */
	MidiAction action;
	uint8_t relative; /* 1 = center-64 relative CC encoder (data2>64=+, data2<64=-) */
	float rel_acc; /* accumulated position [0.0, 1.0] for relative mode */
} MidiBinding;

/* Output binding: djcmd sends this message to the controller.
 * Used for motor start/stop, LED feedback, RGB pads, etc.
 * Format in map file:
 *   out  name  status_hex  data1_dec  data2_dec
 *   rgb  name  r_dec  g_dec  b_dec   (NS7III SysEx RGB — see below)
 */
typedef struct {
	char name[32];
	uint8_t status; /* 0 = not configured; 0xF0 = SysEx mode */
	uint8_t data1;
	uint8_t data2;
	uint8_t sysex[16]; /* full SysEx payload including F0 and F7  */
	uint8_t sysex_len; /* 0 = not SysEx                           */
} MidiOutBinding;

#define MIDI_MAX_BINDINGS 256
#define MIDI_MAX_OUT_BINDINGS 256 /* output bindings: LEDs, motors, RGB pads */
static MidiBinding g_midi_bindings[MIDI_MAX_BINDINGS];
static int g_midi_nbindings = 0;
static MidiOutBinding g_midi_out_bindings[MIDI_MAX_OUT_BINDINGS];
static int g_midi_nout_bindings = 0;

/* Learn mode state */
static volatile int g_midi_learn_active =
	0; /* 1 = waiting for hardware wiggle */
static volatile int g_midi_learn_sel =
	0; /* which action row is being learned */

/* Jog paired-learn state.
 * Jog wheels need three bindings: CC spin, pitch bend (0xE0), and touch (Note On).
 * When g_midi_learn_jog_pair is set, all three are captured from one gesture
 * (touch the platter and spin it with the motor on). Any subset can be committed
 * early via ENTER/W.
 * g_midi_learn_jog_deck tracks which deck (0=A … 3=D) the pair is for.
 * g_midi_learn_jog_spin_status/d1 hold the captured CC spin for display.
 * g_midi_learn_jog_step is a bitmask: bit0=touch captured, bit1=PB captured. */
static volatile int g_midi_learn_jog_pair = 0; /* 1 = in paired jog learn */
static volatile int g_midi_learn_jog_step = 0; /* bitmask: 1=touch, 2=PB  */
static volatile int g_midi_learn_jog_deck = 0; /* deck 0-3 */
static volatile uint8_t g_midi_learn_jog_spin_status = 0;
static volatile uint8_t g_midi_learn_jog_spin_d1 = 0;

/* ── Jog wheel / scratch state ────────────────────────────────────────
 * The NS7III sends 3600 ticks per revolution as relative CC values:
 *   data2 = 64 → center (no motion)
 *   data2 > 64 → forward (clockwise),  delta = data2 - 64
 *   data2 < 64 → reverse (CCW),        delta = data2 - 64 (negative)
 * Touch sensor sends Note On (touch) / Note Off (release).
 *
 * Scratch mode (top touched):
 *   Position is nudged by delta * SCRATCH_FRAMES_PER_TICK frames directly.
 *   While touching, motor feedback keeps platter at track speed.
 *
 * Nudge mode (edge, not touching):
 *   Pitch is temporarily bent by delta * NUDGE_PITCH_PER_TICK, then decays.
 */
#define SCRATCH_FRAMES_PER_TICK 8 /* frame offset per jog tick in scratch */
#define NUDGE_PITCH_PER_TICK 0.004f /* pitch bend per jog tick in nudge    */

/* Display deck number as 1–4 instead of A–D to match NS7III [1][2][3][4] buttons */
#define DECK_NUM(d) ('1' + (d)) /* deck 0='1', 1='2', 2='3', 3='4' */
/* NUDGE_DECAY is defined in djcmd_config.h via CFG_NUDGE_DECAY — reused here */

static int g_jog_touched[MAX_TRACKS] = { 0 }; /* 1 = top surface touched  */
static volatile float g_jog_nudge[MAX_TRACKS] = { 0 };
static float g_last_applied_nudge[MAX_TRACKS] = { 0 };
static float g_last_motor_vel[MAX_TRACKS] = { 0 };
/* Per-deck scratch vinyl character filter state (two-pole cascade IIR) */
static float g_scratch_lpf_l[MAX_TRACKS]  = { 0 }; /* stage 1 L */
static float g_scratch_lpf_r[MAX_TRACKS]  = { 0 }; /* stage 1 R */
static float g_scratch_lpf2_l[MAX_TRACKS] = { 0 }; /* stage 2 L */
static float g_scratch_lpf2_r[MAX_TRACKS] = { 0 }; /* stage 2 R */
static float g_scratch_alpha[MAX_TRACKS] = { 0 }; /* smoothed cutoff */
/* LCG noise state for per-deck needle noise during scratch */
static uint32_t g_noise_state[MAX_TRACKS] = { 12345u, 67890u, 11111u, 22222u };
/* Brown noise integrator: one-pole LP applied to white noise for warmer texture */
static float g_noise_brown_l[MAX_TRACKS] = { 0.0f };
static float g_noise_brown_r[MAX_TRACKS] = { 0.0f };
/* Jog tuning — settable via map file */
static float g_jog_smooth_alpha = 0.30f;
static float g_jog_dead_band = 0.08f;
static float g_jog_spike_thresh = 0.80f;
static float g_jog_slew_rate = 0.04f;
static int g_jog_settle_ms = 2000; /* ms to ignore jog after motor start */
/* Jog streak state — per-deck, reset by settle timer */
static int g_pb_streak[MAX_TRACKS] = { 0 };
static float g_pb_mag_acc[MAX_TRACKS] = { 0.0f };

/* ── Jog wheel encoder type ──────────────────────────────────────────────
 * 0 = JOG_RELATIVE  — standard center-64 relative CC (most controllers)
 * 1 = JOG_NS7III    — NS7III absolute encoder:
 *       CC  d1=0, d2=0-127  : coarse 7-bit absolute angle (128 steps/rev)
 *       PB  (0xE0)          : fine 14-bit sub-step within each coarse step
 *     Together: pos = (coarse*16384 + fine) / (128*16384), 0.0-1.0/rev
 *     Velocity derived by differencing consecutive positions with wraparound.
 * Set in map file:  set  jog_type  ns7iii   (or: relative) */
#define JOG_RELATIVE 0
#define JOG_NS7III 1
static int g_jog_type = JOG_RELATIVE;

/* Reference delta for NS7III motor velocity formula.
 * This is the expected g_jog_abs_vel value when the platter spins freely
 * at reference speed (motor on, hand off, no pitch change).
 *
 * At 33 rpm with 128 coarse steps/rev and one PB message per coarse step:
 *   msg_rate  = 33/60 × 128 = 70.4 msgs/sec
 *   ref_delta = 1/128 = 0.0078125 rev/msg
 *
 * If the NS7III sends multiple PB messages per coarse step, msg_rate is
 * higher and ref_delta is smaller accordingly.
 *
 * Tunable via map file:
 *   set  jog_ref_delta  0.0078125   (set directly — preferred)
 *   set  jog_msg_rate   70          (computes ref_delta = 1/(128×msg_rate/msg_rate))
 *
 * If playback runs fast with motor on, lower jog_ref_delta.
 * If playback runs slow, raise jog_ref_delta.
 * The MIDI monitor shows live pos= and vel= values — at on-speed with hand
 * off the platter, the vel= value IS the current ref_delta to use here. */
static float g_jog_ref_delta =
	0.0078125f; /* 1/128 rev/msg at 33rpm, 1 PB/step */

/* Dead band around jog_ref_delta for motor velocity.
 * Expressed as a fraction of jog_ref_delta (0.0 = none, 0.1 = ±10%).
 * Motor speed variance within this band is treated as on-speed → motor_vel = 0.
 * Widens the "no pitch change" zone to absorb motor inconsistency.
 * Tunable: set  jog_motor_dead  0.05   in map file. */
static float g_jog_motor_dead = 0.05f; /* ±5% of ref_delta by default */

/* Maximum motor velocity deviation (raw_vel clamp).
 * raw_vel = (delta / ref_delta) - 1.0, so:
 *   raw_vel = +3.0 → step = pitch(1.0) + 3.0 = 4.0× forward
 *   raw_vel = -3.0 → step = pitch(1.0) - 3.0 = -2.0× (reverse at 2× speed)
 * Raise for faster spins/backspins. Default 10.0 = effectively unclamped
 * for normal DJ use. Tune down if you want to limit maximum scratch speed.
 * Set via map: set  jog_vel_max  10.0 */
static float g_jog_vel_max = 15.0f;
#define JOG_NS7III_STEPS 128 /* coarse steps per revolution          */
#define JOG_NS7III_FINE_RANGE 16384 /* pitch bend range (14-bit, 0-16383)   */
/* Full ticks per revolution = STEPS * FINE_RANGE = 2,097,152                 */
/* Scratch scale: frames of audio to move per full revolution.
 * At 44100 Hz, 33 1/3 rpm = 1 rev / 1.8 s = 79380 frames/rev.
 * At 44100 Hz, 45 rpm = 1 rev / 1.333 s = 58800 frames/rev. */
static float g_jog_scratch_revs =
	79380.0f; /* frames per revolution (33 1/3 rpm @ 44.1k) */

static int g_jog_coarse[MAX_TRACKS] = { -1, -1, -1,
					-1 }; /* last CC0 value, -1=uninit */
static int g_jog_fine[MAX_TRACKS] = { 8192, 8192, 8192,
				      8192 }; /* last PB raw14 */
/* NS7III slip detection — PB-based touch inference (no hardware touch note) */
#define NS7III_PB_RATIO 1440 /* PB ticks per coarse step at reference speed */
#define NS7III_SLIP_THRESH 200 /* slipError threshold to declare touch */
#define NS7III_RELEASE_CONF \
	3 /* consecutive sync packets needed to release touch */
static int g_jog_last_pb[MAX_TRACKS] = { 8192, 8192, 8192, 8192 };
static int g_jog_release_conf[MAX_TRACKS] = { 0 };
static int g_jog_last_coarse_d[MAX_TRACKS] = { 0 }; /* last coarse CC delta */
static int64_t g_jog_last_spin_ms[MAX_TRACKS] = {
	0
}; /* ms of last coarse update */
static float g_jog_abs_pos[MAX_TRACKS] = {
	0
}; /* last reconstructed position 0.0-1.0 */
static int g_jog_abs_init[MAX_TRACKS] = {
	0
}; /* 1 = have valid last position */
static volatile float g_jog_abs_vel[MAX_TRACKS] = {
	0
}; /* smoothed velocity frac-of-rev/msg — written by MIDI + audio threads */
/* Timestamp (ms) of last encoder message per deck — used by audio thread
 * to detect genuine platter stop vs inter-message gap */
static volatile int64_t g_jog_last_msg_ms[MAX_TRACKS] = { 0 };

/* ── PLL jitter filter for motorised platters ────────────────────────────
 * Optional replacement for the EMA + dead-band motor noise filter.
 * Enabled per-map via:  set  pll_enabled  1
 *
 * A PI loop tracks the platter's actual free-running frequency.  The
 * integrator absorbs DC motor offset (wow/flutter); the proportional term
 * responds to transient noise.  When the phase error (measured delta minus
 * NCO frequency estimate) stays within the bandwidth the output is zeroed —
 * motor noise is cancelled.  When the error exceeds the bandwidth, a hand
 * on the platter is assumed and the full deviation is passed through.
 *
 * Tunable via map file:
 *   set  pll_enabled    1      — 0=EMA (default), 1=PLL
 *   set  pll_kp         0.10   — proportional gain
 *   set  pll_ki         0.003  — integral gain
 *   set  pll_bandwidth  0.30   — lock threshold as fraction of ref_delta
 *                                 smaller = tighter null, more noise immunity
 *                                 larger  = faster touch detection
 */
typedef struct {
	double freq; /* NCO frequency estimate (revolutions per message) */
	double integrator; /* PI integrator accumulator */
} PLLState;

static PLLState g_pll[MAX_TRACKS];
static int g_pll_enabled = 0; /* 0 = EMA, 1 = PLL */
static float g_pll_kp = 0.10f; /* proportional gain */
static float g_pll_ki = 0.003f; /* integral gain */
static float g_pll_bandwidth = 0.30f; /* lock threshold (× ref_delta) */

/* 14-bit pitch fader LSB storage — updated by MACT_PITCH_LSB_* CC messages */
static uint8_t g_pitch_lsb[MAX_TRACKS] = { 0 };

/* Pitch range per deck — cycles: 0=±8%, 1=±25%, 2=±50%
 * This controls how far the MIDI pitch fader spans from center.
 * Keyboard pitch keys (e/d/E/D) are unaffected and still step freely. */
static int g_pitch_range[MAX_TRACKS] = { 0, 0, 0,
					 0 }; /* 0=±8%, 1=±25%, 2=±50% */
/* Pitch range values as multipliers from center (1.0).
 * Center = 1.0, range ±8% = [0.92, 1.08], ±25% = [0.75, 1.25], ±50% = [0.50, 1.50] */
static const float g_pitch_range_vals[3] = { 0.08f, 0.25f, 0.50f };
static const char *g_pitch_range_names[3] = { "±8%", "±25%", "±50%" };

/* UI state */
static int g_active_track = 0;
static int g_view = 1; /* 0=decks, 1=browser+panel, 2=help */
static int g_panel = 0; /* bottom panel: 0=browser, 1=playlist */
/* Library browse auto-view-switch:
 * When in 4-deck view (g_view==0) the browser is hidden.  Scrolling the
 * library encoder or touching it (if bound) temporarily switches to split
 * view so the selection is visible, then reverts after 1 s of inactivity. */
static int64_t g_lib_enc_last_ms = 0; /* ms of last lib_encoder scroll */
static int g_lib_auto_switched = 0; /* 1 = view was auto-set to 1 */
int g_help_scroll = 0; /* first visible line of help page */
static int g_options_open = 0; /* 1 = options overlay showing */
static int g_quit_pending = 0; /* 1 = quit confirm modal active */
static int g_options_tab =
	0; /* 0=Info 1=Audio 2=Display 3=Waveform 4=Sync 5=Theme 6=MIDI-IN 7=MIDI-OUT */
static int g_options_sel =
	0; /* selected row in current tab (MIDI IN: binding index) */
static int g_options_out_sel =
	0; /* selected row in MIDI OUT tab (output binding index) */

/* Manual BPM entry mode (activated by 'B') */
static int g_bpm_entry = 0; /* 1 = collecting BPM digits */
static char g_bpm_buf[8] = ""; /* digit string being typed */
static int g_bpm_deck = 0; /* which deck is being edited */

/* Info-tab display cache ─────────────────────────────────────────────────
 * CPU string: read once at startup from /proc/cpuinfo (never changes).
 * Memory string: re-read at most once per second when the Info tab is open.
 * Both are static so draw_options_overlay() never opens /proc on every frame. */
static char g_cpuinfo_cache[256] =
	""; /* filled by options_read_cpuinfo() at init */
static char g_meminfo_cache[128] = ""; /* refreshed at most 1 Hz */
static time_t g_meminfo_last_t = 0; /* last refresh timestamp */

/* Runtime-adjustable options (loaded from config, saved on exit) */
typedef struct {
	int default_master_vol; /* 0–100 */
	float default_deck_vol; /* 0.0–1.0 */
	int auto_gain_default; /* 0=off, 1=on */
	float auto_gain_target_db; /* dBFS */
	float wfm_visible_secs; /* seconds visible in scrolling waveform */
	int wfm_overview_bins; /* 2048 / 4096 / 8192 */
	float kick_threshold; /* low-band dominance ratio for red highlight */
	float wfm_height_gamma; /* 0.5=sqrt, 1.0=linear */
	int theme_idx; /* index into g_themes[] */
	int wfm_style; /* 0=Kick/Snare (red/blue), 1=Theme (theme colours) */
	int sync_quantize; /* 1=quantize play to next bar-1 beat of master */
	int sync_smart_range; /* 1=prevent BPM octave jumps on sync (e.g. 90→175) */
	int sync_auto_handoff; /* 1=if master deck reloaded, hand master to playing deck */
	int key_lock_default; /* 1=new tracks load with key lock ON */
	int vinyl_mode; /* 1=playhead detaches from motor when untouched (motorised platters only) */
	int ui_fps; /* UI redraw rate: 5–60 in steps of 5 */
	/* ── Advanced Waveform ── */
	float wfm_lo_weight; /* height contribution of lo  band (0.1–2.0, default 0.60) */
	float wfm_mid_weight; /* height contribution of mid band (0.1–2.0, default 0.40) */
	float wfm_hi_weight; /* height contribution of hi  band (0.0–1.0, default 0.15) */
	float wfm_color_sat; /* colour saturation multiplier (0.2–3.0, default 1.0) */
	float wfm_color_floor; /* min colour brightness per column (0.0=black 0.15=always lit) */
	int wfm_anchor; /* 0=centred (Serato style), 1=bottom-anchored (Rekordbox) */
} Options;

static Options g_opts = {
	.default_master_vol = CFG_DEFAULT_MASTER_VOL,
	.default_deck_vol = CFG_DEFAULT_DECK_VOL,
	.auto_gain_default = CFG_AUTO_GAIN_ENABLED,
	.auto_gain_target_db = CFG_AUTO_GAIN_TARGET,
	.wfm_visible_secs = CFG_WFM_VISIBLE_SECS,
	.wfm_overview_bins = CFG_WFM_OVERVIEW_BINS,
	.kick_threshold = CFG_KICK_THRESHOLD,
	.wfm_height_gamma = CFG_WFM_HEIGHT_GAMMA,
	.theme_idx = CFG_DEFAULT_THEME,
	.wfm_style = 0, /* 0=Spectrum (solid bars, hue gradient) */
	.sync_quantize = 1, /* quantize play to bar-1 by default */
	.sync_smart_range = 1, /* prevent octave BPM jumps by default */
	.sync_auto_handoff =
		1, /* auto-handoff master when master deck reloaded */
	.key_lock_default = 0, /* key lock OFF by default — costs CPU */
	.vinyl_mode = 1, /* detached playhead ON by default for motorised platters */
	.ui_fps = 20, /* 20 FPS default (50 ms wtimeout) */
	.wfm_lo_weight = 0.60f,
	.wfm_mid_weight = 0.40f,
	.wfm_hi_weight = 0.15f,
	.wfm_color_sat = 1.0f,
	.wfm_color_floor = 0.06f,
	.wfm_anchor = 0,
};

/* ──────────────────────────────────────────────
   Session Mix Log
   Appends one line per track load to a dated log
   file in ~/.config/djcmd/.  Zero runtime cost —
   all I/O happens in the load-worker thread, not
   the audio thread.
   ────────────────────────────────────────────── */
static FILE *g_mixlog = NULL; /* open log file, NULL = disabled */
static char g_mixlog_path[512] = ""; /* path for display in UI          */

/* Per-deck: wall-clock second when the current track was loaded.
 * 0 = no track has been logged on this deck yet this session. */
static time_t g_mixlog_load_time[MAX_TRACKS] = { 0, 0, 0, 0 };

/* Snapshot of tag_title/tag_artist at log time (read under lt->lock,
 * written in load_worker before the log call). */
static char g_mixlog_title[MAX_TRACKS][128];
static char g_mixlog_artist[MAX_TRACKS][128];
static char g_mixlog_file[MAX_TRACKS][MAX_FILENAME];
static float g_mixlog_bpm[MAX_TRACKS];

/* Sync master — index of the deck others lock to (-1 = none) */
static int g_sync_master = -1;

/* Gang mode — bitmask of decks that receive gang commands */
static int g_gang_mask = 0; /* bit 0=A, 1=B, 2=C, 3=D */
static int g_gang_mode = 0; /* 0=off, 1=on             */

/* Nudge decay rate per audio period (~11ms) — 2% pitch for ~200ms */
/* Nudge decay — see djcmd_config.h for CFG_NUDGE_* values */

WINDOW *g_win_main = NULL;
static WINDOW *g_win_status = NULL;
int g_rows, g_cols;

/* ──────────────────────────────────────────────
   File Browser State
   ────────────────────────────────────────────── */
/* Browser and playlist limits — see djcmd_config.h */

typedef struct {
	char name[256]; /* entry name (basename)      */
	int is_dir; /* 1 = directory, 0 = file    */
	float bpm; /* 0 = unknown / not in Mixxx */
	char tag_title[128];
	char tag_artist[128];
} FBEntry;

static char g_fb_path[FB_PATH_MAX] = "."; /* current directory   */
static FBEntry g_fb_entries[FB_MAX_ENTRIES];
static int g_fb_count = 0;
static int g_fb_sel = 0; /* cursor row          */
static int g_fb_scroll = 0; /* top visible row     */
static char g_fb_status[256] = ""; /* e.g. "Loaded → A"  */
/* 0=name, 1=BPM asc, 2=BPM desc */
static int g_fb_sort = 0;

/* ──────────────────────────────────────────────
   Library — recursive scan of a root directory.
   Stores ALL audio files found in the tree.
   Accessed via g_panel == 2.
   ────────────────────────────────────────────── */
#define LIB_MAX 8192

typedef struct {
	char path[FB_PATH_MAX + 256]; /* full path */
	char name[256]; /* basename */
	float bpm;
	char tag_title[128];
	char tag_artist[128];
} LIBEntry;

static LIBEntry *g_lib = NULL; /* heap-allocated LIB_MAX entries */
static int g_lib_count = 0;
static int g_lib_sel = 0;
static int g_lib_scroll = 0;
static char g_lib_root[FB_PATH_MAX] = ""; /* root scanned (empty=none) */
static int g_lib_sort = 0; /* 0=name, 1=BPM asc, 2=BPM desc */
static volatile int g_lib_scanning = 0; /* 1 while scan thread running */

/* ──────────────────────────────────────────────
   Playlist State
   Simple ordered queue of full paths the user adds
   with 'p' while browsing. Loadable with !@#$/ENTER.
   ────────────────────────────────────────────── */
/* Playlist — see djcmd_config.h */

typedef struct {
	char path[FB_PATH_MAX + 256]; /* full path */
	char name[256]; /* basename for display */
	float bpm; /* from mixxxdb, 0=unknown */
} PLEntry;

static PLEntry g_pl[PL_MAX];
static int g_pl_count = 0;
static int g_pl_sel = 0;
static int g_pl_scroll = 0;

/* ──────────────────────────────────────────────
   Tag Info Panel
   Shown when user presses 'i' on a browser entry.
   We query MusicBrainz via HTTP (libcurl) using the
   filename as a best-effort search term. Falls back
   to "no network / no curl" gracefully.
   ────────────────────────────────────────────── */
typedef struct {
	char artist[128];
	char title[128];
	char album[128];
	char date[32];
	char label[128];
	char status[64]; /* "Searching…", "Not found", "No network", etc. */
	int visible; /* 1 = panel shown */
	char query_name[256]; /* filename being looked up */
} TagInfo;

static TagInfo g_tag_info;

/* ──────────────────────────────────────────────
   Background Load Queue
   One worker thread; UI enqueues a path+deck and
   the worker calls load_track + BPM analysis so
   the audio thread and UI never stall.
   ────────────────────────────────────────────── */
typedef struct {
	char path[FB_PATH_MAX + 256];
	int deck;
	int valid; /* 1 = job waiting */
} LoadJob;

static LoadJob g_load_job;
static pthread_mutex_t g_load_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_load_cond = PTHREAD_COND_INITIALIZER;

/* ──────────────────────────────────────────────
   Biquad EQ Coefficients (44100 Hz)
   ────────────────────────────────────────────── */
/* Pre-computed Butterworth 2nd-order coefficients */
static float g_lp_a[3], g_lp_b[3]; /* 300 Hz low-pass  */
static float g_bp_a[3], g_bp_b[3]; /* 1k–5k band-pass  */
static float g_hp_a[3], g_hp_b[3]; /* 5k Hz high-pass  */

static void biquad_lowpass(float fc, float *b, float *a)
{
	float w0 = 2.0f * M_PI * fc / g_actual_sample_rate;
	float cosw = cosf(w0), sinw = sinf(w0);
	float q = 0.707f;
	float alpha = sinw / (2.0f * q);
	float b0 = (1.0f - cosw) / 2.0f;
	float b1 = 1.0f - cosw;
	float b2 = (1.0f - cosw) / 2.0f;
	float a0 = 1.0f + alpha;
	float a1 = -2.0f * cosw;
	float a2 = 1.0f - alpha;
	b[0] = b0 / a0;
	b[1] = b1 / a0;
	b[2] = b2 / a0;
	a[0] = 1.0f;
	a[1] = a1 / a0;
	a[2] = a2 / a0;
}

static void biquad_highpass(float fc, float *b, float *a)
{
	float w0 = 2.0f * M_PI * fc / g_actual_sample_rate;
	float cosw = cosf(w0), sinw = sinf(w0);
	float q = 0.707f;
	float alpha = sinw / (2.0f * q);
	float b0 = (1.0f + cosw) / 2.0f;
	float b1 = -(1.0f + cosw);
	float b2 = (1.0f + cosw) / 2.0f;
	float a0 = 1.0f + alpha;
	float a1 = -2.0f * cosw;
	float a2 = 1.0f - alpha;
	b[0] = b0 / a0;
	b[1] = b1 / a0;
	b[2] = b2 / a0;
	a[0] = 1.0f;
	a[1] = a1 / a0;
	a[2] = a2 / a0;
}

static void biquad_bandpass(float fc, float *b, float *a)
{
	float w0 = 2.0f * M_PI * fc / g_actual_sample_rate;
	float sinw = sinf(w0), cosw = cosf(w0);
	float q = 1.0f;
	float alpha = sinw / (2.0f * q);
	float b0 = sinw / 2.0f;
	float b1 = 0.0f;
	float b2 = -sinw / 2.0f;
	float a0 = 1.0f + alpha;
	float a1 = -2.0f * cosw;
	float a2 = 1.0f - alpha;
	b[0] = b0 / a0;
	b[1] = b1 / a0;
	b[2] = b2 / a0;
	a[0] = 1.0f;
	a[1] = a1 / a0;
	a[2] = a2 / a0;
}

static void init_eq_coeffs(void)
{
	biquad_lowpass(300.0f, g_lp_b, g_lp_a);
	biquad_bandpass(2500.0f, g_bp_b, g_bp_a);
	biquad_highpass(5000.0f, g_hp_b, g_hp_a);
}

static inline float apply_biquad(float x, float *b, float *a, float *x1,
				 float *x2, float *y1, float *y2)
{
	float y = b[0] * x + b[1] * (*x1) + b[2] * (*x2) - a[1] * (*y1) -
		  a[2] * (*y2);
	*x2 = *x1;
	*x1 = x;
	*y2 = *y1;
	*y1 = y;
	return y;
}

/* ──────────────────────────────────────────────
   Audio File Loader  (WAV / MP3 / FLAC via audiofile.c)
   ────────────────────────────────────────────── */

/* ──────────────────────────────────────────────
   Mixxx Library Import
   Reads BPM + hot cues from ~/.mixxx/mixxxdb.sqlite.
   Opens the DB read-only so we never risk corrupting it.

   Schema (modern Mixxx 2.x):
     library(id, bpm, replaygain, ...)
     track_locations(id, location TEXT)  -- library.location = track_locations.id
     cues(track_id, type, position, hotcue, label)
       type 1 = hot cue, type 4 = loop in/out pair
       position = offset in samples (float, 44100-based)

   Returns 1 if data was found and applied, 0 otherwise.
   ────────────────────────────────────────────── */
/* Max beat positions we import from Mixxx BeatMap (variable-BPM tracks).
 * BeatGrid tracks don't need this — we reconstruct from BPM + offset. */
#define MX_MAX_BEATS 16384

typedef struct {
	float bpm;
	float bpm_offset; /* first beat in djcmd sample frames    */
	int samplerate; /* track's stored samplerate            */
	uint32_t cue[MAX_CUES];
	int cue_set[MAX_CUES];
	/* Beat grid from the 'beats' protobuf blob (optional — only set when
     * Mixxx stored a variable-tempo BeatMap with explicit beat positions) */
	uint32_t *beat_frames; /* malloc'd array, NULL if not present  */
	uint32_t n_beats;
	int found;
} MixxxMeta;

/* ── Minimal protobuf varint / wire-type decoder ─────────────────────────────
 * Mixxx stores beat grids as serialised protobuf blobs in library.beats.
 * Two formats exist:
 *
 *   BeatGrid  (beats_version = "BeatGrid-1.0")
 *     message BeatGrid {
 *       required Beat  first_beat = 1;   // embedded message
 *       required Bpm   bpm        = 2;   // embedded message
 *     }
 *     message Beat { required double frame_position = 1; }
 *     message Bpm  { required double bpm            = 1; }
 *
 *   BeatMap   (beats_version = "BeatMap-1.0")
 *     message BeatMap { repeated Beat beat = 1; }
 *     message Beat    { required double frame_position = 1;
 *                       optional bool   enabled        = 3; }
 *
 * Both store frame positions as little-endian IEEE-754 doubles.
 * Protobuf wire type 1 = 64-bit (double/fixed64), type 2 = length-delimited.
 *
 * We only need to handle these two simple schemas — no full protobuf runtime. */

/* Read a protobuf varint from buf[*pos], advance *pos, return value.
 * Returns -1 and leaves *pos unchanged on error. */
static int64_t pb_read_varint(const uint8_t *buf, int len, int *pos)
{
	int64_t result = 0;
	int shift = 0;
	while (*pos < len) {
		uint8_t b = buf[(*pos)++];
		result |= (int64_t)(b & 0x7F) << shift;
		if (!(b & 0x80))
			return result;
		shift += 7;
		if (shift >= 64)
			return -1;
	}
	return -1;
}

/* Read a little-endian double from buf[*pos], advance *pos by 8. */
static double pb_read_double(const uint8_t *buf, int len, int *pos)
{
	if (*pos + 8 > len)
		return 0.0;
	uint64_t bits = 0;
	for (int i = 0; i < 8; i++)
		bits |= (uint64_t)buf[(*pos)++] << (i * 8);
	double v;
	memcpy(&v, &bits, 8);
	return v;
}

/* Skip an unknown field of wire_type at buf[*pos]. */
static void pb_skip(const uint8_t *buf, int len, int *pos, int wire_type)
{
	switch (wire_type) {
	case 0:
		pb_read_varint(buf, len, pos);
		break; /* varint */
	case 1:
		*pos += 8;
		break; /* 64-bit */
	case 2: {
		int64_t n = pb_read_varint(buf, len, pos); /* LEN    */
		if (n > 0 && *pos + n <= len)
			*pos += (int)n;
		break;
	}
	case 5:
		*pos += 4;
		break; /* 32-bit */
	default:
		*pos = len;
		break; /* bail   */
	}
}

/* Decode a single Beat sub-message; return frame_position or -1. */
static double pb_decode_beat(const uint8_t *buf, int len)
{
	int pos = 0;
	double frame = -1.0;
	while (pos < len) {
		int64_t tag = pb_read_varint(buf, len, &pos);
		if (tag < 0)
			break;
		int field_num = (int)(tag >> 3);
		int wire_type = (int)(tag & 7);
		if (field_num == 1 && wire_type == 1) {
			frame = pb_read_double(buf, len,
					       &pos); /* frame_position */
		} else {
			pb_skip(buf, len, &pos, wire_type);
		}
	}
	return frame;
}

/* Decode a Bpm sub-message; return bpm value or 0. */
static double pb_decode_bpm_msg(const uint8_t *buf, int len)
{
	int pos = 0;
	double bpm = 0.0;
	while (pos < len) {
		int64_t tag = pb_read_varint(buf, len, &pos);
		if (tag < 0)
			break;
		int field_num = (int)(tag >> 3);
		int wire_type = (int)(tag & 7);
		if (field_num == 1 && wire_type == 1) {
			bpm = pb_read_double(buf, len, &pos);
		} else {
			pb_skip(buf, len, &pos, wire_type);
		}
	}
	return bpm;
}

/* Decode the full beats blob.  Fills out->bpm_offset and optionally
 * out->beat_frames / out->n_beats for BeatMap tracks.
 * out->bpm is already filled from the library table (plain float). */
static void mixxx_decode_beats_blob(const uint8_t *blob, int blob_len,
				    const char *version, int src_sr,
				    MixxxMeta *out)
{
	int is_grid = (strstr(version, "BeatGrid") != NULL);
	int is_map = (strstr(version, "BeatMap") != NULL);
	if (!is_grid && !is_map)
		return;

	int pos = 0;

	if (is_grid) {
		/* BeatGrid: field 1 = first Beat sub-message, field 2 = Bpm sub-msg */
		double first_frame = -1.0;
		double pb_bpm = 0.0;
		while (pos < blob_len) {
			int64_t tag = pb_read_varint(blob, blob_len, &pos);
			if (tag < 0)
				break;
			int field_num = (int)(tag >> 3);
			int wire_type = (int)(tag & 7);
			if (wire_type == 2) {
				int64_t sub_len =
					pb_read_varint(blob, blob_len, &pos);
				if (sub_len < 0 || pos + sub_len > blob_len)
					break;
				if (field_num == 1)
					first_frame = pb_decode_beat(
						blob + pos, (int)sub_len);
				else if (field_num == 2)
					pb_bpm = pb_decode_bpm_msg(
						blob + pos, (int)sub_len);
				pos += (int)sub_len;
			} else {
				pb_skip(blob, blob_len, &pos, wire_type);
			}
		}
		if (first_frame >= 0.0) {
			/* Mixxx frame = sample_index (mono).  We store stereo frames.
             * first_frame is at src_sr; rescale to SAMPLE_RATE. */
			double scaled = first_frame *
					(double)g_actual_sample_rate /
					(double)src_sr;
			out->bpm_offset = (float)scaled;
		}
		/* If protobuf BPM is more precise than the library table, use it */
		if (pb_bpm > 0.0)
			out->bpm = (float)pb_bpm;

	} else {
		/* BeatMap: field 1 = repeated Beat sub-messages */
		uint32_t cap = 1024;
		uint32_t count = 0;
		uint32_t *arr = (uint32_t *)malloc(cap * sizeof(uint32_t));
		if (!arr)
			return;

		while (pos < blob_len) {
			int64_t tag = pb_read_varint(blob, blob_len, &pos);
			if (tag < 0)
				break;
			int field_num = (int)(tag >> 3);
			int wire_type = (int)(tag & 7);
			if (field_num == 1 && wire_type == 2) {
				int64_t sub_len =
					pb_read_varint(blob, blob_len, &pos);
				if (sub_len < 0 || pos + sub_len > blob_len)
					break;
				double frame = pb_decode_beat(blob + pos,
							      (int)sub_len);
				pos += (int)sub_len;
				if (frame >= 0.0) {
					uint32_t f =
						(uint32_t)(frame * (double)g_actual_sample_rate /
								   (double)src_sr +
							   0.5);
					if (count >= cap) {
						if (count >= MX_MAX_BEATS)
							continue; /* cap it */
						cap *= 2;
						uint32_t *tmp = (uint32_t *)realloc(
							arr,
							cap * sizeof(uint32_t));
						if (!tmp) {
							free(arr);
							return;
						}
						arr = tmp;
					}
					arr[count++] = f;
				}
			} else {
				pb_skip(blob, blob_len, &pos, wire_type);
			}
		}

		if (count >= 2) {
			/* Derive BPM from median IBI for variable-tempo BeatMap */
			uint32_t *ibi = (uint32_t *)malloc((count - 1) *
							   sizeof(uint32_t));
			if (ibi) {
				for (uint32_t i = 0; i < count - 1; i++)
					ibi[i] = arr[i + 1] - arr[i];
				/* insertion sort for median */
				for (uint32_t i = 1; i < count - 1; i++) {
					uint32_t key = ibi[i];
					int j = (int)i - 1;
					while (j >= 0 && ibi[j] > key) {
						ibi[j + 1] = ibi[j];
						j--;
					}
					ibi[j + 1] = key;
				}
				float med = (float)ibi[(count - 1) / 2];
				if (med > 0.0f)
					out->bpm = (float)g_actual_sample_rate *
						   60.0f / med;
				free(ibi);
			}
			out->bpm_offset = (float)arr[0];
			out->beat_frames = arr;
			out->n_beats = count;
		} else {
			free(arr);
		}
	}
}

static int mixxx_import(const char *audio_path, MixxxMeta *out)
{
	memset(out, 0, sizeof(*out));

	/* Build path to mixxxdb.sqlite */
	const char *home = getenv("HOME");
	if (!home)
		return 0;

	char db_path[1024];
	snprintf(db_path, sizeof(db_path), "%s/.mixxx/mixxxdb.sqlite", home);

	sqlite3 *db = NULL;
	/* READONLY + NOMUTEX — we never write, and we're called from load_worker */
	if (sqlite3_open_v2(db_path, &db,
			    SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX,
			    NULL) != SQLITE_OK) {
		if (db)
			sqlite3_close(db);
		return 0;
	}

	/* ── 1. Look up track by path ────────────────────────────────────────
     *
     * Strategy (tried in order — stop at first hit):
     *
     *  a) Exact absolute path match on tl.location
     *  b) Filename-only match on tl.filename  ← catches moved/remounted libs
     *
     * We do NOT bail out when library.bpm == 0: Mixxx stores 0 for tracks
     * that were imported but not yet run through the Analyze tab.  The beats
     * blob may still be valid (e.g. from a Rekordbox/Serato import).  We
     * always try to decode the blob and use that BPM preferentially.
     * ─────────────────────────────────────────────────────────────────── */
	int track_id = -1;
	float bpm = 0.0f;
	int sr = (int)g_actual_sample_rate;

	/* Extract just the filename component for the fallback query */
	const char *fname = strrchr(audio_path, '/');
	fname = fname ? fname + 1 : audio_path;

	{
		/* Try exact path first, then filename-only */
		const char *sqls[2] = {
			/* (a) exact path */
			"SELECT l.id, l.bpm, l.samplerate "
			"FROM library l "
			"JOIN track_locations tl ON tl.id = l.location "
			"WHERE tl.location = ? AND l.mixxx_deleted = 0 "
			"LIMIT 1;",
			/* (b) filename only — tl.filename stores just the basename */
			"SELECT l.id, l.bpm, l.samplerate "
			"FROM library l "
			"JOIN track_locations tl ON tl.id = l.location "
			"WHERE tl.filename = ? AND l.mixxx_deleted = 0 "
			"LIMIT 1;"
		};
		const char *params[2] = { audio_path, fname };

		for (int attempt = 0; attempt < 2 && track_id < 0; attempt++) {
			sqlite3_stmt *stmt = NULL;
			if (sqlite3_prepare_v2(db, sqls[attempt], -1, &stmt,
					       NULL) == SQLITE_OK) {
				sqlite3_bind_text(stmt, 1, params[attempt], -1,
						  SQLITE_STATIC);
				if (sqlite3_step(stmt) == SQLITE_ROW) {
					track_id = sqlite3_column_int(stmt, 0);
					bpm = (float)sqlite3_column_double(stmt,
									   1);
					sr = sqlite3_column_int(stmt, 2);
					if (sr <= 0)
						sr = (int)g_actual_sample_rate;
				}
			}
			if (stmt)
				sqlite3_finalize(stmt);
		}
	}

	if (track_id < 0) {
		/* Track genuinely not in Mixxx library */
		sqlite3_close(db);
		return 0;
	}

	/* bpm may still be 0 here — that is fine, the beats blob decode below
     * will set out->bpm from the protobuf data.  We only use the library
     * column value as a fallback if the blob is absent or corrupt. */

	out->bpm = bpm; /* may be overwritten by blob decode below */
	out->samplerate = sr;

	/* ── 2. Read hot cues (type=1) sorted by hotcue index ── */
	/* Mixxx stores position in samples at the track's native samplerate.
     * We resample to 44100 if needed (most tracks are already 44100). */
	{
		sqlite3_stmt *stmt = NULL;
		const char *sql =
			"SELECT hotcue, position "
			"FROM cues "
			"WHERE track_id = ? AND type = 1 AND hotcue >= 0 "
			"ORDER BY hotcue ASC "
			"LIMIT ?;";
		if (sqlite3_prepare_v2(db, sql, -1, &stmt, NULL) == SQLITE_OK) {
			sqlite3_bind_int(stmt, 1, track_id);
			sqlite3_bind_int(stmt, 2, MAX_CUES);
			while (sqlite3_step(stmt) == SQLITE_ROW) {
				int hc = sqlite3_column_int(stmt, 0);
				float pos =
					(float)sqlite3_column_double(stmt, 1);
				if (hc >= 0 && hc < MAX_CUES && pos >= 0.0f) {
					/* Rescale from Mixxx samplerate to djcmd samplerate */
					uint32_t frame =
						(uint32_t)(pos * (float)g_actual_sample_rate /
								   (float)sr +
							   0.5f);
					out->cue[hc] = frame;
					out->cue_set[hc] = 1;
				}
			}
		}
		if (stmt)
			sqlite3_finalize(stmt);
	}

	/* ── 3. Decode beat grid from the 'beats' protobuf blob ─────────────
     * library.beats       = serialised protobuf (BeatGrid or BeatMap)
     * library.beats_version = e.g. "BeatGrid-1.0" or "BeatMap-1.0"
     *
     * BeatGrid (constant tempo, default): encodes first_beat frame + BPM.
     * BeatMap  (variable tempo):          encodes every beat individually.
     *
     * Fall back to the type=0 main-cue position if blob is absent/corrupt. */
	{
		sqlite3_stmt *stmt = NULL;
		const char *sql = "SELECT beats, beats_version "
				  "FROM library WHERE id = ? LIMIT 1;";
		int got_blob = 0;
		if (sqlite3_prepare_v2(db, sql, -1, &stmt, NULL) == SQLITE_OK) {
			sqlite3_bind_int(stmt, 1, track_id);
			if (sqlite3_step(stmt) == SQLITE_ROW) {
				const void *blob = sqlite3_column_blob(stmt, 0);
				int blob_len = sqlite3_column_bytes(stmt, 0);
				const char *version =
					(const char *)sqlite3_column_text(stmt,
									  1);
				if (blob && blob_len > 4 && version &&
				    version[0]) {
					mixxx_decode_beats_blob(
						(const uint8_t *)blob, blob_len,
						version, sr, out);
					got_blob = 1;
				}
			}
		}
		if (stmt)
			sqlite3_finalize(stmt);

		/* BPM resolution priority:
         *  1. beats blob decode (most accurate — Mixxx's own analysis)
         *  2. library.bpm column (may be 0 for unanalyzed tracks)
         *  3. Keep whatever we have (caller will fall back to 120)        */
		if (out->bpm <= 0.0f && bpm > 0.0f)
			out->bpm = bpm;

		/* Fallback: use type=0 main cue if blob decode didn't give us an offset */
		if (!got_blob || out->bpm_offset == 0.0f) {
			sqlite3_stmt *stmt2 = NULL;
			const char *sql2 =
				"SELECT position FROM cues "
				"WHERE track_id = ? AND type = 0 LIMIT 1;";
			if (sqlite3_prepare_v2(db, sql2, -1, &stmt2, NULL) ==
			    SQLITE_OK) {
				sqlite3_bind_int(stmt2, 1, track_id);
				if (sqlite3_step(stmt2) == SQLITE_ROW) {
					float pos =
						(float)sqlite3_column_double(
							stmt2, 0);
					if (pos >= 0.0f &&
					    out->bpm_offset == 0.0f)
						out->bpm_offset =
							pos *
							(float)g_actual_sample_rate /
							(float)sr;
				}
			}
			if (stmt2)
				sqlite3_finalize(stmt2);
		}
	}

	sqlite3_close(db);
	out->found = 1;
	return 1;
}

/* Auto-gain: scan peak amplitude, return normalisation factor */
static float calc_auto_gain(const int16_t *data, uint32_t frames)
{
	float peak = 0.0f;
	/* Scan every 8th frame for speed — sufficient for peak detection */
	for (uint32_t i = 0; i < frames; i += 8) {
		float l = fabsf(data[i * 2] / 32768.0f);
		float r = fabsf(data[i * 2 + 1] / 32768.0f);
		if (l > peak)
			peak = l;
		if (r > peak)
			peak = r;
	}
	/* Target level from g_opts.auto_gain_target_db (default -14 dBFS).
     * Converts dBFS to linear: 0 dBFS = 1.0, -14 dBFS ~ 0.2, -1 dBFS ~ 0.891 */
	float target = powf(10.0f, g_opts.auto_gain_target_db / 20.0f);
	return (peak > 0.01f) ? (target / peak) : 1.0f;
}

static int load_track(Track *t, const char *path)
{
	AFBuffer buf;
	int rc = af_load(path, &buf);
	if (rc != AF_OK)
		return -1;

	float gain = g_opts.auto_gain_default ?
			     calc_auto_gain(buf.samples, buf.num_frames) :
			     1.0f;

	int deck_idx = (int)(t - g_tracks);

	/* ── Double-buffer swap to minimise audio thread lock hold time ────── */
	int16_t *old_data = NULL;
	uint8_t *old_wfm_lo = NULL;
	uint8_t *old_wfm_mid = NULL;
	uint8_t *old_wfm_hi = NULL;

	pthread_mutex_lock(&t->lock);
	/* Capture old heap pointers */
	old_data = t->data;
	old_wfm_lo = t->wfm_low;
	old_wfm_mid = t->wfm_mid;
	old_wfm_hi = t->wfm_high;
	/* Install new buffer and reset all fields atomically */
	t->data = buf.samples;
	t->num_frames = buf.num_frames;
	t->pos = 0;
	t->loaded = 1;
	t->playing = 0;
	t->loop_start = 0;
	t->loop_end = t->num_frames;
	t->looping = 0;
	t->gain = gain;
	t->volume = g_opts.default_deck_vol;
	t->sync_locked = 0;
	t->nudge = 0.0f;
	t->filter = 0.5f;
	t->bpm = 0.0f;
	t->bpm_offset = 0.0f;
	t->wfm_low = NULL;
	t->wfm_mid = NULL;
	t->wfm_high = NULL;
	t->wfm_bins = 0;
	for (int i = 0; i < MAX_CUES; i++) {
		t->cue[i] = 0;
		t->cue_set[i] = 0;
	}
	memcpy(t->filename, path, MAX_FILENAME - 1);
	t->filename[MAX_FILENAME - 1] = '\0';
	t->tag_title[0] = '\0';
	t->tag_artist[0] = '\0';

	if (deck_idx >= 0 && deck_idx < MAX_TRACKS) {
		wsola_reset(&g_wsola[deck_idx], 0);
	}

	pthread_mutex_unlock(&t->lock);
	/* Free old buffers outside the lock — no audio dropout */
	free(old_data);
	free(old_wfm_lo);
	free(old_wfm_mid);
	free(old_wfm_hi);
	return 0;
}

/* ──────────────────────────────────────────────
   Waveform + Analysis Cache  (.djcmd sidecar files)

   File layout (little-endian binary):
     [0..3]   magic  "DJCM"
     [4]      version byte  (2)
     [5..8]   uint32  num_frames  (invalidates cache if track length changes)
     [9..12]  float   bpm
     [13..16] float   bpm_offset
     [17..20] uint32  wfm_bins    (== WFM_OVERVIEW_BINS)
     [21..]   uint8[wfm_bins]     low-band  peak values 0-255
     [..]     uint8[wfm_bins]     mid-band  peak values 0-255
     [..]     uint8[wfm_bins]     high-band peak values 0-255

   Version 2 adds 3-band frequency overview (v1 sidecars are re-analysed).
   v5 appends after the three band arrays:
     [21+3*bins..] uint8[8]   cue_set flags (0 or 1 per cue)
     [..]          uint32[8]  cue frame positions (only valid if cue_set[i]==1)
   Sidecar lives next to the audio:  /path/to/song.flac → /path/to/song.flac.djcmd
   ────────────────────────────────────────────── */

#define DJCMD_MAGIC "DJCM"
#define DJCMD_VERSION \
	5 /* v5: + 8 hot cue points saved after waveform overview */

/* Build sidecar path: append ".djcmd" to the audio path */
static void sidecar_path(const char *audio_path, char *out, size_t out_sz)
{
	snprintf(out, out_sz, "%s.djcmd", audio_path);
}

/* Write analysis cache.  Returns 0 on success. */
static int cache_save(const Track *t)
{
	if (!t->loaded || t->wfm_bins == 0 || !t->wfm_low)
		return -1;

	char path[MAX_FILENAME + 8];
	sidecar_path(t->filename, path, sizeof(path));

	FILE *f = fopen(path, "wb");
	if (!f)
		return -1;

	/* Magic + version */
	fwrite(DJCMD_MAGIC, 1, 4, f);
	uint8_t ver = DJCMD_VERSION;
	fwrite(&ver, 1, 1, f);

	/* num_frames (for cache validation) */
	uint32_t nf = t->num_frames;
	fwrite(&nf, 4, 1, f);

	/* BPM + offset */
	float bpm = t->bpm, off = t->bpm_offset;
	fwrite(&bpm, 4, 1, f);
	fwrite(&off, 4, 1, f);

	/* 3-band overview */
	uint32_t bins = t->wfm_bins;
	fwrite(&bins, 4, 1, f);
	fwrite(t->wfm_low, 1, bins, f);
	fwrite(t->wfm_mid, 1, bins, f);
	fwrite(t->wfm_high, 1, bins, f);

	/* v5: 8 cue point flags + frame positions */
	for (int ci = 0; ci < MAX_CUES; ci++) {
		uint8_t flag = (uint8_t)(t->cue_set[ci] ? 1 : 0);
		fwrite(&flag, 1, 1, f);
	}
	for (int ci = 0; ci < MAX_CUES; ci++) {
		uint32_t pos = t->cue_set[ci] ? t->cue[ci] : 0;
		fwrite(&pos, 4, 1, f);
	}

	fclose(f);
	return 0;
}

/* Try to load a cache file into t.
 * Returns 0 if cache was valid and loaded; -1 otherwise (caller must analyse). */
static int cache_load(Track *t)
{
	char path[MAX_FILENAME + 8];
	sidecar_path(t->filename, path, sizeof(path));

	FILE *f = fopen(path, "rb");
	if (!f)
		return -1;

	/* Magic */
	char magic[4];
	if (fread(magic, 1, 4, f) != 4 || memcmp(magic, DJCMD_MAGIC, 4) != 0) {
		fclose(f);
		return -1;
	}

	/* Version */
	uint8_t ver;
	if (fread(&ver, 1, 1, f) != 1 || ver != DJCMD_VERSION) {
		fclose(f);
		return -1;
	}

	/* Validate num_frames matches loaded audio */
	uint32_t cached_frames;
	if (fread(&cached_frames, 4, 1, f) != 1 ||
	    cached_frames != t->num_frames) {
		fclose(f);
		return -1;
	}

	/* BPM + offset */
	float bpm, off;
	if (fread(&bpm, 4, 1, f) != 1 || fread(&off, 4, 1, f) != 1) {
		fclose(f);
		return -1;
	}

	/* Overview */
	uint32_t bins;
	if (fread(&bins, 4, 1, f) != 1 || bins == 0 || bins > 65536) {
		fclose(f);
		return -1;
	}

	uint8_t *lov = (uint8_t *)malloc(bins);
	uint8_t *mov = (uint8_t *)malloc(bins);
	uint8_t *hov = (uint8_t *)malloc(bins);
	if (!lov || !mov || !hov) {
		free(lov);
		free(mov);
		free(hov);
		fclose(f);
		return -1;
	}
	if (fread(lov, 1, bins, f) != bins || fread(mov, 1, bins, f) != bins ||
	    fread(hov, 1, bins, f) != bins) {
		free(lov);
		free(mov);
		free(hov);
		fclose(f);
		return -1;
	}

	/* v5: try to read 8 cue flags + positions (non-fatal if absent) */
	uint8_t cue_flags[MAX_CUES] = { 0 };
	uint32_t cue_pos[MAX_CUES] = { 0 };
	int cues_loaded = 0;
	if (fread(cue_flags, 1, MAX_CUES, f) == (size_t)MAX_CUES) {
		if (fread(cue_pos, 4, MAX_CUES, f) == (size_t)MAX_CUES)
			cues_loaded = 1;
	}

	fclose(f);

	/* All good — commit to track */
	t->bpm = bpm;
	t->bpm_offset = off;
	free(t->wfm_low);
	free(t->wfm_mid);
	free(t->wfm_high);
	t->wfm_low = lov;
	t->wfm_mid = mov;
	t->wfm_high = hov;
	t->wfm_bins = bins;
	wfm_compute_band_max(t);
	/* Apply cached cue points only if not already set by Mixxx DB */
	if (cues_loaded) {
		for (int ci = 0; ci < MAX_CUES; ci++) {
			if (!t->cue_set[ci] && cue_flags[ci] &&
			    cue_pos[ci] < t->num_frames) {
				t->cue[ci] = cue_pos[ci];
				t->cue_set[ci] = 1;
			}
		}
	}
	return 0;
}

/* ──────────────────────────────────────────────
   BPM + Beat Grid Detection via Queen Mary qm-dsp
   Uses TempoTrackV2 (DBN beat tracker) — significantly more accurate than
   simple onset detectors for complex rhythms, rubato, DnB, and tracks without sharp transients.
   The same algorithm powers the QM Vamp plugins used in Sonic Visualiser.

   (spectral flux onset + phase-locked oscillator
   beat tracker, identical to the algorithm in Sonic Visualiser).

   Returns BPM in t->bpm and beat phase in
   t->bpm_offset (frame of first beat).
   ────────────────────────────────────────────── */

/* ── Autocorrelation-based BPM estimator ─────────────────────────────────
 * Designed for PowerPC 7447A: no SIMD, minimal heap, integer-friendly.
 *
 * Algorithm:
 *   1. Downsample to mono at ~4 kHz by averaging blocks of (SR/4000) frames.
 *   2. Compute short-time energy in 10 ms hops (env[]).
 *   3. Half-wave rectify the first-order difference (onset flux).
 *   4. Autocorrelate the flux signal over the lag range corresponding to
 *      BPM_AC_LO..BPM_AC_HI (25–220 BPM).
 *   5. Peak-pick the autocorrelation within that range.
 *   6. Refine with quadratic interpolation for sub-sample accuracy.
 *
 * Working memory: two heap buffers of ~2400 floats each (~19 KB total).
 * Runtime on G4 at 44100/512 hop ≈ 250 ms for a 5-minute track.
 *
 * Returns estimated BPM, or 0.0 if estimation failed (caller keeps 120).
 */
#define BPM_AC_LO 25.0f /* min detectable BPM */
#define BPM_AC_HI 220.0f /* max detectable BPM */
#define BPM_DS_RATE 4000 /* downsample rate (Hz) */
#define BPM_HOP_MS 10 /* energy hop in milliseconds */

static float estimate_bpm_autocorr(const int16_t *data, uint32_t num_frames)
{
	if (num_frames < g_actual_sample_rate * 4u)
		return 0.0f; /* too short */

	/* Limit analysis to first 90 s — enough for any dance track, avoids
     * spending time on long outros.  G4 processes ~50 s/s, so 90 s ≈ 1.8 s
     * of wall-clock analysis. */
	uint32_t analyse_frames = num_frames;
	if (analyse_frames > g_actual_sample_rate * 90u)
		analyse_frames = g_actual_sample_rate * 90u;

	/* ── Step 1 & 2: mono energy envelope ────────────────────────────────
     * hop_frames = samples per energy hop (10 ms at SAMPLE_RATE) */
	uint32_t hop_frames =
		(uint32_t)(g_actual_sample_rate * BPM_HOP_MS / 1000);
	if (hop_frames < 1)
		hop_frames = 1;
	uint32_t env_len = analyse_frames / hop_frames;
	if (env_len < 16)
		return 0.0f;

	float *env = (float *)malloc(env_len * sizeof(float));
	if (!env)
		return 0.0f;

	for (uint32_t h = 0; h < env_len; h++) {
		uint32_t start = h * hop_frames;
		uint32_t end = start + hop_frames;
		if (end > analyse_frames)
			end = analyse_frames;
		float sum = 0.0f;

#if defined(__SSE2__) || defined(__ALTIVEC__)
		/* SIMD Energy: sum of squares */
		uint32_t f = start;
		uint32_t n4 = ((end - start) / 4) * 4;
		uint32_t end4 = start + n4;
		
		#if defined(__SSE2__)
		__m128 vsum = _mm_setzero_ps();
		float scale = 1.0f / 65536.0f;
		__m128 vscale = _mm_set1_ps(scale);
		for (; f < end4; f += 4) {
			/* Load interleaved S16 and convert to float mono */
			float s0 = (data[f*2] + data[f*2+1]) * scale;
			float s1 = (data[(f+1)*2] + data[(f+1)*2+1]) * scale;
			float s2 = (data[(f+2)*2] + data[(f+2)*2+1]) * scale;
			float s3 = (data[(f+3)*2] + data[(f+3)*2+1]) * scale;
			__m128 vs = _mm_set_ps(s3, s2, s1, s0);
			vsum = _mm_add_ps(vsum, _mm_mul_ps(vs, vs));
		}
		float fres[4];
		_mm_storeu_ps(fres, vsum);
		sum = fres[0] + fres[1] + fres[2] + fres[3];
		#elif defined(__ALTIVEC__)
		vfloat vsum = {0,0,0,0};
		float scale = 1.0f / 65536.0f;
		vfloat vscale = {scale, scale, scale, scale};
		for (; f < end4; f += 4) {
			float s0 = (data[f*2] + data[f*2+1]) * scale;
			float s1 = (data[(f+1)*2] + data[(f+1)*2+1]) * scale;
			float s2 = (data[(f+2)*2] + data[(f+2)*2+1]) * scale;
			float s3 = (data[(f+3)*2] + data[(f+3)*2+1]) * scale;
			vfloat vs = {s0, s1, s2, s3};
			vsum = vec_madd(vs, vs, vsum);
		}
		sum = ((float*)&vsum)[0] + ((float*)&vsum)[1] + ((float*)&vsum)[2] + ((float*)&vsum)[3];
		#endif
		/* Remainder */
		for (; f < end; f++) {
			float s = (data[f * 2] + data[f * 2 + 1]) * (1.0f / 65536.0f);
			sum += s * s;
		}
#else
		for (uint32_t f = start; f < end; f++) {
			/* Mono mix — avoid division; scale down to avoid FP overflow */
			float s = (data[f * 2] + data[f * 2 + 1]) *
				  (1.0f / 65536.0f);
			sum += s * s;
		}
#endif
		env[h] = sum / (float)(end - start);
	}

	/* ── Step 3: half-wave-rectified first difference (onset flux) ────── */
	/* Re-use env[] in place: flux[h] = max(env[h] - env[h-1], 0) */
	float prev = env[0];
	for (uint32_t h = 1; h < env_len; h++) {
		float diff = env[h] - prev;
		prev = env[h];
		env[h] = (diff > 0.0f) ? diff : 0.0f;
	}
	env[0] = 0.0f;

	/* ── Step 4: autocorrelation over BPM lag range ─────────────────────
     * lag in hops: lag_frames = SAMPLE_RATE * 60 / BPM / hop_frames
     * BPM_AC_HI → shortest lag,  BPM_AC_LO → longest lag */
	float hops_per_sec = (float)g_actual_sample_rate / (float)hop_frames;
	uint32_t lag_min = (uint32_t)(hops_per_sec * 60.0f / BPM_AC_HI);
	uint32_t lag_max = (uint32_t)(hops_per_sec * 60.0f / BPM_AC_LO);
	if (lag_min < 1)
		lag_min = 1;
	if (lag_max >= env_len)
		lag_max = env_len - 1;
	if (lag_min >= lag_max) {
		free(env);
		return 0.0f;
	}

	uint32_t ac_len = lag_max - lag_min + 1;
	float *ac = (float *)malloc(ac_len * sizeof(float));
	if (!ac) {
		free(env);
		return 0.0f;
	}

	/* Normalised autocorrelation — only sum over the common valid region.
     * Decimated by 2 for speed (even lags only); re-mapped below. */
	uint32_t ac_use = env_len - lag_max; /* valid correlation length */
	for (uint32_t li = 0; li < ac_len; li++) {
		uint32_t lag = lag_min + li;
		float sum = 0.0f;

#if defined(__SSE2__) || defined(__ALTIVEC__)
		uint32_t h = 0;
		uint32_t n_simd = ((ac_use - lag) / 8) * 8;
		
		#if defined(__SSE2__)
		__m128 vsum = _mm_setzero_ps();
		for (; h < n_simd; h += 8) {
			/* Manually gather decimated elements */
			__m128 vh = _mm_set_ps(env[h+6], env[h+4], env[h+2], env[h]);
			__m128 vl = _mm_set_ps(env[h+lag+6], env[h+lag+4], env[h+lag+2], env[h+lag]);
			vsum = _mm_add_ps(vsum, _mm_mul_ps(vh, vl));
		}
		float r[4];
		_mm_storeu_ps(r, vsum);
		sum = r[0] + r[1] + r[2] + r[3];
		#elif defined(__ALTIVEC__)
		vfloat vsum = {0,0,0,0};
		for (; h < n_simd; h += 8) {
			vfloat vh = {env[h], env[h+2], env[h+4], env[h+6]};
			vfloat vl = {env[h+lag], env[h+lag+2], env[h+lag+4], env[h+lag+6]};
			vsum = vec_madd(vh, vl, vsum);
		}
		sum = ((float*)&vsum)[0] + ((float*)&vsum)[1] + ((float*)&vsum)[2] + ((float*)&vsum)[3];
		#endif
		/* Remainder */
		for (; h + lag < ac_use; h += 2)
			sum += env[h] * env[h + lag];
#else
		/* Stride-2 decimation: every-other hop halves the inner loop */
		for (uint32_t h = 0; h + lag < ac_use; h += 2)
			sum += env[h] * env[h + lag];
#endif
		ac[li] = sum;
	}

	free(env);

	/* ── Step 5: find peak in autocorrelation ─────────────────────────── */
	uint32_t peak_li = 0;
	float peak_v = ac[0];
	for (uint32_t li = 1; li < ac_len; li++) {
		if (ac[li] > peak_v) {
			peak_v = ac[li];
			peak_li = li;
		}
	}

	/* ── Step 6: quadratic interpolation for sub-hop precision ──────────
     * Fit a parabola through peak and its two neighbours. */
	float frac_li = (float)peak_li;
	if (peak_li > 0 && peak_li < ac_len - 1) {
		float y0 = ac[peak_li - 1];
		float y1 = ac[peak_li];
		float y2 = ac[peak_li + 1];
		float denom = 2.0f * (y0 - 2.0f * y1 + y2);
		if (fabsf(denom) > 1e-10f)
			frac_li += (y0 - y2) / denom;
	}

	free(ac);

	float best_lag_hops = lag_min + frac_li;
	if (best_lag_hops < 1.0f)
		return 0.0f;

	float bpm = hops_per_sec * 60.0f / best_lag_hops;

	/* Sanity-check: fold into reasonable DJ range 60–180 */
	while (bpm < 60.0f && bpm > 0.0f)
		bpm *= 2.0f;
	while (bpm > 180.0f)
		bpm *= 0.5f;

	return (bpm >= 60.0f && bpm <= 180.0f) ? bpm : 0.0f;
}

/* Simple onset-based beat offset estimator.
 * Scans for the first strong energy onset using a half-rectified flux
 * detector — used as the beat grid anchor after BPM is known. */
#define ONSET_HOP 512

static void detect_bpm_and_offset(Track *t)
{
	/* Snapshot data pointer and frame count under the lock.
     * load_track may swap t->data at any time from the same thread
     * (between jobs), so we work from a local copy of the pointer.
     * The buffer itself is safe to read — load_worker is single-threaded
     * so load_track cannot run again until we return. */
	pthread_mutex_lock(&t->lock);
	int16_t *data = t->data;
	uint32_t nframes = t->num_frames;
	int loaded = t->loaded;
	pthread_mutex_unlock(&t->lock);

	if (!loaded || !data || nframes < 4096) {
		t->bpm = 120.0f;
		t->bpm_offset = 0.0f;
		return;
	}

	/* ── Autocorrelation BPM estimate ────────────────────────────────────
     * Runs first; sets t->bpm to a real estimate or falls back to 120. */
	{
		float ac_bpm = estimate_bpm_autocorr(data, nframes);
		t->bpm = (ac_bpm > 0.0f) ? ac_bpm : 120.0f;
	}

	/* ── Simple spectral-flux onset to find first beat offset ────────
     * We keep BPM at 120 (or whatever Mixxx gave us — this function is
     * only called for non-Mixxx tracks where we have no BPM data).
     * Find the first strong energy transient as beat grid anchor. */
	{
		float prev_energy = 0.0f;
		float peak_flux = 0.0f;
		float threshold = 0.0f;

		/* Pass 1: find mean flux for threshold */
		float flux_sum = 0.0f;
		int flux_n = 0;
		for (uint32_t p = 0; p + ONSET_HOP <= nframes; p += ONSET_HOP) {
			float energy = 0.0f;
			for (int i = 0; i < ONSET_HOP; i++) {
				float s = (data[(p + i) * 2] +
					   data[(p + i) * 2 + 1]) /
					  65536.0f;
				energy += s * s;
			}
			float flux = energy - prev_energy;
			if (flux > 0.0f) {
				flux_sum += flux;
				flux_n++;
			}
			prev_energy = energy;
		}
		threshold = (flux_n > 0) ? (flux_sum / flux_n) * 3.0f : 0.01f;

		/* Pass 2: find first onset above threshold */
		prev_energy = 0.0f;
		uint32_t first_onset = 0;
		for (uint32_t p = 0; p + ONSET_HOP <= nframes; p += ONSET_HOP) {
			float energy = 0.0f;
			for (int i = 0; i < ONSET_HOP; i++) {
				float s = (data[(p + i) * 2] +
					   data[(p + i) * 2 + 1]) /
					  65536.0f;
				energy += s * s;
			}
			float flux = energy - prev_energy;
			if (flux > threshold && first_onset == 0 &&
			    p > ONSET_HOP) {
				first_onset = p;
				if (flux > peak_flux)
					peak_flux = flux;
			}
			prev_energy = energy;
		}
		t->bpm_offset = (float)first_onset;
		/* bpm is already set above by autocorrelation (or 120 fallback) */
	}

	/* ── Build 3-band waveform overview in a single pass over raw PCM ──
     *
     * Frequency split via cascaded single-pole IIR filters (cheap on G4):
     *   low  = one-pole LP at ~300 Hz   (kick, bass)
     *   high = signal - one-pole LP at ~4 kHz
     *   mid  = signal - low - high
     *
     * ACCURACY IMPROVEMENTS vs old version:
     *  1. Filter state is RESET at each bin boundary — prevents energy from
     *     one bin bleeding into the next (the main cause of inaccuracy).
     *  2. RMS (root-mean-square) instead of peak — more representative of
     *     perceptual loudness and reduces single-sample spike artifacts.
     *  3. CFG_WFM_OVERVIEW_BINS = 4096 (was 2048) — twice the resolution.
     *  4. Per-band energy weights tuned to match human perception of kick/snare.
     *
     * For each bin store RMS of |filtered signal| as uint8 0-255.
     */
	{
		uint32_t bins = WFM_OVERVIEW_BINS;
		uint32_t chunk = (nframes + bins - 1) / bins;

		uint8_t *lov = (uint8_t *)malloc(bins);
		uint8_t *mov = (uint8_t *)malloc(bins);
		uint8_t *hov = (uint8_t *)malloc(bins);

		if (lov && mov && hov) {
			const float a_low = 0.0426f; /* fc ≈  300 Hz */
			const float a_hi4k = 0.3996f; /* fc ≈ 4000 Hz */

			float *flo = (float *)malloc(bins * sizeof(float));
			float *fmi = (float *)malloc(bins * sizeof(float));
			float *fhi = (float *)malloc(bins * sizeof(float));
			if (!flo || !fmi || !fhi) {
				free(flo);
				free(fmi);
				free(fhi);
				free(lov);
				free(mov);
				free(hov);
				goto wfm_fail;
			}

			for (uint32_t b = 0; b < bins; b++) {
				uint32_t start = b * chunk;
				/* Guard: start may exceed nframes on the last few bins
                 * due to ceiling division. uint32_t underflow on
                 * (end - start) when start > nframes would give a huge n. */
				if (start >= nframes) {
					flo[b] = fmi[b] = fhi[b] = 0.0f;
					continue;
				}
				uint32_t end = start + chunk;
				if (end > nframes)
					end = nframes;
				uint32_t n =
					end -
					start; /* safe: start < nframes <= end */
				if (n == 0) {
					flo[b] = fmi[b] = fhi[b] = 0.0f;
					continue;
				}

				/* Reset filter state at each bin boundary — prevents
                 * inter-bin energy bleed which was the main accuracy issue */
				float lp_low = 0.0f, lp_4k = 0.0f;

				/* Warm up filter (8 samples) to reduce edge transients */
				uint32_t warmup = (n < 8) ? n : 8;
				for (uint32_t f = start; f < start + warmup;
				     f++) {
					float s = (data[f * 2] +
						   data[f * 2 + 1]) *
						  (0.5f / 32768.0f);
					lp_low += a_low * (s - lp_low);
					lp_4k += a_hi4k * (s - lp_4k);
				}

				/* RMS accumulation */
				float sum_lo = 0.0f, sum_mi = 0.0f,
				      sum_hi = 0.0f;
				for (uint32_t f = start; f < end; f++) {
					float s = (data[f * 2] +
						   data[f * 2 + 1]) *
						  (0.5f / 32768.0f);
					lp_low += a_low * (s - lp_low);
					lp_4k += a_hi4k * (s - lp_4k);

					float lo = lp_low;
					float hi = s - lp_4k;
					float mi = s - lo - hi;

					sum_lo += lo * lo;
					sum_mi += mi * mi;
					sum_hi += hi * hi;
				}
				flo[b] = sqrtf(sum_lo / n);
				fmi[b] = sqrtf(sum_mi / n);
				fhi[b] = sqrtf(sum_hi / n);
			}

			/* Band energy weights — kick/snare priority.
             * lo (kick/bass) and mi (snare/melody) are boosted heavily.
             * hi (hats/cymbals) is suppressed so it does not dominate bar
             * height or colour.  This matches what a DJ needs to see:
             * kick and snare transients as tall, sharp spikes; hats quiet.
             * Changing these weights requires re-analysis (sidecar v4). */
			float global_max = 1e-9f;
			for (uint32_t b = 0; b < bins; b++) {
				flo[b] *=
					12.0f; /* kick/bass: maximum height priority */
				fmi[b] *=
					10.0f; /* snare/body: high priority          */
				fhi[b] *=
					2.0f; /* hats/air:  suppressed               */
				float mx = flo[b] > fmi[b] ? flo[b] : fmi[b];
				mx = mx > fhi[b] ? mx : fhi[b];
				if (mx > global_max)
					global_max = mx;
			}
			for (uint32_t b = 0; b < bins; b++) {
				lov[b] =
					(uint8_t)(flo[b] / global_max * 255.0f +
						  0.5f);
				mov[b] =
					(uint8_t)(fmi[b] / global_max * 255.0f +
						  0.5f);
				hov[b] =
					(uint8_t)(fhi[b] / global_max * 255.0f +
						  0.5f);
			}

			free(flo);
			free(fmi);
			free(fhi);
			free(t->wfm_low);
			free(t->wfm_mid);
			free(t->wfm_high);
			t->wfm_low = lov;
			t->wfm_mid = mov;
			t->wfm_high = hov;
			t->wfm_bins = bins;
			wfm_compute_band_max(t);
		} else {
			free(lov);
			free(mov);
			free(hov);
		}
	wfm_fail:;
	}

	/* Caller is responsible for calling cache_save() after applying any
     * additional overrides (e.g. Mixxx BPM) so the sidecar is correct. */
}

/* ──────────────────────────────────────────────
   Pitch-Shift / Time-Stretch  (linear resampling)
   ────────────────────────────────────────────── */
static inline float lerpf(float a, float b, float t)
{
	return a + (b - a) * t;
}

/* ──────────────────────────────────────────────
   Crossfader (linear/equal-power toggle)
   ────────────────────────────────────────────── */
static float cf_gain(int track_idx)
{
	/* ── Variable Crossfader Curve ──
	 * g_cf_curve:
	 *   0.0 = Slow/Smooth fade (Exponential-like)
	 *   0.5 = Constant Power (Standard DJ mix)
	 *   1.0 = Fast Cut (Scratch / Battle mode)
	 */
	float cf = g_crossfader;
	float curve = g_cf_curve;

	/* Shrink dead-zone as curve sharpens for better scratch response */
	float dz = 0.05f * (1.0f - curve); 
	float lo = 0.5f - dz;
	float hi = 0.5f + dz;

	if (cf >= lo && cf <= hi) return 1.0f;

	if (track_idx % 2 == 0) {
		/* Left deck: fades out on the right side (cf > hi) */
		if (cf <= hi) return 1.0f;
		float t = (cf - hi) / (1.0f - hi);
		/* Power the t value by the curve setting */
		float p = 0.5f + curve * 4.0f; /* p=0.5 (smooth) to p=4.5 (sharp) */
		return powf(cosf(t * (float)(M_PI / 2.0)), p);
	} else {
		/* Right deck: fades out on the left side (cf < lo) */
		if (cf >= lo) return 1.0f;
		float t = 1.0f - (cf / lo);
		float p = 0.5f + curve * 4.0f;
		return powf(cosf(t * (float)(M_PI / 2.0)), p);
	}
}

static void sync_apply(int slave_idx);

/* -----------------------------------------------
   Sync Lock
   1. Matches slave pitch so effective BPM == master BPM.
   2. Phase-aligns: seeks the slave so its nearest
      upcoming beat lands on the master's current
      beat phase.  Without phase alignment the decks
      play at the same speed but can still be a
      half-beat apart.
   ----------------------------------------------- */
static void sync_apply(int slave_idx)
{
	if (g_sync_master < 0 || g_sync_master >= g_num_tracks)
		return;
	Track *master = &g_tracks[g_sync_master];
	Track *slave = &g_tracks[slave_idx];
	if (!master->loaded || master->bpm < 1.0f)
		return;
	if (!slave->loaded || slave->bpm < 1.0f)
		return;

	/* ── 1. Pitch: slave plays at master's effective BPM ── */
	float master_eff_bpm = master->bpm * master->pitch;

	/* Smart range: fold slave's native BPM by octaves until it's within
     * 75%–133% of master BPM.  This prevents syncing a 90 BPM track to
     * 180 BPM, or a 174 DnB track to 87 BPM.
     * Only applied when the option is enabled. */
	float slave_native_bpm = slave->bpm;
	if (g_opts.sync_smart_range && slave_native_bpm > 1.0f &&
	    master_eff_bpm > 1.0f) {
		/* Fold up: if slave is too slow, double it (up to 4×) */
		for (int i = 0; i < 3; i++) {
			if (slave_native_bpm < master_eff_bpm * 0.75f)
				slave_native_bpm *= 2.0f;
			else
				break;
		}
		/* Fold down: if slave is too fast, halve it (up to 4×) */
		for (int i = 0; i < 3; i++) {
			if (slave_native_bpm > master_eff_bpm * 1.334f)
				slave_native_bpm *= 0.5f;
			else
				break;
		}
	}

	slave->pitch = master_eff_bpm / slave_native_bpm;
	if (slave->pitch < 0.25f)
		slave->pitch = 0.25f;
	if (slave->pitch > 4.0f)
		slave->pitch = 4.0f;

	/* ── 2. Phase: align slave beat grid to master beat phase ── */
	/*
     * Master beat grid: beats at  master->bpm_offset + n * master_beat_frames
     * Where we are in that grid (fractional beat):
     *   master_phase = fmod(master->pos - master->bpm_offset, master_beat_frames)
     *                / master_beat_frames         (0..1)
     *
     * We want the slave's position to be at the same fractional beat,
     * snapped to the nearest beat boundary in the slave's own grid.
     *
     * slave beat grid:  slave->bpm_offset + n * slave_beat_frames
     * Find the beat index n_s such that
     *   slave->bpm_offset + n_s * slave_beat_frames  is as close as
     *   possible to slave->pos, then add the master's phase offset.
     */
	float master_beat_frames =
		(float)g_actual_sample_rate * 60.0f / master_eff_bpm;
	float slave_eff_bpm =
		slave_native_bpm * slave->pitch; /* == master_eff_bpm */
	float slave_beat_frames =
		(float)g_actual_sample_rate * 60.0f / slave_eff_bpm;

	/* Master's fractional beat phase (0..1) */
	float master_phase = 0.0f;
	if (master_beat_frames > 0.0f) {
		float beats_elapsed =
			((float)master->pos - master->bpm_offset) /
			master_beat_frames;
		master_phase = beats_elapsed - floorf(beats_elapsed);
	}

	/* Nearest beat boundary in slave grid, closest to current slave pos */
	float slave_beats_elapsed = 0.0f;
	if (slave_beat_frames > 0.0f)
		slave_beats_elapsed = ((float)slave->pos - slave->bpm_offset) /
				      slave_beat_frames;
	float slave_beat_n =
		floorf(slave_beats_elapsed + 0.5f); /* round to nearest */

	/* New slave position: that beat + master's phase offset */
	float new_pos = slave->bpm_offset +
			(slave_beat_n + master_phase) * slave_beat_frames;

	/* Clamp to valid range */
	if (new_pos < 0.0f)
		new_pos = 0.0f;
	if ((uint32_t)new_pos >= slave->num_frames)
		new_pos = (float)(slave->num_frames - 1);

	slave->pos = (uint32_t)new_pos;
}
static float g_tmp_l[PERIOD_FRAMES];
static float g_tmp_r[PERIOD_FRAMES];
static float g_mix_l[PERIOD_FRAMES];
static float g_mix_r[PERIOD_FRAMES];
static float g_mix_hp_l[PERIOD_FRAMES];
static float g_mix_hp_r[PERIOD_FRAMES];
static int16_t g_pcm_buf[PERIOD_FRAMES * 2];
static int16_t g_pcm_hp_buf[PERIOD_FRAMES * 2];

/* Peak meter state — written by audio thread, read by UI thread.
 * No mutex: single float writes are atomic enough for display. */
static float g_vu_l = 0.0f; /* current RMS-ish level 0.0-1.0 */
static int g_blink_tick = 0; /* incremented each redraw, used for LED blink */
static float g_vu_r = 0.0f;
static float g_vu_peak_l = 0.0f; /* peak hold level */
static float g_vu_peak_r = 0.0f;
#define VU_PEAK_HOLD_PERIODS (g_opts.ui_fps * 3) /* 3 s hold at any FPS */

/* Refresh deck selector button LEDs (1/2/3/4 buttons on each side)
 * and the play button LED (led_deck_a/b = PLAY button on NS7III).
 *
 * State-tracked: only sends MIDI when state actually changes, so calling
 * this every redraw frame does not flood the controller with messages. */
static void deck_leds_refresh(void)
{
	/* Track last-sent state to avoid redundant MIDI sends.
	 * We use MAX_TRACKS (4) for selection tracking. */
	static int last_sel_lit[4] = { -1, -1, -1, -1 };
	static int last_play[2] = { -1, -1 };
	static int last_cue[2] = { -1, -1 };
	static int last_loop[2] = { -1, -1 };

	static const char *deck_led_names[4] = { "led_deck_1", "led_deck_2",
						 "led_deck_3", "led_deck_4" };
	static const char *play_leds[2] = { "led_deck_a", "led_deck_b" };
	static const char *cue_leds[2] = { "led_cue_default_a",
					   "led_cue_default_b" };
	static const char *loop_leds[2] = { "led_loop_a", "led_loop_b" };

	/* 1. Selection LEDs (1, 2, 3, 4) — these are absolute */
	for (int i = 0; i < 4; i++) {
		int active = (i == g_side_deck[0] || i == g_side_deck[1]);
		if (active != last_sel_lit[i]) {
			if (active) led_on(deck_led_names[i]);
			else led_off(deck_led_names[i]);
			last_sel_lit[i] = active;
		}
	}

	/* 2. Platter-side LEDs (Play, Cue, Loop) — these follow g_side_deck */
	for (int side = 0; side < 2; side++) {
		int dk = g_side_deck[side];

		/* CUE button LED */
		int want_cue = g_cue_default_set[dk] ? 1 : 0;
		if (want_cue != last_cue[side]) {
			if (want_cue)
				led_on(cue_leds[side]);
			else
				led_off(cue_leds[side]);
			last_cue[side] = want_cue;
		}

		/* Loop LED */
		int want_loop = g_tracks[dk].looping ? 1 : 0;
		if (want_loop != last_loop[side]) {
			if (want_loop)
				led_on(loop_leds[side]);
			else
				led_off(loop_leds[side]);
			last_loop[side] = want_loop;
		}

		/* Play button LED: 0=off 1=solid 2=blink-phase-A 3=blink-phase-B */
		Track *t = &g_tracks[dk];
		int want_play;
		if (!t->loaded) {
			want_play = 0;
		} else if (t->playing) {
			want_play = 1;
		} else {
			int blink_period =
				(g_opts.ui_fps > 0) ? g_opts.ui_fps / 2 : 10;
			if (blink_period < 1)
				blink_period = 1;
			want_play = ((g_blink_tick / blink_period) % 2 == 0) ?
					    2 :
					    3;
		}
		if (want_play != last_play[side]) {
			if (want_play == 0 || want_play == 3)
				led_off(play_leds[side]);
			else
				led_on(play_leds[side]);
			last_play[side] = want_play;
		}
	}
}

/* Refresh FX button LEDs for one deck side.
 * btn1 = slot 0, btn2 = slot 1, btn3 = slot 2.
 * Master compressor has no dedicated button LED. */
static void fx_leds_refresh(int deck)
{
	char side = (deck == g_side_deck[0]) ? 'a' : 'b';
	char n[32];
	for (int sl = 0; sl < FX_SLOTS_PER_DECK; sl++) {
		FXSlot *fx = fx_slot(deck, sl);
		int active = (fx->pending_type >= 0) ?
				     (fx->pending_type != FX_NONE) :
				     (fx->type != FX_NONE);
		snprintf(n, sizeof(n), "led_fx_btn_%d_%c", sl + 1, side);
		if (active)
			led_on(n);
		else
			led_off(n);
	}
}


#define VU_DECAY 0.85f /* per-period level decay */

static void mix_and_write(void)
{
	memset(g_mix_l, 0, sizeof(float) * PERIOD_FRAMES);
	memset(g_mix_r, 0, sizeof(float) * PERIOD_FRAMES);
	memset(g_mix_hp_l, 0, sizeof(float) * PERIOD_FRAMES);
	memset(g_mix_hp_r, 0, sizeof(float) * PERIOD_FRAMES);

	float mvol = g_master_vol / 100.0f;
	float hp_mvol = g_hp_vol / 100.0f;

	/* ── Quantize play logic ── */
	if (g_opts.sync_quantize && g_sync_master >= 0) {
		Track *master = &g_tracks[g_sync_master];
		if (master->loaded && master->playing && master->bpm > 1.0f) {
			float meff = master->bpm * master->pitch;
			float beat_f =
				(float)g_actual_sample_rate * 60.0f / meff;
			float bar_f = beat_f * 4.0f;
			float raw_now = (float)master->pos - master->bpm_offset;
			float raw_prev = raw_now - (float)PERIOD_FRAMES;
			float pos_now = raw_now > 0.0f ? raw_now : 0.0f;
			float pos_prev = raw_prev > 0.0f ? raw_prev : 0.0f;
			int bar_now = (int)floorf(pos_now / bar_f);
			int bar_prev = (int)floorf(pos_prev / bar_f);
			int crossed_bar = (bar_now > bar_prev);

			for (int ti = 0; ti < MAX_TRACKS; ti++) {
				if (ti == g_sync_master)
					continue;
				Track *sl = &g_tracks[ti];
				if (!sl->pending_play || !sl->loaded)
					continue;
				if (crossed_bar) {
					sl->pending_play = 0;
					sl->playing = 1;
					sync_apply(ti);
				}
			}
		}
	}

	for (int t = 0; t < MAX_TRACKS; t++) {
		Track *tr = &g_tracks[t];
		if (!tr->loaded)
			continue;

		/* Allow paused decks through when the user is actively scratching */
		int scratching = !tr->playing &&
				 g_jog_touched[t] &&
				 (fabsf(g_motor_vel[t]) > 0.0001f);
		if (!tr->playing && !scratching)
			continue;

		/* ── Scratch Engine Improvements ── */
		float start_vel = g_last_motor_vel[t];
		float target_vel = g_motor_vel[t];
		float start_nudge = g_last_applied_nudge[t];
		float target_nudge = g_jog_nudge[t];

		pthread_mutex_lock(&tr->lock);

		/* Revert to WSOLA for key-lock (CPU efficient on G4)
		 * AND bypass key-lock during active scratch touch. */
		if (tr->key_lock && !g_jog_touched[t]) {
			WSOLAState *ws = &g_wsola[t];
			double rate = (double)(tr->pitch + tr->nudge + target_vel + target_nudge);
			
			/* Reset if playhead jumped */
			if (fabsf((float)(ws->src_pos - (double)tr->pos)) > WSOLA_WIN * 4)
				wsola_reset(ws, tr->pos);

			wsola_process(tr, ws, g_tmp_l, g_tmp_r, PERIOD_FRAMES, rate, tr->volume * tr->gain);
		} else {
			/* Hermite path with per-sample velocity ramping */
			read_pitched(tr, g_tmp_l, g_tmp_r, PERIOD_FRAMES, start_vel,
				     target_vel, start_nudge, target_nudge);
		}
		pthread_mutex_unlock(&tr->lock);

		/* Update historical values for next block ramping */
		g_last_motor_vel[t] = target_vel;
		g_last_applied_nudge[t] = target_nudge;

		/* ── Vinyl scratch character filter ──────────────────────────── */
		{
			float abs_rate = fabsf(tr->pitch + tr->nudge +
					       g_motor_vel[t] + target_nudge);
			float target_alpha;
			if (abs_rate < 0.02f) {
				target_alpha = 0.02f;
			} else if (abs_rate < 1.0f) {
				target_alpha = powf(abs_rate, 0.8f);
			} else {
				target_alpha = 1.0f / abs_rate;
			}

			/* Inertia Smoothing: prevents filter "pops" from MIDI jitter */
			g_scratch_alpha[t] = g_scratch_alpha[t] * 0.85f + target_alpha * 0.15f;
			float alpha = g_scratch_alpha[t];

			float scratch_gain = 1.0f;
			if (g_jog_touched[t] && abs_rate < 0.15f) {
				float n = abs_rate / 0.15f;
				scratch_gain = 0.05f + 0.95f * n * n;
			}

			if (g_jog_touched[t]) {
				float nl = 0.0018f * scratch_gain;
				for (uint32_t si = 0; si < PERIOD_FRAMES; si++) {
					g_noise_state[t] = g_noise_state[t] * 1664525u + 1013904223u;
					float wl = (float)(int32_t)g_noise_state[t] * (1.0f / 2147483648.0f);
					g_noise_state[t] = g_noise_state[t] * 1664525u + 1013904223u;
					float wr = (float)(int32_t)g_noise_state[t] * (1.0f / 2147483648.0f);
					g_noise_brown_l[t] = g_noise_brown_l[t] * 0.97f + wl * 0.03f;
					g_noise_brown_r[t] = g_noise_brown_r[t] * 0.97f + wr * 0.03f;
					g_tmp_l[si] += nl * (wl * 0.05f + 5.0f * g_noise_brown_l[t] * 0.95f);
					g_tmp_r[si] += nl * (wr * 0.05f + 5.0f * g_noise_brown_r[t] * 0.95f);
				}
			}

			if (alpha < 0.94f) {
				float g_svf = tanf(1.5707963f * alpha);
				float k_svf = 0.333f;
				float a1 = 1.0f / (1.0f + g_svf * (g_svf + k_svf));
				float a2 = g_svf * a1;
				float a3 = g_svf * a2;

				float g_hc = tanf(1.5707963f * 0.18f); /* ~8kHz @ 44.1k */
				float a1_hc = 1.0f / (1.0f + g_hc);

				/* Anti-denormal DC offset (1e-18) prevents G4 CPU spikes during silence */
				const float dc = 1e-18f;

				for (uint32_t si = 0; si < PERIOD_FRAMES; si++) {
					float xl = g_tmp_l[si] * scratch_gain;
					float xr = g_tmp_r[si] * scratch_gain;
					
					{
						float ic1 = g_scratch_lpf_l[t] + dc;
						float ic2 = g_scratch_lpf2_l[t] + dc;
						float v3 = xl - ic2;
						float v1 = a1 * ic1 + a2 * v3;
						float v2 = ic2 + a2 * ic1 + a3 * v3;
						g_scratch_lpf_l[t] = 2.0f * v1 - ic1;
						g_scratch_lpf2_l[t] = 2.0f * v2 - ic2;
						float o = v2 + 0.35f * v1;
						float v3_hc = o - g_noise_brown_l[t];
						float v1_hc = a1_hc * v3_hc;
						float v2_hc = g_noise_brown_l[t] + v1_hc;
						g_noise_brown_l[t] = v2_hc + v1_hc;
						o = v2_hc;
						float abs_o = fabsf(o);
						if (abs_o > 1.0f) o = (o > 0) ? 1.0f : -1.0f;
						else o = o * (1.5f - 0.5f * o * o);
						g_tmp_l[si] = o;
					}
					{
						float ic1 = g_scratch_lpf_r[t] + dc;
						float ic2 = g_scratch_lpf2_r[t] + dc;
						float v3 = xr - ic2;
						float v1 = a1 * ic1 + a2 * v3;
						float v2 = ic2 + a2 * ic1 + a3 * v3;
						g_scratch_lpf_r[t] = 2.0f * v1 - ic1;
						g_scratch_lpf2_r[t] = 2.0f * v2 - ic2;
						float o = v2 + 0.35f * v1;
						float v3_hc = o - g_noise_brown_r[t];
						float v1_hc = a1_hc * v3_hc;
						float v2_hc = g_noise_brown_r[t] + v1_hc;
						g_noise_brown_r[t] = v2_hc + v1_hc;
						o = v2_hc;
						float abs_o = fabsf(o);
						if (abs_o > 1.0f) o = (o > 0) ? 1.0f : -1.0f;
						else o = o * (1.5f - 0.5f * o * o);
						g_tmp_r[si] = o;
					}
				}
			} else {
				if (scratch_gain < 1.0f) {
					for (uint32_t si = 0; si < PERIOD_FRAMES; si++) {
						g_tmp_l[si] *= scratch_gain;
						g_tmp_r[si] *= scratch_gain;
					}
				}
				g_scratch_lpf_l[t]  = 0.0f;
				g_scratch_lpf2_l[t] = g_tmp_l[PERIOD_FRAMES - 1];
				g_scratch_lpf_r[t]  = 0.0f;
				g_scratch_lpf2_r[t] = g_tmp_r[PERIOD_FRAMES - 1];
			}
		}

		/* Decay nudge (applied to global state for next MIDI message) */
		g_jog_nudge[t] *= NUDGE_DECAY;
		if (fabsf(g_jog_nudge[t]) < 0.0001f) {
			g_jog_nudge[t] = 0.0f;
			g_last_applied_nudge[t] = 0.0f;
		}

		/* ── NS7III motor: decay g_jog_abs_vel when silent ── */
		if (g_jog_type == JOG_NS7III && g_motor_running[t]) {
			struct timespec _now;
			clock_gettime(CLOCK_MONOTONIC, &_now);
			int64_t now_ms = (int64_t)_now.tv_sec * 1000 + _now.tv_nsec / 1000000;
			int64_t silent_ms = now_ms - g_jog_last_msg_ms[t];

			if (silent_ms > 40) {
				g_jog_abs_vel[t] *= 0.75f;
				if (fabsf(g_jog_abs_vel[t]) < 0.00001f) g_jog_abs_vel[t] = 0.0f;

				if (g_motor_settle_until[t] == 0) {
					if (g_jog_touched[t]) {
						float raw_vel = (g_jog_abs_vel[t] / g_jog_ref_delta) - 1.0f;
						if (raw_vel > g_jog_motor_dead) raw_vel -= g_jog_motor_dead;
						else if (raw_vel < -g_jog_motor_dead) raw_vel += g_jog_motor_dead;
						else raw_vel = 0.0f;
						
						if (raw_vel > g_jog_vel_max) raw_vel = g_jog_vel_max;
						if (raw_vel < -g_jog_vel_max) raw_vel = -g_jog_vel_max;
						
						float diff = raw_vel - g_motor_vel[t];
						float alpha = (fabsf(diff) > 0.5f) ? 0.55f : 0.20f;
						g_motor_vel[t] = g_motor_vel[t] * (1.0f - alpha) + raw_vel * alpha;
					} else if (g_opts.vinyl_mode) {
						g_motor_vel[t] = 0.0f;
					} else {
						float raw_vel2 = (g_jog_abs_vel[t] / g_jog_ref_delta) - 1.0f;
						if (raw_vel2 > g_jog_motor_dead) raw_vel2 -= g_jog_motor_dead;
						else if (raw_vel2 < -g_jog_motor_dead) raw_vel2 += g_jog_motor_dead;
						else raw_vel2 = 0.0f;
						
						if (raw_vel2 > g_jog_vel_max) raw_vel2 = g_jog_vel_max;
						if (raw_vel2 < -g_jog_vel_max) raw_vel2 = -g_jog_vel_max;
						
						float diff2 = raw_vel2 - g_motor_vel[t];
						float alpha2 = (fabsf(diff2) > 0.5f) ? 0.55f : 0.20f;
						g_motor_vel[t] = g_motor_vel[t] * (1.0f - alpha2) + raw_vel2 * alpha2;
					}
				}
			}
		}

		float gain = cf_gain(t) * mvol;
		EQState *eq = &g_eq[t];
		float gl = 1.0f + tr->eq_low;
		float gm = 1.0f + tr->eq_mid;
		float gh = 1.0f + tr->eq_high;

		/* ── Filter Logic ── */
		float fk = tr->filter;
		/* g_filter_on[t]: 0 = bypass (always flat), 1 = knob active */
		int filter_active =
			g_filter_on[t] && (fabsf(fk - 0.5f) > 0.005f);
		if (filter_active && fabsf(fk - eq->fi_last) > 0.002f) {
			eq->fi_last = fk;
			if (fk < 0.5f) {
				float t_lp = fk / 0.5f;
				float fc = 80.0f * powf(2000.0f / 80.0f, t_lp);
				biquad_lowpass(fc, eq->fi_b, eq->fi_a);
			} else {
				float t_hp = (fk - 0.5f) / 0.5f;
				float fc = 2000.0f *
					   powf(18000.0f / 2000.0f, t_hp);
				biquad_highpass(fc, eq->fi_b, eq->fi_a);
			}
		}

		/* ── EQ pass: write back to g_tmp_l/r (in-place) ── */
		for (int i = 0; i < PERIOD_FRAMES; i++) {
			float l = g_tmp_l[i], r = g_tmp_r[i];
			if (filter_active) {
				l = apply_biquad(l, eq->fi_b, eq->fi_a,
						 &eq->fi_x1l, &eq->fi_x2l,
						 &eq->fi_y1l, &eq->fi_y2l);
				r = apply_biquad(r, eq->fi_b, eq->fi_a,
						 &eq->fi_x1r, &eq->fi_x2r,
						 &eq->fi_y1r, &eq->fi_y2r);
			}
			float lo_l = apply_biquad(l, g_lp_b, g_lp_a,
						  &eq->lp_x1l, &eq->lp_x2l,
						  &eq->lp_y1l, &eq->lp_y2l);
			float lo_r = apply_biquad(r, g_lp_b, g_lp_a,
						  &eq->lp_x1r, &eq->lp_x2r,
						  &eq->lp_y1r, &eq->lp_y2r);
			float mi_l = apply_biquad(l, g_bp_b, g_bp_a,
						  &eq->bp_x1l, &eq->bp_x2l,
						  &eq->bp_y1l, &eq->bp_y2l);
			float mi_r = apply_biquad(r, g_bp_b, g_bp_a,
						  &eq->bp_x1r, &eq->bp_x2r,
						  &eq->bp_y1r, &eq->bp_y2r);
			float hi_l = apply_biquad(l, g_hp_b, g_hp_a,
						  &eq->hp_x1l, &eq->hp_x2l,
						  &eq->hp_y1l, &eq->hp_y2l);
			float hi_r = apply_biquad(r, g_hp_b, g_hp_a,
						  &eq->hp_x1r, &eq->hp_x2r,
						  &eq->hp_y1r, &eq->hp_y2r);
			g_tmp_l[i] = lo_l * gl + mi_l * gm + hi_l * gh;
			g_tmp_r[i] = lo_r * gl + mi_r * gm + hi_r * gh;
		}

		/* ── Insert FX (post-EQ, pre-mix) — slots 0 and 1 serial ── */
		for (int _s = 0; _s < FX_SLOTS_PER_DECK; _s++)
			fx_apply(fx_slot(t, _s), g_tmp_l, g_tmp_r,
				 PERIOD_FRAMES);

		/* ── Accumulate into mix buses ── */
		for (int i = 0; i < PERIOD_FRAMES; i++) {
			g_mix_l[i] += g_tmp_l[i] * gain;
			g_mix_r[i] += g_tmp_r[i] * gain;

			if (tr->cue_active) {
				g_mix_hp_l[i] += g_tmp_l[i] * tr->volume * tr->gain;
				g_mix_hp_r[i] += g_tmp_r[i] * tr->volume * tr->gain;
			}
		}
	}

	/* ── Master bus FX (post-crossfade, pre-clip) ── */
	{
		FXSlot *ms = fx_master();
		if (ms->type != FX_NONE)
			fx_apply(ms, g_mix_l, g_mix_r, PERIOD_FRAMES);
	}

	/* ── Clipping & VU Meters ── */
	float period_peak_l = 0.0f, period_peak_r = 0.0f;
	for (int i = 0; i < PERIOD_FRAMES; i++) {
		float l = g_mix_l[i], r = g_mix_r[i];
		if (l > 0.95f)
			l = 0.95f + 0.05f * tanhf((l - 0.95f) / 0.05f);
		if (l < -0.95f)
			l = -0.95f - 0.05f * tanhf((-l - 0.95f) / 0.05f);
		if (r > 0.95f)
			r = 0.95f + 0.05f * tanhf((r - 0.95f) / 0.05f);
		if (r < -0.95f)
			r = -0.95f - 0.05f * tanhf((-r - 0.95f) / 0.05f);
		float al = fabsf(l), ar = fabsf(r);
		if (al > period_peak_l)
			period_peak_l = al;
		if (ar > period_peak_r)
			period_peak_r = ar;
		g_pcm_buf[i * 2] = (int16_t)(l * 32767.0f);
		g_pcm_buf[i * 2 + 1] = (int16_t)(r * 32767.0f);

		/* Headphone Output processing */
		float hl = g_mix_hp_l[i] * hp_mvol;
		float hr = g_mix_hp_r[i] * hp_mvol;
		if (hl > 0.95f) hl = 0.95f + 0.05f * tanhf((hl - 0.95f) / 0.05f);
		if (hl < -0.95f) hl = -0.95f - 0.05f * tanhf((-hl - 0.95f) / 0.05f);
		if (hr > 0.95f) hr = 0.95f + 0.05f * tanhf((hr - 0.95f) / 0.05f);
		if (hr < -0.95f) hr = -0.95f - 0.05f * tanhf((-hr - 0.95f) / 0.05f);
		g_pcm_hp_buf[i * 2] = (int16_t)(hl * 32767.0f);
		g_pcm_hp_buf[i * 2 + 1] = (int16_t)(hr * 32767.0f);
	}

	g_vu_l = g_vu_l * VU_DECAY + period_peak_l * (1.0f - VU_DECAY);
	g_vu_r = g_vu_r * VU_DECAY + period_peak_r * (1.0f - VU_DECAY);

	/* Peak hold — rise instantly, decay slowly over ~3s */
	if (period_peak_l > g_vu_peak_l)
		g_vu_peak_l = period_peak_l;
	else
		g_vu_peak_l *= 0.998f;
	if (period_peak_r > g_vu_peak_r)
		g_vu_peak_r = period_peak_r;
	else
		g_vu_peak_r *= 0.998f;

	/* ── Output ── */
	int err = snd_pcm_writei(g_pcm, g_pcm_buf, PERIOD_FRAMES);
	if (err == -EPIPE)
		snd_pcm_prepare(g_pcm);
	else if (err < 0)
		usleep(5000);

	if (g_pcm_hp) {
		err = snd_pcm_writei(g_pcm_hp, g_pcm_hp_buf, PERIOD_FRAMES);
		if (err == -EPIPE)
			snd_pcm_prepare(g_pcm_hp);
	}
}
/* ──────────────────────────────────────────────
   Session Mix Log
   ────────────────────────────────────────────── */

/* Open a new dated log file.  Called once from main(). */
static void mixlog_open(void)
{
	const char *home = getenv("HOME");
	if (!home)
		return;

	/* Ensure config directory exists */
	char dir[512];
	snprintf(dir, sizeof(dir), "%s/" CFG_CONFIG_DIR, home);
	mkdir(dir, 0755);

	/* Build filename: YYYY-MM-DD_HH-MM-SS.log */
	time_t now = time(NULL);
	struct tm *tm = localtime(&now);
	char fname[64];
	strftime(fname, sizeof(fname), "%Y-%m-%d_%H-%M-%S.log", tm);

	snprintf(g_mixlog_path, sizeof(g_mixlog_path),
		 "%s/" CFG_CONFIG_DIR "/%s", home, fname);

	g_mixlog = fopen(g_mixlog_path, "w");
	if (!g_mixlog)
		return;

	/* Session header */
	char ts[32];
	strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", tm);
	fprintf(g_mixlog,
		"# djcmd session log\n"
		"# Started : %s\n"
		"# Format  : time | deck | mm:ss | bpm | artist | title | file\n"
		"#           (mm:ss = play time of previous track on this deck)\n"
		"\n",
		ts);
	fflush(g_mixlog);
}

/* Log a newly loaded track on 'deck' (0-indexed).
 * Also writes the play duration of whatever was on this deck before.
 * Must be called from load_worker (not the audio thread) after tags are read. */
static void mixlog_track_loaded(int deck, Track *lt)
{
	if (!g_mixlog)
		return;
	if (deck < 0 || deck >= MAX_TRACKS)
		return;

	time_t now = time(NULL);

	/* Compute play time for the outgoing track (if any) */
	char playtime[32] = "--:--";
	if (g_mixlog_load_time[deck] != 0) {
		long secs = (long)(now - g_mixlog_load_time[deck]);
		if (secs < 0)
			secs = 0;
		snprintf(playtime, sizeof(playtime), "%ld:%02ld", secs / 60,
			 secs % 60);
	}

	/* Wall-clock timestamp for this new load */
	struct tm *tm = localtime(&now);
	char ts[32];
	strftime(ts, sizeof(ts), "%H:%M:%S", tm);

	/* Snapshot track info (already under lt->lock in the caller — but we
     * copy into our own buffers here to avoid any race on future updates) */
	snprintf(g_mixlog_title[deck], sizeof(g_mixlog_title[0]), "%s",
		 lt->tag_title);
	snprintf(g_mixlog_artist[deck], sizeof(g_mixlog_artist[0]), "%s",
		 lt->tag_artist);
	snprintf(g_mixlog_file[deck], sizeof(g_mixlog_file[0]), "%s",
		 lt->filename);
	g_mixlog_bpm[deck] = lt->bpm;

	/* Basename only for the file column */
	const char *base = strrchr(lt->filename, '/');
	base = base ? base + 1 : lt->filename;

	/* Friendly display: prefer artist – title, fall back to filename */
	const char *artist = lt->tag_artist[0] ? lt->tag_artist : "?";
	const char *title = lt->tag_title[0] ? lt->tag_title : base;

	fprintf(g_mixlog,
		"%s | Deck %c | played=%-7s | %6.2f BPM | %-24s | %-32s | %s\n",
		ts, DECK_NUM(deck), playtime, (double)lt->bpm, artist, title,
		base);
	fflush(g_mixlog);

	/* Update load time for the incoming track */
	g_mixlog_load_time[deck] = now;
}

/* Write session footer and close.  Called from cleanup(). */
static void mixlog_close(void)
{
	if (!g_mixlog)
		return;

	time_t now = time(NULL);
	struct tm *tm = localtime(&now);
	char ts[32];
	strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", tm);

	fprintf(g_mixlog, "\n# Ended: %s\n", ts);

	/* Final play-time entries for any decks still loaded */
	for (int d = 0; d < MAX_TRACKS; d++) {
		if (g_mixlog_load_time[d] == 0)
			continue;
		long secs = (long)(now - g_mixlog_load_time[d]);
		if (secs < 0)
			secs = 0;
		const char *base = strrchr(g_mixlog_file[d], '/');
		base = base ? base + 1 : g_mixlog_file[d];
		const char *artist =
			g_mixlog_artist[d][0] ? g_mixlog_artist[d] : "?";
		const char *title =
			g_mixlog_title[d][0] ? g_mixlog_title[d] : base;
		fprintf(g_mixlog,
			"# Deck %c final: played=%ld:%02ld | %.2f BPM | %s – %s\n",
			DECK_NUM(d), secs / 60, secs % 60,
			(double)g_mixlog_bpm[d], artist, title);
	}

	fclose(g_mixlog);
	g_mixlog = NULL;
}

/* ──────────────────────────────────────────────
   Load Worker Thread
   Waits for jobs posted by the UI thread; runs
   load_track + BPM analysis in the background so
   the audio and UI threads never stall.
   ────────────────────────────────────────────── */

/* ── Tag reader ──────────────────────────────────────────────────────────
 * Reads ID3v2 (MP3), ID3v1 (MP3 fallback), Vorbis comments (FLAC), and
 * RIFF INFO (WAV) tags from a file.  No extra library required.
 * Writes up to (max-1) bytes into title/artist; always NUL-terminates.
 * Fields are left empty (first byte NUL) if no tag is found.
 * ─────────────────────────────────────────────────────────────────────── */
static void tag_clean(char *s)
{
	/* Strip leading/trailing whitespace and non-printable bytes */
	int len = (int)strlen(s);
	while (len > 0 && (unsigned char)s[len - 1] <= 0x20)
		s[--len] = '\0';
	int start = 0;
	while (start < len && (unsigned char)s[start] <= 0x20)
		start++;
	if (start > 0)
		memmove(s, s + start, len - start + 1);
}

static void read_tags(const char *path, char *title, int tmax, char *artist,
		      int amax)
{
	title[0] = artist[0] = '\0';

	FILE *f = fopen(path, "rb");
	if (!f)
		return;

	unsigned char hdr[10];
	if (fread(hdr, 1, 10, f) < 10) {
		fclose(f);
		return;
	}

	/* ── ID3v2 ── */
	if (hdr[0] == 'I' && hdr[1] == 'D' && hdr[2] == '3') {
		/* int version = hdr[3]; */
		/* int flags   = hdr[5]; */
		uint32_t id3size = ((uint32_t)(hdr[6] & 0x7f) << 21) |
				   ((uint32_t)(hdr[7] & 0x7f) << 14) |
				   ((uint32_t)(hdr[8] & 0x7f) << 7) |
				   (uint32_t)(hdr[9] & 0x7f);
		uint32_t end = 10 + id3size;
		uint32_t pos = 10;
		int ver = hdr[3]; /* 2, 3, or 4 */

		while (pos + (ver <= 2 ? 6 : 10) < end &&
		       (title[0] == '\0' || artist[0] == '\0')) {
			unsigned char fhdr[10];
			int hlen = (ver <= 2) ? 6 : 10;
			if (fseek(f, (long)pos, SEEK_SET) != 0)
				break;
			if ((int)fread(fhdr, 1, hlen, f) < hlen)
				break;
			if (fhdr[0] == 0)
				break; /* padding */

			uint32_t fsz;
			if (ver <= 2)
				fsz = ((uint32_t)fhdr[3] << 16) |
				      ((uint32_t)fhdr[4] << 8) | fhdr[5];
			else
				fsz = ((uint32_t)fhdr[4] << 24) |
				      ((uint32_t)fhdr[5] << 16) |
				      ((uint32_t)fhdr[6] << 8) | fhdr[7];

			/* Identify TIT2/TT2 and TPE1/TP1 */
			int is_title =
				(ver <= 2) ?
					(fhdr[0] == 'T' && fhdr[1] == 'T' &&
					 fhdr[2] == '2') :
					(fhdr[0] == 'T' && fhdr[1] == 'I' &&
					 fhdr[2] == 'T' && fhdr[3] == '2');
			int is_artist =
				(ver <= 2) ?
					(fhdr[0] == 'T' && fhdr[1] == 'P' &&
					 fhdr[2] == '1') :
					(fhdr[0] == 'T' && fhdr[1] == 'P' &&
					 fhdr[2] == 'E' && fhdr[3] == '1');

			if ((is_title && title[0] == '\0') ||
			    (is_artist && artist[0] == '\0')) {
				char *buf = (is_title) ? title : artist;
				int max = (is_title) ? tmax : amax;
				if (fsz > 1 && fsz < 4096) {
					unsigned char enc;
					fread(&enc, 1, 1,
					      f); /* encoding byte */
					int read = (int)fsz - 1;
					if (read >= max)
						read = max - 1;
					fread(buf, 1, read, f);
					buf[read] = '\0';
					/* UTF-16: skip BOM and take every other byte (ASCII range) */
					if (enc == 1 || enc == 2) {
						int src = 0, dst = 0;
						if (read >= 2 &&
						    (unsigned char)buf[0] ==
							    0xFF)
							src = 2; /* skip BOM */
						while (src + 1 < read) {
							unsigned char
								lo = buf[src],
								hi = buf[src +
									 1];
							buf[dst++] =
								(hi == 0 &&
								 lo >= 0x20) ?
									(char)lo :
									'?';
							src += 2;
						}
						buf[dst] = '\0';
					}
					tag_clean(buf);
				}
			}
			pos += hlen + fsz;
		}
		if (title[0] || artist[0]) {
			fclose(f);
			return;
		}
	}

	/* ── Vorbis comments (FLAC) ── */
	fseek(f, 0, SEEK_SET);
	unsigned char magic[4];
	if (fread(magic, 1, 4, f) == 4 && magic[0] == 'f' && magic[1] == 'L' &&
	    magic[2] == 'a' && magic[3] == 'C') {
		/* Scan METADATA_BLOCKs for VORBIS_COMMENT (type 4) */
		for (int bi = 0; bi < 32; bi++) {
			unsigned char blkhdr[4];
			if (fread(blkhdr, 1, 4, f) < 4)
				break;
			int blktype = blkhdr[0] & 0x7F;
			int last = (blkhdr[0] >> 7) & 1;
			uint32_t blksz = ((uint32_t)blkhdr[1] << 16) |
					 ((uint32_t)blkhdr[2] << 8) | blkhdr[3];
			if (blktype == 4) { /* VORBIS_COMMENT */
				long base = ftell(f);
				/* vendor string */
				uint32_t vlen;
				fread(&vlen, 4, 1, f);
				fseek(f, (long)vlen, SEEK_CUR);
				uint32_t ncomments;
				fread(&ncomments, 4, 1, f);
				for (uint32_t ci = 0; ci < ncomments && ci < 64;
				     ci++) {
					uint32_t clen;
					fread(&clen, 4, 1, f);
					if (clen == 0 || clen > 4096) {
						fseek(f, (long)clen, SEEK_CUR);
						continue;
					}
					char cbuf[4097];
					int rd = (clen < 4096) ? (int)clen :
								 4096;
					fread(cbuf, 1, rd, f);
					cbuf[rd] = '\0';
					/* Skip remaining bytes if truncated */
					if ((uint32_t)rd < clen)
						fseek(f, (long)(clen - rd),
						      SEEK_CUR);
					/* Parse KEY=VALUE */
					char *eq = strchr(cbuf, '=');
					if (!eq)
						continue;
					*eq = '\0';
					char *key = cbuf;
					char *val = eq + 1;
					/* Case-insensitive compare */
					char ku[32];
					int ki = 0;
					while (key[ki] && ki < 31) {
						ku[ki] = toupper(
							(unsigned char)key[ki]);
						ki++;
					}
					ku[ki] = '\0';
					if (!strcmp(ku, "TITLE") &&
					    title[0] == '\0') {
						snprintf(title, tmax, "%s",
							 val);
						tag_clean(title);
					} else if (!strcmp(ku, "ARTIST") &&
						   artist[0] == '\0') {
						snprintf(artist, amax, "%s",
							 val);
						tag_clean(artist);
					}
				}
				(void)base;
				break;
			}
			if (last)
				break;
			fseek(f, (long)blksz, SEEK_CUR);
		}
		if (title[0] || artist[0]) {
			fclose(f);
			return;
		}
	}

	/* ── RIFF INFO (WAV) ── */
	fseek(f, 0, SEEK_SET);
	unsigned char riff[12];
	if (fread(riff, 1, 12, f) == 12 && riff[0] == 'R' && riff[1] == 'I' &&
	    riff[2] == 'F' && riff[3] == 'F' && riff[8] == 'W' &&
	    riff[9] == 'A' && riff[10] == 'V' && riff[11] == 'E') {
		/* Scan chunks looking for LIST/INFO */
		for (int ci = 0; ci < 64; ci++) {
			unsigned char chdr[8];
			if (fread(chdr, 1, 8, f) < 8)
				break;
			uint32_t csz = (uint32_t)chdr[4] |
				       ((uint32_t)chdr[5] << 8) |
				       ((uint32_t)chdr[6] << 16) |
				       ((uint32_t)chdr[7] << 24);
			if (chdr[0] == 'L' && chdr[1] == 'I' &&
			    chdr[2] == 'S' && chdr[3] == 'T') {
				unsigned char ltype[4];
				fread(ltype, 1, 4, f);
				if (ltype[0] == 'I' && ltype[1] == 'N' &&
				    ltype[2] == 'F' && ltype[3] == 'O') {
					uint32_t left = csz - 4;
					while (left >= 8) {
						unsigned char ihdr[8];
						if (fread(ihdr, 1, 8, f) < 8)
							break;
						uint32_t isz =
							(uint32_t)ihdr[4] |
							((uint32_t)ihdr[5]
							 << 8) |
							((uint32_t)ihdr[6]
							 << 16) |
							((uint32_t)ihdr[7]
							 << 24);
						left -= 8;
						int is_t =
							(ihdr[0] == 'I' &&
							 ihdr[1] == 'N' &&
							 ihdr[2] == 'A' &&
							 ihdr[3] ==
								 'M'); /* INAM */
						int is_a =
							(ihdr[0] == 'I' &&
							 ihdr[1] == 'A' &&
							 ihdr[2] == 'R' &&
							 ihdr[3] ==
								 'T'); /* IART */
						if ((is_t || is_a) && isz > 0 &&
						    isz < 256) {
							char *buf =
								is_t ? title :
								       artist;
							int max = is_t ? tmax :
									 amax;
							int rd =
								(int)isz < max - 1 ?
									(int)isz :
									max - 1;
							fread(buf, 1, rd, f);
							buf[rd] = '\0';
							if ((uint32_t)rd < isz)
								fseek(f,
								      (long)(isz -
									     rd),
								      SEEK_CUR);
							tag_clean(buf);
						} else {
							fseek(f, (long)isz,
							      SEEK_CUR);
						}
						left -= (isz < left) ? isz :
								       left;
					}
					break;
				}
				fseek(f, (long)(csz - 4), SEEK_CUR);
			} else {
				fseek(f, (long)csz, SEEK_CUR);
			}
		}
		if (title[0] || artist[0]) {
			fclose(f);
			return;
		}
	}

	/* ── ID3v1 fallback (last 128 bytes of file) ── */
	if (fseek(f, -128, SEEK_END) == 0) {
		unsigned char v1[128];
		if (fread(v1, 1, 128, f) == 128 && v1[0] == 'T' &&
		    v1[1] == 'A' && v1[2] == 'G') {
			if (title[0] == '\0') {
				memcpy(title, v1 + 3,
				       30 < tmax - 1 ? 30 : tmax - 1);
				title[30 < tmax - 1 ? 30 : tmax - 1] = '\0';
				tag_clean(title);
			}
			if (artist[0] == '\0') {
				memcpy(artist, v1 + 33,
				       30 < amax - 1 ? 30 : amax - 1);
				artist[30 < amax - 1 ? 30 : amax - 1] = '\0';
				tag_clean(artist);
			}
		}
	}

	fclose(f);
}

static void *load_worker(void *arg)
{
	(void)arg;
	while (g_running) {
		pthread_mutex_lock(&g_load_mutex);
		while (!g_load_job.valid && g_running)
			pthread_cond_wait(&g_load_cond, &g_load_mutex);

		if (!g_running) {
			pthread_mutex_unlock(&g_load_mutex);
			break;
		}

		/* Consume job */
		char path[FB_PATH_MAX + 256];
		int deck = g_load_job.deck;
		snprintf(path, sizeof(path), "%s", g_load_job.path);
		path[sizeof(path) - 1] = '\0';
		g_load_job.valid = 0;
		pthread_mutex_unlock(&g_load_mutex);

		/* Do the actual work outside the lock */
		Track *lt = &g_tracks[deck];
		pthread_mutex_lock(&lt->lock);
		lt->playing = 0;
		pthread_mutex_unlock(&lt->lock);

		if (load_track(lt, path) == 0) {
			/* ── Try Mixxx library first ──────────────────────────────
             * If mixxxdb.sqlite has this track we get: BPM (already
             * analysed by Mixxx's much faster analyzer), hot cue
             * positions, and a rough beat phase.  Skip qm-dsp analysis entirely
             * when Mixxx data is present — it's faster and more accurate
             * for tracks you've already worked with in Mixxx. */
			MixxxMeta mx;
			int got_mixxx = mixxx_import(path, &mx);

			if (got_mixxx && mx.bpm > 0.0f) {
				pthread_mutex_lock(&lt->lock);
				lt->bpm = mx.bpm;
				lt->bpm_offset = mx.bpm_offset;
				/* Import hot cues — only fill slots not already in use */
				for (int ci = 0; ci < MAX_CUES; ci++) {
					if (mx.cue_set[ci] &&
					    mx.cue[ci] < lt->num_frames) {
						lt->cue[ci] = mx.cue[ci];
						lt->cue_set[ci] = 1;
					}
				}
				pthread_mutex_unlock(&lt->lock);
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Mixxx: %.1f BPM \u2192 Deck %c",
					 mx.bpm, DECK_NUM(deck));
				if (mx.beat_frames) {
					free(mx.beat_frames);
					mx.beat_frames = NULL;
				}
				/* Waveform overview: load from cache or rebuild.
                 * Either way, Mixxx BPM/offset wins — the sidecar may have
                 * a stale BPM from a previous onset-detector run, so we
                 * ALWAYS overwrite after cache_load returns. */
				if (cache_load(lt) != 0) {
					detect_bpm_and_offset(
						lt); /* builds waveform overview */
				}
				/* Overwrite BPM/offset unconditionally — Mixxx is authoritative */
				pthread_mutex_lock(&lt->lock);
				lt->bpm = mx.bpm;
				lt->bpm_offset = mx.bpm_offset;
				pthread_mutex_unlock(&lt->lock);
				/* Persist the correct BPM into the sidecar so future loads
                 * don't revert even if mixxxdb is unavailable */
				cache_save(lt);
			} else {
				/* Not in Mixxx library, or Mixxx had no BPM for this track */
				if (got_mixxx) {
					/* Hot cues may still be valid even without BPM */
					pthread_mutex_lock(&lt->lock);
					for (int ci = 0; ci < MAX_CUES; ci++) {
						if (mx.cue_set[ci] &&
						    mx.cue[ci] <
							    lt->num_frames) {
							lt->cue[ci] =
								mx.cue[ci];
							lt->cue_set[ci] = 1;
						}
					}
					pthread_mutex_unlock(&lt->lock);
					if (mx.beat_frames) {
						free(mx.beat_frames);
						mx.beat_frames = NULL;
					}
				}
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Analyzing Deck %c...",
					 DECK_NUM(deck));
				if (cache_load(lt) != 0) {
					detect_bpm_and_offset(lt);
					cache_save(lt);
				}
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Loaded \u2192 Deck %c",
					 DECK_NUM(deck));
			}
			/* Read ID3/Vorbis/RIFF tags — always, regardless of BPM source */
			pthread_mutex_lock(&lt->lock);
			read_tags(path, lt->tag_title, sizeof(lt->tag_title),
				  lt->tag_artist, sizeof(lt->tag_artist));
			lt->key_lock = g_opts.key_lock_default;
			if (lt->key_lock)
				wsola_reset(&g_wsola[deck], lt->pos);
			pthread_mutex_unlock(&lt->lock);

			/* Session mix log — record this track load */
			mixlog_track_loaded(deck, lt);

			/* ── Auto master handoff ───────────────────────────────────
             * If the deck we just loaded INTO was the sync master, and
             * another deck is currently playing, hand master status to
             * the highest-priority playing deck (lowest index first).
             * This lets the DJ load a new track onto the master without
             * losing sync — the running deck becomes the new master. */
			if (g_opts.sync_auto_handoff && deck == g_sync_master) {
				for (int ti = 0; ti < g_num_tracks; ti++) {
					if (ti == deck)
						continue;
					if (g_tracks[ti].loaded &&
					    g_tracks[ti].playing) {
						g_sync_master = ti;
						/* Keep any sync-locked decks locked to the new master */
						break;
					}
				}
			}
		} else {
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Load FAILED: Deck %c", DECK_NUM(deck));
		}
	}
	return NULL;
}

/* Post a load job — safe to call from the UI thread */
static void enqueue_load(int deck, const char *path)
{
	pthread_mutex_lock(&g_load_mutex);
	g_load_job.deck = deck;
	snprintf(g_load_job.path, sizeof(g_load_job.path), "%s", path);
	g_load_job.path[sizeof(g_load_job.path) - 1] = '\0';
	g_load_job.valid = 1;
	snprintf(g_fb_status, sizeof(g_fb_status), "Loading Deck %c...",
		 DECK_NUM(deck));
	pthread_cond_signal(&g_load_cond);
	pthread_mutex_unlock(&g_load_mutex);
}

/* ──────────────────────────────────────────────
   Audio Thread
   ────────────────────────────────────────────── */
/* ══════════════════════════════════════════════════════════════════════════
   NS7III DISPLAY SUBSYSTEM — moved to ns7iii_displaysub.h
   Disabled: caused CPU spikes and audio dropouts at 20 Hz update rate.
   Re-enable once Numark handshake (0x50/0x52/0x53/0x55) is solved.
   ══════════════════════════════════════════════════════════════════════════ */

#if 0 /* NS7III display subsystem — see ns7iii_displaysub.h */



/* ══════════════════════════════════════════════════════════════════════════
   END NS7III DISPLAY SUBSYSTEM
   ══════════════════════════════════════════════════════════════════════════ */
#endif /* NS7III display subsystem */

static void *audio_thread(void *arg)
{
	(void)arg;
	while (g_running) {
		mix_and_write();
	}
	return NULL;
}

/* ──────────────────────────────────────────────
   ALSA Init
   ────────────────────────────────────────────── */
static int init_alsa(void)
{
	snd_pcm_hw_params_t *hwp;
	int err;

	/* ── Master (Main) Output ── */
	err = snd_pcm_open(&g_pcm, g_pcm_dev_str, SND_PCM_STREAM_PLAYBACK, 0);
	if (err < 0)
		return err;

	snd_pcm_hw_params_alloca(&hwp);
	snd_pcm_hw_params_any(g_pcm, hwp);
	snd_pcm_hw_params_set_access(g_pcm, hwp, SND_PCM_ACCESS_RW_INTERLEAVED);
	snd_pcm_hw_params_set_format(
		g_pcm, hwp,
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
		SND_PCM_FORMAT_S16_BE /* PowerPC / big-endian host */
#else
		SND_PCM_FORMAT_S16_LE /* x86_64 / little-endian host */
#endif
	);
	snd_pcm_hw_params_set_channels(g_pcm, hwp, CHANNELS);
	unsigned int rate = SAMPLE_RATE;
	snd_pcm_hw_params_set_rate_near(g_pcm, hwp, &rate, 0);
	g_actual_sample_rate = rate;
	af_set_target_rate(rate);
	snd_pcm_uframes_t period = PERIOD_FRAMES;
	snd_pcm_hw_params_set_period_size_near(g_pcm, hwp, &period, 0);
	snd_pcm_uframes_t buffer = PERIOD_FRAMES * BUFFER_PERIODS;
	snd_pcm_hw_params_set_buffer_size_near(g_pcm, hwp, &buffer);

	err = snd_pcm_hw_params(g_pcm, hwp);
	if (err < 0) {
		snd_pcm_close(g_pcm);
		return err;
	}

	{
		snd_pcm_sw_params_t *swp;
		snd_pcm_sw_params_alloca(&swp);
		snd_pcm_sw_params_current(g_pcm, swp);
		snd_pcm_sw_params_set_avail_min(g_pcm, swp, PERIOD_FRAMES);
		snd_pcm_sw_params(g_pcm, swp);
	}
	snd_pcm_prepare(g_pcm);

	/* ── Headphone Output ── */
	err = snd_pcm_open(&g_pcm_hp, g_pcm_hp_dev_str, SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK);
	if (err >= 0) {
		snd_pcm_hw_params_t *hwp_hp;
		snd_pcm_hw_params_alloca(&hwp_hp);
		snd_pcm_hw_params_any(g_pcm_hp, hwp_hp);
		snd_pcm_hw_params_set_access(g_pcm_hp, hwp_hp, SND_PCM_ACCESS_RW_INTERLEAVED);
		snd_pcm_hw_params_set_format(
			g_pcm_hp, hwp_hp,
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			SND_PCM_FORMAT_S16_BE
#else
			SND_PCM_FORMAT_S16_LE
#endif
		);
		snd_pcm_hw_params_set_channels(g_pcm_hp, hwp_hp, CHANNELS);
		unsigned int rate_hp = g_actual_sample_rate;
		snd_pcm_hw_params_set_rate_near(g_pcm_hp, hwp_hp, &rate_hp, 0);
		snd_pcm_uframes_t period_hp = PERIOD_FRAMES;
		snd_pcm_hw_params_set_period_size_near(g_pcm_hp, hwp_hp, &period_hp, 0);
		snd_pcm_uframes_t buffer_hp = PERIOD_FRAMES * BUFFER_PERIODS;
		snd_pcm_hw_params_set_buffer_size_near(g_pcm_hp, hwp_hp, &buffer_hp);

		err = snd_pcm_hw_params(g_pcm_hp, hwp_hp);
		if (err < 0) {
			snd_pcm_close(g_pcm_hp);
			g_pcm_hp = NULL;
		} else {
			snd_pcm_sw_params_t *swp_hp;
			snd_pcm_sw_params_alloca(&swp_hp);
			snd_pcm_sw_params_current(g_pcm_hp, swp_hp);
			snd_pcm_sw_params_set_avail_min(g_pcm_hp, swp_hp, PERIOD_FRAMES);
			snd_pcm_sw_params(g_pcm_hp, swp_hp);
			snd_pcm_prepare(g_pcm_hp);
		}
	} else {
		g_pcm_hp = NULL;
	}

	return 0;
}

/* ──────────────────────────────────────────────
   MIDI Map File  (~/.config/djcmd/midi.map)
   ────────────────────────────────────────────── */

static MidiOutBinding *midi_out_lookup(const char *name);
static void led_on(const char *name);
static void led_off(const char *name);

static void midi_map_path(char *out, size_t max)
{
	const char *home = getenv("HOME");
	if (!home)
		home = "/tmp";

	/* Find the human name for the currently open device */
	const char *dev_name = "";
	for (int i = 0; i < g_midi_ndevices; i++) {
		if (strcmp(g_midi_devlist[i].dev, g_midi_dev_str) == 0) {
			dev_name = g_midi_devlist[i].name;
			break;
		}
	}

	/* Build sanitised filename, fall back to "midi" if no device is open */
	char san[128];
	if (dev_name[0])
		midi_map_name_from_device(dev_name, san, sizeof(san));
	else
		snprintf(san, sizeof(san), "midi");

	snprintf(out, max, "%s/" CFG_CONFIG_DIR "/%s.map", home, san);
}

/* Switch to a different MIDI input device at runtime.
 * Safe to call from the UI thread.  Sequence:
 *   1. Null out g_midi_in so the MIDI thread stops reading immediately
 *   2. Close old handle
 *   3. Update g_midi_dev_str and g_midi_dev_sel
 *   4. Open new handle
 *   5. Clear all bindings and reload the per-device map file
 *   6. Save the new device choice to settings
 */
static void midi_open_device(int dev_idx)
{
	if (dev_idx < 0 || dev_idx >= g_midi_ndevices)
		return;

	/* 1+2: stop MIDI thread reads, close old handles */
	snd_rawmidi_t *old_in = g_midi_in;
	snd_rawmidi_t *old_out = g_midi_out;
	g_midi_in = NULL;
	g_midi_out = NULL;
	usleep(5000);
	if (old_in)
		snd_rawmidi_close(old_in);
	if (old_out)
		snd_rawmidi_close(old_out);

	/* 3: update device tracking */
	g_midi_dev_sel = dev_idx;
	snprintf(g_midi_dev_str, sizeof(g_midi_dev_str), "%s",
		 g_midi_devlist[dev_idx].dev);

	/* 4: open input + output.  Output may not be available on all devices
     * (e.g. a receive-only interface) — failure is non-fatal. */
	snd_rawmidi_open(&g_midi_in, &g_midi_out, g_midi_dev_str,
			 SND_RAWMIDI_NONBLOCK);

	/* 5: clear bindings, load per-device map */
	g_midi_nbindings = 0;
	g_midi_nout_bindings = 0;
	midi_map_load();
	/* If no map existed, write a generic starter map for this device */
	if (g_midi_nbindings == 0 && g_midi_nout_bindings == 0)
		midi_map_write_generic_defaults();

	/* Migration: fix old motor_stop entries that used CC65/val0 (wrong).
     * Correct values are CC66/val127.  Silently patch and resave. */
	{
		int migrated = 0;
		const char *stop_names[] = { "motor_stop_a", "motor_stop_b",
					     "motor_stop_c", "motor_stop_d" };
		/* Correct start status bytes for each deck */
		const uint8_t start_st[] = { 0xB1, 0xB2, 0xB3, 0xB4 };
		for (int di = 0; di < 4; di++) {
			MidiOutBinding *b = midi_out_lookup(stop_names[di]);
			if (b && b->data1 == 0x41 && b->data2 == 0) {
				/* Old wrong entry: same CC as start, val0 — fix to CC66 val127 */
				b->data1 = 0x42; /* CC66 */
				b->data2 = 127;
				if (!b->status)
					b->status = start_st[di];
				migrated = 1;
			}
		}
		if (migrated)
			midi_map_save();
	}

	/* 6: persist the choice */
	settings_save();

	snprintf(g_fb_status, sizeof(g_fb_status), "MIDI: %s%s",
		 g_midi_devlist[dev_idx].name,
		 g_midi_out ? " (in+out)" : " (in only)");
	/* Refresh all LEDs for all connected decks */
	deck_leds_refresh();
	for (int _di = 0; _di < g_num_tracks; _di++) {
		pad_mode_leds_refresh(_di);
		pad_leds_refresh(_di);
		fx_leds_refresh(_di);
	}

	/* Send initial deck select LED state — active decks lit, inactive dim */
	if (g_midi_out) {
		/* Default: left=deck1, right=deck2 */
		led_on("led_deck_1");
		led_off("led_deck_3");
		led_on("led_deck_2");
		led_off("led_deck_4");
		/* Reflect current side routing if it differs from default */
		if (g_side_deck[0] == 2) {
			led_on("led_deck_3");
			led_off("led_deck_1");
		}
		if (g_side_deck[1] == 3) {
			led_on("led_deck_4");
			led_off("led_deck_2");
		}
	}

	/* ── NS7III startup signal — DISABLED (possible Serato license negotiation) ─
     *
     * In pcap captures Serato sends CC#66 val=1 on MIDI channels 2–5 immediately
     * after the ExtMfr SysEx handshake.  This triggers a D2H burst of 100+ CC
     * messages reflecting current knob/fader/encoder positions.
     *
     * Capture evidence (powercycle pcap H2D Frame 184, 0-indexed ch notation):
     *   CC ch1 cc=0x42 val=1; CC ch2 cc=0x42 val=1;
     *   CC ch3 cc=0x42 val=1; CC ch4 cc=0x42 val=1
     * cold-init-load sends only ch1+ch2 (ch3/ch4 optional or version-dependent).
     *
     * WHY DISABLED: This sequence — ExtMfr SysEx (0x50) + CC#66 val=1 — is also
     * present in the Serato DJ Pro license handshake protocol.  Sending it from
     * djcmd could constitute licence circumvention or trigger DRM checks inside
     * the NS7III firmware.  Leave commented until the protocol role is clarified.
     *
     * If re-enabled, use: B1–B4 42 01 (MIDI channels 2–5, status 0xB1–0xB4).
     * The NS7III responds within ~1.2ms; no sleep needed before reading.
     *
#if 0
    if (g_midi_out) {
        const char *dn = g_midi_devlist[dev_idx].name;
        char dn_lower[64] = "";
        for (int i = 0; dn[i] && i < (int)sizeof(dn_lower) - 1; i++)
            dn_lower[i] = (char)((unsigned char)dn[i] >= 'A' &&
                                  (unsigned char)dn[i] <= 'Z'
                                  ? dn[i] + 32 : dn[i]);
        if (strstr(dn_lower, "ns7")) {
            for (int ch = 1; ch <= 4; ch++) {
                uint8_t msg[3] = { (uint8_t)(0xB0 | ch), 0x42, 0x01 };
                snd_rawmidi_write(g_midi_out, msg, 3);
            }
            snd_rawmidi_drain(g_midi_out);
        }
    }
#endif
     */
}
/* ── MIDI output helpers ─────────────────────────────────────────────────── */

/* Send a single 3-byte MIDI CC message.  Non-blocking — drops silently if
 * the output buffer is full (motor probing, not time-critical). */
/* ── Pad LED helpers ────────────────────────────────────────────────────────
 * NS7III pads: Note On with velocity = colour on ch2 (deck A) or ch3 (deck B).
 *   Pads 1-8 → notes 0x47-0x4E
 *   Velocity colours (approximate, tuned to NS7III firmware):
 *     0        = off
 *     1-10     = dim (used for "available but unset" state)
 *     0x04     = dark blue  (autoloop background)
 *     0x14     = green      (autoloop active pad)
 *     0x18     = bright green
 *     0x19     = yellow     (roll mode)
 *     0x7F     = white/max bright (hotcue set)
 *     0x10     = red        (hotcue set, cues 1-4 use distinct colours)
 *
 * The NS7III also supports SysEx RGB for pads, but single-byte velocity
 * colours are sufficient and universally supported.
 */
#define PAD_NOTE_BASE 0x47 /* note for pad 1; pad N = PAD_NOTE_BASE + N - 1 */

/* Pad colour velocities */
#define PAD_COL_OFF 0x00
#define PAD_COL_AUTOLOOP_BG 0x04 /* dark blue  — inactive autoloop pad        */
#define PAD_COL_AUTOLOOP_ON 0x14 /* green      — active loop engaged on this  */
#define PAD_COL_ROLL_BG 0x19 /* yellow     — roll mode background         */
#define PAD_COL_ROLL_ON 0x1F /* bright yel — roll actively held           */
#define PAD_COL_MANUALLOOP_BG \
	0x08 /* dim yellow — available manual loop pad    */
#define PAD_COL_MANUALLOOP_SET \
	0x14 /* green      — loop point set / loop active */
#define PAD_COL_CUE_UNSET 0x00 /* off — cue slot empty                      */
#define PAD_COL_CUE_1 0x10 /* red                                       */
#define PAD_COL_CUE_2 0x20 /* orange                                    */
#define PAD_COL_CUE_3 0x25 /* yellow                                    */
#define PAD_COL_CUE_4 0x14 /* green                                     */
#define PAD_COL_CUE_5 0x04 /* cyan                                      */
#define PAD_COL_CUE_6 0x30 /* blue                                      */
#define PAD_COL_CUE_7 0x08 /* purple                                    */
#define PAD_COL_CUE_8 0x7F /* white                                     */

static const uint8_t g_cue_colours[8] = { PAD_COL_CUE_1, PAD_COL_CUE_2,
					  PAD_COL_CUE_3, PAD_COL_CUE_4,
					  PAD_COL_CUE_5, PAD_COL_CUE_6,
					  PAD_COL_CUE_7, PAD_COL_CUE_8 };

/* Send a velocity-coloured Note On to a pad on the given MIDI channel */
static void pad_led(int midi_ch, int pad_idx_1, uint8_t colour)
{
	/* pad_idx_1: 1-8 */
	if (!g_midi_out)
		return;
	int note = PAD_NOTE_BASE + pad_idx_1 - 1;
	uint8_t msg[3];
	msg[0] = (uint8_t)(0x90 | ((midi_ch - 1) & 0x0F));
	msg[1] = (uint8_t)(note & 0x7F);
	msg[2] = colour & 0x7F;
	snd_rawmidi_write(g_midi_out, msg, 3);
	snd_rawmidi_drain(g_midi_out);
}

/* Refresh all 8 pad LEDs for one deck side according to current mode.
 * deck: 0-based track index; side: 0=left(A) 1=right(B) → MIDI ch 2/3 */
static void pad_leds_refresh(int deck)
{
	if (!g_midi_out)
		return;
	int midi_ch = (deck == g_side_deck[0]) ? 2 : 3;
	int mode = g_pad_mode[deck];
	Track *t = &g_tracks[deck];

	if (mode == PAD_MODE_HOTCUE) {
		/* Pad N → cue N: colour if set, off if empty */
		for (int i = 1; i <= 8; i++) {
			int ci = i - 1;
			uint8_t col = (ci < MAX_CUES && t->cue_set[ci]) ?
					      g_cue_colours[ci] :
					      PAD_COL_CUE_UNSET;
			pad_led(midi_ch, i, col);
		}
	} else if (mode == PAD_MODE_AUTOLOOP) {
		/* Pads 1-4: loop sizes. Dark blue = available. Green = active. */
		static const float sizes[4] = { 1.0f, 2.0f, 4.0f, 8.0f };
		float cur_bars = g_autoloop_bars[deck];
		for (int i = 1; i <= 4; i++) {
			uint8_t col = (sizes[i - 1] == cur_bars && t->looping) ?
					      PAD_COL_AUTOLOOP_ON :
					      PAD_COL_AUTOLOOP_BG;
			pad_led(midi_ch, i, col);
		}
		/* Pads 5-8 off */
		for (int i = 5; i <= 8; i++)
			pad_led(midi_ch, i, PAD_COL_OFF);
	} else if (mode == PAD_MODE_ROLL) {
		/* Yellow background; brighter on the held pad */
		static const float sizes[4] = { 1.0f, 2.0f, 4.0f, 8.0f };
		float cur_bars = g_autoloop_bars[deck];
		for (int i = 1; i <= 4; i++) {
			uint8_t col = (sizes[i - 1] == cur_bars &&
				       g_roll_active[deck] == i) ?
					      PAD_COL_ROLL_ON :
					      PAD_COL_ROLL_BG;
			pad_led(midi_ch, i, col);
		}
		for (int i = 5; i <= 8; i++)
			pad_led(midi_ch, i, PAD_COL_OFF);
	} else if (mode == PAD_MODE_MANUALLOOP) {
		/* Pad 1=loop_in, 2=loop_out, 3=halve, 4=double, 5=reloop.
         * Green when set/active, dim yellow when available, off otherwise. */
		Track *tl = &g_tracks[deck];
		pad_led(midi_ch, 1,
			(tl->loop_start > 0) ? PAD_COL_MANUALLOOP_SET :
					       PAD_COL_MANUALLOOP_BG);
		pad_led(midi_ch, 2,
			tl->looping ? PAD_COL_MANUALLOOP_SET :
				      PAD_COL_MANUALLOOP_BG);
		pad_led(midi_ch, 3,
			PAD_COL_MANUALLOOP_BG); /* halve — always available */
		pad_led(midi_ch, 4,
			PAD_COL_MANUALLOOP_BG); /* double — always available */
		pad_led(midi_ch, 5,
			tl->looping /* reloop */
				?
				PAD_COL_MANUALLOOP_SET :
				PAD_COL_MANUALLOOP_BG);
		for (int i = 6; i <= 8; i++)
			pad_led(midi_ch, i, PAD_COL_OFF);
	}
}

/* Refresh the pad mode selector button LEDs (CUES / AUTO-ROLL / LOOP etc.)
 * Uses the out binding table so addresses stay in the map file. */
static void pad_mode_leds_refresh(int deck)
{
	char side = (deck == g_side_deck[0]) ? 'a' : 'b';
	int mode = g_pad_mode[deck];

	/* All mode buttons off first, then light the active one */
	{
		char n[32];
		snprintf(n, sizeof(n), "led_pad_hotcue_%c", side);
		led_off(n);
	}
	{
		char n[32];
		snprintf(n, sizeof(n), "led_pad_roll_%c", side);
		led_off(n);
	}
	{
		char n[32];
		snprintf(n, sizeof(n), "led_pad_loop_%c", side);
		led_off(n);
	}
	{
		char n[32];
		snprintf(n, sizeof(n), "led_pad_sampler_%c", side);
		led_off(n);
	}
	{
		char n[32];
		snprintf(n, sizeof(n), "led_pad_slicer_%c", side);
		led_off(n);
	}

	if (mode == PAD_MODE_HOTCUE) {
		char n[32];
		snprintf(n, sizeof(n), "led_pad_hotcue_%c", side);
		led_on(n);
	} else if (mode == PAD_MODE_AUTOLOOP) {
		char n[32];
		snprintf(n, sizeof(n), "led_pad_loop_%c", side);
		led_on(n);
	} else if (mode == PAD_MODE_ROLL) {
		char n[32];
		snprintf(n, sizeof(n), "led_pad_roll_%c", side);
		led_on(n);
	} else if (mode == PAD_MODE_MANUALLOOP) {
		/* Manual loop uses the same LOOP button LED as autoloop — they are
         * mutually exclusive so there is no ambiguity. */
		char n[32];
		snprintf(n, sizeof(n), "led_pad_loop_%c", side);
		led_on(n);
	}
}

/* Compute loop length in frames from bar count and track BPM.
 * bars: 1.0 / 2.0 / 4.0 / 8.0 etc.
 * Returns 0 if BPM is not set. */
static uint32_t bars_to_frames(int deck, float bars)
{
	float bpm = g_tracks[deck].bpm;
	if (bpm <= 0.0f)
		return 0;
	float beat_frames = (float)g_actual_sample_rate * 60.0f / bpm;
	float beats_per_bar = 4.0f; /* 4/4 time */
	uint32_t len = (uint32_t)(bars * beats_per_bar * beat_frames + 0.5f);
	return len;
}

/* Snap start position to nearest beat boundary for clean loops */
static uint32_t snap_to_beat(int deck, uint32_t pos)
{
	Track *t = &g_tracks[deck];
	if (t->bpm <= 0.0f || t->num_frames == 0)
		return pos;
	float beat_frames = (float)g_actual_sample_rate * 60.0f / t->bpm;
	if (beat_frames <= 0.0f)
		return pos;
	/* Find nearest beat: bpm_offset + N * beat_frames */
	float offset = t->bpm_offset;
	if (offset < 0.0f)
		offset = 0.0f;
	float beats_from_offset = ((float)pos - offset) / beat_frames;
	float nearest_beat = roundf(beats_from_offset) * beat_frames + offset;
	if (nearest_beat < 0.0f)
		nearest_beat = 0.0f;
	if (nearest_beat >= (float)t->num_frames)
		nearest_beat = (float)(t->num_frames - 1);
	return (uint32_t)nearest_beat;
}

/* Engage a beat-aligned auto-loop of 'bars' bars on deck.
 * If loop is already active at this size, toggle it off. */
static void autoloop_engage(int deck, float bars)
{
	Track *t = &g_tracks[deck];
	if (!t->loaded || t->bpm <= 0.0f)
		return;

	uint32_t len = bars_to_frames(deck, bars);
	if (len == 0)
		return;

	pthread_mutex_lock(&t->lock);

	/* If already looping at exactly this size, toggle off */
	uint32_t cur_len =
		(t->loop_end > t->loop_start) ? t->loop_end - t->loop_start : 0;
	if (t->looping && cur_len == len) {
		t->looping = 0;
		pthread_mutex_unlock(&t->lock);
		g_autoloop_bars[deck] = bars;
		pad_leds_refresh(deck);
		return;
	}

	/* Snap start to nearest beat */
	uint32_t start = snap_to_beat(deck, t->pos);
	if (start + len > t->num_frames)
		start = (t->num_frames > len) ? t->num_frames - len : 0;

	t->loop_start = start;
	t->loop_end = start + len;
	t->looping = 1;

	/* Jump to loop start */
	t->pos = start;
	pthread_mutex_unlock(&t->lock);

	g_autoloop_bars[deck] = bars;
	pad_leds_refresh(deck);
	deck_leds_refresh();
}

/* Begin a roll on deck: save current position, start the loop.
 * Resume position is where playback was before the roll. */
static void roll_begin(int deck, int pad_idx, float bars)
{
	Track *t = &g_tracks[deck];
	if (!t->loaded || t->bpm <= 0.0f)
		return;

	uint32_t len = bars_to_frames(deck, bars);
	if (len == 0)
		return;

	/* Save position before entering loop */
	pthread_mutex_lock(&t->lock);
	g_roll_resume_pos[deck] = t->pos;
	uint32_t start = snap_to_beat(deck, t->pos);
	if (start + len > t->num_frames)
		start = (t->num_frames > len) ? t->num_frames - len : 0;
	t->loop_start = start;
	t->loop_end = start + len;
	t->looping = 1;
	t->pos = start;
	pthread_mutex_unlock(&t->lock);

	g_roll_active[deck] = pad_idx;
	g_autoloop_bars[deck] = bars;
	pad_leds_refresh(deck);
}

/* Release a roll: resume from saved position, stop looping */
static void roll_end(int deck)
{
	if (!g_roll_active[deck])
		return;
	Track *t = &g_tracks[deck];
	pthread_mutex_lock(&t->lock);
	t->looping = 0;
	t->pos = g_roll_resume_pos[deck];
	pthread_mutex_unlock(&t->lock);
	g_roll_active[deck] = 0;
	pad_leds_refresh(deck);
	deck_leds_refresh();
}

static void midi_send_cc(int channel, int cc, int value)
{
	if (!g_midi_out)
		return;
	uint8_t msg[3];
	msg[0] = (uint8_t)(0xB0 | ((channel - 1) & 0x0F));
	msg[1] = (uint8_t)(cc & 0x7F);
	msg[2] = (uint8_t)(value & 0x7F);
	pthread_mutex_lock(&g_midi_out_mutex);
	snd_rawmidi_write(g_midi_out, msg, 3);
	snd_rawmidi_drain(g_midi_out);
	pthread_mutex_unlock(&g_midi_out_mutex);
}

static void midi_send_note(int channel, int note, int velocity)
{
	if (!g_midi_out)
		return;
	uint8_t msg[3];
	msg[0] = (uint8_t)((velocity ? 0x90 : 0x80) | ((channel - 1) & 0x0F));
	msg[1] = (uint8_t)(note & 0x7F);
	msg[2] = (uint8_t)(velocity & 0x7F);
	pthread_mutex_lock(&g_midi_out_mutex);
	snd_rawmidi_write(g_midi_out, msg, 3);
	snd_rawmidi_drain(g_midi_out);
	pthread_mutex_unlock(&g_midi_out_mutex);
}

/* ── MIDI output binding table ───────────────────────────────────────────── */

/* Look up an output binding by name. Returns NULL if not found. */
static MidiOutBinding *midi_out_lookup(const char *name)
{
	for (int i = 0; i < g_midi_nout_bindings; i++)
		if (strcmp(g_midi_out_bindings[i].name, name) == 0)
			return &g_midi_out_bindings[i];
	return NULL;
}

static void midi_out_bind(const char *name, uint8_t status, uint8_t data1,
			  uint8_t data2)
{
	for (int i = 0; i < g_midi_nout_bindings; i++) {
		if (strcmp(g_midi_out_bindings[i].name, name) == 0) {
			g_midi_out_bindings[i].status = status;
			g_midi_out_bindings[i].data1 = data1;
			g_midi_out_bindings[i].data2 = data2;
			return;
		}
	}
	if (g_midi_nout_bindings >= MIDI_MAX_OUT_BINDINGS)
		return;
	MidiOutBinding *b = &g_midi_out_bindings[g_midi_nout_bindings++];
	snprintf(b->name, sizeof(b->name), "%s", name);
	b->status = status;
	b->data1 = data1;
	b->data2 = data2;
}

/* Send an output binding by name. Handles both 3-byte and SysEx. */
static void midi_out_send(const char *name)
{
	if (!g_midi_out)
		return;
	MidiOutBinding *b = midi_out_lookup(name);
	if (!b || b->status == 0)
		return;
	if (b->sysex_len > 0) {
		snd_rawmidi_write(g_midi_out, b->sysex, b->sysex_len);
	} else {
		uint8_t msg[3] = { b->status, b->data1, b->data2 };
		snd_rawmidi_write(g_midi_out, msg, 3);
	}
	snd_rawmidi_drain(g_midi_out);
}

/* Bind an RGB SysEx LED for NS7III performance pads.
 * NS7III SysEx format (Numark): F0 00 20 7F 03 01 pp rr gg bb F7
 *   pp = pad index (0-based), rr/gg/bb = colour 0-127
 * 'off' variant uses velocity 0 Note On on ch1 with the pad's note#. */
static void midi_out_bind_rgb(const char *name, uint8_t pad, uint8_t r,
			      uint8_t g, uint8_t b)
{
	if (pad > 127 || g_midi_nout_bindings >= MIDI_MAX_OUT_BINDINGS)
		return;
	/* Find or create entry */
	MidiOutBinding *entry = midi_out_lookup(name);
	if (!entry) {
		entry = &g_midi_out_bindings[g_midi_nout_bindings++];
		snprintf(entry->name, sizeof(entry->name), "%s", name);
	}
	entry->status = 0xF0; /* marks as SysEx */
	entry->data1 = pad;
	entry->data2 = 0;
	entry->sysex[0] = 0xF0;
	entry->sysex[1] = 0x00;
	entry->sysex[2] = 0x20;
	entry->sysex[3] = 0x7F;
	entry->sysex[4] = 0x03; /* NS7III device ID */
	entry->sysex[5] = 0x01; /* LED command      */
	entry->sysex[6] = pad;
	entry->sysex[7] = r & 0x7F;
	entry->sysex[8] = g & 0x7F;
	entry->sysex[9] = b & 0x7F;
	entry->sysex[10] = 0xF7;
	entry->sysex_len = 11;
}

/* Convenience: send a named LED on (using stored binding) or off (vel=0 Note). */
static void led_on(const char *name)
{
	midi_out_send(name);
}
static void led_off(const char *name)
{
	if (!g_midi_out)
		return;
	MidiOutBinding *b = midi_out_lookup(name);
	if (!b || b->status == 0)
		return;
	if (b->sysex_len > 0) {
		/* RGB off = same SysEx with r=g=b=0 */
		uint8_t off[11];
		memcpy(off, b->sysex, b->sysex_len);
		off[7] = off[8] = off[9] = 0;
		snd_rawmidi_write(g_midi_out, off, b->sysex_len);
	} else {
		/* Single-colour off = Note On vel 0 */
		uint8_t msg[3] = { (uint8_t)((b->status & 0xF0) == 0x90 ?
						     b->status :
						     0x90),
				   b->data1, 0 };
		snd_rawmidi_write(g_midi_out, msg, 3);
	}
	snd_rawmidi_drain(g_midi_out);
}

/* ── NS7III motor drive thread ───────────────────────────────────────────
 * NOTE: The sawtooth drive loop (CC#74 7→6→5→4→3) was WRONG.
 * The cold-init pcap shows the real protocol: CC#66=1 starts the motor,
 * CC#66=0 stops it. No continuous drive loop is needed.
 * The motor thread is kept as a stub for the g_running guard. */

/* ── Motor ramp + sawtooth state ─────────────────────────────────────────
 * Confirmed from cold-init-load-play-stop pcap (start=instant):
 *   CC#73 ChN: ramp 0→100 over 18 steps at 20Hz  (per-deck)
 *   CC#74 Ch1: sawtooth 0→5 ascending, 20Hz      (global)
 *   CC#105 ChN: =64 sent with each ramp step      (per-deck)
 * After ramp completes (18 steps), CC#73 stops; CC#74 continues.
 */
#define MOTOR_RAMP_CC 73
#define MOTOR_PITCH_CC 105
#define MOTOR_SAW_CC 74
/* Ramp steps and brake value pulled from djcmd_config.h so the user can
 * tune start/stop feel without recompiling djcmd.c:
 *   CFG_MOTOR_RAMP_STEPS — 20Hz steps to full speed (18 ≈ 0.9s "instant")
 *   CFG_MOTOR_BRAKE_VAL  — CC#68 value before stop; 0=instant, >0=brake */
#define MOTOR_RAMP_STEPS CFG_MOTOR_RAMP_STEPS
#define MOTOR_BRAKE_VAL CFG_MOTOR_BRAKE_VAL
#define MOTOR_SAW_MAX 5

static volatile int g_motor_ramp_step[MAX_TRACKS] = { 0 };
static volatile int g_motor_saw_phase = 0;

/* Ramp table: MOTOR_RAMP_STEPS values linearly interpolated 0→100.
 * Generated at runtime so CFG_MOTOR_RAMP_STEPS can be any value. */
static int g_motor_ramp_table[64]; /* max 64 steps — covers any sane config */

static void motor_ramp_table_build(void)
{
	int n = MOTOR_RAMP_STEPS;
	if (n < 1)
		n = 1;
	if (n > 64)
		n = 64;
	for (int i = 0; i < n; i++)
		g_motor_ramp_table[i] = (int)(i * 100.0f / (n - 1) + 0.5f);
	g_motor_ramp_table[n - 1] = 100; /* ensure exact 100 at last step */
}

static void *motor_thread(void *arg)
{
	(void)arg;
	motor_ramp_table_build();
	static const int motor_ch[MAX_TRACKS] = {
		CFG_MOTOR_CH_A, CFG_MOTOR_CH_B, CFG_MOTOR_CH_C, CFG_MOTOR_CH_D
	};

	while (g_running) {
		usleep(1000000 / 20);
		if (!g_midi_out)
			continue;

		/* ── Motor ramp + sawtooth ───────────────────────────────────────
         * CC#73 ChN and CC#74 Ch1 only fire during spinup (step <
         * MOTOR_RAMP_STEPS). After ramp completes the motor runs freely —
         * no further drive signals are needed or sent. */
		/* Deferred starts: motor_handoff set the flag instead of calling
		 * motor_set() directly so that one full 50ms tick separates the
		 * CC#66 stop from the CC#65 start on the same physical channel. */
		for (int dk = 0; dk < MAX_TRACKS; dk++) {
			if (g_motor_pending_start[dk]) {
				g_motor_pending_start[dk] = 0;
				if (g_tracks[dk].playing && !g_slip_motor_off[dk])
					motor_set(dk, 1);
			}
		}
		/* Auto-stop motor when track reaches its natural end */
		for (int dk = 0; dk < MAX_TRACKS; dk++) {
			if (g_motor_running[dk] && !g_tracks[dk].playing)
				motor_set(dk, 0);
		}

		int any_ramping = 0;
		for (int dk = 0; dk < MAX_TRACKS; dk++) {
			if (!g_motor_running[dk])
				continue;
			int step = g_motor_ramp_step[dk];
			if (step >= MOTOR_RAMP_STEPS)
				continue;

			int ch = motor_ch[dk];
			if (ch == 0)
				continue; /* deck not present */
			any_ramping = 1;
			midi_send_cc(ch, MOTOR_RAMP_CC,
				     g_motor_ramp_table[step]);
			/* During ramp send center pitch; call motor_sync_pitch at final step
             * so the platter reaches the correct speed for the current pitch fader. */
			if (step + 1 >= MOTOR_RAMP_STEPS)
				motor_sync_pitch(dk);
			else
				midi_send_cc(ch, MOTOR_PITCH_CC, 64);
			g_motor_ramp_step[dk] = step + 1;
		}
		if (any_ramping) {
			midi_send_cc(1, MOTOR_SAW_CC, g_motor_saw_phase);
			g_motor_saw_phase =
				(g_motor_saw_phase + 1) % (MOTOR_SAW_MAX + 1);
		}

		/* ── VU meters ───────────────────────────────────────────────────
         * CC#74 Ch1 doubles as the NS7III channel-1 VU meter level after
         * the motor ramp is complete (ramp uses 0-5, VU uses 0-127).
         * We send CC#74 Ch1 = master output level whenever no deck is
         * in ramp phase, so the physical meter follows the audio.
         * g_vu_l / g_vu_r are written by the audio thread without a mutex
         * — single float reads are safe enough for display purposes. */
		if (!any_ramping) {
			float level = g_vu_l > g_vu_r ? g_vu_l : g_vu_r;
			/* Mild log scaling so quiet passages still register on the meter */
			if (level > 0.0f) {
				float db = log10f(level) * 20.0f; /* dBFS */
				/* Map -40dBFS..0dBFS → 0..127 */
				float norm = (db + 40.0f) / 40.0f;
				if (norm < 0.0f)
					norm = 0.0f;
				if (norm > 1.0f)
					norm = 1.0f;
				level = norm;
			}
			int vu_val = (int)(level * 127.0f + 0.5f);
			midi_send_cc(1, MOTOR_SAW_CC,
				     vu_val); /* CC#74 Ch1 = VU level */
		}
	}
	return NULL;
}

/* Start or stop the platter motor for a given deck.
 *
 * Real NS7III motor protocol (from cold-init pcap, H>D direction):
 *
 *   START:
 *     CC#75 = 0 on Ch1        (global motor enable — sent once)
 *     CC#65 = 127 on ChN      (motor START)
 *     CC#69 = 0 on ChN        (FORWARD direction trigger — sent after start)
 *
 *   REVERSE mode:
 *     CC#70 = 1 on ChN        (REVERSE direction trigger — any nonzero value)
 *
 *   STOP:
 *     CC#73 = 0 on ChN        (ramp to zero)
 *     CC#66 = 127 on ChN      (motor STOP — clean stop at value 127)
 *
 *   Confirmed from motor_sweep:
 *     CC#69 = FORWARD trigger (any value → forward, sent after CC#65)
 *     CC#70 = REVERSE trigger (nonzero → reverse, zero has no effect)
 *     CC#66 = STOP register   (127 = clean stop; other values = brake)
 *     CC#68 = BRAKE register  (any value slows/stops motor)
 *     CC#67, #71, #72 = no effect
 *
 * Channel mapping (1-indexed): deck 0→Ch2, 1→Ch3, 2→Ch4, 3→Ch5
 */
static void motor_set(int deck, int on)
{
	if (deck < 0 || deck >= MAX_TRACKS)
		return;

	/* State zeroing — always do this on stop (on=0) to ensure we have a
	 * clean baseline for the next start, even if the motor was already
	 * logically off but has stale velocity state from a previous switch. */
	if (!on) {
		g_motor_settle_until[deck] = 0;
		g_motor_vel[deck] = 0.0f;
		g_jog_abs_vel[deck] = 0.0f;
		g_jog_abs_init[deck] = 0;
		/* Reset PLL on stop */
		g_pll[deck].freq = (double)g_jog_ref_delta;
		g_pll[deck].integrator = 0.0;
	}

	if (on && g_motor_running[deck])
		return;
	g_motor_running[deck] = on;

	if (on) {
		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC, &ts);
		int64_t now_ms =
			(int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
		g_motor_settle_until[deck] = now_ms + (int64_t)g_jog_settle_ms;
		g_jog_abs_init[deck] = 0;
		g_jog_abs_vel[deck] = 0.0f;
		g_pb_streak[deck] = 0;
		g_pb_mag_acc[deck] = 0.0f;
		g_motor_ramp_step[deck] = 0; /* restart ramp from step 0 */
		/* Reset PLL to nominal frequency so it re-locks cleanly after spin-up */
		g_pll[deck].freq = (double)g_jog_ref_delta;
		g_pll[deck].integrator = 0.0;
	}

	/* LED feedback */
	{
		char led_name[32];
		snprintf(led_name, sizeof(led_name), "led_deck_%c", 'a' + deck);
		if (on)
			led_on(led_name);
		else
			led_off(led_name);
	}

	if (!g_midi_out)
		return;

	static const int motor_ch[MAX_TRACKS] = {
		CFG_MOTOR_CH_A, CFG_MOTOR_CH_B, CFG_MOTOR_CH_C, CFG_MOTOR_CH_D
	};
	int ch = motor_ch[deck];
	if (ch == 0)
		return; /* deck not present */

	if (on) {
		/* Global cold-init sequence — required before first motor start.
         * Serato sends this on every cold boot. We send it on the first
         * motor_set(on=1) call to ensure hardware is in the right state. */
		static int global_init_sent = 0;
		if (!global_init_sent) {
			global_init_sent = 1;
			/* Zero out motor-related globals */
			midi_send_cc(1, 71, 0);
			midi_send_cc(1, MOTOR_SAW_CC, 0); /* CC#74=0 */
			midi_send_cc(1, MOTOR_ENABLE_CC, 0); /* CC#75=0 */
			midi_send_cc(1, 76, 0);
			midi_send_cc(1, 77, 0);
			/* Pad LED init — hardware requires this before motor engages */
			midi_send_cc(1, 78, 1);
			midi_send_cc(1, 78, 1);
			midi_send_cc(1, 78, 1);
			midi_send_cc(1, 79, 1);
			midi_send_cc(1, 79, 1);
			midi_send_cc(1, 79, 1);
			midi_send_cc(1, 80, 1);
			midi_send_cc(1, 80, 1);
			midi_send_cc(1, 80, 1);
			midi_send_cc(1, 81, 7);
			midi_send_cc(1, 81, 7);
			midi_send_cc(1, 82, 1);
			midi_send_cc(1, 82, 1);
			midi_send_cc(1, 82, 1);
			midi_send_cc(1, 83, 1);
			midi_send_cc(1, 83, 1);
			midi_send_cc(1, 83, 1);
			midi_send_cc(1, 84, 1);
			midi_send_cc(1, 84, 1);
			midi_send_cc(1, 84, 1);
			midi_send_cc(1, 85, 7);
			midi_send_cc(1, 85, 7);
			usleep(20000); /* 20ms settle */
		}

		midi_send_cc(1, MOTOR_ENABLE_CC,
			     0); /* CC#75=0 global enable          */
		midi_send_cc(ch, 65, 127); /* CC#65=127 motor START          */
		/* Direction trigger AFTER start — confirmed from motor_sweep:
         * CC#69 (any value) = FORWARD trigger
         * CC#70 (nonzero)   = REVERSE trigger */
		if (g_tracks[deck].reverse)
			midi_send_cc(ch, 70,
				     1); /* CC#70=1 REVERSE                */
		else
			midi_send_cc(ch, 69,
				     0); /* CC#69=0 FORWARD                */
		/* motor_thread sends the ramp (CC#73) and sawtooth (CC#74) */
	} else {
		midi_send_cc(ch, 66, 0); /* CC#66=0 motor STOP ONLY */
	}
}

/* Send the motor direction trigger for a deck.
 * CC#69=0 on ChN = FORWARD (any value triggers forward)
 * CC#70=1 on ChN = REVERSE (any nonzero triggers reverse)
 * Confirmed from motor_sweep: these are direction triggers, not level registers.
 * Send AFTER CC#65=127 start for reliable direction control. */
static void motor_set_direction(int deck, int reverse)
{
	if (!g_midi_out)
		return;
	static const int motor_ch[MAX_TRACKS] = {
		CFG_MOTOR_CH_A, CFG_MOTOR_CH_B, CFG_MOTOR_CH_C, CFG_MOTOR_CH_D
	};
	int ch = motor_ch[deck];
	if (ch == 0)
		return; /* deck not present (CFG_MOTOR_CH_* = 0) */
	if (reverse)
		midi_send_cc(ch, 70, 1); /* REVERSE trigger */
	else
		midi_send_cc(ch, 69, 0); /* FORWARD trigger */
}

/* Sync motor speed to current deck pitch via CC#105.
 * Formula matches Mixxx syncPhysicalMotor:
 *   effectiveRate = pitch - 1.0   (fractional deviation from 1× speed)
 *   CC#105 = clamp(round(64 + (-effectiveRate × 64)), 1, 127)  — forward play
 *   CC#105 = clamp(round(64 + ( effectiveRate × 64)), 1, 127)  — reverse play
 * 64 = nominal 33⅓ RPM; values below = faster, above = slower.
 * Called on every pitch fader move when motor is running. */
static void motor_sync_pitch(int deck)
{
	if (!g_midi_out || !g_motor_running[deck])
		return;
	static const int motor_ch[MAX_TRACKS] = {
		CFG_MOTOR_CH_A, CFG_MOTOR_CH_B, CFG_MOTOR_CH_C, CFG_MOTOR_CH_D
	};
	int ch = motor_ch[deck];
	if (ch == 0)
		return;

	float eff = g_tracks[deck].pitch - 1.0f;
	float sign = g_tracks[deck].reverse ? 1.0f : -1.0f;
	int rpm = (int)(64.0f + sign * eff * 64.0f + 0.5f);
	if (rpm < 1)
		rpm = 1;
	if (rpm > 127)
		rpm = 127;
	midi_send_cc(ch, MOTOR_PITCH_CC, rpm);
}

static void motor_probe_send(int value)
{
	if (!g_midi_out) {
		snprintf(g_motor_probe_log, sizeof(g_motor_probe_log),
			 "No MIDI output — open a device first");
		return;
	}
	uint8_t status;
	const char *type_str;
	switch (g_motor_probe_type) {
	case 1:
		status = (uint8_t)(0x90 | ((g_motor_probe_ch - 1) & 0x0F));
		type_str = "NoteOn ";
		break;
	case 2:
		status = (uint8_t)(0x80 | ((g_motor_probe_ch - 1) & 0x0F));
		type_str = "NoteOff";
		break;
	default:
		status = (uint8_t)(0xB0 | ((g_motor_probe_ch - 1) & 0x0F));
		type_str = "CC     ";
		break;
	}
	/* Use the typed helpers so midi_send_note doesn't sit unused */
	switch (g_motor_probe_type) {
	case 1:
		midi_send_note(g_motor_probe_ch, g_motor_probe_cc, value);
		break;
	case 2:
		midi_send_note(g_motor_probe_ch, g_motor_probe_cc, 0);
		break;
	default:
		midi_send_cc(g_motor_probe_ch, g_motor_probe_cc, value);
		break;
	}
	snprintf(g_motor_probe_log, sizeof(g_motor_probe_log),
		 "Sent %s ch%d  d1=%d d2=%d  raw:%02X %02X %02X", type_str,
		 g_motor_probe_ch, g_motor_probe_cc, value, status,
		 g_motor_probe_cc & 0x7F, value & 0x7F);
}

/* Sweep: send every value 0–127 on current ch/CC with 20ms gaps.
 * Runs synchronously in the UI thread — brief freeze is acceptable for a
 * diagnostic tool.  Stops sweeping as soon as the motor responds. */
static void motor_probe_sweep(int from, int to)
{
	if (!g_midi_out)
		return;
	int step = (from <= to) ? 1 : -1;
	for (int v = from; v != to + step; v += step) {
		g_motor_probe_val = v;
		motor_probe_send(v);
		usleep(20000); /* 20 ms between values */
	}
}

static MidiAction midi_action_from_name(const char *name)
{
	for (int i = 1; i < MACT_COUNT; i++)
		if (strcmp(g_mact_names[i], name) == 0)
			return (MidiAction)i;
	return MACT_NONE;
}

/* Add or replace a binding. If status==0, removes any existing binding
 * for that action instead. */
static void midi_bind(uint8_t status, uint8_t data1, MidiAction action)
{
	if (action <= MACT_NONE || action >= MACT_COUNT)
		return;
	/* Remove any existing binding for this action */
	for (int i = 0; i < g_midi_nbindings; i++) {
		if (g_midi_bindings[i].action == action) {
			/* shift down */
			memmove(&g_midi_bindings[i], &g_midi_bindings[i + 1],
				(g_midi_nbindings - i - 1) *
					sizeof(MidiBinding));
			g_midi_nbindings--;
			break;
		}
	}
	if (status == 0)
		return; /* unbind only */
	/* Also remove any existing binding for this exact (status, data1) */
	for (int i = 0; i < g_midi_nbindings; i++) {
		if (g_midi_bindings[i].status == status &&
		    g_midi_bindings[i].data1 == data1) {
			memmove(&g_midi_bindings[i], &g_midi_bindings[i + 1],
				(g_midi_nbindings - i - 1) *
					sizeof(MidiBinding));
			g_midi_nbindings--;
			break;
		}
	}
	if (g_midi_nbindings >= MIDI_MAX_BINDINGS)
		return;
	g_midi_bindings[g_midi_nbindings].status = status;
	g_midi_bindings[g_midi_nbindings].data1 = data1;
	g_midi_bindings[g_midi_nbindings].action = action;
	g_midi_bindings[g_midi_nbindings].relative = 0;
	g_midi_bindings[g_midi_nbindings].rel_acc = 0.5f;
	g_midi_nbindings++;
}

/* Set the relative-encoder flag on an already-inserted binding.
 * Call immediately after midi_bind() when loading a map file. */
static void midi_bind_set_relative(MidiAction action, int rel)
{
	for (int i = 0; i < g_midi_nbindings; i++) {
		if (g_midi_bindings[i].action == action) {
			g_midi_bindings[i].relative = rel ? 1 : 0;
			g_midi_bindings[i].rel_acc = 0.5f;
			return;
		}
	}
}

/* Look up the action for an incoming (status, data1) pair.
 * Returns MACT_NONE if not bound. */
static MidiAction midi_lookup(uint8_t status, uint8_t data1)
{
	/* For NoteOff (0x8n), also try the corresponding NoteOn (0x9n) binding.
     * This lets NoteOff events (button releases) resolve to the same action
     * as their NoteOn press — needed for SHIFT, CUE hold, jog touch, etc. */
	uint8_t try_status = status;
	for (int pass = 0; pass < 2; pass++) {
		for (int i = 0; i < g_midi_nbindings; i++)
			if (g_midi_bindings[i].status == try_status &&
			    g_midi_bindings[i].data1 == data1)
				return g_midi_bindings[i].action;
		/* Second pass: if status was NoteOff (0x8n), retry as NoteOn (0x9n) */
		if ((status & 0xF0) == 0x80)
			try_status = (status & 0x0F) | 0x90;
		else
			break;
	}
	return MACT_NONE;
}

/*
 * midi_map_load() — read ~/.config/djcmd/midi.map
 *
 * File format (one binding per line, # = comment):
 *   action_name  status_hex  data1_dec
 *
 * Examples:
 *   vol_a       B0  7      # CC7 ch1 → deck A volume
 *   crossfader  B0  0E     # CC14 ch1 → crossfader
 *   play_a      90  24     # Note 36 ch1 → play deck A
 *
 * status_hex is the raw status byte (e.g. B0 = CC ch1, B1 = CC ch2,
 * 90 = Note On ch1, 91 = Note On ch2, ...).
 */

/* ──────────────────────────────────────────────
   Settings File  (~/.config/djcmd/settings)
   ──────────────────────────────────────────────
 * Plain key=value, one per line, # = comment.
 * All Options fields plus theme name (for human readability).
 * Unknown keys are silently ignored (forward compat).
 * Called at startup (after defaults) and on every options change.
 */
static void settings_path(char *out, size_t max)
{
	const char *home = getenv("HOME");
	if (!home)
		home = "/tmp";
	snprintf(out, max, "%s/" CFG_CONFIG_DIR "/settings", home);
}

static void settings_save(void)
{
	char path[512];
	settings_path(path, sizeof(path));
	/* Ensure directory */
	char dir[512];
	snprintf(dir, sizeof(dir), "%s", path);
	char *sl = strrchr(dir, '/');
	if (sl) {
		*sl = '\0';
		mkdir(dir, 0755);
	}

	FILE *f = fopen(path, "w");
	if (!f)
		return;
	fprintf(f, "# djcmd settings — auto-saved on every option change\n");
	fprintf(f, "master_vol       = %d\n", g_master_vol);
	fprintf(f, "deck_vol         = %.3f\n",
		(double)g_opts.default_deck_vol);
	fprintf(f, "auto_gain        = %d\n", g_opts.auto_gain_default);
	fprintf(f, "auto_gain_db     = %.1f\n",
		(double)g_opts.auto_gain_target_db);
	fprintf(f, "wfm_visible_secs = %.1f\n",
		(double)g_opts.wfm_visible_secs);
	fprintf(f, "wfm_overview_bins= %d\n", g_opts.wfm_overview_bins);
	fprintf(f, "kick_threshold   = %.2f\n", (double)g_opts.kick_threshold);
	fprintf(f, "wfm_height_gamma = %.2f\n",
		(double)g_opts.wfm_height_gamma);
	fprintf(f, "theme            = %d\n", g_opts.theme_idx);
	fprintf(f, "wfm_style        = %d\n", g_opts.wfm_style);
	fprintf(f, "sync_quantize    = %d\n", g_opts.sync_quantize);
	fprintf(f, "sync_smart_range = %d\n", g_opts.sync_smart_range);
	fprintf(f, "sync_auto_handoff= %d\n", g_opts.sync_auto_handoff);
	fprintf(f, "key_lock_default = %d\n", g_opts.key_lock_default);
	fprintf(f, "vinyl_mode       = %d\n", g_opts.vinyl_mode);
	fprintf(f, "ui_fps           = %d\n", g_opts.ui_fps);
	fprintf(f, "wfm_lo_weight    = %.2f\n", (double)g_opts.wfm_lo_weight);
	fprintf(f, "wfm_mid_weight   = %.2f\n", (double)g_opts.wfm_mid_weight);
	fprintf(f, "wfm_hi_weight    = %.2f\n", (double)g_opts.wfm_hi_weight);
	fprintf(f, "wfm_color_sat    = %.2f\n", (double)g_opts.wfm_color_sat);
	fprintf(f, "wfm_color_floor  = %.3f\n", (double)g_opts.wfm_color_floor);
	fprintf(f, "wfm_anchor       = %d\n", g_opts.wfm_anchor);
	fprintf(f, "num_tracks       = %d\n", g_num_tracks);
	fprintf(f, "pitch_range_a    = %d\n", g_pitch_range[0]);
	fprintf(f, "pitch_range_b    = %d\n", g_pitch_range[1]);
	fprintf(f, "pitch_range_c    = %d\n", g_pitch_range[2]);
	fprintf(f, "pitch_range_d    = %d\n", g_pitch_range[3]);
	fprintf(f, "crossfader       = %.4f\n", (double)g_crossfader);
	fprintf(f, "active_track     = %d\n", g_active_track);
	fprintf(f, "view             = %d\n", g_view);
	fprintf(f, "panel            = %d\n", g_panel);
	fprintf(f, "fb_sort          = %d\n", g_fb_sort);
	fprintf(f, "lib_sort         = %d\n", g_lib_sort);
	fprintf(f, "gang_mode        = %d\n", g_gang_mode);
	fprintf(f, "gang_mask        = %d\n", g_gang_mask);
	if (g_lib_root[0])
		fprintf(f, "lib_root         = %s\n", g_lib_root);
	fprintf(f, "pcm_dev          = %s\n", g_pcm_dev_str);
	fclose(f);
}

static void settings_load(void)
{
	char path[512];
	settings_path(path, sizeof(path));
	FILE *f = fopen(path, "r");
	if (!f)
		return; /* first run — keep compiled-in defaults */

	char line[256];
	while (fgets(line, sizeof(line), f)) {
		/* strip comment and trailing whitespace */
		char *hash = strchr(line, '#');
		if (hash)
			*hash = '\0';
		char key[64], val[128];
		if (sscanf(line, " %63[^= ] = %127s", key, val) != 2)
			continue;

		if (!strcmp(key, "master_vol")) {
			g_opts.default_master_vol = atoi(val);
			g_master_vol = g_opts.default_master_vol;
		} else if (!strcmp(key, "deck_vol"))
			g_opts.default_deck_vol = (float)atof(val);
		else if (!strcmp(key, "auto_gain"))
			g_opts.auto_gain_default = atoi(val);
		else if (!strcmp(key, "auto_gain_db"))
			g_opts.auto_gain_target_db = (float)atof(val);
		else if (!strcmp(key, "wfm_visible_secs"))
			g_opts.wfm_visible_secs = (float)atof(val);
		else if (!strcmp(key, "wfm_overview_bins"))
			g_opts.wfm_overview_bins = atoi(val);
		else if (!strcmp(key, "kick_threshold"))
			g_opts.kick_threshold = (float)atof(val);
		else if (!strcmp(key, "wfm_height_gamma"))
			g_opts.wfm_height_gamma = (float)atof(val);
		else if (!strcmp(key, "theme"))
			g_opts.theme_idx = atoi(val);
		else if (!strcmp(key, "wfm_style"))
			g_opts.wfm_style = atoi(val);
		else if (!strcmp(key, "sync_quantize"))
			g_opts.sync_quantize = atoi(val);
		else if (!strcmp(key, "sync_smart_range"))
			g_opts.sync_smart_range = atoi(val);
		else if (!strcmp(key, "sync_auto_handoff"))
			g_opts.sync_auto_handoff = atoi(val);
		else if (!strcmp(key, "key_lock_default"))
			g_opts.key_lock_default = atoi(val);
		else if (!strcmp(key, "vinyl_mode"))
			g_opts.vinyl_mode = atoi(val);
		else if (!strcmp(key, "ui_fps"))
			g_opts.ui_fps = atoi(val);
		else if (!strcmp(key, "wfm_lo_weight"))
			g_opts.wfm_lo_weight = (float)atof(val);
		else if (!strcmp(key, "wfm_mid_weight"))
			g_opts.wfm_mid_weight = (float)atof(val);
		else if (!strcmp(key, "wfm_hi_weight"))
			g_opts.wfm_hi_weight = (float)atof(val);
		else if (!strcmp(key, "wfm_color_sat"))
			g_opts.wfm_color_sat = (float)atof(val);
		else if (!strcmp(key, "wfm_color_floor"))
			g_opts.wfm_color_floor = (float)atof(val);
		else if (!strcmp(key, "wfm_anchor"))
			g_opts.wfm_anchor = atoi(val);
		else if (!strcmp(key, "num_tracks"))
			g_num_tracks = atoi(val);
		else if (!strcmp(key, "pitch_range_a"))
			g_pitch_range[0] = atoi(val);
		else if (!strcmp(key, "pitch_range_b"))
			g_pitch_range[1] = atoi(val);
		else if (!strcmp(key, "pitch_range_c"))
			g_pitch_range[2] = atoi(val);
		else if (!strcmp(key, "pitch_range_d"))
			g_pitch_range[3] = atoi(val);
		else if (!strcmp(key, "crossfader"))
			g_crossfader = (float)atof(val);
		else if (!strcmp(key, "active_track"))
			g_active_track = atoi(val);
		else if (!strcmp(key, "view"))
			g_view = atoi(val);
		else if (!strcmp(key, "panel"))
			g_panel = atoi(val);
		else if (!strcmp(key, "fb_sort"))
			g_fb_sort = atoi(val);
		else if (!strcmp(key, "lib_sort"))
			g_lib_sort = atoi(val);
		else if (!strcmp(key, "gang_mode"))
			g_gang_mode = atoi(val);
		else if (!strcmp(key, "gang_mask"))
			g_gang_mask = atoi(val);
		else if (!strcmp(key, "pcm_dev")) {
			strncpy(g_pcm_dev_str, val, sizeof(g_pcm_dev_str) - 1);
			g_pcm_dev_str[sizeof(g_pcm_dev_str) - 1] = '\0';
		}
	}
	fclose(f);

	/* Clamp everything to valid ranges after load */
	if (g_opts.default_master_vol < 0)
		g_opts.default_master_vol = 0;
	if (g_opts.default_master_vol > 150)
		g_opts.default_master_vol = 150;
	if (g_opts.default_deck_vol < 0.0f)
		g_opts.default_deck_vol = 0.0f;
	if (g_opts.default_deck_vol > 1.5f)
		g_opts.default_deck_vol = 1.5f;
	if (g_opts.auto_gain_target_db < -24.0f)
		g_opts.auto_gain_target_db = -24.0f;
	if (g_opts.auto_gain_target_db > 0.0f)
		g_opts.auto_gain_target_db = 0.0f;
	if (g_opts.wfm_visible_secs < 1.0f)
		g_opts.wfm_visible_secs = 1.0f;
	if (g_opts.wfm_visible_secs > 16.0f)
		g_opts.wfm_visible_secs = 16.0f;
	if (g_opts.kick_threshold < 0.5f)
		g_opts.kick_threshold = 0.5f;
	if (g_opts.kick_threshold > 8.0f)
		g_opts.kick_threshold = 8.0f;
	if (g_opts.wfm_height_gamma < 0.2f)
		g_opts.wfm_height_gamma = 0.2f;
	if (g_opts.wfm_height_gamma > 1.5f)
		g_opts.wfm_height_gamma = 1.5f;
	if (g_opts.theme_idx < 0)
		g_opts.theme_idx = 0;
	if (g_opts.theme_idx >= THEME_COUNT)
		g_opts.theme_idx = 0;
	if (g_opts.wfm_style < 0 || g_opts.wfm_style > 1)
		g_opts.wfm_style = 0;
	if (g_opts.wfm_overview_bins != 2048 &&
	    g_opts.wfm_overview_bins != 4096 &&
	    g_opts.wfm_overview_bins != 8192)
		g_opts.wfm_overview_bins = CFG_WFM_OVERVIEW_BINS;
	/* Clamp ui_fps to valid 5-step range; round to nearest 5 */
	g_opts.ui_fps = ((g_opts.ui_fps + 2) / 5) * 5; /* round to nearest 5 */
	if (g_opts.ui_fps < 5)
		g_opts.ui_fps = 5;
	if (g_opts.ui_fps > 60)
		g_opts.ui_fps = 60;
	/* Advanced waveform */
	if (g_opts.wfm_lo_weight < 0.1f)
		g_opts.wfm_lo_weight = 0.1f;
	if (g_opts.wfm_lo_weight > 2.0f)
		g_opts.wfm_lo_weight = 2.0f;
	if (g_opts.wfm_mid_weight < 0.1f)
		g_opts.wfm_mid_weight = 0.1f;
	if (g_opts.wfm_mid_weight > 2.0f)
		g_opts.wfm_mid_weight = 2.0f;
	if (g_opts.wfm_hi_weight < 0.0f)
		g_opts.wfm_hi_weight = 0.0f;
	if (g_opts.wfm_hi_weight > 1.0f)
		g_opts.wfm_hi_weight = 1.0f;
	if (g_opts.wfm_color_sat < 0.2f)
		g_opts.wfm_color_sat = 0.2f;
	if (g_opts.wfm_color_sat > 3.0f)
		g_opts.wfm_color_sat = 3.0f;
	if (g_opts.wfm_color_floor < 0.0f)
		g_opts.wfm_color_floor = 0.0f;
	if (g_opts.wfm_color_floor > 0.15f)
		g_opts.wfm_color_floor = 0.15f;
	if (g_opts.wfm_anchor < 0 || g_opts.wfm_anchor > 1)
		g_opts.wfm_anchor = 0;
	/* num_tracks: only 2 or 4 are valid */
	if (g_num_tracks != 2 && g_num_tracks != 4)
		g_num_tracks = 2;
	/* pitch_range: 0=±8%, 1=±25%, 2=±50% */
	for (int i = 0; i < MAX_TRACKS; i++) {
		if (g_pitch_range[i] < 0 || g_pitch_range[i] > 2)
			g_pitch_range[i] = 0;
	}
	/* crossfader */
	if (g_crossfader < 0.0f)
		g_crossfader = 0.0f;
	if (g_crossfader > 1.0f)
		g_crossfader = 1.0f;
	/* active_track */
	if (g_active_track < 0 || g_active_track >= g_num_tracks)
		g_active_track = 0;
	/* view: 0=decks, 1=browser, 2=help — clamp to 0-1 (help not a startup state) */
	if (g_view < 0 || g_view > 2)
		g_view = 1;
	if (g_view == 2)
		g_view = 1; /* don't restore into help screen */
	/* panel: 0=browser, 1=playlist, 2=library */
	if (g_panel < 0 || g_panel > 2)
		g_panel = 0;
	/* sort orders: 0=name/alpha, 1=BPM asc, 2=BPM desc */
	if (g_fb_sort < 0 || g_fb_sort > 2)
		g_fb_sort = 0;
	if (g_lib_sort < 0 || g_lib_sort > 2)
		g_lib_sort = 0;
	/* gang: gang_mode is bool, gang_mask limited to valid deck bits */
	g_gang_mode = (g_gang_mode != 0) ? 1 : 0;
	g_gang_mask &= (1 << MAX_TRACKS) - 1;
	/* Apply master_vol to live state (default_master_vol was already set above
     * via the master_vol key handler, but sync again here for clarity) */
	g_master_vol = g_opts.default_master_vol;

	/* Second pass: read lib_root — path may contain spaces so sscanf
     * would have truncated it.  Re-scan the file looking for "lib_root = ..." */
	{
		FILE *f2 = fopen(path, "r");
		if (f2) {
			char line2[FB_PATH_MAX + 64];
			while (fgets(line2, sizeof(line2), f2)) {
				char *hash = strchr(line2, '#');
				if (hash)
					*hash = '\0';
				/* Look for "lib_root" key */
				char *p = line2;
				while (*p == ' ' || *p == '\t')
					p++;
				if (strncmp(p, "lib_root", 8) != 0)
					continue;
				p += 8;
				while (*p == ' ' || *p == '\t')
					p++;
				if (*p != '=')
					continue;
				p++;
				while (*p == ' ' || *p == '\t')
					p++;
				/* Strip trailing newline/whitespace */
				int len = (int)strlen(p);
				while (len > 0 && (p[len - 1] == '\n' ||
						   p[len - 1] == '\r' ||
						   p[len - 1] == ' '))
					p[--len] = '\0';
				if (len > 0)
					snprintf(g_lib_root, FB_PATH_MAX, "%s",
						 p);
				break;
			}
			fclose(f2);
		}
	}
}

static void midi_map_load(void)
{
	char path[512];
	midi_map_path(path, sizeof(path));
	FILE *f = fopen(path, "r");
	if (!f)
		return;
	g_midi_nbindings = 0;
	g_midi_nout_bindings = 0;
	char line[256];
	while (fgets(line, sizeof(line), f)) {
		char *hash = strchr(line, '#');
		if (hash)
			*hash = '\0';

		/* Tuning parameter: "set  name  value" */
		if (strncmp(line, "set ", 4) == 0 ||
		    strncmp(line, "set\t", 4) == 0) {
			char pname[32];
			float pval;
			if (sscanf(line + 4, "%31s %f", pname, &pval) == 2) {
				if (!strcmp(pname, "jog_smooth"))
					g_jog_smooth_alpha = pval;
				else if (!strcmp(pname, "jog_dead"))
					g_jog_dead_band = pval;
				else if (!strcmp(pname, "jog_spike"))
					g_jog_spike_thresh = pval;
				else if (!strcmp(pname, "jog_slew"))
					g_jog_slew_rate = pval;
				else if (!strcmp(pname, "jog_settle"))
					g_jog_settle_ms = (int)pval;
				else if (!strcmp(pname, "jog_scratch_revs"))
					g_jog_scratch_revs = pval;
				/* jog_ref_delta: expected vel value at on-speed, hand off platter.
                 * Read from MIDI monitor vel= column while motor runs freely. */
				else if (!strcmp(pname, "jog_ref_delta"))
					g_jog_ref_delta = pval;
				else if (!strcmp(pname, "jog_motor_dead"))
					g_jog_motor_dead = pval;
				else if (!strcmp(pname, "jog_vel_max"))
					g_jog_vel_max = pval;
				/* PLL jitter filter parameters */
				else if (!strcmp(pname, "pll_enabled"))
					g_pll_enabled = (int)pval;
				else if (!strcmp(pname, "pll_kp"))
					g_pll_kp = pval;
				else if (!strcmp(pname, "pll_ki"))
					g_pll_ki = pval;
				else if (!strcmp(pname, "pll_bandwidth"))
					g_pll_bandwidth = pval;
				/* jog_msg_rate: alternative — computes ref_delta from rpm geometry.
                 * ref_delta = 1 / (128 steps × msg_rate / (33rpm/60)) — approximate. */
				else if (!strcmp(pname, "jog_msg_rate")) {
					/* ref_delta = SAMPLE_RATE / (scratch_revs × msg_rate) */
					if (pval > 0.0f)
						g_jog_ref_delta =
							(float)g_actual_sample_rate /
							(g_jog_scratch_revs *
							 pval);
				}
			} else {
				/* String-valued set params */
				char sval[32];
				if (sscanf(line + 4, "%31s %31s", pname,
					   sval) == 2) {
					if (!strcmp(pname, "jog_type")) {
						if (!strcmp(sval, "ns7iii"))
							g_jog_type = JOG_NS7III;
						else if (!strcmp(sval,
								 "relative"))
							g_jog_type =
								JOG_RELATIVE;
						else
							g_jog_type = atoi(sval);
					}
				}
			}
			continue;
		}

		/* RGB LED binding: "rgb  name  pad_dec  r_dec  g_dec  b_dec" */
		if (strncmp(line, "rgb ", 4) == 0 ||
		    strncmp(line, "rgb\t", 4) == 0) {
			char name[32];
			unsigned int pad, r, g, b;
			if (sscanf(line + 4, "%31s %u %u %u %u", name, &pad, &r,
				   &g, &b) == 5)
				midi_out_bind_rgb(name, (uint8_t)pad,
						  (uint8_t)(r & 0x7F),
						  (uint8_t)(g & 0x7F),
						  (uint8_t)(b & 0x7F));
			continue;
		}

		/* Output binding: "out  name  status_hex  data1_dec  data2_dec" */
		if (strncmp(line, "out ", 4) == 0 ||
		    strncmp(line, "out\t", 4) == 0) {
			char name[32];
			unsigned int st, d1, d2;
			if (sscanf(line + 4, "%31s %x %u %u", name, &st, &d1,
				   &d2) == 4)
				midi_out_bind(name, (uint8_t)st, (uint8_t)d1,
					      (uint8_t)d2);
			continue;
		}

		/* Input binding: "action_name  status_hex  data1_dec  [rel]" */
		char name[64];
		char modifier[16] = "";
		unsigned int stat_hex, d1;
		int nfields = sscanf(line, "%63s %x %u %15s", name, &stat_hex,
				     &d1, modifier);
		if (nfields >= 3) {
			MidiAction act = midi_action_from_name(name);
			if (act != MACT_NONE) {
				midi_bind((uint8_t)stat_hex, (uint8_t)d1, act);
				if (nfields == 4 &&
				    (!strcmp(modifier, "rel") ||
				     !strcmp(modifier, "relative")))
					midi_bind_set_relative(act, 1);
			}
		}
	}
	fclose(f);
}

static void midi_map_save(void)
{
	char path[512];
	midi_map_path(path, sizeof(path));
	char dir[512];
	snprintf(dir, sizeof(dir), "%s", path);
	char *slash = strrchr(dir, '/');
	if (slash) {
		*slash = '\0';
		mkdir(dir, 0755);
	}
	FILE *f = fopen(path, "w");
	if (!f)
		return;
	fprintf(f, "# djcmd MIDI map — auto-generated\n");
	fprintf(f, "# Input bindings:  action_name  status_hex  data1_dec\n");
	fprintf(f,
		"# Output bindings: out  name  status_hex  data1_dec  data2_dec\n");
	fprintf(f, "# Tuning params:   set  name  value\n");
	fprintf(f,
		"# status: B0-BF=CC ch1-16  90-9F=NoteOn ch1-16  80-8F=NoteOff ch1-16\n");
	fprintf(f, "#\n");
	/* Always write jog_type so the map file is self-documenting */
	fprintf(f, "set  jog_type         %s\n",
		g_jog_type == JOG_NS7III ? "ns7iii" : "relative");
	if (g_jog_type == JOG_NS7III) {
		fprintf(f, "set  jog_scratch_revs %.0f\n",
			(double)g_jog_scratch_revs);
		fprintf(f, "set  jog_ref_delta    %.7f\n",
			(double)g_jog_ref_delta);
		fprintf(f, "set  jog_motor_dead   %.4f\n",
			(double)g_jog_motor_dead);
		fprintf(f, "set  jog_vel_max      %.1f\n",
			(double)g_jog_vel_max);
	}
	/* Write current jog tuning if non-default */
	if (g_jog_smooth_alpha != 0.12f || g_jog_dead_band != 0.005f ||
	    g_jog_spike_thresh != 0.80f || g_jog_slew_rate != 0.04f ||
	    g_jog_settle_ms != 2000) {
		fprintf(f, "set  jog_smooth  %.3f\n",
			(double)g_jog_smooth_alpha);
		fprintf(f, "set  jog_dead    %.4f\n", (double)g_jog_dead_band);
		fprintf(f, "set  jog_spike   %.3f\n",
			(double)g_jog_spike_thresh);
		fprintf(f, "set  jog_slew    %.4f\n", (double)g_jog_slew_rate);
		fprintf(f, "set  jog_settle  %d\n", g_jog_settle_ms);
	}
	fprintf(f, "#\n");
	for (int i = 0; i < g_midi_nbindings; i++) {
		MidiBinding *b = &g_midi_bindings[i];
		if (b->action > MACT_NONE && b->action < MACT_COUNT)
			fprintf(f, "%-18s  %02X  %3u%s\n",
				g_mact_names[b->action], b->status, b->data1,
				b->relative ? "  rel" : "");
	}
	if (g_midi_nout_bindings > 0) {
		fprintf(f, "\n# Output bindings (djcmd → controller)\n");
		for (int i = 0; i < g_midi_nout_bindings; i++) {
			MidiOutBinding *b = &g_midi_out_bindings[i];
			if (b->status == 0)
				continue;
			if (b->sysex_len > 0) {
				/* RGB SysEx binding */
				fprintf(f, "rgb  %-18s  %3u  %3u  %3u  %3u\n",
					b->name, b->data1, /* pad */
					b->sysex_len > 7 ? b->sysex[7] :
							   0, /* r */
					b->sysex_len > 8 ? b->sysex[8] :
							   0, /* g */
					b->sysex_len > 9 ? b->sysex[9] :
							   0); /* b */
			} else {
				fprintf(f, "out  %-18s  %02X  %3u  %3u\n",
					b->name, b->status, b->data1, b->data2);
			}
		}
	}
	fclose(f);
}

/* Write a starter map file when no map exists for a newly connected device.
 * Called after midi_map_load returns zero bindings (first time this device
 * has been seen).
 *
 * Detection: if the ALSA device name contains a known controller identifier
 * (case-insensitive substring match), the full pre-mapped layout for that
 * controller is written.  Otherwise a generic template with instructions is
 * written and the user completes setup via MIDI Learn.
 *
 * Adding a new controller: add a strstr() check for its ALSA name substring
 * and an fputs() block containing the full map content. */
static void midi_map_write_generic_defaults(void)
{
	char path[512];
	midi_map_path(path, sizeof(path));
	/* Never overwrite an existing file */
	FILE *check = fopen(path, "r");
	if (check) {
		fclose(check);
		return;
	}

	/* Ensure config directory exists */
	char dir[512];
	snprintf(dir, sizeof(dir), "%s", path);
	char *slash = strrchr(dir, '/');
	if (slash) {
		*slash = '\0';
		mkdir(dir, 0755);
	}

	FILE *f = fopen(path, "w");
	if (!f)
		return;

	/* ── Identify the connected device ──────────────────────────────────────
     * Look up the human name for g_midi_dev_str in g_midi_devlist[].
     * A case-insensitive substring match selects the known-good map. */
	const char *dev_name = "";
	for (int i = 0; i < g_midi_ndevices; i++) {
		if (strcmp(g_midi_devlist[i].dev, g_midi_dev_str) == 0) {
			dev_name = g_midi_devlist[i].name;
			break;
		}
	}

	/* Case-fold the device name for matching */
	char dev_lower[128] = "";
	for (int i = 0; dev_name[i] && i < (int)sizeof(dev_lower) - 1; i++)
		dev_lower[i] =
			(char)((unsigned char)dev_name[i] >= 'A' &&
					       (unsigned char)dev_name[i] <=
						       'Z' ?
				       dev_name[i] + 32 :
				       dev_name[i]);

	/* ── Numark NS7 III ──────────────────────────────────────────────────────
     * Matches ALSA names: "NS7 III", "NS7III", "Numark NS7" (any variant).
     * Writes the complete pre-mapped layout including all EQ/filter/gain CCs,
     * jog wheel tuning, motor start/stop, and all LED output bindings. */
	if (strstr(dev_lower, "ns7")) {
		snprintf(g_fb_status, sizeof(g_fb_status),
			 "NS7III detected: writing map to %s", path);
		fputs(NS7III_MAP, f);
	} else {
		/* ── Check ~/.config/djcmd/maps/<device>.map ────────────────────────
         * If a pre-built map exists in the maps/ subdirectory, copy it to
         * the config dir and use it instead of the generic template.
         *
         * To add support for a new controller:
         *   1. Get the sanitised device name from the MIDI tab (ESC → MIDI)
         *   2. Drop <device_name>.map into ~/.config/djcmd/maps/
         *   3. Reconnect the device — djcmd installs it automatically
         *
         * Example: Numark Mixtrack Pro 3 → maps/mixtrack_3.map
         *          (device name shown in MIDI tab as "mixtrack_3")
         *
         * The device name is the ALSA name lowercased, spaces→_, non-alphanum
         * stripped — always shown in the MIDI tab when a device is connected. */
		/* Sanitise device name: lowercase already done (dev_lower),
         * now replace spaces with _ and strip non-alphanum characters. */
		char san[128] = "";
		{
			int si = 0;
			for (int ci = 0;
			     dev_lower[ci] && si < (int)sizeof(san) - 1; ci++) {
				char c = dev_lower[ci];
				if ((c >= 'a' && c <= 'z') ||
				    (c >= '0' && c <= '9'))
					san[si++] = c;
				else if (c == ' ' || c == '_')
					san[si++] = '_';
			}
			san[si] = '\0';
			if (si == 0)
				snprintf(san, sizeof(san), "unknown");
		}
		char maps_path[512];
		const char *home_ml = getenv("HOME");
		if (!home_ml)
			home_ml = "/tmp";
		snprintf(maps_path, sizeof(maps_path),
			 "%s/" CFG_CONFIG_DIR "/maps/%s.map", home_ml, san);
		FILE *maps_f = fopen(maps_path, "r");
		if (maps_f) {
			/* Found a pre-built map — copy it to the config dir */
			fseek(maps_f, 0, SEEK_END);
			long maps_sz = ftell(maps_f);
			rewind(maps_f);
			if (maps_sz > 0 && maps_sz < 1024 * 512) {
				char *maps_buf =
					(char *)malloc((size_t)maps_sz + 1);
				if (maps_buf) {
					size_t maps_read =
						fread(maps_buf, 1,
						      (size_t)maps_sz, maps_f);
					maps_buf[maps_read] = '\0';
					fwrite(maps_buf, 1, maps_read, f);
					free(maps_buf);
					snprintf(g_fb_status,
						 sizeof(g_fb_status),
						 "Map loaded from maps/%s.map",
						 san);
				}
			}
			fclose(maps_f);
			fclose(f);
			midi_map_load();
			return;
		}
		/* ── Generic template — any unrecognised controller ─────────────────
         * Writes instructions and format reference only.  The user completes
         * setup via MIDI Learn (L key) in the MIDI IN tab. */
		snprintf(g_fb_status, sizeof(g_fb_status),
			 "New controller — starter map written to %s",
			 path);
		fputs("# djcmd MIDI map\n"
		      "# Generated automatically on first use for this controller.\n"
		      "# Edit freely — this file is only written once.\n"
		      "#\n"
		      "# HOW TO SET UP YOUR CONTROLLER\n"
		      "# ─────────────────────────────\n"
		      "# 1. Open the MIDI IN tab (ESC → MIDI).\n"
		      "# 2. Use j/k to select an action, press L to learn it.\n"
		      "# 3. Move the knob/fader or press the button on your controller.\n"
		      "# 4. Press S to save when done.\n"
		      "#\n"
		      "# Format reference:\n"
		      "#   Input binding:   action_name  status_hex  data1_dec\n"
		      "#   Output binding:  out  name    status_hex  data1_dec  data2_dec\n"
		      "#   Jog parameter:   set  name    value\n"
		      "#\n"
		      "# status_hex: B0-BF=CC ch1-16 | 90-9F=NoteOn ch1-16 | E0-EF=PitchBend ch1-16\n"
		      "#\n"
		      "# ── JOG WHEEL SETUP ────────────────────────────────────────────────\n"
		      "# Standard relative encoders (most controllers without motorized platters):\n"
		      "#   jog_spin_a / jog_spin_b send CC with centre-64 relative deltas.\n"
		      "#   No pitch-bend signal, no touch sensor required.\n"
		      "#\n"
		      "# Use MIDI Learn (W key) for jog-wheel paired learn which captures\n"
		      "# spin CC + pitch bend in one gesture.\n"
		      "#\n"
		      "# For motorized controllers (e.g. Numark NS7III):\n"
		      "#   set  jog_type  ns7iii       — enables absolute-position + pitch-bend mode\n"
		      "#   set  jog_scratch_revs  80182  — 33 rpm: frames per rev at 44100 Hz\n"
		      "#   set  jog_ref_delta  0.0078125  — tune via MIDI monitor (J key) while\n"
		      "#                                    motor runs freely, hand off platter\n"
		      "#   set  jog_motor_dead  0.05     — dead band around reference speed\n"
		      "#   set  jog_vel_max    10.0\n"
		      "#   set  jog_smooth  0.08\n"
		      "#   set  jog_dead    0.08\n"
		      "#   set  jog_spike   0.75\n"
		      "#   set  jog_slew    0.030\n"
		      "#   set  jog_settle  2000\n"
		      "#\n"
		      "# ── MOTOR CONTROL ───────────────────────────────────────────────────\n"
		      "# For controllers with motorized platters, bind motor start/stop outputs:\n"
		      "#   out  motor_start_a  <status_hex>  <cc_or_note_dec>  127\n"
		      "#   out  motor_stop_a   <status_hex>  <cc_or_note_dec>  127\n"
		      "# Use the Motor Probe tool (M key in MIDI OUT tab) to discover\n"
		      "# the correct message for your controller.\n"
		      "# Leave unbound if your controller has no motorized platters.\n"
		      "#\n"
		      "# ── LED / INDICATOR OUTPUTS ─────────────────────────────────────────\n"
		      "# djcmd drives controller LEDs automatically when state changes.\n"
		      "# Bind the relevant output names to the correct MIDI addresses:\n"
		      "#   out  led_sync_a         <status>  <note>  127  — deck A sync LED\n"
		      "#   out  led_slip_a         <status>  <note>  127  — deck A slip LED\n"
		      "#   out  led_loop_a         <status>  <note>  127  — deck A loop LED\n"
		      "#   out  led_pitch_center_a <status>  <note>  127  — deck A pitch=0 LED\n"
		      "#   out  led_cue_1_a        <status>  <note>  127  — deck A cue 1 LED\n"
		      "#   out  led_deck_1         <status>  <note>  127  — [1] button LED\n"
		      "# Mirror with _b suffix for deck B. Discover addresses via MIDI monitor\n"
		      "# while pressing each lit button on your controller.\n"
		      "#\n"
		      "# ── ADD YOUR BINDINGS BELOW ─────────────────────────────────────────\n"
		      "# Use MIDI Learn in the MIDI IN tab rather than editing by hand.\n",
		      f);
	}

	fclose(f);

	/* Immediately load the defaults we just wrote */
	midi_map_load();
}

/* ──────────────────────────────────────────────
   MIDI Thread  (raw MIDI, non-blocking)
   ────────────────────────────────────────────── */
/* ── NS7III absolute jog position update ────────────────────────────────
 * Called from both the CC spin handler and the PB handler.
 * Reconstructs the 21-bit absolute position from latest coarse + fine,
 * diffs against the previous position (handling 0/1 wraparound),
 * and writes the resulting velocity (frac-of-rev per message) into
 * g_jog_abs_vel[deck] with light exponential smoothing.
 * Also dispatches to scratch / nudge / motor-vel as appropriate.
 *
 * coarse: 0-127  (CC d2)
 * fine:   0-16383 (raw14 pitch bend, 0=fully CCW, 8192=center, 16383=fully CW)
 */
static void ns7iii_update_jog(int deck, int coarse, int fine)
{
	if (deck < 0 || deck >= MAX_TRACKS)
		return;

	/* Record timestamp of this message so audio thread knows we're alive */
	{
		struct timespec _ts;
		clock_gettime(CLOCK_MONOTONIC, &_ts);
		g_jog_last_msg_ms[deck] =
			(int64_t)_ts.tv_sec * 1000 + _ts.tv_nsec / 1000000;
	}

	/* Reconstruct normalised position 0.0–1.0 per rev */
	float pos =
		((float)coarse * (float)JOG_NS7III_FINE_RANGE + (float)fine) /
		((float)JOG_NS7III_STEPS * (float)JOG_NS7III_FINE_RANGE);

	if (!g_jog_abs_init[deck]) {
		g_jog_abs_pos[deck] = pos;
		g_jog_abs_init[deck] = 1;
		g_jog_abs_vel[deck] = 0.0f;
		return;
	}

	/* Delta with wraparound */
	float delta = pos - g_jog_abs_pos[deck];
	if (delta > 0.5f)  delta -= 1.0f;
	if (delta < -0.5f) delta += 1.0f;

	g_jog_abs_pos[deck] = pos;

	/* Spike rejection (hand off only) */
	if (!g_jog_touched[deck]) {
		float spike_thresh = g_jog_ref_delta * 8.0f;
		if (fabsf(delta) > spike_thresh) {
			return;
		}
	}

	/* Direction-aware smoothing for the velocity estimate (hand-off path) */
	float smooth_alpha = g_jog_smooth_alpha;
	if ((delta < 0.0f && g_motor_vel[deck] > 0.25f) ||
	    (delta > 0.0f && g_motor_vel[deck] < -0.25f)) {
		smooth_alpha = 0.8f;
	}
	g_jog_abs_vel[deck] = g_jog_abs_vel[deck] * (1.0f - smooth_alpha) +
			      delta * smooth_alpha;

	if (g_motor_running[deck]) {
		if (g_motor_settle_until[deck] == 0) {
			float raw_vel;

			if (g_jog_touched[deck]) {
				/* ── 1:1 MASTER CLOCK VELOCITY ──
				 * Audio thread advances pos by (pitch + g_motor_vel).
				 * We set g_motor_vel = (delta / ref_delta) - 1.0.
				 * If pitch=1.0, speed = 1.0 + delta/ref_delta - 1.0 = delta/ref_delta.
				 * This provides 1:1 tracking without jumping jt->pos directly. */
				if (g_pll_enabled) {
					float pll_err = (float)((double)delta - g_pll[deck].freq);
					raw_vel = pll_err / (float)g_pll[deck].freq;
				} else {
					raw_vel = (delta / g_jog_ref_delta) - 1.0f;
				}
				if (raw_vel > g_jog_vel_max) raw_vel = g_jog_vel_max;
				if (raw_vel < -g_jog_vel_max) raw_vel = -g_jog_vel_max;
				
				/* Alpha=1.0 for instant hand response */
				g_motor_vel[deck] = raw_vel;

			} else if (g_pll_enabled) {
				/* PLL path (hand off) */
				PLLState *pll = &g_pll[deck];
				double err = (double)delta - pll->freq;
				pll->integrator += err;
				double int_clamp = (double)g_jog_ref_delta * 0.20;
				if (pll->integrator > int_clamp) pll->integrator = int_clamp;
				if (pll->integrator < -int_clamp) pll->integrator = -int_clamp;
				pll->freq += (double)g_pll_kp * err + (double)g_pll_ki * pll->integrator;
				
				if (g_opts.vinyl_mode) {
					g_motor_vel[deck] = 0.0f;
				} else {
					float pll_err2 = (float)((double)delta - pll->freq);
					float band2 = g_pll_bandwidth * g_jog_ref_delta;
					raw_vel = (fabsf(pll_err2) < band2) ? 0.0f : pll_err2 / (float)pll->freq;
					if (raw_vel > g_jog_vel_max) raw_vel = g_jog_vel_max;
					if (raw_vel < -g_jog_vel_max) raw_vel = -g_jog_vel_max;
					float diff2 = raw_vel - g_motor_vel[deck];
					float alpha2 = (fabsf(diff2) > 0.5f) ? 0.55f : 0.15f;
					g_motor_vel[deck] = g_motor_vel[deck] * (1.0f - alpha2) + raw_vel * alpha2;
				}
			} else {
				/* Velocity path (hand off) */
				if (g_jog_type == JOG_NS7III) {
					if (g_opts.vinyl_mode) {
						g_motor_vel[deck] = 0.0f;
					} else {
						raw_vel = (delta / g_jog_ref_delta) - 1.0f;
						if (raw_vel > g_jog_motor_dead) raw_vel -= g_jog_motor_dead;
						else if (raw_vel < -g_jog_motor_dead) raw_vel += g_jog_motor_dead;
						else raw_vel = 0.0f;
						if (raw_vel > g_jog_vel_max) raw_vel = g_jog_vel_max;
						if (raw_vel < -g_jog_vel_max) raw_vel = -g_jog_vel_max;
						g_motor_vel[deck] = raw_vel;
					}
				} else {
					raw_vel = (g_jog_abs_vel[deck] / g_jog_ref_delta) - 1.0f;
					if (raw_vel > g_jog_motor_dead) raw_vel -= g_jog_motor_dead;
					else if (raw_vel < -g_jog_motor_dead) raw_vel += g_jog_motor_dead;
					else raw_vel = 0.0f;
					if (raw_vel > g_jog_vel_max) raw_vel = g_jog_vel_max;
					if (raw_vel < -g_jog_vel_max) raw_vel = -g_jog_vel_max;
					float diff = raw_vel - g_motor_vel[deck];
					float alpha = (fabsf(diff) > 0.5f) ? 0.55f : 0.15f;
					g_motor_vel[deck] = g_motor_vel[deck] * (1.0f - alpha) + raw_vel * alpha;
				}
			}
		}
	} else {
		/* Motor off */
		if (g_jog_touched[deck]) {
			float raw_vel = (delta / g_jog_ref_delta) - 1.0f;
			if (raw_vel > g_jog_vel_max) raw_vel = g_jog_vel_max;
			if (raw_vel < -g_jog_vel_max) raw_vel = -g_jog_vel_max;
			g_motor_vel[deck] = raw_vel;
		} else {
			float nudge_step = g_jog_abs_vel[deck] * 8.0f;
			g_jog_nudge[deck] += nudge_step;
			if (g_jog_nudge[deck] > 0.15f) g_jog_nudge[deck] = 0.15f;
			if (g_jog_nudge[deck] < -0.15f) g_jog_nudge[deck] = -0.15f;
		}
	}
}

/* ── Motor handoff on deck layer switch ──────────────────────────────────────
 * When a physical side switches from old_dk to new_dk:
 *   - Stop the motor on old_dk (the platter is no longer under this side's
 *     control; the other side or nothing will drive it going forward).
 *   - Start the motor on new_dk if that deck is currently playing.
 *     If it isn't playing, leave the motor off — the user starts it manually.
 */
static void motor_handoff(int old_dk, int new_dk)
{
	if (old_dk == new_dk)
		return;

	static const int motor_ch[MAX_TRACKS] = {
		CFG_MOTOR_CH_A, CFG_MOTOR_CH_B, CFG_MOTOR_CH_C, CFG_MOTOR_CH_D
	};

	/* ALWAYS stop the motor on the deck we're leaving.
	 * The NS7III hardware requires a clean release (CC 66=0) on the current
	 * channel before it will accept a start command on a different layer
	 * channel (e.g. switching from Ch2 to Ch4 for the same platter). 
	 * We send this regardless of g_motor_running to ensure hardware sync. */
	if (old_dk >= 0 && old_dk < MAX_TRACKS) {
		g_slip_motor_off[old_dk] = 1;
		g_jog_touched[old_dk] = 0;
		g_motor_vel[old_dk] = 0.0f;

		int ch_old = motor_ch[old_dk];
		if (ch_old > 0) {
			motor_set(old_dk, 0); /* software reset */
			midi_send_cc(ch_old, 66, 0); /* hardware force stop */
		}
	}

	/* Prepare the new deck. If it's playing, the motor_thread will pick
	 * up the pending start and call motor_set(new_dk, 1) on the new channel. */
	if (new_dk >= 0 && new_dk < MAX_TRACKS) {
		g_slip_motor_off[new_dk] = 0;
		/* Force a clean position baseline on the first encoder message
		 * after the switch, avoiding a stale-delta spike from the
		 * previous time this deck was active.  Also clear motor_vel so
		 * that a residual non-zero value from a previous session does not
		 * falsely trigger scratch processing in mix_and_write. */
		g_jog_abs_init[new_dk] = 0;
		g_motor_vel[new_dk] = 0.0f;
		if (g_tracks[new_dk].playing)
			g_motor_pending_start[new_dk] = 1;
	}
}

/* ── Side-to-deck rebinding (NS7III layer switching) ─────────────────────────
 * The NS7III has 2 physical platters but djcmd supports 4 virtual decks.
 * Layer switching maps each physical side to a different virtual deck:
 *
 *   side 0 = LEFT  platter (MIDI ch2) — deck_sel [1] → Deck A, [3] → Deck C
 *   side 1 = RIGHT platter (MIDI ch3) — deck_sel [2] → Deck B, [4] → Deck D
 *
 * When a deck selector button is pressed, this function rebinds all per-platter
 * MIDI actions from the old deck's action slot to the new deck's slot.
 * Actions that dispatch through g_side_deck[] (SHIFT, CUE, PAD, etc.) do not
 * need rebinding — they automatically route to the correct deck at dispatch.
 *
 * side  : 0 = left platter (ch2, deck A↔C),  1 = right platter (ch3, deck B↔D)
 * new_dk: 0=A, 1=B, 2=C, 3=D
 *
 * Groups that are per-deck and belong to the platter side:
 *   DECK_PITCH, PLAY, LOOP_IN/OUT/DOUBLE/HALF, KEY_LOCK, SLIP_MODE, REVERSE,
 *   STRIP, JOG_TOUCH, JOG_SPIN, JOG_PB, PITCH_RANGE, PITCH_BEND,
 *   MOTOR_TOGGLE, MOTOR_ON, MOTOR_OFF, CUE_SET/JUMP/DELETE (shared pads),
 *   SYNC_SLAVE
 *
 * NOT remapped (mixer section, physically separate):
 *   DECK_VOL, EQ_LOW/MID/HIGH, GAIN, FILTER, CROSSFADER, MASTER_VOL,
 *   LIB_*, NUDGE_FWD/BACK (global buttons), DECK_SEL (the buttons themselves)
 */
static void side_restack(int side, int new_dk)
{
	/* The NS7III hardware switches MIDI channels (Ch2/Ch3 -> Ch4/Ch5) when
	 * the layer button is pressed.  For this controller, we rely on the
	 * explicit channel-based bindings in the MIDI map rather than dynamic
	 * remapping of the same messages. */
	if (g_jog_type == JOG_NS7III) {
		g_side_deck[side] = new_dk;
		return;
	}

	/* Per-deck action groups that live on the platter side.
     * Each entry is the base action for deck A; B=base+1, C=base+2, D=base+3. */
	static const MidiAction platter_groups[] = {
		MACT_DECK_PITCH_A,   MACT_PITCH_LSB_A,	 MACT_PLAY_A,
		MACT_LOOP_IN_A,	     MACT_LOOP_OUT_A,	 MACT_LOOP_DOUBLE_A,
		MACT_LOOP_HALF_A,    MACT_KEY_LOCK_A,	 MACT_SLIP_MODE_A,
		MACT_REVERSE_A,	     MACT_BLEEP_A,	 MACT_STRIP_A,
		MACT_JOG_TOUCH_A,    MACT_JOG_SPIN_A,	 MACT_JOG_PB_A,
		MACT_PITCH_RANGE_A,  MACT_PITCH_BEND_A,	 MACT_MOTOR_TOGGLE_A,
		MACT_MOTOR_ON_A,     MACT_MOTOR_OFF_A,	 MACT_SYNC_SLAVE_A,
		MACT_NONE /* sentinel */
	};

	/* The old deck for this side is what g_side_deck currently says */
	int old_dk = g_side_deck[side];
	if (old_dk == new_dk)
		return; /* nothing to do */

	for (int g = 0; platter_groups[g] != MACT_NONE; g++) {
		MidiAction old_act = (MidiAction)(platter_groups[g] + old_dk);
		MidiAction new_act = (MidiAction)(platter_groups[g] + new_dk);

		/* Find the binding for old_act and retarget it to new_act */
		for (int i = 0; i < g_midi_nbindings; i++) {
			if (g_midi_bindings[i].action == old_act) {
				uint8_t st = g_midi_bindings[i].status;
				uint8_t d1 = g_midi_bindings[i].data1;
				int rel = g_midi_bindings[i].relative;
				midi_bind(st, d1, new_act);
				if (rel)
					midi_bind_set_relative(new_act, 1);
				break; /* midi_bind may have shifted the table; restart outer */
			}
		}
	}

	/* Also remap hot cue pads — CUE_SET/JUMP/DELETE groups are not per-deck
     * in the action enum (they're per-cue-number, not per-deck), so they
     * are left as-is.  The active deck for pads follows g_active_track which
     * is updated by the deck_sel handler after this call. */

	g_side_deck[side] = new_dk;
}

/* ── Menu mapping helper ── */
static int menu_to_mact(int sel) {
	static const struct { MidiAction f, l; } cats_local[] = {
		{ MACT_DECK_VOL_A, MACT_BOOTH_VOL },
		{ MACT_PLAY_A, MACT_CUE_ACTIVE_D },
		{ MACT_CUE_SET_1, MACT_CUE_DELETE_4 },
		{ MACT_SYNC_SLAVE_A, MACT_NUDGE_BACK_B },
		{ MACT_LOOP_TOGGLE, MACT_LOOP_HALF_D },
		{ MACT_KEY_LOCK_A, MACT_BLEEP_D },
		{ MACT_STRIP_A, MACT_JOG_PB_D },
		{ MACT_LIB_ENCODER, MACT_PANEL_LIBRARY },
		{ MACT_PITCH_RANGE_A, MACT_DECK_SEL_4 },
		{ MACT_SHIFT_A, MACT_PAD_8_B },
		{ MACT_PARAM_LEFT_A, MACT_GRID_SNAP_B },
		{ 0, 0 }
	};
	int cur = 1;
	for (int i = 0; cats_local[i].f != 0; i++) {
		cur++; /* skip header */
		int len = cats_local[i].l - cats_local[i].f + 1;
		if (sel >= cur && sel < cur + len) return cats_local[i].f + (sel - cur);
		cur += len;
	}
	if (sel == cur) return -2; /* Panic button marker */
	return -1; /* header or out of bounds */
}

static void handle_midi(uint8_t status, uint8_t data1, uint8_t data2)
{
	g_midi_last_status = status;
	g_midi_last_d1 = data1;
	g_midi_last_d2 = data2;

	/* ── MIDI monitor: capture raw message into ring buffer ── */
	if (g_midi_mon_open) {
		MidiMonEntry *me = &g_midi_mon_buf[g_midi_mon_head];
		me->status = status;
		me->d1 = data1;
		me->d2 = data2;
		me->matched_act = (int)midi_lookup(status, data1);
		g_midi_mon_head = (g_midi_mon_head + 1) % MIDI_MON_SIZE;
		if (g_midi_mon_count < MIDI_MON_SIZE)
			g_midi_mon_count++;
	}

	/* ── Learn mode: capture this message and bind it ── */
	if (g_midi_learn_active || g_midi_learn_jog_pair) {
		uint8_t type = status & 0xF0;

		/* ── Paired jog learn: capture CC spin, pitch bend, and touch ──────────
         *
         * Physical reality: touching the platter triggers all three signals.
         * Wait for all three in any order. One natural gesture captures everything.
         *
         * got_spin  (jog_learn_jog_spin_status != 0) — CC B0 spin bound
         * got_pb    (g_midi_learn_jog_step & 2)      — pitch bend E0 bound
         * got_touch (g_midi_learn_jog_step & 1)      — Note On touch bound
         */
		if (g_midi_learn_jog_pair) {
			int got_spin = (g_midi_learn_jog_spin_status != 0);
			int got_pb = (g_midi_learn_jog_step & 2);
			int got_touch = (g_midi_learn_jog_step & 1);

			if (!got_spin && type == 0xB0 && data2 != 64) {
				/* CC spin */
				MidiAction spin_act =
					(MidiAction)(MACT_JOG_SPIN_A +
						     g_midi_learn_jog_deck);
				midi_bind(status, data1, spin_act);
				g_midi_learn_jog_spin_status = status;
				g_midi_learn_jog_spin_d1 = data1;
				got_spin = 1;
			}

			if (!got_pb && type == 0xE0) {
				int pb14 = ((int)data2 << 7) | (int)data1;
				if (pb14 != 8192) {
					/* Pitch bend spin — bind to jog_pb_X */
					MidiAction pb_act =
						(MidiAction)(MACT_JOG_PB_A +
							     g_midi_learn_jog_deck);
					midi_bind(status, 0, pb_act);
					g_midi_learn_jog_step |= 2;
					got_pb = 1;
				}
			}

			if (!got_touch && type == 0x90 && data2 > 0) {
				/* Note On → touch */
				MidiAction touch_act =
					(MidiAction)(MACT_JOG_TOUCH_A +
						     g_midi_learn_jog_deck);
				midi_bind(status, data1, touch_act);
				g_midi_learn_jog_step |= 1;
				got_touch = 1;
			}

			if (got_spin && got_pb && got_touch) {
				/* All three captured — complete */
				g_midi_learn_jog_pair = 0;
				g_midi_learn_jog_step = 0;
				g_midi_learn_jog_spin_status = 0;
				g_midi_learn_jog_spin_d1 = 0;
				g_midi_learn_active = 0;
				midi_map_save();
			}
			return;
		}

		/* ── Single-action learn ── */
		MidiAction learning = (MidiAction)g_midi_learn_sel;

		/* Categorise what message type this action expects:
         *   ENCODER  — relative CC (jog spin, lib_encoder, pitch_bend)
         *   NOTE     — buttons, toggles, hot cues, loop, slip, reverse, etc.
         *   ABSOLUTE — faders, EQ, crossfader, volume, filter, strip search
         */
		int is_encoder = (learning >= MACT_JOG_SPIN_A &&
				  learning <= MACT_JOG_SPIN_D) ||
				 (learning >= MACT_JOG_PB_A &&
				  learning <= MACT_JOG_PB_D) ||
				 (learning == MACT_LIB_ENCODER) ||
				 (learning >= MACT_PITCH_BEND_A &&
				  learning <= MACT_PITCH_BEND_D) ||
				 (learning >= MACT_FX_KNOB_1_A &&
				  learning <= MACT_FX_WET_B);

		int is_pb_action = (learning >= MACT_JOG_PB_A &&
				    learning <= MACT_JOG_PB_D);

		int is_note =
			(learning >= MACT_JOG_TOUCH_A &&
			 learning <= MACT_JOG_TOUCH_D) ||
			(learning == MACT_LIB_ENCODER_TOUCH) ||
			(learning >= MACT_PLAY_A && learning <= MACT_PLAY_D) ||
			(learning >= MACT_CUE_SET_1 &&
			 learning <= MACT_CUE_SET_4) ||
			(learning >= MACT_CUE_JUMP_1 &&
			 learning <= MACT_CUE_JUMP_4) ||
			(learning >= MACT_CUE_DELETE_1 &&
			 learning <= MACT_CUE_DELETE_4) ||
			(learning >= MACT_SYNC_SLAVE_A &&
			 learning <= MACT_SYNC_SLAVE_D) ||
			(learning == MACT_NUDGE_FWD) ||
			(learning == MACT_NUDGE_BACK) ||
			(learning == MACT_LOOP_TOGGLE) ||
			(learning >= MACT_LOOP_IN_A &&
			 learning <= MACT_LOOP_HALF_D) ||
			(learning >= MACT_KEY_LOCK_A &&
			 learning <= MACT_KEY_LOCK_D) ||
			(learning >= MACT_SLIP_MODE_A &&
			 learning <= MACT_SLIP_MODE_D) ||
			(learning >= MACT_REVERSE_A &&
			 learning <= MACT_REVERSE_D) ||
			(learning >= MACT_LIB_SELECT &&
			 learning <= MACT_LIB_FWD) ||
			(learning >= MACT_LIB_LOAD_A &&
			 learning <= MACT_LIB_LOAD_D) ||
			(learning == MACT_PANEL_FILES) ||
			(learning == MACT_PANEL_LIBRARY) ||
			(learning >= MACT_PITCH_RANGE_A &&
			 learning <= MACT_PITCH_RANGE_D) ||
			(learning >= MACT_MOTOR_ON_A &&
			 learning <= MACT_MOTOR_OFF_D) ||
			(learning >= MACT_MOTOR_TOGGLE_A &&
			 learning <= MACT_MOTOR_TOGGLE_D) ||
			(learning >= MACT_DECK_SEL_1 &&
			 learning <= MACT_DECK_SEL_4);

		int accepted = 0;
		if (is_encoder) {
			if (is_pb_action) {
				/* jog_pb_X: only accept pitch bend (0xE0), bind with data1=0 */
				if (type == 0xE0) {
					int pb14 =
						((int)data2 << 7) | (int)data1;
					if (pb14 != 8192) {
						midi_bind(status, 0, learning);
						g_midi_learn_active = 0;
						return;
					}
				}
			} else {
				/* All other encoders: accept relative CC (data2 != 64) */
				if (type == 0xB0 && data2 != 64) {
					accepted = 1;
				} else if (type == 0xE0) {
					/* Legacy: allow 0xE0 for non-jog_pb encoders (old map files) */
					int pb14 =
						((int)data2 << 7) | (int)data1;
					if (pb14 != 8192) {
						midi_bind(status, 0, learning);
						g_midi_learn_active = 0;
						return;
					}
				}
			}
		} else if (is_note) {
			/* Only accept Note On press — never bind a CC to a button action */
			accepted = (type == 0x90 && data2 > 0);
		} else {
			/* Absolute CC (fader, EQ, crossfader, etc.) — any CC with data2 > 0 */
			accepted = (type == 0xB0 && data2 > 0);
		}

		if (accepted) {
			midi_bind(status, data1, learning);
			g_midi_learn_active = 0;
			midi_map_save();
		}
		return; /* don't dispatch while learning */
	}

	/* ── Binding-table dispatch ── */
	MidiAction act = midi_lookup(status, data1);
	uint8_t type = status & 0xF0;

	/* ── Relative CC encoder pre-processing ──────────────────────────────────
     * If a CC binding is marked 'relative', the hardware sends a center-64
     * delta (data2>64 = CW/+, data2<64 = CCW/-, data2=64 = no motion) rather
     * than an absolute 0-127 value.  We accumulate the delta into a virtual
     * absolute position [0,1] and substitute it as data2 so all downstream
     * absolute-CC handlers work without modification.
     *
     * Actions that already do their own delta math (jog_spin, lib_encoder,
     * pitch_bend, fx_knob, fx_wet) are excluded — they remain untouched. */
	if (type == 0xB0) {
		int is_self_relative =
			(act >= MACT_JOG_SPIN_A && act <= MACT_JOG_SPIN_D) ||
			(act >= MACT_JOG_PB_A && act <= MACT_JOG_PB_D) ||
			(act == MACT_LIB_ENCODER) ||
			(act >= MACT_PITCH_BEND_A &&
			 act <= MACT_PITCH_BEND_D) ||
			(act >= MACT_FX_KNOB_1_A && act <= MACT_FX_WET_B);
		if (!is_self_relative) {
			for (int bi = 0; bi < g_midi_nbindings; bi++) {
				if (g_midi_bindings[bi].status == status &&
				    g_midi_bindings[bi].data1 == data1 &&
				    g_midi_bindings[bi].relative) {
					if (data2 == 64)
						return; /* center = no motion */
					int d = (data2 > 64) ?
							(int)(data2 - 64) :
							-(int)(64 - data2);
					g_midi_bindings[bi].rel_acc +=
						d * (1.0f / 127.0f);
					if (g_midi_bindings[bi].rel_acc < 0.0f)
						g_midi_bindings[bi].rel_acc =
							0.0f;
					if (g_midi_bindings[bi].rel_acc > 1.0f)
						g_midi_bindings[bi].rel_acc =
							1.0f;
					data2 = (uint8_t)(g_midi_bindings[bi]
								  .rel_acc *
							  127.0f);
					break;
				}
			}
		}
	}

	float val = data2 / 127.0f;

	/* CC actions */
	if (type == 0xB0) {
		switch (act) {
		case MACT_DECK_VOL_A:
			g_tracks[0].volume = val;
			break;
		case MACT_DECK_VOL_B:
			g_tracks[1].volume = val;
			break;
		case MACT_DECK_VOL_C:
			g_tracks[2].volume = val;
			break;
		case MACT_DECK_VOL_D:
			g_tracks[3].volume = val;
			break;
		case MACT_DECK_PITCH_A:
		case MACT_DECK_PITCH_B:
		case MACT_DECK_PITCH_C:
		case MACT_DECK_PITCH_D: {
			/* Pitch fader mapping — 14-bit when pitch_lsb_* is bound, 7-bit otherwise.
             * 14-bit: full14 = (MSB << 7) | LSB, range 0-16383, centre ~8191.5.
             * 7-bit:  LSB stays 0, full14 = data2 << 7, centre at data2=64 → 8192.
             * Both cases use the same 14-bit normalisation formula; 7-bit loses the
             * lower 7 bits but is otherwise identical in behaviour.
             * Pitch range is per-deck (g_pitch_range[]): ±8%, ±25%, or ±50%. */
			int deck = act - MACT_DECK_PITCH_A;
			if (deck < 0 || deck >= 4)
				break;
			float range = g_pitch_range_vals[g_pitch_range[deck]];
			int full14 = ((int)data2 << 7) | (int)g_pitch_lsb[deck];
			/* Normalise to -1.0 … +1.0, centre at 8191.5 */
			float norm = ((float)full14 - 8191.5f) / 8191.5f;
			/* Apply 2% dead zone: |norm| < dead_norm → pitch = 1.0 */
			float dead_norm = 0.02f / range;
			if (dead_norm > 0.08f)
				dead_norm = 0.08f; /* cap dead band */
			if (norm > -dead_norm && norm < dead_norm) {
				g_tracks[deck].pitch = 1.0f;
			} else {
				float pitch = 1.0f + norm * range;
				if (pitch < 0.25f)
					pitch = 0.25f;
				if (pitch > 2.00f)
					pitch = 2.00f;
				g_tracks[deck].pitch = pitch;
			}
			motor_sync_pitch(
				deck); /* keep physical platter speed in sync */
			break;
		}
		case MACT_PITCH_LSB_A:
		case MACT_PITCH_LSB_B:
		case MACT_PITCH_LSB_C:
		case MACT_PITCH_LSB_D: {
			int deck = act - MACT_PITCH_LSB_A;
			if (deck < 0 || deck >= 4)
				break;
			g_pitch_lsb[deck] = data2;
			break;
		}
		case MACT_EQ_LOW_A:
			g_tracks[0].eq_low = val * 2.0f - 1.0f;
			break;
		case MACT_EQ_LOW_B:
			g_tracks[1].eq_low = val * 2.0f - 1.0f;
			break;
		case MACT_EQ_LOW_C:
			g_tracks[2].eq_low = val * 2.0f - 1.0f;
			break;
		case MACT_EQ_LOW_D:
			g_tracks[3].eq_low = val * 2.0f - 1.0f;
			break;
		case MACT_EQ_MID_A:
			g_tracks[0].eq_mid = val * 2.0f - 1.0f;
			break;
		case MACT_EQ_MID_B:
			g_tracks[1].eq_mid = val * 2.0f - 1.0f;
			break;
		case MACT_EQ_MID_C:
			g_tracks[2].eq_mid = val * 2.0f - 1.0f;
			break;
		case MACT_EQ_MID_D:
			g_tracks[3].eq_mid = val * 2.0f - 1.0f;
			break;
		case MACT_EQ_HIGH_A:
			g_tracks[0].eq_high = val * 2.0f - 1.0f;
			break;
		case MACT_EQ_HIGH_B:
			g_tracks[1].eq_high = val * 2.0f - 1.0f;
			break;
		case MACT_EQ_HIGH_C:
			g_tracks[2].eq_high = val * 2.0f - 1.0f;
			break;
		case MACT_EQ_HIGH_D:
			g_tracks[3].eq_high = val * 2.0f - 1.0f;
			break;
		case MACT_GAIN_A:
			g_tracks[0].gain = val * 2.0f;
			break;
		case MACT_GAIN_B:
			g_tracks[1].gain = val * 2.0f;
			break;
		case MACT_GAIN_C:
			g_tracks[2].gain = val * 2.0f;
			break;
		case MACT_GAIN_D:
			g_tracks[3].gain = val * 2.0f;
			break;
		/* Filter knob — variable LP/HP filter with ±3% center dead zone.
         * CC range 0-127, center = 63.5.  Dead zone: CC 59-67 (±3%) = flat.
         * Below dead zone: low-pass, cutoff sweeps 80Hz → 2kHz.
         * Above dead zone: high-pass, cutoff sweeps 2kHz → 18kHz.
         * filter field stores normalised position: 0.0=LP full, 0.5=flat, 1.0=HP full */
		case MACT_FILTER_A:
		case MACT_FILTER_B:
		case MACT_FILTER_C:
		case MACT_FILTER_D: {
			int deck = act - MACT_FILTER_A;
			if (deck >= 4)
				break;
			float cc = (float)data2; /* 0–127 */
			float norm = (cc - 63.5f) / 63.5f; /* -1.0 to +1.0 */
			/* ±3% dead zone → flat passthrough */
			if (norm > -0.03f && norm < 0.03f)
				norm = 0.0f;
			else if (norm > 0.0f)
				norm = (norm - 0.03f) /
				       0.97f; /* rescale above dead zone to 0–1 */
			else
				norm = (norm + 0.03f) /
				       0.97f; /* rescale below dead zone to -1–0 */
			/* Store as 0.0–1.0 (0=LP full, 0.5=flat, 1.0=HP full) */
			g_tracks[deck].filter = norm * 0.5f + 0.5f;
			/* Reset EQ filter state when crossing through flat to avoid clicks */
			if (fabsf(norm) < 0.01f) {
				g_eq[deck].fi_x1l = g_eq[deck].fi_x2l = 0.0f;
				g_eq[deck].fi_y1l = g_eq[deck].fi_y2l = 0.0f;
				g_eq[deck].fi_x1r = g_eq[deck].fi_x2r = 0.0f;
				g_eq[deck].fi_y1r = g_eq[deck].fi_y2r = 0.0f;
			}
			break;
		}
		case MACT_CROSSFADER:
			g_crossfader = val;
			break;
		case MACT_CF_CURVE:
			g_cf_curve = val;
			snprintf(g_fb_status, sizeof(g_fb_status), "X-Fader Curve: %.0f%%", (double)(val * 100.0f));
			break;
		case MACT_TAP_BPM_A:
			tap_bpm(0);
			break;
		case MACT_TAP_BPM_B:
			tap_bpm(1);
			break;
		case MACT_GRID_SNAP_A:
			snap_grid(0);
			break;
		case MACT_GRID_SNAP_B:
			snap_grid(1);
			break;
		case MACT_MASTER_VOL:
			g_master_vol = (int)(val * 150.0f);
			g_opts.default_master_vol = g_master_vol;
			break;
		case MACT_BOOTH_VOL: /* booth/monitor vol — stored but not routed to a separate output */
			/* For now store as a display-only value; a proper booth output
             * would need a second ALSA device or a separate mix bus. */
			break;
		/* Strip search: absolute CC 0–127 → seek to proportional track position */
		case MACT_STRIP_A:
		case MACT_STRIP_B:
		case MACT_STRIP_C:
		case MACT_STRIP_D: {
			int deck = act - MACT_STRIP_A;
			if (deck >= 4)
				break;
			Track *st = &g_tracks[deck];
			if (!st->loaded || st->num_frames == 0)
				break;
			pthread_mutex_lock(&st->lock);
			st->pos = (uint32_t)((float)data2 / 127.0f *
					     (float)(st->num_frames - 1));
			pthread_mutex_unlock(&st->lock);
			break;
		}
		case MACT_JOG_SPIN_A:
		case MACT_JOG_SPIN_B:
		case MACT_JOG_SPIN_C:
		case MACT_JOG_SPIN_D: {
			int deck;
			if (act <= MACT_JOG_SPIN_B)
				deck = act - MACT_JOG_SPIN_A;
			else
				deck = (act - MACT_JOG_SPIN_C) + 2;
			if (deck < 0 || deck >= MAX_TRACKS) /* allow C/D via layer switch */
				break;

			if (g_jog_type == JOG_NS7III) {
				/* ── NS7III absolute coarse angle ─────────────────────────
                 * data2 = 7-bit absolute platter angle, 0-127, wrapping.
                 * Store and reconstruct position using latest fine (PB) value.
                 * Also record the coarse delta + timestamp for PB slip detection. */
				int prev_c = g_jog_coarse[deck];
				g_jog_coarse[deck] = (int)data2;
				if (prev_c >= 0) {
					int cd = g_jog_coarse[deck] - prev_c;
					if (cd > 64)
						cd -= 128;
					if (cd < -64)
						cd += 128;
					g_jog_last_coarse_d[deck] = cd;
					struct timespec _cts;
					clock_gettime(CLOCK_MONOTONIC, &_cts);
					g_jog_last_spin_ms[deck] =
						(int64_t)_cts.tv_sec * 1000 +
						_cts.tv_nsec / 1000000;
				}
				ns7iii_update_jog(deck, g_jog_coarse[deck],
						  g_jog_fine[deck]);
				break;
			}

			/* ── Standard relative encoder (center-64 delta) ─────────── */
			int delta;
			if (data2 == 0)
				delta = -64;
			else if (data2 == 127)
				delta = 63;
			else
				delta = (int)data2 - 64;
			if (delta == 0)
				break;

			Track *jt = &g_tracks[deck];
			if (g_motor_running[deck]) {
				g_motor_vel[deck] += (delta * 0.0005f);
				if (g_motor_vel[deck] > 0.15f)
					g_motor_vel[deck] = 0.15f;
				if (g_motor_vel[deck] < -0.15f)
					g_motor_vel[deck] = -0.15f;
			} else {
				MidiAction touch_act =
					(MidiAction)(MACT_JOG_TOUCH_A + deck);
				int touch_bound = 0;
				for (int bi = 0; bi < g_midi_nbindings; bi++) {
					if (g_midi_bindings[bi].action ==
					    touch_act) {
						touch_bound = 1;
						break;
					}
				}
				int in_scratch =
					(!touch_bound) || g_jog_touched[deck];
				if (in_scratch) {
					pthread_mutex_lock(&jt->lock);
					int64_t new_pos =
						(int64_t)jt->pos +
						(int64_t)(delta *
							  SCRATCH_FRAMES_PER_TICK);
					if (new_pos < 0)
						new_pos = 0;
					if (new_pos >= (int64_t)jt->num_frames)
						new_pos =
							(int64_t)
								jt->num_frames -
							1;
					jt->pos = (uint32_t)new_pos;
					pthread_mutex_unlock(&jt->lock);
				} else {
					int nudge_dir = (delta > 0) ? 1 : -1;
					g_jog_nudge[deck] +=
						nudge_dir *
						NUDGE_PITCH_PER_TICK;
					if (g_jog_nudge[deck] > 0.15f)
						g_jog_nudge[deck] = 0.15f;
					if (g_jog_nudge[deck] < -0.15f)
						g_jog_nudge[deck] = -0.15f;
				}
			}
			break;
		}
		/* Library/browser navigation encoder — relative CC, same center-64 encoding */
		case MACT_LIB_ENCODER: {
			/* Each MIDI message = one physical encoder detent = one list entry.
             * data2 encodes direction only: >64 = down/forward, <64 = up/back.
             * The magnitude of (data2-64) indicates speed but we intentionally
             * ignore it — speed is expressed by how fast messages arrive, not
             * by jumping multiple entries per message. One tick, one step.
             *
             * 4-deck auto-switch: in 4-deck view (g_view==0) the library panel
             * is hidden.  Any scroll on the encoder temporarily switches to
             * split view so the selection is visible; g_lib_enc_last_ms is
             * updated and redraw() will restore g_view=0 after 1 s of silence. */
			if (data2 == 64)
				break; /* no motion */
			{
				struct timespec _lts;
				clock_gettime(CLOCK_MONOTONIC, &_lts);
				g_lib_enc_last_ms =
					(int64_t)_lts.tv_sec * 1000 +
					_lts.tv_nsec / 1000000;
			}
			if (g_num_tracks == 4 && g_view == 0) {
				g_view = 1;
				g_lib_auto_switched = 1;
			}
			int step = (data2 > 64) ? 1 : -1;
			if (g_panel == 0 && g_fb_count > 0) {
				g_fb_sel += step;
				if (g_fb_sel < 0)
					g_fb_sel = 0;
				if (g_fb_sel >= g_fb_count)
					g_fb_sel = g_fb_count - 1;
			} else if (g_panel == 1 && g_pl_count > 0) {
				g_pl_sel += step;
				if (g_pl_sel < 0)
					g_pl_sel = 0;
				if (g_pl_sel >= g_pl_count)
					g_pl_sel = g_pl_count - 1;
			} else if (g_panel == 2 && g_lib_count > 0) {
				g_lib_sel += step;
				if (g_lib_sel < 0)
					g_lib_sel = 0;
				if (g_lib_sel >= g_lib_count)
					g_lib_sel = g_lib_count - 1;
			}
			break;
		}
		/* Per-deck MIDI pitch bend — relative encoder, decaying nudge */
		case MACT_PITCH_BEND_A:
		case MACT_PITCH_BEND_B:
		case MACT_PITCH_BEND_C:
		case MACT_PITCH_BEND_D: {
			int deck = act - MACT_PITCH_BEND_A;
			if (deck < 0 || deck >= 4)
				break;
			if (data2 == 64)
				break; /* no motion */
			/* One tick = one nudge step; speed is expressed by message rate */
			int nudge_dir = (data2 > 64) ? 1 : -1;
			g_tracks[deck].nudge += nudge_dir * CFG_NUDGE_AMOUNT;
			if (g_tracks[deck].nudge >
			    CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP)
				g_tracks[deck].nudge =
					CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP;
			if (g_tracks[deck].nudge <
			    -CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP)
				g_tracks[deck].nudge =
					-CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP;
			break;
		}
		/* ── FX knobs — center-64 relative CC encoders ───────────────────────
         * data2 > 64 = CW (increase), data2 < 64 = CCW (decrease), 64 = no motion.
         * Each tick nudges the accumulated parameter value by FX_KNOB_STEP.
         * Accumulator lives in g_fx_param_acc so the value persists across messages. */
		case MACT_FX_KNOB_1_A:
		case MACT_FX_KNOB_2_A:
		case MACT_FX_KNOB_3_A:
		case MACT_FX_KNOB_1_B:
		case MACT_FX_KNOB_2_B:
		case MACT_FX_KNOB_3_B: {
			int is_b = (act >= MACT_FX_KNOB_1_B);
			int dk = is_b ? g_side_deck[1] : g_side_deck[0];
			int param = is_b ? (int)(act - MACT_FX_KNOB_1_B) :
					   (int)(act - MACT_FX_KNOB_1_A);
			if (dk >= MAX_TRACKS) /* allow deck C/D via layer switch */
				break;
			if (data2 == 64)
				break; /* center = no motion */
			/* Center-64 relative encoding: data2 > 64 = CW (+), data2 < 64 = CCW (-) */
			float delta =
				(data2 > 64) ? FX_KNOB_STEP : -FX_KNOB_STEP;
			g_fx_param_acc[dk][param] += delta;
			if (g_fx_param_acc[dk][param] < 0.0f)
				g_fx_param_acc[dk][param] = 0.0f;
			if (g_fx_param_acc[dk][param] > 1.0f)
				g_fx_param_acc[dk][param] = 1.0f;
			fx_set_param(dk, g_fx_ui_slot[dk], param,
				     g_fx_param_acc[dk][param]);
			{
				FXSlot *_s = fx_slot(dk, g_fx_ui_slot[dk]);
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Deck %c FX%d %s p%d: %.0f%%",
					 DECK_NUM(dk), g_fx_ui_slot[dk] + 1,
					 fx_names[_s->type], param + 1,
					 g_fx_param_acc[dk][param] * 100.0f);
			}
			break;
		}
		case MACT_FX_WET_A:
		case MACT_FX_WET_B: {
			int dk = (act == MACT_FX_WET_B) ? g_side_deck[1] :
							  g_side_deck[0];
			if (dk >= MAX_TRACKS) /* allow deck C/D via layer switch */
				break;
			if (data2 == 64)
				break; /* center = no motion */
			float delta =
				(data2 > 64) ? FX_KNOB_STEP : -FX_KNOB_STEP;
			g_fx_param_acc[dk][3] += delta;
			if (g_fx_param_acc[dk][3] < 0.0f)
				g_fx_param_acc[dk][3] = 0.0f;
			if (g_fx_param_acc[dk][3] > 1.0f)
				g_fx_param_acc[dk][3] = 1.0f;
			fx_set_wet(dk, g_fx_ui_slot[dk], g_fx_param_acc[dk][3]);
			{
				FXSlot *_s = fx_slot(dk, g_fx_ui_slot[dk]);
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Deck %c FX%d %s wet: %.0f%%",
					 DECK_NUM(dk), g_fx_ui_slot[dk] + 1,
					 fx_names[_s->type],
					 g_fx_param_acc[dk][3] * 100.0f);
			}
			break;
		}
		default:
			break;
		}
	}

	/* ── Pitch Bend (0xE0) — jog wheel with motor running ───────────────────
     * NS7III sends 14-bit pitch bend (0xE2 deck A ch3, 0xE3 deck B ch4).
     * With motors running at reference speed:
     *   raw14 = 8192 (center)  → platter at reference speed → normal playback
     *   raw14 > 8192           → platter faster than ref    → speed up
     *   raw14 < 8192           → platter slower / reverse   → slow down / reverse
     *
     * Error handling — two layers:
     *
     * 1. Spike rejection: the motor stop/brake sends full-throw values (±8192)
     *    for 1–3 messages.  Any message with |norm| > JOG_SPIKE_THRESH that
     *    differs in sign from the previous value is treated as a transient spike
     *    and discarded.  Genuine direction reversals build up over multiple messages.
     *
     * 2. Slew limiter: the nudge value cannot change by more than JOG_SLEW_RATE
     *    per MIDI message (~2ms).  This prevents any residual spike from causing
     *    a step change in playback speed, giving the audio thread time to react
     *    smoothly.  At normal platter speed the slew rate is never reached. */
	if (type == 0xE0) {
		MidiAction pb_act = midi_lookup(status, 0);
		int deck =
			-1; /* hoisted — must be in scope at pb_settled label */
		if (pb_act >= MACT_JOG_PB_A && pb_act <= MACT_JOG_PB_D) {
			deck = pb_act - MACT_JOG_PB_A;
			if (deck >= 0 && deck < MAX_TRACKS) { /* allow C/D via layer switch */
				int raw14 = ((int)data2 << 7) |
					    (int)data1; /* 0-16383 */

				if (g_jog_type == JOG_NS7III) {
					/* ── NS7III absolute fine position ───────────────────────
                     * raw14 is the fine angular sub-step within the current
                     * coarse CC step.  Store it and recompute full position.
                     * The settle timer still applies when motor is running —
                     * during spin-up the fine values are garbage. */
					if (g_motor_running[deck] &&
					    g_motor_settle_until[deck] > 0) {
						struct timespec ts;
						clock_gettime(CLOCK_MONOTONIC,
							      &ts);
						int64_t now_ms =
							(int64_t)ts.tv_sec *
								1000 +
							ts.tv_nsec / 1000000;
						if (now_ms <
						    g_motor_settle_until
							    [deck]) {
							/* Still settling — reset absolute encoder state */
							g_jog_abs_init[deck] =
								0;
							g_jog_abs_vel[deck] =
								0.0f;
							g_jog_nudge[deck] =
								0.0f;
							g_motor_vel[deck] =
								0.0f;
							g_jog_last_pb[deck] =
								raw14; /* reset PB base; avoid false slip */
							goto pb_settled;
						}
						g_motor_settle_until[deck] = 0;
						g_jog_abs_init[deck] =
							0; /* discard pre-settle position */
						g_jog_last_pb[deck] =
							raw14; /* fresh PB baseline after settle */
					}
					g_jog_fine[deck] = raw14;

					/* ── PB slip detection: infer hand touch (no hardware touch note) ──
                     * When the motor drives the platter at reference speed the PB stream
                     * tracks the coarse steps: deltaPB ≈ lastCoarseDelta × PB_RATIO.
                     * A human hand disturbing the platter breaks this relationship.
                     * This mirrors Mixxx's jogPB slip-detection algorithm exactly.
                     * Only active while the motor is running. */
					if (g_motor_running[deck]) {
						int deltaPB =
							raw14 -
							g_jog_last_pb[deck];
						if (deltaPB > 8192)
							deltaPB -= 16384;
						if (deltaPB < -8192)
							deltaPB += 16384;

						struct timespec _pts;
						clock_gettime(CLOCK_MONOTONIC,
							      &_pts);
						int64_t _now =
							(int64_t)_pts.tv_sec *
								1000 +
							_pts.tv_nsec / 1000000;
						int effDelta =
							(_now - g_jog_last_spin_ms
									 [deck] <
							 30) ?
								g_jog_last_coarse_d
									[deck] :
								0;
						int slipErr = abs(
							deltaPB -
							effDelta *
								NS7III_PB_RATIO);

						if (slipErr >
							    NS7III_SLIP_THRESH ||
						    (effDelta == 0 &&
						     abs(deltaPB) > 100)) {
							/* Slip detected — hand on platter */
							g_jog_release_conf
								[deck] = 0;
							g_jog_touched[deck] = 1;
						} else if (g_jog_touched
								   [deck]) {
							/* In sync — count toward release */
							g_jog_release_conf
								[deck]++;
							if (g_jog_release_conf
								    [deck] >=
							    NS7III_RELEASE_CONF) {
								g_jog_touched
									[deck] =
										0;
								g_jog_release_conf
									[deck] =
										0;
							}
						}
						g_jog_last_pb[deck] = raw14;
					}

					/* Do NOT call ns7iii_update_jog here.
                     *
                     * PB fires much more frequently than coarse CC. Between two
                     * coarse steps the PB delta is tiny (sub-step fine position),
                     * so (delta / ref_delta) - 1 ≈ -1 → motor_vel ≈ -1 → track
                     * locked.  Velocity is only meaningful when computed from the
                     * coarse CC delta (one full encoder step ≈ ref_delta).
                     *
                     * The coarse CC handler calls ns7iii_update_jog with the
                     * latest g_jog_fine[deck] already set above, so fine
                     * position resolution is preserved without the PB call. */

				} else {
					/* ── Relative mode: legacy sustained-direction streak model ── */
					if (!g_motor_running[deck])
						goto pb_settled;

					/* Check motor settle timer */
					if (g_motor_settle_until[deck] > 0) {
						struct timespec ts;
						clock_gettime(CLOCK_MONOTONIC,
							      &ts);
						int64_t now_ms =
							(int64_t)ts.tv_sec *
								1000 +
							ts.tv_nsec / 1000000;
						if (now_ms <
						    g_motor_settle_until
							    [deck]) {
							g_jog_nudge[deck] =
								0.0f;
							g_pb_streak[deck] = 0;
							g_pb_mag_acc[deck] =
								0.0f;
							goto pb_settled;
						}
						g_motor_settle_until[deck] = 0;
						g_pb_streak[deck] = 0;
						g_pb_mag_acc[deck] = 0.0f;
					}

					float norm = ((float)raw14 - 8192.0f) /
						     8192.0f;
					if (norm > 1.0f)
						norm = 1.0f;
					if (norm < -1.0f)
						norm = -1.0f;

#define JOG_SUSTAIN 12
#define JOG_MAGNITUDE g_jog_dead_band
#define JOG_SCALE 0.12f
					int *pb_streak = g_pb_streak;
					float *pb_mag_acc = g_pb_mag_acc;
					Track *jt = &g_tracks[deck];

					if (fabsf(norm) < JOG_MAGNITUDE) {
						if (pb_streak[deck] > 0)
							pb_streak[deck]--;
						else if (pb_streak[deck] < 0)
							pb_streak[deck]++;
					} else {
						int cur_sign =
							(norm > 0.0f) ? 1 : -1;
						int str_sign =
							(pb_streak[deck] >= 0) ?
								1 :
								-1;
						if (cur_sign == str_sign ||
						    pb_streak[deck] == 0) {
							pb_streak[deck] +=
								cur_sign;
							if (pb_streak[deck] >
							    127)
								pb_streak[deck] =
									127;
							if (pb_streak[deck] <
							    -127)
								pb_streak[deck] =
									-127;
							pb_mag_acc[deck] =
								pb_mag_acc[deck] *
									0.95f +
								fabsf(norm) *
									0.05f;
						} else {
							pb_streak[deck] =
								cur_sign;
							pb_mag_acc[deck] =
								fabsf(norm);
						}
						if (abs(pb_streak[deck]) >=
						    JOG_SUSTAIN) {
							float raw_vel =
								(float)cur_sign *
								pb_mag_acc
									[deck] *
								JOG_SCALE;
							if (jt->playing)
								g_motor_vel[deck] =
									(g_motor_vel
										 [deck] *
									 0.98f) +
									(raw_vel *
									 0.02f);
							else {
								g_motor_vel[deck] =
									0.0f;
								g_jog_nudge[deck] =
									0.0f;
							}
						}
					}
#undef JOG_SUSTAIN
#undef JOG_MAGNITUDE
#undef JOG_SCALE
				} /* end relative mode */
			} /* end deck valid */
		} /* end pb_act matched */

	pb_settled:; /* settle timer goto target */

		if (deck >= 0 && deck < g_num_tracks && g_motor_running[deck]) {
			/* Settle path: motor running but still spinning up — state already
             * zeroed in the branch above; nothing more to do here. */
		}
	} /* end if (type == 0xE0) */

	/* Note On actions (velocity > 0 = press, 0 = release — only act on press) */
	if (type == 0x90 && data2 > 0) {
		switch (act) {
		case MACT_PLAY_A:
			g_tracks[0].playing = !g_tracks[0].playing;
			if (g_tracks[0].playing && !g_slip_motor_off[0]) {
				motor_set(0, 1);
			} else if (!g_tracks[0].playing) {
				motor_set(0, 0);
				g_jog_nudge[0] = 0;
				g_last_applied_nudge[0] = 0;
			}
			deck_leds_refresh();
			break;
		case MACT_PLAY_B:
			g_tracks[1].playing = !g_tracks[1].playing;
			if (g_tracks[1].playing && !g_slip_motor_off[1])
				motor_set(1, 1);
			else if (!g_tracks[1].playing)
				motor_set(1, 0);
			deck_leds_refresh();
			break;
		case MACT_PLAY_C:
			if (g_tracks[2].loaded) {
				g_tracks[2].playing = !g_tracks[2].playing;
				if (g_tracks[2].playing && !g_slip_motor_off[2])
					motor_set(2, 1);
				else if (!g_tracks[2].playing) {
					motor_set(2, 0);
					g_jog_nudge[2] = 0;
					g_last_applied_nudge[2] = 0;
				}
				deck_leds_refresh();
			}
			break;
		case MACT_PLAY_D:
			if (g_tracks[3].loaded) {
				g_tracks[3].playing = !g_tracks[3].playing;
				if (g_tracks[3].playing && !g_slip_motor_off[3])
					motor_set(3, 1);
				else if (!g_tracks[3].playing) {
					motor_set(3, 0);
					g_jog_nudge[3] = 0;
					g_last_applied_nudge[3] = 0;
				}
				deck_leds_refresh();
			}
			break;
		/* Jog touch: Note On = top touched (scratch), Note Off = released */
		case MACT_JOG_TOUCH_D:
			g_jog_touched[3] = (data2 > 0);
			break;
		case MACT_CUE_ACTIVE_A:
			g_tracks[0].cue_active ^= 1;
			break;
		case MACT_CUE_ACTIVE_B:
			g_tracks[1].cue_active ^= 1;
			break;
		case MACT_CUE_ACTIVE_C:
			if (g_num_tracks >= 3)
				g_tracks[2].cue_active ^= 1;
			break;
		case MACT_CUE_ACTIVE_D:
			if (g_num_tracks >= 4)
				g_tracks[3].cue_active ^= 1;
			break;
		case MACT_CUE_JUMP_1:
		case MACT_CUE_JUMP_2:
		case MACT_CUE_JUMP_3:
		case MACT_CUE_JUMP_4: {
			int ci = act - MACT_CUE_JUMP_1;
			Track *t = &g_tracks[g_active_track];
			if (t->cue_set[ci]) {
				pthread_mutex_lock(&t->lock);
				t->pos = t->cue[ci];
				pthread_mutex_unlock(&t->lock);
			}
			break;
		}
		case MACT_SYNC_SLAVE_A:
			g_tracks[0].sync_locked ^= 1;
			if (g_tracks[0].sync_locked)
				led_on("led_sync_a");
			else
				led_off("led_sync_a");
			break;
		case MACT_SYNC_SLAVE_B:
			g_tracks[1].sync_locked ^= 1;
			if (g_tracks[1].sync_locked)
				led_on("led_sync_b");
			else
				led_off("led_sync_b");
			break;
		case MACT_SYNC_SLAVE_C:
			g_tracks[2].sync_locked ^= 1;
			if (g_tracks[2].sync_locked)
				led_on("led_sync_c");
			else
				led_off("led_sync_c");
			break;
		case MACT_SYNC_SLAVE_D:
			g_tracks[3].sync_locked ^= 1;
			if (g_tracks[3].sync_locked)
				led_on("led_sync_d");
			else
				led_off("led_sync_d");
			break;
		case MACT_NUDGE_FWD:
			g_tracks[g_active_track].nudge += CFG_NUDGE_AMOUNT;
			if (g_tracks[g_active_track].nudge >
			    CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP)
				g_tracks[g_active_track].nudge =
					CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP;
			break;
		case MACT_NUDGE_BACK:
			g_tracks[g_active_track].nudge -= CFG_NUDGE_AMOUNT;
			if (g_tracks[g_active_track].nudge <
			    -CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP)
				g_tracks[g_active_track].nudge =
					-CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP;
			break;
		/* Deck B nudge buttons — route to g_side_deck[1] independent of active track */
		case MACT_NUDGE_FWD_B: {
			int dk = g_side_deck[1];
			if (dk < g_num_tracks) {
				g_tracks[dk].nudge += CFG_NUDGE_AMOUNT;
				if (g_tracks[dk].nudge >
				    CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP)
					g_tracks[dk].nudge = CFG_NUDGE_AMOUNT *
							     CFG_NUDGE_CAP;
			}
			break;
		}
		case MACT_NUDGE_BACK_B: {
			int dk = g_side_deck[1];
			if (dk < g_num_tracks) {
				g_tracks[dk].nudge -= CFG_NUDGE_AMOUNT;
				if (g_tracks[dk].nudge <
				    -CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP)
					g_tracks[dk].nudge = -CFG_NUDGE_AMOUNT *
							     CFG_NUDGE_CAP;
			}
			break;
		}
		case MACT_LOOP_TOGGLE: {
			Track *t = &g_tracks[g_active_track];
			t->looping ^= 1;
			deck_leds_refresh();
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Deck %c loop %s", DECK_NUM(g_active_track),
				 t->looping ? "ON" : "OFF");
			break;
		}
		/* Library encoder touch — Note On: switch to split view in 4-deck mode
		 * and record timestamp so the 1 s auto-restore timer starts on release. */
		case MACT_LIB_ENCODER_TOUCH:
			if (g_num_tracks == 4 && g_view == 0) {
				g_view = 1;
				g_lib_auto_switched = 1;
			}
			{
				struct timespec _lts2;
				clock_gettime(CLOCK_MONOTONIC, &_lts2);
				g_lib_enc_last_ms =
					(int64_t)_lts2.tv_sec * 1000 +
					_lts2.tv_nsec / 1000000;
			}
			break;
		/* Library select: enter directory if a dir is highlighted, do nothing on files.
         * Files are loaded only via lib_load_a/b/c/d. */
		case MACT_LIB_SELECT: {
			if (g_panel == 0 && g_fb_count > 0) {
				if (g_fb_entries[g_fb_sel].is_dir)
					fb_enter_dir(
						g_fb_entries[g_fb_sel].name);
				/* files: do nothing — use lib_load_a/b/c/d to load */
			}
			/* playlist and library panels have no directory structure — no-op */
			break;
		}
		/* Library back: go up one directory in the file browser */
		case MACT_LIB_BACK: {
			if (g_panel == 0)
				fb_enter_dir("..");
			break;
		}
		case MACT_LIB_LOAD_A:
		case MACT_LIB_LOAD_B:
		case MACT_LIB_LOAD_C:
		case MACT_LIB_LOAD_D: {
			int deck;
			if (act == MACT_LIB_LOAD_A)
				deck = g_side_deck
					[0]; /* left side — follows layer selection */
			else if (act == MACT_LIB_LOAD_B)
				deck = g_side_deck
					[1]; /* right side — follows layer selection */
			else
				deck = act -
				       MACT_LIB_LOAD_A; /* C/D: explicit deck */
			if (deck < 0 || deck >= 4)
				break;
			int _loaded = 0;
			if (g_panel == 0 && g_fb_count > 0 &&
			    !g_fb_entries[g_fb_sel].is_dir) {
				char full[FB_PATH_MAX + 256];
				fb_selected_path(full, sizeof(full));
				enqueue_load(deck, full);
				_loaded = 1;
			} else if (g_panel == 1 && g_pl_count > 0) {
				enqueue_load(deck, g_pl[g_pl_sel].path);
				_loaded = 1;
			} else if (g_panel == 2 && g_lib && g_lib_count > 0) {
				enqueue_load(deck, g_lib[g_lib_sel].path);
				_loaded = 1;
			}
			if (_loaded)
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Loading \u2192 Deck %c",
					 DECK_NUM(deck));
			else
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "No track selected");
			/* Refresh pad LEDs so cue colours reflect newly loaded track */
			pad_leds_refresh(deck);
			deck_leds_refresh();
			break;
		}
		/* Panel switchers: switch the library pane and enter split view */
		case MACT_PANEL_FILES:
			g_panel = 0;
			if (g_view == 0)
				g_view = 1;
			break;
		case MACT_PANEL_LIBRARY:
			g_panel = 2;
			if (g_view == 0)
				g_view = 1;
			break;
		/* Pitch range cycle: ±8% → ±25% → ±50% → ±8% */
		case MACT_PITCH_RANGE_A:
		case MACT_PITCH_RANGE_B:
		case MACT_PITCH_RANGE_C:
		case MACT_PITCH_RANGE_D: {
			int deck = act - MACT_PITCH_RANGE_A;
			if (deck >= 4)
				break;
			g_pitch_range[deck] = (g_pitch_range[deck] + 1) % 3;
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Deck %c pitch range: %s", DECK_NUM(deck),
				 g_pitch_range_names[g_pitch_range[deck]]);
			/* Dim pitch-center LED when range changes; it re-lights when pitch == 0 */
			{
				char ln[32];
				snprintf(ln, sizeof(ln), "led_pitch_center_%c",
					 'a' + deck);
				led_off(ln);
			}
			break;
		}
		/* Hot cue SET — records current playback position */
		case MACT_CUE_SET_1:
		case MACT_CUE_SET_2:
		case MACT_CUE_SET_3:
		case MACT_CUE_SET_4: {
			int ci = act - MACT_CUE_SET_1;
			Track *t = &g_tracks[g_active_track];
			if (t->loaded) {
				t->cue[ci] = t->pos;
				t->cue_set[ci] = 1;
				cache_save(t);
				char side = (g_active_track == g_side_deck[0]) ?
						    'a' :
						    'b';
				char ln[32];
				snprintf(ln, sizeof(ln), "led_cue_%d_%c",
					 ci + 1, side);
				led_on(ln);
				pad_leds_refresh(g_active_track);
			}
			break;
		}
		/* Hot cue DELETE */
		case MACT_CUE_DELETE_1:
		case MACT_CUE_DELETE_2:
		case MACT_CUE_DELETE_3:
		case MACT_CUE_DELETE_4: {
			int ci = act - MACT_CUE_DELETE_1;
			Track *t = &g_tracks[g_active_track];
			t->cue[ci] = 0;
			t->cue_set[ci] = 0;
			cache_save(&g_tracks[g_active_track]);
			{
				char side = (g_active_track == g_side_deck[0]) ?
						    'a' :
						    'b';
				char ln[32];
				snprintf(ln, sizeof(ln), "led_cue_%d_%c",
					 ci + 1, side);
				led_off(ln);
			}
			pad_leds_refresh(g_active_track);
			break;
		}
		/* Loop in/out — set loop boundaries at current position */
		case MACT_LOOP_IN_A:
		case MACT_LOOP_IN_B:
		case MACT_LOOP_IN_C:
		case MACT_LOOP_IN_D: {
			int deck = act - MACT_LOOP_IN_A;
			if (deck >= 4)
				break;
			pthread_mutex_lock(&g_tracks[deck].lock);
			g_tracks[deck].loop_start = g_tracks[deck].pos;
			pthread_mutex_unlock(&g_tracks[deck].lock);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Deck %c loop IN set at %.2fs", DECK_NUM(deck),
				 (float)g_tracks[deck].pos /
					 (float)g_actual_sample_rate);
			break;
		}
		case MACT_LOOP_OUT_A:
		case MACT_LOOP_OUT_B:
		case MACT_LOOP_OUT_C:
		case MACT_LOOP_OUT_D: {
			int deck = act - MACT_LOOP_OUT_A;
			if (deck >= 4)
				break;
			Track *tlo = &g_tracks[deck];
			pthread_mutex_lock(&tlo->lock);
			if (tlo->looping) {
				/* Second press: exit loop */
				tlo->looping = 0;
				pthread_mutex_unlock(&tlo->lock);
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Deck %c loop OFF", DECK_NUM(deck));
			} else if (tlo->pos > tlo->loop_start) {
				/* First press after LOOP_IN: set loop out and engage */
				tlo->loop_end = tlo->pos;
				tlo->looping = 1;
				pthread_mutex_unlock(&tlo->lock);
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Deck %c loop ON (%.2fs)",
					 DECK_NUM(deck),
					 (float)(tlo->loop_end -
						 tlo->loop_start) /
						 (float)g_actual_sample_rate);
			} else {
				pthread_mutex_unlock(&tlo->lock);
			}
			deck_leds_refresh();
			break;
		}
		/* Loop double / half — multiply loop length by 2× or 0.5× */
		case MACT_LOOP_DOUBLE_A:
		case MACT_LOOP_DOUBLE_B:
		case MACT_LOOP_DOUBLE_C:
		case MACT_LOOP_DOUBLE_D: {
			int deck = act - MACT_LOOP_DOUBLE_A;
			if (deck >= 4)
				break;
			Track *ld = &g_tracks[deck];
			uint32_t len = ld->loop_end > ld->loop_start ?
					       ld->loop_end - ld->loop_start :
					       0;
			if (len > 0) {
				ld->loop_end = ld->loop_start + len * 2;
				if (ld->loop_end > ld->num_frames)
					ld->loop_end = ld->num_frames;
			}
			break;
		}
		case MACT_LOOP_HALF_A:
		case MACT_LOOP_HALF_B:
		case MACT_LOOP_HALF_C:
		case MACT_LOOP_HALF_D: {
			int deck = act - MACT_LOOP_HALF_A;
			if (deck >= 4)
				break;
			Track *lh = &g_tracks[deck];
			uint32_t len = lh->loop_end > lh->loop_start ?
					       lh->loop_end - lh->loop_start :
					       0;
			if (len > 2) {
				lh->loop_end = lh->loop_start + len / 2;
			}
			break;
		}
		/* Key lock toggle per deck */
		case MACT_KEY_LOCK_A:
		case MACT_KEY_LOCK_B:
		case MACT_KEY_LOCK_C:
		case MACT_KEY_LOCK_D: {
			int deck = act - MACT_KEY_LOCK_A;
			if (deck >= 4)
				break;
			g_tracks[deck].key_lock = !g_tracks[deck].key_lock;
			if (g_tracks[deck].key_lock)
				wsola_reset(&g_wsola[deck], g_tracks[deck].pos);
			break;
		}
		/* Slip mode — when on, the real playback position advances normally
         * under scratch/loop, and playback resumes from there on release.
         * Stored per-deck as a flag; audio thread uses it in read_pitched(). */
		/* Slip mode — toggles motor hold-off.
         * ON:  stops motor, holds it off until toggled back.
         * OFF: restarts motor if deck is still playing. */
		case MACT_SLIP_MODE_A:
		case MACT_SLIP_MODE_B:
		case MACT_SLIP_MODE_C:
		case MACT_SLIP_MODE_D: {
			int deck = act - MACT_SLIP_MODE_A;
			if (deck >= MAX_TRACKS) /* allow C/D via layer switch */
				break;
			g_slip_motor_off[deck] ^= 1;
			if (g_slip_motor_off[deck]) {
				motor_set(deck, 0);
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Deck %c: motor OFF (slip)",
					 DECK_NUM(deck));
			} else {
				if (g_tracks[deck].playing)
					motor_set(deck, 1);
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Deck %c: motor %s", DECK_NUM(deck),
					 g_tracks[deck].playing ? "restarted" :
								  "ready");
			}
			/* led_slip_a/b: on when slip/motor-hold is active */
			{
				char ln[32];
				snprintf(ln, sizeof(ln), "led_slip_%c",
					 'a' + deck);
				if (g_slip_motor_off[deck])
					led_on(ln);
				else
					led_off(ln);
			}
			break;
		}
		/* Reverse playback toggle */
		case MACT_REVERSE_A:
		case MACT_REVERSE_B:
		case MACT_REVERSE_C:
		case MACT_REVERSE_D: {
			int deck = act - MACT_REVERSE_A;
			if (deck >= MAX_TRACKS) /* allow C/D via layer switch */
				break;
			g_tracks[deck].reverse ^= 1;
			/* Flip hardware platter direction immediately via CC#70 */
			if (g_motor_running[deck])
				motor_set_direction(deck,
						    g_tracks[deck].reverse);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Deck %c: %s", DECK_NUM(deck),
				 g_tracks[deck].reverse ? "REVERSE" :
							  "forward");
			break;
		}
		/* Bleep — momentary slip+reverse.  Note On = engage, Note Off = release.
         * Saves current position; while held, audio plays in reverse.
         * On release, position snaps back to the saved point. */
		case MACT_BLEEP_A:
		case MACT_BLEEP_B:
		case MACT_BLEEP_C:
		case MACT_BLEEP_D: {
			int deck = act - MACT_BLEEP_A;
			if (deck >= 4)
				break;
			g_bleep_held[deck] = 1;
			g_bleep_save_pos[deck] = g_tracks[deck].pos;
			g_tracks[deck].reverse = 1;
			if (g_motor_running[deck])
				motor_set_direction(deck, 1);
			break;
		}
		/* Filter toggle — engage or bypass the filter sweep knob per deck.
         * When bypassed the knob position is remembered but audio passes flat. */
		case MACT_FILTER_TOGGLE_A:
		case MACT_FILTER_TOGGLE_B:
		case MACT_FILTER_TOGGLE_C:
		case MACT_FILTER_TOGGLE_D: {
			int deck = act - MACT_FILTER_TOGGLE_A;
			if (deck >= 4)
				break;
			g_filter_on[deck] ^= 1;
			{
				char ln[32];
				snprintf(ln, sizeof(ln), "led_filter_%c",
					 'a' + deck);
				if (g_filter_on[deck])
					led_on(ln);
				else
					led_off(ln);
			}
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Deck %c: filter %s", DECK_NUM(deck),
				 g_filter_on[deck] ? "ON" : "OFF");
			break;
		}
		/* lib_fwd — enter highlighted directory (same as ENTER on a dir entry) */
		case MACT_LIB_FWD: {
			if (g_panel == 0 && g_fb_count > 0 &&
			    g_fb_entries[g_fb_sel].is_dir)
				fb_enter_dir(g_fb_entries[g_fb_sel].name);
			break;
		}
		/* Motor on/off — now driven by play and slip mode, not separate buttons.
         * These actions remain bindable for manual override if needed. */
		case MACT_MOTOR_TOGGLE_A:
		case MACT_MOTOR_TOGGLE_B:
		case MACT_MOTOR_TOGGLE_C:
		case MACT_MOTOR_TOGGLE_D: {
			int deck = act - MACT_MOTOR_TOGGLE_A;
			if (deck >= 4)
				break;
			if (g_motor_running[deck]) {
				g_slip_motor_off[deck] = 1;
				motor_set(deck, 0);
			} else {
				g_slip_motor_off[deck] = 0;
				motor_set(deck, 1);
			}
			break;
		}
		case MACT_MOTOR_ON_A:
			g_slip_motor_off[0] = 0;
			motor_set(0, 1);
			break;
		case MACT_MOTOR_ON_B:
			g_slip_motor_off[1] = 0;
			motor_set(1, 1);
			break;
		case MACT_MOTOR_ON_C:
			if (g_num_tracks > 2) {
				g_slip_motor_off[2] = 0;
				motor_set(2, 1);
			}
			break;
		case MACT_MOTOR_ON_D:
			if (g_num_tracks > 3) {
				g_slip_motor_off[3] = 0;
				motor_set(3, 1);
			}
			break;
		case MACT_MOTOR_OFF_A:
			g_slip_motor_off[0] = 1;
			motor_set(0, 0);
			break;
		case MACT_MOTOR_OFF_B:
			g_slip_motor_off[1] = 1;
			motor_set(1, 0);
			break;
		case MACT_MOTOR_OFF_C:
			if (g_num_tracks > 2) {
				g_slip_motor_off[2] = 1;
				motor_set(2, 0);
			}
			break;
		case MACT_MOTOR_OFF_D:
			if (g_num_tracks > 3) {
				g_slip_motor_off[3] = 1;
				motor_set(3, 0);
			}
			break;
		/* NS7III layer switch — 2 physical platters, 4 virtual decks.
         * Left  platter side [1]/[3]: Deck A (default) ↔ Deck C (layer 2).
         * Right platter side [2]/[4]: Deck B (default) ↔ Deck D (layer 2).
         * Active deck LED lights, inactive dims.  Playback on the previous deck
         * continues — only the routing of new input changes. */
		case MACT_DECK_SEL_1:
			motor_handoff(g_side_deck[0], 0);
			side_restack(0, 0); /* left side → deck A */
			g_active_track = 0;
			deck_leds_refresh();
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Left → Deck 1%s",
				 g_motor_running[0] ? " [motor ON]" :
						      " [motor off]");
			break;
		case MACT_DECK_SEL_3:
			motor_handoff(g_side_deck[0], 2);
			side_restack(0, 2); /* left side → deck C */
			g_active_track = 2;
			deck_leds_refresh();
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Left → Deck 3%s",
				 g_motor_running[2] ? " [motor ON]" :
						      " [motor off]");
			break;
		case MACT_DECK_SEL_2:
			motor_handoff(g_side_deck[1], 1);
			side_restack(1, 1); /* right side → deck B */
			g_active_track = 1;
			deck_leds_refresh();
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Right → Deck 2%s",
				 g_motor_running[1] ? " [motor ON]" :
						      " [motor off]");
			break;
		case MACT_DECK_SEL_4:
			motor_handoff(g_side_deck[1], 3);
			side_restack(1, 3); /* right side → deck D */
			g_active_track = 3;
			deck_leds_refresh();
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Right → Deck 4%s",
				 g_motor_running[3] ? " [motor ON]" :
						      " [motor off]");
			break;

		/* ── SHIFT button — sets modifier state ─────────────────────── */
		case MACT_SHIFT_A:
			g_pad_shift[g_side_deck[0]] = 1;
			break;
		case MACT_SHIFT_B:
			g_pad_shift[g_side_deck[1]] = 1;
			break;

		/* ── Pitch center — reset pitch to 0% ───────────────────────── */
		case MACT_PITCH_CENTER_A: {
			int dk = g_side_deck[0];
			g_tracks[dk].pitch = 1.0f;
			char ln[32];
			snprintf(ln, sizeof(ln), "led_pitch_center_%c",
				 'a' + dk);
			led_on(ln);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Deck %c pitch reset to 0%%", DECK_NUM(dk));
			break;
		}
		case MACT_PITCH_CENTER_B: {
			int dk = g_side_deck[1];
			g_tracks[dk].pitch = 1.0f;
			char ln[32];
			snprintf(ln, sizeof(ln), "led_pitch_center_%c",
				 'a' + dk);
			led_on(ln);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Deck %c pitch reset to 0%%", DECK_NUM(dk));
			break;
		}

		/* ── CUE button — standard Serato/CDJ cue behaviour ─────────── */
		/* Press while paused: set master cue point at current position   */
		/* Press while playing: jump to master cue point and pause        */
		/* Hold while paused: play from cue point; release returns to it  */
#define CUE_DEFAULT_HANDLER(ACT, DECK_IDX)                                       \
	case ACT: {                                                              \
		int dk = (DECK_IDX);                                             \
		if (dk >= g_num_tracks)                                          \
			break;                                                   \
		Track *tc = &g_tracks[dk];                                       \
		if (!tc->loaded)                                                 \
			break;                                                   \
		if (tc->playing) {                                               \
			/* Playing: jump to cue and pause */                     \
			pthread_mutex_lock(&tc->lock);                           \
			if (g_cue_default_set[dk])                               \
				tc->pos = g_cue_default_pos[dk];                 \
			tc->playing = 0;                                         \
			pthread_mutex_unlock(&tc->lock);                         \
			motor_set(dk, 0);                                        \
			deck_leds_refresh();                                     \
		} else {                                                         \
			/* Paused: set cue point here and begin held playback */ \
			g_cue_default_pos[dk] = tc->pos;                         \
			g_cue_default_set[dk] = 1;                               \
			g_cue_default_held[dk] = 1;                              \
			tc->playing = 1;                                         \
			if (!g_slip_motor_off[dk])                               \
				motor_set(dk, 1);                                \
			deck_leds_refresh();                                     \
		}                                                                \
		break;                                                           \
	}
			CUE_DEFAULT_HANDLER(MACT_CUE_DEFAULT_A, g_side_deck[0])
			CUE_DEFAULT_HANDLER(MACT_CUE_DEFAULT_B, g_side_deck[1])
#undef CUE_DEFAULT_HANDLER

		/* ── Pad mode select ──────────────────────────────────────────── */
		case MACT_PAD_MODE_CUES_A:
			g_pad_mode[g_side_deck[0]] = PAD_MODE_HOTCUE;
			pad_mode_leds_refresh(g_side_deck[0]);
			pad_leds_refresh(g_side_deck[0]);
			break;
		case MACT_PAD_MODE_CUES_B:
			g_pad_mode[g_side_deck[1]] = PAD_MODE_HOTCUE;
			pad_mode_leds_refresh(g_side_deck[1]);
			pad_leds_refresh(g_side_deck[1]);
			break;
		case MACT_PAD_MODE_AUTOROLL_A: {
			/* Toggle between AUTOLOOP and ROLL on each press of the same button.
             * First press = AUTOLOOP, SHIFT held = ROLL immediately. */
			int dk = g_side_deck[0];
			if (g_pad_shift[dk] ||
			    g_pad_mode[dk] == PAD_MODE_AUTOLOOP)
				g_pad_mode[dk] = PAD_MODE_ROLL;
			else
				g_pad_mode[dk] = PAD_MODE_AUTOLOOP;
			pad_mode_leds_refresh(dk);
			pad_leds_refresh(dk);
			break;
		}
		case MACT_PAD_MODE_AUTOROLL_B: {
			int dk = g_side_deck[1];
			if (g_pad_shift[dk] ||
			    g_pad_mode[dk] == PAD_MODE_AUTOLOOP)
				g_pad_mode[dk] = PAD_MODE_ROLL;
			else
				g_pad_mode[dk] = PAD_MODE_AUTOLOOP;
			pad_mode_leds_refresh(dk);
			pad_leds_refresh(dk);
			break;
		}
		/* Manual loop pad mode — pads 1-5 become loop_in/out/halve/double/reloop */
		case MACT_PAD_MODE_MANUAL_A:
			g_pad_mode[g_side_deck[0]] = PAD_MODE_MANUALLOOP;
			pad_mode_leds_refresh(g_side_deck[0]);
			pad_leds_refresh(g_side_deck[0]);
			break;
		case MACT_PAD_MODE_MANUAL_B:
			g_pad_mode[g_side_deck[1]] = PAD_MODE_MANUALLOOP;
			pad_mode_leds_refresh(g_side_deck[1]);
			pad_leds_refresh(g_side_deck[1]);
			break;

			/* ── Performance pads ─────────────────────────────────────────── */
			/* Pads are handled by both Note On (press) and Note Off (release).
         * Note On = velocity > 0 lands here.
         * Note Off (vel=0) is handled in the Note Off block below. */
#define PAD_HANDLER(MACT_BASE, DECK_IDX)                                             \
	case MACT_BASE + 0:                                                          \
	case MACT_BASE + 1:                                                          \
	case MACT_BASE + 2:                                                          \
	case MACT_BASE + 3:                                                          \
	case MACT_BASE + 4:                                                          \
	case MACT_BASE + 5:                                                          \
	case MACT_BASE + 6:                                                          \
	case MACT_BASE + 7: {                                                        \
		int pad = (int)(act - MACT_BASE) + 1; /* 1-8 */                      \
		int dk = (DECK_IDX);                                                 \
		if (dk >= g_num_tracks)                                              \
			break;                                                       \
		Track *tp = &g_tracks[dk];                                           \
		int mode = g_pad_mode[dk];                                           \
		if (mode == PAD_MODE_HOTCUE) {                                       \
			int ci = pad - 1; /* cue index 0-7 */                        \
			if (g_pad_shift[dk]) {                                       \
				/* SHIFT + pad = delete cue */                       \
				tp->cue[ci] = 0;                                     \
				tp->cue_set[ci] = 0;                                 \
				cache_save(tp);                                      \
				char ln[32];                                         \
				snprintf(ln, sizeof(ln), "led_cue_%d_%c", pad,       \
					 dk == g_side_deck[0] ? 'a' : 'b');          \
				led_off(ln);                                         \
				pad_leds_refresh(dk);                                \
			} else if (tp->cue_set[ci]) {                                \
				/* Jump to cue */                                    \
				pthread_mutex_lock(&tp->lock);                       \
				tp->pos = tp->cue[ci];                               \
				pthread_mutex_unlock(&tp->lock);                     \
			} else if (tp->loaded) {                                     \
				/* Set new cue */                                    \
				tp->cue[ci] = tp->pos;                               \
				tp->cue_set[ci] = 1;                                 \
				cache_save(tp);                                      \
				char ln[32];                                         \
				snprintf(ln, sizeof(ln), "led_cue_%d_%c", pad,       \
					 dk == g_side_deck[0] ? 'a' : 'b');          \
				led_on(ln);                                          \
				pad_leds_refresh(dk);                                \
			}                                                            \
		} else if (mode == PAD_MODE_AUTOLOOP && pad <= 4) {                  \
			static const float szs[4] = { 1.0f, 2.0f, 4.0f,              \
						      8.0f };                        \
			autoloop_engage(dk, szs[pad - 1]);                           \
		} else if (mode == PAD_MODE_ROLL && pad <= 4) {                      \
			static const float szs[4] = { 1.0f, 2.0f, 4.0f,              \
						      8.0f };                        \
			roll_begin(dk, pad, szs[pad - 1]);                           \
		} else if (mode == PAD_MODE_MANUALLOOP) {                            \
			/* Pad 1=loop_in  2=loop_out  3=halve  4=double  5=reloop */ \
			if (pad == 1) {                                              \
				tp->loop_start = tp->pos;                            \
				pad_leds_refresh(dk);                                \
			} else if (pad == 2 && tp->loop_start < tp->pos) {           \
				pthread_mutex_lock(&tp->lock);                       \
				tp->loop_end = tp->pos;                              \
				tp->looping = 1;                                     \
				pthread_mutex_unlock(&tp->lock);                     \
				pad_leds_refresh(dk);                                \
			} else if (pad == 3 && tp->looping) {                        \
				pthread_mutex_lock(&tp->lock);                       \
				uint32_t half =                                      \
					(tp->loop_end - tp->loop_start) / 2;         \
				if (half >= 1)                                       \
					tp->loop_end = tp->loop_start + half;        \
				pthread_mutex_unlock(&tp->lock);                     \
			} else if (pad == 4 && tp->looping) {                        \
				pthread_mutex_lock(&tp->lock);                       \
				uint32_t len = tp->loop_end - tp->loop_start;        \
				uint32_t newend = tp->loop_start + len * 2;          \
				if (newend <= tp->num_frames)                        \
					tp->loop_end = newend;                       \
				pthread_mutex_unlock(&tp->lock);                     \
			} else if (pad == 5) {                                       \
				pthread_mutex_lock(&tp->lock);                       \
				tp->looping = !tp->looping;                          \
				if (tp->looping && tp->pos >= tp->loop_end)          \
					tp->pos = tp->loop_start;                    \
				pthread_mutex_unlock(&tp->lock);                     \
				pad_leds_refresh(dk);                                \
			}                                                            \
		}                                                                    \
		break;                                                               \
	}
			PAD_HANDLER(MACT_PAD_1_A, g_side_deck[0])
			PAD_HANDLER(MACT_PAD_1_B, g_side_deck[1])
#undef PAD_HANDLER

			/* ── Parameter left/right: resize current loop ───────────────── */
#define PARAM_HANDLER(LEFT_ACT, RIGHT_ACT, DECK_IDX)           \
	case LEFT_ACT: {                                       \
		int dk = (DECK_IDX);                           \
		if (dk >= g_num_tracks)                        \
			break;                                 \
		if (g_pad_mode[dk] == PAD_MODE_AUTOLOOP ||     \
		    g_pad_mode[dk] == PAD_MODE_ROLL) {         \
			float nb = g_autoloop_bars[dk] * 0.5f; \
			if (nb < 0.125f)                       \
				nb = 0.125f;                   \
			g_autoloop_bars[dk] = nb;              \
			if (g_tracks[dk].looping)              \
				autoloop_engage(dk, nb);       \
			pad_leds_refresh(dk);                  \
		}                                              \
		break;                                         \
	}                                                      \
	case RIGHT_ACT: {                                      \
		int dk = (DECK_IDX);                           \
		if (dk >= g_num_tracks)                        \
			break;                                 \
		if (g_pad_mode[dk] == PAD_MODE_AUTOLOOP ||     \
		    g_pad_mode[dk] == PAD_MODE_ROLL) {         \
			float nb = g_autoloop_bars[dk] * 2.0f; \
			if (nb > 64.0f)                        \
				nb = 64.0f;                    \
			g_autoloop_bars[dk] = nb;              \
			if (g_tracks[dk].looping)              \
				autoloop_engage(dk, nb);       \
			pad_leds_refresh(dk);                  \
		}                                              \
		break;                                         \
	}
			PARAM_HANDLER(MACT_PARAM_LEFT_A, MACT_PARAM_RIGHT_A,
				      g_side_deck[0])
			PARAM_HANDLER(MACT_PARAM_LEFT_B, MACT_PARAM_RIGHT_B,
				      g_side_deck[1])
#undef PARAM_HANDLER

			/* ── FX button: cycle effect type in the slot ──────────────────── */
/* FX_SLOT_TOGGLE: press = toggle slot on/off; shift+press = cycle to next effect.
 * g_fx_last_type[][] remembers the last non-NONE type so toggling back on
 * restores the previously selected effect rather than defaulting to type 1. */
#define FX_SLOT_TOGGLE(dk, sl)                                     \
	if (g_pad_shift[dk]) {                                     \
		/* Shift: cycle to next effect type */             \
		int nt = (fx_slot(dk, sl)->type + 1) % FX_COUNT;   \
		if (nt == FX_NONE)                                 \
			nt = 1;                                    \
		g_fx_last_type[dk][sl] = nt;                       \
		fx_set_type(dk, sl, nt);                           \
		if (g_fx_param_acc[dk][3] < 0.3f)                  \
			g_fx_param_acc[dk][3] = 0.5f;              \
		fx_set_wet(dk, sl, g_fx_param_acc[dk][3]);         \
	} else if (fx_slot(dk, sl)->type == FX_NONE) {             \
		/* Currently off: turn on to last selected type */ \
		int last = g_fx_last_type[dk][sl];                 \
		if (last <= FX_NONE || last >= FX_COUNT)           \
			last = 1;                                  \
		fx_set_type(dk, sl, last);                         \
		if (g_fx_param_acc[dk][3] < 0.3f)                  \
			g_fx_param_acc[dk][3] = 0.5f;              \
		fx_set_wet(dk, sl, g_fx_param_acc[dk][3]);         \
	} else {                                                   \
		/* Currently on: save type and turn off */         \
		g_fx_last_type[dk][sl] = fx_slot(dk, sl)->type;    \
		fx_set_type(dk, sl, FX_NONE);                      \
	}

#define FX_BTN_HANDLER(BTN1, BTN2, BTN3, DECK_IDX)                            \
	case BTN1: {                                                          \
		int dk = (DECK_IDX);                                          \
		g_fx_ui_slot[dk] = 0;                                         \
		FX_SLOT_TOGGLE(dk, 0)                                         \
		snprintf(g_fb_status, sizeof(g_fb_status), "Deck %c FX1: %s", \
			 DECK_NUM(dk),                                        \
			 fx_names[fx_slot(dk, 0)->pending_type >= 0 ?         \
					  fx_slot(dk, 0)->pending_type :      \
					  fx_slot(dk, 0)->type]);             \
		fx_leds_refresh(dk);                                          \
		break;                                                        \
	}                                                                     \
	case BTN2: {                                                          \
		int dk = (DECK_IDX);                                          \
		g_fx_ui_slot[dk] = 1;                                         \
		FX_SLOT_TOGGLE(dk, 1)                                         \
		snprintf(g_fb_status, sizeof(g_fb_status), "Deck %c FX2: %s", \
			 DECK_NUM(dk),                                        \
			 fx_names[fx_slot(dk, 1)->pending_type >= 0 ?         \
					  fx_slot(dk, 1)->pending_type :      \
					  fx_slot(dk, 1)->type]);             \
		fx_leds_refresh(dk);                                          \
		break;                                                        \
	}                                                                     \
	case BTN3: {                                                          \
		int dk = (DECK_IDX);                                          \
		g_fx_ui_slot[dk] = 2;                                         \
		FX_SLOT_TOGGLE(dk, 2)                                         \
		snprintf(g_fb_status, sizeof(g_fb_status), "Deck %c FX3: %s", \
			 DECK_NUM(dk),                                        \
			 fx_names[fx_slot(dk, 2)->pending_type >= 0 ?         \
					  fx_slot(dk, 2)->pending_type :      \
					  fx_slot(dk, 2)->type]);             \
		fx_leds_refresh(dk);                                          \
		break;                                                        \
	}
		case MACT_TAP_BPM_A:
		case MACT_TAP_BPM_B: {
			int deck = act - MACT_TAP_BPM_A;
			if (g_pad_shift[deck])
				snap_grid(deck);
			else
				tap_bpm(deck);
			break;
		}
		case MACT_GRID_SNAP_A:
		case MACT_GRID_SNAP_B: {
			int deck = act - MACT_GRID_SNAP_A;
			snap_grid(deck);
			break;
		}

			FX_BTN_HANDLER(MACT_FX_BTN_1_A, MACT_FX_BTN_2_A,
				       MACT_FX_BTN_3_A, g_side_deck[0])
			FX_BTN_HANDLER(MACT_FX_BTN_1_B, MACT_FX_BTN_2_B,
				       MACT_FX_BTN_3_B, g_side_deck[1])
#undef FX_BTN_HANDLER

		default:
			break;
		}
	}

	/* Note Off (type 0x80 or Note On with velocity 0): handle jog touch release */
	if (type == 0x80 || (type == 0x90 && data2 == 0)) {
		switch (act) {
		case MACT_JOG_TOUCH_A:
			g_jog_touched[0] = 0;
			g_jog_nudge[0] = 0;
			g_last_applied_nudge[0] = 0;
			if (!g_motor_running[0])
				g_motor_vel[0] = 0.0f; /* resume normal play */
			break;
		case MACT_JOG_TOUCH_B:
			g_jog_touched[1] = 0;
			g_jog_nudge[1] = 0;
			g_last_applied_nudge[1] = 0;
			if (!g_motor_running[1])
				g_motor_vel[1] = 0.0f;
			break;
		case MACT_JOG_TOUCH_C:
			/* No g_num_tracks guard — touch release must work via layer switch */
			g_jog_touched[2] = 0;
			g_jog_nudge[2] = 0;
			g_last_applied_nudge[2] = 0;
			if (!g_motor_running[2])
				g_motor_vel[2] = 0.0f;
			break;
		case MACT_JOG_TOUCH_D:
			g_jog_touched[3] = 0;
			g_jog_nudge[3] = 0;
			g_last_applied_nudge[3] = 0;
			if (!g_motor_running[3])
				g_motor_vel[3] = 0.0f;
			break;
		/* Library encoder touch release: start the 1 s auto-restore timer */
		case MACT_LIB_ENCODER_TOUCH: {
			struct timespec _lts3;
			clock_gettime(CLOCK_MONOTONIC, &_lts3);
			g_lib_enc_last_ms = (int64_t)_lts3.tv_sec * 1000 +
					    _lts3.tv_nsec / 1000000;
			break;
		}
		/* SHIFT release */
		case MACT_SHIFT_A:
			g_pad_shift[g_side_deck[0]] = 0;
			break;
		case MACT_SHIFT_B:
			g_pad_shift[g_side_deck[1]] = 0;
			break;
		/* Bleep release — restore forward direction and snap back to saved position */
		case MACT_BLEEP_A:
		case MACT_BLEEP_B:
		case MACT_BLEEP_C:
		case MACT_BLEEP_D: {
			int deck = act - MACT_BLEEP_A;
			if (!g_bleep_held[deck] || deck >= 4)
				break;
			g_bleep_held[deck] = 0;
			g_tracks[deck].reverse = 0;
			if (g_motor_running[deck])
				motor_set_direction(deck, 0);
			pthread_mutex_lock(&g_tracks[deck].lock);
			g_tracks[deck].pos = g_bleep_save_pos[deck];
			pthread_mutex_unlock(&g_tracks[deck].lock);
			break;
		}
		/* CUE release — if held, stop and return to cue point */
		case MACT_CUE_DEFAULT_A: {
			int dk = g_side_deck[0];
			if (g_cue_default_held[dk] && dk < 4) {
				g_cue_default_held[dk] = 0;
				g_tracks[dk].playing = 0;
				motor_set(dk, 0);
				pthread_mutex_lock(&g_tracks[dk].lock);
				if (g_cue_default_set[dk])
					g_tracks[dk].pos =
						g_cue_default_pos[dk];
				pthread_mutex_unlock(&g_tracks[dk].lock);
				deck_leds_refresh();
			}
			break;
		}
		case MACT_CUE_DEFAULT_B: {
			int dk = g_side_deck[1];
			if (g_cue_default_held[dk] && dk < 4) {
				g_cue_default_held[dk] = 0;
				g_tracks[dk].playing = 0;
				motor_set(dk, 0);
				pthread_mutex_lock(&g_tracks[dk].lock);
				if (g_cue_default_set[dk])
					g_tracks[dk].pos =
						g_cue_default_pos[dk];
				pthread_mutex_unlock(&g_tracks[dk].lock);
				deck_leds_refresh();
			}
			break;
		}
		/* Roll release — pad note-off ends the roll and resumes position.
         * Manual loop pads do nothing on release. */
		case MACT_PAD_1_A:
		case MACT_PAD_2_A:
		case MACT_PAD_3_A:
		case MACT_PAD_4_A:
		case MACT_PAD_5_A:
		case MACT_PAD_6_A:
		case MACT_PAD_7_A:
		case MACT_PAD_8_A:
			if (g_pad_mode[g_side_deck[0]] == PAD_MODE_ROLL)
				roll_end(g_side_deck[0]);
			break;
		case MACT_PAD_1_B:
		case MACT_PAD_2_B:
		case MACT_PAD_3_B:
		case MACT_PAD_4_B:
		case MACT_PAD_5_B:
		case MACT_PAD_6_B:
		case MACT_PAD_7_B:
		case MACT_PAD_8_B:
			if (g_pad_mode[g_side_deck[1]] == PAD_MODE_ROLL)
				roll_end(g_side_deck[1]);
			break;
		default:
			break;
		}
	}
}

static void *midi_thread(void *arg)
{
	(void)arg;

	uint8_t running_status = 0;
	uint8_t msg[3] = { 0 };
	int msg_len = 0;
	int msg_pos = 0;
	int in_sysex = 0;
	snd_rawmidi_t *last_handle =
		NULL; /* detect handle change → reset parser */

	while (g_running) {
		/* Wait for a device to be opened */
		if (!g_midi_in) {
			usleep(20000);
			continue;
		}

		/* Device changed (hot-switch) — reset parser state */
		if (g_midi_in != last_handle) {
			last_handle = g_midi_in;
			running_status = 0;
			msg_len = 0;
			msg_pos = 0;
			in_sysex = 0;
		}

		/* Snapshot the handle — if g_midi_in is NULLed mid-read by the UI
         * thread, the read returns an error and we loop back safely. */
		snd_rawmidi_t *h = g_midi_in;
		if (!h) {
			usleep(500);
			continue;
		}

		uint8_t b;
		int r = snd_rawmidi_read(h, &b, 1);
		if (r != 1) {
			usleep(500); /* nothing available — yield briefly */
			continue;
		}

		/* ── Real-time messages (single byte, can appear anywhere) ── */
		if (b >= 0xF8) {
			/* Clock (0xF8), Start (0xFA), Stop (0xFC), etc. — ignore */
			continue;
		}

		/* ── SysEx handling ── */
		if (b == 0xF0) {
			in_sysex = 1;
			continue;
		}
		if (in_sysex) {
			if (b == 0xF7)
				in_sysex = 0;
			continue;
		}

		/* ── Status byte (bit 7 set, not real-time, not SysEx) ── */
		if (b & 0x80) {
			uint8_t type = b & 0xF0;
			/* Determine expected message length */
			if (type == 0x80 ||
			    type == 0x90 || /* Note Off / Note On     */
			    type == 0xA0 ||
			    type == 0xB0 || /* Aftertouch / CC        */
			    type == 0xE0) { /* Pitch Bend             */
				msg_len = 3;
			} else if (type == 0xC0 ||
				   type == 0xD0) { /* Program / Chan Pressure */
				msg_len = 2;
			} else {
				/* 0xF1–0xF7 system messages we don't use — reset and skip */
				msg_len = 0;
				msg_pos = 0;
				running_status = 0;
				continue;
			}
			running_status = b;
			msg[0] = b;
			msg_pos = 1;
			continue;
		}

		/* ── Data byte ── */
		if (msg_len == 0) {
			/* No status established yet — could be running status from before
             * we started listening.  If we have a running status, use it. */
			if (running_status == 0)
				continue; /* truly lost — discard */
			msg[0] = running_status;
			msg_pos = 1;
			msg_len = ((running_status & 0xF0) == 0xC0 ||
				   (running_status & 0xF0) == 0xD0) ?
					  2 :
					  3;
		}

		if (msg_pos == 0) {
			/* Running status resumes: re-use last status byte */
			msg[0] = running_status;
			msg_pos = 1;
		}

		msg[msg_pos++] = b;

		if (msg_pos == msg_len) {
			/* Complete message assembled */
			if (msg_len == 3)
				handle_midi(msg[0], msg[1], msg[2]);
			else if (msg_len == 2)
				handle_midi(msg[0], msg[1], 0);

			/* Running status: keep msg[0], reset data position for next pair */
			msg_pos = 1;
		}
	}
	return NULL;
}

/* ──────────────────────────────────────────────
   File Browser
   ────────────────────────────────────────────── */

/* qsort comparator: dirs first, then alpha */
static int fb_cmp(const void *a, const void *b)
{
	const FBEntry *ea = (const FBEntry *)a;
	const FBEntry *eb = (const FBEntry *)b;
	if (ea->is_dir != eb->is_dir)
		return eb->is_dir - ea->is_dir;
	return strcasecmp(ea->name, eb->name);
}

static int fb_cmp_bpm_asc(const void *a, const void *b)
{
	const FBEntry *ea = (const FBEntry *)a;
	const FBEntry *eb = (const FBEntry *)b;
	if (ea->is_dir != eb->is_dir)
		return eb->is_dir - ea->is_dir;
	if (ea->bpm == 0.0f && eb->bpm == 0.0f)
		return strcasecmp(ea->name, eb->name);
	if (ea->bpm == 0.0f)
		return 1; /* unknown BPM sinks to bottom */
	if (eb->bpm == 0.0f)
		return -1;
	if (ea->bpm < eb->bpm)
		return -1;
	if (ea->bpm > eb->bpm)
		return 1;
	return 0;
}

static int fb_cmp_bpm_desc(const void *a, const void *b)
{
	return fb_cmp_bpm_asc(b, a);
}

static void fb_apply_sort(void)
{
	if (g_fb_count <= 1)
		return;
	switch (g_fb_sort) {
	case 1:
		qsort(g_fb_entries, (size_t)g_fb_count, sizeof(FBEntry),
		      fb_cmp_bpm_asc);
		break;
	case 2:
		qsort(g_fb_entries, (size_t)g_fb_count, sizeof(FBEntry),
		      fb_cmp_bpm_desc);
		break;
	default:
		qsort(g_fb_entries, (size_t)g_fb_count, sizeof(FBEntry),
		      fb_cmp);
		break;
	}
}

/* ── Library sort comparators ── */
static int lib_cmp_name(const void *a, const void *b)
{
	return strcasecmp(((const LIBEntry *)a)->name,
			  ((const LIBEntry *)b)->name);
}
static int lib_cmp_bpm_asc(const void *a, const void *b)
{
	float ba = ((const LIBEntry *)a)->bpm, bb = ((const LIBEntry *)b)->bpm;
	if (ba == 0.0f && bb == 0.0f)
		return strcasecmp(((const LIBEntry *)a)->name,
				  ((const LIBEntry *)b)->name);
	if (ba == 0.0f)
		return 1;
	if (bb == 0.0f)
		return -1;
	return (ba < bb) ? -1 : (ba > bb) ? 1 : 0;
}
static int lib_cmp_bpm_desc(const void *a, const void *b)
{
	return lib_cmp_bpm_asc(b, a);
}

static void lib_apply_sort(void)
{
	if (!g_lib || g_lib_count <= 1)
		return;
	switch (g_lib_sort) {
	case 1:
		qsort(g_lib, (size_t)g_lib_count, sizeof(LIBEntry),
		      lib_cmp_bpm_asc);
		break;
	case 2:
		qsort(g_lib, (size_t)g_lib_count, sizeof(LIBEntry),
		      lib_cmp_bpm_desc);
		break;
	default:
		qsort(g_lib, (size_t)g_lib_count, sizeof(LIBEntry),
		      lib_cmp_name);
		break;
	}
}

/* Recursive library scan — called in background thread.
 * Walks dirpath depth-first and appends every supported audio file
 * to g_lib[].  Safe to call with g_lib_scanning == 1 guard. */
static void lib_scan_dir(const char *dirpath)
{
	DIR *d = opendir(dirpath);
	if (!d)
		return;
	struct dirent *ent;
	while ((ent = readdir(d)) && g_lib_count < LIB_MAX) {
		if (ent->d_name[0] == '.')
			continue;
		char full[FB_PATH_MAX + 256];
		snprintf(full, sizeof(full), "%s/%s", dirpath, ent->d_name);
		struct stat st;
		if (stat(full, &st) != 0)
			continue;
		if (S_ISDIR(st.st_mode)) {
			lib_scan_dir(full); /* recurse */
		} else if (S_ISREG(st.st_mode) &&
			   af_is_supported(ent->d_name)) {
			LIBEntry *e = &g_lib[g_lib_count++];
			snprintf(e->path, sizeof(e->path), "%s", full);
			snprintf(e->name, sizeof(e->name), "%s", ent->d_name);
			e->bpm = 0.0f;
			e->tag_title[0] = '\0';
			e->tag_artist[0] = '\0';
		}
	}
	closedir(d);
}

static void *lib_scan_thread(void *arg)
{
	(void)arg;
	g_lib_count = 0;
	g_lib_sel = 0;
	g_lib_scroll = 0;
	lib_scan_dir(g_lib_root);

	/* Read tags for every file found */
	for (int i = 0; i < g_lib_count; i++) {
		LIBEntry *e = &g_lib[i];
		read_tags(e->path, e->tag_title, sizeof(e->tag_title),
			  e->tag_artist, sizeof(e->tag_artist));
	}

	/* Try to pull BPMs from mixxxdb — match by full path */
	{
		const char *home = getenv("HOME");
		char db_path[1024];
		if (home) {
			snprintf(db_path, sizeof(db_path),
				 "%s/.mixxx/mixxxdb.sqlite", home);
			sqlite3 *db = NULL;
			if (sqlite3_open_v2(db_path, &db,
					    SQLITE_OPEN_READONLY |
						    SQLITE_OPEN_NOMUTEX,
					    NULL) == SQLITE_OK) {
				const char *sql =
					"SELECT tl.location, l.bpm "
					"FROM library l "
					"JOIN track_locations tl ON tl.id = l.location "
					"WHERE l.mixxx_deleted = 0 AND l.bpm > 0;";
				sqlite3_stmt *stmt = NULL;
				if (sqlite3_prepare_v2(db, sql, -1, &stmt,
						       NULL) == SQLITE_OK) {
					while (sqlite3_step(stmt) ==
					       SQLITE_ROW) {
						const char *loc = (const char *)
							sqlite3_column_text(
								stmt, 0);
						float bpm = (float)
							sqlite3_column_double(
								stmt, 1);
						if (!loc || bpm <= 0.0f)
							continue;
						for (int i = 0; i < g_lib_count;
						     i++) {
							if (strcmp(g_lib[i].path,
								   loc) == 0) {
								g_lib[i].bpm =
									bpm;
								break;
							}
						}
					}
					sqlite3_finalize(stmt);
				}
				sqlite3_close(db);
			}
		}
	}

	lib_apply_sort();
	g_lib_scanning = 0;
	return NULL;
}

static void lib_start_scan(const char *root)
{
	if (g_lib_scanning)
		return; /* scan already in progress */
	if (!g_lib)
		g_lib = malloc(LIB_MAX * sizeof(LIBEntry));
	if (!g_lib)
		return;
	strncpy(g_lib_root, root, sizeof(g_lib_root) - 1);
	g_lib_root[sizeof(g_lib_root) - 1] = '\0';
	g_lib_scanning = 1;
	pthread_t tid;
	pthread_create(&tid, NULL, lib_scan_thread, NULL);
	pthread_detach(tid);
}

/* Populate g_fb_entries for g_fb_path */
/* Query mixxxdb for BPMs of all file entries in the current directory.
 * Uses the same tl.location path match that mixxx_import() uses —
 * we know it works because BPM shows correctly on track load.
 * Match: tl.location LIKE '/path/to/dir/%'  (all files in that dir).
 * tl.filename (basename) is returned so we can match FBEntry.name.   */
static void fb_lookup_bpms(void)
{
	const char *home = getenv("HOME");
	if (!home)
		return;
	char db_path[1024];
	snprintf(db_path, sizeof(db_path), "%s/.mixxx/mixxxdb.sqlite", home);

	sqlite3 *db = NULL;
	if (sqlite3_open_v2(db_path, &db,
			    SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX,
			    NULL) != SQLITE_OK) {
		if (db)
			sqlite3_close(db);
		return;
	}

	/* Normalize the path: collapse any double-slashes and strip trailing
     * slash.  g_fb_path can accumulate "//" when navigating from a path
     * that already had a trailing slash (e.g. from config or argv).    */
	char clean_path[FB_PATH_MAX];
	{
		const char *s = g_fb_path;
		int o = 0;
		while (*s && o < FB_PATH_MAX - 1) {
			if (*s == '/' && *(s + 1) == '/') {
				s++;
				continue;
			}
			clean_path[o++] = *s++;
		}
		clean_path[o] = '\0';
		while (o > 1 && clean_path[o - 1] == '/')
			clean_path[--o] = '\0';
	}

	/* Build LIKE pattern: clean_path + "/%" matches every file in dir. */
	char like_pat[FB_PATH_MAX * 2 + 4];
	{
		const char *src = clean_path;
		int out = 0;
		while (*src && out < (int)sizeof(like_pat) - 4) {
			if (*src == '%' || *src == '_' || *src == '\\')
				like_pat[out++] = '\\';
			like_pat[out++] = *src++;
		}
		/* Ensure no trailing slash before the wildcard (except root) */
		while (out > 1 && like_pat[out - 1] == '/')
			out--;
		like_pat[out++] = '/';
		like_pat[out++] = '%';
		like_pat[out] = '\0';
	}

	/* Simple query: location prefix match → return (basename, bpm).
     * We use tl.filename which Mixxx stores as just the basename. */
	const char *sql = "SELECT tl.filename, l.bpm "
			  "FROM library l "
			  "JOIN track_locations tl ON tl.id = l.location "
			  "WHERE l.mixxx_deleted = 0 "
			  "  AND l.bpm > 0 "
			  "  AND tl.location LIKE ? ESCAPE '\\';";

	sqlite3_stmt *stmt = NULL;
	if (sqlite3_prepare_v2(db, sql, -1, &stmt, NULL) != SQLITE_OK) {
		sqlite3_close(db);
		return;
	}
	sqlite3_bind_text(stmt, 1, like_pat, -1, SQLITE_STATIC);

	while (sqlite3_step(stmt) == SQLITE_ROW) {
		const char *fname = (const char *)sqlite3_column_text(stmt, 0);
		float bpm = (float)sqlite3_column_double(stmt, 1);
		if (!fname || bpm <= 0.0f)
			continue;

		/* Match against browser entries by basename */
		for (int i = 0; i < g_fb_count; i++) {
			if (!g_fb_entries[i].is_dir &&
			    strcmp(g_fb_entries[i].name, fname) == 0) {
				g_fb_entries[i].bpm = bpm;
				break;
			}
		}
	}
	sqlite3_finalize(stmt);
	sqlite3_close(db);
}

/* ──────────────────────────────────────────────
   MusicBrainz tag lookup (run on a detached thread)
   Uses /usr/bin/curl or libcurl via popen.
   Parses the JSON response minimally — no JSON lib needed.
   ────────────────────────────────────────────── */

/* Tiny helper: find first occurrence of key:"value" in a JSON blob and
 * copy the value string into out[max].  Returns 1 on success.          */
static int json_str(const char *json, const char *key, char *out, int max)
{
	char search[64];
	snprintf(search, sizeof(search), "\"%s\":", key);
	const char *p = strstr(json, search);
	if (!p)
		return 0;
	p += strlen(search);
	while (*p == ' ')
		p++;
	if (*p == '"') {
		p++;
		int i = 0;
		while (*p && *p != '"' && i < max - 1) {
			if (*p == '\\')
				p++; /* skip escape char */
			if (*p)
				out[i++] = *p++;
		}
		out[i] = '\0';
		return i > 0;
	}
	return 0;
}

/* Thread entry: queries MusicBrainz for the filename stored in
 * g_tag_info.query_name, fills g_tag_info fields, clears .status. */
static void *tag_lookup_thread(void *arg)
{
	(void)arg;

	/* Build a clean search term from the filename:
     * Strip extension, replace _ and - with spaces. */
	char term[256];
	snprintf(term, sizeof(term), "%s", g_tag_info.query_name);
	term[255] = '\0';
	/* Remove extension */
	char *dot = strrchr(term, '.');
	if (dot)
		*dot = '\0';
	/* Replace underscores/dashes with spaces */
	for (int i = 0; term[i]; i++)
		if (term[i] == '_' || term[i] == '-')
			term[i] = ' ';
	/* Trim leading digits (track IDs like "13141123 ") */
	char *start = term;
	while (*start && (isdigit((unsigned char)*start) || *start == ' '))
		start++;
	if (*start == '\0')
		start = term; /* all digits — use full name */

	/* URL-encode the search term (basic: replace space with +) */
	char encoded[512];
	int ei = 0;
	for (int i = 0; start[i] && ei < 510; i++) {
		unsigned char c = (unsigned char)start[i];
		if (c == ' ') {
			encoded[ei++] = '+';
		} else if (isalnum(c) || c == '.' || c == '-' || c == '_') {
			encoded[ei++] = (char)c;
		} else {
			ei += snprintf(encoded + ei, sizeof(encoded) - ei,
				       "%%%02X", c);
		}
	}
	encoded[ei] = '\0';

	/* MusicBrainz recording search — returns JSON */
	char url[768];
	snprintf(url, sizeof(url),
		 "https://musicbrainz.org/ws/2/recording"
		 "?query=%s&limit=1&fmt=json",
		 encoded);

	/* Use curl via popen — no libcurl dependency needed */
	char cmd[1024];
	snprintf(cmd, sizeof(cmd),
		 "curl -s --max-time 8 -A 'djcmd/1.0 (linux)' '%s' 2>/dev/null",
		 url);

	FILE *fp = popen(cmd, "r");
	if (!fp) {
		snprintf(g_tag_info.status, sizeof(g_tag_info.status),
			 "curl not available");
		return NULL;
	}

	char buf[8192];
	int len = 0;
	int nr;
	while ((nr = (int)fread(buf + len, 1, sizeof(buf) - len - 1, fp)) > 0)
		len += nr;
	buf[len] = '\0';
	pclose(fp);

	if (len == 0) {
		snprintf(g_tag_info.status, sizeof(g_tag_info.status),
			 "No response");
		return NULL;
	}

	/* Parse key fields from the JSON */
	/* recordings[0].title */
	char title[128] = "";
	char artist[128] = "";
	char album[128] = "";
	char date[32] = "";
	char label[128] = "";

	json_str(buf, "title", title, sizeof(title));

	/* Artist: recordings[0].artist-credit[0].artist.name */
	const char *ac = strstr(buf, "\"artist-credit\"");
	if (ac) {
		const char *an = strstr(ac, "\"name\"");
		if (an) {
			char tmp[128] = "";
			/* Re-use json_str trick inline */
			const char *p = an + 6; /* skip "name": */
			while (*p == ':' || *p == ' ')
				p++;
			if (*p == '"') {
				p++;
				int i = 0;
				while (*p && *p != '"' && i < 127) {
					if (*p == '\\')
						p++;
					if (*p)
						tmp[i++] = *p++;
				}
				tmp[i] = '\0';
				snprintf(artist, sizeof(artist), "%s", tmp);
			}
		}
	}

	/* Release (album) title */
	const char *rel = strstr(buf, "\"releases\"");
	if (rel) {
		const char *rt = strstr(rel, "\"title\"");
		if (rt) {
			const char *p = rt + 7;
			while (*p == ':' || *p == ' ')
				p++;
			if (*p == '"') {
				p++;
				int i = 0;
				while (*p && *p != '"' && i < 127) {
					if (*p == '\\')
						p++;
					if (*p)
						album[i++] = *p++;
				}
				album[i] = '\0';
			}
		}
		json_str(rel, "date", date, sizeof(date));
		/* Label */
		const char *li = strstr(rel, "\"label-info\"");
		if (li) {
			const char *ln = strstr(li, "\"name\"");
			if (ln) {
				const char *p = ln + 6;
				while (*p == ':' || *p == ' ')
					p++;
				if (*p == '"') {
					p++;
					int i = 0;
					while (*p && *p != '"' && i < 127) {
						if (*p == '\\')
							p++;
						if (*p)
							label[i++] = *p++;
					}
					label[i] = '\0';
				}
			}
		}
	}

	if (title[0] == '\0') {
		snprintf(g_tag_info.status, sizeof(g_tag_info.status),
			 "Not found");
	} else {
		snprintf(g_tag_info.title, sizeof(g_tag_info.title), "%s",
			 title);
		snprintf(g_tag_info.artist, sizeof(g_tag_info.artist), "%s",
			 artist);
		snprintf(g_tag_info.album, sizeof(g_tag_info.album), "%s",
			 album);
		snprintf(g_tag_info.date, sizeof(g_tag_info.date), "%s", date);
		snprintf(g_tag_info.label, sizeof(g_tag_info.label), "%s",
			 label);
		g_tag_info.status[0] = '\0';
	}
	return NULL;
}

/* Kick off a tag lookup for the given filename (basename only) */
static void tag_lookup_start(const char *filename)
{
	snprintf(g_tag_info.query_name, sizeof(g_tag_info.query_name), "%s",
		 filename);
	g_tag_info.query_name[255] = '\0';
	g_tag_info.title[0] = '\0';
	g_tag_info.artist[0] = '\0';
	g_tag_info.album[0] = '\0';
	g_tag_info.date[0] = '\0';
	g_tag_info.label[0] = '\0';
	snprintf(g_tag_info.status, sizeof(g_tag_info.status),
		 "Searching\u2026");
	g_tag_info.visible = 1;

	pthread_t tid;
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	pthread_create(&tid, &attr, tag_lookup_thread, NULL);
	pthread_attr_destroy(&attr);
}

/* ──────────────────────────────────────────────
   Playlist helpers
   ────────────────────────────────────────────── */

static void pl_add(const char *fullpath, const char *basename, float bpm)
{
	if (g_pl_count >= PL_MAX)
		return;
	snprintf(g_pl[g_pl_count].path, sizeof(g_pl[0].path), "%s", fullpath);
	snprintf(g_pl[g_pl_count].name, sizeof(g_pl[0].name), "%s", basename);
	g_pl[g_pl_count].bpm = bpm;
	g_pl_count++;
	/* Lookup BPM from mixxxdb if not already known */
	if (bpm <= 0.0f) {
		/* Best-effort: re-use the Mixxx import struct */
		/* (BPM will be 0 if not in DB — display as ---) */
	}
}

static void pl_remove(int idx)
{
	if (idx < 0 || idx >= g_pl_count)
		return;
	for (int i = idx; i < g_pl_count - 1; i++)
		g_pl[i] = g_pl[i + 1];
	g_pl_count--;
	if (g_pl_sel >= g_pl_count)
		g_pl_sel = g_pl_count - 1;
	if (g_pl_sel < 0)
		g_pl_sel = 0;
}

static void pl_fix_scroll(int visible)
{
	if (g_pl_sel < g_pl_scroll)
		g_pl_scroll = g_pl_sel;
	if (g_pl_sel >= g_pl_scroll + visible)
		g_pl_scroll = g_pl_sel - visible + 1;
	if (g_pl_scroll < 0)
		g_pl_scroll = 0;
}

/* Returns a display string for a browser entry:
 *   "Artist — Title"  if both tags present
 *   "Title"           if only title
 *   entry->name       (raw filename) as fallback */
static const char *fb_display_name(const FBEntry *e, char *buf, int bufsz)
{
	if (e->tag_artist[0] && e->tag_title[0]) {
		snprintf(buf, bufsz, "%s \xe2\x80\x94 %s", e->tag_artist,
			 e->tag_title);
		return buf;
	} else if (e->tag_title[0]) {
		return e->tag_title;
	}
	return e->name;
}

static void fb_scan(void)
{
	g_fb_count = 0;
	g_fb_sel = 0;
	g_fb_scroll = 0;

	DIR *d = opendir(g_fb_path);
	if (!d)
		return;

	/* Always add ".." unless we're at root */
	if (strcmp(g_fb_path, "/") != 0) {
		snprintf(g_fb_entries[g_fb_count].name,
			 sizeof(g_fb_entries[0].name), "%s", "..");
		g_fb_entries[g_fb_count].is_dir = 1;
		g_fb_entries[g_fb_count].bpm = 0.0f;
		g_fb_count++;
	}

	struct dirent *ent;
	while ((ent = readdir(d)) && g_fb_count < FB_MAX_ENTRIES) {
		if (ent->d_name[0] == '.')
			continue; /* skip hidden */

		/* Stat to determine type */
		char full[FB_PATH_MAX + 256];
		snprintf(full, sizeof(full), "%s/%s", g_fb_path, ent->d_name);
		struct stat st;
		if (stat(full, &st) != 0)
			continue;

		if (S_ISDIR(st.st_mode)) {
			snprintf(g_fb_entries[g_fb_count].name,
				 sizeof(g_fb_entries[0].name), "%s",
				 ent->d_name);
			g_fb_entries[g_fb_count].is_dir = 1;
			g_fb_entries[g_fb_count].bpm = 0.0f;
			g_fb_count++;
		} else if (S_ISREG(st.st_mode) &&
			   af_is_supported(ent->d_name)) {
			snprintf(g_fb_entries[g_fb_count].name,
				 sizeof(g_fb_entries[0].name), "%s",
				 ent->d_name);
			g_fb_entries[g_fb_count].is_dir = 0;
			g_fb_entries[g_fb_count].bpm = 0.0f;
			g_fb_count++;
		}
	}
	closedir(d);

	/* Sort by current sort mode */
	fb_apply_sort();

	/* Annotate file entries with BPM from mixxxdb (best-effort, silent fail) */
	fb_lookup_bpms();

	/* Read ID3/Vorbis tags for audio files — fast header-only reads */
	for (int i = 0; i < g_fb_count; i++) {
		FBEntry *e = &g_fb_entries[i];
		if (e->is_dir)
			continue;
		char full[FB_PATH_MAX + 256 + 2]; /* +2: '/' separator + NUL */
		full[0] = '\0';
		if (strcmp(g_fb_path, "/") == 0) {
			full[0] = '/';
			full[1] = '\0';
			/* e->name is char[256]; copy at most 255 chars + NUL into remaining space */
			memcpy(full + 1, e->name, 255);
			full[256] = '\0';
		} else {
			strncpy(full, g_fb_path, FB_PATH_MAX);
			full[FB_PATH_MAX] = '\0';
			size_t used = strlen(full);
			if (used + 1 + 255 + 1 <= sizeof(full)) {
				full[used] = '/';
				memcpy(full + used + 1, e->name, 255);
				full[used + 1 + 255] = '\0';
			}
		}
		read_tags(full, e->tag_title, sizeof(e->tag_title),
			  e->tag_artist, sizeof(e->tag_artist));
	}
}

/* Navigate into a subdirectory */
static void fb_enter_dir(const char *name)
{
	char newpath[FB_PATH_MAX + 256 +
		     2]; /* room for path + '/' + name + NUL */
	if (strcmp(name, "..") == 0) {
		/* Go up: strip last component */
		snprintf(newpath, sizeof(newpath), "%s", g_fb_path);
		char *slash = strrchr(newpath, '/');
		if (slash && slash != newpath) {
			*slash = '\0';
		} else {
			strcpy(newpath, "/");
		}
	} else {
		if (strcmp(g_fb_path, "/") == 0)
			snprintf(newpath, sizeof(newpath), "/%s", name);
		else
			snprintf(newpath, sizeof(newpath), "%s/%s", g_fb_path,
				 name);
	}
	snprintf(g_fb_path, FB_PATH_MAX, "%s", newpath);
	fb_scan();
}

static void crates_load(void)
{
	FILE *f = fopen("crates.txt", "r");
	if (!f) return;
	g_ncrate = 0;
	char line[1024];
	while (fgets(line, sizeof(line), f) && g_ncrate < MAX_CRATES) {
		if (line[0] == '#' || line[0] == '\n') continue;
		char alias[32], path[FB_PATH_MAX];
		if (sscanf(line, "%31s %1023[^\n]", alias, path) == 2) {
			strncpy(g_crates[g_ncrate].alias, alias, sizeof(g_crates[g_ncrate].alias) - 1);
			g_crates[g_ncrate].alias[sizeof(g_crates[g_ncrate].alias) - 1] = '\0';
			strncpy(g_crates[g_ncrate].path, path, sizeof(g_crates[g_ncrate].path) - 1);
			g_crates[g_ncrate].path[sizeof(g_crates[g_ncrate].path) - 1] = '\0';
			g_ncrate++;
		}
	}
	fclose(f);
}

/* ── Phase drift calculation — returns relative beat offset [-0.5, 0.5] ── */
static float get_phase_drift(int master_idx, int slave_idx)
{
	if (master_idx < 0 || slave_idx < 0) return 0.0f;
	Track *m = &g_tracks[master_idx];
	Track *s = &g_tracks[slave_idx];
	if (!m->loaded || !s->loaded || m->bpm < 1.0f || s->bpm < 1.0f) return 0.0f;
	
	/* Frames per beat at base BPM */
	float m_fpb = (60.0f * g_actual_sample_rate) / m->bpm;
	float s_fpb = (60.0f * g_actual_sample_rate) / s->bpm;

	/* Current position within the beat (0.0 to 1.0) */
	float m_phase = fmodf((float)m->pos - m->bpm_offset, m_fpb) / m_fpb;
	float s_phase = fmodf((float)s->pos - s->bpm_offset, s_fpb) / s_fpb;
	if (m_phase < 0) m_phase += 1.0f;
	if (s_phase < 0) s_phase += 1.0f;

	float diff = s_phase - m_phase;
	if (diff > 0.5f) diff -= 1.0f;
	if (diff < -0.5f) diff += 1.0f;
	return diff;
}

static void snap_grid(int deck)
{
	if (deck < 0 || deck >= MAX_TRACKS) return;
	Track *t = &g_tracks[deck];
	if (!t->loaded) return;
	
	pthread_mutex_lock(&t->lock);
	/* Set first beat offset to current position */
	t->bpm_offset = (float)t->pos;
	pthread_mutex_unlock(&t->lock);
	cache_save(t);
	snprintf(g_fb_status, sizeof(g_fb_status), "Grid snapped to Deck %c pos", DECK_NUM(deck));
}

static void tap_bpm(int deck)
{
	if (deck < 0 || deck >= MAX_TRACKS) return;
	Track *t = &g_tracks[deck];
	if (!t->loaded) return;

	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	int64_t now_ms = (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;

	/* Reset taps if idle for 2 seconds */
	if (g_tap_count[deck] > 0) {
		int last = (g_tap_idx[deck] + MAX_TAPS - 1) % MAX_TAPS;
		if (now_ms - g_tap_ms[deck][last] > 2000) {
			g_tap_count[deck] = 0;
		}
	}

	g_tap_ms[deck][g_tap_idx[deck]] = now_ms;
	g_tap_idx[deck] = (g_tap_idx[deck] + 1) % MAX_TAPS;
	if (g_tap_count[deck] < MAX_TAPS) g_tap_count[deck]++;

	if (g_tap_count[deck] >= 2) {
		/* Calculate average interval over all captured taps */
		int64_t sum = 0;
		int n = g_tap_count[deck] - 1;
		for (int i = 0; i < n; i++) {
			int i1 = (g_tap_idx[deck] + MAX_TAPS - 1 - i) % MAX_TAPS;
			int i0 = (g_tap_idx[deck] + MAX_TAPS - 2 - i) % MAX_TAPS;
			sum += (g_tap_ms[deck][i1] - g_tap_ms[deck][i0]);
		}
		float avg_ms = (float)sum / (float)n;
		float new_bpm = 60000.0f / avg_ms;

		pthread_mutex_lock(&t->lock);
		t->bpm = new_bpm;
		pthread_mutex_unlock(&t->lock);
		cache_save(t);
		snprintf(g_fb_status, sizeof(g_fb_status), "Deck %c BPM Tap: %.1f", DECK_NUM(deck), (double)new_bpm);
	} else {
		snprintf(g_fb_status, sizeof(g_fb_status), "Deck %c Tapping...", DECK_NUM(deck));
	}
}

static void crate_jump(const char *alias)
{
	for (int i = 0; i < g_ncrate; i++) {
		if (strcasecmp(g_crates[i].alias, alias) == 0) {
			strncpy(g_fb_path, g_crates[i].path, FB_PATH_MAX - 1);
			fb_scan();
			g_panel = 0;
			if (g_view == 0) g_view = 1;
			snprintf(g_fb_status, sizeof(g_fb_status), "Jumped to crate: %s", alias);
			return;
		}
	}
	snprintf(g_fb_status, sizeof(g_fb_status), "Crate not found: %s", alias);
}

/* Build the full path of the currently selected entry */
static void fb_selected_path(char *out, size_t max)
{
	if (strcmp(g_fb_path, "/") == 0)
		snprintf(out, max, "/%s", g_fb_entries[g_fb_sel].name);
	else
		snprintf(out, max, "%s/%s", g_fb_path,
			 g_fb_entries[g_fb_sel].name);
}

/* Keep scroll window tracking cursor */
static void fb_fix_scroll(int visible_rows)
{
	if (g_fb_sel < g_fb_scroll)
		g_fb_scroll = g_fb_sel;
	if (g_fb_sel >= g_fb_scroll + visible_rows)
		g_fb_scroll = g_fb_sel - visible_rows + 1;
}

/* ──────────────────────────────────────────────
   ncurses UI
   ────────────────────────────────────────────── */
/* COLOR_HEADER/ACTIVE/VU/WFM/STATUS/HOT defined in djcmd_help.h */

/* Waveform gradient uses xterm-256 color cube (indices 16–231).
 * These are available on any 256-color terminal without can_change_color.
 * We pre-compute a 32-step green→yellow→red ramp through the cube.
 * Pair indices WFM_PAIR_BASE .. WFM_PAIR_BASE+WFM_GRADIENT_PAIRS-1.
 */
/* 6×6×6 = 216 RGB cube pairs starting at WFM_PAIR_BASE.
 * Index = r*36 + g*6 + b  (each 0–5). */
#define WFM_GRADIENT_PAIRS 216
#define WFM_PAIR_BASE 16 /* color pairs 16–231 */

/* 8-color fallback gradient pairs (always initialized, used on TTY) */
#define WFM_8_GREEN 7 /* pair 7: green fg  */
#define WFM_8_YELLOW 8 /* pair 8: yellow fg */
#define WFM_8_RED 9 /* pair 9: red fg    */

/* 8-color split-frequency pairs: fg=bass color, bg=treble color.
 * Used in TTY mode to show two frequency bands per half-block column.
 * Pairs 10-17 cover the 8 useful fg/bg combos for waveform display. */
#define WFM_8_LO_HI 10 /* red fg, cyan bg   — bass heavy + highs    */
#define WFM_8_LO_MID 11 /* red fg, green bg  — bass + mids            */
#define WFM_8_MID_HI 12 /* green fg, cyan bg — mids + highs           */
#define WFM_8_KICK 13 /* bold red fg, black bg — pure kick           */
#define WFM_8_MID 14 /* bold green fg, black bg — mid-heavy         */
#define WFM_8_HI 15 /* bold cyan fg, black bg — treble-heavy       */

static int g_has_256 = 0;
static int g_is_tty =
	0; /* 1 when stdout is a real TTY (no UTF-8 block chars) */

/* Per-band running maximum for spectral normalization.
 * Tracks the brightest value seen per band over recent columns so quiet
 * treble still shows as full-height treble color rather than disappearing
 * behind loud bass.  Decays slowly so it adapts to the section being played. */
/* g_wfm_band_max removed -- band maxima now stored per-track in wfm_band_max[3] */
/* forward declarations — defined near draw_scrolling_waveform */
static void wfm_normalize_bands(float lo_raw, float mi_raw, float hi_raw,
				float *lo_n, float *mi_n, float *hi_n,
				const float band_max[3]);
static int wfm_pair_256(float ch_r, float ch_g, float ch_b);
static void draw_quit_modal(void);

/* Map a 0–5 RGB cube component to nearest xterm-256 index.
 * xterm color cube: index = 16 + 36*r + 6*g + b  (r,g,b in 0..5) */
static int xterm_cube(int r, int g, int b)
{
	return 16 + 36 * r + 6 * g + b;
}

/* ── Theme table ─────────────────────────────────────────────────────── */
/* Built from the macros in djcmd_config.h.  Index matches CFG_DEFAULT_THEME. */
static const ThemeDef g_themes[THEME_COUNT] = {
	THEME_DEFAULT,   THEME_AMBER,	  THEME_PHOSPHOR, THEME_RED_SECTOR,
	THEME_ICE,	 THEME_SYNTHWAVE, THEME_MIDNIGHT, THEME_SOLAR,
	THEME_DEEPSEA,	 THEME_VAMPIRE,
};

/* Apply g_themes[idx] — safe to call at any time after start_color().
 * Re-initialises the 6 UI pairs and the 3 waveform 8-colour pairs. */
static void apply_theme(int idx)
{
	if (idx < 0 || idx >= THEME_COUNT)
		idx = 0;
	const ThemeDef *t = &g_themes[idx];
	init_pair(COLOR_HEADER, t->header_fg, t->header_bg);
	init_pair(COLOR_ACTIVE, t->active_fg, t->active_bg);
	init_pair(COLOR_VU, t->vu_fg, t->vu_bg);
	init_pair(COLOR_WFM, t->header_fg, t->header_bg);
	init_pair(COLOR_STATUS, t->status_fg, t->status_bg);
	init_pair(COLOR_HOT, t->hot_fg, t->hot_bg);

	/* 8-colour waveform — three pure-band pairs (lo / mid / hi) */
	init_pair(WFM_8_GREEN, t->wfm8_lo, -1); /* low band  */
	init_pair(WFM_8_YELLOW, t->wfm8_mid, -1); /* mid band  */
	init_pair(WFM_8_RED, t->wfm8_hi, -1); /* high band */

	/* Split half-block pairs: derive from theme's three waveform colors.
     * FG = bottom-half band color, BG = top-half band color.
     * Named by content (lo/hi) not by literal color so all themes work. */
	init_pair(WFM_8_LO_HI, t->wfm8_lo, t->wfm8_hi); /* bass fg, hi bg  */
	init_pair(WFM_8_LO_MID, t->wfm8_lo, t->wfm8_mid); /* bass fg, mid bg */
	init_pair(WFM_8_MID_HI, t->wfm8_mid, t->wfm8_hi); /* mid  fg, hi bg  */
	init_pair(WFM_8_KICK, t->wfm8_lo, -1); /* lo only, bold   */
	init_pair(WFM_8_MID, t->wfm8_mid, -1); /* mid only        */
	init_pair(WFM_8_HI, t->wfm8_hi, -1); /* hi only, bold   */
}

static void init_colors(void)
{
	start_color();
	use_default_colors();

	/* Apply the startup theme (includes UI + 8-colour waveform pairs) */
	apply_theme(g_opts.theme_idx);

	g_has_256 = (COLORS >= 256);

	if (g_has_256) {
		/* Waveform RGB color pairs — one per (r,g,b) tuple in the
         * xterm-256 6×6×6 color cube (216 combinations).
         * Pair index = WFM_PAIR_BASE + r*36 + g*6 + b   (r,g,b in 0..5)
         * Background -1 = terminal default (transparent).
         * This is essential: half-block edge cells (▀ ▄) need a transparent
         * background so the empty half blends with the panel bg rather than
         * rendering as a hard black stripe. */
		for (int r = 0; r < 6; r++)
			for (int g = 0; g < 6; g++)
				for (int b = 0; b < 6; b++)
					init_pair(WFM_PAIR_BASE + r * 36 +
							  g * 6 + b,
						  xterm_cube(r, g, b), -1);
	}
}

/* Draw a horizontal bar */
static void draw_bar(WINDOW *w, int y, int x, int width, float val,
		     int color_pair)
{
	int filled = (int)(val * width);
	wattron(w, COLOR_PAIR(color_pair));
	for (int i = 0; i < width; i++) {
		mvwaddch(w, y, x + i, (i < filled) ? ACS_CKBOARD : ' ');
	}
	wattroff(w, COLOR_PAIR(color_pair));
}

/* Mini waveform with beat grid overlay and cue point markers */
static void draw_waveform(WINDOW *w, int y, int x, int width, Track *t)
{
	if (!t->loaded) {
		wattron(w, A_DIM);
		for (int i = 0; i < width; i++)
			mvwaddch(w, y, x + i, '-');
		wattroff(w, A_DIM);
		return;
	}

	/* ── Mini overview waveform ─────────────────────────────────────────── */
	for (int i = 0; i < width; i++) {
		float lo = 0.0f, mi = 0.0f, hi = 0.0f;
		if (t->wfm_low && t->wfm_bins > 0) {
			/* PEAK-PRESERVING SAMPLING: Scan all bins in this column and take the max */
			uint32_t bin_start = (uint32_t)i * t->wfm_bins / (uint32_t)width;
			uint32_t bin_end = (uint32_t)(i + 1) * t->wfm_bins / (uint32_t)width;
			if (bin_end > t->wfm_bins) bin_end = t->wfm_bins;
			for (uint32_t b = bin_start; b < bin_end; b++) {
				float cl = t->wfm_low[b] / 255.0f;
				float cm = t->wfm_mid[b] / 255.0f;
				float ch = t->wfm_high[b] / 255.0f;
				if (cl > lo) lo = cl;
				if (cm > mi) mi = cm;
				if (ch > hi) hi = ch;
			}
		} else if (t->num_frames > 0) {
			uint32_t step = t->num_frames / (uint32_t)width;
			if (step < 1) step = 1;
			uint32_t idx2 = (uint32_t)i * step;
			lo = fabsf(t->data[idx2 * 2] / 32768.0f);
		}

		float amp = lo > mi ? lo : mi;
		amp = amp > hi ? amp : hi;

		float ln, mn, hn;
		wfm_normalize_bands(lo, mi, hi, &ln, &mn, &hn, t->wfm_band_max);

		int pair;
		if (!g_has_256) {
			/* ── 8-color Spectral: Pick the most energetic band ── */
			if (ln >= mn && ln >= hn) pair = WFM_8_GREEN;
			else if (mn >= ln && mn >= hn) pair = WFM_8_YELLOW;
			else pair = WFM_8_RED;
		} else {
			pair = wfm_pair_256(ln, mn * 0.5f, hn * 0.2f);
		}

		wattron(w, COLOR_PAIR(pair));
		if (g_is_tty) {
			/* Enhanced ASCII ramp for TTY */
			static const char tty_chars[] = " .:-=+*#%@";
			int bi = (int)(amp * 9.0f);
			if (bi > 9) bi = 9;
			mvwaddch(w, y, x + i, tty_chars[bi]);
		} else {
			/* BRAILLE HIGH-RES (Unicode): use dots to show amplitude envelope */
			int levels = (int)(amp * 4.0f); /* 0-4 dots high per column */
			if (levels > 4) levels = 4;
			
			/* Construct Braille character based on height.
			 * Braille dot map (2x4):
			 * 1 4
			 * 2 5
			 * 3 6
			 * 7 8
			 * We'll use both columns 1 and 2 to show the same height. */
			uint8_t braille = 0;
			if (levels >= 1) braille |= (0x40 | 0x80); /* dots 7,8 (bottom) */
			if (levels >= 2) braille |= (0x04 | 0x20); /* dots 3,6 */
			if (levels >= 3) braille |= (0x02 | 0x10); /* dots 2,5 */
			if (levels >= 4) braille |= (0x01 | 0x08); /* dots 1,4 (top) */
			
			if (braille == 0) {
				mvwaddch(w, y, x + i, ' ');
			} else {
				wchar_t blk = (wchar_t)(0x2800 + braille);
				cchar_t cc;
				setcchar(&cc, (wchar_t[]){ blk, 0 }, A_NORMAL, pair, NULL);
				mvwadd_wch(w, y, x + i, &cc);
			}
		}
		wattroff(w, COLOR_PAIR(pair));
	}

	/* ── Beat grid and Ghost Phase Overlay ─────────────────────────────── */
	if (t->bpm > 0.0f && t->num_frames > 0) {
		float beat_frames = (g_actual_sample_rate * 60.0f) / t->bpm;
		float offset = t->bpm_offset;
		int beat_n = 0;
		for (float bf = offset; bf < (float)t->num_frames; bf += beat_frames, beat_n++) {
			int col = (int)(bf / (float)t->num_frames * (float)width);
			if (col < 0 || col >= width) continue;
			
			/* Only show Downbeats (Bar starts) in the mini-preview to avoid clutter. */
			int is_downbeat = ((beat_n % 4) == 0);
			if (is_downbeat) {
				/* Highlight the existing waveform character instead of overwriting it */
				mvwchgat(w, y, x + col, 1, A_REVERSE | A_BOLD, COLOR_ACTIVE, NULL);
			}
		}

		/* ── Ghost Beat Overlay ── */
		if (t->sync_locked && g_sync_master >= 0 && g_sync_master < MAX_TRACKS) {
			Track *master = &g_tracks[g_sync_master];
			if (master->loaded && master->bpm > 0.0f) {
				float m_beat_f = (g_actual_sample_rate * 60.0f) / master->bpm;
				float m_phase = fmodf((float)master->pos - master->bpm_offset, m_beat_f);
				if (m_phase < 0.0f) m_phase += m_beat_f;

				for (float bf = offset + m_phase; bf < (float)t->num_frames; bf += beat_frames) {
					int col = (int)(bf / (float)t->num_frames * (float)width);
					if (col >= 0 && col < width) {
						/* Subtle underline/bold for the ghost phase */
						mvwchgat(w, y, x + col, 1, A_BOLD, COLOR_VU, NULL);
					}
				}
			}
		}
	}

	/* ── Cue point markers ───────────────────────────────────────────── */
	static const char cue_chars[] = "12345678";
	for (int ci = 0; ci < MAX_CUES; ci++) {
		if (!t->cue_set[ci]) continue;
		int col = (int)((float)t->cue[ci] / (float)t->num_frames * (float)width);
		if (col >= 0 && col < width) {
			wattron(w, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
			mvwaddch(w, y, x + col, cue_chars[ci]);
			wattroff(w, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
		}
	}

	/* ── Playhead ────────────────────────────────────────────────────── */
	if (t->num_frames > 0) {
		int ph = (int)((float)t->pos / (float)t->num_frames * (float)width);
		if (ph >= width) ph = width - 1;
		
		/* Playhead: inverted bold bar */
		mvwchgat(w, y, x + ph, 1, A_REVERSE | A_BOLD, COLOR_HOT, NULL);
	}
}

static void draw_deck(WINDOW *w, int y, int x, int w_width, int idx)
{
	Track *t = &g_tracks[idx];
	int active = (idx == g_active_track);
	int is_master = (idx == g_sync_master);
	int in_gang = g_gang_mode && (g_gang_mask & (1 << idx));

	/* Deck header */
	if (active)
		wattron(w, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
	else
		wattron(w, COLOR_PAIR(COLOR_HEADER) | A_BOLD);

	/* Status flags in header */
	char flags[16] = "";
	if (is_master)
		strcat(flags, "M");
	if (t->sync_locked)
		strcat(flags, "S");
	if (in_gang)
		strcat(flags, "G");
	if (t->nudge != 0.0f)
		strcat(flags, "~");
	if (t->bpm_display_double == 1)
		strcat(flags, "\xc3\xb7"
			      "2"); /* ×2 — U+00D7 is 0xC3 0xB7 in UTF-8 */
	if (t->bpm_display_double == -1)
		strcat(flags,
		       "\xc2\xbd"); /* ½  — U+00BD is 0xC2 0xBD          */
	if (t->key_lock)
		strcat(flags, "KEY");
	if (t->cue_active)
		strcat(flags, "CUE");

	const char *play_status;
	if (t->pending_play)
		play_status = " \xe2\x8f\xb3 WAIT"; /* ⏳ WAIT */
	else if (t->playing)
		play_status = "\xe2\x96\xb6 PLAY"; /* ▶ PLAY */
	else if (t->loaded)
		play_status = "  STOP";
	else
		play_status = " EMPTY";

	mvwprintw(w, y, x, " DECK %c %s%s%s ", 'A' + idx, play_status,
		  flags[0] ? " [" : "", flags[0] ? flags : "");
	if (flags[0])
		mvwprintw(w, y, x + w_width - (int)strlen(flags) - 3, "%s] ",
			  flags);

	if (active)
		wattroff(w, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
	else
		wattroff(w, COLOR_PAIR(COLOR_HEADER) | A_BOLD);

	/* Filename / tag info */
	if (t->loaded) {
		char display[256];
		if (t->tag_artist[0] && t->tag_title[0])
			snprintf(display, sizeof(display), "%s \xe2\x80\x94 %s",
				 t->tag_artist, t->tag_title);
		else if (t->tag_title[0])
			snprintf(display, sizeof(display), "%s", t->tag_title);
		else {
			char *bn = strrchr(t->filename, '/');
			bn = bn ? bn + 1 : t->filename;
			snprintf(display, sizeof(display), "%s", bn);
		}
		mvwprintw(w, y + 1, x, " %-*.*s", w_width - 2, w_width - 2,
			  display);
	} else {
		mvwprintw(w, y + 1, x, " %-*s", w_width - 2, "(no file)");
	}

	/* Waveform with beat grid and cues */
	draw_waveform(w, y + 2, x + 1, w_width - 2, t);

	/* Position / total time */
	if (t->loaded && g_actual_sample_rate > 0) {
		float cur = (float)t->pos / g_actual_sample_rate;
		float tot = (float)t->num_frames / g_actual_sample_rate;
		mvwprintw(w, y + 3, x, " %02d:%05.2f / %02d:%05.2f",
			  (int)cur / 60, fmodf(cur, 60.0f), (int)tot / 60,
			  fmodf(tot, 60.0f));
	}

	/* Phase drift meter — middle of status line */
	if (g_sync_master >= 0 && idx != g_sync_master && t->loaded && g_tracks[g_sync_master].loaded) {
		float drift = get_phase_drift(g_sync_master, idx);
		char meter[14] = "[     :     ]";
		int center = 6;
		int off = (int)(drift * 12.0f); /* -6 to +6 */
		if (off < -5)
			off = -5;
		if (off > 5)
			off = 5;
		
		if (off == 0) meter[center] = '|';
		else if (off > 0) {
			for (int i=1; i<=off; i++) meter[center+i] = '>';
		} else {
			for (int i=1; i<=abs(off); i++) meter[center-i] = '<';
		}

		if (fabsf(drift) < 0.04f) wattron(w, COLOR_PAIR(COLOR_VU) | A_BOLD);
		else wattron(w, COLOR_PAIR(COLOR_HOT));
		mvwprintw(w, y + 3, x + (w_width / 2) - 7, "%s", meter);
		if (fabsf(drift) < 0.04f) wattroff(w, COLOR_PAIR(COLOR_VU) | A_BOLD);
		else wattroff(w, COLOR_PAIR(COLOR_HOT));
	}

	/* BPM — highlight if sync locked; H cycles ½/normal/×2 displayed value */
	if (t->bpm > 0) {
		float disp_bpm = t->bpm * t->pitch;
		if (t->bpm_display_double == 1)
			disp_bpm *= 2.0f;
		else if (t->bpm_display_double == -1)
			disp_bpm *= 0.5f;
		const char *bpm_marker = (t->bpm_display_double ==  1) ? "\u00d7" :   /* × */
                                 (t->bpm_display_double == -1) ? "\u00bd" :   /* ½ */
                                                                  " ";
		if (t->sync_locked)
			wattron(w, COLOR_PAIR(COLOR_HOT) | A_BOLD);
		mvwprintw(w, y + 3, x + w_width - 12, "BPM:%5.1f%s%s", disp_bpm,
			  bpm_marker, t->sync_locked ? "*" : " ");
		if (t->sync_locked)
			wattroff(w, COLOR_PAIR(COLOR_HOT) | A_BOLD);
	}

	/* Pitch slider — shows current range and center dead-zone indicator */
	float p_range = g_pitch_range_vals[g_pitch_range[idx]];
	float p_norm =
		(t->pitch - 1.0f) / p_range; /* -1 to +1 within current range */
	if (p_norm < -1.0f)
		p_norm = -1.0f;
	if (p_norm > 1.0f)
		p_norm = 1.0f;
	mvwprintw(w, y + 4, x, " PITCH");
	draw_bar(w, y + 4, x + 7, w_width - 20, (p_norm + 1.0f) * 0.5f,
		 COLOR_VU); /* map -1..+1 → 0..1 for bar */
	mvwprintw(w, y + 4, x + w_width - 13, "%s %+5.1f%%",
		  g_pitch_range_names[g_pitch_range[idx]],
		  (t->pitch - 1.0f) * 100.0f);

	/* Volume + gain indicator */
	mvwprintw(w, y + 5, x, " VOL  ");
	draw_bar(w, y + 5, x + 7, w_width - 14, t->volume, COLOR_VU);
	mvwprintw(w, y + 5, x + w_width - 7, "G%+4.1f",
		  20.0f * log10f(t->gain > 0 ? t->gain : 1e-6f));

	/* EQ — three bands with label, bar, and ±% readout.
     * Layout: LOW(6) + bar(eq_w) + read(5) + MID(6) + bar + read + HIG(6) + bar + read
     * Last char index = 31 + eq_w*3 + 4 = 35 + eq_w*3.
     * Constraint: 35 + eq_w*3 <= w_width-1  →  eq_w = (w_width-36)/3.
     * No minimum floor: at eq_w<1 fall back to compact single-line values. */
	int eq_w = (w_width - 36) / 3;

	if (eq_w >= 1) {
		/* LOW */
		mvwprintw(w, y + 6, x, " LOW  ");
		draw_bar(w, y + 6, x + 7, eq_w, (t->eq_low + 1.0f) * 0.5f,
			 COLOR_VU);
		mvwprintw(w, y + 6, x + 7 + eq_w, "%+4.0f%%",
			  t->eq_low * 100.0f);

		/* MID */
		mvwprintw(w, y + 6, x + 12 + eq_w, " MID  ");
		draw_bar(w, y + 6, x + 19 + eq_w, eq_w,
			 (t->eq_mid + 1.0f) * 0.5f, COLOR_VU);
		mvwprintw(w, y + 6, x + 19 + eq_w * 2, "%+4.0f%%",
			  t->eq_mid * 100.0f);

		/* HIG */
		mvwprintw(w, y + 6, x + 24 + eq_w * 2, " HIG  ");
		draw_bar(w, y + 6, x + 31 + eq_w * 2, eq_w,
			 (t->eq_high + 1.0f) * 0.5f, COLOR_VU);
		mvwprintw(w, y + 6, x + 31 + eq_w * 3, "%+4.0f%%",
			  t->eq_high * 100.0f);
	} else if (w_width >= 22) {
		/* Compact fallback for very narrow decks: no bars, values only */
		mvwprintw(w, y + 6, x, " L%+3.0f%% M%+3.0f%% H%+3.0f%%",
			  t->eq_low * 100.0f, t->eq_mid * 100.0f,
			  t->eq_high * 100.0f);
	}

	/* Cue point labels */
	int cx = x + 1;
	for (int ci = 0; ci < MAX_CUES; ci++) {
		if (t->cue_set[ci]) {
			float ct = (float)t->cue[ci] / g_actual_sample_rate;
			wattron(w, COLOR_PAIR(COLOR_ACTIVE));
			mvwprintw(w, y + 7, cx, "C%d:%02d:%04.1f ", ci + 1,
				  (int)ct / 60, fmodf(ct, 60.0f));
			wattroff(w, COLOR_PAIR(COLOR_ACTIVE));
		} else {
			wattron(w, A_DIM);
			mvwprintw(w, y + 7, cx, "C%d:--:-- ", ci + 1);
			wattroff(w, A_DIM);
		}
		cx += 10;
		if (cx + 10 > x + w_width)
			break;
	}

	/* Loop indicator */
	if (t->looping) {
		wattron(w, COLOR_PAIR(COLOR_HOT) | A_BOLD);
		mvwprintw(w, y + 7, x + w_width - 9, " [LOOP] ");
		wattroff(w, COLOR_PAIR(COLOR_HOT) | A_BOLD);
	}

	/* ── FX slot status row (row y+8) ───────────────────────────────────
     * Shows all three per-deck effect slots as: INIT+wet% or "---".
     * The currently UI-selected slot is drawn bold.
     * Format (15 chars):  " FX:E50 R-- F30"
     *   E=Echo G=Ping-pong R=Reverb F=Flanger C=Chorus
     *   H=pHaser D=Distortion B=Bitcrusher A=gAte W=Widener  -=None  */
	{
		static const char s_fx_ch[11] = { '-', 'E', 'G', 'R', 'F', 'C',
						  'H', 'D', 'B', 'A', 'W' };
		char seg[FX_SLOTS_PER_DECK]
			[8]; /* "E50" or "---" per slot — 8 silences GCC truncation warning */
		int seg_active[FX_SLOTS_PER_DECK];
		for (int sl = 0; sl < FX_SLOTS_PER_DECK; sl++) {
			FXSlot *fs = fx_slot(idx, sl);
			int at = (fs->pending_type >= 0) ? fs->pending_type :
							   fs->type;
			seg_active[sl] = (at > FX_NONE && at >= 0);
			if (!seg_active[sl]) {
				strcpy(seg[sl], "---");
			} else {
				char ch = (at < 11) ? s_fx_ch[at] : '?';
				int wet = (int)(fs->params[3] * 100.0f + 0.5f);
				if (wet > 99)
					wet = 99;
				if (wet < 0)
					wet = 0;
				snprintf(seg[sl], sizeof(seg[sl]), "%c%02d", ch,
					 wet);
			}
		}
		/* Draw base line dimmed */
		wattron(w, A_DIM);
		mvwprintw(w, y + 8, x, " FX:%s %s %s", seg[0], seg[1], seg[2]);
		wattroff(w, A_DIM);
		/* Redraw the UI-selected slot bold if active */
		int sel = g_fx_ui_slot[idx];
		if (sel >= 0 && sel < FX_SLOTS_PER_DECK && seg_active[sel]) {
			int col =
				x + 4 +
				sel * 4; /* " FX:" = 4, then 4 chars per slot */
			wattron(w, A_BOLD);
			mvwprintw(w, y + 8, col, "%s", seg[sel]);
			wattroff(w, A_BOLD);
		}
	}
}

/* ──────────────────────────────────────────────
   Scrolling RGB Waveform Panel
   Drawn below the deck strip for WFM_DECKS decks.

   Layout per deck panel (WFM_ROWS + 3 rows total):
     row 0        : deck label + time position
     rows 1..WFM_ROWS : waveform (amplitude bars, half-block chars)
     row WFM_ROWS+1 : beat-number ruler
     row WFM_ROWS+2 : cue/loop markers
   ────────────────────────────────────────────── */

/* Compute 3-band amplitudes for one display column.
 * out[0]=low, out[1]=mid, out[2]=high  (each 0.0-1.0).
 *
 * Fast path uses the pre-computed overview arrays.
 * Slow path (analysis not yet done) scans raw PCM with inline IIR.
 *
 * HEIGHT (return value): RMS of per-bin peak values across all overview bins
 * mapping to this column.  RMS instead of max preserves dynamic shape when
 * zoomed out -- a column with one loud kick and many quiet bins looks shorter
 * than a column of sustained loud material.  Fixes "flat at wide zoom".
 *
 * COLOUR (out[]): mean of per-bin band values so spectral character is
 * preserved at any zoom level. */
static float col_bands(const Track *t, uint32_t frame, uint32_t frames_per_col,
		       float out[3])
{
	out[0] = out[1] = out[2] = 0.0f;

	/* -- Fast path: overview arrays ready -- */
	if (t->wfm_low && t->wfm_bins > 0 && t->num_frames > 0) {
		float bin_f = (float)frame / (float)t->num_frames *
			      (float)t->wfm_bins;
		float bin_e = (float)(frame + frames_per_col) /
			      (float)t->num_frames * (float)t->wfm_bins;
		int b0 = (int)bin_f;
		int b1 = (int)bin_e;
		if (b0 >= (int)t->wfm_bins)
			b0 = (int)t->wfm_bins - 1;
		if (b1 >= (int)t->wfm_bins)
			b1 = (int)t->wfm_bins - 1;

		float sum_pk_sq = 0.0f;
		float sum_lo = 0.0f, sum_mi = 0.0f, sum_hi = 0.0f;
		int n_bins = b1 - b0 + 1;

		for (int b = b0; b <= b1; b++) {
			float lo = t->wfm_low[b] / 255.0f;
			float mi = t->wfm_mid[b] / 255.0f;
			float hi = t->wfm_high[b] / 255.0f;
			float pk = lo > mi ? lo : mi;
			pk = pk > hi ? pk : hi;
			sum_pk_sq += pk * pk;
			sum_lo += lo;
			sum_mi += mi;
			sum_hi += hi;
		}

		/* RMS height -- preserves dynamics at any zoom */
		float rms = sqrtf(sum_pk_sq / (float)n_bins);

		/* Mean colour bands */
		out[0] = sum_lo / (float)n_bins;
		out[1] = sum_mi / (float)n_bins;
		out[2] = sum_hi / (float)n_bins;

		return rms;
	}

	/* ── Slow path: scan raw PCM with inline IIR, reset state each call ── */
	float lp_low = 0.0f, lp_4k = 0.0f;
	const float a_low = 0.0426f, a_hi4k = 0.3996f;
	/* Warmup */
	uint32_t warmup = (frames_per_col < 8) ? frames_per_col : 8;
	for (uint32_t i = 0; i < warmup; i++) {
		if (frame + i >= t->num_frames)
			break;
		float s = (t->data[(frame + i) * 2] +
			   t->data[(frame + i) * 2 + 1]) *
			  (0.5f / 32768.0f);
		lp_low += a_low * (s - lp_low);
		lp_4k += a_hi4k * (s - lp_4k);
	}
	float sum_lo = 0.0f, sum_mi = 0.0f, sum_hi = 0.0f;
	uint32_t n = 0;
	for (uint32_t i = 0; i < frames_per_col; i++) {
		uint32_t f = frame + i;
		if (f >= t->num_frames)
			break;
		float s = (t->data[f * 2] + t->data[f * 2 + 1]) *
			  (0.5f / 32768.0f);
		lp_low += a_low * (s - lp_low);
		lp_4k += a_hi4k * (s - lp_4k);
		float lo = lp_low;
		float hi = s - lp_4k;
		float mi = s - lo - hi;
		sum_lo += lo * lo;
		sum_mi += mi * mi;
		sum_hi += hi * hi;
		n++;
	}
	if (n == 0)
		return 0.0f;
	out[0] = sqrtf(sum_lo / n) * 8.0f;
	out[1] = sqrtf(sum_mi / n) * 6.0f;
	out[2] = sqrtf(sum_hi / n) * 4.0f;
	float mx = out[0] > out[1] ? out[0] : out[1];
	mx = mx > out[2] ? mx : out[2];
	return mx;
}

/* Compute per-band global maxima from the overview arrays and store on Track.
 * Called once whenever wfm_low/mid/high are assigned.  Subsequent draws use
 * t->wfm_band_max[] for stable, zoom-independent normalisation. */
static void wfm_compute_band_max(Track *t)
{
	t->wfm_band_max[0] = 0.01f;
	t->wfm_band_max[1] = 0.01f;
	t->wfm_band_max[2] = 0.01f;
	if (!t->wfm_low || t->wfm_bins == 0)
		return;
	for (uint32_t b = 0; b < t->wfm_bins; b++) {
		float lo = t->wfm_low[b] / 255.0f;
		float mi = t->wfm_mid[b] / 255.0f;
		float hi = t->wfm_high[b] / 255.0f;
		if (lo > t->wfm_band_max[0])
			t->wfm_band_max[0] = lo;
		if (mi > t->wfm_band_max[1])
			t->wfm_band_max[1] = mi;
		if (hi > t->wfm_band_max[2])
			t->wfm_band_max[2] = hi;
	}
}

/* Normalize three raw band mean values against the track-global per-band max.
 * band_max[] is computed once when the overview arrays are assigned, so
 * normalization is stable and zoom-independent -- no running-max drift. */
static void wfm_normalize_bands(float lo_raw, float mi_raw, float hi_raw,
				float *lo_n, float *mi_n, float *hi_n,
				const float band_max[3])
{
	float lm = band_max[0] > 0.01f ? band_max[0] : 0.01f;
	float mm = band_max[1] > 0.01f ? band_max[1] : 0.01f;
	float hm = band_max[2] > 0.01f ? band_max[2] : 0.01f;
	*lo_n = lo_raw / lm;
	if (*lo_n > 1.0f)
		*lo_n = 1.0f;
	*mi_n = mi_raw / mm;
	if (*mi_n > 1.0f)
		*mi_n = 1.0f;
	*hi_n = hi_raw / hm;
	if (*hi_n > 1.0f)
		*hi_n = 1.0f;
}

/* Build an xterm-256 cube color pair from three independent band intensities.
 * Each channel maps one theme waveform color: lo→r, mid→g, hi→b in cube space.
 * Mild gamma opens up the darker range without crushing peaks. */
static int wfm_pair_256(float ch_r, float ch_g, float ch_b)
{
	/* Open up the darker range and increase saturation */
	ch_r = powf(ch_r, 0.5f);
	ch_g = powf(ch_g, 0.5f);
	ch_b = powf(ch_b, 0.5f);
	
	/* Map 0..1 to 0..5 for the 216-color cube */
	int r = (int)(ch_r * 5.0f + 0.5f);
	int g = (int)(ch_g * 5.0f + 0.5f);
	int b = (int)(ch_b * 5.0f + 0.5f);
	
	/* Clamp */
	if (r > 5) r = 5;
	if (g > 5) g = 5;
	if (b > 5) b = 5;
	
	/* Ensure silent parts aren't pitch black if terminal bg is dark */
	if (r == 0 && g == 0 && b == 0) { r = 1; g = 1; b = 1; }
	
	return WFM_PAIR_BASE + r * 36 + g * 6 + b;
}

/* Map a ncurses COLOR_* constant (0-7) to approximate xterm-256 RGB cube
 * components (each 0-5).  Used by the Theme waveform style so kick/snare
 * amplitude bars are tinted with the active theme's waveform palette colours.
 *
 * ncurses colours: 0=BLACK 1=RED 2=GREEN 3=YELLOW 4=BLUE 5=MAGENTA 6=CYAN 7=WHITE */
static void color_to_rgb5(short ncurses_color, int *r5, int *g5, int *b5)
{
	/* Table: approximate xterm-256 cube coords (0..5) for each ncurses color */
	static const int tbl[8][3] = {
		{ 0, 0, 0 }, /* 0 BLACK   */
		{ 4, 0, 0 }, /* 1 RED     */
		{ 0, 3, 0 }, /* 2 GREEN   */
		{ 4, 3, 0 }, /* 3 YELLOW  */
		{ 0, 0, 4 }, /* 4 BLUE    */
		{ 4, 0, 4 }, /* 5 MAGENTA */
		{ 0, 4, 4 }, /* 6 CYAN    */
		{ 5, 5, 5 }, /* 7 WHITE   */
	};
	int idx = (int)ncurses_color;
	if (idx < 0 || idx > 7)
		idx = 7;
	*r5 = tbl[idx][0];
	*g5 = tbl[idx][1];
	*b5 = tbl[idx][2];
}

/*
 * Draw the scrolling waveform panel for deck `deck_idx`.
 *
 * Serato-style centred bars using Unicode half-block characters.
 * Colors follow the active theme's three waveform hues (wfm8_lo/mid/hi)
 * and are weighted by per-band-normalized spectral content so the display
 * shows WHAT frequencies are present, not just how loud the bass is.
 */
static void draw_scrolling_waveform(WINDOW *win, int y, int x, int w,
				    int deck_idx)
{
	Track *t = &g_tracks[deck_idx];

	/* ── Panel header ── */
	int is_active = (deck_idx == g_active_track);
	wattron(win, is_active ? COLOR_PAIR(COLOR_ACTIVE) | A_BOLD :
				 COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	mvwprintw(win, y, x, " DECK %c ", DECK_NUM(deck_idx));
	wattroff(win, is_active ? COLOR_PAIR(COLOR_ACTIVE) | A_BOLD :
				  COLOR_PAIR(COLOR_HEADER) | A_BOLD);

	if (!t->loaded) {
		/* Clear panel rows cleanly */
		for (int row = 1; row <= WFM_ROWS + 2; row++) {
			wmove(win, y + row, x);
			for (int col = 0; col < w; col++)
				waddch(win, ' ');
		}
		wattron(win, A_DIM);
		mvwprintw(win, y + WFM_ROWS / 2 + 1, x + w / 2 - 5,
			  " (no track) ");
		wattroff(win, A_DIM);
		return;
	}

	/* Scrolling waveform centred on the current playback position.
     * visible_frames covers g_opts.wfm_visible_secs of audio.
     * left_frame is the track frame at the left edge of the display;
     * the playhead is always at column w/2. */
	float visible_frames = g_opts.wfm_visible_secs * g_actual_sample_rate;
	float frames_per_col = visible_frames / (float)w;
	int64_t left_frame = (int64_t)t->pos - (int64_t)(visible_frames * 0.5f);

	/* Pre-compute loop region column bounds */
	int loop_col_start = -1, loop_col_end = -1;
	if (t->looping && t->loop_end > t->loop_start) {
		loop_col_start = (int)((float)(t->loop_start - left_frame) /
				       frames_per_col);
		loop_col_end = (int)((float)(t->loop_end - left_frame) /
				     frames_per_col);
		if (loop_col_start < 0)
			loop_col_start = 0;
		if (loop_col_end >= w)
			loop_col_end = w - 1;
	}

	for (int col = 0; col < w; col++) {
		int64_t fs = left_frame + (int64_t)(col * frames_per_col);
		int in_loop = t->looping && fs >= (int64_t)t->loop_start &&
			      fs < (int64_t)t->loop_end;
		float bands[3] = { 0, 0, 0 };
		if (fs >= 0 && fs < (int64_t)t->num_frames)
			col_bands(t, (uint32_t)fs, (uint32_t)frames_per_col,
				  bands);

		/* Normalise per-band against track global maxima */
		float lo, mi, hi;
		float lo_raw = bands[0], mi_raw = bands[1], hi_raw = bands[2];
		wfm_normalize_bands(lo_raw, mi_raw, hi_raw, &lo, &mi, &hi,
				    t->wfm_band_max);

		/* Bar height: configurable band weights let the user tune how much
         * bass, snare, and hats contribute — default matches old behaviour. */
		float w_sum = g_opts.wfm_lo_weight + g_opts.wfm_mid_weight +
			      g_opts.wfm_hi_weight;
		if (w_sum < 1e-3f)
			w_sum = 1e-3f;
		float disp_amp = (lo * g_opts.wfm_lo_weight +
				  mi * g_opts.wfm_mid_weight +
				  hi * g_opts.wfm_hi_weight) /
				 w_sum;
		if (disp_amp > 1.0f)
			disp_amp = 1.0f;
		float disp = powf(disp_amp, g_opts.wfm_height_gamma);

		/* ── Bar geometry — centred vs bottom-anchored ───────────────────
         * Centred (Serato style, wfm_anchor=0): bar grows from centre out.
         *   subs_half = half the total sub-levels; bar fills both halves.
         * Bottom-anchored (Rekordbox style, wfm_anchor=1): bar grows upward
         *   from the bottom row; all sub-levels used for one direction only. */
		int total_subs = WFM_ROWS * 8;
		int subs_half =
			(g_opts.wfm_anchor == 0) ? total_subs / 2 : total_subs;
		int filled_subs = (int)(disp * (float)subs_half + 0.5f);
		if (filled_subs > subs_half)
			filled_subs = subs_half;

		/* centre_row: first row of the lower half (centred) or bottom row+1
         * (bottom-anchored, so dist is computed from the bottom). */
		int centre_row =
			(g_opts.wfm_anchor == 0) ? WFM_ROWS / 2 : WFM_ROWS;

		for (int row = 0; row < WFM_ROWS; row++) {
			int screen_row = y + 1 + row;

			/* How far is this row from the fill origin?
             * Centred:        above = row < centre_row; dist from centre.
             * Bottom-anchored: all rows count as "above" (bar grows up);
             *   dist = WFM_ROWS - 1 - row  (0 = bottom row = first filled). */
			int above, dist;
			if (g_opts.wfm_anchor == 0) {
				above = (row < centre_row);
				dist = above ? (centre_row - 1 - row) :
					       (row - centre_row);
			} else {
				above = 1;
				dist = WFM_ROWS - 1 - row;
			}

			/* Sub-levels filled in this row.
             * Innermost row (dist=0): filled_subs, capped to 8.
             * Next row out (dist=1): filled_subs - 8, etc. */
			int subs_in_row = filled_subs - dist * 8;
			if (subs_in_row <= 0) {
				/* Row is fully outside the bar — clear it */
				mvwaddch(win, screen_row, x + col, ' ');
				continue;
			}
			if (subs_in_row > 8)
				subs_in_row = 8;

			/* Vertical position fraction for colour (0=centre, 1=tip) */
			float v_frac = (subs_half > 0) ?
					       (float)(dist * 8 + subs_in_row) /
						       (float)subs_half :
					       0.0f;
			if (v_frac > 1.0f)
				v_frac = 1.0f;

			/* ── Colour at this row ──────────────────────────────────────
             * Centre rows (v_frac near 0): kick colour (red) — the body
             *   of the bar is always coloured by the bass content.
             * Mid rows (v_frac ~0.4): snare colour (green/yellow).
             * Tip rows (v_frac ~1.0): treble / hat colour (dim blue).
             *
             * Saturation scales with the actual band amplitude at this
             * column, so quiet passages are dark and loud kicks are vivid.
             *
             * onset_boost brightens the colour on transient attacks. */
			int pair;
			attr_t row_attr = A_NORMAL;

			if (!g_has_256) {
				/* 8-colour TTY: kicks=red(WFM_8_KICK), snares=blue(WFM_8_HI),
                 * hats dim.  Bold on strong transients. */
				if (lo >= mi)
					pair = (lo > 0.5f) ? WFM_8_KICK :
							     WFM_8_LO_MID;
				else
					pair = (mi > 0.5f) ? WFM_8_HI :
							     WFM_8_MID_HI;
				if (lo > 0.65f || mi > 0.65f)
					row_attr = A_BOLD;
			} else if (g_opts.wfm_style == 0) {
				/* ── Style 0: Kick/Snare ─────────────────────────────────
                 * kicks=RED, snares=BLUE.  dom ratio sharpens toward pure
                 * hues; wfm_color_sat scales the hue contrast;
                 * wfm_color_floor is the minimum total brightness so silent
                 * columns can go fully black when set to 0. */
				float r_c, g_c, b_c;
				float bright = 0.5f + lo * 0.3f + mi * 0.2f;
				if (bright > 1.0f)
					bright = 1.0f;
				bright *= g_opts.wfm_color_sat;
				if (bright > 1.0f)
					bright = 1.0f;

				float raw_sum = lo_raw + mi_raw;
				float dom = (raw_sum > 1e-6f) ?
						    lo_raw / raw_sum :
						    0.5f;
				/* sharpen: multiplying spread by sat increases colour contrast */
				dom = (dom - 0.5f) *
					      (3.0f * g_opts.wfm_color_sat) +
				      0.5f;
				if (dom > 1.0f)
					dom = 1.0f;
				if (dom < 0.0f)
					dom = 0.0f;

				float tip_dim = 1.0f - v_frac * 0.4f;
				r_c = dom * bright * tip_dim;
				g_c = hi * 0.10f * bright * tip_dim;
				b_c = (1.0f - dom) * bright * tip_dim;

				float tot = r_c + g_c + b_c;
				float cf = g_opts.wfm_color_floor;
				if (cf > 1e-6f && tot < cf) {
					float sc = cf / (tot + 0.001f);
					r_c *= sc;
					g_c *= sc;
					b_c *= sc;
				}
				pair = wfm_pair_256(r_c, g_c, b_c);

			} else {
				/* ── Style 1: Theme ──────────────────────────────────────────
                 * Amplitude bars coloured with the active theme's waveform
                 * palette.  kick amplitude (lo_raw) drives the theme's
                 * wfm8_lo ("kick") colour; snare amplitude (mi_raw) drives
                 * wfm8_hi ("snare/hot") colour.  Both are blended by their
                 * relative raw energy so the dominant hit shows in its
                 * theme colour at full brightness.
                 *
                 * colour_to_rgb5 maps ncurses COLOR_* (0-7) → cube coords.
                 * The two theme colours are mixed by the same dom ratio used
                 * in style 0, then multiplied by bar brightness and tip fade.
                 */
				const ThemeDef *th =
					&g_themes[g_opts.theme_idx];

				int kr5, kg5,
					kb5; /* theme kick colour in RGB5 */
				int sr5, sg5,
					sb5; /* theme snare colour in RGB5 */
				color_to_rgb5(th->wfm8_lo, &kr5, &kg5, &kb5);
				color_to_rgb5(th->wfm8_hi, &sr5, &sg5, &sb5);

				/* dom: 1.0 = pure kick column, 0.0 = pure snare column */
				float raw_sum = lo_raw + mi_raw;
				float dom = (raw_sum > 1e-6f) ?
						    lo_raw / raw_sum :
						    0.5f;
				dom = (dom - 0.5f) * 3.0f + 0.5f;
				if (dom > 1.0f)
					dom = 1.0f;
				if (dom < 0.0f)
					dom = 0.0f;

				/* Mix the two theme colours by dom */
				float mr5 = dom * kr5 + (1.0f - dom) * sr5;
				float mg5 = dom * kg5 + (1.0f - dom) * sg5;
				float mb5 = dom * kb5 + (1.0f - dom) * sb5;

				/* Scale by bar brightness, saturation, and tip fade */
				float bright = 0.4f + lo * 0.35f + mi * 0.25f;
				if (bright > 1.0f)
					bright = 1.0f;
				bright *= g_opts.wfm_color_sat;
				if (bright > 1.0f)
					bright = 1.0f;
				float tip_dim = 1.0f - v_frac * 0.4f;
				float scale = bright * tip_dim / 5.0f;

				float r_c = mr5 * scale;
				float g_c = mg5 * scale;
				float b_c = mb5 * scale;

				float tot = r_c + g_c + b_c;
				float cf = g_opts.wfm_color_floor;
				if (cf > 1e-6f && tot < cf) {
					float sc = cf / (tot + 0.001f);
					r_c *= sc;
					g_c *= sc;
					b_c *= sc;
				}
				pair = wfm_pair_256(r_c, g_c, b_c);
			}

			/* ── Draw character ──────────────────────────────────────────
             * TTY: ACS_BLOCK (full) or ACS_CKBOARD (partial row).
             * Unicode terminal: eighth-block chars for sub-cell resolution.
             *
             * For the top-edge row of each half of the bar (the partial row),
             * we use ▁▂▃▄▅▆▇ (lower N/8 block) for the BOTTOM half of the bar
             * (bar grows upward from centre) and the mirrored upper-bar
             * characters ▔ .. but ncurses wide-char has no upper-eighth blocks.
             * Instead we use ▄ (lower half) with a dim-matched pair for the
             * bottom partial edge, and ▀ (upper half) for the top partial edge,
             * then fall back to the lower-eighth blocks for finer resolution
             * on the bottom half only (where they naturally align). */

			if (g_is_tty) {
				wattron(win, COLOR_PAIR(pair) | row_attr);
				chtype ch = (subs_in_row >= 8) ? ACS_BLOCK :
								 ACS_CKBOARD;
				mvwaddch(win, screen_row, x + col, ch);
				wattroff(win, COLOR_PAIR(pair) | row_attr);
				continue;
			}

			wchar_t wch;
			if (subs_in_row >= 8) {
				/* Full row: solid block █ */
				wch = L'\u2588';
			} else if (above) {
				/* Top half of bar — partial row at the tip.
                 * Use LOWER-N/8 blocks (▁=1/8 .. ▇=7/8) but flipped:
                 * since the bar grows upward, the partial cell at the TOP
                 * tip should show only the bottom N/8 of the character,
                 * which is exactly what ▁..▇ provide.
                 * subs_in_row 1->▁(2581) .. 7->▇(2587) */
				wch = (wchar_t)(0x2580 +
						subs_in_row); /* ▁▂▃▄▅▆▇ at tip */
				/* Dim the tip slightly so the bar tapers visually */
				float dim = 0.45f + subs_in_row * 0.07f;
				pair = wfm_pair_256(
					((pair - WFM_PAIR_BASE) / 36) / 5.0f *
						dim,
					(((pair - WFM_PAIR_BASE) / 6) % 6) /
						5.0f * dim,
					((pair - WFM_PAIR_BASE) % 6) / 5.0f *
						dim);
			} else {
				/* Bottom half of bar — partial row at the lower tip.
                 * The bar grows downward from centre, so the filled portion
                 * is always the UPPER part of this cell → ▀ (U+2580).
                 * Dim proportionally to how few subs are filled. */
				wch = L'\u2580'; /* ▀ upper-half block — always correct here */
				float dim = 0.35f + subs_in_row * 0.08f;
				pair = wfm_pair_256(
					((pair - WFM_PAIR_BASE) / 36) / 5.0f *
						dim,
					(((pair - WFM_PAIR_BASE) / 6) % 6) /
						5.0f * dim,
					((pair - WFM_PAIR_BASE) % 6) / 5.0f *
						dim);
			}

			wattron(win, COLOR_PAIR(pair) | row_attr);
			cchar_t cc;
			wchar_t wcs[2] = { wch, L'\0' };
			setcchar(&cc, wcs, A_NORMAL, pair, NULL);
			mvwadd_wch(win, screen_row, x + col, &cc);
			wattroff(win, COLOR_PAIR(pair) | row_attr);
		}

		/* Loop region: tint background with a dim overlay.
         * We draw a half-transparent effect by toggling A_DIM on empty cells
         * and A_REVERSE on the loop boundary columns. */
		if (in_loop) {
			/* Highlight loop region boundary columns */
			int is_boundary =
				(col == loop_col_start || col == loop_col_end);
			if (is_boundary) {
				/* Draw a bright vertical bar at loop in/out points */
				for (int row = 0; row < WFM_ROWS; row++) {
					wattron(win,
						A_BOLD | COLOR_PAIR(
								 COLOR_ACTIVE));
					mvwaddch(win, y + 1 + row, x + col,
						 col == loop_col_start ? '[' :
									 ']');
					wattroff(win,
						 A_BOLD |
							 COLOR_PAIR(
								 COLOR_ACTIVE));
				}
			}
		}

		/* Playhead — always at the centre column of the scrolling window. */
		if (col == w / 2) {
			for (int row = 0; row < WFM_ROWS; row++) {
				wattron(win, A_REVERSE | A_BOLD |
						     COLOR_PAIR(COLOR_STATUS));
				mvwaddch(win, y + 1 + row, x + col, '|');
				wattroff(win, A_REVERSE | A_BOLD |
						      COLOR_PAIR(COLOR_STATUS));
			}
		}
	}

	/* ── Beat ruler ── */
	int ruler_y = y + 1 + WFM_ROWS;
	for (int col = 0; col < w; col++)
		mvwaddch(win, ruler_y, x + col, '-');

	if (t->bpm > 1.0f) {
		/* bpm_display_double: tri-state display multiplier — does not affect
         * playback or the underlying bpm/bpm_offset values.
         *  -1 = half  (÷2): beat markers every 2 real beats, labelled 1-4
         *   0 = normal: one marker per beat
         *  +1 = double (×2): markers every half-beat, labelled 1-4 at ×2 BPM */
		float disp_bpm;
		if (t->bpm_display_double == 1)
			disp_bpm = t->bpm * 2.0f;
		else if (t->bpm_display_double == -1)
			disp_bpm = t->bpm * 0.5f;
		else
			disp_bpm = t->bpm;
		float beat_frames = (g_actual_sample_rate * 60.0f) / disp_bpm;
		float left_beat =
			((float)left_frame - t->bpm_offset) / beat_frames;
		int first_beat = (int)floorf(left_beat);

		for (int bn = first_beat;; bn++) {
			float beat_frame = t->bpm_offset + bn * beat_frames;
			int64_t col_pos =
				(int64_t)((beat_frame - (float)left_frame) /
					  frames_per_col);
			if (col_pos >= w)
				break;
			if (col_pos < 0)
				continue;

			/* beat_in_bar: 1..4 using floor division so negatives work correctly */
			int beat_in_bar =
				((bn % 4) + 4) % 4 + 1; /* 1=downbeat, 2/3/4 */
			int bar = (int)floorf((float)bn / 4.0f) + 1;

			if (beat_in_bar == 1) {
				/* Downbeat: bright reversed block so it's readable over SSH */
				wattron(win, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD |
						     A_REVERSE);
				mvwaddch(win, ruler_y, x + (int)col_pos, '|');
				if (bar > 0)
					mvwprintw(win, ruler_y,
						  x + (int)col_pos + 1, "%d",
						  bar);
				wattroff(win, COLOR_PAIR(COLOR_ACTIVE) |
						      A_BOLD | A_REVERSE);
			} else {
				/* Beats 2, 3, 4: plain dim tick */
				wattron(win, COLOR_PAIR(COLOR_HOT));
				mvwaddch(win, ruler_y, x + (int)col_pos, '|');
				mvwprintw(win, ruler_y, x + (int)col_pos + 1,
					  "%d", beat_in_bar);
				wattroff(win, COLOR_PAIR(COLOR_HOT));
			}
		}

		float cur_beat = ((float)t->pos - t->bpm_offset) / beat_frames;
		int cur_bar = (int)(cur_beat / 4.0f) + 1;
		int cur_b = (int)fmodf(cur_beat, 4.0f) + 1;
		wattron(win, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
		mvwprintw(win, y, x + w - 14, "BAR:%3d", cur_bar);
		wattroff(win, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
		wattron(win, COLOR_PAIR(COLOR_HOT) | A_BOLD);
		mvwprintw(win, y, x + w - 7, " BEAT:%d", cur_b);
		wattroff(win, COLOR_PAIR(COLOR_HOT) | A_BOLD);
	}

	/* ── Loop region on beat ruler ── */
	if (t->looping && loop_col_start >= 0 && loop_col_end >= 0) {
		/* Fill loop region on ruler with a tinted background */
		for (int col = loop_col_start; col <= loop_col_end && col < w;
		     col++) {
			if (col == loop_col_start || col == loop_col_end) {
				wattron(win, A_BOLD | A_REVERSE |
						     COLOR_PAIR(COLOR_ACTIVE));
				mvwaddch(win, ruler_y, x + col,
					 col == loop_col_start ? '[' : ']');
				wattroff(win, A_BOLD | A_REVERSE |
						      COLOR_PAIR(COLOR_ACTIVE));
			} else {
				wattron(win,
					A_REVERSE | COLOR_PAIR(COLOR_ACTIVE));
				mvwaddch(win, ruler_y, x + col, '~');
				wattroff(win,
					 A_REVERSE | COLOR_PAIR(COLOR_ACTIVE));
			}
		}
		/* Show loop length label */
		float loop_secs = (float)(t->loop_end - t->loop_start) /
				  (float)g_actual_sample_rate;
		int lbl_col = loop_col_start + 1;
		if (lbl_col < loop_col_end - 6 && lbl_col < w - 6) {
			wattron(win,
				A_BOLD | A_REVERSE | COLOR_PAIR(COLOR_ACTIVE));
			if (loop_secs < 10.0f)
				mvwprintw(win, ruler_y, x + lbl_col, "|%.2fs|",
					  loop_secs);
			else
				mvwprintw(win, ruler_y, x + lbl_col, "|%.1fs|",
					  loop_secs);
			wattroff(win,
				 A_BOLD | A_REVERSE | COLOR_PAIR(COLOR_ACTIVE));
		}
	}

	/* ── Cue markers ── */
	int cue_y = y + 1 + WFM_ROWS + 1;
	for (int col = 0; col < w; col++)
		mvwaddch(win, cue_y, x + col, ' ');

	for (int ci = 0; ci < MAX_CUES; ci++) {
		if (!t->cue_set[ci])
			continue;
		int64_t col_pos =
			(int64_t)(((float)t->cue[ci] - (float)left_frame) /
				  frames_per_col);
		if (col_pos < 0 || col_pos >= w)
			continue;
		wattron(win, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		mvwprintw(win, cue_y, x + (int)col_pos, "C%d", ci + 1);
		wattroff(win, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
	}

	/* Time position */
	float cur_secs = (float)t->pos / g_actual_sample_rate;
	wattron(win, A_DIM);
	mvwprintw(win, y, x + 10, " %02d:%06.3f ", (int)cur_secs / 60,
		  fmodf(cur_secs, 60.0f));
	wattroff(win, A_DIM);

	/* LOOP indicator in header */
	if (t->looping) {
		float loop_secs = (float)(t->loop_end - t->loop_start) /
				  (float)g_actual_sample_rate;
		wattron(win, A_BOLD | A_REVERSE | COLOR_PAIR(COLOR_ACTIVE));
		mvwprintw(win, y, x + w - 22, " âº LOOP %.2fs ", loop_secs);
		wattroff(win, A_BOLD | A_REVERSE | COLOR_PAIR(COLOR_ACTIVE));
	}
}

static void draw_crossfader(WINDOW *w, int y, int x, int width)
{
	mvwprintw(w, y, x, " XFADE A");
	draw_bar(w, y, x + 9, width - 18, g_crossfader, COLOR_ACTIVE);
	mvwprintw(w, y, x + width - 9, "B  (%3.0f%%)", g_crossfader * 100.0f);
}

/* ── Vertical peak meter ──────────────────────────────────────────────────
 * Drawn as a 2-column (L+R) vertical bar between decks.
 * x/y: top-left corner.  h: height in rows (must equal draw_deck height = 8).
 *
 * On TTY (g_is_tty): ACS_BLOCK / ACS_CKBOARD, always with mvwaddch so each
 * character lands at the correct cell regardless of cursor position.
 * On terminals: Unicode half-blocks (▀ ▄ █) for 2× vertical resolution.
 *
 * Color zones are position-based (top = red zone) and only appear when the
 * bar actually reaches that zone.  "LR" label occupies the last row;
 * usable bar height = h-1.  All writes stay strictly within columns x..x+1. */
static void draw_vu_meter(WINDOW *w, int y, int x, int h)
{
	float lvl_l = g_vu_l < 1.0f ? g_vu_l : 1.0f;
	float lvl_r = g_vu_r < 1.0f ? g_vu_r : 1.0f;
	float pk_l = g_vu_peak_l < 1.0f ? g_vu_peak_l : 1.0f;
	float pk_r = g_vu_peak_r < 1.0f ? g_vu_peak_r : 1.0f;

	if (g_is_tty) {
		/* ── TTY path: one ACS char per row per channel, always mvwaddch ── */
		int bar_h = h - 1;
		int fill_l = (int)(lvl_l * (float)bar_h + 0.5f);
		int fill_r = (int)(lvl_r * (float)bar_h + 0.5f);
		int peak_row_l =
			bar_h - 1 - (int)(pk_l * (float)(bar_h - 1) + 0.5f);
		int peak_row_r =
			bar_h - 1 - (int)(pk_r * (float)(bar_h - 1) + 0.5f);
		if (fill_l > bar_h)
			fill_l = bar_h;
		if (fill_r > bar_h)
			fill_r = bar_h;

		for (int row = 0; row < bar_h; row++) {
			int row_from_bottom = bar_h - 1 - row;
			float zone =
				(float)row /
				(float)bar_h; /* 0=top(red), 1=bottom(green) */
			int color = (zone < 0.10f) ? COLOR_HOT :
				    (zone < 0.30f) ? COLOR_VU :
						     COLOR_HEADER;

			for (int ch = 0; ch < 2; ch++) {
				int cx = x + ch;
				int fill = (ch == 0) ? fill_l : fill_r;
				int pkr = (ch == 0) ? peak_row_l : peak_row_r;
				int on = (row_from_bottom < fill);
				int is_pk = (row == pkr && pk_l > 0.02f);

				wattron(w, COLOR_PAIR(color));
				mvwaddch(w, y + row, cx,
					 (on || is_pk) ?
						 (is_pk && !on ? ACS_CKBOARD :
								 ACS_BLOCK) :
						 ' ');
				wattroff(w, COLOR_PAIR(color));
			}
		}

		wattron(w, A_DIM | COLOR_PAIR(COLOR_STATUS));
		mvwprintw(w, y + h - 1, x, "LR");
		wattroff(w, A_DIM | COLOR_PAIR(COLOR_STATUS));
		return;
	}

	/* ── Terminal path: Unicode half-blocks, 2× vertical resolution ── */
	int bar_h = h - 1;
	int half_h = bar_h * 2;

	int fill_l = (int)(lvl_l * (float)half_h);
	int fill_r = (int)(lvl_r * (float)half_h);
	int peak_l = (int)(pk_l * (float)(half_h - 1));
	int peak_r = (int)(pk_r * (float)(half_h - 1));
	if (fill_l > half_h)
		fill_l = half_h;
	if (fill_r > half_h)
		fill_r = half_h;

	for (int row = 0; row < bar_h; row++) {
		int bot_half = (bar_h - 1 - row) * 2;
		int top_half = bot_half + 1;
		float zone = (float)row / (float)bar_h;
		int color = (zone < 0.10f) ? COLOR_HOT :
			    (zone < 0.30f) ? COLOR_VU :
					     COLOR_HEADER;

		for (int ch = 0; ch < 2; ch++) {
			int cx = x + ch;
			int fill = (ch == 0) ? fill_l : fill_r;
			int pk = (ch == 0) ? peak_l : peak_r;
			int bot_on = (bot_half < fill);
			int top_on = (top_half < fill);
			int bot_pk = (bot_half == pk && pk > 0);
			int top_pk = (top_half == pk && pk > 0);

			wattron(w, COLOR_PAIR(color));
			cchar_t cc;
			if ((top_on || top_pk) && (bot_on || bot_pk)) {
				setcchar(&cc, L"\u2588", A_NORMAL, color, NULL);
				mvwadd_wch(w, y + row, cx, &cc);
			} else if (top_on || top_pk) {
				setcchar(&cc, L"\u2580", A_NORMAL, color, NULL);
				mvwadd_wch(w, y + row, cx, &cc);
			} else if (bot_on || bot_pk) {
				setcchar(&cc, L"\u2584", A_NORMAL, color, NULL);
				mvwadd_wch(w, y + row, cx, &cc);
			} else {
				mvwaddch(w, y + row, cx, ' ');
			}
			wattroff(w, COLOR_PAIR(color));
		}
	}

	wattron(w, A_DIM | COLOR_PAIR(COLOR_STATUS));
	mvwprintw(w, y + h - 1, x, "LR");
	wattroff(w, A_DIM | COLOR_PAIR(COLOR_STATUS));
}

static void draw_decks_view(void)
{
	/* Deck strip at top — leave 2 cols in the middle for the VU meter.
     * 2-deck: visual order A,B  — meter after position 0.
     * 4-deck: visual order C,A,B,D (3,1,2,4) — meter after position 1
     *         (between A and B), matching the CDJ/NS7III physical layout. */
	static const int deck_pos4[4] = { 2, 0, 1, 3 };
	int meter_after = (g_num_tracks == 4) ? 1 : 0;
	int deck_rows = 9; /* rows 0-7 = deck info, row 8 = FX status */
	int meter_w = 2;
	int usable = g_cols - meter_w;
	int dw = usable / g_num_tracks;

	for (int i = 0; i < g_num_tracks; i++) {
		int di = (g_num_tracks == 4) ? deck_pos4[i] : i;
		int offset = (i > meter_after) ? meter_w : 0;
		draw_deck(g_win_main, 0, i * dw + offset, dw, di);
	}

	/* VU meter column */
	int meter_x = (meter_after + 1) * dw;
	draw_vu_meter(g_win_main, 0, meter_x, deck_rows);

	/* Scrolling waveform panels — full width, same visual order */
	int panel_h = WFM_ROWS + 3;
	int panel_y = deck_rows;
	for (int i = 0; i < g_num_tracks; i++) {
		int di = (g_num_tracks == 4) ? deck_pos4[i] : i;
		draw_scrolling_waveform(g_win_main, panel_y, 0, g_cols, di);
		panel_y += panel_h;
	}

	if (panel_y < g_rows - 1)
		draw_crossfader(g_win_main, panel_y, 0, g_cols);
}

/*
 * Split view (2-deck mode + browser):
 *   Top section  — deck strip + both waveforms + crossfader (same as decks view)
 *   Horizontal divider
 *   Bottom section — file browser using all remaining rows
 *
 * On a fullscreen TTY (e.g. PowerBook G4 at 180×56) the two waveform panels
 * consume ~28 rows, leaving ~26 rows of browser — plenty comfortable.
 */

/* Returns the number of terminal rows available for the browser/playlist panel.
 * A value <= 0 means the library is completely off-screen.
 * A value < 4 means it is technically on-screen but too small to be useful.
 * Callers can use this to show warnings. */
static int library_rows_available(void)
{
	int deck_rows = 9; /* deck strip height */
	int panel_h = WFM_ROWS + 3; /* per waveform panel */
	/* Split view always renders 2 waveform panels (the active layer on each
	 * side).  Use 2 here so the library row count reflects actual usage. */
	int used = deck_rows + panel_h * 2 + 1 /* crossfader row */
		   + 1 /* divider row    */
		   + 1; /* status bar     */
	return g_rows - used;
}

/* Minimum terminal height for the library to show at least 4 rows. */
static int library_min_rows(void)
{
	int deck_rows = 9;
	int panel_h = WFM_ROWS + 3;
	return deck_rows + panel_h * 2 + 1 + 1 + 1 + 4;
}

static void draw_split_view(void)
{
	/* ── Top: deck strip ── */
	int deck_rows = 9;
	int meter_w = 2;
	int usable = g_cols - meter_w;
	int dw = usable / 2;
	int vis[2] = { g_side_deck[0], g_side_deck[1] };

	draw_deck(g_win_main, 0, 0, dw, vis[0]);
	draw_vu_meter(g_win_main, 0, dw, deck_rows);
	draw_deck(g_win_main, 0, dw + meter_w, dw, vis[1]);

	/* ── Top: waveform panels ── */
	int panel_h = WFM_ROWS + 3;
	int panel_y = deck_rows;
	for (int i = 0; i < 2; i++) {
		draw_scrolling_waveform(g_win_main, panel_y, 0, g_cols, vis[i]);
		panel_y += panel_h;
	}

	/* ── Crossfader ── */
	draw_crossfader(g_win_main, panel_y, 0, g_cols);
	panel_y++;

	/* ── Horizontal divider with Crate Jump indicator ── */
	int div_y = panel_y;
	wattron(g_win_main, A_DIM);
	for (int i = 0; i < g_cols; i++)
		mvwaddch(g_win_main, div_y, i, ACS_HLINE);
	
	if (g_crate_jump_active) {
		wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD | A_REVERSE);
		mvwprintw(g_win_main, div_y, 2, " CRATE JUMP: %s_ ", g_crate_input);
		wattroff(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD | A_REVERSE);
	} else {
		wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
		if (g_panel == 0) {
			static const char *sort_labels[] = { "NAME", "BPM\u25b2", "BPM\u25bc" };
			mvwprintw(g_win_main, div_y, 2, " BROWSER [%s] \u25BC  TAB=next panel ", sort_labels[g_fb_sort]);
		} else if (g_panel == 1) {
			mvwprintw(g_win_main, div_y, 2, " PLAYLIST \u25BC (%d)  TAB=next panel ", g_pl_count);
		} else {
			static const char *lsort_labels[] = { "NAME", "BPM\u25b2", "BPM\u25bc" };
			if (g_lib_scanning)
				mvwprintw(g_win_main, div_y, 2, " LIBRARY \u25BC (scanning\xe2\x80\xa6) ");
			else
				mvwprintw(g_win_main, div_y, 2, " LIBRARY [%s] \u25BC (%d) ", lsort_labels[g_lib_sort], g_lib_count);
		}
		wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	}
	wattroff(g_win_main, A_DIM);
	panel_y++;

	/* ── Bottom panel ── */
	int by = panel_y;
	int brows = g_rows - 1 - by;
	if (brows < 4) {
		/* Terminal too short — library is hidden. Show a warning in the
         * area just below the divider if there's even one row available. */
		if (brows >= 1) {
			int need = library_min_rows();
			wattron(g_win_main,
				COLOR_PAIR(COLOR_HOT) | A_BOLD | A_REVERSE);
			mvwprintw(
				g_win_main, by, 0,
				" ⚠  LIBRARY HIDDEN — terminal too short"
				" (%d rows, need %d). Resize or use fewer decks / smaller WFM_ROWS.",
				g_rows, need);
			wattroff(g_win_main,
				 COLOR_PAIR(COLOR_HOT) | A_BOLD | A_REVERSE);
		}
		return;
	}

	if (g_panel == 0) {
		/* ── Browser panel ── */
		wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
		int plen = (int)strlen(g_fb_path);
		int pmax = g_cols - 4;
		const char *pshow =
			(plen > pmax) ? g_fb_path + plen - pmax : g_fb_path;
		mvwprintw(g_win_main, by, 0, " \u25B6 %-*.*s", pmax, pmax,
			  pshow);
		wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);

		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, by + 1, 0, " %-4s %6s  %-*s", "TYPE",
			  "BPM", g_cols - 14, "NAME");
		wattroff(g_win_main, A_BOLD);

		int list_y = by + 2;
		int list_rows = brows - 3;
		if (list_rows < 1)
			list_rows = 1;
		fb_fix_scroll(list_rows);

		for (int row = 0; row < list_rows; row++) {
			int idx = g_fb_scroll + row;
			int sy = list_y + row;
			if (idx >= g_fb_count) {
				mvwprintw(g_win_main, sy, 0, "%*s", g_cols, "");
				continue;
			}
			FBEntry *e = &g_fb_entries[idx];
			int sel = (idx == g_fb_sel);
			char dbuf[256];
			const char *dname =
				fb_display_name(e, dbuf, sizeof(dbuf));
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			if (e->is_dir) {
				if (!sel)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_HEADER));
				mvwprintw(g_win_main, sy, 0,
					  " DIR  %6s  %-*.*s", "", g_cols - 14,
					  g_cols - 14, e->name);
				if (!sel)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_HEADER));
			} else {
				char ext[8] = "???";
				const char *dot = strrchr(e->name, '.');
				if (dot) {
					strncpy(ext, dot + 1, 7);
					ext[7] = '\0';
					for (int i = 0; ext[i]; i++)
						ext[i] = (char)toupper(
							(unsigned char)ext[i]);
				}
				if (e->bpm > 0.0f) {
					if (!sel)
						wattron(g_win_main, A_DIM);
					mvwprintw(g_win_main, sy, 0,
						  " %-4s %6.1f  ", ext, e->bpm);
					if (!sel)
						wattroff(g_win_main, A_DIM);
					mvwprintw(g_win_main, sy, 14, "%-*.*s",
						  g_cols - 14, g_cols - 14,
						  dname);
				} else {
					mvwprintw(g_win_main, sy, 0,
						  " %-4s %6s  %-*.*s", ext,
						  "---", g_cols - 14,
						  g_cols - 14, dname);
				}
			}
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		}

		int fy = by + brows - 1;
		wattron(g_win_main, A_DIM);
		mvwprintw(
			g_win_main, fy, 0,
			" ENTER=load  !=A @=B #=C $=D  j/k=nav  BKSP=up  p=+playlist  P=playlist  i=tag");
		for (int i = 72; i < g_cols; i++)
			mvwaddch(g_win_main, fy, i, ' ');
		wattroff(g_win_main, A_DIM);
		if (g_fb_status[0]) {
			wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
			mvwprintw(g_win_main, fy,
				  g_cols - (int)strlen(g_fb_status) - 2, " %s ",
				  g_fb_status);
			wattroff(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
		}
		if (g_fb_count > list_rows)
			mvwprintw(g_win_main, list_y, g_cols - 10, "[%3d/%3d]",
				  g_fb_sel + 1, g_fb_count);

	} else if (g_panel == 1) {
		/* ── Playlist panel ── */
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, by, 0, " %-3s %6s  %-*s", "#", "BPM",
			  g_cols - 13, "NAME");
		wattroff(g_win_main, A_BOLD);

		int list_y = by + 1;
		int list_rows = brows - 2;
		if (list_rows < 1)
			list_rows = 1;
		pl_fix_scroll(list_rows);

		for (int row = 0; row < list_rows; row++) {
			int idx = g_pl_scroll + row;
			int sy = list_y + row;
			if (idx >= g_pl_count) {
				mvwprintw(g_win_main, sy, 0, "%*s", g_cols, "");
				continue;
			}
			PLEntry *e = &g_pl[idx];
			int selected = (idx == g_pl_sel);
			if (selected)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			if (e->bpm > 0.0f) {
				if (!selected)
					wattron(g_win_main, A_DIM);
				mvwprintw(g_win_main, sy, 0, " %-3d %6.1f  ",
					  idx + 1, e->bpm);
				if (!selected)
					wattroff(g_win_main, A_DIM);
				mvwprintw(g_win_main, sy, 13, "%-*.*s",
					  g_cols - 13, g_cols - 13, e->name);
			} else {
				mvwprintw(g_win_main, sy, 0,
					  " %-3d %6s  %-*.*s", idx + 1, "---",
					  g_cols - 13, g_cols - 13, e->name);
			}
			if (selected)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		}

		int fy = by + brows - 1;
		wattron(g_win_main, A_DIM);
		mvwprintw(
			g_win_main, fy, 0,
			" ENTER=load  !=A @=B #=C $=D  j/k=nav  DEL=remove  C-x=clear  P=browser");
		for (int i = 70; i < g_cols; i++)
			mvwaddch(g_win_main, fy, i, ' ');
		wattroff(g_win_main, A_DIM);
		if (g_pl_count > list_rows)
			mvwprintw(g_win_main, list_y, g_cols - 10, "[%3d/%3d]",
				  g_pl_sel + 1, g_pl_count);

	} else {
		/* ── Library panel (g_panel == 2) ── */
		if (!g_lib || (g_lib_count == 0 && !g_lib_scanning)) {
			wattron(g_win_main, A_DIM);
			mvwprintw(
				g_win_main, by, 0,
				" No library loaded. Press L to scan a directory.");
			mvwprintw(
				g_win_main, by + 1, 0,
				" (Scans all subfolders for .mp3 .wav .flac)");
			wattroff(g_win_main, A_DIM);
		} else {
			/* column header */
			wattron(g_win_main, A_BOLD);
			mvwprintw(g_win_main, by, 0, " %6s  %-*s", "BPM",
				  g_cols - 10, "NAME / ARTIST — TITLE");
			wattroff(g_win_main, A_BOLD);

			int list_y = by + 1;
			int list_rows = brows - 2;
			if (list_rows < 1)
				list_rows = 1;

			/* scroll clamp */
			if (g_lib_sel < 0)
				g_lib_sel = 0;
			if (g_lib_count > 0 && g_lib_sel >= g_lib_count)
				g_lib_sel = g_lib_count - 1;
			if (g_lib_scroll > g_lib_sel)
				g_lib_scroll = g_lib_sel;
			if (g_lib_scroll < g_lib_sel - list_rows + 1)
				g_lib_scroll = g_lib_sel - list_rows + 1;
			if (g_lib_scroll < 0)
				g_lib_scroll = 0;

			for (int row = 0; row < list_rows; row++) {
				int idx = g_lib_scroll + row;
				int sy = list_y + row;
				if (!g_lib || idx >= g_lib_count) {
					mvwprintw(g_win_main, sy, 0, "%*s",
						  g_cols, "");
					continue;
				}
				LIBEntry *e = &g_lib[idx];
				int sel = (idx == g_lib_sel);
				if (sel)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_ACTIVE) |
							A_BOLD);

				/* Build display name: "artist — title" or fallback to basename */
				char disp
					[264]; /* 128 artist + " — " (5 bytes) + 128 title + NUL */
				if (e->tag_artist[0] && e->tag_title[0])
					snprintf(disp, sizeof(disp),
						 "%s \xe2\x80\x94 %s",
						 e->tag_artist, e->tag_title);
				else if (e->tag_title[0])
					snprintf(disp, sizeof(disp), "%s",
						 e->tag_title);
				else
					snprintf(disp, sizeof(disp), "%s",
						 e->name);

				if (e->bpm > 0.0f) {
					if (!sel)
						wattron(g_win_main, A_DIM);
					mvwprintw(g_win_main, sy, 0, " %6.1f  ",
						  e->bpm);
					if (!sel)
						wattroff(g_win_main, A_DIM);
					mvwprintw(g_win_main, sy, 9, "%-*.*s",
						  g_cols - 9, g_cols - 9, disp);
				} else {
					mvwprintw(g_win_main, sy, 0,
						  " %6s  %-*.*s", "---",
						  g_cols - 9, g_cols - 9, disp);
				}
				if (sel)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_ACTIVE) |
							 A_BOLD);
			}

			int fy = by + brows - 1;
			wattron(g_win_main, A_DIM);
			if (g_lib_scanning)
				mvwprintw(
					g_win_main, fy, 0,
					" Scanning\xe2\x80\xa6 %d found  |  L=rescan  O=sort  P=browser",
					g_lib_count);
			else
				mvwprintw(
					g_win_main, fy, 0,
					" ENTER=load  !=A @=B #=C $=D  j/k=nav  p=+playlist  O=sort  L=rescan  P=browser");
			for (int i = 72; i < g_cols; i++)
				mvwaddch(g_win_main, fy, i, ' ');
			wattroff(g_win_main, A_DIM);

			if (g_lib_count > list_rows)
				mvwprintw(g_win_main, list_y, g_cols - 10,
					  "[%3d/%3d]", g_lib_sel + 1,
					  g_lib_count);
		}
	}
}

static void draw_browser_view(void)
{
	/* How many rows are available for the file list */
	int header_rows = 3; /* path + column header + divider */
	int footer_rows = 1; /* load-hint line */
	int list_rows = g_rows - 1 - header_rows - footer_rows;
	if (list_rows < 1)
		list_rows = 1;

	fb_fix_scroll(list_rows);

	/* ── Header: current path ── */
	wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	mvwprintw(g_win_main, 0, 0, " \u25B6 %.*s", g_cols - 4, g_fb_path);
	/* pad to full width */
	int plen = (int)strlen(g_fb_path) + 3;
	for (int i = plen; i < g_cols; i++)
		mvwaddch(g_win_main, 0, i, ' ');
	wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);

	/* ── Column header ── */
	wattron(g_win_main, A_BOLD);
	mvwprintw(g_win_main, 1, 0, " %-4s %6s  %-*s", "TYPE", "BPM",
		  g_cols - 14, "NAME");
	wattroff(g_win_main, A_BOLD);

	/* ── Divider ── */
	wattron(g_win_main, A_DIM);
	for (int i = 0; i < g_cols; i++)
		mvwaddch(g_win_main, 2, i, ACS_HLINE);
	wattroff(g_win_main, A_DIM);

	/* ── File list ── */
	for (int row = 0; row < list_rows; row++) {
		int idx = g_fb_scroll + row;
		int y = header_rows + row;

		if (idx >= g_fb_count) {
			mvwprintw(g_win_main, y, 0, "%*s", g_cols, "");
			continue;
		}

		FBEntry *e = &g_fb_entries[idx];
		int selected = (idx == g_fb_sel);

		if (selected)
			wattron(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);

		if (e->is_dir) {
			wattron(g_win_main,
				selected ? 0 : COLOR_PAIR(COLOR_HEADER));
			mvwprintw(g_win_main, y, 0, " DIR  %6s  %-*.*s", "",
				  g_cols - 14, g_cols - 14, e->name);
			if (!selected)
				wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER));
		} else {
			char ext[8];
			const char *dot = strrchr(e->name, '.');
			if (dot) {
				strncpy(ext, dot + 1, 7);
				ext[7] = '\0';
				for (int i = 0; ext[i]; i++)
					ext[i] = (char)toupper(
						(unsigned char)ext[i]);
			} else {
				strcpy(ext, "???");
			}

			char dbuf[256];
			const char *dname =
				fb_display_name(e, dbuf, sizeof(dbuf));

			if (e->bpm > 0.0f) {
				if (!selected)
					wattron(g_win_main, A_DIM);
				mvwprintw(g_win_main, y, 0, " %-4s %6.1f  ",
					  ext, e->bpm);
				if (!selected)
					wattroff(g_win_main, A_DIM);
				int name_col = 14;
				int name_w = g_cols - name_col;
				mvwprintw(g_win_main, y, name_col, "%-*.*s",
					  name_w, name_w, dname);
			} else {
				mvwprintw(g_win_main, y, 0, " %-4s %6s  %-*.*s",
					  ext, "---", g_cols - 14, g_cols - 14,
					  dname);
			}
		}

		if (selected)
			wattroff(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
	}

	/* ── Footer: load hint + status message ── */
	int fy = g_rows - 2;
	wattron(g_win_main, A_DIM);
	mvwprintw(
		g_win_main, fy, 0,
		" ENTER=load  !=A @=B #=C $=D  j/k=nav  BKSP=up  p=+playlist  i=tag  P=playlist");
	/* pad */
	for (int i = 56; i < g_cols; i++)
		mvwaddch(g_win_main, fy, i, ' ');
	wattroff(g_win_main, A_DIM);

	/* Status message (e.g. "Loaded → Deck A") */
	if (g_fb_status[0]) {
		wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
		mvwprintw(g_win_main, fy, g_cols - (int)strlen(g_fb_status) - 2,
			  " %s ", g_fb_status);
		wattroff(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
	}

	/* Scrollbar indicator */
	if (g_fb_count > list_rows) {
		mvwprintw(g_win_main, header_rows, g_cols - 8, "[%3d/%3d]",
			  g_fb_sel + 1, g_fb_count);
	}
}


/* ── Playlist view ────────────────────────────────────────────────────── */
/* ── Tag info overlay panel (drawn over any view) ───────────────────── */
static void draw_tag_panel(void)
{
	if (!g_tag_info.visible)
		return;

	int pw = 60;
	int ph = 10;
	int px = (g_cols - pw) / 2;
	int py = (g_rows - 1 - ph) / 2;
	if (px < 0)
		px = 0;
	if (py < 0)
		py = 0;
	if (pw > g_cols)
		pw = g_cols;

	/* Box */
	wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	for (int r = py; r < py + ph && r < g_rows - 1; r++) {
		for (int c = px; c < px + pw && c < g_cols; c++) {
			if (r == py || r == py + ph - 1)
				mvwaddch(
					g_win_main, r, c,
					(r == py && c == px) ? ACS_ULCORNER :
					(r == py && c == px + pw - 1) ?
							       ACS_URCORNER :
					(r == py + ph - 1 && c == px) ?
							       ACS_LLCORNER :
					(r == py + ph - 1 && c == px + pw - 1) ?
							       ACS_LRCORNER :
							       ACS_HLINE);
			else if (c == px || c == px + pw - 1)
				mvwaddch(g_win_main, r, c, ACS_VLINE);
			else
				mvwaddch(g_win_main, r, c, ' ');
		}
	}
	/* Title bar */
	mvwprintw(g_win_main, py, px + 2, " MusicBrainz Tag Lookup ");
	wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);

	int row = py + 1;
	int iw = pw - 4; /* inner width */

	/* Searching / error status */
	if (g_tag_info.status[0]) {
		wattron(g_win_main, A_DIM);
		mvwprintw(g_win_main, row++, px + 2, "  %-*.*s", iw, iw,
			  g_tag_info.status);
		mvwprintw(g_win_main, row++, px + 2, "  File: %-*.*s", iw - 6,
			  iw - 6, g_tag_info.query_name);
		wattroff(g_win_main, A_DIM);
	} else {
		/* Results */
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, row++, px + 2, "  Title:  %-*.*s", iw - 9,
			  iw - 9, g_tag_info.title);
		wattroff(g_win_main, A_BOLD);
		mvwprintw(g_win_main, row++, px + 2, "  Artist: %-*.*s", iw - 9,
			  iw - 9, g_tag_info.artist);
		mvwprintw(g_win_main, row++, px + 2, "  Album:  %-*.*s", iw - 9,
			  iw - 9, g_tag_info.album);
		mvwprintw(g_win_main, row++, px + 2, "  Date:   %-*.*s", iw - 9,
			  iw - 9, g_tag_info.date);
		mvwprintw(g_win_main, row++, px + 2, "  Label:  %-*.*s", iw - 9,
			  iw - 9, g_tag_info.label);
		mvwprintw(g_win_main, row++, px + 2, "  File:   %-*.*s", iw - 9,
			  iw - 9, g_tag_info.query_name);
	}

	wattron(g_win_main, A_DIM);
	mvwprintw(g_win_main, py + ph - 2, px + 2, "  Press any key to close");
	wattroff(g_win_main, A_DIM);
}

/* ── Options overlay (ESC) ───────────────────────────────────────────── */
/* Apply the current g_opts.ui_fps to the ncurses key-wait timeout.
 * Must be called whenever ui_fps changes, and once after ncurses init.
 * g_win_main must already exist. */
static void apply_ui_fps(void)
{
	int ms = 1000 /
		 g_opts.ui_fps; /* 5 FPS→200ms, 20 FPS→50ms, 60 FPS→16ms */
	wtimeout(g_win_main, ms);
}

/* Reads /proc/cpuinfo and ALSA info for the Info tab.
 * Audio tab: adjustable default vol, gain, normalization.
 * Display tab: waveform gamma, kick threshold, visible seconds. */

static void options_read_cpuinfo(char *out, int max)
{
	FILE *f = fopen("/proc/cpuinfo", "r");
	if (!f) {
		snprintf(out, max, "unavailable");
		return;
	}
	char line[256];
	out[0] = '\0';
	while (fgets(line, sizeof(line), f)) {
		if (strncmp(line, "cpu\t\t:", 6) == 0 ||
		    strncmp(line, "model name", 10) == 0 ||
		    strncmp(line, "cpu MHz", 7) == 0 ||
		    strncmp(line, "BogoMIPS", 8) == 0) {
			char *colon = strchr(line, ':');
			if (colon) {
				/* Strip leading spaces and trailing newline */
				char *val = colon + 1;
				while (*val == ' ' || *val == '\t')
					val++;
				int vlen = (int)strlen(val);
				while (vlen > 0 && (val[vlen - 1] == '\n' ||
						    val[vlen - 1] == '\r'))
					val[--vlen] = '\0';
				int cur = (int)strlen(out);
				if (cur > 0 && cur < max - 2) {
					out[cur++] = ' ';
					out[cur] = '\0';
				}
				strncat(out, val, max - strlen(out) - 1);
			}
		}
	}
	fclose(f);
	if (out[0] == '\0')
		snprintf(out, max, "unknown");
}

static void options_read_meminfo(char *out, int max)
{
	FILE *f = fopen("/proc/meminfo", "r");
	if (!f) {
		snprintf(out, max, "unavailable");
		return;
	}
	char line[128];
	long total = 0, avail = 0;
	while (fgets(line, sizeof(line), f)) {
		if (strncmp(line, "MemTotal:", 9) == 0)
			sscanf(line + 9, "%ld", &total);
		if (strncmp(line, "MemAvailable:", 13) == 0)
			sscanf(line + 13, "%ld", &avail);
	}
	fclose(f);
	snprintf(out, max, "%ld MB total, %ld MB free", total / 1024,
		 avail / 1024);
}

static void draw_options_overlay(void)
{
	if (!g_options_open)
		return;

	/* Floating panel — background stays visible behind it (btop style).
     * Fixed width, centred, tall enough for content. */
	int ow = (g_cols > 78) ? 78 : g_cols;
	int oh = g_rows - 4;
	if (oh < 10)
		oh = 10;
	int ox = (g_cols - ow) / 2;
	int oy = 2;

	/* Fill panel interior with STATUS colour so content is readable */
	wattron(g_win_main, COLOR_PAIR(COLOR_STATUS));
	for (int r = oy; r < oy + oh; r++) {
		wmove(g_win_main, r, ox);
		for (int c = 0; c < ow; c++)
			waddch(g_win_main, ' ');
	}
	wattroff(g_win_main, COLOR_PAIR(COLOR_STATUS));

	/* ── Top border with tabs embedded in it (btop style) ──
     * Draw the full top border first, then overwrite with tab labels.
     * Active tab: bold + COLOR_ACTIVE, surrounded by ┤ ├.
     * Inactive tabs: dim, plain text. */
	const char *tabs[] = { " INFO ", " AUDIO ", " DISP ",
			       " WAVE ", " SYNC ",  " THEME ",
			       " MIDI ", " OUT ",   " FX " };
	const int ntabs = 9;

	/* Drop shadow (1 cell right + 1 cell down) so panel floats above content */
	wattron(g_win_main, A_DIM);
	for (int r = oy + 1; r < oy + oh + 1 && r < g_rows - 1; r++)
		mvwaddch(g_win_main, r, ox + ow, ' ');
	for (int c = ox + 1; c < ox + ow + 1 && c < g_cols; c++)
		mvwaddch(g_win_main, oy + oh, c, ' ');
	wattroff(g_win_main, A_DIM);

	/* Draw plain top border */
	wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	for (int c = ox; c < ox + ow; c++) {
		mvwaddch(g_win_main, oy, c,
			 (c == ox)	    ? ACS_ULCORNER :
			 (c == ox + ow - 1) ? ACS_URCORNER :
					      ACS_HLINE);
	}
	/* Title on left side of border */
	mvwprintw(g_win_main, oy, ox + 2, "\u2524 djcmd OPTIONS \u251c");
	wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);

	/* Lay tabs into the border starting after the title */
	int tx = ox + 19; /* start right after title */
	for (int i = 0; i < ntabs; i++) {
		int tlen = (int)strlen(tabs[i]);
		if (i == g_options_tab) {
			/* Active tab: ┤ label ├ in COLOR_ACTIVE — creates a "notch" */
			wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
			mvwaddch(g_win_main, oy, tx, ACS_RTEE);
			wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
			wattron(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			for (int j = 0; j < tlen; j++)
				mvwaddch(g_win_main, oy, tx + 1 + j,
					 tabs[i][j]);
			wattroff(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
			mvwaddch(g_win_main, oy, tx + 1 + tlen, ACS_LTEE);
			wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
			tx += tlen + 3;
		} else {
			/* Inactive tab: plain dim label on the border line */
			wattron(g_win_main, A_DIM);
			for (int j = 0; j < tlen; j++)
				mvwaddch(g_win_main, oy, tx + j, tabs[i][j]);
			wattroff(g_win_main, A_DIM);
			tx += tlen; /* no gap between inactive tabs saves space */
		}
		if (tx >= ox + ow - 2)
			break;
	}

	/* Side borders + bottom border */
	wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	for (int r = oy + 1; r < oy + oh - 1; r++) {
		mvwaddch(g_win_main, r, ox, ACS_VLINE);
		mvwaddch(g_win_main, r, ox + ow - 1, ACS_VLINE);
	}
	for (int c = ox; c < ox + ow; c++) {
		mvwaddch(g_win_main, oy + oh - 1, c,
			 (c == ox)	    ? ACS_LLCORNER :
			 (c == ox + ow - 1) ? ACS_LRCORNER :
					      ACS_HLINE);
	}
	/* Footer hint in bottom border */
	mvwprintw(
		g_win_main, oy + oh - 1, ox + 2,
		"\u2524 ESC=close  \u25c4\u25ba=tabs  j/k=select  -/+=adjust  ENTER=apply \u251c");
	wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);

	int cy = oy + 2; /* content start row (below top border) */
	int iw = ow - 4; /* inner width */

	if (g_options_tab == 0) {
		/* ── INFO ── */
		/* CPU string was read once at startup into g_cpuinfo_cache.
         * Memory string is refreshed at most once per second. */
		time_t now = time(NULL);
		if (now != g_meminfo_last_t) {
			options_read_meminfo(g_meminfo_cache,
					     sizeof(g_meminfo_cache));
			g_meminfo_last_t = now;
		}
		const char *cpu = g_cpuinfo_cache;
		const char *mem = g_meminfo_cache;

		/* ALSA device */
		char alsa_dev[64];
		snprintf(alsa_dev, sizeof(alsa_dev), "%s", g_pcm_dev_str);
		char alsa_sr[48];
		snprintf(alsa_sr, sizeof(alsa_sr),
			 "%d Hz, %d ch, %d frames/period", g_actual_sample_rate,
			 CHANNELS, PERIOD_FRAMES);

		/* MIDI */
		char midi_info[64];
		if (g_midi_in) {
			snd_rawmidi_info_t *info;
			snd_rawmidi_info_alloca(&info);
			if (snd_rawmidi_info(g_midi_in, info) == 0)
				snprintf(midi_info, sizeof(midi_info), "%s",
					 snd_rawmidi_info_get_name(info));
			else
				snprintf(midi_info, sizeof(midi_info),
					 "connected");
		} else {
			snprintf(midi_info, sizeof(midi_info), "not connected");
		}

		/* Loaded tracks memory */
		long pcm_kb = 0;
		for (int i = 0; i < MAX_TRACKS; i++)
			if (g_tracks[i].loaded)
				pcm_kb +=
					(long)g_tracks[i].num_frames * 4 / 1024;

		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " HARDWARE");
		wattroff(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, "  CPU      : %-*.*s",
			  iw - 12, iw - 12, cpu);
		mvwprintw(g_win_main, cy++, ox + 2, "  Memory   : %-*.*s",
			  iw - 12, iw - 12, mem);
		mvwprintw(g_win_main, cy++, ox + 2, "  ALSA dev : %s",
			  alsa_dev);
		mvwprintw(g_win_main, cy++, ox + 2, "  ALSA cfg : %s", alsa_sr);
		mvwprintw(g_win_main, cy++, ox + 2, "  MIDI     : %-*.*s",
			  iw - 12, iw - 12, midi_info);
		cy++;
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " RUNTIME");
		wattroff(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, "  Decks    : %d active",
			  g_num_tracks);
		mvwprintw(g_win_main, cy++, ox + 2,
			  "  PCM mem  : %ld KB loaded", pcm_kb);
		mvwprintw(g_win_main, cy++, ox + 2, "  256color : %s",
			  g_has_256 ? "yes" : "no (8-color mode)");
		mvwprintw(g_win_main, cy++, ox + 2, "  TTY mode : %s",
			  g_is_tty ? "yes (block chars)" : "no (Unicode)");
		mvwprintw(g_win_main, cy++, ox + 2,
			  "  Settings : ~/" CFG_CONFIG_DIR "/settings");
		/* Show per-device map filename */
		{
			char map_path[512];
			midi_map_path(map_path, sizeof(map_path));
			const char *slash = strrchr(map_path, '/');
			mvwprintw(g_win_main, cy++, ox + 2,
				  "  MIDI map : ~/" CFG_CONFIG_DIR "/%s",
				  slash ? slash + 1 : map_path);
		}
		mvwprintw(g_win_main, cy++, ox + 2,
			  "  Library  : ~/" CFG_CONFIG_DIR
			  "/" CFG_LIBRARY_FILE);

	} else if (g_options_tab == 1) {
		/* ── AUDIO ── */

		/* ── PCM output device picker ───────────────────────────────────────
         * j/k navigate, ENTER switches, R rescans */
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " AUDIO OUTPUT DEVICE");
		wattroff(g_win_main, A_BOLD);

		if (g_pcm_ndevices == 0) {
			wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
			mvwprintw(
				g_win_main, cy++, ox + 4,
				"  No PCM devices found — press R to rescan.");
			wattroff(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
		} else {
			if (g_pcm_dev_sel < 0)
				g_pcm_dev_sel = 0;
			if (g_pcm_dev_sel >= g_pcm_ndevices)
				g_pcm_dev_sel = g_pcm_ndevices - 1;
			for (int di = 0;
			     di < g_pcm_ndevices && cy < oy + oh - 16; di++) {
				int is_active = (strcmp(g_pcm_devlist[di].dev,
							g_pcm_dev_str) == 0);
				int is_sel = (di == g_pcm_dev_sel);
				if (is_sel)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_ACTIVE) |
							A_BOLD);
				else if (is_active)
					wattron(g_win_main, A_BOLD);
				mvwprintw(g_win_main, cy++, ox + 4,
					  "  %s %-*.*s  [%s]",
					  is_active ? "\u25CF" : "\u25CB",
					  ow > 20 ? ow - 28 : 20,
					  ow > 20 ? ow - 28 : 20,
					  g_pcm_devlist[di].name,
					  g_pcm_devlist[di].dev);
				if (is_sel)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_ACTIVE) |
							 A_BOLD);
				else if (is_active)
					wattroff(g_win_main, A_BOLD);
			}
			wattron(g_win_main, A_DIM);
			mvwprintw(
				g_win_main, cy++, ox + 4,
				"  j/k = select   ENTER = switch to selected   R = rescan");
			wattroff(g_win_main, A_DIM);
		}
		cy++;

		/* Editable rows — highlight selected; ESC+arrow to adjust */
		const char *labels[] = {
			"Default master vol",
			"Default deck vol  ",
			"Auto-gain default ",
			"Auto-gain target  ",
		};
		float vals[] = {
			(float)g_opts.default_master_vol,
			g_opts.default_deck_vol * 100.0f,
			(float)g_opts.auto_gain_default,
			g_opts.auto_gain_target_db,
		};
		const char *units[] = { "%", "%", "(0=off 1=on)", "dBFS" };

		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " VOLUME & NORMALISATION");
		wattroff(g_win_main, A_BOLD);
		cy++;

		for (int i = 0; i < 4; i++) {
			int sel = (g_options_sel == i);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			if (i == 2)
				mvwprintw(g_win_main, cy, ox + 4, "  %s : %s",
					  labels[i],
					  vals[i] > 0.5f ? "ON" : "OFF");
			else
				mvwprintw(g_win_main, cy, ox + 4,
					  "  %s : %5.1f %s", labels[i], vals[i],
					  units[i]);
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			cy++;
		}
		cy++;
		wattron(g_win_main, A_DIM);
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  j/k = select row   LEFT/RIGHT = adjust value");
		wattroff(g_win_main, A_DIM);

	} else if (g_options_tab == 2) {
		/* ── DISPLAY ── */
		const char *dlabels[] = {
			"Waveform gamma    ",
			"Visible seconds   ",
			"Waveform style    ",
		};
		const char *dunits[] = {
			"(0.5=sqrt  1.0=linear)",
			"secs",
			NULL, /* style uses a separate display */
		};
		const char *wfm_style_names[] = { "Kick/Snare", "Theme" };

		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " WAVEFORM DISPLAY");
		wattroff(g_win_main, A_BOLD);
		cy++;

		/* Row 0: gamma */
		{
			int sel = (g_options_sel == 0);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(g_win_main, cy, ox + 4, "  %s : %5.2f  %s",
				  dlabels[0], (double)g_opts.wfm_height_gamma,
				  dunits[0]);
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			cy++;
		}
		/* Row 1: visible secs */
		{
			int sel = (g_options_sel == 1);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(g_win_main, cy, ox + 4, "  %s : %5.1f  %s",
				  dlabels[1], (double)g_opts.wfm_visible_secs,
				  dunits[1]);
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			cy++;
		}
		/* Row 2: waveform style — cycle with left/right */
		{
			int sel = (g_options_sel == 2);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(g_win_main, cy, ox + 4, "  %s :  < %s >",
				  dlabels[2],
				  wfm_style_names[g_opts.wfm_style]);
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			cy++;
		}
		/* Row 3: UI redraw rate */
		{
			int sel = (g_options_sel == 3);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(
				g_win_main, cy, ox + 4,
				"  UI redraw rate    : %3d FPS  (%d ms/frame)",
				g_opts.ui_fps, 1000 / g_opts.ui_fps);
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			cy++;
		}
		cy++;
		wattron(g_win_main, A_DIM);
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  Kick/Snare: red=kick  blue=snare");
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  Theme:      amplitude bars in theme colours");
		cy++;
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  j/k = select   LEFT/RIGHT = adjust");
		wattroff(g_win_main, A_DIM);

	} else if (g_options_tab == 3) {
		/* ── WAVEFORM (advanced) ── */
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " ADVANCED WAVEFORM");
		wattroff(g_win_main, A_BOLD);
		cy++;

		/* Row 0: Lo band height weight */
		{
			int sel = (g_options_sel == 0);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(
				g_win_main, cy, ox + 4,
				"  Bass  height weight : %4.2f  (kick/sub — default 0.60)",
				(double)g_opts.wfm_lo_weight);
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			cy++;
		}
		/* Row 1: Mid band height weight */
		{
			int sel = (g_options_sel == 1);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(
				g_win_main, cy, ox + 4,
				"  Snare height weight : %4.2f  (snare/perc — default 0.40)",
				(double)g_opts.wfm_mid_weight);
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			cy++;
		}
		/* Row 2: Hi band height weight */
		{
			int sel = (g_options_sel == 2);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(
				g_win_main, cy, ox + 4,
				"  Treble height weight: %4.2f  (hi-hat/air — default 0.15)",
				(double)g_opts.wfm_hi_weight);
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			cy++;
		}
		/* Row 3: Colour saturation */
		{
			int sel = (g_options_sel == 3);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(
				g_win_main, cy, ox + 4,
				"  Colour saturation   : %4.2f  (0.2=grey  1.0=normal  3.0=vivid)",
				(double)g_opts.wfm_color_sat);
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			cy++;
		}
		/* Row 4: Colour floor */
		{
			int sel = (g_options_sel == 4);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(
				g_win_main, cy, ox + 4,
				"  Colour floor        : %5.3f (0.00=black gaps  0.06=default  0.15=always lit)",
				(double)g_opts.wfm_color_floor);
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			cy++;
		}
		/* Row 5: Anchor mode */
		{
			int sel = (g_options_sel == 5);
			const char *anchor_names[] = { "Centred (Serato)",
						       "Bottom (Rekordbox)" };
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(g_win_main, cy, ox + 4,
				  "  Bar anchor          :  < %s >",
				  anchor_names[g_opts.wfm_anchor]);
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			cy++;
		}
		cy++;
		wattron(g_win_main, A_DIM);
		mvwprintw(
			g_win_main, cy++, ox + 4,
			"  Band weights: raise bass to fix solid-colour dense music.");
		mvwprintw(
			g_win_main, cy++, ox + 4,
			"  Sat<1 = greyer; Sat>1 = more vivid kick/snare contrast.");
		mvwprintw(
			g_win_main, cy++, ox + 4,
			"  Floor=0.00 lets silent gaps go black between transients.");
		cy++;
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  j/k = select   LEFT/RIGHT = adjust");
		wattroff(g_win_main, A_DIM);

	} else if (g_options_tab == 4) {
		/* ── SYNC ── */
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " SYNC OPTIONS");
		wattroff(g_win_main, A_BOLD);
		cy++;

		/* Row 0: Quantize sync */
		{
			int sel = (g_options_sel == 0);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(g_win_main, cy, ox + 4,
				  "  Quantize play       :  %s",
				  g_opts.sync_quantize ? "ON " : "OFF");
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			cy++;
		}
		/* Row 1: Smart range */
		{
			int sel = (g_options_sel == 1);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(g_win_main, cy, ox + 4,
				  "  Smart BPM range     :  %s",
				  g_opts.sync_smart_range ? "ON " : "OFF");
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			cy++;
		}
		/* Row 2: Auto master handoff */
		{
			int sel = (g_options_sel == 2);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(g_win_main, cy, ox + 4,
				  "  Auto master handoff :  %s",
				  g_opts.sync_auto_handoff ? "ON " : "OFF");
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			cy++;
		}
		/* Row 3: Key lock default */
		{
			int sel = (g_options_sel == 3);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(g_win_main, cy, ox + 4,
				  "  Key lock default    :  %s",
				  g_opts.key_lock_default ? "ON " : "OFF");
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			cy++;
		}
		/* Row 4: Vinyl mode (motorised platters only) */
		{
			int sel = (g_options_sel == 4);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(g_win_main, cy, ox + 4,
				  "  Vinyl mode          :  %s",
				  g_opts.vinyl_mode ? "ON " : "OFF");
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			cy++;
		}
		cy++;
		wattron(g_win_main, A_DIM);
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  Quantize: waits for bar-1 of master before");
		mvwprintw(
			g_win_main, cy++, ox + 4,
			"    starting a sync-locked deck. (\xe2\x8f\xb3 WAIT shown)");
		cy++;
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  Smart range: folds BPM by octaves before");
		mvwprintw(g_win_main, cy++, ox + 4,
			  "    sync to prevent 90"
			  "\xe2\x86\x92"
			  "180 jumps.");
		cy++;
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  Auto handoff: if the master deck is reloaded");
		mvwprintw(g_win_main, cy++, ox + 4,
			  "    while another deck is playing, that deck");
		mvwprintw(g_win_main, cy++, ox + 4,
			  "    becomes the new sync master automatically.");
		cy++;
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  Key lock (K): pitch-preserving time-stretch.");
		mvwprintw(g_win_main, cy++, ox + 4,
			  "    Default sets state for newly loaded tracks.");
		mvwprintw(g_win_main, cy++, ox + 4,
			  "    Toggle per-deck with K. Shows KEY in header.");
		cy++;
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  Vinyl mode: motorised platters only.");
		mvwprintw(g_win_main, cy++, ox + 4,
			  "    ON : playhead runs independently until you");
		mvwprintw(g_win_main, cy++, ox + 4,
			  "    touch the vinyl or platter rim (scratch).");
		mvwprintw(g_win_main, cy++, ox + 4,
			  "    OFF: platter velocity always drives audio");
		mvwprintw(g_win_main, cy++, ox + 4,
			  "    (DVS / timecode-vinyl style).");
		cy++;
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  j/k = select   LEFT/RIGHT = toggle");
		wattroff(g_win_main, A_DIM);

	} else if (g_options_tab == 5) {
		/* ── THEME ── */
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " SELECT THEME");
		wattroff(g_win_main, A_BOLD);
		cy++;

		for (int i = 0; i < THEME_COUNT; i++) {
			int sel = (g_options_sel == i);
			int active = (i == g_opts.theme_idx);

			/* Show active theme with a checkmark */
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			else if (active)
				wattron(g_win_main, A_BOLD);

			mvwprintw(g_win_main, cy, ox + 4, "  %s %s",
				  active ? "\u25CF" : "\u25CB", /* ● or ○ */
				  g_themes[i].name);

			/* Draw a mini colour preview swatch next to each theme name */
			if (cy < oy + oh - 3) {
				int sx = ox + 4 + 22;
				/* Draw swatch: header / active / hot colours */
				wattron(g_win_main,
					COLOR_PAIR(COLOR_HEADER) | A_BOLD);
				mvwprintw(g_win_main, cy, sx, "HDR");
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_HEADER) | A_BOLD);
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
				mvwprintw(g_win_main, cy, sx + 4, "ACT");
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
				wattron(g_win_main,
					COLOR_PAIR(COLOR_HOT) | A_BOLD);
				mvwprintw(g_win_main, cy, sx + 8, "HOT");
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_HOT) | A_BOLD);
				wattron(g_win_main, COLOR_PAIR(COLOR_VU));
				mvwprintw(g_win_main, cy, sx + 12, "VU ");
				wattroff(g_win_main, COLOR_PAIR(COLOR_VU));
			}

			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			else if (active)
				wattroff(g_win_main, A_BOLD);
			cy++;
		}
		cy++;
		wattron(g_win_main, A_DIM);
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  j/k = select   ENTER or RIGHT = apply theme");
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  Themes take effect immediately.");
		mvwprintw(
			g_win_main, cy++, ox + 4,
			"  Add custom themes in djcmd_config.h (see THEME_* macros).");
		wattroff(g_win_main, A_DIM);
	} else if (g_options_tab == 6) {
		/* ── MIDI ── */

		/* ── Device picker ──────────────────────────────────────────────────
         * Lists all ALSA rawmidi inputs.  j/k navigate, ENTER switches.
         * The active device is shown with ● and the map filename displayed. */
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " MIDI INPUT DEVICES");
		wattroff(g_win_main, A_BOLD);

		if (g_midi_ndevices == 0) {
			wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 4,
				  "  No MIDI input devices found.");
			wattroff(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
			wattron(g_win_main, A_DIM);
			mvwprintw(
				g_win_main, cy++, ox + 4,
				"  Connect a controller and press R to re-scan.");
			wattroff(g_win_main, A_DIM);
		} else {
			/* Clamp g_midi_dev_sel */
			if (g_midi_dev_sel < 0)
				g_midi_dev_sel = 0;
			if (g_midi_dev_sel >= g_midi_ndevices)
				g_midi_dev_sel = g_midi_ndevices - 1;

			for (int di = 0;
			     di < g_midi_ndevices && cy < oy + oh - 12; di++) {
				int is_active = (strcmp(g_midi_devlist[di].dev,
							g_midi_dev_str) == 0);
				int is_sel = (di == g_midi_dev_sel) &&
					     !g_midi_learn_active &&
					     !g_midi_learn_jog_pair;

				if (is_sel)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_ACTIVE) |
							A_BOLD);
				else if (is_active)
					wattron(g_win_main, A_BOLD);

				/* Build map filename for display */
				char map_san[64];
				midi_map_name_from_device(
					g_midi_devlist[di].name, map_san,
					sizeof(map_san));

				/* name_w: panel-relative, leaves room for "[hw:X,X,X]  map:XXXX.map" suffix */
				int name_w = (ow > 52) ? ow - 52 : 4;
				mvwprintw(g_win_main, cy++, ox + 4,
					  "  %s %-*.*s  [%.12s]  map:%.14s.map",
					  is_active ? "\u25CF" : "\u25CB",
					  name_w, name_w,
					  g_midi_devlist[di].name,
					  g_midi_devlist[di].dev, map_san);

				if (is_sel)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_ACTIVE) |
							 A_BOLD);
				else if (is_active)
					wattroff(g_win_main, A_BOLD);
			}
			cy++;
			wattron(g_win_main, A_DIM);
			mvwprintw(
				g_win_main, cy++, ox + 4,
				"  j/k = select device   ENTER = switch to selected   R = rescan");
			if (g_midi_in) {
				/* Show active map file path */
				char map_path[512];
				midi_map_path(map_path, sizeof(map_path));
				const char *slash = strrchr(map_path, '/');
				mvwprintw(g_win_main, cy++, ox + 4,
					  "  Active map: ~/" CFG_CONFIG_DIR
					  "/%s",
					  slash ? slash + 1 : map_path);
			}
			wattroff(g_win_main, A_DIM);
		}
		cy++;

		/* Live activity monitor — shows raw bytes and decoded type */
		if (g_midi_last_status) {
			uint8_t mtype = g_midi_last_status & 0xF0;
			uint8_t mch = g_midi_last_status & 0x0F;
			const char *ts = (mtype == 0x80) ? "NoteOff" :
					 (mtype == 0x90) ? "NoteOn " :
					 (mtype == 0xB0) ? "CC     " :
					 (mtype == 0xE0) ? "PitchBd" :
							   "Other  ";
			wattron(g_win_main, A_DIM);
			if (mtype == 0xE0) {
				int pb14 = ((int)g_midi_last_d2 << 7) |
					   (int)g_midi_last_d1;
				mvwprintw(
					g_win_main, cy++, ox + 2,
					" Last: %s ch%d  14bit:%-5d (%+d)  raw:%02X %02X %02X",
					ts, mch + 1, pb14, pb14 - 8192,
					g_midi_last_status, g_midi_last_d1,
					g_midi_last_d2);
			} else {
				mvwprintw(
					g_win_main, cy++, ox + 2,
					" Last: %s ch%d  d1:%-3d d2:%-3d  raw:%02X %02X %02X",
					ts, mch + 1, g_midi_last_d1,
					g_midi_last_d2, g_midi_last_status,
					g_midi_last_d1, g_midi_last_d2);
			}
			mvwprintw(
				g_win_main, cy++, ox + 2,
				" Left\u2192Deck %c (nudge_fwd/back)   Right\u2192Deck %c (nudge_fwd/back_b)",
				DECK_NUM(g_side_deck[0]),
				DECK_NUM(g_side_deck[1]));
			mvwprintw(
				g_win_main, cy++, ox + 2,
				" Pitch range: %s / %s / %s / %s",
				g_pitch_range_names[g_pitch_range[0]],
				g_pitch_range_names[g_pitch_range[1]],
				g_num_tracks > 2 ?
					g_pitch_range_names[g_pitch_range[2]] :
					"--",
				g_num_tracks > 3 ?
					g_pitch_range_names[g_pitch_range[3]] :
					"--");
			wattroff(g_win_main, A_DIM);
		}
		cy++;

		/* Learn mode banner */
		if (g_midi_learn_jog_pair) {
			int got_spin = (g_midi_learn_jog_spin_status != 0);
			int got_pb = (g_midi_learn_jog_step & 2);
			int got_touch = (g_midi_learn_jog_step & 1);
			int ns7mode = (g_jog_type == JOG_NS7III);
			wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 2,
				  " >> JOG LEARN DECK %c  [%s] <<",
				  DECK_NUM(g_midi_learn_jog_deck),
				  ns7mode ? "ns7iii mode" : "standard mode");
			wattroff(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
			wattron(g_win_main, A_DIM);
			if (ns7mode) {
				mvwprintw(
					g_win_main, cy++, ox + 4,
					"   CC Spin [%s]  PitchBend [%s]  Touch [n/a - NS7III has no touch sensor]",
					got_spin ? "CAPTURED" : "waiting..",
					got_pb ? "CAPTURED" : "waiting..");
			} else {
				mvwprintw(
					g_win_main, cy++, ox + 4,
					"   CC Spin [%s]  PitchBend [%s]  Touch [%s]",
					got_spin ? "CAPTURED" : "waiting..",
					got_pb ? "CAPTURED" : "waiting..",
					got_touch ? "CAPTURED" : "waiting..");
			}
			if (got_spin) {
				uint8_t sp_type =
					g_midi_learn_jog_spin_status & 0xF0;
				int sp_ch =
					(g_midi_learn_jog_spin_status & 0x0F) +
					1;
				if (sp_type == 0xE0)
					mvwprintw(
						g_win_main, cy++, ox + 4,
						"   CC Spin: PitchBend ch%d (status %02X d1=0)",
						sp_ch,
						g_midi_learn_jog_spin_status);
				else
					mvwprintw(g_win_main, cy++, ox + 4,
						  "   CC Spin: CC ch%d d1=%d",
						  sp_ch,
						  g_midi_learn_jog_spin_d1);
			}
			if (!got_spin || !got_pb) {
				mvwprintw(
					g_win_main, cy++, ox + 4,
					ns7mode ?
						"   Spin the jog wheel with motor on." :
						"   Touch and spin the jog wheel (motor on for pitch bend).");
			} else if (!ns7mode && !got_touch) {
				mvwprintw(
					g_win_main, cy++, ox + 4,
					"   Spin+PB captured. Touch the platter top, or ENTER to finish.");
			}
			mvwprintw(g_win_main, cy++, ox + 4,
				  "   ENTER = done   ESC = cancel");
			wattroff(g_win_main, A_DIM);
		} else if (g_midi_learn_active) {
			MidiAction la = (MidiAction)g_midi_learn_sel;
			int is_enc =
				(la >= MACT_JOG_SPIN_A &&
				 la <= MACT_JOG_SPIN_D) ||
				(la >= MACT_JOG_PB_A && la <= MACT_JOG_PB_D) ||
				(la == MACT_LIB_ENCODER) ||
				(la >= MACT_PITCH_BEND_A &&
				 la <= MACT_PITCH_BEND_D) ||
				(la >= MACT_FX_KNOB_1_A && la <= MACT_FX_WET_B);
			int is_note =
				(la >= MACT_JOG_TOUCH_A &&
				 la <= MACT_JOG_TOUCH_D) ||
				(la == MACT_LIB_ENCODER_TOUCH) ||
				(la >= MACT_PLAY_A && la <= MACT_PLAY_D) ||
				(la >= MACT_CUE_SET_1 &&
				 la <= MACT_CUE_SET_4) ||
				(la >= MACT_CUE_JUMP_1 &&
				 la <= MACT_CUE_JUMP_4) ||
				(la >= MACT_CUE_DELETE_1 &&
				 la <= MACT_CUE_DELETE_4) ||
				(la >= MACT_SYNC_SLAVE_A &&
				 la <= MACT_SYNC_SLAVE_D) ||
				(la == MACT_NUDGE_FWD) ||
				(la == MACT_NUDGE_BACK) ||
				(la == MACT_NUDGE_FWD_B) ||
				(la == MACT_NUDGE_BACK_B) ||
				(la == MACT_LOOP_TOGGLE) ||
				(la >= MACT_LOOP_IN_A &&
				 la <= MACT_LOOP_HALF_D) ||
				(la >= MACT_KEY_LOCK_A &&
				 la <= MACT_KEY_LOCK_D) ||
				(la >= MACT_SLIP_MODE_A &&
				 la <= MACT_SLIP_MODE_D) ||
				(la >= MACT_REVERSE_A &&
				 la <= MACT_REVERSE_D) ||
				(la >= MACT_LIB_SELECT && la <= MACT_LIB_FWD) ||
				(la >= MACT_LIB_LOAD_A &&
				 la <= MACT_LIB_LOAD_D) ||
				(la == MACT_PANEL_FILES) ||
				(la == MACT_PANEL_LIBRARY) ||
				(la >= MACT_PITCH_RANGE_A &&
				 la <= MACT_PITCH_RANGE_D) ||
				(la >= MACT_MOTOR_ON_A &&
				 la <= MACT_MOTOR_OFF_D) ||
				(la >= MACT_MOTOR_TOGGLE_A &&
				 la <= MACT_MOTOR_TOGGLE_D) ||
				(la >= MACT_DECK_SEL_1 &&
				 la <= MACT_DECK_SEL_4) ||
				(la >= MACT_SHIFT_A &&
				 la <= MACT_CUE_DEFAULT_B) ||
				(la >= MACT_PAD_MODE_CUES_A &&
				 la <= MACT_PAD_MODE_AUTOROLL_B) ||
				(la >= MACT_PAD_1_A && la <= MACT_PAD_8_B) ||
				(la >= MACT_PARAM_LEFT_A &&
				 la <= MACT_PARAM_RIGHT_B) ||
				(la >= MACT_FX_BTN_1_A &&
				 la <= MACT_FX_BTN_3_B);
			wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 2,
				  " >> LEARNING: %-20s <<",
				  g_mact_names[g_midi_learn_sel]);
			wattroff(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
			wattron(g_win_main, A_DIM);
			if (is_enc)
				mvwprintw(
					g_win_main, cy++, ox + 4,
					"   Rotate the encoder/jog in either direction.  ESC = cancel.");
			else if (is_note)
				mvwprintw(
					g_win_main, cy++, ox + 4,
					"   Press the button/pad.  ESC = cancel.");
			else
				mvwprintw(
					g_win_main, cy++, ox + 4,
					"   Move the fader/knob.  ESC = cancel.");
			wattroff(g_win_main, A_DIM);
		} else {
			wattron(g_win_main, A_DIM);
			mvwprintw(
				g_win_main, cy++, ox + 2,
				" L=learn single  W=jog learn  U=unbind  S=save  ESC=cancel");
			mvwprintw(
				g_win_main, cy++, ox + 2,
				" j/k=select device  ENTER=switch device  R=rescan");
			mvwprintw(
				g_win_main, cy++, ox + 2,
				" UP/DOWN=scroll bindings   J=MIDI monitor   \u25ba MIDI OUT tab for motor/LED/probe");
			wattroff(g_win_main, A_DIM);
		}

		/* ── Motor probe panel ────────────────────────────────────────────── */
		if (g_motor_probe_open) {
			cy++;
			wattron(g_win_main, A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 2,
				  " MOTOR PROBE  (ESC or M to close)");
			wattroff(g_win_main, A_BOLD);

			/* Message type row */
			{
				const char *types[] = { "CC", "NoteOn",
							"NoteOff" };
				mvwprintw(g_win_main, cy, ox + 4, "  Type: ");
				for (int ti = 0; ti < 3; ti++) {
					int sel = (ti == g_motor_probe_type);
					if (sel)
						wattron(g_win_main,
							COLOR_PAIR(
								COLOR_ACTIVE) |
								A_BOLD);
					wprintw(g_win_main, "%s%s%s ",
						sel ? "[" : "", types[ti],
						sel ? "]" : "");
					if (sel)
						wattroff(g_win_main,
							 COLOR_PAIR(
								 COLOR_ACTIVE) |
								 A_BOLD);
				}
				mvwprintw(g_win_main, cy, ox + 4 + 40,
					  "  (T to cycle)");
				cy++;
			}

			/* Channel row */
			mvwprintw(g_win_main, cy, ox + 4, "  Ch: ");
			for (int ch = 1; ch <= 16; ch++) {
				int sel = (ch == g_motor_probe_ch);
				if (sel)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_ACTIVE) |
							A_BOLD);
				wprintw(g_win_main, "%s%d%s ", sel ? "[" : "",
					ch, sel ? "]" : "");
				if (sel)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_ACTIVE) |
							 A_BOLD);
			}
			cy++;

			/* CC/Note number and value */
			mvwprintw(g_win_main, cy++, ox + 4,
				  "  d1 (CC/Note#): %-3d    d2 (value): %-3d",
				  g_motor_probe_cc, g_motor_probe_val);

			/* Controls */
			cy++;
			wattron(g_win_main, A_BOLD);
			mvwprintw(
				g_win_main, cy++, ox + 4,
				"  SPACE=send(127)  BKSP=send(0)  V=send(val)  X=sweep 0→127");
			wattroff(g_win_main, A_BOLD);
			wattron(g_win_main, A_DIM);
			mvwprintw(
				g_win_main, cy++, ox + 4,
				"  LEFT/RIGHT=channel   -/+=d1   ,/.=d2 value   T=type");
			wattroff(g_win_main, A_DIM);

			/* Log */
			cy++;
			wattron(g_win_main,
				g_midi_out ? 0 :
					     COLOR_PAIR(COLOR_HOT) | A_BOLD);
			mvwprintw(
				g_win_main, cy++, ox + 4, "  %s",
				g_midi_out ?
					g_motor_probe_log :
					"No MIDI output — switch device first");
			if (g_midi_out)
				wattroff(g_win_main, 0);
			else
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_HOT) | A_BOLD);

			mvwprintw(
				g_win_main, cy++, ox + 4,
				"  Use motor probe to discover start/stop CC for your controller.");
			cy++;
		} /* end if (g_motor_probe_open) */

		/* ── MIDI monitor panel ───────────────────────────────────────────── */
		if (g_midi_mon_open) {
			cy++;
			wattron(g_win_main, A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 2,
				  " MIDI MONITOR  (J to close)   jog_type: %s",
				  g_jog_type == JOG_NS7III ? "ns7iii" :
							     "relative");
			wattroff(g_win_main, A_BOLD);
			wattron(g_win_main, A_DIM);
			mvwprintw(g_win_main, cy++, ox + 4,
				  "  %-6s %-4s %-4s %-4s  %-18s  %s", "TYPE",
				  "CH", "D1", "D2", "ACTION", "EXTRA");
			wattroff(g_win_main, A_DIM);

			int n = g_midi_mon_count < MIDI_MON_SIZE ?
					g_midi_mon_count :
					MIDI_MON_SIZE;
			for (int i = 0; i < n && cy < oy + oh - 4; i++) {
				/* Oldest first: walk backwards from head */
				int idx = (g_midi_mon_head - n + i +
					   MIDI_MON_SIZE) %
					  MIDI_MON_SIZE;
				MidiMonEntry *me = &g_midi_mon_buf[idx];
				uint8_t mtype = me->status & 0xF0;
				int mch = (me->status & 0x0F) + 1;
				const char *tname = mtype == 0xB0 ? "CC   " :
						    mtype == 0x90 ? "NoteOn" :
						    mtype == 0x80 ? "NoteOff" :
						    mtype == 0xE0 ? "PitchBnd" :
								    "Other";

				/* For pitch bend, decode the 14-bit value and show mode-appropriate info */
				char extra[64] = "";
				if (mtype == 0xE0) {
					int raw14 = ((int)me->d2 << 7) |
						    (int)me->d1;
					MidiAction pa =
						(MidiAction)me->matched_act;
					int pdeck = -1;
					if (pa >= MACT_JOG_PB_A &&
					    pa <= MACT_JOG_PB_D)
						pdeck = pa - MACT_JOG_PB_A;
					if (g_jog_type == JOG_NS7III &&
					    pdeck >= 0 && pdeck < MAX_TRACKS) {
						snprintf(
							extra, sizeof(extra),
							"fine=%-5d pos=%.5f vel=%+.4f",
							raw14,
							g_jog_abs_pos[pdeck],
							g_jog_abs_vel[pdeck]);
					} else {
						float norm = ((float)raw14 -
							      8192.0f) /
							     8192.0f;
						if (pdeck >= 0 &&
						    pdeck < MAX_TRACKS)
							snprintf(
								extra,
								sizeof(extra),
								"raw14=%-5d norm=%+.3f streak=%d",
								raw14, norm,
								g_pb_streak
									[pdeck]);
						else
							snprintf(
								extra,
								sizeof(extra),
								"raw14=%-5d norm=%+.3f",
								raw14, norm);
					}
				} else if (mtype == 0xB0) {
					MidiAction pa =
						(MidiAction)me->matched_act;
					int pdeck = -1;
					if (pa >= MACT_JOG_SPIN_A &&
					    pa <= MACT_JOG_SPIN_D)
						pdeck = pa - MACT_JOG_SPIN_A;
					if (g_jog_type == JOG_NS7III &&
					    pdeck >= 0) {
						/* Show coarse angle and reconstructed position */
						snprintf(
							extra, sizeof(extra),
							"coarse=%-3d pos=%.5f vel=%+.4f",
							(int)me->d2,
							g_jog_abs_pos[pdeck],
							g_jog_abs_vel[pdeck]);
					} else {
						int delta = (int)me->d2 - 64;
						snprintf(extra, sizeof(extra),
							 "delta=%+d", delta);
					}
				}

				const char *act_name =
					(me->matched_act > 0 &&
					 me->matched_act < MACT_COUNT) ?
						g_mact_names[me->matched_act] :
						"--";

				/* Highlight the most recent entry */
				int is_latest = (idx == (g_midi_mon_head +
							 MIDI_MON_SIZE - 1) %
								MIDI_MON_SIZE);
				if (is_latest)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_ACTIVE));
				mvwprintw(
					g_win_main, cy++, ox + 4,
					"  %-8s %-4d %-4d %-4d  %-18.18s  %.24s",
					tname, mch, me->d1, me->d2, act_name,
					extra);
				if (is_latest)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_ACTIVE));
			}
			if (n == 0) {
				wattron(g_win_main, A_DIM);
				mvwprintw(
					g_win_main, cy++, ox + 4,
					"  (no messages yet — touch the controller)");
				wattroff(g_win_main, A_DIM);
			}
			cy++;
		} /* end midi monitor panel */

		/* Action binding list — scrollable, categorized with headers */
		int list_rows = oy + oh - cy - 3;
		
		/* ── Category definitions ── */
		typedef struct {
			const char *header;
			MidiAction first, last;
		} MactCat;
		
		static const MactCat cats[] = {
			{ "--- MIXER / FADERS ---", MACT_DECK_VOL_A, MACT_BOOTH_VOL },
			{ "--- TRANSPORT / PLAY ---", MACT_PLAY_A, MACT_CUE_ACTIVE_D },
			{ "--- CUE POINTS ---", MACT_CUE_SET_1, MACT_CUE_DELETE_4 },
			{ "--- SYNC / NUDGE ---", MACT_SYNC_SLAVE_A, MACT_NUDGE_BACK_B },
			{ "--- LOOPS ---", MACT_LOOP_TOGGLE, MACT_LOOP_HALF_D },
			{ "--- DECK TOGGLES ---", MACT_KEY_LOCK_A, MACT_BLEEP_D },
			{ "--- JOG WHEEL ---", MACT_STRIP_A, MACT_JOG_PB_D },
			{ "--- LIBRARY / BROWSER ---", MACT_LIB_ENCODER, MACT_PANEL_LIBRARY },
			{ "--- HARDWARE CONTROL ---", MACT_PITCH_RANGE_A, MACT_DECK_SEL_4 },
			{ "--- PADS / SHIFT ---", MACT_SHIFT_A, MACT_PAD_8_B },
			{ "--- FX / UTILITY ---", MACT_PARAM_LEFT_A, MACT_GRID_SNAP_B },
			{ NULL, 0, 0 }
		};

		/* Calculate flat index for selection tracking, including headers */
		int total_menu_items = 0;
		for (int ci = 0; cats[ci].header; ci++) {
			total_menu_items++; /* header */
			total_menu_items += (cats[ci].last - cats[ci].first + 1);
		}
		total_menu_items++; /* Panic button */

		/* Adjust g_options_sel to stay within menu bounds if MACT_COUNT changed */
		if (g_options_sel > total_menu_items) g_options_sel = 1;

		int scroll = 0;
		if (g_options_sel > list_rows)
			scroll = g_options_sel - list_rows;

		int current_item_idx = 1; /* 1-based index for comparison with g_options_sel */
		
		for (int ci = 0; cats[ci].header && cy < oy + oh - 3; ci++) {
			/* Draw header */
			if (current_item_idx > scroll) {
				wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
				mvwprintw(g_win_main, cy++, ox + 2, "%s", cats[ci].header);
				wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
			}
			current_item_idx++;

			for (int ai = cats[ci].first; ai <= cats[ci].last && cy < oy + oh - 3; ai++) {
				if (current_item_idx <= scroll) {
					current_item_idx++;
					continue;
				}

				int sel = (g_options_sel == current_item_idx);
				
				/* Find current binding for this action */
				const char *bound_str = "  --unbound--  ";
				char bound_buf[32] = "";
				for (int bi = 0; bi < g_midi_nbindings; bi++) {
					if (g_midi_bindings[bi].action == (MidiAction)ai) {
						uint8_t st = g_midi_bindings[bi].status;
						uint8_t d1 = g_midi_bindings[bi].data1;
						uint8_t btype = st & 0xF0;
						uint8_t bch = st & 0x0F;
						snprintf(bound_buf, sizeof(bound_buf),
							 "%s ch%d  d1:%3d",
							 btype == 0xB0 ? "CC    " :
							 btype == 0x90 ? "NoteOn" : "Other ",
							 bch + 1, d1);
						bound_str = bound_buf;
						break;
					}
				}

				/* Routing annotation */
				char route_buf[24] = "";
				if ((MidiAction)ai == MACT_LIB_LOAD_A)
					snprintf(route_buf, sizeof(route_buf), " \u2192 Deck %c", DECK_NUM(g_side_deck[0]));
				else if ((MidiAction)ai == MACT_LIB_LOAD_B)
					snprintf(route_buf, sizeof(route_buf), " \u2192 Deck %c", DECK_NUM(g_side_deck[1]));
				else if ((MidiAction)ai == MACT_DECK_SEL_1)
					snprintf(route_buf, sizeof(route_buf), " [left \u2192 A]");
				else if ((MidiAction)ai == MACT_DECK_SEL_3)
					snprintf(route_buf, sizeof(route_buf), " [left \u2192 C]");
				else if ((MidiAction)ai == MACT_DECK_SEL_2)
					snprintf(route_buf, sizeof(route_buf), " [right \u2192 B]");
				else if ((MidiAction)ai == MACT_DECK_SEL_4)
					snprintf(route_buf, sizeof(route_buf), " [right \u2192 D]");

				if (sel && g_midi_learn_active)
					wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
				else if (sel)
					wattron(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);

				mvwprintw(g_win_main, cy++, ox + 4, "  %-16s  %s%s", g_mact_names[ai], bound_str, route_buf);

				if (sel && g_midi_learn_active)
					wattroff(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
				else if (sel)
					wattroff(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
				
				current_item_idx++;
			}
		}

		/* Panic button at the very bottom */
		if (cy < oy + oh - 3) {
			if (current_item_idx > scroll) {
				cy++;
				int sel = (g_options_sel == current_item_idx);
				if (sel) wattron(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
				mvwprintw(g_win_main, cy++, ox + 2, "  [ MIDI PANIC — reset all faders/EQ ]");
				if (sel) wattroff(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			}
			current_item_idx++;
		}
		
		/* Important: we must update max_sel in the input handler to match this new total_menu_items */
	} /* end g_options_tab == 6 (MIDI IN) */

	else if (g_options_tab == 7) {
		/* ── MIDI OUT ─────────────────────────────────────────────────────── */

		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " MIDI OUTPUT DEVICE");
		wattroff(g_win_main, A_BOLD);

		/* Show active output device — same as input (shared handle) */
		if (g_midi_out) {
			wattron(g_win_main, A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 4,
				  "  \u25CF %-*.*s  [%.12s]  (in+out)", iw - 32,
				  iw - 32,
				  g_midi_dev_sel >= 0 &&
						  g_midi_dev_sel <
							  g_midi_ndevices ?
					  g_midi_devlist[g_midi_dev_sel].name :
					  "connected",
				  g_midi_dev_str);
			wattroff(g_win_main, A_BOLD);
		} else {
			wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
			mvwprintw(
				g_win_main, cy++, ox + 4,
				"  No MIDI output — select device in MIDI IN tab");
			wattroff(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
		}
		cy++;

		/* ── Motor probe panel ── */
		if (g_motor_probe_open) {
			wattron(g_win_main, A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 2,
				  " MOTOR PROBE  (M to close)");
			wattroff(g_win_main, A_BOLD);

			/* Type row */
			{
				const char *types[] = { "CC", "NoteOn",
							"NoteOff" };
				mvwprintw(g_win_main, cy, ox + 4, "  Type: ");
				for (int ti = 0; ti < 3; ti++) {
					int sel = (ti == g_motor_probe_type);
					if (sel)
						wattron(g_win_main,
							COLOR_PAIR(
								COLOR_ACTIVE) |
								A_BOLD);
					wprintw(g_win_main, "%s%s%s ",
						sel ? "[" : "", types[ti],
						sel ? "]" : "");
					if (sel)
						wattroff(g_win_main,
							 COLOR_PAIR(
								 COLOR_ACTIVE) |
								 A_BOLD);
				}
				mvwprintw(g_win_main, cy, ox + 4 + 40,
					  "  (T to cycle)");
				cy++;
			}

			/* Channel row */
			mvwprintw(g_win_main, cy, ox + 4, "  Ch: ");
			for (int ch = 1; ch <= 16; ch++) {
				int sel = (ch == g_motor_probe_ch);
				if (sel)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_ACTIVE) |
							A_BOLD);
				wprintw(g_win_main, "%s%d%s ", sel ? "[" : "",
					ch, sel ? "]" : "");
				if (sel)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_ACTIVE) |
							 A_BOLD);
			}
			cy++;

			mvwprintw(g_win_main, cy++, ox + 4,
				  "  d1 (CC/Note#): %-3d    d2 (value): %-3d",
				  g_motor_probe_cc, g_motor_probe_val);
			cy++;
			wattron(g_win_main, A_BOLD);
			mvwprintw(
				g_win_main, cy++, ox + 4,
				"  SPACE=send(127)  BKSP=send(0)  V=send(val)  X=sweep 0\u2192127");
			wattroff(g_win_main, A_BOLD);
			wattron(g_win_main, A_DIM);
			mvwprintw(
				g_win_main, cy++, ox + 4,
				"  LEFT/RIGHT=channel   -/+=d1(CC#)   ,/.=d2(value)   T=type");
			wattroff(g_win_main, A_DIM);
			cy++;
			wattron(g_win_main,
				g_midi_out ? 0 :
					     COLOR_PAIR(COLOR_HOT) | A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 4, "  %s",
				  g_midi_out ? g_motor_probe_log :
					       "No MIDI output available");
			if (!g_midi_out)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_HOT) | A_BOLD);
			mvwprintw(
				g_win_main, cy++, ox + 4,
				"  Use motor probe to discover start/stop CC for your controller.");
			cy++;
		}

		/* ── Output binding list ── */
		{
			wattron(g_win_main, A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 2,
				  " OUTPUT BINDINGS  (%d configured)",
				  g_midi_nout_bindings);
			wattroff(g_win_main, A_BOLD);

			if (g_midi_nout_bindings == 0) {
				wattron(g_win_main, A_DIM);
				mvwprintw(
					g_win_main, cy++, ox + 4,
					"  No output bindings. Edit map file or use motor probe.");
				wattroff(g_win_main, A_DIM);
			} else {
				/* Clamp selection */
				if (g_options_out_sel < 0)
					g_options_out_sel = 0;
				if (g_options_out_sel >= g_midi_nout_bindings)
					g_options_out_sel =
						g_midi_nout_bindings - 1;

				int list_rows = oy + oh - cy - 4;
				int scroll = 0;
				if (g_options_out_sel + 1 > list_rows)
					scroll = g_options_out_sel + 1 -
						 list_rows;

				for (int oi = 0; oi < g_midi_nout_bindings &&
						 cy < oy + oh - 4;
				     oi++) {
					if (oi < scroll)
						continue;
					MidiOutBinding *b =
						&g_midi_out_bindings[oi];
					int sel = (oi == g_options_out_sel);
					if (sel)
						wattron(g_win_main,
							COLOR_PAIR(
								COLOR_ACTIVE) |
								A_BOLD);
					if (b->sysex_len > 0) {
						mvwprintw(
							g_win_main, cy++,
							ox + 4,
							"  %-18s  RGB  pad=%-3d r=%-3d g=%-3d b=%-3d",
							b->name, b->data1,
							b->sysex_len > 7 ?
								b->sysex[7] :
								0,
							b->sysex_len > 8 ?
								b->sysex[8] :
								0,
							b->sysex_len > 9 ?
								b->sysex[9] :
								0);
					} else {
						mvwprintw(
							g_win_main, cy++,
							ox + 4,
							"  %-18s  %02X  d1=%-3d  d2=%-3d",
							b->name, b->status,
							b->data1, b->data2);
					}
					if (sel)
						wattroff(g_win_main,
							 COLOR_PAIR(
								 COLOR_ACTIVE) |
								 A_BOLD);
				}
			}
			cy++;
			wattron(g_win_main, A_DIM);
			mvwprintw(
				g_win_main, cy++, ox + 2,
				" M=motor probe  j/k=select row  S=save map  R=rescan");
			mvwprintw(
				g_win_main, cy++, ox + 2,
				" Edit map file directly to add/change LED/motor bindings");
			mvwprintw(
				g_win_main, cy++, ox + 2,
				" NS7III LEDs: led_sync_a/b  led_slip_a/b  led_loop_a/b");
			mvwprintw(
				g_win_main, cy++, ox + 2,
				"              led_pitch_center_a/b  led_cue_1..4_a/b");
			mvwprintw(
				g_win_main, cy++, ox + 2,
				"              led_pad_hotcue/roll/loop/sampler/slicer_a/b");
			wattroff(g_win_main, A_DIM);
		}
	} /* end g_options_tab == 7 (MIDI OUT) */

	else if (g_options_tab == 8) {
		/* FX tab — erase stale content from panel interior before drawing */
		wattron(g_win_main, COLOR_PAIR(COLOR_STATUS));
		for (int _r = oy + 1; _r < oy + oh - 1; _r++) {
			wmove(g_win_main, _r, ox + 1);
			for (int _c = 1; _c < ow - 1; _c++)
				waddch(g_win_main, ' ');
		}
		wattroff(g_win_main, COLOR_PAIR(COLOR_STATUS));
		/* ── FX ──────────────────────────────────────────────────────── */
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " EFFECTS ENGINE");
		wattroff(g_win_main, A_BOLD);
		wattron(g_win_main, A_DIM);
		mvwprintw(g_win_main, cy++, ox + 2,
			  " j/k=select  ENTER=cycle type  +/-=wet  >/<=param0");
		wattroff(g_win_main, A_DIM);
		cy++;

		static const char *fx_desc[FX_COUNT] = {
			"No effect",
			"p0=time  p1=feedback  +/-=wet",
			"p0=time  p1=feedback  +/-=wet  (L/R alternate)",
			"p0=room  p1=damp  p2=width  +/-=wet",
			"p0=delay  p1=depth  p2=rate  +/-=wet",
			"p0=delay  p1=depth  p2=rate  +/-=wet  (slower)",
			"p0=base_freq  p1=sweep  p2=rate  +/-=wet",
			"p0=drive  p1=tone  +/-=wet",
			"p0=bits  p1=sr_div  +/-=wet",
			"p0=threshold  p1=attack  p2=release  +/-=wet",
			"p0=width  +/-=wet",
		};

		int master_row = g_num_tracks * FX_SLOTS_PER_DECK;
		if (g_options_sel < 0)
			g_options_sel = 0;
		if (g_options_sel > master_row)
			g_options_sel = master_row;

		for (int dk = 0; dk < g_num_tracks && cy < oy + oh - 6; dk++) {
			wattron(g_win_main, A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 2, " Deck %c",
				  DECK_NUM(dk));
			wattroff(g_win_main, A_BOLD);
			for (int sl = 0;
			     sl < FX_SLOTS_PER_DECK && cy < oy + oh - 6; sl++) {
				int row_idx = dk * FX_SLOTS_PER_DECK + sl;
				int sel = (g_options_sel == row_idx);
				FXSlot *fx = fx_slot(dk, sl);
				int cur_type = (fx->pending_type >= 0) ?
						       fx->pending_type :
						       fx->type;
				if (sel)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_ACTIVE) |
							A_BOLD);
				mvwprintw(
					g_win_main, cy++, ox + 4,
					"  Slot %d: %-12s  wet:%3.0f%%  p0:%3.0f%%  p1:%3.0f%%  p2:%3.0f%%  %s",
					sl + 1, fx_names[cur_type],
					fx->params[3] * 100.0f,
					fx->params[0] * 100.0f,
					fx->params[1] * 100.0f,
					fx->params[2] * 100.0f,
					(g_fx_ui_slot[dk] == sl) ? "[knobs]" :
								   "");
				if (sel)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_ACTIVE) |
							 A_BOLD);
				if (sel && cur_type < FX_COUNT) {
					wattron(g_win_main, A_DIM);
					mvwprintw(g_win_main, cy++, ox + 6,
						  "  %s", fx_desc[cur_type]);
					wattroff(g_win_main, A_DIM);
				}
			}
		}
		cy++;
		/* Master bus */
		{
			int sel = (g_options_sel == master_row);
			FXSlot *ms = fx_master();
			int cur_type = (ms->pending_type >= 0) ?
					       ms->pending_type :
					       ms->type;
			wattron(g_win_main, A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 2, " Master Bus");
			wattroff(g_win_main, A_BOLD);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(
				g_win_main, cy++, ox + 4,
				"  %-12s  wet:%3.0f%%  thresh:%3.0f%%  ratio:%4.1f:1  makeup:%4.1fdB",
				fx_names[cur_type], ms->params[3] * 100.0f,
				ms->params[0] * 100.0f,
				1.0f + ms->params[1] * 19.0f,
				ms->params[2] * 12.0f);
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		}
	} /* end g_options_tab == 8 (FX) */
}

static void draw_status(void)
{
	static const char *views[] = { "DECKS", "BROWSER", "HELP" };
	char view_label[32];
	wattron(g_win_status, COLOR_PAIR(COLOR_STATUS));
	werase(g_win_status);

	if (g_view == 1)
		snprintf(view_label, sizeof(view_label), "[%s]",
			 g_panel == 0 ? "BROWSER" :
			 g_panel == 1 ? "PLAYLIST" :
					"LIBRARY");
	else
		snprintf(view_label, sizeof(view_label), "%s",
			 views[g_view < 3 ? g_view : 0]);

	/* Build gang mask string e.g. "AB" */
	char gang_str[8] = "off";
	if (g_gang_mode) {
		int p = 0;
		for (int i = 0; i < g_num_tracks; i++)
			if (g_gang_mask & (1 << i))
				gang_str[p++] = 'A' + i;
		if (p == 0)
			strcpy(gang_str, "---");
		else
			gang_str[p] = '\0';
	}

	char master_str[4] = "-";
	if (g_sync_master >= 0)
		master_str[0] = DECK_NUM(g_sync_master), master_str[1] = '\0';

	/* Crossfader visualizer bar: [A....:....B] where | moves */
	char cf_bar[14] = "[A....:....B]";
	int cf_pos = (int)(g_crossfader * 10.0f + 0.5f); /* 0 to 10 */
	if (cf_pos < 0)
		cf_pos = 0;
	if (cf_pos > 10)
		cf_pos = 10;
	/* Re-render bar with current position */
	for (int i=1; i<=11; i++) cf_bar[i] = '.';
	cf_bar[6] = ':'; /* center marker */
	int fader_idx = cf_pos + 1;
	cf_bar[fader_idx] = '|'; 
	/* Restore A and B labels if not overwritten */
	if (cf_bar[1] == '.') cf_bar[1] = 'A';
	if (cf_bar[11] == '.') cf_bar[11] = 'B';

	mvwprintw(
		g_win_status, 0, 0,
		" djcmd %-11s| Deck:%c | Vol:%3d%% | XF:%s | Gang:%-4s | Mstr:%s | MIDI:%s | TAB=browser ?=help Q=quit",
		view_label, 'A' + g_active_track, g_master_vol, cf_bar, gang_str,
		master_str, g_midi_in ? "ON" : "off");
	wattroff(g_win_status, COLOR_PAIR(COLOR_STATUS));

	/* Clock: always visible, right-justified in status bar */
	time_t _now = time(NULL);
	struct tm *_lt = localtime(&_now);
	wattron(g_win_status, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	mvwprintw(g_win_status, 0, g_cols - 6, " %02d:%02d", _lt->tm_hour,
		  _lt->tm_min);
	wattroff(g_win_status, COLOR_PAIR(COLOR_HEADER) | A_BOLD);

	wrefresh(g_win_status);

	/* If the library panel would be hidden in split mode, warn in status bar.
     * The redraw() function falls back to full-screen browser in this case,
     * so TAB still works — the warning just explains waveforms are gone. */
	if (library_rows_available() < 4) {
		int need = library_min_rows();
		char warn[64];
		snprintf(warn, sizeof(warn),
			 " \xe2\x9a\xa0 need %d rows for split ", need);
		int wx = g_cols - (int)strlen(warn) - 6;
		if (wx < 0)
			wx = 0;
		wattron(g_win_status,
			COLOR_PAIR(COLOR_HOT) | A_BOLD | A_REVERSE);
		mvwprintw(g_win_status, 0, wx, "%s", warn);
		wattroff(g_win_status,
			 COLOR_PAIR(COLOR_HOT) | A_BOLD | A_REVERSE);
		wrefresh(g_win_status);
	}
}

static void redraw(void)
{
	g_blink_tick++;
	/* Update blinking LEDs (play button when loaded+paused) */
	if (g_midi_out)
		deck_leds_refresh();

	/* ── Library auto-view restore ──────────────────────────────────────
     * If the view was auto-switched to split view by a lib_encoder scroll
     * or touch, revert to 4-deck view after 1 second of inactivity. */
	if (g_lib_auto_switched && g_lib_enc_last_ms > 0) {
		struct timespec _rts;
		clock_gettime(CLOCK_MONOTONIC, &_rts);
		int64_t _now =
			(int64_t)_rts.tv_sec * 1000 + _rts.tv_nsec / 1000000;
		if (_now - g_lib_enc_last_ms >= 1000) {
			if (g_view == 1 && g_num_tracks == 4)
				g_view = 0;
			g_lib_auto_switched = 0;
			g_lib_enc_last_ms = 0;
		}
	}

	getmaxyx(stdscr, g_rows, g_cols);
	wresize(g_win_main, g_rows - 1, g_cols);
	wresize(g_win_status, 1, g_cols);
	mvwin(g_win_status, g_rows - 1, 0);

	werase(g_win_main);
	switch (g_view) {
	case 0:
		draw_decks_view();
		break;
	case 1:
		/* Use split view (waveforms + browser) whenever the terminal is tall
         * enough, regardless of deck count.  Fall back to full-screen browser
         * when there isn't room, so TAB always works for loading tracks. */
		if (library_rows_available() >= 4) {
			draw_split_view();
		} else {
			/* Terminal too short — full-screen browser with a warning banner */
			int need = library_min_rows();
			wattron(g_win_main,
				COLOR_PAIR(COLOR_HOT) | A_BOLD | A_REVERSE);
			mvwprintw(
				g_win_main, 0, 0,
				" \xe2\x9a\xa0  Waveforms hidden \xe2\x80\x94 terminal too short"
				" (%d rows, need %d for split view)%*s",
				g_rows, need, g_cols - 60, "");
			wattroff(g_win_main,
				 COLOR_PAIR(COLOR_HOT) | A_BOLD | A_REVERSE);
			draw_browser_view();
		}
		break;
	case 2:
		draw_help_view();
		break;
	}
	/* Tag info overlay — drawn on top of any view */
	if (g_tag_info.visible)
		draw_tag_panel();
	/* Options overlay — drawn on top of everything */
	if (g_options_open)
		draw_options_overlay();
	/* Quit confirm modal — drawn on top of everything including options */
	if (g_quit_pending)
		draw_quit_modal();
	wrefresh(g_win_main);
	draw_status();
}

/* ──────────────────────────────────────────────
   Input Handling
   ────────────────────────────────────────────── */
static void options_adjust(int direction)
{
	/* Adjust currently-selected option by direction (+1/-1) */
	if (g_options_tab == 1) {
		switch (g_options_sel) {
		case 0: /* master vol */
			g_opts.default_master_vol += direction * 5;
			if (g_opts.default_master_vol < 0)
				g_opts.default_master_vol = 0;
			if (g_opts.default_master_vol > 150)
				g_opts.default_master_vol = 150;
			g_master_vol = g_opts.default_master_vol;
			break;
		case 1: /* deck vol */
			g_opts.default_deck_vol += direction * 0.05f;
			if (g_opts.default_deck_vol < 0.0f)
				g_opts.default_deck_vol = 0.0f;
			if (g_opts.default_deck_vol > 1.5f)
				g_opts.default_deck_vol = 1.5f;
			break;
		case 2: /* auto gain toggle */
			g_opts.auto_gain_default = !g_opts.auto_gain_default;
			break;
		case 3: /* auto gain target */
			g_opts.auto_gain_target_db += direction * 1.0f;
			if (g_opts.auto_gain_target_db < -24.0f)
				g_opts.auto_gain_target_db = -24.0f;
			if (g_opts.auto_gain_target_db > 0.0f)
				g_opts.auto_gain_target_db = 0.0f;
			break;
		}
	} else if (g_options_tab == 2) {
		switch (g_options_sel) {
		case 0: /* height gamma */
			g_opts.wfm_height_gamma += direction * 0.05f;
			if (g_opts.wfm_height_gamma < 0.2f)
				g_opts.wfm_height_gamma = 0.2f;
			if (g_opts.wfm_height_gamma > 1.5f)
				g_opts.wfm_height_gamma = 1.5f;
			break;
		case 1: /* visible secs */
			g_opts.wfm_visible_secs += direction * 0.5f;
			if (g_opts.wfm_visible_secs < 1.0f)
				g_opts.wfm_visible_secs = 1.0f;
			if (g_opts.wfm_visible_secs > 16.0f)
				g_opts.wfm_visible_secs = 16.0f;
			break;
		case 2: /* waveform style: cycle 0/1 */
			g_opts.wfm_style = (g_opts.wfm_style + 1) % 2;
			break;
		case 3: /* UI FPS: ±5 in steps, clamped 5–60 */
			g_opts.ui_fps += direction * 5;
			if (g_opts.ui_fps < 5)
				g_opts.ui_fps = 5;
			if (g_opts.ui_fps > 60)
				g_opts.ui_fps = 60;
			apply_ui_fps();
			break;
		}
	} else if (g_options_tab == 3) {
		/* WAVEFORM (advanced) tab */
		switch (g_options_sel) {
		case 0: /* bass weight */
			g_opts.wfm_lo_weight += direction * 0.05f;
			if (g_opts.wfm_lo_weight < 0.1f)
				g_opts.wfm_lo_weight = 0.1f;
			if (g_opts.wfm_lo_weight > 2.0f)
				g_opts.wfm_lo_weight = 2.0f;
			break;
		case 1: /* snare weight */
			g_opts.wfm_mid_weight += direction * 0.05f;
			if (g_opts.wfm_mid_weight < 0.1f)
				g_opts.wfm_mid_weight = 0.1f;
			if (g_opts.wfm_mid_weight > 2.0f)
				g_opts.wfm_mid_weight = 2.0f;
			break;
		case 2: /* treble weight */
			g_opts.wfm_hi_weight += direction * 0.05f;
			if (g_opts.wfm_hi_weight < 0.0f)
				g_opts.wfm_hi_weight = 0.0f;
			if (g_opts.wfm_hi_weight > 1.0f)
				g_opts.wfm_hi_weight = 1.0f;
			break;
		case 3: /* colour saturation */
			g_opts.wfm_color_sat += direction * 0.1f;
			if (g_opts.wfm_color_sat < 0.2f)
				g_opts.wfm_color_sat = 0.2f;
			if (g_opts.wfm_color_sat > 3.0f)
				g_opts.wfm_color_sat = 3.0f;
			break;
		case 4: /* colour floor */
			g_opts.wfm_color_floor += direction * 0.005f;
			if (g_opts.wfm_color_floor < 0.0f)
				g_opts.wfm_color_floor = 0.0f;
			if (g_opts.wfm_color_floor > 0.15f)
				g_opts.wfm_color_floor = 0.15f;
			break;
		case 5: /* anchor: toggle centred / bottom */
			g_opts.wfm_anchor = (g_opts.wfm_anchor + 1) % 2;
			break;
		}
	} else if (g_options_tab == 4) {
		/* SYNC tab — all toggles, direction ignored */
		switch (g_options_sel) {
		case 0:
			g_opts.sync_quantize = !g_opts.sync_quantize;
			break;
		case 1:
			g_opts.sync_smart_range = !g_opts.sync_smart_range;
			break;
		case 2:
			g_opts.sync_auto_handoff = !g_opts.sync_auto_handoff;
			break;
		case 3:
			g_opts.key_lock_default = !g_opts.key_lock_default;
			break;
		case 4:
			g_opts.vinyl_mode = !g_opts.vinyl_mode;
			break;
		}
	} else if (g_options_tab == 8) {
		/* FX tab: +/- adjusts wet; direction on param0 via < / > in key handler */
		int row = g_options_sel;
		int master_row = g_num_tracks * FX_SLOTS_PER_DECK;
		FXSlot *fx;
		if (row < master_row) {
			fx = fx_slot(row / FX_SLOTS_PER_DECK,
				     row % FX_SLOTS_PER_DECK);
		} else {
			fx = fx_master();
		}
		/* +/- adjusts wet (params[3]) in 5% steps */
		fx->params[3] += direction * 0.05f;
		if (fx->params[3] < 0.0f)
			fx->params[3] = 0.0f;
		if (fx->params[3] > 1.0f)
			fx->params[3] = 1.0f;
	}
	/* Persist every change immediately */
	settings_save();
}

/* Show a centered "Quit? y/n" prompt and return 1 if user confirms. */
/* Arm the quit confirm modal (non-blocking — audio/UI keeps running).
 * If nothing is playing, quits immediately without showing the modal. */
static void quit_confirm(void)
{
	int any_playing = 0;
	for (int i = 0; i < g_num_tracks; i++)
		if (g_tracks[i].playing) {
			any_playing = 1;
			break;
		}
	if (!any_playing) {
		g_running = 0;
		return;
	}
	g_quit_pending =
		1; /* redraw() draws the modal; handle_key/ui_thread dismiss */
}

/* Draw the non-blocking quit confirm modal — called from redraw() when
 * g_quit_pending is set, so the waveform playheads keep animating. */
static void draw_quit_modal(void)
{
	const char *msg = " Track playing.  Quit? ";
	const char *hint = "  Y = yes      N / ESC = cancel  ";
	int pw = (int)strlen(hint) + 4;
	int ph = 5;
	int px = (g_cols - pw) / 2;
	int py = (g_rows - ph) / 2;
	if (px < 0)
		px = 0;
	if (py < 0)
		py = 0;

	/* Shadow */
	wattron(g_win_main, A_DIM | COLOR_PAIR(COLOR_STATUS));
	for (int r = py + 1; r < py + ph + 1 && r < g_rows - 1; r++)
		for (int c = px + 2; c < px + pw + 2 && c < g_cols; c++)
			mvwaddch(g_win_main, r, c, ' ');
	wattroff(g_win_main, A_DIM | COLOR_PAIR(COLOR_STATUS));

	/* Box background */
	wattron(g_win_main, COLOR_PAIR(COLOR_STATUS));
	for (int r = py; r < py + ph; r++)
		for (int c = px; c < px + pw; c++)
			mvwaddch(g_win_main, r, c, ' ');
	wattroff(g_win_main, COLOR_PAIR(COLOR_STATUS));

	/* Border */
	wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
	for (int c = px; c < px + pw; c++) {
		mvwaddch(g_win_main, py, c,
			 (c == px)	    ? ACS_ULCORNER :
			 (c == px + pw - 1) ? ACS_URCORNER :
					      ACS_HLINE);
		mvwaddch(g_win_main, py + ph - 1, c,
			 (c == px)	    ? ACS_LLCORNER :
			 (c == px + pw - 1) ? ACS_LRCORNER :
					      ACS_HLINE);
	}
	for (int r = py + 1; r < py + ph - 1; r++) {
		mvwaddch(g_win_main, r, px, ACS_VLINE);
		mvwaddch(g_win_main, r, px + pw - 1, ACS_VLINE);
	}
	/* Title embedded in top border */
	mvwprintw(g_win_main, py, px + 2, "\u2524 QUIT \u251c");
	wattroff(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);

	/* Message line */
	int mx = px + (pw - (int)strlen(msg)) / 2;
	wattron(g_win_main, A_BOLD);
	mvwprintw(g_win_main, py + 1, mx, "%s", msg);
	wattroff(g_win_main, A_BOLD);

	/* Divider */
	wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_DIM);
	for (int c = px + 1; c < px + pw - 1; c++)
		mvwaddch(g_win_main, py + 2, c, ACS_HLINE);
	wattroff(g_win_main, COLOR_PAIR(COLOR_HOT) | A_DIM);

	/* Y / N hint */
	int hx = px + (pw - (int)strlen(hint)) / 2;
	wattron(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
	mvwprintw(g_win_main, py + 3, hx, "%s", hint);
	wattroff(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
}

static void handle_key(int c)
{
	Track *t = &g_tracks[g_active_track];

	/* ── Manual BPM entry mode ──────────────────────────────────────────
     * Activated by 'B'.  Collects digits; Enter commits, Esc cancels.
     * Backspace deletes last digit.  Dots are accepted for decimal input.
     * Updates the deck's BPM and resets the beat offset to 0 on commit. */
	if (g_bpm_entry) {
		/* ... existing BPM handling ... */
	}

	/* ── Crate jump mode ─────────────────────────────────────────── */
	if (g_crate_jump_active) {
		if (c == '\n' || c == '\r' || c == KEY_ENTER) {
			crate_jump(g_crate_input);
			g_crate_jump_active = 0;
			g_crate_input[0] = '\0';
		} else if (c == 27) { /* ESC */
			g_crate_jump_active = 0;
			g_crate_input[0] = '\0';
			snprintf(g_fb_status, sizeof(g_fb_status), "Crate jump cancelled");
		} else if (c == KEY_BACKSPACE || c == 127 || c == 8) {
			int len = (int)strlen(g_crate_input);
			if (len > 0) g_crate_input[len - 1] = '\0';
			snprintf(g_fb_status, sizeof(g_fb_status), "Jump to: %s_", g_crate_input);
		} else if (c >= 32 && c <= 126) {
			int len = (int)strlen(g_crate_input);
			if (len < (int)sizeof(g_crate_input) - 1) {
				g_crate_input[len] = (char)c;
				g_crate_input[len + 1] = '\0';
			}
			snprintf(g_fb_status, sizeof(g_fb_status), "Jump to: %s_", g_crate_input);
		}
		return;
	}

	if (g_view == 2) {
		switch (c) {
		case 'j':
		case KEY_DOWN:
			g_help_scroll++;
			break;
		case 'k':
		case KEY_UP:
			g_help_scroll--;
			break;
		case KEY_NPAGE: /* PgDn */
			g_help_scroll += (g_rows - 4);
			break;
		case KEY_PPAGE: /* PgUp */
			g_help_scroll -= (g_rows - 4);
			break;
		case KEY_HOME:
		case 'g':
			g_help_scroll = 0;
			break;
		case KEY_END:
		case 'G':
			g_help_scroll = 9999;
			break; /* clamped in draw */
		case '?':
			g_view = 1;
			g_help_scroll = 0;
			break;
		case 'Q': /* fall through to quit check below */
			break;
		default:
			return; /* swallow unhandled keys in help view */
		}
		if (c != 'Q')
			return;
	}

	/* ── Options overlay intercepts most keys ── */
	if (g_options_open) {
		int total_midi_items = 0;
		if (g_options_tab == 6) {
			/* mixer (4vol + 4pitch + 4lsb + 4low + 4mid + 4high + 4gain + 4filter + 4ftog + 1cf + 1mvol + 1booth) = 39 actions
			 * + 1 header = 40
			 * trans (4play + 4cueact) = 8 actions + 1 header = 9
			 * cue (4set + 4jump + 4del) = 12 actions + 1 header = 13
			 * sync (4slave + 2nudge + 2nudge_b) = 8 actions + 1 header = 9
			 * loop (1toggle + 4in + 4out + 4double + 4half) = 17 actions + 1 header = 18
			 * deckt (4key + 4slip + 4rev + 4bleep) = 16 actions + 1 header = 17
			 * jog (4strip + 4touch + 4spin + 4pb) = 16 actions + 1 header = 17
			 * lib (1enc + 1touch + 1sel + 1back + 1fwd + 4load + 1files + 1lib) = 11 actions + 1 header = 12
			 * hw (4prange + 4pbend + 4mtog + 4mon + 4moff + 4dsel) = 24 actions + 1 header = 25
			 * pads (2shift + 2pcenter + 2cdef + 4pmodes + 16pads) = 26 actions + 1 header = 27
			 * fx (4pleft + 4pright + 6fbtn + 6fknob + 2fwet + 1cfc + 2tap + 2snap) = 27 actions + 1 header = 28
			 * Total = 40+9+13+9+18+17+17+12+25+27+28 + 1 (Panic) = 216 approx.
			 * Actually we can just calculate it from the same 'cats' logic.
			 */
			int ci = 0;
			/* Categorized counts from 'cats' array in options_draw */
			int counts[] = { 39, 8, 12, 8, 17, 16, 16, 11, 24, 26, 27, 0 };
			while (counts[ci] > 0) {
				total_midi_items += counts[ci] + 1; /* header + actions */
				ci++;
			}
			total_midi_items++; /* Panic */
		}

		int max_sel = (g_options_tab == 0) ? 0 :
                      (g_options_tab == 1) ? 3 :
                      (g_options_tab == 2) ? 3 :
                      (g_options_tab == 3) ? 5 :
                      (g_options_tab == 4) ? 4 :
                      (g_options_tab == 5) ? THEME_COUNT - 1 :
                      (g_options_tab == 6) ? total_midi_items :
                                             g_midi_nout_bindings; /* MIDI OUT: output bindings */
		switch (c) {
		case 27: /* ESC: cancel learn, close probe, or close overlay */
			if (g_midi_learn_active) {
				g_midi_learn_active = 0;
				break;
			}
			if (g_midi_learn_jog_pair) {
				g_midi_learn_jog_pair = 0;
				g_midi_learn_jog_step = 0;
				g_midi_learn_jog_spin_status = 0;
				g_midi_learn_jog_spin_d1 = 0;
				break;
			}
			if (g_motor_probe_open) {
				g_motor_probe_open = 0;
				break;
			}
			if (g_options_tab == 5)
				apply_theme(g_opts.theme_idx);
			g_options_open = 0;
			break;
		case KEY_LEFT:
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open) {
				if (g_motor_probe_ch > 1)
					g_motor_probe_ch--;
			} else {
				g_options_tab = (g_options_tab + 8) % 9;
				g_options_sel = (g_options_tab == 5) ?
							g_opts.theme_idx :
							0;
			}
			break;
		case KEY_RIGHT:
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open) {
				if (g_motor_probe_ch < 16)
					g_motor_probe_ch++;
			} else {
				g_options_tab = (g_options_tab + 1) % 9;
				g_options_sel = (g_options_tab == 5) ?
							g_opts.theme_idx :
							0;
			}
			break;
		case KEY_DOWN:
			g_options_sel++;
			if (g_options_sel > max_sel)
				g_options_sel = (g_options_tab == 6) ? 1 : 0;
			if (g_options_tab == 6 && menu_to_mact(g_options_sel) == -1)
				g_options_sel++; /* skip header */
			if (g_options_tab == 5)
				apply_theme(g_options_sel);
			break;
		case KEY_UP:
			g_options_sel--;
			if (g_options_sel < ((g_options_tab == 6) ? 1 : 0))
				g_options_sel = max_sel;
			if (g_options_tab == 6 && menu_to_mact(g_options_sel) == -1)
				g_options_sel--; /* skip header backward */
			if (g_options_sel < 1 && g_options_tab == 6)
				g_options_sel = max_sel;
			if (g_options_tab == 5)
				apply_theme(g_options_sel);
			break;
		case 'k':
			if (g_options_tab == 6 && !g_midi_learn_jog_pair &&
			    !g_midi_learn_active) {
				if (g_midi_ndevices > 0)
					g_midi_dev_sel = (g_midi_dev_sel +
							  g_midi_ndevices - 1) %
							 g_midi_ndevices;
			} else if (g_options_tab == 7 && !g_motor_probe_open) {
				if (g_options_out_sel > 0)
					g_options_out_sel--;
			} else if (g_options_tab == 1) {
				if (g_pcm_ndevices > 0)
					g_pcm_dev_sel = (g_pcm_dev_sel +
							 g_pcm_ndevices - 1) %
							g_pcm_ndevices;
			} else if (g_options_tab != 6 && g_options_tab != 7) {
				int max_sel_k =
					(g_options_tab == 0) ? 0 :
					(g_options_tab == 1) ? 3 :
					(g_options_tab == 2) ? 3 :
					(g_options_tab == 3) ? 5 :
					(g_options_tab == 4) ? 4 :
							       THEME_COUNT - 1;
				g_options_sel--;
				if (g_options_sel < 0)
					g_options_sel = max_sel_k;
				if (g_options_tab == 5)
					apply_theme(g_options_sel);
			}
			break;
		case '-':
		case KEY_SLEFT:
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open) {
				/* -  adjusts d1 (CC/note number) */
				if (g_motor_probe_cc > 0)
					g_motor_probe_cc--;
			} else {
				options_adjust(-1);
			}
			break;
		case '=':
		case '+':
		case KEY_SRIGHT:
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open) {
				if (g_motor_probe_cc < 127)
					g_motor_probe_cc++;
			} else {
				options_adjust(+1);
			}
			break;
		case ',':
		case '<':
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open) {
				if (g_motor_probe_val > 0)
					g_motor_probe_val--;
			} else if (g_options_tab == 8) {
				/* FX tab: < adjusts param0 down */
				int row = g_options_sel,
				    mr = g_num_tracks * FX_SLOTS_PER_DECK;
				FXSlot *fx =
					(row < mr) ?
						fx_slot(row / FX_SLOTS_PER_DECK,
							row % FX_SLOTS_PER_DECK) :
						fx_master();
				fx->params[0] -= 0.05f;
				if (fx->params[0] < 0.0f)
					fx->params[0] = 0.0f;
			}
			break;
		case '.':
		case '>':
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open) {
				if (g_motor_probe_val < 127)
					g_motor_probe_val++;
			} else if (g_options_tab == 8) {
				/* FX tab: > adjusts param0 up */
				int row = g_options_sel,
				    mr = g_num_tracks * FX_SLOTS_PER_DECK;
				FXSlot *fx =
					(row < mr) ?
						fx_slot(row / FX_SLOTS_PER_DECK,
							row % FX_SLOTS_PER_DECK) :
						fx_master();
				fx->params[0] += 0.05f;
				if (fx->params[0] > 1.0f)
					fx->params[0] = 1.0f;
			}
			break;
		/* T = cycle message type (CC → NoteOn → NoteOff → CC) */
		case 't':
		case 'T':
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open)
				g_motor_probe_type =
					(g_motor_probe_type + 1) % 3;
			break;
		/* V = send current d2 value (for testing specific stop values) */
		case 'v':
		case 'V':
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open)
				motor_probe_send(g_motor_probe_val);
			break;
		/* X = sweep d2 values 0→127, then 127→0 to find stop signal */
		case 'x':
		case 'X':
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open) {
				snprintf(
					g_motor_probe_log,
					sizeof(g_motor_probe_log),
					"Sweeping 0→127 on ch%d CC%d... (listen for stop)",
					g_motor_probe_ch, g_motor_probe_cc);
				motor_probe_sweep(0, 127);
			}
			break;
		case '\n':
		case KEY_ENTER:
			/* During jog learn: ENTER commits with whatever has been captured so far */
			if (g_midi_learn_jog_pair) {
				int anything =
					(g_midi_learn_jog_spin_status !=
					 0) /* CC spin */
					|| (g_midi_learn_jog_step &
					    2) /* pitch bend */
					||
					(g_midi_learn_jog_step & 1); /* touch */
				if (anything) {
					g_midi_learn_jog_pair = 0;
					g_midi_learn_jog_step = 0;
					g_midi_learn_jog_spin_status = 0;
					g_midi_learn_jog_spin_d1 = 0;
					g_midi_learn_active = 0;
					midi_map_save();
				}
				break; /* if nothing captured yet, do nothing */
			}
			/* On theme tab: ENTER commits the currently-previewed theme */
			if (g_options_tab == 5) {
				g_opts.theme_idx = g_options_sel;
				apply_theme(g_opts.theme_idx);
				settings_save();
			}
			/* On AUDIO tab: ENTER switches to selected PCM device */
			if (g_options_tab == 1) {
				pcm_open_device(g_pcm_dev_sel);
				break;
			}
			/* On MIDI tab: ENTER switches to selected device (when not in learn mode) */
			if (g_options_tab == 6 && !g_midi_learn_active &&
			    !g_midi_learn_jog_pair) {
				int mact = menu_to_mact(g_options_sel);
				if (mact == -2) {
					/* Panic */
					for (int i = 0; i < MAX_TRACKS; i++) {
						pthread_mutex_lock(&g_tracks[i].lock);
						g_tracks[i].volume = 1.0f;
						g_tracks[i].eq_low = 0.0f;
						g_tracks[i].eq_mid = 0.0f;
						g_tracks[i].eq_high = 0.0f;
						pthread_mutex_unlock(&g_tracks[i].lock);
					}
					g_crossfader = 0.5f;
				} else if (g_options_sel <= g_midi_ndevices && g_options_sel > 0) {
					midi_open_device(g_midi_dev_sel);
				}
				break;
			}
			break;
		/* MIDI tab: Learn, Unbind, Save */
		case 'l':
		case 'L':
			if (g_options_tab == 6) {
				int mact = menu_to_mact(g_options_sel);
				if (mact >= 1) {
					g_midi_learn_active = 1;
					g_midi_learn_sel = mact;
					g_midi_learn_jog_pair = 0;
					g_midi_learn_jog_step = 0;
				}
			}
			break;
		/* J = jog learn / monitor toggle / device-list navigation depending on context */
		case 'j':
		case 'J':
			if (g_options_tab == 6) {
				if (g_midi_learn_jog_pair) {
					/* J again during jog learn: commit if spin captured */
					if (g_midi_learn_jog_spin_status != 0) {
						g_midi_learn_jog_pair = 0;
						g_midi_learn_jog_step = 0;
						g_midi_learn_jog_spin_status =
							0;
						g_midi_learn_jog_spin_d1 = 0;
						g_midi_learn_active = 0;
						midi_map_save();
					}
				} else if (g_midi_learn_active) {
					/* in single learn — ignore J */
				} else {
					/* Toggle MIDI monitor panel */
					g_midi_mon_open ^= 1;
					if (g_midi_mon_open) {
						g_midi_mon_count =
							0; /* clear buffer on open for fresh capture */
						g_midi_mon_head = 0;
					}
				}
			} else if (g_options_tab == 7 && !g_motor_probe_open) {
				/* MIDI OUT tab: navigate output binding list down */
				if (g_options_out_sel <
				    g_midi_nout_bindings - 1)
					g_options_out_sel++;
			} else if (g_options_tab == 1) {
				/* AUDIO tab: navigate PCM device list down */
				if (g_pcm_ndevices > 0)
					g_pcm_dev_sel = (g_pcm_dev_sel + 1) %
							g_pcm_ndevices;
			} else if (g_options_tab != 6 && g_options_tab != 7) {
				/* Other tabs: navigate rows down */
				int max_sel_j =
					(g_options_tab == 0) ?
						0 :
					(g_options_tab == 2) ?
						3 :
					(g_options_tab == 3) ?
						5 :
					(g_options_tab == 4) ?
						3 :
					(g_options_tab == 8) ?
						g_num_tracks *
							FX_SLOTS_PER_DECK :
						THEME_COUNT - 1;
				g_options_sel++;
				if (g_options_sel > max_sel_j)
					g_options_sel = 0;
				if (g_options_tab == 5)
					apply_theme(g_options_sel);
			}
			break;
		/* R = rescan MIDI devices (MIDI IN/OUT tab) or PCM devices (AUDIO tab) */
		case 'r':
		case 'R':
			if (g_options_tab == 6 || g_options_tab == 7) {
				midi_enumerate_devices();
				if (g_midi_dev_sel >= g_midi_ndevices)
					g_midi_dev_sel =
						g_midi_ndevices > 0 ?
							g_midi_ndevices - 1 :
							0;
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "MIDI: found %d device%s",
					 g_midi_ndevices,
					 g_midi_ndevices == 1 ? "" : "s");
			} else if (g_options_tab == 1) {
				pcm_enumerate_devices();
				if (g_pcm_dev_sel >= g_pcm_ndevices)
					g_pcm_dev_sel =
						g_pcm_ndevices > 0 ?
							g_pcm_ndevices - 1 :
							0;
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Audio: found %d device%s",
					 g_pcm_ndevices,
					 g_pcm_ndevices == 1 ? "" : "s");
			}
			break;
		/* W = start guided jog wheel learn (J is now device navigation) */
		case 'w':
		case 'W':
			if (g_options_tab == 6 && !g_midi_learn_active) {
				int jdeck = 0;
				MidiAction sel_act = (MidiAction)g_options_sel;
				if (sel_act >= MACT_JOG_SPIN_A &&
				    sel_act <= MACT_JOG_SPIN_D)
					jdeck = sel_act - MACT_JOG_SPIN_A;
				else if (sel_act >= MACT_JOG_TOUCH_A &&
					 sel_act <= MACT_JOG_TOUCH_D)
					jdeck = sel_act - MACT_JOG_TOUCH_A;
				else if (sel_act >= MACT_JOG_PB_A &&
					 sel_act <= MACT_JOG_PB_D)
					jdeck = sel_act - MACT_JOG_PB_A;
				else
					jdeck = 0;
				int anything =
					(g_midi_learn_jog_spin_status != 0) ||
					(g_midi_learn_jog_step & 2) ||
					(g_midi_learn_jog_step & 1);
				if (g_midi_learn_jog_pair && anything) {
					/* W again = commit whatever was captured */
					g_midi_learn_jog_pair = 0;
					g_midi_learn_jog_step = 0;
					g_midi_learn_jog_spin_status = 0;
					g_midi_learn_jog_spin_d1 = 0;
					midi_map_save();
				} else {
					g_midi_learn_jog_pair = 1;
					g_midi_learn_jog_step = 0;
					g_midi_learn_jog_deck = jdeck;
					g_midi_learn_jog_spin_status = 0;
					g_midi_learn_jog_spin_d1 = 0;
					g_midi_learn_active = 0;
				}
			}
			break;
		/* U = unbind selected action */
		case 'u':
		case 'U':
			if (g_options_tab == 6) {
				int mact = menu_to_mact(g_options_sel);
				if (mact >= 1) {
					midi_bind(0, 0, (MidiAction)mact);
					g_midi_learn_active = 0;
				}
			}
			break;
		/* M = toggle motor probe panel (MIDI tab only) */
		case 'm':
		case 'M':
			if (g_options_tab == 6 || g_options_tab == 7) {
				g_motor_probe_open ^= 1;
				if (g_motor_probe_open) {
					g_midi_learn_active = 0;
					g_midi_learn_jog_pair = 0;
					/* Switch to MIDI OUT tab where probe lives */
					if (g_options_tab == 6)
						g_options_tab = 7;
				}
			}
			break;

		/* Motor probe: SPACE = start motor (127), BACKSPACE = stop (0) */
		case ' ':
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open)
				motor_probe_send(127);
			break;
		case KEY_BACKSPACE:
		case 127:
		case '\b':
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open)
				motor_probe_send(0);
			break;
		case 's':
		case 'S':
			if (g_options_tab == 6 || g_options_tab == 7)
				midi_map_save();
			settings_save();
			break;
		case 'Q':
			quit_confirm();
			break;
		}
		return; /* swallow all other keys when options is open */
	}

	/* ── ESC opens options ── */
	if (c == 27) {
		g_options_open = 1;
		g_options_tab = 0;
		g_options_sel = 0;
		return;
	}

	switch (c) {
	/* Deck selection */
	case KEY_DECK_A:
		g_active_track = 0;
		break;
	case KEY_DECK_B:
		g_active_track = 1;
		break;
	case KEY_DECK_C:
		if (g_num_tracks >= 3)
			g_active_track = 2;
		break;
	case KEY_DECK_D:
		if (g_num_tracks >= 4)
			g_active_track = 3;
		break;

	/* Headphone Cue (PFL) toggles */
	case KEY_HEADPHONE_CUE_A:
		g_tracks[0].cue_active ^= 1;
		break;
	case KEY_HEADPHONE_CUE_B:
		g_tracks[1].cue_active ^= 1;
		break;
	case KEY_HEADPHONE_CUE_C:
		if (g_num_tracks >= 3)
			g_tracks[2].cue_active ^= 1;
		break;
	case KEY_HEADPHONE_CUE_D:
		if (g_num_tracks >= 4)
			g_tracks[3].cue_active ^= 1;
		break;

	/* Play / Pause */
	case ' ':
		if (t->loaded) {
			int was_playing = t->playing;
			t->pending_play = 0;
			if (!was_playing) {
				int can_quantize =
					g_opts.sync_quantize &&
					t->sync_locked && g_sync_master >= 0 &&
					g_sync_master != g_active_track &&
					g_tracks[g_sync_master].playing &&
					g_tracks[g_sync_master].bpm > 1.0f;
				if (can_quantize) {
					t->pending_play = 1;
				} else {
					t->playing = 1;
					if (!g_slip_motor_off[g_active_track])
						motor_set(g_active_track, 1);
				}
			} else {
				t->playing = 0;
				motor_set(g_active_track, 0);
			}
			if (g_gang_mode)
				for (int i = 0; i < g_num_tracks; i++)
					if ((g_gang_mask & (1 << i)) &&
					    i != g_active_track &&
					    g_tracks[i].loaded) {
						g_tracks[i].pending_play = 0;
						g_tracks[i].playing =
							t->playing;
						if (t->playing &&
						    !g_slip_motor_off[i])
							motor_set(i, 1);
						else if (!t->playing)
							motor_set(i, 0);
					}
		}
		break;

	/* Stop */
	case 's':
		t->playing = 0;
		t->pending_play = 0;
		motor_set(g_active_track, 0);
		if (g_gang_mode)
			for (int i = 0; i < g_num_tracks; i++)
				if ((g_gang_mask & (1 << i)) &&
				    i != g_active_track) {
					g_tracks[i].playing = 0;
					g_tracks[i].pending_play = 0;
					motor_set(i, 0);
				}
		break;

	/* Restart / Cue */
	case 'r':
		pthread_mutex_lock(&t->lock);
		t->pos = 0;
		pthread_mutex_unlock(&t->lock);
		if (g_gang_mode)
			for (int i = 0; i < g_num_tracks; i++)
				if ((g_gang_mask & (1 << i)) &&
				    i != g_active_track) {
					pthread_mutex_lock(&g_tracks[i].lock);
					g_tracks[i].pos = 0;
					pthread_mutex_unlock(&g_tracks[i].lock);
				}
		break;

	/* Loop toggle */
	case 'l':
		t->looping = !t->looping;
		if (g_gang_mode)
			for (int i = 0; i < g_num_tracks; i++)
				if ((g_gang_mask & (1 << i)) &&
				    i != g_active_track)
					g_tracks[i].looping = t->looping;
		break;

	/* ── BPM detect ── */
	/* Ctrl+T = TAP BPM */
	case 20:
		tap_bpm(g_active_track);
		break;
	/* s = Stop, S = Snap Grid */
	case 'S':
		if (g_options_tab == 6 || g_options_tab == 7) {
			midi_map_save();
			settings_save();
		} else {
			snap_grid(g_active_track);
		}
		break;
	case 'b':
		if (t->loaded) {
			/* Re-analysis via background worker — just re-enqueue the same file.
             * The worker will call detect_bpm_and_offset and overwrite the cache. */
			pthread_mutex_lock(&g_load_mutex);
			g_load_job.deck = g_active_track;
			snprintf(g_load_job.path, sizeof(g_load_job.path), "%s",
				 t->filename);
			g_load_job.path[sizeof(g_load_job.path) - 1] = '\0';
			g_load_job.valid = 1;
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Re-analyzing Deck %c...",
				 DECK_NUM(g_active_track));
			pthread_cond_signal(&g_load_cond);
			pthread_mutex_unlock(&g_load_mutex);
		}
		break;

	/* Manual BPM entry: B = type a BPM value for the active deck.
     * Enter to confirm, Esc to cancel, Backspace to delete.
     * Useful when auto-detection is wrong and you know the BPM already. */
	case 'B':
		if (t->loaded) {
			g_bpm_entry = 1;
			g_bpm_deck = g_active_track;
			g_bpm_buf[0] = '\0';
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "BPM Deck %c: _  (Enter=confirm  Esc=cancel)",
				 DECK_NUM(g_active_track));
		}
		break;

	/* BPM display cycle: H steps through normal → ×2 → ½ → normal ...
     * Affects only the displayed BPM value and beat ruler spacing.
     * Playback speed, pitch, and the underlying bpm/bpm_offset are untouched. */
	case 'H':
		if (t->loaded && t->bpm > 1.0f) {
			/* Cycle: 0 (normal) → 1 (×2) → -1 (½) → 0 … */
			if (t->bpm_display_double == 0)
				t->bpm_display_double = 1;
			else if (t->bpm_display_double == 1)
				t->bpm_display_double = -1;
			else
				t->bpm_display_double = 0;
		}
		break;

	/* Key lock (master tempo): K toggles pitch-preserving time-stretch.
     * When ON, pitch stays constant regardless of playback speed.
     * Uses phase vocoder — costs ~4-6% extra CPU per deck on G4 (scalar). */
	case 'K':
		if (t->loaded) {
			t->key_lock = !t->key_lock;
			/* Reset stretcher state on toggle so there's no glitch */
			int di = (int)(t - g_tracks);
			if (di >= 0 && di < MAX_TRACKS)
				wsola_reset(&g_wsola[di], t->pos);
		}
		break;

	/* ── Sync lock ── */
	/* M = make active deck the sync master */
	case 'M':
		if (t->loaded && t->bpm > 0) {
			g_sync_master = g_active_track;
			t->sync_locked = 0; /* master is never a slave */
			/* Re-sync all locked slaves */
			for (int i = 0; i < g_num_tracks; i++)
				if (i != g_sync_master &&
				    g_tracks[i].sync_locked)
					sync_apply(i);
		}
		break;
	/* y = toggle sync lock on active deck (slave to master) */
	case 'y':
		if (t->loaded && t->bpm > 0 &&
		    g_active_track != g_sync_master) {
			t->sync_locked = !t->sync_locked;
			if (t->sync_locked)
				sync_apply(g_active_track);
		}
		break;

	/* ── Beat nudge ── accumulates on repeat presses, decays to 0 ── */
	/* nudge forward: ] */
	case ']':
		t->nudge += NUDGE_AMOUNT;
		if (t->nudge > NUDGE_AMOUNT * 8)
			t->nudge = NUDGE_AMOUNT * 8;
		/* Gang: apply to all gang members too */
		if (g_gang_mode)
			for (int i = 0; i < g_num_tracks; i++)
				if ((g_gang_mask & (1 << i)) &&
				    i != g_active_track) {
					g_tracks[i].nudge += NUDGE_AMOUNT;
					if (g_tracks[i].nudge >
					    NUDGE_AMOUNT * 8)
						g_tracks[i].nudge =
							NUDGE_AMOUNT * 8;
				}
		break;
	/* nudge back: [ */
	case '[':
		t->nudge -= NUDGE_AMOUNT;
		if (t->nudge < -NUDGE_AMOUNT * 8)
			t->nudge = -NUDGE_AMOUNT * 8;
		if (g_gang_mode)
			for (int i = 0; i < g_num_tracks; i++)
				if ((g_gang_mask & (1 << i)) &&
				    i != g_active_track) {
					g_tracks[i].nudge -= NUDGE_AMOUNT;
					if (g_tracks[i].nudge <
					    -NUDGE_AMOUNT * 8)
						g_tracks[i].nudge =
							-NUDGE_AMOUNT * 8;
				}
		break;

	/* ── Coarse seek: arrow keys ─────────────────────────────────
     *  LEFT  / RIGHT       ±1 beat
     *  SHIFT+LEFT  (KEY_SLEFT)  / SHIFT+RIGHT (KEY_SRIGHT)  ±8 beats (2 bars)
     *
     *  Use these to align beats by ear when auto-sync isn't
     *  quite right.  The playhead jumps instantly; no audio
     *  artifact beyond a brief discontinuity.
     * ────────────────────────────────────────────────────────── */
	case KEY_LEFT:
	case KEY_RIGHT:
	case KEY_SLEFT:
	case KEY_SRIGHT: {
		if (g_view == 2)
			break; /* help view: don't seek */
		if (!t->loaded)
			break;
		float beat_frames =
			(t->bpm > 0.0f) ? (float)g_actual_sample_rate * 60.0f /
						  (t->bpm * t->pitch) :
					  (float)g_actual_sample_rate *
						  0.5f; /* fallback: 0.5 s */

		int beats = (c == KEY_SLEFT || c == KEY_SRIGHT) ? 8 : 1;
		int sign = (c == KEY_RIGHT || c == KEY_SRIGHT) ? +1 : -1;

		int64_t delta = (int64_t)(sign * beats * beat_frames);
		int64_t newpos = (int64_t)t->pos + delta;
		if (newpos < 0)
			newpos = 0;
		if (newpos >= (int64_t)t->num_frames)
			newpos = (int64_t)(t->num_frames - 1);
		t->pos = (uint32_t)newpos;
		break;
	}

	/* ── Beat grid offset ──────────────────────────────────────────────────
     * { / }  shift the beat grid by ±1 beat (one full beat-length in frames).
     * This moves every beat marker and bar number left or right by exactly
     * one beat, letting you re-align the grid when the auto-detected phase
     * is off by one or more beats.
     *
     * Previous code shifted by ±1 *frame* per keypress — at 128 BPM a beat
     * is ~20 000 frames, making the control appear completely non-functional.
     * ──────────────────────────────────────────────────────────────────── */
	case '{':
		if (t->loaded && t->bpm > 1.0f) {
			float beat_f =
				(float)g_actual_sample_rate * 60.0f / t->bpm;
			t->bpm_offset -= beat_f;
			if (t->bpm_offset < 0.0f)
				t->bpm_offset = 0.0f;
		}
		break;
	case '}':
		if (t->loaded && t->bpm > 1.0f) {
			float beat_f =
				(float)g_actual_sample_rate * 60.0f / t->bpm;
			t->bpm_offset += beat_f;
		}
		break;

	/* ── Beat grid fine adjustment: ( / ) = ±¼ beat ───────────────────────
     * One quarter-beat = one 16th-note position.  Use these when the grid
     * is close but drifting — a full-beat jump overshoots but a nudge with
     * ] / [ only shifts playback pitch, not the grid anchor.
     * ──────────────────────────────────────────────────────────────────── */
	case '(':
		if (t->loaded && t->bpm > 1.0f) {
			float quarter_f = (float)g_actual_sample_rate * 60.0f /
					  t->bpm / 4.0f;
			t->bpm_offset -= quarter_f;
			if (t->bpm_offset < 0.0f)
				t->bpm_offset = 0.0f;
		}
		break;
	case ')':
		if (t->loaded && t->bpm > 1.0f) {
			float quarter_f = (float)g_actual_sample_rate * 60.0f /
					  t->bpm / 4.0f;
			t->bpm_offset += quarter_f;
		}
		break;

	/* ── Beat grid reset: Z ────────────────────────────────────────────────
     * Resets bpm_offset to 0 (the very first sample), which is the state
     * after a fresh autocorrelation analysis before the onset scan refines
     * it.  Useful when grid adjustments have drifted far off or when loading
     * a track whose first beat truly starts at frame 0 (e.g. a loop file).
     * The BPM itself is unchanged; only the phase anchor is cleared.
     * ──────────────────────────────────────────────────────────────────── */
	case 'Z':
		if (t->loaded) {
			t->bpm_offset = 0.0f;
			cache_save(t);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Beat grid reset → Deck %c",
				 DECK_NUM(g_active_track));
		}
		break;

	/* ── Gang mode ── */
	/* G = toggle gang mode on/off */
	case 'G':
		g_gang_mode = !g_gang_mode;
		if (g_gang_mode && g_gang_mask == 0)
			g_gang_mask = (1 << g_num_tracks) -
				      1; /* default: all decks */
		break;

	/* C = toggle crate jump mode */
	case 'C':
		g_crate_jump_active = !g_crate_jump_active;
		if (g_crate_jump_active) {
			g_crate_input[0] = '\0';
			if (g_ncrate > 0) {
				char list[256] = "Crates: ";
				for (int i = 0; i < g_ncrate; i++) {
					strncat(list, g_crates[i].alias, sizeof(list) - strlen(list) - 1);
					if (i < g_ncrate - 1)
						strncat(list, ", ", sizeof(list) - strlen(list) - 1);
				}
				snprintf(g_fb_status, sizeof(g_fb_status), "%s | Jump to: _", list);
			} else {
				snprintf(g_fb_status, sizeof(g_fb_status), "No crates found in crates.txt | Jump to: _");
			}
		} else {
			snprintf(g_fb_status, sizeof(g_fb_status), "Crate jump cancelled");
		}
		break;
	/* F1-F4 = toggle individual decks in/out of gang */
	case KEY_F(1):
		g_gang_mask ^= (1 << 0);
		break;
	case KEY_F(2):
		g_gang_mask ^= (1 << 1);
		break;
	case KEY_F(3):
		g_gang_mask ^= (1 << 2);
		break;
	case KEY_F(4):
		g_gang_mask ^= (1 << 3);
		break;

	/* ── Cue points ── */
	/* Shift+F1–F4 = set cue,  F5–F8 = jump to cue */
	case KEY_F(5):
	case KEY_F(6):
	case KEY_F(7):
	case KEY_F(8): {
		int ci = c - KEY_F(5);
		if (ci >= 0 && ci < MAX_CUES) {
			t->cue[ci] = t->pos;
			t->cue_set[ci] = 1;
			cache_save(t);
			/* Light cue LED if bound */
			{
				char side = (g_active_track == g_side_deck[0]) ?
						    'a' :
						    'b';
				char ln[32];
				snprintf(ln, sizeof(ln), "led_cue_%d_%c",
					 ci + 1, side);
				led_on(ln);
			}
			pad_leds_refresh(g_active_track);
		}
		break;
	}
	case KEY_F(9):
	case KEY_F(10):
	case KEY_F(11):
	case KEY_F(12): {
		int ci = c - KEY_F(9);
		if (ci >= 0 && ci < MAX_CUES && t->cue_set[ci]) {
			pthread_mutex_lock(&t->lock);
			t->pos = t->cue[ci];
			pthread_mutex_unlock(&t->lock);
		}
		break;
	}

	/* ── Auto-gain toggle ── recalc gain or reset to 1.0 ── */
	case 'A':
		if (t->loaded) {
			/* Toggle: if gain != 1.0 reset, else recalculate */
			if (fabsf(t->gain - 1.0f) < 0.01f)
				t->gain =
					calc_auto_gain(t->data, t->num_frames);
			else
				t->gain = 1.0f;
		}
		break;

	/* Pitch control */
	case 'e':
		t->pitch += 0.005f;
		if (t->pitch > 2.0f)
			t->pitch = 2.0f;
		break;
	case 'd':
		t->pitch -= 0.005f;
		if (t->pitch < 0.5f)
			t->pitch = 0.5f;
		break;
	case 'E':
		t->pitch += 0.05f;
		if (t->pitch > 2.0f)
			t->pitch = 2.0f;
		break;
	case 'D':
		t->pitch -= 0.05f;
		if (t->pitch < 0.5f)
			t->pitch = 0.5f;
		break;
	case '0':
		t->pitch = 1.0f;
		break;

	/* V = cycle pitch range for active deck: ±8% → ±25% → ±50% → ±8%
     * Affects the MIDI pitch fader span only; keyboard pitch steps are unchanged. */
	case 'V':
		g_pitch_range[g_active_track] =
			(g_pitch_range[g_active_track] + 1) % 3;
		snprintf(g_fb_status, sizeof(g_fb_status),
			 "Deck %c pitch range: %s", DECK_NUM(g_active_track),
			 g_pitch_range_names[g_pitch_range[g_active_track]]);
		break;

	/* Volume */
	case '+':
	case '=':
		t->volume += 0.05f;
		if (t->volume > 1.0f)
			t->volume = 1.0f;
		break;
	case '-':
		t->volume -= 0.05f;
		if (t->volume < 0.0f)
			t->volume = 0.0f;
		break;

	/* EQ */
	case 'q':
		t->eq_low += 0.1f;
		if (t->eq_low > 1.0f)
			t->eq_low = 1.0f;
		break;
	case 'a':
		t->eq_low -= 0.1f;
		if (t->eq_low < -1.0f)
			t->eq_low = -1.0f;
		break;
	case 'w':
		t->eq_mid += 0.1f;
		if (t->eq_mid > 1.0f)
			t->eq_mid = 1.0f;
		break;
	case 'x':
		t->eq_mid -= 0.1f;
		if (t->eq_mid < -1.0f)
			t->eq_mid = -1.0f;
		break;
	case 't':
		t->eq_high += 0.1f;
		if (t->eq_high > 1.0f)
			t->eq_high = 1.0f;
		break;
	case 'g':
		t->eq_high -= 0.1f;
		if (t->eq_high < -1.0f)
			t->eq_high = -1.0f;
		break;

	/* Crossfader */
	case '<':
	case ',':
		g_crossfader -= 0.05f;
		if (g_crossfader < 0.0f)
			g_crossfader = 0.0f;
		break;
	case '>':
	case '.':
		g_crossfader += 0.05f;
		if (g_crossfader > 1.0f)
			g_crossfader = 1.0f;
		break;

	/* Master volume */
	case 'm':
		g_master_vol += 5;
		if (g_master_vol > 150)
			g_master_vol = 150;
		g_opts.default_master_vol = g_master_vol;
		break;
	case 'n':
		g_master_vol -= 5;
		if (g_master_vol < 0)
			g_master_vol = 0;
		g_opts.default_master_vol = g_master_vol;
		break;

	/* View switch */
	case '\t':
		if (g_num_tracks <= 2) {
			/* 2-deck mode: TAB cycles the bottom panel (browser/playlist/library)
             * The bottom pane is always visible — never goes blank. */
			g_panel = (g_panel + 1) % 3;
			g_view = 1; /* always stay in split view */
		} else {
			/* 4-deck mode: TAB toggles split view on/off (screen space tight) */
			g_view = (g_view == 1) ? 0 : 1;
		}
		g_tag_info.visible = 0;
		break;
	case '?':
		g_tag_info.visible = 0;
		g_view = (g_view == 2) ? 1 : 2;
		break;
	case 'P':
		g_tag_info.visible = 0;
		g_panel = (g_panel + 1) % 3;
		if (g_view != 1)
			g_view = 1; /* ensure split view is showing */
		break;

	/* ── File browser / playlist navigation ── */
	case 'j':
	case KEY_DOWN:
		if (g_view == 1 && g_panel == 0 && g_fb_count > 0)
			g_fb_sel = (g_fb_sel + 1) % g_fb_count;
		else if (g_view == 1 && g_panel == 1 && g_pl_count > 0)
			g_pl_sel = (g_pl_sel + 1) % g_pl_count;
		else if (g_view == 1 && g_panel == 2 && g_lib_count > 0)
			g_lib_sel = (g_lib_sel + 1) % g_lib_count;
		break;
	case 'k':
	case KEY_UP:
		if (g_view == 1 && g_panel == 0 && g_fb_count > 0)
			g_fb_sel = (g_fb_sel + g_fb_count - 1) % g_fb_count;
		else if (g_view == 1 && g_panel == 1 && g_pl_count > 0)
			g_pl_sel = (g_pl_sel + g_pl_count - 1) % g_pl_count;
		else if (g_view == 1 && g_panel == 2 && g_lib_count > 0)
			g_lib_sel = (g_lib_sel + g_lib_count - 1) % g_lib_count;
		break;
	case KEY_NPAGE:
		if (g_view == 1 && g_panel == 0) {
			g_fb_sel += (g_rows - 6);
			if (g_fb_sel >= g_fb_count)
				g_fb_sel = g_fb_count - 1;
		} else if (g_view == 1 && g_panel == 1) {
			g_pl_sel += (g_rows - 6);
			if (g_pl_sel >= g_pl_count)
				g_pl_sel = g_pl_count - 1;
		} else if (g_view == 1 && g_panel == 2) {
			g_lib_sel += (g_rows - 6);
			if (g_lib_sel >= g_lib_count)
				g_lib_sel =
					g_lib_count > 0 ? g_lib_count - 1 : 0;
		}
		break;
	case KEY_PPAGE:
		if (g_view == 1 && g_panel == 0) {
			g_fb_sel -= (g_rows - 6);
			if (g_fb_sel < 0)
				g_fb_sel = 0;
		} else if (g_view == 1 && g_panel == 1) {
			g_pl_sel -= (g_rows - 6);
			if (g_pl_sel < 0)
				g_pl_sel = 0;
		} else if (g_view == 1 && g_panel == 2) {
			g_lib_sel -= (g_rows - 6);
			if (g_lib_sel < 0)
				g_lib_sel = 0;
		}
		break;

	/* Enter: open dir / load file (browser) or load from playlist */
	case '\n':
	case KEY_ENTER:
		if (g_tag_info.visible) {
			g_tag_info.visible = 0;
			break;
		}
		if (g_view == 1 && g_panel == 0 && g_fb_count > 0) {
			FBEntry *e = &g_fb_entries[g_fb_sel];
			if (e->is_dir) {
				fb_enter_dir(e->name);
			} else {
				char full[FB_PATH_MAX + 256];
				fb_selected_path(full, sizeof(full));
				enqueue_load(g_active_track, full);
				if (g_num_tracks > 2)
					g_view = 0;
			}
		} else if (g_view == 1 && g_panel == 1 && g_pl_count > 0) {
			enqueue_load(g_active_track, g_pl[g_pl_sel].path);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Loaded \u2192 Deck %c", 'A' + g_active_track);
		} else if (g_view == 1 && g_panel == 2 && g_lib &&
			   g_lib_count > 0) {
			enqueue_load(g_active_track, g_lib[g_lib_sel].path);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Loaded \u2192 Deck %c", 'A' + g_active_track);
		}
		break;

	/* Backspace = go up one directory (browser only) */
	case KEY_BACKSPACE:
	case 127:
	case '\b':
		if (g_tag_info.visible) {
			g_tag_info.visible = 0;
			break;
		}
		if (g_view == 1 && g_panel == 0)
			fb_enter_dir("..");
		break;

	/* !@#$ = load to specific deck */
	case '!':
	case '@':
	case '#':
	case '$': {
		if (g_tag_info.visible) {
			g_tag_info.visible = 0;
			break;
		}
		int deck = (c == '!') ? 0 : (c == '@') ? 1 : (c == '#') ? 2 : 3;
		if (deck >= g_num_tracks)
			break;
		if (g_view == 1 && g_panel == 0 && g_fb_count > 0 &&
		    !g_fb_entries[g_fb_sel].is_dir) {
			char full[FB_PATH_MAX + 256];
			fb_selected_path(full, sizeof(full));
			enqueue_load(deck, full);
		} else if (g_view == 1 && g_panel == 1 && g_pl_count > 0) {
			enqueue_load(deck, g_pl[g_pl_sel].path);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Loaded \u2192 Deck %c", 'A' + deck);
		} else if (g_view == 1 && g_panel == 2 && g_lib &&
			   g_lib_count > 0) {
			enqueue_load(deck, g_lib[g_lib_sel].path);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Loaded \u2192 Deck %c", 'A' + deck);
		}
		break;
	}

	/* p = add selected browser/library entry to playlist */
	case 'p':
		if (g_view == 1 && g_panel == 0 && g_fb_count > 0 &&
		    !g_fb_entries[g_fb_sel].is_dir) {
			char full[FB_PATH_MAX + 256];
			fb_selected_path(full, sizeof(full));
			FBEntry *e = &g_fb_entries[g_fb_sel];
			pl_add(full, e->name, e->bpm);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "+Playlist [%d]", g_pl_count);
		} else if (g_view == 1 && g_panel == 2 && g_lib &&
			   g_lib_count > 0) {
			LIBEntry *e = &g_lib[g_lib_sel];
			pl_add(e->path, e->name, e->bpm);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "+Playlist [%d]", g_pl_count);
		}
		break;

	/* O = cycle sort order (browser or library) */
	case 'O':
		if (g_view == 1 && g_panel == 0) {
			g_fb_sort = (g_fb_sort + 1) % 3;
			fb_apply_sort();
			g_fb_sel = 0;
			g_fb_scroll = 0;
		} else if (g_view == 1 && g_panel == 2) {
			g_lib_sort = (g_lib_sort + 1) % 3;
			lib_apply_sort();
			g_lib_sel = 0;
			g_lib_scroll = 0;
		}
		break;

	/* L = scan library from current browser dir (or rescan if panel==2) */
	case 'L':
		if (g_view == 1 && !g_lib_scanning) {
			const char *root = (g_panel == 2 && g_lib_root[0]) ?
						   g_lib_root :
						   g_fb_path;
			lib_start_scan(root);
			g_panel = 2;
			if (g_view != 1)
				g_view = 1;
		}
		break;

	/* i = MusicBrainz tag lookup */
	case 'i':
		if (g_tag_info.visible) {
			g_tag_info.visible = 0;
			break;
		}
		if (g_view == 1 && g_panel == 0 && g_fb_count > 0 &&
		    !g_fb_entries[g_fb_sel].is_dir)
			tag_lookup_start(g_fb_entries[g_fb_sel].name);
		else if (g_view == 1 && g_panel == 1 && g_pl_count > 0)
			tag_lookup_start(g_pl[g_pl_sel].name);
		else if (g_view == 1 && g_panel == 2 && g_lib &&
			 g_lib_count > 0)
			tag_lookup_start(g_lib[g_lib_sel].name);
		break;

	/* DEL = remove from playlist */
	case KEY_DC:
		if (g_view == 1 && g_panel == 1 && g_pl_count > 0)
			pl_remove(g_pl_sel);
		break;

	/* C-x = clear entire playlist */
	case 24:
		if (g_view == 1 && g_panel == 1) {
			g_pl_count = 0;
			g_pl_sel = 0;
			g_pl_scroll = 0;
		}
		break;

	/* Open browser at home dir */
	case '~': {
		const char *home = getenv("HOME");
		if (home) {
			snprintf(g_fb_path, FB_PATH_MAX, "%s", home);
			fb_scan();
			g_view = 1;
		}
		break;
	}

	/* Open browser at / */
	case '\\':
		strcpy(g_fb_path, "/");
		fb_scan();
		g_view = 1;
		break;

	/* Toggle 2/4 decks */
	case 'T':
		g_num_tracks = (g_num_tracks == 2) ? 4 : 2;
		if (g_active_track >= g_num_tracks)
			g_active_track = 0;
		settings_save();
		break;

	/* Quit */
	case 'Q':
		quit_confirm();
		break;
	}
}

/* ──────────────────────────────────────────────
   Splash screen
   ────────────────────────────────────────────── */

/* Count display columns in a UTF-8 string.
 * Block/box-drawing chars (U+2500–U+259F) are single-width. */
/* Count terminal display columns for a UTF-8 string using wcwidth(). */
static int utf8_cols(const char *s)
{
	int n = 0;
	while (*s) {
		unsigned char c = (unsigned char)*s;
		wchar_t wc;
		int len;
		if (c < 0x80) {
			wc = c;
			len = 1;
		} else if (c >= 0xF0 && s[1] && s[2] && s[3]) {
			wc = ((wchar_t)(c & 0x07) << 18) |
			     ((wchar_t)((unsigned char)s[1] & 0x3F) << 12) |
			     ((wchar_t)((unsigned char)s[2] & 0x3F) << 6) |
			     ((wchar_t)((unsigned char)s[3] & 0x3F));
			len = 4;
		} else if (c >= 0xE0 && s[1] && s[2]) {
			wc = ((wchar_t)(c & 0x0F) << 12) |
			     ((wchar_t)((unsigned char)s[1] & 0x3F) << 6) |
			     ((wchar_t)((unsigned char)s[2] & 0x3F));
			len = 3;
		} else if (c >= 0xC0 && s[1]) {
			wc = ((wchar_t)(c & 0x1F) << 6) |
			     ((wchar_t)((unsigned char)s[1] & 0x3F));
			len = 2;
		} else {
			s++;
			continue;
		}
		int w = wcwidth(wc);
		if (w > 0)
			n += w;
		s += len;
	}
	return n;
}

/* Render one logo line with lx that may be negative (terminal too narrow).
 * Skips leading display columns so the logo clips symmetrically on both
 * sides rather than left-aligning when wider than the screen. */
static void splash_logo_line(int y, int lx, const char *s)
{
	if (lx >= 0) {
		mvwaddstr(g_win_main, y, lx, s);
		return;
	}
	/* Skip |lx| display columns from the left of the string. */
	int skip = -lx;
	const char *p = s;
	while (*p && skip > 0) {
		unsigned char c = (unsigned char)*p;
		int w;
		if (c < 0x80) {
			w = 1;
			p += 1;
		} else if (c >= 0xF0) {
			w = 2;
			p += 4;
		} else if (c >= 0xE0) {
			w = 2;
			p += 3;
		} else if (c >= 0xC0) {
			w = 1;
			p += 2;
		} else {
			w = 0;
			p += 1; /* continuation byte */
		}
		skip -= w;
	}
	/* skip may be -1 if a double-wide char straddles col 0; in that case
	 * the right half starts at col 1 (left half already off-screen). */
	int start_col = (skip < 0) ? -skip : 0;
	if (*p)
		mvwaddstr(g_win_main, y, start_col, p);
}

static void draw_splash(int64_t elapsed_ms)
{
	/* 7-line block-shading logo from djcmdlogo.txt */
	static const char *logo[] = {
		"\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\x20\x20\x20\x20\x20\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91",
		"\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\x20\x20\x20\x20\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91",
		"\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\x20\x20\x20\x20\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\x20\x20\x20\x20\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91",
		"\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\x20\x20\x20\x20\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\x20\x20\x20\x20\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91",
		"\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\x20\x20\x20\x20\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91",
		"\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91",
		"\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91",
	};
	static const int nlogo = 7;

	werase(g_win_main);

	int logo_w = utf8_cols(logo[0]);
	int lx = (g_cols - logo_w) / 2;
	int ly = (g_rows - nlogo - 5) / 2;
	if (ly < 0)
		ly = 0;

	wattron(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
	for (int i = 0; i < nlogo; i++)
		splash_logo_line(ly + i, lx, logo[i]);
	wattroff(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);

	/* Tagline */
	const char *tagline = "Terminal DJ Application";
	int tx = (g_cols - (int)strlen(tagline)) / 2;
	wattron(g_win_main, A_BOLD);
	mvwaddstr(g_win_main, ly + nlogo + 1, tx, tagline);
	wattroff(g_win_main, A_BOLD);

	/* Countdown bar */
	int64_t remain_ms = 5000 - elapsed_ms;
	if (remain_ms < 0)
		remain_ms = 0;
	int bar_w = 30;
	int filled = (int)(bar_w * remain_ms / 5000);
	if (filled > bar_w)
		filled = bar_w;
	char bar[64];
	int bi = 0;
	bar[bi++] = '[';
	for (int i = 0; i < bar_w; i++)
		bar[bi++] = (i < filled) ? '#' : ' ';
	bar[bi++] = ']';
	bar[bi] = '\0';

	int bx = (g_cols - bar_w - 2) / 2;
	wattron(g_win_main, A_DIM);
	mvwaddstr(g_win_main, ly + nlogo + 2, bx, bar);

	const char *hint = "Press any key to skip";
	int hx = (g_cols - (int)strlen(hint)) / 2;
	mvwaddstr(g_win_main, ly + nlogo + 3, hx, hint);
	wattroff(g_win_main, A_DIM);

	wrefresh(g_win_main);
}

/* ──────────────────────────────────────────────
   UI Thread
   ────────────────────────────────────────────── */
static void *ui_thread(void *arg)
{
	(void)arg;

	/* ── Splash screen — 2 s, skippable ── */
	{
		struct timespec t0, tn;
		clock_gettime(CLOCK_MONOTONIC, &t0);
		wtimeout(g_win_main, 50); /* 50 ms poll */
		for (;;) {
			clock_gettime(CLOCK_MONOTONIC, &tn);
			int64_t elapsed =
				(int64_t)(tn.tv_sec - t0.tv_sec) * 1000 +
				(tn.tv_nsec - t0.tv_nsec) / 1000000;
			if (elapsed >= 2000)
				break;
			draw_splash(elapsed);
			if (wgetch(g_win_main) != ERR)
				break;
		}
		wclear(g_win_main);
		wrefresh(g_win_main);
		/* Restore the configured UI rate */
		apply_ui_fps();
	}

	while (g_running) {
		redraw();
		int c = wgetch(g_win_main);
		if (c == ERR) {
			usleep(20000);
			continue;
		}

		/* Quit confirm modal intercepts all keys */
		if (g_quit_pending) {
			if (c == 'y' || c == 'Y') {
				g_running = 0;
				break;
			} else {
				g_quit_pending = 0;
			} /* any other key cancels */
			continue;
		}

		handle_key(c);
	}
	return NULL;
}

/* ──────────────────────────────────────────────
   Cleanup
   ────────────────────────────────────────────── */
static void cleanup(void)
{
	settings_save();
	mixlog_close();
	/* Stop all platter motors before closing the MIDI port */
	for (int i = 0; i < MAX_TRACKS; i++)
		motor_set(i, 0);
	/* Close display handles — re-enable when ns7iii_displaysub.h is wired in */
	/* disp_close(); */
	if (g_pcm) {
		snd_pcm_close(g_pcm);
		g_pcm = NULL;
	}
	if (g_midi_in) {
		snd_rawmidi_close(g_midi_in);
		g_midi_in = NULL;
	}
	if (g_midi_out) {
		snd_rawmidi_close(g_midi_out);
		g_midi_out = NULL;
	}
	for (int i = 0; i < MAX_TRACKS; i++) {
		free(g_tracks[i].data);
		g_tracks[i].data = NULL;
		free(g_tracks[i].wfm_low);
		g_tracks[i].wfm_low = NULL;
		free(g_tracks[i].wfm_mid);
		g_tracks[i].wfm_mid = NULL;
		free(g_tracks[i].wfm_high);
		g_tracks[i].wfm_high = NULL;
	}
	endwin();
}

static void sig_handler(int s)
{
	(void)s;
	g_running = 0;
}

/* ──────────────────────────────────────────────
   ALSA silent error handler
   Suppresses "Cannot get card index" spam when
   probing for MIDI devices that don't exist.
   ────────────────────────────────────────────── */
static void alsa_silent_error(const char *file, int line, const char *func,
			      int err, const char *fmt, ...)
{
	(void)file;
	(void)line;
	(void)func;
	(void)err;
	(void)fmt;
}

/* ──────────────────────────────────────────────
   MIDI device enumeration
   Walks all ALSA cards/devices and collects every
   rawmidi input into g_midi_devlist[].
   Returns number of devices found.
   ────────────────────────────────────────────── */
static int midi_enumerate_devices(void)
{
	snd_ctl_t *ctl;
	snd_rawmidi_info_t *info;
	snd_rawmidi_info_alloca(&info);

	g_midi_ndevices = 0;

	int card = -1;
	while (snd_card_next(&card) == 0 && card >= 0) {
		char ctl_name[32];
		snprintf(ctl_name, sizeof(ctl_name), "hw:%d", card);
		if (snd_ctl_open(&ctl, ctl_name, 0) < 0)
			continue;

		int device = -1;
		while (snd_ctl_rawmidi_next_device(ctl, &device) == 0 &&
		       device >= 0) {
			snd_rawmidi_info_set_device(info, (unsigned)device);
			snd_rawmidi_info_set_subdevice(info, 0);
			snd_rawmidi_info_set_stream(info,
						    SND_RAWMIDI_STREAM_INPUT);

			if (snd_ctl_rawmidi_info(ctl, info) == 0 &&
			    g_midi_ndevices < MIDI_MAX_DEVICES) {
				const char *iname =
					snd_rawmidi_info_get_name(info);
				/* Skip NS7III display ports — output-only under VDJ protocol */
				if (strstr(iname, "Display Right") ||
				    strstr(iname, "Display Left"))
					continue;
				MidiDevEntry *e =
					&g_midi_devlist[g_midi_ndevices++];
				snprintf(e->dev, sizeof(e->dev), "hw:%d,%d,0",
					 card, device);
				snprintf(e->name, sizeof(e->name), "%s", iname);
			}
		}
		snd_ctl_close(ctl);
	}
	return g_midi_ndevices;
}

/* ──────────────────────────────────────────────
   Per-device map filename
   Sanitises the device name into a safe filename:
   e.g. "NS7III Controller" → "ns7iii_controller"
   Falls back to "default" if name is empty.
   Full path: ~/.config/djcmd/<sanitised>.map
   ────────────────────────────────────────────── */
static void midi_map_name_from_device(const char *dev_name, char *out,
				      size_t max)
{
	char san[128] = "";
	int oi = 0;
	for (int i = 0; dev_name[i] && oi < (int)sizeof(san) - 1; i++) {
		unsigned char c = (unsigned char)dev_name[i];
		if (c >= 'A' && c <= 'Z')
			san[oi++] = (char)(c + 32); /* to lower */
		else if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9'))
			san[oi++] = (char)c;
		else if (oi > 0 && san[oi - 1] != '_')
			san[oi++] = '_'; /* collapse non-alnum to _ */
	}
	/* Trim trailing underscores */
	while (oi > 0 && san[oi - 1] == '_')
		oi--;
	san[oi] = '\0';

	if (oi == 0)
		snprintf(san, sizeof(san), "default");
	snprintf(out, max, "%s", san);
}

/* ──────────────────────────────────────────────
   PCM (audio output) device enumeration
   Walks ALSA PCM playback devices and collects
   them into g_pcm_devlist[].
   ────────────────────────────────────────────── */
static int pcm_enumerate_devices(void)
{
	g_pcm_ndevices = 0;

	/* Always add "default" as the first entry */
	snprintf(g_pcm_devlist[g_pcm_ndevices].dev,
		 sizeof(g_pcm_devlist[0].dev), "default");
	snprintf(g_pcm_devlist[g_pcm_ndevices].name,
		 sizeof(g_pcm_devlist[0].name), "default (system)");
	g_pcm_ndevices++;

	/* Walk cards/devices */
	int card = -1;
	while (snd_card_next(&card) == 0 && card >= 0 &&
	       g_pcm_ndevices < PCM_MAX_DEVICES) {
		snd_ctl_t *ctl;
		char ctl_name[32];
		snprintf(ctl_name, sizeof(ctl_name), "hw:%d", card);
		if (snd_ctl_open(&ctl, ctl_name, 0) < 0)
			continue;

		snd_pcm_info_t *info;
		snd_pcm_info_alloca(&info);

		int device = -1;
		while (snd_ctl_pcm_next_device(ctl, &device) == 0 &&
		       device >= 0 && g_pcm_ndevices < PCM_MAX_DEVICES) {
			snd_pcm_info_set_device(info, (unsigned)device);
			snd_pcm_info_set_subdevice(info, 0);
			snd_pcm_info_set_stream(info, SND_PCM_STREAM_PLAYBACK);
			if (snd_ctl_pcm_info(ctl, info) == 0) {
				PcmDevEntry *e =
					&g_pcm_devlist[g_pcm_ndevices++];
				snprintf(e->dev, sizeof(e->dev), "hw:%d,%d",
					 card, device);
				snprintf(e->name, sizeof(e->name), "%s",
					 snd_pcm_info_get_name(info));
			}
		}
		snd_ctl_close(ctl);
	}
	return g_pcm_ndevices;
}

/* Switch to a different PCM output device at runtime.
 * Stops audio thread momentarily, closes old PCM, opens new one. */
static void pcm_open_device(int dev_idx)
{
	if (dev_idx < 0 || dev_idx >= g_pcm_ndevices)
		return;

	/* Close existing device — audio thread will stall briefly */
	if (g_pcm) {
		snd_pcm_drain(g_pcm);
		snd_pcm_close(g_pcm);
		g_pcm = NULL;
	}

	g_pcm_dev_sel = dev_idx;
	snprintf(g_pcm_dev_str, sizeof(g_pcm_dev_str), "%s",
		 g_pcm_devlist[dev_idx].dev);

	if (init_alsa() == 0) {
		snprintf(g_fb_status, sizeof(g_fb_status), "Audio: %s",
			 g_pcm_devlist[dev_idx].name);
		settings_save();
	} else {
		snprintf(g_fb_status, sizeof(g_fb_status),
			 "Audio: FAILED to open %s", g_pcm_dev_str);
	}
}

/* ──────────────────────────────────────────────
   main()
   ────────────────────────────────────────────── */
int main(int argc, char **argv)
{
	setlocale(LC_ALL, ""); /* enable wide-char / UTF-8 output */
	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);

	/* Suppress ALSA's stderr noise during device probing */
	snd_lib_error_set_handler(alsa_silent_error);

	/* Init EQ coefficients */
	init_eq_coeffs();

	/* Init phase vocoder twiddle/Hann tables */
	pv_init_tables();

	/* Init effects engine */
	fx_init_all();

	/* Init tracks */
	for (int i = 0; i < MAX_TRACKS; i++) {
		Track *t = &g_tracks[i];
		memset(t, 0, sizeof(*t));
		t->volume = 1.0f;
		t->pitch = 1.0f;
		t->gain = 1.0f;
		t->nudge = 0.0f;
		t->filter = 0.5f; /* flat — no filtering */
		t->sync_locked = 0;
		pthread_mutex_init(&t->lock, NULL);
		g_eq[i].fi_last =
			-1.0f; /* force coefficient compute on first use */
	}

	/* PCM — enumerate playback devices, apply saved device string */
	pcm_enumerate_devices();
	/* Sync g_pcm_dev_sel to the saved device string */
	for (int i = 0; i < g_pcm_ndevices; i++) {
		if (strcmp(g_pcm_devlist[i].dev, g_pcm_dev_str) == 0) {
			g_pcm_dev_sel = i;
			break;
		}
	}

	/* ALSA */
	if (init_alsa() < 0) {
		fprintf(stderr, "djcmd: failed to open ALSA device '%s'\n",
			g_pcm_dev_str);
		return 1;
	}

	/* MIDI — enumerate all inputs, then open the saved device (or first found) */
	{
		midi_enumerate_devices();

		/* If a device was saved in settings and is still present, use it.
         * Otherwise fall back to the first device in the list. */
		const char *to_open = NULL;
		if (g_midi_dev_str[0]) {
			for (int i = 0; i < g_midi_ndevices; i++) {
				if (strcmp(g_midi_devlist[i].dev,
					   g_midi_dev_str) == 0) {
					to_open = g_midi_devlist[i].dev;
					g_midi_dev_sel = i;
					break;
				}
			}
		}
		if (!to_open && g_midi_ndevices > 0) {
			to_open = g_midi_devlist[0].dev;
			g_midi_dev_sel = 0;
		}
		if (to_open) {
			strncpy(g_midi_dev_str, to_open,
				sizeof(g_midi_dev_str) - 1);
			g_midi_dev_str[sizeof(g_midi_dev_str) - 1] = '\0';
			snd_rawmidi_open(&g_midi_in, &g_midi_out, to_open,
					 SND_RAWMIDI_NONBLOCK);
		}
		/* g_midi_in stays NULL if no controller present — that's fine */
	}

	/* Load persisted settings (overrides compiled-in defaults) */
	settings_load();

	/* Read CPU info once — it never changes at runtime */
	options_read_cpuinfo(g_cpuinfo_cache, sizeof(g_cpuinfo_cache));

	/* Open session mix log */
	mixlog_open();

	/* Load MIDI map file; write NS7III defaults if none exists for this device */
	midi_map_load();
	crates_load();
	if (g_midi_nbindings == 0 && g_midi_nout_bindings == 0)
		midi_map_write_generic_defaults();
	/* Migration: fix old motor_stop entries with wrong values */
	{
		int migrated = 0;
		const char *stop_names[] = { "motor_stop_a", "motor_stop_b",
					     "motor_stop_c", "motor_stop_d" };
		const uint8_t start_st[] = { 0xB1, 0xB2, 0xB3, 0xB4 };
		for (int di = 0; di < 4; di++) {
			MidiOutBinding *b = midi_out_lookup(stop_names[di]);
			if (b && b->data1 == 0x41 && b->data2 == 0) {
				b->data1 = 0x42;
				b->data2 = 127;
				if (!b->status)
					b->status = start_st[di];
				migrated = 1;
			}
		}
		if (migrated)
			midi_map_save();
	}

	/* Resolve starting library path — priority order:
     *   1. Command-line argument  (djcmd /path/to/music)
     *   2. ~/.config/djcmd/library  (one path, no newline required)
     *   3. Current working directory
     *
     * To set a permanent default:
     *   mkdir -p ~/.config/djcmd
     *   echo /path/to/music > ~/.config/djcmd/library
     */
	if (argc > 1) {
		snprintf(g_fb_path, FB_PATH_MAX, "%s", argv[1]);
		g_fb_path[FB_PATH_MAX - 1] = '\0';
	} else {
		/* Try config file */
		int loaded = 0;
		const char *home = getenv("HOME");
		if (home) {
			char cfg[FB_PATH_MAX];
			snprintf(cfg, sizeof(cfg), "%s/.config/djcmd/library",
				 home);
			FILE *cf = fopen(cfg, "r");
			if (cf) {
				if (fgets(g_fb_path, FB_PATH_MAX, cf)) {
					/* Strip trailing newline / whitespace */
					int len = (int)strlen(g_fb_path);
					while (len > 0 &&
					       (g_fb_path[len - 1] == '\n' ||
						g_fb_path[len - 1] == '\r' ||
						g_fb_path[len - 1] == ' '))
						g_fb_path[--len] = '\0';
					if (len > 0)
						loaded = 1;
				}
				fclose(cf);
			}
		}
		if (!loaded) {
			char *cwd = getcwd(g_fb_path, FB_PATH_MAX);
			if (!cwd)
				strcpy(g_fb_path, ".");
		}
	}
	fb_scan();

	/* ncurses */
	initscr();
	cbreak();
	noecho();
	keypad(stdscr, TRUE);
	curs_set(0);
	timeout(0);
	getmaxyx(stdscr, g_rows, g_cols);

	/* Detect real TTY: isatty(1) is true everywhere, but a TTY framebuffer
     * won't honour UTF-8 wide chars reliably.  Check $TERM — a real VT/TTY
     * sets it to "linux" or "vt100"; terminal emulators use "xterm*" etc. */
	{
		const char *term = getenv("TERM");
		g_is_tty = (term && (strncmp(term, "linux", 5) == 0 ||
				     strncmp(term, "vt", 2) == 0 ||
				     strncmp(term, "con", 3) == 0));
	}

	init_colors();

	g_win_main = newwin(g_rows - 1, g_cols, 0, 0);
	g_win_status = newwin(1, g_cols, g_rows - 1, 0);
	keypad(g_win_main, TRUE);
	apply_ui_fps(); /* sets wtimeout from g_opts.ui_fps (loaded from settings) */

	/* Threads */
	pthread_t at, mt, ut, lt, mot;
	pthread_create(&at, NULL, audio_thread, NULL);
	pthread_create(&lt, NULL, load_worker, NULL);
	pthread_create(&mt, NULL, midi_thread,
		       NULL); /* always running — waits for device */
	pthread_create(&mot, NULL, motor_thread, NULL); /* NS7III motor stub */

	/* Open NS7III display devices and launch display thread
     * — disabled; see ns7iii_displaysub.h. Re-enable once Numark handshake is solved.
     * disp_open();
     * pthread_create(&dt, NULL, display_thread, NULL);
     */

	pthread_create(&ut, NULL, ui_thread, NULL);

	pthread_join(ut, NULL);
	g_running = 0;
	/* Wake load worker so it can exit its cond_wait */
	pthread_cond_signal(&g_load_cond);
	pthread_join(at, NULL);
	pthread_join(lt, NULL);
	pthread_join(mt, NULL);
	pthread_join(mot, NULL);
	/* pthread_join(dt,  NULL); — disabled with display subsystem */

	cleanup();
	printf("djcmd: goodbye!\n");
	return 0;
}
