/*
 * djcmd -- Command-line DJ application
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
 * minimp3.h  -- CC0 (public domain)  -- https://github.com/lieff/minimp3
 * dr_flac.h  -- public domain / MIT-0 -- https://github.com/mackron/dr_libs
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
#include <sched.h>
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
#include <poll.h>

#include "ns7iii_map.h"
#include "djcmd_shared.h"
#include "djcmd_ui.h"
#include "djcmd_midi.h"
#include "djcmd_audio.h"
#include "djcmd_fx.h"
#include "djcmd_help.h"
#include "djcmd_usb.h"
#include "djcmd_library.h"

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

void sig_handler(int s)
{
	(void)s;
	g_running = 0;
}

/* ──────────────────────────────────────────────
   Globals
   ────────────────────────────────────────────── */
Track g_tracks[MAX_TRACKS];
SamplerSlot g_samplers[MAX_SAMPLER_SLOTS];
EQState g_eq[MAX_TRACKS];
PVState g_pv[MAX_TRACKS];
WSOLAState g_wsola[MAX_TRACKS];
int g_num_tracks = 2;
_Atomic float g_crossfader = 0.5f;
float g_cf_curve = 0.5f;
_Atomic int g_master_vol = 100;
_Atomic int g_running = 1;
int g_is_tty = 0;

/* UI state */
_Atomic int g_autoplay_pending[MAX_TRACKS]; /* 1 = autoplay load in progress */
_Atomic int g_autoplay_ready[MAX_TRACKS]; /* 1 = autoplay track loaded and waiting to play */
_Atomic float g_autoplay_xf_target = -1.0f; /* target crossfader position, -1 = inactive */
int g_autoplay_deck = 0; /* next deck to use for autoplay transition */

/* TAP BPM state */
int64_t g_tap_ms[MAX_TRACKS][MAX_TAPS];
int g_tap_idx[MAX_TRACKS] = { 0 };
int g_tap_count[MAX_TRACKS] = { 0 };
/* g_pv_cos/sin moved to djcmd_audio.c */

/* Actual sample rate negotiated with the ALSA hardware device.
 * Set after pcm_open(); used for all time↔frame conversions and
 * pushed to audiofile via af_set_target_rate() so loads resample
 * to the hardware rate rather than the compiled-in default. */
unsigned int g_actual_sample_rate =
	CFG_SAMPLE_RATE; /* one stretcher per deck */
char g_pcm_hp_dev_str[64] = CFG_PCM_HEADPHONE;
int g_hp_vol = CFG_DEFAULT_HEADPHONE_VOL;
int g_pcm_hp_dev_sel = 0; /* cursor for headphone device picker */
int g_audio_hp_picker = 0; /* 0 = master picker active, 1 = hp picker active */

/* ── Batch BPM analyze state ─────────────────────────────────────────────
 * Double-tap N opens a range prompt; ENTER starts a background pass over
 * every audio file in the current panel (Browser/Playlist/Library/Crate).
 * Each file is loaded into a temporary Track, re-analyzed with the user's
 * BPM range, sidecar-saved, then freed.  g_batch_job_done is set by the
 * load_worker when each job finishes so the main loop can enqueue the next.
 */
/* Slip motor override: when 1, motor is held off even if deck is playing.
 * Toggled by the SLIP MODE button. Motor restarts when toggled back off
 * if the deck is still playing. */
int g_slip_motor_off[MAX_TRACKS] = { 0 };

#define MOTOR_ENABLE_CC 75 /* one-shot global enable, value=0, Ch1 */

/* ── Pad mode state ─────────────────────────────────────────────────────
 * Each physical side has its own pad mode and shift state.
 * PAD_MODE_HOTCUE  : pads 1-8 = hot cue assign/jump; SHIFT+pad = delete
 * PAD_MODE_AUTOLOOP: pads 1-4 = quantised beat loops (1/2/4/8 bars)
 *                    SHIFT+AUTO/ROLL button → PAD_MODE_ROLL
 * PAD_MODE_ROLL    : hold pad = temporary loop-roll, release = resume
 * See djcmd_shared.h for PAD_MODE_* definitions.
 */

/* Master cue point per deck (standard CUE button -- distinct from hot cues)
 * cue_default_set: 1 if a master cue point has been placed
 * cue_default_pos: frame position of master cue point */
int g_cue_default_set[MAX_TRACKS] = { 0 };
uint32_t g_cue_default_pos[MAX_TRACKS] = { 0 };
int g_cue_default_held[MAX_TRACKS] = { 0 }; /* 1 while CUE button held */

/* Bleep (momentary slip+reverse): save position on press, restore on release */
int g_censor_held[MAX_TRACKS] = { 0 };
uint32_t g_censor_save_pos[MAX_TRACKS] = { 0 };

/* Filter toggle: 1 = filter sweep knob is engaged, 0 = bypassed (flat) */
int g_filter_on[MAX_TRACKS] = { 0 };
int g_filter_roll_held[MAX_TRACKS] = { 0 };
int g_filter_was_on[MAX_TRACKS] = { 0 };
int g_touch_mode = 0; /* 1 = capacitive touch EQ kills active */

/* EQ kills: 1 = band is killed (-inf), 0 = active.
 * We store the last knob value to restore it when kill is toggled off. */

/* NS7III physical side routing -- 2 platters, 4 virtual decks via layers.
 *
 * The NS7III has two physical platters.  djcmd supports 4 software decks (A-D).
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
 * Note on NS7III display orientation: the two center displays are "crossed" --
 *   Display Right ALSA card (dev_id=0x20) shows Deck A (left platter).
 *   Display Left  ALSA card (dev_id=0x10) shows Deck B (right platter).
 * This is confirmed by cold-init pcap captures of Serato sending to the hardware.
 */
int g_side_deck[2] = { 0, 1 }; /* side→active deck index */
/* ── PCM (audio output) device list ─────────────────────────────────────
 * Populated at startup and on demand (R key in AUDIO tab).
 * Lets the user pick the audio output device from the options menu. */
PCMDevice g_pcm_devlist[PCM_MAX_DEVICES];
int g_pcm_ndevices = 0;
int g_pcm_dev_sel = 0;
char g_pcm_dev_str[64] = CFG_PCM_DEVICE;
/* active device */ /* e.g. "hw:1,0,0"           */

/* ──────────────────────────────────────────────
   Session Mix Log
   Appends one line per track load to a dated log
   file in ~/.config/djcmd/.  Zero runtime cost --
   all I/O happens in the load-worker thread, not
   the audio thread.
   ────────────────────────────────────────────── */

/* tag_clean, load_worker, audio_thread, batch/enqueue → djcmd_audio.c */

void alsa_silent_error(const char *file, int line, const char *func, int err,
		       const char *fmt, ...)
{
	(void)file;
	(void)line;
	(void)func;
	(void)err;
	(void)fmt;
}

/* pcm_enumerate_devices, pcm_open_device, hp_open_device → djcmd_audio.c */

static void cleanup(void)
{
	settings_save();
	mixlog_close();
	/* Stop all platter motors before closing the MIDI port */
	for (int i = 0; i < MAX_TRACKS; i++)
		motor_set(i, 0);
	audio_pcm_close_all();
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

/* ──────────────────────────────────────────────
   main()
   ────────────────────────────────────────────── */
int main(int argc, char **argv)
{
	setlocale(LC_ALL, ""); /* enable wide-char / UTF-8 output */
	ensure_sidecar_cache_dir();
	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);

	/* Load persisted settings (overrides compiled-in defaults) */
	settings_load();

#ifdef USE_OPENBLAS
	/* Force single-threaded OpenBLAS to avoid OpenMP contention/starvation
	 * on single-core G4 and ensure real-time audio stability. */
	openblas_set_num_threads(1);
#endif

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
		t->filter = 0.5f; /* flat -- no filtering */
		t->synced = 0;
		pthread_mutex_init(&t->lock, NULL);
		g_eq[i].fi_last =
			-1.0f; /* force coefficient compute on first use */
	}

	/* Init Samplers */
	for (int i = 0; i < MAX_SAMPLER_SLOTS; i++) {
		SamplerSlot *s = &g_samplers[i];
		memset(s, 0, sizeof(*s));
		s->volume = 1.0f;
		pthread_mutex_init(&s->lock, NULL);
	}

	/* PCM -- enumerate playback devices, apply saved device string */
	pcm_enumerate_devices();
	/* Sync g_pcm_dev_sel to the saved device string */
	for (int i = 0; i < g_pcm_ndevices; i++) {
		if (strcmp(g_pcm_devlist[i].dev, g_pcm_dev_str) == 0) {
			g_pcm_dev_sel = i;
			break;
		}
	}
	/* Sync g_pcm_hp_dev_sel to the saved headphone device string */
	for (int i = 0; i < g_pcm_ndevices; i++) {
		if (strcmp(g_pcm_devlist[i].dev, g_pcm_hp_dev_str) == 0) {
			g_pcm_hp_dev_sel = i;
			break;
		}
	}

	/* ALSA */
	if (init_alsa() < 0) {
		/* Fallback 1: try explicit hw:0,0 if default failed */
		strncpy(g_pcm_dev_str, "hw:0,0", sizeof(g_pcm_dev_str) - 1);
		if (init_alsa() < 0) {
			/* Fallback 2: try plughw:0,0 */
			strncpy(g_pcm_dev_str, "plughw:0,0",
				sizeof(g_pcm_dev_str) - 1);
		}

		if (!audio_pcm_is_open()) {
			/* Failed primary and explicit fallback -- try device list */
			for (int i = 1; i < g_pcm_ndevices; i++) {
				strncpy(g_pcm_dev_str, g_pcm_devlist[i].dev,
					sizeof(g_pcm_dev_str) - 1);
				if (init_alsa() == 0) {
					g_pcm_dev_sel = i;
					break;
				}
			}
		}

		if (!audio_pcm_is_open()) {
			/* Stay on default but mark as failed */
			strncpy(g_pcm_dev_str, g_pcm_devlist[0].dev,
				sizeof(g_pcm_dev_str) - 1);
			g_pcm_dev_str[sizeof(g_pcm_dev_str) - 1] = '\0';
			g_pcm_dev_sel = 0;
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Audio: FAILED to open any device");
		} else {
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Audio: Fallback to %s",
				 g_pcm_devlist[g_pcm_dev_sel].name);
		}
	} else {
		snprintf(g_fb_status, sizeof(g_fb_status), "Audio: %s",
			 g_pcm_devlist[g_pcm_dev_sel].name);
	}

	/* MIDI -- enumerate all inputs, then open the saved device (or first found) */
	midi_init(g_midi_dev_str);

	/* Read CPU info once -- it never changes at runtime */
	options_read_cpuinfo(g_cpuinfo_cache, sizeof(g_cpuinfo_cache));

	/* Open session mix log */
	mixlog_open();

	crates_load();

	/* Resolve starting library path -- priority order:
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

	/* ncurses -- set ESCDELAY before initscr so ESC is not mistaken for
	 * the start of an escape sequence and delayed by ~1 second */
	set_escdelay(75);
	initscr();
	cbreak();
	noecho();
	keypad(stdscr, TRUE);
	curs_set(0);
	getmaxyx(stdscr, g_rows, g_cols);

	/* Detect real TTY: isatty(1) is true everywhere, but a TTY framebuffer
     * won't honour UTF-8 wide chars reliably.  Check $TERM -- a real VT/TTY
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
	pthread_t at, ut, lt;
	pthread_create(&at, NULL, audio_thread, NULL);

	/* Elevate audio thread to real-time priority to prevent terminal glitches */
	set_realtime_priority(at, 40);

	pthread_create(&lt, NULL, load_worker, NULL);

	/* Open NS7III display devices and launch display thread
     * -- disabled; see ns7iii_displaysub.h. Re-enable once Numark handshake is solved.
     * disp_open();
     * pthread_create(&dt, NULL, display_thread, NULL);
     */

	pthread_create(&ut, NULL, ui_thread, NULL);

	pthread_join(ut, NULL);
	g_running = 0;
	/* Interrupt any blocking snd_pcm_writei() so the audio thread exits fast */
	audio_pcm_drop_all();
	/* Wake load worker so it can exit its cond_wait */
	pthread_cond_signal(&g_load_cond);
	pthread_join(at, NULL);
	pthread_join(lt, NULL);
	/* pthread_join(dt,  NULL); -- disabled with display subsystem */

	cleanup();
	printf("djcmd: goodbye!\n");
	return 0;
}
