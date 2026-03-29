/*
 * djcmd_audio.h — Audio processing engine interface for djcmd
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
 */

#ifndef DJCMD_AUDIO_H
#define DJCMD_AUDIO_H

#include <stdint.h>
#include <pthread.h>
#include "djcmd_config.h"

/* Max hot cue points per deck */
#define MAX_CUES 8
#define MAX_FILENAME 512

/* ── WSOLA (Waveform Similarity Overlap-Add) key-lock stretcher ────────── */
#define WSOLA_WIN 2048 /* analysis window in frames (~46 ms) */
#define WSOLA_HOP 512 /* synthesis hop = PERIOD_FRAMES */
#define WSOLA_SEARCH 512 /* cross-correlation search range (±256 frames) */
#define WSOLA_BUF 16384 /* output ring-buffer size (must be power of 2) */

typedef struct {
	/* Ring buffer of time-stretched output waiting to be consumed */
	float buf_l[WSOLA_BUF];
	float buf_r[WSOLA_BUF];
	int write_pos; /* next write position in ring */
	int read_pos; /* next read position in ring */
	int fill; /* samples available to read */
	/* Analysis state */
	double src_pos; /* fractional read position in source track */
	float prev_l[WSOLA_WIN]; /* previous synthesis window for overlap-add */
	float prev_r[WSOLA_WIN];
	int prev_valid; /* 1 once prev window is filled */
} WSOLAState;

/* ── Phase Vocoder (PV) key-lock time-stretcher ────────────────────────── */
#define PV_N    512
#define PV_HS   128
#define PV_BINS (PV_N / 2 + 1)
#define PV_BUF  4096

typedef struct {
	float   ph_an_l[PV_BINS];  /* analysis phase, previous frame, L */
	float   ph_an_r[PV_BINS];  /* analysis phase, previous frame, R */
	float   ph_syn_l[PV_BINS]; /* synthesis phase accumulator, L */
	float   ph_syn_r[PV_BINS]; /* synthesis phase accumulator, R */
	double  src_pos;           /* fractional source read position */
	float   out_l[PV_BUF];     /* output ring buffer, L */
	float   out_r[PV_BUF];     /* output ring buffer, R */
	int     out_write;         /* write pointer */
	int     out_read;          /* read pointer */
	int     out_fill;          /* samples available to read */
	int     initialized;       /* 1 after first analysis frame loaded */
} PVState;

typedef struct {
	int16_t *data; /* interleaved stereo PCM */
	uint32_t num_frames; /* total stereo frames */
	uint32_t pos; /* playback cursor (frames) */
	float pitch; /* pitch/speed multiplier 0.5–2.0 */
	float volume; /* 0.0–1.0 */
	float gain; /* auto-gain normalisation factor */
	float eq_low; /* EQ gain -1.0 to +1.0 */
	float eq_mid;
	float eq_high;
	float filter; /* filter knob: 0.0=LP full, 0.5=flat, 1.0=HP full */
	float fader; /* crossfader contribution 0.0–1.0 */
	int playing; /* bool */
	int looping; /* bool */
	int reverse; /* bool — 1 = platter/playback running in reverse */
	uint32_t loop_start;
	uint32_t loop_end;
	float bpm; /* detected BPM (used for pitch/sync) */
	float bpm_offset; /* beat phase offset in frames */
	int bpm_display_double; /* -1=half (÷2), 0=normal, 1=double (×2) */
	int key_lock; /* 1 = time-stretch to preserve pitch at any tempo */
	int cue_active; /* 1 = deck routed to headphones (PFL) */
	/* Sync */
	int sync_locked; /* 1 = locked to master tempo */
	int pending_play; /* 1 = waiting for quantize-sync beat to start */
	/* Nudge */
	float nudge; /* transient pitch offset, decays to 0 */
	/* Cue points */
	uint32_t cue[MAX_CUES]; /* frame positions, 0 = unset */
	int cue_set[MAX_CUES];
	char filename[MAX_FILENAME];
	char tag_title[128]; /* ID3/Vorbis title tag */
	char tag_artist[128]; /* ID3/Vorbis artist tag */
	char tag_key[16]; /* Musical key (Camelot or note) */
	int loaded;
	/* Waveform overview peaks 0-255 per band per bin */
	uint8_t *wfm_low;
	uint8_t *wfm_mid;
	uint8_t *wfm_high;
	uint32_t wfm_bins;
	float wfm_band_max[3];
	float period_peak; /* peak level from last period (0.0-1.0) */
	pthread_mutex_t lock;
} Track;

/* Audio processing functions */
void read_pitched(Track *t, float *out_l, float *out_r,
                 uint32_t out_frames, float start_vel,
                 float target_vel, float start_nudge,
                 float target_nudge);

void wsola_process(Track *t, WSOLAState *ws, float *out_l, float *out_r,
		  uint32_t out_frames, double rate, float vol_gain, int is_eco);
void wsola_reset(WSOLAState *ws, uint32_t start_pos);

void pv_process(Track *t, PVState *pv, float *out_l, float *out_r,
                uint32_t out_frames, double rate, float gain);
void pv_reset(PVState *pv, uint32_t start_pos);
void pv_init_tables(void);

#endif /* DJCMD_AUDIO_H */
