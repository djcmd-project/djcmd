/*
 * djcmd_fx.h — Audio effects engine interface for djcmd
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

#ifndef DJCMD_FX_H
#define DJCMD_FX_H

#include "djcmd_config.h" /* MAX_TRACKS */

/* ── Effect type IDs ──────────────────────────────────────────────────────── */
#define FX_NONE       0
#define FX_ECHO       1  /* Tape echo — feedback delay line                   */
#define FX_PINGPONG   2  /* Ping-pong stereo delay                            */
#define FX_REVERB     3  /* Freeverb Schroeder reverb                         */
#define FX_FLANGER    4  /* Short delay + LFO (comb sweep)                    */
#define FX_CHORUS     5  /* Multi-voice flanger, deeper / slower              */
#define FX_PHASER     6  /* 4-stage allpass + LFO (notch sweep)               */
#define FX_DISTORTION 7  /* tanh soft-clip waveshaper with pre-gain           */
#define FX_BITCRUSH   8  /* Bit depth reduction + sample-rate reduction       */
#define FX_GATE       9  /* Noise gate — silence below threshold              */
#define FX_WIDENER    10 /* Stereo widener via M/S processing                 */
#define FX_COUNT      11

/* ── Slot layout ─────────────────────────────────────────────────────────── */
#define FX_SLOTS_PER_DECK \
	3 /* per deck: slot 0, 1, 2 — each toggled by FX btn 1/2/3 */
#define FX_MASTER_SLOT (MAX_TRACKS * FX_SLOTS_PER_DECK) /* master bus    */
#define FX_TOTAL_SLOTS (FX_MASTER_SLOT + 1)
#define FX_PARAMS 4

typedef struct {
	int type;         /* FX_* id currently active                 */
	int pending_type; /* -1 = no change; else swap next period     */
	float params[FX_PARAMS]; /* [time, feedback, tone, wet]               */
	void *state;      /* malloc'd effect state; NULL for FX_NONE  */
} FXSlot;

/* ── Per-effect state structs ─────────────────────────────────────────────── */

/* Echo / Ping-pong: circular stereo delay buffer */
#define FX_DELAY_MAX_FRAMES \
	96000 /* 2s at 48000 Hz — safe for 44100 and 48000 */
typedef struct {
	float *buf_l;
	float *buf_r;
	int size; /* allocated frames */
	int pos;  /* write head */
} DelayState;

/* Freeverb: 8 comb + 4 allpass per channel (tuned to 44100 Hz) */
#define COMB_COUNT 8
#define AP_COUNT   4
typedef struct {
	float *comb_buf_l[COMB_COUNT], *comb_buf_r[COMB_COUNT];
	int comb_size_l[COMB_COUNT]; /* L delay lengths */
	int comb_size_r[COMB_COUNT]; /* R delay lengths (L + 23) */
	int comb_pos_l[COMB_COUNT];  /* separate L/R positions — must not share */
	int comb_pos_r[COMB_COUNT];
	float comb_filt_l[COMB_COUNT], comb_filt_r[COMB_COUNT];
	float *ap_buf_l[AP_COUNT], *ap_buf_r[AP_COUNT];
	int ap_size_l[AP_COUNT];
	int ap_size_r[AP_COUNT];
	int ap_pos_l[AP_COUNT]; /* separate L/R positions */
	int ap_pos_r[AP_COUNT];
} ReverbState;

/* Flanger / Chorus: short delay + LFO */
#define FLANGER_BUF_FRAMES 4096 /* ~93ms at 44100 Hz */
typedef struct {
	float buf_l[FLANGER_BUF_FRAMES];
	float buf_r[FLANGER_BUF_FRAMES];
	int pos;
	float lfo_phase; /* 0..2π */
} FlangerState;

/* Phaser: 4 allpass biquad stages + LFO */
#define PHASER_STAGES 4
typedef struct {
	float x1l[PHASER_STAGES], x2l[PHASER_STAGES];
	float y1l[PHASER_STAGES], y2l[PHASER_STAGES];
	float x1r[PHASER_STAGES], x2r[PHASER_STAGES];
	float y1r[PHASER_STAGES], y2r[PHASER_STAGES];
	float lfo_phase;
} PhaserState;

/* Bitcrusher: SR reduction accumulator */
typedef struct {
	float held_l, held_r;
	float accum;
} BitcrushState;

/* Gate: envelope follower */
typedef struct {
	float env;
	float gain; /* smoothed output gain 0..1 */
} GateState;

/* Distortion: per-slot tone filter state */
typedef struct {
	float lp_l, lp_r;
} DistortionState;

/* Compressor/limiter (master bus): envelope follower + gain computer */
typedef struct {
	float env_peak;
	float gain;
} CompState;

/* ── Globals ──────────────────────────────────────────────────────────────── */

extern const char *fx_names[FX_COUNT];
extern FXSlot g_fx[FX_TOTAL_SLOTS]; /* [deck*FX_SLOTS_PER_DECK+slot], then [FX_MASTER_SLOT] */
extern int g_fx_ui_slot[MAX_TRACKS]; /* which slot the FX knobs control per deck */
extern int g_fx_last_type[MAX_TRACKS][3]; /* last non-NONE type per deck per slot */
extern float g_fx_param_acc[MAX_TRACKS][FX_PARAMS]; /* relative encoder accumulator */

/* Step size per encoder tick — 1/64 traverses full range in ~half a revolution */
#define FX_KNOB_STEP 0.015625f /* 1/64 */

/* ── Accessors (inline — no function call overhead in audio thread) ────────── */

static inline FXSlot *fx_slot(int deck, int slot)
{
	return &g_fx[deck * FX_SLOTS_PER_DECK + slot];
}

static inline FXSlot *fx_master(void)
{
	return &g_fx[FX_MASTER_SLOT];
}

/* ── Function prototypes ──────────────────────────────────────────────────── */

void fx_set_type(int deck, int slot_idx, int type);
void fx_set_param(int deck, int slot_idx, int param_idx, float value);
void fx_set_wet(int deck, int slot_idx, float wet);
void fx_apply(FXSlot *slot, float *l, float *r, int n);
void fx_init_all(void);

#endif /* DJCMD_FX_H */
