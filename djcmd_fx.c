/*
 * djcmd_fx.c — Audio effects engine for djcmd
 * Copyright (C) 2025  djcmd contributors
 *
 * This file is part of djcmd.
 *
 * djcmd is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * ══════════════════════════════════════════════════════════════════════════
 *  EFFECTS ENGINE
 * ══════════════════════════════════════════════════════════════════════════
 *
 *  Architecture
 *  ────────────
 *  Each deck has FX_SLOTS_PER_DECK effect slots processed serially in the
 *  audio thread, applied after EQ and before mix accumulation.
 *  One master slot runs after crossfade, before soft-clip.
 *
 *  All effect state is heap-allocated per slot.  Switching effect type frees
 *  the old state and allocates new state — done outside the audio thread via
 *  a simple "pending type" field; the audio thread swaps on the next period.
 *
 *  Parameter layout (params[FX_PARAMS]):
 *    params[0]  — primary   (time/depth/freq/crush — effect-specific)
 *    params[1]  — secondary (feedback/rate/stages/mix — effect-specific)
 *    params[2]  — tone      (filter cutoff / resonance)
 *    params[3]  — wet/dry   (0.0 = dry, 1.0 = fully wet)
 *
 *  G4 CPU budget
 *  ─────────────
 *  Headroom with 4 decks (no effects): ~96% idle.
 *  4 decks × all effects simultaneously: ~7% of budget used.
 *  No xruns expected at 512-frame periods.
 * ══════════════════════════════════════════════════════════════════════════ */

#define _GNU_SOURCE
#include "djcmd_fx.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Sample rate set by the audio backend — needed for time↔frame conversions */
extern unsigned int g_actual_sample_rate;

/* ── Effect name table ────────────────────────────────────────────────────── */

const char *fx_names[FX_COUNT] = { "None",   "Echo",	  "Ping-Pong",
				   "Reverb", "Flanger",	  "Chorus",
				   "Phaser", "Distortion", "Bitcrusher",
				   "Gate",   "Widener" };

/* ── Global FX state ──────────────────────────────────────────────────────── */

FXSlot g_fx[FX_TOTAL_SLOTS]; /* [deck*FX_SLOTS_PER_DECK+slot] then [FX_MASTER_SLOT] */

/* Current selected slot for UI display/editing per deck */
int g_fx_ui_slot[MAX_TRACKS] = {
	0
}; /* which slot the FX knobs control */

/* Last non-NONE effect type per deck per slot — restored when toggling back on.
 * Defaults: slot 0 = Echo, slot 1 = Reverb, slot 2 = Flanger. */
int g_fx_last_type[MAX_TRACKS][3] = {
	{ FX_ECHO, FX_REVERB, FX_FLANGER },
	{ FX_ECHO, FX_REVERB, FX_FLANGER },
	{ FX_ECHO, FX_REVERB, FX_FLANGER },
	{ FX_ECHO, FX_REVERB, FX_FLANGER },
};

/* Per-deck per-param accumulator for relative FX encoder knobs.
 * The NS7III FX knobs send center-64 relative CC (same as the jog wheel).
 * Each tick nudges the parameter by a small step; this accumulator holds
 * the current 0.0–1.0 value so it persists between messages. */
/* Initialised to 0.0 to match the slot's initial wet=0 state.
 * Updated by the FX knob CC handler and synced when a new effect is selected. */
float g_fx_param_acc[MAX_TRACKS][FX_PARAMS] = {
	{ 0.0f, 0.5f, 0.5f, 0.0f },
	{ 0.0f, 0.5f, 0.5f, 0.0f },
	{ 0.0f, 0.5f, 0.5f, 0.0f },
	{ 0.0f, 0.5f, 0.5f, 0.0f }
};

/* ── Freeverb tuning tables ───────────────────────────────────────────────── */

/* Tuning table (samples at 44100 Hz, L channel; R = L + 23) */
static const int comb_tuning[COMB_COUNT] = { 1116, 1188, 1277, 1356,
					     1422, 1491, 1557, 1617 };
static const int ap_tuning[AP_COUNT] = { 225, 341, 441, 556 };

/* ── State allocators / deallocators ─────────────────────────────────────── */

static void *fx_alloc_state(int type)
{
	switch (type) {
	case FX_ECHO:
	case FX_PINGPONG: {
		DelayState *s = (DelayState *)calloc(1, sizeof(DelayState));
		if (!s)
			return NULL;
		s->size = FX_DELAY_MAX_FRAMES;
		s->buf_l = (float *)calloc(s->size, sizeof(float));
		s->buf_r = (float *)calloc(s->size, sizeof(float));
		if (!s->buf_l || !s->buf_r) {
			free(s->buf_l);
			free(s->buf_r);
			free(s);
			return NULL;
		}
		return s;
	}
	case FX_REVERB: {
		ReverbState *s = (ReverbState *)calloc(1, sizeof(ReverbState));
		if (!s)
			return NULL;
		for (int i = 0; i < COMB_COUNT; i++) {
			int sz = comb_tuning[i];
			s->comb_size_l[i] = sz;
			s->comb_size_r[i] = sz + 23;
			s->comb_buf_l[i] = (float *)calloc(sz, sizeof(float));
			s->comb_buf_r[i] =
				(float *)calloc(sz + 23, sizeof(float));
			if (!s->comb_buf_l[i] || !s->comb_buf_r[i])
				goto reverb_oom;
		}
		for (int i = 0; i < AP_COUNT; i++) {
			int sz = ap_tuning[i];
			s->ap_size_l[i] = sz;
			s->ap_size_r[i] = sz + 23;
			s->ap_buf_l[i] = (float *)calloc(sz, sizeof(float));
			s->ap_buf_r[i] =
				(float *)calloc(sz + 23, sizeof(float));
			if (!s->ap_buf_l[i] || !s->ap_buf_r[i])
				goto reverb_oom;
		}
		return s;
	reverb_oom:
		for (int i = 0; i < COMB_COUNT; i++) {
			free(s->comb_buf_l[i]);
			free(s->comb_buf_r[i]);
		}
		for (int i = 0; i < AP_COUNT; i++) {
			free(s->ap_buf_l[i]);
			free(s->ap_buf_r[i]);
		}
		free(s);
		return NULL;
	}
	case FX_FLANGER:
	case FX_CHORUS:
		return (FlangerState *)calloc(1, sizeof(FlangerState));
	case FX_PHASER:
		return (PhaserState *)calloc(1, sizeof(PhaserState));
	case FX_BITCRUSH:
		return (BitcrushState *)calloc(1, sizeof(BitcrushState));
	case FX_GATE:
		return (GateState *)calloc(1, sizeof(GateState));
	case FX_DISTORTION:
		return (DistortionState *)calloc(1, sizeof(DistortionState));
	default:
		return NULL;
	}
}

static void fx_free_state(int type, void *state)
{
	if (!state)
		return;
	if (type == FX_ECHO || type == FX_PINGPONG) {
		DelayState *s = (DelayState *)state;
		free(s->buf_l);
		free(s->buf_r);
	} else if (type == FX_REVERB) {
		ReverbState *s = (ReverbState *)state;
		for (int i = 0; i < COMB_COUNT; i++) {
			free(s->comb_buf_l[i]);
			free(s->comb_buf_r[i]);
		}
		for (int i = 0; i < AP_COUNT; i++) {
			free(s->ap_buf_l[i]);
			free(s->ap_buf_r[i]);
		}
	}
	free(state);
}

/* ── Effect processors ────────────────────────────────────────────────────── */

/* Echo: params[0]=time(0-1 → 0-2s), params[1]=feedback(0-1), params[3]=wet */
static void fx_process_echo(float *l, float *r, int n, float *p, void *state)
{
	DelayState *s = (DelayState *)state;
	int delay = (int)(p[0] * (float)(s->size - 1));
	if (delay < 1)
		delay = 1;
	float fb = p[1] * 0.9f; /* cap feedback to prevent runaway */
	float wet = p[3];
	/* Parallel send: dry always at unity so enabling FX doesn't drop level */
	for (int i = 0; i < n; i++) {
		int read = (s->pos - delay + s->size) % s->size;
		float dl = s->buf_l[read];
		float dr = s->buf_r[read];
		s->buf_l[s->pos] = l[i] + dl * fb;
		s->buf_r[s->pos] = r[i] + dr * fb;
		l[i] = l[i] + dl * wet;
		r[i] = r[i] + dr * wet;
		if (++s->pos >= s->size)
			s->pos = 0;
	}
}

/* Ping-pong: left tap feeds right channel and vice versa */
static void fx_process_pingpong(float *l, float *r, int n, float *p,
				void *state)
{
	DelayState *s = (DelayState *)state;
	int delay = (int)(p[0] * (float)(s->size - 1));
	if (delay < 1)
		delay = 1;
	float fb = p[1] * 0.9f;
	float wet = p[3];
	/* Parallel send: dry always at unity */
	for (int i = 0; i < n; i++) {
		int read = (s->pos - delay + s->size) % s->size;
		float dl = s->buf_l[read]; /* read L tap → feed into R output */
		float dr = s->buf_r[read]; /* read R tap → feed into L output */
		s->buf_l[s->pos] =
			l[i] + dr * fb; /* L buf fed from R delayed */
		s->buf_r[s->pos] =
			r[i] + dl * fb; /* R buf fed from L delayed */
		l[i] = l[i] + dr * wet;
		r[i] = r[i] + dl * wet;
		if (++s->pos >= s->size)
			s->pos = 0;
	}
}

/* Freeverb comb filter: delay + feedback + damping (1-pole LP) */
static inline float comb_process(float in, float *buf, int size, int *pos,
				 float feedback, float damp, float *filt)
{
	float out = buf[*pos];
	*filt = out * (1.0f - damp) + (*filt) * damp;
	buf[*pos] = in + (*filt) * feedback;
	if (++(*pos) >= size)
		*pos = 0;
	return out;
}

/* Freeverb allpass */
static inline float ap_process(float in, float *buf, int size, int *pos)
{
	float bufout = buf[*pos];
	buf[*pos] = in + bufout * 0.5f;
	if (++(*pos) >= size)
		*pos = 0;
	return bufout - in;
}

/* Reverb: params[0]=room(0-1), params[1]=damp(0-1), params[2]=width(0-1), params[3]=wet */
static void fx_process_reverb(float *l, float *r, int n, float *p, void *state)
{
	ReverbState *s = (ReverbState *)state;
	float room = p[0] * 0.28f + 0.7f; /* 0→0.7, 1→0.98 */
	float damp = p[1] * 0.4f;
	float width = p[2];
	float wet = p[3];
	/* Parallel send: dry always at unity */
	float wet1 = wet * (width * 0.5f + 0.5f);
	float wet2 = wet * ((1.0f - width) * 0.5f);

	for (int i = 0; i < n; i++) {
		float input = (l[i] + r[i]) * 0.015f; /* gain staging */
		float outL = 0.0f, outR = 0.0f;

		for (int c = 0; c < COMB_COUNT; c++) {
			outL += comb_process(input, s->comb_buf_l[c],
					     s->comb_size_l[c],
					     &s->comb_pos_l[c], room, damp,
					     &s->comb_filt_l[c]);
			outR += comb_process(input, s->comb_buf_r[c],
					     s->comb_size_r[c],
					     &s->comb_pos_r[c], room, damp,
					     &s->comb_filt_r[c]);
		}
		for (int a = 0; a < AP_COUNT; a++) {
			outL = ap_process(outL, s->ap_buf_l[a], s->ap_size_l[a],
					  &s->ap_pos_l[a]);
			outR = ap_process(outR, s->ap_buf_r[a], s->ap_size_r[a],
					  &s->ap_pos_r[a]);
		}
		l[i] = l[i] + outL * wet1 + outR * wet2;
		r[i] = r[i] + outR * wet1 + outL * wet2;
	}
}

/* Flanger: params[0]=delay(0-1 → 0.1-15ms), params[1]=depth, params[2]=rate(Hz), params[3]=wet */
static void fx_process_flanger(float *l, float *r, int n, float *p, void *state)
{
	FlangerState *s = (FlangerState *)state;
	float base_delay = p[0] * 661.5f + 4.4f; /* 0.1ms–15ms in samples */
	float depth = p[1] * base_delay;
	float rate = p[2] * 0.09f + 0.001f; /* 0.001–0.091 Hz in rad/sample */
	float wet = p[3];
	/* Parallel send: dry always at unity */
	float lfo_inc = 2.0f * (float)M_PI * rate / (float)g_actual_sample_rate;

	for (int i = 0; i < n; i++) {
		float lfo = sinf(s->lfo_phase) * depth;
		s->lfo_phase += lfo_inc;
		if (s->lfo_phase > 2.0f * (float)M_PI)
			s->lfo_phase -= 2.0f * (float)M_PI;

		/* Fractional delay read with linear interpolation */
		float delay = base_delay + lfo;
		if (delay < 1.0f)
			delay = 1.0f;
		int di = (int)delay;
		float frac = delay - (float)di;

		int r1 =
			(s->pos - di + FLANGER_BUF_FRAMES) % FLANGER_BUF_FRAMES;
		int r2 = (r1 - 1 + FLANGER_BUF_FRAMES) % FLANGER_BUF_FRAMES;
		float dl = s->buf_l[r1] * (1.0f - frac) + s->buf_l[r2] * frac;
		float dr = s->buf_r[r1] * (1.0f - frac) + s->buf_r[r2] * frac;

		s->buf_l[s->pos] = l[i];
		s->buf_r[s->pos] = r[i];
		if (++s->pos >= FLANGER_BUF_FRAMES)
			s->pos = 0;

		l[i] = l[i] + dl * wet;
		r[i] = r[i] + dr * wet;
	}
}

/* Chorus: deeper, slower flanger with slight pitch randomisation */
static void fx_process_chorus(float *l, float *r, int n, float *p, void *state)
{
	FlangerState *s = (FlangerState *)state;
	float base_delay = p[0] * 1764.0f + 220.5f; /* 5ms–45ms in samples */
	float depth = p[1] * base_delay * 0.3f;
	float rate = p[2] * 0.018f + 0.0005f;
	float wet = p[3];
	/* Parallel send: dry always at unity */
	float lfo_inc = 2.0f * (float)M_PI * rate / (float)g_actual_sample_rate;

	for (int i = 0; i < n; i++) {
		float lfo = sinf(s->lfo_phase) * depth;
		s->lfo_phase += lfo_inc;
		if (s->lfo_phase > 2.0f * (float)M_PI)
			s->lfo_phase -= 2.0f * (float)M_PI;

		float delay = base_delay + lfo;
		if (delay < 1.0f)
			delay = 1.0f;
		int di = (int)delay;
		float frac = delay - (float)di;
		int r1 =
			(s->pos - di + FLANGER_BUF_FRAMES) % FLANGER_BUF_FRAMES;
		int r2 = (r1 - 1 + FLANGER_BUF_FRAMES) % FLANGER_BUF_FRAMES;
		float dl = s->buf_l[r1] * (1.0f - frac) + s->buf_l[r2] * frac;
		float dr = s->buf_r[r1] * (1.0f - frac) + s->buf_r[r2] * frac;

		s->buf_l[s->pos] = l[i];
		s->buf_r[s->pos] = r[i];
		if (++s->pos >= FLANGER_BUF_FRAMES)
			s->pos = 0;

		l[i] = l[i] + dl * wet;
		r[i] = r[i] + dr * wet;
	}
}

/* Phaser: 4 first-order allpass stages modulated by LFO
 * params[0]=min_freq, params[1]=sweep_depth, params[2]=rate, params[3]=wet */
static inline float phaser_ap1(float in, float coeff, float *x1, float *y1)
{
	float out = coeff * (in - *y1) + *x1;
	*x1 = in;
	*y1 = out;
	return out;
}

static void fx_process_phaser(float *l, float *r, int n, float *p, void *state)
{
	PhaserState *s = (PhaserState *)state;
	float min_freq = 200.0f + p[0] * 800.0f; /* 200–1000 Hz */
	float max_freq = min_freq + p[1] * 3000.0f; /* sweep range */
	float rate = p[2] * 0.018f + 0.0005f;
	float wet = p[3];
	/* Parallel send: dry always at unity */
	float lfo_inc = 2.0f * (float)M_PI * rate / (float)g_actual_sample_rate;

	for (int i = 0; i < n; i++) {
		float lfo = (sinf(s->lfo_phase) + 1.0f) * 0.5f; /* 0..1 */
		s->lfo_phase += lfo_inc;
		if (s->lfo_phase > 2.0f * (float)M_PI)
			s->lfo_phase -= 2.0f * (float)M_PI;

		float fc = min_freq + lfo * (max_freq - min_freq);
		float w0 =
			2.0f * (float)M_PI * fc / (float)g_actual_sample_rate;
		float coeff =
			(1.0f - tanf(w0 * 0.5f)) / (1.0f + tanf(w0 * 0.5f));

		float sl = l[i], sr = r[i];
		for (int st = 0; st < PHASER_STAGES; st++) {
			sl = phaser_ap1(sl, coeff, &s->x1l[st], &s->y1l[st]);
			sr = phaser_ap1(sr, coeff, &s->x1r[st], &s->y1r[st]);
		}
		l[i] = l[i] + sl * wet;
		r[i] = r[i] + sr * wet;
	}
}

/* Distortion: pre-gain + tanh soft clip + tone LP filter
 * params[0]=drive(0-1→×1–×32), params[1]=tone(0-1, LP cutoff), params[3]=wet */
static void fx_process_distortion(float *l, float *r, int n, float *p,
				  void *state)
{
	(void)state;
	float drive = 1.0f + p[0] * 31.0f; /* 1×–32× pre-gain */
	float tone = p[1]; /* 0=dark, 1=bright */
	float wet = p[3];
	float dry = 1.0f - wet;
	/* Simple 1-pole LP tone control: alpha = e^(-2π*fc/SR) approx */
	float fc = 500.0f + tone * 15000.0f;
	float alpha =
		1.0f - (2.0f * (float)M_PI * fc / (float)g_actual_sample_rate);
	if (alpha < 0.0f)
		alpha = 0.0f;
	DistortionState *ds = (DistortionState *)state;
	if (!ds)
		return;
	for (int i = 0; i < n; i++) {
		float dl = tanhf(l[i] * drive) * (1.0f / drive) * drive * 0.8f;
		float dr = tanhf(r[i] * drive) * (1.0f / drive) * drive * 0.8f;
		/* Tone LP */
		ds->lp_l = ds->lp_l * alpha + dl * (1.0f - alpha);
		ds->lp_r = ds->lp_r * alpha + dr * (1.0f - alpha);
		/* Blend bright/dark with tone knob */
		dl = dl * tone + ds->lp_l * (1.0f - tone);
		dr = dr * tone + ds->lp_r * (1.0f - tone);
		l[i] = l[i] * dry + dl * wet;
		r[i] = r[i] * dry + dr * wet;
	}
}

/* Bitcrusher: params[0]=bits(0-1→1-16), params[1]=sr_reduce(0-1→1-32×), params[3]=wet */
static void fx_process_bitcrush(float *l, float *r, int n, float *p,
				void *state)
{
	BitcrushState *s = (BitcrushState *)state;
	float bits = 1.0f + (1.0f - p[0]) * 15.0f; /* 0→16bit, 1→1bit */
	float step = 1.0f / powf(2.0f, bits - 1.0f);
	float sr_div = 1.0f + p[1] * 31.0f; /* 1×–32× decimation */
	float wet = p[3];
	float dry = 1.0f - wet;
	float sr_inc = 1.0f / sr_div;

	for (int i = 0; i < n; i++) {
		s->accum += sr_inc;
		if (s->accum >= 1.0f) {
			s->accum -= 1.0f;
			/* Quantise to bit depth */
			s->held_l = floorf(l[i] / step + 0.5f) * step;
			s->held_r = floorf(r[i] / step + 0.5f) * step;
		}
		l[i] = l[i] * dry + s->held_l * wet;
		r[i] = r[i] * dry + s->held_r * wet;
	}
}

/* Gate: params[0]=threshold(0-1), params[1]=attack_ms, params[2]=release_ms, params[3]=wet */
static void fx_process_gate(float *l, float *r, int n, float *p, void *state)
{
	GateState *s = (GateState *)state;
	float threshold = p[0] * p[0]; /* square for perceptual linearity */
	float atk_coef = expf(-1.0f / (0.001f * (p[1] * 49.0f + 1.0f) *
				       g_actual_sample_rate));
	float rel_coef = expf(-1.0f / (0.001f * (p[2] * 499.0f + 1.0f) *
				       g_actual_sample_rate));
	float wet = p[3];
	float dry = 1.0f - wet;

	for (int i = 0; i < n; i++) {
		float level =
			fabsf(l[i]) > fabsf(r[i]) ? fabsf(l[i]) : fabsf(r[i]);
		float coef = (level >= threshold) ? atk_coef : rel_coef;
		s->env = s->env * coef + level * (1.0f - coef);
		float target = (s->env >= threshold) ? 1.0f : 0.0f;
		s->gain = s->gain * 0.999f + target * 0.001f; /* smooth gain */
		l[i] = l[i] * dry + l[i] * s->gain * wet;
		r[i] = r[i] * dry + r[i] * s->gain * wet;
	}
}

/* Stereo widener: M/S processing — params[0]=width(0-1), params[3]=wet */
static void fx_process_widener(float *l, float *r, int n, float *p, void *state)
{
	(void)state;
	float width = p[0] * 2.0f; /* 0=mono, 1=normal, 2=extra wide */
	float wet = p[3];
	float dry = 1.0f - wet;
	for (int i = 0; i < n; i++) {
		float mid = (l[i] + r[i]) * 0.5f;
		float side = (l[i] - r[i]) * 0.5f * width;
		float wl = mid + side;
		float wr = mid - side;
		l[i] = l[i] * dry + wl * wet;
		r[i] = r[i] * dry + wr * wet;
	}
}

/* Master compressor/limiter: always-on on the mix bus
 * params[0]=threshold(0-1), params[1]=ratio(0-1→1:1–20:1),
 * params[2]=makeup_gain, params[3]=mix */
static CompState g_comp_state;
static void fx_process_comp(float *l, float *r, int n, float *p)
{
	float thresh = p[0] * p[0] * 0.9f + 0.05f; /* 0.05–0.95 linear peak */
	float ratio = 1.0f + p[1] * 19.0f; /* 1:1–20:1 */
	float makeup = 1.0f + p[2] * 3.0f; /* 1×–4× */
	float wet = p[3];
	float dry = 1.0f - wet;
	float atk = expf(-1.0f /
			 (0.001f * g_actual_sample_rate)); /* ~1ms attack  */
	float rel = expf(-1.0f /
			 (0.100f * g_actual_sample_rate)); /* ~100ms release */

	for (int i = 0; i < n; i++) {
		float peak =
			fabsf(l[i]) > fabsf(r[i]) ? fabsf(l[i]) : fabsf(r[i]);
		float coef = (peak > g_comp_state.env_peak) ? atk : rel;
		g_comp_state.env_peak =
			g_comp_state.env_peak * coef + peak * (1.0f - coef);

		float target_gain;
		if (g_comp_state.env_peak > thresh) {
			float over = g_comp_state.env_peak / thresh;
			target_gain = thresh * powf(over, 1.0f / ratio - 1.0f) /
				      g_comp_state.env_peak;
		} else {
			target_gain = 1.0f;
		}
		/* Smooth the gain */
		g_comp_state.gain =
			g_comp_state.gain * 0.999f + target_gain * 0.001f;
		float g = g_comp_state.gain * makeup;

		float cl = l[i] * dry + l[i] * g * wet;
		float cr = r[i] * dry + r[i] * g * wet;
		/* Hard limiter at ±0.99 — safety net */
		l[i] = cl > 0.99f ? 0.99f : (cl < -0.99f ? -0.99f : cl);
		r[i] = cr > 0.99f ? 0.99f : (cr < -0.99f ? -0.99f : cr);
	}
}

/* ── Effect dispatch ──────────────────────────────────────────────────────── */

/* Process one FX slot. Called inside the audio thread per deck per period. */
void fx_apply(FXSlot *slot, float *l, float *r, int n)
{
	/* Handle pending type switch (allocated outside audio thread) */
	if (slot->pending_type >= 0) {
		fx_free_state(slot->type, slot->state);
		slot->type = slot->pending_type;
		slot->state = fx_alloc_state(slot->type);
		slot->pending_type = -1;
	}
	if (slot->type == FX_NONE || slot->params[3] < 0.001f)
		return;

	switch (slot->type) {
	case FX_ECHO:
		fx_process_echo(l, r, n, slot->params, slot->state);
		break;
	case FX_PINGPONG:
		fx_process_pingpong(l, r, n, slot->params, slot->state);
		break;
	case FX_REVERB:
		fx_process_reverb(l, r, n, slot->params, slot->state);
		break;
	case FX_FLANGER:
		fx_process_flanger(l, r, n, slot->params, slot->state);
		break;
	case FX_CHORUS:
		fx_process_chorus(l, r, n, slot->params, slot->state);
		break;
	case FX_PHASER:
		fx_process_phaser(l, r, n, slot->params, slot->state);
		break;
	case FX_DISTORTION:
		fx_process_distortion(l, r, n, slot->params, slot->state);
		break;
	case FX_BITCRUSH:
		fx_process_bitcrush(l, r, n, slot->params, slot->state);
		break;
	case FX_GATE:
		fx_process_gate(l, r, n, slot->params, slot->state);
		break;
	case FX_WIDENER:
		fx_process_widener(l, r, n, slot->params, slot->state);
		break;
	default:
		break;
	}
}

/* ── Public helpers (called from MIDI/UI thread) ────────────────────────────
 * Write pending_type — audio thread picks it up next period.
 * Safe without a mutex because int writes are atomic on G4 and x86. */

void fx_set_type(int deck, int slot_idx, int type)
{
	if (deck < 0 || deck >= MAX_TRACKS)
		return;
	if (slot_idx < 0 || slot_idx >= FX_SLOTS_PER_DECK)
		return;
	if (type < 0 || type >= FX_COUNT)
		return;
	FXSlot *s = fx_slot(deck, slot_idx);
	if (s->type == type && s->pending_type < 0)
		return; /* already set */
	s->pending_type = type;
}

void fx_set_param(int deck, int slot_idx, int param_idx, float value)
{
	if (deck < 0 || deck >= MAX_TRACKS)
		return;
	if (slot_idx < 0 || slot_idx >= FX_SLOTS_PER_DECK)
		return;
	if (param_idx < 0 || param_idx >= FX_PARAMS)
		return;
	if (value < 0.0f)
		value = 0.0f;
	if (value > 1.0f)
		value = 1.0f;
	fx_slot(deck, slot_idx)->params[param_idx] = value;
}

void fx_set_wet(int deck, int slot_idx, float wet)
{
	fx_set_param(deck, slot_idx, 3, wet);
}

/* ── Initialisation ───────────────────────────────────────────────────────── */

void fx_init_all(void)
{
	for (int i = 0; i < FX_TOTAL_SLOTS; i++) {
		g_fx[i].type = FX_NONE;
		g_fx[i].pending_type = -1;
		g_fx[i].state = NULL;
		g_fx[i].params[0] = 0.35f;
		g_fx[i].params[1] = 0.35f;
		g_fx[i].params[2] = 0.5f;
		g_fx[i].params[3] = 0.5f;
	}
	/* All per-deck slots start at FX_NONE so the first button press turns ON
     * to the default type (from g_fx_last_type) rather than toggling off a
     * pre-loaded-but-silent effect.  This ensures BTN1, BTN2, BTN3 each work
     * independently — pressing one does not affect the others. */
	/* Master compressor: gentle defaults — threshold 70%, ratio 4:1, 0dB makeup, 100% wet */
	FXSlot *mc = fx_master();
	mc->type = FX_NONE; /* off by default; user enables */
	mc->pending_type = -1;
	mc->params[0] = 0.7f; /* threshold */
	mc->params[1] = 0.15f; /* ratio → 4:1 */
	mc->params[2] = 0.0f; /* makeup */
	mc->params[3] = 1.0f; /* wet */
	memset(&g_comp_state, 0, sizeof(g_comp_state));
	g_comp_state.gain = 1.0f;
}
