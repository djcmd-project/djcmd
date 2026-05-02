/*
 * djcmd_audio.c — Audio processing engine for djcmd
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

#include "djcmd_audio.h"
#include "audiofile.h"
#include "djcmd_fx.h"
#include "djcmd_library.h"
#include "djcmd_midi.h"
#include "djcmd_ui.h"
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define SAMPLE_RATE CFG_SAMPLE_RATE
#define CHANNELS CFG_CHANNELS
#define PERIOD_FRAMES CFG_PERIOD_FRAMES
#define BUFFER_PERIODS CFG_BUFFER_PERIODS
#define NUDGE_DECAY CFG_NUDGE_DECAY
#define WFM_OVERVIEW_BINS CFG_WFM_OVERVIEW_BINS
#define ONSET_HOP 512
#define BPM_AC_LO 25.0f
#define BPM_AC_HI 220.0f

/* SIMD headers */
#if defined(__SSE2__)
#include <emmintrin.h>
#elif defined(__ALTIVEC__)
#include <altivec.h>
/* AltiVec uses 'vector' keyword; undefine any potential conflicts */
#undef vector
#define vfloat __vector float
#endif

/* Twiddle tables for Phase Vocoder FFT */
static float g_pv_cos[PV_N];
static float g_pv_sin[PV_N];

/* ──────────────────────────────────────────────
   Phase Vocoder (PV) internal functions
   ────────────────────────────────────────────── */

void pv_init_tables(void)
{
	for (int k = 0; k < PV_N; k++) {
		double a = 2.0 * M_PI * k / PV_N;
		g_pv_cos[k] = (float)cos(a);
		g_pv_sin[k] = (float)sin(a);
	}
}

static void pv_fft(float *re, float *im, int n, int forward)
{
	for (int i = 1, j = 0; i < n; i++) {
		int bit = n >> 1;
		for (; j & bit; bit >>= 1) {
			j ^= bit;
		}
		j ^= bit;
		if (i < j) {
			float t;
			t = re[i];
			re[i] = re[j];
			re[j] = t;
			t = im[i];
			im[i] = im[j];
			im[j] = t;
		}
	}
	for (int m = 1; m < n; m <<= 1) {
		int step = PV_N / (m << 1);
		for (int k = 0; k < m; k++) {
			float wr = g_pv_cos[k * step];
			float wi =
			    (forward ? -1.0f : 1.0f) * g_pv_sin[k * step];
			for (int j = k; j < n; j += m << 1) {
				float ur = re[j], ui = im[j];
				float vr = re[j + m] * wr - im[j + m] * wi;
				float vi = re[j + m] * wi + im[j + m] * wr;
				re[j] = ur + vr;
				im[j] = ui + vi;
				re[j + m] = ur - vr;
				im[j + m] = ui - vi;
			}
		}
	}
}

void pv_reset(PVState *pv, uint32_t start_pos)
{
	memset(pv, 0, sizeof(PVState));
	pv->src_pos = (double)start_pos;
}

static void pv_hop(Track *t, PVState *pv, float ha)
{
	int src0 = (int)pv->src_pos;
	int max_src = (int)t->num_frames - PV_N;
	if (src0 < 0) {
		src0 = 0;
	}
	if (src0 > max_src) {
		src0 = (max_src > 0) ? max_src : 0;
	}

	float re_l[PV_N], im_l[PV_N];
	float re_r[PV_N], im_r[PV_N];

	for (int i = 0; i < PV_N; i++) {
		float w = 0.5f * (1.0f - g_pv_cos[i]);
		int fi = src0 + i;
		if (fi >= (int)t->num_frames) {
			fi = (int)t->num_frames - 1;
		}
		re_l[i] = (t->data[fi * 2] / 32768.0f) * w;
		im_l[i] = 0.0f;
		re_r[i] = (t->data[fi * 2 + 1] / 32768.0f) * w;
		im_r[i] = 0.0f;
	}

	pv_fft(re_l, im_l, PV_N, 1);
	pv_fft(re_r, im_r, PV_N, 1);

	float two_pi = 2.0f * (float)M_PI;
	float inv_ha = (fabsf(ha) >= 1.0f) ? (1.0f / ha) : 0.0f;

	for (int k = 0; k < PV_BINS; k++) {
		float omega_k = two_pi * (float)k / (float)PV_N;
		float expected = omega_k * ha;

		float an_l = atan2f(im_l[k], re_l[k]);
		float mag_l = sqrtf(re_l[k] * re_l[k] + im_l[k] * im_l[k]);
		if (pv->initialized) {
			float dev = an_l - pv->ph_an_l[k] - expected;
			dev -= two_pi * floorf(dev / two_pi + 0.5f);
			pv->ph_syn_l[k] +=
			    (omega_k + dev * inv_ha) * (float)PV_HS;
		} else {
			pv->ph_syn_l[k] = an_l;
		}
		pv->ph_an_l[k] = an_l;
		re_l[k] = mag_l * cosf(pv->ph_syn_l[k]);
		im_l[k] = mag_l * sinf(pv->ph_syn_l[k]);

		float an_r = atan2f(im_r[k], re_r[k]);
		float mag_r = sqrtf(re_r[k] * re_r[k] + im_r[k] * im_r[k]);
		if (pv->initialized) {
			float dev = an_r - pv->ph_an_r[k] - expected;
			dev -= two_pi * floorf(dev / two_pi + 0.5f);
			pv->ph_syn_r[k] +=
			    (omega_k + dev * inv_ha) * (float)PV_HS;
		} else {
			pv->ph_syn_r[k] = an_r;
		}
		pv->ph_an_r[k] = an_r;
		re_r[k] = mag_r * cosf(pv->ph_syn_r[k]);
		im_r[k] = mag_r * sinf(pv->ph_syn_r[k]);
	}

	for (int k = PV_BINS; k < PV_N; k++) {
		int m = PV_N - k;
		re_l[k] = re_l[m];
		im_l[k] = -im_l[m];
		re_r[k] = re_r[m];
		im_r[k] = -im_r[m];
	}

	pv_fft(re_l, im_l, PV_N, 0);
	pv_fft(re_r, im_r, PV_N, 0);

	float norm = 2.0f / (3.0f * (float)PV_N);

	for (int i = 0; i < PV_N; i++) {
		float w = 0.5f * (1.0f - g_pv_cos[i]);
		float ol = re_l[i] * norm * w;
		float or_ = re_r[i] * norm * w;
		int rp = (pv->out_write - (PV_N - PV_HS) + i + PV_BUF * 4) &
		         (PV_BUF - 1);
		pv->out_l[rp] += ol;
		pv->out_r[rp] += or_;
	}

	pv->out_write = (pv->out_write + PV_HS) & (PV_BUF - 1);
	pv->out_fill += PV_HS;
	pv->src_pos += (double)ha;
	pv->initialized = 1;
}

void pv_process(Track *t, PVState *pv, float *out_l, float *out_r,
                uint32_t out_frames, double rate, float gain)
{
	if (rate < 0.0) {
		return;
	}

	while (pv->out_fill < (int)out_frames) {
		pv_hop(t, pv, (float)(rate * PV_HS));
	}

	for (uint32_t i = 0; i < out_frames; i++) {
		out_l[i] = pv->out_l[pv->out_read] * gain;
		out_r[i] = pv->out_r[pv->out_read] * gain;
		pv->out_l[pv->out_read] = 0.0f;
		pv->out_r[pv->out_read] = 0.0f;
		pv->out_read = (pv->out_read + 1) & (PV_BUF - 1);
		pv->out_fill--;
	}
	t->pos = (uint32_t)pv->src_pos;
}

/* ──────────────────────────────────────────────
   WSOLA (Waveform Similarity Overlap-Add)
   ────────────────────────────────────────────── */

static void wsola_hann(float *win, int n)
{
	for (int i = 0; i < n; i++) {
		float w =
		    0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (n - 1)));
		win[i] *= w;
	}
}

static int wsola_find_best(const float *ref, const float *cand_base,
                           int cand_center, int cand_len, int win, int range,
                           int is_eco)
{
	int half = range / 2;
	int best_d = 0;
	float best_corr = -1e30f;

	if (is_eco) {
		/* ECO mode: skip every other d, and sparse correlation search
		 * (1/8 samples) */
		for (int d = -half; d <= half; d += 2) {
			int start = cand_center + d;
			if (start < 0 || start + win > cand_len) {
				continue;
			}
			float corr = 0.0f;
			for (int i = 0; i < win; i += 8) {
				corr += ref[i] * cand_base[start + i];
			}
			if (corr > best_corr) {
				best_corr = corr;
				best_d = d;
			}
		}
	} else {
		/* Normal mode: every d, and 1/4 sample correlation search */
		for (int d = -half; d <= half; d++) {
			int start = cand_center + d;
			if (start < 0 || start + win > cand_len) {
				continue;
			}
			float corr = 0.0f;
			for (int i = 0; i < win; i += 4) {
				corr += ref[i] * cand_base[start + i];
			}
			if (corr > best_corr) {
				best_corr = corr;
				best_d = d;
			}
		}
	}
	return best_d;
}

void wsola_process(Track *t, WSOLAState *ws, float *out_l, float *out_r,
                   uint32_t out_frames, double rate, float vol_gain, int is_eco)
{
	uint32_t end = t->looping ? t->loop_end : t->num_frames;
	float win_l[WSOLA_WIN] __attribute__((aligned(32)));
	float win_r[WSOLA_WIN] __attribute__((aligned(32)));

	while (ws->fill < (int)out_frames) {
		int src = (int)ws->src_pos;
		if (src >= (int)end) {
			if (t->looping) {
				ws->src_pos = (double)t->loop_start;
				src = t->loop_start;
			} else {
				t->playing = 0;
				while (ws->fill < (int)out_frames) {
					ws->buf_l[ws->write_pos &
					          (WSOLA_BUF - 1)] = 0.0f;
					ws->buf_r[ws->write_pos &
					          (WSOLA_BUF - 1)] = 0.0f;
					ws->write_pos++;
					ws->fill++;
				}
				break;
			}
		}

		int best_offset = 0;
		if (ws->prev_valid) {
			float ref[WSOLA_WIN] __attribute__((aligned(32)));
			for (int i = 0; i < WSOLA_WIN; i++) {
				ref[i] = (ws->prev_l[i] + ws->prev_r[i]) * 0.5f;
			}

			float cand[WSOLA_WIN + WSOLA_SEARCH]
			    __attribute__((aligned(32)));
			int cand_start = src - WSOLA_SEARCH / 2;
			if (cand_start < 0) {
				cand_start = 0;
			}
			int cand_len = WSOLA_WIN + WSOLA_SEARCH;
			for (int i = 0; i < cand_len; i++) {
				int fi = cand_start + i;
				if (fi < (int)end) {
					cand[i] = (t->data[fi * 2] +
					           t->data[fi * 2 + 1]) /
					          65536.0f;
				} else {
					cand[i] = 0.0f;
				}
			}
			best_offset = wsola_find_best(
			    ref, cand, src - cand_start, cand_len, WSOLA_WIN,
			    WSOLA_SEARCH, is_eco);
		}

		int read_pos = src + best_offset;
		if (read_pos < 0) {
			read_pos = 0;
		}

		for (int i = 0; i < WSOLA_WIN; i++) {
			int fi = read_pos + i;
			if (fi < (int)end) {
				win_l[i] = t->data[fi * 2] / 32768.0f;
				win_r[i] = t->data[fi * 2 + 1] / 32768.0f;
			} else {
				win_l[i] = win_r[i] = 0.0f;
			}
		}
		wsola_hann(win_l, WSOLA_WIN);
		wsola_hann(win_r, WSOLA_WIN);

		for (int i = 0; i < WSOLA_WIN; i++) {
			int rp = (ws->write_pos - (WSOLA_WIN - WSOLA_HOP) + i +
			          WSOLA_BUF * 4) &
			         (WSOLA_BUF - 1);
			ws->buf_l[rp] += win_l[i];
			ws->buf_r[rp] += win_r[i];
		}

		memcpy(ws->prev_l, win_l, sizeof(float) * WSOLA_WIN);
		memcpy(ws->prev_r, win_r, sizeof(float) * WSOLA_WIN);
		ws->prev_valid = 1;
		ws->write_pos = (ws->write_pos + WSOLA_HOP) & (WSOLA_BUF - 1);
		ws->fill += WSOLA_HOP;
		ws->src_pos += rate * WSOLA_HOP;
		if (ws->src_pos < 0.0) {
			ws->src_pos = 0.0;
		}
	}

	for (uint32_t i = 0; i < out_frames; i++) {
		out_l[i] = ws->buf_l[ws->read_pos & (WSOLA_BUF - 1)] * vol_gain;
		out_r[i] = ws->buf_r[ws->read_pos & (WSOLA_BUF - 1)] * vol_gain;
		ws->buf_l[ws->read_pos & (WSOLA_BUF - 1)] = 0.0f;
		ws->buf_r[ws->read_pos & (WSOLA_BUF - 1)] = 0.0f;
		ws->read_pos = (ws->read_pos + 1) & (WSOLA_BUF - 1);
		ws->fill--;
	}
	t->pos = (uint32_t)ws->src_pos;
}

void wsola_reset(WSOLAState *ws, uint32_t start_pos)
{
	memset(ws, 0, sizeof(WSOLAState));
	ws->src_pos = (double)start_pos;
}

/* ──────────────────────────────────────────────
   Hermite Resampler with Velocity Ramping
   ────────────────────────────────────────────── */

void read_pitched(Track *t, float *out_l, float *out_r, uint32_t out_frames,
                  float start_vel, float target_vel, float start_nudge,
                  float target_nudge)
{
	float vel_inc = (target_vel - start_vel) / (float)out_frames;
	float nudge_inc = (target_nudge - start_nudge) / (float)out_frames;
	float current_vel = start_vel;
	float current_nudge = start_nudge;
	float vol_gain = t->volume * t->gain;

	double fpos = (double)t->pos;
	uint32_t end = t->looping ? t->loop_end : t->num_frames;

	/* Pre-scale volume for SIMD */
	float vgain = vol_gain;

	uint32_t i = 0;
#if defined(__ALTIVEC__)
	/* AltiVec 4-lane optimization: process 2 stereo samples at once */
	for (; i + 1 < out_frames; i += 2) {
		if (fpos < 0.0) {
			fpos = 0.0;
		}
		uint32_t idx1 = (uint32_t)fpos;
		float frac1 = (float)(fpos - (double)idx1);

		float next_fpos =
		    fpos + (double)(t->pitch + current_vel + current_nudge);
		if (next_fpos < 0.0) {
			next_fpos = 0.0;
		}
		uint32_t idx2 = (uint32_t)next_fpos;
		float frac2 = (float)(next_fpos - (double)idx2);

		if (idx1 >= end || idx2 >= end) {
			/* Fallback to scalar for end-of-track or loop-wrap
			 * logic */
			break;
		}

		/* Sample 1 indices */
		uint32_t i1_0 = (idx1 > 0) ? idx1 - 1 : 0;
		uint32_t i1_1 = idx1;
		uint32_t i1_2 = (idx1 + 1 < end) ? idx1 + 1 : end - 1;
		uint32_t i1_3 = (idx1 + 2 < end) ? idx1 + 2 : end - 1;

		/* Sample 2 indices */
		uint32_t i2_0 = (idx2 > 0) ? idx2 - 1 : 0;
		uint32_t i2_1 = idx2;
		uint32_t i2_2 = (idx2 + 1 < end) ? idx2 + 1 : end - 1;
		uint32_t i2_3 = (idx2 + 2 < end) ? idx2 + 2 : end - 1;

		/* Vectorized load: {L1, R1, L2, R2} */
		float p1_0 = (float)t->data[i1_0 * 2] * (1.0f / 32768.0f);
		float p1_1 = (float)t->data[i1_0 * 2 + 1] * (1.0f / 32768.0f);
		float p2_0 = (float)t->data[i2_0 * 2] * (1.0f / 32768.0f);
		float p2_1 = (float)t->data[i2_0 * 2 + 1] * (1.0f / 32768.0f);
		vfloat v_p0 = (vfloat){p1_0, p1_1, p2_0, p2_1};

		p1_0 = (float)t->data[i1_1 * 2] * (1.0f / 32768.0f);
		p1_1 = (float)t->data[i1_1 * 2 + 1] * (1.0f / 32768.0f);
		p2_0 = (float)t->data[i2_1 * 2] * (1.0f / 32768.0f);
		p2_1 = (float)t->data[i2_1 * 2 + 1] * (1.0f / 32768.0f);
		vfloat v_p1 = (vfloat){p1_0, p1_1, p2_0, p2_1};

		p1_0 = (float)t->data[i1_2 * 2] * (1.0f / 32768.0f);
		p1_1 = (float)t->data[i1_2 * 2 + 1] * (1.0f / 32768.0f);
		p2_0 = (float)t->data[i2_2 * 2] * (1.0f / 32768.0f);
		p2_1 = (float)t->data[i2_2 * 2 + 1] * (1.0f / 32768.0f);
		vfloat v_p2 = (vfloat){p1_0, p1_1, p2_0, p2_1};

		p1_0 = (float)t->data[i1_3 * 2] * (1.0f / 32768.0f);
		p1_1 = (float)t->data[i1_3 * 2 + 1] * (1.0f / 32768.0f);
		p2_0 = (float)t->data[i2_3 * 2] * (1.0f / 32768.0f);
		p2_1 = (float)t->data[i2_3 * 2 + 1] * (1.0f / 32768.0f);
		vfloat v_p3 = (vfloat){p1_0, p1_1, p2_0, p2_1};

		vfloat v_frac = (vfloat){frac1, frac1, frac2, frac2};
		vfloat v_gain = (vfloat){vgain, vgain, vgain, vgain};
		vfloat v_05 = (vfloat){0.5f, 0.5f, 0.5f, 0.5f};
		vfloat v_15 = (vfloat){1.5f, 1.5f, 1.5f, 1.5f};
		vfloat v_20 = (vfloat){2.0f, 2.0f, 2.0f, 2.0f};
		vfloat v_25 = (vfloat){2.5f, 2.5f, 2.5f, 2.5f};
		vfloat v_zero = (vfloat){0.0f, 0.0f, 0.0f, 0.0f};

		/* c1 = 0.5 * (p2 - p0) */
		vfloat v_c1 = vec_madd(v_05, vec_sub(v_p2, v_p0), v_zero);
		/* c2 = p0 - 2.5*p1 + 2.0*p2 - 0.5*p3 */
		vfloat v_c2 =
		    vec_add(vec_sub(v_p0, vec_madd(v_25, v_p1, v_zero)),
		            vec_sub(vec_madd(v_20, v_p2, v_zero),
		                    vec_madd(v_05, v_p3, v_zero)));
		/* c3 = 0.5*(p3 - p0) + 1.5*(p1 - p2) */
		vfloat v_c3 =
		    vec_add(vec_madd(v_05, vec_sub(v_p3, v_p0), v_zero),
		            vec_madd(v_15, vec_sub(v_p1, v_p2), v_zero));

		/* Horner form: out = ((c3*f + c2)*f + c1)*f + p1 */
		vfloat v_out = vec_madd(v_c3, v_frac, v_c2);
		v_out = vec_madd(v_out, v_frac, v_c1);
		v_out = vec_madd(v_out, v_frac, v_p1);
		v_out = vec_madd(v_out, v_gain, v_zero);

		float res[4] __attribute__((aligned(16)));
		vec_st(v_out, 0, res);
		out_l[i] = res[0];
		out_r[i] = res[1];
		out_l[i + 1] = res[2];
		out_r[i + 1] = res[3];

		fpos = next_fpos + (double)(t->pitch + current_vel + vel_inc +
		                            current_nudge + nudge_inc);
		current_vel += vel_inc * 2.0f;
		current_nudge += nudge_inc * 2.0f;
	}
#endif

	for (; i < out_frames; i++) {
		if (fpos < 0.0) {
			fpos = 0.0;
		}
		uint32_t idx = (uint32_t)fpos;
		float frac = (float)(fpos - (double)idx);

		if (idx >= end) {
			if (t->looping) {
				fpos = (double)t->loop_start;
				idx = t->loop_start;
				frac = 0.0f;
			} else {
				t->playing = 0;
				out_l[i] = out_r[i] = 0.0f;
				fpos = (double)end;
				continue;
			}
		}

		uint32_t i1 = idx;
		uint32_t i0 = (idx > 0) ? idx - 1 : 0;
		uint32_t i2 = (idx + 1 < end) ? idx + 1 : end - 1;
		uint32_t i3 = (idx + 2 < end) ? idx + 2 : end - 1;

		/* Load 4-point window for L and R */
		float p0l = t->data[i0 * 2] / 32768.0f;
		float p1l = t->data[i1 * 2] / 32768.0f;
		float p2l = t->data[i2 * 2] / 32768.0f;
		float p3l = t->data[i3 * 2] / 32768.0f;
		float p0r = t->data[i0 * 2 + 1] / 32768.0f;
		float p1r = t->data[i1 * 2 + 1] / 32768.0f;
		float p2r = t->data[i2 * 2 + 1] / 32768.0f;
		float p3r = t->data[i3 * 2 + 1] / 32768.0f;

#if defined(__SSE2__)
		/* SSE2 implementation: process L and R simultaneously */
		__m128 v_p0 = _mm_set_ps(0.0f, 0.0f, p0r, p0l);
		__m128 v_p1 = _mm_set_ps(0.0f, 0.0f, p1r, p1l);
		__m128 v_p2 = _mm_set_ps(0.0f, 0.0f, p2r, p2l);
		__m128 v_p3 = _mm_set_ps(0.0f, 0.0f, p3r, p3l);
		__m128 v_frac = _mm_set1_ps(frac);
		__m128 v_gain = _mm_set1_ps(vgain);

		/* Hermite coefficients */
		__m128 v_c1 =
		    _mm_mul_ps(_mm_set1_ps(0.5f), _mm_sub_ps(v_p2, v_p0));
		__m128 v_c2 = _mm_sub_ps(
		    _mm_add_ps(v_p0, _mm_mul_ps(_mm_set1_ps(2.0f), v_p2)),
		    _mm_add_ps(_mm_mul_ps(_mm_set1_ps(2.5f), v_p1),
		               _mm_mul_ps(_mm_set1_ps(0.5f), v_p3)));
		__m128 v_c3 = _mm_add_ps(
		    _mm_mul_ps(_mm_set1_ps(0.5f), _mm_sub_ps(v_p3, v_p0)),
		    _mm_mul_ps(_mm_set1_ps(1.5f), _mm_sub_ps(v_p1, v_p2)));

		/* Horner form: out = ((c3*f + c2)*f + c1)*f + p1 */
		__m128 v_out = _mm_add_ps(
		    _mm_mul_ps(
		        _mm_add_ps(
		            _mm_mul_ps(
		                _mm_add_ps(_mm_mul_ps(v_c3, v_frac), v_c2),
		                v_frac),
		            v_c1),
		        v_frac),
		    v_p1);
		v_out = _mm_mul_ps(v_out, v_gain);

		float res[4];
		_mm_storeu_ps(res, v_out);
		out_l[i] = res[0];
		out_r[i] = res[1];

#else
		/* Standard C fallback */
		float c1l = 0.5f * (p2l - p0l);
		float c2l = p0l - 2.5f * p1l + 2.0f * p2l - 0.5f * p3l;
		float c3l = 0.5f * (p3l - p0l) + 1.5f * (p1l - p2l);
		out_l[i] =
		    (((c3l * frac + c2l) * frac + c1l) * frac + p1l) * vgain;

		float c1r = 0.5f * (p2r - p0r);
		float c2r = p0r - 2.5f * p1r + 2.0f * p2r - 0.5f * p3r;
		float c3r = 0.5f * (p3r - p0r) + 1.5f * (p1r - p2r);
		out_r[i] =
		    (((c3r * frac + c2r) * frac + c1r) * frac + p1r) * vgain;
#endif

		double step = (double)(t->pitch + current_nudge + current_vel);
		fpos += step;
		current_vel += vel_inc;
		current_nudge += nudge_inc;
	}

	if (fpos < 0.0) {
		fpos = 0.0;
	}
	t->pos = (uint32_t)fpos;
}

void mix_sampler(SamplerSlot *slot, float *mix_l, float *mix_r, uint32_t frames)
{
	pthread_mutex_lock(&slot->lock);
	if (!slot->playing || !slot->data) {
		pthread_mutex_unlock(&slot->lock);
		return;
	}

	uint32_t i = 0;
	float vol = slot->volume;

#if defined(__ALTIVEC__)
	vfloat v_vol = (vfloat){vol, vol, vol, vol};
	vfloat v_inv_32768 = (vfloat){1.0f / 32768.0f, 1.0f / 32768.0f,
	                              1.0f / 32768.0f, 1.0f / 32768.0f};
	vfloat v_zero = (vfloat){0.0f, 0.0f, 0.0f, 0.0f};

	for (; i + 1 < frames; i += 2) {
		if (slot->pos + 1 >= slot->num_frames) {
			if (slot->looping) {
				slot->pos = 0;
			} else {
				slot->playing = 0;
				break;
			}
		}
		vfloat v_in =
		    (vfloat){(float)slot->data[slot->pos * 2],
		             (float)slot->data[slot->pos * 2 + 1],
		             (float)slot->data[(slot->pos + 1) * 2],
		             (float)slot->data[(slot->pos + 1) * 2 + 1]};
		vfloat v_gain = vec_madd(vec_madd(v_in, v_inv_32768, v_zero),
		                         v_vol, v_zero);

		mix_l[i] += ((float *)&v_gain)[0];
		mix_r[i] += ((float *)&v_gain)[1];
		mix_l[i + 1] += ((float *)&v_gain)[2];
		mix_r[i + 1] += ((float *)&v_gain)[3];
		slot->pos += 2;
	}
#endif

	for (; i < frames; i++) {
		if (slot->pos >= slot->num_frames) {
			if (slot->looping) {
				slot->pos = 0;
			} else {
				slot->playing = 0;
				break;
			}
		}
		mix_l[i] += (slot->data[slot->pos * 2] / 32768.0f) * vol;
		mix_r[i] += (slot->data[slot->pos * 2 + 1] / 32768.0f) * vol;
		slot->pos++;
	}
	pthread_mutex_unlock(&slot->lock);
}

/* ──────────────────────────────────────────────
   Biquad EQ Coefficients
   Pre-computed Butterworth 2nd-order filters.
   g_actual_sample_rate from djcmd_ui.h (extern).
   ────────────────────────────────────────────── */
static float g_lp_a[3], g_lp_b[3]; /* 300 Hz low-pass  */
static float g_bp_a[3], g_bp_b[3]; /* 1k-5k band-pass  */
static float g_hp_a[3], g_hp_b[3]; /* 5k Hz high-pass  */

void biquad_lowpass(float fc, float *b, float *a)
{
	float w0 = 2.0f * (float)M_PI * fc / (float)g_actual_sample_rate;
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

void biquad_highpass(float fc, float *b, float *a)
{
	float w0 = 2.0f * (float)M_PI * fc / (float)g_actual_sample_rate;
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
	float w0 = 2.0f * (float)M_PI * fc / (float)g_actual_sample_rate;
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

void init_eq_coeffs(void)
{
	biquad_lowpass(300.0f, g_lp_b, g_lp_a);
	biquad_bandpass(2500.0f, g_bp_b, g_bp_a);
	biquad_highpass(5000.0f, g_hp_b, g_hp_a);
}

/* ── Exposed coefficient arrays for mix_and_write (still in djcmd.c) ── */
float *audio_lp_b(void)
{
	return g_lp_b;
}
float *audio_lp_a(void)
{
	return g_lp_a;
}
float *audio_bp_b(void)
{
	return g_bp_b;
}
float *audio_bp_a(void)
{
	return g_bp_a;
}
float *audio_hp_b(void)
{
	return g_hp_b;
}
float *audio_hp_a(void)
{
	return g_hp_a;
}
float audio_apply_biquad(float x, float *b, float *a, float *x1, float *x2,
                         float *y1, float *y2)
{
	return apply_biquad(x, b, a, x1, x2, y1, y2);
}

/* ──────────────────────────────────────────────
   Auto-gain calculation
   ────────────────────────────────────────────── */
float calc_auto_gain(const int16_t *data, uint32_t frames)
{
	float peak = 0.0f;
	for (uint32_t i = 0; i < frames; i += 8) {
		float l = fabsf(data[i * 2] / 32768.0f);
		float r = fabsf(data[i * 2 + 1] / 32768.0f);
		if (l > peak) {
			peak = l;
		}
		if (r > peak) {
			peak = r;
		}
	}
	float target = powf(10.0f, g_opts.auto_gain_target_db / 20.0f);
	return (peak > 0.01f) ? (target / peak) : 1.0f;
}

/* ──────────────────────────────────────────────
   Sampler loader
   ────────────────────────────────────────────── */
int load_sampler(SamplerSlot *s, const char *path)
{
	AFBuffer buf;
	int rc = af_load(path, &buf);
	if (rc != AF_OK) {
		return -1;
	}

	pthread_mutex_lock(&s->lock);
	if (s->data) {
		free(s->data);
	}
	s->data = buf.samples;
	s->num_frames = buf.num_frames;
	s->pos = 0;
	s->playing = 0;
	s->looping = 0;
	const char *slash = strrchr(path, '/');
	strncpy(s->name, slash ? slash + 1 : path, 127);
	strncpy(s->filename, path, MAX_FILENAME - 1);
	pthread_mutex_unlock(&s->lock);
	return 0;
}

/* ──────────────────────────────────────────────
   Track loader
   ────────────────────────────────────────────── */
int load_track(Track *t, const char *path)
{
	AFBuffer buf;
	int rc = af_load(path, &buf);
	if (rc != AF_OK) {
		return -1;
	}

	float gain = g_opts.auto_gain_default
	                 ? calc_auto_gain(buf.samples, buf.num_frames)
	                 : 1.0f;

	int deck_idx = (int)(t - g_tracks);

	int16_t *old_data = NULL;
	uint8_t *old_wfm_lo = NULL;
	uint8_t *old_wfm_mid = NULL;
	uint8_t *old_wfm_hi = NULL;

	pthread_mutex_lock(&t->lock);
	old_data = t->data;
	old_wfm_lo = t->wfm_low;
	old_wfm_mid = t->wfm_mid;
	old_wfm_hi = t->wfm_high;

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
	t->synced = 0;
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

	free(old_data);
	free(old_wfm_lo);
	free(old_wfm_mid);
	free(old_wfm_hi);
	return 0;
}

/* ──────────────────────────────────────────────
   Autocorrelation BPM estimator
   ────────────────────────────────────────────── */
float estimate_bpm_autocorr(const int16_t *data, uint32_t num_frames)
{
	if (num_frames < g_actual_sample_rate * 4u) {
		return 0.0f;
	}

	uint32_t analyse_frames = num_frames;
	if (analyse_frames > g_actual_sample_rate * 90u) {
		analyse_frames = g_actual_sample_rate * 90u;
	}

	uint32_t hop_frames = (uint32_t)(g_actual_sample_rate * 10 / 1000);
	if (hop_frames < 1) {
		hop_frames = 1;
	}
	uint32_t env_len = analyse_frames / hop_frames;
	if (env_len < 16) {
		return 0.0f;
	}

	float *env = (float *)malloc(env_len * sizeof(float));
	if (!env) {
		return 0.0f;
	}

	for (uint32_t h = 0; h < env_len; h++) {
		uint32_t start = h * hop_frames;
		uint32_t end = start + hop_frames;
		if (end > analyse_frames) {
			end = analyse_frames;
		}
		float sum = 0.0f;

#if defined(__SSE2__) || defined(__ALTIVEC__)
		uint32_t f = start;
		uint32_t n4 = ((end - start) / 4) * 4;
		uint32_t end4 = start + n4;
#if defined(__SSE2__)
		__m128 vsum = _mm_setzero_ps();
		float scale = 1.0f / 65536.0f;
		for (; f < end4; f += 4) {
			float s0 = (data[f * 2] + data[f * 2 + 1]) * scale;
			float s1 =
			    (data[(f + 1) * 2] + data[(f + 1) * 2 + 1]) * scale;
			float s2 =
			    (data[(f + 2) * 2] + data[(f + 2) * 2 + 1]) * scale;
			float s3 =
			    (data[(f + 3) * 2] + data[(f + 3) * 2 + 1]) * scale;
			__m128 vs = _mm_set_ps(s3, s2, s1, s0);
			vsum = _mm_add_ps(vsum, _mm_mul_ps(vs, vs));
		}
		float fres[4];
		_mm_storeu_ps(fres, vsum);
		sum = fres[0] + fres[1] + fres[2] + fres[3];
#elif defined(__ALTIVEC__)
		vfloat vsum = {0, 0, 0, 0};
		float scale = 1.0f / 65536.0f;
		for (; f < end4; f += 4) {
			float s0 = (data[f * 2] + data[f * 2 + 1]) * scale;
			float s1 =
			    (data[(f + 1) * 2] + data[(f + 1) * 2 + 1]) * scale;
			float s2 =
			    (data[(f + 2) * 2] + data[(f + 2) * 2 + 1]) * scale;
			float s3 =
			    (data[(f + 3) * 2] + data[(f + 3) * 2 + 1]) * scale;
			vfloat vs = {s0, s1, s2, s3};
			vsum = vec_madd(vs, vs, vsum);
		}
		sum = ((float *)&vsum)[0] + ((float *)&vsum)[1] +
		      ((float *)&vsum)[2] + ((float *)&vsum)[3];
#endif
		for (; f < end; f++) {
			float s =
			    (data[f * 2] + data[f * 2 + 1]) * (1.0f / 65536.0f);
			sum += s * s;
		}
#else
		for (uint32_t f2 = start; f2 < end; f2++) {
			float s = (data[f2 * 2] + data[f2 * 2 + 1]) *
			          (1.0f / 65536.0f);
			sum += s * s;
		}
#endif
		env[h] = sum / (float)(end - start);
	}

	float prev = env[0];
	for (uint32_t h = 1; h < env_len; h++) {
		float diff = env[h] - prev;
		prev = env[h];
		env[h] = (diff > 0.0f) ? diff : 0.0f;
	}
	env[0] = 0.0f;

	float hops_per_sec = (float)g_actual_sample_rate / (float)hop_frames;
	if (env_len > (uint32_t)(hops_per_sec * 30.0f)) {
		uint32_t skip = (uint32_t)(hops_per_sec * 10.0f);
		if (skip > env_len) {
			skip = env_len;
		}
		memset(env, 0, skip * sizeof(float));
	}

	float ac_lo = (g_bpm_detect_lo > 0.0f) ? g_bpm_detect_lo : BPM_AC_LO;
	float ac_hi = (g_bpm_detect_hi > 0.0f) ? g_bpm_detect_hi : BPM_AC_HI;
	uint32_t lag_min = (uint32_t)(hops_per_sec * 60.0f / ac_hi);
	uint32_t lag_max = (uint32_t)(hops_per_sec * 60.0f / ac_lo);
	if (lag_min < 1) {
		lag_min = 1;
	}
	if (lag_max >= env_len) {
		lag_max = env_len - 1;
	}
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

	uint32_t ac_use = env_len - lag_max;
	for (uint32_t li = 0; li < ac_len; li++) {
		uint32_t lag = lag_min + li;
		float sum = 0.0f;
#if defined(__SSE2__) || defined(__ALTIVEC__)
		uint32_t h = 0;
		uint32_t n_simd = ((ac_use - lag) / 8) * 8;
#if defined(__SSE2__)
		__m128 vsum2 = _mm_setzero_ps();
		for (; h < n_simd; h += 8) {
			__m128 vh = _mm_set_ps(env[h + 6], env[h + 4],
			                       env[h + 2], env[h]);
			__m128 vl =
			    _mm_set_ps(env[h + lag + 6], env[h + lag + 4],
			               env[h + lag + 2], env[h + lag]);
			vsum2 = _mm_add_ps(vsum2, _mm_mul_ps(vh, vl));
		}
		float r[4];
		_mm_storeu_ps(r, vsum2);
		sum = r[0] + r[1] + r[2] + r[3];
#elif defined(__ALTIVEC__)
		vfloat vsum2 = {0, 0, 0, 0};
		for (; h < n_simd; h += 8) {
			vfloat vh2 = {env[h], env[h + 2], env[h + 4],
			              env[h + 6]};
			vfloat vl2 = {env[h + lag], env[h + lag + 2],
			              env[h + lag + 4], env[h + lag + 6]};
			vsum2 = vec_madd(vh2, vl2, vsum2);
		}
		sum = ((float *)&vsum2)[0] + ((float *)&vsum2)[1] +
		      ((float *)&vsum2)[2] + ((float *)&vsum2)[3];
#endif
		for (; h + lag < ac_use; h += 2) {
			sum += env[h] * env[h + lag];
		}
#else
		for (uint32_t h2 = 0; h2 + lag < ac_use; h2 += 2) {
			sum += env[h2] * env[h2 + lag];
		}
#endif
		ac[li] = sum;
	}
	free(env);

	uint32_t peak_li = 0;
	float peak_v = ac[0];
	for (uint32_t li = 1; li < ac_len; li++) {
		if (ac[li] > peak_v) {
			peak_v = ac[li];
			peak_li = li;
		}
	}

	float frac_li = (float)peak_li;
	if (peak_li > 0 && peak_li < ac_len - 1) {
		float y0 = ac[peak_li - 1], y1 = ac[peak_li],
		      y2 = ac[peak_li + 1];
		float denom = 2.0f * (y0 - 2.0f * y1 + y2);
		if (fabsf(denom) > 1e-10f) {
			frac_li += (y0 - y2) / denom;
		}
	}
	free(ac);

	float best_lag_hops = (float)lag_min + frac_li;
	if (best_lag_hops < 1.0f) {
		return 0.0f;
	}
	float bpm = hops_per_sec * 60.0f / best_lag_hops;

	float fold_lo = (g_bpm_detect_lo > 0.0f) ? g_bpm_detect_lo : 60.0f;
	float fold_hi = (g_bpm_detect_hi > 0.0f) ? g_bpm_detect_hi : 180.0f;
	while (bpm < fold_lo && bpm > 0.0f) {
		bpm *= 2.0f;
	}
	while (bpm > fold_hi) {
		bpm *= 0.5f;
	}
	return (bpm >= fold_lo && bpm <= fold_hi) ? bpm : 0.0f;
}

/* ──────────────────────────────────────────────
   Waveform + BPM Grid builder
   ────────────────────────────────────────────── */
void rebuild_waveform_and_grid(Track *t, int force_analyze)
{
	pthread_mutex_lock(&t->lock);
	int16_t *data = t->data;
	uint32_t nframes = t->num_frames;
	int loaded = t->loaded;
	pthread_mutex_unlock(&t->lock);

	if (!loaded || !data || nframes < 4096) {
		if (loaded) {
			t->bpm = 120.0f;
			t->bpm_offset = 0.0f;
		}
		return;
	}

	if (force_analyze) {
		float ac_bpm = estimate_bpm_autocorr(data, nframes);
		t->bpm = (ac_bpm > 0.0f) ? ac_bpm : 120.0f;

		{
			float prev_energy = 0.0f, threshold = 0.0f;
			float flux_sum = 0.0f;
			int flux_n = 0;
			for (uint32_t p = 0; p + ONSET_HOP <= nframes;
			     p += ONSET_HOP) {
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
			threshold = (flux_n > 0)
			                ? (flux_sum / (float)flux_n) * 3.0f
			                : 0.01f;

			prev_energy = 0.0f;
			uint32_t first_onset = 0;
			for (uint32_t p = 0; p + ONSET_HOP <= nframes;
			     p += ONSET_HOP) {
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
				}
				prev_energy = energy;
			}
			t->bpm_offset = (float)first_onset;
		}
	} else {
		if (t->bpm <= 1.0f) {
			t->bpm = 120.0f;
			t->bpm_offset = 0.0f;
		}
	}

	{
		uint32_t bins = WFM_OVERVIEW_BINS;
		uint32_t chunk = (nframes + bins - 1) / bins;
		uint8_t *lov = (uint8_t *)malloc(bins);
		uint8_t *mov = (uint8_t *)malloc(bins);
		uint8_t *hov = (uint8_t *)malloc(bins);

		if (lov && mov && hov) {
			const float a_low = 0.0426f;
			const float a_hi4k = 0.3996f;
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
				if (start >= nframes) {
					flo[b] = fmi[b] = fhi[b] = 0.0f;
					continue;
				}
				uint32_t end = start + chunk;
				if (end > nframes) {
					end = nframes;
				}
				uint32_t n = end - start;
				if (n == 0) {
					flo[b] = fmi[b] = fhi[b] = 0.0f;
					continue;
				}

				float lp_low = 0.0f, lp_4k = 0.0f;
				uint32_t warmup = (n < 8) ? n : 8;
				for (uint32_t f = start; f < start + warmup;
				     f++) {
					float s =
					    (data[f * 2] + data[f * 2 + 1]) *
					    (0.5f / 32768.0f);
					lp_low += a_low * (s - lp_low);
					lp_4k += a_hi4k * (s - lp_4k);
				}
				float sum_lo = 0.0f, sum_mi = 0.0f,
				      sum_hi = 0.0f;
				for (uint32_t f = start; f < end; f++) {
					float s =
					    (data[f * 2] + data[f * 2 + 1]) *
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
				flo[b] = sqrtf(sum_lo / (float)n);
				fmi[b] = sqrtf(sum_mi / (float)n);
				fhi[b] = sqrtf(sum_hi / (float)n);
			}

			float global_max = 1e-9f;
			for (uint32_t b = 0; b < bins; b++) {
				flo[b] *= 12.0f;
				fmi[b] *= 10.0f;
				fhi[b] *= 2.0f;
				float mx = flo[b] > fmi[b] ? flo[b] : fmi[b];
				mx = mx > fhi[b] ? mx : fhi[b];
				if (mx > global_max) {
					global_max = mx;
				}
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
}

/* ──────────────────────────────────────────────
   Sync follower deck to leader's BPM and beat phase
   ────────────────────────────────────────────── */
void sync_to_leader(int follower_idx)
{
	if (g_sync_leader < 0 || g_sync_leader >= g_num_tracks) {
		return;
	}
	Track *master = &g_tracks[g_sync_leader];
	Track *slave = &g_tracks[follower_idx];
	if (!master->loaded || master->bpm < 1.0f) {
		return;
	}
	if (!slave->loaded || slave->bpm < 1.0f) {
		return;
	}

	/* ── 1. Pitch: slave plays at master's effective BPM ── */
	float leader_eff_bpm = master->bpm * master->pitch;

	float slave_native_bpm = slave->bpm;
	if (g_opts.sync_smart_range && slave_native_bpm > 1.0f &&
	    leader_eff_bpm > 1.0f) {
		for (int i = 0; i < 3; i++) {
			if (slave_native_bpm < leader_eff_bpm * 0.75f) {
				slave_native_bpm *= 2.0f;
			} else {
				break;
			}
		}
		for (int i = 0; i < 3; i++) {
			if (slave_native_bpm > leader_eff_bpm * 1.334f) {
				slave_native_bpm *= 0.5f;
			} else {
				break;
			}
		}
	}

	slave->pitch = leader_eff_bpm / slave_native_bpm;
	if (slave->pitch < 0.25f) {
		slave->pitch = 0.25f;
	}
	if (slave->pitch > 4.0f) {
		slave->pitch = 4.0f;
	}

	/* ── 2. Phase: align slave beat grid to master beat phase ── */
	float master_beat_frames =
	    (float)g_actual_sample_rate * 60.0f / leader_eff_bpm;
	float slave_eff_bpm = slave_native_bpm * slave->pitch;
	float slave_beat_frames =
	    (float)g_actual_sample_rate * 60.0f / slave_eff_bpm;

	float leader_phase = 0.0f;
	if (master_beat_frames > 0.0f) {
		float beats_elapsed =
		    ((float)master->pos - master->bpm_offset) /
		    master_beat_frames;
		leader_phase = beats_elapsed - floorf(beats_elapsed);
	}

	float slave_beats_elapsed = 0.0f;
	if (slave_beat_frames > 0.0f) {
		slave_beats_elapsed =
		    ((float)slave->pos - slave->bpm_offset) / slave_beat_frames;
	}
	float slave_beat_n = floorf(slave_beats_elapsed + 0.5f);

	float new_pos = slave->bpm_offset +
	                (slave_beat_n + leader_phase) * slave_beat_frames;

	if (new_pos < 0.0f) {
		new_pos = 0.0f;
	}
	if ((uint32_t)new_pos >= slave->num_frames) {
		new_pos = (float)(slave->num_frames - 1);
	}

	slave->pos = (uint32_t)new_pos;
}

/* ──────────────────────────────────────────────
   Functions moved from djcmd.c (Phase D)
   ────────────────────────────────────────────── */

/* --- cf_gain --- */
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

	if (cf >= lo && cf <= hi) {
		return 1.0f;
	}

	if (track_idx % 2 == 0) {
		/* Left deck: fades out on the right side (cf > hi) */
		if (cf <= hi) {
			return 1.0f;
		}
		float t = (cf - hi) / (1.0f - hi);
		/* Power the t value by the curve setting */
		float p =
		    0.5f + curve * 4.0f; /* p=0.5 (smooth) to p=4.5 (sharp) */
		float c = cosf(t * (float)(M_PI / 2.0));
		if (c < 0.0f) {
			c = 0.0f;
		}
		return powf(c, p);
	} else {
		/* Right deck: fades out on the left side (cf < lo) */
		if (cf >= lo) {
			return 1.0f;
		}
		float t = 1.0f - (cf / lo);
		float p = 0.5f + curve * 4.0f;
		float c = cosf(t * (float)(M_PI / 2.0));
		if (c < 0.0f) {
			c = 0.0f;
		}
		return powf(c, p);
	}
}

/* --- pcm_handles --- */
static snd_pcm_t *g_pcm = NULL;
static snd_pcm_t *g_pcm_hp = NULL;

/* --- pcm_buffers_vu --- */
static float g_tmp_l[PERIOD_FRAMES] __attribute__((aligned(16)));
static float g_tmp_r[PERIOD_FRAMES] __attribute__((aligned(16)));
static float g_mix_l[PERIOD_FRAMES] __attribute__((aligned(16)));
static float g_mix_r[PERIOD_FRAMES] __attribute__((aligned(16)));
static float g_mix_hp_l[PERIOD_FRAMES] __attribute__((aligned(16)));
static float g_mix_hp_r[PERIOD_FRAMES] __attribute__((aligned(16)));
static int16_t g_pcm_buf[PERIOD_FRAMES * 2];
static int16_t g_pcm_hp_buf[PERIOD_FRAMES * 2];

/* Peak meter state -- written by audio thread, read by UI thread.
 * No mutex: single float writes are atomic enough for display. */
_Atomic float g_vu_l = 0.0f;
_Atomic float g_vu_r = 0.0f;
_Atomic float g_vu_peak_l = 0.0f;
_Atomic float g_vu_peak_r = 0.0f;
#define VU_PEAK_HOLD_PERIODS (g_opts.ui_fps * 3) /* 3 s hold at any FPS */

#define VU_DECAY 0.85f /* per-period level decay */

/* --- mix_and_write --- */
void mix_and_write(void)
{
	memset(g_mix_l, 0, sizeof(float) * PERIOD_FRAMES);
	memset(g_mix_r, 0, sizeof(float) * PERIOD_FRAMES);
	memset(g_mix_hp_l, 0, sizeof(float) * PERIOD_FRAMES);
	memset(g_mix_hp_r, 0, sizeof(float) * PERIOD_FRAMES);

	float mvol = g_master_vol / 100.0f;
	float hp_mvol = g_hp_vol / 100.0f;

	/* ── Quantize play logic ── */
	if (g_opts.sync_quantize && g_sync_leader >= 0) {
		Track *master = &g_tracks[g_sync_leader];
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
				if (ti == g_sync_leader) {
					continue;
				}
				Track *sl = &g_tracks[ti];
				if (!sl->pending_play || !sl->loaded) {
					continue;
				}
				if (crossed_bar) {
					sl->pending_play = 0;
					sl->playing = 1;
					sync_to_leader(ti);
				}
			}
		}
	}

	for (int t = 0; t < MAX_TRACKS; t++) {
		Track *tr = &g_tracks[t];
		pthread_mutex_lock(&tr->lock);
		if (!tr->loaded) {
			pthread_mutex_unlock(&tr->lock);
			continue;
		}

		/* Allow paused decks through when the user is actively
		 * scratching */
		int scratching = !tr->playing && g_jog_touched[t] &&
		                 (fabsf((float)g_motor_vel[t]) > 0.0001f);
		if (!tr->playing && !scratching) {
			pthread_mutex_unlock(&tr->lock);
			continue;
		}

		/* ── Scratch Engine Improvements ── */
		float start_vel = g_last_motor_vel[t];
		float target_vel = g_motor_vel[t];
		float start_nudge = g_last_applied_nudge[t];
		float target_nudge = g_jog_nudge[t];

		/* Revert to WSOLA for key-lock (CPU efficient on G4)
		 * AND bypass key-lock during active scratch touch. */
		if (tr->key_lock && !g_jog_touched[t]) {
			WSOLAState *ws = &g_wsola[t];
			double rate = (double)(tr->pitch + tr->nudge +
			                       target_vel + target_nudge);

			/* Advance slip_pos (real background playhead) */
			if (tr->playing) {
				tr->slip_pos += rate * PERIOD_FRAMES;
				if (tr->slip_pos >= (double)tr->num_frames) {
					if (tr->looping) {
						tr->slip_pos = fmod(
						    tr->slip_pos,
						    (double)tr->num_frames);
					} else {
						tr->slip_pos =
						    (double)tr->num_frames;
					}
				}
			}

			/* Reset if playhead jumped */
			if (fabsf((float)(ws->src_pos - (double)tr->pos)) >
			    WSOLA_WIN * 4) {
				wsola_reset(ws, tr->pos);
			}

			wsola_process(tr, ws, g_tmp_l, g_tmp_r, PERIOD_FRAMES,
			              rate, tr->volume * tr->gain,
			              g_opts.eco_mode);
		} else {
			/* Hermite path with per-sample velocity ramping */
			read_pitched(tr, g_tmp_l, g_tmp_r, PERIOD_FRAMES,
			             start_vel, target_vel, start_nudge,
			             target_nudge);

			/* Advance slip_pos */
			double rate = (double)(tr->pitch + tr->nudge +
			                       target_vel + target_nudge);
			if (tr->playing || scratching) {
				tr->slip_pos += rate * PERIOD_FRAMES;
				if (tr->slip_pos >= (double)tr->num_frames) {
					if (tr->looping) {
						tr->slip_pos = fmod(
						    tr->slip_pos,
						    (double)tr->num_frames);
					} else {
						tr->slip_pos =
						    (double)tr->num_frames;
					}
				}
			}
		}
		pthread_mutex_unlock(&tr->lock);

		/* Update historical values for next block ramping */
		g_last_motor_vel[t] = target_vel;
		g_last_applied_nudge[t] = target_nudge;

		/* ── Vinyl scratch character filter
		 * ──────────────────────────── */
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

			/* Inertia Smoothing: prevents filter "pops" from MIDI
			 * jitter */
			g_scratch_alpha[t] =
			    g_scratch_alpha[t] * 0.85f + target_alpha * 0.15f;
			float alpha = g_scratch_alpha[t];

			float scratch_gain = 1.0f;
			if (g_jog_touched[t] && abs_rate < 0.15f) {
				float n = abs_rate / 0.15f;
				scratch_gain = 0.05f + 0.95f * n * n;
			}

			if (g_jog_touched[t]) {
				float nl = 0.0018f * scratch_gain;
				for (uint32_t si = 0; si < PERIOD_FRAMES;
				     si++) {
					g_noise_state[t] =
					    g_noise_state[t] * 1664525u +
					    1013904223u;
					float wl =
					    (float)(int32_t)g_noise_state[t] *
					    (1.0f / 2147483648.0f);
					g_noise_state[t] =
					    g_noise_state[t] * 1664525u +
					    1013904223u;
					float wr =
					    (float)(int32_t)g_noise_state[t] *
					    (1.0f / 2147483648.0f);
					g_noise_brown_l[t] =
					    g_noise_brown_l[t] * 0.97f +
					    wl * 0.03f;
					g_noise_brown_r[t] =
					    g_noise_brown_r[t] * 0.97f +
					    wr * 0.03f;
					g_tmp_l[si] +=
					    nl *
					    (wl * 0.05f +
					     5.0f * g_noise_brown_l[t] * 0.95f);
					g_tmp_r[si] +=
					    nl *
					    (wr * 0.05f +
					     5.0f * g_noise_brown_r[t] * 0.95f);
				}
			}

			if (alpha < 0.94f) {
				float g_svf = tanf(1.5707963f * alpha);
				float k_svf = 0.333f;
				float a1 =
				    1.0f / (1.0f + g_svf * (g_svf + k_svf));
				float a2 = g_svf * a1;
				float a3 = g_svf * a2;

				float g_hc = tanf(1.5707963f *
				                  0.18f); /* ~8kHz @ 44.1k */
				float a1_hc = 1.0f / (1.0f + g_hc);

				/* Anti-denormal DC offset (1e-18) prevents G4
				 * CPU spikes during silence */
				const float dc = 1e-18f;

				for (uint32_t si = 0; si < PERIOD_FRAMES;
				     si++) {
					float xl = g_tmp_l[si] * scratch_gain;
					float xr = g_tmp_r[si] * scratch_gain;

					{
						float ic1 =
						    g_scratch_lpf_l[t] + dc;
						float ic2 =
						    g_scratch_lpf2_l[t] + dc;
						float v3 = xl - ic2;
						float v1 = a1 * ic1 + a2 * v3;
						float v2 =
						    ic2 + a2 * ic1 + a3 * v3;
						g_scratch_lpf_l[t] =
						    2.0f * v1 - ic1;
						g_scratch_lpf2_l[t] =
						    2.0f * v2 - ic2;
						float o = v2 + 0.35f * v1;
						float v3_hc =
						    o - g_noise_brown_l[t];
						float v1_hc = a1_hc * v3_hc;
						float v2_hc =
						    g_noise_brown_l[t] + v1_hc;
						g_noise_brown_l[t] =
						    v2_hc + v1_hc;
						o = v2_hc;
						float abs_o = fabsf(o);
						if (abs_o > 1.0f) {
							o = (o > 0) ? 1.0f
							            : -1.0f;
						} else {
							o = o * (1.5f -
							         0.5f * o * o);
						}
						g_tmp_l[si] = o;
					}
					{
						float ic1 =
						    g_scratch_lpf_r[t] + dc;
						float ic2 =
						    g_scratch_lpf2_r[t] + dc;
						float v3 = xr - ic2;
						float v1 = a1 * ic1 + a2 * v3;
						float v2 =
						    ic2 + a2 * ic1 + a3 * v3;
						g_scratch_lpf_r[t] =
						    2.0f * v1 - ic1;
						g_scratch_lpf2_r[t] =
						    2.0f * v2 - ic2;
						float o = v2 + 0.35f * v1;
						float v3_hc =
						    o - g_noise_brown_r[t];
						float v1_hc = a1_hc * v3_hc;
						float v2_hc =
						    g_noise_brown_r[t] + v1_hc;
						g_noise_brown_r[t] =
						    v2_hc + v1_hc;
						o = v2_hc;
						float abs_o = fabsf(o);
						if (abs_o > 1.0f) {
							o = (o > 0) ? 1.0f
							            : -1.0f;
						} else {
							o = o * (1.5f -
							         0.5f * o * o);
						}
						g_tmp_r[si] = o;
					}
				}
			} else {
				if (scratch_gain < 1.0f) {
					for (uint32_t si = 0;
					     si < PERIOD_FRAMES; si++) {
						g_tmp_l[si] *= scratch_gain;
						g_tmp_r[si] *= scratch_gain;
					}
				}
				g_scratch_lpf_l[t] = 0.0f;
				g_scratch_lpf2_l[t] =
				    g_tmp_l[PERIOD_FRAMES - 1];
				g_scratch_lpf_r[t] = 0.0f;
				g_scratch_lpf2_r[t] =
				    g_tmp_r[PERIOD_FRAMES - 1];
			}
		}

		/* Decay nudge (applied to global state for next MIDI message)
		 */
		g_jog_nudge[t] = (float)g_jog_nudge[t] * NUDGE_DECAY;
		if (fabsf((float)g_jog_nudge[t]) < 0.0001f) {
			g_jog_nudge[t] = 0.0f;
			g_last_applied_nudge[t] = 0.0f;
		}

		/* ── NS7III motor: decay g_jog_abs_vel when silent ── */
		if (g_jog_type == JOG_NS7III && g_motor_running[t]) {
			struct timespec _now;
			clock_gettime(CLOCK_MONOTONIC, &_now);
			int64_t now_ms = (int64_t)_now.tv_sec * 1000 +
			                 _now.tv_nsec / 1000000;
			int64_t silent_ms = now_ms - g_jog_last_msg_ms[t];

			if (silent_ms > 40) {
				g_jog_abs_vel[t] *= 0.75f;
				if (fabsf((float)g_jog_abs_vel[t]) < 0.00001f) {
					g_jog_abs_vel[t] = 0.0f;
				}

				if (g_motor_settle_until[t] == 0) {
					if (g_jog_touched[t]) {
						float raw_vel =
						    (g_jog_abs_vel[t] /
						     g_jog_ref_delta) -
						    1.0f;
						if (raw_vel >
						    g_jog_motor_dead) {
							raw_vel -=
							    g_jog_motor_dead;
						} else if (raw_vel <
						           -g_jog_motor_dead) {
							raw_vel +=
							    g_jog_motor_dead;
						} else {
							raw_vel = 0.0f;
						}

						if (raw_vel > g_jog_vel_max) {
							raw_vel = g_jog_vel_max;
						}
						if (raw_vel < -g_jog_vel_max) {
							raw_vel =
							    -g_jog_vel_max;
						}

						float diff =
						    raw_vel - g_motor_vel[t];
						float alpha =
						    (fabsf(diff) > 0.5f)
						        ? 0.55f
						        : 0.20f;
						g_motor_vel[t] =
						    g_motor_vel[t] *
						        (1.0f - alpha) +
						    raw_vel * alpha;
					} else if (g_opts.vinyl_mode) {
						g_motor_vel[t] = 0.0f;
					} else {
						float raw_vel2 =
						    (g_jog_abs_vel[t] /
						     g_jog_ref_delta) -
						    1.0f;
						if (raw_vel2 >
						    g_jog_motor_dead) {
							raw_vel2 -=
							    g_jog_motor_dead;
						} else if (raw_vel2 <
						           -g_jog_motor_dead) {
							raw_vel2 +=
							    g_jog_motor_dead;
						} else {
							raw_vel2 = 0.0f;
						}

						if (raw_vel2 > g_jog_vel_max) {
							raw_vel2 =
							    g_jog_vel_max;
						}
						if (raw_vel2 < -g_jog_vel_max) {
							raw_vel2 =
							    -g_jog_vel_max;
						}

						float diff2 =
						    raw_vel2 - g_motor_vel[t];
						float alpha2 =
						    (fabsf(diff2) > 0.5f)
						        ? 0.55f
						        : 0.20f;
						g_motor_vel[t] =
						    g_motor_vel[t] *
						        (1.0f - alpha2) +
						    raw_vel2 * alpha2;
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
				float fc =
				    2000.0f * powf(18000.0f / 2000.0f, t_hp);
				biquad_highpass(fc, eq->fi_b, eq->fi_a);
			}
		}

		/* ── EQ pass: write back to g_tmp_l/r (in-place) ── */
		{
			float *lp_b = audio_lp_b(), *lp_a = audio_lp_a();
			float *bp_b = audio_bp_b(), *bp_a = audio_bp_a();
			float *hp_b = audio_hp_b(), *hp_a = audio_hp_a();
			for (int i = 0; i < PERIOD_FRAMES; i++) {
				float l = g_tmp_l[i], r = g_tmp_r[i];
				if (filter_active) {
					l = audio_apply_biquad(
					    l, eq->fi_b, eq->fi_a, &eq->fi_x1l,
					    &eq->fi_x2l, &eq->fi_y1l,
					    &eq->fi_y2l);
					r = audio_apply_biquad(
					    r, eq->fi_b, eq->fi_a, &eq->fi_x1r,
					    &eq->fi_x2r, &eq->fi_y1r,
					    &eq->fi_y2r);
				}
				float lo_l = audio_apply_biquad(
				    l, lp_b, lp_a, &eq->lp_x1l, &eq->lp_x2l,
				    &eq->lp_y1l, &eq->lp_y2l);
				float lo_r = audio_apply_biquad(
				    r, lp_b, lp_a, &eq->lp_x1r, &eq->lp_x2r,
				    &eq->lp_y1r, &eq->lp_y2r);
				float mi_l = audio_apply_biquad(
				    l, bp_b, bp_a, &eq->bp_x1l, &eq->bp_x2l,
				    &eq->bp_y1l, &eq->bp_y2l);
				float mi_r = audio_apply_biquad(
				    r, bp_b, bp_a, &eq->bp_x1r, &eq->bp_x2r,
				    &eq->bp_y1r, &eq->bp_y2r);
				float hi_l = audio_apply_biquad(
				    l, hp_b, hp_a, &eq->hp_x1l, &eq->hp_x2l,
				    &eq->hp_y1l, &eq->hp_y2l);
				float hi_r = audio_apply_biquad(
				    r, hp_b, hp_a, &eq->hp_x1r, &eq->hp_x2r,
				    &eq->hp_y1r, &eq->hp_y2r);
				g_tmp_l[i] = lo_l * gl + mi_l * gm + hi_l * gh;
				g_tmp_r[i] = lo_r * gl + mi_r * gm + hi_r * gh;
			}
		}

		/* ── Insert FX (post-EQ, pre-mix) -- slots 0 and 1 serial ── */
		for (int _s = 0; _s < FX_SLOTS_PER_DECK; _s++) {
			fx_apply(fx_slot(t, _s), g_tmp_l, g_tmp_r,
			         PERIOD_FRAMES);
		}

		/* ── Compute per-deck peak for VU meter ── */
		float dpk = 0.0f;
		for (int i = 0; i < PERIOD_FRAMES; i++) {
			float al = fabsf(g_tmp_l[i]);
			float ar = fabsf(g_tmp_r[i]);
			if (al > dpk) {
				dpk = al;
			}
			if (ar > dpk) {
				dpk = ar;
			}
		}
		tr->period_peak = dpk;

		/* ── Accumulate into mix buses ── */
#if defined(__ALTIVEC__)
		vfloat v_gain = (vfloat){gain, gain, gain, gain};
		float hpg = tr->volume * tr->gain;
		vfloat v_hp_gain = (vfloat){hpg, hpg, hpg, hpg};
		vfloat v_zero = (vfloat){0.0f, 0.0f, 0.0f, 0.0f};
		for (int i = 0; i < PERIOD_FRAMES; i += 4) {
			vfloat v_tmp_l = vec_ld(0, &g_tmp_l[i]);
			vfloat v_tmp_r = vec_ld(0, &g_tmp_r[i]);
			vfloat v_mix_l = vec_ld(0, &g_mix_l[i]);
			vfloat v_mix_r = vec_ld(0, &g_mix_r[i]);

			v_mix_l = vec_madd(v_tmp_l, v_gain, v_mix_l);
			v_mix_r = vec_madd(v_tmp_r, v_gain, v_mix_r);
			vec_st(v_mix_l, 0, (float *)&g_mix_l[i]);
			vec_st(v_mix_r, 0, (float *)&g_mix_r[i]);

			if (tr->cue_active) {
				vfloat v_hp_l = vec_ld(0, &g_mix_hp_l[i]);
				vfloat v_hp_r = vec_ld(0, &g_mix_hp_r[i]);
				v_hp_l = vec_madd(v_tmp_l, v_hp_gain, v_hp_l);
				v_hp_r = vec_madd(v_tmp_r, v_hp_gain, v_hp_r);
				vec_st(v_hp_l, 0, (float *)&g_mix_hp_l[i]);
				vec_st(v_hp_r, 0, (float *)&g_mix_hp_r[i]);
			}
		}
#else
		for (int i = 0; i < PERIOD_FRAMES; i++) {
			g_mix_l[i] += g_tmp_l[i] * gain;
			g_mix_r[i] += g_tmp_r[i] * gain;

			if (tr->cue_active) {
				g_mix_hp_l[i] +=
				    g_tmp_l[i] * tr->volume * tr->gain;
				g_mix_hp_r[i] +=
				    g_tmp_r[i] * tr->volume * tr->gain;
			}
		}
#endif
	}

	/* ── Master bus FX (post-crossfade, pre-clip) ── */
	{
		FXSlot *ms = fx_master();
		if (ms->type != FX_NONE) {
			fx_apply(ms, g_mix_l, g_mix_r, PERIOD_FRAMES);
		}
	}

	/* ── Sampler Mixing ── */
	for (int s = 0; s < MAX_SAMPLER_SLOTS; s++) {
		mix_sampler(&g_samplers[s], g_mix_l, g_mix_r, PERIOD_FRAMES);
	}

	/* ── Clipping & VU Meters ── */
	float period_peak_l = 0.0f, period_peak_r = 0.0f;
	for (int i = 0; i < PERIOD_FRAMES; i++) {
		float l = g_mix_l[i], r = g_mix_r[i];
		if (l > 0.95f) {
			l = 0.95f + 0.05f * tanhf((l - 0.95f) / 0.05f);
		}
		if (l < -0.95f) {
			l = -0.95f - 0.05f * tanhf((-l - 0.95f) / 0.05f);
		}
		if (r > 0.95f) {
			r = 0.95f + 0.05f * tanhf((r - 0.95f) / 0.05f);
		}
		if (r < -0.95f) {
			r = -0.95f - 0.05f * tanhf((-r - 0.95f) / 0.05f);
		}
		float al = fabsf(l), ar = fabsf(r);
		if (al > period_peak_l) {
			period_peak_l = al;
		}
		if (ar > period_peak_r) {
			period_peak_r = ar;
		}
		g_pcm_buf[i * 2] = (int16_t)(l * 32767.0f);
		g_pcm_buf[i * 2 + 1] = (int16_t)(r * 32767.0f);

		/* Headphone Output processing */
		float hl = g_mix_hp_l[i] * hp_mvol;
		float hr = g_mix_hp_r[i] * hp_mvol;
		if (hl > 0.95f) {
			hl = 0.95f + 0.05f * tanhf((hl - 0.95f) / 0.05f);
		}
		if (hl < -0.95f) {
			hl = -0.95f - 0.05f * tanhf((-hl - 0.95f) / 0.05f);
		}
		if (hr > 0.95f) {
			hr = 0.95f + 0.05f * tanhf((hr - 0.95f) / 0.05f);
		}
		if (hr < -0.95f) {
			hr = -0.95f - 0.05f * tanhf((-hr - 0.95f) / 0.05f);
		}
		g_pcm_hp_buf[i * 2] = (int16_t)(hl * 32767.0f);
		g_pcm_hp_buf[i * 2 + 1] = (int16_t)(hr * 32767.0f);
	}

	g_vu_l = g_vu_l * VU_DECAY + period_peak_l * (1.0f - VU_DECAY);
	g_vu_r = g_vu_r * VU_DECAY + period_peak_r * (1.0f - VU_DECAY);

	/* Peak hold -- rise instantly, decay slowly over ~3s */
	if (period_peak_l > g_vu_peak_l) {
		g_vu_peak_l = period_peak_l;
	} else {
		g_vu_peak_l *= 0.998f;
	}
	if (period_peak_r > g_vu_peak_r) {
		g_vu_peak_r = period_peak_r;
	} else {
		g_vu_peak_r *= 0.998f;
	}

	/* ── Output ── */
	if (g_pcm) {
		int err = snd_pcm_writei(g_pcm, g_pcm_buf, PERIOD_FRAMES);
		if (err < 0) {
			snd_pcm_recover(g_pcm, err, 0);
		} else if (err < (int)PERIOD_FRAMES) {
			usleep(1000);
		}
	} else {
		/* No audio device opened? Don't burn CPU.
		 * 512 / 44100 = 11.6ms */
		usleep(11000);
	}

	if (g_pcm_hp) {
		int err = snd_pcm_writei(g_pcm_hp, g_pcm_hp_buf, PERIOD_FRAMES);
		if (err < 0)
			snd_pcm_recover(g_pcm_hp, err, 0);
	}
}

/* --- alsa_init --- */
/* ──────────────────────────────────────────────
   ALSA Init
   ────────────────────────────────────────────── */

/* Set a thread to real-time priority (SCHED_FIFO) */
void set_realtime_priority(pthread_t thread, int priority)
{
	struct sched_param param;
	param.sched_priority = priority;
	int policy = SCHED_FIFO;
	int s = pthread_setschedparam(thread, policy, &param);
	if (s != 0) {
		if (s == EPERM) {
			/* Not running as root or missing CAP_SYS_NICE */
			return;
		}
		fprintf(stderr, "Warning: failed to set RT priority (%d): %s\n",
		        s, strerror(s));
	}
}

int init_alsa(void)
{
	snd_pcm_hw_params_t *hwp;
	int err, i;
	snd_pcm_format_t formats[] = {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	    SND_PCM_FORMAT_S16_BE, SND_PCM_FORMAT_S16_LE
#else
	    SND_PCM_FORMAT_S16_LE, SND_PCM_FORMAT_S16_BE
#endif
	};

	/* ── Master (Main) Output ── */
	err = snd_pcm_open(&g_pcm, g_pcm_dev_str, SND_PCM_STREAM_PLAYBACK, 0);
	if (err < 0) {
		return err;
	}

	snd_pcm_hw_params_alloca(&hwp);

	/* Try multiple formats */
	int format_ok = 0;
	for (i = 0; i < 2; i++) {
		snd_pcm_hw_params_any(g_pcm, hwp);
		snd_pcm_hw_params_set_access(g_pcm, hwp,
		                             SND_PCM_ACCESS_RW_INTERLEAVED);
		if (snd_pcm_hw_params_set_format(g_pcm, hwp, formats[i]) == 0) {
			format_ok = 1;
			break;
		}
	}
	if (!format_ok) {
		snd_pcm_close(g_pcm);
		g_pcm = NULL;
		return -EINVAL;
	}

	snd_pcm_hw_params_set_channels(g_pcm, hwp, CHANNELS);
	unsigned int rate = SAMPLE_RATE;
	snd_pcm_hw_params_set_rate_near(g_pcm, hwp, &rate, 0);
	g_actual_sample_rate = rate;
	af_set_target_rate(rate);
	snd_pcm_uframes_t period = (snd_pcm_uframes_t)g_opts.period_frames;
	snd_pcm_hw_params_set_period_size_near(g_pcm, hwp, &period, 0);
	snd_pcm_uframes_t buffer =
		(snd_pcm_uframes_t)g_opts.period_frames * (snd_pcm_uframes_t)g_opts.buffer_periods;
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
		snd_pcm_uframes_t pf = (snd_pcm_uframes_t)g_opts.period_frames;
		snd_pcm_sw_params_set_start_threshold(g_pcm, swp, pf);
		snd_pcm_sw_params_set_avail_min(g_pcm, swp, pf);
		snd_pcm_sw_params(g_pcm, swp);
	}
	snd_pcm_prepare(g_pcm);

	/* ── Headphone Output ── */
	err = snd_pcm_open(&g_pcm_hp, g_pcm_hp_dev_str, SND_PCM_STREAM_PLAYBACK,
	                   SND_PCM_NONBLOCK);
	if (err >= 0) {
		snd_pcm_hw_params_t *hwp_hp;
		snd_pcm_hw_params_alloca(&hwp_hp);
		snd_pcm_hw_params_any(g_pcm_hp, hwp_hp);
		snd_pcm_hw_params_set_access(g_pcm_hp, hwp_hp,
		                             SND_PCM_ACCESS_RW_INTERLEAVED);
		snd_pcm_hw_params_set_format(g_pcm_hp, hwp_hp,
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
		                             SND_PCM_FORMAT_S16_BE
#else
		                             SND_PCM_FORMAT_S16_LE
#endif
		);
		snd_pcm_hw_params_set_channels(g_pcm_hp, hwp_hp, CHANNELS);
		unsigned int rate_hp = g_actual_sample_rate;
		snd_pcm_hw_params_set_rate_near(g_pcm_hp, hwp_hp, &rate_hp, 0);
		snd_pcm_uframes_t period_hp = (snd_pcm_uframes_t)g_opts.period_frames;
		snd_pcm_hw_params_set_period_size_near(g_pcm_hp, hwp_hp,
		                                       &period_hp, 0);
		snd_pcm_uframes_t buffer_hp =
			(snd_pcm_uframes_t)g_opts.period_frames *
			(snd_pcm_uframes_t)g_opts.buffer_periods;
		snd_pcm_hw_params_set_buffer_size_near(g_pcm_hp, hwp_hp,
		                                       &buffer_hp);

		err = snd_pcm_hw_params(g_pcm_hp, hwp_hp);
		if (err < 0) {
			snd_pcm_close(g_pcm_hp);
			g_pcm_hp = NULL;
		} else {
			snd_pcm_sw_params_t *swp_hp;
			snd_pcm_sw_params_alloca(&swp_hp);
			snd_pcm_sw_params_current(g_pcm_hp, swp_hp);
			snd_pcm_sw_params_set_avail_min(g_pcm_hp, swp_hp,
			                                PERIOD_FRAMES);
			snd_pcm_sw_params(g_pcm_hp, swp_hp);
			snd_pcm_prepare(g_pcm_hp);
		}
	} else {
		g_pcm_hp = NULL;
	}

	return 0;
}

/* --- pcm_devices --- */
void pcm_enumerate_devices(void)
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
		if (snd_ctl_open(&ctl, ctl_name, 0) < 0) {
			continue;
		}

		snd_pcm_info_t *info;
		snd_pcm_info_alloca(&info);

		int device = -1;
		while (snd_ctl_pcm_next_device(ctl, &device) == 0 &&
		       device >= 0 && g_pcm_ndevices < PCM_MAX_DEVICES) {
			snd_pcm_info_set_device(info, (unsigned)device);
			snd_pcm_info_set_subdevice(info, 0);
			snd_pcm_info_set_stream(info, SND_PCM_STREAM_PLAYBACK);
			if (snd_ctl_pcm_info(ctl, info) == 0) {
				PCMDevice *e = &g_pcm_devlist[g_pcm_ndevices++];
				snprintf(e->dev, sizeof(e->dev), "hw:%d,%d",
				         card, device);
				snprintf(e->name, sizeof(e->name), "%s",
				         snd_pcm_info_get_name(info));
			}
		}
		snd_ctl_close(ctl);
	}
	return;
}

/* Switch to a different PCM output device at runtime.
 * Stops audio thread momentarily, closes old PCM, opens new one. */
void pcm_open_device(int dev_idx)
{
	if (dev_idx < 0 || dev_idx >= g_pcm_ndevices) {
		return;
	}

	/* Close existing device -- audio thread will stall briefly */
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

/* Switch to a different headphone PCM device at runtime. */
void hp_open_device(int dev_idx)
{
	if (dev_idx < 0 || dev_idx >= g_pcm_ndevices) {
		return;
	}

	if (g_pcm_hp) {
		snd_pcm_drain(g_pcm_hp);
		snd_pcm_close(g_pcm_hp);
		g_pcm_hp = NULL;
	}

	g_pcm_hp_dev_sel = dev_idx;
	snprintf(g_pcm_hp_dev_str, sizeof(g_pcm_hp_dev_str), "%s",
	         g_pcm_devlist[dev_idx].dev);

	int err = snd_pcm_open(&g_pcm_hp, g_pcm_hp_dev_str,
	                       SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK);
	if (err >= 0) {
		snd_pcm_hw_params_t *hwp_hp;
		snd_pcm_hw_params_alloca(&hwp_hp);
		snd_pcm_hw_params_any(g_pcm_hp, hwp_hp);
		snd_pcm_hw_params_set_access(g_pcm_hp, hwp_hp,
		                             SND_PCM_ACCESS_RW_INTERLEAVED);
		snd_pcm_hw_params_set_format(g_pcm_hp, hwp_hp,
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
		                             SND_PCM_FORMAT_S16_BE
#else
		                             SND_PCM_FORMAT_S16_LE
#endif
		);
		snd_pcm_hw_params_set_channels(g_pcm_hp, hwp_hp, CHANNELS);
		unsigned int rate_hp = g_actual_sample_rate;
		snd_pcm_hw_params_set_rate_near(g_pcm_hp, hwp_hp, &rate_hp, 0);
		snd_pcm_uframes_t period_hp = (snd_pcm_uframes_t)g_opts.period_frames;
		snd_pcm_hw_params_set_period_size_near(g_pcm_hp, hwp_hp,
		                                       &period_hp, 0);
		snd_pcm_uframes_t buffer_hp =
			(snd_pcm_uframes_t)g_opts.period_frames *
			(snd_pcm_uframes_t)g_opts.buffer_periods;
		snd_pcm_hw_params_set_buffer_size_near(g_pcm_hp, hwp_hp,
		                                       &buffer_hp);
		err = snd_pcm_hw_params(g_pcm_hp, hwp_hp);
		if (err < 0) {
			snd_pcm_close(g_pcm_hp);
			g_pcm_hp = NULL;
			snprintf(g_fb_status, sizeof(g_fb_status),
			         "Headphones: FAILED to open %s",
			         g_pcm_hp_dev_str);
		} else {
			snd_pcm_sw_params_t *swp_hp;
			snd_pcm_sw_params_alloca(&swp_hp);
			snd_pcm_sw_params_current(g_pcm_hp, swp_hp);
			snd_pcm_sw_params_set_avail_min(g_pcm_hp, swp_hp,
			                                PERIOD_FRAMES);
			snd_pcm_sw_params(g_pcm_hp, swp_hp);
			snd_pcm_prepare(g_pcm_hp);
			snprintf(g_fb_status, sizeof(g_fb_status),
			         "Headphones: %s", g_pcm_devlist[dev_idx].name);
			settings_save();
		}
	} else {
		g_pcm_hp = NULL;
		snprintf(g_fb_status, sizeof(g_fb_status),
		         "Headphones: FAILED to open %s", g_pcm_hp_dev_str);
	}
}

/* ──────────────────────────────────────────────
   Cleanup helper: close PCM handles
   ────────────────────────────────────────────── */
void audio_pcm_drop_all(void)
{
	if (g_pcm)
		snd_pcm_drop(g_pcm);
	if (g_pcm_hp)
		snd_pcm_drop(g_pcm_hp);
}

void audio_pcm_close_all(void)
{
	if (g_pcm) {
		snd_pcm_close(g_pcm);
		g_pcm = NULL;
	}
	if (g_pcm_hp) {
		snd_pcm_close(g_pcm_hp);
		g_pcm_hp = NULL;
	}
}

int audio_pcm_is_open(void)
{
	return g_pcm != NULL;
}

/* ──────────────────────────────────────────────
   Load worker, audio thread, batch/enqueue (moved from djcmd.c)
   ────────────────────────────────────────────── */
static void tag_clean(char *s)
{
	/* Strip leading/trailing whitespace and non-printable bytes */
	int len = (int)strlen(s);
	while (len > 0 && (unsigned char)s[len - 1] <= 0x20) {
		s[--len] = '\0';
	}
	int start = 0;
	while (start < len && (unsigned char)s[start] <= 0x20) {
		start++;
	}
	if (start > 0) {
		memmove(s, s + start, len - start + 1);
	}
}

void *load_worker(void *arg)
{
	(void)arg;
	while (g_running) {
		pthread_mutex_lock(&g_load_mutex);
		while (!g_load_job.valid && g_running) {
			pthread_cond_wait(&g_load_cond, &g_load_mutex);
		}

		if (!g_running) {
			pthread_mutex_unlock(&g_load_mutex);
			break;
		}

		/* Consume job */
		char path[FB_PATH_MAX + 256];
		int deck = g_load_job.deck;
		int analyze_only = g_load_job.analyze_only;
		int batch_path_only = g_load_job.batch_path_only;
		snprintf(path, sizeof(path), "%s", g_load_job.path);
		path[sizeof(path) - 1] = '\0';
		g_load_job.valid = 0;
		pthread_mutex_unlock(&g_load_mutex);

		/* ── Batch-mode: load into temp Track, analyze, save sidecar,
		 * free ── */
		if (batch_path_only) {
			Track *bt = (Track *)calloc(1, sizeof(Track));
			if (bt) {
				pthread_mutex_init(&bt->lock, NULL);
				if (load_track(bt, path) == 0) {
					rebuild_waveform_and_grid(bt, 1);
					sidecar_save(bt);
					snprintf(
					    g_fb_status, sizeof(g_fb_status),
					    "Batch: %.1f BPM \u2192 %.200s",
					    (double)bt->bpm,
					    bt->tag_title[0] ? bt->tag_title
					                     : bt->filename);
				}
				free(bt->data);
				free(bt->wfm_low);
				free(bt->wfm_mid);
				free(bt->wfm_high);
				pthread_mutex_destroy(&bt->lock);
				free(bt);
			}
			g_batch_job_done = 1;
			continue;
		}

		/* Do the actual work outside the lock */
		Track *lt = &g_tracks[deck];

		if (analyze_only) {
			if (lt->loaded) {
				rebuild_waveform_and_grid(lt, 1);
				sidecar_save(lt);
				snprintf(g_fb_status, sizeof(g_fb_status),
				         "Deck %c: %.1f BPM found",
				         DECK_NUM(deck), (double)lt->bpm);
			}
			continue;
		}

		pthread_mutex_lock(&lt->lock);
		if (!g_autoplay_pending[deck]) {
			lt->playing = 0;
		}
		pthread_mutex_unlock(&lt->lock);

		if (load_track(lt, path) == 0) {
			/* ── Metadata strategy
			 * ────────────────────────────────────
			 * 1. Try djcmd's own .djcmd sidecar first
			 * (authoritative).
			 * 2. If missing, use Mixxx library as fallback.
			 * 3. Hot cues and keys from Mixxx are imported if not
			 * already present. */

			MixxxMeta mx;
			memset(&mx, 0, sizeof(mx));
			int got_mixxx = mixxx_import(path, &mx);

			/* Always load waveform/cache first */
			int got_cache = (sidecar_load(lt) == 0);

			if (got_cache) {
				snprintf(g_fb_status, sizeof(g_fb_status),
				         "Sidecar: %.1f BPM \u2192 Deck %c",
				         (double)lt->bpm, DECK_NUM(deck));
			} else if (got_mixxx && mx.bpm > 0.0f) {
				pthread_mutex_lock(&lt->lock);
				lt->bpm = mx.bpm;
				lt->bpm_offset = mx.bpm_offset;
				pthread_mutex_unlock(&lt->lock);
				snprintf(g_fb_status, sizeof(g_fb_status),
				         "Mixxx: %.1f BPM \u2192 Deck %c",
				         (double)mx.bpm, DECK_NUM(deck));
			}

			/* Analysis fallback if both failed */
			if (!got_cache && (!got_mixxx || mx.bpm <= 0.0f)) {
				snprintf(g_fb_status, sizeof(g_fb_status),
				         "Analyzing Deck %c...",
				         DECK_NUM(deck));
				rebuild_waveform_and_grid(lt, 0);
				sidecar_save(lt);
				snprintf(g_fb_status, sizeof(g_fb_status),
				         "Loaded \u2192 Deck %c",
				         DECK_NUM(deck));
			}

			/* Import Key and Hot Cues from Mixxx if missing */
			if (got_mixxx) {
				pthread_mutex_lock(&lt->lock);
				if (!lt->tag_key[0] && mx.tag_key[0]) {
					snprintf(lt->tag_key,
					         sizeof(lt->tag_key), "%s",
					         mx.tag_key);
				}
				for (int ci = 0; ci < MAX_CUES; ci++) {
					if (!lt->cue_set[ci] &&
					    mx.cue_set[ci] &&
					    mx.cue[ci] < lt->num_frames) {
						lt->cue[ci] = mx.cue[ci];
						lt->cue_set[ci] = 1;
					}
				}
				pthread_mutex_unlock(&lt->lock);
				if (mx.beat_frames) {
					free(mx.beat_frames);
				}
			}

			/* If we used Mixxx data, persist it to sidecar now */
			if (!got_cache && got_mixxx && mx.bpm > 0.0f) {
				sidecar_save(lt);
			}

			/* ── Instant Doubles
			 * ────────────────────────────────────────── Check if
			 * this exact file is already playing on another deck.
			 * If so, copy the playback state (pos, pitch, loops)
			 * exactly. */
			for (int d = 0; d < g_num_tracks; d++) {
				if (d == deck) {
					continue;
				}
				Track *ot = &g_tracks[d];
				if (ot->loaded &&
				    strcmp(ot->filename, path) == 0) {
					pthread_mutex_lock(&ot->lock);
					uint32_t opos = ot->pos;
					float opitch = ot->pitch;
					uint32_t ols = ot->loop_start;
					uint32_t ole = ot->loop_end;
					int olooping = ot->looping;
					int oplaying = ot->playing;
					pthread_mutex_unlock(&ot->lock);

					pthread_mutex_lock(&lt->lock);
					lt->pos = opos;
					lt->pitch = opitch;
					lt->loop_start = ols;
					lt->loop_end = ole;
					lt->looping = olooping;
					lt->playing = oplaying;
					/* Sync audio engine state to new
					 * position */
					wsola_reset(&g_wsola[deck], lt->pos);
					pthread_mutex_unlock(&lt->lock);

					snprintf(
					    g_fb_status, sizeof(g_fb_status),
					    "Instant Double \u2192 Deck %c",
					    DECK_NUM(deck));
					break;
				}
			}

			/* Read ID3/Vorbis/RIFF tags -- always, regardless of
			 * BPM source */
			pthread_mutex_lock(&lt->lock);
			read_tags(path, lt->tag_title, sizeof(lt->tag_title),
			          lt->tag_artist, sizeof(lt->tag_artist));
			lt->key_lock = g_opts.key_lock_default;
			if (lt->key_lock) {
				wsola_reset(&g_wsola[deck], lt->pos);
			}
			pthread_mutex_unlock(&lt->lock);

			/* Session mix log -- record this track load */
			mixlog_track_loaded(deck, lt);
			if (g_autoplay_pending[deck]) {
				g_autoplay_ready[deck] = 1;
			}
			g_autoplay_pending[deck] = 0;

			/* ── Auto master handoff
			 * ─────────────────────────────────── If the deck we
			 * just loaded INTO was the sync master, and another
			 * deck is currently playing, hand master status to the
			 * highest-priority playing deck (lowest index first).
			 * This lets the DJ load a new track onto the master
			 * without losing sync -- the running deck becomes the
			 * new master. */
			if (g_opts.sync_auto_handoff && deck == g_sync_leader) {
				for (int ti = 0; ti < g_num_tracks; ti++) {
					if (ti == deck) {
						continue;
					}
					if (g_tracks[ti].loaded &&
					    g_tracks[ti].playing) {
						g_sync_leader = ti;
						/* Keep any sync-locked decks
						 * locked to the new master */
						break;
					}
				}
			}
		} else {
			snprintf(g_fb_status, sizeof(g_fb_status),
			         "Load FAILED: Deck %c", DECK_NUM(deck));
			g_autoplay_pending[deck] = 0;
		}
	}
	return NULL;
}

/* Post a load job -- safe to call from the UI thread */
/* ── Batch BPM analyze helpers ───────────────────────────────────────────── */
void batch_free_queue(void)
{
	if (g_batch_queue) {
		for (int i = 0; i < g_batch_queue_count; i++) {
			free(g_batch_queue[i].path);
		}
		free(g_batch_queue);
		g_batch_queue = NULL;
	}
	g_batch_queue_count = 0;
	g_batch_queue_pos = 0;
}

/* Move the panel cursor to the entry currently being analyzed so the list
 * auto-scrolls and the user can see live BPM values appearing. */
static void batch_update_panel_cursor(int panel_idx)
{
	switch (g_batch_panel) {
	case 0:
		g_fb_sel = panel_idx;
		break;
	case 1:
		g_pl_sel = panel_idx;
		break;
	case 2:
		g_lib_sel = panel_idx;
		break;
	case 3:
		g_crate_tracks_sel = panel_idx;
		break;
	}
}

static void batch_enqueue_next(void)
{
	g_batch_job_done = 0;

	if (g_batch_queue_pos >= g_batch_queue_count) {
		/* All done -- update each entry's BPM from the freshly written
		 * sidecar */
		for (int i = 0; i < g_batch_queue_count; i++) {
			float bpm = cache_get_bpm(g_batch_queue[i].path);
			if (bpm <= 0.0f) {
				continue;
			}
			int pi = g_batch_queue[i].panel_idx;
			switch (g_batch_panel) {
			case 0:
				if (pi < g_fb_count) {
					g_fb_entries[pi].bpm = bpm;
				}
				break;
			case 1:
				if (pi < g_pl_count) {
					g_pl[pi].bpm = bpm;
				}
				break;
			case 2:
				if (g_lib && pi < g_lib_count) {
					g_lib[pi].bpm = bpm;
				}
				break;
			case 3:
				if (pi < g_crate_tracks_count) {
					g_crate_tracks[pi].bpm = bpm;
				}
				break;
			}
		}
		/* Return cursor to start of the analyzed range */
		if (g_batch_queue_count > 0) {
			batch_update_panel_cursor(g_batch_queue[0].panel_idx);
		}
		g_batch_running = 0;
		g_bpm_detect_lo = 0.0f;
		g_bpm_detect_hi = 0.0f;
		batch_free_queue();
		snprintf(g_fb_status, sizeof(g_fb_status),
		         "Batch BPM analyze complete");
		return;
	}

	BatchQEntry *e = &g_batch_queue[g_batch_queue_pos++];
	/* Advance the panel cursor so the list scrolls to the current track */
	batch_update_panel_cursor(e->panel_idx);

	pthread_mutex_lock(&g_load_mutex);
	snprintf(g_load_job.path, sizeof(g_load_job.path), "%s", e->path);
	g_load_job.deck = 0;
	g_load_job.analyze_only = 0;
	g_load_job.batch_path_only = 1;
	g_load_job.valid = 1;
	snprintf(g_fb_status, sizeof(g_fb_status),
	         "Batch BPM [%d/%d] analyzing...", g_batch_queue_pos,
	         g_batch_queue_count);
	pthread_cond_signal(&g_load_cond);
	pthread_mutex_unlock(&g_load_mutex);
}

void batch_analyze_start(float lo, float hi)
{
	batch_free_queue();
	g_bpm_detect_lo = lo;
	g_bpm_detect_hi = hi;
	g_batch_panel = g_panel;

	int max_paths = 0;
	if (g_panel == 0) {
		max_paths = g_fb_count;
	} else if (g_panel == 1) {
		max_paths = g_pl_count;
	} else if (g_panel == 2) {
		max_paths = g_lib_count;
	} else if (g_panel == 3 && g_crate_view_level == 1) {
		max_paths = g_crate_tracks_count;
	}

	if (max_paths <= 0) {
		g_bpm_detect_lo = 0.0f;
		g_bpm_detect_hi = 0.0f;
		snprintf(g_fb_status, sizeof(g_fb_status),
		         "Batch BPM: no tracks in panel");
		return;
	}

	g_batch_queue =
	    (BatchQEntry *)malloc((size_t)max_paths * sizeof(BatchQEntry));
	if (!g_batch_queue) {
		g_bpm_detect_lo = 0.0f;
		g_bpm_detect_hi = 0.0f;
		return;
	}

	char pathbuf[FB_PATH_MAX + 256];
	for (int i = 0; i < max_paths; i++) {
		pathbuf[0] = '\0';
		if (g_panel == 0) {
			if (g_fb_entries[i].is_dir) {
				continue;
			}
			snprintf(pathbuf, sizeof(pathbuf), "%.*s/%.*s",
			         FB_PATH_MAX - 1, g_fb_path, 255,
			         g_fb_entries[i].name);
		} else if (g_panel == 1) {
			snprintf(pathbuf, sizeof(pathbuf), "%s", g_pl[i].path);
		} else if (g_panel == 2 && g_lib) {
			snprintf(pathbuf, sizeof(pathbuf), "%s", g_lib[i].path);
		} else if (g_panel == 3 && g_crate_view_level == 1) {
			snprintf(pathbuf, sizeof(pathbuf), "%s",
			         g_crate_tracks[i].path);
		}
		if (!pathbuf[0]) {
			continue;
		}
		char *dup = strdup(pathbuf);
		if (!dup) {
			continue;
		}
		g_batch_queue[g_batch_queue_count].path = dup;
		g_batch_queue[g_batch_queue_count].panel_idx = i;
		g_batch_queue_count++;
	}

	if (g_batch_queue_count == 0) {
		batch_free_queue();
		g_bpm_detect_lo = 0.0f;
		g_bpm_detect_hi = 0.0f;
		snprintf(g_fb_status, sizeof(g_fb_status),
		         "Batch BPM: no audio files in panel");
		return;
	}

	g_batch_running = 1;
	g_batch_job_done = 0;
	batch_enqueue_next();
}

/* Called from the UI thread each frame to advance the batch queue when a job
 * finishes.  Mirrors the check that lived in main()'s loop in the monolith. */
void batch_tick(void)
{
	if (g_batch_running && g_batch_job_done) {
		batch_enqueue_next();
	}
}

void enqueue_load(int deck, const char *path)
{
	pthread_mutex_lock(&g_load_mutex);
	g_load_job.deck = deck;
	g_load_job.analyze_only = 0;
	g_load_job.batch_path_only = 0;
	snprintf(g_load_job.path, sizeof(g_load_job.path), "%s", path);
	g_load_job.path[sizeof(g_load_job.path) - 1] = '\0';
	g_load_job.valid = 1;
	snprintf(g_fb_status, sizeof(g_fb_status), "Loading Deck %c...",
	         DECK_NUM(deck));
	pthread_cond_signal(&g_load_cond);
	pthread_mutex_unlock(&g_load_mutex);
}

void enqueue_analyze(int deck)
{
	pthread_mutex_lock(&g_load_mutex);
	g_load_job.deck = deck;
	g_load_job.analyze_only = 1;
	g_load_job.batch_path_only = 0;
	g_load_job.valid = 1;
	snprintf(g_fb_status, sizeof(g_fb_status), "Analyzing Deck %c BPM...",
	         DECK_NUM(deck));
	pthread_cond_signal(&g_load_cond);
	pthread_mutex_unlock(&g_load_mutex);
}

/* ──────────────────────────────────────────────
   Audio Thread
   ────────────────────────────────────────────── */
/* ══════════════════════════════════════════════════════════════════════════
   NS7III DISPLAY SUBSYSTEM -- moved to ns7iii_displaysub.h
   Disabled: caused CPU spikes and audio dropouts at 20 Hz update rate.
   Re-enable once Numark handshake (0x50/0x52/0x53/0x55) is solved.
   ══════════════════════════════════════════════════════════════════════════ */

#if 0  /* NS7III display subsystem -- see ns7iii_displaysub.h */



/* ══════════════════════════════════════════════════════════════════════════
   END NS7III DISPLAY SUBSYSTEM
   ══════════════════════════════════════════════════════════════════════════ */
#endif /* NS7III display subsystem */

void *audio_thread(void *arg)
{
	(void)arg;
	while (g_running) {
		mix_and_write();
	}
	return NULL;
}

/* Recursive library scan, file browser populate, MusicBrainz lookup,
 * playlist helpers, and crate row mapping → djcmd_library.c
 */
