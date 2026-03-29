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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "djcmd_audio.h"

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
		double a   = 2.0 * M_PI * k / PV_N;
		g_pv_cos[k] = (float)cos(a);
		g_pv_sin[k] = (float)sin(a);
	}
}

static void pv_fft(float *re, float *im, int n, int forward)
{
	for (int i = 1, j = 0; i < n; i++) {
		int bit = n >> 1;
		for (; j & bit; bit >>= 1)
			j ^= bit;
		j ^= bit;
		if (i < j) {
			float t;
			t = re[i]; re[i] = re[j]; re[j] = t;
			t = im[i]; im[i] = im[j]; im[j] = t;
		}
	}
	for (int m = 1; m < n; m <<= 1) {
		int step = PV_N / (m << 1);
		for (int k = 0; k < m; k++) {
			float wr =  g_pv_cos[k * step];
			float wi = (forward ? -1.0f : 1.0f) * g_pv_sin[k * step];
			for (int j = k; j < n; j += m << 1) {
				float ur = re[j],     ui = im[j];
				float vr = re[j+m]*wr - im[j+m]*wi;
				float vi = re[j+m]*wi + im[j+m]*wr;
				re[j]   = ur + vr;  im[j]   = ui + vi;
				re[j+m] = ur - vr;  im[j+m] = ui - vi;
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
	int src0    = (int)pv->src_pos;
	int max_src = (int)t->num_frames - PV_N;
	if (src0 < 0)          src0 = 0;
	if (src0 > max_src)    src0 = (max_src > 0) ? max_src : 0;

	float re_l[PV_N], im_l[PV_N];
	float re_r[PV_N], im_r[PV_N];

	for (int i = 0; i < PV_N; i++) {
		float w  = 0.5f * (1.0f - g_pv_cos[i]);
		int   fi = src0 + i;
		if (fi >= (int)t->num_frames)
			fi = (int)t->num_frames - 1;
		re_l[i] = (t->data[fi * 2]     / 32768.0f) * w;
		im_l[i] = 0.0f;
		re_r[i] = (t->data[fi * 2 + 1] / 32768.0f) * w;
		im_r[i] = 0.0f;
	}

	pv_fft(re_l, im_l, PV_N, 1);
	pv_fft(re_r, im_r, PV_N, 1);

	float two_pi = 2.0f * (float)M_PI;
	float inv_ha = (fabsf(ha) >= 1.0f) ? (1.0f / ha) : 0.0f;

	for (int k = 0; k < PV_BINS; k++) {
		float omega_k  = two_pi * (float)k / (float)PV_N;
		float expected = omega_k * ha;

		float an_l  = atan2f(im_l[k], re_l[k]);
		float mag_l = sqrtf(re_l[k]*re_l[k] + im_l[k]*im_l[k]);
		if (pv->initialized) {
			float dev = an_l - pv->ph_an_l[k] - expected;
			dev -= two_pi * floorf(dev / two_pi + 0.5f);
			pv->ph_syn_l[k] += (omega_k + dev * inv_ha) * (float)PV_HS;
		} else {
			pv->ph_syn_l[k] = an_l;
		}
		pv->ph_an_l[k] = an_l;
		re_l[k] = mag_l * cosf(pv->ph_syn_l[k]);
		im_l[k] = mag_l * sinf(pv->ph_syn_l[k]);

		float an_r  = atan2f(im_r[k], re_r[k]);
		float mag_r = sqrtf(re_r[k]*re_r[k] + im_r[k]*im_r[k]);
		if (pv->initialized) {
			float dev = an_r - pv->ph_an_r[k] - expected;
			dev -= two_pi * floorf(dev / two_pi + 0.5f);
			pv->ph_syn_r[k] += (omega_k + dev * inv_ha) * (float)PV_HS;
		} else {
			pv->ph_syn_r[k] = an_r;
		}
		pv->ph_an_r[k] = an_r;
		re_r[k] = mag_r * cosf(pv->ph_syn_r[k]);
		im_r[k] = mag_r * sinf(pv->ph_syn_r[k]);
	}

	for (int k = PV_BINS; k < PV_N; k++) {
		int m    = PV_N - k;
		re_l[k]  =  re_l[m];  im_l[k] = -im_l[m];
		re_r[k]  =  re_r[m];  im_r[k] = -im_r[m];
	}

	pv_fft(re_l, im_l, PV_N, 0);
	pv_fft(re_r, im_r, PV_N, 0);

	float norm = 2.0f / (3.0f * (float)PV_N);

	for (int i = 0; i < PV_N; i++) {
		float w  = 0.5f * (1.0f - g_pv_cos[i]);
		float ol = re_l[i] * norm * w;
		float or_ = re_r[i] * norm * w;
		int rp   = (pv->out_write - (PV_N - PV_HS) + i + PV_BUF * 4) &
			   (PV_BUF - 1);
		pv->out_l[rp] += ol;
		pv->out_r[rp] += or_;
	}

	pv->out_write   = (pv->out_write + PV_HS) & (PV_BUF - 1);
	pv->out_fill   += PV_HS;
	pv->src_pos    += (double)ha;
	pv->initialized = 1;
}

void pv_process(Track *t, PVState *pv, float *out_l, float *out_r,
                uint32_t out_frames, double rate, float gain)
{
	if (rate < 0.0) return;

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
		float w = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (n - 1)));
		win[i] *= w;
	}
}

static int wsola_find_best(const float *ref, const float *cand_base,
			   int cand_center, int cand_len, int win, int range, int is_eco)
{
	int half = range / 2;
	int best_d = 0;
	float best_corr = -1e30f;

	if (is_eco) {
		/* ECO mode: skip every other d, and sparse correlation search (1/8 samples) */
		for (int d = -half; d <= half; d += 2) {
			int start = cand_center + d;
			if (start < 0 || start + win > cand_len)
				continue;
			float corr = 0.0f;
			for (int i = 0; i < win; i += 8)
				corr += ref[i] * cand_base[start + i];
			if (corr > best_corr) {
				best_corr = corr;
				best_d = d;
			}
		}
	} else {
		/* Normal mode: every d, and 1/4 sample correlation search */
		for (int d = -half; d <= half; d++) {
			int start = cand_center + d;
			if (start < 0 || start + win > cand_len)
				continue;
			float corr = 0.0f;
			for (int i = 0; i < win; i += 4)
				corr += ref[i] * cand_base[start + i];
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
					ws->buf_l[ws->write_pos & (WSOLA_BUF - 1)] = 0.0f;
					ws->buf_r[ws->write_pos & (WSOLA_BUF - 1)] = 0.0f;
					ws->write_pos++;
					ws->fill++;
				}
				break;
			}
		}

		int best_offset = 0;
		if (ws->prev_valid) {
			float ref[WSOLA_WIN] __attribute__((aligned(32)));
			for (int i = 0; i < WSOLA_WIN; i++)
				ref[i] = (ws->prev_l[i] + ws->prev_r[i]) * 0.5f;

			float cand[WSOLA_WIN + WSOLA_SEARCH] __attribute__((aligned(32)));
			int cand_start = src - WSOLA_SEARCH / 2;
			if (cand_start < 0) cand_start = 0;
			int cand_len = WSOLA_WIN + WSOLA_SEARCH;
			for (int i = 0; i < cand_len; i++) {
				int fi = cand_start + i;
				if (fi < (int)end)
					cand[i] = (t->data[fi * 2] + t->data[fi * 2 + 1]) / 65536.0f;
				else
					cand[i] = 0.0f;
			}
			best_offset = wsola_find_best(ref, cand, src - cand_start, cand_len, WSOLA_WIN, WSOLA_SEARCH, is_eco);
		}

		int read_pos = src + best_offset;
		if (read_pos < 0) read_pos = 0;

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
			int rp = (ws->write_pos - (WSOLA_WIN - WSOLA_HOP) + i + WSOLA_BUF * 4) & (WSOLA_BUF - 1);
			ws->buf_l[rp] += win_l[i];
			ws->buf_r[rp] += win_r[i];
		}

		memcpy(ws->prev_l, win_l, sizeof(float) * WSOLA_WIN);
		memcpy(ws->prev_r, win_r, sizeof(float) * WSOLA_WIN);
		ws->prev_valid = 1;
		ws->write_pos = (ws->write_pos + WSOLA_HOP) & (WSOLA_BUF - 1);
		ws->fill += WSOLA_HOP;
		ws->src_pos += rate * WSOLA_HOP;
		if (ws->src_pos < 0.0) ws->src_pos = 0.0;
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

void read_pitched(Track *t, float *out_l, float *out_r,
                 uint32_t out_frames, float start_vel,
                 float target_vel, float start_nudge,
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

	for (uint32_t i = 0; i < out_frames; i++) {
		if (fpos < 0.0) fpos = 0.0;
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
		float p0l = t->data[i0 * 2]     / 32768.0f;
		float p1l = t->data[i1 * 2]     / 32768.0f;
		float p2l = t->data[i2 * 2]     / 32768.0f;
		float p3l = t->data[i3 * 2]     / 32768.0f;
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
		__m128 v_c1 = _mm_mul_ps(_mm_set1_ps(0.5f), _mm_sub_ps(v_p2, v_p0));
		__m128 v_c2 = _mm_sub_ps(_mm_add_ps(v_p0, _mm_mul_ps(_mm_set1_ps(2.0f), v_p2)),
		                         _mm_add_ps(_mm_mul_ps(_mm_set1_ps(2.5f), v_p1), 
		                                    _mm_mul_ps(_mm_set1_ps(0.5f), v_p3)));
		__m128 v_c3 = _mm_add_ps(_mm_mul_ps(_mm_set1_ps(0.5f), _mm_sub_ps(v_p3, v_p0)),
		                         _mm_mul_ps(_mm_set1_ps(1.5f), _mm_sub_ps(v_p1, v_p2)));

		/* Horner form: out = ((c3*f + c2)*f + c1)*f + p1 */
		__m128 v_out = _mm_add_ps(_mm_mul_ps(_mm_add_ps(_mm_mul_ps(_mm_add_ps(_mm_mul_ps(v_c3, v_frac), v_c2), v_frac), v_c1), v_frac), v_p1);
		v_out = _mm_mul_ps(v_out, v_gain);

		float res[4];
		_mm_storeu_ps(res, v_out);
		out_l[i] = res[0];
		out_r[i] = res[1];

#elif defined(__ALTIVEC__)
		/* AltiVec implementation: process L and R simultaneously */
		vfloat v_p0 = { p0l, p0r, 0.0f, 0.0f };
		vfloat v_p1 = { p1l, p1r, 0.0f, 0.0f };
		vfloat v_p2 = { p2l, p2r, 0.0f, 0.0f };
		vfloat v_p3 = { p3l, p3r, 0.0f, 0.0f };
		vfloat v_frac = { frac, frac, frac, frac };
		vfloat v_gain = { vgain, vgain, vgain, vgain };
		vfloat v_05   = { 0.5f, 0.5f, 0.5f, 0.5f };
		vfloat v_15   = { 1.5f, 1.5f, 1.5f, 1.5f };
		vfloat v_20   = { 2.0f, 2.0f, 2.0f, 2.0f };
		vfloat v_25   = { 2.5f, 2.5f, 2.5f, 2.5f };

		vfloat v_c1 = vec_madd(v_05, vec_sub(v_p2, v_p0), (vfloat){0});
		
		/* c2 = p0 - 2.5*p1 + 2*p2 - 0.5*p3 */
		vfloat v_c2 = vec_add(v_p0, vec_madd(v_20, v_p2, (vfloat){0}));
		v_c2 = vec_sub(v_c2, vec_madd(v_25, v_p1, (vfloat){0}));
		v_c2 = vec_sub(v_c2, vec_madd(v_05, v_p3, (vfloat){0}));

		/* c3 = 0.5*(p3 - p0) + 1.5*(p1 - p2) */
		vfloat v_c3 = vec_madd(v_05, vec_sub(v_p3, v_p0), vec_madd(v_15, vec_sub(v_p1, v_p2), (vfloat){0}));

		/* out = ((c3*f + c2)*f + c1)*f + p1 */
		vfloat v_out = vec_madd(v_c3, v_frac, v_c2);
		v_out = vec_madd(v_out, v_frac, v_c1);
		v_out = vec_madd(v_out, v_frac, v_p1);
		v_out = vec_madd(v_out, v_gain, (vfloat){0});

		out_l[i] = ((float*)&v_out)[0];
		out_r[i] = ((float*)&v_out)[1];

#else
		/* Standard C fallback */
		float c1l = 0.5f * (p2l - p0l);
		float c2l = p0l - 2.5f * p1l + 2.0f * p2l - 0.5f * p3l;
		float c3l = 0.5f * (p3l - p0l) + 1.5f * (p1l - p2l);
		out_l[i] = (((c3l * frac + c2l) * frac + c1l) * frac + p1l) * vgain;

		float c1r = 0.5f * (p2r - p0r);
		float c2r = p0r - 2.5f * p1r + 2.0f * p2r - 0.5f * p3r;
		float c3r = 0.5f * (p3r - p0r) + 1.5f * (p1r - p2r);
		out_r[i] = (((c3r * frac + c2r) * frac + c1r) * frac + p1r) * vgain;
#endif

		double step = (double)(t->pitch + current_nudge + current_vel);
		fpos += step;
		current_vel += vel_inc;
		current_nudge += nudge_inc;
	}

	if (fpos < 0.0) fpos = 0.0;
	t->pos = (uint32_t)fpos;
}
