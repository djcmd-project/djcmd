/*
 * audiofile.c — implementation
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
 * Build dependencies (single-header, drop alongside this file):
 *   minimp3.h   https://github.com/lieff/minimp3   (public domain)
 *   dr_flac.h   https://github.com/mackron/dr_libs  (public domain / MIT-0)
 *
 * Compile flags (PowerPC 7447A):
 *   -mcpu=7450 -mtune=7450 -ffast-math -funroll-loops
 *
 * No external shared libraries required beyond libc.
 */

#define _GNU_SOURCE
#include "audiofile.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <dirent.h>
#include <sys/stat.h>

/* ── Activate single-header implementations ─────────────────────── */
#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3 /* skip MP1/MP2 to save ~4 kB code */
#include "minimp3.h"

#define DR_FLAC_IMPLEMENTATION
#define DR_FLAC_NO_OGG /* skip Ogg-FLAC to save code       */
#include "dr_flac.h"

/* ── Runtime target sample rate ─────────────────────────────────────
 * Default 44100 Hz; overridden by af_set_target_rate() after ALSA open.
 */
static uint32_t af_target_rate = AF_TARGET_RATE_DEFAULT;

uint32_t af_set_target_rate(uint32_t rate)
{
	if (rate > 0)
		af_target_rate = rate;
	return af_target_rate;
}
uint32_t af_get_target_rate(void)
{
	return af_target_rate;
}

/* ─────────────────────────────────────────────────────────────────
   Internal helpers
   ───────────────────────────────────────────────────────────────── */

/* Lower-case copy of file extension (without dot), e.g. "MP3" → "mp3" */
static void get_ext_lower(const char *filename, char *ext_out, size_t max)
{
	const char *dot = strrchr(filename, '.');
	if (!dot || dot == filename) {
		ext_out[0] = '\0';
		return;
	}
	dot++;
	size_t i;
	for (i = 0; i < max - 1 && dot[i]; i++)
		ext_out[i] = (char)tolower((unsigned char)dot[i]);
	ext_out[i] = '\0';
}

/* ── Simple linear resampler ─────────────────────────────────────── */
/*
 * Resample src (src_rate Hz, stereo int16) → af_get_target_rate() Hz stereo int16.
 * Returns newly malloc'd buffer in *dst and frame count in *dst_frames.
 * Returns 0 on success.
 */
static int resample_to_target(const int16_t *src, uint32_t src_frames,
			      uint32_t src_rate, int16_t **dst,
			      uint32_t *dst_frames)
{
	if (src_rate == AF_TARGET_RATE) {
		/* No conversion needed — just copy.
         * FIX: cast to size_t BEFORE multiplying to prevent uint32_t
         * overflow on 32-bit targets (e.g. PowerPC G4). */
		size_t bytes = (size_t)src_frames * 2 * sizeof(int16_t);
		*dst = (int16_t *)malloc(bytes);
		if (!*dst)
			return AF_ERR_MEMORY;
		memcpy(*dst, src, bytes);
		*dst_frames = src_frames;
		return AF_OK;
	}

	double ratio = (double)AF_TARGET_RATE / (double)src_rate;
	/* Compute output frame count in double first to avoid uint32_t overflow
     * when ratio > 1.0 (upsampling).  Cap at SIZE_MAX/4 so the malloc
     * size calculation below cannot overflow size_t. */
	double out_frames_d = (double)src_frames * ratio + 0.5;
	if (out_frames_d > (double)(SIZE_MAX / (2 * sizeof(int16_t)))) {
		return AF_ERR_MEMORY; /* output would be too large for this platform */
	}
	uint32_t out_frames = (uint32_t)out_frames_d;
	/* FIX: (size_t) cast prevents uint32_t overflow in malloc argument */
	*dst = (int16_t *)malloc((size_t)out_frames * 2 * sizeof(int16_t));
	if (!*dst)
		return AF_ERR_MEMORY;

	for (uint32_t i = 0; i < out_frames; i++) {
		double src_pos = (double)i / ratio;
		/* FIX: use size_t for sample index arithmetic so idx*2 cannot
         * overflow when idx approaches 2^31 on 32-bit platforms. */
		size_t idx = (size_t)src_pos;
		float frac = (float)(src_pos - (double)idx);
		size_t next = (idx + 1 < (size_t)src_frames) ? idx + 1 : idx;

		float l = (float)src[idx * 2] * (1.0f - frac) +
			  (float)src[next * 2] * frac;
		float r = (float)src[idx * 2 + 1] * (1.0f - frac) +
			  (float)src[next * 2 + 1] * frac;

		/* clamp */
		if (l > 32767.0f)
			l = 32767.0f;
		if (l < -32768.0f)
			l = -32768.0f;
		if (r > 32767.0f)
			r = 32767.0f;
		if (r < -32768.0f)
			r = -32768.0f;

		(*dst)[i * 2] = (int16_t)l;
		(*dst)[i * 2 + 1] = (int16_t)r;
	}
	*dst_frames = out_frames;
	return AF_OK;
}

/* ── Mono → stereo duplication ───────────────────────────────────── */
static int16_t *mono_to_stereo(const int16_t *mono, uint32_t frames)
{
	/* FIX: (size_t) cast prevents uint32_t overflow on 32-bit targets. */
	int16_t *stereo =
		(int16_t *)malloc((size_t)frames * 2 * sizeof(int16_t));
	if (!stereo)
		return NULL;
	for (uint32_t i = 0; i < frames; i++) {
		stereo[i * 2] = mono[i];
		stereo[i * 2 + 1] = mono[i];
	}
	return stereo;
}

/* ─────────────────────────────────────────────────────────────────
   WAV loader (no external library needed)
   ───────────────────────────────────────────────────────────────── */
static int load_wav(const char *path, AFBuffer *out)
{
	FILE *f = fopen(path, "rb");
	if (!f)
		return AF_ERR_OPEN;

	/* Read full RIFF header */
	uint8_t hdr[12];
	if (fread(hdr, 1, 12, f) != 12 || memcmp(hdr, "RIFF", 4) ||
	    memcmp(hdr + 8, "WAVE", 4)) {
		fclose(f);
		return AF_ERR_FORMAT;
	}

	/* Parse chunks */
	uint16_t audio_fmt = 0, num_ch = 0, bit_depth = 0;
	uint32_t sample_rate = 0, data_size = 0;
	long data_offset = 0;

	uint8_t chunk_hdr[8];
	while (fread(chunk_hdr, 1, 8, f) == 8) {
		uint32_t chunk_size = (uint32_t)(chunk_hdr[4]) |
				      ((uint32_t)(chunk_hdr[5]) << 8) |
				      ((uint32_t)(chunk_hdr[6]) << 16) |
				      ((uint32_t)(chunk_hdr[7]) << 24);

		if (memcmp(chunk_hdr, "fmt ", 4) == 0) {
			uint8_t fmt[16];
			uint32_t to_read = chunk_size < 16 ? chunk_size : 16;
			fread(fmt, 1, to_read, f);
			if (chunk_size > 16)
				fseek(f, chunk_size - 16, SEEK_CUR);

			audio_fmt = fmt[0] | (uint16_t)(fmt[1] << 8);
			num_ch = fmt[2] | (uint16_t)(fmt[3] << 8);
			sample_rate = fmt[4] | ((uint32_t)fmt[5] << 8) |
				      ((uint32_t)fmt[6] << 16) |
				      ((uint32_t)fmt[7] << 24);
			bit_depth = fmt[14] | (uint16_t)(fmt[15] << 8);

		} else if (memcmp(chunk_hdr, "data", 4) == 0) {
			data_size = chunk_size;
			data_offset = ftell(f);
			break;
		} else {
			/* skip unknown chunk (pad to even size per spec) */
			fseek(f, (chunk_size + 1) & ~1U, SEEK_CUR);
		}
	}

	if (!data_size || !sample_rate) {
		fclose(f);
		return AF_ERR_FORMAT;
	}
	/* Only PCM (1) or IEEE float (3) */
	if (audio_fmt != 1 && audio_fmt != 3) {
		fclose(f);
		return AF_ERR_FORMAT;
	}
	if (num_ch < 1 || num_ch > 2) {
		fclose(f);
		return AF_ERR_FORMAT;
	}
	if (bit_depth != 8 && bit_depth != 16 && bit_depth != 24 &&
	    bit_depth != 32) {
		fclose(f);
		return AF_ERR_FORMAT;
	}

	/* Read raw data */
	fseek(f, data_offset, SEEK_SET);
	uint8_t *raw = (uint8_t *)malloc(data_size);
	if (!raw) {
		fclose(f);
		return AF_ERR_MEMORY;
	}
	fread(raw, 1, data_size, f);
	fclose(f);

	uint32_t bytes_per_sample = bit_depth / 8;
	uint32_t total_samples = data_size / bytes_per_sample;
	uint32_t src_frames = total_samples / num_ch;

	/* Convert everything to int16_t stereo.
     * FIX: (size_t) cast prevents uint32_t overflow on 32-bit targets. */
	int16_t *pcm =
		(int16_t *)malloc((size_t)src_frames * 2 * sizeof(int16_t));
	if (!pcm) {
		free(raw);
		return AF_ERR_MEMORY;
	}

	for (uint32_t i = 0; i < src_frames; i++) {
		int16_t ch[2] = { 0, 0 };
		for (int c = 0; c < (int)num_ch && c < 2; c++) {
			uint32_t byte_off =
				(i * num_ch + (uint32_t)c) * bytes_per_sample;
			if (bit_depth == 8) {
				/* 8-bit WAV is unsigned */
				ch[c] = (int16_t)(((int)raw[byte_off] - 128)
						  << 8);
			} else if (bit_depth == 16) {
				ch[c] = (int16_t)(raw[byte_off] |
						  (raw[byte_off + 1] << 8));
			} else if (bit_depth == 24) {
				int32_t s =
					(int32_t)((raw[byte_off + 2] << 24) |
						  (raw[byte_off + 1] << 16) |
						  (raw[byte_off] << 8)) >>
					8;
				ch[c] = (int16_t)(s >> 8);
			} else if (bit_depth == 32) {
				if (audio_fmt == 3) {
					/* 32-bit float — WAV is always little-endian,
                       swap bytes on big-endian hosts (e.g. PowerPC) */
					uint8_t b[4] = { raw[byte_off + 3],
							 raw[byte_off + 2],
							 raw[byte_off + 1],
							 raw[byte_off + 0] };
					float fv;
					memcpy(&fv, b, 4);
					int32_t iv = (int32_t)(fv * 32767.0f);
					if (iv > 32767)
						iv = 32767;
					if (iv < -32768)
						iv = -32768;
					ch[c] = (int16_t)iv;
				} else {
					/* 32-bit PCM — little-endian bytes, read explicitly */
					int32_t s =
						(int32_t)((uint32_t)
								  raw[byte_off +
								      0] |
							  ((uint32_t)raw
								   [byte_off +
								    1]
							   << 8) |
							  ((uint32_t)raw
								   [byte_off +
								    2]
							   << 16) |
							  ((uint32_t)raw
								   [byte_off +
								    3]
							   << 24));
					ch[c] = (int16_t)(s >> 16);
				}
			}
		}
		/* Duplicate mono to stereo */
		pcm[i * 2] = ch[0];
		pcm[i * 2 + 1] = (num_ch == 2) ? ch[1] : ch[0];
	}
	free(raw);

	/* Resample if needed */
	int16_t *resampled = NULL;
	uint32_t resampled_frames = 0;
	int rc = resample_to_target(pcm, src_frames, sample_rate, &resampled,
				    &resampled_frames);
	free(pcm);
	if (rc != AF_OK)
		return rc;

	out->samples = resampled;
	out->num_frames = resampled_frames;
	out->sample_rate = AF_TARGET_RATE;
	out->channels = 2;
	return AF_OK;
}

/* ─────────────────────────────────────────────────────────────────
   MP3 loader  (minimp3)
   ───────────────────────────────────────────────────────────────── */
static int load_mp3(const char *path, AFBuffer *out)
{
	/* Read whole file into memory */
	FILE *f = fopen(path, "rb");
	if (!f)
		return AF_ERR_OPEN;

	fseek(f, 0, SEEK_END);
	long file_size = ftell(f);
	rewind(f);
	if (file_size <= 0) {
		fclose(f);
		return AF_ERR_FORMAT;
	}

	uint8_t *file_buf = (uint8_t *)malloc((size_t)file_size);
	if (!file_buf) {
		fclose(f);
		return AF_ERR_MEMORY;
	}
	fread(file_buf, 1, (size_t)file_size, f);
	fclose(f);

	/* Decode all frames */
	mp3dec_t dec;
	mp3dec_init(&dec);

	/* First pass: count total samples */
	mp3dec_frame_info_t info;
	int16_t frame_buf[MINIMP3_MAX_SAMPLES_PER_FRAME];
	size_t offset = 0;
	uint32_t total_pcm = 0;
	uint32_t src_rate = 0;
	int src_ch = 0;

	/* We'll collect chunks in a growing buffer */
	uint32_t cap = af_target_rate * 2 * 3; /* pre-alloc ~3 sec stereo */
	int16_t *pcm_buf = (int16_t *)malloc(cap * sizeof(int16_t));
	if (!pcm_buf) {
		free(file_buf);
		return AF_ERR_MEMORY;
	}

	while (offset < (size_t)file_size) {
		int samples = mp3dec_decode_frame(
			&dec, file_buf + offset,
			(int)(file_size - (long)offset), frame_buf, &info);
		if (info.frame_bytes == 0)
			break; /* end / bad data */
		offset += (size_t)info.frame_bytes;
		if (samples == 0)
			continue; /* ID3 / padding */

		if (src_rate == 0) {
			src_rate = (uint32_t)info.hz;
			src_ch = info.channels;
		}

		uint32_t new_samples = (uint32_t)(samples * info.channels);
		/* Grow buffer if needed */
		if (total_pcm + new_samples > cap) {
			cap = (cap + new_samples) * 2;
			int16_t *tmp = (int16_t *)realloc(
				pcm_buf, cap * sizeof(int16_t));
			if (!tmp) {
				free(pcm_buf);
				free(file_buf);
				return AF_ERR_MEMORY;
			}
			pcm_buf = tmp;
		}
		memcpy(pcm_buf + total_pcm, frame_buf,
		       new_samples * sizeof(int16_t));
		total_pcm += new_samples;
	}
	free(file_buf);

	if (total_pcm == 0 || src_rate == 0) {
		free(pcm_buf);
		return AF_ERR_DECODE;
	}

	uint32_t src_frames = total_pcm / (uint32_t)src_ch;

	/* Convert mono → stereo if needed */
	int16_t *stereo_pcm = pcm_buf;
	uint32_t stereo_frames = src_frames;
	int own_stereo = 0;
	if (src_ch == 1) {
		stereo_pcm = mono_to_stereo(pcm_buf, src_frames);
		free(pcm_buf);
		if (!stereo_pcm)
			return AF_ERR_MEMORY;
		own_stereo = 1;
		stereo_frames = src_frames;
	}

	/* Resample to target rate if needed */
	int16_t *resampled = NULL;
	uint32_t resampled_frames = 0;
	int rc = resample_to_target(stereo_pcm, stereo_frames, src_rate,
				    &resampled, &resampled_frames);
	if (own_stereo || src_ch != 1)
		free(stereo_pcm);
	else
		free(pcm_buf);
	if (rc != AF_OK)
		return rc;

	out->samples = resampled;
	out->num_frames = resampled_frames;
	out->sample_rate = AF_TARGET_RATE;
	out->channels = 2;
	return AF_OK;
}

/* ─────────────────────────────────────────────────────────────────
   FLAC loader  (dr_flac)
   ───────────────────────────────────────────────────────────────── */
static int load_flac(const char *path, AFBuffer *out)
{
	drflac *flac = drflac_open_file(path, NULL);
	if (!flac)
		return AF_ERR_OPEN;

	uint32_t src_rate = flac->sampleRate;
	uint32_t src_ch = flac->channels;
	uint64_t total_frames = flac->totalPCMFrameCount;

	if (src_ch < 1 || src_ch > 2 || src_rate == 0) {
		drflac_close(flac);
		return AF_ERR_FORMAT;
	}

	/* Handle streaming FLACs where totalPCMFrameCount == 0.
     * Some encoders omit or zero the STREAMINFO sample count.
     * In that case, read in 10-second chunks until EOF. */
	if (total_frames == 0) {
		uint32_t chunk = src_rate * 10; /* 10-second chunks */
		uint64_t cap = (uint64_t)chunk * src_ch;
		int16_t *grow =
			(int16_t *)malloc((size_t)cap * sizeof(int16_t));
		if (!grow) {
			drflac_close(flac);
			return AF_ERR_MEMORY;
		}
		uint64_t filled = 0;
		for (;;) {
			if (filled + (uint64_t)cap >
			    (uint64_t)(SIZE_MAX / sizeof(int16_t)))
				break;
			uint64_t read = drflac_read_pcm_frames_s16(
				flac, chunk, grow + filled);
			if (read == 0)
				break;
			filled += read * src_ch;
			if (read < chunk)
				break;
			/* Grow buffer for next chunk */
			uint64_t new_cap = cap + (uint64_t)chunk * src_ch;
			if (new_cap > (uint64_t)(SIZE_MAX / sizeof(int16_t)))
				break;
			int16_t *tmp = (int16_t *)realloc(
				grow, (size_t)new_cap * sizeof(int16_t));
			if (!tmp)
				break;
			grow = tmp;
			cap = new_cap;
		}
		drflac_close(flac);
		if (filled == 0) {
			free(grow);
			return AF_ERR_DECODE;
		}
		total_frames = filled / src_ch;
		/* Fall through with grow as pcm — same path as normal decode below,
         * but we skip the second drflac_open/decode; inject directly. */
		uint32_t sf2 = (uint32_t)total_frames;
		int16_t *stereo2 =
			(src_ch == 1) ? mono_to_stereo(grow, sf2) : grow;
		if (src_ch == 1)
			free(grow);
		if (!stereo2)
			return AF_ERR_MEMORY;
		int16_t *rs2 = NULL;
		uint32_t rsf2 = 0;
		int rc2 =
			resample_to_target(stereo2, sf2, src_rate, &rs2, &rsf2);
		free(stereo2);
		if (rc2 != AF_OK)
			return rc2;
		out->samples = rs2;
		out->num_frames = rsf2;
		out->sample_rate = AF_TARGET_RATE;
		out->channels = 2;
		return AF_OK;
	}

	/*
     * BUG FIX: On 32-bit PowerPC, uint64_t * uint32_t arithmetic in the
     * malloc() call was silently truncating to 32 bits for files larger
     * than ~46 minutes (stereo) or ~92 minutes (mono), producing a far-too-
     * small allocation.  drflac_read_pcm_frames_s16 then wrote past the end
     * of the buffer → SIGSEGV / core dump.
     *
     * Guard: reject files whose decoded size would exceed SIZE_MAX so the
     * cast to size_t below is always safe.
     */
	uint64_t total_samples = total_frames * (uint64_t)src_ch;
	/* Guard 1: input buffer must fit in memory */
	if (total_samples > (uint64_t)(SIZE_MAX / sizeof(int16_t))) {
		drflac_close(flac);
		return AF_ERR_MEMORY;
	}
	/* Guard 2: after upsampling to target rate, output must also fit.
     * Compute worst-case output size using the target rate ratio. */
	{
		uint32_t tgt = af_get_target_rate();
		if (src_rate > 0 && tgt > src_rate) {
			double upsample_ratio = (double)tgt / (double)src_rate;
			double out_samples =
				(double)total_frames * upsample_ratio * 2.0;
			if (out_samples >
			    (double)(SIZE_MAX / sizeof(int16_t))) {
				drflac_close(flac);
				return AF_ERR_MEMORY;
			}
		}
	}

	int16_t *pcm =
		(int16_t *)malloc((size_t)total_samples * sizeof(int16_t));
	if (!pcm) {
		drflac_close(flac);
		return AF_ERR_MEMORY;
	}

	uint64_t decoded = drflac_read_pcm_frames_s16(flac, total_frames, pcm);
	drflac_close(flac);

	if (decoded == 0) {
		free(pcm);
		return AF_ERR_DECODE;
	}

	/*
     * BUG FIX: use the actual decoded frame count (not total_frames) for all
     * subsequent processing so a truncated/corrupt file doesn't cause us to
     * pass stale memory to mono_to_stereo / resample_to_44100.
     */
	uint32_t src_frames = (uint32_t)
		decoded; /* decoded <= total_frames <= UINT32_MAX after guard above */

	/* Mono → stereo */
	int16_t *stereo_pcm = pcm;
	uint32_t stereo_frames = src_frames;
	if (src_ch == 1) {
		stereo_pcm = mono_to_stereo(pcm, src_frames);
		free(pcm);
		if (!stereo_pcm)
			return AF_ERR_MEMORY;
		stereo_frames = src_frames;
	}

	/* Resample */
	int16_t *resampled = NULL;
	uint32_t resampled_frames = 0;
	int rc = resample_to_target(stereo_pcm, stereo_frames, src_rate,
				    &resampled, &resampled_frames);
	/*
     * BUG FIX: stereo_pcm was not freed on the resample-error path,
     * leaking the entire decoded buffer.  Free unconditionally here;
     * resample_to_44100 never aliases its input.
     */
	free(stereo_pcm);
	if (rc != AF_OK)
		return rc;

	out->samples = resampled;
	out->num_frames = resampled_frames;
	out->sample_rate = AF_TARGET_RATE;
	out->channels = 2;
	return AF_OK;
}

/* ─────────────────────────────────────────────────────────────────
   Public API — af_load / af_free
   ───────────────────────────────────────────────────────────────── */
int af_load(const char *path, AFBuffer *out)
{
	if (!path || !out)
		return AF_ERR_FORMAT;
	memset(out, 0, sizeof(*out));

	char ext[8];
	get_ext_lower(path, ext, sizeof(ext));

	if (strcmp(ext, "wav") == 0)
		return load_wav(path, out);
	if (strcmp(ext, "mp3") == 0)
		return load_mp3(path, out);
	if (strcmp(ext, "flac") == 0)
		return load_flac(path, out);

	return AF_ERR_FORMAT;
}

void af_free(AFBuffer *buf)
{
	if (!buf)
		return;
	free(buf->samples);
	buf->samples = NULL;
	buf->num_frames = 0;
}

/* ─────────────────────────────────────────────────────────────────
   af_is_supported()
   ───────────────────────────────────────────────────────────────── */
int af_is_supported(const char *filename)
{
	char ext[8];
	get_ext_lower(filename, ext, sizeof(ext));
	return (strcmp(ext, "wav") == 0 || strcmp(ext, "mp3") == 0 ||
		strcmp(ext, "flac") == 0);
}

/* ─────────────────────────────────────────────────────────────────
   Directory scanner
   ───────────────────────────────────────────────────────────────── */
static int scan_append(AFScanResult *r, const char *dir, const char *name)
{
	/* Grow array if needed */
	if (r->count >= r->capacity) {
		int new_cap = r->capacity ? r->capacity * 2 : 64;
		AFScanEntry *tmp = (AFScanEntry *)realloc(
			r->entries, (size_t)new_cap * sizeof(AFScanEntry));
		if (!tmp)
			return AF_ERR_MEMORY;
		r->entries = tmp;
		r->capacity = new_cap;
	}

	AFScanEntry *e = &r->entries[r->count];

	/* Build full path */
	size_t dlen = strlen(dir);
	size_t nlen = strlen(name);
	e->path = (char *)malloc(dlen + 1 + nlen + 1);
	if (!e->path)
		return AF_ERR_MEMORY;
	memcpy(e->path, dir, dlen);
	e->path[dlen] = '/';
	memcpy(e->path + dlen + 1, name, nlen + 1);

	/* basename pointer into path */
	e->basename = e->path + dlen + 1;

	/* extension */
	get_ext_lower(name, e->ext, sizeof(e->ext));

	r->count++;
	return AF_OK;
}

int af_scan_dir(const char *dir_path, AFScanResult *result, int max_files)
{
	if (!dir_path || !result)
		return -1;
	memset(result, 0, sizeof(*result));

	DIR *d = opendir(dir_path);
	if (!d)
		return -1;

	struct dirent *ent;
	int found = 0;
	while ((ent = readdir(d))) {
		if (ent->d_name[0] == '.')
			continue;
		if (!af_is_supported(ent->d_name))
			continue;
		if (max_files > 0 && found >= max_files)
			break;
		if (scan_append(result, dir_path, ent->d_name) == AF_OK)
			found++;
	}
	closedir(d);
	return found;
}

int af_scan_dir_recursive(const char *dir_path, AFScanResult *result,
			  int max_files, int max_depth)
{
	if (!dir_path || !result)
		return -1;
	if (result->entries == NULL)
		memset(result, 0, sizeof(*result));
	if (max_depth < 0)
		return result->count;

	DIR *d = opendir(dir_path);
	if (!d)
		return result->count;

	struct dirent *ent;
	while ((ent = readdir(d))) {
		if (ent->d_name[0] == '.')
			continue;
		if (max_files > 0 && result->count >= max_files)
			break;

		/* Check if directory */
		char subpath[4096];
		snprintf(subpath, sizeof(subpath), "%s/%s", dir_path,
			 ent->d_name);
		struct stat st;
		if (stat(subpath, &st) != 0)
			continue;

		if (S_ISDIR(st.st_mode)) {
			af_scan_dir_recursive(subpath, result, max_files,
					      max_depth - 1);
		} else if (S_ISREG(st.st_mode) &&
			   af_is_supported(ent->d_name)) {
			scan_append(result, dir_path, ent->d_name);
		}
	}
	closedir(d);
	return result->count;
}

void af_scan_free(AFScanResult *result)
{
	if (!result)
		return;
	for (int i = 0; i < result->count; i++)
		free(result->entries[i].path);
	free(result->entries);
	result->entries = NULL;
	result->count = 0;
	result->capacity = 0;
}

/* ─────────────────────────────────────────────────────────────────
   Sorting helpers
   ───────────────────────────────────────────────────────────────── */
static int cmp_alpha(const void *a, const void *b)
{
	const AFScanEntry *ea = (const AFScanEntry *)a;
	const AFScanEntry *eb = (const AFScanEntry *)b;
	return strcasecmp(ea->basename, eb->basename);
}

static int ext_rank(const char *ext)
{
	if (strcmp(ext, "flac") == 0)
		return 0;
	if (strcmp(ext, "wav") == 0)
		return 1;
	if (strcmp(ext, "mp3") == 0)
		return 2;
	return 3;
}

static int cmp_ext(const void *a, const void *b)
{
	const AFScanEntry *ea = (const AFScanEntry *)a;
	const AFScanEntry *eb = (const AFScanEntry *)b;
	int ra = ext_rank(ea->ext), rb = ext_rank(eb->ext);
	if (ra != rb)
		return ra - rb;
	return strcasecmp(ea->basename, eb->basename);
}

void af_scan_sort_alpha(AFScanResult *result)
{
	if (!result || result->count < 2)
		return;
	qsort(result->entries, (size_t)result->count, sizeof(AFScanEntry),
	      cmp_alpha);
}

void af_scan_sort_ext(AFScanResult *result)
{
	if (!result || result->count < 2)
		return;
	qsort(result->entries, (size_t)result->count, sizeof(AFScanEntry),
	      cmp_ext);
}

/* ─────────────────────────────────────────────────────────────────
   af_format_name()
   ───────────────────────────────────────────────────────────────── */
const char *af_format_name(const char *ext)
{
	if (!ext)
		return "???";
	if (strcmp(ext, "mp3") == 0)
		return "MP3";
	if (strcmp(ext, "wav") == 0)
		return "WAV";
	if (strcmp(ext, "flac") == 0)
		return "FLAC";
	return "???";
}
