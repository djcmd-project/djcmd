/* audio_win32.h — PortAudio wrapper that mimics the ALSA PCM interface
 * used by djcmd's audio thread.
 *
 * djcmd's audio path (simplified):
 *
 *   snd_pcm_open()       → pa_open()
 *   snd_pcm_hw_params_*  → pa_configure()   (sample rate, channels, period)
 *   snd_pcm_writei()     → pa_write()        (blocking, like ALSA)
 *   snd_pcm_close()      → pa_close()
 *
 * Compile with:  -lportaudio
 */

#pragma once
#ifndef DJCMD_AUDIO_WIN32_H
#define DJCMD_AUDIO_WIN32_H

#ifdef _WIN32
#include <portaudio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* ── Internal state ──────────────────────────────────────────────────── */
typedef struct {
	PaStream *stream;
	unsigned int sample_rate;
	unsigned int channels;
	unsigned int period_frames;
	int16_t *ring;
	volatile int ring_write;
	volatile int ring_read;
	int ring_frames;
} DJPCMState;

static DJPCMState g_pa_main = { 0 };
static DJPCMState g_pa_hp = { 0 };

/* PortAudio callback — runs on PA audio thread, drains ring buffer */
static int pa_callback(const void *in, void *out, unsigned long frames_per_buf,
		       const PaStreamCallbackTimeInfo *ti,
		       PaStreamCallbackFlags flags, void *user)
{
	(void)in; (void)ti; (void)flags;
	DJPCMState *s = (DJPCMState *)user;
	int16_t *dst = (int16_t *)out;
	unsigned long f;

	for (f = 0; f < frames_per_buf; f++) {
		int avail = s->ring_write - s->ring_read;
		if (avail <= 0) {
			*dst++ = 0;
			if (s->channels > 1) *dst++ = 0;
			continue;
		}
		int ri = (s->ring_read % s->ring_frames) * s->channels;
		*dst++ = s->ring[ri];
		if (s->channels > 1) *dst++ = s->ring[ri + 1];
		s->ring_read++;
	}
	return paContinue;
}

/* Open and start PortAudio stream. */
static inline int pa_open_stream(DJPCMState *s, int device_idx, unsigned int sample_rate, 
                                unsigned int channels, unsigned int period_frames)
{
	PaError err;
	s->sample_rate = sample_rate;
	s->channels = channels;
	s->period_frames = period_frames;
	s->ring_frames = period_frames * 8;
	s->ring = (int16_t *)calloc((size_t)s->ring_frames * channels, sizeof(int16_t));
	if (!s->ring) return -1;

	static int pa_inited = 0;
	if (!pa_inited) {
		err = Pa_Initialize();
		if (err != paNoError) return -1;
		pa_inited = 1;
	}

	PaStreamParameters op = { 0 };
	op.device = (device_idx >= 0) ? device_idx : Pa_GetDefaultOutputDevice();
	if (op.device == paNoDevice) return -1;
	
	op.channelCount = (int)channels;
	op.sampleFormat = paInt16;
	op.suggestedLatency = Pa_GetDeviceInfo(op.device)->defaultLowOutputLatency;
	op.hostApiSpecificStreamInfo = NULL;

	err = Pa_OpenStream(&s->stream, NULL, &op, (double)sample_rate,
			    period_frames, paClipOff, pa_callback, s);
	if (err != paNoError) return -1;

	err = Pa_StartStream(s->stream);
	return (err == paNoError) ? 0 : -1;
}

/* Blocking write — mirrors snd_pcm_writei(). */
static inline ssize_t pa_write_stream(DJPCMState *s, const int16_t *buf, unsigned int frames)
{
	if (!s->stream) return 0;
	unsigned int f;
	for (f = 0; f < frames; f++) {
		while ((s->ring_write - s->ring_read) >= s->ring_frames)
			Sleep(1);
		int wi = (s->ring_write % s->ring_frames) * s->channels;
		s->ring[wi] = buf[f * s->channels];
		if (s->channels > 1)
			s->ring[wi + 1] = buf[f * s->channels + 1];
		s->ring_write++;
	}
	return (ssize_t)frames;
}

static inline void pa_close_stream(DJPCMState *s)
{
	if (s->stream) {
		Pa_StopStream(s->stream);
		Pa_CloseStream(s->stream);
		s->stream = NULL;
	}
	if (s->ring) {
		free(s->ring);
		s->ring = NULL;
	}
}

static inline void pa_shutdown(void) {
	pa_close_stream(&g_pa_main);
	pa_close_stream(&g_pa_hp);
	Pa_Terminate();
}

/* ── Macro shims — redirect ALSA calls to PortAudio wrappers ─────────── *
 * In djcmd.c, guard the ALSA audio-open section with:
 *   #ifndef _WIN32
 *     ... existing ALSA code ...
 *   #else
 *     pa_open(sample_rate, channels, period_frames);
 *   #endif
 * And the write loop:
 *   #ifndef _WIN32
 *     snd_pcm_writei(g_pcm, buf, frames);
 *   #else
 *     pa_write(buf, frames);
 *   #endif
 */

#endif /* _WIN32 */
#endif /* DJCMD_AUDIO_WIN32_H */
