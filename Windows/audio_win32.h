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
	unsigned int period_frames; /* frames per write() call */
	int16_t *ring; /* write-ahead ring buffer  */
	volatile int ring_write; /* write index (in frames)  */
	volatile int ring_read; /* read  index              */
	int ring_frames; /* total ring capacity      */
} DJPCMState;

static DJPCMState g_pa_state = { 0 };

/* PortAudio callback — runs on PA audio thread, drains ring buffer */
static int pa_callback(const void *in, void *out, unsigned long frames_per_buf,
		       const PaStreamCallbackTimeInfo *ti,
		       PaStreamCallbackFlags flags, void *user)
{
	(void)in;
	(void)ti;
	(void)flags;
	(void)user;
	DJPCMState *s = &g_pa_state;
	int16_t *dst = (int16_t *)out;
	unsigned long f;

	for (f = 0; f < frames_per_buf; f++) {
		int avail = s->ring_write - s->ring_read;
		if (avail <= 0) {
			/* Underrun: fill with silence */
			*dst++ = 0;
			if (s->channels > 1)
				*dst++ = 0;
			continue;
		}
		int ri = (s->ring_read % s->ring_frames) * s->channels;
		*dst++ = s->ring[ri];
		if (s->channels > 1)
			*dst++ = s->ring[ri + 1];
		s->ring_read++;
	}
	return paContinue;
}

/* Open and start PortAudio stream.
 * Call before the audio thread starts writing. */
static inline int pa_open(unsigned int sample_rate, unsigned int channels,
			  unsigned int period_frames)
{
	PaError err;
	g_pa_state.sample_rate = sample_rate;
	g_pa_state.channels = channels;
	g_pa_state.period_frames = period_frames;
	g_pa_state.ring_frames = period_frames * 8;
	g_pa_state.ring = (int16_t *)calloc(
		(size_t)g_pa_state.ring_frames * channels, sizeof(int16_t));
	if (!g_pa_state.ring)
		return -1;

	err = Pa_Initialize();
	if (err != paNoError)
		return -1;

	PaStreamParameters op = { 0 };
	op.device = Pa_GetDefaultOutputDevice();
	op.channelCount = (int)channels;
	op.sampleFormat = paInt16;
	op.suggestedLatency =
		Pa_GetDeviceInfo(op.device)->defaultLowOutputLatency;
	op.hostApiSpecificStreamInfo = NULL;

	err = Pa_OpenStream(&g_pa_state.stream, NULL, &op, (double)sample_rate,
			    period_frames, paClipOff, pa_callback, NULL);
	if (err != paNoError)
		return -1;

	err = Pa_StartStream(g_pa_state.stream);
	return (err == paNoError) ? 0 : -1;
}

/* Blocking write — mirrors snd_pcm_writei(). */
static inline ssize_t pa_write(const int16_t *buf, unsigned int frames)
{
	DJPCMState *s = &g_pa_state;
	unsigned int f;
	for (f = 0; f < frames; f++) {
		/* Spin until ring has space */
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

static inline void pa_close(void)
{
	if (g_pa_state.stream) {
		Pa_StopStream(g_pa_state.stream);
		Pa_CloseStream(g_pa_state.stream);
		Pa_Terminate();
		g_pa_state.stream = NULL;
	}
	free(g_pa_state.ring);
	g_pa_state.ring = NULL;
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
