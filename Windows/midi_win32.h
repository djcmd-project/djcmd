/* midi_win32.h — WinMM MIDI wrapper that mirrors djcmd's ALSA rawmidi API.
 *
 * djcmd's MIDI path uses:
 *   snd_rawmidi_open()   → midi_win32_open()
 *   snd_rawmidi_read()   → midi_win32_read()   (blocking, byte at a time)
 *   snd_rawmidi_write()  → midi_win32_write()
 *   snd_rawmidi_close()  → midi_win32_close()
 *
 * WinMM delivers MIDI input via callback into a FIFO; the djcmd reader
 * thread blocks on a semaphore until bytes are available.
 *
 * Compile with: -lwinmm
 */

#pragma once
#ifndef DJCMD_MIDI_WIN32_H
#define DJCMD_MIDI_WIN32_H

#ifdef _WIN32
#include <windows.h>
#include <mmsystem.h>
#include <stdint.h>
#include <string.h>

#define MIDI_FIFO_SIZE 4096

typedef struct {
	HMIDIIN in;
	HMIDIOUT out;
	uint8_t fifo[MIDI_FIFO_SIZE];
	volatile int fifo_r, fifo_w;
	HANDLE sem; /* signalled by callback when data arrives */
	int open;
} MidiWin32State;

static MidiWin32State g_midi_win32 = { 0 };

/* WinMM input callback — runs in WinMM thread */
static void CALLBACK midi_win32_in_cb(HMIDIIN hi, UINT msg, DWORD_PTR inst,
				      DWORD_PTR p1, DWORD_PTR p2)
{
	(void)hi;
	(void)inst;
	(void)p2;
	if (msg != MIM_DATA)
		return;
	MidiWin32State *s = &g_midi_win32;
	/* p1 = packed short message: status | (d1<<8) | (d2<<16) */
	uint8_t b0 = (uint8_t)(p1 & 0xFF);
	uint8_t b1 = (uint8_t)((p1 >> 8) & 0xFF);
	uint8_t b2 = (uint8_t)((p1 >> 16) & 0xFF);
	/* Push bytes (may drop on overflow — rare at normal MIDI rates) */
	int next;
	next = (s->fifo_w + 1) % MIDI_FIFO_SIZE;
	if (next != s->fifo_r) {
		s->fifo[s->fifo_w] = b0;
		s->fifo_w = next;
	}
	if (b0 < 0xF0) { /* not sysex: write d1, d2 */
		next = (s->fifo_w + 1) % MIDI_FIFO_SIZE;
		if (next != s->fifo_r) {
			s->fifo[s->fifo_w] = b1;
			s->fifo_w = next;
		}
		if ((b0 & 0xE0) != 0xC0) { /* not prog change / chan pressure */
			next = (s->fifo_w + 1) % MIDI_FIFO_SIZE;
			if (next != s->fifo_r) {
				s->fifo[s->fifo_w] = b2;
				s->fifo_w = next;
			}
		}
	}
	ReleaseSemaphore(s->sem, 1, NULL);
}

/* Open MIDI input (device index) and output (device index).
 * Pass -1 to skip input or output. */
static inline int midi_win32_open(int in_dev, int out_dev)
{
	MidiWin32State *s = &g_midi_win32;
	s->sem = CreateSemaphore(NULL, 0, MIDI_FIFO_SIZE, NULL);
	if (!s->sem)
		return -1;

	if (in_dev >= 0) {
		if (midiInOpen(&s->in, (UINT)in_dev,
			       (DWORD_PTR)midi_win32_in_cb, 0,
			       CALLBACK_FUNCTION) != MMSYSERR_NOERROR)
			return -1;
		midiInStart(s->in);
	}
	if (out_dev >= 0) {
		if (midiOutOpen(&s->out, (UINT)out_dev, 0, 0, CALLBACK_NULL) !=
		    MMSYSERR_NOERROR)
			return -1;
	}
	s->open = 1;
	return 0;
}

/* Blocking read — blocks until at least one byte is in the FIFO. */
static inline ssize_t midi_win32_read(uint8_t *buf, size_t n)
{
	MidiWin32State *s = &g_midi_win32;
	WaitForSingleObject(s->sem, INFINITE);
	size_t got = 0;
	while (got < n && s->fifo_r != s->fifo_w) {
		buf[got++] = s->fifo[s->fifo_r];
		s->fifo_r = (s->fifo_r + 1) % MIDI_FIFO_SIZE;
	}
	return (ssize_t)got;
}

/* Send short MIDI message (status, d1, d2).
 * For Note Off / Note On / CC / PB — covers all djcmd output messages. */
static inline void midi_win32_send(uint8_t s, uint8_t d1, uint8_t d2)
{
	if (!g_midi_win32.out)
		return;
	midiOutShortMsg(g_midi_win32.out,
			(DWORD)s | ((DWORD)d1 << 8) | ((DWORD)d2 << 16));
}

/* Write raw bytes (for sysex) */
static inline ssize_t midi_win32_write(const uint8_t *buf, size_t n)
{
	if (!g_midi_win32.out || n < 1)
		return 0;
	if (buf[0] == 0xF0) {
		/* SysEx */
		MIDIHDR hdr = { 0 };
		hdr.lpData = (LPSTR)buf;
		hdr.dwBufferLength = (DWORD)n;
		hdr.dwFlags = 0;
		midiOutPrepareHeader(g_midi_win32.out, &hdr, sizeof(hdr));
		midiOutLongMsg(g_midi_win32.out, &hdr, sizeof(hdr));
		midiOutUnprepareHeader(g_midi_win32.out, &hdr, sizeof(hdr));
	} else if (n >= 1) {
		uint8_t d1 = (n > 1) ? buf[1] : 0;
		uint8_t d2 = (n > 2) ? buf[2] : 0;
		midi_win32_send(buf[0], d1, d2);
	}
	return (ssize_t)n;
}

static inline void midi_win32_close(void)
{
	MidiWin32State *s = &g_midi_win32;
	if (!s->open)
		return;
	if (s->in) {
		midiInStop(s->in);
		midiInClose(s->in);
		s->in = 0;
	}
	if (s->out) {
		midiOutClose(s->out);
		s->out = 0;
	}
	if (s->sem) {
		CloseHandle(s->sem);
		s->sem = NULL;
	}
	s->open = 0;
}

/* ── Device enumeration helpers ──────────────────────────────────────── */
static inline void midi_win32_list_devices(void)
{
	UINT n_in = midiInGetNumDevs();
	UINT n_out = midiOutGetNumDevs();
	MIDIINCAPS ic;
	MIDIOUTCAPS oc;
	printf("MIDI Input devices:\n");
	for (UINT i = 0; i < n_in; i++) {
		midiInGetDevCaps(i, &ic, sizeof(ic));
		printf("  [%u] %s\n", i, ic.szPname);
	}
	printf("MIDI Output devices:\n");
	for (UINT i = 0; i < n_out; i++) {
		midiOutGetDevCaps(i, &oc, sizeof(oc));
		printf("  [%u] %s\n", i, oc.szPname);
	}
}

#endif /* _WIN32 */
#endif /* DJCMD_MIDI_WIN32_H */
