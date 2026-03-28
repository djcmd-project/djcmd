/* platform_win32.h — POSIX compatibility shims for Windows (MinGW-w64)
 *
 * Drop this header before all other system includes in djcmd.c and
 * audiofile.c when building for Windows.  It provides:
 *
 *   • clock_gettime(CLOCK_MONOTONIC, ...)  via QueryPerformanceCounter
 *   • usleep()                              via Sleep()
 *   • ssize_t typedef
 *   • snprintf / vsnprintf                  (already standard in MinGW-w64)
 *
 * Build dependencies (all free / open-source, available as MinGW packages):
 *   Audio   : PortAudio  (www.portaudio.com)             -lportaudio
 *   MIDI    : WinMM      (Windows built-in)              -lwinmm
 *   Curses  : PDCurses   (pdcurses.sourceforge.net)      -lpdcurses
 *   Threads : pthreads-w32 (sourceware.org/pthreads-win32) -lpthread
 *   SQLite3 : sqlite3     (sqlite.org – amalgamation)    -lsqlite3
 *   minimp3 : header-only (no change needed)
 *   dr_flac : header-only (no change needed)
 */

#pragma once
#ifndef DJCMD_PLATFORM_WIN32_H
#define DJCMD_PLATFORM_WIN32_H

#ifdef _WIN32

/* ── Standard Windows headers ─────────────────────────────────────────── */
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <mmsystem.h> /* WinMM MIDI */
#include <io.h>
#include <process.h>

/* ── pthreads-w32 drop-in ─────────────────────────────────────────────── */
#include <pthread.h> /* pthreads-win32: same API as POSIX */

/* ── PDCurses drop-in ─────────────────────────────────────────────────── */
#include <curses.h> /* PDCurses: same API as ncurses */

/* ── PortAudio for audio output ──────────────────────────────────────── */
#include <portaudio.h>

/* ── ssize_t ──────────────────────────────────────────────────────────── */
#ifndef _SSIZE_T_DEFINED
#define _SSIZE_T_DEFINED
typedef SSIZE_T ssize_t;
#endif

/* ── POSIX clock_gettime ──────────────────────────────────────────────── */
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif
#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif

typedef int clockid_t;

static inline int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
	(void)clk_id;
	static LARGE_INTEGER freq = { 0 };
	LARGE_INTEGER count;
	if (!freq.QuadPart)
		QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&count);
	tp->tv_sec = (time_t)(count.QuadPart / freq.QuadPart);
	tp->tv_nsec = (long)((count.QuadPart % freq.QuadPart) * 1000000000LL /
			     freq.QuadPart);
	return 0;
}

/* ── usleep (microseconds → milliseconds; rounds up) ─────────────────── */
static inline int usleep(unsigned int us)
{
	Sleep((us + 999) / 1000);
	return 0;
}

/* ── POSIX signals — minimal stub ────────────────────────────────────── */
/* djcmd uses SIGINT/SIGTERM for graceful exit.  On Windows use
 * SetConsoleCtrlHandler instead; the djcmd.c Windows section does this
 * after main() entry.  These stubs keep the signal() calls harmless. */
#ifndef SIGPIPE
#define SIGPIPE 13
#endif

/* ── PATH separator ──────────────────────────────────────────────────── */
#define PATH_SEP '\\'
#define PATH_SEP_STR "\\"

/* ── dirent — MinGW-w64 includes its own dirent.h ────────────────────── */
#include <dirent.h>

/* ── S_ISDIR, S_ISREG ────────────────────────────────────────────────── */
#ifndef S_ISDIR
#define S_ISDIR(m) (((m) & _S_IFMT) == _S_IFDIR)
#endif
#ifndef S_ISREG
#define S_ISREG(m) (((m) & _S_IFMT) == _S_IFREG)
#endif

#endif /* _WIN32 */
#endif /* DJCMD_PLATFORM_WIN32_H */
