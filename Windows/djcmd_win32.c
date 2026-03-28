/* djcmd_win32.c — Windows port entry point for djcmd.
 *
 * Strategy: include the Windows compatibility headers FIRST, then pull in
 * the main djcmd.c with the ALSA/ncurses/pthread calls replaced via macros.
 *
 * Platform substitutions:
 *   ALSA PCM    → PortAudio       (audio_win32.h)
 *   ALSA rawmidi→ WinMM           (midi_win32.h)
 *   ncurses     → PDCurses        (platform_win32.h → curses.h)
 *   pthreads    → pthreads-win32  (platform_win32.h → pthread.h)
 *   clock_gettime → QPC wrapper   (platform_win32.h)
 *   usleep      → Sleep wrapper   (platform_win32.h)
 *
 * ── How to add a new platform section ───────────────────────────────
 * Find the ALSA or signal-specific block in ../djcmd.c and bracket it:
 *   #ifndef _WIN32
 *     ... original code ...
 *   #else
 *     ... Windows equivalent ...
 *   #endif
 * Then rebuild from this directory.
 *
 * ── MIDI device selection ────────────────────────────────────────────
 * On Windows, MIDI devices are indexed (not named like /dev/midi1).
 * djcmd_win32 reads the environment variables:
 *   DJCMD_MIDI_IN_DEV   (default: 0)
 *   DJCMD_MIDI_OUT_DEV  (default: 0)
 * Run with --list-midi to see available device indices.
 */

/* ── Platform compatibility layer — must come first ─────────────────── */
#include "platform_win32.h"
#include "audio_win32.h"
#include "midi_win32.h"

/* ── Redirect ALSA PCM macros ───────────────────────────────────────── *
 * djcmd.c wraps its audio-open and write sections in
 *   #ifndef _WIN32 / #else / #endif
 * so these macros only need to cover the Windows path.
 *
 * If you haven't patched ../djcmd.c with the ifdefs yet, do it here
 * by defining stub macros that the compiler will use instead of the
 * ALSA functions declared in <alsa/asoundlib.h>.
 *
 * NOTE: This file is a COPY of ../djcmd.c plus the ifdefs below.
 *       Keep it in sync with the parent when features are added.
 */

/* Prevent <alsa/asoundlib.h> from being included on Windows */
#define _ALSA_ASOUNDLIB_H /* sentinel — ALSA uses this guard on some versions */

/* Stub out the ALSA types and functions used by djcmd.c.
 * Each is replaced by the PortAudio / WinMM equivalents below. */
typedef void snd_pcm_t;
typedef void snd_rawmidi_t;
typedef int snd_pcm_uframes_t;
typedef int snd_pcm_sframes_t;
typedef int snd_pcm_hw_params_t;

/* djcmd.c calls snd_pcm_open / snd_pcm_writei etc.
 * Map them to our PortAudio wrapper. */
static inline int win32_pcm_open(void)
{
	int main_dev = -1;
	int hp_dev = -1;
	const char *env_main = getenv("DJCMD_AUDIO_DEV");
	const char *env_hp = getenv("DJCMD_HP_DEV");
	if (env_main) main_dev = atoi(env_main);
	if (env_hp) hp_dev = atoi(env_hp);

	int err = pa_open_stream(&g_pa_main, main_dev, 44100, 2, 512);
	if (err < 0) return err;

	/* Try to open headphone stream if configured */
	if (hp_dev >= 0) {
		pa_open_stream(&g_pa_hp, hp_dev, 44100, 2, 512);
	}
	return 0;
}
static inline int win32_pcm_write(const int16_t *buf, int frames)
{
	return (int)pa_write_stream(&g_pa_main, buf, (unsigned)frames);
}
static inline int win32_pcm_write_hp(const int16_t *buf, int frames)
{
	return (int)pa_write_stream(&g_pa_hp, buf, (unsigned)frames);
}
static inline void win32_pcm_close(void)
{
	pa_shutdown();
}

/* ── Console Ctrl handler (replaces SIGINT/SIGTERM) ─────────────────── */
static volatile int g_win32_quit = 0;
static BOOL WINAPI win32_ctrl_handler(DWORD ctrl)
{
	if (ctrl == CTRL_C_EVENT || ctrl == CTRL_CLOSE_EVENT) {
		g_win32_quit = 1;
		return TRUE;
	}
	return FALSE;
}

/* ── Windows main wrapper ────────────────────────────────────────────── *
 * djcmd.c defines main(); we rename it via macro and wrap it here to
 * do Windows-specific init (console handler, MIDI device enumeration,
 * PDCurses terminal setup) before calling through.
 */
#define main djcmd_main_impl

/* ── Pull in the full djcmd source ──────────────────────────────────── *
 * The #include makes djcmd_main_impl() visible here.
 * All POSIX functions are already shimmed via the headers above.
 *
 * Remaining ALSA calls inside djcmd.c are guarded with:
 *   #ifndef _WIN32 ... #else ... #endif
 * — see the patch notes below for which sections need bracketing.
 */
#include "../djcmd.c"

#undef main

/* ── True entry point ────────────────────────────────────────────────── */
int main(int argc, char **argv)
{
	/* List MIDI devices if requested */
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--list-midi") == 0) {
			midi_win32_list_devices();
			return 0;
		}
	}

	/* Install console quit handler */
	SetConsoleCtrlHandler(win32_ctrl_handler, TRUE);

	/* Open MIDI — indices from environment or default 0 */
	int in_dev = 0;
	int out_dev = 0;
	const char *env_in = getenv("DJCMD_MIDI_IN_DEV");
	const char *env_out = getenv("DJCMD_MIDI_OUT_DEV");
	if (env_in)
		in_dev = atoi(env_in);
	if (env_out)
		out_dev = atoi(env_out);

	if (midi_win32_open(in_dev, out_dev) != 0) {
		fprintf(stderr, "djcmd: failed to open MIDI devices %d/%d\n",
			in_dev, out_dev);
		fprintf(stderr,
			"Run with --list-midi to see available devices.\n");
		/* Continue anyway — djcmd works without MIDI (keyboard only) */
	}

	/* PDCurses on Windows needs the console in the right mode */
#ifdef PDC_WIDE
	PDC_set_resize_limits(24, 999, 80, 999);
#endif

	/* Hand off to djcmd proper */
	int ret = djcmd_main_impl(argc, argv);

	midi_win32_close();
	pa_close();
	return ret;
}

/*
 * ── Sections of ../djcmd.c that need #ifndef _WIN32 guards ──────────
 *
 * Search for these patterns in ../djcmd.c and wrap them:
 *
 * 1. ALSA header include (near top of file):
 *      #ifndef _WIN32
 *        #include <alsa/asoundlib.h>
 *      #endif
 *
 * 2. snd_pcm_open / hw_params / start (audio thread init):
 *      #ifndef _WIN32
 *        snd_pcm_open(&g_pcm, ...);
 *        // ... hw_params calls ...
 *        snd_pcm_start(g_pcm);
 *      #else
 *        win32_pcm_open();
 *      #endif
 *
 * 3. snd_pcm_writei (audio write loop):
 *      #ifndef _WIN32
 *        snd_pcm_writei(g_pcm, g_audio_buf, PERIOD_FRAMES);
 *      #else
 *        win32_pcm_write(g_audio_buf, PERIOD_FRAMES);
 *      #endif
 *
 * 4. snd_pcm_close (cleanup):
 *      #ifndef _WIN32
 *        if (g_pcm) { snd_pcm_close(g_pcm); g_pcm = NULL; }
 *      #else
 *        win32_pcm_close();
 *      #endif
 *
 * 5. snd_rawmidi_open / read / write / close (MIDI thread):
 *      #ifndef _WIN32
 *        snd_rawmidi_open(&g_midi_in, ...);
 *        // ... read loop ...
 *      #else
 *        // midi_win32_read / midi_win32_write called from midi_send_cc etc.
 *      #endif
 *
 * 6. signal() calls:
 *      #ifndef _WIN32
 *        signal(SIGPIPE, SIG_IGN);
 *      #endif
 *
 * 7. /proc/asound or /dev/snd paths — replace with WinMM device listing.
 *
 * Once all guards are in place, this file compiles without modification.
 */
