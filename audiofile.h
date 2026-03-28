/*
 * audiofile.h — Multi-format audio file loader + directory scanner
 *
 * Supports: .wav  .mp3  .flac
 * All formats decoded to: 44100 Hz, stereo, int16_t interleaved PCM
 *
 * Depends on two single-header public-domain libraries dropped
 * alongside this file:
 *   minimp3.h   — https://github.com/lieff/minimp3
 *   dr_flac.h   — https://github.com/mackron/dr_libs
 *
 * PowerPC 7447A notes:
 *   - No SIMD intrinsics used; plain C so GCC can auto-vectorise
 *     with -mcpu=7450 -ffast-math
 *   - Decoding is done once at load time, not at playback time,
 *     so runtime CPU cost is zero.
 */

#ifndef AUDIOFILE_H
#define AUDIOFILE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Result codes ───────────────────────────────────────────────── */
#define AF_OK 0
#define AF_ERR_OPEN -1 /* could not open file              */
#define AF_ERR_FORMAT -2 /* unrecognised / unsupported format */
#define AF_ERR_DECODE -3 /* decode error inside library       */
#define AF_ERR_RESAMPLE -4 /* sample-rate conversion failed     */
#define AF_ERR_MEMORY -5 /* malloc/realloc failed             */

/* ── Loaded audio buffer ────────────────────────────────────────── */
typedef struct {
	int16_t *samples; /* interleaved stereo int16_t PCM     */
	uint32_t num_frames; /* number of stereo frames            */
	uint32_t sample_rate; /* always af_get_target_rate() after load */
	int channels; /* always 2 after load                */
} AFBuffer;

/* Default target rate — overridden at runtime via af_set_target_rate().
 * djcmd calls this after negotiating with ALSA so the resampler matches
 * the hardware (e.g. NS7III soundcard requires 48000 Hz, not 44100 Hz). */
#define AF_TARGET_RATE_DEFAULT 44100
#define AF_TARGET_CH 2

/* af_set_target_rate() — set the output sample rate for af_load().
 * Call this once after opening your audio device, before loading any tracks.
 * Supported rates: any value > 0 (8000–192000 is the practical range).
 * Default: AF_TARGET_RATE_DEFAULT (44100 Hz).
 * Returns the rate that was actually set.
 */
uint32_t af_set_target_rate(uint32_t rate);

/* af_get_target_rate() — return the currently configured target rate. */
uint32_t af_get_target_rate(void);

/* AF_TARGET_RATE — resolves to the current runtime target rate.
 * Use af_get_target_rate() in C code; this macro is for header compat. */
#define AF_TARGET_RATE (af_get_target_rate())

/*
 * af_load() — load any supported audio file into an AFBuffer.
 *
 *   path   : absolute or relative file path
 *   out    : caller-allocated AFBuffer; filled on success
 *
 * Returns AF_OK on success.  On failure, out->samples is NULL.
 * Caller must call af_free() when done.
 */
int af_load(const char *path, AFBuffer *out);

/*
 * af_free() — release memory allocated by af_load().
 */
void af_free(AFBuffer *buf);

/* ── Directory scanner ──────────────────────────────────────────── */

/*
 * af_is_supported() — returns 1 if the filename extension is a
 *                     supported audio format (.wav .mp3 .flac),
 *                     0 otherwise.  Case-insensitive.
 */
int af_is_supported(const char *filename);

/*
 * AFScanResult — one entry returned by af_scan_dir()
 */
typedef struct {
	char *path; /* heap-allocated full path; free with af_scan_free() */
	char *basename; /* points inside path, do not free separately          */
	char ext[8]; /* lower-case extension e.g. "mp3"                    */
} AFScanEntry;

typedef struct {
	AFScanEntry *
		entries; /* heap-allocated array                                */
	int count;
	int capacity;
} AFScanResult;

/*
 * af_scan_dir() — scan a directory for supported audio files.
 *
 *   dir_path  : directory to scan (non-recursive)
 *   result    : output; caller must call af_scan_free() when done
 *   max_files : maximum entries to collect (0 = unlimited)
 *
 * Returns the number of files found, or -1 on error.
 * Files are returned in the order readdir() provides them
 * (typically alphabetical on most Linux filesystems).
 */
int af_scan_dir(const char *dir_path, AFScanResult *result, int max_files);

/*
 * af_scan_dir_recursive() — like af_scan_dir but descends into
 *                           sub-directories (up to max_depth levels).
 */
int af_scan_dir_recursive(const char *dir_path, AFScanResult *result,
			  int max_files, int max_depth);

/*
 * af_scan_free() — release all memory owned by an AFScanResult.
 */
void af_scan_free(AFScanResult *result);

/*
 * af_scan_sort_alpha() — sort AFScanResult entries alphabetically
 *                        by basename (case-insensitive).
 */
void af_scan_sort_alpha(AFScanResult *result);

/*
 * af_scan_sort_ext() — sort by extension then basename.
 *                      Groups: flac → wav → mp3 (highest → lowest quality).
 */
void af_scan_sort_ext(AFScanResult *result);

/*
 * af_format_name() — human-readable format name from extension string.
 *   e.g. "mp3" → "MP3", "flac" → "FLAC", unknown → "???"
 */
const char *af_format_name(const char *ext);

#ifdef __cplusplus
}
#endif
#endif /* AUDIOFILE_H */
