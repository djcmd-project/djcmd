#define _GNU_SOURCE
#include "djcmd_library.h"
#include "audiofile.h"
#include "djcmd_audio.h"
#include "djcmd_config.h"
#include "djcmd_ui.h"
#include "djcmd_usb.h"
#include <ctype.h>
#include <dirent.h>
#include <math.h>
#include <pthread.h>
#include <sqlite3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

/* ── Constants ──────────────────────────────────────────────────────── */
#define MX_MAX_BEATS 16384
#define DJCMD_MAGIC "DJCM"
#define DJCMD_VERSION 6
#define DJCMD_ENDIAN_SENTINEL ((uint16_t)0xDAED)

/* ── Globals ────────────────────────────────────────────────────────── */
extern _Atomic int g_running;
extern unsigned int g_actual_sample_rate;

char g_fb_path[FB_PATH_MAX] = ".";
FBEntry g_fb_entries[FB_MAX_ENTRIES];
int g_fb_count = 0;
int g_fb_sel = 0;
int g_fb_scroll = 0;
char g_fb_status[256] = "";
int g_fb_sort = 0;

PLEntry g_pl[PL_MAX];
int g_pl_count = 0;
int g_pl_sel = 0;
int g_pl_scroll = 0;

LIBEntry *g_lib = NULL;
int g_lib_count = 0;
int g_lib_sel = 0;
int g_lib_scroll = 0;
volatile int g_lib_scanning = 0;
char g_lib_root[FB_PATH_MAX] = "";
int g_lib_sort = 0;

Crate g_crates[MAX_CRATES];
int g_ncrate = 0;
int g_crate_sel = 0;
int g_crate_vrow = 0;
int g_crate_vscroll = 0;
CrateEntry *g_crate_tracks = NULL;
int g_crate_tracks_count = 0;
int g_crate_tracks_sel = 0;
int g_crate_tracks_scroll = 0;
int g_crate_view_level = 0;
char g_active_crate_name[256] = "";
CrateGroup g_crate_groups[32];
int g_ncrate_groups = 0;

int g_crate_jump_active = 0;
char g_crate_input[64] = "";
char g_crate_orig_input[64] = "";
int g_crate_cycle_idx = -1;
int g_crate_matches[MAX_CRATES];
int g_ncrate_matches = 0;

int g_crate_add_active = 0;
char g_crate_add_input[64] = "";
char g_crate_add_path[FB_PATH_MAX] = "";

int g_track_add_crate_active = 0;
char g_track_add_crate_input[64] = "";
char g_pending_track_path[FB_PATH_MAX] = "";

USBDevice g_usb_devices[USB_MAX_DEVICES];
int g_usb_devices_count = 0;
int g_usb_eject_active = 0;
char g_usb_eject_mount[512] = "";
char g_usb_eject_label[128] = "";

int g_usb_picker_active = 0;
int g_usb_picker_sel = 0;
int g_usb_export_crate_idx = -1;

int g_usb_conflict_active = 0;
char g_usb_conflict_rename[64] = "";
char g_usb_conflict_mount[512] = "";
char g_usb_conflict_crate_name[64] = "";

int g_batch_prompt_active = 0;
int g_batch_prompt_field = 0;
char g_batch_lo_buf[16] = "80";
char g_batch_hi_buf[16] = "160";
int g_batch_running = 0;
BatchQEntry *g_batch_queue = NULL;
int g_batch_queue_pos = 0;
int g_batch_queue_count = 0;
int g_batch_panel = 0;
int g_batch_job_done = 0;
int64_t g_n_last_tap_ms = 0;

float g_bpm_detect_lo = 80.0f;
float g_bpm_detect_hi = 160.0f;

char g_machine_id[64] = "";

/* ── Helpers ────────────────────────────────────────────────────────── */
static uint32_t djb2_hash(const char *s)
{
	uint32_t h = 5381;
	while (*s) {
		h = ((h << 5) + h) ^ (uint8_t)*s++;
	}
	return h;
}

static uint32_t bswap32(uint32_t v)
{
	return ((v & 0xFF000000u) >> 24) | ((v & 0x00FF0000u) >> 8) |
	       ((v & 0x0000FF00u) << 8) | ((v & 0x000000FFu) << 24);
}

static float bswap_float(float v)
{
	uint32_t tmp;
	memcpy(&tmp, &v, 4);
	tmp = bswap32(tmp);
	float r;
	memcpy(&r, &tmp, 4);
	return r;
}

static void sidecar_path(const char *audio_path, char *out, size_t out_sz)
{
	const char *home = getenv("HOME");
	if (!home) {
		home = "/tmp";
	}
	const char *bn = strrchr(audio_path, '/');
	bn = bn ? bn + 1 : audio_path;
	uint32_t h = djb2_hash(audio_path);
	snprintf(out, out_sz, "%s/.config/djcmd/cache/%s-%08x.djcmd", home, bn,
	         h);
}

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

/* ── Analysis Cache ─────────────────────────────────────────────────── */
void ensure_sidecar_cache_dir(void)
{
	const char *home = getenv("HOME");
	if (!home) {
		return;
	}
	char dir[512];
	snprintf(dir, sizeof(dir), "%s/.config", home);
	mkdir(dir, 0755);
	snprintf(dir, sizeof(dir), "%s/.config/djcmd", home);
	mkdir(dir, 0755);
	snprintf(dir, sizeof(dir), "%s/.config/djcmd/cache", home);
	mkdir(dir, 0755);
}

void sidecar_save(const Track *t)
{
	if (!t->loaded || t->wfm_bins == 0 || !t->wfm_low) {
		return;
	}
	char path[MAX_FILENAME + 8];
	sidecar_path(t->filename, path, sizeof(path));
	FILE *f = fopen(path, "wb");
	if (!f) {
		return;
	}
	fwrite(DJCMD_MAGIC, 1, 4, f);
	uint8_t ver = DJCMD_VERSION;
	fwrite(&ver, 1, 1, f);
	uint16_t sentinel = DJCMD_ENDIAN_SENTINEL;
	fwrite(&sentinel, 2, 1, f);
	uint32_t nf = t->num_frames;
	fwrite(&nf, 4, 1, f);
	float bpm = t->bpm, off = t->bpm_offset;
	fwrite(&bpm, 4, 1, f);
	fwrite(&off, 4, 1, f);
	uint32_t bins = t->wfm_bins;
	fwrite(&bins, 4, 1, f);
	fwrite(t->wfm_low, 1, bins, f);
	fwrite(t->wfm_mid, 1, bins, f);
	fwrite(t->wfm_high, 1, bins, f);
	for (int ci = 0; ci < MAX_CUES; ci++) {
		uint8_t flag = (uint8_t)(t->cue_set[ci] ? 1 : 0);
		fwrite(&flag, 1, 1, f);
	}
	for (int ci = 0; ci < MAX_CUES; ci++) {
		uint32_t pos = t->cue_set[ci] ? t->cue[ci] : 0;
		fwrite(&pos, 4, 1, f);
	}
	fclose(f);
}

int sidecar_load(Track *t)
{
	char path[MAX_FILENAME + 8];
	sidecar_path(t->filename, path, sizeof(path));
	FILE *f = fopen(path, "rb");
	if (!f) {
		snprintf(path, sizeof(path), "%s.djcmd", t->filename);
		f = fopen(path, "rb");
		if (!f) {
			return -1;
		}
	}
	char magic[4];
	if (fread(magic, 1, 4, f) != 4 || memcmp(magic, DJCMD_MAGIC, 4) != 0) {
		fclose(f);
		return -1;
	}
	uint8_t ver;
	if (fread(&ver, 1, 1, f) != 1 || ver != DJCMD_VERSION) {
		fclose(f);
		return -1;
	}
	uint16_t sentinel;
	if (fread(&sentinel, 2, 1, f) != 1) {
		fclose(f);
		return -1;
	}
	int need_swap = (sentinel == (uint16_t)0xEDDA);
	if (!need_swap && sentinel != DJCMD_ENDIAN_SENTINEL) {
		fclose(f);
		return -1;
	}
	uint32_t cached_frames;
	if (fread(&cached_frames, 4, 1, f) != 1) {
		fclose(f);
		return -1;
	}
	if (need_swap) {
		cached_frames = bswap32(cached_frames);
	}
	if (cached_frames != t->num_frames) {
		fclose(f);
		return -1;
	}
	float bpm, off;
	if (fread(&bpm, 4, 1, f) != 1 || fread(&off, 4, 1, f) != 1) {
		fclose(f);
		return -1;
	}
	if (need_swap) {
		bpm = bswap_float(bpm);
		off = bswap_float(off);
	}
	uint32_t bins;
	if (fread(&bins, 4, 1, f) != 1) {
		fclose(f);
		return -1;
	}
	if (need_swap) {
		bins = bswap32(bins);
	}
	uint8_t *lov = malloc(bins), *mov = malloc(bins), *hov = malloc(bins);
	if (fread(lov, 1, bins, f) != bins || fread(mov, 1, bins, f) != bins ||
	    fread(hov, 1, bins, f) != bins) {
		free(lov);
		free(mov);
		free(hov);
		fclose(f);
		return -1;
	}
	uint8_t cue_flags[MAX_CUES];
	uint32_t cue_pos[MAX_CUES];
	int cues_loaded = 0;
	if (fread(cue_flags, 1, MAX_CUES, f) == MAX_CUES &&
	    fread(cue_pos, 4, MAX_CUES, f) == MAX_CUES) {
		if (need_swap) {
			for (int i = 0; i < MAX_CUES; i++) {
				cue_pos[i] = bswap32(cue_pos[i]);
			}
		}
		cues_loaded = 1;
	}
	fclose(f);
	t->bpm = bpm;
	t->bpm_offset = off;
	free(t->wfm_low);
	free(t->wfm_mid);
	free(t->wfm_high);
	t->wfm_low = lov;
	t->wfm_mid = mov;
	t->wfm_high = hov;
	t->wfm_bins = bins;
	wfm_compute_band_max(t);
	if (cues_loaded) {
		for (int i = 0; i < MAX_CUES; i++) {
			if (!t->cue_set[i] && cue_flags[i]) {
				t->cue[i] = cue_pos[i];
				t->cue_set[i] = 1;
			}
		}
	}
	return 0;
}

float cache_get_bpm(const char *audio_path)
{
	char path[MAX_FILENAME + 8];
	sidecar_path(audio_path, path, sizeof(path));
	FILE *f = fopen(path, "rb");
	if (!f) {
		return 0.0f;
	}
	char magic[4];
	uint8_t ver;
	uint16_t sentinel;
	uint32_t nf;
	float bpm = 0.0f;
	if (fread(magic, 1, 4, f) != 4 || memcmp(magic, DJCMD_MAGIC, 4) != 0) {
		goto done;
	}
	if (fread(&ver, 1, 1, f) != 1 || ver != DJCMD_VERSION) {
		goto done;
	}
	if (fread(&sentinel, 2, 1, f) != 1) {
		goto done;
	}
	int need_swap = (sentinel == (uint16_t)0xEDDA);
	if (!need_swap && sentinel != DJCMD_ENDIAN_SENTINEL) {
		goto done;
	}
	if (fread(&nf, 4, 1, f) != 1) {
		goto done;
	}
	if (fread(&bpm, 4, 1, f) == 1 && need_swap) {
		bpm = bswap_float(bpm);
	}
done:
	fclose(f);
	return bpm;
}

/* ── Browser & Sorting ──────────────────────────────────────────────── */
static int fb_cmp(const void *a, const void *b)
{
	const FBEntry *ea = (const FBEntry *)a;
	const FBEntry *eb = (const FBEntry *)b;
	if (ea->is_dir != eb->is_dir) {
		return eb->is_dir - ea->is_dir;
	}
	return strcasecmp(ea->name, eb->name);
}
static int fb_cmp_bpm_asc(const void *a, const void *b)
{
	const FBEntry *ea = (const FBEntry *)a;
	const FBEntry *eb = (const FBEntry *)b;
	if (ea->is_dir != eb->is_dir) {
		return eb->is_dir - ea->is_dir;
	}
	if (ea->bpm < eb->bpm) {
		return -1;
	}
	if (ea->bpm > eb->bpm) {
		return 1;
	}
	return strcasecmp(ea->name, eb->name);
}
static int fb_cmp_bpm_desc(const void *a, const void *b)
{
	const FBEntry *ea = (const FBEntry *)a;
	const FBEntry *eb = (const FBEntry *)b;
	if (ea->is_dir != eb->is_dir) {
		return eb->is_dir - ea->is_dir;
	}
	if (ea->bpm > eb->bpm) {
		return -1;
	}
	if (ea->bpm < eb->bpm) {
		return 1;
	}
	return strcasecmp(ea->name, eb->name);
}
void fb_apply_sort(void)
{
	if (g_fb_count < 2) {
		return;
	}
	if (g_fb_sort == 1) {
		qsort(g_fb_entries, g_fb_count, sizeof(FBEntry),
		      fb_cmp_bpm_asc);
	} else if (g_fb_sort == 2) {
		qsort(g_fb_entries, g_fb_count, sizeof(FBEntry),
		      fb_cmp_bpm_desc);
	} else {
		qsort(g_fb_entries, g_fb_count, sizeof(FBEntry), fb_cmp);
	}
}

void read_tags(const char *path, char *title, int tmax, char *artist, int amax)
{
	title[0] = artist[0] = '\0';

	FILE *f = fopen(path, "rb");
	if (!f) {
		return;
	}

	unsigned char hdr[10];
	if (fread(hdr, 1, 10, f) < 10) {
		fclose(f);
		return;
	}

	/* ── ID3v2 ── */
	if (hdr[0] == 'I' && hdr[1] == 'D' && hdr[2] == '3') {
		/* int version = hdr[3]; */
		/* int flags   = hdr[5]; */
		uint32_t id3size = ((uint32_t)(hdr[6] & 0x7f) << 21) |
		                   ((uint32_t)(hdr[7] & 0x7f) << 14) |
		                   ((uint32_t)(hdr[8] & 0x7f) << 7) |
		                   (uint32_t)(hdr[9] & 0x7f);
		uint32_t end = 10 + id3size;
		uint32_t pos = 10;
		int ver = hdr[3]; /* 2, 3, or 4 */

		while (pos + (ver <= 2 ? 6 : 10) < end &&
		       (title[0] == '\0' || artist[0] == '\0')) {
			unsigned char fhdr[10];
			int hlen = (ver <= 2) ? 6 : 10;
			if (fseek(f, (long)pos, SEEK_SET) != 0) {
				break;
			}
			if ((int)fread(fhdr, 1, hlen, f) < hlen) {
				break;
			}
			if (fhdr[0] == 0) {
				break; /* padding */
			}

			uint32_t fsz;
			if (ver <= 2) {
				fsz = ((uint32_t)fhdr[3] << 16) |
				      ((uint32_t)fhdr[4] << 8) | fhdr[5];
			} else {
				fsz = ((uint32_t)fhdr[4] << 24) |
				      ((uint32_t)fhdr[5] << 16) |
				      ((uint32_t)fhdr[6] << 8) | fhdr[7];
			}

			/* Identify TIT2/TT2 and TPE1/TP1 */
			int is_title =
			    (ver <= 2) ? (fhdr[0] == 'T' && fhdr[1] == 'T' &&
			                  fhdr[2] == '2')
			               : (fhdr[0] == 'T' && fhdr[1] == 'I' &&
			                  fhdr[2] == 'T' && fhdr[3] == '2');
			int is_artist =
			    (ver <= 2) ? (fhdr[0] == 'T' && fhdr[1] == 'P' &&
			                  fhdr[2] == '1')
			               : (fhdr[0] == 'T' && fhdr[1] == 'P' &&
			                  fhdr[2] == 'E' && fhdr[3] == '1');

			if ((is_title && title[0] == '\0') ||
			    (is_artist && artist[0] == '\0')) {
				char *buf = (is_title) ? title : artist;
				int max = (is_title) ? tmax : amax;
				if (fsz > 1 && fsz < 4096) {
					unsigned char enc;
					(void)fread(&enc, 1, 1,
					            f); /* encoding byte */
					int read = (int)fsz - 1;
					if (read >= max) {
						read = max - 1;
					}
					(void)fread(buf, 1, read, f);
					buf[read] = '\0';
					/* UTF-16: skip BOM and take every other
					 * byte (ASCII range) */
					if (enc == 1 || enc == 2) {
						int src = 0, dst = 0;
						if (read >= 2 &&
						    (unsigned char)buf[0] ==
						        0xFF) {
							src = 2; /* skip BOM */
						}
						while (src + 1 < read) {
							unsigned char
							    lo = buf[src],
							    hi = buf[src + 1];
							buf[dst++] =
							    (hi == 0 &&
							     lo >= 0x20)
							        ? (char)lo
							        : '?';
							src += 2;
						}
						buf[dst] = '\0';
					}
					tag_clean(buf);
				}
			}
			pos += hlen + fsz;
		}
		if (title[0] || artist[0]) {
			fclose(f);
			return;
		}
	}

	/* ── Vorbis comments (FLAC) ── */
	fseek(f, 0, SEEK_SET);
	unsigned char magic[4];
	if (fread(magic, 1, 4, f) == 4 && magic[0] == 'f' && magic[1] == 'L' &&
	    magic[2] == 'a' && magic[3] == 'C') {
		/* Scan METADATA_BLOCKs for VORBIS_COMMENT (type 4) */
		for (int bi = 0; bi < 32; bi++) {
			unsigned char blkhdr[4];
			if (fread(blkhdr, 1, 4, f) < 4) {
				break;
			}
			int blktype = blkhdr[0] & 0x7F;
			int last = (blkhdr[0] >> 7) & 1;
			uint32_t blksz = ((uint32_t)blkhdr[1] << 16) |
			                 ((uint32_t)blkhdr[2] << 8) | blkhdr[3];
			if (blktype == 4) { /* VORBIS_COMMENT */
				long base = ftell(f);
				/* vendor string */
				uint32_t vlen;
				(void)fread(&vlen, 4, 1, f);
				fseek(f, (long)vlen, SEEK_CUR);
				uint32_t ncomments;
				(void)fread(&ncomments, 4, 1, f);
				for (uint32_t ci = 0; ci < ncomments && ci < 64;
				     ci++) {
					uint32_t clen;
					(void)fread(&clen, 4, 1, f);
					if (clen == 0 || clen > 4096) {
						fseek(f, (long)clen, SEEK_CUR);
						continue;
					}
					char cbuf[4097];
					int rd =
					    (clen < 4096) ? (int)clen : 4096;
					(void)fread(cbuf, 1, rd, f);
					cbuf[rd] = '\0';
					/* Skip remaining bytes if truncated */
					if ((uint32_t)rd < clen) {
						fseek(f, (long)(clen - rd),
						      SEEK_CUR);
					}
					/* Parse KEY=VALUE */
					char *eq = strchr(cbuf, '=');
					if (!eq) {
						continue;
					}
					*eq = '\0';
					char *key = cbuf;
					char *val = eq + 1;
					/* Case-insensitive compare */
					char ku[32];
					int ki = 0;
					while (key[ki] && ki < 31) {
						ku[ki] = toupper(
						    (unsigned char)key[ki]);
						ki++;
					}
					ku[ki] = '\0';
					if (!strcmp(ku, "TITLE") &&
					    title[0] == '\0') {
						snprintf(title, tmax, "%s",
						         val);
						tag_clean(title);
					} else if (!strcmp(ku, "ARTIST") &&
					           artist[0] == '\0') {
						snprintf(artist, amax, "%s",
						         val);
						tag_clean(artist);
					}
				}
				(void)base;
				break;
			}
			if (last) {
				break;
			}
			fseek(f, (long)blksz, SEEK_CUR);
		}
		if (title[0] || artist[0]) {
			fclose(f);
			return;
		}
	}

	/* ── RIFF INFO (WAV) ── */
	fseek(f, 0, SEEK_SET);
	unsigned char riff[12];
	if (fread(riff, 1, 12, f) == 12 && riff[0] == 'R' && riff[1] == 'I' &&
	    riff[2] == 'F' && riff[3] == 'F' && riff[8] == 'W' &&
	    riff[9] == 'A' && riff[10] == 'V' && riff[11] == 'E') {
		/* Scan chunks looking for LIST/INFO */
		for (int ci = 0; ci < 64; ci++) {
			unsigned char chdr[8];
			if (fread(chdr, 1, 8, f) < 8) {
				break;
			}
			uint32_t csz = (uint32_t)chdr[4] |
			               ((uint32_t)chdr[5] << 8) |
			               ((uint32_t)chdr[6] << 16) |
			               ((uint32_t)chdr[7] << 24);
			if (chdr[0] == 'L' && chdr[1] == 'I' &&
			    chdr[2] == 'S' && chdr[3] == 'T') {
				unsigned char ltype[4];
				(void)fread(ltype, 1, 4, f);
				if (ltype[0] == 'I' && ltype[1] == 'N' &&
				    ltype[2] == 'F' && ltype[3] == 'O') {
					uint32_t left = csz - 4;
					while (left >= 8) {
						unsigned char ihdr[8];
						if (fread(ihdr, 1, 8, f) < 8) {
							break;
						}
						uint32_t isz =
						    (uint32_t)ihdr[4] |
						    ((uint32_t)ihdr[5] << 8) |
						    ((uint32_t)ihdr[6] << 16) |
						    ((uint32_t)ihdr[7] << 24);
						left -= 8;
						int is_t =
						    (ihdr[0] == 'I' &&
						     ihdr[1] == 'N' &&
						     ihdr[2] == 'A' &&
						     ihdr[3] == 'M'); /* INAM */
						int is_a =
						    (ihdr[0] == 'I' &&
						     ihdr[1] == 'A' &&
						     ihdr[2] == 'R' &&
						     ihdr[3] == 'T'); /* IART */
						if ((is_t || is_a) && isz > 0 &&
						    isz < 256) {
							char *buf =
							    is_t ? title
							         : artist;
							int max =
							    is_t ? tmax : amax;
							int rd =
							    (int)isz < max - 1
							        ? (int)isz
							        : max - 1;
							(void)fread(buf, 1, rd,
							            f);
							buf[rd] = '\0';
							if ((uint32_t)rd <
							    isz) {
								fseek(
								    f,
								    (long)(isz -
								           rd),
								    SEEK_CUR);
							}
							tag_clean(buf);
						} else {
							fseek(f, (long)isz,
							      SEEK_CUR);
						}
						left -=
						    (isz < left) ? isz : left;
					}
					break;
				}
				fseek(f, (long)(csz - 4), SEEK_CUR);
			} else {
				fseek(f, (long)csz, SEEK_CUR);
			}
		}
	}
	fclose(f);
}

void fb_lookup_bpms(void)
{
	/* Authoritative: check djcmd sidecars first */
	for (int i = 0; i < g_fb_count; i++) {
		if (!g_fb_entries[i].is_dir) {
			char full[FB_PATH_MAX + 256];
			snprintf(full, sizeof(full), "%.*s/%.*s",
			         FB_PATH_MAX - 1, g_fb_path, 255,
			         g_fb_entries[i].name);
			g_fb_entries[i].bpm = cache_get_bpm(full);
		}
	}

	/* Fallback: Mixxx database */
	const char *home = getenv("HOME");
	if (!home) {
		return;
	}
	char db_path[1024];
	snprintf(db_path, sizeof(db_path), "%s/.mixxx/mixxxdb.sqlite", home);

	sqlite3 *db = NULL;
	if (sqlite3_open_v2(db_path, &db,
	                    SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX,
	                    NULL) != SQLITE_OK) {
		if (db) {
			sqlite3_close(db);
		}
		return;
	}

	/* Normalize the path: collapse any double-slashes and strip trailing
	 * slash.  g_fb_path can accumulate "//" when navigating from a path
	 * that already had a trailing slash (e.g. from config or argv).    */
	char clean_path[FB_PATH_MAX];
	{
		const char *s = g_fb_path;
		int o = 0;
		while (*s && o < FB_PATH_MAX - 1) {
			if (*s == '/' && *(s + 1) == '/') {
				s++;
				continue;
			}
			clean_path[o++] = *s++;
		}
		clean_path[o] = '\0';
		while (o > 1 && clean_path[o - 1] == '/') {
			clean_path[--o] = '\0';
		}
	}

	/* Build LIKE pattern: clean_path + "/%" matches every file in dir. */
	char like_pat[FB_PATH_MAX * 2 + 4];
	{
		const char *src = clean_path;
		int out = 0;
		while (*src && out < (int)sizeof(like_pat) - 4) {
			if (*src == '%' || *src == '_' || *src == '\\') {
				like_pat[out++] = '\\';
			}
			like_pat[out++] = *src++;
		}
		/* Ensure no trailing slash before the wildcard (except root) */
		while (out > 1 && like_pat[out - 1] == '/') {
			out--;
		}
		like_pat[out++] = '/';
		like_pat[out++] = '%';
		like_pat[out] = '\0';
	}

	/* Simple query: location prefix match → return (basename, bpm, key).
	 * We use tl.filename which Mixxx stores as just the basename. */
	const char *sql = "SELECT tl.filename, l.bpm, l.key "
	                  "FROM library l "
	                  "JOIN track_locations tl ON tl.id = l.location "
	                  "WHERE l.mixxx_deleted = 0 "
	                  "  AND tl.location LIKE ? ESCAPE '\\';";

	sqlite3_stmt *stmt = NULL;
	if (sqlite3_prepare_v2(db, sql, -1, &stmt, NULL) != SQLITE_OK) {
		sqlite3_close(db);
		return;
	}
	sqlite3_bind_text(stmt, 1, like_pat, -1, SQLITE_STATIC);

	while (sqlite3_step(stmt) == SQLITE_ROW) {
		const char *fname = (const char *)sqlite3_column_text(stmt, 0);
		float bpm = (float)sqlite3_column_double(stmt, 1);
		const char *key = (const char *)sqlite3_column_text(stmt, 2);
		if (!fname) {
			continue;
		}

		/* Match against browser entries by basename */
		for (int i = 0; i < g_fb_count; i++) {
			if (!g_fb_entries[i].is_dir &&
			    strcmp(g_fb_entries[i].name, fname) == 0) {
				/* Update if missing */
				if (g_fb_entries[i].bpm <= 0.0f && bpm > 0.0f) {
					g_fb_entries[i].bpm = bpm;
				}
				if (!g_fb_entries[i].tag_key[0] && key) {
					snprintf(
					    g_fb_entries[i].tag_key,
					    sizeof(g_fb_entries[i].tag_key),
					    "%s", key);
				}
				break;
			}
		}
	}
	sqlite3_finalize(stmt);
	sqlite3_close(db);
}

void fb_scan(void)
{
	g_fb_count = 0;
	g_fb_sel = 0;
	g_fb_scroll = 0;

	DIR *d = opendir(g_fb_path);
	if (!d) {
		return;
	}

	/* Always add ".." unless we're at root */
	if (strcmp(g_fb_path, "/") != 0) {
		snprintf(g_fb_entries[g_fb_count].name,
		         sizeof(g_fb_entries[0].name), "%s", "..");
		g_fb_entries[g_fb_count].is_dir = 1;
		g_fb_entries[g_fb_count].bpm = 0.0f;
		g_fb_count++;
	}

	struct dirent *ent;
	while ((ent = readdir(d)) && g_fb_count < FB_MAX_ENTRIES) {
		if (ent->d_name[0] == '.') {
			continue; /* skip hidden */
		}

		/* Stat to determine type */
		char full[FB_PATH_MAX + 256];
		snprintf(full, sizeof(full), "%s/%s", g_fb_path, ent->d_name);
		struct stat st;
		if (stat(full, &st) != 0) {
			continue;
		}

		if (S_ISDIR(st.st_mode)) {
			snprintf(g_fb_entries[g_fb_count].name,
			         sizeof(g_fb_entries[0].name), "%s",
			         ent->d_name);
			g_fb_entries[g_fb_count].is_dir = 1;
			g_fb_entries[g_fb_count].bpm = 0.0f;
			g_fb_count++;
		} else if (S_ISREG(st.st_mode) &&
		           af_is_supported(ent->d_name)) {
			snprintf(g_fb_entries[g_fb_count].name,
			         sizeof(g_fb_entries[0].name), "%s",
			         ent->d_name);
			g_fb_entries[g_fb_count].is_dir = 0;
			g_fb_entries[g_fb_count].bpm = 0.0f;
			g_fb_count++;
		}
	}
	closedir(d);

	/* Sort by current sort mode */
	fb_apply_sort();

	/* Annotate file entries with BPM from mixxxdb (best-effort, silent
	 * fail) */
	fb_lookup_bpms();

	/* Read ID3/Vorbis tags for audio files -- fast header-only reads */
	for (int i = 0; i < g_fb_count; i++) {
		FBEntry *e = &g_fb_entries[i];
		if (e->is_dir) {
			continue;
		}
		char full[FB_PATH_MAX + 256 + 2]; /* +2: '/' separator + NUL */
		full[0] = '\0';
		if (strcmp(g_fb_path, "/") == 0) {
			full[0] = '/';
			full[1] = '\0';
			/* e->name is char[256]; copy at most 255 chars + NUL
			 * into remaining space */
			memcpy(full + 1, e->name, 255);
			full[256] = '\0';
		} else {
			strncpy(full, g_fb_path, FB_PATH_MAX);
			full[FB_PATH_MAX] = '\0';
			size_t used = strlen(full);
			if (used + 1 + 255 + 1 <= sizeof(full)) {
				full[used] = '/';
				memcpy(full + used + 1, e->name, 255);
				full[used + 1 + 255] = '\0';
			}
		}
		read_tags(full, e->tag_title, sizeof(e->tag_title),
		          e->tag_artist, sizeof(e->tag_artist));
	}
}

void fb_enter_dir(const char *name)
{
	char newpath[FB_PATH_MAX + 256];
	if (strcmp(name, "..") == 0) {
		strncpy(newpath, g_fb_path, sizeof(newpath) - 1);
		char *slash = strrchr(newpath, '/');
		if (slash && slash != newpath) {
			*slash = '\0';
		} else if (slash == newpath) {
			strcpy(newpath, "/");
		}
	} else {
		if (strcmp(g_fb_path, "/") == 0) {
			snprintf(newpath, sizeof(newpath), "/%s", name);
		} else {
			snprintf(newpath, sizeof(newpath), "%s/%s", g_fb_path,
			         name);
		}
	}
	strncpy(g_fb_path, newpath, FB_PATH_MAX - 1);
	g_fb_path[FB_PATH_MAX - 1] = '\0';
	fb_scan();
}

void fb_selected_path(char *out, size_t max)
{
	if (g_fb_count == 0) {
		out[0] = '\0';
		return;
	}
	if (strcmp(g_fb_path, "/") == 0) {
		snprintf(out, max, "/%s", g_fb_entries[g_fb_sel].name);
	} else {
		snprintf(out, max, "%s/%s", g_fb_path,
		         g_fb_entries[g_fb_sel].name);
	}
}

void fb_fix_scroll(int visible_rows)
{
	if (g_fb_sel < 0) {
		g_fb_sel = 0;
	}
	if (g_fb_sel >= g_fb_count) {
		g_fb_sel = g_fb_count - 1;
	}
	if (g_fb_scroll > g_fb_sel) {
		g_fb_scroll = g_fb_sel;
	}
	if (g_fb_scroll < g_fb_sel - visible_rows + 1) {
		g_fb_scroll = g_fb_sel - visible_rows + 1;
	}
	if (g_fb_scroll < 0) {
		g_fb_scroll = 0;
	}
}

const char *fb_display_name(const FBEntry *e, char *buf, int max)
{
	if (e->tag_artist[0] && e->tag_title[0]) {
		snprintf(buf, max, "%s \xe2\x80\x94 %s", e->tag_artist,
		         e->tag_title);
		return buf;
	} else if (e->tag_title[0]) {
		return e->tag_title;
	}
	return e->name;
}

/* ── Playlist ───────────────────────────────────────────────────────── */
void pl_add(const char *fullpath, const char *basename, float bpm,
            const char *key)
{
	if (g_pl_count >= PL_MAX) {
		return;
	}
	snprintf(g_pl[g_pl_count].path, sizeof(g_pl[0].path), "%s", fullpath);
	snprintf(g_pl[g_pl_count].name, sizeof(g_pl[0].name), "%s", basename);
	g_pl[g_pl_count].bpm = bpm;
	if (key) {
		snprintf(g_pl[g_pl_count].tag_key, sizeof(g_pl[0].tag_key),
		         "%s", key);
	} else {
		g_pl[g_pl_count].tag_key[0] = '\0';
	}
	g_pl_count++;
}

void pl_remove(int idx)
{
	if (idx < 0 || idx >= g_pl_count) {
		return;
	}
	for (int i = idx; i < g_pl_count - 1; i++) {
		g_pl[i] = g_pl[i + 1];
	}
	g_pl_count--;
	if (g_pl_sel >= g_pl_count) {
		g_pl_sel = g_pl_count - 1;
	}
	if (g_pl_sel < 0) {
		g_pl_sel = 0;
	}
}

void pl_fix_scroll(int visible)
{
	if (g_pl_sel < g_pl_scroll) {
		g_pl_scroll = g_pl_sel;
	}
	if (g_pl_sel >= g_pl_scroll + visible) {
		g_pl_scroll = g_pl_sel - visible + 1;
	}
	if (g_pl_scroll < 0) {
		g_pl_scroll = 0;
	}
}

/* ── Library Scan ───────────────────────────────────────────────────── */
static void lib_scan_dir(const char *dirpath)
{
	DIR *d = opendir(dirpath);
	if (!d)
		return;
	struct dirent *ent;
	while ((ent = readdir(d)) && g_lib_count < LIB_MAX) {
		if (ent->d_name[0] == '.')
			continue;
		char full[FB_PATH_MAX + 256];
		snprintf(full, sizeof(full), "%s/%s", dirpath, ent->d_name);
		struct stat st;
		if (stat(full, &st) != 0)
			continue;
		if (S_ISDIR(st.st_mode)) {
			lib_scan_dir(full);
		} else if (S_ISREG(st.st_mode) && af_is_supported(ent->d_name)) {
			LIBEntry *e = &g_lib[g_lib_count++];
			strncpy(e->path, full, sizeof(e->path) - 1);
			e->path[sizeof(e->path) - 1] = '\0';
			snprintf(e->name, sizeof(e->name), "%s", ent->d_name);
			e->bpm = 0.0f;
			e->tag_title[0] = '\0';
			e->tag_artist[0] = '\0';
			e->tag_key[0] = '\0';
		}
	}
	closedir(d);
}

static void *lib_scan_thread(void *arg)
{
	(void)arg;
	g_lib_count = 0;
	g_lib_sel = 0;
	g_lib_scroll = 0;

	lib_scan_dir(g_lib_root);

	/* Sort initially by name */
	lib_apply_sort();

	/* Read tags for all files */
	for (int i = 0; i < g_lib_count; i++) {
		LIBEntry *e = &g_lib[i];
		read_tags(e->path, e->tag_title, sizeof(e->tag_title),
		          e->tag_artist, sizeof(e->tag_artist));
		/* Authoritative: check djcmd sidecar first */
		e->bpm = cache_get_bpm(e->path);
	}

	/* Try to pull BPMs from mixxxdb -- match by full path -- as fallback */
	{
		const char *home = getenv("HOME");
		if (home) {
			char db_path[1024];
			snprintf(db_path, sizeof(db_path),
			         "%s/.mixxx/mixxxdb.sqlite", home);
			sqlite3 *db = NULL;
			if (sqlite3_open_v2(db_path, &db,
			                    SQLITE_OPEN_READONLY |
			                        SQLITE_OPEN_NOMUTEX,
			                    NULL) == SQLITE_OK) {
				const char *sql =
				    "SELECT tl.location, l.bpm, l.key FROM "
				    "library l JOIN track_locations tl ON "
				    "tl.id = l.location WHERE l.mixxx_deleted "
				    "= 0;";
				sqlite3_stmt *stmt = NULL;
				if (sqlite3_prepare_v2(db, sql, -1, &stmt,
				                       NULL) == SQLITE_OK) {
					while (sqlite3_step(stmt) ==
					       SQLITE_ROW) {
						const char *loc = (const char *)
						    sqlite3_column_text(stmt,
						                        0);
						float bpm = (float)
						    sqlite3_column_double(stmt,
						                          1);
						const char *key = (const char *)
						    sqlite3_column_text(stmt,
						                        2);
						if (!loc) {
							continue;
						}
						for (int i = 0; i < g_lib_count;
						     i++) {
							if (strcmp(
							        g_lib[i].path,
							        loc) == 0) {
								if (g_lib[i].bpm <=
								        0.0f &&
								    bpm >
								        0.0f) {
									g_lib[i]
									    .bpm =
									    bpm;
								}
								if (!g_lib[i]
								         .tag_key
								             [0] &&
								    key) {
									strncpy(
									    g_lib[i]
									        .tag_key,
									    key,
									    sizeof(
									        g_lib[i]
									            .tag_key) -
									        1);
								}
								break;
							}
						}
					}
					sqlite3_finalize(stmt);
				}
				sqlite3_close(db);
			}
		}
	}

	g_lib_scanning = 0;
	return NULL;
}

void lib_start_scan(const char *root)
{
	if (g_lib_scanning) {
		return; /* scan already in progress */
	}
	if (!g_lib) {
		g_lib = malloc(LIB_MAX * sizeof(LIBEntry));
	}
	if (!g_lib) {
		return;
	}
	strncpy(g_lib_root, root, sizeof(g_lib_root) - 1);
	g_lib_root[sizeof(g_lib_root) - 1] = '\0';
	g_lib_scanning = 1;
	pthread_t tid;
	pthread_create(&tid, NULL, lib_scan_thread, NULL);
	pthread_detach(tid);
}

/* ── Library sort comparators (LIBEntry-typed) ── */
static int lib_cmp_name(const void *a, const void *b)
{
	return strcasecmp(((const LIBEntry *)a)->name,
	                  ((const LIBEntry *)b)->name);
}
static int lib_cmp_bpm_asc(const void *a, const void *b)
{
	float ba = ((const LIBEntry *)a)->bpm, bb = ((const LIBEntry *)b)->bpm;
	if (ba == 0.0f && bb == 0.0f) {
		return strcasecmp(((const LIBEntry *)a)->name,
		                  ((const LIBEntry *)b)->name);
	}
	if (ba == 0.0f) {
		return 1;
	}
	if (bb == 0.0f) {
		return -1;
	}
	return (ba < bb) ? -1 : (ba > bb) ? 1 : 0;
}
static int lib_cmp_bpm_desc(const void *a, const void *b)
{
	return lib_cmp_bpm_asc(b, a);
}

void lib_apply_sort(void)
{
	if (g_lib_count < 2) {
		return;
	}
	static int (*cmps[])(const void *, const void *) = {
	    lib_cmp_name, lib_cmp_bpm_asc, lib_cmp_bpm_desc};
	int sort = g_lib_sort;
	if (sort < 0 || sort > 2) {
		sort = 0;
	}
	qsort(g_lib, g_lib_count, sizeof(LIBEntry), cmps[sort]);
}

/* ── MusicBrainz Tag Lookup ─────────────────────────────────────────── */
static int json_str(const char *json, const char *key, char *out, int max)
{
	char search[64];
	snprintf(search, sizeof(search), "\"%s\":", key);
	const char *p = strstr(json, search);
	if (!p) {
		return 0;
	}
	p += strlen(search);
	while (*p == ' ') {
		p++;
	}
	if (*p == '"') {
		p++;
		int i = 0;
		while (*p && *p != '"' && i < max - 1) {
			if (*p == '\\') {
				p++; /* skip escape char */
			}
			if (*p) {
				out[i++] = *p++;
			}
		}
		out[i] = '\0';
		return i > 0;
	}
	return 0;
}

static void *tag_lookup_thread(void *arg)
{
	(void)arg;

	/* Build a clean search term from the filename:
	 * Strip extension, replace _ and - with spaces. */
	char term[256];
	snprintf(term, sizeof(term), "%s", g_tag_info.query_name);
	term[255] = '\0';
	/* Remove extension */
	char *dot = strrchr(term, '.');
	if (dot) {
		*dot = '\0';
	}
	/* Replace underscores/dashes with spaces */
	for (int i = 0; term[i]; i++) {
		if (term[i] == '_' || term[i] == '-') {
			term[i] = ' ';
		}
	}
	/* Trim leading digits (track IDs like "13141123 ") */
	char *start = term;
	while (*start && (isdigit((unsigned char)*start) || *start == ' ')) {
		start++;
	}
	if (*start == '\0') {
		start = term; /* all digits -- use full name */
	}

	/* URL-encode the search term (basic: replace space with +) */
	char encoded[512];
	int ei = 0;
	for (int i = 0; start[i] && ei < 510; i++) {
		unsigned char c = (unsigned char)start[i];
		if (c == ' ') {
			encoded[ei++] = '+';
		} else if (isalnum(c) || c == '.' || c == '-' || c == '_') {
			encoded[ei++] = (char)c;
		} else {
			ei += snprintf(encoded + ei, sizeof(encoded) - ei,
			               "%%%02X", c);
		}
	}
	encoded[ei] = '\0';

	/* MusicBrainz recording search -- returns JSON */
	char url[768];
	snprintf(url, sizeof(url),
	         "https://musicbrainz.org/ws/2/recording"
	         "?query=%s&limit=1&fmt=json",
	         encoded);

	/* Use curl via popen -- no libcurl dependency needed */
	char cmd[1024];
	snprintf(cmd, sizeof(cmd),
	         "curl -s --max-time 8 -A 'djcmd/1.0 (linux)' '%s' 2>/dev/null",
	         url);

	FILE *fp = popen(cmd, "r");
	if (!fp) {
		snprintf(g_tag_info.status, sizeof(g_tag_info.status),
		         "curl not available");
		return NULL;
	}

	char buf[8192];
	int len = 0;
	int nr;
	while ((nr = (int)fread(buf + len, 1, sizeof(buf) - len - 1, fp)) > 0) {
		len += nr;
	}
	buf[len] = '\0';
	pclose(fp);

	if (len == 0) {
		snprintf(g_tag_info.status, sizeof(g_tag_info.status),
		         "No response");
		return NULL;
	}

	/* Parse key fields from the JSON */
	/* recordings[0].title */
	char title[128] = "";
	char artist[128] = "";
	char album[128] = "";
	char date[32] = "";
	char label[128] = "";

	json_str(buf, "title", title, sizeof(title));

	/* Artist: recordings[0].artist-credit[0].artist.name */
	const char *ac = strstr(buf, "\"artist-credit\"");
	if (ac) {
		const char *an = strstr(ac, "\"name\"");
		if (an) {
			char tmp[128] = "";
			/* Re-use json_str trick inline */
			const char *p = an + 6; /* skip "name": */
			while (*p == ':' || *p == ' ') {
				p++;
			}
			if (*p == '"') {
				p++;
				int i = 0;
				while (*p && *p != '"' && i < 127) {
					if (*p == '\\') {
						p++;
					}
					if (*p) {
						tmp[i++] = *p++;
					}
				}
				tmp[i] = '\0';
				snprintf(artist, sizeof(artist), "%s", tmp);
			}
		}
	}

	/* Release (album) title */
	const char *rel = strstr(buf, "\"releases\"");
	if (rel) {
		const char *rt = strstr(rel, "\"title\"");
		if (rt) {
			const char *p = rt + 7;
			while (*p == ':' || *p == ' ') {
				p++;
			}
			if (*p == '"') {
				p++;
				int i = 0;
				while (*p && *p != '"' && i < 127) {
					if (*p == '\\') {
						p++;
					}
					if (*p) {
						album[i++] = *p++;
					}
				}
				album[i] = '\0';
			}
		}
		json_str(rel, "date", date, sizeof(date));
		/* Label */
		const char *li = strstr(rel, "\"label-info\"");
		if (li) {
			const char *ln = strstr(li, "\"name\"");
			if (ln) {
				const char *p = ln + 6;
				while (*p == ':' || *p == ' ') {
					p++;
				}
				if (*p == '"') {
					p++;
					int i = 0;
					while (*p && *p != '"' && i < 127) {
						if (*p == '\\') {
							p++;
						}
						if (*p) {
							label[i++] = *p++;
						}
					}
					label[i] = '\0';
				}
			}
		}
	}

	if (title[0] == '\0') {
		snprintf(g_tag_info.status, sizeof(g_tag_info.status),
		         "Not found");
	} else {
		snprintf(g_tag_info.title, sizeof(g_tag_info.title), "%s",
		         title);
		snprintf(g_tag_info.artist, sizeof(g_tag_info.artist), "%s",
		         artist);
		snprintf(g_tag_info.album, sizeof(g_tag_info.album), "%s",
		         album);
		snprintf(g_tag_info.date, sizeof(g_tag_info.date), "%s", date);
		snprintf(g_tag_info.label, sizeof(g_tag_info.label), "%s",
		         label);
		g_tag_info.status[0] = '\0';
	}
	return NULL;
}

void tag_lookup_start(const char *filename)
{
	snprintf(g_tag_info.query_name, sizeof(g_tag_info.query_name), "%s",
	         filename);
	g_tag_info.query_name[255] = '\0';
	g_tag_info.title[0] = '\0';
	g_tag_info.artist[0] = '\0';
	g_tag_info.album[0] = '\0';
	g_tag_info.date[0] = '\0';
	g_tag_info.label[0] = '\0';
	snprintf(g_tag_info.status, sizeof(g_tag_info.status),
	         "Searching\u2026");
	g_tag_info.visible = 1;

	pthread_t tid;
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	pthread_create(&tid, &attr, tag_lookup_thread, NULL);
	pthread_attr_destroy(&attr);
}

/* ── Crate Management ───────────────────────────────────────────────── */
void crates_load(void)
{
	const char *home = getenv("HOME");
	if (!home) {
		return;
	}
	if (!g_machine_id[0]) {
		usb_get_machine_id(g_machine_id, sizeof(g_machine_id));
	}
	g_ncrate = 0;
	memset(g_crates, 0, sizeof(g_crates));
	char cfg[1024];
	snprintf(cfg, sizeof(cfg), "%s/.config/djcmd/crates.txt", home);
	FILE *f = fopen(cfg, "r");
	if (!f) {
		f = fopen("crates.txt", "r");
	}
	if (f) {
		char line[1024];
		while (fgets(line, sizeof(line), f) && g_ncrate < MAX_CRATES) {
			char alias[32], path[FB_PATH_MAX];
			if (sscanf(line, "%31s %1023[^\n]", alias, path) == 2) {
				snprintf(g_crates[g_ncrate].alias,
				         sizeof(g_crates[g_ncrate].alias), "%s",
				         alias);
				snprintf(g_crates[g_ncrate].path,
				         sizeof(g_crates[g_ncrate].path),
				         "%.511s", path);
				strncpy(g_crates[g_ncrate].name, alias, 63);
				g_ncrate++;
			}
		}
		fclose(f);
	}
}

void crate_vrow_resolve(int vrow, int *type, int *idx)
{
	int row = 0;
	for (int g = 0; g < g_ncrate_groups; g++) {
		if (row == vrow) {
			*type = 1;
			*idx = g;
			return;
		}
		row++;
		for (int i = 0; i < g_crate_groups[g].count; i++) {
			if (row == vrow) {
				*type = 0;
				*idx = g_crate_groups[g].start_idx + i;
				return;
			}
			row++;
		}
	}
	*type = -1;
	*idx = -1;
}

int crate_total_vrows(void)
{
	return g_ncrate_groups + g_ncrate;
}

void crate_view_open(int idx)
{
	if (idx < 0 || idx >= g_ncrate) {
		return;
	}

	if (!g_crate_tracks) {
		g_crate_tracks = calloc(MAX_CRATE_ENTRIES, sizeof(CrateEntry));
	}

	FILE *f = fopen(g_crates[idx].filename, "r");
	if (!f) {
		return;
	}

	/* Save selection if we are just refreshing the same crate */
	int old_sel = g_crate_tracks_sel;
	int old_scroll = g_crate_tracks_scroll;
	int refreshing = (g_crate_view_level == 1 &&
	                  strcmp(g_active_crate_name, g_crates[idx].name) == 0);

	g_crate_tracks_count = 0;
	int is_usb_crate = g_crates[idx].is_usb;
	char line[FB_PATH_MAX + 512];
	while (fgets(line, sizeof(line), f) &&
	       g_crate_tracks_count < MAX_CRATE_ENTRIES) {
		line[strcspn(line, "\r\n")] = '\0';
		if (!line[0] || line[0] == '#') {
			continue; /* skip v2 header lines */
		}

		CrateEntry *e = &g_crate_tracks[g_crate_tracks_count++];

		/* USB crates store paths relative to <mount>/djcmd-usb/ */
		if (is_usb_crate) {
			snprintf(e->path, sizeof(e->path), "%.200s/%s/%.250s",
			         g_crates[idx].mount_point, DJCMD_USB_DIR,
			         line);
		} else {
			strncpy(e->path, line, sizeof(e->path) - 1);
			e->path[sizeof(e->path) - 1] = '\0';
		}

		/* basename for display */
		char *bn = strrchr(e->path, '/');
		strncpy(e->name, bn ? bn + 1 : e->path, sizeof(e->name) - 1);
		e->name[sizeof(e->name) - 1] = '\0';
		e->bpm = 0.0f;
		e->tag_key[0] = '\0';
	}
	fclose(f);

	/* Authoritative: check djcmd sidecars first */
	for (int i = 0; i < g_crate_tracks_count; i++) {
		g_crate_tracks[i].bpm = cache_get_bpm(g_crate_tracks[i].path);
	}

	/* Fallback: Annotate from Mixxx DB if possible */
	const char *home = getenv("HOME");
	if (home) {
		char db_path[1024];
		snprintf(db_path, sizeof(db_path), "%s/.mixxx/mixxxdb.sqlite",
		         home);
		sqlite3 *db = NULL;
		if (sqlite3_open_v2(db_path, &db,
		                    SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX,
		                    NULL) == SQLITE_OK) {
			const char *sql =
			    "SELECT tl.location, l.bpm, l.key FROM library l "
			    "JOIN track_locations tl ON tl.id = l.location "
			    "WHERE l.mixxx_deleted = 0;";
			sqlite3_stmt *stmt = NULL;
			if (sqlite3_prepare_v2(db, sql, -1, &stmt, NULL) ==
			    SQLITE_OK) {
				while (sqlite3_step(stmt) == SQLITE_ROW) {
					const char *loc =
					    (const char *)sqlite3_column_text(
					        stmt, 0);
					float bpm =
					    (float)sqlite3_column_double(stmt,
					                                 1);
					const char *key =
					    (const char *)sqlite3_column_text(
					        stmt, 2);
					if (!loc) {
						continue;
					}
					for (int i = 0;
					     i < g_crate_tracks_count; i++) {
						if (strcmp(
						        g_crate_tracks[i].path,
						        loc) == 0) {
							/* Update BPM if missing
							 */
							if (g_crate_tracks[i]
							            .bpm <=
							        0.0f &&
							    bpm > 0.0f) {
								g_crate_tracks
								    [i]
								        .bpm =
								    bpm;
							}
							/* Always update key if
							 * missing */
							if (!g_crate_tracks[i]
							         .tag_key[0] &&
							    key) {
								strncpy(
								    g_crate_tracks
								        [i]
								            .tag_key,
								    key,
								    sizeof(
								        g_crate_tracks
								            [i]
								                .tag_key) -
								        1);
							}
							break;
						}
					}
				}
				sqlite3_finalize(stmt);
			}
			sqlite3_close(db);
		}
	}

	g_crate_view_level = 1;
	if (refreshing) {
		g_crate_tracks_sel = old_sel;
		g_crate_tracks_scroll = old_scroll;
	} else {
		g_crate_tracks_sel = 0;
		g_crate_tracks_scroll = 0;
	}
	strncpy(g_active_crate_name, g_crates[idx].name,
	        sizeof(g_active_crate_name) - 1);
}

void crate_jump(const char *alias)
{
	for (int i = 0; i < g_ncrate; i++) {
		if (strcasecmp(g_crates[i].alias, alias) == 0) {
			strncpy(g_fb_path, g_crates[i].path, FB_PATH_MAX - 1);
			fb_scan();
			g_panel = 0;
			g_view = 1;
			break;
		}
	}
}

void crate_create(const char *name)
{
	const char *home = getenv("HOME");
	if (!home) {
		return;
	}

	/* Create the .crate collection file */
	char path[1024];
	snprintf(path, sizeof(path), "%s/.config/djcmd/crates/%s.crate", home,
	         name);

	FILE *f = fopen(path, "w");
	if (!f) {
		snprintf(g_fb_status, sizeof(g_fb_status),
		         "Error creating crate '%s'", name);
		return;
	}
	/* Write v2 header so the origin is recorded from creation */
	fprintf(f, "%s\n", DJCMD_CRATE_HDR);
	fprintf(f, "# origin: %s\n", g_machine_id);
	fclose(f);

	/* If a directory was selected, also register it as a jump alias in
	 * crates.txt */
	if (g_crate_add_path[0]) {
		char cfg[1024];
		snprintf(cfg, sizeof(cfg), "%s/.config/djcmd/crates.txt", home);
		FILE *ct = fopen(cfg, "a");
		if (ct) {
			fprintf(ct, "%s %s\n", name, g_crate_add_path);
			fclose(ct);
		}
		g_crate_add_path[0] = '\0';
	}

	crates_load();
	snprintf(g_fb_status, sizeof(g_fb_status), "Crate '%s' created", name);
}

void crate_add_to(int crate_idx, const char *track_path)
{
	if (crate_idx < 0 || crate_idx >= g_ncrate) {
		return;
	}

	FILE *f = fopen(g_crates[crate_idx].filename, "a");
	if (!f) {
		return;
	}

	fprintf(f, "%s\n", track_path);
	fclose(f);

	snprintf(g_fb_status, sizeof(g_fb_status), "Added to crate '%s'",
	         g_crates[crate_idx].name);

	/* If this crate is currently open in the UI, refresh it immediately */
	if (g_crate_view_level == 1 &&
	    strcmp(g_active_crate_name, g_crates[crate_idx].name) == 0) {
		crate_view_open(crate_idx);
	}
}

void crate_remove_at(int crate_idx, int track_idx)
{
	if (crate_idx < 0 || crate_idx >= g_ncrate) {
		return;
	}
	if (track_idx < 0 || track_idx >= g_crate_tracks_count) {
		return;
	}

	char tmp_path[1024];
	snprintf(tmp_path, sizeof(tmp_path), "%s.tmp",
	         g_crates[crate_idx].filename);

	FILE *fin = fopen(g_crates[crate_idx].filename, "r");
	if (!fin) {
		return;
	}
	FILE *fout = fopen(tmp_path, "w");
	if (!fout) {
		fclose(fin);
		return;
	}

	char line[2048];
	int current = 0;
	while (fgets(line, sizeof(line), fin)) {
		if (current != track_idx) {
			fputs(line, fout);
		}
		current++;
	}

	fclose(fin);
	fclose(fout);

	rename(tmp_path, g_crates[crate_idx].filename);
	snprintf(g_fb_status, sizeof(g_fb_status), "Removed from crate '%s'",
	         g_crates[crate_idx].name);

	/* Refresh the UI if this crate is currently open */
	if (g_crate_view_level == 1 &&
	    strcmp(g_active_crate_name, g_crates[crate_idx].name) == 0) {
		crate_view_open(crate_idx);
	}
}

void update_crate_matches(const char *input, const char *prefix)
{
	g_ncrate_matches = 0;
	if (input[0]) {
		for (int i = 0; i < g_ncrate; i++) {
			/* Match against alias (for jump) or name (for add) */
			if (strncasecmp(g_crates[i].alias, input,
			                strlen(input)) == 0 ||
			    strncasecmp(g_crates[i].name, input,
			                strlen(input)) == 0) {
				g_crate_matches[g_ncrate_matches++] = i;
				if (g_ncrate_matches >= MAX_CRATES) {
					break;
				}
			}
		}
	}

	/* Build visual hint string: "Prefix: input | Match1, Match2..." */
	char hint[256];
	snprintf(hint, sizeof(hint), "%s %s_", prefix, input);
	if (g_ncrate_matches > 0) {
		strncat(hint, "  Matches: ", sizeof(hint) - strlen(hint) - 1);
		for (int i = 0; i < g_ncrate_matches && i < 5; i++) {
			int cidx = g_crate_matches[i];
			/* For jump mode use alias, for add mode use name */
			const char *name = (g_crate_jump_active)
			                       ? g_crates[cidx].alias
			                       : g_crates[cidx].name;
			if (i == g_crate_cycle_idx) {
				strncat(hint, "[",
				        sizeof(hint) - strlen(hint) - 1);
				strncat(hint, name,
				        sizeof(hint) - strlen(hint) - 1);
				strncat(hint, "]",
				        sizeof(hint) - strlen(hint) - 1);
			} else {
				strncat(hint, name,
				        sizeof(hint) - strlen(hint) - 1);
			}
			if (i < g_ncrate_matches - 1 && i < 4) {
				strncat(hint, ", ",
				        sizeof(hint) - strlen(hint) - 1);
			}
		}
		if (g_ncrate_matches > 5) {
			strncat(hint, "...", sizeof(hint) - strlen(hint) - 1);
		}
	}
	strncpy(g_fb_status, hint, sizeof(g_fb_status) - 1);
	g_fb_status[sizeof(g_fb_status) - 1] = '\0';
}

/* ── Grid & BPM ─────────────────────────────────────────────────────── */
void snap_grid(int deck)
{
	if (deck < 0 || deck >= MAX_TRACKS) {
		return;
	}
	Track *t = &g_tracks[deck];
	if (!t->loaded) {
		return;
	}

	pthread_mutex_lock(&t->lock);
	/* Set first beat offset to current position */
	t->bpm_offset = (float)t->pos;
	pthread_mutex_unlock(&t->lock);
	sidecar_save(t);
	snprintf(g_fb_status, sizeof(g_fb_status),
	         "Grid snapped to Deck %c pos", DECK_NUM(deck));
}

void tap_bpm(int deck)
{
	if (deck < 0 || deck >= MAX_TRACKS) {
		return;
	}
	Track *t = &g_tracks[deck];
	if (!t->loaded) {
		return;
	}

	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	int64_t now_ms = (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;

	/* Reset taps if idle for 2 seconds */
	if (g_tap_count[deck] > 0) {
		int last = (g_tap_idx[deck] + MAX_TAPS - 1) % MAX_TAPS;
		if (now_ms - g_tap_ms[deck][last] > 2000) {
			g_tap_count[deck] = 0;
		}
	}

	g_tap_ms[deck][g_tap_idx[deck]] = now_ms;
	g_tap_idx[deck] = (g_tap_idx[deck] + 1) % MAX_TAPS;
	if (g_tap_count[deck] < MAX_TAPS) {
		g_tap_count[deck]++;
	}

	if (g_tap_count[deck] >= 2) {
		/* Calculate average interval over all captured taps */
		int64_t sum = 0;
		int n = g_tap_count[deck] - 1;
		for (int i = 0; i < n; i++) {
			int i1 =
			    (g_tap_idx[deck] + MAX_TAPS - 1 - i) % MAX_TAPS;
			int i0 =
			    (g_tap_idx[deck] + MAX_TAPS - 2 - i) % MAX_TAPS;
			sum += (g_tap_ms[deck][i1] - g_tap_ms[deck][i0]);
		}
		float avg_ms = (float)sum / (float)n;
		float new_bpm = 60000.0f / avg_ms;

		pthread_mutex_lock(&t->lock);
		t->bpm = new_bpm;
		pthread_mutex_unlock(&t->lock);
		sidecar_save(t);
		snprintf(g_fb_status, sizeof(g_fb_status),
		         "Deck %c BPM Tap: %.1f", DECK_NUM(deck),
		         (double)new_bpm);
	} else {
		snprintf(g_fb_status, sizeof(g_fb_status), "Deck %c Tapping...",
		         DECK_NUM(deck));
	}
}

/* ── Settings ───────────────────────────────────────────────────────── */
static void settings_path(char *out, size_t max)
{
	const char *home = getenv("HOME");
	if (!home) {
		home = "/tmp";
	}
	snprintf(out, max, "%s/" CFG_CONFIG_DIR "/settings", home);
}

void settings_save(void)
{
	char path[512];
	settings_path(path, sizeof(path));
	/* Ensure directory */
	char dir[512];
	snprintf(dir, sizeof(dir), "%s", path);
	char *sl = strrchr(dir, '/');
	if (sl) {
		*sl = '\0';
		mkdir(dir, 0755);
	}

	FILE *f = fopen(path, "w");
	if (!f) {
		return;
	}
	fprintf(f, "# djcmd settings -- auto-saved on every option change\n");
	fprintf(f, "master_vol       = %d\n", g_master_vol);
	fprintf(f, "deck_vol         = %.3f\n",
	        (double)g_opts.default_deck_vol);
	fprintf(f, "auto_gain        = %d\n", g_opts.auto_gain_default);
	fprintf(f, "auto_gain_db     = %.1f\n",
	        (double)g_opts.auto_gain_target_db);
	fprintf(f, "wfm_visible_secs = %.1f\n",
	        (double)g_opts.wfm_visible_secs);
	fprintf(f, "wfm_overview_bins= %d\n", g_opts.wfm_overview_bins);
	fprintf(f, "kick_threshold   = %.2f\n", (double)g_opts.kick_threshold);
	fprintf(f, "wfm_height_gamma = %.2f\n",
	        (double)g_opts.wfm_height_gamma);
	fprintf(f, "theme            = %d\n", g_opts.theme_idx);
	fprintf(f, "wfm_style        = %d\n", g_opts.wfm_style);
	fprintf(f, "sync_quantize    = %d\n", g_opts.sync_quantize);
	fprintf(f, "sync_smart_range = %d\n", g_opts.sync_smart_range);
	fprintf(f, "sync_auto_handoff= %d\n", g_opts.sync_auto_handoff);
	fprintf(f, "key_lock_default = %d\n", g_opts.key_lock_default);
	fprintf(f, "vinyl_mode       = %d\n", g_opts.vinyl_mode);
	fprintf(f, "ui_fps           = %d\n", g_opts.ui_fps);
	fprintf(f, "wfm_lo_weight    = %.2f\n", (double)g_opts.wfm_lo_weight);
	fprintf(f, "wfm_mid_weight   = %.2f\n", (double)g_opts.wfm_mid_weight);
	fprintf(f, "wfm_hi_weight    = %.2f\n", (double)g_opts.wfm_hi_weight);
	fprintf(f, "wfm_color_sat    = %.2f\n", (double)g_opts.wfm_color_sat);
	fprintf(f, "wfm_color_floor  = %.3f\n", (double)g_opts.wfm_color_floor);
	fprintf(f, "wfm_anchor       = %d\n", g_opts.wfm_anchor);
	fprintf(f, "eco_mode         = %d\n", g_opts.eco_mode);
	fprintf(f, "library_autoplay = %d\n", g_opts.library_autoplay);
	fprintf(f, "period_frames    = %d\n", g_opts.period_frames);
	fprintf(f, "buffer_periods   = %d\n", g_opts.buffer_periods);
	fprintf(f, "num_tracks       = %d\n", g_num_tracks);
	fprintf(f, "pitch_range_a    = %d\n", g_pitch_range[0]);
	fprintf(f, "pitch_range_b    = %d\n", g_pitch_range[1]);
	fprintf(f, "pitch_range_c    = %d\n", g_pitch_range[2]);
	fprintf(f, "pitch_range_d    = %d\n", g_pitch_range[3]);
	fprintf(f, "crossfader       = %.4f\n", (double)g_crossfader);
	fprintf(f, "active_track     = %d\n", g_active_track);
	fprintf(f, "view             = %d\n", g_view);
	fprintf(f, "panel            = %d\n", g_panel);
	fprintf(f, "fb_sort          = %d\n", g_fb_sort);
	fprintf(f, "lib_sort         = %d\n", g_lib_sort);
	fprintf(f, "gang_mode        = %d\n", g_gang_mode);
	fprintf(f, "gang_mask        = %d\n", g_gang_mask);
	if (g_lib_root[0]) {
		fprintf(f, "lib_root         = %s\n", g_lib_root);
	}
	fprintf(f, "pcm_dev          = %s\n", g_pcm_dev_str);
	fprintf(f, "hp_pcm_dev       = %s\n", g_pcm_hp_dev_str);
	fprintf(f, "hp_vol           = %d\n", g_hp_vol);
	fprintf(f, "batch_bpm_lo     = %s\n", g_batch_lo_buf);
	fprintf(f, "batch_bpm_hi     = %s\n", g_batch_hi_buf);
	fclose(f);
}

void settings_load(void)
{
	char path[512];
	settings_path(path, sizeof(path));
	FILE *f = fopen(path, "r");
	if (!f) {
		return; /* first run -- keep compiled-in defaults */
	}

	char line[256];
	while (fgets(line, sizeof(line), f)) {
		/* strip comment and trailing whitespace */
		char *hash = strchr(line, '#');
		if (hash) {
			*hash = '\0';
		}
		char key[64], val[128];
		if (sscanf(line, " %63[^= ] = %127s", key, val) != 2) {
			continue;
		}

		if (!strcmp(key, "master_vol")) {
			g_opts.default_master_vol = atoi(val);
			g_master_vol = g_opts.default_master_vol;
		} else if (!strcmp(key, "deck_vol")) {
			g_opts.default_deck_vol = (float)atof(val);
		} else if (!strcmp(key, "auto_gain")) {
			g_opts.auto_gain_default = atoi(val);
		} else if (!strcmp(key, "auto_gain_db")) {
			g_opts.auto_gain_target_db = (float)atof(val);
		} else if (!strcmp(key, "wfm_visible_secs")) {
			g_opts.wfm_visible_secs = (float)atof(val);
		} else if (!strcmp(key, "wfm_overview_bins")) {
			g_opts.wfm_overview_bins = atoi(val);
		} else if (!strcmp(key, "kick_threshold")) {
			g_opts.kick_threshold = (float)atof(val);
		} else if (!strcmp(key, "wfm_height_gamma")) {
			g_opts.wfm_height_gamma = (float)atof(val);
		} else if (!strcmp(key, "theme")) {
			g_opts.theme_idx = atoi(val);
		} else if (!strcmp(key, "wfm_style")) {
			g_opts.wfm_style = atoi(val);
		} else if (!strcmp(key, "sync_quantize")) {
			g_opts.sync_quantize = atoi(val);
		} else if (!strcmp(key, "sync_smart_range")) {
			g_opts.sync_smart_range = atoi(val);
		} else if (!strcmp(key, "sync_auto_handoff")) {
			g_opts.sync_auto_handoff = atoi(val);
		} else if (!strcmp(key, "key_lock_default")) {
			g_opts.key_lock_default = atoi(val);
		} else if (!strcmp(key, "vinyl_mode")) {
			g_opts.vinyl_mode = atoi(val);
		} else if (!strcmp(key, "ui_fps")) {
			g_opts.ui_fps = atoi(val);
		} else if (!strcmp(key, "wfm_lo_weight")) {
			g_opts.wfm_lo_weight = (float)atof(val);
		} else if (!strcmp(key, "wfm_mid_weight")) {
			g_opts.wfm_mid_weight = (float)atof(val);
		} else if (!strcmp(key, "wfm_hi_weight")) {
			g_opts.wfm_hi_weight = (float)atof(val);
		} else if (!strcmp(key, "wfm_color_sat")) {
			g_opts.wfm_color_sat = (float)atof(val);
		} else if (!strcmp(key, "wfm_color_floor")) {
			g_opts.wfm_color_floor = (float)atof(val);
		} else if (!strcmp(key, "wfm_anchor")) {
			g_opts.wfm_anchor = atoi(val);
		} else if (!strcmp(key, "eco_mode")) {
			g_opts.eco_mode = atoi(val);
		} else if (!strcmp(key, "period_frames")) {
			int pf = atoi(val);
			if (pf == 512 || pf == 1024 || pf == 2048)
				g_opts.period_frames = pf;
		} else if (!strcmp(key, "buffer_periods")) {
			int bp = atoi(val);
			if (bp >= 2 && bp <= 16)
				g_opts.buffer_periods = bp;
		} else if (!strcmp(key, "library_autoplay")) {
			g_opts.library_autoplay = atoi(val);
		} else if (!strcmp(key, "num_tracks")) {
			g_num_tracks = atoi(val);
		} else if (!strcmp(key, "pitch_range_a")) {
			g_pitch_range[0] = atoi(val);
		} else if (!strcmp(key, "pitch_range_b")) {
			g_pitch_range[1] = atoi(val);
		} else if (!strcmp(key, "pitch_range_c")) {
			g_pitch_range[2] = atoi(val);
		} else if (!strcmp(key, "pitch_range_d")) {
			g_pitch_range[3] = atoi(val);
		} else if (!strcmp(key, "crossfader")) {
			g_crossfader = (float)atof(val);
		} else if (!strcmp(key, "active_track")) {
			g_active_track = atoi(val);
		} else if (!strcmp(key, "view")) {
			g_view = atoi(val);
		} else if (!strcmp(key, "panel")) {
			g_panel = atoi(val);
		} else if (!strcmp(key, "fb_sort")) {
			g_fb_sort = atoi(val);
		} else if (!strcmp(key, "lib_sort")) {
			g_lib_sort = atoi(val);
		} else if (!strcmp(key, "gang_mode")) {
			g_gang_mode = atoi(val);
		} else if (!strcmp(key, "gang_mask")) {
			g_gang_mask = atoi(val);
		} else if (!strcmp(key, "pcm_dev")) {
			snprintf(g_pcm_dev_str, sizeof(g_pcm_dev_str), "%.63s",
			         val);
		} else if (!strcmp(key, "hp_pcm_dev")) {
			snprintf(g_pcm_hp_dev_str, sizeof(g_pcm_hp_dev_str),
			         "%.63s", val);
		} else if (!strcmp(key, "hp_vol")) {
			g_hp_vol = atoi(val);
		} else if (!strcmp(key, "batch_bpm_lo")) {
			snprintf(g_batch_lo_buf, sizeof(g_batch_lo_buf),
			         "%.15s", val);
		} else if (!strcmp(key, "batch_bpm_hi")) {
			snprintf(g_batch_hi_buf, sizeof(g_batch_hi_buf),
			         "%.15s", val);
		}
	}
	fclose(f);

	/* Clamp everything to valid ranges after load */
	if (g_opts.default_master_vol < 0) {
		g_opts.default_master_vol = 0;
	}
	if (g_opts.default_master_vol > 150) {
		g_opts.default_master_vol = 150;
	}
	if (g_opts.default_deck_vol < 0.0f) {
		g_opts.default_deck_vol = 0.0f;
	}
	if (g_opts.default_deck_vol > 1.5f) {
		g_opts.default_deck_vol = 1.5f;
	}
	if (g_opts.auto_gain_target_db < -24.0f) {
		g_opts.auto_gain_target_db = -24.0f;
	}
	if (g_opts.auto_gain_target_db > 0.0f) {
		g_opts.auto_gain_target_db = 0.0f;
	}
	if (g_opts.wfm_visible_secs < 1.0f) {
		g_opts.wfm_visible_secs = 1.0f;
	}
	if (g_opts.wfm_visible_secs > 16.0f) {
		g_opts.wfm_visible_secs = 16.0f;
	}
	if (g_opts.kick_threshold < 0.5f) {
		g_opts.kick_threshold = 0.5f;
	}
	if (g_opts.kick_threshold > 8.0f) {
		g_opts.kick_threshold = 8.0f;
	}
	if (g_opts.wfm_height_gamma < 0.2f) {
		g_opts.wfm_height_gamma = 0.2f;
	}
	if (g_opts.wfm_height_gamma > 1.5f) {
		g_opts.wfm_height_gamma = 1.5f;
	}
	if (g_opts.theme_idx < 0) {
		g_opts.theme_idx = 0;
	}
	if (g_opts.theme_idx >= THEME_COUNT) {
		g_opts.theme_idx = 0;
	}
	if (g_opts.wfm_style < 0 || g_opts.wfm_style > 1) {
		g_opts.wfm_style = 0;
	}
	if (g_opts.wfm_overview_bins != 2048 &&
	    g_opts.wfm_overview_bins != 4096 &&
	    g_opts.wfm_overview_bins != 8192) {
		g_opts.wfm_overview_bins = CFG_WFM_OVERVIEW_BINS;
	}
	/* Clamp ui_fps to valid 5-step range; round to nearest 5 */
	g_opts.ui_fps = ((g_opts.ui_fps + 2) / 5) * 5; /* round to nearest 5 */
	if (g_opts.ui_fps < 5) {
		g_opts.ui_fps = 5;
	}
	if (g_opts.ui_fps > 60) {
		g_opts.ui_fps = 60;
	}
	/* Advanced waveform */
	if (g_opts.wfm_lo_weight < 0.1f) {
		g_opts.wfm_lo_weight = 0.1f;
	}
	if (g_opts.wfm_lo_weight > 2.0f) {
		g_opts.wfm_lo_weight = 2.0f;
	}
	if (g_opts.wfm_mid_weight < 0.1f) {
		g_opts.wfm_mid_weight = 0.1f;
	}
	if (g_opts.wfm_mid_weight > 2.0f) {
		g_opts.wfm_mid_weight = 2.0f;
	}
	if (g_opts.wfm_hi_weight < 0.0f) {
		g_opts.wfm_hi_weight = 0.0f;
	}
	if (g_opts.wfm_hi_weight > 1.0f) {
		g_opts.wfm_hi_weight = 1.0f;
	}
	if (g_opts.wfm_color_sat < 0.2f) {
		g_opts.wfm_color_sat = 0.2f;
	}
	if (g_opts.wfm_color_sat > 3.0f) {
		g_opts.wfm_color_sat = 3.0f;
	}
	if (g_opts.wfm_color_floor < 0.0f) {
		g_opts.wfm_color_floor = 0.0f;
	}
	if (g_opts.wfm_color_floor > 0.15f) {
		g_opts.wfm_color_floor = 0.15f;
	}
	if (g_opts.wfm_anchor < 0 || g_opts.wfm_anchor > 1) {
		g_opts.wfm_anchor = 0;
	}
	/* num_tracks: only 2 or 4 are valid */
	if (g_num_tracks != 2 && g_num_tracks != 4) {
		g_num_tracks = 2;
	}
	/* pitch_range: 0=±8%, 1=±25%, 2=±50% */
	for (int i = 0; i < MAX_TRACKS; i++) {
		if (g_pitch_range[i] < 0 || g_pitch_range[i] > 2) {
			g_pitch_range[i] = 0;
		}
	}
	/* headphone vol */
	if (g_hp_vol < 0) {
		g_hp_vol = 0;
	}
	if (g_hp_vol > 150) {
		g_hp_vol = 150;
	}
	/* crossfader */
	if (g_crossfader < 0.0f) {
		g_crossfader = 0.0f;
	}
	if (g_crossfader > 1.0f) {
		g_crossfader = 1.0f;
	}
	/* active_track */
	if (g_active_track < 0 || g_active_track >= g_num_tracks) {
		g_active_track = 0;
	}
	/* view: 0=decks, 1=browser, 2=help -- clamp to 0-1 (help not a startup
	 * state) */
	if (g_view < 0 || g_view > 2) {
		g_view = 1;
	}
	if (g_view == 2) {
		g_view = 1; /* don't restore into help screen */
	}
	/* panel: 0=browser, 1=playlist, 2=library */
	if (g_panel < 0 || g_panel > 2) {
		g_panel = 0;
	}
	/* sort orders: 0=name/alpha, 1=BPM asc, 2=BPM desc */
	if (g_fb_sort < 0 || g_fb_sort > 2) {
		g_fb_sort = 0;
	}
	if (g_lib_sort < 0 || g_lib_sort > 2) {
		g_lib_sort = 0;
	}
	/* gang: gang_mode is bool, gang_mask limited to valid deck bits */
	g_gang_mode = (g_gang_mode != 0) ? 1 : 0;
	g_gang_mask &= (1 << MAX_TRACKS) - 1;
	/* Apply master_vol to live state (default_master_vol was already set
	 * above via the master_vol key handler, but sync again here for
	 * clarity) */
	g_master_vol = g_opts.default_master_vol;

	/* Second pass: read lib_root -- path may contain spaces so sscanf
	 * would have truncated it.  Re-scan the file looking for "lib_root =
	 * ..." */
	{
		FILE *f2 = fopen(path, "r");
		if (f2) {
			char line2[FB_PATH_MAX + 64];
			while (fgets(line2, sizeof(line2), f2)) {
				char *hash = strchr(line2, '#');
				if (hash) {
					*hash = '\0';
				}
				/* Look for "lib_root" key */
				char *p = line2;
				while (*p == ' ' || *p == '\t') {
					p++;
				}
				if (strncmp(p, "lib_root", 8) != 0) {
					continue;
				}
				p += 8;
				while (*p == ' ' || *p == '\t') {
					p++;
				}
				if (*p != '=') {
					continue;
				}
				p++;
				while (*p == ' ' || *p == '\t') {
					p++;
				}
				/* Strip trailing newline/whitespace */
				int len = (int)strlen(p);
				while (len > 0 && (p[len - 1] == '\n' ||
				                   p[len - 1] == '\r' ||
				                   p[len - 1] == ' ')) {
					p[--len] = '\0';
				}
				if (len > 0) {
					snprintf(g_lib_root, FB_PATH_MAX, "%s",
					         p);
				}
				break;
			}
			fclose(f2);
		}
	}
}

/* ── Protobuf / Mixxx Import ────────────────────────────────────────── */
static int64_t pb_read_varint(const uint8_t *buf, int len, int *pos)
{
	int64_t result = 0;
	int shift = 0;
	while (*pos < len) {
		uint8_t b = buf[(*pos)++];
		result |= (int64_t)(b & 0x7F) << shift;
		if (!(b & 0x80)) {
			return result;
		}
		shift += 7;
		if (shift >= 64) {
			return -1;
		}
	}
	return -1;
}

static double pb_read_double(const uint8_t *buf, int len, int *pos)
{
	if (*pos + 8 > len) {
		return 0.0;
	}
	uint64_t bits = 0;
	for (int i = 0; i < 8; i++) {
		bits |= (uint64_t)buf[(*pos)++] << (i * 8);
	}
	double v;
	memcpy(&v, &bits, 8);
	return v;
}

static void pb_skip(const uint8_t *buf, int len, int *pos, int wire_type)
{
	switch (wire_type) {
	case 0:
		pb_read_varint(buf, len, pos);
		break;
	case 1:
		*pos += 8;
		break;
	case 2: {
		int64_t n = pb_read_varint(buf, len, pos);
		if (n > 0 && *pos + n <= len) {
			*pos += (int)n;
		}
		break;
	}
	case 5:
		*pos += 4;
		break;
	default:
		*pos = len;
		break;
	}
}

static double pb_decode_beat(const uint8_t *buf, int len)
{
	int pos = 0;
	double frame = -1.0;
	while (pos < len) {
		int64_t tag = pb_read_varint(buf, len, &pos);
		if (tag < 0) {
			break;
		}
		int field_num = (int)(tag >> 3);
		int wire_type = (int)(tag & 7);
		if (field_num == 1 && wire_type == 1) {
			frame = pb_read_double(buf, len, &pos);
		} else {
			pb_skip(buf, len, &pos, wire_type);
		}
	}
	return frame;
}

static double pb_decode_bpm_msg(const uint8_t *buf, int len)
{
	int pos = 0;
	double bpm = 0.0;
	while (pos < len) {
		int64_t tag = pb_read_varint(buf, len, &pos);
		if (tag < 0) {
			break;
		}
		int field_num = (int)(tag >> 3);
		int wire_type = (int)(tag & 7);
		if (field_num == 1 && wire_type == 1) {
			bpm = pb_read_double(buf, len, &pos);
		} else {
			pb_skip(buf, len, &pos, wire_type);
		}
	}
	return bpm;
}

static void mixxx_decode_beats_blob(const uint8_t *blob, int blob_len,
                                    const char *version, int src_sr,
                                    MixxxMeta *out)
{
	int is_grid = (strstr(version, "BeatGrid") != NULL);
	int is_map = (strstr(version, "BeatMap") != NULL);
	if (!is_grid && !is_map) {
		return;
	}
	int pos = 0;
	if (is_grid) {
		double first_frame = -1.0, pb_bpm = 0.0;
		while (pos < blob_len) {
			int64_t tag = pb_read_varint(blob, blob_len, &pos);
			if (tag < 0) {
				break;
			}
			int field_num = (int)(tag >> 3),
			    wire_type = (int)(tag & 7);
			if (wire_type == 2) {
				int64_t sub_len =
				    pb_read_varint(blob, blob_len, &pos);
				if (sub_len < 0 || pos + sub_len > blob_len) {
					break;
				}
				if (field_num == 1) {
					first_frame = pb_decode_beat(
					    blob + pos, (int)sub_len);
				} else if (field_num == 2) {
					pb_bpm = pb_decode_bpm_msg(
					    blob + pos, (int)sub_len);
				}
				pos += (int)sub_len;
			} else {
				pb_skip(blob, blob_len, &pos, wire_type);
			}
		}
		if (first_frame >= 0.0) {
			out->bpm_offset =
			    (float)(first_frame * (double)g_actual_sample_rate /
			            (double)src_sr);
		}
		if (pb_bpm > 0.0) {
			out->bpm = (float)pb_bpm;
		}
	} else {
		uint32_t cap = 1024, count = 0;
		uint32_t *arr = malloc(cap * sizeof(uint32_t));
		if (!arr) {
			return;
		}
		while (pos < blob_len) {
			int64_t tag = pb_read_varint(blob, blob_len, &pos);
			if (tag < 0) {
				break;
			}
			int field_num = (int)(tag >> 3),
			    wire_type = (int)(tag & 7);
			if (field_num == 1 && wire_type == 2) {
				int64_t sub_len =
				    pb_read_varint(blob, blob_len, &pos);
				if (sub_len < 0 || pos + sub_len > blob_len) {
					break;
				}
				double frame =
				    pb_decode_beat(blob + pos, (int)sub_len);
				pos += (int)sub_len;
				if (frame >= 0.0) {
					uint32_t f =
					    (uint32_t)(frame *
					                   (double)
					                       g_actual_sample_rate /
					                   (double)src_sr +
					               0.5);
					if (count >= cap) {
						cap *= 2;
						uint32_t *tmp = realloc(
						    arr,
						    cap * sizeof(uint32_t));
						if (!tmp) {
							free(arr);
							return;
						}
						arr = tmp;
					}
					arr[count++] = f;
				}
			} else {
				pb_skip(blob, blob_len, &pos, wire_type);
			}
		}
		if (count >= 2) {
			uint32_t *ibi = malloc((count - 1) * sizeof(uint32_t));
			if (ibi) {
				for (uint32_t i = 0; i < count - 1; i++) {
					ibi[i] = arr[i + 1] - arr[i];
				}
				qsort(ibi, count - 1, sizeof(uint32_t),
				      (int (*)(const void *, const void *))
				          strcmp); /* lazy sort */
				float med = (float)ibi[(count - 1) / 2];
				if (med > 0.0f) {
					out->bpm = (float)g_actual_sample_rate *
					           60.0f / med;
				}
				free(ibi);
			}
			out->bpm_offset = (float)arr[0];
			out->beat_frames = arr;
			out->n_beats = count;
		} else {
			free(arr);
		}
	}
}

int mixxx_import(const char *audio_path, MixxxMeta *out)
{
	memset(out, 0, sizeof(*out));
	const char *home = getenv("HOME");
	if (!home) {
		return 0;
	}
	char db_path[1024];
	snprintf(db_path, sizeof(db_path), "%s/.mixxx/mixxxdb.sqlite", home);
	sqlite3 *db = NULL;
	if (sqlite3_open_v2(db_path, &db,
	                    SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX,
	                    NULL) != SQLITE_OK) {
		if (db) {
			sqlite3_close(db);
		}
		return 0;
	}
	int track_id = -1;
	float bpm = 0.0f;
	int sr = (int)g_actual_sample_rate;
	const char *fname = strrchr(audio_path, '/');
	fname = fname ? fname + 1 : audio_path;
	{
		const char *sqls[2] = {
		    "SELECT l.id, l.bpm, l.samplerate, l.key FROM library l "
		    "JOIN track_locations tl ON tl.id = l.location WHERE "
		    "tl.location = ? AND l.mixxx_deleted = 0 LIMIT 1;",
		    "SELECT l.id, l.bpm, l.samplerate, l.key FROM library l "
		    "JOIN track_locations tl ON tl.id = l.location WHERE "
		    "tl.filename = ? AND l.mixxx_deleted = 0 LIMIT 1;"};
		const char *params[2] = {audio_path, fname};
		for (int attempt = 0; attempt < 2 && track_id < 0; attempt++) {
			sqlite3_stmt *stmt = NULL;
			if (sqlite3_prepare_v2(db, sqls[attempt], -1, &stmt,
			                       NULL) == SQLITE_OK) {
				sqlite3_bind_text(stmt, 1, params[attempt], -1,
				                  SQLITE_STATIC);
				if (sqlite3_step(stmt) == SQLITE_ROW) {
					track_id = sqlite3_column_int(stmt, 0);
					bpm = (float)sqlite3_column_double(stmt,
					                                   1);
					sr = sqlite3_column_int(stmt, 2);
					const char *key =
					    (const char *)sqlite3_column_text(
					        stmt, 3);
					if (key) {
						strncpy(out->tag_key, key,
						        sizeof(out->tag_key) -
						            1);
					}
					if (sr <= 0) {
						sr = (int)g_actual_sample_rate;
					}
				}
			}
			if (stmt) {
				sqlite3_finalize(stmt);
			}
		}
	}
	if (track_id < 0) {
		sqlite3_close(db);
		return 0;
	}
	out->bpm = bpm;
	out->samplerate = sr;
	{
		sqlite3_stmt *stmt = NULL;
		const char *sql =
		    "SELECT hotcue, position FROM cues WHERE track_id = ? AND "
		    "type = 1 AND hotcue >= 0 ORDER BY hotcue ASC LIMIT ?;";
		if (sqlite3_prepare_v2(db, sql, -1, &stmt, NULL) == SQLITE_OK) {
			sqlite3_bind_int(stmt, 1, track_id);
			sqlite3_bind_int(stmt, 2, MAX_CUES);
			while (sqlite3_step(stmt) == SQLITE_ROW) {
				int hc = sqlite3_column_int(stmt, 0);
				float pos =
				    (float)sqlite3_column_double(stmt, 1);
				if (hc >= 0 && hc < MAX_CUES && pos >= 0.0f) {
					uint32_t frame =
					    (uint32_t)(pos *
					                   (float)
					                       g_actual_sample_rate /
					                   (float)sr +
					               0.5f);
					out->cue[hc] = frame;
					out->cue_set[hc] = 1;
				}
			}
		}
		if (stmt) {
			sqlite3_finalize(stmt);
		}
	}
	{
		sqlite3_stmt *stmt = NULL;
		const char *sql = "SELECT beats, beats_version FROM library "
		                  "WHERE id = ? LIMIT 1;";
		int got_blob = 0;
		if (sqlite3_prepare_v2(db, sql, -1, &stmt, NULL) == SQLITE_OK) {
			sqlite3_bind_int(stmt, 1, track_id);
			if (sqlite3_step(stmt) == SQLITE_ROW) {
				const void *blob = sqlite3_column_blob(stmt, 0);
				int blob_len = sqlite3_column_bytes(stmt, 0);
				const char *version =
				    (const char *)sqlite3_column_text(stmt, 1);
				if (blob && blob_len > 4 && version &&
				    version[0]) {
					mixxx_decode_beats_blob(
					    (const uint8_t *)blob, blob_len,
					    version, sr, out);
					got_blob = 1;
				}
			}
		}
		if (stmt) {
			sqlite3_finalize(stmt);
		}
		if (out->bpm <= 0.0f && bpm > 0.0f) {
			out->bpm = bpm;
		}
		if (!got_blob || out->bpm_offset == 0.0f) {
			sqlite3_stmt *stmt2 = NULL;
			const char *sql2 = "SELECT position FROM cues WHERE "
			                   "track_id = ? AND type = 0 LIMIT 1;";
			if (sqlite3_prepare_v2(db, sql2, -1, &stmt2, NULL) ==
			    SQLITE_OK) {
				sqlite3_bind_int(stmt2, 1, track_id);
				if (sqlite3_step(stmt2) == SQLITE_ROW) {
					float pos =
					    (float)sqlite3_column_double(stmt2,
					                                 0);
					if (pos >= 0.0f &&
					    out->bpm_offset == 0.0f) {
						out->bpm_offset =
						    pos *
						    (float)
						        g_actual_sample_rate /
						    (float)sr;
					}
				}
			}
			if (stmt2) {
				sqlite3_finalize(stmt2);
			}
		}
	}
	sqlite3_close(db);
	out->found = 1;
	return 1;
}

/* ──────────────────────────────────────────────
   Session Play History
   Flat set of absolute paths played this session.
   ────────────────────────────────────────────── */
#define HIST_MAX 512
static char g_hist_paths[HIST_MAX][MAX_FILENAME];
static int g_hist_count = 0;

void history_mark_played(const char *path)
{
	if (!path || !path[0] || g_hist_count >= HIST_MAX) {
		return;
	}
	for (int i = 0; i < g_hist_count; i++) {
		if (strcmp(g_hist_paths[i], path) == 0) {
			return;
		}
	}
	snprintf(g_hist_paths[g_hist_count++], MAX_FILENAME, "%s", path);
}

int history_was_played(const char *path)
{
	if (!path || !path[0]) {
		return 0;
	}
	for (int i = 0; i < g_hist_count; i++) {
		if (strcmp(g_hist_paths[i], path) == 0) {
			return 1;
		}
	}
	return 0;
}

/* ──────────────────────────────────────────────
   Session Mix Log (moved from djcmd.c)
   ────────────────────────────────────────────── */
static FILE *g_mixlog = NULL;        /* open log file, NULL = disabled */
static char g_mixlog_path[512] = ""; /* path for display in UI          */

/* Per-deck: wall-clock second when the current track was loaded.
 * 0 = no track has been logged on this deck yet this session. */
static time_t g_mixlog_load_time[MAX_TRACKS] = {0, 0, 0, 0};

/* Snapshot of tag_title/tag_artist at log time (read under lt->lock,
 * written in load_worker before the log call). */
static char g_mixlog_title[MAX_TRACKS][128];
static char g_mixlog_artist[MAX_TRACKS][128];
static char g_mixlog_file[MAX_TRACKS][MAX_FILENAME];
static float g_mixlog_bpm[MAX_TRACKS];

/* Sync master -- index of the deck others lock to (-1 = none) */
_Atomic int g_sync_leader = -1;

/* Gang mode -- bitmask of decks that receive gang commands */
int g_gang_mask = 0; /* bit 0=A, 1=B, 2=C, 3=D */
int g_gang_mode = 0; /* 0=off, 1=on             */

/* Nudge decay rate per audio period (~11ms) -- 2% pitch for ~200ms */
/* Nudge decay -- see djcmd_config.h for CFG_NUDGE_* values */

/* ──────────────────────────────────────────────
   Tag Info Panel
   Shown when user presses 'i' on a browser entry.
   We query MusicBrainz via HTTP (libcurl) using the
   filename as a best-effort search term. Falls back
   to "no network / no curl" gracefully.
   ────────────────────────────────────────────── */
TagInfo g_tag_info;

/* ──────────────────────────────────────────────
   Background Load Queue
   One worker thread; UI enqueues a path+deck and
   the worker calls load_track + BPM analysis so
   the audio thread and UI never stall.
   ────────────────────────────────────────────── */

LoadJob g_load_job;
pthread_mutex_t g_load_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t g_load_cond = PTHREAD_COND_INITIALIZER;

/* biquad_*, init_eq_coeffs, calc_auto_gain, load_sampler, load_track,
   estimate_bpm_autocorr, rebuild_waveform_and_grid -> djcmd_audio.c */

/* ──────────────────────────────────────────────
   Session Mix Log
   ────────────────────────────────────────────── */

/* Open a new dated log file.  Called once from main(). */
void mixlog_open(void)
{
	const char *home = getenv("HOME");
	if (!home) {
		return;
	}

	/* Ensure config directory exists */
	char dir[512];
	snprintf(dir, sizeof(dir), "%s/" CFG_CONFIG_DIR, home);
	mkdir(dir, 0755);

	/* Build filename: YYYY-MM-DD_HH-MM-SS.log */
	time_t now = time(NULL);
	struct tm *tm = localtime(&now);
	char fname[64];
	strftime(fname, sizeof(fname), "%Y-%m-%d_%H-%M-%S.log", tm);

	snprintf(g_mixlog_path, sizeof(g_mixlog_path),
	         "%s/" CFG_CONFIG_DIR "/%s", home, fname);

	g_mixlog = fopen(g_mixlog_path, "w");
	if (!g_mixlog) {
		return;
	}

	/* Session header */
	char ts[32];
	strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", tm);
	fprintf(
	    g_mixlog,
	    "# djcmd session log\n"
	    "# Started : %s\n"
	    "# Format  : time | deck | mm:ss | bpm | artist | title | file\n"
	    "#           (mm:ss = play time of previous track on this deck)\n"
	    "\n",
	    ts);
	fflush(g_mixlog);
}

/* Log a newly loaded track on 'deck' (0-indexed).
 * Also writes the play duration of whatever was on this deck before.
 * Must be called from load_worker (not the audio thread) after tags are read.
 */
void mixlog_track_loaded(int deck, Track *lt)
{
	if (!g_mixlog) {
		return;
	}
	if (deck < 0 || deck >= MAX_TRACKS) {
		return;
	}

	time_t now = time(NULL);

	/* Compute play time for the outgoing track (if any) */
	char playtime[32] = "--:--";
	if (g_mixlog_load_time[deck] != 0) {
		long secs = (long)(now - g_mixlog_load_time[deck]);
		if (secs < 0) {
			secs = 0;
		}
		snprintf(playtime, sizeof(playtime), "%ld:%02ld", secs / 60,
		         secs % 60);
	}

	/* Wall-clock timestamp for this new load */
	struct tm *tm = localtime(&now);
	char ts[32];
	strftime(ts, sizeof(ts), "%H:%M:%S", tm);

	/* Snapshot track info (already under lt->lock in the caller -- but we
	 * copy into our own buffers here to avoid any race on future updates)
	 */
	snprintf(g_mixlog_title[deck], sizeof(g_mixlog_title[0]), "%s",
	         lt->tag_title);
	snprintf(g_mixlog_artist[deck], sizeof(g_mixlog_artist[0]), "%s",
	         lt->tag_artist);
	snprintf(g_mixlog_file[deck], sizeof(g_mixlog_file[0]), "%s",
	         lt->filename);
	g_mixlog_bpm[deck] = lt->bpm;
	history_mark_played(lt->filename);

	/* Basename only for the file column */
	const char *base = strrchr(lt->filename, '/');
	base = base ? base + 1 : lt->filename;

	/* Friendly display: prefer artist - title, fall back to filename */
	const char *artist = lt->tag_artist[0] ? lt->tag_artist : "?";
	const char *title = lt->tag_title[0] ? lt->tag_title : base;

	fprintf(g_mixlog,
	        "%s | Deck %c | played=%-7s | %6.2f BPM | %-24s | %-32s | %s\n",
	        ts, DECK_NUM(deck), playtime, (double)lt->bpm, artist, title,
	        base);
	fflush(g_mixlog);

	/* Update load time for the incoming track */
	g_mixlog_load_time[deck] = now;
}

/* Write session footer and close.  Called from cleanup(). */
void mixlog_close(void)
{
	if (!g_mixlog) {
		return;
	}

	time_t now = time(NULL);
	struct tm *tm = localtime(&now);
	char ts[32];
	strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", tm);

	fprintf(g_mixlog, "\n# Ended: %s\n", ts);

	/* Final play-time entries for any decks still loaded */
	for (int d = 0; d < MAX_TRACKS; d++) {
		if (g_mixlog_load_time[d] == 0) {
			continue;
		}
		long secs = (long)(now - g_mixlog_load_time[d]);
		if (secs < 0) {
			secs = 0;
		}
		const char *base = strrchr(g_mixlog_file[d], '/');
		base = base ? base + 1 : g_mixlog_file[d];
		const char *artist =
		    g_mixlog_artist[d][0] ? g_mixlog_artist[d] : "?";
		const char *title =
		    g_mixlog_title[d][0] ? g_mixlog_title[d] : base;
		fprintf(
		    g_mixlog,
		    "# Deck %c final: played=%ld:%02ld | %.2f BPM | %s - %s\n",
		    DECK_NUM(d), secs / 60, secs % 60, (double)g_mixlog_bpm[d],
		    artist, title);
	}

	fclose(g_mixlog);
	g_mixlog = NULL;
}

const char *mixlog_get_path(void)
{
	return g_mixlog_path[0] ? g_mixlog_path : NULL;
}
