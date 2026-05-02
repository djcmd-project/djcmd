#define _GNU_SOURCE
#include <wchar.h>
#include "djcmd_ui.h"
#include "djcmd_fx.h"
#include "djcmd_audio.h"
#include "djcmd_help.h"
#include "djcmd_usb.h"
#include "djcmd_library.h"
#include "djcmd_midi.h"
#include <ncurses.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <sys/stat.h>

/* ── UI Global State ─────────────────────────────────────────────────── */
WINDOW *g_win_main = NULL;
WINDOW *g_win_status = NULL;
int g_rows, g_cols;
int g_active_track = 0;
int g_view = 1; /* 0=decks, 1=browser+panel, 2=help */
int g_panel = 0; /* bottom panel: 0=browser, 1=playlist */
int64_t g_lib_enc_last_ms = 0; /* ms of last lib_encoder scroll */
int g_lib_auto_switched = 0; /* 1 = view was auto-set to 1 */
int g_lib_touched = 0; /* 1 = library encoder being touched */
int g_help_scroll = 0; /* first visible line of help page */
int g_options_open = 0; /* 1 = options overlay showing */
int g_quit_pending = 0; /* 1 = quit confirm modal active */
int g_options_tab =
	0; /* 0=Info 1=Audio 2=Display 3=Waveform 4=Sync 5=Theme 6=MIDI-IN 7=MIDI-OUT */
int g_options_sel =
	0; /* selected row in current tab (MIDI IN: binding index) */
int g_options_out_sel =
	0; /* selected row in MIDI OUT tab (output binding index) */
int g_blink_tick = 0;

/* Manual BPM entry mode (activated by 'B') */
int g_bpm_entry = 0; /* 1 = collecting BPM digits */
char g_bpm_buf[8] = ""; /* digit string being typed */
int g_bpm_deck = 0; /* which deck is being edited */

/* Info-tab display cache */
char g_cpuinfo_cache[256] = "";
char g_meminfo_cache[256] = "";
time_t g_meminfo_last_t = 0;

/* Runtime-adjustable options */
Options g_opts = {
	.default_master_vol = CFG_DEFAULT_MASTER_VOL,
	.default_deck_vol = CFG_DEFAULT_DECK_VOL,
	.auto_gain_default = CFG_AUTO_GAIN_ENABLED,
	.auto_gain_target_db = CFG_AUTO_GAIN_TARGET,
	.wfm_visible_secs = CFG_WFM_VISIBLE_SECS,
	.wfm_overview_bins = CFG_WFM_OVERVIEW_BINS,
	.kick_threshold = CFG_KICK_THRESHOLD,
	.wfm_height_gamma = CFG_WFM_HEIGHT_GAMMA,
	.theme_idx = CFG_DEFAULT_THEME,
	.wfm_style = 0,
	.sync_quantize = 1,
	.sync_smart_range = 1,
	.sync_auto_handoff = 1,
	.key_lock_default = 0,
	.vinyl_mode = 1,
	.ui_fps = 20,
	.wfm_lo_weight = 0.60f,
	.wfm_mid_weight = 0.40f,
	.wfm_hi_weight = 0.15f,
	.wfm_color_sat = 1.0f,
	.wfm_color_floor = 0.06f,
	.wfm_anchor = 0,
	.eco_mode = 0,
	.library_autoplay = 0,
	.enable_slicer = 1,
};

int g_has_256 = 0;

/* ── Internal UI Logic ───────────────────────────────────────────────── */

static void wfm_normalize_bands(float lo_raw, float mi_raw, float hi_raw,
				float *lo_n, float *mi_n, float *hi_n,
				const float band_max[3]);
static int wfm_pair_256(float ch_r, float ch_g, float ch_b);
static void draw_quit_modal(void);

/* Map a 0-5 RGB cube component to nearest xterm-256 index.
 * xterm color cube: index = 16 + 36*r + 6*g + b  (r,g,b in 0..5) */
static int xterm_cube(int r, int g, int b)
{
	return 16 + 36 * r + 6 * g + b;
}

/* ── Theme table ─────────────────────────────────────────────────────── */
static const ThemeDef g_themes[THEME_COUNT] = {
	THEME_DEFAULT, THEME_AMBER,	THEME_PHOSPHOR, THEME_RED_SECTOR,
	THEME_ICE,     THEME_SYNTHWAVE, THEME_MIDNIGHT, THEME_SOLAR,
	THEME_DEEPSEA, THEME_VAMPIRE,
};

void apply_theme(int idx)
{
	if (idx < 0 || idx >= THEME_COUNT)
		idx = 0;
	const ThemeDef *t = &g_themes[idx];
	init_pair(COLOR_HEADER, t->header_fg, t->header_bg);
	init_pair(COLOR_ACTIVE, t->active_fg, t->active_bg);
	init_pair(COLOR_VU, t->vu_fg, t->vu_bg);
	init_pair(COLOR_WFM, t->header_fg, t->header_bg);
	init_pair(COLOR_STATUS, t->status_fg, t->status_bg);
	init_pair(COLOR_HOT, t->hot_fg, t->hot_bg);
	init_pair(COLOR_PLAYED, COLOR_YELLOW, -1);
	init_pair(WFM_8_GREEN, t->wfm8_lo, -1);
	init_pair(WFM_8_YELLOW, t->wfm8_mid, -1);
	init_pair(WFM_8_RED, t->wfm8_hi, -1);
	init_pair(WFM_8_LO_HI, t->wfm8_lo, t->wfm8_hi);
	init_pair(WFM_8_LO_MID, t->wfm8_lo, t->wfm8_mid);
	init_pair(WFM_8_MID_HI, t->wfm8_mid, t->wfm8_hi);
	init_pair(WFM_8_KICK, t->wfm8_lo, -1);
	init_pair(WFM_8_MID, t->wfm8_mid, -1);
	init_pair(WFM_8_HI, t->wfm8_hi, -1);
}

void init_colors(void)
{
	start_color();
	use_default_colors();
	apply_theme(g_opts.theme_idx);
	g_has_256 = (COLORS >= 256);
	if (g_has_256) {
		for (int r = 0; r < 6; r++)
			for (int g = 0; g < 6; g++)
				for (int b = 0; b < 6; b++)
					init_pair(WFM_PAIR_BASE + r * 36 +
							  g * 6 + b,
						  xterm_cube(r, g, b), -1);
	}
}

/* Draw a horizontal bar */
static void draw_bar(WINDOW *w, int y, int x, int width, float val,
		     int color_pair)
{
	int filled = (int)(val * width);
	wattron(w, COLOR_PAIR(color_pair));
	for (int i = 0; i < width; i++) {
		mvwaddch(w, y, x + i, (i < filled) ? ACS_CKBOARD : ' ');
	}
	wattroff(w, COLOR_PAIR(color_pair));
}

/* Mini waveform with beat grid overlay and cue point markers */
void draw_waveform(WINDOW *w, int y, int x, int width, Track *t)
{
	if (!t->loaded) {
		wattron(w, A_DIM);
		for (int i = 0; i < width; i++)
			mvwaddch(w, y, x + i, '-');
		wattroff(w, A_DIM);
		return;
	}

	for (int i = 0; i < width; i++) {
		float lo = 0.0f, mi = 0.0f, hi = 0.0f;
		if (t->wfm_low && t->wfm_bins > 0) {
			uint32_t bin_start =
				(uint32_t)i * t->wfm_bins / (uint32_t)width;
			uint32_t bin_end = (uint32_t)(i + 1) * t->wfm_bins /
					   (uint32_t)width;
			if (bin_end > t->wfm_bins)
				bin_end = t->wfm_bins;
			for (uint32_t b = bin_start; b < bin_end; b++) {
				float cl = t->wfm_low[b] / 255.0f;
				float cm = t->wfm_mid[b] / 255.0f;
				float ch = t->wfm_high[b] / 255.0f;
				if (cl > lo)
					lo = cl;
				if (cm > mi)
					mi = cm;
				if (ch > hi)
					hi = ch;
			}
		} else if (t->num_frames > 0) {
			uint32_t step = t->num_frames / (uint32_t)width;
			if (step < 1)
				step = 1;
			uint32_t idx2 = (uint32_t)i * step;
			lo = fabsf(t->data[idx2 * 2] / 32768.0f);
		}

		float amp = lo > mi ? lo : mi;
		amp = amp > hi ? amp : hi;

		float ln, mn, hn;
		wfm_normalize_bands(lo, mi, hi, &ln, &mn, &hn, t->wfm_band_max);

		int pair;
		if (!g_has_256) {
			if (ln >= mn && ln >= hn)
				pair = WFM_8_GREEN;
			else if (mn >= ln && mn >= hn)
				pair = WFM_8_YELLOW;
			else
				pair = WFM_8_RED;
		} else {
			pair = wfm_pair_256(ln, mn * 0.5f, hn * 0.2f);
		}

		wattron(w, COLOR_PAIR(pair));
		if (g_is_tty) {
			static const char tty_chars[] = " .:-=+*#%@";
			int bi = (int)(amp * 9.0f);
			if (bi > 9)
				bi = 9;
			mvwaddch(w, y, x + i, tty_chars[bi]);
		} else {
			int levels = (int)(amp * 4.0f);
			if (levels > 4)
				levels = 4;
			uint8_t braille = 0;
			if (levels >= 1)
				braille |= (0x40 | 0x80);
			if (levels >= 2)
				braille |= (0x04 | 0x20);
			if (levels >= 3)
				braille |= (0x02 | 0x10);
			if (levels >= 4)
				braille |= (0x01 | 0x08);

			if (braille == 0) {
				mvwaddch(w, y, x + i, ' ');
			} else {
				wchar_t blk = (wchar_t)(0x2800 + braille);
				cchar_t cc;
				setcchar(&cc, (wchar_t[]){ blk, 0 }, A_NORMAL,
					 pair, NULL);
				mvwadd_wch(w, y, x + i, &cc);
			}
		}
		wattroff(w, COLOR_PAIR(pair));
	}

	if (t->bpm > 0.0f && t->num_frames > 0) {
		float beat_frames = (g_actual_sample_rate * 60.0f) / t->bpm;
		float offset = t->bpm_offset;
		int beat_n = 0;
		for (float bf = offset; bf < (float)t->num_frames;
		     bf += beat_frames, beat_n++) {
			int col =
				(int)(bf / (float)t->num_frames * (float)width);
			if (col < 0 || col >= width)
				continue;
			int is_downbeat = ((beat_n % 4) == 0);
			if (is_downbeat) {
				mvwchgat(w, y, x + col, 1, A_REVERSE | A_BOLD,
					 COLOR_ACTIVE, NULL);
			}
		}

		if (t->synced && g_sync_leader >= 0 &&
		    g_sync_leader < MAX_TRACKS) {
			Track *master = &g_tracks[g_sync_leader];
			if (master->loaded && master->bpm > 0.0f) {
				float m_beat_f =
					(g_actual_sample_rate * 60.0f) /
					master->bpm;
				float m_phase = fmodf(
					(float)master->pos - master->bpm_offset,
					m_beat_f);
				if (m_phase < 0.0f)
					m_phase += m_beat_f;
				for (float bf = offset + m_phase;
				     bf < (float)t->num_frames;
				     bf += beat_frames) {
					int col = (int)(bf /
							(float)t->num_frames *
							(float)width);
					if (col >= 0 && col < width) {
						mvwchgat(w, y, x + col, 1,
							 A_BOLD, COLOR_VU,
							 NULL);
					}
				}
			}
		}
	}

	static const char cue_chars[] = "12345678";
	for (int ci = 0; ci < MAX_CUES; ci++) {
		if (!t->cue_set[ci])
			continue;
		int col = (int)((float)t->cue[ci] / (float)t->num_frames *
				(float)width);
		if (col >= 0 && col < width) {
			wattron(w,
				COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
			mvwaddch(w, y, x + col, cue_chars[ci]);
			wattroff(w,
				 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
		}
	}

	if (t->num_frames > 0) {
		int ph = (int)((float)t->pos / (float)t->num_frames *
			       (float)width);
		if (ph >= width)
			ph = width - 1;
		mvwchgat(w, y, x + ph, 1, A_REVERSE | A_BOLD, COLOR_HOT, NULL);
	}
}

static void draw_deck(WINDOW *w, int y, int x, int w_width, int idx)
{
	Track *t = &g_tracks[idx];
	int active = (idx == g_active_track);
	int is_leader = (idx == g_sync_leader);
	int in_gang = g_gang_mode && (g_gang_mask & (1 << idx));

	if (active)
		wattron(w, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
	else
		wattron(w, COLOR_PAIR(COLOR_HEADER) | A_BOLD);

	char flags[64] = "";
	if (is_leader)
		strcat(flags, "M");
	if (t->synced)
		strcat(flags, "S");
	if (in_gang)
		strcat(flags, "G");
	if (t->tag_key[0]) {
		if (flags[0])
			strcat(flags, " ");
		strncat(flags, t->tag_key, 15);
	}
	if (t->nudge != 0.0f)
		strcat(flags, "~");
	if (t->bpm_display_double == 1)
		strcat(flags, "\xc3\xb7"
			      "2");
	if (t->bpm_display_double == -1)
		strcat(flags, "\xc2\xbd");
	if (t->key_lock)
		strcat(flags, " KEY");
	if (t->cue_active)
		strcat(flags, " CUE");

	const char *play_status;
	if (t->pending_play)
		play_status = " \xe2\x8f\xb3 WAIT";
	else if (t->playing)
		play_status = "\xe2\x96\xb6 PLAY";
	else if (t->loaded)
		play_status = "  STOP";
	else
		play_status = " EMPTY";

	mvwprintw(w, y, x, " DECK %c %s", 'A' + idx, play_status);
	if (flags[0]) {
		int fx = x + 15;
		int avail = (x + w_width) - fx - 6;
		if (avail > 0) {
			wattron(w, A_DIM);
			mvwprintw(w, y, fx, " [%.*s] ", avail, flags);
			wattroff(w, A_DIM);
		}
	}

	float pk = t->period_peak;
	int vucolor = (pk > 0.95f) ? COLOR_HOT :
		      (pk > 0.5f)  ? COLOR_VU :
				     COLOR_HEADER;
	char vuchar = (pk > 0.95f) ? '!' : (pk > 0.1f) ? '#' : '-';
	wattron(w, COLOR_PAIR(vucolor));
	if (pk > 0.95f)
		wattron(w, A_BLINK);
	mvwaddch(w, y, x + w_width - 2, vuchar);
	if (pk > 0.95f)
		wattroff(w, A_BLINK);
	wattroff(w, COLOR_PAIR(vucolor));

	if (active)
		wattroff(w, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
	else
		wattroff(w, COLOR_PAIR(COLOR_HEADER) | A_BOLD);

	if (t->loaded) {
		char display[256];
		if (t->tag_artist[0] && t->tag_title[0])
			snprintf(display, sizeof(display), "%s \xe2\x80\x94 %s",
				 t->tag_artist, t->tag_title);
		else if (t->tag_title[0])
			snprintf(display, sizeof(display), "%s", t->tag_title);
		else {
			char *bn = strrchr(t->filename, '/');
			bn = bn ? bn + 1 : t->filename;
			snprintf(display, sizeof(display), "%s", bn);
		}
		mvwprintw(w, y + 1, x, " %-*.*s", w_width - 2, w_width - 2,
			  display);
	} else
		mvwprintw(w, y + 1, x, " %-*s", w_width - 2, "(no file)");

	draw_waveform(w, y + 2, x + 1, w_width - 2, t);

	if (t->loaded && g_actual_sample_rate > 0) {
		float cur = (float)t->pos / g_actual_sample_rate;
		float tot = (float)t->num_frames / g_actual_sample_rate;
		mvwprintw(w, y + 3, x, " %02d:%05.2f / %02d:%05.2f",
			  (int)cur / 60, fmodf(cur, 60.0f), (int)tot / 60,
			  fmodf(tot, 60.0f));
	}

	if (t->bpm > 0) {
		float disp_bpm = t->bpm * t->pitch;
		if (t->bpm_display_double == 1)
			disp_bpm *= 2.0f;
		else if (t->bpm_display_double == -1)
			disp_bpm *= 0.5f;
		const char *bpm_marker =
			(t->bpm_display_double == 1)  ? "\u00d7" :
			(t->bpm_display_double == -1) ? "\u00bd" :
							" ";
		if (t->synced)
			wattron(w, COLOR_PAIR(COLOR_HOT) | A_BOLD);
		mvwprintw(w, y + 3, x + w_width - 12, "BPM:%5.1f%s%s", disp_bpm,
			  bpm_marker, t->synced ? "*" : " ");
		if (t->synced)
			wattroff(w, COLOR_PAIR(COLOR_HOT) | A_BOLD);
	}

	float p_range = g_pitch_range_vals[g_pitch_range[idx]];
	float p_norm = (t->pitch - 1.0f) / p_range;
	if (p_norm < -1.0f)
		p_norm = -1.0f;
	if (p_norm > 1.0f)
		p_norm = 1.0f;
	mvwprintw(w, y + 4, x, " PITCH");
	draw_bar(w, y + 4, x + 7, w_width - 20, (p_norm + 1.0f) * 0.5f,
		 COLOR_VU);
	mvwprintw(w, y + 4, x + w_width - 13, "%s %+5.1f%%",
		  g_pitch_range_names[g_pitch_range[idx]],
		  (t->pitch - 1.0f) * 100.0f);

	mvwprintw(w, y + 5, x, " VOL  ");
	draw_bar(w, y + 5, x + 7, w_width - 14, t->volume, COLOR_VU);
	mvwprintw(w, y + 5, x + w_width - 7, "G%+4.1f",
		  20.0f * log10f(t->gain > 0 ? t->gain : 1e-6f));

	int eq_w = (w_width - 36) / 3;
	if (eq_w >= 1) {
		mvwprintw(w, y + 6, x, " LOW  ");
		draw_bar(w, y + 6, x + 7, eq_w, (t->eq_low + 1.0f) * 0.5f,
			 COLOR_VU);
		mvwprintw(w, y + 6, x + 7 + eq_w, "%+4.0f%%",
			  t->eq_low * 100.0f);
		mvwprintw(w, y + 6, x + 12 + eq_w, " MID  ");
		draw_bar(w, y + 6, x + 19 + eq_w, eq_w,
			 (t->eq_mid + 1.0f) * 0.5f, COLOR_VU);
		mvwprintw(w, y + 6, x + 19 + eq_w * 2, "%+4.0f%%",
			  t->eq_mid * 100.0f);
		mvwprintw(w, y + 6, x + 24 + eq_w * 2, " HIG  ");
		draw_bar(w, y + 6, x + 31 + eq_w * 2, eq_w,
			 (t->eq_high + 1.0f) * 0.5f, COLOR_VU);
		mvwprintw(w, y + 6, x + 31 + eq_w * 3, "%+4.0f%%",
			  t->eq_high * 100.0f);
	} else if (w_width >= 22)
		mvwprintw(w, y + 6, x, " L%+3.0f%% M%+3.0f%% H%+3.0f%%",
			  t->eq_low * 100.0f, t->eq_mid * 100.0f,
			  t->eq_high * 100.0f);

	int cx = x + 1;
	for (int ci = 0; ci < MAX_CUES; ci++) {
		if (t->cue_set[ci]) {
			float ct = (float)t->cue[ci] / g_actual_sample_rate;
			wattron(w, COLOR_PAIR(COLOR_ACTIVE));
			mvwprintw(w, y + 7, cx, "C%d:%02d:%04.1f ", ci + 1,
				  (int)ct / 60, fmodf(ct, 60.0f));
			wattroff(w, COLOR_PAIR(COLOR_ACTIVE));
		} else {
			wattron(w, A_DIM);
			mvwprintw(w, y + 7, cx, "C%d:--:-- ", ci + 1);
			wattroff(w, A_DIM);
		}
		cx += 10;
		if (cx + 10 > x + w_width)
			break;
	}
	if (t->looping) {
		wattron(w, COLOR_PAIR(COLOR_HOT) | A_BOLD);
		mvwprintw(w, y + 7, x + w_width - 9, " [LOOP] ");
		wattroff(w, COLOR_PAIR(COLOR_HOT) | A_BOLD);
	}

	{
		static const char s_fx_ch[11] = { '-', 'E', 'G', 'R', 'F', 'C',
						  'H', 'D', 'B', 'A', 'W' };
		char seg[FX_SLOTS_PER_DECK][8];
		int seg_active[FX_SLOTS_PER_DECK];
		for (int sl = 0; sl < FX_SLOTS_PER_DECK; sl++) {
			FXSlot *fs = fx_slot(idx, sl);
			int at = (fs->pending_type >= 0) ? fs->pending_type :
							   fs->type;
			seg_active[sl] = (at > FX_NONE && at >= 0);
			if (!seg_active[sl])
				strcpy(seg[sl], "---");
			else {
				char ch = (at < 11) ? s_fx_ch[at] : '?';
				int wet = (int)(fs->params[3] * 100.0f + 0.5f);
				if (wet > 99)
					wet = 99;
				if (wet < 0)
					wet = 0;
				snprintf(seg[sl], sizeof(seg[sl]), "%c%02d", ch,
					 wet);
			}
		}
		wattron(w, A_DIM);
		mvwprintw(w, y + 8, x, " FX:%s %s %s", seg[0], seg[1], seg[2]);
		wattroff(w, A_DIM);
		int sel = g_fx_ui_slot[idx];
		if (sel >= 0 && sel < FX_SLOTS_PER_DECK && seg_active[sel]) {
			int col = x + 4 + sel * 4;
			wattron(w, A_BOLD);
			mvwprintw(w, y + 8, col, "%s", seg[sel]);
			wattroff(w, A_BOLD);
		}
	}
}

static float col_bands(const Track *t, uint32_t frame, uint32_t frames_per_col,
		       float out[3])
{
	out[0] = out[1] = out[2] = 0.0f;
	if (t->wfm_low && t->wfm_bins > 0 && t->num_frames > 0) {
		float bin_f = (float)frame / (float)t->num_frames *
			      (float)t->wfm_bins;
		if (g_opts.eco_mode) {
			int b0 = (int)bin_f;
			if (b0 >= (int)t->wfm_bins)
				b0 = (int)t->wfm_bins - 1;
			out[0] = t->wfm_low[b0] / 255.0f;
			out[1] = t->wfm_mid[b0] / 255.0f;
			out[2] = t->wfm_high[b0] / 255.0f;
			float pk = out[0] > out[1] ? out[0] : out[1];
			if (out[2] > pk)
				pk = out[2];
			return pk;
		}
		float bin_e = (float)(frame + frames_per_col) /
			      (float)t->num_frames * (float)t->wfm_bins;
		int b0 = (int)bin_f;
		int b1 = (int)bin_e;
		if (b0 >= (int)t->wfm_bins)
			b0 = (int)t->wfm_bins - 1;
		if (b1 >= (int)t->wfm_bins)
			b1 = (int)t->wfm_bins - 1;
		float sum_pk_sq = 0.0f;
		float sum_lo = 0.0f, sum_mi = 0.0f, sum_hi = 0.0f;
		int n_bins = b1 - b0 + 1;
		for (int b = b0; b <= b1; b++) {
			float lo = t->wfm_low[b] / 255.0f;
			float mi = t->wfm_mid[b] / 255.0f;
			float hi = t->wfm_high[b] / 255.0f;
			float pk = lo > mi ? lo : mi;
			pk = pk > hi ? pk : hi;
			sum_pk_sq += pk * pk;
			sum_lo += lo;
			sum_mi += mi;
			sum_hi += hi;
		}
		float rms = sqrtf(sum_pk_sq / (float)n_bins);
		out[0] = sum_lo / (float)n_bins;
		out[1] = sum_mi / (float)n_bins;
		out[2] = sum_hi / (float)n_bins;
		return rms;
	}
	if (g_opts.eco_mode) {
		float sum_lo = 0.0f;
		uint32_t n = 0;
		for (uint32_t i = 0; i < frames_per_col; i += 4) {
			uint32_t f = frame + i;
			if (f >= t->num_frames)
				break;
			float l = t->data[f * 2] / 32768.0f;
			float r = t->data[f * 2 + 1] / 32768.0f;
			float s = fabsf((l + r) * 0.5f);
			sum_lo += s;
			n++;
		}
		if (n == 0)
			return 0.0f;
		out[0] = out[1] = out[2] = sum_lo / n;
		return out[0];
	}
	float lp_low = 0.0f, lp_4k = 0.0f;
	const float a_low = 0.0426f, a_hi4k = 0.3996f;
	uint32_t warmup = (frames_per_col < 8) ? frames_per_col : 8;
	for (uint32_t i = 0; i < warmup; i++) {
		if (frame + i >= t->num_frames)
			break;
		float s = (t->data[(frame + i) * 2] +
			   t->data[(frame + i) * 2 + 1]) *
			  (0.5f / 32768.0f);
		lp_low += a_low * (s - lp_low);
		lp_4k += a_hi4k * (s - lp_4k);
	}
	float sum_lo = 0.0f, sum_mi = 0.0f, sum_hi = 0.0f;
	uint32_t n = 0;
	for (uint32_t i = 0; i < frames_per_col; i++) {
		uint32_t f = frame + i;
		if (f >= t->num_frames)
			break;
		float s = (t->data[f * 2] + t->data[f * 2 + 1]) *
			  (0.5f / 32768.0f);
		lp_low += a_low * (s - lp_low);
		lp_4k += a_hi4k * (s - lp_4k);
		float lo = lp_low;
		float hi = s - lp_4k;
		float mi = s - lo - hi;
		sum_lo += lo * lo;
		sum_mi += mi * mi;
		sum_hi += hi * hi;
		n++;
	}
	if (n == 0)
		return 0.0f;
	out[0] = sqrtf(sum_lo / n) * 8.0f;
	out[1] = sqrtf(sum_mi / n) * 6.0f;
	out[2] = sqrtf(sum_hi / n) * 4.0f;
	float mx = out[0] > out[1] ? out[0] : out[1];
	mx = mx > out[2] ? mx : out[2];
	return mx;
}

void wfm_compute_band_max(Track *t)
{
	t->wfm_band_max[0] = 0.01f;
	t->wfm_band_max[1] = 0.01f;
	t->wfm_band_max[2] = 0.01f;
	if (!t->wfm_low || t->wfm_bins == 0)
		return;
	for (uint32_t b = 0; b < t->wfm_bins; b++) {
		float lo = t->wfm_low[b] / 255.0f;
		float mi = t->wfm_mid[b] / 255.0f;
		float hi = t->wfm_high[b] / 255.0f;
		if (lo > t->wfm_band_max[0])
			t->wfm_band_max[0] = lo;
		if (mi > t->wfm_band_max[1])
			t->wfm_band_max[1] = mi;
		if (hi > t->wfm_band_max[2])
			t->wfm_band_max[2] = hi;
	}
}

static void wfm_normalize_bands(float lo_raw, float mi_raw, float hi_raw,
				float *lo_n, float *mi_n, float *hi_n,
				const float band_max[3])
{
	float lm = band_max[0] > 0.01f ? band_max[0] : 0.01f;
	float mm = band_max[1] > 0.01f ? band_max[1] : 0.01f;
	float hm = band_max[2] > 0.01f ? band_max[2] : 0.01f;
	*lo_n = lo_raw / lm;
	if (*lo_n > 1.0f)
		*lo_n = 1.0f;
	*mi_n = mi_raw / mm;
	if (*mi_n > 1.0f)
		*mi_n = 1.0f;
	*hi_n = hi_raw / hm;
	if (*hi_n > 1.0f)
		*hi_n = 1.0f;
}

static int wfm_pair_256(float ch_r, float ch_g, float ch_b)
{
	if (g_opts.eco_mode) {
		if (ch_r > ch_g && ch_r > ch_b)
			return COLOR_HOT;
		if (ch_g > ch_b)
			return COLOR_VU;
		return COLOR_ACTIVE;
	}
	ch_r = powf(ch_r, 0.5f);
	ch_g = powf(ch_g, 0.5f);
	ch_b = powf(ch_b, 0.5f);
	int r = (int)(ch_r * 5.0f + 0.5f);
	int g = (int)(ch_g * 5.0f + 0.5f);
	int b = (int)(ch_b * 5.0f + 0.5f);
	if (r > 5)
		r = 5;
	if (g > 5)
		g = 5;
	if (b > 5)
		b = 5;
	if (r == 0 && g == 0 && b == 0) {
		r = 1;
		g = 1;
		b = 1;
	}
	return WFM_PAIR_BASE + r * 36 + g * 6 + b;
}

static void color_to_rgb5(short ncurses_color, int *r5, int *g5, int *b5)
{
	static const int tbl[8][3] = { { 0, 0, 0 }, { 4, 0, 0 }, { 0, 3, 0 },
				       { 4, 3, 0 }, { 0, 0, 4 }, { 4, 0, 4 },
				       { 0, 4, 4 }, { 5, 5, 5 } };
	int idx = (int)ncurses_color;
	if (idx < 0 || idx > 7)
		idx = 7;
	*r5 = tbl[idx][0];
	*g5 = tbl[idx][1];
	*b5 = tbl[idx][2];
}

static void draw_scrolling_waveform(WINDOW *win, int y, int x, int w,
				    int deck_idx)
{
	Track *t = &g_tracks[deck_idx];
	int is_active = (deck_idx == g_active_track);
	wattron(win, is_active ? COLOR_PAIR(COLOR_ACTIVE) | A_BOLD :
				 COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	mvwprintw(win, y, x, " DECK %c ", DECK_NUM(deck_idx));
	wattroff(win, is_active ? COLOR_PAIR(COLOR_ACTIVE) | A_BOLD :
				  COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	if (!t->loaded) {
		for (int row = 1; row <= WFM_ROWS + 2; row++) {
			wmove(win, y + row, x);
			for (int col = 0; col < w; col++)
				waddch(win, ' ');
		}
		wattron(win, A_DIM);
		mvwprintw(win, y + WFM_ROWS / 2 + 1, x + w / 2 - 5,
			  " (no track) ");
		wattroff(win, A_DIM);
		return;
	}
	float visible_frames = g_opts.wfm_visible_secs * g_actual_sample_rate;
	float frames_per_col = visible_frames / (float)w;
	int64_t left_frame = (int64_t)t->pos - (int64_t)(visible_frames * 0.5f);
	int loop_col_start = -1, loop_col_end = -1;
	if (t->looping && t->loop_end > t->loop_start) {
		loop_col_start = (int)((float)(t->loop_start - left_frame) /
				       frames_per_col);
		loop_col_end = (int)((float)(t->loop_end - left_frame) /
				     frames_per_col);
		if (loop_col_start < 0)
			loop_col_start = 0;
		if (loop_col_end >= w)
			loop_col_end = w - 1;
	}
	int slice_cols[9];
	int slicer_active = (g_pad_mode[deck_idx] == PAD_MODE_SLICER &&
			     g_opts.enable_slicer);
	if (slicer_active && t->bpm > 10.0f) {
		float beat_f = (float)g_actual_sample_rate * 60.0f / t->bpm;
		/* Round slip_pos down to the start of the current 8-beat slicer domain */
		uint32_t domain_start =
			(uint32_t)(floorf(((float)t->slip_pos - t->bpm_offset) /
					  (beat_f * 8.0f)) *
					   (beat_f * 8.0f) +
				   t->bpm_offset);
		for (int i = 0; i <= 8; i++)
			slice_cols[i] = (int)((float)((int64_t)domain_start +
						      (int64_t)(i * beat_f) -
						      left_frame) /
					      frames_per_col);
	}
	int col_step = g_opts.eco_mode ? 2 : 1;
	for (int col = 0; col < w; col += col_step) {
		int64_t fs = left_frame + (int64_t)(col * frames_per_col);
		float bands[3] = { 0, 0, 0 };
		if (fs >= 0 && fs < (int64_t)t->num_frames)
			col_bands(t, (uint32_t)fs, (uint32_t)frames_per_col,
				  bands);
		float lo, mi, hi;
		float lo_raw = bands[0], mi_raw = bands[1], hi_raw = bands[2];
		wfm_normalize_bands(lo_raw, mi_raw, hi_raw, &lo, &mi, &hi,
				    t->wfm_band_max);
		float w_sum = g_opts.wfm_lo_weight + g_opts.wfm_mid_weight +
			      g_opts.wfm_hi_weight;
		if (w_sum < 1e-3f)
			w_sum = 1e-3f;
		float disp_amp = (lo * g_opts.wfm_lo_weight +
				  mi * g_opts.wfm_mid_weight +
				  hi * g_opts.wfm_hi_weight) /
				 w_sum;
		if (disp_amp > 1.0f)
			disp_amp = 1.0f;
		float disp = powf(disp_amp, g_opts.wfm_height_gamma);
		int total_subs = WFM_ROWS * 8;
		int subs_half =
			(g_opts.wfm_anchor == 0) ? total_subs / 2 : total_subs;
		int filled_subs = (int)(disp * (float)subs_half + 0.5f);
		if (filled_subs > subs_half)
			filled_subs = subs_half;
		int centre_row =
			(g_opts.wfm_anchor == 0) ? WFM_ROWS / 2 : WFM_ROWS;
		for (int row = 0; row < WFM_ROWS; row++) {
			int screen_row = y + 1 + row;
			int above, dist;
			if (g_opts.wfm_anchor == 0) {
				above = (row < centre_row);
				dist = above ? (centre_row - 1 - row) :
					       (row - centre_row);
			} else {
				above = 1;
				dist = WFM_ROWS - 1 - row;
			}
			int subs_in_row = filled_subs - dist * 8;
			if (subs_in_row <= 0) {
				mvwaddch(win, screen_row, x + col, ' ');
				continue;
			}
			if (subs_in_row > 8)
				subs_in_row = 8;
			float v_frac = (subs_half > 0) ?
					       (float)(dist * 8 + subs_in_row) /
						       (float)subs_half :
					       0.0f;
			if (v_frac > 1.0f)
				v_frac = 1.0f;
			int pair;
			attr_t row_attr = A_NORMAL;
			if (!g_has_256) {
				if (lo >= mi)
					pair = (lo > 0.5f) ? WFM_8_KICK :
							     WFM_8_LO_MID;
				else
					pair = (mi > 0.5f) ? WFM_8_HI :
							     WFM_8_MID_HI;
				if (lo > 0.65f || mi > 0.65f)
					row_attr = A_BOLD;
			} else if (g_opts.wfm_style == 0) {
				float r_c, g_c, b_c;
				float bright = 0.5f + lo * 0.3f + mi * 0.2f;
				if (bright > 1.0f)
					bright = 1.0f;
				bright *= g_opts.wfm_color_sat;
				if (bright > 1.0f)
					bright = 1.0f;
				float raw_sum = lo_raw + mi_raw;
				float dom = (raw_sum > 1e-6f) ?
						    lo_raw / raw_sum :
						    0.5f;
				dom = (dom - 0.5f) *
					      (3.0f * g_opts.wfm_color_sat) +
				      0.5f;
				if (dom > 1.0f)
					dom = 1.0f;
				if (dom < 0.0f)
					dom = 0.0f;
				float tip_dim = 1.0f - v_frac * 0.4f;
				r_c = dom * bright * tip_dim;
				g_c = hi * 0.10f * bright * tip_dim;
				b_c = (1.0f - dom) * bright * tip_dim;
				float tot = r_c + g_c + b_c;
				float cf = g_opts.wfm_color_floor;
				if (cf > 1e-6f && tot < cf) {
					float sc = cf / (tot + 0.001f);
					r_c *= sc;
					g_c *= sc;
					b_c *= sc;
				}
				pair = wfm_pair_256(r_c, g_c, b_c);
			} else {
				const ThemeDef *th =
					&g_themes[g_opts.theme_idx];
				int kr5, kg5, kb5, sr5, sg5, sb5;
				color_to_rgb5(th->wfm8_lo, &kr5, &kg5, &kb5);
				color_to_rgb5(th->wfm8_hi, &sr5, &sg5, &sb5);
				float raw_sum = lo_raw + mi_raw;
				float dom = (raw_sum > 1e-6f) ?
						    lo_raw / raw_sum :
						    0.5f;
				dom = (dom - 0.5f) * 3.0f + 0.5f;
				if (dom > 1.0f)
					dom = 1.0f;
				if (dom < 0.0f)
					dom = 0.0f;
				float mr5 = dom * kr5 + (1.0f - dom) * sr5;
				float mg5 = dom * kg5 + (1.0f - dom) * sg5;
				float mb5 = dom * kb5 + (1.0f - dom) * sb5;
				float bright = 0.4f + lo * 0.35f + mi * 0.25f;
				if (bright > 1.0f)
					bright = 1.0f;
				bright *= g_opts.wfm_color_sat;
				if (bright > 1.0f)
					bright = 1.0f;
				float tip_dim = 1.0f - v_frac * 0.4f;
				float scale = bright * tip_dim / 5.0f;
				float r_c = mr5 * scale;
				float g_c = mg5 * scale;
				float b_c = mb5 * scale;
				float tot = r_c + g_c + b_c;
				float cf = g_opts.wfm_color_floor;
				if (cf > 1e-6f && tot < cf) {
					float sc = cf / (tot + 0.001f);
					r_c *= sc;
					g_c *= sc;
					b_c *= sc;
				}
				pair = wfm_pair_256(r_c, g_c, b_c);
			}

			if (g_is_tty) {
				wattron(win, COLOR_PAIR(pair) | row_attr);
				chtype ch = (subs_in_row >= 8) ? ACS_BLOCK :
								 ACS_CKBOARD;
				mvwaddch(win, screen_row, x + col, ch);
				wattroff(win, COLOR_PAIR(pair) | row_attr);
				continue;
			}

			wchar_t wch;
			if (subs_in_row >= 8) {
				wch = L'\u2588';
			} else if (above) {
				wch = (wchar_t)(0x2580 + subs_in_row);
				float dim = 0.45f + subs_in_row * 0.07f;
				int r = ((pair - WFM_PAIR_BASE) / 36);
				int g = (((pair - WFM_PAIR_BASE) / 6) % 6);
				int b = ((pair - WFM_PAIR_BASE) % 6);
				pair = wfm_pair_256(r / 5.0f * dim,
						    g / 5.0f * dim,
						    b / 5.0f * dim);
			} else {
				wch = L'\u2580';
				float dim = 0.35f + subs_in_row * 0.08f;
				int r = ((pair - WFM_PAIR_BASE) / 36);
				int g = (((pair - WFM_PAIR_BASE) / 6) % 6);
				int b = ((pair - WFM_PAIR_BASE) % 6);
				pair = wfm_pair_256(r / 5.0f * dim,
						    g / 5.0f * dim,
						    b / 5.0f * dim);
			}

			wattron(win, COLOR_PAIR(pair) | row_attr);
			cchar_t cc;
			wchar_t wcs[2] = { wch, L'\0' };
			setcchar(&cc, wcs, A_NORMAL, pair, NULL);
			mvwadd_wch(win, screen_row, x + col, &cc);
			wattroff(win, COLOR_PAIR(pair) | row_attr);
		}
	}

	if (slicer_active && t->bpm > 10.0f) {
		float b_f = (float)g_actual_sample_rate * 60.0f / t->bpm;
		for (int i = 0; i <= 8; i++) {
			int col = slice_cols[i];
			if (col < 0 || col >= w)
				continue;

			/* Highlight active slice (where playhead is) vs others */
			int active_slice =
				(int)((t->pos - t->bpm_offset) / b_f);
			int this_slice =
				(int)(((int64_t)left_frame +
				       (int64_t)(col * frames_per_col) -
				       t->bpm_offset) /
				      b_f);
			int sl_pair = (this_slice == active_slice) ?
					      COLOR_HOT :
					      COLOR_ACTIVE;

			for (int row_i = 0; row_i < WFM_ROWS; row_i++) {
				wattron(win, COLOR_PAIR(sl_pair) | A_BOLD);
				mvwaddch(win, y + 1 + row_i, x + col, '|');
				wattroff(win, COLOR_PAIR(sl_pair) | A_BOLD);
			}
		}
	}

	for (int col = 0; col < w; col += col_step) {
		if (col == w / 2) {
			for (int row = 0; row < WFM_ROWS; row++) {
				wattron(win, A_REVERSE | A_BOLD |
						     COLOR_PAIR(COLOR_STATUS));
				mvwaddch(win, y + 1 + row, x + col, '|');
				wattroff(win, A_REVERSE | A_BOLD |
						      COLOR_PAIR(COLOR_STATUS));
			}
		}
		if (g_opts.eco_mode && col + 1 < w) {
			for (int row = 0; row < WFM_ROWS; row++) {
				int screen_row = y + 1 + row;
				chtype ch = mvwinch(win, screen_row, x + col);
				mvwaddch(win, screen_row, x + col + 1, ch);
			}
		}
	}

	int ruler_y = y + 1 + WFM_ROWS;
	for (int col = 0; col < w; col++)
		mvwaddch(win, ruler_y, x + col, '-');
	if (t->bpm > 1.0f) {
		float disp_bpm = (t->bpm_display_double == 1)  ? t->bpm * 2.0f :
				 (t->bpm_display_double == -1) ? t->bpm * 0.5f :
								 t->bpm;
		float beat_frames = (g_actual_sample_rate * 60.0f) / disp_bpm;
		float left_beat =
			((float)left_frame - t->bpm_offset) / beat_frames;
		int first_beat = (int)floorf(left_beat);
		for (int bn = first_beat;; bn++) {
			float beat_frame = t->bpm_offset + bn * beat_frames;
			int64_t col_pos =
				(int64_t)((beat_frame - (float)left_frame) /
					  frames_per_col);
			if (col_pos >= w)
				break;
			if (col_pos < 0)
				continue;
			int beat_in_bar = ((bn % 4) + 4) % 4 + 1;
			int bar = (int)floorf((float)bn / 4.0f) + 1;
			if (beat_in_bar == 1) {
				wattron(win, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD |
						     A_REVERSE);
				mvwaddch(win, ruler_y, x + (int)col_pos, '|');
				if (bar > 0)
					mvwprintw(win, ruler_y,
						  x + (int)col_pos + 1, "%d",
						  bar);
				wattroff(win, COLOR_PAIR(COLOR_ACTIVE) |
						      A_BOLD | A_REVERSE);
			} else {
				wattron(win, COLOR_PAIR(COLOR_HOT));
				mvwaddch(win, ruler_y, x + (int)col_pos, '|');
				mvwprintw(win, ruler_y, x + (int)col_pos + 1,
					  "%d", beat_in_bar);
				wattroff(win, COLOR_PAIR(COLOR_HOT));
			}
		}
		float cur_beat = ((float)t->pos - t->bpm_offset) / beat_frames;
		int cur_bar = (int)(cur_beat / 4.0f) + 1;
		int cur_b = (int)fmodf(cur_beat, 4.0f) + 1;
		wattron(win, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
		mvwprintw(win, y, x + w - 14, "BAR:%3d", cur_bar);
		wattroff(win, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
		wattron(win, COLOR_PAIR(COLOR_HOT) | A_BOLD);
		mvwprintw(win, y, x + w - 7, " BEAT:%d", cur_b);
		wattroff(win, COLOR_PAIR(COLOR_HOT) | A_BOLD);
	}
	int cue_y = y + 1 + WFM_ROWS + 1;
	for (int col = 0; col < w; col++)
		mvwaddch(win, cue_y, x + col, ' ');
	for (int ci = 0; ci < MAX_CUES; ci++) {
		if (!t->cue_set[ci])
			continue;
		int64_t col_pos =
			(int64_t)(((float)t->cue[ci] - (float)left_frame) /
				  frames_per_col);
		if (col_pos >= 0 && col_pos < w) {
			wattron(win, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(win, cue_y, x + (int)col_pos, "C%d", ci + 1);
			wattroff(win, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		}
	}
	float cur_secs = (float)t->pos / g_actual_sample_rate;
	wattron(win, A_DIM);
	mvwprintw(win, y, x + 10, " %02d:%06.3f ", (int)cur_secs / 60,
		  fmodf(cur_secs, 60.0f));
	wattroff(win, A_DIM);
	if (t->looping) {
		float loop_secs = (float)(t->loop_end - t->loop_start) /
				  (float)g_actual_sample_rate;
		wattron(win, A_BOLD | A_REVERSE | COLOR_PAIR(COLOR_ACTIVE));
		mvwprintw(win, y, x + w - 22, "  circular LOOP %.2fs ",
			  loop_secs);
		wattroff(win, A_BOLD | A_REVERSE | COLOR_PAIR(COLOR_ACTIVE));
	}
}

static void draw_crossfader(WINDOW *w, int y, int x, int width)
{
	int fader_w = width / 2;
	if (fader_w < 20)
		fader_w = 20;
	int fx = x + (width - fader_w) / 2;
	wattron(w, COLOR_PAIR(COLOR_HEADER));
	mvwprintw(w, y, fx - 8, "XFADE A");
	mvwprintw(w, y, fx + fader_w + 2, "B (%3.0f%%)", g_crossfader * 100.0f);
	wattroff(w, COLOR_PAIR(COLOR_HEADER));
	int usable_w = fader_w - 2;
	int cf_pos = (int)(g_crossfader * (float)usable_w + 0.5f);
	if (cf_pos < 0)
		cf_pos = 0;
	if (cf_pos > usable_w)
		cf_pos = usable_w;
	mvwaddch(w, y, fx, '[');
	mvwaddch(w, y, fx + fader_w - 1, ']');
	for (int i = 1; i < fader_w - 1; i++) {
		if (i == usable_w / 2 + 1)
			mvwaddch(w, y, fx + i, ':');
		else
			mvwaddch(w, y, fx + i, '.');
	}
	mvwaddch(w, y, fx + 1 + cf_pos, '|');
	if (g_sync_leader >= 0 || g_num_tracks == 2) {
		int meter_w = 15;
		int mx = x + width - meter_w - 2;
		wattron(w, A_DIM);
		mvwaddch(w, y, mx + meter_w, (g_sync_leader >= 0) ? 'L' : 'A');
		mvwaddch(w, y + 1, mx + meter_w,
			 (g_sync_leader >= 0) ? 'F' : 'B');
		wattroff(w, A_DIM);
		for (int row = 0; row < 2; row++) {
			int deck_idx =
				(g_sync_leader >= 0) ?
					(row == 0 ? g_sync_leader :
						    (g_num_tracks == 2 ?
							     1 - g_sync_leader :
							     g_active_track)) :
					row;
			if (deck_idx < 0 || deck_idx >= MAX_TRACKS)
				continue;
			Track *t = &g_tracks[deck_idx];
			mvwaddch(w, y + row, mx, '[');
			mvwaddch(w, y + row, mx + meter_w - 1, ']');
			mvwaddch(w, y + row, mx + (meter_w / 2), ':');
			if (t->loaded && t->bpm > 0.0f) {
				float beat_frames =
					(float)g_actual_sample_rate * 60.0f /
					t->bpm;
				float phase =
					fmodf((float)t->pos - t->bpm_offset,
					      beat_frames) /
					beat_frames;
				int bx =
					(int)(phase * (float)(meter_w - 3)) + 1;
				wattron(w,
					COLOR_PAIR(deck_idx == g_sync_leader ?
							   COLOR_ACTIVE :
							   COLOR_HOT) |
						A_BOLD);
				mvwaddch(w, y + row, mx + bx, '#');
				wattroff(w,
					 COLOR_PAIR(deck_idx == g_sync_leader ?
							    COLOR_ACTIVE :
							    COLOR_HOT) |
						 A_BOLD);
			}
		}
	}
}

static void draw_vu_meter(WINDOW *w, int y, int x, int h)
{
	float lvl_l = g_vu_l < 1.0f ? g_vu_l : 1.0f;
	float lvl_r = g_vu_r < 1.0f ? g_vu_r : 1.0f;
	float pk_l = g_vu_peak_l < 1.0f ? g_vu_peak_l : 1.0f;
	float pk_r = g_vu_peak_r < 1.0f ? g_vu_peak_r : 1.0f;
	if (g_is_tty) {
		int bar_h = h - 1;
		int fill_l = (int)(lvl_l * (float)bar_h + 0.5f);
		int fill_r = (int)(lvl_r * (float)bar_h + 0.5f);
		int peak_row_l =
			bar_h - 1 - (int)(pk_l * (float)(bar_h - 1) + 0.5f);
		int peak_row_r =
			bar_h - 1 - (int)(pk_r * (float)(bar_h - 1) + 0.5f);
		for (int row = 0; row < bar_h; row++) {
			int row_from_bottom = bar_h - 1 - row;
			float zone = (float)row / (float)bar_h;
			int color = (zone < 0.10f) ? COLOR_HOT :
				    (zone < 0.30f) ? COLOR_VU :
						     COLOR_HEADER;
			for (int ch = 0; ch < 2; ch++) {
				int fill = (ch == 0) ? fill_l : fill_r;
				int pkr = (ch == 0) ? peak_row_l : peak_row_r;
				int on = (row_from_bottom < fill);
				int is_pk = (row == pkr && pk_l > 0.02f);
				wattron(w, COLOR_PAIR(color));
				mvwaddch(w, y + row, x + ch,
					 (on || is_pk) ?
						 (is_pk && !on ? ACS_CKBOARD :
								 ACS_BLOCK) :
						 ' ');
				wattroff(w, COLOR_PAIR(color));
			}
		}
		wattron(w, A_DIM | COLOR_PAIR(COLOR_STATUS));
		mvwprintw(w, y + h - 1, x, "LR");
		wattroff(w, A_DIM | COLOR_PAIR(COLOR_STATUS));
		return;
	}
	int bar_h = h - 1;
	int half_h = bar_h * 2;
	int fill_l = (int)(lvl_l * (float)half_h);
	int fill_r = (int)(lvl_r * (float)half_h);
	int peak_l = (int)(pk_l * (float)(half_h - 1));
	int peak_r = (int)(pk_r * (float)(half_h - 1));
	for (int row = 0; row < bar_h; row++) {
		int bot_half = (bar_h - 1 - row) * 2;
		int top_half = bot_half + 1;
		float zone = (float)row / (float)bar_h;
		int color = (zone < 0.10f) ? COLOR_HOT :
			    (zone < 0.30f) ? COLOR_VU :
					     COLOR_HEADER;
		for (int ch = 0; ch < 2; ch++) {
			int fill = (ch == 0) ? fill_l : fill_r;
			int pk = (ch == 0) ? peak_l : peak_r;
			int bot_on = (bot_half < fill);
			int top_on = (top_half < fill);
			int bot_pk = (bot_half == pk && pk > 0);
			int top_pk = (top_half == pk && pk > 0);
			wattron(w, COLOR_PAIR(color));
			cchar_t cc;
			if ((top_on || top_pk) && (bot_on || bot_pk)) {
				setcchar(&cc, L"\u2588", A_NORMAL, color, NULL);
				mvwadd_wch(w, y + row, x + ch, &cc);
			} else if (top_on || top_pk) {
				setcchar(&cc, L"\u2580", A_NORMAL, color, NULL);
				mvwadd_wch(w, y + row, x + ch, &cc);
			} else if (bot_on || bot_pk) {
				setcchar(&cc, L"\u2584", A_NORMAL, color, NULL);
				mvwadd_wch(w, y + row, x + ch, &cc);
			} else
				mvwaddch(w, y + row, x + ch, ' ');
			wattroff(w, COLOR_PAIR(color));
		}
	}
	wattron(w, A_DIM | COLOR_PAIR(COLOR_STATUS));
	mvwprintw(w, y + h - 1, x, "LR");
	wattroff(w, A_DIM | COLOR_PAIR(COLOR_STATUS));
}

static void draw_decks_view(void)
{
	static const int deck_pos4[4] = { 2, 0, 1, 3 };
	int meter_after = (g_num_tracks == 4) ? 1 : 0;
	int deck_rows = 9;
	int meter_w = 2;
	int usable = g_cols - meter_w;
	int dw = usable / g_num_tracks;
	for (int i = 0; i < g_num_tracks; i++) {
		int di = (g_num_tracks == 4) ? deck_pos4[i] : i;
		draw_deck(g_win_main, 0,
			  i * dw + (i > meter_after ? meter_w : 0), dw, di);
	}
	draw_vu_meter(g_win_main, 0, (meter_after + 1) * dw, deck_rows);
	int panel_h = WFM_ROWS + 3;
	int panel_y = deck_rows;
	for (int i = 0; i < g_num_tracks; i++) {
		int di = (g_num_tracks == 4) ? deck_pos4[i] : i;
		draw_scrolling_waveform(g_win_main, panel_y, 0, g_cols, di);
		panel_y += panel_h;
	}
	draw_crossfader(g_win_main,
			(panel_y < g_rows - 3) ? panel_y : g_rows - 3, 0,
			g_cols);
}

static int library_rows_available(void)
{
	return g_rows - (9 + (WFM_ROWS + 3) * 2 + 2 + 1 + 1);
}
static int library_min_rows(void)
{
	return 9 + (WFM_ROWS + 3) * 2 + 2 + 1 + 1 + 4;
}

static void draw_full_panel_view(int by, int brows)
{
	if (brows < 4) {
		if (brows >= 1) {
			int need = library_min_rows();
			wattron(g_win_main,
				COLOR_PAIR(COLOR_HOT) | A_BOLD | A_REVERSE);
			mvwprintw(
				g_win_main, by, 0,
				" \u26a0  LIBRARY HIDDEN -- too short (%d rows, need %d)",
				g_rows, need);
			wattroff(g_win_main,
				 COLOR_PAIR(COLOR_HOT) | A_BOLD | A_REVERSE);
		}
		return;
	}
	if (g_panel == 0) {
		wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
		int plen = (int)strlen(g_fb_path);
		int pmax = g_cols - 4;
		mvwprintw(g_win_main, by, 0, " \u25b6 %-*.*s", pmax, pmax,
			  (plen > pmax) ? g_fb_path + plen - pmax : g_fb_path);
		wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, by + 1, 0, " %-4s %6s %-4s %-*s", "TYPE",
			  "BPM", "KEY", g_cols - 18, "NAME");
		wattroff(g_win_main, A_BOLD);
		int list_rows = brows - 3;
		fb_fix_scroll(list_rows);
		for (int row = 0; row < list_rows; row++) {
			int idx = g_fb_scroll + row;
			if (idx >= g_fb_count) {
				mvwprintw(g_win_main, by + 2 + row, 0, "%*s",
					  g_cols, "");
				continue;
			}
			FBEntry *e = &g_fb_entries[idx];
			int sel = (idx == g_fb_sel);
			char dbuf[256];
			const char *dname =
				fb_display_name(e, dbuf, sizeof(dbuf));
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			if (e->is_dir) {
				if (!sel)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_HEADER));
				mvwprintw(g_win_main, by + 2 + row, 0,
					  " DIR  %6s  %-*.*s", "", g_cols - 14,
					  g_cols - 14, e->name);
				if (!sel)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_HEADER));
			} else {
				char ext[8] = "";
				const char *dot = strrchr(e->name, '.');
				if (dot) {
					strncpy(ext, dot + 1, 7);
					for (int i = 0; ext[i]; i++)
						ext[i] = toupper(ext[i]);
				}
				char full_path[FB_PATH_MAX + 256];
				snprintf(full_path, sizeof(full_path), "%s/%s",
					 g_fb_path, e->name);
				int played =
					!sel && history_was_played(full_path);
				if (e->bpm > 0.0f) {
					if (!sel)
						wattron(g_win_main, A_DIM);
					mvwprintw(g_win_main, by + 2 + row, 0,
						  " %-4s %6.1f  ", ext, e->bpm);
					if (!sel)
						wattroff(g_win_main, A_DIM);
					if (e->tag_key[0]) {
						wattron(g_win_main,
							COLOR_PAIR(
								COLOR_ACTIVE));
						mvwprintw(g_win_main,
							  by + 2 + row, 14,
							  "%-4s", e->tag_key);
						wattroff(g_win_main,
							 COLOR_PAIR(
								 COLOR_ACTIVE));
					}
					if (played)
						wattron(g_win_main,
							COLOR_PAIR(
								COLOR_PLAYED));
					mvwprintw(g_win_main, by + 2 + row, 19,
						  "%-*.*s", g_cols - 19,
						  g_cols - 19, dname);
					if (played)
						wattroff(g_win_main,
							 COLOR_PAIR(
								 COLOR_PLAYED));
				} else {
					if (played)
						wattron(g_win_main,
							COLOR_PAIR(
								COLOR_PLAYED));
					mvwprintw(g_win_main, by + 2 + row, 0,
						  " %-4s %6s  %-*.*s", ext,
						  "---", g_cols - 14,
						  g_cols - 14, dname);
					if (played)
						wattroff(g_win_main,
							 COLOR_PAIR(
								 COLOR_PLAYED));
				}
			}
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		}
		wattron(g_win_main, A_DIM);
		mvwprintw(
			g_win_main, by + brows - 1, 0,
			" ENTER=load  !=A @=B #=C $=D  j/k=nav  BKSP=up  p=+playlist  P=playlist  i=tag");
		for (int i = 72; i < g_cols; i++)
			mvwaddch(g_win_main, by + brows - 1, i, ' ');
		wattroff(g_win_main, A_DIM);
		if (g_fb_status[0]) {
			wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
			mvwprintw(g_win_main, by + brows - 1,
				  g_cols - (int)strlen(g_fb_status) - 2, " %s ",
				  g_fb_status);
			wattroff(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
		}
		if (g_fb_count > list_rows)
			mvwprintw(g_win_main, by + 2, g_cols - 10, "[%3d/%3d]",
				  g_fb_sel + 1, g_fb_count);
	} else if (g_panel == 3) {
		if (g_crate_view_level == 0) {
			wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
			mvwprintw(g_win_main, by, 0, " \u25b6 CRATES ");
			wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
			int total_vrows = crate_total_vrows();
			int list_rows = brows - 2;
			if (list_rows < 1)
				list_rows = 1;
			if (g_crate_vrow < 0)
				g_crate_vrow = 0;
			if (total_vrows > 0 && g_crate_vrow >= total_vrows)
				g_crate_vrow = total_vrows - 1;
			if (g_crate_vscroll > g_crate_vrow)
				g_crate_vscroll = g_crate_vrow;
			if (g_crate_vscroll < g_crate_vrow - list_rows + 1)
				g_crate_vscroll = g_crate_vrow - list_rows + 1;
			if (g_crate_vscroll < 0)
				g_crate_vscroll = 0;
			for (int row = 0; row < list_rows; row++) {
				int vrow = g_crate_vscroll + row;
				if (vrow >= total_vrows) {
					mvwprintw(g_win_main, by + 1 + row, 0,
						  "%*s", g_cols, "");
					continue;
				}
				int vt, vi;
				crate_vrow_resolve(vrow, &vt, &vi);
				int sel = (vrow == g_crate_vrow);
				if (vt == 1) {
					CrateGroup *grp = &g_crate_groups[vi];
					if (sel)
						wattron(g_win_main,
							COLOR_PAIR(
								COLOR_ACTIVE) |
								A_BOLD);
					else
						wattron(g_win_main,
							A_DIM | A_BOLD);
					mvwprintw(g_win_main, by + 1 + row, 0,
						  " \u2500\u2500 %s ",
						  grp->label);
					for (int x = (int)strlen(grp->label) +
						     5;
					     x < g_cols; x++)
						mvwaddch(g_win_main,
							 by + 1 + row, x,
							 sel ? ' ' : ACS_HLINE);
					if (sel)
						wattroff(g_win_main,
							 COLOR_PAIR(
								 COLOR_ACTIVE) |
								 A_BOLD);
					else
						wattroff(g_win_main,
							 A_DIM | A_BOLD);
				} else {
					Crate *cr = &g_crates[vi];
					if (sel)
						wattron(g_win_main,
							COLOR_PAIR(
								COLOR_ACTIVE) |
								A_BOLD);
					mvwprintw(g_win_main, by + 1 + row, 0,
						  "   \u25b8 %-*.*s%s",
						  g_cols - 10, g_cols - 10,
						  cr->name,
						  cr->is_usb ? " [USB]" : "");
					if (sel)
						wattroff(g_win_main,
							 COLOR_PAIR(
								 COLOR_ACTIVE) |
								 A_BOLD);
				}
			}
			wattron(g_win_main, A_DIM);
			mvwprintw(g_win_main, by + brows - 1, 0,
				  " ENTER=open  TAB=next panel  P=browser");
			for (int i = 40; i < g_cols; i++)
				mvwaddch(g_win_main, by + brows - 1, i, ' ');
			wattroff(g_win_main, A_DIM);
		} else {
			wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
			mvwprintw(g_win_main, by, 0, " \u25b6 CRATE: %s",
				  g_active_crate_name);
			wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
			wattron(g_win_main, A_BOLD);
			mvwprintw(g_win_main, by + 1, 0, " %6s %-4s %-*s",
				  "BPM", "KEY", g_cols - 14, "NAME");
			wattroff(g_win_main, A_BOLD);
			int list_rows = brows - 3;
			if (list_rows < 1)
				list_rows = 1;
			if (g_crate_tracks_sel < 0)
				g_crate_tracks_sel = 0;
			if (g_crate_tracks_count > 0 &&
			    g_crate_tracks_sel >= g_crate_tracks_count)
				g_crate_tracks_sel = g_crate_tracks_count - 1;
			if (g_crate_tracks_scroll > g_crate_tracks_sel)
				g_crate_tracks_scroll = g_crate_tracks_sel;
			if (g_crate_tracks_scroll <
			    g_crate_tracks_sel - list_rows + 1)
				g_crate_tracks_scroll =
					g_crate_tracks_sel - list_rows + 1;
			if (g_crate_tracks_scroll < 0)
				g_crate_tracks_scroll = 0;
			for (int row = 0; row < list_rows; row++) {
				int idx = g_crate_tracks_scroll + row;
				if (idx >= g_crate_tracks_count) {
					mvwprintw(g_win_main, by + 2 + row, 0,
						  "%*s", g_cols, "");
					continue;
				}
				CrateEntry *e = &g_crate_tracks[idx];
				int sel = (idx == g_crate_tracks_sel);
				if (sel)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_ACTIVE) |
							A_BOLD);
				if (e->bpm > 0.0f) {
					if (!sel)
						wattron(g_win_main, A_DIM);
					mvwprintw(g_win_main, by + 2 + row, 0,
						  " %6.1f  ", e->bpm);
					if (!sel)
						wattroff(g_win_main, A_DIM);
					if (e->tag_key[0]) {
						wattron(g_win_main,
							COLOR_PAIR(
								COLOR_ACTIVE));
						mvwprintw(g_win_main,
							  by + 2 + row, 9,
							  "%-4s", e->tag_key);
						wattroff(g_win_main,
							 COLOR_PAIR(
								 COLOR_ACTIVE));
					}
					mvwprintw(g_win_main, by + 2 + row, 14,
						  "%-*.*s", g_cols - 14,
						  g_cols - 14, e->name);
				} else
					mvwprintw(g_win_main, by + 2 + row, 0,
						  " %6s  %-*.*s", "---",
						  g_cols - 9, g_cols - 9,
						  e->name);
				if (sel)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_ACTIVE) |
							 A_BOLD);
			}
			wattron(g_win_main, A_DIM);
			mvwprintw(
				g_win_main, by + brows - 1, 0,
				" ENTER=load  BKSP=back  !=A @=B #=C $=D  TAB=next  P=browser");
			for (int i = 60; i < g_cols; i++)
				mvwaddch(g_win_main, by + brows - 1, i, ' ');
			wattroff(g_win_main, A_DIM);
		}
	} else if (g_panel == 1) {
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, by, 0, " %-3s %6s %-4s %-*s", "#", "BPM",
			  "KEY", g_cols - 17, "NAME");
		wattroff(g_win_main, A_BOLD);
		int list_rows = brows - 2;
		pl_fix_scroll(list_rows);
		for (int row = 0; row < list_rows; row++) {
			int idx = g_pl_scroll + row;
			if (idx >= g_pl_count) {
				mvwprintw(g_win_main, by + 1 + row, 0, "%*s",
					  g_cols, "");
				continue;
			}
			PLEntry *e = &g_pl[idx];
			int selected = (idx == g_pl_sel);
			if (selected)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			if (e->bpm > 0.0f) {
				if (!selected)
					wattron(g_win_main, A_DIM);
				mvwprintw(g_win_main, by + 1 + row, 0,
					  " %-3d %6.1f  ", idx + 1, e->bpm);
				if (!selected)
					wattroff(g_win_main, A_DIM);
				if (e->tag_key[0]) {
					wattron(g_win_main,
						COLOR_PAIR(COLOR_ACTIVE));
					mvwprintw(g_win_main, by + 1 + row, 13,
						  "%-4s", e->tag_key);
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_ACTIVE));
				}
				mvwprintw(g_win_main, by + 1 + row, 18,
					  "%-*.*s", g_cols - 18, g_cols - 18,
					  e->name);
			} else
				mvwprintw(g_win_main, by + 1 + row, 0,
					  " %-3d %6s  %-*.*s", idx + 1, "---",
					  g_cols - 13, g_cols - 13, e->name);
			if (selected)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		}
		wattron(g_win_main, A_DIM);
		mvwprintw(
			g_win_main, by + brows - 1, 0,
			" ENTER=load  !=A @=B #=C $=D  j/k=nav  DEL=remove  C-x=clear  P=browser");
		for (int i = 70; i < g_cols; i++)
			mvwaddch(g_win_main, by + brows - 1, i, ' ');
		wattroff(g_win_main, A_DIM);
	} else if (g_panel == 2) {
		if (!g_lib || (g_lib_count == 0 && !g_lib_scanning)) {
			wattron(g_win_main, A_DIM);
			mvwprintw(
				g_win_main, by, 0,
				" No library loaded. Press L to scan a directory.");
			wattroff(g_win_main, A_DIM);
		} else {
			wattron(g_win_main, A_BOLD);
			mvwprintw(g_win_main, by, 0, " %6s %-4s %-*s", "BPM",
				  "KEY", g_cols - 14, "NAME / ARTIST -- TITLE");
			wattroff(g_win_main, A_BOLD);
			int list_rows = brows - 2;
			if (g_lib_sel < 0)
				g_lib_sel = 0;
			if (g_lib_count > 0 && g_lib_sel >= g_lib_count)
				g_lib_sel = g_lib_count - 1;
			if (g_lib_scroll > g_lib_sel)
				g_lib_scroll = g_lib_sel;
			if (g_lib_scroll < g_lib_sel - list_rows + 1)
				g_lib_scroll = g_lib_sel - list_rows + 1;
			if (g_lib_scroll < 0)
				g_lib_scroll = 0;
			for (int row = 0; row < list_rows; row++) {
				int idx = g_lib_scroll + row;
				if (!g_lib || idx >= g_lib_count) {
					mvwprintw(g_win_main, by + 1 + row, 0,
						  "%*s", g_cols, "");
					continue;
				}
				LIBEntry *e = &g_lib[idx];
				int sel = (idx == g_lib_sel);
				if (sel)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_ACTIVE) |
							A_BOLD);
				char disp[264];
				if (e->tag_artist[0] && e->tag_title[0])
					snprintf(disp, sizeof(disp),
						 "%s \u25b6 %s", e->tag_artist,
						 e->tag_title);
				else if (e->tag_title[0])
					snprintf(disp, sizeof(disp), "%s",
						 e->tag_title);
				else
					snprintf(disp, sizeof(disp), "%s",
						 e->name);
				if (e->bpm > 0.0f) {
					if (!sel)
						wattron(g_win_main, A_DIM);
					mvwprintw(g_win_main, by + 1 + row, 0,
						  " %6.1f  ", e->bpm);
					if (!sel)
						wattroff(g_win_main, A_DIM);
					if (e->tag_key[0]) {
						wattron(g_win_main,
							COLOR_PAIR(
								COLOR_ACTIVE));
						mvwprintw(g_win_main,
							  by + 1 + row, 9,
							  "%-4s", e->tag_key);
						wattroff(g_win_main,
							 COLOR_PAIR(
								 COLOR_ACTIVE));
					}
					mvwprintw(g_win_main, by + 1 + row, 14,
						  "%-*.*s", g_cols - 14,
						  g_cols - 14, disp);
				} else
					mvwprintw(g_win_main, by + 1 + row, 0,
						  " %6s  %-*.*s", "---",
						  g_cols - 9, g_cols - 9, disp);
				if (sel)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_ACTIVE) |
							 A_BOLD);
			}
			wattron(g_win_main, A_DIM);
			mvwprintw(
				g_win_main, by + brows - 1, 0,
				" ENTER=load  !=A @=B #=C $=D  j/k=nav  p=+playlist  O=sort  L=rescan  P=browser");
			for (int i = 72; i < g_cols; i++)
				mvwaddch(g_win_main, by + brows - 1, i, ' ');
			wattroff(g_win_main, A_DIM);
		}
	} else if (g_panel == 4) {
		wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
		mvwprintw(g_win_main, by, 0, " \u25b6 SAMPLER ");
		wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
		int list_y = by + 2;
		for (int i = 0; i < MAX_SAMPLER_SLOTS; i++) {
			int sy = list_y + i;
			if (sy >= by + brows - 1)
				break;
			SamplerSlot *s = &g_samplers[i];
			if (s->playing)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_HOT) | A_BOLD);
			else if (s->data)
				wattron(g_win_main, COLOR_PAIR(COLOR_ACTIVE));
			else
				wattron(g_win_main, A_DIM);
			mvwprintw(g_win_main, sy, 2,
				  "[%d] %-*.*s  Vol:%3.0f%%  [%s]", i + 1,
				  g_cols - 28, g_cols - 28,
				  s->data ? s->name : "(empty)",
				  (double)s->volume * 100,
				  s->playing ? "PLAYING" : "STOPPED");
			if (s->playing)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_HOT) | A_BOLD);
			else if (s->data)
				wattroff(g_win_main, COLOR_PAIR(COLOR_ACTIVE));
			else
				wattroff(g_win_main, A_DIM);
		}
		wattron(g_win_main, A_DIM);
		mvwprintw(
			g_win_main, by + brows - 1, 0,
			" 1-8=trigger  !@#$%%^&*=load->slot  j/k=vol  TAB=next panel");
		for (int i = 58; i < g_cols; i++)
			mvwaddch(g_win_main, by + brows - 1, i, ' ');
		wattroff(g_win_main, A_DIM);
	}
}

static void draw_split_view(void)
{
	int deck_rows = 9;
	int meter_w = 2;
	int dw = (g_cols - meter_w) / 2;
	int vis[2] = { g_side_deck[0], g_side_deck[1] };
	draw_deck(g_win_main, 0, 0, dw, vis[0]);
	draw_vu_meter(g_win_main, 0, dw, deck_rows);
	draw_deck(g_win_main, 0, dw + meter_w, dw, vis[1]);
	int panel_h = WFM_ROWS + 3;
	int panel_y = deck_rows;
	for (int i = 0; i < 2; i++) {
		draw_scrolling_waveform(g_win_main, panel_y, 0, g_cols, vis[i]);
		panel_y += panel_h;
	}
	draw_crossfader(g_win_main, panel_y, 0, g_cols);
	panel_y += 2;
	int div_y = panel_y;
	wattron(g_win_main, A_DIM);
	for (int i = 0; i < g_cols; i++)
		mvwaddch(g_win_main, div_y, i, ACS_HLINE);
	if (g_crate_jump_active) {
		wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD | A_REVERSE);
		mvwprintw(g_win_main, div_y, 2, " CRATE JUMP: %s_ ",
			  g_crate_input);
		wattroff(g_win_main,
			 COLOR_PAIR(COLOR_HOT) | A_BOLD | A_REVERSE);
	} else if (g_crate_add_active) {
		wattron(g_win_main,
			COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
		mvwprintw(g_win_main, div_y, 2, " NEW CRATE: %s_ ",
			  g_crate_add_input);
		wattroff(g_win_main,
			 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
	} else if (g_track_add_crate_active) {
		wattron(g_win_main,
			COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
		mvwprintw(g_win_main, div_y, 2, " ADD TO CRATE: %s_ ",
			  g_track_add_crate_input);
		wattroff(g_win_main,
			 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
	} else if (g_usb_eject_active) {
		wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD | A_REVERSE);
		mvwprintw(g_win_main, div_y, 2, " Eject \"%s\"? [Y/n] ",
			  g_usb_eject_label);
		wattroff(g_win_main,
			 COLOR_PAIR(COLOR_HOT) | A_BOLD | A_REVERSE);
	} else if (g_usb_conflict_active) {
		wattron(g_win_main,
			COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
		mvwprintw(g_win_main, div_y, 2,
			  " CONFLICT \"%s\" exists. Rename: %s_ ESC=cancel ",
			  g_usb_conflict_crate_name, g_usb_conflict_rename);
		wattroff(g_win_main,
			 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
	} else if (g_usb_picker_active) {
		wattron(g_win_main,
			COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
		mvwprintw(g_win_main, div_y, 2, " Export to USB: ");
		for (int i = 0; i < g_usb_devices_count && i < 5; i++) {
			if (i == g_usb_picker_sel)
				wprintw(g_win_main, "[%s]",
					g_usb_devices[i].label);
			else
				wprintw(g_win_main, " %s ",
					g_usb_devices[i].label);
			if (i < g_usb_devices_count - 1)
				wprintw(g_win_main, "  ");
		}
		wprintw(g_win_main, "  j/k=select  ENTER=confirm  ESC=cancel ");
		wattroff(g_win_main,
			 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
	} else if (g_batch_running) {
		wattron(g_win_main,
			COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
		mvwprintw(g_win_main, div_y, 2,
			  " BPM ANALYZE [%d/%d] %.0f-%.0f BPM ESC=cancel ",
			  g_batch_queue_pos, g_batch_queue_count,
			  (double)g_bpm_detect_lo, (double)g_bpm_detect_hi);
		wattroff(g_win_main,
			 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
	} else if (g_batch_prompt_active) {
		wattron(g_win_main,
			COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
		mvwprintw(
			g_win_main, div_y, 2,
			" BPM RANGE: LO[%s%s] HI[%s%s] TAB=next ENTER=start ESC=cancel ",
			g_batch_lo_buf, g_batch_prompt_field == 0 ? "_" : " ",
			g_batch_hi_buf, g_batch_prompt_field == 1 ? "_" : " ");
		wattroff(g_win_main,
			 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
	} else {
		wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
		if (g_panel == 0) {
			static const char *sort_labels[] = { "NAME",
							     "BPM\u25b2",
							     "BPM\u25bc" };
			mvwprintw(g_win_main, div_y, 2,
				  " BROWSER [%s] \u25BC  TAB=next panel ",
				  sort_labels[g_fb_sort]);
		} else if (g_panel == 1) {
			mvwprintw(g_win_main, div_y, 2,
				  " PLAYLIST \u25BC (%d)  TAB=next panel ",
				  g_pl_count);
		} else if (g_panel == 2) {
			static const char *lsort_labels[] = { "NAME",
							      "BPM\u25b2",
							      "BPM\u25bc" };
			if (g_lib_scanning)
				mvwprintw(
					g_win_main, div_y, 2,
					" LIBRARY \u25BC (scanning\xe2\x80\xa6) ");
			else
				mvwprintw(g_win_main, div_y, 2,
					  " LIBRARY [%s] \u25BC (%d) ",
					  lsort_labels[g_lib_sort],
					  g_lib_count);
		} else if (g_panel == 3) {
			mvwprintw(g_win_main, div_y, 2,
				  " CRATES \u25BC (%d)  TAB=next panel ",
				  g_ncrate);
		} else {
			mvwprintw(g_win_main, div_y, 2,
				  " SAMPLER \u25BC (8 slots)  TAB=next panel ");
		}
		wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	}
	wattroff(g_win_main, A_DIM);
	panel_y++;
	draw_full_panel_view(panel_y, g_rows - 1 - panel_y);
}

static void draw_tag_panel(void)
{
	if (!g_tag_info.visible)
		return;
	int pw = 60;
	int ph = 10;
	int px = (g_cols - pw) / 2;
	int py = (g_rows - 1 - ph) / 2;
	if (px < 0)
		px = 0;
	if (py < 0)
		py = 0;
	if (pw > g_cols)
		pw = g_cols;
	wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	for (int r = py; r < py + ph && r < g_rows - 1; r++) {
		for (int c = px; c < px + pw && c < g_cols; c++) {
			if (r == py || r == py + ph - 1)
				mvwaddch(
					g_win_main, r, c,
					(r == py && c == px) ? ACS_ULCORNER :
					(r == py && c == px + pw - 1) ?
							       ACS_URCORNER :
					(r == py + ph - 1 && c == px) ?
							       ACS_LLCORNER :
					(r == py + ph - 1 && c == px + pw - 1) ?
							       ACS_LRCORNER :
							       ACS_HLINE);
			else if (c == px || c == px + pw - 1)
				mvwaddch(g_win_main, r, c, ACS_VLINE);
			else
				mvwaddch(g_win_main, r, c, ' ');
		}
	}
	mvwprintw(g_win_main, py, px + 2, " MusicBrainz Tag Lookup ");
	wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	int row = py + 1;
	int iw = pw - 4;
	if (g_tag_info.status[0]) {
		wattron(g_win_main, A_DIM);
		mvwprintw(g_win_main, row++, px + 2, "  %-*.*s", iw, iw,
			  g_tag_info.status);
		mvwprintw(g_win_main, row++, px + 2, "  File: %-*.*s", iw - 6,
			  iw - 6, g_tag_info.query_name);
		wattroff(g_win_main, A_DIM);
	} else {
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, row++, px + 2, "  Title:  %-*.*s", iw - 9,
			  iw - 9, g_tag_info.title);
		wattroff(g_win_main, A_BOLD);
		mvwprintw(g_win_main, row++, px + 2, "  Artist: %-*.*s", iw - 9,
			  iw - 9, g_tag_info.artist);
		mvwprintw(g_win_main, row++, px + 2, "  Album:  %-*.*s", iw - 9,
			  iw - 9, g_tag_info.album);
		mvwprintw(g_win_main, row++, px + 2, "  Date:   %-*.*s", iw - 9,
			  iw - 9, g_tag_info.date);
		mvwprintw(g_win_main, row++, px + 2, "  Label:  %-*.*s", iw - 9,
			  iw - 9, g_tag_info.label);
		mvwprintw(g_win_main, row++, px + 2, "  File:   %-*.*s", iw - 9,
			  iw - 9, g_tag_info.query_name);
	}
	wattron(g_win_main, A_DIM);
	mvwprintw(g_win_main, py + ph - 2, px + 2, "  Press any key to close");
	wattroff(g_win_main, A_DIM);
}

void apply_ui_fps(void)
{
	wtimeout(g_win_main, 1000 / g_opts.ui_fps);
}

void options_read_cpuinfo(char *out, int max)
{
	FILE *f = fopen("/proc/cpuinfo", "r");
	if (!f) {
		snprintf(out, max, "unavailable");
		return;
	}
	char line[256];
	out[0] = '\0';
	while (fgets(line, sizeof(line), f)) {
		if (strncmp(line, "cpu\t\t:", 6) == 0 ||
		    strncmp(line, "model name", 10) == 0 ||
		    strncmp(line, "cpu MHz", 7) == 0 ||
		    strncmp(line, "BogoMIPS", 8) == 0) {
			char *colon = strchr(line, ':');
			if (colon) {
				char *val = colon + 1;
				while (*val == ' ' || *val == '\t')
					val++;
				int vlen = (int)strlen(val);
				while (vlen > 0 && (val[vlen - 1] == '\n' ||
						    val[vlen - 1] == '\r'))
					val[--vlen] = '\0';
				int cur = (int)strlen(out);
				if (cur > 0 && cur < max - 2) {
					out[cur++] = ' ';
					out[cur] = '\0';
				}
				strncat(out, val, max - strlen(out) - 1);
			}
		}
	}
	fclose(f);
	if (out[0] == '\0')
		snprintf(out, max, "unknown");
}

void options_read_meminfo(char *out, int max)
{
	FILE *f = fopen("/proc/meminfo", "r");
	if (!f) {
		snprintf(out, max, "unavailable");
		return;
	}
	char line[128];
	long total = 0, avail = 0;
	while (fgets(line, sizeof(line), f)) {
		if (strncmp(line, "MemTotal:", 9) == 0)
			sscanf(line + 9, "%ld", &total);
		if (strncmp(line, "MemAvailable:", 13) == 0)
			sscanf(line + 13, "%ld", &avail);
	}
	fclose(f);
	snprintf(out, max, "%ld MB total, %ld MB free", total / 1024,
		 avail / 1024);
}

void draw_options_overlay(void)
{
	if (!g_options_open)
		return;
	int ow = (g_cols > 78) ? 78 : g_cols;
	int oh = g_rows - 4;
	if (oh < 10)
		oh = 10;
	int ox = (g_cols - ow) / 2;
	int oy = 2;
	wattron(g_win_main, COLOR_PAIR(COLOR_STATUS));
	for (int r = oy; r < oy + oh; r++) {
		wmove(g_win_main, r, ox);
		for (int c = 0; c < ow; c++)
			waddch(g_win_main, ' ');
	}
	wattroff(g_win_main, COLOR_PAIR(COLOR_STATUS));
	const char *tabs[] = { " INFO ", " AUDIO ", " DISP ",
			       " WAVE ", " SYNC ",  " THEME ",
			       " MIDI ", " OUT ",   " FX " };
	const int ntabs = 9;
	wattron(g_win_main, A_DIM);
	for (int r = oy + 1; r < oy + oh + 1 && r < g_rows - 1; r++)
		mvwaddch(g_win_main, r, ox + ow, ' ');
	for (int c = ox + 1; c < ox + ow + 1 && c < g_cols; c++)
		mvwaddch(g_win_main, oy + oh, c, ' ');
	wattroff(g_win_main, A_DIM);
	wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	for (int c = ox; c < ox + ow; c++)
		mvwaddch(g_win_main, oy, c,
			 (c == ox)	    ? ACS_ULCORNER :
			 (c == ox + ow - 1) ? ACS_URCORNER :
					      ACS_HLINE);
	mvwprintw(g_win_main, oy, ox + 2, "\u2524 djcmd OPTIONS \u251c");
	wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	int tx = ox + 19;
	for (int i = 0; i < ntabs; i++) {
		int tlen = (int)strlen(tabs[i]);
		if (i == g_options_tab) {
			wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
			mvwaddch(g_win_main, oy, tx, ACS_RTEE);
			wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
			wattron(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			for (int j = 0; j < tlen; j++)
				mvwaddch(g_win_main, oy, tx + 1 + j,
					 tabs[i][j]);
			wattroff(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
			mvwaddch(g_win_main, oy, tx + 1 + tlen, ACS_LTEE);
			wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
			tx += tlen + 3;
		} else {
			wattron(g_win_main, A_DIM);
			for (int j = 0; j < tlen; j++)
				mvwaddch(g_win_main, oy, tx + j, tabs[i][j]);
			wattroff(g_win_main, A_DIM);
			tx += tlen;
		}
		if (tx >= ox + ow - 2)
			break;
	}
	wattron(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	for (int r = oy + 1; r < oy + oh - 1; r++) {
		mvwaddch(g_win_main, r, ox, ACS_VLINE);
		mvwaddch(g_win_main, r, ox + ow - 1, ACS_VLINE);
	}
	for (int c = ox; c < ox + ow; c++)
		mvwaddch(g_win_main, oy + oh - 1, c,
			 (c == ox)	    ? ACS_LLCORNER :
			 (c == ox + ow - 1) ? ACS_LRCORNER :
					      ACS_HLINE);
	mvwprintw(
		g_win_main, oy + oh - 1, ox + 2,
		"\u2524 ESC=close  \u25c4\u25ba=tabs  j/k=select  -/+=adjust  ENTER=apply \u251c");
	wattroff(g_win_main, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	int cy = oy + 2;
	int iw = ow - 4;
	if (g_options_tab == 0) {
		time_t now = time(NULL);
		if (now != g_meminfo_last_t) {
			options_read_meminfo(g_meminfo_cache,
					     sizeof(g_meminfo_cache));
			g_meminfo_last_t = now;
		}
		char midi_info[64] = "not connected";
		if (g_midi_in) {
			snd_rawmidi_info_t *info;
			snd_rawmidi_info_alloca(&info);
			if (snd_rawmidi_info(g_midi_in, info) == 0)
				snprintf(midi_info, sizeof(midi_info), "%s",
					 snd_rawmidi_info_get_name(info));
			else
				strcpy(midi_info, "connected");
		}
		long pcm_kb = 0;
		for (int i = 0; i < MAX_TRACKS; i++)
			if (g_tracks[i].loaded)
				pcm_kb +=
					(long)g_tracks[i].num_frames * 4 / 1024;
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " HARDWARE");
		wattroff(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, "  CPU      : %-*.*s",
			  iw - 12, iw - 12, g_cpuinfo_cache);
		mvwprintw(g_win_main, cy++, ox + 2, "  Memory   : %-*.*s",
			  iw - 12, iw - 12, g_meminfo_cache);
		mvwprintw(g_win_main, cy++, ox + 2, "  ALSA dev : %s",
			  g_pcm_dev_str);
		mvwprintw(g_win_main, cy++, ox + 2, "  ALSA cfg : %d Hz, %d ch",
			  g_actual_sample_rate, CFG_CHANNELS);
		mvwprintw(g_win_main, cy++, ox + 2, "  MIDI     : %-*.*s",
			  iw - 12, iw - 12, midi_info);
		cy++;
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " RUNTIME");
		wattroff(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, "  Decks    : %d active",
			  g_num_tracks);
		mvwprintw(g_win_main, cy++, ox + 2,
			  "  PCM mem  : %ld KB loaded", pcm_kb);
		mvwprintw(g_win_main, cy++, ox + 2, "  256color : %s",
			  g_has_256 ? "yes" : "no");
		const char *lp = mixlog_get_path();
		if (lp)
			mvwprintw(g_win_main, cy++, ox + 2, "  Mix log  : %.*s",
				  iw - 12, lp);
	} else if (g_options_tab == 1) {
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " MASTER OUTPUT DEVICE%s",
			  g_audio_hp_picker ? "" : " [focused]");
		wattroff(g_win_main, A_BOLD);
		if (g_pcm_ndevices == 0)
			mvwprintw(
				g_win_main, cy++, ox + 4,
				"  No PCM devices found -- press R to rescan.");
		else {
			for (int di = 0;
			     di < g_pcm_ndevices && cy < oy + oh - 20; di++) {
				int is_active = (strcmp(g_pcm_devlist[di].dev,
							g_pcm_dev_str) == 0);
				int is_sel = (di == g_pcm_dev_sel &&
					      !g_audio_hp_picker);
				if (is_sel)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_ACTIVE) |
							A_BOLD);
				mvwprintw(g_win_main, cy++, ox + 4,
					  "  %s %-*.*s  [%s]",
					  is_active ? "\u25cf" : "\u25cb", 20,
					  20, g_pcm_devlist[di].name,
					  g_pcm_devlist[di].dev);
				if (is_sel)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_ACTIVE) |
							 A_BOLD);
			}
		}
		cy++;
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2,
			  " HEADPHONE OUTPUT DEVICE%s",
			  g_audio_hp_picker ? " [focused]" : "");
		wattroff(g_win_main, A_BOLD);
		if (g_pcm_ndevices == 0)
			mvwprintw(
				g_win_main, cy++, ox + 4,
				"  No PCM devices found -- press R to rescan.");
		else {
			for (int di = 0;
			     di < g_pcm_ndevices && cy < oy + oh - 14; di++) {
				int is_active = (strcmp(g_pcm_devlist[di].dev,
							g_pcm_hp_dev_str) == 0);
				int is_sel = (di == g_pcm_hp_dev_sel &&
					      g_audio_hp_picker);
				if (is_sel)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_ACTIVE) |
							A_BOLD);
				mvwprintw(g_win_main, cy++, ox + 4,
					  "  %s %-*.*s  [%s]",
					  is_active ? "\u25cf" : "\u25cb", 20,
					  20, g_pcm_devlist[di].name,
					  g_pcm_devlist[di].dev);
				if (is_sel)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_ACTIVE) |
							 A_BOLD);
			}
		}
		wattron(g_win_main, A_DIM);
		mvwprintw(
			g_win_main, cy++, ox + 4,
			"  j/k = select   ENTER = switch   H = toggle focus   R = rescan");
		wattroff(g_win_main, A_DIM);
		cy++;
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " VOLUME & PERFORMANCE");
		wattroff(g_win_main, A_BOLD);
		cy++;
		const char *labels[] = {
			"Default master vol", "Default deck vol",
			"Auto-gain default",  "Auto-gain target",
			"ECO Mode (Low CPU)", "Headphone vol"
		};
		float vals[] = { (float)g_opts.default_master_vol,
				 g_opts.default_deck_vol * 100.0f,
				 (float)g_opts.auto_gain_default,
				 g_opts.auto_gain_target_db,
				 (float)g_opts.eco_mode,
				 (float)g_hp_vol };
		const char *units[] = { "%",
					"%",
					"(0=off 1=on)",
					"dBFS",
					"(halve search samples)",
					"%" };
		for (int i = 0; i < 6; i++) {
			int sel = (g_options_sel == i);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			if (i == 2 || i == 4)
				mvwprintw(g_win_main, cy++, ox + 4,
					  "  %-18s : %s  %s", labels[i],
					  vals[i] > 0.5f ? "ON " : "OFF",
					  units[i]);
			else
				mvwprintw(g_win_main, cy++, ox + 4,
					  "  %-18s : %5.1f %s", labels[i],
					  (double)vals[i], units[i]);
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		}
		cy++;
		wattron(g_win_main, A_DIM);
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  j/k = select row   LEFT/RIGHT = adjust value");
		wattroff(g_win_main, A_DIM);
	} else if (g_options_tab == 2) {
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " WAVEFORM DISPLAY");
		wattroff(g_win_main, A_BOLD);
		cy++;
		const char *wfm_style_names[] = { "Kick/Snare", "Theme" };
		struct {
			const char *l;
			float *v;
			const char *u;
		} rows[] = { { "Waveform gamma", &g_opts.wfm_height_gamma,
			       "(0.5=sqrt 1.0=linear)" },
			     { "Visible seconds", &g_opts.wfm_visible_secs,
			       "secs" } };
		for (int i = 0; i < 2; i++) {
			int sel = (g_options_sel == i);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 4,
				  "  %-16s : %5.2f  %s", rows[i].l,
				  (double)*rows[i].v, rows[i].u);
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		}
		int sel2 = (g_options_sel == 2);
		if (sel2)
			wattron(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 4, "  %-16s : < %s >",
			  "Waveform style",
			  wfm_style_names[g_opts.wfm_style % 2]);
		if (sel2)
			wattroff(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		int sel3 = (g_options_sel == 3);
		if (sel3)
			wattron(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  %-16s : %3d FPS  (%d ms/frame)", "UI redraw rate",
			  g_opts.ui_fps,
			  1000 / (g_opts.ui_fps ? g_opts.ui_fps : 1));
		if (sel3)
			wattroff(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		cy++;
		wattron(g_win_main, A_DIM);
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  Kick/Snare: red=kick  blue=snare");
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  Theme:      amplitude bars in theme colours");
		cy++;
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  j/k = select   LEFT/RIGHT = adjust");
		wattroff(g_win_main, A_DIM);
	} else if (g_options_tab == 3) {
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " ADVANCED WAVEFORM");
		wattroff(g_win_main, A_BOLD);
		cy++;
		struct {
			const char *l;
			float *v;
			const char *h;
		} rows[] = { { "Bass weight", &g_opts.wfm_lo_weight,
			       "(kick/sub -- default 0.60)" },
			     { "Snare weight", &g_opts.wfm_mid_weight,
			       "(snare/perc -- default 0.40)" },
			     { "Treble weight", &g_opts.wfm_hi_weight,
			       "(hi-hat/air -- default 0.15)" },
			     { "Saturation", &g_opts.wfm_color_sat,
			       "(0.2=grey 1.0=normal 3.0=vivid)" },
			     { "Color floor", &g_opts.wfm_color_floor,
			       "(0.00=black gaps 0.06=def 0.15=lit)" } };
		for (int i = 0; i < 5; i++) {
			int sel = (g_options_sel == i);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 4,
				  "  %-16s : %5.2f  %s", rows[i].l,
				  (double)*rows[i].v, rows[i].h);
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		}
		const char *anchors[] = { "Centred (Serato)",
					  "Bottom (Rekordbox)" };
		int sel5 = (g_options_sel == 5);
		if (sel5)
			wattron(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 4, "  %-16s : < %s >",
			  "Bar anchor", anchors[g_opts.wfm_anchor % 2]);
		if (sel5)
			wattroff(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		cy++;
		wattron(g_win_main, A_DIM);
		mvwprintw(
			g_win_main, cy++, ox + 4,
			"  Band weights: raise bass to fix solid-colour dense music.");
		mvwprintw(
			g_win_main, cy++, ox + 4,
			"  Sat<1 = greyer; Sat>1 = more vivid kick/snare contrast.");
		mvwprintw(
			g_win_main, cy++, ox + 4,
			"  Floor=0.00 lets silent gaps go black between transients.");
		cy++;
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  j/k = select   LEFT/RIGHT = adjust");
		wattroff(g_win_main, A_DIM);
	} else if (g_options_tab == 4) {
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " SYNC OPTIONS");
		wattroff(g_win_main, A_BOLD);
		cy++;
		struct {
			const char *l;
			int *v;
		} rows[] = { { "Quantize play", &g_opts.sync_quantize },
			     { "Smart BPM range", &g_opts.sync_smart_range },
			     { "Auto handoff", &g_opts.sync_auto_handoff },
			     { "Key lock default", &g_opts.key_lock_default },
			     { "Vinyl mode", &g_opts.vinyl_mode },
			     { "Autoplay", &g_opts.library_autoplay },
			     { "Enable slicer", &g_opts.enable_slicer } };
		for (int i = 0; i < 7; i++) {
			int sel = (g_options_sel == i);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 4, "  %-16s : %s",
				  rows[i].l, *rows[i].v ? "ON " : "OFF");
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		}
		cy++;
		wattron(g_win_main, A_DIM);
		mvwprintw(
			g_win_main, cy++, ox + 4,
			"  Quantize: waits for bar-1 of leader before starting a synced deck.");
		mvwprintw(
			g_win_main, cy++, ox + 4,
			"  Smart range: folds BPM by octaves before sync to prevent jumps.");
		mvwprintw(
			g_win_main, cy++, ox + 4,
			"  Auto handoff: leader deck reloaded? next playing deck becomes leader.");
		mvwprintw(
			g_win_main, cy++, ox + 4,
			"  Key lock (K): pitch-preserving time-stretch default state.");
		mvwprintw(
			g_win_main, cy++, ox + 4,
			"  Vinyl mode: motorised platter scratch/velocity control toggle.");
		mvwprintw(
			g_win_main, cy++, ox + 4,
			"  Autoplay: load next track from Browser/Lib on completion.");
		cy++;
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  j/k = select   LEFT/RIGHT = toggle");
		wattroff(g_win_main, A_DIM);
	} else if (g_options_tab == 5) {
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " SELECT THEME");
		wattroff(g_win_main, A_BOLD);
		cy++;
		for (int i = 0; i < THEME_COUNT; i++) {
			int sel = (g_options_sel == i);
			int active = (i == g_opts.theme_idx);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			else if (active)
				wattron(g_win_main, A_BOLD);
			mvwprintw(g_win_main, cy, ox + 4, "  %s %s",
				  active ? "\u25cf" : "\u25cb",
				  g_themes[i].name);
			if (sel || active) {
				int sx = ox + 30;
				wattron(g_win_main,
					COLOR_PAIR(COLOR_HEADER) | A_BOLD);
				mvwprintw(g_win_main, cy, sx, "HDR");
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_HEADER) | A_BOLD);
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
				mvwprintw(g_win_main, cy, sx + 4, "ACT");
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
				wattron(g_win_main,
					COLOR_PAIR(COLOR_HOT) | A_BOLD);
				mvwprintw(g_win_main, cy, sx + 8, "HOT");
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_HOT) | A_BOLD);
				wattron(g_win_main, COLOR_PAIR(COLOR_VU));
				mvwprintw(g_win_main, cy, sx + 12, "VU ");
				wattroff(g_win_main, COLOR_PAIR(COLOR_VU));
			}
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			else if (active)
				wattroff(g_win_main, A_BOLD);
			cy++;
		}
		cy++;
		wattron(g_win_main, A_DIM);
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  j/k = select   ENTER or RIGHT = apply theme");
		mvwprintw(g_win_main, cy++, ox + 4,
			  "  Themes take effect immediately.");
		mvwprintw(
			g_win_main, cy++, ox + 4,
			"  Add custom themes in djcmd_config.h (see THEME_* macros).");
		wattroff(g_win_main, A_DIM);
	} else if (g_options_tab == 6) {
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " MIDI INPUT DEVICES");
		wattroff(g_win_main, A_BOLD);
		if (g_midi_ndevices == 0)
			mvwprintw(g_win_main, cy++, ox + 4,
				  "  No MIDI devices found.");
		else {
			for (int di = 0;
			     di < g_midi_ndevices && cy < oy + oh - 18; di++) {
				int is_active = (strcmp(g_midi_devlist[di].dev,
							g_midi_dev_str) == 0);
				int is_sel = (di == g_midi_dev_sel &&
					      !g_midi_learn_active &&
					      !g_midi_learn_jog_pair);
				if (is_sel)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_ACTIVE) |
							A_BOLD);
				else if (is_active)
					wattron(g_win_main, A_BOLD);
				char map_san[64];
				midi_map_name_from_device(
					g_midi_devlist[di].name, map_san,
					sizeof(map_san));
				int name_w = (ow > 52) ? ow - 52 : 4;
				mvwprintw(g_win_main, cy++, ox + 4,
					  "  %s %-*.*s [%.12s] map:%.14s.map",
					  is_active ? "\u25cf" : "\u25cb",
					  name_w, name_w,
					  g_midi_devlist[di].name,
					  g_midi_devlist[di].dev, map_san);
				if (is_sel)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_ACTIVE) |
							 A_BOLD);
				else if (is_active)
					wattroff(g_win_main, A_BOLD);
			}
		}
		cy++;
		if (g_midi_learn_active) {
			wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 2,
				  " >> LEARNING: %s <<",
				  g_mact_names[g_midi_learn_sel]);
			wattroff(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
			wattron(g_win_main, A_DIM);
			mvwprintw(
				g_win_main, cy++, ox + 4,
				"   Move the control or press the button. ESC=cancel.");
			wattroff(g_win_main, A_DIM);
		} else if (g_midi_learn_jog_pair) {
			wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 2,
				  " >> JOG LEARN DECK %c [%s] <<",
				  DECK_NUM(g_midi_learn_jog_deck),
				  (g_jog_type == JOG_NS7III) ? "ns7iii mode" :
							       "standard mode");
			wattroff(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
		} else {
			wattron(g_win_main, A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 2, " MIDI BINDINGS");
			wattroff(g_win_main, A_BOLD);
			int list_rows = oy + oh - cy - 3;
			int scroll = (g_options_sel > list_rows) ?
					     g_options_sel - list_rows :
					     0;
			int cur = 1;
			static const struct {
				const char *h;
				int f, l;
			} cats[] = { { "--- MIXER / FADERS ---",
				       MACT_DECK_VOL_A, MACT_BOOTH_VOL },
				     { "--- TRANSPORT / PLAY ---", MACT_PLAY_A,
				       MACT_CUE_ACTIVE_D },
				     { "--- CUE POINTS ---", MACT_CUE_SET_1,
				       MACT_CUE_DELETE_4 },
				     { "--- SYNC / NUDGE ---",
				       MACT_SYNC_FOLLOW_A, MACT_NUDGE_BACK_B },
				     { "--- LOOPS ---", MACT_LOOP_TOGGLE,
				       MACT_LOOP_HALF_D },
				     { "--- DECK TOGGLES ---", MACT_KEY_LOCK_A,
				       MACT_CENSOR_D },
				     { "--- JOG WHEEL ---", MACT_STRIP_A,
				       MACT_JOG_PB_D },
				     { "--- LIBRARY / BROWSER ---",
				       MACT_LIB_ENCODER, MACT_PANEL_LIBRARY },
				     { "--- HARDWARE CONTROL ---",
				       MACT_PITCH_RANGE_A, MACT_DECK_SEL_4 },
				     { "--- PADS / SHIFT ---", MACT_SHIFT_A,
				       MACT_PAD_8_B },
				     { "--- FX / UTILITY ---",
				       MACT_PARAM_LEFT_A, MACT_GRID_SNAP_B },
				     { "--- HI-RES LSB BINDINGS ---",
				       MACT_PITCH_LSB_A, MACT_MASTER_VOL_LSB },
				     { NULL, 0, 0 } };
			for (int ci = 0; cats[ci].h; ci++) {
				if (cur > scroll && cy < oy + oh - 3) {
					wattron(g_win_main,
						COLOR_PAIR(COLOR_HEADER) |
							A_BOLD);
					mvwprintw(g_win_main, cy++, ox + 2,
						  "%s", cats[ci].h);
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_HEADER) |
							 A_BOLD);
				}
				cur++;
				for (int ai = cats[ci].f; ai <= cats[ci].l;
				     ai++) {
					if (cur > scroll && cy < oy + oh - 3) {
						int sel =
							(g_options_sel == cur);
						const char *bound_str =
							" --unbound-- ";
						char bound_buf[32] = "";
						for (int bi = 0;
						     bi < g_midi_nbindings;
						     bi++) {
							if (g_midi_bindings[bi]
								    .action ==
							    (MidiAction)ai) {
								uint8_t st =
									g_midi_bindings[bi]
										.status;
								uint8_t d1 =
									g_midi_bindings[bi]
										.data1;
								uint8_t btype =
									st &
									0xF0;
								uint8_t bch =
									st &
									0x0F;
								snprintf(
									bound_buf,
									sizeof(bound_buf),
									"%s ch%d d1:%3d",
									(btype ==
									 0xB0) ?
										"CC    " :
									(btype ==
									 0x90) ?
										"NoteOn" :
										"Other ",
									bch + 1,
									d1);
								bound_str =
									bound_buf;
								break;
							}
						}
						char route_buf[24] = "";
						if ((MidiAction)ai ==
						    MACT_LIB_LOAD_A)
							snprintf(
								route_buf,
								sizeof(route_buf),
								" \u2192 Deck %c",
								DECK_NUM(
									g_side_deck
										[0]));
						else if ((MidiAction)ai ==
							 MACT_LIB_LOAD_B)
							snprintf(
								route_buf,
								sizeof(route_buf),
								" \u2192 Deck %c",
								DECK_NUM(
									g_side_deck
										[1]));
						else if ((MidiAction)ai ==
							 MACT_DECK_SEL_1)
							snprintf(
								route_buf,
								sizeof(route_buf),
								" [left \u2192 A]");
						else if ((MidiAction)ai ==
							 MACT_DECK_SEL_3)
							snprintf(
								route_buf,
								sizeof(route_buf),
								" [left \u2192 C]");
						else if ((MidiAction)ai ==
							 MACT_DECK_SEL_2)
							snprintf(
								route_buf,
								sizeof(route_buf),
								" [right \u2192 B]");
						else if ((MidiAction)ai ==
							 MACT_DECK_SEL_4)
							snprintf(
								route_buf,
								sizeof(route_buf),
								" [right \u2192 D]");
						if (sel)
							wattron(g_win_main,
								COLOR_PAIR(
									COLOR_ACTIVE) |
									A_BOLD);
						mvwprintw(g_win_main, cy++,
							  ox + 4,
							  "  %-16s %s%s",
							  g_mact_names[ai],
							  bound_str, route_buf);
						if (sel)
							wattroff(
								g_win_main,
								COLOR_PAIR(
									COLOR_ACTIVE) |
									A_BOLD);
					}
					cur++;
				}
				if (cy >= oy + oh - 3)
					break;
			}
			if (cy < oy + oh - 3) {
				if (cur > scroll) {
					int sel = (g_options_sel == cur);
					if (sel)
						wattron(g_win_main,
							COLOR_PAIR(
								COLOR_ACTIVE) |
								A_BOLD);
					mvwprintw(
						g_win_main, cy++, ox + 2,
						"  [ MIDI PANIC -- reset all faders/EQ ]");
					if (sel)
						wattroff(g_win_main,
							 COLOR_PAIR(
								 COLOR_ACTIVE) |
								 A_BOLD);
				}
				cur++;
			}
		}
	} else if (g_options_tab == 7) {
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " MIDI OUTPUT DEVICE");
		wattroff(g_win_main, A_BOLD);
		if (g_midi_out) {
			wattron(g_win_main, A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 4,
				  "  \u25CF %-*.*s [%.12s] (in+out)", iw - 32,
				  iw - 32,
				  (g_midi_dev_sel >= 0 &&
				   g_midi_dev_sel < g_midi_ndevices) ?
					  g_midi_devlist[g_midi_dev_sel].name :
					  "connected",
				  g_midi_dev_str);
			wattroff(g_win_main, A_BOLD);
		} else {
			wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
			mvwprintw(
				g_win_main, cy++, ox + 4,
				"  No MIDI output -- select device in MIDI IN tab");
			wattroff(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
		}
		cy++;
		if (g_motor_probe_open) {
			wattron(g_win_main, A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 2,
				  " MOTOR PROBE (M to close)");
			wattroff(g_win_main, A_BOLD);
			const char *types[] = { "CC", "NoteOn", "NoteOff" };
			mvwprintw(g_win_main, cy, ox + 4, "  Type: ");
			for (int ti = 0; ti < 3; ti++) {
				int sel = (ti == g_motor_probe_type);
				if (sel)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_ACTIVE) |
							A_BOLD);
				wprintw(g_win_main, "%s%s%s ", sel ? "[" : "",
					types[ti], sel ? "]" : "");
				if (sel)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_ACTIVE) |
							 A_BOLD);
			}
			mvwprintw(g_win_main, cy, ox + 4 + 40, " (T to cycle)");
			cy++;
			mvwprintw(g_win_main, cy, ox + 4, "  Ch: ");
			for (int ch = 1; ch <= 16; ch++) {
				int sel = (ch == g_motor_probe_ch);
				if (sel)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_ACTIVE) |
							A_BOLD);
				wprintw(g_win_main, "%s%d%s ", sel ? "[" : "",
					ch, sel ? "]" : "");
				if (sel)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_ACTIVE) |
							 A_BOLD);
			}
			cy++;
			mvwprintw(g_win_main, cy++, ox + 4,
				  "  d1 (CC/Note#): %-3d    d2 (value): %-3d",
				  g_motor_probe_cc, g_motor_probe_val);
			wattron(g_win_main, A_DIM);
			mvwprintw(
				g_win_main, cy++, ox + 4,
				"  SPACE=send(127) BKSP=send(0) V=send(val) X=sweep 0\u2192127");
			wattroff(g_win_main, A_DIM);
			mvwprintw(g_win_main, cy++, ox + 4, "  Log: %s",
				  g_motor_probe_log);
		} else {
			wattron(g_win_main, A_BOLD);
			mvwprintw(g_win_main, cy++, ox + 2,
				  " OUTPUT BINDINGS (%d configured)",
				  g_midi_nout_bindings);
			wattroff(g_win_main, A_BOLD);
			if (g_midi_nout_bindings == 0) {
				wattron(g_win_main, A_DIM);
				mvwprintw(
					g_win_main, cy++, ox + 4,
					"  No output bindings. Press M for motor probe.");
				wattroff(g_win_main, A_DIM);
			} else {
				int list_rows = oy + oh - cy - 4;
				int scroll = (g_options_out_sel >= list_rows) ?
						     g_options_out_sel -
							     list_rows + 1 :
						     0;
				for (int i = 0; i < g_midi_nout_bindings &&
						cy < oy + oh - 2;
				     i++) {
					if (i < scroll)
						continue;
					int sel = (i == g_options_out_sel);
					if (sel)
						wattron(g_win_main,
							COLOR_PAIR(
								COLOR_ACTIVE) |
								A_BOLD);
					MidiOutBinding *b =
						&g_midi_out_bindings[i];
					if (b->sysex_len > 0)
						mvwprintw(
							g_win_main, cy++,
							ox + 4,
							"  %-18s RGB pad=%-3d r=%-3d g=%-3d b=%-3d",
							b->name, b->data1,
							b->sysex_len > 7 ?
								b->sysex[7] :
								0,
							b->sysex_len > 8 ?
								b->sysex[8] :
								0,
							b->sysex_len > 9 ?
								b->sysex[9] :
								0);
					else
						mvwprintw(
							g_win_main, cy++,
							ox + 4,
							"  %-18s %02X d1=%-3d d2=%-3d",
							b->name, b->status,
							b->data1, b->data2);
					if (sel)
						wattroff(g_win_main,
							 COLOR_PAIR(
								 COLOR_ACTIVE) |
								 A_BOLD);
				}
			}
		}
	} else if (g_options_tab == 8) {
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2, " EFFECTS ENGINE");
		wattroff(g_win_main, A_BOLD);
		cy++;
		for (int dk = 0; dk < g_num_tracks && cy < oy + oh - 10; dk++) {
			mvwprintw(g_win_main, cy++, ox + 2, "  DECK %c",
				  'A' + dk);
			for (int sl = 0; sl < FX_SLOTS_PER_DECK; sl++) {
				int row = dk * FX_SLOTS_PER_DECK + sl;
				int sel = (g_options_sel == row);
				if (sel)
					wattron(g_win_main,
						COLOR_PAIR(COLOR_ACTIVE) |
							A_BOLD);
				FXSlot *fs = fx_slot(dk, sl);
				mvwprintw(
					g_win_main, cy++, ox + 4,
					"    Slot %d: %-12s W:%2.0f%% P0:%2.0f%% P1:%2.0f%% P2:%2.0f%%",
					sl + 1, fx_names[fs->type % FX_COUNT],
					(double)fs->params[3] * 100,
					(double)fs->params[0] * 100,
					(double)fs->params[1] * 100,
					(double)fs->params[2] * 100);
				if (sel)
					wattroff(g_win_main,
						 COLOR_PAIR(COLOR_ACTIVE) |
							 A_BOLD);
			}
		}
		cy++;
		wattron(g_win_main, A_BOLD);
		mvwprintw(g_win_main, cy++, ox + 2,
			  " SAMPLER SLOTS (ENTER to load highlighted)");
		wattroff(g_win_main, A_BOLD);
		for (int i = 0; i < 8 && cy < oy + oh - 2; i++) {
			int row = g_num_tracks * FX_SLOTS_PER_DECK + i;
			int sel = (g_options_sel == row);
			if (sel)
				wattron(g_win_main,
					COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
			SamplerSlot *s = &g_samplers[i];
			mvwprintw(g_win_main, cy++, ox + 4,
				  "    Sampler %d: %-24.24s Vol:%3.0f%% [%s]",
				  i + 1, s->data ? s->name : "-- empty --",
				  (double)s->volume * 100,
				  s->playing ? "PLAYING" : "STOPPED");
			if (sel)
				wattroff(g_win_main,
					 COLOR_PAIR(COLOR_ACTIVE) | A_BOLD);
		}
	}
}

void draw_status(void)
{
	wattron(g_win_status, COLOR_PAIR(COLOR_STATUS));
	werase(g_win_status);
	mvwprintw(
		g_win_status, 0, 0,
		" djcmd | Deck:%c | Vol:%3d%% | XF:%.2f | Gang:%s | MIDI:%s | AUTO:%s",
		'A' + g_active_track, g_master_vol, (double)g_crossfader,
		g_gang_mode ? "ON" : "off", g_midi_in ? "ON" : "off",
		g_opts.library_autoplay ? "ON" : "OFF");
	time_t _now = time(NULL);
	struct tm *_lt = localtime(&_now);
	wattron(g_win_status, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	mvwprintw(g_win_status, 0, g_cols - 6, " %02d:%02d", _lt->tm_hour,
		  _lt->tm_min);
	wattroff(g_win_status, COLOR_PAIR(COLOR_HEADER) | A_BOLD);
	wrefresh(g_win_status);
}

static void options_adjust(int direction)
{
	if (g_options_tab == 1) {
		switch (g_options_sel) {
		case 0:
			g_opts.default_master_vol += direction * 5;
			if (g_opts.default_master_vol < 0)
				g_opts.default_master_vol = 0;
			if (g_opts.default_master_vol > 150)
				g_opts.default_master_vol = 150;
			g_master_vol = g_opts.default_master_vol;
			break;
		case 1:
			g_opts.default_deck_vol += direction * 0.05f;
			if (g_opts.default_deck_vol < 0.0f)
				g_opts.default_deck_vol = 0.0f;
			if (g_opts.default_deck_vol > 1.5f)
				g_opts.default_deck_vol = 1.5f;
			break;
		case 2:
			g_opts.auto_gain_default = !g_opts.auto_gain_default;
			break;
		case 3:
			g_opts.auto_gain_target_db += direction * 1.0f;
			if (g_opts.auto_gain_target_db < -24.0f)
				g_opts.auto_gain_target_db = -24.0f;
			if (g_opts.auto_gain_target_db > 0.0f)
				g_opts.auto_gain_target_db = 0.0f;
			break;
		case 4:
			g_opts.eco_mode = !g_opts.eco_mode;
			break;
		case 5:
			g_hp_vol += direction * 5;
			if (g_hp_vol < 0)
				g_hp_vol = 0;
			if (g_hp_vol > 150)
				g_hp_vol = 150;
			break;
		}
	} else if (g_options_tab == 2) {
		switch (g_options_sel) {
		case 0:
			g_opts.wfm_height_gamma += direction * 0.05f;
			if (g_opts.wfm_height_gamma < 0.2f)
				g_opts.wfm_height_gamma = 0.2f;
			if (g_opts.wfm_height_gamma > 3.0f)
				g_opts.wfm_height_gamma = 3.0f;
			break;
		case 1:
			g_opts.wfm_visible_secs += direction * 1.0f;
			if (g_opts.wfm_visible_secs < 2.0f)
				g_opts.wfm_visible_secs = 2.0f;
			if (g_opts.wfm_visible_secs > 30.0f)
				g_opts.wfm_visible_secs = 30.0f;
			break;
		case 2:
			g_opts.wfm_style =
				(g_opts.wfm_style + 2 + direction) % 2;
			break;
		case 3:
			g_opts.ui_fps += direction * 5;
			if (g_opts.ui_fps < 10)
				g_opts.ui_fps = 10;
			if (g_opts.ui_fps > 120)
				g_opts.ui_fps = 120;
			apply_ui_fps();
			break;
		}
	} else if (g_options_tab == 3) {
		switch (g_options_sel) {
		case 0:
			g_opts.wfm_lo_weight += direction * 0.05f;
			break;
		case 1:
			g_opts.wfm_mid_weight += direction * 0.05f;
			break;
		case 2:
			g_opts.wfm_hi_weight += direction * 0.05f;
			break;
		case 3:
			g_opts.wfm_color_sat += direction * 0.1f;
			break;
		case 4:
			g_opts.wfm_color_floor += direction * 0.01f;
			break;
		case 5:
			g_opts.wfm_anchor =
				(g_opts.wfm_anchor + 2 + direction) % 2;
			break;
		}
	} else if (g_options_tab == 4) {
		switch (g_options_sel) {
		case 0:
			g_opts.sync_quantize = !g_opts.sync_quantize;
			break;
		case 1:
			g_opts.sync_smart_range = !g_opts.sync_smart_range;
			break;
		case 2:
			g_opts.sync_auto_handoff = !g_opts.sync_auto_handoff;
			break;
		case 3:
			g_opts.key_lock_default = !g_opts.key_lock_default;
			break;
		case 4:
			g_opts.vinyl_mode = !g_opts.vinyl_mode;
			break;
		case 5:
			g_opts.library_autoplay = !g_opts.library_autoplay;
			break;
		case 6:
			g_opts.enable_slicer = !g_opts.enable_slicer;
			break;
		}
	} else if (g_options_tab == 8) {
		int fx_total = g_num_tracks * FX_SLOTS_PER_DECK;
		if (g_options_sel < fx_total) {
			int dk = g_options_sel / FX_SLOTS_PER_DECK,
			    sl = g_options_sel % FX_SLOTS_PER_DECK;
			FXSlot *fs = fx_slot(dk, sl);
			fs->params[3] += direction * 0.05f;
			if (fs->params[3] < 0.0f)
				fs->params[3] = 0.0f;
			if (fs->params[3] > 1.0f)
				fs->params[3] = 1.0f;
		} else {
			int si = g_options_sel - fx_total;
			if (si >= 0 && si < 8) {
				g_samplers[si].volume += direction * 0.05f;
				if (g_samplers[si].volume < 0.0f)
					g_samplers[si].volume = 0.0f;
				if (g_samplers[si].volume > 1.5f)
					g_samplers[si].volume = 1.5f;
			}
		}
	}
	settings_save();
}

static void quit_confirm(void)
{
	int any = 0;
	for (int i = 0; i < g_num_tracks; i++)
		if (g_tracks[i].playing)
			any = 1;
	if (!any) {
		g_running = 0;
		return;
	}
	g_quit_pending = 1;
}

static void draw_quit_modal(void)
{
	const char *msg = " Track playing. Quit? ";
	const char *hint = " Y = yes N / ESC = cancel ";
	int pw = (int)strlen(hint) + 4;
	int ph = 5;
	int px = (g_cols - pw) / 2;
	int py = (g_rows - ph) / 2;
	wattron(g_win_main, COLOR_PAIR(COLOR_STATUS));
	for (int r = py; r < py + ph; r++)
		for (int c = px; c < px + pw; c++)
			mvwaddch(g_win_main, r, c, ' ');
	wattron(g_win_main, COLOR_PAIR(COLOR_HOT) | A_BOLD);
	mvwprintw(g_win_main, py + 1, px + (pw - (int)strlen(msg)) / 2, "%s",
		  msg);
	mvwprintw(g_win_main, py + 3, px + (pw - (int)strlen(hint)) / 2, "%s",
		  hint);
}

void redraw(void)
{
	g_blink_tick++;
	if (g_midi_out)
		deck_leds_refresh();
	getmaxyx(stdscr, g_rows, g_cols);
	wresize(g_win_main, g_rows - 1, g_cols);
	wresize(g_win_status, 1, g_cols);
	mvwin(g_win_status, g_rows - 1, 0);
	werase(g_win_main);
	switch (g_view) {
	case 0:
		draw_decks_view();
		break;
	case 1:
		if (library_rows_available() >= 4)
			draw_split_view();
		else
			draw_full_panel_view(0, g_rows - 1);
		break;
	case 2:
		draw_help_view();
		break;
	}
	if (g_tag_info.visible)
		draw_tag_panel();
	if (g_options_open)
		draw_options_overlay();
	if (g_quit_pending)
		draw_quit_modal();
	wrefresh(g_win_main);
	draw_status();
}

/* ── Menu mapping helper ── */
static int menu_to_mact(int sel)
{
	static const struct {
		MidiAction f, l;
	} cats_local[] = { { MACT_DECK_VOL_A, MACT_BOOTH_VOL },
			   { MACT_PLAY_A, MACT_CUE_ACTIVE_D },
			   { MACT_CUE_SET_1, MACT_CUE_DELETE_4 },
			   { MACT_SYNC_FOLLOW_A, MACT_NUDGE_BACK_B },
			   { MACT_LOOP_TOGGLE, MACT_LOOP_HALF_D },
			   { MACT_KEY_LOCK_A, MACT_CENSOR_D },
			   { MACT_STRIP_A, MACT_JOG_PB_D },
			   { MACT_LIB_ENCODER, MACT_PANEL_LIBRARY },
			   { MACT_PITCH_RANGE_A, MACT_DECK_SEL_4 },
			   { MACT_SHIFT_A, MACT_PAD_8_B },
			   { MACT_PARAM_LEFT_A, MACT_GRID_SNAP_B },
			   { MACT_PITCH_LSB_A, MACT_MASTER_VOL_LSB },
			   { 0, 0 } };
	int cur = 1;
	for (int i = 0; cats_local[i].f != 0; i++) {
		cur++; /* skip header */
		int len = cats_local[i].l - cats_local[i].f + 1;
		if (sel >= cur && sel < cur + len)
			return cats_local[i].f + (sel - cur);
		cur += len;
	}
	if (sel == cur)
		return -2; /* Panic button marker */
	return -1; /* header or out of bounds */
}

void handle_key(int c)
{
	Track *t = &g_tracks[g_active_track];

	/* Quit confirm modal intercepts all keys */
	if (g_quit_pending) {
		if (c == 'y' || c == 'Y') {
			g_running = 0;
		} else {
			g_quit_pending = 0;
		}
		return;
	}

	if (g_options_open) {
		int total_midi_items = 0;
		if (g_options_tab == 6) {
			static const struct {
				int f, l;
			} cats_local[] = {
				{ MACT_DECK_VOL_A, MACT_BOOTH_VOL },
				{ MACT_PLAY_A, MACT_CUE_ACTIVE_D },
				{ MACT_CUE_SET_1, MACT_CUE_DELETE_4 },
				{ MACT_SYNC_FOLLOW_A, MACT_NUDGE_BACK_B },
				{ MACT_LOOP_TOGGLE, MACT_LOOP_HALF_D },
				{ MACT_KEY_LOCK_A, MACT_CENSOR_D },
				{ MACT_STRIP_A, MACT_JOG_PB_D },
				{ MACT_LIB_ENCODER, MACT_PANEL_LIBRARY },
				{ MACT_PITCH_RANGE_A, MACT_DECK_SEL_4 },
				{ MACT_SHIFT_A, MACT_PAD_8_B },
				{ MACT_PARAM_LEFT_A, MACT_GRID_SNAP_B },
				{ MACT_PITCH_LSB_A, MACT_MASTER_VOL_LSB },
				{ 0, 0 }
			};
			for (int i = 0; cats_local[i].f != 0; i++) {
				total_midi_items +=
					(cats_local[i].l - cats_local[i].f + 2);
			}
			total_midi_items++; /* Panic */
		}
		int max_sel = (g_options_tab == 0) ? 0 :
			      (g_options_tab == 1) ? 5 :
			      (g_options_tab == 2) ? 3 :
			      (g_options_tab == 3) ? 5 :
			      (g_options_tab == 4) ? 6 :
			      (g_options_tab == 5) ? THEME_COUNT - 1 :
			      (g_options_tab == 6) ? total_midi_items :
			      (g_options_tab == 7) ? g_midi_nout_bindings - 1 :
						     (g_num_tracks *
						      FX_SLOTS_PER_DECK) +
							     8 - 1;
		switch (c) {
		case 27: /* ESC */
			if (g_midi_learn_active) {
				g_midi_learn_active = 0;
				break;
			}
			if (g_midi_learn_jog_pair) {
				g_midi_learn_jog_pair = 0;
				break;
			}
			if (g_motor_probe_open) {
				g_motor_probe_open = 0;
				break;
			}
			if (g_options_tab == 5)
				apply_theme(g_opts.theme_idx);
			g_options_open = 0;
			break;
		case KEY_LEFT:
			g_options_tab = (g_options_tab + 8) % 9;
			g_options_sel =
				(g_options_tab == 5) ? g_opts.theme_idx : 0;
			break;
		case KEY_RIGHT:
			g_options_tab = (g_options_tab + 1) % 9;
			g_options_sel =
				(g_options_tab == 5) ? g_opts.theme_idx : 0;
			break;
		case KEY_DOWN:
		case 'j':
			if (g_options_tab == 7 && !g_motor_probe_open) {
				if (g_options_out_sel <
				    g_midi_nout_bindings - 1)
					g_options_out_sel++;
			} else {
				g_options_sel++;
				if (g_options_sel > max_sel)
					g_options_sel =
						(g_options_tab == 6) ? 1 : 0;
				if (g_options_tab == 6 &&
				    menu_to_mact(g_options_sel) == -1)
					g_options_sel++;
				if (g_options_tab == 5)
					apply_theme(g_options_sel);
			}
			break;
		case KEY_UP:
		case 'k':
			if (g_options_tab == 7 && !g_motor_probe_open) {
				if (g_options_out_sel > 0)
					g_options_out_sel--;
			} else {
				g_options_sel--;
				if (g_options_sel <
				    ((g_options_tab == 6) ? 1 : 0))
					g_options_sel = max_sel;
				if (g_options_tab == 6 &&
				    menu_to_mact(g_options_sel) == -1)
					g_options_sel--;
				if (g_options_tab == 5)
					apply_theme(g_options_sel);
			}
			break;
		case '-':
			options_adjust(-1);
			break;
		case '=':
		case '+':
			options_adjust(+1);
			break;
		case '\n':
		case KEY_ENTER:
			if (g_options_tab == 1) {
				if (g_audio_hp_picker)
					strncpy(g_pcm_hp_dev_str,
						g_pcm_devlist[g_pcm_hp_dev_sel]
							.dev,
						63);
				else
					strncpy(g_pcm_dev_str,
						g_pcm_devlist[g_pcm_dev_sel]
							.dev,
						63);
				settings_save();
			} else if (g_options_tab == 5) {
				g_opts.theme_idx = g_options_sel;
				apply_theme(g_opts.theme_idx);
				settings_save();
			} else if (g_options_tab == 6) {
				if (g_options_sel == total_midi_items) {
					for (int i = 0; i < g_num_tracks; i++) {
						g_tracks[i].volume = 1.0f;
						g_tracks[i].eq_low = 0.0f;
						g_tracks[i].eq_mid = 0.0f;
						g_tracks[i].eq_high = 0.0f;
					}
				} else if (!g_midi_learn_active) {
					g_midi_learn_sel =
						menu_to_mact(g_options_sel);
					if (g_midi_learn_sel >= 0)
						g_midi_learn_active = 1;
				}
			} else if (g_options_tab == 8) {
				int fx_total = g_num_tracks * FX_SLOTS_PER_DECK;
				if (g_options_sel >= fx_total) {
					int si = g_options_sel - fx_total;
					if (si >= 0 && si < 8) {
						char full[1024];
						fb_selected_path(full,
								 sizeof(full));
						if (full[0]) {
							load_sampler(
								&g_samplers[si],
								full);
							snprintf(
								g_fb_status,
								sizeof(g_fb_status),
								"Loaded to Sampler %d: %.200s",
								si + 1,
								strrchr(full,
									'/') ?
									strrchr(full,
										'/') +
										1 :
									full);
						}
					}
				}
			}
			break;
		case 'h':
		case 'H':
			if (g_options_tab == 1)
				g_audio_hp_picker = !g_audio_hp_picker;
			break;
		case 'r':
		case 'R':
			if (g_options_tab == 1)
				pcm_enumerate_devices();
			if (g_options_tab == 6 || g_options_tab == 7)
				midi_enumerate_devices();
			break;
		case 'w':
		case 'W':
			if (g_options_tab == 6 && !g_midi_learn_active) {
				int mact = menu_to_mact(g_options_sel);
				if (mact >= MACT_JOG_TOUCH_A &&
				    mact <= MACT_JOG_PB_D) {
					g_midi_learn_jog_deck =
						(mact - MACT_JOG_TOUCH_A) % 4;
					g_midi_learn_jog_pair = 1;
				}
			}
			break;
		case 'u':
		case 'U':
			if (g_options_tab == 6) {
				int mact = menu_to_mact(g_options_sel);
				if (mact >= 1)
					midi_bind(0, 0, (MidiAction)mact);
			}
			break;
		case 'm':
		case 'M':
			if (g_options_tab == 6 || g_options_tab == 7)
				g_motor_probe_open = !g_motor_probe_open;
			break;
		case ' ':
			if (g_motor_probe_open)
				motor_probe_send(127);
			break;
		case KEY_BACKSPACE:
		case 127:
		case 8:
			if (g_motor_probe_open)
				motor_probe_send(0);
			break;
		case 's':
		case 'S':
			if (g_options_tab == 6 || g_options_tab == 7)
				midi_map_save();
			settings_save();
			break;
		case 'Q':
			quit_confirm();
			break;
		}
		return;
	}

	/* Track consecutive Ctrl+U presses for manual USB rescan.
	 * Any other key resets the sequence. */
	static int g_usb_rescan_seq = 0;
	if (c != 21)
		g_usb_rescan_seq = 0;

	/* ── Manual BPM entry mode ──────────────────────────────────────────
     * Activated by 'B'.  Collects digits; Enter commits, Esc cancels.
     * Backspace deletes last digit.  Dots are accepted for decimal input.
     * Updates the deck's BPM and resets the beat offset to 0 on commit. */
	if (g_bpm_entry) {
		if (c == '\n' || c == '\r' || c == KEY_ENTER) {
			float b = (float)atof(g_bpm_buf);
			if (b >= 40.0f && b <= 250.0f) {
				pthread_mutex_lock(&g_tracks[g_bpm_deck].lock);
				g_tracks[g_bpm_deck].bpm = b;
				g_tracks[g_bpm_deck].bpm_offset =
					(float)g_tracks[g_bpm_deck].pos;
				pthread_mutex_unlock(
					&g_tracks[g_bpm_deck].lock);
				sidecar_save(&g_tracks[g_bpm_deck]);
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Deck %c BPM set to %.1f",
					 DECK_NUM(g_bpm_deck), (double)b);
			}
			g_bpm_entry = 0;
		} else if (c == 27) { /* ESC */
			g_bpm_entry = 0;
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "BPM entry cancelled");
		} else if (c == 'B' || c == 'b') {
			/* B again = trigger manual Auto-Scan */
			g_bpm_entry = 0;
			enqueue_analyze(g_bpm_deck);
		} else if (c == KEY_BACKSPACE || c == 127 || c == 8) {
			int len = (int)strlen(g_bpm_buf);
			if (len > 0)
				g_bpm_buf[len - 1] = '\0';
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "BPM Deck %c: %s_", DECK_NUM(g_bpm_deck),
				 g_bpm_buf);
		} else if ((c >= '0' && c <= '9') || c == '.') {
			int len = (int)strlen(g_bpm_buf);
			if (len < (int)sizeof(g_bpm_buf) - 1) {
				g_bpm_buf[len] = (char)c;
				g_bpm_buf[len + 1] = '\0';
			}
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "BPM Deck %c: %s_", DECK_NUM(g_bpm_deck),
				 g_bpm_buf);
		}
		return;
	}

	/* ── Batch BPM range prompt ──────────────────────────────────────────
     * g_batch_prompt_active=1 while user types the lo/hi range.
     * ENTER starts the batch; ESC cancels.  During a running batch (g_batch_running)
     * only ESC is handled here (to abort). */
	if (g_batch_prompt_active) {
		char *buf = (g_batch_prompt_field == 0) ? g_batch_lo_buf :
							  g_batch_hi_buf;
		if (c == '\t') {
			g_batch_prompt_field ^= 1;
		} else if (c == '\n' || c == '\r' || c == KEY_ENTER) {
			float lo = (float)atof(g_batch_lo_buf);
			float hi = (float)atof(g_batch_hi_buf);
			if (lo >= 20.0f && hi > lo) {
				g_batch_prompt_active = 0;
				batch_analyze_start(lo, hi);
			} else {
				snprintf(
					g_fb_status, sizeof(g_fb_status),
					"Batch BPM: invalid range (lo=%.0f hi=%.0f)",
					(double)lo, (double)hi);
			}
		} else if (c == 27) { /* ESC */
			g_batch_prompt_active = 0;
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Batch BPM analyze cancelled");
		} else if (c == KEY_BACKSPACE || c == 127 || c == 8) {
			int len = (int)strlen(buf);
			if (len > 0)
				buf[len - 1] = '\0';
		} else if (c >= '0' && c <= '9') {
			int len = (int)strlen(buf);
			if (len < 15) {
				buf[len] = (char)c;
				buf[len + 1] = '\0';
			}
		}
		return;
	}
	if (g_batch_running && c == 27) {
		/* ESC during running batch: cancel */
		g_batch_running = 0;
		g_bpm_detect_lo = 0.0f;
		g_bpm_detect_hi = 0.0f;
		batch_free_queue();
		snprintf(g_fb_status, sizeof(g_fb_status),
			 "Batch BPM analyze cancelled");
		return;
	}

	/* ── Crate jump mode ─────────────────────────────────────────── */
	if (g_crate_jump_active) {
		if (c == '\n' || c == '\r' || c == KEY_ENTER) {
			crate_jump(g_crate_input);
			g_crate_jump_active = 0;
			g_crate_input[0] = '\0';
			g_crate_cycle_idx = -1;
		} else if (c == 27) { /* ESC */
			g_crate_jump_active = 0;
			g_crate_input[0] = '\0';
			g_crate_cycle_idx = -1;
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Crate jump cancelled");
		} else if (c == '\t') { /* TAB Cycling */
			if (g_ncrate_matches > 0) {
				g_crate_cycle_idx = (g_crate_cycle_idx + 1) %
						    g_ncrate_matches;
				int cidx = g_crate_matches[g_crate_cycle_idx];
				strncpy(g_crate_input, g_crates[cidx].alias,
					sizeof(g_crate_input) - 1);
				update_crate_matches(g_crate_orig_input,
						     "Jump to:");
			}
		} else if (c == KEY_BACKSPACE || c == 127 || c == 8) {
			int len = (int)strlen(g_crate_input);
			if (len > 0)
				g_crate_input[len - 1] = '\0';
			strncpy(g_crate_orig_input, g_crate_input,
				sizeof(g_crate_orig_input) - 1);
			g_crate_orig_input[sizeof(g_crate_orig_input) - 1] =
				'\0';
			g_crate_cycle_idx = -1;
			update_crate_matches(g_crate_input, "Jump to:");
		} else if (c >= 32 && c <= 126) {
			int len = (int)strlen(g_crate_input);
			if (len < (int)sizeof(g_crate_input) - 1) {
				g_crate_input[len] = (char)c;
				g_crate_input[len + 1] = '\0';
			}
			strncpy(g_crate_orig_input, g_crate_input,
				sizeof(g_crate_orig_input) - 1);
			g_crate_orig_input[sizeof(g_crate_orig_input) - 1] =
				'\0';
			g_crate_cycle_idx = -1;
			update_crate_matches(g_crate_input, "Jump to:");
		}
		return;
	}

	/* ── Crate add mode (Directory creation) ── */
	if (g_crate_add_active) {
		if (c == '\n' || c == '\r' || c == KEY_ENTER) {
			if (g_crate_add_input[0]) {
				crate_create(g_crate_add_input);
			}
			g_crate_add_active = 0;
			g_crate_add_input[0] = '\0';
		} else if (c == 27) { /* ESC */
			g_crate_add_active = 0;
			g_crate_add_input[0] = '\0';
			g_crate_add_path[0] = '\0';
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Crate create cancelled");
		} else if (c == KEY_BACKSPACE || c == 127 || c == 8) {
			int len = (int)strlen(g_crate_add_input);
			if (len > 0)
				g_crate_add_input[len - 1] = '\0';
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "New crate: %s_", g_crate_add_input);
		} else if (c >= 32 && c <= 126) {
			int len = (int)strlen(g_crate_add_input);
			if (len < (int)sizeof(g_crate_add_input) - 1) {
				g_crate_add_input[len] = (char)c;
				g_crate_add_input[len + 1] = '\0';
			}
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "New crate: %s_", g_crate_add_input);
		}
		return;
	}

	/* ── Add Track to Crate mode (File Assignment) ── */
	if (g_track_add_crate_active) {
		if (c == '\n' || c == '\r' || c == KEY_ENTER) {
			if (g_track_add_crate_input[0]) {
				int idx = -1;
				for (int i = 0; i < g_ncrate; i++) {
					if (strcasecmp(
						    g_crates[i].name,
						    g_track_add_crate_input) ==
					    0) {
						idx = i;
						break;
					}
				}
				if (idx < 0) {
					crate_create(g_track_add_crate_input);
					for (int i = 0; i < g_ncrate; i++) {
						if (strcasecmp(
							    g_crates[i].name,
							    g_track_add_crate_input) ==
						    0) {
							idx = i;
							break;
						}
					}
				}
				if (idx >= 0) {
					if (g_crates[idx].is_usb)
						snprintf(
							g_fb_status,
							sizeof(g_fb_status),
							"Cannot add directly to a USB crate"
							" \u2014 export a local crate instead");
					else
						crate_add_to(
							idx,
							g_pending_track_path);
				}
			}
			g_track_add_crate_active = 0;
			g_track_add_crate_input[0] = '\0';
			g_crate_cycle_idx = -1;
		} else if (c == 27) { /* ESC */
			g_track_add_crate_active = 0;
			g_track_add_crate_input[0] = '\0';
			g_crate_cycle_idx = -1;
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Add to crate cancelled");
		} else if (c == '\t') { /* TAB Cycling */
			if (g_ncrate_matches > 0) {
				g_crate_cycle_idx = (g_crate_cycle_idx + 1) %
						    g_ncrate_matches;
				int cidx = g_crate_matches[g_crate_cycle_idx];
				strncpy(g_track_add_crate_input,
					g_crates[cidx].name,
					sizeof(g_track_add_crate_input) - 1);
				update_crate_matches(g_crate_orig_input,
						     "Add to crate:");
			}
		} else if (c == KEY_BACKSPACE || c == 127 || c == 8) {
			int len = (int)strlen(g_track_add_crate_input);
			if (len > 0)
				g_track_add_crate_input[len - 1] = '\0';
			strncpy(g_crate_orig_input, g_track_add_crate_input,
				sizeof(g_track_add_crate_input) - 1);
			g_crate_orig_input[sizeof(g_crate_orig_input) - 1] =
				'\0';
			g_crate_cycle_idx = -1;
			update_crate_matches(g_track_add_crate_input,
					     "Add to crate:");
		} else if (c >= 32 && c <= 126) {
			int len = (int)strlen(g_track_add_crate_input);
			if (len < (int)sizeof(g_track_add_crate_input) - 1) {
				g_track_add_crate_input[len] = (char)c;
				g_track_add_crate_input[len + 1] = '\0';
			}
			strncpy(g_crate_orig_input, g_track_add_crate_input,
				sizeof(g_track_add_crate_input) - 1);
			g_crate_orig_input[sizeof(g_crate_orig_input) - 1] =
				'\0';
			g_crate_cycle_idx = -1;
			update_crate_matches(g_track_add_crate_input,
					     "Add to crate:");
		}
		return;
	}

	/* ── USB eject confirmation ── */
	if (g_usb_eject_active) {
		if (c == 'y' || c == 'Y' || c == '\n' || c == '\r' ||
		    c == KEY_ENTER) {
			if (usb_eject(g_usb_eject_mount) == 0)
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Ejected %s", g_usb_eject_label);
			else
				snprintf(
					g_fb_status, sizeof(g_fb_status),
					"Eject failed for %s — check if tracks are in use",
					g_usb_eject_label);
			g_usb_eject_active = 0;
			g_usb_eject_mount[0] = '\0';
			crates_load();
		} else {
			g_usb_eject_active = 0;
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Eject cancelled");
		}
		return;
	}

	/* ── USB conflict rename prompt ── */
	if (g_usb_conflict_active) {
		if (c == '\n' || c == '\r' || c == KEY_ENTER) {
			if (g_usb_conflict_rename[0]) {
				char status[256];
				usb_export_crate(
					g_crates[g_usb_export_crate_idx]
						.filename,
					g_crates[g_usb_export_crate_idx].name,
					g_machine_id, g_usb_conflict_mount,
					g_usb_conflict_rename, status,
					sizeof(status));
				snprintf(g_fb_status, sizeof(g_fb_status), "%s",
					 status);
				crates_load();
			}
			g_usb_conflict_active = 0;
			g_usb_conflict_rename[0] = '\0';
			g_usb_conflict_crate_name[0] = '\0';
		} else if (c == 27) {
			g_usb_conflict_active = 0;
			g_usb_conflict_rename[0] = '\0';
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Export cancelled");
		} else if (c == KEY_BACKSPACE || c == 127 || c == 8) {
			int len = (int)strlen(g_usb_conflict_rename);
			if (len > 0)
				g_usb_conflict_rename[len - 1] = '\0';
		} else if (c >= 32 && c <= 126) {
			int len = (int)strlen(g_usb_conflict_rename);
			if (len < 63) {
				g_usb_conflict_rename[len] = (char)c;
				g_usb_conflict_rename[len + 1] = '\0';
			}
		}
		return;
	}

	/* ── USB device picker (multiple USBs detected) ── */
	if (g_usb_picker_active) {
		if (c == 'j' || c == KEY_DOWN || c == KEY_RIGHT) {
			if (g_usb_picker_sel < g_usb_devices_count - 1)
				g_usb_picker_sel++;
		} else if (c == 'k' || c == KEY_UP || c == KEY_LEFT) {
			if (g_usb_picker_sel > 0)
				g_usb_picker_sel--;
		} else if (c == '\n' || c == '\r' || c == KEY_ENTER) {
			const char *mount =
				g_usb_devices[g_usb_picker_sel].mount_point;
			const char *cname =
				g_crates[g_usb_export_crate_idx].name;
			char usb_crate_path[1024];
			snprintf(usb_crate_path, sizeof(usb_crate_path),
				 "%s/%s/%s.crate", mount, DJCMD_USB_CRATES,
				 cname);
			char remote_origin[64] = "";
			usb_crate_read_origin(usb_crate_path, remote_origin,
					      sizeof(remote_origin));
			struct stat _usb_st2;
			int crate_exists2 =
				(stat(usb_crate_path, &_usb_st2) == 0);
			g_usb_picker_active = 0;
			if ((remote_origin[0] &&
			     strcmp(remote_origin, g_machine_id) != 0) ||
			    (crate_exists2 && !remote_origin[0])) {
				/* Conflict: different machine, or file exists without origin header */
				g_usb_conflict_active = 1;
				strncpy(g_usb_conflict_mount, mount,
					sizeof(g_usb_conflict_mount) - 1);
				strncpy(g_usb_conflict_crate_name, cname,
					sizeof(g_usb_conflict_crate_name) - 1);
				g_usb_conflict_rename[0] = '\0';
			} else {
				char status[256];
				usb_export_crate(
					g_crates[g_usb_export_crate_idx]
						.filename,
					cname, g_machine_id, mount, cname,
					status, sizeof(status));
				snprintf(g_fb_status, sizeof(g_fb_status), "%s",
					 status);
				crates_load();
			}
		} else if (c == 27) {
			g_usb_picker_active = 0;
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Export cancelled");
		}
		return;
	}

	if (g_view == 2) {
		switch (c) {
		case 'j':
		case KEY_DOWN:
			g_help_scroll++;
			break;
		case 'k':
		case KEY_UP:
			g_help_scroll--;
			break;
		case KEY_NPAGE: /* PgDn */
			g_help_scroll += (g_rows - 4);
			break;
		case KEY_PPAGE: /* PgUp */
			g_help_scroll -= (g_rows - 4);
			break;
		case KEY_HOME:
		case 'g':
			g_help_scroll = 0;
			break;
		case KEY_END:
		case 'G':
			g_help_scroll = 9999;
			break; /* clamped in draw */
		case '?':
		case 27: /* ESC closes help */
			g_view = 1;
			g_help_scroll = 0;
			break;
		case 'Q': /* fall through to quit check below */
			break;
		default:
			return; /* swallow unhandled keys in help view */
		}
		if (c != 'Q')
			return;
	}

	/* ── Options overlay intercepts most keys ── */
	if (g_options_open) {
		int total_midi_items = 0;
		if (g_options_tab == 6) {
			int ci = 0;
			/* Categorized counts from 'cats' array in options_draw */
			int counts[] = { 39, 8,	 12, 8,	 17, 16, 16,
					 11, 24, 26, 27, 31, 0 };
			while (counts[ci] > 0) {
				total_midi_items +=
					counts[ci] + 1; /* header + actions */
				ci++;
			}
			total_midi_items++; /* Panic */
		}

		int max_sel =
			(g_options_tab == 0) ?
				0 :
			(g_options_tab == 1) ?
				5 :
			(g_options_tab == 2) ?
				3 :
			(g_options_tab == 3) ?
				5 :
			(g_options_tab == 4) ?
				6 :
			(g_options_tab == 5) ?
				THEME_COUNT - 1 :
			(g_options_tab == 6) ?
				total_midi_items :
				g_midi_nout_bindings; /* MIDI OUT: output bindings */
		switch (c) {
		case 27: /* ESC: cancel learn, close probe, or close overlay */
			if (g_midi_learn_active) {
				g_midi_learn_active = 0;
				break;
			}
			if (g_midi_learn_jog_pair) {
				g_midi_learn_jog_pair = 0;
				g_midi_learn_jog_step = 0;
				g_midi_learn_jog_spin_status = 0;
				g_midi_learn_jog_spin_d1 = 0;
				break;
			}
			if (g_motor_probe_open) {
				g_motor_probe_open = 0;
				break;
			}
			if (g_options_tab == 5)
				apply_theme(g_opts.theme_idx);
			g_options_open = 0;
			break;
		case KEY_LEFT:
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open) {
				if (g_motor_probe_ch > 1)
					g_motor_probe_ch--;
			} else {
				g_options_tab = (g_options_tab + 8) % 9;
				g_options_sel = (g_options_tab == 5) ?
							g_opts.theme_idx :
							0;
			}
			break;
		case KEY_RIGHT:
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open) {
				if (g_motor_probe_ch < 16)
					g_motor_probe_ch++;
			} else {
				g_options_tab = (g_options_tab + 1) % 9;
				g_options_sel = (g_options_tab == 5) ?
							g_opts.theme_idx :
							0;
			}
			break;
		case KEY_DOWN:
			g_options_sel++;
			if (g_options_sel > max_sel)
				g_options_sel = (g_options_tab == 6) ? 1 : 0;
			if (g_options_tab == 6 &&
			    menu_to_mact(g_options_sel) == -1)
				g_options_sel++; /* skip header */
			if (g_options_tab == 5)
				apply_theme(g_options_sel);
			break;
		case KEY_UP:
			g_options_sel--;
			if (g_options_sel < ((g_options_tab == 6) ? 1 : 0))
				g_options_sel = max_sel;
			if (g_options_tab == 6 &&
			    menu_to_mact(g_options_sel) == -1)
				g_options_sel--; /* skip header backward */
			if (g_options_sel < 1 && g_options_tab == 6)
				g_options_sel = max_sel;
			if (g_options_tab == 5)
				apply_theme(g_options_sel);
			break;
		case 'k':
			if (g_options_tab == 6 && !g_midi_learn_jog_pair &&
			    !g_midi_learn_active) {
				if (g_midi_ndevices > 0)
					g_midi_dev_sel = (g_midi_dev_sel +
							  g_midi_ndevices - 1) %
							 g_midi_ndevices;
			} else if (g_options_tab == 7 && !g_motor_probe_open) {
				if (g_options_out_sel > 0)
					g_options_out_sel--;
			} else if (g_options_tab == 1) {
				if (g_pcm_ndevices > 0) {
					if (g_audio_hp_picker)
						g_pcm_hp_dev_sel =
							(g_pcm_hp_dev_sel +
							 g_pcm_ndevices - 1) %
							g_pcm_ndevices;
					else
						g_pcm_dev_sel =
							(g_pcm_dev_sel +
							 g_pcm_ndevices - 1) %
							g_pcm_ndevices;
				}
			} else if (g_options_tab != 6 && g_options_tab != 7) {
				int max_sel_k =
					(g_options_tab == 0) ? 0 :
					(g_options_tab == 1) ? 3 :
					(g_options_tab == 2) ? 3 :
					(g_options_tab == 3) ? 5 :
					(g_options_tab == 4) ? 4 :
							       THEME_COUNT - 1;
				g_options_sel--;
				if (g_options_sel < 0)
					g_options_sel = max_sel_k;
				if (g_options_tab == 5)
					apply_theme(g_options_sel);
			}
			break;
		case '-':
		case KEY_SLEFT:
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open) {
				/* -  adjusts d1 (CC/note number) */
				if (g_motor_probe_cc > 0)
					g_motor_probe_cc--;
			} else {
				options_adjust(-1);
			}
			break;
		case '=':
		case '+':
		case KEY_SRIGHT:
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open) {
				if (g_motor_probe_cc < 127)
					g_motor_probe_cc++;
			} else {
				options_adjust(+1);
			}
			break;
		case ',':
		case '<':
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open) {
				if (g_motor_probe_val > 0)
					g_motor_probe_val--;
			} else if (g_options_tab == 8) {
				/* FX tab: < adjusts param0 down */
				int row = g_options_sel,
				    mr = g_num_tracks * FX_SLOTS_PER_DECK;
				FXSlot *fx =
					(row < mr) ?
						fx_slot(row / FX_SLOTS_PER_DECK,
							row % FX_SLOTS_PER_DECK) :
						fx_master();
				fx->params[0] -= 0.05f;
				if (fx->params[0] < 0.0f)
					fx->params[0] = 0.0f;
			}
			break;
		case '.':
		case '>':
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open) {
				if (g_motor_probe_val < 127)
					g_motor_probe_val++;
			} else if (g_options_tab == 8) {
				/* FX tab: > adjusts param0 up */
				int row = g_options_sel,
				    mr = g_num_tracks * FX_SLOTS_PER_DECK;
				FXSlot *fx =
					(row < mr) ?
						fx_slot(row / FX_SLOTS_PER_DECK,
							row % FX_SLOTS_PER_DECK) :
						fx_master();
				fx->params[0] += 0.05f;
				if (fx->params[0] > 1.0f)
					fx->params[0] = 1.0f;
			}
			break;
		/* T = cycle message type (CC → NoteOn → NoteOff → CC) */
		case 't':
		case 'T':
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open)
				g_motor_probe_type =
					(g_motor_probe_type + 1) % 3;
			break;
		/* V = send current d2 value (for testing specific stop values) */
		case 'v':
		case 'V':
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open)
				motor_probe_send(g_motor_probe_val);
			break;
		/* X = sweep d2 values 0→127, then 127→0 to find stop signal */
		case 'x':
		case 'X':
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open) {
				snprintf(
					g_motor_probe_log,
					sizeof(g_motor_probe_log),
					"Sweeping 0→127 on ch%d CC%d... (listen for stop)",
					g_motor_probe_ch, g_motor_probe_cc);
				motor_probe_sweep(0, 127);
			}
			break;
		case '\n':
		case KEY_ENTER:
			/* During jog learn: ENTER commits with whatever has been captured so far */
			if (g_midi_learn_jog_pair) {
				int anything =
					(g_midi_learn_jog_spin_status !=
					 0) /* CC spin */
					|| (g_midi_learn_jog_step &
					    2) /* pitch bend */
					||
					(g_midi_learn_jog_step & 1); /* touch */
				if (anything) {
					g_midi_learn_jog_pair = 0;
					g_midi_learn_jog_step = 0;
					g_midi_learn_jog_spin_status = 0;
					g_midi_learn_jog_spin_d1 = 0;
					g_midi_learn_active = 0;
					midi_map_save();
				}
				break; /* if nothing captured yet, do nothing */
			}
			/* On theme tab: ENTER commits the currently-previewed theme */
			if (g_options_tab == 5) {
				g_opts.theme_idx = g_options_sel;
				apply_theme(g_opts.theme_idx);
				settings_save();
			}
			/* On AUDIO tab: ENTER switches to the focused device */
			if (g_options_tab == 1) {
				if (g_audio_hp_picker)
					hp_open_device(g_pcm_hp_dev_sel);
				else
					pcm_open_device(g_pcm_dev_sel);
				break;
			}
			/* On MIDI tab: ENTER switches to selected device (when not in learn mode) */
			if (g_options_tab == 6 && !g_midi_learn_active &&
			    !g_midi_learn_jog_pair) {
				int mact = menu_to_mact(g_options_sel);
				if (mact == -2) {
					/* Panic */
					for (int i = 0; i < MAX_TRACKS; i++) {
						pthread_mutex_lock(
							&g_tracks[i].lock);
						g_tracks[i].volume = 1.0f;
						g_tracks[i].eq_low = 0.0f;
						g_tracks[i].eq_mid = 0.0f;
						g_tracks[i].eq_high = 0.0f;
						pthread_mutex_unlock(
							&g_tracks[i].lock);
					}
					g_crossfader = 0.5f;
				} else if (g_options_sel <= g_midi_ndevices &&
					   g_options_sel > 0) {
					midi_open_device(g_midi_dev_sel);
				}
				break;
			}
			break;
		/* MIDI tab: Learn, Unbind, Save */
		case 'l':
		case 'L':
			if (g_options_tab == 6) {
				int mact = menu_to_mact(g_options_sel);
				if (mact >= 1) {
					g_midi_learn_active = 1;
					g_midi_learn_sel = mact;
					g_midi_learn_jog_pair = 0;
					g_midi_learn_jog_step = 0;
				}
			}
			break;
		/* J = jog learn / monitor toggle / device-list navigation depending on context */
		case 'j':
		case 'J':
			if (g_options_tab == 6) {
				if (g_midi_learn_jog_pair) {
					/* J again during jog learn: commit if spin captured */
					if (g_midi_learn_jog_spin_status != 0) {
						g_midi_learn_jog_pair = 0;
						g_midi_learn_jog_step = 0;
						g_midi_learn_jog_spin_status =
							0;
						g_midi_learn_jog_spin_d1 = 0;
						g_midi_learn_active = 0;
						midi_map_save();
					}
				} else if (g_midi_learn_active) {
					/* in single learn -- ignore J */
				} else {
					/* Toggle MIDI monitor panel */
					g_midi_mon_open ^= 1;
					if (g_midi_mon_open) {
						g_midi_mon_count =
							0; /* clear buffer on open for fresh capture */
						g_midi_mon_head = 0;
					}
				}
			} else if (g_options_tab == 7 && !g_motor_probe_open) {
				/* MIDI OUT tab: navigate output binding list down */
				if (g_options_out_sel <
				    g_midi_nout_bindings - 1)
					g_options_out_sel++;
			} else if (g_options_tab == 1) {
				/* AUDIO tab: navigate focused device list down */
				if (g_pcm_ndevices > 0) {
					if (g_audio_hp_picker)
						g_pcm_hp_dev_sel =
							(g_pcm_hp_dev_sel + 1) %
							g_pcm_ndevices;
					else
						g_pcm_dev_sel =
							(g_pcm_dev_sel + 1) %
							g_pcm_ndevices;
				}
			} else if (g_options_tab != 6 && g_options_tab != 7) {
				/* Other tabs: navigate rows down */
				int max_sel_j =
					(g_options_tab == 0) ?
						0 :
					(g_options_tab == 2) ?
						3 :
					(g_options_tab == 3) ?
						5 :
					(g_options_tab == 4) ?
						3 :
					(g_options_tab == 8) ?
						g_num_tracks *
							FX_SLOTS_PER_DECK :
						THEME_COUNT - 1;
				g_options_sel++;
				if (g_options_sel > max_sel_j)
					g_options_sel = 0;
				if (g_options_tab == 5)
					apply_theme(g_options_sel);
			}
			break;
		/* H = toggle headphone/master device picker focus (AUDIO tab) */
		case 'h':
		case 'H':
			if (g_options_tab == 1)
				g_audio_hp_picker ^= 1;
			break;
		/* R = rescan MIDI devices (MIDI IN/OUT tab) or PCM devices (AUDIO tab) */
		case 'r':
		case 'R':
			if (g_options_tab == 6 || g_options_tab == 7) {
				midi_enumerate_devices();
				if (g_midi_dev_sel >= g_midi_ndevices)
					g_midi_dev_sel =
						g_midi_ndevices > 0 ?
							g_midi_ndevices - 1 :
							0;
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "MIDI: found %d device%s",
					 g_midi_ndevices,
					 g_midi_ndevices == 1 ? "" : "s");
			} else if (g_options_tab == 1) {
				pcm_enumerate_devices();
				if (g_pcm_dev_sel >= g_pcm_ndevices)
					g_pcm_dev_sel =
						g_pcm_ndevices > 0 ?
							g_pcm_ndevices - 1 :
							0;
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Audio: found %d device%s",
					 g_pcm_ndevices,
					 g_pcm_ndevices == 1 ? "" : "s");
			}
			break;
		/* W = start guided jog wheel learn (J is now device navigation) */
		case 'w':
		case 'W':
			if (g_options_tab == 6 && !g_midi_learn_active) {
				int jdeck = 0;
				MidiAction sel_act = (MidiAction)g_options_sel;
				if (sel_act >= MACT_JOG_SPIN_A &&
				    sel_act <= MACT_JOG_SPIN_D)
					jdeck = sel_act - MACT_JOG_SPIN_A;
				else if (sel_act >= MACT_JOG_TOUCH_A &&
					 sel_act <= MACT_JOG_TOUCH_D)
					jdeck = sel_act - MACT_JOG_TOUCH_A;
				else if (sel_act >= MACT_JOG_PB_A &&
					 sel_act <= MACT_JOG_PB_D)
					jdeck = sel_act - MACT_JOG_PB_A;
				else
					jdeck = 0;
				int anything =
					(g_midi_learn_jog_spin_status != 0) ||
					(g_midi_learn_jog_step & 2) ||
					(g_midi_learn_jog_step & 1);
				if (g_midi_learn_jog_pair && anything) {
					/* W again = commit whatever was captured */
					g_midi_learn_jog_pair = 0;
					g_midi_learn_jog_step = 0;
					g_midi_learn_jog_spin_status = 0;
					g_midi_learn_jog_spin_d1 = 0;
					midi_map_save();
				} else {
					g_midi_learn_jog_pair = 1;
					g_midi_learn_jog_step = 0;
					g_midi_learn_jog_deck = jdeck;
					g_midi_learn_jog_spin_status = 0;
					g_midi_learn_jog_spin_d1 = 0;
					g_midi_learn_active = 0;
				}
			}
			break;
		/* U = unbind selected action */
		case 'u':
		case 'U':
			if (g_options_tab == 6) {
				int mact = menu_to_mact(g_options_sel);
				if (mact >= 1) {
					midi_bind(0, 0, (MidiAction)mact);
					g_midi_learn_active = 0;
				}
			}
			break;
		/* M = toggle motor probe panel (MIDI tab only) */
		case 'm':
		case 'M':
			if (g_options_tab == 6 || g_options_tab == 7) {
				g_motor_probe_open ^= 1;
				if (g_motor_probe_open) {
					g_midi_learn_active = 0;
					g_midi_learn_jog_pair = 0;
					/* Switch to MIDI OUT tab where probe lives */
					if (g_options_tab == 6)
						g_options_tab = 7;
				}
			}
			break;

		/* Motor probe: SPACE = start motor (127), BACKSPACE = stop (0) */
		case ' ':
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open)
				motor_probe_send(127);
			break;
		case KEY_BACKSPACE:
		case 127:
		case '\b':
			if ((g_options_tab == 6 || g_options_tab == 7) &&
			    g_motor_probe_open)
				motor_probe_send(0);
			break;
		case 's':
		case 'S':
			if (g_options_tab == 6 || g_options_tab == 7)
				midi_map_save();
			settings_save();
			break;
		case 'Q':
			quit_confirm();
			break;
		}
		return; /* swallow all other keys when options is open */
	}

	/* ── Sampler pane: 1-8 trigger; !@#$%^&* load from browser ── */
	if (g_panel == 4 && !g_options_open && !g_bpm_entry) {
		if (c >= '1' && c <= '8') {
			int si = c - '1';
			SamplerSlot *s = &g_samplers[si];
			pthread_mutex_lock(&s->lock);
			if (s->data) {
				s->pos = 0;
				s->playing = 1;
			}
			pthread_mutex_unlock(&s->lock);
			return;
		}
		static const char shift_map[] = "!@#$%^&*";
		for (int i = 0; i < 8; i++) {
			if (c == shift_map[i]) {
				char full[1024];
				fb_selected_path(full, sizeof(full));
				if (full[0]) {
					load_sampler(&g_samplers[i], full);
					snprintf(g_fb_status,
						 sizeof(g_fb_status),
						 "Sampler %d: %.200s", i + 1,
						 strrchr(full, '/') ?
							 strrchr(full, '/') +
								 1 :
							 full);
				}
				return;
			}
		}
		/* j/k adjust volume of slot under cursor (first slot for simplicity) */
		if (c == 'j' || c == KEY_DOWN) {
			for (int i = 0; i < MAX_SAMPLER_SLOTS; i++) {
				g_samplers[i].volume -= 0.05f;
				if (g_samplers[i].volume < 0.0f)
					g_samplers[i].volume = 0.0f;
			}
			return;
		}
		if (c == 'k' || c == KEY_UP) {
			for (int i = 0; i < MAX_SAMPLER_SLOTS; i++) {
				g_samplers[i].volume += 0.05f;
				if (g_samplers[i].volume > 1.5f)
					g_samplers[i].volume = 1.5f;
			}
			return;
		}
	}

	/* ── ESC opens options ── */
	if (c == 27) {
		g_options_open = 1;
		g_options_tab = 0;
		g_options_sel = 0;
		return;
	}

	switch (c) {
	/* Deck selection */
	case KEY_DECK_A:
		g_active_track = 0;
		break;
	case KEY_DECK_B:
		g_active_track = 1;
		break;
	case KEY_DECK_C:
		if (g_num_tracks >= 3)
			g_active_track = 2;
		break;
	case KEY_DECK_D:
		if (g_num_tracks >= 4)
			g_active_track = 3;
		break;

	/* Headphone Cue (PFL) toggles */
	case KEY_HEADPHONE_CUE_A:
		g_tracks[0].cue_active ^= 1;
		break;
	case KEY_HEADPHONE_CUE_B:
		g_tracks[1].cue_active ^= 1;
		break;
	case KEY_HEADPHONE_CUE_C:
		if (g_num_tracks >= 3)
			g_tracks[2].cue_active ^= 1;
		break;
	case KEY_HEADPHONE_CUE_D:
		if (g_num_tracks >= 4)
			g_tracks[3].cue_active ^= 1;
		break;

	/* Play / Pause */
	case ' ':
		if (t->loaded) {
			int was_playing = t->playing;
			t->pending_play = 0;
			if (!was_playing) {
				int can_quantize =
					g_opts.sync_quantize && t->synced &&
					g_sync_leader >= 0 &&
					g_sync_leader != g_active_track &&
					g_tracks[g_sync_leader].playing &&
					g_tracks[g_sync_leader].bpm > 1.0f;
				if (can_quantize) {
					t->pending_play = 1;
				} else {
					t->playing = 1;
					if (!g_slip_motor_off[g_active_track])
						motor_set(g_active_track, 1);
				}
			} else {
				t->playing = 0;
				motor_set(g_active_track, 0);
			}
			if (g_gang_mode)
				for (int i = 0; i < g_num_tracks; i++)
					if ((g_gang_mask & (1 << i)) &&
					    i != g_active_track &&
					    g_tracks[i].loaded) {
						g_tracks[i].pending_play = 0;
						g_tracks[i].playing =
							t->playing;
						if (t->playing &&
						    !g_slip_motor_off[i])
							motor_set(i, 1);
						else if (!t->playing)
							motor_set(i, 0);
					}
		}
		break;

	/* Stop (skipped when sampler pane is active -- 's' has no meaning there) */
	case 's':
		if (g_panel == 4)
			break;
		t->playing = 0;
		t->pending_play = 0;
		motor_set(g_active_track, 0);
		if (g_gang_mode)
			for (int i = 0; i < g_num_tracks; i++)
				if ((g_gang_mask & (1 << i)) &&
				    i != g_active_track) {
					g_tracks[i].playing = 0;
					g_tracks[i].pending_play = 0;
					motor_set(i, 0);
				}
		break;

	/* Restart / Cue */
	case 'r':
		pthread_mutex_lock(&t->lock);
		t->pos = 0;
		pthread_mutex_unlock(&t->lock);
		if (g_gang_mode)
			for (int i = 0; i < g_num_tracks; i++)
				if ((g_gang_mask & (1 << i)) &&
				    i != g_active_track) {
					pthread_mutex_lock(&g_tracks[i].lock);
					g_tracks[i].pos = 0;
					pthread_mutex_unlock(&g_tracks[i].lock);
				}
		break;

	/* Loop toggle */
	case 'l':
		t->looping = !t->looping;
		if (g_gang_mode)
			for (int i = 0; i < g_num_tracks; i++)
				if ((g_gang_mask & (1 << i)) &&
				    i != g_active_track)
					g_tracks[i].looping = t->looping;
		break;

	/* ── BPM detect ── */
	/* Ctrl+T = TAP BPM */
	case 20:
		tap_bpm(g_active_track);
		break;
	/* s = Stop, S = Snap Grid (skipped when sampler pane is active) */
	case 'S':
		if (g_panel == 4)
			break;
		if (g_options_tab == 6 || g_options_tab == 7) {
			midi_map_save();
			settings_save();
		} else {
			snap_grid(g_active_track);
		}
		break;
	/* Ctrl+S = toggle Sampler pane */
	case 19:
		g_panel = (g_panel == 4) ? 0 : 4;
		if (g_view != 1)
			g_view = 1;
		g_tag_info.visible = 0;
		break;
	case 'b':
		if (t->loaded) {
			/* Re-analysis via background worker -- just re-enqueue the same file.
             * The worker will call detect_bpm_and_offset and overwrite the cache. */
			pthread_mutex_lock(&g_load_mutex);
			g_load_job.deck = g_active_track;
			snprintf(g_load_job.path, sizeof(g_load_job.path), "%s",
				 t->filename);
			g_load_job.path[sizeof(g_load_job.path) - 1] = '\0';
			g_load_job.valid = 1;
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Re-analyzing Deck %c...",
				 DECK_NUM(g_active_track));
			pthread_cond_signal(&g_load_cond);
			pthread_mutex_unlock(&g_load_mutex);
		}
		break;

	/* Manual BPM entry: B = type a BPM value for the active deck.
     * Enter to confirm, Esc to cancel, Backspace to delete.
     * Useful when auto-detection is wrong and you know the BPM already. */
	case 'B':
		if (t->loaded) {
			g_bpm_entry = 1;
			g_bpm_deck = g_active_track;
			g_bpm_buf[0] = '\0';
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "BPM Deck %c: _  (Enter=confirm  Esc=cancel)",
				 DECK_NUM(g_active_track));
		}
		break;

	/* BPM display cycle: H steps through normal → ×2 → ½ → normal ...
     * Affects only the displayed BPM value and beat ruler spacing.
     * Playback speed, pitch, and the underlying bpm/bpm_offset are untouched. */
	case 'H':
		if (t->loaded && t->bpm > 1.0f) {
			/* Cycle: 0 (normal) → 1 (×2) → -1 (½) → 0 … */
			if (t->bpm_display_double == 0)
				t->bpm_display_double = 1;
			else if (t->bpm_display_double == 1)
				t->bpm_display_double = -1;
			else
				t->bpm_display_double = 0;
		}
		break;

	/* Key lock (master tempo): K toggles pitch-preserving time-stretch.
     * When ON, pitch stays constant regardless of playback speed.
     * Uses phase vocoder -- costs ~4-6% extra CPU per deck on G4 (scalar). */
	case 'K':
		if (t->loaded) {
			t->key_lock = !t->key_lock;
			/* Reset stretcher state on toggle so there's no glitch */
			int di = (int)(t - g_tracks);
			if (di >= 0 && di < MAX_TRACKS)
				wsola_reset(&g_wsola[di], t->pos);
		}
		break;

	/* ── Sync lock ── */
	/* M = make active deck the sync master */
	case 'M':
		if (t->loaded && t->bpm > 0) {
			g_sync_leader = g_active_track;
			t->synced = 0; /* master is never a slave */
			/* Re-sync all locked slaves */
			for (int i = 0; i < g_num_tracks; i++)
				if (i != g_sync_leader && g_tracks[i].synced)
					sync_to_leader(i);
		}
		break;
	/* y = toggle sync lock on active deck (slave to master) */
	case 'y':
		if (t->loaded && t->bpm > 0 &&
		    g_active_track != g_sync_leader) {
			t->synced = !t->synced;
			if (t->synced)
				sync_to_leader(g_active_track);
		}
		break;

	/* ── Beat nudge ── accumulates on repeat presses, decays to 0 ── */
	/* nudge forward: ] */
	case ']':
		t->nudge += CFG_NUDGE_AMOUNT;
		if (t->nudge > CFG_NUDGE_AMOUNT * 8)
			t->nudge = CFG_NUDGE_AMOUNT * 8;
		/* Gang: apply to all gang members too */
		if (g_gang_mode)
			for (int i = 0; i < g_num_tracks; i++)
				if ((g_gang_mask & (1 << i)) &&
				    i != g_active_track) {
					g_tracks[i].nudge += CFG_NUDGE_AMOUNT;
					if (g_tracks[i].nudge >
					    CFG_NUDGE_AMOUNT * 8)
						g_tracks[i].nudge =
							CFG_NUDGE_AMOUNT * 8;
				}
		break;
	/* nudge back: [ */
	case '[':
		t->nudge -= CFG_NUDGE_AMOUNT;
		if (t->nudge < -CFG_NUDGE_AMOUNT * 8)
			t->nudge = -CFG_NUDGE_AMOUNT * 8;
		if (g_gang_mode)
			for (int i = 0; i < g_num_tracks; i++)
				if ((g_gang_mask & (1 << i)) &&
				    i != g_active_track) {
					g_tracks[i].nudge -= CFG_NUDGE_AMOUNT;
					if (g_tracks[i].nudge <
					    -CFG_NUDGE_AMOUNT * 8)
						g_tracks[i].nudge =
							-CFG_NUDGE_AMOUNT * 8;
				}
		break;

	/* ── Coarse seek: arrow keys ─────────────────────────────────
     *  LEFT  / RIGHT       ±1 beat
     *  SHIFT+LEFT  (KEY_SLEFT)  / SHIFT+RIGHT (KEY_SRIGHT)  ±8 beats (2 bars)
     *
     *  Use these to align beats by ear when auto-sync isn't
     *  quite right.  The playhead jumps instantly; no audio
     *  artifact beyond a brief discontinuity.
     * ────────────────────────────────────────────────────────── */
	case KEY_LEFT:
	case KEY_RIGHT:
	case KEY_SLEFT:
	case KEY_SRIGHT: {
		if (g_view == 2)
			break; /* help view: don't seek */
		if (!t->loaded)
			break;
		float beat_frames =
			(t->bpm > 0.0f) ? (float)g_actual_sample_rate * 60.0f /
						  (t->bpm * t->pitch) :
					  (float)g_actual_sample_rate *
						  0.5f; /* fallback: 0.5 s */

		int beats = (c == KEY_SLEFT || c == KEY_SRIGHT) ? 8 : 1;
		int sign = (c == KEY_RIGHT || c == KEY_SRIGHT) ? +1 : -1;

		int64_t delta = (int64_t)(sign * beats * beat_frames);
		int64_t newpos = (int64_t)t->pos + delta;
		if (newpos < 0)
			newpos = 0;
		if (newpos >= (int64_t)t->num_frames)
			newpos = (int64_t)(t->num_frames - 1);
		t->pos = (uint32_t)newpos;
		if (t->key_lock)
			wsola_reset(&g_wsola[g_active_track], t->pos);
		break;
	}

	/* ── Beat grid offset ──────────────────────────────────────────────────
     * { / }  shift the beat grid by ±1 beat (one full beat-length in frames).
     * This moves every beat marker and bar number left or right by exactly
     * one beat, letting you re-align the grid when the auto-detected phase
     * is off by one or more beats.
     *
     * Previous code shifted by ±1 *frame* per keypress -- at 128 BPM a beat
     * is ~20 000 frames, making the control appear completely non-functional.
     * ──────────────────────────────────────────────────────────────────── */
	case '{':
		if (t->loaded && t->bpm > 1.0f) {
			float beat_f =
				(float)g_actual_sample_rate * 60.0f / t->bpm;
			t->bpm_offset -= beat_f;
			if (t->bpm_offset < 0.0f)
				t->bpm_offset = 0.0f;
		}
		break;
	case '}':
		if (t->loaded && t->bpm > 1.0f) {
			float beat_f =
				(float)g_actual_sample_rate * 60.0f / t->bpm;
			t->bpm_offset += beat_f;
		}
		break;

	/* ── Beat grid fine adjustment: ( / ) = ±¼ beat ───────────────────────
     * One quarter-beat = one 16th-note position.  Use these when the grid
     * is close but drifting -- a full-beat jump overshoots but a nudge with
     * ] / [ only shifts playback pitch, not the grid anchor.
     * ──────────────────────────────────────────────────────────────────── */
	case '(':
		if (t->loaded && t->bpm > 1.0f) {
			float quarter_f = (float)g_actual_sample_rate * 60.0f /
					  t->bpm / 4.0f;
			t->bpm_offset -= quarter_f;
			if (t->bpm_offset < 0.0f)
				t->bpm_offset = 0.0f;
		}
		break;
	case ')':
		if (t->loaded && t->bpm > 1.0f) {
			float quarter_f = (float)g_actual_sample_rate * 60.0f /
					  t->bpm / 4.0f;
			t->bpm_offset += quarter_f;
		}
		break;

	/* ── Beat grid reset: Z ────────────────────────────────────────────────
     * Resets bpm_offset to 0 (the very first sample), which is the state
     * after a fresh autocorrelation analysis before the onset scan refines
     * it.  Useful when grid adjustments have drifted far off or when loading
     * a track whose first beat truly starts at frame 0 (e.g. a loop file).
     * The BPM itself is unchanged; only the phase anchor is cleared.
     * ──────────────────────────────────────────────────────────────────── */
	case 'Z':
		if (t->loaded) {
			t->bpm_offset = 0.0f;
			sidecar_save(t);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Beat grid reset \u2192 Deck %c",
				 DECK_NUM(g_active_track));
		}
		break;

	/* ── Gang mode ── */
	/* G = toggle gang mode on/off */
	case 'G':
		g_gang_mode = !g_gang_mode;
		if (g_gang_mode && g_gang_mask == 0)
			g_gang_mask = (1 << g_num_tracks) -
				      1; /* default: all decks */
		break;

	/* C = toggle crate jump mode */
	case 'C':
		g_crate_jump_active = !g_crate_jump_active;
		if (g_crate_jump_active) {
			g_crate_input[0] = '\0';
			g_crate_orig_input[0] = '\0';
			g_crate_cycle_idx = -1;
			update_crate_matches("", "Jump to:");
		} else {
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Crate jump cancelled");
		}
		break;
	/* Ctrl+E (5) = eject USB (only when a USB group header is selected) */
	case 5:
		if (g_view == 1 && g_panel == 3 && g_crate_view_level == 0) {
			int vt, vi;
			crate_vrow_resolve(g_crate_vrow, &vt, &vi);
			if (vt == 1 && g_crate_groups[vi].is_usb) {
				g_usb_eject_active = 1;
				strncpy(g_usb_eject_mount,
					g_crate_groups[vi].mount_point,
					sizeof(g_usb_eject_mount) - 1);
				strncpy(g_usb_eject_label,
					g_crate_groups[vi].label,
					sizeof(g_usb_eject_label) - 1);
			}
		}
		break;

	/* Ctrl+U (21) = export crate to USB (when on a local crate) or rescan (anywhere) */
	case 21: {
		int vt = -1, vi = -1;
		int on_local_crate = 0;
		if (g_view == 1 && g_panel == 3 && g_crate_view_level == 0) {
			crate_vrow_resolve(g_crate_vrow, &vt, &vi);
			on_local_crate =
				(vt == 0 && vi >= 0 && !g_crates[vi].is_usb);
		}
		if (on_local_crate) {
			/* Cursor is on a local crate name \u2014 export it */
			g_usb_rescan_seq = 0;
			g_usb_export_crate_idx = vi;
			g_usb_devices_count =
				usb_scan(g_usb_devices, USB_MAX_DEVICES);
			if (g_usb_devices_count == 0) {
				snprintf(
					g_fb_status, sizeof(g_fb_status),
					"No djcmd USB found \u2014 plug in a USB and try again");
			} else if (g_usb_devices_count == 1) {
				const char *mount =
					g_usb_devices[0].mount_point;
				const char *cname = g_crates[vi].name;
				char usb_crate_path[2048];
				snprintf(usb_crate_path, sizeof(usb_crate_path),
					 "%s/%s/%s.crate", mount,
					 DJCMD_USB_CRATES, cname);
				char remote_origin[64] = "";
				usb_crate_read_origin(usb_crate_path,
						      remote_origin,
						      sizeof(remote_origin));
				struct stat _usb_st;
				int crate_exists =
					(stat(usb_crate_path, &_usb_st) == 0);
				if ((remote_origin[0] &&
				     strcmp(remote_origin, g_machine_id) !=
					     0) ||
				    (crate_exists && !remote_origin[0])) {
					/* Conflict: different machine, or file exists without origin header */
					g_usb_conflict_active = 1;
					strncpy(g_usb_conflict_mount, mount,
						sizeof(g_usb_conflict_mount) -
							1);
					g_usb_conflict_mount
						[sizeof(g_usb_conflict_mount) -
						 1] = '\0';
					strncpy(g_usb_conflict_crate_name,
						cname,
						sizeof(g_usb_conflict_crate_name) -
							1);
					g_usb_conflict_rename[0] = '\0';
				} else {
					char status[256];
					usb_export_crate(g_crates[vi].filename,
							 cname, g_machine_id,
							 mount, cname, status,
							 sizeof(status));
					snprintf(g_fb_status,
						 sizeof(g_fb_status), "%s",
						 status);
					crates_load();
				}
			} else {
				/* Multiple USBs: let user pick */
				g_usb_picker_active = 1;
				g_usb_picker_sel = 0;
			}
		} else {
			/* Not on a local crate \u2014 count toward manual USB rescan (works from any panel) */
			g_usb_rescan_seq++;
			if (g_usb_rescan_seq >= 3) {
				g_usb_rescan_seq = 0;
				crates_load();
				snprintf(
					g_fb_status, sizeof(g_fb_status),
					"USB scan complete \u2014 %d device(s) found",
					g_ncrate_groups > 0 ?
						g_ncrate_groups -
							(g_crate_groups[0]
									 .is_usb ?
								 0 :
								 1) :
						0);
			} else {
				snprintf(
					g_fb_status, sizeof(g_fb_status),
					"USB rescan: press Ctrl+U %d more time(s)",
					3 - g_usb_rescan_seq);
			}
		}
	} break;

	/* F1-F4 = toggle individual decks in/out of gang */
	case KEY_F(1):
		g_gang_mask ^= (1 << 0);
		break;
	case KEY_F(2):
		g_gang_mask ^= (1 << 1);
		break;
	case KEY_F(3):
		g_gang_mask ^= (1 << 2);
		break;
	case KEY_F(4):
		g_gang_mask ^= (1 << 3);
		break;

	/* ── Cue points ── */
	/* Shift+F1-F4 = set cue,  F5-F8 = jump to cue */
	case KEY_F(5):
	case KEY_F(6):
	case KEY_F(7):
	case KEY_F(8): {
		int ci = c - KEY_F(5);
		if (ci >= 0 && ci < MAX_CUES) {
			t->cue[ci] = t->pos;
			t->cue_set[ci] = 1;
			sidecar_save(t);
			/* Light cue LED if bound */
			{
				char side = (g_active_track == g_side_deck[0]) ?
						    'a' :
						    'b';
				char ln[32];
				snprintf(ln, sizeof(ln), "led_cue_%d_%c",
					 ci + 1, side);
				led_on(ln);
			}
			pad_leds_refresh(g_active_track);
		}
		break;
	}
	case KEY_F(9):
	case KEY_F(10):
	case KEY_F(11):
	case KEY_F(12): {
		int ci = c - KEY_F(9);
		if (ci >= 0 && ci < MAX_CUES && t->cue_set[ci]) {
			pthread_mutex_lock(&t->lock);
			t->pos = t->cue[ci];
			pthread_mutex_unlock(&t->lock);
		}
		break;
	}

	/* ── Auto-gain toggle ── recalc gain or reset to 1.0 ── */
	case 'A':
		if (t->loaded) {
			/* Toggle: if gain != 1.0 reset, else recalculate */
			if (fabsf(t->gain - 1.0f) < 0.01f)
				t->gain =
					calc_auto_gain(t->data, t->num_frames);
			else
				t->gain = 1.0f;
		}
		break;

	/* Pitch control */
	case 'e':
		t->pitch += 0.005f;
		if (t->pitch > 2.0f)
			t->pitch = 2.0f;
		break;
	case 'd':
		t->pitch -= 0.005f;
		if (t->pitch < 0.5f)
			t->pitch = 0.5f;
		break;
	case 'E':
		t->pitch += 0.05f;
		if (t->pitch > 2.0f)
			t->pitch = 2.0f;
		break;
	case 'D':
		t->pitch -= 0.05f;
		if (t->pitch < 0.5f)
			t->pitch = 0.5f;
		break;
	case '0':
		t->pitch = 1.0f;
		break;

	/* V = cycle pitch range for active deck: \u00b18% \u2192 \u00b125% \u2192 \u00b150% \u2192 \u00b18%
     * Affects the MIDI pitch fader span only; keyboard pitch steps are unchanged. */
	case 'V':
		g_pitch_range[g_active_track] =
			(g_pitch_range[g_active_track] + 1) % 3;
		snprintf(g_fb_status, sizeof(g_fb_status),
			 "Deck %c pitch range: %s", DECK_NUM(g_active_track),
			 g_pitch_range_names[g_pitch_range[g_active_track]]);
		break;

	/* Volume */
	case '+':
	case '=':
		t->volume += 0.05f;
		if (t->volume > 1.0f)
			t->volume = 1.0f;
		break;
	case '-':
		t->volume -= 0.05f;
		if (t->volume < 0.0f)
			t->volume = 0.0f;
		break;

	/* EQ */
	case 'q':
		t->eq_low += 0.1f;
		if (t->eq_low > 1.0f)
			t->eq_low = 1.0f;
		break;
	case 'a':
		t->eq_low -= 0.1f;
		if (t->eq_low < -1.0f)
			t->eq_low = -1.0f;
		break;
	case 'w':
		t->eq_mid += 0.1f;
		if (t->eq_mid > 1.0f)
			t->eq_mid = 1.0f;
		break;
	case 'x':
		t->eq_mid -= 0.1f;
		if (t->eq_mid < -1.0f)
			t->eq_mid = -1.0f;
		break;
	case 't':
		t->eq_high += 0.1f;
		if (t->eq_high > 1.0f)
			t->eq_high = 1.0f;
		break;
	case 'g':
		t->eq_high -= 0.1f;
		if (t->eq_high < -1.0f)
			t->eq_high = -1.0f;
		break;

	/* Crossfader */
	case '<':
	case ',':
		g_crossfader -= 0.05f;
		if (g_crossfader < 0.0f)
			g_crossfader = 0.0f;
		g_autoplay_xf_target = -1.0f;
		break;
	case '>':
	case '.':
		g_crossfader += 0.05f;
		if (g_crossfader > 1.0f)
			g_crossfader = 1.0f;
		g_autoplay_xf_target = -1.0f;
		break;

	/* Master volume */
	case 'm':
		g_master_vol += 5;
		if (g_master_vol > 150)
			g_master_vol = 150;
		g_opts.default_master_vol = g_master_vol;
		break;
	case 'n':
		g_master_vol -= 5;
		if (g_master_vol < 0)
			g_master_vol = 0;
		g_opts.default_master_vol = g_master_vol;
		break;

	case 'N': {
		/* Double-tap N within 500 ms opens the batch BPM range prompt */
		struct timespec ts_n;
		clock_gettime(CLOCK_MONOTONIC, &ts_n);
		int64_t now_ms = (int64_t)ts_n.tv_sec * 1000 +
				 (int64_t)(ts_n.tv_nsec / 1000000);
		if (now_ms - g_n_last_tap_ms < 500) {
			g_batch_prompt_active = 1;
			g_batch_prompt_field = 0;
			g_n_last_tap_ms =
				0; /* reset so triple-tap doesn't re-fire */
		} else {
			g_n_last_tap_ms = now_ms;
		}
		break;
	}

	case 'X':
		if (g_view == 1 && g_panel == 3 && g_crate_view_level == 1 &&
		    g_crate_tracks_count > 0) {
			/* Identify which crate we are in */
			int crate_idx = -1;
			for (int i = 0; i < g_ncrate; i++) {
				if (strcmp(g_crates[i].name,
					   g_active_crate_name) == 0) {
					crate_idx = i;
					break;
				}
			}
			if (crate_idx >= 0) {
				crate_remove_at(crate_idx, g_crate_tracks_sel);
				if (g_crate_tracks_sel >=
				    g_crate_tracks_count) {
					g_crate_tracks_sel =
						g_crate_tracks_count > 0 ?
							g_crate_tracks_count -
								1 :
							0;
				}
			}
		}
		break;
	/* View switch */
	case '\t':
		if (g_num_tracks <= 2) {
			/* 2-deck mode: TAB cycles the bottom panel (browser/playlist/library/crates)
             * The bottom pane is always visible \u2014 never goes blank. */
			g_panel = (g_panel + 1) % 5;
			g_view = 1; /* always stay in split view */
			/* Rescan for USB devices when switching to the crates panel */
			if (g_panel == 3)
				crates_load();
		} else {
			/* 4-deck mode: TAB toggles split view on/off (screen space tight) */
			g_view = (g_view == 1) ? 0 : 1;
		}
		g_tag_info.visible = 0;
		break;
	case '?':
		g_tag_info.visible = 0;
		g_view = (g_view == 2) ? 1 : 2;
		break;
	case 'P':
		g_tag_info.visible = 0;
		g_panel = (g_panel + 1) % 5;
		if (g_view != 1)
			g_view = 1; /* ensure split view is showing */
		break;

	/* ── File browser / playlist navigation ── */
	case 'j':
	case KEY_DOWN:
		if (g_view == 1 && g_panel == 0 && g_fb_count > 0)
			g_fb_sel = (g_fb_sel + 1) % g_fb_count;
		else if (g_view == 1 && g_panel == 1 && g_pl_count > 0)
			g_pl_sel = (g_pl_sel + 1) % g_pl_count;
		else if (g_view == 1 && g_panel == 2 && g_lib_count > 0)
			g_lib_sel = (g_lib_sel + 1) % g_lib_count;
		else if (g_view == 1 && g_panel == 3) {
			if (g_crate_view_level == 0) {
				int total = crate_total_vrows();
				if (total > 0)
					g_crate_vrow =
						(g_crate_vrow + 1) % total;
			} else if (g_crate_view_level == 1 &&
				   g_crate_tracks_count > 0)
				g_crate_tracks_sel = (g_crate_tracks_sel + 1) %
						     g_crate_tracks_count;
		}
		break;
	case 'k':
	case KEY_UP:
		if (g_view == 1 && g_panel == 0 && g_fb_count > 0)
			g_fb_sel = (g_fb_sel + g_fb_count - 1) % g_fb_count;
		else if (g_view == 1 && g_panel == 1 && g_pl_count > 0)
			g_pl_sel = (g_pl_sel + g_pl_count - 1) % g_pl_count;
		else if (g_view == 1 && g_panel == 2 && g_lib_count > 0)
			g_lib_sel = (g_lib_sel + g_lib_count - 1) % g_lib_count;
		else if (g_view == 1 && g_panel == 3) {
			if (g_crate_view_level == 0) {
				int total = crate_total_vrows();
				if (total > 0)
					g_crate_vrow =
						(g_crate_vrow + total - 1) %
						total;
			} else if (g_crate_view_level == 1 &&
				   g_crate_tracks_count > 0)
				g_crate_tracks_sel =
					(g_crate_tracks_sel +
					 g_crate_tracks_count - 1) %
					g_crate_tracks_count;
		}
		break;
	case KEY_NPAGE:
		if (g_view == 0 && t->loaded) {
			/* Decks view: skip +16 beats */
			float beat_frames =
				(t->bpm > 0.0f) ?
					(float)g_actual_sample_rate * 60.0f /
						(t->bpm * t->pitch) :
					(float)g_actual_sample_rate * 0.5f;
			int64_t delta = (int64_t)(16 * beat_frames);
			int64_t newpos = (int64_t)t->pos + delta;
			if (newpos >= (int64_t)t->num_frames)
				newpos = (int64_t)t->num_frames - 1;
			t->pos = (uint32_t)newpos;
			if (t->key_lock)
				wsola_reset(&g_wsola[g_active_track], t->pos);
			break;
		}
		if (g_view == 1 && g_panel == 0) {
			g_fb_sel += (g_rows - 6);
			if (g_fb_sel >= g_fb_count)
				g_fb_sel = g_fb_count - 1;
		} else if (g_view == 1 && g_panel == 1) {
			g_pl_sel += (g_rows - 6);
			if (g_pl_sel >= g_pl_count)
				g_pl_sel = g_pl_count - 1;
		} else if (g_view == 1 && g_panel == 2) {
			g_lib_sel += (g_rows - 6);
			if (g_lib_sel >= g_lib_count)
				g_lib_sel =
					g_lib_count > 0 ? g_lib_count - 1 : 0;
		} else if (g_view == 1 && g_panel == 3) {
			if (g_crate_view_level == 0) {
				int total = crate_total_vrows();
				g_crate_vrow += (g_rows - 6);
				if (g_crate_vrow >= total)
					g_crate_vrow =
						total > 0 ? total - 1 : 0;
			} else {
				g_crate_tracks_sel += (g_rows - 6);
				if (g_crate_tracks_sel >= g_crate_tracks_count)
					g_crate_tracks_sel =
						g_crate_tracks_count > 0 ?
							g_crate_tracks_count -
								1 :
							0;
			}
		}
		break;
	case KEY_PPAGE:
		if (g_view == 0 && t->loaded) {
			/* Decks view: skip -16 beats */
			float beat_frames =
				(t->bpm > 0.0f) ?
					(float)g_actual_sample_rate * 60.0f /
						(t->bpm * t->pitch) :
					(float)g_actual_sample_rate * 0.5f;
			int64_t delta = (int64_t)(16 * beat_frames);
			int64_t newpos = (int64_t)t->pos - delta;
			if (newpos < 0)
				newpos = 0;
			t->pos = (uint32_t)newpos;
			if (t->key_lock)
				wsola_reset(&g_wsola[g_active_track], t->pos);
			break;
		}
		if (g_view == 1 && g_panel == 0) {
			g_fb_sel -= (g_rows - 6);
			if (g_fb_sel < 0)
				g_fb_sel = 0;
		} else if (g_view == 1 && g_panel == 1) {
			g_pl_sel -= (g_rows - 6);
			if (g_pl_sel < 0)
				g_pl_sel = 0;
		} else if (g_view == 1 && g_panel == 2) {
			g_lib_sel -= (g_rows - 6);
			if (g_lib_sel < 0)
				g_lib_sel = 0;
		} else if (g_view == 1 && g_panel == 3) {
			if (g_crate_view_level == 0) {
				g_crate_vrow -= (g_rows - 6);
				if (g_crate_vrow < 0)
					g_crate_vrow = 0;
			} else {
				g_crate_tracks_sel -= (g_rows - 6);
				if (g_crate_tracks_sel < 0)
					g_crate_tracks_sel = 0;
			}
		}
		break;

	/* Enter: open dir / load file (browser) or load from playlist */
	case '\n':
	case KEY_ENTER:
		if (g_tag_info.visible) {
			g_tag_info.visible = 0;
			break;
		}
		if (g_view == 1 && g_panel == 0 && g_fb_count > 0) {
			FBEntry *e = &g_fb_entries[g_fb_sel];
			if (e->is_dir) {
				fb_enter_dir(e->name);
			} else {
				char full[FB_PATH_MAX + 256];
				fb_selected_path(full, sizeof(full));
				enqueue_load(g_active_track, full);
				if (g_num_tracks > 2)
					g_view = 0;
			}
		} else if (g_view == 1 && g_panel == 1 && g_pl_count > 0) {
			enqueue_load(g_active_track, g_pl[g_pl_sel].path);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Loaded \u2192 Deck %c", 'A' + g_active_track);
		} else if (g_view == 1 && g_panel == 2 && g_lib &&
			   g_lib_count > 0) {
			enqueue_load(g_active_track, g_lib[g_lib_sel].path);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Loaded \u2192 Deck %c", 'A' + g_active_track);
		} else if (g_view == 1 && g_panel == 3) {
			if (g_crate_view_level == 0) {
				/* Only open actual crate entries \u2014 ignore group header rows */
				int vt, vi;
				crate_vrow_resolve(g_crate_vrow, &vt, &vi);
				if (vt == 0) {
					Crate *c = &g_crates[vi];
					if (c->filename[0]) {
						crate_view_open(vi);
					} else if (c->path[0]) {
						crate_jump(c->alias);
					}
				}
			} else if (g_crate_view_level == 1 &&
				   g_crate_tracks_count > 0) {
				enqueue_load(g_active_track,
					     g_crate_tracks[g_crate_tracks_sel]
						     .path);
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Loaded \u2192 Deck %c",
					 'A' + g_active_track);
			}
		}
		break;

	/* Backspace = go up one directory (browser only) */
	case KEY_BACKSPACE:
	case 127:
	case '\b':
		if (g_tag_info.visible) {
			g_tag_info.visible = 0;
			break;
		}
		if (g_view == 1 && g_panel == 0)
			fb_enter_dir("..");
		else if (g_view == 1 && g_panel == 3 && g_crate_view_level == 1)
			g_crate_view_level = 0;
		break;

	/* !@#$ = load to specific deck */
	case '!':
	case '@':
	case '#':
	case '$': {
		if (g_tag_info.visible) {
			g_tag_info.visible = 0;
			break;
		}
		int deck = (c == '!') ? 0 : (c == '@') ? 1 : (c == '#') ? 2 : 3;
		if (deck >= g_num_tracks)
			break;
		if (g_view == 1 && g_panel == 0 && g_fb_count > 0 &&
		    !g_fb_entries[g_fb_sel].is_dir) {
			char full[FB_PATH_MAX + 256];
			fb_selected_path(full, sizeof(full));
			enqueue_load(deck, full);
		} else if (g_view == 1 && g_panel == 1 && g_pl_count > 0) {
			enqueue_load(deck, g_pl[g_pl_sel].path);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Loaded \u2192 Deck %c", 'A' + deck);
		} else if (g_view == 1 && g_panel == 2 && g_lib &&
			   g_lib_count > 0) {
			enqueue_load(deck, g_lib[g_lib_sel].path);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Loaded \u2192 Deck %c", 'A' + deck);
		} else if (g_view == 1 && g_panel == 3 &&
			   g_crate_view_level == 1 &&
			   g_crate_tracks_count > 0) {
			enqueue_load(deck,
				     g_crate_tracks[g_crate_tracks_sel].path);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Loaded \u2192 Deck %c", 'A' + deck);
		}
		break;
	}

	/* p = add selected browser/library entry to playlist */
	case 'p':
		if (g_view == 1 && g_panel == 0 && g_fb_count > 0 &&
		    !g_fb_entries[g_fb_sel].is_dir) {
			char full[FB_PATH_MAX + 256];
			fb_selected_path(full, sizeof(full));
			FBEntry *e = &g_fb_entries[g_fb_sel];
			pl_add(full, e->name, e->bpm, e->tag_key);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "+Playlist [%d]", g_pl_count);
		} else if (g_view == 1 && g_panel == 2 && g_lib &&
			   g_lib_count > 0) {
			LIBEntry *e = &g_lib[g_lib_sel];
			pl_add(e->path, e->name, e->bpm, e->tag_key);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "+Playlist [%d]", g_pl_count);
		} else if (g_view == 1 && g_panel == 3 &&
			   g_crate_view_level == 1 &&
			   g_crate_tracks_count > 0) {
			CrateEntry *e = &g_crate_tracks[g_crate_tracks_sel];
			pl_add(e->path, e->name, e->bpm, e->tag_key);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "+Playlist [%d]", g_pl_count);
		}
		break;

	/* c = add selected directory as a crate or add track to crate */
	case 'c':
		if (g_view == 1 && g_panel == 0 && g_fb_count > 0) {
			FBEntry *e = &g_fb_entries[g_fb_sel];
			if (e->is_dir) {
				if (strcmp(e->name, "..") != 0) {
					fb_selected_path(
						g_crate_add_path,
						sizeof(g_crate_add_path));
					g_crate_add_active = 1;
					g_crate_add_input[0] = '\0';
					snprintf(g_fb_status,
						 sizeof(g_fb_status),
						 "New crate: _");
				}
			} else {
				/* File selected: Add to crate */
				fb_selected_path(g_pending_track_path,
						 sizeof(g_pending_track_path));
				g_track_add_crate_active = 1;
				g_track_add_crate_input[0] = '\0';
				g_crate_orig_input[0] = '\0';
				g_crate_cycle_idx = -1;
				update_crate_matches("", "Add to crate:");
			}
		} else if (g_view == 1 && g_panel == 2 && g_lib &&
			   g_lib_count > 0) {
			/* Library track: Add to crate */
			LIBEntry *e = &g_lib[g_lib_sel];
			strncpy(g_pending_track_path, e->path,
				sizeof(g_pending_track_path) - 1);
			g_track_add_crate_active = 1;
			g_track_add_crate_input[0] = '\0';
			g_crate_orig_input[0] = '\0';
			g_crate_cycle_idx = -1;
			update_crate_matches("", "Add to crate:");
		}
		break;

	/* O = cycle sort order (browser or library) */
	case 'O':
		if (g_view == 1 && g_panel == 0) {
			g_fb_sort = (g_fb_sort + 1) % 3;
			fb_apply_sort();
			g_fb_sel = 0;
			g_fb_scroll = 0;
		} else if (g_view == 1 && g_panel == 2) {
			g_lib_sort = (g_lib_sort + 1) % 3;
			lib_apply_sort();
			g_lib_sel = 0;
			g_lib_scroll = 0;
		}
		break;

	/* L = scan library from current browser dir (or rescan if panel==2) */
	case 'L':
		if (g_view == 1 && !g_lib_scanning) {
			const char *root = (g_panel == 2 && g_lib_root[0]) ?
						   g_lib_root :
						   g_fb_path;
			lib_start_scan(root);
			g_panel = 2;
			if (g_view != 1)
				g_view = 1;
		}
		break;

	/* i = MusicBrainz tag lookup */
	case 'i':
		if (g_tag_info.visible) {
			g_tag_info.visible = 0;
			break;
		}
		if (g_view == 1 && g_panel == 0 && g_fb_count > 0 &&
		    !g_fb_entries[g_fb_sel].is_dir)
			tag_lookup_start(g_fb_entries[g_fb_sel].name);
		else if (g_view == 1 && g_panel == 1 && g_pl_count > 0)
			tag_lookup_start(g_pl[g_pl_sel].name);
		else if (g_view == 1 && g_panel == 2 && g_lib &&
			 g_lib_count > 0)
			tag_lookup_start(g_lib[g_lib_sel].name);
		break;

	/* DEL = remove from playlist */
	case KEY_DC:
		if (g_view == 1 && g_panel == 1 && g_pl_count > 0)
			pl_remove(g_pl_sel);
		break;

	/* C-x = clear entire playlist */
	case 24:
		if (g_view == 1 && g_panel == 1) {
			g_pl_count = 0;
			g_pl_sel = 0;
			g_pl_scroll = 0;
		}
		break;

	/* Open browser at home dir */
	case '~': {
		const char *home = getenv("HOME");
		if (home) {
			snprintf(g_fb_path, FB_PATH_MAX, "%s", home);
			fb_scan();
			g_view = 1;
		}
		break;
	}

	/* Open browser at / */
	case '\\':
		strcpy(g_fb_path, "/");
		fb_scan();
		g_view = 1;
		break;

	case 1: /* Ctrl+A -- Toggle Autoplay */
		g_opts.library_autoplay = !g_opts.library_autoplay;
		snprintf(g_fb_status, sizeof(g_fb_status), "Autoplay: %s",
			 g_opts.library_autoplay ? "ON" : "OFF");
		settings_save();
		break;

	/* Toggle 2/4 decks */
	case 'T':
		g_num_tracks = (g_num_tracks == 2) ? 4 : 2;
		if (g_num_tracks == 4)
			g_view = 0; /* jump to decks view */
		else
			g_view = 1; /* restore split view with last panel */
		if (g_active_track >= g_num_tracks)
			g_active_track = 0;
		settings_save();
		break;

	/* Quit */
	case 'Q':
		quit_confirm();
		break;

	/* ── Navigation Pane Shortcuts ── */
	case 6: /* Ctrl+F = Files */
		g_tag_info.visible = 0;
		g_panel = 0;
		if (g_view != 1)
			g_view = 1;
		break;
	case 12: /* Ctrl+L = Library */
		g_tag_info.visible = 0;
		g_panel = 2;
		if (g_view != 1)
			g_view = 1;
		break;
	case 3: /* Ctrl+C = Crates */
		g_tag_info.visible = 0;
		g_panel = 3;
		if (g_view != 1)
			g_view = 1;
		crates_load();
		break;

	/* ── ESC opens options ── */
	case 27:
		g_options_open = 1;
		g_options_tab = 0;
		g_options_sel = 0;
		break;
	}
}

/* ──────────────────────────────────────────────
   Splash screen helpers
   ────────────────────────────────────────────── */
static int utf8_cols(const char *s)
{
	int n = 0;
	while (*s) {
		unsigned char c = (unsigned char)*s;
		wchar_t wc;
		int len;
		if (c < 0x80) {
			wc = c; len = 1;
		} else if (c >= 0xF0 && s[1] && s[2] && s[3]) {
			wc = ((wchar_t)(c & 0x07) << 18) |
			     ((wchar_t)((unsigned char)s[1] & 0x3F) << 12) |
			     ((wchar_t)((unsigned char)s[2] & 0x3F) << 6) |
			     ((wchar_t)((unsigned char)s[3] & 0x3F));
			len = 4;
		} else if (c >= 0xE0 && s[1] && s[2]) {
			wc = ((wchar_t)(c & 0x0F) << 12) |
			     ((wchar_t)((unsigned char)s[1] & 0x3F) << 6) |
			     ((wchar_t)((unsigned char)s[2] & 0x3F));
			len = 3;
		} else if (c >= 0xC0 && s[1]) {
			wc = ((wchar_t)(c & 0x1F) << 6) |
			     ((wchar_t)((unsigned char)s[1] & 0x3F));
			len = 2;
		} else {
			s++; continue;
		}
		int w = wcwidth(wc);
		if (w > 0) n += w;
		s += len;
	}
	return n;
}

static void splash_logo_line(int y, int lx, const char *s)
{
	if (lx >= 0) { mvwaddstr(g_win_main, y, lx, s); return; }
	int skip = -lx;
	const char *p = s;
	while (*p && skip > 0) {
		unsigned char c = (unsigned char)*p;
		int w;
		if (c < 0x80)       { w = 1; p += 1; }
		else if (c >= 0xF0) { w = 2; p += 4; }
		else if (c >= 0xE0) { w = 2; p += 3; }
		else if (c >= 0xC0) { w = 1; p += 2; }
		else                { w = 0; p += 1; }
		skip -= w;
	}
	int start_col = (skip < 0) ? -skip : 0;
	if (*p) mvwaddstr(g_win_main, y, start_col, p);
}

static void draw_splash(int64_t elapsed_ms)
{
	static const char *logo[] = {
		"\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\x20\x20\x20\x20\x20\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91",
		"\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\x20\x20\x20\x20\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91",
		"\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\x20\x20\x20\x20\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\x20\x20\x20\x20\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91",
		"\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\x20\x20\x20\x20\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\x20\x20\x20\x20\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91",
		"\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\x20\x20\x20\x20\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91",
		"\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91",
		"\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\x20\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91\xe2\x96\x92\xe2\x96\x93\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x88\xe2\x96\x93\xe2\x96\x92\xe2\x96\x91",
	};
	static const int nlogo = 7;

	werase(g_win_main);

	int logo_w = utf8_cols(logo[0]);
	int lx = (g_cols - logo_w) / 2;
	int ly = (g_rows - nlogo - 5) / 2;
	if (ly < 0) ly = 0;

	wattron(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);
	for (int i = 0; i < nlogo; i++)
		splash_logo_line(ly + i, lx, logo[i]);
	wattroff(g_win_main, COLOR_PAIR(COLOR_ACTIVE) | A_BOLD | A_REVERSE);

	const char *tagline = "Terminal DJ Application";
	int tx = (g_cols - (int)strlen(tagline)) / 2;
	wattron(g_win_main, A_BOLD);
	mvwaddstr(g_win_main, ly + nlogo + 1, tx, tagline);
	wattroff(g_win_main, A_BOLD);

	int64_t remain_ms = 2000 - elapsed_ms;
	if (remain_ms < 0) remain_ms = 0;
	int bar_w = 30;
	int filled = (int)(bar_w * remain_ms / 2000);
	if (filled > bar_w) filled = bar_w;
	char bar[64];
	int bi = 0;
	bar[bi++] = '[';
	for (int i = 0; i < bar_w; i++)
		bar[bi++] = (i < filled) ? '#' : ' ';
	bar[bi++] = ']';
	bar[bi] = '\0';

	int bx = (g_cols - bar_w - 2) / 2;
	wattron(g_win_main, A_DIM);
	mvwaddstr(g_win_main, ly + nlogo + 2, bx, bar);
	const char *hint = "Press any key to skip";
	int hx = (g_cols - (int)strlen(hint)) / 2;
	mvwaddstr(g_win_main, ly + nlogo + 3, hx, hint);
	wattroff(g_win_main, A_DIM);

	wrefresh(g_win_main);
}

void *ui_thread(void *arg)
{
	(void)arg;

	/* ── Splash screen (2 s, skippable) ── */
	{
		struct timespec t0, tn;
		clock_gettime(CLOCK_MONOTONIC, &t0);
		wtimeout(g_win_main, 50);
		for (;;) {
			clock_gettime(CLOCK_MONOTONIC, &tn);
			int64_t elapsed =
				(int64_t)(tn.tv_sec - t0.tv_sec) * 1000 +
				(tn.tv_nsec - t0.tv_nsec) / 1000000;
			if (elapsed >= 2000)
				break;
			draw_splash(elapsed);
			if (wgetch(g_win_main) != ERR)
				break;
		}
		wclear(g_win_main);
		wrefresh(g_win_main);
		apply_ui_fps();
	}

	struct timespec ts;
	while (g_running) {
		batch_tick(); /* advance batch BPM queue when load worker signals done */

		/* ── Library Autoplay state machine ── */
		if (g_opts.library_autoplay) {
			for (int i = 0; i < 2; i++) {
				Track *tr = &g_tracks[i];
				int other = 1 - i;
				if (!tr->playing || !tr->loaded)
					continue;
				float frames_left =
					(float)(tr->num_frames - tr->pos);
				float sec30_frames =
					30.0f * (float)g_actual_sample_rate;
				float bar8_frames =
					(tr->bpm > 0.0f) ?
						((float)g_actual_sample_rate *
						 60.0f / tr->bpm) *
							32.0f :
						8.0f *
							(float)
								g_actual_sample_rate;

				/* Stage 1: load next track when 30 s remain */
				if (frames_left <= sec30_frames &&
				    !g_autoplay_pending[other] &&
				    !g_autoplay_ready[other] &&
				    !g_tracks[other].playing) {
					char next_path[FB_PATH_MAX + 512];
					next_path[0] = '\0';
					int found = 0;
					if (g_panel == 0 && g_fb_count > 0) {
						for (int j = 1;
						     j <= g_fb_count; j++) {
							int idx = (g_fb_sel + j) %
								  g_fb_count;
							if (!g_fb_entries[idx]
								 .is_dir) {
								g_fb_sel = idx;
								fb_selected_path(
									next_path,
									sizeof(next_path));
								found = 1;
								break;
							}
						}
					} else if (g_panel == 1 &&
						   g_pl_count > 0) {
						g_pl_sel = (g_pl_sel + 1) %
							   g_pl_count;
						strncpy(next_path,
							g_pl[g_pl_sel].path,
							sizeof(next_path) - 1);
						found = 1;
					} else if (g_panel == 2 &&
						   g_lib_count > 0) {
						g_lib_sel = (g_lib_sel + 1) %
							    g_lib_count;
						strncpy(next_path,
							g_lib[g_lib_sel].path,
							sizeof(next_path) - 1);
						found = 1;
					} else if (g_panel == 3 &&
						   g_crate_view_level == 1 &&
						   g_crate_tracks_count > 0) {
						g_crate_tracks_sel =
							(g_crate_tracks_sel + 1) %
							g_crate_tracks_count;
						strncpy(next_path,
							g_crate_tracks
								[g_crate_tracks_sel]
									.path,
							sizeof(next_path) - 1);
						found = 1;
					}
					if (found && next_path[0]) {
						g_autoplay_pending[other] = 1;
						enqueue_load(other, next_path);
						snprintf(g_fb_status,
							 sizeof(g_fb_status),
							 "Autoplay: Loading Deck %c...",
							 'A' + other);
					}
				}

				/* Stage 2: start playing when 8 bars remain */
				if (frames_left <= bar8_frames &&
				    g_autoplay_ready[other] &&
				    !g_tracks[other].playing) {
					pthread_mutex_lock(
						&g_tracks[other].lock);
					g_tracks[other].playing = 1;
					pthread_mutex_unlock(
						&g_tracks[other].lock);
					g_autoplay_ready[other] = 0;
					g_autoplay_xf_target =
						(other == 0) ? 0.0f : 1.0f;
					snprintf(g_fb_status,
						 sizeof(g_fb_status),
						 "Autoplay Mix \xe2\x86\x92 Deck %c",
						 'A' + other);
				}

				/* Stop finished deck at end */
				if (frames_left <= 1024) {
					pthread_mutex_lock(&tr->lock);
					tr->playing = 0;
					pthread_mutex_unlock(&tr->lock);
					g_autoplay_ready[i] = 0;
				}
			}

			/* Smooth crossfade automation */
			if (g_autoplay_xf_target >= 0.0f) {
				float step = 0.005f;
				if (g_crossfader < g_autoplay_xf_target) {
					g_crossfader += step;
					if (g_crossfader > g_autoplay_xf_target)
						g_crossfader =
							g_autoplay_xf_target;
				} else if (g_crossfader >
					   g_autoplay_xf_target) {
					g_crossfader -= step;
					if (g_crossfader < g_autoplay_xf_target)
						g_crossfader =
							g_autoplay_xf_target;
				}
				if (g_crossfader == g_autoplay_xf_target)
					g_autoplay_xf_target = -1.0f;
			}
		} else {
			for (int i = 0; i < MAX_TRACKS; i++) {
				g_autoplay_pending[i] = 0;
				g_autoplay_ready[i] = 0;
			}
			g_autoplay_xf_target = -1.0f;
		}

		redraw();
		int c = wgetch(g_win_main);
		if (c != ERR)
			handle_key(c);
		ts.tv_sec = 0;
		ts.tv_nsec = (1000 / g_opts.ui_fps) * 1000000;
		nanosleep(&ts, NULL);
	}
	return NULL;
}
