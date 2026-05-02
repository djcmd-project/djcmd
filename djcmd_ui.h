#ifndef DJCMD_UI_H
#define DJCMD_UI_H

#include "djcmd_shared.h"
#include <ncurses.h>

/* ── UI Constants ────────────────────────────────────────────────────── */
#define WFM_GRADIENT_PAIRS 216
#define WFM_PAIR_BASE 16 /* color pairs 16-231 */

#define WFM_8_GREEN 7
#define WFM_8_YELLOW 8
#define WFM_8_RED 9
#define WFM_8_LO_HI 10
#define WFM_8_LO_MID 11
#define WFM_8_MID_HI 12
#define WFM_8_KICK 13
#define WFM_8_MID 14
#define WFM_8_HI 15

/* ── UI Global State ─────────────────────────────────────────────────── */
extern WINDOW *g_win_main;
extern WINDOW *g_win_status;
extern int g_rows, g_cols;
extern int g_active_track;
extern int g_view;
extern int g_panel;
extern int g_help_scroll;
extern int g_options_open;
extern int g_quit_pending;
extern int g_options_menu;
extern int g_adv_sel;
extern int g_options_tab;
extern int g_options_sel;
extern int g_options_out_sel;
extern int g_blink_tick;
extern int g_bpm_entry;
extern char g_bpm_buf[8];
extern int g_bpm_deck;
extern Options g_opts;
extern char g_cpuinfo_cache[256];
extern char g_meminfo_cache[256];
extern time_t g_meminfo_last_t;

/* ── Globals from djcmd.c ────────────────────────────────────────────── */
extern Track g_tracks[MAX_TRACKS];
extern int g_num_tracks;
extern _Atomic int g_sync_leader;
extern int g_gang_mode;
extern int g_gang_mask;
extern unsigned int g_actual_sample_rate;
extern _Atomic float g_vu_l, g_vu_r;
extern _Atomic float g_vu_peak_l, g_vu_peak_r;
extern _Atomic float g_crossfader;
extern int g_hp_vol;
extern _Atomic int g_master_vol;
extern _Atomic int g_running;
extern int g_is_tty;
extern int g_pad_mode[MAX_TRACKS];
extern TagInfo g_tag_info;
extern WSOLAState g_wsola[MAX_TRACKS];
extern int g_side_deck[2];
extern int g_has_256;
extern int g_fx_ui_slot[MAX_TRACKS];
extern _Atomic float g_autoplay_xf_target;
extern _Atomic int g_autoplay_pending[MAX_TRACKS];
extern _Atomic int g_autoplay_ready[MAX_TRACKS];
extern int g_slip_motor_off[MAX_TRACKS];

/* View auto-switch state */
extern int64_t g_lib_enc_last_ms;
extern int g_lib_auto_switched;
extern int g_lib_touched;

/* ── UI Functions ───────────────────────────────────────────────────── */
void init_colors(void);
void apply_theme(int idx);
void redraw(void);
void handle_key(int ch);
void draw_waveform(WINDOW *w, int y, int x, int width, Track *t);
void draw_status(void);
void draw_options_overlay(void);
void apply_ui_fps(void);
void *ui_thread(void *arg);
void wfm_compute_band_max(Track *t);
void options_read_cpuinfo(char *out, int max);
void options_read_meminfo(char *out, int max);

#endif /* DJCMD_UI_H */
