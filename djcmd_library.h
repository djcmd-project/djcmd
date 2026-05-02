#ifndef DJCMD_LIBRARY_H
#define DJCMD_LIBRARY_H

#include "djcmd_shared.h"

/* Library & Crate management */
void crates_load(void);
void crate_view_open(int idx);
void sidecar_save(const Track *t);
int sidecar_load(Track *t);
float cache_get_bpm(const char *audio_path);

/* ── Promoted from djcmd.c ── */
void fb_fix_scroll(int list_rows);
void pl_fix_scroll(int list_rows);
const char *fb_display_name(const FBEntry *e, char *buf, int max);
void fb_selected_path(char *out, size_t max);
void fb_enter_dir(const char *name);
void fb_scan(void);
void fb_apply_sort(void);
void pl_add(const char *path, const char *name, float bpm, const char *key);
void pl_remove(int idx);
void lib_start_scan(const char *root);
void lib_apply_sort(void);
void tag_lookup_start(const char *name);
int crate_total_vrows(void);
void crate_vrow_resolve(int vrow, int *type, int *index);
void crate_jump(const char *alias);
void crate_create(const char *name);
void crate_add_to(int idx, const char *path);
void crate_remove_at(int crate_idx, int entry_idx);
void update_crate_matches(const char *input, const char *prefix);
int mixxx_import(const char *audio_path, MixxxMeta *out);
void snap_grid(int deck);
void tap_bpm(int deck);
void settings_save(void);
void settings_load(void);
void ensure_sidecar_cache_dir(void);
void read_tags(const char *path, char *title, int tmax, char *artist, int amax);

/* ── Global State ── */
extern char g_fb_path[FB_PATH_MAX];
extern FBEntry g_fb_entries[FB_MAX_ENTRIES];
extern int g_fb_count;
extern int g_fb_sel;
extern int g_fb_scroll;
extern char g_fb_status[256];
extern int g_fb_sort;

extern PLEntry g_pl[PL_MAX];
extern int g_pl_count;
extern int g_pl_sel;
extern int g_pl_scroll;

extern LIBEntry *g_lib;
extern int g_lib_count;
extern int g_lib_sel;
extern int g_lib_scroll;
extern volatile int g_lib_scanning;
extern char g_lib_root[FB_PATH_MAX];
extern int g_lib_sort;

extern Crate g_crates[MAX_CRATES];
extern int g_ncrate;
extern int g_crate_sel;
extern int g_crate_vrow;
extern int g_crate_vscroll;
extern CrateEntry *g_crate_tracks;
extern int g_crate_tracks_count;
extern int g_crate_tracks_sel;
extern int g_crate_tracks_scroll;
extern int g_crate_view_level;
extern char g_active_crate_name[256];
extern CrateGroup g_crate_groups[32];
extern int g_ncrate_groups;

extern int g_crate_jump_active;
extern char g_crate_input[64];
extern char g_crate_orig_input[64];
extern int g_crate_cycle_idx;
extern int g_crate_matches[MAX_CRATES];
extern int g_ncrate_matches;

extern int g_crate_add_active;
extern char g_crate_add_input[64];
extern char g_crate_add_path[FB_PATH_MAX];

extern int g_track_add_crate_active;
extern char g_track_add_crate_input[64];
extern char g_pending_track_path[FB_PATH_MAX];

extern USBDevice g_usb_devices[USB_MAX_DEVICES];
extern int g_usb_devices_count;
extern int g_usb_eject_active;
extern char g_usb_eject_mount[512];
extern char g_usb_eject_label[128];

extern int g_usb_picker_active;
extern int g_usb_picker_sel;
extern int g_usb_export_crate_idx;

extern int g_usb_conflict_active;
extern char g_usb_conflict_rename[64];
extern char g_usb_conflict_mount[512];
extern char g_usb_conflict_crate_name[64];

extern int g_batch_prompt_active;
extern int g_batch_prompt_field;
extern char g_batch_lo_buf[16];
extern char g_batch_hi_buf[16];
extern int g_batch_running;
extern BatchQEntry *g_batch_queue;
extern int g_batch_queue_pos;
extern int g_batch_queue_count;
extern int g_batch_panel;
extern int g_batch_job_done;
extern int64_t g_n_last_tap_ms;

extern float g_bpm_detect_lo;
extern float g_bpm_detect_hi;

extern char g_machine_id[64];

/* ── Session Play History ── */
void history_mark_played(const char *path);
int history_was_played(const char *path);

/* ── Session Mix Log ── */
void mixlog_open(void);
void mixlog_track_loaded(int deck, Track *t);
void mixlog_close(void);
const char *mixlog_get_path(void);

#endif /* DJCMD_LIBRARY_H */
