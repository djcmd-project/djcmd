#ifndef DJCMD_SHARED_H
#define DJCMD_SHARED_H

#include <stdint.h>
#include <pthread.h>
#include <stdatomic.h>
#include "djcmd_config.h"

/* ── Constants ──────────────────────────────────────────────────────── */
#ifndef MAX_TRACKS
#define MAX_TRACKS 4
#endif

#define MAX_SAMPLER_SLOTS 8
#define MAX_CUES 8
#define WFM_ROWS CFG_WFM_ROWS
#define FB_PATH_MAX CFG_FB_PATH_MAX
#define PL_MAX CFG_PL_MAX
#define MAX_FILENAME 512
#define MAX_CRATES 128
#define MAX_CRATE_ENTRIES 2048
#define USB_MAX_DEVICES 8
#define MIDI_MAX_BINDINGS 256
#define MIDI_MAX_OUT_BINDINGS 256
#define MIDI_MON_SIZE 128
#define FB_MAX_ENTRIES CFG_FB_MAX_ENTRIES
#define LIB_MAX 8192
#define MAX_TAPS 8

#define PAD_MODE_HOTCUE 0
#define PAD_MODE_AUTOLOOP 1
#define PAD_MODE_ROLL 2
#define PAD_MODE_MANUALLOOP 3
#define PAD_MODE_SAMPLER 4
#define PAD_MODE_SLICER 5

/* Macros */
#define DECK_NUM(d) ('1' + (d))

/* ── Shared Structs (Pre-Enum) ───────────────────────────────────────── */

typedef struct {
	uint8_t status, d1, d2;
	int matched_act;
} MidiMonEntry;

/* ── MIDI Actions ───────────────────────────────────────────────────── */
typedef enum {
	MACT_NONE = 0,
	MACT_DECK_VOL_A,
	MACT_DECK_VOL_B,
	MACT_DECK_VOL_C,
	MACT_DECK_VOL_D,
	MACT_PITCH_A,
	MACT_PITCH_B,
	MACT_PITCH_C,
	MACT_PITCH_D,
	MACT_PITCH_LSB_A,
	MACT_PITCH_LSB_B,
	MACT_PITCH_LSB_C,
	MACT_PITCH_LSB_D,
	MACT_VOL_LSB_A,
	MACT_VOL_LSB_B,
	MACT_VOL_LSB_C,
	MACT_VOL_LSB_D,
	MACT_EQ_LOW_LSB_A,
	MACT_EQ_LOW_LSB_B,
	MACT_EQ_LOW_LSB_C,
	MACT_EQ_LOW_LSB_D,
	MACT_EQ_MID_LSB_A,
	MACT_EQ_MID_LSB_B,
	MACT_EQ_MID_LSB_C,
	MACT_EQ_MID_LSB_D,
	MACT_EQ_HIGH_LSB_A,
	MACT_EQ_HIGH_LSB_B,
	MACT_EQ_HIGH_LSB_C,
	MACT_EQ_HIGH_LSB_D,
	MACT_GAIN_LSB_A,
	MACT_GAIN_LSB_B,
	MACT_GAIN_LSB_C,
	MACT_GAIN_LSB_D,
	MACT_FILTER_LSB_A,
	MACT_FILTER_LSB_B,
	MACT_FILTER_LSB_C,
	MACT_FILTER_LSB_D,
	MACT_CROSSFADER_LSB,
	MACT_MASTER_VOL_LSB,
	MACT_EQ_LOW_A,
	MACT_EQ_LOW_B,
	MACT_EQ_LOW_C,
	MACT_EQ_LOW_D,
	MACT_EQ_MID_A,
	MACT_EQ_MID_B,
	MACT_EQ_MID_C,
	MACT_EQ_MID_D,
	MACT_EQ_HIGH_A,
	MACT_EQ_HIGH_B,
	MACT_EQ_HIGH_C,
	MACT_EQ_HIGH_D,
	MACT_GAIN_A,
	MACT_GAIN_B,
	MACT_GAIN_C,
	MACT_GAIN_D,
	MACT_FILTER_A,
	MACT_FILTER_B,
	MACT_FILTER_C,
	MACT_FILTER_D,
	MACT_FILTER_TOGGLE_A,
	MACT_FILTER_TOGGLE_B,
	MACT_FILTER_TOGGLE_C,
	MACT_FILTER_TOGGLE_D,
	MACT_CROSSFADER,
	MACT_MASTER_VOL,
	MACT_BOOTH_VOL,
	MACT_PLAY_A,
	MACT_PLAY_B,
	MACT_PLAY_C,
	MACT_PLAY_D,
	MACT_CUE_ACTIVE_A,
	MACT_CUE_ACTIVE_B,
	MACT_CUE_ACTIVE_C,
	MACT_CUE_ACTIVE_D,
	MACT_CUE_SET_1,
	MACT_CUE_SET_2,
	MACT_CUE_SET_3,
	MACT_CUE_SET_4,
	MACT_CUE_JUMP_1,
	MACT_CUE_JUMP_2,
	MACT_CUE_JUMP_3,
	MACT_CUE_JUMP_4,
	MACT_CUE_DELETE_1,
	MACT_CUE_DELETE_2,
	MACT_CUE_DELETE_3,
	MACT_CUE_DELETE_4,
	MACT_SYNC_FOLLOW_A,
	MACT_SYNC_FOLLOW_B,
	MACT_SYNC_FOLLOW_C,
	MACT_SYNC_FOLLOW_D,
	MACT_NUDGE_FWD,
	MACT_NUDGE_BACK,
	MACT_NUDGE_FWD_B,
	MACT_NUDGE_BACK_B,
	MACT_LOOP_TOGGLE,
	MACT_LOOP_IN_A,
	MACT_LOOP_IN_B,
	MACT_LOOP_IN_C,
	MACT_LOOP_IN_D,
	MACT_LOOP_OUT_A,
	MACT_LOOP_OUT_B,
	MACT_LOOP_OUT_C,
	MACT_LOOP_OUT_D,
	MACT_LOOP_DOUBLE_A,
	MACT_LOOP_DOUBLE_B,
	MACT_LOOP_DOUBLE_C,
	MACT_LOOP_DOUBLE_D,
	MACT_LOOP_HALF_A,
	MACT_LOOP_HALF_B,
	MACT_LOOP_HALF_C,
	MACT_LOOP_HALF_D,
	MACT_KEY_LOCK_A,
	MACT_KEY_LOCK_B,
	MACT_KEY_LOCK_C,
	MACT_KEY_LOCK_D,
	MACT_SLIP_MODE_A,
	MACT_SLIP_MODE_B,
	MACT_SLIP_MODE_C,
	MACT_SLIP_MODE_D,
	MACT_REVERSE_A,
	MACT_REVERSE_B,
	MACT_REVERSE_C,
	MACT_REVERSE_D,
	MACT_CENSOR_A,
	MACT_CENSOR_B,
	MACT_CENSOR_C,
	MACT_CENSOR_D,
	MACT_STRIP_A,
	MACT_STRIP_B,
	MACT_STRIP_C,
	MACT_STRIP_D,
	MACT_JOG_TOUCH_A,
	MACT_JOG_TOUCH_B,
	MACT_JOG_TOUCH_C,
	MACT_JOG_TOUCH_D,
	MACT_JOG_SPIN_A,
	MACT_JOG_SPIN_B,
	MACT_JOG_SPIN_C,
	MACT_JOG_SPIN_D,
	MACT_JOG_PB_A,
	MACT_JOG_PB_B,
	MACT_JOG_PB_C,
	MACT_JOG_PB_D,
	MACT_LIB_ENCODER,
	MACT_LIB_ENCODER_TOUCH,
	MACT_LIB_SELECT,
	MACT_LIB_BACK,
	MACT_LIB_FWD,
	MACT_LIB_LOAD_A,
	MACT_LIB_LOAD_B,
	MACT_LIB_LOAD_C,
	MACT_LIB_LOAD_D,
	MACT_PANEL_FILES,
	MACT_PANEL_LIBRARY,
	MACT_PITCH_RANGE_A,
	MACT_PITCH_RANGE_B,
	MACT_PITCH_RANGE_C,
	MACT_PITCH_RANGE_D,
	MACT_PITCH_BEND_A,
	MACT_PITCH_BEND_B,
	MACT_PITCH_BEND_C,
	MACT_PITCH_BEND_D,
	MACT_MOTOR_TOGGLE_A,
	MACT_MOTOR_TOGGLE_B,
	MACT_MOTOR_TOGGLE_C,
	MACT_MOTOR_TOGGLE_D,
	MACT_MOTOR_ON_A,
	MACT_MOTOR_ON_B,
	MACT_MOTOR_ON_C,
	MACT_MOTOR_ON_D,
	MACT_MOTOR_OFF_A,
	MACT_MOTOR_OFF_B,
	MACT_MOTOR_OFF_C,
	MACT_MOTOR_OFF_D,
	MACT_DECK_SEL_1,
	MACT_DECK_SEL_2,
	MACT_DECK_SEL_3,
	MACT_DECK_SEL_4,
	MACT_SHIFT_A,
	MACT_SHIFT_B,
	MACT_PITCH_CENTER_A,
	MACT_PITCH_CENTER_B,
	MACT_CUE_DEFAULT_A,
	MACT_CUE_DEFAULT_B,
	MACT_PAD_MODE_CUES_A,
	MACT_PAD_MODE_CUES_B,
	MACT_PAD_MODE_AUTOROLL_A,
	MACT_PAD_MODE_AUTOROLL_B,
	MACT_PAD_MODE_MANUAL_A,
	MACT_PAD_MODE_MANUAL_B,
	MACT_PAD_MODE_SAMPLER_A,
	MACT_PAD_MODE_SAMPLER_B,
	MACT_PAD_MODE_SLICER_A,
	MACT_PAD_MODE_SLICER_B,
	MACT_PAD_1_A,
	MACT_PAD_2_A,
	MACT_PAD_3_A,
	MACT_PAD_4_A,
	MACT_PAD_5_A,
	MACT_PAD_6_A,
	MACT_PAD_7_A,
	MACT_PAD_8_A,
	MACT_PAD_1_B,
	MACT_PAD_2_B,
	MACT_PAD_3_B,
	MACT_PAD_4_B,
	MACT_PAD_5_B,
	MACT_PAD_6_B,
	MACT_PAD_7_B,
	MACT_PAD_8_B,
	MACT_PARAM_LEFT_A,
	MACT_PARAM_RIGHT_A,
	MACT_PARAM_LEFT_B,
	MACT_PARAM_RIGHT_B,
	MACT_FX_BTN_1_A,
	MACT_FX_BTN_2_A,
	MACT_FX_BTN_3_A,
	MACT_FX_BTN_1_B,
	MACT_FX_BTN_2_B,
	MACT_FX_BTN_3_B,
	MACT_FX_KNOB_1_A,
	MACT_FX_KNOB_2_A,
	MACT_FX_KNOB_3_A,
	MACT_FX_KNOB_1_B,
	MACT_FX_KNOB_2_B,
	MACT_FX_KNOB_3_B,
	MACT_FX_WET_A,
	MACT_FX_WET_B,
	MACT_CF_CURVE,
	MACT_TAP_BPM_A,
	MACT_TAP_BPM_B,
	MACT_GRID_SNAP_A,
	MACT_GRID_SNAP_B,
	MACT_EQ_LOW_KILL_A,
	MACT_EQ_LOW_KILL_B,
	MACT_EQ_LOW_KILL_C,
	MACT_EQ_LOW_KILL_D,
	MACT_EQ_MID_KILL_A,
	MACT_EQ_MID_KILL_B,
	MACT_EQ_MID_KILL_C,
	MACT_EQ_MID_KILL_D,
	MACT_EQ_HIGH_KILL_A,
	MACT_EQ_HIGH_KILL_B,
	MACT_EQ_HIGH_KILL_C,
	MACT_EQ_HIGH_KILL_D,
	MACT_FILTER_ROLL_TOUCH_A,
	MACT_FILTER_ROLL_TOUCH_B,
	MACT_FILTER_ROLL_TOUCH_C,
	MACT_FILTER_ROLL_TOUCH_D,
	MACT_TOUCH_MODE_TOGGLE,
	MACT_DECK_PITCH_A,
	MACT_DECK_PITCH_B,
	MACT_DECK_PITCH_C,
	MACT_DECK_PITCH_D,
	MACT_COUNT
} MidiAction;

/* ── Shared Structs ──────────────────────────────────────────────────── */

typedef struct {
	int16_t *data;
	uint32_t num_frames;
	uint32_t pos;
	float volume;
	float pitch;
	float nudge;
	float gain;
	float eq_low;
	float eq_mid;
	float eq_high;
	float filter;
	float fader;
	int playing;
	int pending_play;
	int loaded;
	char filename[MAX_FILENAME];
	pthread_mutex_t lock;
	int looping;
	int reverse;
	uint32_t loop_start;
	uint32_t loop_end;
	int synced;
	float bpm;
	float bpm_offset;
	int bpm_display_double;
	uint32_t cue[MAX_CUES];
	int cue_set[MAX_CUES];
	int cue_active;
	int key_lock;
	float period_peak;
	uint8_t *wfm_low, *wfm_mid, *wfm_high;
	uint32_t wfm_bins;
	float wfm_band_max[3];
	char tag_artist[128];
	char tag_title[128];
	char tag_key[16];
	double slip_pos;
} Track;

#define PV_N 512
#define PV_HS 128
#define PV_BINS (PV_N / 2 + 1)
#define PV_BUF 4096

typedef struct {
	float ph_an_l[PV_BINS];
	float ph_an_r[PV_BINS];
	float ph_syn_l[PV_BINS];
	float ph_syn_r[PV_BINS];
	double src_pos;
	float out_l[PV_BUF];
	float out_r[PV_BUF];
	int out_write;
	int out_read;
	int out_fill;
	int initialized;
} PVState;

#define WSOLA_WIN 1024
#define WSOLA_BUF 8192
#define WSOLA_HOP 256
#define WSOLA_SEARCH 512

typedef struct {
	float buf_l[WSOLA_BUF];
	float buf_r[WSOLA_BUF];
	float prev_l[WSOLA_WIN];
	float prev_r[WSOLA_WIN];
	int write_pos;
	int read_pos;
	int fill;
	double src_pos;
	int prev_valid;
	int init;
} WSOLAState;

typedef struct {
	uint8_t status;
	uint8_t data1;
	MidiAction action;
	uint8_t relative;
	float rel_acc;
} MidiBinding;

typedef struct {
	char artist[128];
	char title[128];
	char album[128];
	char date[32];
	char label[128];
	char status[64];
	int visible;
	char query_name[256];
} TagInfo;

typedef struct {
	char name[256];
	int is_dir;
	float bpm;
	char tag_title[128];
	char tag_artist[128];
	char tag_key[16];
} FBEntry;

typedef struct {
	char path[512];
	char name[256];
	float bpm;
	char tag_key[16];
} PLEntry;

typedef struct {
	char path[512];
	char name[256];
	float bpm;
	char tag_artist[128];
	char tag_title[128];
	char tag_key[16];
} LIBEntry;

typedef struct {
	char name[64];
	char alias[32];
	char filename[512];
	char path[512];
	char origin[64];
	char usb_label[128];
	char mount_point[512];
	int is_usb;
} Crate;

typedef struct {
	char path[512];
	char name[256];
	float bpm;
	char tag_key[16];
} CrateEntry;

typedef struct {
	int start_idx;
	int count;
	char label[128];
	int is_usb;
	char mount_point[512];
} CrateGroup;

typedef struct {
	char label[64];
	char mount_point[FB_PATH_MAX];
} USBDevice;

typedef struct {
	int16_t *data;
	uint32_t num_frames;
	uint32_t pos;
	int playing;
	int looping;
	char name[128];
	char filename[MAX_FILENAME];
	float volume;
	pthread_mutex_t lock;
	float params[4];
} SamplerSlot;

typedef struct {
	float lp_x1l, lp_x2l, lp_y1l, lp_y2l;
	float lp_x1r, lp_x2r, lp_y1r, lp_y2r;
	float bp_x1l, bp_x2l, bp_y1l, bp_y2l;
	float bp_x1r, bp_x2r, bp_y1r, bp_y2r;
	float hp_x1l, hp_x2l, hp_y1l, hp_y2l;
	float hp_x1r, hp_x2r, hp_y1r, hp_y2r;
	float fi_x1l, fi_x2l, fi_y1l, fi_y2l;
	float fi_x1r, fi_x2r, fi_y1r, fi_y2r;
	float fi_b[3], fi_a[3];
	float fi_last;
} EQState;

typedef struct {
	int default_master_vol;
	float default_deck_vol;
	int auto_gain_default;
	float auto_gain_target_db;
	float wfm_visible_secs;
	int wfm_overview_bins;
	float kick_threshold;
	float wfm_height_gamma;
	int theme_idx;
	int wfm_style;
	int sync_quantize;
	int sync_smart_range;
	int sync_auto_handoff;
	int key_lock_default;
	int vinyl_mode;
	int ui_fps;
	float wfm_lo_weight;
	float wfm_mid_weight;
	float wfm_hi_weight;
	float wfm_color_sat;
	float wfm_color_floor;
	int wfm_anchor;
	int eco_mode;
	int library_autoplay;
	int enable_slicer;
	int period_frames;   /* ALSA period size (512/1024/2048); take effect on reinit */
	int buffer_periods;  /* ALSA ring buffer depth (2-16); take effect on reinit */
} Options;

typedef struct {
	char *path;
	int panel_idx;
} BatchQEntry;

typedef struct {
	float bpm;
	float bpm_offset;
	int samplerate;
	uint32_t cue[MAX_CUES];
	int cue_set[MAX_CUES];
	uint32_t *beat_frames;
	uint32_t n_beats;
	char tag_key[16];
	int found;
} MixxxMeta;

typedef struct {
	double freq;
	double integrator;
} PLLState;

typedef struct {
	char name[128];
	char dev[64];
} MidiDevice;

typedef struct {
	char name[128];
	char dev[64];
} PCMDevice;

typedef struct {
	char path[FB_PATH_MAX + 256];
	int deck;
	int valid;
	int analyze_only;
	int batch_path_only;
} LoadJob;

/* ── Globals ────────────────────────────────────────────────────────── */
extern Track g_tracks[MAX_TRACKS];
extern int g_num_tracks;
extern _Atomic float g_crossfader;
extern _Atomic int g_master_vol;
extern _Atomic int g_running;
extern int g_hp_vol;
extern char g_pcm_dev_str[64];
extern char g_pcm_hp_dev_str[64];
extern int g_pitch_range[MAX_TRACKS];
extern _Atomic int g_sync_leader;
extern int g_gang_mask;
extern int g_gang_mode;
extern int64_t g_tap_ms[MAX_TRACKS][MAX_TAPS];
extern int g_tap_idx[MAX_TRACKS];
extern int g_tap_count[MAX_TRACKS];
extern char g_fb_status[256];
extern Options g_opts;
extern TagInfo g_tag_info;
extern float g_cf_curve;
extern int g_filter_on[MAX_TRACKS];

#endif /* DJCMD_SHARED_H */
