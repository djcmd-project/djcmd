#include "djcmd_midi.h"
#include "djcmd_ui.h"
#include "djcmd_audio.h"
#include "djcmd_library.h"
#include "djcmd_fx.h"
#include "ns7iii_map.h"
#include <alsa/asoundlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <poll.h>

/* ── MIDI Global State ───────────────────────────────────────────────── */
snd_rawmidi_t *g_midi_in = NULL;
snd_rawmidi_t *g_midi_out = NULL;
char g_midi_dev_str[64] = "";
MidiDevice g_midi_devlist[MIDI_MAX_DEVICES];
int g_midi_ndevices = 0;
int g_midi_dev_sel = 0;

MidiBinding g_midi_bindings[MIDI_MAX_BINDINGS];
int g_midi_nbindings = 0;
MidiOutBinding g_midi_out_bindings[MIDI_MAX_OUT_BINDINGS];
int g_midi_nout_bindings = 0;

uint8_t g_pitch_lsb[MAX_TRACKS] = { 0 };
uint8_t g_vol_lsb[MAX_TRACKS] = { 0 };
uint8_t g_eq_low_lsb[MAX_TRACKS] = { 0 };
uint8_t g_eq_mid_lsb[MAX_TRACKS] = { 0 };
uint8_t g_eq_high_lsb[MAX_TRACKS] = { 0 };
uint8_t g_gain_lsb[MAX_TRACKS] = { 0 };
uint8_t g_filter_lsb[MAX_TRACKS] = { 0 };
uint8_t g_crossfader_lsb = 0;
uint8_t g_master_vol_lsb = 0;

float g_eq_low_knob[MAX_TRACKS] = { 0 };
float g_eq_mid_knob[MAX_TRACKS] = { 0 };
float g_eq_high_knob[MAX_TRACKS] = { 0 };
int g_eq_low_kill[MAX_TRACKS] = { 0 };
int g_eq_mid_kill[MAX_TRACKS] = { 0 };
int g_eq_high_kill[MAX_TRACKS] = { 0 };

int g_pad_mode[MAX_TRACKS] = { 0 };
int g_pad_shift[MAX_TRACKS] = { 0 };
float g_autoloop_bars[MAX_TRACKS] = { 1.0f, 1.0f, 1.0f, 1.0f };
uint32_t g_roll_resume_pos[MAX_TRACKS] = { 0 };
int g_roll_active[MAX_TRACKS] = { 0 };

static pthread_mutex_t g_midi_out_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Human-readable name for each action */
const char *g_mact_names[MACT_COUNT] = { "none",
					 "vol_a",
					 "vol_b",
					 "vol_c",
					 "vol_d",
					 "pitch_a",
					 "pitch_b",
					 "pitch_c",
					 "pitch_d",
					 "pitch_lsb_a",
					 "pitch_lsb_b",
					 "pitch_lsb_c",
					 "pitch_lsb_d",
					 "vol_lsb_a",
					 "vol_lsb_b",
					 "vol_lsb_c",
					 "vol_lsb_d",
					 "eq_low_lsb_a",
					 "eq_low_lsb_b",
					 "eq_low_lsb_c",
					 "eq_low_lsb_d",
					 "eq_mid_lsb_a",
					 "eq_mid_lsb_b",
					 "eq_mid_lsb_c",
					 "eq_mid_lsb_d",
					 "eq_high_lsb_a",
					 "eq_high_lsb_b",
					 "eq_high_lsb_c",
					 "eq_high_lsb_d",
					 "gain_lsb_a",
					 "gain_lsb_b",
					 "gain_lsb_c",
					 "gain_lsb_d",
					 "filter_lsb_a",
					 "filter_lsb_b",
					 "filter_lsb_c",
					 "filter_lsb_d",
					 "crossfader_lsb",
					 "master_vol_lsb",
					 "eq_low_a",
					 "eq_low_b",
					 "eq_low_c",
					 "eq_low_d",
					 "eq_mid_a",
					 "eq_mid_b",
					 "eq_mid_c",
					 "eq_mid_d",
					 "eq_high_a",
					 "eq_high_b",
					 "eq_high_c",
					 "eq_high_d",
					 "gain_a",
					 "gain_b",
					 "gain_c",
					 "gain_d",
					 "filter_a",
					 "filter_b",
					 "filter_c",
					 "filter_d",
					 "filter_toggle_a",
					 "filter_toggle_b",
					 "filter_toggle_c",
					 "filter_toggle_d",
					 "crossfader",
					 "master_vol",
					 "booth_vol",
					 "play_a",
					 "play_b",
					 "play_c",
					 "play_d",
					 "cue_active_a",
					 "cue_active_b",
					 "cue_active_c",
					 "cue_active_d",
					 "cue_set_1",
					 "cue_set_2",
					 "cue_set_3",
					 "cue_set_4",
					 "cue_1",
					 "cue_2",
					 "cue_3",
					 "cue_4",
					 "cue_del_1",
					 "cue_del_2",
					 "cue_del_3",
					 "cue_del_4",
					 "sync_a",
					 "sync_b",
					 "sync_c",
					 "sync_d",
					 "nudge_fwd",
					 "nudge_back",
					 "nudge_fwd_b",
					 "nudge_back_b",
					 "loop",
					 "loop_in_a",
					 "loop_in_b",
					 "loop_in_c",
					 "loop_in_d",
					 "loop_out_a",
					 "loop_out_b",
					 "loop_out_c",
					 "loop_out_d",
					 "loop_double_a",
					 "loop_double_b",
					 "loop_double_c",
					 "loop_double_d",
					 "loop_half_a",
					 "loop_half_b",
					 "loop_half_c",
					 "loop_half_d",
					 "key_lock_a",
					 "key_lock_b",
					 "key_lock_c",
					 "key_lock_d",
					 "slip_a",
					 "slip_b",
					 "slip_c",
					 "slip_d",
					 "reverse_a",
					 "reverse_b",
					 "reverse_c",
					 "reverse_d",
					 "censor_a",
					 "censor_b",
					 "censor_c",
					 "censor_d",
					 "strip_a",
					 "strip_b",
					 "strip_c",
					 "strip_d",
					 "jog_touch_a",
					 "jog_touch_b",
					 "jog_touch_c",
					 "jog_touch_d",
					 "jog_spin_a",
					 "jog_spin_b",
					 "jog_spin_c",
					 "jog_spin_d",
					 "jog_pb_a",
					 "jog_pb_b",
					 "jog_pb_c",
					 "jog_pb_d",
					 "lib_encoder",
					 "lib_encoder_touch",
					 "lib_select",
					 "lib_back",
					 "lib_fwd",
					 "lib_load_a",
					 "lib_load_b",
					 "lib_load_c",
					 "lib_load_d",
					 "panel_files",
					 "panel_library",
					 "pitch_range_a",
					 "pitch_range_b",
					 "pitch_range_c",
					 "pitch_range_d",
					 "pitch_bend_a",
					 "pitch_bend_b",
					 "pitch_bend_c",
					 "pitch_bend_d",
					 "motor_toggle_a",
					 "motor_toggle_b",
					 "motor_toggle_c",
					 "motor_toggle_d",
					 "motor_on_a",
					 "motor_on_b",
					 "motor_on_c",
					 "motor_on_d",
					 "motor_off_a",
					 "motor_off_b",
					 "motor_off_c",
					 "motor_off_d",
					 "deck_sel_1",
					 "deck_sel_2",
					 "deck_sel_3",
					 "deck_sel_4",
					 "shift_a",
					 "shift_b",
					 "pitch_center_a",
					 "pitch_center_b",
					 "cue_default_a",
					 "cue_default_b",
					 "pad_mode_cues_a",
					 "pad_mode_cues_b",
					 "pad_mode_autoroll_a",
					 "pad_mode_autoroll_b",
					 "pad_mode_manual_a",
					 "pad_mode_manual_b",
					 "pad_mode_sampler_a",
					 "pad_mode_sampler_b",
					 "pad_mode_slicer_a",
					 "pad_mode_slicer_b",
					 "pad_1_a",
					 "pad_2_a",
					 "pad_3_a",
					 "pad_4_a",
					 "pad_5_a",
					 "pad_6_a",
					 "pad_7_a",
					 "pad_8_a",
					 "pad_1_b",
					 "pad_2_b",
					 "pad_3_b",
					 "pad_4_b",
					 "pad_5_b",
					 "pad_6_b",
					 "pad_7_b",
					 "pad_8_b",
					 "param_left_a",
					 "param_right_a",
					 "param_left_b",
					 "param_right_b",
					 "fx_btn_1_a",
					 "fx_btn_2_a",
					 "fx_btn_3_a",
					 "fx_btn_1_b",
					 "fx_btn_2_b",
					 "fx_btn_3_b",
					 "fx_knob_1_a",
					 "fx_knob_2_a",
					 "fx_knob_3_a",
					 "fx_knob_1_b",
					 "fx_knob_2_b",
					 "fx_knob_3_b",
					 "fx_wet_a",
					 "fx_wet_b",
					 "cf_curve",
					 "tap_bpm_a",
					 "tap_bpm_b",
					 "grid_snap_a",
					 "grid_snap_b",
					 "eq_low_kill_a",
					 "eq_low_kill_b",
					 "eq_low_kill_c",
					 "eq_low_kill_d",
					 "eq_mid_kill_a",
					 "eq_mid_kill_b",
					 "eq_mid_kill_c",
					 "eq_mid_kill_d",
					 "eq_high_kill_a",
					 "eq_high_kill_b",
					 "eq_high_kill_c",
					 "eq_high_kill_d",
					 "filter_roll_touch_a",
					 "filter_roll_touch_b",
					 "filter_roll_touch_c",
					 "filter_roll_touch_d",
					 "touch_mode",
					 "deck_pitch_a",
					 "deck_pitch_b",
					 "deck_pitch_c",
					 "deck_pitch_d" };

/* MIDI Monitor state */
MidiMonEntry g_midi_mon_buf[MIDI_MON_SIZE];
int g_midi_mon_head = 0;
int g_midi_mon_count = 0;
int g_midi_mon_open = 0;

/* MIDI Learn state */
_Atomic int g_midi_learn_active = 0;
_Atomic int g_midi_learn_sel = 0;
_Atomic int g_midi_learn_jog_pair = 0;
_Atomic int g_midi_learn_jog_step = 0;
_Atomic int g_midi_learn_jog_deck = 0;
_Atomic uint8_t g_midi_learn_jog_spin_status = 0;
_Atomic uint8_t g_midi_learn_jog_spin_d1 = 0;

/* Motor Probe state */
int g_motor_probe_ch = 1;
int g_motor_probe_cc = 65;
int g_motor_probe_val = 0;
int g_motor_probe_type = 0;
char g_motor_probe_log[256] = "Press M in MIDI tab to open motor probe";
int g_motor_probe_open = 0;

/* Jog Wheel / Scratch state */
JogType g_jog_type = JOG_RELATIVE;
float g_jog_abs_pos[MAX_TRACKS] = { 0 };
_Atomic float g_jog_abs_vel[MAX_TRACKS];
int g_pb_streak[MAX_TRACKS] = { 0 };
int g_motor_running[MAX_TRACKS] = { 0 };

_Atomic float g_motor_vel[MAX_TRACKS];
_Atomic float g_jog_nudge[MAX_TRACKS];
float g_last_motor_vel[MAX_TRACKS] = { 0 };
float g_last_applied_nudge[MAX_TRACKS] = { 0 };
_Atomic int g_jog_touched[MAX_TRACKS];
float g_scratch_alpha[MAX_TRACKS] = { 0 };
uint32_t g_noise_state[MAX_TRACKS] = { 12345u, 67890u, 11111u, 22222u };
float g_noise_brown_l[MAX_TRACKS] = { 0.0f };
float g_noise_brown_r[MAX_TRACKS] = { 0.0f };
_Atomic int g_motor_pending_start[MAX_TRACKS];
int64_t g_motor_settle_until[MAX_TRACKS] = { 0 };

/* ── Internal (static) MIDI-related state ── */
static uint8_t g_midi_last_status = 0;
static uint8_t g_midi_last_d1 = 0;
static uint8_t g_midi_last_d2 = 0;

float g_jog_ref_delta = 0.0078125f;
float g_jog_motor_dead = 0.05f;
float g_jog_vel_max = 15.0f;
float g_jog_scratch_revs = 79380.0f;

int64_t g_jog_last_msg_ms[MAX_TRACKS] = { 0 };

float g_scratch_lpf_l[MAX_TRACKS] = { 0 };
float g_scratch_lpf_r[MAX_TRACKS] = { 0 };
float g_scratch_lpf2_l[MAX_TRACKS] = { 0 };
float g_scratch_lpf2_r[MAX_TRACKS] = { 0 };

static int g_jog_coarse[MAX_TRACKS] = { -1, -1, -1, -1 };
static int g_jog_fine[MAX_TRACKS] = { 8192, 8192, 8192, 8192 };
static int g_jog_last_pb[MAX_TRACKS] = { 8192, 8192, 8192, 8192 };
static int g_jog_release_conf[MAX_TRACKS] = { 0 };
static int g_jog_last_coarse_d[MAX_TRACKS] = { 0 };
static int64_t g_jog_last_spin_ms[MAX_TRACKS] = { 0 };
static int64_t g_jog_last_update_ms[MAX_TRACKS] = { 0 };
static int g_jog_abs_init[MAX_TRACKS] = { 0 };

static volatile int g_motor_ramp_step[MAX_TRACKS] = { 0 };

#define MOTOR_RAMP_CC 73
#define MOTOR_PITCH_CC 105
#define MOTOR_SAW_CC 74
#define MOTOR_ENABLE_CC 75
#define MOTOR_RAMP_STEPS CFG_MOTOR_RAMP_STEPS
#define MOTOR_SAW_MAX 5

static int g_motor_ramp_table[64];
static int g_motor_saw_phase = 0;

static float g_pb_mag_acc[MAX_TRACKS] = { 0.0f };

static float g_jog_smooth_alpha = 0.30f;
static float g_jog_dead_band = 0.08f;
static float g_jog_spike_thresh = 0.80f;
static float g_jog_slew_rate = 0.04f;
static int g_jog_settle_ms = 2000;

static int g_pll_enabled = 0;
static float g_pll_kp = 0.10f;
static float g_pll_ki = 0.003f;
static float g_pll_bandwidth = 0.30f;
static PLLState g_pll[MAX_TRACKS];

#define PAD_NOTE_BASE 0x47
#define PAD_COL_OFF 0x00
#define PAD_COL_AUTOLOOP_BG 0x04
#define PAD_COL_AUTOLOOP_ON 0x14
#define PAD_COL_ROLL_BG 0x19
#define PAD_COL_ROLL_ON 0x1F
#define PAD_COL_MANUALLOOP_BG 0x08
#define PAD_COL_MANUALLOOP_SET 0x14
#define PAD_COL_CUE_UNSET 0x00
#define PAD_COL_CUE_1 0x10
#define PAD_COL_CUE_2 0x20
#define PAD_COL_CUE_3 0x25
#define PAD_COL_CUE_4 0x14
#define PAD_COL_CUE_5 0x04
#define PAD_COL_CUE_6 0x30
#define PAD_COL_CUE_7 0x08
#define PAD_COL_CUE_8 0x7F

static const uint8_t g_cue_colours[8] = { PAD_COL_CUE_1, PAD_COL_CUE_2,
					  PAD_COL_CUE_3, PAD_COL_CUE_4,
					  PAD_COL_CUE_5, PAD_COL_CUE_6,
					  PAD_COL_CUE_7, PAD_COL_CUE_8 };

/* Pitch range per deck -- cycles: 0=±8%, 1=±25%, 2=±50%
 * This controls how far the MIDI pitch fader spans from center.
 * Keyboard pitch keys (e/d/E/D) are unaffected and still step freely. */
int g_pitch_range[MAX_TRACKS] = { 0, 0, 0, 0 };
/* Pitch range values as multipliers from center (1.0).
 * Center = 1.0, range ±8% = [0.92, 1.08], ±25% = [0.75, 1.25], ±50% = [0.50, 1.50] */
float g_pitch_range_vals[3] = { 0.08f, 0.25f, 0.50f };
const char *g_pitch_range_names[3] = { "±8%", "±25%", "±50%" };

#define VAL14(msb, lsb) (((float)(((int)(msb) << 7) | (int)(lsb))) / 16383.0f)

/* ── Externs needed from other modules ── */
extern int g_side_deck[2];
extern Track g_tracks[MAX_TRACKS];
extern _Atomic int g_running;
extern int g_active_track;
extern int g_gang_mode;
extern _Atomic int g_master_vol;
extern Options g_opts;
extern _Atomic float g_vu_l, g_vu_r;
extern int g_num_tracks;
extern unsigned int g_actual_sample_rate;
extern int g_cue_default_set[MAX_TRACKS];
extern uint32_t g_cue_default_pos[MAX_TRACKS];
extern int g_cue_default_held[MAX_TRACKS];
extern int g_pad_mode[MAX_TRACKS];
extern int g_pad_shift[MAX_TRACKS];
extern float g_autoloop_bars[MAX_TRACKS];
extern int g_roll_active[MAX_TRACKS];
extern uint8_t g_pitch_lsb[MAX_TRACKS];
extern uint8_t g_vol_lsb[MAX_TRACKS];
extern uint8_t g_eq_low_lsb[MAX_TRACKS];
extern uint8_t g_eq_mid_lsb[MAX_TRACKS];
extern uint8_t g_eq_high_lsb[MAX_TRACKS];
extern uint8_t g_gain_lsb[MAX_TRACKS];
extern uint8_t g_filter_lsb[MAX_TRACKS];
extern uint8_t g_crossfader_lsb;
extern uint8_t g_master_vol_lsb;
extern float g_eq_low_knob[MAX_TRACKS];
extern float g_eq_mid_knob[MAX_TRACKS];
extern float g_eq_high_knob[MAX_TRACKS];
extern int g_eq_low_kill[MAX_TRACKS];
extern int g_eq_mid_kill[MAX_TRACKS];
extern int g_eq_high_kill[MAX_TRACKS];
extern float g_pitch_range_vals[3];
extern const char *g_pitch_range_names[3];
extern int g_pitch_range[MAX_TRACKS];
extern int g_slip_motor_off[MAX_TRACKS];
extern _Atomic float g_crossfader;
extern float g_cf_curve;
extern _Atomic float g_autoplay_xf_target;
extern char g_fb_status[256];
extern int g_view;
extern int g_panel;
extern int g_fb_sel;
extern int g_fb_count;
extern FBEntry g_fb_entries[FB_MAX_ENTRIES];
extern int g_pl_sel;
extern int g_pl_count;
extern PLEntry g_pl[PL_MAX];
extern LIBEntry *g_lib;
extern int g_lib_count;
extern int g_lib_sel;
extern int64_t g_lib_enc_last_ms;
extern int g_lib_auto_switched;
extern int g_lib_touched;
extern int g_filter_on[MAX_TRACKS];
extern int g_filter_was_on[MAX_TRACKS];
extern int g_filter_roll_held[MAX_TRACKS];
extern int g_touch_mode;
extern uint32_t g_censor_save_pos[MAX_TRACKS];
extern int g_censor_held[MAX_TRACKS];

/* Prototypes for functions still in djcmd.c or elsewhere */
void enqueue_load(int deck, const char *path);
void fb_enter_dir(const char *name);
void fb_selected_path(char *out, size_t max);
void side_restack(int side, int new_dk);
void settings_save(void);
void roll_begin(int deck, int pad_idx, float bars);
void roll_end(int deck);

/* ── Internal Prototypes ── */
static void midi_map_load(void);
static void midi_map_write_generic_defaults(void);
static MidiOutBinding *midi_out_lookup(const char *name);
static void midi_out_bind(const char *name, uint8_t status, uint8_t data1,
			  uint8_t data2);
static void midi_out_bind_rgb(const char *name, uint8_t pad, uint8_t r,
			      uint8_t g, uint8_t b);
static void midi_out_send(const char *name);
static void midi_send_cc(int channel, int cc, int value);
static void midi_send_note(int channel, int note, int velocity);
static MidiAction midi_lookup(uint8_t status, uint8_t data1);
static MidiAction midi_action_from_name(const char *name);
static void pad_led(int midi_ch, int pad_idx_1, uint8_t colour);
static void motor_sync_pitch(int deck);
static void ns7iii_update_jog(int deck, int coarse, int fine);
static void *motor_thread(void *arg);
static uint32_t bars_to_frames(int deck, float bars);
static uint32_t snap_to_beat(int deck, uint32_t pos);

/* ── Implementations ── */

/* Compute loop length in frames from bar count and track BPM.
 * bars: 1.0 / 2.0 / 4.0 / 8.0 etc.
 * Returns 0 if BPM is not set. */
static uint32_t bars_to_frames(int deck, float bars)
{
	float bpm = g_tracks[deck].bpm;
	if (bpm <= 0.0f)
		return 0;
	float beat_frames = (float)g_actual_sample_rate * 60.0f / bpm;
	float beats_per_bar = 4.0f; /* 4/4 time */
	uint32_t len = (uint32_t)(bars * beats_per_bar * beat_frames + 0.5f);
	return len;
}

/* Snap start position to nearest beat boundary for clean loops */
static uint32_t snap_to_beat(int deck, uint32_t pos)
{
	Track *t = &g_tracks[deck];
	if (t->bpm <= 0.0f || t->num_frames == 0)
		return pos;
	float beat_frames = (float)g_actual_sample_rate * 60.0f / t->bpm;
	if (beat_frames <= 0.0f)
		return pos;
	/* Find nearest beat: bpm_offset + N * beat_frames */
	float offset = t->bpm_offset;
	if (offset < 0.0f)
		offset = 0.0f;
	float beats_from_offset = ((float)pos - offset) / beat_frames;
	float nearest_beat = roundf(beats_from_offset) * beat_frames + offset;
	if (nearest_beat < 0.0f)
		nearest_beat = 0.0f;
	if (nearest_beat >= (float)t->num_frames)
		nearest_beat = (float)(t->num_frames - 1);
	return (uint32_t)nearest_beat;
}

/* Engage a beat-aligned auto-loop of 'bars' bars on deck.
 * If loop is already active at this size, toggle it off. */
void autoloop_engage(int deck, float bars)
{
	Track *t = &g_tracks[deck];
	if (!t->loaded || t->bpm <= 0.0f)
		return;

	uint32_t len = bars_to_frames(deck, bars);
	if (len == 0)
		return;

	pthread_mutex_lock(&t->lock);

	/* If already looping at exactly this size, toggle off */
	uint32_t cur_len =
		(t->loop_end > t->loop_start) ? t->loop_end - t->loop_start : 0;
	if (t->looping && cur_len == len) {
		t->looping = 0;
		pthread_mutex_unlock(&t->lock);
		g_autoloop_bars[deck] = bars;
		pad_leds_refresh(deck);
		return;
	}

	/* Snap start to nearest beat */
	uint32_t start = snap_to_beat(deck, t->pos);
	if (start + len > t->num_frames)
		start = (t->num_frames > len) ? t->num_frames - len : 0;

	t->loop_start = start;
	t->loop_end = start + len;
	t->looping = 1;

	pthread_mutex_unlock(&t->lock);

	g_autoloop_bars[deck] = bars;
	pad_leds_refresh(deck);
	deck_leds_refresh();
}

/* Begin a roll on deck: save current position, start the loop.
 * Resume position is where playback was before the roll. */
void roll_begin(int deck, int pad_idx, float bars)
{
	Track *t = &g_tracks[deck];
	if (!t->loaded || t->bpm <= 0.0f)
		return;

	uint32_t len = bars_to_frames(deck, bars);
	if (len == 0)
		return;

	/* Save position before entering loop */
	pthread_mutex_lock(&t->lock);
	g_roll_resume_pos[deck] = t->pos;
	uint32_t start = snap_to_beat(deck, t->pos);
	if (start + len > t->num_frames)
		start = (t->num_frames > len) ? t->num_frames - len : 0;
	t->loop_start = start;
	t->loop_end = start + len;
	t->looping = 1;
	t->pos = start;
	pthread_mutex_unlock(&t->lock);

	g_roll_active[deck] = pad_idx;
	g_autoloop_bars[deck] = bars;
	pad_leds_refresh(deck);
}

/* Release a roll: resume from saved position, stop looping */
void roll_end(int deck)
{
	if (!g_roll_active[deck])
		return;
	Track *t = &g_tracks[deck];
	pthread_mutex_lock(&t->lock);
	t->looping = 0;
	t->pos = g_roll_resume_pos[deck];
	pthread_mutex_unlock(&t->lock);
	g_roll_active[deck] = 0;
	pad_leds_refresh(deck);
}

void midi_map_path(char *out, size_t max)
{
	const char *home = getenv("HOME");
	if (!home)
		home = "/tmp";

	/* Find the human name for the currently open device */
	const char *dev_name = "";
	for (int i = 0; i < g_midi_ndevices; i++) {
		if (strcmp(g_midi_devlist[i].dev, g_midi_dev_str) == 0) {
			dev_name = g_midi_devlist[i].name;
			break;
		}
	}

	/* Build sanitised filename, fall back to "midi" if no device is open */
	char san[128];
	if (dev_name[0])
		midi_map_name_from_device(dev_name, san, sizeof(san));
	else
		snprintf(san, sizeof(san), "midi");

	snprintf(out, max, "%s/" CFG_CONFIG_DIR "/%s.map", home, san);
}

void midi_open_device(int dev_idx)
{
	if (dev_idx < 0 || dev_idx >= g_midi_ndevices)
		return;

	/* 1+2: stop MIDI thread reads, close old handles */
	snd_rawmidi_t *old_in = g_midi_in;
	snd_rawmidi_t *old_out = g_midi_out;
	g_midi_in = NULL;
	g_midi_out = NULL;
	usleep(5000);
	if (old_in)
		snd_rawmidi_close(old_in);
	if (old_out)
		snd_rawmidi_close(old_out);

	/* 3: update device tracking */
	g_midi_dev_sel = dev_idx;
	snprintf(g_midi_dev_str, sizeof(g_midi_dev_str), "%s",
		 g_midi_devlist[dev_idx].dev);

	/* 4: open input + output.  Output may not be available on all devices
     * (e.g. a receive-only interface) -- failure is non-fatal. */
	snd_rawmidi_open(&g_midi_in, &g_midi_out, g_midi_dev_str,
			 SND_RAWMIDI_NONBLOCK);

	/* 5: clear bindings, load per-device map */
	g_midi_nbindings = 0;
	g_midi_nout_bindings = 0;
	midi_map_load();
	/* If no map existed, write a generic starter map for this device */
	if (g_midi_nbindings == 0 && g_midi_nout_bindings == 0)
		midi_map_write_generic_defaults();

	/* Migration: fix old motor_stop entries that used CC65/val0 (wrong).
     * Correct values are CC66/val127.  Silently patch and resave. */
	{
		int migrated = 0;
		const char *stop_names[] = { "motor_stop_a", "motor_stop_b",
					     "motor_stop_c", "motor_stop_d" };
		/* Correct start status bytes for each deck */
		const uint8_t start_st[] = { 0xB1, 0xB2, 0xB3, 0xB4 };
		for (int di = 0; di < 4; di++) {
			MidiOutBinding *b = midi_out_lookup(stop_names[di]);
			if (b && b->data1 == 0x41 && b->data2 == 0) {
				/* Old wrong entry: same CC as start, val0 -- fix to CC66 val127 */
				b->data1 = 0x42; /* CC66 */
				b->data2 = 127;
				if (!b->status)
					b->status = start_st[di];
				migrated = 1;
			}
		}
		if (migrated)
			midi_map_save();
	}

	/* 6: persist the choice */
	settings_save();

	snprintf(g_fb_status, sizeof(g_fb_status), "MIDI: %s%s",
		 g_midi_devlist[dev_idx].name,
		 g_midi_out ? " (in+out)" : " (in only)");
	/* Refresh all LEDs for all connected decks */
	deck_leds_refresh();
	for (int _di = 0; _di < g_num_tracks; _di++) {
		pad_mode_leds_refresh(_di);
		pad_leds_refresh(_di);
		fx_leds_refresh(_di);
	}

	/* Send initial deck select LED state -- active decks lit, inactive dim */
	if (g_midi_out) {
		/* Default: left=deck1, right=deck2 */
		led_on("led_deck_1");
		led_off("led_deck_3");
		led_on("led_deck_2");
		led_off("led_deck_4");
		/* Reflect current side routing if it differs from default */
		if (g_side_deck[0] == 2) {
			led_on("led_deck_3");
			led_off("led_deck_1");
		}
		if (g_side_deck[1] == 3) {
			led_on("led_deck_4");
			led_off("led_deck_2");
		}
	}
}

void deck_leds_refresh(void)
{
	static int last_sel_lit[4] = { -1, -1, -1, -1 };
	static int last_play[2] = { -1, -1 };
	static int last_cue[2] = { -1, -1 };
	static int last_loop[2] = { -1, -1 };

	static const char *deck_led_names[4] = { "led_deck_1", "led_deck_2",
						 "led_deck_3", "led_deck_4" };
	static const char *play_leds[2] = { "led_deck_a", "led_deck_b" };
	static const char *cue_leds[2] = { "led_cue_default_a",
					   "led_cue_default_b" };
	static const char *loop_leds[2] = { "led_loop_a", "led_loop_b" };

	for (int i = 0; i < 4; i++) {
		int active = (i == g_side_deck[0] || i == g_side_deck[1]);
		if (active != last_sel_lit[i]) {
			if (active)
				led_on(deck_led_names[i]);
			else
				led_off(deck_led_names[i]);
			last_sel_lit[i] = active;
		}
	}

	for (int side = 0; side < 2; side++) {
		int dk = g_side_deck[side];
		int want_cue = g_cue_default_set[dk] ? 1 : 0;
		if (want_cue != last_cue[side]) {
			if (want_cue)
				led_on(cue_leds[side]);
			else
				led_off(cue_leds[side]);
			last_cue[side] = want_cue;
		}
		int want_loop = g_tracks[dk].looping ? 1 : 0;
		if (want_loop != last_loop[side]) {
			if (want_loop)
				led_on(loop_leds[side]);
			else
				led_off(loop_leds[side]);
			last_loop[side] = want_loop;
		}
		Track *t = &g_tracks[dk];
		int want_play;
		if (!t->loaded) {
			want_play = 0;
		} else if (t->playing) {
			want_play = 1;
		} else {
			int blink_period =
				(g_opts.ui_fps > 0) ? g_opts.ui_fps / 2 : 10;
			if (blink_period < 1)
				blink_period = 1;
			want_play = ((g_blink_tick / blink_period) % 2 == 0) ?
					    2 :
					    3;
		}
		if (want_play != last_play[side]) {
			if (want_play == 0 || want_play == 3)
				led_off(play_leds[side]);
			else
				led_on(play_leds[side]);
			last_play[side] = want_play;
		}
	}
}

void fx_leds_refresh(int deck)
{
	char side = (deck == g_side_deck[0]) ? 'a' : 'b';
	char n[32];
	for (int sl = 0; sl < FX_SLOTS_PER_DECK; sl++) {
		FXSlot *fx = fx_slot(deck, sl);
		int active = (fx->pending_type >= 0) ?
				     (fx->pending_type != FX_NONE) :
				     (fx->type != FX_NONE);
		snprintf(n, sizeof(n), "led_fx_btn_%d_%c", sl + 1, side);
		if (active)
			led_on(n);
		else
			led_off(n);
	}
}

static void pad_led(int midi_ch, int pad_idx_1, uint8_t colour)
{
	if (!g_midi_out)
		return;
	int note = PAD_NOTE_BASE + pad_idx_1 - 1;
	uint8_t msg[3];
	msg[0] = (uint8_t)(0x90 | ((midi_ch - 1) & 0x0F));
	msg[1] = (uint8_t)(note & 0x7F);
	msg[2] = colour & 0x7F;
	pthread_mutex_lock(&g_midi_out_mutex);
	snd_rawmidi_write(g_midi_out, msg, 3);
	snd_rawmidi_drain(g_midi_out);
	pthread_mutex_unlock(&g_midi_out_mutex);
}

void pad_leds_refresh(int deck)
{
	if (!g_midi_out)
		return;
	int midi_ch = (deck == g_side_deck[0]) ? 2 : 3;
	int mode = g_pad_mode[deck];
	Track *t = &g_tracks[deck];

	if (mode == PAD_MODE_HOTCUE) {
		for (int i = 1; i <= 8; i++) {
			int ci = i - 1;
			uint8_t col = (ci < MAX_CUES && t->cue_set[ci]) ?
					      g_cue_colours[ci] :
					      PAD_COL_CUE_UNSET;
			pad_led(midi_ch, i, col);
		}
	} else if (mode == PAD_MODE_AUTOLOOP) {
		static const float sizes[4] = { 1.0f, 2.0f, 4.0f, 8.0f };
		float cur_bars = g_autoloop_bars[deck];
		for (int i = 1; i <= 4; i++) {
			uint8_t col = (sizes[i - 1] == cur_bars && t->looping) ?
					      PAD_COL_AUTOLOOP_ON :
					      PAD_COL_AUTOLOOP_BG;
			pad_led(midi_ch, i, col);
		}
		for (int i = 5; i <= 8; i++)
			pad_led(midi_ch, i, PAD_COL_OFF);
	} else if (mode == PAD_MODE_ROLL) {
		static const float sizes[4] = { 1.0f, 2.0f, 4.0f, 8.0f };
		float cur_bars = g_autoloop_bars[deck];
		for (int i = 1; i <= 4; i++) {
			uint8_t col = (sizes[i - 1] == cur_bars &&
				       g_roll_active[deck] == i) ?
					      PAD_COL_ROLL_ON :
					      PAD_COL_ROLL_BG;
			pad_led(midi_ch, i, col);
		}
		for (int i = 5; i <= 8; i++)
			pad_led(midi_ch, i, PAD_COL_OFF);
	} else if (mode == PAD_MODE_MANUALLOOP) {
		Track *tl = &g_tracks[deck];
		pad_led(midi_ch, 1,
			(tl->loop_start > 0) ? PAD_COL_MANUALLOOP_SET :
					       PAD_COL_MANUALLOOP_BG);
		pad_led(midi_ch, 2,
			tl->looping ? PAD_COL_MANUALLOOP_SET :
				      PAD_COL_MANUALLOOP_BG);
		pad_led(midi_ch, 3, PAD_COL_MANUALLOOP_BG);
		pad_led(midi_ch, 4, PAD_COL_MANUALLOOP_BG);
		pad_led(midi_ch, 5,
			tl->looping ? PAD_COL_MANUALLOOP_SET :
				      PAD_COL_MANUALLOOP_BG);
		for (int i = 6; i <= 8; i++)
			pad_led(midi_ch, i, PAD_COL_OFF);
	}
}

void pad_mode_leds_refresh(int deck)
{
	char side = (deck == g_side_deck[0]) ? 'a' : 'b';
	int mode = g_pad_mode[deck];
	char n[32];
	snprintf(n, sizeof(n), "led_pad_hotcue_%c", side);
	led_off(n);
	snprintf(n, sizeof(n), "led_pad_roll_%c", side);
	led_off(n);
	snprintf(n, sizeof(n), "led_pad_loop_%c", side);
	led_off(n);
	snprintf(n, sizeof(n), "led_pad_sampler_%c", side);
	led_off(n);
	snprintf(n, sizeof(n), "led_pad_slicer_%c", side);
	led_off(n);

	if (mode == PAD_MODE_HOTCUE) {
		snprintf(n, sizeof(n), "led_pad_hotcue_%c", side);
		led_on(n);
	} else if (mode == PAD_MODE_AUTOLOOP) {
		snprintf(n, sizeof(n), "led_pad_loop_%c", side);
		led_on(n);
	} else if (mode == PAD_MODE_ROLL) {
		snprintf(n, sizeof(n), "led_pad_roll_%c", side);
		led_on(n);
	} else if (mode == PAD_MODE_MANUALLOOP) {
		snprintf(n, sizeof(n), "led_pad_loop_%c", side);
		led_on(n);
	}
}

void led_on(const char *name)
{
	midi_out_send(name);
}

void led_off(const char *name)
{
	if (!g_midi_out)
		return;
	MidiOutBinding *b = midi_out_lookup(name);
	if (!b || b->status == 0)
		return;
	if (b->sysex_len > 0) {
		uint8_t off[11];
		memcpy(off, b->sysex, b->sysex_len);
		off[7] = off[8] = off[9] = 0;
		pthread_mutex_lock(&g_midi_out_mutex);
		snd_rawmidi_write(g_midi_out, off, b->sysex_len);
		snd_rawmidi_drain(g_midi_out);
		pthread_mutex_unlock(&g_midi_out_mutex);
	} else {
		uint8_t msg[3] = { (uint8_t)((b->status & 0xF0) == 0x90 ?
						     b->status :
						     0x90),
				   b->data1, 0 };
		pthread_mutex_lock(&g_midi_out_mutex);
		snd_rawmidi_write(g_midi_out, msg, 3);
		snd_rawmidi_drain(g_midi_out);
		pthread_mutex_unlock(&g_midi_out_mutex);
	}
}

static MidiOutBinding *midi_out_lookup(const char *name)
{
	for (int i = 0; i < g_midi_nout_bindings; i++)
		if (strcmp(g_midi_out_bindings[i].name, name) == 0)
			return &g_midi_out_bindings[i];
	return NULL;
}

static void midi_out_bind(const char *name, uint8_t status, uint8_t data1,
			  uint8_t data2)
{
	for (int i = 0; i < g_midi_nout_bindings; i++) {
		if (strcmp(g_midi_out_bindings[i].name, name) == 0) {
			g_midi_out_bindings[i].status = status;
			g_midi_out_bindings[i].data1 = data1;
			g_midi_out_bindings[i].data2 = data2;
			return;
		}
	}
	if (g_midi_nout_bindings >= MIDI_MAX_OUT_BINDINGS)
		return;
	MidiOutBinding *b = &g_midi_out_bindings[g_midi_nout_bindings++];
	snprintf(b->name, sizeof(b->name), "%s", name);
	b->status = status;
	b->data1 = data1;
	b->data2 = data2;
}

static void midi_out_bind_rgb(const char *name, uint8_t pad, uint8_t r,
			      uint8_t g, uint8_t b)
{
	if (pad > 127 || g_midi_nout_bindings >= MIDI_MAX_OUT_BINDINGS)
		return;
	MidiOutBinding *entry = midi_out_lookup(name);
	if (!entry) {
		entry = &g_midi_out_bindings[g_midi_nout_bindings++];
		snprintf(entry->name, sizeof(entry->name), "%s", name);
	}
	entry->status = 0xF0;
	entry->data1 = pad;
	entry->data2 = 0;
	entry->sysex[0] = 0xF0;
	entry->sysex[1] = 0x00;
	entry->sysex[2] = 0x20;
	entry->sysex[3] = 0x7F;
	entry->sysex[4] = 0x03;
	entry->sysex[5] = 0x01;
	entry->sysex[6] = pad;
	entry->sysex[7] = r & 0x7F;
	entry->sysex[8] = g & 0x7F;
	entry->sysex[9] = b & 0x7F;
	entry->sysex[10] = 0xF7;
	entry->sysex_len = 11;
}

static void midi_out_send(const char *name)
{
	if (!g_midi_out)
		return;
	MidiOutBinding *b = midi_out_lookup(name);
	if (!b || b->status == 0)
		return;
	pthread_mutex_lock(&g_midi_out_mutex);
	if (b->sysex_len > 0) {
		snd_rawmidi_write(g_midi_out, b->sysex, b->sysex_len);
	} else {
		uint8_t msg[3] = { b->status, b->data1, b->data2 };
		snd_rawmidi_write(g_midi_out, msg, 3);
	}
	snd_rawmidi_drain(g_midi_out);
	pthread_mutex_unlock(&g_midi_out_mutex);
}

static void midi_send_cc(int channel, int cc, int value)
{
	if (!g_midi_out)
		return;
	uint8_t msg[3];
	msg[0] = (uint8_t)(0xB0 | ((channel - 1) & 0x0F));
	msg[1] = (uint8_t)(cc & 0x7F);
	msg[2] = (uint8_t)(value & 0x7F);
	pthread_mutex_lock(&g_midi_out_mutex);
	snd_rawmidi_write(g_midi_out, msg, 3);
	snd_rawmidi_drain(g_midi_out);
	pthread_mutex_unlock(&g_midi_out_mutex);
}

static void midi_send_note(int channel, int note, int velocity)
{
	if (!g_midi_out)
		return;
	uint8_t msg[3];
	msg[0] = (uint8_t)((velocity ? 0x90 : 0x80) | ((channel - 1) & 0x0F));
	msg[1] = (uint8_t)(note & 0x7F);
	msg[2] = (uint8_t)(velocity & 0x7F);
	pthread_mutex_lock(&g_midi_out_mutex);
	snd_rawmidi_write(g_midi_out, msg, 3);
	snd_rawmidi_drain(g_midi_out);
	pthread_mutex_unlock(&g_midi_out_mutex);
}

void midi_enumerate_devices(void)
{
	snd_ctl_t *ctl;
	snd_rawmidi_info_t *info;
	snd_rawmidi_info_alloca(&info);
	g_midi_ndevices = 0;
	int card = -1;
	while (snd_card_next(&card) == 0 && card >= 0) {
		char ctl_name[32];
		snprintf(ctl_name, sizeof(ctl_name), "hw:%d", card);
		if (snd_ctl_open(&ctl, ctl_name, 0) < 0)
			continue;
		int device = -1;
		while (snd_ctl_rawmidi_next_device(ctl, &device) == 0 &&
		       device >= 0) {
			snd_rawmidi_info_set_device(info, (unsigned)device);
			snd_rawmidi_info_set_subdevice(info, 0);
			snd_rawmidi_info_set_stream(info,
						    SND_RAWMIDI_STREAM_INPUT);
			if (snd_ctl_rawmidi_info(ctl, info) == 0 &&
			    g_midi_ndevices < MIDI_MAX_DEVICES) {
				const char *iname =
					snd_rawmidi_info_get_name(info);
				if (strstr(iname, "Display Right") ||
				    strstr(iname, "Display Left"))
					continue;
				MidiDevice *e =
					&g_midi_devlist[g_midi_ndevices++];
				snprintf(e->dev, sizeof(e->dev), "hw:%d,%d,0",
					 card, device);
				snprintf(e->name, sizeof(e->name), "%s", iname);
			}
		}
		snd_ctl_close(ctl);
	}
}

void midi_map_name_from_device(const char *dev_name, char *out, size_t max)
{
	char san[128] = "";
	int oi = 0;
	for (int i = 0; dev_name[i] && oi < (int)sizeof(san) - 1; i++) {
		unsigned char c = (unsigned char)dev_name[i];
		if (c >= 'A' && c <= 'Z')
			san[oi++] = (char)(c + 32);
		else if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9'))
			san[oi++] = (char)c;
		else if (oi > 0 && san[oi - 1] != '_')
			san[oi++] = '_';
	}
	while (oi > 0 && san[oi - 1] == '_')
		oi--;
	san[oi] = '\0';
	if (oi == 0)
		snprintf(san, sizeof(san), "default");
	snprintf(out, max, "%s", san);
}

static MidiAction midi_action_from_name(const char *name)
{
	for (int i = 1; i < MACT_COUNT; i++)
		if (strcmp(g_mact_names[i], name) == 0)
			return (MidiAction)i;
	return MACT_NONE;
}

void midi_bind(uint8_t status, uint8_t data1, MidiAction action)
{
	if (action <= MACT_NONE || action >= MACT_COUNT)
		return;
	for (int i = 0; i < g_midi_nbindings; i++) {
		if (g_midi_bindings[i].action == action) {
			memmove(&g_midi_bindings[i], &g_midi_bindings[i + 1],
				(g_midi_nbindings - i - 1) *
					sizeof(MidiBinding));
			g_midi_nbindings--;
			break;
		}
	}
	if (status == 0)
		return;
	for (int i = 0; i < g_midi_nbindings; i++) {
		if (g_midi_bindings[i].status == status &&
		    g_midi_bindings[i].data1 == data1) {
			memmove(&g_midi_bindings[i], &g_midi_bindings[i + 1],
				(g_midi_nbindings - i - 1) *
					sizeof(MidiBinding));
			g_midi_nbindings--;
			break;
		}
	}
	if (g_midi_nbindings >= MIDI_MAX_BINDINGS)
		return;
	g_midi_bindings[g_midi_nbindings].status = status;
	g_midi_bindings[g_midi_nbindings].data1 = data1;
	g_midi_bindings[g_midi_nbindings].action = action;
	g_midi_bindings[g_midi_nbindings].relative = 0;
	g_midi_bindings[g_midi_nbindings].rel_acc = 0.5f;
	g_midi_nbindings++;
}

void midi_bind_set_relative(MidiAction action, int rel)
{
	for (int i = 0; i < g_midi_nbindings; i++) {
		if (g_midi_bindings[i].action == action) {
			g_midi_bindings[i].relative = rel ? 1 : 0;
			g_midi_bindings[i].rel_acc = 0.5f;
			return;
		}
	}
}

static MidiAction midi_lookup(uint8_t status, uint8_t data1)
{
	uint8_t try_status = status;
	for (int pass = 0; pass < 2; pass++) {
		for (int i = 0; i < g_midi_nbindings; i++)
			if (g_midi_bindings[i].status == try_status &&
			    g_midi_bindings[i].data1 == data1)
				return g_midi_bindings[i].action;
		if ((status & 0xF0) == 0x80)
			try_status = (status & 0x0F) | 0x90;
		else
			break;
	}
	return MACT_NONE;
}

static void midi_map_load(void)
{
	char path[512];
	midi_map_path(path, sizeof(path));
	FILE *f = fopen(path, "r");
	if (!f)
		return;
	g_midi_nbindings = 0;
	g_midi_nout_bindings = 0;
	char line[256];
	while (fgets(line, sizeof(line), f)) {
		char *hash = strchr(line, '#');
		if (hash)
			*hash = '\0';
		if (strncmp(line, "set ", 4) == 0 ||
		    strncmp(line, "set\t", 4) == 0) {
			char pname[32];
			float pval;
			if (sscanf(line + 4, "%31s %f", pname, &pval) == 2) {
				if (!strcmp(pname, "jog_smooth"))
					g_jog_smooth_alpha = pval;
				else if (!strcmp(pname, "jog_dead"))
					g_jog_dead_band = pval;
				else if (!strcmp(pname, "jog_spike"))
					g_jog_spike_thresh = pval;
				else if (!strcmp(pname, "jog_slew"))
					g_jog_slew_rate = pval;
				else if (!strcmp(pname, "jog_settle"))
					g_jog_settle_ms = (int)pval;
				else if (!strcmp(pname, "jog_scratch_revs"))
					g_jog_scratch_revs = pval;
				else if (!strcmp(pname, "jog_ref_delta"))
					g_jog_ref_delta = pval;
				else if (!strcmp(pname, "jog_motor_dead"))
					g_jog_motor_dead = pval;
				else if (!strcmp(pname, "jog_vel_max"))
					g_jog_vel_max = pval;
				else if (!strcmp(pname, "pll_enabled"))
					g_pll_enabled = (int)pval;
				else if (!strcmp(pname, "pll_kp"))
					g_pll_kp = pval;
				else if (!strcmp(pname, "pll_ki"))
					g_pll_ki = pval;
				else if (!strcmp(pname, "pll_bandwidth"))
					g_pll_bandwidth = pval;
				else if (!strcmp(pname, "jog_msg_rate")) {
					if (pval > 0.0f)
						g_jog_ref_delta =
							(float)g_actual_sample_rate /
							(g_jog_scratch_revs *
							 pval);
				}
			} else {
				char sval[32];
				if (sscanf(line + 4, "%31s %31s", pname,
					   sval) == 2) {
					if (!strcmp(pname, "jog_type")) {
						if (!strcmp(sval, "ns7iii"))
							g_jog_type = JOG_NS7III;
						else if (!strcmp(sval,
								 "relative"))
							g_jog_type =
								JOG_RELATIVE;
						else
							g_jog_type = atoi(sval);
					}
				}
			}
			continue;
		}
		if (strncmp(line, "rgb ", 4) == 0 ||
		    strncmp(line, "rgb\t", 4) == 0) {
			char name[32];
			unsigned int pad, r, g, b;
			if (sscanf(line + 4, "%31s %u %u %u %u", name, &pad, &r,
				   &g, &b) == 5)
				midi_out_bind_rgb(name, (uint8_t)pad,
						  (uint8_t)(r & 0x7F),
						  (uint8_t)(g & 0x7F),
						  (uint8_t)(b & 0x7F));
			continue;
		}
		if (strncmp(line, "out ", 4) == 0 ||
		    strncmp(line, "out\t", 4) == 0) {
			char name[32];
			unsigned int st, d1, d2;
			if (sscanf(line + 4, "%31s %x %u %u", name, &st, &d1,
				   &d2) == 4)
				midi_out_bind(name, (uint8_t)st, (uint8_t)d1,
					      (uint8_t)d2);
			continue;
		}
		char name[64];
		char modifier[16] = "";
		unsigned int stat_hex, d1;
		int nfields = sscanf(line, "%63s %x %u %15s", name, &stat_hex,
				     &d1, modifier);
		if (nfields >= 3) {
			MidiAction act = midi_action_from_name(name);
			if (act != MACT_NONE) {
				midi_bind((uint8_t)stat_hex, (uint8_t)d1, act);
				if (nfields == 4 &&
				    (!strcmp(modifier, "rel") ||
				     !strcmp(modifier, "relative")))
					midi_bind_set_relative(act, 1);
			}
		}
	}
	fclose(f);
}

void midi_map_save(void)
{
	char path[512];
	midi_map_path(path, sizeof(path));
	char dir[512];
	snprintf(dir, sizeof(dir), "%s", path);
	char *slash = strrchr(dir, '/');
	if (slash) {
		*slash = '\0';
		mkdir(dir, 0755);
	}
	FILE *f = fopen(path, "w");
	if (!f)
		return;
	fprintf(f, "# djcmd MIDI map -- auto-generated\n");
	fprintf(f, "# Input bindings:  action_name  status_hex  data1_dec\n");
	fprintf(f,
		"# Output bindings: out  name  status_hex  data1_dec  data2_dec\n");
	fprintf(f, "# Tuning params:   set  name  value\n");
	fprintf(f,
		"# status: B0-BF=CC ch1-16  90-9F=NoteOn ch1-16  80-8F=NoteOff ch1-16\n#\n");
	fprintf(f, "set  jog_type         %s\n",
		g_jog_type == JOG_NS7III ? "ns7iii" : "relative");
	if (g_jog_type == JOG_NS7III) {
		fprintf(f, "set  jog_scratch_revs %.0f\n",
			(double)g_jog_scratch_revs);
		fprintf(f, "set  jog_ref_delta    %.7f\n",
			(double)g_jog_ref_delta);
		fprintf(f, "set  jog_motor_dead   %.4f\n",
			(double)g_jog_motor_dead);
		fprintf(f, "set  jog_vel_max      %.1f\n",
			(double)g_jog_vel_max);
	}
	if (g_jog_smooth_alpha != 0.12f || g_jog_dead_band != 0.005f ||
	    g_jog_spike_thresh != 0.80f || g_jog_slew_rate != 0.04f ||
	    g_jog_settle_ms != 2000) {
		fprintf(f, "set  jog_smooth  %.3f\n",
			(double)g_jog_smooth_alpha);
		fprintf(f, "set  jog_dead    %.4f\n", (double)g_jog_dead_band);
		fprintf(f, "set  jog_spike   %.3f\n",
			(double)g_jog_spike_thresh);
		fprintf(f, "set  jog_slew    %.4f\n", (double)g_jog_slew_rate);
		fprintf(f, "set  jog_settle  %d\n", g_jog_settle_ms);
	}
	fprintf(f, "#\n");
	for (int i = 0; i < g_midi_nbindings; i++) {
		MidiBinding *b = &g_midi_bindings[i];
		if (b->action > MACT_NONE && b->action < MACT_COUNT)
			fprintf(f, "%-18s  %02X  %3u%s\n",
				g_mact_names[b->action], b->status, b->data1,
				b->relative ? "  rel" : "");
	}
	if (g_midi_nout_bindings > 0) {
		fprintf(f, "\n# Output bindings (djcmd -> controller)\n");
		for (int i = 0; i < g_midi_nout_bindings; i++) {
			MidiOutBinding *b = &g_midi_out_bindings[i];
			if (b->status == 0)
				continue;
			if (b->sysex_len > 0) {
				fprintf(f, "rgb  %-18s  %3u  %3u  %3u  %3u\n",
					b->name, b->data1,
					b->sysex_len > 7 ? b->sysex[7] : 0,
					b->sysex_len > 8 ? b->sysex[8] : 0,
					b->sysex_len > 9 ? b->sysex[9] : 0);
			} else {
				fprintf(f, "out  %-18s  %02X  %3u  %3u\n",
					b->name, b->status, b->data1, b->data2);
			}
		}
	}
	fclose(f);
}

static void midi_map_write_generic_defaults(void)
{
	char path[512];
	midi_map_path(path, sizeof(path));
	FILE *check = fopen(path, "r");
	if (check) {
		fclose(check);
		return;
	}
	char dir[512];
	snprintf(dir, sizeof(dir), "%s", path);
	char *slash = strrchr(dir, '/');
	if (slash) {
		*slash = '\0';
		mkdir(dir, 0755);
	}
	FILE *f = fopen(path, "w");
	if (!f)
		return;
	const char *dev_name = "";
	for (int i = 0; i < g_midi_ndevices; i++) {
		if (strcmp(g_midi_devlist[i].dev, g_midi_dev_str) == 0) {
			dev_name = g_midi_devlist[i].name;
			break;
		}
	}
	char dev_lower[128] = "";
	for (int i = 0; dev_name[i] && i < (int)sizeof(dev_lower) - 1; i++)
		dev_lower[i] =
			(char)((unsigned char)dev_name[i] >= 'A' &&
					       (unsigned char)dev_name[i] <=
						       'Z' ?
				       dev_name[i] + 32 :
				       dev_name[i]);
	if (strstr(dev_lower, "ns7")) {
		snprintf(g_fb_status, sizeof(g_fb_status),
			 "NS7III detected: writing map to %.200s", path);
		fputs(NS7III_MAP, f);
	} else {
		char san[128] = "";
		midi_map_name_from_device(dev_name, san, sizeof(san));
		fprintf(f, "# djcmd MIDI map for %s\n", dev_name);
		fprintf(f,
			"# Use MIDI Learn (ESC -> MIDI tab) to bind controls.\n");
	}
	fclose(f);
}

void handle_midi(uint8_t status, uint8_t data1, uint8_t data2)
{
	g_midi_last_status = status;
	g_midi_last_d1 = data1;
	g_midi_last_d2 = data2;

	if (g_midi_mon_open) {
		MidiMonEntry *me = &g_midi_mon_buf[g_midi_mon_head];
		me->status = status;
		me->d1 = data1;
		me->d2 = data2;
		me->matched_act = (int)midi_lookup(status, data1);
		g_midi_mon_head = (g_midi_mon_head + 1) % MIDI_MON_SIZE;
		if (g_midi_mon_count < MIDI_MON_SIZE)
			g_midi_mon_count++;
	}

	if (g_midi_learn_active || g_midi_learn_jog_pair) {
		uint8_t type = status & 0xF0;
		if (g_midi_learn_jog_pair) {
			int got_spin = (g_midi_learn_jog_spin_status != 0);
			int got_pb = (g_midi_learn_jog_step & 2);
			int got_touch = (g_midi_learn_jog_step & 1);
			if (!got_spin && type == 0xB0 && data2 != 64) {
				MidiAction spin_act =
					(MidiAction)(MACT_JOG_SPIN_A +
						     g_midi_learn_jog_deck);
				midi_bind(status, data1, spin_act);
				g_midi_learn_jog_spin_status = status;
				g_midi_learn_jog_spin_d1 = data1;
				got_spin = 1;
			}
			if (!got_pb && type == 0xE0) {
				int pb14 = ((int)data2 << 7) | (int)data1;
				if (pb14 != 8192) {
					MidiAction pb_act =
						(MidiAction)(MACT_JOG_PB_A +
							     g_midi_learn_jog_deck);
					midi_bind(status, 0, pb_act);
					g_midi_learn_jog_step |= 2;
					got_pb = 1;
				}
			}
			if (!got_touch && type == 0x90 && data2 > 0) {
				MidiAction touch_act =
					(MidiAction)(MACT_JOG_TOUCH_A +
						     g_midi_learn_jog_deck);
				midi_bind(status, data1, touch_act);
				g_midi_learn_jog_step |= 1;
				got_touch = 1;
			}
			if (got_spin && got_pb && got_touch) {
				g_midi_learn_jog_pair = 0;
				g_midi_learn_jog_step = 0;
				g_midi_learn_jog_spin_status = 0;
				g_midi_learn_jog_spin_d1 = 0;
				g_midi_learn_active = 0;
				midi_map_save();
			}
			return;
		}
		MidiAction learning = (MidiAction)g_midi_learn_sel;
		int is_encoder = (learning >= MACT_JOG_SPIN_A &&
				  learning <= MACT_JOG_SPIN_D) ||
				 (learning >= MACT_JOG_PB_A &&
				  learning <= MACT_JOG_PB_D) ||
				 (learning == MACT_LIB_ENCODER) ||
				 (learning >= MACT_PITCH_BEND_A &&
				  learning <= MACT_PITCH_BEND_D) ||
				 (learning >= MACT_FX_KNOB_1_A &&
				  learning <= MACT_FX_WET_B);
		int is_pb_action = (learning >= MACT_JOG_PB_A &&
				    learning <= MACT_JOG_PB_D);
		int is_note =
			(learning >= MACT_JOG_TOUCH_A &&
			 learning <= MACT_JOG_TOUCH_D) ||
			(learning == MACT_LIB_ENCODER_TOUCH) ||
			(learning >= MACT_PLAY_A && learning <= MACT_PLAY_D) ||
			(learning >= MACT_CUE_SET_1 &&
			 learning <= MACT_CUE_SET_4) ||
			(learning >= MACT_CUE_JUMP_1 &&
			 learning <= MACT_CUE_JUMP_4) ||
			(learning >= MACT_CUE_DELETE_1 &&
			 learning <= MACT_CUE_DELETE_4) ||
			(learning >= MACT_SYNC_FOLLOW_A &&
			 learning <= MACT_SYNC_FOLLOW_D) ||
			(learning == MACT_NUDGE_FWD) ||
			(learning == MACT_NUDGE_BACK) ||
			(learning == MACT_LOOP_TOGGLE) ||
			(learning >= MACT_LOOP_IN_A &&
			 learning <= MACT_LOOP_HALF_D) ||
			(learning >= MACT_KEY_LOCK_A &&
			 learning <= MACT_KEY_LOCK_D) ||
			(learning >= MACT_SLIP_MODE_A &&
			 learning <= MACT_SLIP_MODE_D) ||
			(learning >= MACT_REVERSE_A &&
			 learning <= MACT_REVERSE_D) ||
			(learning >= MACT_PAD_MODE_SAMPLER_A &&
			 learning <= MACT_PAD_MODE_SLICER_B) ||
			(learning >= MACT_LIB_SELECT &&
			 learning <= MACT_LIB_FWD) ||
			(learning >= MACT_LIB_LOAD_A &&
			 learning <= MACT_LIB_LOAD_D) ||
			(learning == MACT_PANEL_FILES) ||
			(learning == MACT_PANEL_LIBRARY) ||
			(learning >= MACT_PITCH_RANGE_A &&
			 learning <= MACT_PITCH_RANGE_D) ||
			(learning >= MACT_MOTOR_ON_A &&
			 learning <= MACT_MOTOR_OFF_D) ||
			(learning >= MACT_MOTOR_TOGGLE_A &&
			 learning <= MACT_MOTOR_TOGGLE_D) ||
			(learning >= MACT_DECK_SEL_1 &&
			 learning <= MACT_DECK_SEL_4);
		int accepted = 0;
		if (is_encoder) {
			if (is_pb_action) {
				if (type == 0xE0) {
					int pb14 =
						((int)data2 << 7) | (int)data1;
					if (pb14 != 8192) {
						midi_bind(status, 0, learning);
						g_midi_learn_active = 0;
						return;
					}
				}
			} else {
				if (type == 0xB0 && data2 != 64)
					accepted = 1;
				else if (type == 0xE0) {
					int pb14 =
						((int)data2 << 7) | (int)data1;
					if (pb14 != 8192) {
						midi_bind(status, 0, learning);
						g_midi_learn_active = 0;
						return;
					}
				}
			}
		} else if (is_note) {
			accepted = (type == 0x90 && data2 > 0);
		} else {
			accepted = (type == 0xB0 && data2 > 0);
		}
		if (accepted) {
			midi_bind(status, data1, learning);
			g_midi_learn_active = 0;
			midi_map_save();
		}
		return;
	}

	MidiAction act = midi_lookup(status, data1);
	uint8_t type = status & 0xF0;

	if (type == 0xB0) {
		int is_self_relative =
			(act >= MACT_JOG_SPIN_A && act <= MACT_JOG_SPIN_D) ||
			(act >= MACT_JOG_PB_A && act <= MACT_JOG_PB_D) ||
			(act == MACT_LIB_ENCODER) ||
			(act >= MACT_PITCH_BEND_A &&
			 act <= MACT_PITCH_BEND_D) ||
			(act >= MACT_FX_KNOB_1_A && act <= MACT_FX_WET_B);
		if (!is_self_relative) {
			for (int bi = 0; bi < g_midi_nbindings; bi++) {
				if (g_midi_bindings[bi].status == status &&
				    g_midi_bindings[bi].data1 == data1 &&
				    g_midi_bindings[bi].relative) {
					if (data2 == 64)
						return;
					int d = (data2 > 64) ?
							(int)(data2 - 64) :
							-(int)(64 - data2);
					g_midi_bindings[bi].rel_acc +=
						d * (1.0f / 127.0f);
					if (g_midi_bindings[bi].rel_acc < 0.0f)
						g_midi_bindings[bi].rel_acc =
							0.0f;
					if (g_midi_bindings[bi].rel_acc > 1.0f)
						g_midi_bindings[bi].rel_acc =
							1.0f;
					data2 = (uint8_t)(g_midi_bindings[bi]
								  .rel_acc *
							  127.0f);
					break;
				}
			}
		}
	}

	float val = data2 / 127.0f;

	if (type == 0xB0) {
		switch (act) {
		case MACT_DECK_VOL_A:
			g_tracks[0].volume = VAL14(data2, g_vol_lsb[0]);
			break;
		case MACT_DECK_VOL_B:
			g_tracks[1].volume = VAL14(data2, g_vol_lsb[1]);
			break;
		case MACT_DECK_VOL_C:
			g_tracks[2].volume = VAL14(data2, g_vol_lsb[2]);
			break;
		case MACT_DECK_VOL_D:
			g_tracks[3].volume = VAL14(data2, g_vol_lsb[3]);
			break;
		case MACT_PITCH_A:
		case MACT_PITCH_B:
		case MACT_PITCH_C:
		case MACT_PITCH_D: {
			int deck = act - MACT_PITCH_A;
			if (deck < 0 || deck >= 4)
				break;
			float range = g_pitch_range_vals[g_pitch_range[deck]];
			int full14 = ((int)data2 << 7) | (int)g_pitch_lsb[deck];
			float norm = ((float)full14 - 8191.5f) / 8191.5f;
			float dead_norm = 0.02f / range;
			if (dead_norm > 0.08f)
				dead_norm = 0.08f;
			if (norm > -dead_norm && norm < dead_norm)
				g_tracks[deck].pitch = 1.0f;
			else {
				float pitch = 1.0f + norm * range;
				if (pitch < 0.25f)
					pitch = 0.25f;
				if (pitch > 2.00f)
					pitch = 2.00f;
				g_tracks[deck].pitch = pitch;
			}
			motor_sync_pitch(deck);
			break;
		}
		case MACT_PITCH_LSB_A:
		case MACT_PITCH_LSB_B:
		case MACT_PITCH_LSB_C:
		case MACT_PITCH_LSB_D:
			g_pitch_lsb[act - MACT_PITCH_LSB_A] = data2;
			break;
		case MACT_VOL_LSB_A:
		case MACT_VOL_LSB_B:
		case MACT_VOL_LSB_C:
		case MACT_VOL_LSB_D:
			g_vol_lsb[act - MACT_VOL_LSB_A] = data2;
			break;
		case MACT_EQ_LOW_LSB_A:
		case MACT_EQ_LOW_LSB_B:
		case MACT_EQ_LOW_LSB_C:
		case MACT_EQ_LOW_LSB_D:
			g_eq_low_lsb[act - MACT_EQ_LOW_LSB_A] = data2;
			break;
		case MACT_EQ_MID_LSB_A:
		case MACT_EQ_MID_LSB_B:
		case MACT_EQ_MID_LSB_C:
		case MACT_EQ_MID_LSB_D:
			g_eq_mid_lsb[act - MACT_EQ_MID_LSB_A] = data2;
			break;
		case MACT_EQ_HIGH_LSB_A:
		case MACT_EQ_HIGH_LSB_B:
		case MACT_EQ_HIGH_LSB_C:
		case MACT_EQ_HIGH_LSB_D:
			g_eq_high_lsb[act - MACT_EQ_HIGH_LSB_A] = data2;
			break;
		case MACT_GAIN_LSB_A:
		case MACT_GAIN_LSB_B:
		case MACT_GAIN_LSB_C:
		case MACT_GAIN_LSB_D:
			g_gain_lsb[act - MACT_GAIN_LSB_A] = data2;
			break;
		case MACT_FILTER_LSB_A:
		case MACT_FILTER_LSB_B:
		case MACT_FILTER_LSB_C:
		case MACT_FILTER_LSB_D:
			g_filter_lsb[act - MACT_FILTER_LSB_A] = data2;
			break;
		case MACT_CROSSFADER_LSB:
			g_crossfader_lsb = data2;
			break;
		case MACT_MASTER_VOL_LSB:
			g_master_vol_lsb = data2;
			break;
		case MACT_EQ_LOW_A:
		case MACT_EQ_LOW_B:
		case MACT_EQ_LOW_C:
		case MACT_EQ_LOW_D: {
			int dk = act - MACT_EQ_LOW_A;
			g_eq_low_knob[dk] =
				VAL14(data2, g_eq_low_lsb[dk]) * 2.0f - 1.0f;
			if (!g_eq_low_kill[dk])
				g_tracks[dk].eq_low = g_eq_low_knob[dk];
			break;
		}
		case MACT_EQ_MID_A:
		case MACT_EQ_MID_B:
		case MACT_EQ_MID_C:
		case MACT_EQ_MID_D: {
			int dk = act - MACT_EQ_MID_A;
			g_eq_mid_knob[dk] =
				VAL14(data2, g_eq_mid_lsb[dk]) * 2.0f - 1.0f;
			if (!g_eq_mid_kill[dk])
				g_tracks[dk].eq_mid = g_eq_mid_knob[dk];
			break;
		}
		case MACT_EQ_HIGH_A:
		case MACT_EQ_HIGH_B:
		case MACT_EQ_HIGH_C:
		case MACT_EQ_HIGH_D: {
			int dk = act - MACT_EQ_HIGH_A;
			g_eq_high_knob[dk] =
				VAL14(data2, g_eq_high_lsb[dk]) * 2.0f - 1.0f;
			if (!g_eq_high_kill[dk])
				g_tracks[dk].eq_high = g_eq_high_knob[dk];
			break;
		}
		case MACT_GAIN_A:
			g_tracks[0].gain = VAL14(data2, g_gain_lsb[0]) * 2.0f;
			break;
		case MACT_GAIN_B:
			g_tracks[1].gain = VAL14(data2, g_gain_lsb[1]) * 2.0f;
			break;
		case MACT_GAIN_C:
			g_tracks[2].gain = VAL14(data2, g_gain_lsb[2]) * 2.0f;
			break;
		case MACT_GAIN_D:
			g_tracks[3].gain = VAL14(data2, g_gain_lsb[3]) * 2.0f;
			break;
		case MACT_FILTER_A:
		case MACT_FILTER_B:
		case MACT_FILTER_C:
		case MACT_FILTER_D: {
			int deck = act - MACT_FILTER_A;
			if (deck >= 4)
				break;
			int full14 =
				((int)data2 << 7) | (int)g_filter_lsb[deck];
			float norm = ((float)full14 - 8191.5f) / 8191.5f;
			if (norm > -0.03f && norm < 0.03f)
				norm = 0.0f;
			else if (norm > 0.0f)
				norm = (norm - 0.03f) / 0.97f;
			else
				norm = (norm + 0.03f) / 0.97f;
			g_tracks[deck].filter = norm * 0.5f + 0.5f;
			if (fabsf(norm) < 0.01f) {
				g_eq[deck].fi_x1l = g_eq[deck].fi_x2l = 0.0f;
				g_eq[deck].fi_y1l = g_eq[deck].fi_y2l = 0.0f;
				g_eq[deck].fi_x1r = g_eq[deck].fi_x2r = 0.0f;
				g_eq[deck].fi_y1r = g_eq[deck].fi_y2r = 0.0f;
			}
			break;
		}
		case MACT_CROSSFADER:
			g_crossfader = VAL14(data2, g_crossfader_lsb);
			g_autoplay_xf_target = -1.0f;
			break;
		case MACT_CF_CURVE:
			g_cf_curve = val;
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "X-Fader Curve: %.0f%%",
				 (double)(val * 100.0f));
			break;
		case MACT_TAP_BPM_A:
			tap_bpm(0);
			break;
		case MACT_TAP_BPM_B:
			tap_bpm(1);
			break;
		case MACT_GRID_SNAP_A:
			snap_grid(0);
			break;
		case MACT_GRID_SNAP_B:
			snap_grid(1);
			break;
		case MACT_MASTER_VOL:
			g_master_vol =
				(int)(VAL14(data2, g_master_vol_lsb) * 150.0f);
			g_opts.default_master_vol = g_master_vol;
			break;
		case MACT_STRIP_A:
		case MACT_STRIP_B:
		case MACT_STRIP_C:
		case MACT_STRIP_D: {
			int deck = act - MACT_STRIP_A;
			if (deck >= 4)
				break;
			if (g_tracks[deck].playing)
				break;
			Track *st = &g_tracks[deck];
			if (!st->loaded || st->num_frames == 0)
				break;
			pthread_mutex_lock(&st->lock);
			st->pos = (uint32_t)((float)data2 / 127.0f *
					     (float)(st->num_frames - 1));
			pthread_mutex_unlock(&st->lock);
			break;
		}
		case MACT_JOG_SPIN_A:
		case MACT_JOG_SPIN_B:
		case MACT_JOG_SPIN_C:
		case MACT_JOG_SPIN_D: {
			int deck = (act <= MACT_JOG_SPIN_B) ?
					   act - MACT_JOG_SPIN_A :
					   (act - MACT_JOG_SPIN_C) + 2;
			if (deck < 0 || deck >= MAX_TRACKS)
				break;
			if (g_jog_type == JOG_NS7III) {
				int prev_c = g_jog_coarse[deck];
				g_jog_coarse[deck] = (int)data2;
				if (prev_c >= 0) {
					int cd = g_jog_coarse[deck] - prev_c;
					if (cd > 64)
						cd -= 128;
					if (cd < -64)
						cd += 128;
					g_jog_last_coarse_d[deck] = cd;
					struct timespec _cts;
					clock_gettime(CLOCK_MONOTONIC, &_cts);
					g_jog_last_spin_ms[deck] =
						(int64_t)_cts.tv_sec * 1000 +
						_cts.tv_nsec / 1000000;
				}
				struct timespec _uts;
				clock_gettime(CLOCK_MONOTONIC, &_uts);
				int64_t now_ms = (int64_t)_uts.tv_sec * 1000 +
						 _uts.tv_nsec / 1000000;
				if (now_ms - g_jog_last_update_ms[deck] >= 4) {
					ns7iii_update_jog(deck,
							  g_jog_coarse[deck],
							  g_jog_fine[deck]);
					g_jog_last_update_ms[deck] = now_ms;
				}
				break;
			}
			int delta = (data2 == 0)   ? -64 :
				    (data2 == 127) ? 63 :
						     (int)data2 - 64;
			if (delta == 0)
				break;
			Track *jt = &g_tracks[deck];
			if (g_motor_running[deck]) {
				g_motor_vel[deck] += (delta * 0.0005f);
				if (g_motor_vel[deck] > 0.15f)
					g_motor_vel[deck] = 0.15f;
				if (g_motor_vel[deck] < -0.15f)
					g_motor_vel[deck] = -0.15f;
			} else {
				MidiAction touch_act =
					(MidiAction)(MACT_JOG_TOUCH_A + deck);
				int touch_bound = 0;
				for (int bi = 0; bi < g_midi_nbindings; bi++)
					if (g_midi_bindings[bi].action ==
					    touch_act) {
						touch_bound = 1;
						break;
					}
				if ((!touch_bound) || g_jog_touched[deck]) {
					pthread_mutex_lock(&jt->lock);
					int64_t new_pos =
						(int64_t)jt->pos +
						(int64_t)(delta *
							  SCRATCH_FRAMES_PER_TICK);
					if (new_pos < 0)
						new_pos = 0;
					if (new_pos >= (int64_t)jt->num_frames)
						new_pos =
							(int64_t)
								jt->num_frames -
							1;
					jt->pos = (uint32_t)new_pos;
					pthread_mutex_unlock(&jt->lock);
				} else {
					int nudge_dir = (delta > 0) ? 1 : -1;
					g_jog_nudge[deck] +=
						nudge_dir *
						NUDGE_PITCH_PER_TICK;
					if (g_jog_nudge[deck] > 0.15f)
						g_jog_nudge[deck] = 0.15f;
					if (g_jog_nudge[deck] < -0.15f)
						g_jog_nudge[deck] = -0.15f;
				}
			}
			break;
		}
		case MACT_LIB_ENCODER: {
			if (data2 == 64)
				break;
			{
				struct timespec _lts;
				clock_gettime(CLOCK_MONOTONIC, &_lts);
				g_lib_enc_last_ms =
					(int64_t)_lts.tv_sec * 1000 +
					_lts.tv_nsec / 1000000;
			}
			if (g_num_tracks == 4 && g_view == 0) {
				g_view = 1;
				g_lib_auto_switched = 1;
			}
			int step = (data2 > 64) ? 1 : -1;
			if (g_panel == 0 && g_fb_count > 0) {
				g_fb_sel += step;
				if (g_fb_sel < 0)
					g_fb_sel = 0;
				if (g_fb_sel >= g_fb_count)
					g_fb_sel = g_fb_count - 1;
			} else if (g_panel == 1 && g_pl_count > 0) {
				g_pl_sel += step;
				if (g_pl_sel < 0)
					g_pl_sel = 0;
				if (g_pl_sel >= g_pl_count)
					g_pl_sel = g_pl_count - 1;
			} else if (g_panel == 2 && g_lib_count > 0) {
				g_lib_sel += step;
				if (g_lib_sel < 0)
					g_lib_sel = 0;
				if (g_lib_sel >= g_lib_count)
					g_lib_sel = g_lib_count - 1;
			}
			break;
		}
		case MACT_PITCH_BEND_A:
		case MACT_PITCH_BEND_B:
		case MACT_PITCH_BEND_C:
		case MACT_PITCH_BEND_D: {
			int deck = act - MACT_PITCH_BEND_A;
			if (deck < 0 || deck >= 4)
				break;
			if (data2 == 64)
				break;
			int nudge_dir = (data2 > 64) ? 1 : -1;
			g_tracks[deck].nudge += nudge_dir * CFG_NUDGE_AMOUNT;
			if (g_tracks[deck].nudge >
			    CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP)
				g_tracks[deck].nudge =
					CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP;
			if (g_tracks[deck].nudge <
			    -CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP)
				g_tracks[deck].nudge =
					-CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP;
			break;
		}
		case MACT_FX_KNOB_1_A:
		case MACT_FX_KNOB_2_A:
		case MACT_FX_KNOB_3_A:
		case MACT_FX_KNOB_1_B:
		case MACT_FX_KNOB_2_B:
		case MACT_FX_KNOB_3_B: {
			int is_b = (act >= MACT_FX_KNOB_1_B);
			int dk = is_b ? g_side_deck[1] : g_side_deck[0];
			int param = is_b ? (int)(act - MACT_FX_KNOB_1_B) :
					   (int)(act - MACT_FX_KNOB_1_A);
			if (dk >= MAX_TRACKS || data2 == 64)
				break;
			float delta =
				(data2 > 64) ? FX_KNOB_STEP : -FX_KNOB_STEP;
			g_fx_param_acc[dk][param] += delta;
			if (g_fx_param_acc[dk][param] < 0.0f)
				g_fx_param_acc[dk][param] = 0.0f;
			if (g_fx_param_acc[dk][param] > 1.0f)
				g_fx_param_acc[dk][param] = 1.0f;
			fx_set_param(dk, g_fx_ui_slot[dk], param,
				     g_fx_param_acc[dk][param]);
			{
				FXSlot *_s = fx_slot(dk, g_fx_ui_slot[dk]);
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Deck %c FX%d %s p%d: %.0f%%",
					 DECK_NUM(dk), g_fx_ui_slot[dk] + 1,
					 fx_names[_s->type], param + 1,
					 g_fx_param_acc[dk][param] * 100.0f);
			}
			break;
		}
		case MACT_FX_WET_A:
		case MACT_FX_WET_B: {
			int dk = (act == MACT_FX_WET_B) ? g_side_deck[1] :
							  g_side_deck[0];
			if (dk >= MAX_TRACKS || data2 == 64)
				break;
			float delta =
				(data2 > 64) ? FX_KNOB_STEP : -FX_KNOB_STEP;
			g_fx_param_acc[dk][3] += delta;
			if (g_fx_param_acc[dk][3] < 0.0f)
				g_fx_param_acc[dk][3] = 0.0f;
			if (g_fx_param_acc[dk][3] > 1.0f)
				g_fx_param_acc[dk][3] = 1.0f;
			fx_set_wet(dk, g_fx_ui_slot[dk], g_fx_param_acc[dk][3]);
			{
				FXSlot *_s = fx_slot(dk, g_fx_ui_slot[dk]);
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Deck %c FX%d %s wet: %.0f%%",
					 DECK_NUM(dk), g_fx_ui_slot[dk] + 1,
					 fx_names[_s->type],
					 g_fx_param_acc[dk][3] * 100.0f);
			}
			break;
		}
		default:
			break;
		}
	}

	if (type == 0xE0) {
		MidiAction pb_act = midi_lookup(status, 0);
		int deck = -1;
		if (pb_act >= MACT_JOG_PB_A && pb_act <= MACT_JOG_PB_D) {
			deck = pb_act - MACT_JOG_PB_A;
			if (deck >= 0 && deck < MAX_TRACKS) {
				int raw14 = ((int)data2 << 7) | (int)data1;
				if (g_jog_type == JOG_NS7III) {
					if (g_motor_running[deck] &&
					    g_motor_settle_until[deck] > 0) {
						struct timespec ts;
						clock_gettime(CLOCK_MONOTONIC,
							      &ts);
						int64_t now_ms =
							(int64_t)ts.tv_sec *
								1000 +
							ts.tv_nsec / 1000000;
						if (now_ms <
						    g_motor_settle_until
							    [deck]) {
							g_jog_abs_init[deck] =
								0;
							g_jog_abs_vel[deck] =
								0.0f;
							g_jog_nudge[deck] =
								0.0f;
							g_motor_vel[deck] =
								0.0f;
							g_jog_last_pb[deck] =
								raw14;
							goto pb_settled;
						}
						g_motor_settle_until[deck] = 0;
						g_jog_abs_init[deck] = 0;
						g_jog_last_pb[deck] = raw14;
					}
					g_jog_fine[deck] = raw14;
					if (g_motor_running[deck]) {
						int deltaPB =
							raw14 -
							g_jog_last_pb[deck];
						if (deltaPB > 8192)
							deltaPB -= 16384;
						if (deltaPB < -8192)
							deltaPB += 16384;
						struct timespec _pts;
						clock_gettime(CLOCK_MONOTONIC,
							      &_pts);
						int64_t _now =
							(int64_t)_pts.tv_sec *
								1000 +
							_pts.tv_nsec / 1000000;
						int effDelta =
							(_now - g_jog_last_spin_ms
									 [deck] <
							 30) ?
								g_jog_last_coarse_d
									[deck] :
								0;
						int slipErr = abs(
							deltaPB -
							effDelta *
								NS7III_PB_RATIO);
						if (slipErr >
							    NS7III_SLIP_THRESH ||
						    (effDelta == 0 &&
						     abs(deltaPB) > 100)) {
							g_jog_release_conf
								[deck] = 0;
							g_jog_touched[deck] = 1;
						} else if (g_jog_touched
								   [deck]) {
							g_jog_release_conf
								[deck]++;
							if (g_jog_release_conf
								    [deck] >=
							    NS7III_RELEASE_CONF) {
								g_jog_touched
									[deck] =
										0;
								g_jog_release_conf
									[deck] =
										0;
							}
						}
						g_jog_last_pb[deck] = raw14;
					}
				} else {
					if (!g_motor_running[deck])
						goto pb_settled;
					if (g_motor_settle_until[deck] > 0) {
						struct timespec ts;
						clock_gettime(CLOCK_MONOTONIC,
							      &ts);
						int64_t now_ms =
							(int64_t)ts.tv_sec *
								1000 +
							ts.tv_nsec / 1000000;
						if (now_ms <
						    g_motor_settle_until
							    [deck]) {
							g_jog_nudge[deck] =
								0.0f;
							g_pb_streak[deck] = 0;
							g_pb_mag_acc[deck] =
								0.0f;
							goto pb_settled;
						}
						g_motor_settle_until[deck] = 0;
						g_pb_streak[deck] = 0;
						g_pb_mag_acc[deck] = 0.0f;
					}
					float norm = ((float)raw14 - 8192.0f) /
						     8192.0f;
					if (norm > 1.0f)
						norm = 1.0f;
					if (norm < -1.0f)
						norm = -1.0f;
#define JOG_SUSTAIN 12
#define JOG_MAGNITUDE g_jog_dead_band
#define JOG_SCALE 0.12f
					if (fabsf(norm) < JOG_MAGNITUDE) {
						if (g_pb_streak[deck] > 0)
							g_pb_streak[deck]--;
						else if (g_pb_streak[deck] < 0)
							g_pb_streak[deck]++;
					} else {
						int cur_sign =
							(norm > 0.0f) ? 1 : -1;
						int str_sign =
							(g_pb_streak[deck] >=
							 0) ?
								1 :
								-1;
						if (cur_sign == str_sign ||
						    g_pb_streak[deck] == 0) {
							g_pb_streak[deck] +=
								cur_sign;
							if (g_pb_streak[deck] >
							    127)
								g_pb_streak[deck] =
									127;
							if (g_pb_streak[deck] <
							    -127)
								g_pb_streak[deck] =
									-127;
							g_pb_mag_acc[deck] =
								g_pb_mag_acc[deck] *
									0.95f +
								fabsf(norm) *
									0.05f;
						} else {
							g_pb_streak[deck] =
								cur_sign;
							g_pb_mag_acc[deck] =
								fabsf(norm);
						}
						if (abs(g_pb_streak[deck]) >=
						    JOG_SUSTAIN) {
							float raw_vel =
								(float)cur_sign *
								g_pb_mag_acc
									[deck] *
								JOG_SCALE;
							if (g_tracks[deck]
								    .playing)
								g_motor_vel[deck] =
									(g_motor_vel
										 [deck] *
									 0.98f) +
									(raw_vel *
									 0.02f);
							else {
								g_motor_vel[deck] =
									0.0f;
								g_jog_nudge[deck] =
									0.0f;
							}
						}
					}
#undef JOG_SUSTAIN
#undef JOG_MAGNITUDE
#undef JOG_SCALE
				}
			}
		}
	pb_settled:;
	}

	if (type == 0x90 && data2 > 0) {
		switch (act) {
		case MACT_PLAY_A:
			g_tracks[0].playing = !g_tracks[0].playing;
			if (g_tracks[0].playing && !g_slip_motor_off[0])
				motor_set(0, 1);
			else if (!g_tracks[0].playing) {
				motor_set(0, 0);
				g_jog_nudge[0] = 0;
				g_last_applied_nudge[0] = 0;
			}
			deck_leds_refresh();
			break;
		case MACT_PLAY_B:
			g_tracks[1].playing = !g_tracks[1].playing;
			if (g_tracks[1].playing && !g_slip_motor_off[1])
				motor_set(1, 1);
			else if (!g_tracks[1].playing)
				motor_set(1, 0);
			deck_leds_refresh();
			break;
		case MACT_PLAY_C:
			if (g_tracks[2].loaded) {
				g_tracks[2].playing = !g_tracks[2].playing;
				if (g_tracks[2].playing && !g_slip_motor_off[2])
					motor_set(2, 1);
				else if (!g_tracks[2].playing) {
					motor_set(2, 0);
					g_jog_nudge[2] = 0;
					g_last_applied_nudge[2] = 0;
				}
				deck_leds_refresh();
			}
			break;
		case MACT_PLAY_D:
			if (g_tracks[3].loaded) {
				g_tracks[3].playing = !g_tracks[3].playing;
				if (g_tracks[3].playing && !g_slip_motor_off[3])
					motor_set(3, 1);
				else if (!g_tracks[3].playing) {
					motor_set(3, 0);
					g_jog_nudge[3] = 0;
					g_last_applied_nudge[3] = 0;
				}
				deck_leds_refresh();
			}
			break;
		case MACT_JOG_TOUCH_D:
			g_jog_touched[3] = (data2 > 0);
			break;
		case MACT_CUE_ACTIVE_A:
			g_tracks[0].cue_active ^= 1;
			break;
		case MACT_CUE_ACTIVE_B:
			g_tracks[1].cue_active ^= 1;
			break;
		case MACT_CUE_ACTIVE_C:
			if (g_num_tracks >= 3)
				g_tracks[2].cue_active ^= 1;
			break;
		case MACT_CUE_ACTIVE_D:
			if (g_num_tracks >= 4)
				g_tracks[3].cue_active ^= 1;
			break;
		case MACT_CUE_JUMP_1:
		case MACT_CUE_JUMP_2:
		case MACT_CUE_JUMP_3:
		case MACT_CUE_JUMP_4: {
			int ci = act - MACT_CUE_JUMP_1;
			Track *t = &g_tracks[g_active_track];
			if (t->cue_set[ci]) {
				pthread_mutex_lock(&t->lock);
				t->pos = t->cue[ci];
				pthread_mutex_unlock(&t->lock);
			}
			break;
		}
		case MACT_SYNC_FOLLOW_A:
			g_tracks[0].synced ^= 1;
			if (g_tracks[0].synced)
				led_on("led_sync_a");
			else
				led_off("led_sync_a");
			break;
		case MACT_SYNC_FOLLOW_B:
			g_tracks[1].synced ^= 1;
			if (g_tracks[1].synced)
				led_on("led_sync_b");
			else
				led_off("led_sync_b");
			break;
		case MACT_SYNC_FOLLOW_C:
			g_tracks[2].synced ^= 1;
			if (g_tracks[2].synced)
				led_on("led_sync_c");
			else
				led_off("led_sync_c");
			break;
		case MACT_SYNC_FOLLOW_D:
			g_tracks[3].synced ^= 1;
			if (g_tracks[3].synced)
				led_on("led_sync_d");
			else
				led_off("led_sync_d");
			break;
		case MACT_NUDGE_FWD:
			g_tracks[g_active_track].nudge += CFG_NUDGE_AMOUNT;
			if (g_tracks[g_active_track].nudge >
			    CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP)
				g_tracks[g_active_track].nudge =
					CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP;
			break;
		case MACT_NUDGE_BACK:
			g_tracks[g_active_track].nudge -= CFG_NUDGE_AMOUNT;
			if (g_tracks[g_active_track].nudge <
			    -CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP)
				g_tracks[g_active_track].nudge =
					-CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP;
			break;
		case MACT_NUDGE_FWD_B: {
			int dk = g_side_deck[1];
			if (dk < g_num_tracks) {
				g_tracks[dk].nudge += CFG_NUDGE_AMOUNT;
				if (g_tracks[dk].nudge >
				    CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP)
					g_tracks[dk].nudge = CFG_NUDGE_AMOUNT *
							     CFG_NUDGE_CAP;
			}
			break;
		}
		case MACT_NUDGE_BACK_B: {
			int dk = g_side_deck[1];
			if (dk < g_num_tracks) {
				g_tracks[dk].nudge -= CFG_NUDGE_AMOUNT;
				if (g_tracks[dk].nudge <
				    -CFG_NUDGE_AMOUNT * CFG_NUDGE_CAP)
					g_tracks[dk].nudge = -CFG_NUDGE_AMOUNT *
							     CFG_NUDGE_CAP;
			}
			break;
		}
		case MACT_LOOP_TOGGLE: {
			Track *t = &g_tracks[g_active_track];
			t->looping ^= 1;
			deck_leds_refresh();
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Deck %c loop %s", DECK_NUM(g_active_track),
				 t->looping ? "ON" : "OFF");
			break;
		}
		case MACT_LIB_ENCODER_TOUCH:
			g_lib_touched = (data2 > 0);
			if (g_lib_touched) {
				if (g_num_tracks == 4 && g_view == 0) {
					g_view = 1;
					g_lib_auto_switched = 1;
				}
			}
			{
				struct timespec _lts2;
				clock_gettime(CLOCK_MONOTONIC, &_lts2);
				g_lib_enc_last_ms =
					(int64_t)_lts2.tv_sec * 1000 +
					_lts2.tv_nsec / 1000000;
			}
			break;
		case MACT_LIB_SELECT:
			if (g_panel == 0 && g_fb_count > 0) {
				if (g_fb_entries[g_fb_sel].is_dir)
					fb_enter_dir(
						g_fb_entries[g_fb_sel].name);
			}
			break;
		case MACT_LIB_BACK:
			if (g_panel == 0)
				fb_enter_dir("..");
			break;
		case MACT_LIB_LOAD_A:
		case MACT_LIB_LOAD_B:
		case MACT_LIB_LOAD_C:
		case MACT_LIB_LOAD_D: {
			int deck;
			if (act == MACT_LIB_LOAD_A)
				deck = g_side_deck[0];
			else if (act == MACT_LIB_LOAD_B)
				deck = g_side_deck[1];
			else
				deck = act - MACT_LIB_LOAD_A;
			if (deck < 0 || deck >= 4)
				break;
			int _loaded = 0;
			if (g_panel == 0 && g_fb_count > 0 &&
			    !g_fb_entries[g_fb_sel].is_dir) {
				char full[FB_PATH_MAX + 256];
				fb_selected_path(full, sizeof(full));
				enqueue_load(deck, full);
				_loaded = 1;
			} else if (g_panel == 1 && g_pl_count > 0) {
				enqueue_load(deck, g_pl[g_pl_sel].path);
				_loaded = 1;
			} else if (g_panel == 2 && g_lib && g_lib_count > 0) {
				enqueue_load(deck, g_lib[g_lib_sel].path);
				_loaded = 1;
			}
			if (_loaded)
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Loading \u2192 Deck %c",
					 DECK_NUM(deck));
			else
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "No track selected");
			pad_leds_refresh(deck);
			deck_leds_refresh();
			break;
		}
		case MACT_PANEL_FILES:
			g_panel = 0;
			if (g_view == 0)
				g_view = 1;
			break;
		case MACT_PANEL_LIBRARY:
			g_panel = 2;
			if (g_view == 0)
				g_view = 1;
			break;
		case MACT_PITCH_RANGE_A:
		case MACT_PITCH_RANGE_B:
		case MACT_PITCH_RANGE_C:
		case MACT_PITCH_RANGE_D: {
			int deck = act - MACT_PITCH_RANGE_A;
			if (deck >= 4)
				break;
			g_pitch_range[deck] = (g_pitch_range[deck] + 1) % 3;
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Deck %c pitch range: %s", DECK_NUM(deck),
				 g_pitch_range_names[g_pitch_range[deck]]);
			{
				char ln[32];
				snprintf(ln, sizeof(ln), "led_pitch_center_%c",
					 'a' + deck);
				led_off(ln);
			}
			break;
		}
		case MACT_CUE_SET_1:
		case MACT_CUE_SET_2:
		case MACT_CUE_SET_3:
		case MACT_CUE_SET_4: {
			int ci = act - MACT_CUE_SET_1;
			Track *t = &g_tracks[g_active_track];
			if (t->loaded) {
				t->cue[ci] = t->pos;
				t->cue_set[ci] = 1;
				sidecar_save(t);
				char side = (g_active_track == g_side_deck[0]) ?
						    'a' :
						    'b';
				char ln[32];
				snprintf(ln, sizeof(ln), "led_cue_%d_%c",
					 ci + 1, side);
				led_on(ln);
				pad_leds_refresh(g_active_track);
			}
			break;
		}
		case MACT_CUE_DELETE_1:
		case MACT_CUE_DELETE_2:
		case MACT_CUE_DELETE_3:
		case MACT_CUE_DELETE_4: {
			int ci = act - MACT_CUE_DELETE_1;
			Track *t = &g_tracks[g_active_track];
			t->cue[ci] = 0;
			t->cue_set[ci] = 0;
			sidecar_save(&g_tracks[g_active_track]);
			{
				char side = (g_active_track == g_side_deck[0]) ?
						    'a' :
						    'b';
				char ln[32];
				snprintf(ln, sizeof(ln), "led_cue_%d_%c",
					 ci + 1, side);
				led_off(ln);
			}
			pad_leds_refresh(g_active_track);
			break;
		}
		case MACT_LOOP_IN_A:
		case MACT_LOOP_IN_B:
		case MACT_LOOP_IN_C:
		case MACT_LOOP_IN_D: {
			int deck = act - MACT_LOOP_IN_A;
			if (deck >= 4)
				break;
			pthread_mutex_lock(&g_tracks[deck].lock);
			g_tracks[deck].loop_start = g_tracks[deck].pos;
			pthread_mutex_unlock(&g_tracks[deck].lock);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Deck %c loop IN set at %.2fs", DECK_NUM(deck),
				 (float)g_tracks[deck].pos /
					 (float)g_actual_sample_rate);
			break;
		}
		case MACT_LOOP_OUT_A:
		case MACT_LOOP_OUT_B:
		case MACT_LOOP_OUT_C:
		case MACT_LOOP_OUT_D: {
			int deck = act - MACT_LOOP_OUT_A;
			if (deck >= 4)
				break;
			Track *tlo = &g_tracks[deck];
			pthread_mutex_lock(&tlo->lock);
			if (tlo->looping) {
				tlo->looping = 0;
				pthread_mutex_unlock(&tlo->lock);
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Deck %c loop OFF", DECK_NUM(deck));
			} else if (tlo->pos > tlo->loop_start) {
				tlo->loop_end = tlo->pos;
				tlo->looping = 1;
				pthread_mutex_unlock(&tlo->lock);
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Deck %c loop ON (%.2fs)",
					 DECK_NUM(deck),
					 (float)(tlo->loop_end -
						 tlo->loop_start) /
						 (float)g_actual_sample_rate);
			} else {
				pthread_mutex_unlock(&tlo->lock);
			}
			deck_leds_refresh();
			break;
		}
		case MACT_LOOP_DOUBLE_A:
		case MACT_LOOP_DOUBLE_B:
		case MACT_LOOP_DOUBLE_C:
		case MACT_LOOP_DOUBLE_D: {
			int deck = act - MACT_LOOP_DOUBLE_A;
			if (deck >= 4)
				break;
			Track *ld = &g_tracks[deck];
			uint32_t len = ld->loop_end > ld->loop_start ?
					       ld->loop_end - ld->loop_start :
					       0;
			if (len > 0) {
				ld->loop_end = ld->loop_start + len * 2;
				if (ld->loop_end > ld->num_frames)
					ld->loop_end = ld->num_frames;
			}
			break;
		}
		case MACT_LOOP_HALF_A:
		case MACT_LOOP_HALF_B:
		case MACT_LOOP_HALF_C:
		case MACT_LOOP_HALF_D: {
			int deck = act - MACT_LOOP_HALF_A;
			if (deck >= 4)
				break;
			Track *lh = &g_tracks[deck];
			uint32_t len = lh->loop_end > lh->loop_start ?
					       lh->loop_end - lh->loop_start :
					       0;
			if (len > 2) {
				lh->loop_end = lh->loop_start + len / 2;
			}
			break;
		}
		case MACT_KEY_LOCK_A:
		case MACT_KEY_LOCK_B:
		case MACT_KEY_LOCK_C:
		case MACT_KEY_LOCK_D: {
			int deck = act - MACT_KEY_LOCK_A;
			if (deck >= 4)
				break;
			g_tracks[deck].key_lock = !g_tracks[deck].key_lock;
			if (g_tracks[deck].key_lock)
				wsola_reset(&g_wsola[deck], g_tracks[deck].pos);
			break;
		}
		case MACT_SLIP_MODE_A:
		case MACT_SLIP_MODE_B:
		case MACT_SLIP_MODE_C:
		case MACT_SLIP_MODE_D: {
			int deck = act - MACT_SLIP_MODE_A;
			if (deck >= MAX_TRACKS)
				break;
			g_slip_motor_off[deck] ^= 1;
			if (g_slip_motor_off[deck]) {
				motor_set(deck, 0);
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Deck %c: motor OFF (slip)",
					 DECK_NUM(deck));
			} else {
				if (g_tracks[deck].playing)
					motor_set(deck, 1);
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Deck %c: motor %s", DECK_NUM(deck),
					 g_tracks[deck].playing ? "restarted" :
								  "ready");
			}
			{
				char ln[32];
				snprintf(ln, sizeof(ln), "led_slip_%c",
					 'a' + deck);
				if (g_slip_motor_off[deck])
					led_on(ln);
				else
					led_off(ln);
			}
			break;
		}
		case MACT_REVERSE_A:
		case MACT_REVERSE_B:
		case MACT_REVERSE_C:
		case MACT_REVERSE_D: {
			int deck = act - MACT_REVERSE_A;
			if (deck >= MAX_TRACKS)
				break;
			g_tracks[deck].reverse ^= 1;
			if (g_motor_running[deck])
				motor_set_direction(deck,
						    g_tracks[deck].reverse);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Deck %c: %s", DECK_NUM(deck),
				 g_tracks[deck].reverse ? "REVERSE" :
							  "forward");
			break;
		}
		case MACT_CENSOR_A:
		case MACT_CENSOR_B:
		case MACT_CENSOR_C:
		case MACT_CENSOR_D: {
			int deck = act - MACT_CENSOR_A;
			if (deck >= 4)
				break;
			g_censor_held[deck] = 1;
			g_censor_save_pos[deck] = g_tracks[deck].pos;
			g_tracks[deck].reverse = 1;
			if (g_motor_running[deck])
				motor_set_direction(deck, 1);
			break;
		}
		case MACT_FILTER_TOGGLE_A:
		case MACT_FILTER_TOGGLE_B:
		case MACT_FILTER_TOGGLE_C:
		case MACT_FILTER_TOGGLE_D: {
			int deck = act - MACT_FILTER_TOGGLE_A;
			if (deck >= 4)
				break;
			g_filter_on[deck] ^= 1;
			{
				char ln[32];
				snprintf(ln, sizeof(ln), "led_filter_%c",
					 'a' + deck);
				if (g_filter_on[deck])
					led_on(ln);
				else
					led_off(ln);
			}
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Deck %c: filter %s", DECK_NUM(deck),
				 g_filter_on[deck] ? "ON" : "OFF");
			break;
		}
		case MACT_LIB_FWD: {
			if (g_panel == 0 && g_fb_count > 0 &&
			    g_fb_entries[g_fb_sel].is_dir)
				fb_enter_dir(g_fb_entries[g_fb_sel].name);
			break;
		}
		case MACT_MOTOR_TOGGLE_A:
		case MACT_MOTOR_TOGGLE_B:
		case MACT_MOTOR_TOGGLE_C:
		case MACT_MOTOR_TOGGLE_D: {
			int deck = act - MACT_MOTOR_TOGGLE_A;
			if (deck >= 4)
				break;
			if (g_motor_running[deck]) {
				g_slip_motor_off[deck] = 1;
				motor_set(deck, 0);
			} else {
				g_slip_motor_off[deck] = 0;
				motor_set(deck, 1);
			}
			break;
		}
		case MACT_MOTOR_ON_A:
			g_slip_motor_off[0] = 0;
			motor_set(0, 1);
			break;
		case MACT_MOTOR_ON_B:
			g_slip_motor_off[1] = 0;
			motor_set(1, 1);
			break;
		case MACT_MOTOR_ON_C:
			if (g_num_tracks > 2) {
				g_slip_motor_off[2] = 0;
				motor_set(2, 1);
			}
			break;
		case MACT_MOTOR_ON_D:
			if (g_num_tracks > 3) {
				g_slip_motor_off[3] = 0;
				motor_set(3, 1);
			}
			break;
		case MACT_MOTOR_OFF_A:
			g_slip_motor_off[0] = 1;
			motor_set(0, 0);
			break;
		case MACT_MOTOR_OFF_B:
			g_slip_motor_off[1] = 1;
			motor_set(1, 0);
			break;
		case MACT_MOTOR_OFF_C:
			if (g_num_tracks > 2) {
				g_slip_motor_off[2] = 1;
				motor_set(2, 0);
			}
			break;
		case MACT_MOTOR_OFF_D:
			if (g_num_tracks > 3) {
				g_slip_motor_off[3] = 1;
				motor_set(3, 0);
			}
			break;
		case MACT_DECK_SEL_1:
			motor_handoff(g_side_deck[0], 0);
			side_restack(0, 0);
			g_active_track = 0;
			deck_leds_refresh();
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Left \u2192 Deck 1%s",
				 g_motor_running[0] ? " [motor ON]" :
						      " [motor off]");
			break;
		case MACT_DECK_SEL_3:
			motor_handoff(g_side_deck[0], 2);
			side_restack(0, 2);
			g_active_track = 2;
			deck_leds_refresh();
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Left \u2192 Deck 3%s",
				 g_motor_running[2] ? " [motor ON]" :
						      " [motor off]");
			break;
		case MACT_DECK_SEL_2:
			motor_handoff(g_side_deck[1], 1);
			side_restack(1, 1);
			g_active_track = 1;
			deck_leds_refresh();
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Right \u2192 Deck 2%s",
				 g_motor_running[1] ? " [motor ON]" :
						      " [motor off]");
			break;
		case MACT_DECK_SEL_4:
			motor_handoff(g_side_deck[1], 3);
			side_restack(1, 3);
			g_active_track = 3;
			deck_leds_refresh();
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Right \u2192 Deck 4%s",
				 g_motor_running[3] ? " [motor ON]" :
						      " [motor off]");
			break;
		case MACT_SHIFT_A:
			g_pad_shift[g_side_deck[0]] = 1;
			break;
		case MACT_SHIFT_B:
			g_pad_shift[g_side_deck[1]] = 1;
			break;
		case MACT_PITCH_CENTER_A: {
			int dk = g_side_deck[0];
			g_tracks[dk].pitch = 1.0f;
			char ln[32];
			snprintf(ln, sizeof(ln), "led_pitch_center_%c",
				 'a' + dk);
			led_on(ln);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Deck %c pitch reset to 0%%", DECK_NUM(dk));
			break;
		}
		case MACT_PITCH_CENTER_B: {
			int dk = g_side_deck[1];
			g_tracks[dk].pitch = 1.0f;
			char ln[32];
			snprintf(ln, sizeof(ln), "led_pitch_center_%c",
				 'a' + dk);
			led_on(ln);
			snprintf(g_fb_status, sizeof(g_fb_status),
				 "Deck %c pitch reset to 0%%", DECK_NUM(dk));
			break;
		}
#define CUE_DEFAULT_HANDLER(ACT, DECK_IDX)                       \
	case ACT: {                                              \
		int dk = (DECK_IDX);                             \
		if (dk >= g_num_tracks)                          \
			break;                                   \
		Track *tc = &g_tracks[dk];                       \
		if (!tc->loaded)                                 \
			break;                                   \
		if (tc->playing) {                               \
			pthread_mutex_lock(&tc->lock);           \
			if (g_cue_default_set[dk])               \
				tc->pos = g_cue_default_pos[dk]; \
			tc->playing = 0;                         \
			pthread_mutex_unlock(&tc->lock);         \
			motor_set(dk, 0);                        \
			deck_leds_refresh();                     \
		} else {                                         \
			g_cue_default_pos[dk] = tc->pos;         \
			g_cue_default_set[dk] = 1;               \
			g_cue_default_held[dk] = 1;              \
			tc->playing = 1;                         \
			if (!g_slip_motor_off[dk])               \
				motor_set(dk, 1);                \
			deck_leds_refresh();                     \
		}                                                \
		break;                                           \
	}
			CUE_DEFAULT_HANDLER(MACT_CUE_DEFAULT_A, g_side_deck[0])
			CUE_DEFAULT_HANDLER(MACT_CUE_DEFAULT_B, g_side_deck[1])
#undef CUE_DEFAULT_HANDLER
		case MACT_PAD_MODE_CUES_A:
			g_pad_mode[g_side_deck[0]] = PAD_MODE_HOTCUE;
			pad_mode_leds_refresh(g_side_deck[0]);
			pad_leds_refresh(g_side_deck[0]);
			break;
		case MACT_PAD_MODE_SAMPLER_A:
			g_pad_mode[g_side_deck[0]] = PAD_MODE_SAMPLER;
			pad_mode_leds_refresh(g_side_deck[0]);
			pad_leds_refresh(g_side_deck[0]);
			break;
		case MACT_PAD_MODE_SLICER_A:
			if (g_opts.enable_slicer) {
				g_pad_mode[g_side_deck[0]] = PAD_MODE_SLICER;
				pad_mode_leds_refresh(g_side_deck[0]);
				pad_leds_refresh(g_side_deck[0]);
			}
			break;
		case MACT_PAD_MODE_CUES_B:
			g_pad_mode[g_side_deck[1]] = PAD_MODE_HOTCUE;
			pad_mode_leds_refresh(g_side_deck[1]);
			pad_leds_refresh(g_side_deck[1]);
			break;
		case MACT_PAD_MODE_SAMPLER_B:
			g_pad_mode[g_side_deck[1]] = PAD_MODE_SAMPLER;
			pad_mode_leds_refresh(g_side_deck[1]);
			pad_leds_refresh(g_side_deck[1]);
			break;
		case MACT_PAD_MODE_SLICER_B:
			if (g_opts.enable_slicer) {
				g_pad_mode[g_side_deck[1]] = PAD_MODE_SLICER;
				pad_mode_leds_refresh(g_side_deck[1]);
				pad_leds_refresh(g_side_deck[1]);
			}
			break;
		case MACT_PAD_MODE_AUTOROLL_A: {
			int dk = g_side_deck[0];
			if (g_pad_shift[dk] ||
			    g_pad_mode[dk] == PAD_MODE_AUTOLOOP)
				g_pad_mode[dk] = PAD_MODE_ROLL;
			else
				g_pad_mode[dk] = PAD_MODE_AUTOLOOP;
			pad_mode_leds_refresh(dk);
			pad_leds_refresh(dk);
			break;
		}
		case MACT_PAD_MODE_AUTOROLL_B: {
			int dk = g_side_deck[1];
			if (g_pad_shift[dk] ||
			    g_pad_mode[dk] == PAD_MODE_AUTOLOOP)
				g_pad_mode[dk] = PAD_MODE_ROLL;
			else
				g_pad_mode[dk] = PAD_MODE_AUTOLOOP;
			pad_mode_leds_refresh(dk);
			pad_leds_refresh(dk);
			break;
		}
		case MACT_PAD_MODE_MANUAL_A:
			g_pad_mode[g_side_deck[0]] = PAD_MODE_MANUALLOOP;
			pad_mode_leds_refresh(g_side_deck[0]);
			pad_leds_refresh(g_side_deck[0]);
			break;
		case MACT_PAD_MODE_MANUAL_B:
			g_pad_mode[g_side_deck[1]] = PAD_MODE_MANUALLOOP;
			pad_mode_leds_refresh(g_side_deck[1]);
			pad_leds_refresh(g_side_deck[1]);
			break;
#define PAD_HANDLER(MACT_BASE, DECK_IDX)                                       \
	case MACT_BASE + 0:                                                    \
	case MACT_BASE + 1:                                                    \
	case MACT_BASE + 2:                                                    \
	case MACT_BASE + 3:                                                    \
	case MACT_BASE + 4:                                                    \
	case MACT_BASE + 5:                                                    \
	case MACT_BASE + 6:                                                    \
	case MACT_BASE + 7: {                                                  \
		int pad = (int)(act - MACT_BASE) + 1;                          \
		int dk = (DECK_IDX);                                           \
		if (dk >= g_num_tracks)                                        \
			break;                                                 \
		Track *tp = &g_tracks[dk];                                     \
		int mode = g_pad_mode[dk];                                     \
		if (mode == PAD_MODE_HOTCUE) {                                 \
			int ci = pad - 1;                                      \
			if (g_pad_shift[dk]) {                                 \
				tp->cue[ci] = 0;                               \
				tp->cue_set[ci] = 0;                           \
				sidecar_save(tp);                              \
				char ln[32];                                   \
				snprintf(ln, sizeof(ln), "led_cue_%d_%c", pad, \
					 dk == g_side_deck[0] ? 'a' : 'b');    \
				led_off(ln);                                   \
				pad_leds_refresh(dk);                          \
			} else if (tp->cue_set[ci]) {                          \
				pthread_mutex_lock(&tp->lock);                 \
				tp->pos = tp->cue[ci];                         \
				pthread_mutex_unlock(&tp->lock);               \
			} else if (tp->loaded) {                               \
				tp->cue[ci] = tp->pos;                         \
				tp->cue_set[ci] = 1;                           \
				sidecar_save(tp);                              \
				char ln[32];                                   \
				snprintf(ln, sizeof(ln), "led_cue_%d_%c", pad, \
					 dk == g_side_deck[0] ? 'a' : 'b');    \
				led_on(ln);                                    \
				pad_leds_refresh(dk);                          \
			}                                                      \
		} else if (mode == PAD_MODE_AUTOLOOP && pad <= 4) {            \
			static const float szs[4] = { 1.0f, 2.0f, 4.0f,        \
						      8.0f };                  \
			autoloop_engage(dk, szs[pad - 1]);                     \
		} else if (mode == PAD_MODE_ROLL && pad <= 4) {                \
			static const float szs[4] = { 1.0f, 2.0f, 4.0f,        \
						      8.0f };                  \
			roll_begin(dk, pad, szs[pad - 1]);                     \
		} else if (mode == PAD_MODE_MANUALLOOP) {                      \
			if (pad == 1) {                                        \
				tp->loop_start = tp->pos;                      \
				pad_leds_refresh(dk);                          \
			} else if (pad == 2 && tp->loop_start < tp->pos) {     \
				pthread_mutex_lock(&tp->lock);                 \
				tp->loop_end = tp->pos;                        \
				tp->looping = 1;                               \
				pthread_mutex_unlock(&tp->lock);               \
				pad_leds_refresh(dk);                          \
			} else if (pad == 3 && tp->looping) {                  \
				pthread_mutex_lock(&tp->lock);                 \
				uint32_t half =                                \
					(tp->loop_end - tp->loop_start) / 2;   \
				if (half >= 1)                                 \
					tp->loop_end = tp->loop_start + half;  \
				pthread_mutex_unlock(&tp->lock);               \
			} else if (pad == 4 && tp->looping) {                  \
				pthread_mutex_lock(&tp->lock);                 \
				uint32_t len = tp->loop_end - tp->loop_start;  \
				uint32_t newend = tp->loop_start + len * 2;    \
				if (newend <= tp->num_frames)                  \
					tp->loop_end = newend;                 \
				pthread_mutex_unlock(&tp->lock);               \
			} else if (pad == 5) {                                 \
				pthread_mutex_lock(&tp->lock);                 \
				tp->looping = !tp->looping;                    \
				if (tp->looping && tp->pos >= tp->loop_end)    \
					tp->pos = tp->loop_start;              \
				pthread_mutex_unlock(&tp->lock);               \
				pad_leds_refresh(dk);                          \
			}                                                      \
		} else if (mode == PAD_MODE_SAMPLER) {                         \
			int si = pad - 1;                                      \
			SamplerSlot *s = &g_samplers[si];                      \
			pthread_mutex_lock(&s->lock);                          \
			if (s->data) {                                         \
				s->pos = 0;                                    \
				s->playing = 1;                                \
			}                                                      \
			pthread_mutex_unlock(&s->lock);                        \
		} else if (mode == PAD_MODE_SLICER) {                          \
			float beat_f =                                         \
				(float)g_actual_sample_rate * 60.0f / tp->bpm; \
			uint32_t domain_start =                                \
				(uint32_t)(floorf((tp->slip_pos -              \
						   tp->bpm_offset) /           \
						  (beat_f * 8.0f)) *           \
						   (beat_f * 8.0f) +           \
					   tp->bpm_offset);                    \
			uint32_t target =                                      \
				domain_start + (uint32_t)((pad - 1) * beat_f); \
			pthread_mutex_lock(&tp->lock);                         \
			tp->pos = target;                                      \
			pthread_mutex_unlock(&tp->lock);                       \
		}                                                              \
		break;                                                         \
	}
			PAD_HANDLER(MACT_PAD_1_A, g_side_deck[0])
			PAD_HANDLER(MACT_PAD_1_B, g_side_deck[1])
#undef PAD_HANDLER
			break;
#define PARAM_HANDLER(LEFT_ACT, RIGHT_ACT, DECK_IDX)           \
	case LEFT_ACT: {                                       \
		int dk = (DECK_IDX);                           \
		if (dk >= g_num_tracks)                        \
			break;                                 \
		if (g_pad_mode[dk] == PAD_MODE_AUTOLOOP ||     \
		    g_pad_mode[dk] == PAD_MODE_ROLL) {         \
			float nb = g_autoloop_bars[dk] * 0.5f; \
			if (nb < 0.125f)                       \
				nb = 0.125f;                   \
			g_autoloop_bars[dk] = nb;              \
			if (g_tracks[dk].looping)              \
				autoloop_engage(dk, nb);       \
			pad_leds_refresh(dk);                  \
		}                                              \
		break;                                         \
	}                                                      \
	case RIGHT_ACT: {                                      \
		int dk = (DECK_IDX);                           \
		if (dk >= g_num_tracks)                        \
			break;                                 \
		if (g_pad_mode[dk] == PAD_MODE_AUTOLOOP ||     \
		    g_pad_mode[dk] == PAD_MODE_ROLL) {         \
			float nb = g_autoloop_bars[dk] * 2.0f; \
			if (nb > 64.0f)                        \
				nb = 64.0f;                    \
			g_autoloop_bars[dk] = nb;              \
			if (g_tracks[dk].looping)              \
				autoloop_engage(dk, nb);       \
			pad_leds_refresh(dk);                  \
		}                                              \
		break;                                         \
	}
			PARAM_HANDLER(MACT_PARAM_LEFT_A, MACT_PARAM_RIGHT_A,
				      g_side_deck[0])
			PARAM_HANDLER(MACT_PARAM_LEFT_B, MACT_PARAM_RIGHT_B,
				      g_side_deck[1])
#undef PARAM_HANDLER
#define FX_SLOT_TOGGLE(dk, sl)                                   \
	if (g_pad_shift[dk]) {                                   \
		int nt = (fx_slot(dk, sl)->type + 1) % FX_COUNT; \
		if (nt == FX_NONE)                               \
			nt = 1;                                  \
		g_fx_last_type[dk][sl] = nt;                     \
		fx_set_type(dk, sl, nt);                         \
		if (fx_slot(dk, sl)->params[3] < 0.3f)           \
			fx_set_wet(dk, sl, 0.5f);                \
	} else if (fx_slot(dk, sl)->type == FX_NONE) {           \
		int last = g_fx_last_type[dk][sl];               \
		if (last <= FX_NONE || last >= FX_COUNT)         \
			last = 1;                                \
		fx_set_type(dk, sl, last);                       \
		if (fx_slot(dk, sl)->params[3] < 0.01f)          \
			fx_set_wet(dk, sl, 0.5f);                \
	} else {                                                 \
		g_fx_last_type[dk][sl] = fx_slot(dk, sl)->type;  \
		fx_set_type(dk, sl, FX_NONE);                    \
	}
#define FX_BTN_HANDLER(BTN1, BTN2, BTN3, DECK_IDX)                            \
	case BTN1: {                                                          \
		int dk = (DECK_IDX);                                          \
		g_fx_ui_slot[dk] = 0;                                         \
		FX_SLOT_TOGGLE(dk, 0)                                         \
		snprintf(g_fb_status, sizeof(g_fb_status), "Deck %c FX1: %s", \
			 DECK_NUM(dk),                                        \
			 fx_names[fx_slot(dk, 0)->pending_type >= 0 ?         \
					  fx_slot(dk, 0)->pending_type :      \
					  fx_slot(dk, 0)->type]);             \
		fx_leds_refresh(dk);                                          \
		break;                                                        \
	}                                                                     \
	case BTN2: {                                                          \
		int dk = (DECK_IDX);                                          \
		g_fx_ui_slot[dk] = 1;                                         \
		FX_SLOT_TOGGLE(dk, 1)                                         \
		snprintf(g_fb_status, sizeof(g_fb_status), "Deck %c FX2: %s", \
			 DECK_NUM(dk),                                        \
			 fx_names[fx_slot(dk, 1)->pending_type >= 0 ?         \
					  fx_slot(dk, 1)->pending_type :      \
					  fx_slot(dk, 1)->type]);             \
		fx_leds_refresh(dk);                                          \
		break;                                                        \
	}                                                                     \
	case BTN3: {                                                          \
		int dk = (DECK_IDX);                                          \
		g_fx_ui_slot[dk] = 2;                                         \
		FX_SLOT_TOGGLE(dk, 2)                                         \
		snprintf(g_fb_status, sizeof(g_fb_status), "Deck %c FX3: %s", \
			 DECK_NUM(dk),                                        \
			 fx_names[fx_slot(dk, 2)->pending_type >= 0 ?         \
					  fx_slot(dk, 2)->pending_type :      \
					  fx_slot(dk, 2)->type]);             \
		fx_leds_refresh(dk);                                          \
		break;                                                        \
	}
		case MACT_TAP_BPM_A:
		case MACT_TAP_BPM_B: {
			int deck = act - MACT_TAP_BPM_A;
			if (g_pad_shift[deck])
				snap_grid(deck);
			else
				tap_bpm(deck);
			break;
		}
		case MACT_GRID_SNAP_A:
		case MACT_GRID_SNAP_B: {
			int deck = act - MACT_GRID_SNAP_A;
			snap_grid(deck);
			break;
		}
		case MACT_EQ_LOW_KILL_A:
		case MACT_EQ_LOW_KILL_B:
		case MACT_EQ_LOW_KILL_C:
		case MACT_EQ_LOW_KILL_D: {
			int dk = act - MACT_EQ_LOW_KILL_A;
			if (g_touch_mode) {
				g_eq_low_kill[dk] = 1;
				g_tracks[dk].eq_low = -1.0f;
			}
			break;
		}
		case MACT_EQ_MID_KILL_A:
		case MACT_EQ_MID_KILL_B:
		case MACT_EQ_MID_KILL_C:
		case MACT_EQ_MID_KILL_D: {
			int dk = act - MACT_EQ_MID_KILL_A;
			if (g_touch_mode) {
				g_eq_mid_kill[dk] = 1;
				g_tracks[dk].eq_mid = -1.0f;
			}
			break;
		}
		case MACT_EQ_HIGH_KILL_A:
		case MACT_EQ_HIGH_KILL_B:
		case MACT_EQ_HIGH_KILL_C:
		case MACT_EQ_HIGH_KILL_D: {
			int dk = act - MACT_EQ_HIGH_KILL_A;
			if (g_touch_mode) {
				g_eq_high_kill[dk] = 1;
				g_tracks[dk].eq_high = -1.0f;
			}
			break;
		}
		case MACT_FILTER_ROLL_TOUCH_A:
		case MACT_FILTER_ROLL_TOUCH_B:
		case MACT_FILTER_ROLL_TOUCH_C:
		case MACT_FILTER_ROLL_TOUCH_D: {
			int dk = act - MACT_FILTER_ROLL_TOUCH_A;
			if (!g_filter_roll_held[dk]) {
				g_filter_was_on[dk] = g_filter_on[dk];
				g_filter_roll_held[dk] = 1;
			}
			g_filter_on[dk] = 1;
			{
				char ln[32];
				snprintf(ln, sizeof(ln), "led_filter_%c",
					 'a' + dk);
				led_on(ln);
			}
			break;
		}
		case MACT_TOUCH_MODE_TOGGLE: {
			g_touch_mode ^= 1;
			if (g_touch_mode) {
				led_on("led_touch_mode");
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Touch mode ON (capacitive EQ kills)");
			} else {
				led_off("led_touch_mode");
				snprintf(g_fb_status, sizeof(g_fb_status),
					 "Touch mode OFF");
				for (int i = 0; i < MAX_TRACKS; i++) {
					g_eq_low_kill[i] = 0;
					g_eq_mid_kill[i] = 0;
					g_eq_high_kill[i] = 0;
					g_tracks[i].eq_low = g_eq_low_knob[i];
					g_tracks[i].eq_mid = g_eq_mid_knob[i];
					g_tracks[i].eq_high = g_eq_high_knob[i];
				}
			}
			break;
		}
			FX_BTN_HANDLER(MACT_FX_BTN_1_A, MACT_FX_BTN_2_A,
				       MACT_FX_BTN_3_A, g_side_deck[0])
			FX_BTN_HANDLER(MACT_FX_BTN_1_B, MACT_FX_BTN_2_B,
				       MACT_FX_BTN_3_B, g_side_deck[1])
#undef FX_BTN_HANDLER
#undef FX_SLOT_TOGGLE
		default:
			break;
		}
	}

	if (type == 0x80 || (type == 0x90 && data2 == 0)) {
		switch (act) {
		case MACT_JOG_TOUCH_A:
			g_jog_touched[0] = 0;
			g_jog_nudge[0] = 0;
			g_last_applied_nudge[0] = 0;
			if (!g_motor_running[0])
				g_motor_vel[0] = 0.0f;
			break;
		case MACT_JOG_TOUCH_B:
			g_jog_touched[1] = 0;
			g_jog_nudge[1] = 0;
			g_last_applied_nudge[1] = 0;
			if (!g_motor_running[1])
				g_motor_vel[1] = 0.0f;
			break;
		case MACT_JOG_TOUCH_C:
			g_jog_touched[2] = 0;
			g_jog_nudge[2] = 0;
			g_last_applied_nudge[2] = 0;
			if (!g_motor_running[2])
				g_motor_vel[2] = 0.0f;
			break;
		case MACT_JOG_TOUCH_D:
			g_jog_touched[3] = 0;
			g_jog_nudge[3] = 0;
			g_last_applied_nudge[3] = 0;
			if (!g_motor_running[3])
				g_motor_vel[3] = 0.0f;
			break;
		case MACT_LIB_ENCODER_TOUCH: {
			g_lib_touched = (data2 > 0);
			struct timespec _lts3;
			clock_gettime(CLOCK_MONOTONIC, &_lts3);
			g_lib_enc_last_ms = (int64_t)_lts3.tv_sec * 1000 +
					    _lts3.tv_nsec / 1000000;
			break;
		}
		case MACT_SHIFT_A:
			g_pad_shift[g_side_deck[0]] = 0;
			break;
		case MACT_SHIFT_B:
			g_pad_shift[g_side_deck[1]] = 0;
			break;
		case MACT_CENSOR_A:
		case MACT_CENSOR_B:
		case MACT_CENSOR_C:
		case MACT_CENSOR_D: {
			int deck = act - MACT_CENSOR_A;
			if (!g_censor_held[deck] || deck >= 4)
				break;
			g_censor_held[deck] = 0;
			g_tracks[deck].reverse = 0;
			if (g_motor_running[deck])
				motor_set_direction(deck, 0);
			pthread_mutex_lock(&g_tracks[deck].lock);
			g_tracks[deck].pos = g_censor_save_pos[deck];
			pthread_mutex_unlock(&g_tracks[deck].lock);
			break;
		}
		case MACT_FILTER_ROLL_TOUCH_A:
		case MACT_FILTER_ROLL_TOUCH_B:
		case MACT_FILTER_ROLL_TOUCH_C:
		case MACT_FILTER_ROLL_TOUCH_D: {
			int dk = act - MACT_FILTER_ROLL_TOUCH_A;
			g_filter_on[dk] = g_filter_was_on[dk];
			g_filter_roll_held[dk] = 0;
			{
				char ln[32];
				snprintf(ln, sizeof(ln), "led_filter_%c",
					 'a' + dk);
				if (g_filter_on[dk])
					led_on(ln);
				else
					led_off(ln);
			}
			break;
		}
		case MACT_EQ_LOW_KILL_A:
		case MACT_EQ_LOW_KILL_B:
		case MACT_EQ_LOW_KILL_C:
		case MACT_EQ_LOW_KILL_D: {
			int dk = act - MACT_EQ_LOW_KILL_A;
			if (g_touch_mode) {
				g_eq_low_kill[dk] = 0;
				g_tracks[dk].eq_low = g_eq_low_knob[dk];
			}
			break;
		}
		case MACT_EQ_MID_KILL_A:
		case MACT_EQ_MID_KILL_B:
		case MACT_EQ_MID_KILL_C:
		case MACT_EQ_MID_KILL_D: {
			int dk = act - MACT_EQ_MID_KILL_A;
			if (g_touch_mode) {
				g_eq_mid_kill[dk] = 0;
				g_tracks[dk].eq_mid = g_eq_mid_knob[dk];
			}
			break;
		}
		case MACT_EQ_HIGH_KILL_A:
		case MACT_EQ_HIGH_KILL_B:
		case MACT_EQ_HIGH_KILL_C:
		case MACT_EQ_HIGH_KILL_D: {
			int dk = act - MACT_EQ_HIGH_KILL_A;
			if (g_touch_mode) {
				g_eq_high_kill[dk] = 0;
				g_tracks[dk].eq_high = g_eq_high_knob[dk];
			}
			break;
		}
		case MACT_CUE_DEFAULT_A: {
			int dk = g_side_deck[0];
			if (g_cue_default_held[dk] && dk < 4) {
				g_cue_default_held[dk] = 0;
				g_tracks[dk].playing = 0;
				motor_set(dk, 0);
				pthread_mutex_lock(&g_tracks[dk].lock);
				if (g_cue_default_set[dk])
					g_tracks[dk].pos =
						g_cue_default_pos[dk];
				pthread_mutex_unlock(&g_tracks[dk].lock);
				deck_leds_refresh();
			}
			break;
		}
		case MACT_CUE_DEFAULT_B: {
			int dk = g_side_deck[1];
			if (g_cue_default_held[dk] && dk < 4) {
				g_cue_default_held[dk] = 0;
				g_tracks[dk].playing = 0;
				motor_set(dk, 0);
				pthread_mutex_lock(&g_tracks[dk].lock);
				if (g_cue_default_set[dk])
					g_tracks[dk].pos =
						g_cue_default_pos[dk];
				pthread_mutex_unlock(&g_tracks[dk].lock);
				deck_leds_refresh();
			}
			break;
		}
		case MACT_PAD_1_A:
		case MACT_PAD_2_A:
		case MACT_PAD_3_A:
		case MACT_PAD_4_A:
		case MACT_PAD_5_A:
		case MACT_PAD_6_A:
		case MACT_PAD_7_A:
		case MACT_PAD_8_A: {
			int dk = g_side_deck[0];
			if (g_pad_mode[dk] == PAD_MODE_SLICER) {
				Track *tp = &g_tracks[dk];
				pthread_mutex_lock(&tp->lock);
				tp->pos = (uint32_t)tp->slip_pos;
				pthread_mutex_unlock(&tp->lock);
			}
			if (g_pad_mode[dk] == PAD_MODE_ROLL)
				roll_end(dk);
			break;
		}
		case MACT_PAD_1_B:
		case MACT_PAD_2_B:
		case MACT_PAD_3_B:
		case MACT_PAD_4_B:
		case MACT_PAD_5_B:
		case MACT_PAD_6_B:
		case MACT_PAD_7_B:
		case MACT_PAD_8_B: {
			int dk = g_side_deck[1];
			if (g_pad_mode[dk] == PAD_MODE_SLICER) {
				Track *tp = &g_tracks[dk];
				pthread_mutex_lock(&tp->lock);
				tp->pos = (uint32_t)tp->slip_pos;
				pthread_mutex_unlock(&tp->lock);
			}
			if (g_pad_mode[dk] == PAD_MODE_ROLL)
				roll_end(dk);
			break;
		}
		default:
			break;
		}
	}
}

void *midi_thread(void *arg)
{
	(void)arg;
	uint8_t running_status = 0;
	uint8_t msg[3] = { 0 };
	int msg_len = 0;
	int msg_pos = 0;
	int in_sysex = 0;
	snd_rawmidi_t *last_handle = NULL;
	while (g_running) {
		if (!g_midi_in) {
			usleep(20000);
			continue;
		}
		if (g_midi_in != last_handle) {
			last_handle = g_midi_in;
			running_status = 0;
			msg_len = 0;
			msg_pos = 0;
			in_sysex = 0;
		}
		snd_rawmidi_t *h = g_midi_in;
		if (!h) {
			usleep(500);
			continue;
		}
		int count = snd_rawmidi_poll_descriptors_count(h);
		struct pollfd *pfds = alloca(sizeof(struct pollfd) * count);
		snd_rawmidi_poll_descriptors(h, pfds, count);
		if (poll(pfds, count, 500) <= 0)
			continue;
		uint8_t b;
		int r = snd_rawmidi_read(h, &b, 1);
		if (r != 1)
			continue;
		if (b >= 0xF8)
			continue;
		if (b == 0xF0) {
			in_sysex = 1;
			continue;
		}
		if (in_sysex) {
			if (b == 0xF7)
				in_sysex = 0;
			continue;
		}
		if (b & 0x80) {
			uint8_t type = b & 0xF0;
			if (type == 0x80 || type == 0x90 || type == 0xA0 ||
			    type == 0xB0 || type == 0xE0)
				msg_len = 3;
			else if (type == 0xC0 || type == 0xD0)
				msg_len = 2;
			else {
				msg_len = 0;
				msg_pos = 0;
				running_status = 0;
				continue;
			}
			running_status = b;
			msg[0] = b;
			msg_pos = 1;
			continue;
		}
		if (msg_len == 0) {
			if (running_status == 0)
				continue;
			msg[0] = running_status;
			msg_pos = 1;
			msg_len = ((running_status & 0xF0) == 0xC0 ||
				   (running_status & 0xF0) == 0xD0) ?
					  2 :
					  3;
		}
		if (msg_pos == 0) {
			msg[0] = running_status;
			msg_pos = 1;
		}
		msg[msg_pos++] = b;
		if (msg_pos == msg_len) {
			if (msg_len == 3)
				handle_midi(msg[0], msg[1], msg[2]);
			else if (msg_len == 2)
				handle_midi(msg[0], msg[1], 0);
			msg_pos = 1;
		}
	}
	return NULL;
}

static void ns7iii_update_jog(int deck, int coarse, int fine)
{
	if (deck < 0 || deck >= MAX_TRACKS)
		return;
	{
		struct timespec _ts;
		clock_gettime(CLOCK_MONOTONIC, &_ts);
		g_jog_last_msg_ms[deck] =
			(int64_t)_ts.tv_sec * 1000 + _ts.tv_nsec / 1000000;
	}
	float pos =
		((float)coarse * (float)JOG_NS7III_FINE_RANGE + (float)fine) /
		((float)JOG_NS7III_STEPS * (float)JOG_NS7III_FINE_RANGE);
	if (!g_jog_abs_init[deck]) {
		g_jog_abs_pos[deck] = pos;
		g_jog_abs_init[deck] = 1;
		g_jog_abs_vel[deck] = 0.0f;
		return;
	}
	float delta = pos - g_jog_abs_pos[deck];
	if (delta > 0.5f)
		delta -= 1.0f;
	if (delta < -0.5f)
		delta += 1.0f;
	g_jog_abs_pos[deck] = pos;
	if (!g_jog_touched[deck]) {
		float spike_thresh = g_jog_ref_delta * 8.0f;
		if (fabsf(delta) > spike_thresh)
			return;
	}
	float smooth_alpha = g_jog_smooth_alpha;
	if ((delta < 0.0f && g_motor_vel[deck] > 0.25f) ||
	    (delta > 0.0f && g_motor_vel[deck] < -0.25f))
		smooth_alpha = 0.8f;
	g_jog_abs_vel[deck] = g_jog_abs_vel[deck] * (1.0f - smooth_alpha) +
			      delta * smooth_alpha;
	if (g_motor_running[deck]) {
		if (g_motor_settle_until[deck] == 0) {
			float raw_vel;
			if (g_jog_touched[deck]) {
				if (g_pll_enabled) {
					float pll_err =
						(float)((double)delta -
							g_pll[deck].freq);
					raw_vel = pll_err /
						  (float)g_pll[deck].freq;
				} else {
					raw_vel = (delta / g_jog_ref_delta) -
						  1.0f;
				}
				if (raw_vel > g_jog_vel_max)
					raw_vel = g_jog_vel_max;
				if (raw_vel < -g_jog_vel_max)
					raw_vel = -g_jog_vel_max;
				g_motor_vel[deck] = raw_vel;
			} else if (g_pll_enabled) {
				PLLState *pll = &g_pll[deck];
				double err = (double)delta - pll->freq;
				pll->integrator += err;
				double int_clamp =
					(double)g_jog_ref_delta * 0.20;
				if (pll->integrator > int_clamp)
					pll->integrator = int_clamp;
				if (pll->integrator < -int_clamp)
					pll->integrator = -int_clamp;
				pll->freq += (double)g_pll_kp * err +
					     (double)g_pll_ki * pll->integrator;
				if (g_opts.vinyl_mode) {
					g_motor_vel[deck] = 0.0f;
				} else {
					float pll_err2 = (float)((double)delta -
								 pll->freq);
					float band2 = g_pll_bandwidth *
						      g_jog_ref_delta;
					raw_vel =
						(fabsf(pll_err2) < band2) ?
							0.0f :
							pll_err2 /
								(float)pll
									->freq;
					if (raw_vel > g_jog_vel_max)
						raw_vel = g_jog_vel_max;
					if (raw_vel < -g_jog_vel_max)
						raw_vel = -g_jog_vel_max;
					float diff2 =
						raw_vel - g_motor_vel[deck];
					float alpha2 = (fabsf(diff2) > 0.5f) ?
							       0.55f :
							       0.15f;
					g_motor_vel[deck] =
						g_motor_vel[deck] *
							(1.0f - alpha2) +
						raw_vel * alpha2;
				}
			} else {
				if (g_jog_type == JOG_NS7III) {
					if (g_opts.vinyl_mode) {
						g_motor_vel[deck] = 0.0f;
					} else {
						raw_vel = (delta /
							   g_jog_ref_delta) -
							  1.0f;
						if (raw_vel > g_jog_motor_dead)
							raw_vel -=
								g_jog_motor_dead;
						else if (raw_vel <
							 -g_jog_motor_dead)
							raw_vel +=
								g_jog_motor_dead;
						else
							raw_vel = 0.0f;
						if (raw_vel > g_jog_vel_max)
							raw_vel = g_jog_vel_max;
						if (raw_vel < -g_jog_vel_max)
							raw_vel =
								-g_jog_vel_max;
						g_motor_vel[deck] = raw_vel;
					}
				} else {
					raw_vel = (g_jog_abs_vel[deck] /
						   g_jog_ref_delta) -
						  1.0f;
					if (raw_vel > g_jog_motor_dead)
						raw_vel -= g_jog_motor_dead;
					else if (raw_vel < -g_jog_motor_dead)
						raw_vel += g_jog_motor_dead;
					else
						raw_vel = 0.0f;
					if (raw_vel > g_jog_vel_max)
						raw_vel = g_jog_vel_max;
					if (raw_vel < -g_jog_vel_max)
						raw_vel = -g_jog_vel_max;
					float diff =
						raw_vel - g_motor_vel[deck];
					float alpha = (fabsf(diff) > 0.5f) ?
							      0.55f :
							      0.15f;
					g_motor_vel[deck] =
						g_motor_vel[deck] *
							(1.0f - alpha) +
						raw_vel * alpha;
				}
			}
		}
	} else {
		if (g_jog_touched[deck]) {
			float raw_vel = (delta / g_jog_ref_delta) - 1.0f;
			if (raw_vel > g_jog_vel_max)
				raw_vel = g_jog_vel_max;
			if (raw_vel < -g_jog_vel_max)
				raw_vel = -g_jog_vel_max;
			g_motor_vel[deck] = raw_vel;
		} else {
			float nudge_step = g_jog_abs_vel[deck] * 8.0f;
			g_jog_nudge[deck] += nudge_step;
			if (g_jog_nudge[deck] > 0.15f)
				g_jog_nudge[deck] = 0.15f;
			if (g_jog_nudge[deck] < -0.15f)
				g_jog_nudge[deck] = -0.15f;
		}
	}
}

void motor_ramp_table_build(void)
{
	int n = MOTOR_RAMP_STEPS;
	if (n < 1)
		n = 1;
	if (n > 64)
		n = 64;
	for (int i = 0; i < n; i++)
		g_motor_ramp_table[i] = (int)(i * 100.0f / (n - 1) + 0.5f);
	g_motor_ramp_table[n - 1] = 100;
}

static void *motor_thread(void *arg)
{
	(void)arg;
	motor_ramp_table_build();
	static const int motor_ch[MAX_TRACKS] = {
		CFG_MOTOR_CH_A, CFG_MOTOR_CH_B, CFG_MOTOR_CH_C, CFG_MOTOR_CH_D
	};

	while (g_running) {
		usleep(1000000 / 20);
		if (!g_midi_out)
			continue;

		for (int dk = 0; dk < MAX_TRACKS; dk++) {
			if (g_motor_pending_start[dk]) {
				g_motor_pending_start[dk] = 0;
				if (g_tracks[dk].playing &&
				    !g_slip_motor_off[dk])
					motor_set(dk, 1);
			}
		}
		for (int dk = 0; dk < MAX_TRACKS; dk++) {
			if (g_motor_running[dk] && !g_tracks[dk].playing)
				motor_set(dk, 0);
		}

		for (int dk = 0; dk < MAX_TRACKS; dk++) {
			if (g_motor_running[dk] &&
			    g_motor_ramp_step[dk] >= MOTOR_RAMP_STEPS)
				motor_sync_pitch(dk);
		}

		int any_ramping = 0;
		for (int dk = 0; dk < MAX_TRACKS; dk++) {
			if (!g_motor_running[dk])
				continue;
			int step = g_motor_ramp_step[dk];
			if (step >= MOTOR_RAMP_STEPS)
				continue;

			int ch = motor_ch[dk];
			if (ch == 0)
				continue;
			any_ramping = 1;
			midi_send_cc(ch, MOTOR_RAMP_CC,
				     g_motor_ramp_table[step]);
			if (step + 1 >= MOTOR_RAMP_STEPS)
				motor_sync_pitch(dk);
			else
				midi_send_cc(ch, MOTOR_PITCH_CC, 64);
			g_motor_ramp_step[dk] = step + 1;
		}
		if (any_ramping) {
			midi_send_cc(1, MOTOR_SAW_CC, g_motor_saw_phase);
			g_motor_saw_phase =
				(g_motor_saw_phase + 1) % (MOTOR_SAW_MAX + 1);
		}

		if (!any_ramping) {
			float level = g_vu_l > g_vu_r ? g_vu_l : g_vu_r;
			if (level > 0.0f) {
				float db = log10f(level) * 20.0f;
				float norm = (db + 40.0f) / 40.0f;
				if (norm < 0.0f)
					norm = 0.0f;
				if (norm > 1.0f)
					norm = 1.0f;
				level = norm;
			}
			int vu_val = (int)(level * 127.0f + 0.5f);
			midi_send_cc(1, MOTOR_SAW_CC, vu_val);
		}
	}
	return NULL;
}

void motor_set(int deck, int on)
{
	if (deck < 0 || deck >= MAX_TRACKS)
		return;
	if (!on) {
		g_motor_settle_until[deck] = 0;
		g_motor_vel[deck] = 0.0f;
		g_jog_abs_vel[deck] = 0.0f;
		g_jog_abs_init[deck] = 0;
		g_pll[deck].freq = (double)g_jog_ref_delta;
		g_pll[deck].integrator = 0.0;
	}
	if (on && g_motor_running[deck])
		return;
	g_motor_running[deck] = on;
	if (on) {
		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC, &ts);
		int64_t now_ms =
			(int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
		g_motor_settle_until[deck] = now_ms + (int64_t)g_jog_settle_ms;
		g_jog_abs_init[deck] = 0;
		g_jog_abs_vel[deck] = 0.0f;
		g_pb_streak[deck] = 0;
		g_pb_mag_acc[deck] = 0.0f;
		g_motor_ramp_step[deck] = 0;
		g_pll[deck].freq = (double)g_jog_ref_delta;
		g_pll[deck].integrator = 0.0;
	}
	{
		char led_name[32];
		snprintf(led_name, sizeof(led_name), "led_deck_%c", 'a' + deck);
		if (on)
			led_on(led_name);
		else
			led_off(led_name);
	}
	if (!g_midi_out)
		return;
	static const int motor_ch[MAX_TRACKS] = {
		CFG_MOTOR_CH_A, CFG_MOTOR_CH_B, CFG_MOTOR_CH_C, CFG_MOTOR_CH_D
	};
	int ch = motor_ch[deck];
	if (ch == 0)
		return;
	if (on) {
		static int global_init_sent = 0;
		if (!global_init_sent) {
			global_init_sent = 1;
			midi_send_cc(1, 71, 0);
			midi_send_cc(1, MOTOR_SAW_CC, 0);
			midi_send_cc(1, MOTOR_ENABLE_CC, 0);
			midi_send_cc(1, 76, 0);
			midi_send_cc(1, 77, 0);
			midi_send_cc(1, 78, 1);
			midi_send_cc(1, 78, 1);
			midi_send_cc(1, 78, 1);
			midi_send_cc(1, 79, 1);
			midi_send_cc(1, 79, 1);
			midi_send_cc(1, 79, 1);
			midi_send_cc(1, 80, 1);
			midi_send_cc(1, 80, 1);
			midi_send_cc(1, 80, 1);
			midi_send_cc(1, 81, 7);
			midi_send_cc(1, 81, 7);
			midi_send_cc(1, 82, 1);
			midi_send_cc(1, 82, 1);
			midi_send_cc(1, 82, 1);
			midi_send_cc(1, 83, 1);
			midi_send_cc(1, 83, 1);
			midi_send_cc(1, 83, 1);
			midi_send_cc(1, 84, 1);
			midi_send_cc(1, 84, 1);
			midi_send_cc(1, 84, 1);
			midi_send_cc(1, 85, 7);
			midi_send_cc(1, 85, 7);
			usleep(20000);
		}
		midi_send_cc(1, MOTOR_ENABLE_CC, 0);
		midi_send_cc(ch, 65, 127);
		if (g_tracks[deck].reverse)
			midi_send_cc(ch, 70, 1);
		else
			midi_send_cc(ch, 69, 0);
	} else {
		midi_send_cc(ch, 66, 0);
	}
}

void motor_set_direction(int deck, int reverse)
{
	if (!g_midi_out)
		return;
	static const int motor_ch[MAX_TRACKS] = {
		CFG_MOTOR_CH_A, CFG_MOTOR_CH_B, CFG_MOTOR_CH_C, CFG_MOTOR_CH_D
	};
	int ch = motor_ch[deck];
	if (ch == 0)
		return;
	if (reverse)
		midi_send_cc(ch, 70, 1);
	else
		midi_send_cc(ch, 69, 0);
}

static void motor_sync_pitch(int deck)
{
	if (!g_midi_out || !g_motor_running[deck])
		return;
	static const int motor_ch[MAX_TRACKS] = {
		CFG_MOTOR_CH_A, CFG_MOTOR_CH_B, CFG_MOTOR_CH_C, CFG_MOTOR_CH_D
	};
	int ch = motor_ch[deck];
	if (ch == 0)
		return;
	float eff = g_tracks[deck].pitch - 1.0f;
	float sign = g_tracks[deck].reverse ? 1.0f : -1.0f;
	int rpm = (int)(64.0f + sign * eff * 64.0f + 0.5f);
	if (rpm < 1)
		rpm = 1;
	if (rpm > 127)
		rpm = 127;
	midi_send_cc(ch, MOTOR_PITCH_CC, rpm);
}

void motor_probe_send(int value)
{
	if (!g_midi_out) {
		snprintf(g_motor_probe_log, sizeof(g_motor_probe_log),
			 "No MIDI output -- open a device first");
		return;
	}
	uint8_t status;
	const char *type_str;
	switch (g_motor_probe_type) {
	case 1:
		status = (uint8_t)(0x90 | ((g_motor_probe_ch - 1) & 0x0F));
		type_str = "NoteOn ";
		break;
	case 2:
		status = (uint8_t)(0x80 | ((g_motor_probe_ch - 1) & 0x0F));
		type_str = "NoteOff";
		break;
	default:
		status = (uint8_t)(0xB0 | ((g_motor_probe_ch - 1) & 0x0F));
		type_str = "CC     ";
		break;
	}
	switch (g_motor_probe_type) {
	case 1:
		midi_send_note(g_motor_probe_ch, g_motor_probe_cc, value);
		break;
	case 2:
		midi_send_note(g_motor_probe_ch, g_motor_probe_cc, 0);
		break;
	default:
		midi_send_cc(g_motor_probe_ch, g_motor_probe_cc, value);
		break;
	}
	snprintf(g_motor_probe_log, sizeof(g_motor_probe_log),
		 "Sent %s ch%d  d1=%d d2=%d  raw:%02X %02X %02X", type_str,
		 g_motor_probe_ch, g_motor_probe_cc, value, status,
		 g_motor_probe_cc & 0x7F, value & 0x7F);
}

void motor_probe_sweep(int from, int to)
{
	if (!g_midi_out)
		return;
	int step = (from <= to) ? 1 : -1;
	for (int v = from; v != to + step; v += step) {
		g_motor_probe_val = v;
		motor_probe_send(v);
		usleep(20000);
	}
}

void motor_handoff(int old_dk, int new_dk)
{
	if (old_dk == new_dk)
		return;
	static const int motor_ch[MAX_TRACKS] = {
		CFG_MOTOR_CH_A, CFG_MOTOR_CH_B, CFG_MOTOR_CH_C, CFG_MOTOR_CH_D
	};
	if (old_dk >= 0 && old_dk < MAX_TRACKS) {
		g_slip_motor_off[old_dk] = 1;
		g_jog_touched[old_dk] = 0;
		g_motor_vel[old_dk] = 0.0f;
		int ch_old = motor_ch[old_dk];
		if (ch_old > 0) {
			motor_set(old_dk, 0);
			midi_send_cc(ch_old, 66, 0);
		}
	}
	if (new_dk >= 0 && new_dk < MAX_TRACKS) {
		g_slip_motor_off[new_dk] = 0;
		g_jog_abs_init[new_dk] = 0;
		g_motor_vel[new_dk] = 0.0f;
		if (g_tracks[new_dk].playing)
			g_motor_pending_start[new_dk] = 1;
	}
}

void side_restack(int side, int new_dk)
{
	if (g_jog_type == JOG_NS7III) {
		g_side_deck[side] = new_dk;
		return;
	}

	static const MidiAction platter_groups[] = {
		MACT_DECK_PITCH_A,  MACT_PITCH_LSB_A,  MACT_PLAY_A,
		MACT_LOOP_IN_A,	    MACT_LOOP_OUT_A,   MACT_LOOP_DOUBLE_A,
		MACT_LOOP_HALF_A,   MACT_KEY_LOCK_A,   MACT_SLIP_MODE_A,
		MACT_REVERSE_A,	    MACT_CENSOR_A,     MACT_STRIP_A,
		MACT_JOG_TOUCH_A,   MACT_JOG_SPIN_A,   MACT_JOG_PB_A,
		MACT_PITCH_RANGE_A, MACT_PITCH_BEND_A, MACT_MOTOR_TOGGLE_A,
		MACT_MOTOR_ON_A,    MACT_MOTOR_OFF_A,  MACT_SYNC_FOLLOW_A,
		MACT_NONE /* sentinel */
	};

	int old_dk = g_side_deck[side];
	if (old_dk == new_dk)
		return;

	for (int g = 0; platter_groups[g] != MACT_NONE; g++) {
		MidiAction old_act = (MidiAction)(platter_groups[g] + old_dk);
		MidiAction new_act = (MidiAction)(platter_groups[g] + new_dk);

		for (int i = 0; i < g_midi_nbindings; i++) {
			if (g_midi_bindings[i].action == old_act) {
				uint8_t st = g_midi_bindings[i].status;
				uint8_t d1 = g_midi_bindings[i].data1;
				int rel = g_midi_bindings[i].relative;
				midi_bind(st, d1, new_act);
				if (rel)
					midi_bind_set_relative(new_act, 1);
				break;
			}
		}
	}
}

void midi_init(const char *dev)
{
	midi_enumerate_devices();
	const char *to_open = NULL;
	if (dev && dev[0]) {
		for (int i = 0; i < g_midi_ndevices; i++) {
			if (strcmp(g_midi_devlist[i].dev, dev) == 0) {
				to_open = g_midi_devlist[i].dev;
				g_midi_dev_sel = i;
				break;
			}
		}
	}
	if (!to_open && g_midi_ndevices > 0) {
		to_open = g_midi_devlist[0].dev;
		g_midi_dev_sel = 0;
	}
	if (to_open) {
		midi_open_device(g_midi_dev_sel);
	}

	pthread_t mt, mot;
	pthread_create(&mt, NULL, midi_thread, NULL);
	pthread_create(&mot, NULL, motor_thread, NULL);
}

