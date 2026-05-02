#ifndef DJCMD_MIDI_H
#define DJCMD_MIDI_H

#include "djcmd_shared.h"
#include <stdint.h>
#include <alsa/asoundlib.h>

/* MIDI Constants */
#define MIDI_MAX_DEVICES 16
#define MIDI_MAX_BINDINGS 256
#define MIDI_MAX_OUT_BINDINGS 256

/* Jog Wheel Constants */
#define JOG_NS7III_STEPS 128
#define JOG_NS7III_FINE_RANGE 16384
#define NS7III_PB_RATIO 1440
#define NS7III_SLIP_THRESH 200
#define NS7III_RELEASE_CONF 3

/* Scratch & Nudge Constants */
#define SCRATCH_FRAMES_PER_TICK 8
#define NUDGE_PITCH_PER_TICK 0.004f

/* MIDI & Controller logic */
void handle_midi(uint8_t status, uint8_t data1, uint8_t data2);
void midi_init(const char *dev);

/* ── Promoted from djcmd.c ── */
void deck_leds_refresh(void);
void fx_leds_refresh(int deck);
void pad_leds_refresh(int deck);
void pad_mode_leds_refresh(int deck);
void led_on(const char *name);
void led_off(const char *name);
void motor_set(int deck, int on);
void motor_set_direction(int deck, int reverse);
void motor_handoff(int old_dk, int new_dk);
void autoloop_engage(int deck, float bars);
void side_restack(int side, int new_dk);
void midi_bind(uint8_t status, uint8_t data1, MidiAction action);
void midi_map_save(void);
void midi_open_device(int idx);
void midi_enumerate_devices(void);
void midi_map_name_from_device(const char *full, char *out, size_t max);
void midi_map_path(char *out, size_t max);
void motor_probe_send(int val);
void motor_probe_sweep(int start, int end);

/* ── MIDI Global State ───────────────────────────────────────────────── */
extern snd_rawmidi_t *g_midi_in;
extern snd_rawmidi_t *g_midi_out;
extern MidiBinding g_midi_bindings[MIDI_MAX_BINDINGS];
extern int g_midi_nbindings;
extern const char *g_mact_names[MACT_COUNT];

/* MIDI Monitor state */
extern MidiMonEntry g_midi_mon_buf[MIDI_MON_SIZE];
extern int g_midi_mon_head;
extern int g_midi_mon_count;
extern int g_midi_mon_open;

/* MIDI Learn state */
extern _Atomic int g_midi_learn_active;
extern _Atomic int g_midi_learn_sel;
extern _Atomic int g_midi_learn_jog_pair;
extern _Atomic int g_midi_learn_jog_step;
extern _Atomic int g_midi_learn_jog_deck;
extern _Atomic uint8_t g_midi_learn_jog_spin_status;
extern _Atomic uint8_t g_midi_learn_jog_spin_d1;

/* MIDI Devices */
extern MidiDevice g_midi_devlist[MIDI_MAX_DEVICES];
extern int g_midi_ndevices;
extern int g_midi_dev_sel;
extern char g_midi_dev_str[64];

/* MIDI Output state */
typedef struct {
	char name[64];
	uint8_t status, data1, data2;
	uint8_t sysex[32];
	int sysex_len;
} MidiOutBinding;

extern MidiOutBinding g_midi_out_bindings[MIDI_MAX_OUT_BINDINGS];
extern int g_midi_nout_bindings;

/* Motor Probe state */
extern int g_motor_probe_open;
extern int g_motor_probe_type;
extern int g_motor_probe_ch;
extern int g_motor_probe_cc;
extern int g_motor_probe_val;
extern char g_motor_probe_log[256];

/* Jog Wheel state */
typedef enum { JOG_RELATIVE, JOG_NS7III } JogType;
extern JogType g_jog_type;
extern float g_jog_abs_pos[MAX_TRACKS];
extern _Atomic float g_jog_abs_vel[MAX_TRACKS];
extern int g_pb_streak[MAX_TRACKS];
extern int g_motor_running[MAX_TRACKS];

extern _Atomic float g_motor_vel[MAX_TRACKS];
extern _Atomic float g_jog_nudge[MAX_TRACKS];
extern float g_last_motor_vel[MAX_TRACKS];
extern float g_last_applied_nudge[MAX_TRACKS];
extern _Atomic int g_jog_touched[MAX_TRACKS];
extern float g_scratch_alpha[MAX_TRACKS];
extern uint32_t g_noise_state[MAX_TRACKS];
extern float g_noise_brown_l[MAX_TRACKS];
extern float g_noise_brown_r[MAX_TRACKS];
extern _Atomic int g_motor_pending_start[MAX_TRACKS];
extern int64_t g_motor_settle_until[MAX_TRACKS];

extern int64_t g_jog_last_msg_ms[MAX_TRACKS];
extern float g_jog_ref_delta;
extern float g_jog_motor_dead;
extern float g_jog_vel_max;
extern float g_scratch_lpf_l[MAX_TRACKS];
extern float g_scratch_lpf_r[MAX_TRACKS];
extern float g_scratch_lpf2_l[MAX_TRACKS];
extern float g_scratch_lpf2_r[MAX_TRACKS];

extern int g_pad_mode[MAX_TRACKS];
extern int g_pad_shift[MAX_TRACKS];
extern float g_autoloop_bars[MAX_TRACKS];
extern uint32_t g_roll_resume_pos[MAX_TRACKS];
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

#endif /* DJCMD_MIDI_H */
