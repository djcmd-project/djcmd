#ifndef DJCMD_AUDIO_H
#define DJCMD_AUDIO_H

#include <stdint.h>
#include <pthread.h>
#include "djcmd_config.h"
#include "djcmd_shared.h"

/* Audio processing functions */
void read_pitched(Track *t, float *out_l, float *out_r, uint32_t out_frames,
		  float start_vel, float target_vel, float start_nudge,
		  float target_nudge);
void mix_sampler(SamplerSlot *slot, float *mix_l, float *mix_r,
		 uint32_t frames);

void wsola_process(Track *t, WSOLAState *ws, float *out_l, float *out_r,
		   uint32_t out_frames, double rate, float vol_gain,
		   int is_eco);
void pv_process(Track *t, PVState *pv, float *out_l, float *out_r,
		uint32_t out_frames, double rate, float gain);
void pv_init_tables(void);
void pv_reset(PVState *pv, uint32_t start_pos);

/* ── Load worker, audio thread, batch/enqueue ── */
void *load_worker(void *arg);
void *audio_thread(void *arg);

/* ── Promoted from djcmd.c ── */
void hp_open_device(int idx);
void pcm_open_device(int idx);
void pcm_enumerate_devices(void);
void sync_to_leader(int deck);
void wsola_reset(WSOLAState *ws, uint32_t pos);
float calc_auto_gain(const int16_t *data, uint32_t frames);
void enqueue_analyze(int deck);
void batch_analyze_start(float lo, float hi);
void batch_free_queue(void);
void batch_tick(void); /* call from UI loop each frame to advance the queue */
void enqueue_load(int deck, const char *path);

/* ── Phase D extractions (audio engine) ── */
void biquad_lowpass(float fc, float *b, float *a);
void biquad_highpass(float fc, float *b, float *a);
void init_eq_coeffs(void);
int load_sampler(SamplerSlot *s, const char *path);
int load_track(Track *t, const char *path);
float estimate_bpm_autocorr(const int16_t *data, uint32_t num_frames);
void rebuild_waveform_and_grid(Track *t, int force_analyze);

/* ── Main audio loop and ALSA management ── */
int init_alsa(void);
void mix_and_write(void);
void audio_pcm_drop_all(void);
void audio_pcm_close_all(void);
int audio_pcm_is_open(void);
void set_realtime_priority(pthread_t thread, int priority);

/* ── EQ coefficient accessors (for mix_and_write in djcmd.c) ── */
float *audio_lp_b(void);
float *audio_lp_a(void);
float *audio_bp_b(void);
float *audio_bp_a(void);
float *audio_hp_b(void);
float *audio_hp_a(void);
float audio_apply_biquad(float x, float *b, float *a, float *x1, float *x2,
			 float *y1, float *y2);

/* ── Audio Global State ──────────────────────────────────────────────── */
#define PCM_MAX_DEVICES 16
extern PCMDevice g_pcm_devlist[PCM_MAX_DEVICES];
extern int g_pcm_ndevices;
extern int g_pcm_dev_sel;
extern int g_pcm_hp_dev_sel;
extern char g_pcm_dev_str[64];
extern char g_pcm_hp_dev_str[64];
extern int g_audio_hp_picker;

extern SamplerSlot g_samplers[MAX_SAMPLER_SLOTS];
extern EQState g_eq[MAX_TRACKS];
extern PVState g_pv[MAX_TRACKS];
extern WSOLAState g_wsola[MAX_TRACKS];

extern int g_side_deck[2];
extern float g_pitch_range_vals[3];
extern const char *g_pitch_range_names[3];
extern int g_pitch_range[MAX_TRACKS];
extern int g_slip_motor_off[MAX_TRACKS];

extern pthread_mutex_t g_load_mutex;
extern pthread_cond_t g_load_cond;
extern LoadJob g_load_job;

#endif /* DJCMD_AUDIO_H */
