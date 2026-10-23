/* filters.h - small, allocation-free signal conditioning blocks used by the SensorManager. */
#ifndef FILTERS_H
#define FILTERS_H

#include <stdbool.h>
#include <stdint.h>

/* First-order low-pass, time-step aware: alpha = dt / (tau + dt), tau = 1/(2*pi*fc).
 * Using the real dt keeps the cutoff correct when sample timing jitters. */
typedef struct { float y; float tau_s; bool init; } lpf_t;
void  lpf_init(lpf_t *f, float cutoff_hz);              /* cutoff_hz <= 0 => pass-through */
void  lpf_reset(lpf_t *f);
float lpf_update(lpf_t *f, float x, float dt_s);        /* first sample initialises the output */

/* Stuck-value detector: true once `limit` consecutive samples stayed within +/-eps of the
 * first sample of the run. Real sensors always carry noise; a frozen value signals a hung
 * sensor or a bus returning stale data. */
typedef struct { float ref; uint16_t run; uint16_t limit; float eps; bool init; } stuck_t;
void stuck_init(stuck_t *s, uint16_t limit, float eps);
bool stuck_update(stuck_t *s, float x);                 /* returns current "stuck" state */
void stuck_reset(stuck_t *s);

/* Spike/outlier gate with step-change confirmation: a sample further than max_step from the last
 * accepted value is rejected, unless `confirm` consecutive samples agree with each other
 * (then it is a genuine step, e.g. fast climb, and is accepted). */
typedef struct { float last; float cand; uint8_t cand_n; uint8_t confirm; float max_step; bool init; } spike_t;
void spike_init(spike_t *s, float max_step, uint8_t confirm);
bool spike_update(spike_t *s, float x, float *out);     /* true = accepted; *out = value to use */
void spike_reset(spike_t *s);

#endif
