/* filters.c - see filters.h */
#include "filters.h"
#include <math.h>
#include <stddef.h>

#define TWO_PI 6.28318530718f

void lpf_init(lpf_t *f, float cutoff_hz)
{
    if (!f) return;
    f->y = 0.0f; f->init = false;
    f->tau_s = (cutoff_hz > 0.0f) ? 1.0f / (TWO_PI * cutoff_hz) : 0.0f;
}
void lpf_reset(lpf_t *f) { if (f) { f->init = false; f->y = 0.0f; } }
float lpf_update(lpf_t *f, float x, float dt_s)
{
    if (!f) return x;
    if (!isfinite(x)) return f->y;                          /* never poison the state with NaN/Inf */
    if (!f->init || f->tau_s <= 0.0f || !(dt_s > 0.0f)) { f->y = x; f->init = true; return x; }
    float a = dt_s / (f->tau_s + dt_s);
    f->y += a * (x - f->y);
    return f->y;
}

void stuck_init(stuck_t *s, uint16_t limit, float eps) { if (s) { s->limit = limit; s->eps = eps; stuck_reset(s); } }
void stuck_reset(stuck_t *s) { if (s) { s->init = false; s->run = 0; s->ref = 0.0f; } }
bool stuck_update(stuck_t *s, float x)
{
    if (!s || s->limit == 0u) return false;
    if (!s->init || fabsf(x - s->ref) > s->eps) { s->ref = x; s->run = 1u; s->init = true; return false; }
    if (s->run < 0xFFFFu) s->run++;
    return s->run >= s->limit;
}

void spike_init(spike_t *s, float max_step, uint8_t confirm)
{
    if (!s) return;
    s->max_step = max_step; s->confirm = (confirm < 1u) ? 1u : confirm; spike_reset(s);
}
void spike_reset(spike_t *s) { if (s) { s->init = false; s->cand_n = 0u; s->last = 0.0f; s->cand = 0.0f; } }
bool spike_update(spike_t *s, float x, float *out)
{
    if (!s || !out) return false;
    if (!isfinite(x)) { *out = s->last; return false; }
    if (!s->init || fabsf(x - s->last) <= s->max_step) {    /* normal sample */
        s->last = x; s->init = true; s->cand_n = 0u; *out = x; return true;
    }
    if (s->cand_n > 0u && fabsf(x - s->cand) <= s->max_step) {   /* agrees with the pending new level */
        s->cand_n++; s->cand = x;
    } else { s->cand = x; s->cand_n = 1u; }
    if (s->cand_n >= s->confirm) { s->last = x; s->cand_n = 0u; *out = x; return true; }   /* genuine step */
    *out = s->last; return false;                           /* outlier: hold previous value */
}
