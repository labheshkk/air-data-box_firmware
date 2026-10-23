/* sensor_manager.c - see sensor_manager.h */
#include "sensor_manager.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

sm_cfg_t sm_default_cfg(void)
{
    sm_cfg_t c = {
        .stale_timeout_us = 200000u, .fail_after_errors = 5u, .recover_after_good = 10u,
        .static_disagree_pa = 150.0f, .disagree_confirm = 3u, .agree_recover_cycles = 20u,
        .sigma_a_pa = 0.41f, .sigma_b_pa = 1.2f,
        .static_spike_pa = 200.0f, .spike_confirm = 3u, .stuck_samples = 100u,
        .static_lpf_hz = 10.0f, .dp_lpf_hz = 10.0f, .dp_spike_pa = 1500.0f, .dp_neg_tol_pa = 10.0f,
        .zero_samples = 100u, .zero_max_abs_pa = 150.0f, .zero_max_std_pa = 3.0f
    };
    return c;
}

adb_status_t sm_init(sm_t *sm, const sm_cfg_t *cfg)
{
    if (!sm || !cfg || cfg->fail_after_errors == 0u || cfg->recover_after_good == 0u ||
        !(cfg->sigma_a_pa > 0.0f) || !(cfg->sigma_b_pa > 0.0f) || !(cfg->static_disagree_pa > 0.0f) ||
        cfg->zero_samples == 0u) return ADB_ERR_PARAM;
    memset(sm, 0, sizeof *sm);
    sm->cfg = *cfg;
    for (int i = 0; i < SM_CH_COUNT; i++) {
        float step = (i == SM_CH_DIFF) ? cfg->dp_spike_pa : cfg->static_spike_pa;
        spike_init(&sm->ch[i].spike, step, cfg->spike_confirm);
        stuck_init(&sm->ch[i].stuck, cfg->stuck_samples, 0.0f);
        sm->ch[i].health = SM_HEALTH_UNKNOWN;
    }
    lpf_init(&sm->static_lpf, cfg->static_lpf_hz);
    lpf_init(&sm->dp_lpf, cfg->dp_lpf_hz);
    return ADB_OK;
}

static void record_error(sm_t *sm, sm_chan_t *c, adb_status_t st)
{
    c->last_status = st; c->err_total++; c->consec_good = 0u;
    if (c->consec_err < 0xFFFFu) c->consec_err++;
    if (c->consec_err >= sm->cfg.fail_after_errors) c->health = SM_HEALTH_FAILED;
    else if (c->health != SM_HEALTH_FAILED) c->health = SM_HEALTH_DEGRADED;
}
static void record_good(sm_t *sm, sm_chan_t *c, uint32_t now_us)
{
    c->last_status = ADB_OK; c->good_total++; c->consec_err = 0u; c->last_good_us = now_us; c->have = true;
    if (c->consec_good < 0xFFFFu) c->consec_good++;
    if (c->health == SM_HEALTH_FAILED) { if (c->consec_good >= sm->cfg.recover_after_good) c->health = SM_HEALTH_OK; }
    else c->health = SM_HEALTH_OK;
}

/* Common ingest: driver status -> spike gate -> stuck check -> health bookkeeping. */
static bool ingest(sm_t *sm, sm_chan_t *c, uint32_t now_us, adb_status_t st, float v)
{
    if (st != ADB_OK || !isfinite(v)) { record_error(sm, c, (st != ADB_OK) ? st : ADB_ERR_INVALID); return false; }
    float held;
    if (!spike_update(&c->spike, v, &held)) { record_error(sm, c, ADB_ERR_RANGE); return false; }
    if (stuck_update(&c->stuck, v)) { record_error(sm, c, ADB_ERR_STATE); return false; }
    c->value = held;
    record_good(sm, c, now_us);
    return true;
}

void sm_feed_static(sm_t *sm, sm_channel_t ch, uint32_t now_us, adb_status_t status, float pressure_pa, float temp_c)
{
    if (!sm || (ch != SM_CH_STATIC_A && ch != SM_CH_STATIC_B)) return;
    sm_chan_t *c = &sm->ch[ch];
    if (ingest(sm, c, now_us, status, pressure_pa)) c->temp_c = temp_c;
}

void sm_feed_diff(sm_t *sm, uint32_t now_us, adb_status_t status, float dp_pa, int32_t praw, int32_t tcode)
{
    if (!sm) return;
    sm_chan_t *c = &sm->ch[SM_CH_DIFF];
    if (sm->zero_state == SM_ZERO_COLLECTING) {            /* collect raw statistics; bypass spike/stuck gates */
        if (status == ADB_OK && isfinite(dp_pa)) {
            sm->zs_p += (double)dp_pa; sm->zs_pp += (double)dp_pa * (double)dp_pa;
            sm->zs_raw += (double)praw; sm->zs_t += (double)tcode; sm->zero_n++;
            record_good(sm, c, now_us);
            if (sm->zero_n >= sm->cfg.zero_samples) {
                double n = (double)sm->zero_n, mean = sm->zs_p / n;
                double var = sm->zs_pp / n - mean * mean; if (var < 0.0) var = 0.0;
                if (sqrt(var) > (double)sm->cfg.zero_max_std_pa) sm->zero_state = SM_ZERO_FAIL_SPREAD;
                else if (fabs(mean) > (double)sm->cfg.zero_max_abs_pa) sm->zero_state = SM_ZERO_FAIL_OFFSET;
                else { sm->zero_state = SM_ZERO_DONE;
                       sm->zero_praw0 = (int32_t)lround(sm->zs_raw / n); sm->zero_traw0 = (int32_t)lround(sm->zs_t / n); }
            }
        } else record_error(sm, c, (status != ADB_OK) ? status : ADB_ERR_INVALID);
        return;
    }
    if (ingest(sm, c, now_us, status, dp_pa)) sm->dp_last = c->value;
}

static bool usable(const sm_t *sm, const sm_chan_t *c, uint32_t now_us)
{
    if (!c->have || c->health == SM_HEALTH_FAILED || c->health == SM_HEALTH_UNKNOWN) return false;
    return (uint32_t)(now_us - c->last_good_us) <= sm->cfg.stale_timeout_us;
}
static bool stale(const sm_t *sm, const sm_chan_t *c, uint32_t now_us)
{
    return c->have && (uint32_t)(now_us - c->last_good_us) > sm->cfg.stale_timeout_us;
}

void sm_update(sm_t *sm, uint32_t now_us, sm_output_t *o)
{
    if (!sm || !o) return;
    memset(o, 0, sizeof *o);
    float dt = 0.0f;
    if (sm->upd_init) dt = (float)(uint32_t)(now_us - sm->last_update_us) * 1e-6f;
    sm->upd_init = true; sm->last_update_us = now_us;

    sm_chan_t *A = &sm->ch[SM_CH_STATIC_A], *B = &sm->ch[SM_CH_STATIC_B], *D = &sm->ch[SM_CH_DIFF];
    /* staleness escalates to FAILED after 3x the timeout so a dead bus cannot hide behind "degraded" */
    for (int i = 0; i < SM_CH_COUNT; i++) {
        sm_chan_t *c = &sm->ch[i];
        if (c->have && (uint32_t)(now_us - c->last_good_us) > 3u * sm->cfg.stale_timeout_us) c->health = SM_HEALTH_FAILED;
    }
    bool ua = usable(sm, A, now_us), ub = usable(sm, B, now_us), ud = usable(sm, D, now_us);
    float va = sm->cfg.sigma_a_pa * sm->cfg.sigma_a_pa, vb = sm->cfg.sigma_b_pa * sm->cfg.sigma_b_pa;
    float thr = sm->cfg.static_disagree_pa;

    bool have_out = false; float fused = 0.0f, var = 0.0f; sm_static_mode_t mode = SM_STATIC_NONE;
    if (ua && ub) {
        float d = fabsf(A->value - B->value);
        switch (sm->arb) {
        case ARB_AGREE: case ARB_SUSPECT:
            if (d <= thr) { sm->arb = ARB_AGREE; sm->suspect_n = 0u; }
            else {
                sm->arb = ARB_SUSPECT;
                if (++sm->suspect_n >= sm->cfg.disagree_confirm) {
                    if (sm->have_last_fused)
                        sm->arb = (fabsf(A->value - sm->last_fused) <= fabsf(B->value - sm->last_fused)) ? ARB_A : ARB_B;
                    else sm->arb = ARB_UNRESOLVED;
                    sm->recover_n = 0u;
                }
            }
            break;
        case ARB_A: case ARB_B:
            if (d <= 0.5f * thr) { if (++sm->recover_n >= sm->cfg.agree_recover_cycles) { sm->arb = ARB_AGREE; sm->suspect_n = 0u; } }
            else sm->recover_n = 0u;
            break;
        case ARB_UNRESOLVED:
            if (d <= thr) { sm->arb = ARB_AGREE; sm->suspect_n = 0u; }
            break;
        }
        switch (sm->arb) {
        case ARB_AGREE: {
            float wa = 1.0f / va, wb = 1.0f / vb;
            fused = (A->value * wa + B->value * wb) / (wa + wb); var = 1.0f / (wa + wb);
            mode = SM_STATIC_DUAL; have_out = true; break; }
        case ARB_SUSPECT:                                          /* hold previous output during the confirm window */
            if (sm->have_last_fused) { fused = sm->last_fused; var = va + (d * 0.5f) * (d * 0.5f); mode = SM_STATIC_DUAL; have_out = true; }
            o->flags |= SM_F_STATIC_DISAGREE; break;
        case ARB_A: fused = A->value; var = va + (d * 0.5f) * (d * 0.5f); mode = SM_STATIC_SINGLE_A; have_out = true;
            o->flags |= SM_F_STATIC_DISAGREE; break;
        case ARB_B: fused = B->value; var = vb + (d * 0.5f) * (d * 0.5f); mode = SM_STATIC_SINGLE_B; have_out = true;
            o->flags |= SM_F_STATIC_DISAGREE; break;
        case ARB_UNRESOLVED: o->flags |= SM_F_STATIC_DISAGREE; break;     /* no basis to choose: invalid */
        }
    } else if (ua || ub) {
        sm->arb = ARB_AGREE; sm->suspect_n = 0u;                  /* the other sensor must prove itself again */
        fused = ua ? A->value : B->value; var = ua ? va : vb;
        mode = ua ? SM_STATIC_SINGLE_A : SM_STATIC_SINGLE_B; have_out = true;
    }
    if (have_out) {
        sm->last_fused = fused; sm->have_last_fused = true;
        o->static_valid = true; o->static_mode = mode;
        o->static_pa = lpf_update(&sm->static_lpf, fused, dt); o->static_var_pa2 = (var < 0.01f) ? 0.01f : var;
        if (mode != SM_STATIC_DUAL) o->flags |= SM_F_STATIC_SINGLE;
    } else {
        o->flags |= SM_F_STATIC_NONE; lpf_reset(&sm->static_lpf);
    }
    float tsum = 0.0f; int tn = 0;
    if (ua) { tsum += A->temp_c; tn++; }
    if (ub) { tsum += B->temp_c; tn++; }
    if (tn) { o->temp_c = tsum / (float)tn; o->temp_valid = true; }

    if (sm->zero_state == SM_ZERO_COLLECTING) o->flags |= SM_F_ZEROING;
    else if (ud) {
        o->dp_valid = true; o->dp_pa = lpf_update(&sm->dp_lpf, D->value, dt);
        if (D->value < -sm->cfg.dp_neg_tol_pa) o->flags |= SM_F_DP_NEGATIVE;
    } else lpf_reset(&sm->dp_lpf);
    if (sm->zero_state == SM_ZERO_FAIL_SPREAD || sm->zero_state == SM_ZERO_FAIL_OFFSET) o->flags |= SM_F_ZERO_FAILED;

    for (int i = 0; i < SM_CH_COUNT; i++) o->health[i] = sm->ch[i].health;
    if (A->health == SM_HEALTH_FAILED) o->flags |= SM_F_A_FAILED;
    if (B->health == SM_HEALTH_FAILED) o->flags |= SM_F_B_FAILED;
    if (D->health == SM_HEALTH_FAILED) o->flags |= SM_F_DIFF_FAILED;
    if (stale(sm, A, now_us)) o->flags |= SM_F_A_STALE;
    if (stale(sm, B, now_us)) o->flags |= SM_F_B_STALE;
    if (stale(sm, D, now_us)) o->flags |= SM_F_DIFF_STALE;
    if (A->last_status == ADB_ERR_STATE) o->flags |= SM_F_A_STUCK;
    if (B->last_status == ADB_ERR_STATE) o->flags |= SM_F_B_STUCK;
    if (D->last_status == ADB_ERR_RANGE) o->flags |= SM_F_DP_RANGE;
}

void sm_zero_begin(sm_t *sm)
{
    if (!sm) return;
    sm->zero_state = SM_ZERO_COLLECTING; sm->zero_n = 0u;
    sm->zs_p = sm->zs_pp = sm->zs_raw = sm->zs_t = 0.0;
}
sm_zero_state_t sm_zero_state(const sm_t *sm) { return sm ? sm->zero_state : SM_ZERO_IDLE; }
adb_status_t sm_zero_result(const sm_t *sm, int32_t *praw0, int32_t *traw0)
{
    if (!sm || !praw0 || !traw0) return ADB_ERR_PARAM;
    if (sm->zero_state != SM_ZERO_DONE) return ADB_ERR_NOT_READY;
    *praw0 = sm->zero_praw0; *traw0 = sm->zero_traw0; return ADB_OK;
}
void sm_zero_abort(sm_t *sm)
{
    if (sm && (sm->zero_state == SM_ZERO_COLLECTING || sm->zero_state == SM_ZERO_FAIL_SPREAD || sm->zero_state == SM_ZERO_FAIL_OFFSET))
        sm->zero_state = SM_ZERO_IDLE;                      /* also clears a latched zero failure */
}
void sm_zero_applied(sm_t *sm)
{
    if (!sm) return;
    sm->zero_state = SM_ZERO_IDLE;
    lpf_reset(&sm->dp_lpf); spike_reset(&sm->ch[SM_CH_DIFF].spike); stuck_reset(&sm->ch[SM_CH_DIFF].stuck);
    sm->ch[SM_CH_DIFF].have = false;
}

uint8_t sm_node_health(const sm_output_t *o)
{
    if (!o) return 3u;
    if (!o->static_valid && !o->dp_valid) return 3u;                                  /* CRITICAL: no air data at all */
    if (!o->static_valid || (!o->dp_valid && !(o->flags & SM_F_ZEROING))) return 2u;  /* ERROR: a required quantity is missing */
    if (o->flags & (SM_F_STATIC_SINGLE | SM_F_STATIC_DISAGREE | SM_F_ZERO_FAILED | SM_F_DP_NEGATIVE |
                    SM_F_A_FAILED | SM_F_B_FAILED | SM_F_DP_RANGE)) return 1u;       /* WARNING: degraded redundancy */
    return 0u;
}
