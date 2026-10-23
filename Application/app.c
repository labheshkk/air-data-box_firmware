/* app.c - see app.h */
#include "app.h"
#include <math.h>
#include <string.h>

enum { S_BMP = 0, S_MS = 1, S_RSC = 2 };
#define ZERO_COLLECT_TIMEOUT_US 5000000u

app_cfg_t app_default_cfg(void)
{
    app_cfg_t c;
    memset(&c, 0, sizeof c);
    c.node_id = 42u;
    c.bmp = (bmp581_config_t){ .osr_p = BMP581_OSR_X4, .osr_t = BMP581_OSR_X1, .odr = 0x0Fu };   /* 50 Hz, valid per Table 7 */
    c.ms_osr = MS5611_OSR_4096; c.ms_temp_every_n = 8u;
    c.rsc = (rsc_config_t){ RSC_N90, 10u, RSC_CRC_ACCEPT_UNVERIFIED, "RSCDJJI060MDSE3" };
    c.air = airdata_default_cfg(); c.sm = sm_default_cfg();
    c.autozero_on_boot = true; c.autozero_window_us = 10000000u;
    c.output_period_us = 20000u; c.temp_period_us = 100000u; c.status_period_us = 1000000u; c.reinit_period_us = 1000000u;
    c.dp_sigma_pa = 30.0f;
    return c;
}

static void ev(app_t *a, app_event_t code, int32_t arg) { if (a->hw.event) a->hw.event(a->hw.event_ctx, (uint16_t)code, arg); }
static bool due(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }

static void send(app_t *a, const dc_frame_t *f)
{
    if (a->hw.can_tx && a->hw.can_tx(a->hw.can_ctx, f)) a->diag.frames_sent++;
    else { a->diag.tx_drops++; ev(a, APP_EV_TX_DROP, (int32_t)f->id); }
}

static adb_status_t init_sensor(app_t *a, int which)
{
    adb_status_t s;
    a->diag.init_attempts[which]++;
    if (which == S_BMP) s = bmp581_init(&a->bmp, &a->hw.bmp_bus, &a->cfg.bmp);
    else if (which == S_MS) s = ms5611_init(&a->ms, &a->hw.ms_bus, a->cfg.ms_osr, a->cfg.ms_temp_every_n);
    else {
        s = rsc_init(&a->rsc, &a->hw.rsc_ee_bus, &a->hw.rsc_adc_bus, &a->cfg.rsc);
        if (s == ADB_OK) {
            if (!a->rsc.crc_verified) ev(a, APP_EV_RSC_CRC_UNVERIFIED, 0);
            if (!a->rsc.catalog_match) ev(a, APP_EV_RSC_PART_MISMATCH, 0);
        }
    }
    a->diag.last_status[which] = s;
    a->ok[which] = (s == ADB_OK);
    ev(a, (s == ADB_OK) ? APP_EV_INIT_OK : APP_EV_INIT_FAIL, which | ((int32_t)s << 8));
    return s;
}

adb_status_t app_init(app_t *a, const app_cfg_t *cfg, const app_hw_t *hw, uint32_t now_us)
{
    if (!a || !cfg || !hw) return ADB_ERR_PARAM;
    memset(a, 0, sizeof *a);
    a->cfg = *cfg; a->hw = *hw;
    adb_status_t s = sm_init(&a->sm, &cfg->sm);
    if (s != ADB_OK) return s;
    s = dc_init(&a->node, cfg->node_id);
    if (s != ADB_OK) return s;
    if (cfg->output_period_us == 0u || cfg->temp_period_us == 0u || cfg->status_period_us == 0u || cfg->reinit_period_us == 0u)
        return ADB_ERR_PARAM;
    a->t_boot = now_us; a->t_out = now_us + cfg->output_period_us; a->t_temp = now_us + cfg->temp_period_us;
    a->t_status = now_us;                                    /* announce ourselves immediately */
    ev(a, APP_EV_BOOT, 0);
    for (int i = 0; i < 3; i++) { (void)init_sensor(a, i); a->t_reinit[i] = now_us + cfg->reinit_period_us; }
    a->zero_pending = cfg->autozero_on_boot;                 /* partial availability is fine: each sensor is independent */
    return ADB_OK;                                           /* never fails just because a sensor is missing */
}

void app_request_zero(app_t *a)                               /* explicit: ignores the boot window, clears an old failure */
{
    if (!a) return;
    sm_zero_abort(&a->sm);
    a->zero_pending = true; a->zero_forced = true; a->zero_started_us = 0u; a->zero_fail_reported = false;
}

static void poll_sensors(app_t *a, uint32_t now)
{
    adb_status_t s; bool fresh;
    if (a->ok[S_BMP]) {
        bmp581_sample_t smp;
        s = bmp581_poll(&a->bmp, &smp, &fresh);
        if (s != ADB_OK) sm_feed_static(&a->sm, SM_CH_STATIC_A, now, s, 0.0f, 0.0f);
        else if (fresh) sm_feed_static(&a->sm, SM_CH_STATIC_A, now, ADB_OK, (float)smp.pressure_pa_q6 / 64.0f, (float)smp.temp_c_q16 / 65536.0f);
    }
    if (a->ok[S_MS]) {
        s = ms5611_poll(&a->ms, now, &fresh);
        if (s != ADB_OK && s != ADB_ERR_NOT_READY) sm_feed_static(&a->sm, SM_CH_STATIC_B, now, s, 0.0f, 0.0f);
        else if (fresh) sm_feed_static(&a->sm, SM_CH_STATIC_B, now, ADB_OK, (float)a->ms.last.pressure_pa, (float)a->ms.last.temp_centi_c / 100.0f);
    }
    if (a->ok[S_RSC]) {
        s = rsc_poll(&a->rsc, now, &fresh);
        if (s != ADB_OK && s != ADB_ERR_NOT_READY) sm_feed_diff(&a->sm, now, s, 0.0f, 0, 0);
        else if (fresh) sm_feed_diff(&a->sm, now, ADB_OK, (float)a->rsc.last.pressure_pa, a->rsc.last.p_raw, a->rsc.last.t_code);
    }
}

static void zero_begin(app_t *a, uint32_t now)
{
    sm_zero_begin(&a->sm);
    a->zero_started_us = (now != 0u) ? now : 1u;             /* timeout is measured per attempt */
    a->zero_fail_reported = false;
}

static void run_zero(app_t *a, uint32_t now)
{
    sm_zero_state_t zs = sm_zero_state(&a->sm);
    bool in_boot_window = (uint32_t)(now - a->t_boot) <= a->cfg.autozero_window_us;
    if (zs == SM_ZERO_IDLE && a->zero_pending && a->ok[S_RSC]) {
        if (a->zero_forced || in_boot_window) zero_begin(a, now);
        else a->zero_pending = false;                         /* window over: only an explicit request can zero now */
    } else if (zs == SM_ZERO_COLLECTING) {
        if ((uint32_t)(now - a->zero_started_us) > ZERO_COLLECT_TIMEOUT_US) {
            sm_zero_abort(&a->sm); a->zero_pending = false; a->zero_forced = false; a->zero_started_us = 0u;
            ev(a, APP_EV_ZERO_ABORT, 0);
        }
    } else if (zs == SM_ZERO_DONE) {
        int32_t pr, tr;
        if (sm_zero_result(&a->sm, &pr, &tr) == ADB_OK) { rsc_set_zero(&a->rsc, pr, tr); ev(a, APP_EV_ZERO_OK, pr); }
        sm_zero_applied(&a->sm); a->zero_pending = false; a->zero_forced = false; a->zero_started_us = 0u;
    } else if ((zs == SM_ZERO_FAIL_SPREAD || zs == SM_ZERO_FAIL_OFFSET) && !a->zero_fail_reported) {
        a->zero_fail_reported = true;                         /* report once; the failure stays latched in the manager flags */
        ev(a, APP_EV_ZERO_FAIL, (int32_t)zs);
        if (a->zero_pending && !a->zero_forced && in_boot_window) zero_begin(a, now);       /* boot: retry inside the window */
        else { a->zero_pending = false; a->zero_forced = false; a->zero_started_us = 0u; }   /* explicit request: single attempt */
    }
}

static void output_task(app_t *a, uint32_t now)
{
    sm_update(&a->sm, now, &a->out);
    a->diag.out_cycles++;
    const sm_output_t *o = &a->out;
    if (o->flags != a->prev_flags) { ev(a, APP_EV_FLAGS, (int32_t)o->flags); a->prev_flags = o->flags; }
    if (o->static_mode != a->prev_mode) { ev(a, APP_EV_STATIC_MODE, (int32_t)o->static_mode); a->prev_mode = o->static_mode; }
    if (o->static_valid) a->seen_valid = true;

    dc_frame_t f;
    a->alt_status = ADB_ERR_NOT_READY;
    if (o->static_valid) {
        a->alt_status = airdata_pressure_altitude(o->static_pa, 101325.0f, &a->altitude_m);
        if (dc_encode_static_pressure(&a->node, o->static_pa, o->static_var_pa2, &f) == ADB_OK) send(a, &f);
    }
    a->ias_status = ADB_ERR_NOT_READY;
    if (o->dp_valid) {
        a->ias_status = airdata_airspeed_ias(&a->cfg.air, o->dp_pa, &a->ias_ms);
        if (a->ias_status == ADB_OK) {
            float v = (a->ias_ms > 5.0f) ? a->ias_ms : 5.0f;                       /* dV = dq/(rho*V); floor V to bound the variance */
            float sd = a->cfg.dp_sigma_pa / (a->cfg.air.rho0_kg_m3 * v);
            a->ias_var = sd * sd;
            if (dc_encode_indicated_airspeed(&a->node, a->ias_ms, a->ias_var, &f) == ADB_OK) send(a, &f);
        } else a->diag.ias_rejected++;                                              /* faulty value is withheld, not published */
    }
}

void app_step(app_t *a, uint32_t now)
{
    if (!a) return;
    for (int i = 0; i < 3; i++)                                                     /* retry sensors that failed to start */
        if (!a->ok[i] && due(now, a->t_reinit[i])) { (void)init_sensor(a, i); a->t_reinit[i] = now + a->cfg.reinit_period_us; }
    poll_sensors(a, now);
    run_zero(a, now);
    if (due(now, a->t_out)) {
        output_task(a, now);
        do { a->t_out += a->cfg.output_period_us; } while (due(now, a->t_out));     /* catch up without bursts */
    }
    dc_frame_t f;
    if (due(now, a->t_temp)) {
        if (a->out.temp_valid && dc_encode_static_temperature(&a->node, a->out.temp_c + 273.15f, 1.0f, &f) == ADB_OK) send(a, &f);
        do { a->t_temp += a->cfg.temp_period_us; } while (due(now, a->t_temp));
    }
    if (due(now, a->t_status)) {
        dc_mode_t mode = !a->seen_valid ? DC_MODE_INITIALIZATION
                         : (a->out.flags & SM_F_ZEROING) ? DC_MODE_MAINTENANCE : DC_MODE_OPERATIONAL;
        uint8_t health = sm_node_health(&a->out);
        if (!a->seen_valid) health = 0u;                                            /* still starting: don't alarm */
        if (dc_encode_node_status(&a->node, (now - a->t_boot) / 1000000u, health, mode,
                                  (uint16_t)(a->out.flags & 0xFFFFu), &f) == ADB_OK) send(a, &f);
        do { a->t_status += a->cfg.status_period_us; } while (due(now, a->t_status));
    }
}
