/* test_app.c - END-TO-END host simulation. Three simulated sensors (driven by a shared "world": static pressure,
 * differential pressure, temperature) speak their real register/command protocols to the real drivers, which feed
 * the real SensorManager, air-data maths and DroneCAN encoder. Frames leaving the application are captured and
 * decoded with an independent spec-based decoder.
 *
 * LIMIT: the simulators are written by the same author as the drivers from the same datasheets. They prove the
 * pieces fit together and behave under faults; they do NOT prove the sensors behave like this on a real board. */
#include <math.h>
#include <string.h>
#include "app.h"
#include "test_common.h"

#define NEAR(a, b, tol) do { g_tests_run++; double _a = (a), _b = (b); \
    if (!(fabs(_a - _b) <= (tol))) { g_tests_failed++; \
    printf("  FAIL %s:%d: %s ~ %s (got %.4f, expected %.4f)\n", __FILE__, __LINE__, #a, #b, _a, _b); } } while (0)

/* ------------------------------------------------------------------ world + simulated devices */
typedef struct {
    double p_static, dp, temp_c, ms_bias_pa, rsc_zero_err_counts;
    int ms_fail, rsc_fail, bmp_fail, bmp_bad_id;
} world_t;
static world_t W;
static uint32_t NOW_US;
static unsigned NZ;
static double nz(double amp) { NZ++; return amp * sin((double)NZ * 0.77); }

/* ---- MS5611 ---- */
static uint16_t MS_PROM[8] = { 0x1234, 40127, 36924, 23317, 23282, 33464, 28312, 0 };
static struct { int pending, is_d2; uint32_t start; } MS;
static adb_status_t ms_write(void *c, const uint8_t *tx, uint16_t len)
{
    (void)c; if (W.ms_fail) return ADB_ERR_BUS;
    if (len != 1) return ADB_ERR_PARAM;
    if (tx[0] == 0x1E) return ADB_OK;
    if ((tx[0] & 0xF0) == 0x40) { MS.pending = 1; MS.is_d2 = 0; MS.start = NOW_US; return ADB_OK; }
    if ((tx[0] & 0xF0) == 0x50) { MS.pending = 1; MS.is_d2 = 1; MS.start = NOW_US; return ADB_OK; }
    return ADB_ERR_PARAM;
}
static adb_status_t ms_wr(void *c, const uint8_t *tx, uint16_t txl, uint8_t *rx, uint16_t rxl)
{
    (void)c; (void)txl; if (W.ms_fail) return ADB_ERR_BUS;
    if (tx[0] >= 0xA0 && tx[0] <= 0xAE && rxl == 2) { uint16_t w = MS_PROM[(tx[0] - 0xA0) >> 1]; rx[0] = (uint8_t)(w >> 8); rx[1] = (uint8_t)w; return ADB_OK; }
    if (tx[0] == 0x00 && rxl == 3) {
        uint32_t v = 0;
        if (MS.pending && (uint32_t)(NOW_US - MS.start) >= 8220u) {
            const int64_t dT = ((int64_t)llround(W.temp_c * 100.0) - 2000) * 8388608LL / 28312LL;   /* from TEMP = 2000 + dT*C6/2^23 */
            if (MS.is_d2) v = (uint32_t)(33464LL * 256 + dT);
            else {
                int64_t off = 36924LL * 65536 + (23282LL * dT) / 128, sens = 40127LL * 32768 + (23317LL * dT) / 256;
                double p = W.p_static + W.ms_bias_pa + nz(1.5);
                v = (uint32_t)(((p * 32768.0 + (double)off) * 2097152.0) / (double)sens);
            }
            MS.pending = 0;
        }
        rx[0] = (uint8_t)(v >> 16); rx[1] = (uint8_t)(v >> 8); rx[2] = (uint8_t)v; return ADB_OK;
    }
    return ADB_ERR_PARAM;
}

/* ---- BMP581 ---- */
static uint8_t BR[128]; static uint32_t bmp_next_us;
static void bmp_reset(void) { memset(BR, 0, sizeof BR); BR[1] = W.bmp_bad_id ? 0x00 : 0x50; BR[0x27] = 0x10; BR[0x28] = 0x02; BR[0x30] = 3; BR[0x37] = 0x70; for (int r = 0x1D; r <= 0x22; r++) BR[r] = 0x7F; }
static void bmp_tick(void)
{
    if (!(BR[0x37] & 3) || (int32_t)(NOW_US - bmp_next_us) < 0) return;
    bmp_next_us = NOW_US + 20000u;
    uint32_t t = (uint32_t)(int32_t)(W.temp_c * 65536.0) & 0xFFFFFFu, p = (uint32_t)(int32_t)((W.p_static + nz(0.2)) * 64.0) & 0xFFFFFFu;
    BR[0x1D] = (uint8_t)t; BR[0x1E] = (uint8_t)(t >> 8); BR[0x1F] = (uint8_t)(t >> 16);
    BR[0x20] = (uint8_t)p; BR[0x21] = (uint8_t)(p >> 8); BR[0x22] = (uint8_t)(p >> 16); BR[0x27] |= 1;
}
static adb_status_t bmp_w(void *c, const uint8_t *tx, uint16_t len)
{
    (void)c; if (W.bmp_fail) return ADB_ERR_BUS; if (len != 2) return ADB_ERR_PARAM;
    if (tx[0] == 0x7E && tx[1] == 0xB6) { bmp_reset(); return ADB_OK; }
    BR[tx[0] & 0x7F] = tx[1]; return ADB_OK;
}
static adb_status_t bmp_wr(void *c, const uint8_t *tx, uint16_t txl, uint8_t *rx, uint16_t rxl)
{
    (void)c; (void)txl; if (W.bmp_fail) return ADB_ERR_BUS;
    for (uint16_t i = 0; i < rxl; i++) { uint8_t r = (uint8_t)(tx[0] + i); rx[i] = (r == 0x38) ? 0x80 : BR[r]; }
    if (tx[0] <= 0x27 && tx[0] + rxl > 0x27) BR[0x27] = 0;
    return ADB_OK;
}

/* ---- RSC (synthetic EEPROM identical in layout to the datasheet; ADC returns physically consistent raw codes) ---- */
static uint8_t REE[RSC_EEPROM_SIZE]; static int rsc_ts, rsc_pending; static uint32_t rsc_start;
static const double OFF0 = -4000000.0, OFF1 = 0.125, SPAN0 = 8000000.0, SPAN1 = 128.0, SH1 = 1.0, SH2 = 0.015625;
static void putf(unsigned a, float f) { memcpy(REE + a, &f, 4); }
static void rsc_build(void)
{
    memset(REE, 0, sizeof REE); memcpy(REE, "RSCDJJI060MDSE3", 15); putf(27, 120.0f); putf(31, -60.0f); memcpy(REE + 35, "mbar ", 5);
    REE[61] = 0x11; REE[63] = 0x22; REE[65] = 0x33; REE[67] = 0x44;
    putf(130, (float)OFF0); putf(134, (float)OFF1); putf(210, (float)SPAN0); putf(214, (float)SPAN1); putf(294, (float)SH1); putf(298, (float)SH2);
    uint16_t crc = rsc_crc16_ccitt(REE, 450, 0xFFFF); REE[450] = (uint8_t)crc; REE[451] = (uint8_t)(crc >> 8);
}
static adb_status_t ree_wr(void *c, const uint8_t *tx, uint16_t txl, uint8_t *rx, uint16_t rxl)
{ (void)c; (void)tx; (void)txl; if (W.rsc_fail) return ADB_ERR_BUS; memcpy(rx, REE, rxl); return ADB_OK; }
static adb_status_t radc_w(void *c, const uint8_t *tx, uint16_t len)
{
    (void)c; if (W.rsc_fail) return ADB_ERR_BUS;
    if (len == 3 && tx[0] == 0x44) { rsc_ts = (tx[1] >> 1) & 1; rsc_pending = 1; rsc_start = NOW_US; }
    return ADB_OK;
}
static adb_status_t radc_r(void *c, uint8_t *rx, uint16_t len)
{
    (void)c; (void)len; if (W.rsc_fail) return ADB_ERR_BUS;
    uint32_t v = 0;
    if (rsc_pending && (uint32_t)(NOW_US - rsc_start) >= 11260u) {
        double tcode = W.temp_c / 0.03125;
        if (rsc_ts) v = ((uint32_t)(int32_t)tcode & 0x3FFFu) << 10;
        else {
            double target = (W.dp + nz(0.3)) / 100.0, fs = (target + 60.0) / 120.0, x = fs;
            for (int i = 0; i < 12; i++) x -= ((SH1 * x + SH2 * x * x) - fs) / (SH1 + 2.0 * SH2 * x);
            double raw = (OFF0 + OFF1 * tcode) + x * (SPAN0 + SPAN1 * tcode) + W.rsc_zero_err_counts;
            v = (uint32_t)(int32_t)llround(raw) & 0xFFFFFFu;
        }
        rsc_pending = 0;
    }
    rx[0] = (uint8_t)(v >> 16); rx[1] = (uint8_t)(v >> 8); rx[2] = (uint8_t)v; return ADB_OK;
}
static void dly(void *c, uint32_t us) { (void)c; NOW_US += us; }

/* ---- capture ---- */
typedef struct { uint32_t t; dc_frame_t f; } cap_t;
static cap_t CAP[20000]; static unsigned NCAP; static uint16_t EV[256]; static int32_t EVA[256]; static unsigned NEV; static int tx_full;
static bool can_tx(void *c, const dc_frame_t *f) { (void)c; if (tx_full) return false; if (NCAP < 20000u) { CAP[NCAP].t = NOW_US; CAP[NCAP++].f = *f; } return true; }
static void event(void *c, uint16_t code, int32_t arg) { (void)c; if (NEV < 256u) { EV[NEV] = code; EVA[NEV++] = arg; } }
static unsigned ev_count(uint16_t code) { unsigned n = 0; for (unsigned i = 0; i < NEV; i++) if (EV[i] == code) n++; return n; }

static unsigned dt_of(const dc_frame_t *f) { return (f->id >> 8) & 0xFFFFu; }
static unsigned count(unsigned dtid, uint32_t t0, uint32_t t1) { unsigned n = 0; for (unsigned i = 0; i < NCAP; i++) if (dt_of(&CAP[i].f) == dtid && CAP[i].t >= t0 && CAP[i].t < t1) n++; return n; }
static const dc_frame_t *last(unsigned dtid) { for (unsigned i = NCAP; i > 0; i--) if (dt_of(&CAP[i - 1].f) == dtid) return &CAP[i - 1].f; return NULL; }
static float f32_at(const uint8_t *d) { uint32_t u = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24); float f; memcpy(&f, &u, 4); return f; }
static float f16_at(const uint8_t *d) { return dc_f16_to_f32((uint16_t)(d[0] | (d[1] << 8))); }
static unsigned status_health(void) { const dc_frame_t *f = last(341); return f ? (unsigned)(f->data[4] >> 6) : 99u; }
static unsigned status_vendor(void) { const dc_frame_t *f = last(341); return f ? (unsigned)(f->data[5] | (f->data[6] << 8)) : 0u; }

/* ---- harness ---- */
static app_t APP;
static void world_default(void) { memset(&W, 0, sizeof W); W.p_static = 90000.0; W.dp = 0.0; W.temp_c = 25.0; W.rsc_zero_err_counts = 40000.0; }
static void boot(const app_cfg_t *cfg)
{
    NOW_US = 1000000u; NCAP = 0; NEV = 0; NZ = 0; tx_full = 0; memset(&MS, 0, sizeof MS); rsc_ts = rsc_pending = 0; bmp_next_us = 0;
    uint16_t crcp[8]; memcpy(crcp, MS_PROM, sizeof crcp); crcp[7] = 0; MS_PROM[7] = ms5611_crc4(crcp);
    bmp_reset(); rsc_build();
    app_hw_t hw; memset(&hw, 0, sizeof hw);
    hw.bmp_bus = (adb_bus_t){ .kind = ADB_BUS_I2C, .write = bmp_w, .write_read = bmp_wr, .delay_us = dly };
    hw.ms_bus = (adb_bus_t){ .kind = ADB_BUS_SPI, .write = ms_write, .write_read = ms_wr, .delay_us = dly };
    hw.rsc_ee_bus = (adb_bus_t){ .kind = ADB_BUS_SPI, .write_read = ree_wr, .delay_us = dly };
    hw.rsc_adc_bus = (adb_bus_t){ .kind = ADB_BUS_SPI, .write = radc_w, .read = radc_r, .delay_us = dly };
    hw.can_tx = can_tx; hw.event = event;
    app_cfg_t c = cfg ? *cfg : app_default_cfg();
    CHECK_EQ_I(app_init(&APP, &c, &hw, NOW_US), ADB_OK);
}
static void run_ms(uint32_t ms) { for (uint32_t i = 0; i < ms; i++) { NOW_US += 1000u; bmp_tick(); app_step(&APP, NOW_US); } }

/* ------------------------------------------------------------------ scenarios */
static void test_normal_flight(void)
{
    world_default(); boot(NULL);
    CHECK(APP.ok[0] && APP.ok[1] && APP.ok[2]);
    CHECK_EQ_I(ev_count(APP_EV_INIT_OK), 3);
    CHECK_EQ_I(ev_count(APP_EV_RSC_CRC_UNVERIFIED), 0);                   /* synthetic EEPROM carries a valid CRC (init 0xFFFF) */
    run_ms(4000);                                                          /* stationary: autozero happens here */
    CHECK_EQ_I(ev_count(APP_EV_ZERO_OK), 1);
    W.dp = 600.0; run_ms(1500);
    uint32_t t1 = NOW_US;
    run_ms(2000);
    const dc_frame_t *sp = last(1028), *ias = last(1021), *st = last(341), *tp = last(1029);
    CHECK(sp && ias && st && tp);
    NEAR(f32_at(sp->data), 90000.0, 2.0);                                  /* fused static pressure */
    NEAR(f16_at(sp->data + 4), 0.15, 0.05);                                /* inverse-variance blend of BMP581 and MS5611 */
    NEAR(f16_at(ias->data), 31.298, 0.3);                                  /* sqrt(2*600/1.225); an uncorrected 59 Pa offset would give 32.8 */
    NEAR(f16_at(tp->data), 298.15, 0.3);                                   /* mean of both sensors' temperature, in kelvin */
    NEAR(APP.altitude_m, 988.5, 1.5);                                      /* ISA pressure altitude of 90000 Pa (1000 m is 89874.6 Pa) */
    unsigned n_sp = count(1028, t1, t1 + 2000000u), n_ias = count(1021, t1, t1 + 2000000u), n_st = count(341, t1, t1 + 2000000u);
    CHECK(n_sp >= 98 && n_sp <= 102); CHECK(n_ias >= 98 && n_ias <= 102);  /* 50 Hz */
    CHECK(count(1029, t1, t1 + 2000000u) >= 19 && count(1029, t1, t1 + 2000000u) <= 21);   /* 10 Hz */
    CHECK(n_st >= 2 && n_st <= 3);                                         /* 1 Hz */
    CHECK_EQ_I(status_health(), 0); CHECK_EQ_I(status_vendor(), 0);
    CHECK_EQ_I(st->data[4] & 0x38, 0);                                     /* MODE_OPERATIONAL */
    CHECK_EQ_I(APP.diag.tx_drops, 0);
    for (unsigned i = 0; i < NCAP; i++) {                                  /* every frame is a legal single-frame transfer */
        CHECK_EQ_I(CAP[i].f.data[CAP[i].f.len - 1] & 0xE0, 0xC0);
        CHECK_EQ_I(CAP[i].f.id & 0x7F, 42); CHECK(((CAP[i].f.id >> 7) & 1) == 0);
    }
}
static void test_static_sensor_failure_and_recovery(void)
{
    world_default(); boot(NULL); run_ms(4000); W.dp = 400.0; run_ms(1000);
    W.ms_fail = 1; run_ms(1000);                                           /* MS5611 bus dies */
    CHECK_EQ_I(APP.out.static_mode, SM_STATIC_SINGLE_A); CHECK_EQ_I(status_health(), 1);
    CHECK(status_vendor() & SM_F_B_FAILED);
    uint32_t t0 = NOW_US; run_ms(1000);
    CHECK(count(1028, t0, NOW_US) >= 48);                                  /* static pressure never stops */
    NEAR(f32_at(last(1028)->data), 90000.0, 2.0);
    W.ms_fail = 0; run_ms(3000);                                           /* bus returns: sensor must re-prove itself */
    CHECK_EQ_I(APP.out.static_mode, SM_STATIC_DUAL); CHECK_EQ_I(status_health(), 0);
}
static void test_differential_sensor_failure(void)
{
    world_default(); boot(NULL); run_ms(4000); W.dp = 600.0; run_ms(1500);
    W.rsc_fail = 1; run_ms(1500);
    uint32_t t0 = NOW_US; run_ms(1000);
    CHECK_EQ_I(count(1021, t0, NOW_US), 0);                                /* no airspeed published without a valid dp sensor */
    CHECK(count(1028, t0, NOW_US) >= 48);                                  /* static still flows */
    CHECK_EQ_I(status_health(), 2);                                        /* NodeStatus ERROR */
    CHECK(status_vendor() & SM_F_DIFF_FAILED);
    W.rsc_fail = 0; run_ms(3000);
    CHECK(count(1021, NOW_US - 1000000u, NOW_US) >= 45); CHECK_EQ_I(status_health(), 0);
}
static void test_static_disagreement(void)
{
    world_default(); boot(NULL); run_ms(4000); W.dp = 300.0; run_ms(1000);
    W.ms_bias_pa = 400.0; run_ms(2500);                                    /* MS5611 reads 400 Pa high */
    CHECK(status_vendor() & SM_F_STATIC_DISAGREE); CHECK_EQ_I(status_health(), 1);
    NEAR(f32_at(last(1028)->data), 90000.0, 3.0);                          /* arbitration by continuity keeps the unbiased sensor */
    CHECK(f16_at(last(1028)->data + 4) > 50.0f);                           /* published uncertainty is inflated */
    W.ms_bias_pa = 0.0; run_ms(3000);
    CHECK_EQ_I(APP.out.static_mode, SM_STATIC_DUAL); CHECK_EQ_I(status_health(), 0);
}
static void test_sensor_missing_at_boot_then_recovers(void)
{
    world_default(); W.bmp_bad_id = 1; boot(NULL);
    CHECK(!APP.ok[0]); CHECK(APP.ok[1]); CHECK(APP.ok[2]); CHECK_EQ_I(ev_count(APP_EV_INIT_FAIL), 1);
    CHECK_EQ_I(APP.diag.last_status[0], ADB_ERR_ID);
    run_ms(4000);
    CHECK_EQ_I(APP.out.static_mode, SM_STATIC_SINGLE_B);                   /* runs on one static sensor */
    CHECK(count(1028, 2000000u, NOW_US) > 0);
    W.bmp_bad_id = 0; run_ms(3000);                                        /* sensor repaired: init is retried every 1 s */
    CHECK(APP.ok[0]); CHECK_EQ_I(APP.out.static_mode, SM_STATIC_DUAL);
    CHECK(APP.diag.init_attempts[0] >= 2);
}
static void test_moving_air_during_zero(void)
{
    world_default(); W.dp = 500.0; boot(NULL); run_ms(12000);              /* wind on the probe at power-up */
    CHECK_EQ_I(ev_count(APP_EV_ZERO_OK), 0); CHECK(ev_count(APP_EV_ZERO_FAIL) >= 1);   /* zero rejected: offset too large */
    CHECK(status_vendor() & SM_F_ZERO_FAILED); CHECK_EQ_I(status_health(), 1);          /* flagged, not hidden */
    CHECK(count(1021, 11000000u, NOW_US) > 0);                              /* airspeed resumes after the zero window closes */
}
static void test_explicit_zero_after_boot_window(void)
{
    world_default(); boot(NULL); run_ms(4000);
    CHECK_EQ_I(ev_count(APP_EV_ZERO_OK), 1);
    W.rsc_zero_err_counts = 80000.0;                                       /* sensor drifts: ~ +118 Pa offset */
    W.dp = 0.0; run_ms(20000);                                             /* long after the boot window */
    CHECK(ev_count(APP_EV_ZERO_OK) == 1);                                  /* no automatic re-zero in flight */
    app_request_zero(&APP); run_ms(4000);
    CHECK_EQ_I(ev_count(APP_EV_ZERO_OK), 2);                               /* explicit request is honoured */
    W.dp = 600.0; run_ms(2000); NEAR(f16_at(last(1021)->data), 31.298, 0.3);
    W.dp = 300.0; app_request_zero(&APP); run_ms(6000);                    /* request while air is moving: rejected, flagged */
    CHECK(ev_count(APP_EV_ZERO_FAIL) >= 1); CHECK(status_vendor() & SM_F_ZERO_FAILED);
    CHECK(ev_count(APP_EV_ZERO_FAIL) < 3);                                 /* reported once, no event flood */
}
static void test_tx_overrun_is_counted(void)
{
    world_default(); boot(NULL); run_ms(2000); tx_full = 1; run_ms(500);
    CHECK(APP.diag.tx_drops > 20); CHECK(ev_count(APP_EV_TX_DROP) > 20);
    tx_full = 0; uint32_t t0 = NOW_US; run_ms(500); CHECK(count(1028, t0, NOW_US) >= 20);   /* resumes, no burst backlog */
    CHECK(count(1028, t0, NOW_US) <= 26);
}
static void test_init_robustness(void)
{
    app_hw_t hw; memset(&hw, 0, sizeof hw); app_cfg_t c = app_default_cfg();
    CHECK_EQ_I(app_init(NULL, &c, &hw, 0), ADB_ERR_PARAM);
    c.node_id = 0; CHECK_EQ_I(app_init(&APP, &c, &hw, 0), ADB_ERR_PARAM);
    c = app_default_cfg(); c.output_period_us = 0; CHECK_EQ_I(app_init(&APP, &c, &hw, 0), ADB_ERR_PARAM);
    app_step(NULL, 0);                                                     /* must not crash */
    c = app_default_cfg();                                                 /* no hardware at all: every driver rejects its null bus ... */
    CHECK_EQ_I(app_init(&APP, &c, &hw, 0), ADB_OK);                        /* ... the app itself still starts (degraded) and survives stepping */
    CHECK(!APP.ok[0] && !APP.ok[1] && !APP.ok[2]);
    for (uint32_t t = 1000; t < 3000000u; t += 1000u) app_step(&APP, t);
    CHECK(!APP.out.static_valid && !APP.out.dp_valid); CHECK_EQ_I(sm_node_health(&APP.out), 3);
}

int main(void)
{
    RUN(test_normal_flight); RUN(test_static_sensor_failure_and_recovery); RUN(test_differential_sensor_failure);
    RUN(test_static_disagreement); RUN(test_sensor_missing_at_boot_then_recovers); RUN(test_moving_air_during_zero); RUN(test_explicit_zero_after_boot_window);
    RUN(test_tx_overrun_is_counted); RUN(test_init_robustness);
    TEST_SUMMARY(); return g_tests_failed ? 1 : 0;
}
