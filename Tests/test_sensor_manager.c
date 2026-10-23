/* test_sensor_manager.c - scenario tests: fusion, fault handling, recovery, zeroing. Deterministic. */
#include <math.h>
#include "sensor_manager.h"
#include "test_common.h"
#define NEAR(a, b, tol) do { g_tests_run++; double _a = (a), _b = (b); \
    if (!(fabs(_a - _b) <= (tol))) { g_tests_failed++; \
    printf("  FAIL %s:%d: %s ~ %s (got %.4f, expected %.4f)\n", __FILE__, __LINE__, #a, #b, _a, _b); } } while (0)

#define DT 20000u                                   /* 50 Hz */
static uint32_t T;
static int N;
static float noise(float amp) { N++; return amp * sinf((float)N * 0.9f); }
static sm_t sm; static sm_output_t o;

static void fresh(void) { sm_cfg_t c = sm_default_cfg(); T = 1000000u; N = 0; sm_init(&sm, &c); }
/* one 50 Hz cycle: feed what is given (NAN-free), then update */
static void cycle(adb_status_t sa, float a, adb_status_t sb, float b, adb_status_t sd, float d)
{
    T += DT;
    if (sa != ADB_ERR_NOT_READY) sm_feed_static(&sm, SM_CH_STATIC_A, T, sa, a, 20.0f);
    if (sb != ADB_ERR_NOT_READY) sm_feed_static(&sm, SM_CH_STATIC_B, T, sb, b, 21.0f);
    if (sd != ADB_ERR_NOT_READY) sm_feed_diff(&sm, T, sd, d, 1000, 800);
    sm_update(&sm, T, &o);
}
#define OKV ADB_OK
#define NOFEED ADB_ERR_NOT_READY
static void good_cycles(int n, float a0, float b0)
{
    for (int i = 0; i < n; i++) cycle(OKV, a0 + noise(0.3f), OKV, b0 + noise(1.0f), OKV, 100.0f + noise(0.5f));
}

static void test_normal_dual(void)
{
    fresh(); good_cycles(50, 90000.0f, 90000.5f);
    CHECK(o.static_valid); CHECK_EQ_I(o.static_mode, SM_STATIC_DUAL);
    NEAR(o.static_pa, 90000.1, 1.0);
    CHECK(o.static_var_pa2 < 0.2f && o.static_var_pa2 > 0.1f);   /* 1/(1/0.168+1/1.44) = 0.15 */
    CHECK(o.dp_valid); NEAR(o.dp_pa, 100.0, 1.0);
    CHECK(o.temp_valid); NEAR(o.temp_c, 20.5, 1e-3);
    CHECK_EQ_I(o.flags, 0); CHECK_EQ_I(sm_node_health(&o), 0);
    CHECK_EQ_I(o.health[SM_CH_STATIC_A], SM_HEALTH_OK);
}
static void test_one_sensor_fails_and_recovers(void)
{
    fresh(); good_cycles(30, 90000.0f, 90000.5f);
    for (int i = 0; i < 6; i++) cycle(OKV, 90000.0f + noise(0.3f), ADB_ERR_BUS, 0.0f, OKV, 100.0f + noise(0.5f));
    CHECK_EQ_I(o.health[SM_CH_STATIC_B], SM_HEALTH_FAILED);
    CHECK_EQ_I(o.static_mode, SM_STATIC_SINGLE_A); CHECK(o.flags & SM_F_B_FAILED); CHECK(o.flags & SM_F_STATIC_SINGLE);
    CHECK_EQ_I(sm_node_health(&o), 1);                                  /* WARNING: redundancy lost */
    NEAR(o.static_pa, 90000.0, 1.0);
    good_cycles(9, 90000.0f, 90000.5f);                                 /* 9 good < 10 needed: still failed */
    CHECK_EQ_I(o.health[SM_CH_STATIC_B], SM_HEALTH_FAILED);
    good_cycles(2, 90000.0f, 90000.5f);
    CHECK_EQ_I(o.health[SM_CH_STATIC_B], SM_HEALTH_OK); CHECK_EQ_I(o.static_mode, SM_STATIC_DUAL);
}
static void test_both_fail_and_stale(void)
{
    fresh(); good_cycles(30, 90000.0f, 90000.5f);
    for (int i = 0; i < 6; i++) cycle(ADB_ERR_TIMEOUT, 0, ADB_ERR_TIMEOUT, 0, OKV, 100.0f + noise(0.5f));
    CHECK(!o.static_valid); CHECK(o.flags & SM_F_STATIC_NONE); CHECK_EQ_I(sm_node_health(&o), 2);   /* ERROR */
    fresh(); good_cycles(30, 90000.0f, 90000.5f);                       /* silence: no feeds at all */
    for (int i = 0; i < 12; i++) cycle(NOFEED, 0, NOFEED, 0, NOFEED, 0);   /* 240 ms > 200 ms timeout */
    CHECK(!o.static_valid); CHECK(o.flags & SM_F_A_STALE); CHECK(!o.dp_valid); CHECK_EQ_I(sm_node_health(&o), 3);
    for (int i = 0; i < 40; i++) cycle(NOFEED, 0, NOFEED, 0, NOFEED, 0);   /* > 600 ms -> FAILED */
    CHECK(o.flags & SM_F_A_FAILED);
}
static void test_disagreement_arbitration_by_continuity(void)
{
    fresh(); good_cycles(40, 90000.0f, 90000.5f);
    float last = o.static_pa;
    for (int i = 0; i < 12; i++) cycle(OKV, 90000.0f + noise(0.3f), OKV, 90400.0f + noise(1.0f), OKV, 100.0f + noise(0.5f));
    CHECK(o.flags & SM_F_STATIC_DISAGREE); CHECK_EQ_I(o.static_mode, SM_STATIC_SINGLE_A);   /* B (+400 Pa) rejected: A is continuous */
    NEAR(o.static_pa, last, 5.0); CHECK_EQ_I(sm_node_health(&o), 1);
    CHECK(o.static_var_pa2 > 100.0f);                                   /* uncertainty inflated by the disagreement */
    /* sensors converge again: needs 20 consecutive agreeing cycles (hysteresis) */
    good_cycles(10, 90000.0f, 90000.5f); CHECK(o.flags & SM_F_STATIC_DISAGREE);
    good_cycles(15, 90000.0f, 90000.5f);
    CHECK_EQ_I(o.static_mode, SM_STATIC_DUAL); CHECK(!(o.flags & SM_F_STATIC_DISAGREE));
}
static void test_transient_disagreement_is_held(void)
{
    fresh(); good_cycles(40, 90000.0f, 90000.5f); float held = o.static_pa;
    cycle(OKV, 90000.0f, OKV, 90250.0f, OKV, 100.0f);                   /* B glitches +250 once: spike gate AND hold */
    NEAR(o.static_pa, held, 2.0);
    good_cycles(5, 90000.0f, 90000.5f);
    CHECK_EQ_I(o.static_mode, SM_STATIC_DUAL);
}
static void test_no_history_unresolved(void)
{
    fresh();                                                            /* sensors disagree from the very first sample */
    for (int i = 0; i < 10; i++) cycle(OKV, 90000.0f + noise(0.3f), OKV, 90400.0f + noise(1.0f), OKV, 100.0f);
    CHECK(!o.static_valid); CHECK(o.flags & SM_F_STATIC_DISAGREE);      /* refuses to guess */
    CHECK_EQ_I(sm_node_health(&o), 2);
}
static void test_stuck_sensor_detected(void)
{
    fresh(); good_cycles(20, 90000.0f, 90000.5f);
    for (int i = 0; i < 112; i++) cycle(OKV, 90000.0f, OKV, 90000.5f + noise(1.0f), OKV, 100.0f + noise(0.5f));   /* A frozen */
    CHECK(o.flags & SM_F_A_STUCK); CHECK(o.flags & SM_F_A_FAILED); CHECK_EQ_I(o.static_mode, SM_STATIC_SINGLE_B);
}
static void test_spike_rejection_and_real_step(void)
{
    fresh(); good_cycles(40, 90000.0f, 90000.5f);
    cycle(OKV, 95000.0f, OKV, 90000.5f, OKV, 100.0f);                   /* one wild A sample */
    CHECK(o.static_pa < 90100.0f);                                      /* not followed */
    CHECK_EQ_I(o.health[SM_CH_STATIC_A], SM_HEALTH_DEGRADED);
    good_cycles(3, 90000.0f, 90000.5f); CHECK_EQ_I(o.health[SM_CH_STATIC_A], SM_HEALTH_OK);
}
static void test_dp_flags(void)
{
    fresh(); good_cycles(30, 90000.0f, 90000.5f);
    for (int i = 0; i < 30; i++) cycle(OKV, 90000.0f + noise(0.3f), OKV, 90000.5f + noise(1.0f), OKV, -50.0f + noise(0.5f));
    CHECK(o.dp_valid); CHECK(o.flags & SM_F_DP_NEGATIVE); CHECK_EQ_I(sm_node_health(&o), 1);
    for (int i = 0; i < 6; i++) cycle(OKV, 90000.0f + noise(0.3f), OKV, 90000.5f + noise(1.0f), ADB_ERR_RANGE, 0.0f);
    CHECK(!o.dp_valid); CHECK(o.flags & SM_F_DIFF_FAILED); CHECK_EQ_I(sm_node_health(&o), 2);
}
static void test_zeroing(void)
{
    int32_t pr, tr;
    fresh(); good_cycles(10, 90000.0f, 90000.5f); sm_zero_begin(&sm);
    CHECK_EQ_I(sm_zero_result(&sm, &pr, &tr), ADB_ERR_NOT_READY);
    for (int i = 0; i < 100; i++) { T += DT; sm_feed_diff(&sm, T, OKV, 12.0f + noise(0.5f), 12345 + (i % 3) - 1, 800); sm_update(&sm, T, &o);
        if (i < 99) CHECK(o.flags & SM_F_ZEROING); }
    CHECK_EQ_I(sm_zero_state(&sm), SM_ZERO_DONE);
    CHECK_EQ_I(sm_zero_result(&sm, &pr, &tr), ADB_OK); CHECK_EQ_I(pr, 12345); CHECK_EQ_I(tr, 800);
    sm_zero_applied(&sm); CHECK_EQ_I(sm_zero_state(&sm), SM_ZERO_IDLE);
    good_cycles(5, 90000.0f, 90000.5f); CHECK(o.dp_valid);              /* measuring again after apply */
    sm_zero_begin(&sm);                                                 /* moving air: large spread */
    for (int i = 0; i < 100; i++) { T += DT; sm_feed_diff(&sm, T, OKV, 12.0f + ((i & 1) ? 30.0f : -30.0f), 12345, 800); }
    CHECK_EQ_I(sm_zero_state(&sm), SM_ZERO_FAIL_SPREAD);
    CHECK_EQ_I(sm_zero_result(&sm, &pr, &tr), ADB_ERR_NOT_READY);
    sm_update(&sm, T, &o); CHECK(o.flags & SM_F_ZERO_FAILED); CHECK_EQ_I(sm_node_health(&o), 2);
    sm_zero_begin(&sm);                                                 /* steady but huge offset (e.g. tube blocked) */
    for (int i = 0; i < 100; i++) { T += DT; sm_feed_diff(&sm, T, OKV, 400.0f + noise(0.5f), 99999, 800); }
    CHECK_EQ_I(sm_zero_state(&sm), SM_ZERO_FAIL_OFFSET);
}
static void test_init_and_nodehealth(void)
{
    sm_cfg_t c = sm_default_cfg(); sm_t s;
    CHECK_EQ_I(sm_init(&s, &c), ADB_OK); CHECK_EQ_I(sm_init(NULL, &c), ADB_ERR_PARAM);
    c.sigma_a_pa = 0.0f; CHECK_EQ_I(sm_init(&s, &c), ADB_ERR_PARAM);
    c = sm_default_cfg(); c.fail_after_errors = 0; CHECK_EQ_I(sm_init(&s, &c), ADB_ERR_PARAM);
    CHECK_EQ_I(sm_node_health(NULL), 3);
    sm_output_t x = {0}; CHECK_EQ_I(sm_node_health(&x), 3);
    x.static_valid = true; CHECK_EQ_I(sm_node_health(&x), 2);
    x.dp_valid = true; CHECK_EQ_I(sm_node_health(&x), 0);
}
int main(void)
{
    RUN(test_normal_dual); RUN(test_one_sensor_fails_and_recovers); RUN(test_both_fail_and_stale);
    RUN(test_disagreement_arbitration_by_continuity); RUN(test_transient_disagreement_is_held);
    RUN(test_no_history_unresolved); RUN(test_stuck_sensor_detected); RUN(test_spike_rejection_and_real_step);
    RUN(test_dp_flags); RUN(test_zeroing); RUN(test_init_and_nodehealth);
    TEST_SUMMARY(); return g_tests_failed ? 1 : 0;
}
