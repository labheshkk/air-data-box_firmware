/* test_can_supervisor.c - bus-off recovery with back-off, quiet-period reset, stuck-TX flush. */
#include "can_supervisor.h"
#include "test_common.h"

static cansup_t S;
static void fresh(void) { cansup_cfg_t c = cansup_default_cfg(); cansup_init(&S, &c); }
/* run time forward in 1 ms steps with constant inputs; returns time of first RESTART or 0 */
static uint32_t run(uint32_t *t, uint32_t ms, bool bus_off, uint32_t tx_free)
{
    for (uint32_t i = 0; i < ms; i++) { *t += 1000u; if (cansup_step(&S, *t, bus_off, tx_free) == CANSUP_RESTART) return *t; }
    return 0;
}

static void test_healthy_bus_never_restarts(void)
{
    fresh(); uint32_t t = 5000000u;
    CHECK_EQ_I(run(&t, 20000, false, 3), 0); CHECK_EQ_I(S.busoff_events, 0);
}
static void test_busoff_recovers_after_backoff(void)
{
    fresh(); uint32_t t = 1000000u, t0 = t;
    uint32_t r = run(&t, 200, true, 3);
    CHECK(r != 0); CHECK(r - t0 >= 100000u && r - t0 <= 102000u);        /* ~100 ms back-off, not instantly */
    CHECK_EQ_I(S.restarts_busoff, 1); CHECK_EQ_I(S.busoff_events, 1);
}
static void test_backoff_doubles_and_caps(void)
{
    fresh(); uint32_t t = 0, last = 0, prev_gap = 0;
    uint32_t gaps[6];
    for (int i = 0; i < 6; i++) {                                        /* bus stays off after every restart */
        uint32_t t_start = t; uint32_t r = run(&t, 3000, true, 3); CHECK(r != 0); gaps[i] = r - t_start; last = r; (void)prev_gap;
    }
    CHECK(gaps[0] >= 100000u && gaps[0] < 103000u);
    CHECK(gaps[1] >= 200000u && gaps[1] < 203000u);
    CHECK(gaps[2] >= 400000u && gaps[2] < 403000u);
    CHECK(gaps[3] >= 800000u && gaps[3] < 803000u);
    CHECK(gaps[4] >= 1000000u && gaps[4] < 1003000u);                    /* capped at 1 s */
    CHECK(gaps[5] >= 1000000u && gaps[5] < 1003000u);
    CHECK(last != 0);
}
static void test_quiet_period_resets_backoff(void)
{
    fresh(); uint32_t t = 0;
    run(&t, 3000, true, 3); run(&t, 3000, true, 3);                      /* two restarts: back-off now 400 ms */
    CHECK(S.backoff_us >= 400000u);
    run(&t, 11000, false, 3);                                            /* healthy for 11 s */
    uint32_t t0 = t; uint32_t r = run(&t, 3000, true, 3);
    CHECK(r - t0 >= 100000u && r - t0 < 103000u);                        /* back to the initial 100 ms */
}
static void test_recovers_if_busoff_clears_before_restart(void)
{
    fresh(); uint32_t t = 0;
    run(&t, 50, true, 3);                                                /* bus-off, restart pending */
    CHECK_EQ_I(run(&t, 500, false, 3), 0);                               /* cleared by itself: no unnecessary restart */
    CHECK_EQ_I(S.restarts_busoff, 0);
}
static void test_tx_stuck_flush(void)
{
    fresh(); uint32_t t = 0;
    CHECK_EQ_I(run(&t, 900, false, 0), 0);                               /* full for 0.9 s: not yet */
    uint32_t r = run(&t, 200, false, 0); CHECK(r != 0); CHECK_EQ_I(S.restarts_tx_stuck, 1);
    t = 0; fresh();
    for (int i = 0; i < 40; i++) { run(&t, 500, false, 0); run(&t, 1, false, 2); }   /* never full for 1 s continuously */
    CHECK_EQ_I(S.restarts_tx_stuck, 0);
}
static void test_null_safety(void)
{
    cansup_init(NULL, NULL); CHECK_EQ_I(cansup_step(NULL, 0, true, 0), CANSUP_NONE);
    cansup_t s; cansup_init(&s, NULL); CHECK_EQ_I(s.backoff_us, 100000u);
}
int main(void)
{
    RUN(test_healthy_bus_never_restarts); RUN(test_busoff_recovers_after_backoff); RUN(test_backoff_doubles_and_caps);
    RUN(test_quiet_period_resets_backoff); RUN(test_recovers_if_busoff_clears_before_restart); RUN(test_tx_stuck_flush); RUN(test_null_safety);
    TEST_SUMMARY(); return g_tests_failed ? 1 : 0;
}
