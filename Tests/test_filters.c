#include <math.h>
#include "filters.h"
#include "test_common.h"
#define NEAR(a, b, tol) do { g_tests_run++; double _a = (a), _b = (b); \
    if (!(fabs(_a - _b) <= (tol))) { g_tests_failed++; \
    printf("  FAIL %s:%d: %s ~ %s (got %.6f, expected %.6f)\n", __FILE__, __LINE__, #a, #b, _a, _b); } } while (0)

static void test_lpf(void)
{
    lpf_t f; lpf_init(&f, 1.0f);                       /* fc = 1 Hz -> tau = 0.159155 s */
    NEAR(lpf_update(&f, 10.0f, 0.01f), 10.0, 0);       /* first sample initialises */
    float y = 0; for (int i = 0; i < 100; i++) y = lpf_update(&f, 20.0f, 0.01f);   /* 1 s of a step to 20 */
    NEAR(y, 10.0 + 10.0 * (1.0 - exp(-1.0 / 0.159155)), 0.05);      /* analytic first-order step response */
    for (int i = 0; i < 2000; i++) y = lpf_update(&f, 20.0f, 0.01f);
    NEAR(y, 20.0, 1e-3);
    float held = lpf_update(&f, NAN, 0.01f); NEAR(held, y, 0);       /* NaN ignored */
    lpf_t p; lpf_init(&p, 0.0f); lpf_update(&p, 1.0f, 0.01f); NEAR(lpf_update(&p, 5.0f, 0.01f), 5.0, 0);  /* pass-through */
    lpf_reset(&f); NEAR(lpf_update(&f, 3.0f, 0.01f), 3.0, 0);
}
static void test_stuck(void)
{
    stuck_t s; stuck_init(&s, 5, 0.0f);
    for (int i = 0; i < 4; i++) CHECK(!stuck_update(&s, 100.0f));
    CHECK(stuck_update(&s, 100.0f));                   /* 5th identical sample */
    CHECK(!stuck_update(&s, 100.5f));                  /* any change clears it */
    stuck_init(&s, 3, 0.2f);                           /* tolerance band */
    stuck_update(&s, 50.0f); stuck_update(&s, 50.1f); CHECK(stuck_update(&s, 49.9f));
    stuck_init(&s, 0, 0.0f); CHECK(!stuck_update(&s, 1.0f));   /* disabled */
}
static void test_spike(void)
{
    spike_t s; float o; spike_init(&s, 100.0f, 3);
    CHECK(spike_update(&s, 1000.0f, &o)); NEAR(o, 1000, 0);
    CHECK(!spike_update(&s, 5000.0f, &o)); NEAR(o, 1000, 0);        /* single outlier rejected */
    CHECK(spike_update(&s, 1010.0f, &o)); NEAR(o, 1010, 0);         /* back to normal clears candidate */
    CHECK(!spike_update(&s, 3000.0f, &o)); CHECK(!spike_update(&s, 3005.0f, &o));
    CHECK(spike_update(&s, 3010.0f, &o)); NEAR(o, 3010, 0);         /* 3 consistent samples = real step */
    CHECK(!spike_update(&s, NAN, &o)); NEAR(o, 3010, 0);
}
int main(void) { RUN(test_lpf); RUN(test_stuck); RUN(test_spike); TEST_SUMMARY(); return g_tests_failed ? 1 : 0; }
