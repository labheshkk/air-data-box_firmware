/* test_airdata.c - reference values from an independent Python/double model (ISA constants). */
#include <math.h>
#include "airdata.h"
#include "test_common.h"

#define NEAR(a, b, tol) do { g_tests_run++; double _a = (a), _b = (b); \
    if (fabs(_a - _b) > (tol)) { g_tests_failed++; \
    printf("  FAIL %s:%d: %s ~ %s (got %.6f, expected %.6f)\n", __FILE__, __LINE__, #a, #b, _a, _b); } } while (0)

static void test_ias(void)
{
    airdata_cfg_t c = airdata_default_cfg(); float v;
    CHECK_EQ_I(airdata_airspeed_ias(&c, 600.0f, &v), ADB_OK);
    NEAR(v, 31.298432, 1e-3);                       /* sqrt(2*600/1.225) */
    CHECK_EQ_I(airdata_airspeed_ias(&c, 6000.0f, &v), ADB_OK);
    NEAR(v, 98.974332, 1e-2);                       /* sensor full scale */
}
static void test_ias_edge_cases(void)
{
    airdata_cfg_t c = airdata_default_cfg(); float v;
    CHECK_EQ_I(airdata_airspeed_ias(&c, 0.0f, &v), ADB_OK);   NEAR(v, 0.0, 0.0);
    CHECK_EQ_I(airdata_airspeed_ias(&c, 2.0f, &v), ADB_OK);   NEAR(v, 0.0, 0.0);   /* inside dead-band */
    CHECK_EQ_I(airdata_airspeed_ias(&c, -4.0f, &v), ADB_OK);  NEAR(v, 0.0, 0.0);   /* small negative = offset noise */
    CHECK_EQ_I(airdata_airspeed_ias(&c, -50.0f, &v), ADB_ERR_RANGE); NEAR(v, 0.0, 0.0);  /* reversed tubes */
    CHECK_EQ_I(airdata_airspeed_ias(&c, 7000.0f, &v), ADB_ERR_RANGE); NEAR(v, 98.974332, 1e-2); /* clamped */
    CHECK_EQ_I(airdata_airspeed_ias(&c, NAN, &v), ADB_ERR_INVALID);   NEAR(v, 0.0, 0.0);
    CHECK_EQ_I(airdata_airspeed_ias(&c, INFINITY, &v), ADB_ERR_INVALID);
    CHECK_EQ_I(airdata_airspeed_ias(NULL, 1.0f, &v), ADB_ERR_PARAM);
    CHECK_EQ_I(airdata_airspeed_ias(&c, 1.0f, NULL), ADB_ERR_PARAM);
    airdata_cfg_t bad = c; bad.rho0_kg_m3 = 0.0f;
    CHECK_EQ_I(airdata_airspeed_ias(&bad, 100.0f, &v), ADB_ERR_PARAM);   /* would divide by zero */
}
static void test_cas(void)
{
    airdata_cfg_t c = airdata_default_cfg(); float v;
    CHECK_EQ_I(airdata_airspeed_cas(&c, 600.0f, &v), ADB_OK);   NEAR(v, 31.265431, 2e-3);
    CHECK_EQ_I(airdata_airspeed_cas(&c, 6000.0f, &v), ADB_OK);  NEAR(v, 97.956570, 1e-2);
    /* compressibility makes CAS smaller than the incompressible figure: ~1 % at 99 m/s */
}
static void test_tas(void)
{
    airdata_cfg_t c = airdata_default_cfg(); float v;
    CHECK_EQ_I(airdata_airspeed_tas(&c, 600.0f, 90000.0f, 15.0f, &v), ADB_OK);
    NEAR(v, 33.209293, 2e-3);                        /* rho = 1.08808 kg/m3 */
    CHECK_EQ_I(airdata_airspeed_tas(&c, 600.0f, 5000.0f, 15.0f, &v), ADB_ERR_RANGE);
    CHECK_EQ_I(airdata_airspeed_tas(&c, 600.0f, 90000.0f, -200.0f, &v), ADB_ERR_RANGE);
    CHECK_EQ_I(airdata_airspeed_tas(&c, 600.0f, NAN, 15.0f, &v), ADB_ERR_INVALID);
}
static void test_altitude(void)
{
    float h;
    CHECK_EQ_I(airdata_pressure_altitude(101325.0f, 101325.0f, &h), ADB_OK);  NEAR(h, 0.0, 1e-3);
    CHECK_EQ_I(airdata_pressure_altitude(89874.6f, 101325.0f, &h), ADB_OK);   NEAR(h, 999.9966, 0.02);  /* ISA 1000 m */
    CHECK_EQ_I(airdata_pressure_altitude(95000.0f, 101325.0f, &h), ADB_OK);   NEAR(h, 540.3371, 0.02);
    CHECK_EQ_I(airdata_pressure_altitude(95000.0f, 102500.0f, &h), ADB_OK);   NEAR(h, 636.2932, 0.02);  /* with QNH */
    CHECK_EQ_I(airdata_pressure_altitude(22632.06f, 101325.0f, &h), ADB_OK);  NEAR(h, 10999.994, 0.1);  /* tropopause */
    CHECK_EQ_I(airdata_pressure_altitude(20000.0f, 101325.0f, &h), ADB_ERR_RANGE);   /* above model limit, value returned */
    CHECK(h > 11000.0f);
    CHECK_EQ_I(airdata_pressure_altitude(0.0f, 101325.0f, &h), ADB_ERR_RANGE);
    CHECK_EQ_I(airdata_pressure_altitude(-5.0f, 101325.0f, &h), ADB_ERR_RANGE);
    CHECK_EQ_I(airdata_pressure_altitude(NAN, 101325.0f, &h), ADB_ERR_INVALID);
    CHECK_EQ_I(airdata_pressure_altitude(90000.0f, 101325.0f, NULL), ADB_ERR_PARAM);
}
int main(void)
{
    RUN(test_ias); RUN(test_ias_edge_cases); RUN(test_cas); RUN(test_tas); RUN(test_altitude);
    TEST_SUMMARY();
    return g_tests_failed ? 1 : 0;
}
