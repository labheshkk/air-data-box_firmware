/* airdata.c - see airdata.h */
#include "airdata.h"
#include <math.h>
#include <stddef.h>

/* ISA constants (ICAO Doc 7488) */
#define ISA_P0_PA      101325.0f
#define ISA_T0_K       288.15f
#define ISA_LAPSE      0.0065f            /* K/m, troposphere       */
#define ISA_R          287.05287f         /* J/(kg K) dry air       */
#define ISA_G          9.80665f
#define ISA_A0_MS      340.294f           /* speed of sound at 15 C */
#define ISA_EXP        0.190263f          /* R*L/g                  */
#define ALT_MIN_M      (-1000.0f)
#define ALT_MAX_M      11000.0f

airdata_cfg_t airdata_default_cfg(void)
{
    airdata_cfg_t c = { 1.225f, 10.0f, 3.0f, 6000.0f };
    return c;
}

static int cfg_ok(const airdata_cfg_t *c)
{
    return c && c->rho0_kg_m3 > 0.5f && c->rho0_kg_m3 < 2.0f && c->dp_neg_tol_pa >= 0.0f &&
           c->dp_deadband_pa >= 0.0f && c->dp_max_pa > 0.0f;
}

/* Common differential-pressure gate. Returns status; *q is the usable dynamic pressure (>= 0). */
static adb_status_t gate_dp(const airdata_cfg_t *c, float dp, float *q)
{
    *q = 0.0f;
    if (!isfinite(dp)) return ADB_ERR_INVALID;
    if (dp < -c->dp_neg_tol_pa) return ADB_ERR_RANGE;       /* reversed tubes / big negative offset */
    if (dp > c->dp_max_pa) { *q = c->dp_max_pa; return ADB_ERR_RANGE; }
    if (dp < c->dp_deadband_pa) return ADB_OK;              /* includes small negatives: speed 0 */
    *q = dp;
    return ADB_OK;
}

adb_status_t airdata_airspeed_ias(const airdata_cfg_t *cfg, float dp_pa, float *v_ms)
{
    if (!cfg_ok(cfg) || !v_ms) return ADB_ERR_PARAM;
    float q;
    adb_status_t s = gate_dp(cfg, dp_pa, &q);
    *v_ms = sqrtf(2.0f * q / cfg->rho0_kg_m3);
    return s;
}

adb_status_t airdata_airspeed_cas(const airdata_cfg_t *cfg, float dp_pa, float *v_ms)
{
    if (!cfg_ok(cfg) || !v_ms) return ADB_ERR_PARAM;
    float q;
    adb_status_t s = gate_dp(cfg, dp_pa, &q);
    /* CAS = a0 * sqrt( 5 * [ (qc/p0 + 1)^(2/7) - 1 ] ), valid for M < 1 */
    *v_ms = ISA_A0_MS * sqrtf(5.0f * (powf(q / ISA_P0_PA + 1.0f, 2.0f / 7.0f) - 1.0f));
    return s;
}

adb_status_t airdata_airspeed_tas(const airdata_cfg_t *cfg, float dp_pa, float static_pa,
                                  float oat_c, float *v_ms)
{
    if (!cfg_ok(cfg) || !v_ms) return ADB_ERR_PARAM;
    *v_ms = 0.0f;
    if (!isfinite(static_pa) || !isfinite(oat_c)) return ADB_ERR_INVALID;
    if (static_pa < 10000.0f || static_pa > 120000.0f || oat_c < -80.0f || oat_c > 60.0f)
        return ADB_ERR_RANGE;
    float q;
    adb_status_t s = gate_dp(cfg, dp_pa, &q);
    float rho = static_pa / (ISA_R * (oat_c + 273.15f));
    *v_ms = sqrtf(2.0f * q / rho);
    return s;
}

adb_status_t airdata_pressure_altitude(float static_pa, float ref_pa, float *alt_m)
{
    if (!alt_m) return ADB_ERR_PARAM;
    *alt_m = 0.0f;
    if (!isfinite(static_pa) || !isfinite(ref_pa)) return ADB_ERR_INVALID;
    if (static_pa <= 0.0f || ref_pa <= 0.0f) return ADB_ERR_RANGE;
    /* h = (T0/L) * (1 - (p/p_ref)^(R*L/g)) */
    float h = (ISA_T0_K / ISA_LAPSE) * (1.0f - powf(static_pa / ref_pa, ISA_EXP));
    *alt_m = h;
    return (h < ALT_MIN_M || h > ALT_MAX_M) ? ADB_ERR_RANGE : ADB_OK;
}
