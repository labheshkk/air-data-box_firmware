/* airdata.h - air-data calculations (hardware independent, no dynamic memory).
 *
 * UNITS: pressures in Pa, temperatures in degC, speeds in m/s, altitude in m.
 * Single-precision float is used deliberately: the STM32H7 FPU is single-precision
 * hardware and the sensors' own resolution (>= 0.1 Pa) is far coarser than float
 * round-off (1 part in 1e7 -> ~0.004 m in altitude, see tests).
 *
 * MODEL LIMITS (also documented in docs/assumptions_open_questions.md):
 *  - airspeed_ias : V = sqrt(2*q/rho0). Equals true airspeed only at ISA sea-level
 *                   density, and ignores air compressibility. Measured against the
 *                   compressible formula (see tests): +0.1 % at 31 m/s, +1.0 % at 99 m/s.
 *  - airspeed_cas : subsonic compressible impact-pressure formula (ISA constants).
 *  - airspeed_tas : uses local density from static pressure and air temperature.
 *                   The temperature passed in must be OUTSIDE-AIR temperature; the
 *                   on-chip sensor temperature is NOT that (self-heating, enclosure).
 *  - altitude     : PRESSURE altitude from the ISA troposphere model (valid
 *                   -1000..11000 m). Not GPS altitude, not a calibrated aviation altitude.
 *  All thresholds in airdata_cfg_t are PROVISIONAL until accepted by the customer. */
#ifndef AIRDATA_H
#define AIRDATA_H

#include "adb_status.h"

typedef struct {
    float rho0_kg_m3;        /* reference density, ISA sea level 1.225               */
    float dp_neg_tol_pa;     /* negative dp down to -tol is treated as noise (->0)   */
    float dp_deadband_pa;    /* |dp| below this reports 0 m/s (kills zero-offset jitter) */
    float dp_max_pa;         /* sensor full-scale (RSC060MD: 6000 Pa)                */
} airdata_cfg_t;

/* Provisional defaults. */
airdata_cfg_t airdata_default_cfg(void);

/* Indicated airspeed, incompressible, at rho0.
 *  ADB_OK            valid (dp in [-tol, max]); speed 0 inside the dead-band
 *  ADB_ERR_RANGE     dp < -tol (likely swapped tubes) or dp > max; *v_ms = 0 / clamped max
 *  ADB_ERR_INVALID   NaN / Inf input; *v_ms = 0
 *  ADB_ERR_PARAM     NULL pointer or invalid cfg */
adb_status_t airdata_airspeed_ias(const airdata_cfg_t *cfg, float dp_pa, float *v_ms);

/* Calibrated airspeed, subsonic compressible formula (a0 = 340.294 m/s, p0 = 101325 Pa). Same status rules. */
adb_status_t airdata_airspeed_cas(const airdata_cfg_t *cfg, float dp_pa, float *v_ms);

/* True airspeed with local density rho = p / (R * T). static_pa must be 10 kPa..120 kPa,
 * oat_c -80..60 degC, otherwise ADB_ERR_RANGE (speed still 0). Same dp rules as IAS. */
adb_status_t airdata_airspeed_tas(const airdata_cfg_t *cfg, float dp_pa, float static_pa,
                                  float oat_c, float *v_ms);

/* Pressure altitude above the reference pressure (default ISA 101325 Pa; pass a QNH to
 * get altitude above that datum). ADB_ERR_RANGE outside -1000..11000 m (value still returned). */
adb_status_t airdata_pressure_altitude(float static_pa, float ref_pa, float *alt_m);

#endif
