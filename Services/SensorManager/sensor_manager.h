/* sensor_manager.h - validity/health tracking, dual static-pressure fusion, differential zeroing.
 *
 * Hardware independent: it is fed (status, value) results from the drivers and produces one
 * consolidated, flagged air-data snapshot. All thresholds are PROVISIONAL (see sm_default_cfg).
 *
 * Static-pressure strategy (documented decision, not a customer requirement):
 *  1. A sensor is USABLE if its last driver result was fresh (< stale_timeout), passed the spike
 *     gate and stuck check, and its health is not FAILED.
 *  2. Both usable and |A-B| <= threshold: inverse-variance weighted blend (the lower-noise BMP581 counts more).
 *  3. Both usable but |A-B| > threshold: output HOLDS the last fused value for `confirm` cycles
 *     (transient/glitch rejection), then ARBITRATES by continuity - the sensor closer to the last fused
 *     value is trusted - and the disagreement is latched until the sensors agree again with hysteresis.
 *     If there is no history to arbitrate with, the static output is declared INVALID (never a coin toss).
 *  4. One usable: that sensor alone, flagged SINGLE, variance as that sensor. None: INVALID. */
#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "adb_status.h"
#include "filters.h"

typedef enum { SM_CH_STATIC_A = 0, SM_CH_STATIC_B, SM_CH_DIFF, SM_CH_COUNT } sm_channel_t;   /* A = BMP581, B = MS5611 */
typedef enum { SM_HEALTH_UNKNOWN = 0, SM_HEALTH_OK, SM_HEALTH_DEGRADED, SM_HEALTH_FAILED } sm_health_t;
typedef enum { SM_STATIC_NONE = 0, SM_STATIC_DUAL, SM_STATIC_SINGLE_A, SM_STATIC_SINGLE_B } sm_static_mode_t;
typedef enum { SM_ZERO_IDLE = 0, SM_ZERO_COLLECTING, SM_ZERO_DONE, SM_ZERO_FAIL_SPREAD, SM_ZERO_FAIL_OFFSET } sm_zero_state_t;

/* Fault / status bits in sm_output_t.flags */
#define SM_F_A_FAILED       (1u << 0)
#define SM_F_B_FAILED       (1u << 1)
#define SM_F_DIFF_FAILED    (1u << 2)
#define SM_F_A_STALE        (1u << 3)
#define SM_F_B_STALE        (1u << 4)
#define SM_F_DIFF_STALE     (1u << 5)
#define SM_F_A_STUCK        (1u << 6)
#define SM_F_B_STUCK        (1u << 7)
#define SM_F_STATIC_DISAGREE (1u << 8)
#define SM_F_STATIC_SINGLE  (1u << 9)
#define SM_F_STATIC_NONE    (1u << 10)
#define SM_F_DP_NEGATIVE    (1u << 11)
#define SM_F_ZERO_FAILED    (1u << 12)
#define SM_F_ZEROING        (1u << 13)
#define SM_F_DP_RANGE       (1u << 14)

typedef struct {
    uint32_t stale_timeout_us;          /* no valid sample for this long -> unusable (stale)          */
    uint16_t fail_after_errors;         /* consecutive bad results -> FAILED                           */
    uint16_t recover_after_good;        /* consecutive good results to leave FAILED                    */
    float    static_disagree_pa;        /* PROVISIONAL 150 Pa                                          */
    uint8_t  disagree_confirm;          /* cycles of disagreement before arbitration                   */
    uint8_t  agree_recover_cycles;      /* cycles of agreement (|d| <= thr/2) to release arbitration   */
    float    sigma_a_pa, sigma_b_pa;    /* 1-sigma noise for weights: BMP581 ~0.4 Pa, MS5611 ~1.2 Pa   */
    float    static_spike_pa;           /* max step per sample before spike gate (confirm samples to accept) */
    uint8_t  spike_confirm;
    uint16_t stuck_samples;             /* identical consecutive samples -> stuck                      */
    float    static_lpf_hz, dp_lpf_hz;
    float    dp_spike_pa;
    float    dp_neg_tol_pa;             /* below -tol the dp is flagged negative                       */
    uint16_t zero_samples;              /* samples averaged for zeroing                                */
    float    zero_max_abs_pa;           /* reject zero if |mean| larger (RSC TEB 0.75 %FSS = 90 Pa)    */
    float    zero_max_std_pa;           /* reject zero if noisy/moving air                             */
} sm_cfg_t;

typedef struct {
    sm_health_t health; uint16_t consec_err, consec_good; uint32_t last_good_us; bool have;
    float value, temp_c; stuck_t stuck; spike_t spike; uint32_t err_total, good_total; adb_status_t last_status;
} sm_chan_t;

typedef struct {
    bool             static_valid;
    sm_static_mode_t static_mode;
    float            static_pa;          /* fused + low-passed */
    float            static_var_pa2;
    float            temp_c;             /* mean sensor temperature; NOT outside-air temperature */
    bool             temp_valid;
    bool             dp_valid;
    float            dp_pa;              /* zero-corrected (by driver), filtered */
    sm_health_t      health[SM_CH_COUNT];
    uint32_t         flags;
} sm_output_t;

typedef struct {
    sm_cfg_t cfg; sm_chan_t ch[SM_CH_COUNT];
    enum { ARB_AGREE = 0, ARB_SUSPECT, ARB_A, ARB_B, ARB_UNRESOLVED } arb; uint8_t suspect_n, recover_n;
    float last_fused; bool have_last_fused; lpf_t static_lpf, dp_lpf; uint32_t last_update_us; bool upd_init;
    sm_zero_state_t zero_state; uint16_t zero_n; double zs_p, zs_pp, zs_raw, zs_t; int32_t zero_praw0, zero_traw0; float dp_last;
} sm_t;

sm_cfg_t     sm_default_cfg(void);
adb_status_t sm_init(sm_t *sm, const sm_cfg_t *cfg);

/* Feed one driver result. status != ADB_OK counts as an error for that channel. */
void sm_feed_static(sm_t *sm, sm_channel_t ch, uint32_t now_us, adb_status_t status, float pressure_pa, float temp_c);
void sm_feed_diff(sm_t *sm, uint32_t now_us, adb_status_t status, float dp_pa, int32_t praw, int32_t tcode);

/* Compute the consolidated snapshot. Call at the output rate (>= 50 Hz). */
void sm_update(sm_t *sm, uint32_t now_us, sm_output_t *out);

/* Zero procedure (tubes unpressurised / equal pressure on both ports). */
void             sm_zero_begin(sm_t *sm);
sm_zero_state_t  sm_zero_state(const sm_t *sm);
adb_status_t     sm_zero_result(const sm_t *sm, int32_t *praw0, int32_t *traw0);   /* ADB_OK only in SM_ZERO_DONE */
void             sm_zero_abort(sm_t *sm);                                           /* give up collecting / clear a latched failure */
void             sm_zero_applied(sm_t *sm);                                         /* resets dp filters after driver zero */

/* DroneCAN NodeStatus health: 0 OK, 1 WARNING, 2 ERROR, 3 CRITICAL */
uint8_t sm_node_health(const sm_output_t *o);

#endif
