/* app.h - top-level cooperative application: sensors -> SensorManager -> air data -> DroneCAN frames.
 *
 * SCHEDULING MODEL (bare-metal, no RTOS - justified: 3 sensors, ~100 frames/s, no hard-real-time loops):
 *   app_step() is called from the main loop as fast as possible (>= 1 kHz recommended, never blocks except
 *   during sensor (re)initialisation, <= ~12 ms, at boot and then at most once per reinit_period per failed sensor).
 *   Sensors self-time via their state machines. Fixed-period tasks inside app_step (wrap-safe, no drift):
 *       output task   20 ms (50 Hz): fuse, compute IAS/altitude, publish StaticPressure + IndicatedAirspeed
 *       temperature  100 ms (10 Hz): publish StaticTemperature (sensor temperature - NOT outside-air temperature)
 *       status      1000 ms ( 1 Hz): publish NodeStatus (health, mode, fault flags as vendor-specific code)
 *   Invalid data is NEVER published as a measurement; consumers see the message stop and NodeStatus health degrade.
 *
 * FAILURE BEHAVIOUR: a sensor that fails to initialise is retried every reinit_period; one that fails at run time
 * is tracked by the SensorManager (health FAILED, recovery after N good samples). A dead static sensor degrades
 * to single-sensor operation; a dead differential sensor stops IAS and raises NodeStatus ERROR.
 * UPTIME: NodeStatus uptime is accumulated in 64 bit from the wrap-safe step deltas, so it never goes backwards
 * (consumers read a backwards jump as a node restart). app_step() must therefore be called at least once per ~71 minutes.
 * Watchdog: feed the IWDG/WWDG from the main loop AFTER app_step(); a stuck bus call is then reset by the platform. */
#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stdint.h>
#include "adb_bus.h"
#include "bmp581.h"
#include "ms5611.h"
#include "rsc.h"
#include "airdata.h"
#include "sensor_manager.h"
#include "dronecan.h"

typedef struct {
    adb_bus_t bmp_bus, ms_bus, rsc_ee_bus, rsc_adc_bus;
    bool    (*can_tx)(void *ctx, const dc_frame_t *f);       /* returns false if the frame could not be queued */
    void     *can_ctx;
    void    (*event)(void *ctx, uint16_t code, int32_t arg); /* structured diagnostics, may be NULL (UART/ITM/log ring) */
    void     *event_ctx;
    uint32_t (*can_tx_free)(void *ctx);              /* optional: free TX FIFO slots; lets multi-frame messages be skipped, never split */
    /* Zero-offset safety (see run_zero in app.c):
     *  cold_boot  : platform sets true ONLY after a power-on/brown-out reset, false after watchdog/software/pin reset.
     *               Boot-time auto-zero runs only on a cold boot, because a warm reboot may happen in flight.
     *  zero_load/zero_store : optional non-volatile storage of the last good zero (praw0, traw0). The platform must
     *               protect it with its own CRC and return false when it is missing or corrupt. */
    bool     cold_boot;
    bool    (*zero_load)(void *ctx, int32_t *praw0, int32_t *traw0);
    void    (*zero_store)(void *ctx, int32_t praw0, int32_t traw0);
    void     *zero_ctx;
} app_hw_t;

typedef struct {
    uint8_t         node_id;                 /* PROVISIONAL 42 */
    bmp581_config_t bmp;
    ms5611_osr_t    ms_osr;
    uint8_t         ms_temp_every_n;
    rsc_config_t    rsc;
    airdata_cfg_t   air;
    sm_cfg_t        sm;
    bool            autozero_on_boot;        /* zero RSC at start-up (aircraft stationary, no wind on the probe!) */
    uint32_t        autozero_window_us;      /* attempts allowed only within this time after boot */
    uint32_t        output_period_us, temp_period_us, status_period_us, reinit_period_us, raw_period_us;
    bool            pub_raw_air_data, pub_ias;   /* which airspeed messages the autopilot needs: CONFIRM with the integrator */
    float           dp_sigma_pa;             /* 1-sigma dp error after zero (RSC: 0.25 %FSS = 30 Pa), for IAS variance */
} app_cfg_t;

app_cfg_t app_default_cfg(void);

typedef enum {                               /* diagnostic event codes (arg meaning in brackets) */
    APP_EV_BOOT = 1, APP_EV_INIT_OK = 10, APP_EV_INIT_FAIL = 11,          /* [sensor | status<<8]; sensor 0=BMP581 1=MS5611 2=RSC */
    APP_EV_ZERO_OK = 20, APP_EV_ZERO_FAIL = 21, APP_EV_ZERO_ABORT = 22,   /* [praw0] / [sm_zero_state] */
    APP_EV_ZERO_RESTORED = 23, APP_EV_ZERO_SKIPPED_WARM = 24,                /* [praw0] / [0] */
    APP_EV_FLAGS = 30, APP_EV_STATIC_MODE = 31, APP_EV_RSC_CRC_UNVERIFIED = 40, APP_EV_RSC_PART_MISMATCH = 41,
    APP_EV_TX_DROP = 50
} app_event_t;

typedef struct {
    uint32_t out_cycles, frames_sent, tx_drops, ias_rejected, raw_skipped, init_attempts[3];
    adb_status_t last_status[3];
} app_diag_t;

typedef struct {
    app_cfg_t cfg; app_hw_t hw;
    bmp581_t bmp; ms5611_t ms; rsc_t rsc; bool ok[3];
    sm_t sm; sm_output_t out; dc_node_t node;
    float ias_ms, ias_var, altitude_m; adb_status_t ias_status, alt_status;   /* latest computed values */
    uint32_t t_boot, t_out, t_temp, t_status, t_raw, t_reinit[3];
    uint64_t uptime_us; uint32_t last_step_us;             /* 64-bit accumulation: survives the 71.6 min wrap of the 32-bit us clock */
    bool zero_pending, zero_forced, zero_fail_reported, zero_restored; uint32_t zero_started_us; uint32_t prev_flags; sm_static_mode_t prev_mode; bool seen_valid;
    app_diag_t diag;
} app_t;

adb_status_t app_init(app_t *a, const app_cfg_t *cfg, const app_hw_t *hw, uint32_t now_us);
void         app_step(app_t *a, uint32_t now_us);
void         app_request_zero(app_t *a);      /* operator/maintenance command; only call when the probe is at rest */

#endif
