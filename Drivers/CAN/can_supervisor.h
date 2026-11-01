/* can_supervisor.h - CAN link supervision policy (hardware independent, unit-tested).
 *
 * Two failures would otherwise leave the node silent forever:
 *   1. BUS-OFF: the controller disconnects after too many errors and stays off until the software restarts it.
 *   2. TX STUCK: the TX FIFO stays full (nobody acknowledges, wiring fault). DroneCAN spec 4.1: "Message transmission
 *      should be aborted if it cannot be completed in 1 second" - flushing the queue also avoids sending stale data
 *      the moment the bus comes back.
 * Policy: restart the controller after a back-off that doubles on repeated bus-offs (so a permanently broken bus does
 * not cause a restart storm) and returns to the initial value after a quiet period. The platform layer performs the
 * actual restart (HAL stop/start) when told to. */
#ifndef CAN_SUPERVISOR_H
#define CAN_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t backoff_initial_us;     /* PROVISIONAL 100 ms */
    uint32_t backoff_max_us;         /* PROVISIONAL 1 s    */
    uint32_t quiet_reset_us;         /* no bus-off for this long -> back-off returns to initial (PROVISIONAL 10 s) */
    uint32_t tx_stuck_us;            /* TX FIFO full this long -> flush/restart (spec: 1 s) */
} cansup_cfg_t;

typedef enum { CANSUP_NONE = 0, CANSUP_RESTART } cansup_action_t;

typedef struct {
    cansup_cfg_t cfg;
    bool     restart_pending, full_active;
    uint32_t t_restart_due, t_last_restart, t_full_since, backoff_us;
    uint32_t busoff_events, restarts_busoff, restarts_tx_stuck;
} cansup_t;

cansup_cfg_t    cansup_default_cfg(void);
void            cansup_init(cansup_t *s, const cansup_cfg_t *cfg);

/* Call periodically (every main-loop pass is fine). bus_off: controller reports bus-off.
 * tx_free: free TX FIFO slots right now. Returns CANSUP_RESTART when the platform must restart the controller. */
cansup_action_t cansup_step(cansup_t *s, uint32_t now_us, bool bus_off, uint32_t tx_free);

#endif
