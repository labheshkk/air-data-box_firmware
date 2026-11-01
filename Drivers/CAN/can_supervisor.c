/* can_supervisor.c - see can_supervisor.h */
#include "can_supervisor.h"
#include <stddef.h>

cansup_cfg_t cansup_default_cfg(void)
{
    cansup_cfg_t c = { 100000u, 1000000u, 10000000u, 1000000u };
    return c;
}

void cansup_init(cansup_t *s, const cansup_cfg_t *cfg)
{
    if (!s) return;
    *s = (cansup_t){0};
    s->cfg = cfg ? *cfg : cansup_default_cfg();
    s->backoff_us = s->cfg.backoff_initial_us;
}

static bool due(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }

cansup_action_t cansup_step(cansup_t *s, uint32_t now_us, bool bus_off, uint32_t tx_free)
{
    if (!s) return CANSUP_NONE;
    if (bus_off) {
        if (!s->restart_pending) {                           /* new bus-off episode: schedule recovery */
            s->restart_pending = true; s->busoff_events++;
            s->t_restart_due = now_us + s->backoff_us;
        } else if (due(now_us, s->t_restart_due)) {
            s->restart_pending = false;
            s->t_last_restart = now_us; s->restarts_busoff++;
            s->backoff_us = (s->backoff_us > s->cfg.backoff_max_us / 2u) ? s->cfg.backoff_max_us : s->backoff_us * 2u;
            s->full_active = false;
            return CANSUP_RESTART;
        }
        return CANSUP_NONE;
    }
    s->restart_pending = false;                              /* controller is on the bus again */
    if (s->restarts_busoff > 0u && (uint32_t)(now_us - s->t_last_restart) >= s->cfg.quiet_reset_us)
        s->backoff_us = s->cfg.backoff_initial_us;           /* link healthy for a while: forgive */

    if (tx_free == 0u) {                                     /* FIFO full */
        if (!s->full_active) { s->full_active = true; s->t_full_since = now_us; }
        else if ((uint32_t)(now_us - s->t_full_since) >= s->cfg.tx_stuck_us) {
            s->full_active = false; s->restarts_tx_stuck++; s->t_last_restart = now_us;
            return CANSUP_RESTART;
        }
    } else s->full_active = false;
    return CANSUP_NONE;
}
