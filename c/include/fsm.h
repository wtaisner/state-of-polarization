#ifndef FSM_H
#define FSM_H

#include <stdint.h>
#include "fixedpoint.h"
#include "polctrl.h"

/*
 * Finite state machine: TRACK / SEARCH / RECOVERY.
 *
 * State transitions:
 *   TRACK   -> SEARCH:   zscore > k2 OR sudden fast drop (y_fast << y_slow)
 *   SEARCH  -> RECOVERY: zscore < k1 (start counting hysteresis windows)
 *   RECOVERY-> TRACK:    HYSTERESIS_WINDOWS consecutive windows with zscore < k1
 *   RECOVERY-> SEARCH:   zscore > k2 (regression)
 *
 * Dead-zone gate: fsm_should_actuate returns 1 if zscore > k1
 * (i.e., degradation exceeds dead-zone threshold).
 *
 * Periodic probe: every PERIODIC_PROBE_INTERVAL samples, force one
 * SPSA round even in dead-zone (except in SEARCH mode).
 */

typedef enum {
    STATE_TRACK,
    STATE_SEARCH,
    STATE_RECOVERY
} ControllerMode;

typedef struct {
    ControllerMode mode;
    uint16_t consecutive_good_windows;  /* For SEARCH -> TRACK hysteresis */
    uint32_t periodic_probe_counter;    /* Counts samples since last probe */
} FsmState;

/* Initialize FSM state. */
FsmState fsm_init(void);

/*
 * Update FSM state based on z-score.
 * Also takes y_fast and y_slow for sudden-fade detection.
 */
FsmState fsm_update(FsmState s, fp_t zscore, fp_t y_fast, fp_t y_slow);

/*
 * Dead-zone gate: should the controller actuate (run SPSA)?
 * Returns 1 if zscore > k1, or if periodic probe is due, or in SEARCH mode.
 */
uint8_t fsm_should_actuate(FsmState s, fp_t zscore);

/*
 * Check if a periodic probe is due (and not in SEARCH mode).
 * Resets the counter if due.
 * Returns 1 if probe should happen.
 */
uint8_t fsm_check_periodic_probe(FsmState *s);

/*
 * Get SPSA gain profile for current mode.
 * In SEARCH: returns SEARCH_GAIN_PROFILE (ignores bandit).
 * In TRACK/RECOVERY: returns bandit_selected_profile.
 */
typedef struct {
    fp_t a_gain;
    fp_t c_gain;
} SpsaGainProfile;

SpsaGainProfile fsm_gain_for_mode(ControllerMode mode,
                                   SpsaGainProfile bandit_selected_profile);

#endif /* FSM_H */
