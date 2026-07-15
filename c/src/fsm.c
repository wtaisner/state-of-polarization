#include "fsm.h"
#include "polctrl.h"

/*
 * FSM logic: TRACK / SEARCH / RECOVERY with dead-zone and hysteresis.
 *
 * Thresholds (Q8.8):
 *   k1 = K1_DEADZONE_Q88 = 640  (2.5 sigma)
 *   k2 = K2_SEARCH_Q88   = 2048 (8.0 sigma)
 *
 * Sudden fade detection:
 *   If y_fast drops far below y_slow (by more than k2 * sigma approximation),
 *   trigger SEARCH immediately. This is independent of the z-score which
 *   uses y_slow (which lags).
 *
 * We use a simplified sudden-fade check:
 *   If (y_slow - y_fast) > (y_slow / 4), i.e., y_fast is more than 25%
 *   below y_slow, trigger SEARCH. This is a heuristic that doesn't need
 *   sigma (which may not be reliable during transients).
 */

/* Sudden fade threshold: y_fast must be this much below y_slow (as fraction of y_slow) */
/* Using Q8.8: threshold = y_slow * 64 / 256 = y_slow / 4 */
#define SUDDEN_FADE_FRAC_Q88  64   /* 0.25 * 256 */

FsmState fsm_init(void)
{
    FsmState s;
    s.mode = STATE_TRACK;
    s.consecutive_good_windows = 0;
    s.periodic_probe_counter = 0;
    return s;
}

FsmState fsm_update(FsmState s, fp_t zscore, fp_t y_fast, fp_t y_slow)
{
    /* Check for sudden fade (y_fast far below y_slow) */
    /* Only meaningful if y_slow is positive (initialized) */
    fp_t fade_threshold;
    fp_t drop;

    if (y_slow > 0) {
        fade_threshold = fp_mul(y_slow, SUDDEN_FADE_FRAC_Q88);
        drop = y_slow - y_fast;
        if (drop > fade_threshold) {
            s.mode = STATE_SEARCH;
            s.consecutive_good_windows = 0;
            return s;
        }
    }

    switch (s.mode) {
    case STATE_TRACK:
        /* Enter SEARCH if zscore exceeds k2 */
        if (zscore > K2_SEARCH_Q88) {
            s.mode = STATE_SEARCH;
            s.consecutive_good_windows = 0;
        }
        break;

    case STATE_SEARCH:
        /* Exit to RECOVERY when zscore drops below k1 */
        if (zscore < K1_DEADZONE_Q88) {
            s.mode = STATE_RECOVERY;
            s.consecutive_good_windows = 1;
        }
        break;

    case STATE_RECOVERY:
        if (zscore < K1_DEADZONE_Q88) {
            /* Still good: count consecutive good windows */
            s.consecutive_good_windows++;
            if (s.consecutive_good_windows >= HYSTERESIS_WINDOWS) {
                s.mode = STATE_TRACK;
                s.consecutive_good_windows = 0;
            }
        } else if (zscore > K2_SEARCH_Q88) {
            /* Regression: back to SEARCH */
            s.mode = STATE_SEARCH;
            s.consecutive_good_windows = 0;
        } else {
            /* zscore between k1 and k2: reset counter */
            s.consecutive_good_windows = 0;
        }
        break;
    }

    return s;
}

uint8_t fsm_should_actuate(FsmState s, fp_t zscore)
{
    /* In SEARCH mode: always actuate */
    if (s.mode == STATE_SEARCH) {
        return 1;
    }
    /* In TRACK/RECOVERY: actuate if zscore > k1 (dead-zone gate) */
    if (zscore > K1_DEADZONE_Q88) {
        return 1;
    }
    return 0;
}

uint8_t fsm_check_periodic_probe(FsmState *s)
{
    /* No periodic probe in SEARCH mode (already exploring) */
    if (s->mode == STATE_SEARCH) {
        s->periodic_probe_counter = 0;
        return 0;
    }

    s->periodic_probe_counter++;
    if (s->periodic_probe_counter >= PERIODIC_PROBE_INTERVAL) {
        s->periodic_probe_counter = 0;
        return 1;
    }
    return 0;
}

SpsaGainProfile fsm_gain_for_mode(ControllerMode mode,
                                   SpsaGainProfile bandit_selected_profile)
{
    if (mode == STATE_SEARCH) {
        /* SEARCH mode: use fixed aggressive profile, ignore bandit */
        SpsaGainProfile search_profile;
        search_profile.a_gain = SEARCH_GAIN_A_Q88;
        search_profile.c_gain = SEARCH_GAIN_C_Q88;
        return search_profile;
    }
    /* TRACK / RECOVERY: use bandit-selected profile */
    return bandit_selected_profile;
}
