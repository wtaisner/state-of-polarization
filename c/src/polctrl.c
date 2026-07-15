#include "polctrl.h"
#include "polctrl_internal.h"
#include "fixedpoint.h"
#include "rng.h"
#include "baseline.h"
#include "spsa.h"
#include "fsm.h"
#include "bandit.h"

/*
 * ====================================================================
 * PolCtrl — Main controller implementation.
 *
 * polctrl_step is called every 1ms. It:
 *   1. Updates baseline (EMA fast/slow, adaptive ceiling, noise sigma)
 *   2. Computes z-score from baseline
 *   3. Updates FSM (TRACK/SEARCH/RECOVERY)
 *   4. Runs SPSA sub-state machine if triggered
 *   5. Updates bandit periodically
 *
 * SPSA sub-state machine (internal):
 *   IDLE -> SET_PLUS -> MEASURE_PLUS -> SET_MINUS -> MEASURE_MINUS -> APPLY -> IDLE
 *
 * The SET_PLUS/SET_MINUS states output probe voltages (actuate=1).
 * The MEASURE_PLUS/MEASURE_MINUS states wait for y_slow to settle (actuate=0).
 * The APPLY state computes gradient and updates theta (actuate=1 with new theta).
 * ====================================================================
 */

/* Initial voltage: 30V (middle of range) */
#define INITIAL_VOLTAGE_Q88  7680   /* 30 * 256 */

/* EMA alpha for drift estimate (uses slow EMA rate) */
#define ALPHA_DRIFT_Q88  ALPHA_SLOW_Q88

PolCtrlState polctrl_init(uint32_t rng_seed)
{
    PolCtrlState state;
    uint8_t i;

    state.baseline = baseline_init();
    state.fsm = fsm_init();
    state.bandit = bandit_init();

    /* Initialize SPSA state */
    for (i = 0; i < NUM_SECTIONS; i++) {
        state.spsa.theta[i] = INITIAL_VOLTAGE_Q88;
        state.spsa.last_grad_estimate[i] = 0;
        state.spsa.delta[i] = 0;
        state.spsa.c_k[i] = 0;
    }
    state.spsa.rng = rng_init(rng_seed);

    /* SPSA sub-state */
    state.spsa_sub = SPSA_SUB_IDLE;
    state.spsa_settle_counter = 0;
    state.y_plus = 0;
    state.y_minus = 0;

    /* Bandit integration */
    state.bandit_iter_counter = 0;
    state.current_arm = 0;
    state.current_context = 0;

    /* Reward accumulator */
    state.reward_power_sum = 0;
    state.reward_power_count = 0;
    state.reward_boundary_sum = 0;
    state.reward_movement_sum = 0;

    /* Drift estimate */
    state.drift_estimate = 0;

    /* Periodic probe */
    state.periodic_probe_counter = 0;

    /* Step count */
    state.step_count = 0;

    /* Current gain profile (default: arm 0) */
    state.current_gain = ARM_PROFILES[0];

    return state;
}

/*
 * Helper: compute boundary proximity fraction for reward.
 * Returns count of sections in boundary zone / NUM_SECTIONS, in Q8.8.
 */
static fp_t compute_boundary_fraction(const SpsaState *spsa)
{
    uint8_t i;
    uint8_t count = 0;
    for (i = 0; i < NUM_SECTIONS; i++) {
        fp_t w = boundary_weight(spsa->theta[i]);
        /* If weight < 1.0, section is in boundary zone */
        if (w < FP_ONE) {
            count++;
        }
    }
    /* fraction = count / NUM_SECTIONS, in Q8.8 */
    return (fp_t)((uint32_t)count * FP_ONE / NUM_SECTIONS);
}

/*
 * Helper: compute total voltage movement |delta_theta| in Q8.8.
 */
static fp_t compute_movement(const SpsaState *old_spsa,
                              const SpsaState *new_spsa)
{
    uint8_t i;
    fp_t total = 0;
    for (i = 0; i < NUM_SECTIONS; i++) {
        fp_t diff = new_spsa->theta[i] - old_spsa->theta[i];
        total += fp_abs(diff);
    }
    return total;
}

/*
 * Helper: update drift estimate from gradient.
 * drift = EMA of mean(|grad_i|) across sections.
 */
static fp_t update_drift(fp_t current_drift, const SpsaState *spsa)
{
    uint8_t i;
    fp_t sum = 0;
    fp_t mean_grad;
    fp_t diff;

    for (i = 0; i < NUM_SECTIONS; i++) {
        sum += fp_abs(spsa->last_grad_estimate[i]);
    }
    /* mean = sum / NUM_SECTIONS */
    mean_grad = (fp_t)((int32_t)sum / NUM_SECTIONS);

    /* EMA update */
    diff = mean_grad - current_drift;
    return current_drift + fp_mul(ALPHA_DRIFT_Q88, diff);
}

PolCtrlState polctrl_step(PolCtrlState state, fp_t beatnote_reading,
                          PolCtrlOutput *out)
{
    fp_t zscore;
    uint8_t should_actuate;
    uint8_t periodic_probe;
    uint8_t currently_in_deadzone;
    SpsaState old_spsa;

    state.step_count++;

    /* Default: don't actuate */
    out->actuate = 0;
    for (uint8_t i = 0; i < NUM_SECTIONS; i++) {
        out->voltages[i] = state.spsa.theta[i];
    }

    /* --- Determine dead-zone status (for sigma update) --- */
    /* We're in dead-zone if SPSA is idle and we're not in SEARCH */
    currently_in_deadzone = (state.spsa_sub == SPSA_SUB_IDLE) &&
                            (state.fsm.mode != STATE_SEARCH);

    /* --- Update baseline --- */
    state.baseline = baseline_update(state.baseline, beatnote_reading,
                                      currently_in_deadzone);

    /* --- Compute z-score --- */
    zscore = baseline_zscore(state.baseline);

    /* --- Update FSM (always, even during SPSA) --- */
    state.fsm = fsm_update(state.fsm, zscore,
                           state.baseline.y_fast,
                           state.baseline.y_slow);

    /* --- SPSA sub-state machine --- */
    switch (state.spsa_sub) {
    case SPSA_SUB_IDLE:
        /* Check if we should start a new SPSA round */
        should_actuate = fsm_should_actuate(state.fsm, zscore);
        periodic_probe = fsm_check_periodic_probe(&state.fsm);

        if (should_actuate || periodic_probe) {
            /* Determine gain profile */
            if (state.fsm.mode == STATE_SEARCH) {
                state.current_gain = fsm_gain_for_mode(STATE_SEARCH,
                                                       state.current_gain);
            } else {
                /* Select arm via bandit (if window is complete) */
                state.current_context = discretize_context(
                    state.drift_estimate,
                    state.baseline.noise_sigma);
                state.current_arm = bandit_select_arm(state.bandit,
                                                      state.current_context);
                state.current_gain = ARM_PROFILES[state.current_arm];
            }

            /* Start SPSA round */
            state.spsa_sub = SPSA_SUB_SET_PLUS;
        }
        break;

    case SPSA_SUB_SET_PLUS:
        /* Compute probe voltages */
        {
            fp_t theta_plus[NUM_SECTIONS];
            fp_t theta_minus[NUM_SECTIONS];
            uint8_t disable_bw = (state.fsm.mode == STATE_SEARCH) ? 1 : 0;

            state.spsa = spsa_compute_probe(state.spsa, theta_plus,
                                            theta_minus,
                                            state.current_gain.a_gain,
                                            state.current_gain.c_gain,
                                            disable_bw);
            /* Output theta_plus */
            out->actuate = 1;
            for (uint8_t i = 0; i < NUM_SECTIONS; i++) {
                out->voltages[i] = theta_plus[i];
            }
        }
        state.spsa_settle_counter = SPSA_SETTLE_SAMPLES;
        state.spsa_sub = SPSA_SUB_MEASURE_PLUS;
        break;

    case SPSA_SUB_MEASURE_PLUS:
        if (state.spsa_settle_counter > 0) {
            state.spsa_settle_counter--;
        } else {
            /* Record y_plus */
            state.y_plus = state.baseline.y_slow;
            state.spsa_sub = SPSA_SUB_SET_MINUS;
        }
        break;

    case SPSA_SUB_SET_MINUS:
        /* Output theta_minus (already computed in SET_PLUS) */
        {
            fp_t theta_plus[NUM_SECTIONS];
            fp_t theta_minus[NUM_SECTIONS];
            uint8_t disable_bw = (state.fsm.mode == STATE_SEARCH) ? 1 : 0;

            /* Recompute to get theta_minus (delta and c_k already stored) */
            /* Actually, we stored delta and c_k in SET_PLUS, so we can
             * reconstruct theta_minus directly. */
            for (uint8_t i = 0; i < NUM_SECTIONS; i++) {
                fp_t perturbation = fp_mul(state.spsa.c_k[i],
                                           state.spsa.delta[i]);
                theta_minus[i] = fp_clamp(
                    state.spsa.theta[i] - perturbation,
                    V_MIN_Q88, V_MAX_Q88);
            }
            /* Suppress unused warning */
            (void)theta_plus;
            (void)disable_bw;

            out->actuate = 1;
            for (uint8_t i = 0; i < NUM_SECTIONS; i++) {
                out->voltages[i] = theta_minus[i];
            }
        }
        state.spsa_settle_counter = SPSA_SETTLE_SAMPLES;
        state.spsa_sub = SPSA_SUB_MEASURE_MINUS;
        break;

    case SPSA_SUB_MEASURE_MINUS:
        if (state.spsa_settle_counter > 0) {
            state.spsa_settle_counter--;
        } else {
            /* Record y_minus */
            state.y_minus = state.baseline.y_slow;
            state.spsa_sub = SPSA_SUB_APPLY;
        }
        /* After settling, need to set theta_minus voltage on the last
         * settle step. Actually, the voltage was already set in SET_MINUS.
         * During MEASURE_MINUS, we keep the voltage (no actuate needed). */
        break;

    case SPSA_SUB_APPLY:
        /* Save old spsa for movement computation */
        old_spsa = state.spsa;

        /* Apply SPSA result */
        state.spsa = spsa_apply_result(state.spsa, state.y_plus,
                                       state.y_minus,
                                       state.current_gain.a_gain);

        /* Output new theta */
        out->actuate = 1;
        for (uint8_t i = 0; i < NUM_SECTIONS; i++) {
            out->voltages[i] = state.spsa.theta[i];
        }

        /* Update drift estimate */
        state.drift_estimate = update_drift(state.drift_estimate,
                                            &state.spsa);

        /* Update reward accumulator */
        state.reward_power_sum += state.baseline.y_slow;
        state.reward_power_count++;
        state.reward_boundary_sum += compute_boundary_fraction(&state.spsa);
        state.reward_movement_sum += compute_movement(&old_spsa,
                                                       &state.spsa);

        /* Bandit window check */
        state.bandit_iter_counter++;
        if (state.bandit_iter_counter >= BANDIT_WINDOW_ITERATIONS &&
            state.fsm.mode != STATE_SEARCH) {
            /* Compute reward */
            /* reward = mean(y_slow) - LAMBDA * boundary_frac - MU * movement */
            fp_t mean_power = fp_div(state.reward_power_sum,
                                     (fp_t)(state.reward_power_count << FP_SHIFT));
            fp_t mean_boundary = fp_div(state.reward_boundary_sum,
                                        (fp_t)(state.reward_power_count << FP_SHIFT));
            fp_t mean_movement = fp_div(state.reward_movement_sum,
                                        (fp_t)(state.reward_power_count << FP_SHIFT));
            fp_t reward = mean_power
                - fp_mul(LAMBDA_BOUNDARY_Q88, mean_boundary)
                - fp_mul(MU_MOVEMENT_Q88, mean_movement);

            /* Update bandit */
            state.bandit = bandit_update(state.bandit,
                                         state.current_context,
                                         state.current_arm,
                                         reward);

            /* Reset accumulators */
            state.reward_power_sum = 0;
            state.reward_power_count = 0;
            state.reward_boundary_sum = 0;
            state.reward_movement_sum = 0;
            state.bandit_iter_counter = 0;
        }

        /* Return to idle */
        state.spsa_sub = SPSA_SUB_IDLE;
        break;

    default:
        /* Should never happen */
        state.spsa_sub = SPSA_SUB_IDLE;
        break;
    }

    return state;
}
