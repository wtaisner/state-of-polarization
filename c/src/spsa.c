#include "spsa.h"
#include "fixedpoint.h"
#include "polctrl.h"

/*
 * SPSA with per-coordinate boundary weighting.
 *
 * Gradient estimate (per coordinate i):
 *   ĝ_i = (y_plus - y_minus) / (2 * c_{k,i} * delta_i)
 *
 * Update:
 *   theta_i = clamp(theta_i + a_gain * ĝ_i, V_MIN, V_MAX)
 *   then snap to 0.1V grid
 *
 * Boundary weight:
 *   w(theta) = 1.0 for theta in [BOUNDARY_MARGIN, V_MAX - BOUNDARY_MARGIN]
 *   w(theta) = linear ramp from BOUNDARY_FLOOR_WEIGHT to 1.0 in edge zones
 *
 * c_{k,i} = c_gain * w(theta_i)
 *
 * If disable_boundary_weight (SEARCH mode): w = 1.0 everywhere, so c_{k,i} = c_gain.
 * This is a conscious deviation from 3d — in SEARCH we want aggressive exploration
 * everywhere, including near edges (spec section 3e).
 */

fp_t boundary_weight(fp_t theta_i)
{
    fp_t lo_margin = BOUNDARY_MARGIN_Q88;      /* 5V in Q8.8 = 1280 */
    fp_t hi_margin = V_MAX_Q88 - BOUNDARY_MARGIN_Q88;  /* 55V = 14080 */

    /* In center zone: weight = 1.0 */
    if (theta_i >= lo_margin && theta_i <= hi_margin) {
        return FP_ONE;
    }

    /* In edge zone: linear ramp from BOUNDARY_FLOOR_WEIGHT to 1.0 */
    fp_t dist_from_edge;
    fp_t range;

    if (theta_i < lo_margin) {
        /* Near 0V: dist = theta_i, range = lo_margin */
        dist_from_edge = theta_i;
        range = lo_margin;
    } else {
        /* Near V_MAX: dist = V_MAX - theta_i, range = lo_margin */
        dist_from_edge = V_MAX_Q88 - theta_i;
        range = lo_margin;
    }

    if (range <= 0) {
        return BOUNDARY_FLOOR_WEIGHT_Q88;
    }

    /* weight = BOUNDARY_FLOOR + (1.0 - BOUNDARY_FLOOR) * (dist / range) */
    /* In Q8.8: floor + (FP_ONE - floor) * fp_div(dist, range) */
    fp_t scale = fp_div(dist_from_edge, range);
    fp_t ramp = fp_mul(FP_ONE - BOUNDARY_FLOOR_WEIGHT_Q88, scale);
    fp_t weight = BOUNDARY_FLOOR_WEIGHT_Q88 + ramp;

    /* Clamp to valid range */
    if (weight < BOUNDARY_FLOOR_WEIGHT_Q88) weight = BOUNDARY_FLOOR_WEIGHT_Q88;
    if (weight > FP_ONE) weight = FP_ONE;

    return weight;
}

int8_t forced_inward_sign(fp_t theta_i)
{
    if (theta_i < BOUNDARY_FORCE_INWARD_Q88) {
        /* Very close to 0V: force positive (inward) */
        return 1;
    }
    if (theta_i > V_MAX_Q88 - BOUNDARY_FORCE_INWARD_Q88) {
        /* Very close to V_MAX: force negative (inward) */
        return -1;
    }
    /* Not at extreme: random sign allowed */
    return 0;
}

fp_t snap_to_voltage_grid(fp_t v)
{
    /*
     * Snap to nearest 0.1V grid step, then clamp to [V_MIN, V_MAX].
     * V_STEP_Q88 = 26 (0.1V in Q8.8, rounded from 25.6).
     * Note: V_MAX_Q88 (15360) is not an exact multiple of 26, so the
     * maximum grid point is 26*590 = 15340 (59.92V). Values above that
     * snap to 15366 which is then clamped to V_MAX_Q88.
     */
    fp_t half_step = V_STEP_Q88 / 2;
    fp_t grid_idx;
    fp_t result;

    if (v >= 0) {
        grid_idx = (v + half_step) / V_STEP_Q88;
    } else {
        grid_idx = (v - half_step) / V_STEP_Q88;
    }
    result = grid_idx * V_STEP_Q88;

    /* Clamp to voltage range */
    if (result < V_MIN_Q88) result = V_MIN_Q88;
    if (result > V_MAX_Q88) result = V_MAX_Q88;

    return result;
}

SpsaState spsa_compute_probe(SpsaState s,
                             fp_t out_plus[NUM_SECTIONS],
                             fp_t out_minus[NUM_SECTIONS],
                             fp_t a_gain,
                             fp_t c_gain,
                             uint8_t disable_boundary_weight)
{
    (void)a_gain;  /* a_gain used in apply_result, not here */

    for (uint8_t i = 0; i < NUM_SECTIONS; i++) {
        fp_t weight;
        int8_t force_sign;
        int8_t sign;

        /* Compute boundary weight */
        if (disable_boundary_weight) {
            weight = FP_ONE;
        } else {
            weight = boundary_weight(s.theta[i]);
        }

        /* Per-coordinate perturbation size */
        s.c_k[i] = fp_mul(c_gain, weight);

        /* Determine perturbation direction */
        force_sign = forced_inward_sign(s.theta[i]);
        if (force_sign != 0) {
            sign = force_sign;
        } else {
            sign = rng_sign(&s.rng);
        }

        /* Store delta as Q8.8 (±FP_ONE) */
        s.delta[i] = (sign > 0) ? FP_ONE : (fp_t)(-FP_ONE);

        /* Compute probe voltages */
        fp_t perturbation = fp_mul(s.c_k[i], s.delta[i]);
        out_plus[i] = fp_clamp(s.theta[i] + perturbation, V_MIN_Q88, V_MAX_Q88);
        out_minus[i] = fp_clamp(s.theta[i] - perturbation, V_MIN_Q88, V_MAX_Q88);
    }

    return s;
}

SpsaState spsa_apply_result(SpsaState s, fp_t y_plus, fp_t y_minus,
                            fp_t a_gain)
{
    fp_t y_diff = y_plus - y_minus;

    for (uint8_t i = 0; i < NUM_SECTIONS; i++) {
        /*
         * ĝ_i = (y_plus - y_minus) / (2 * c_{k,i} * delta_i)
         *
         * Since delta_i is ±1 (stored as ±FP_ONE):
         *   2 * c_k * delta = 2 * c_k * (±1) = ±(2 * c_k)
         *
         * So: ĝ_i = y_diff / (2 * c_k_i * delta_i)
         *          = y_diff * delta_i / (2 * c_k_i)    [since 1/delta = delta for ±1]
         *
         * Wait, actually 1/(delta) = delta since delta = ±1.
         * So ĝ_i = y_diff / (2 * c_k_i) * delta_i... no.
         *
         * Let me be careful:
         * ĝ_i = (y_plus - y_minus) / (2 * c_{k,i} * delta_i)
         *
         * delta_i = ±1 (as FP_ONE or -FP_ONE)
         * 2 * c_{k,i} * delta_i = fp_mul(2*c_k, delta) = 2*c_k * (±1)
         *
         * But actually delta is stored as ±FP_ONE (= ±256).
         * c_k is in Q8.8.
         * So 2 * c_k * delta in Q8.8 terms:
         *   fp_mul(fp_mul(2*c_k, delta), ...) -- but 2*c_k might overflow.
         *
         * Simpler approach:
         *   denominator = fp_mul(s.c_k[i], s.delta[i])  -- this is c_k * delta
         *   Then multiply by 2: denominator2 = denominator + denominator
         *   ĝ_i = fp_div(y_diff, denominator2)
         *
         * But if c_k is 0 (degenerate), fp_div handles it (returns saturating value).
         */

        fp_t denom = fp_mul(s.c_k[i], s.delta[i]);  /* c_k * delta (Q8.8) */
        fp_t denom2 = denom + denom;                  /* 2 * c_k * delta */

        fp_t grad = fp_div(y_diff, denom2);

        /* Clamp gradient to prevent wild updates from numerical issues */
        /* Max reasonable gradient: y_diff range ~7680, denom min ~1, so grad ~7680 */
        /* Clamp to FP_MAX to stay in int16_t */
        grad = fp_clamp(grad, -FP_ONE * 4, FP_ONE * 4);  /* ±4.0 in Q8.8 */

        s.last_grad_estimate[i] = grad;

        /* Update: theta_i += a_gain * ĝ_i */
        fp_t step = fp_mul(a_gain, grad);
        fp_t new_theta = s.theta[i] + step;

        /* Clamp to voltage range */
        new_theta = fp_clamp(new_theta, V_MIN_Q88, V_MAX_Q88);

        /* Snap to 0.1V grid */
        new_theta = snap_to_voltage_grid(new_theta);

        /* Final clamp after snapping */
        new_theta = fp_clamp(new_theta, V_MIN_Q88, V_MAX_Q88);

        s.theta[i] = new_theta;
    }

    return s;
}
