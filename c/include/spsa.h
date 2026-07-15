#ifndef SPSA_H
#define SPSA_H

#include <stdint.h>
#include "fixedpoint.h"
#include "rng.h"
#include "polctrl.h"
#include "fsm.h"

/*
 * SPSA (Simultaneous Perturbation Stochastic Approximation) optimizer
 * with per-coordinate boundary weighting.
 *
 * Two-phase API (intentional — see spec section 3d):
 *   1. spsa_compute_probe: generates theta_plus/theta_minus to set on actuator
 *   2. spsa_apply_result:  after measuring y_plus/y_minus, computes gradient & updates theta
 *
 * Boundary weighting:
 *   c_{k,i} = c_gain * boundary_weight(theta_i)
 *   boundary_weight is 1.0 in center, decreases to BOUNDARY_FLOOR_WEIGHT near edges.
 *
 * Forced inward:
 *   If theta_i very close to 0 or V_MAX, force perturbation direction inward.
 */

typedef struct {
    fp_t theta[NUM_SECTIONS];              /* Current voltages (Q8.8, volts) */
    fp_t last_grad_estimate[NUM_SECTIONS]; /* Last gradient (for bandit context) */
    rng_state_t rng;                       /* PRNG state (shared with controller) */
    /* Stored between compute_probe and apply_result */
    fp_t delta[NUM_SECTIONS];              /* Perturbation directions (±1 in Q8.8) */
    fp_t c_k[NUM_SECTIONS];                /* Per-coordinate perturbation sizes */
} SpsaState;

/*
 * Compute probe voltages (theta_plus, theta_minus).
 *
 * Parameters:
 *   s            - current SPSA state
 *   out_plus     - output: theta + delta * c_k (Q8.8 volts)
 *   out_minus    - output: theta - delta * c_k (Q8.8 volts)
 *   a_gain       - SPSA step size gain (Q8.8)
 *   c_gain       - SPSA perturbation size gain (Q8.8)
 *   disable_boundary_weight - if 1, flatten boundary_weight to 1.0 (SEARCH mode)
 *
 * Returns updated state (with delta and c_k stored).
 */
SpsaState spsa_compute_probe(SpsaState s,
                             fp_t out_plus[NUM_SECTIONS],
                             fp_t out_minus[NUM_SECTIONS],
                             fp_t a_gain,
                             fp_t c_gain,
                             uint8_t disable_boundary_weight);

/*
 * Apply measurement results: compute gradient and update theta.
 *
 * Parameters:
 *   s       - current SPSA state (must have been through compute_probe)
 *   y_plus  - measured power at theta_plus (Q8.8)
 *   y_minus - measured power at theta_minus (Q8.8)
 *   a_gain  - SPSA step size gain (Q8.8)
 *
 * Returns updated state with new theta.
 */
SpsaState spsa_apply_result(SpsaState s, fp_t y_plus, fp_t y_minus,
                            fp_t a_gain);

/*
 * Boundary weight: 1.0 in center, BOUNDARY_FLOOR_WEIGHT near edges.
 * theta_i in Q8.8 volts.
 */
fp_t boundary_weight(fp_t theta_i);

/*
 * Forced inward sign: +1 if theta near 0, -1 if near V_MAX, 0 otherwise.
 */
int8_t forced_inward_sign(fp_t theta_i);

/*
 * Snap voltage to the 0.1V grid.
 */
fp_t snap_to_voltage_grid(fp_t v);

#endif /* SPSA_H */
