#ifndef POLCTRL_INTERNAL_H
#define POLCTRL_INTERNAL_H

#include "fixedpoint.h"
#include "polctrl.h"
#include "rng.h"
#include "baseline.h"
#include "spsa.h"
#include "fsm.h"
#include "bandit.h"

/*
 * ====================================================================
 * Internal structures for the polarization controller.
 * Not exposed to HAL — only used by polctrl.c and tests.
 * ====================================================================
 */

/*
 * Input scaling convention (see polctrl.h):
 *
 *   beatnote_reading (fp_t, Q8.8) = (dBm + 65) * 256
 *
 *   Range: [0, 7680] for dBm in [-65, -35].
 *   Default ceiling (-38 dBm) = 6912 in Q8.8.
 *
 * All internal power values (baseline, y_fast, y_slow, sigma) use this
 * same unit. The z-score is dimensionless (Q8.8 ratio).
 */

/* SPSA sub-state machine (internal to polctrl_step) */
typedef enum {
    SPSA_SUB_IDLE,            /* Not in SPSA cycle */
    SPSA_SUB_SET_PLUS,        /* About to set theta_plus */
    SPSA_SUB_MEASURE_PLUS,    /* Settling, measuring y_plus */
    SPSA_SUB_SET_MINUS,       /* About to set theta_minus */
    SPSA_SUB_MEASURE_MINUS,   /* Settling, measuring y_minus */
    SPSA_SUB_APPLY            /* Compute gradient, update theta */
} SpsaSubState;

/* Full controller state */
struct PolCtrlState {
    /* Sub-module states */
    BaselineState baseline;
    FsmState fsm;
    SpsaState spsa;     /* contains rng_state */
    BanditState bandit;

    /* SPSA sub-state machine */
    SpsaSubState spsa_sub;
    uint16_t spsa_settle_counter;     /* Samples remaining in settle phase */
    fp_t y_plus;                      /* Measured power at theta_plus */
    fp_t y_minus;                     /* Measured power at theta_minus */

    /* Bandit integration */
    uint16_t bandit_iter_counter;     /* SPSA iterations since last bandit update */
    uint8_t current_arm;              /* Currently selected arm */
    uint8_t current_context;          /* Current context bucket */

    /* Reward accumulator (for bandit window) */
    fp_t reward_power_sum;            /* Sum of y_slow in window */
    uint16_t reward_power_count;
    fp_t reward_boundary_sum;         /* Sum of boundary proximity fraction */
    fp_t reward_movement_sum;         /* Sum of |delta_theta| */

    /* Drift estimate (EMA of mean |grad|) */
    fp_t drift_estimate;

    /* Periodic probe counter */
    uint32_t periodic_probe_counter;

    /* Step counter */
    uint32_t step_count;

    /* Current gain profile (from bandit or SEARCH) */
    SpsaGainProfile current_gain;
};

/* ARM_PROFILES declared in bandit.h */
/* SEARCH_GAIN_PROFILE defined below */

#endif /* POLCTRL_INTERNAL_H */
