#ifndef POLCTRL_H
#define POLCTRL_H

#include <stdint.h>
#include "fixedpoint.h"

/*
 * ====================================================================
 * PolCtrl — Polarization Controller Core (SPSA + adaptive baseline + bandit)
 * ====================================================================
 *
 * Public API for the polarization controller.
 *
 * HAL CONTRACT (see README_HAL.md for full details):
 * -----------------------------------------------
 * The hardware abstraction layer (HAL) on the target MCU must:
 *
 *   1. Call polctrl_init(rng_seed) once at system startup.
 *   2. Every 1 ms, call polctrl_step(state, beatnote_reading, &output).
 *   3. If output.actuate == 1, set the 4 DAC channels to output.voltages[].
 *   4. Provide beatnote_reading as a Q8.8 value representing (dBm + 65).
 *      Formula: beatnote_reading = (fp_t)((dBm + 65.0) * 256)
 *      Range: [0, 7680] for dBm in [-65, -35].
 *
 * The HAL implements NO decision logic — all algorithm logic is inside
 * polctrl_step. The HAL only does: timer -> ADC read -> polctrl_step ->
 * DAC write (if actuate).
 *
 * Internal unit convention:
 *   beatnote_reading_raw is in "shifted dBm" Q8.8:
 *     value = (dBm + 65) * 256
 *   So -65 dBm -> 0, -38 dBm -> 6912, -35 dBm -> 7680.
 *   voltages[] are in Q8.8 volts: 0V -> 0, 60V -> 15360.
 * ====================================================================
 */

/* === Section count === */
#define NUM_SECTIONS 4

/* === Voltage range [V] === */
#define V_MIN_VOLT   0
#define V_MAX_VOLT   60
#define V_STEP_VOLT  0.1f

/* === Voltage in Q8.8 === */
/* 0.1 * 256 = 25.6 -> 26 (nearest integer) */
#define V_STEP_Q88   26
#define V_MIN_Q88    0
#define V_MAX_Q88    15360   /* 60 * 256 */

/* === Beatnote range [dBm] === */
#define BEATNOTE_MIN_DBM  (-65)
#define BEATNOTE_MAX_DBM  (-35)

/* === Internal unit scaling === */
/* beatnote_reading = (dBm + 65) * 256, range [0, 7680] */
#define BEATNOTE_OFFSET  65   /* dBm offset added before scaling */

/* === Sampling === */
#define SAMPLING_PERIOD_MS  1

/* === EMA time constants (in samples at 1ms) === */
#define TAU_FAST_MS   8
#define TAU_SLOW_MS   75

/* EMA alpha = 1/tau (Q8.8): alpha = FP_ONE / tau */
#define ALPHA_FAST_Q88  32    /* 256/8 = 32 */
#define ALPHA_SLOW_Q88  3     /* 256/75 = 3.41 -> 3 */

/* === Dead-zone / SEARCH thresholds (multiples of sigma) === */
/* k1 = 5/2 = 2.5, k2 = 8/1 = 8.0 */
#define K1_DEADZONE_NUM  5
#define K1_DEADZONE_DEN  2
#define K2_SEARCH_NUM    8
#define K2_SEARCH_DEN    1

/* k1, k2 in Q8.8 (precomputed) */
#define K1_DEADZONE_Q88  640   /* 2.5 * 256 */
#define K2_SEARCH_Q88    2048  /* 8.0 * 256 */

/* === Hysteresis === */
#define HYSTERESIS_WINDOWS  5

/* === Adaptive baseline === */
/* Per-step decay when y_slow < baseline. Value 1 = smallest Q8.8 step. */
/* At 1ms sampling, 1 step/iteration gives ~30s to decay 1 internal unit (~1 dBm). */
#define BASELINE_DECAY_Q88  1

/* Cold-start warmup iterations before baseline is initialized */
#define COLD_START_WARMUP  200

/* === Boundary zone (actuator edge avoidance) === */
#define BOUNDARY_MARGIN_VOLT         5
#define BOUNDARY_MARGIN_Q88          1280   /* 5 * 256 */
#define BOUNDARY_FLOOR_WEIGHT        0.2f
#define BOUNDARY_FLOOR_WEIGHT_Q88    51     /* 0.2 * 256 = 51.2 -> 51 */
#define BOUNDARY_FORCE_INWARD_VOLT   2
#define BOUNDARY_FORCE_INWARD_Q88    512    /* 2 * 256 */

/* === Periodic probe (exploration ping in dead-zone) === */
/* Once every ~30 seconds at 1ms sampling */
#define PERIODIC_PROBE_INTERVAL  30000

/* === SPSA settle samples (time for y_slow EMA to reflect new voltage) === */
/* 3 * TAU_SLOW_MS = 225 samples for 95% settling */
#define SPSA_SETTLE_SAMPLES  (3 * TAU_SLOW_MS)

/* === Bandit === */
#define NUM_ARMS          4
#define NUM_CONTEXT_BUCKETS  6

/* Bandit update interval (SPSA iterations between bandit updates) */
#define BANDIT_WINDOW_ITERATIONS  50

/* UCB1 exploration constant in Q8.8 */
#define C_EXPLORE_Q88  512   /* 2.0 * 256 */

/* Reward shaping weights (in Q8.8) */
#define LAMBDA_BOUNDARY_Q88  256   /* 1.0 */
#define MU_MOVEMENT_Q88      256   /* 1.0 */

/* === Context discretization thresholds (Q8.8) === */
#define DRIFT_LOW_THRESH_Q88   5     /* ~0.02 */
#define DRIFT_HIGH_THRESH_Q88  50    /* ~0.195 */
#define NOISE_HIGH_THRESH_Q88  20    /* ~0.078 */

/* === Gain profiles (Q8.8) === */
/* ARM_PROFILES: (a_gain, c_gain) for each arm */
/* Arm 0: conservative (a=1V, c=1V) */
#define ARM0_A_Q88  256
#define ARM0_C_Q88  256
/* Arm 1: cautious explore (a=1V, c=4V) */
#define ARM1_A_Q88  256
#define ARM1_C_Q88  1024
/* Arm 2: aggressive exploit (a=4V, c=1V) */
#define ARM2_A_Q88  1024
#define ARM2_C_Q88  256
/* Arm 3: aggressive explore (a=4V, c=4V) */
#define ARM3_A_Q88  1024
#define ARM3_C_Q88  1024

/* SEARCH mode gain profile (overrides bandit) */
#define SEARCH_GAIN_A_Q88  2048   /* 8.0V */
#define SEARCH_GAIN_C_Q88  2048   /* 8.0V */

/* === Output struct === */
typedef struct {
    uint8_t actuate;                      /* 0/1: whether to set new voltages */
    fp_t voltages[NUM_SECTIONS];          /* Target voltages (Q8.8, volts) */
} PolCtrlOutput;

/* === Controller state (opaque to HAL, full def in polctrl_internal.h) === */
typedef struct PolCtrlState PolCtrlState;

/* === Public API === */

/* Initialize controller. Returns initial state. */
PolCtrlState polctrl_init(uint32_t rng_seed);

/*
 * Advance controller by one step (1 ms).
 *
 * Parameters:
 *   state    - current controller state
 *   beatnote_reading - Q8.8 value: (dBm + 65) * 256, range [0, 7680]
 *   out      - output: whether to actuate and what voltages to set
 *
 * Returns:
 *   Updated controller state.
 */
PolCtrlState polctrl_step(PolCtrlState state, fp_t beatnote_reading,
                          PolCtrlOutput *out);

#endif /* POLCTRL_H */
