"""
reference_impl.py — Pure Python reference implementation of the polarization controller.

This is a direct transliteration of the C code in c/src/ using fixedpoint.py
for all arithmetic. It produces bit-identical results to the compiled C library,
enabling parity testing (test_parity_c_vs_python.py).

No float/double used in any computation — all arithmetic is Q8.8 fixed-point,
exactly matching the C implementation.
"""

import fixedpoint as fp
from constants import *
from fixedpoint import FP_ONE, FP_MAX, FP_MIN, FP_SHIFT


# === RNG (xorshift32) — mirrors c/src/rng.c ===

def rng_init(seed):
    return 1 if seed == 0 else seed

def rng_next(state):
    state = state & 0xFFFFFFFF
    state ^= (state << 13) & 0xFFFFFFFF
    state ^= (state >> 17) & 0xFFFFFFFF
    state ^= (state << 5) & 0xFFFFFFFF
    return state

def rng_sign(state):
    r = rng_next(state[0])
    state[0] = r
    return 1 if (r & 1) else -1


# === Baseline — mirrors c/src/baseline.c ===

SIGMA_EPS_Q88 = 1

class BaselineState:
    def __init__(self):
        self.baseline = 0
        self.noise_sigma = SIGMA_EPS_Q88
        self.y_fast = 0
        self.y_slow = 0
        self.initialized = 0
        self.warmup_counter = 0

def baseline_init():
    return BaselineState()

def baseline_update(s, y_raw, currently_in_deadzone):
    # Update fast EMA
    diff = y_raw - s.y_fast
    s.y_fast = s.y_fast + fp.fp_mul(ALPHA_FAST_Q88, diff)

    # Update slow EMA
    diff = y_raw - s.y_slow
    s.y_slow = s.y_slow + fp.fp_mul(ALPHA_SLOW_Q88, diff)

    # Cold-start warmup
    if not s.initialized:
        s.warmup_counter += 1
        if s.warmup_counter >= COLD_START_WARMUP:
            s.initialized = 1
            s.baseline = s.y_slow

    # Baseline update
    if s.initialized:
        if s.y_slow > s.baseline:
            s.baseline = s.y_slow
        else:
            s.baseline = s.baseline - BASELINE_DECAY_Q88

    # Sigma update
    if currently_in_deadzone:
        resid = fp.fp_abs(y_raw - s.y_slow)
        s.noise_sigma = s.noise_sigma + fp.fp_mul(
            ALPHA_SLOW_Q88, resid - s.noise_sigma)
        if s.noise_sigma < SIGMA_EPS_Q88:
            s.noise_sigma = SIGMA_EPS_Q88

    return s

def baseline_zscore(s):
    if not s.initialized:
        return FP_MAX
    diff = s.baseline - s.y_slow
    sigma = s.noise_sigma
    if sigma < SIGMA_EPS_Q88:
        sigma = SIGMA_EPS_Q88
    return fp.fp_div(diff, sigma)


# === FSM — mirrors c/src/fsm.c ===

STATE_TRACK = 0
STATE_SEARCH = 1
STATE_RECOVERY = 2

SUDDEN_FADE_FRAC_Q88 = 64  # 0.25 * 256

class FsmState:
    def __init__(self):
        self.mode = STATE_TRACK
        self.consecutive_good_windows = 0
        self.periodic_probe_counter = 0

def fsm_init():
    return FsmState()

def fsm_update(s, zscore, y_fast, y_slow):
    # Sudden fade check
    if y_slow > 0:
        fade_threshold = fp.fp_mul(y_slow, SUDDEN_FADE_FRAC_Q88)
        drop = y_slow - y_fast
        if drop > fade_threshold:
            s.mode = STATE_SEARCH
            s.consecutive_good_windows = 0
            return s

    if s.mode == STATE_TRACK:
        if zscore > K2_SEARCH_Q88:
            s.mode = STATE_SEARCH
            s.consecutive_good_windows = 0
    elif s.mode == STATE_SEARCH:
        if zscore < K1_DEADZONE_Q88:
            s.mode = STATE_RECOVERY
            s.consecutive_good_windows = 1
    elif s.mode == STATE_RECOVERY:
        if zscore < K1_DEADZONE_Q88:
            s.consecutive_good_windows += 1
            if s.consecutive_good_windows >= HYSTERESIS_WINDOWS:
                s.mode = STATE_TRACK
                s.consecutive_good_windows = 0
        elif zscore > K2_SEARCH_Q88:
            s.mode = STATE_SEARCH
            s.consecutive_good_windows = 0
        else:
            s.consecutive_good_windows = 0

    return s

def fsm_should_actuate(s, zscore):
    if s.mode == STATE_SEARCH:
        return 1
    if zscore > K1_DEADZONE_Q88:
        return 1
    return 0

def fsm_check_periodic_probe(s):
    if s.mode == STATE_SEARCH:
        s.periodic_probe_counter = 0
        return 0
    s.periodic_probe_counter += 1
    if s.periodic_probe_counter >= PERIODIC_PROBE_INTERVAL:
        s.periodic_probe_counter = 0
        return 1
    return 0


# === SPSA — mirrors c/src/spsa.c ===

class SpsaState:
    def __init__(self):
        self.theta = [0] * NUM_SECTIONS
        self.last_grad_estimate = [0] * NUM_SECTIONS
        self.rng = 0
        self.delta = [0] * NUM_SECTIONS
        self.c_k = [0] * NUM_SECTIONS

def boundary_weight(theta_i):
    lo_margin = BOUNDARY_MARGIN_Q88
    hi_margin = V_MAX_Q88 - BOUNDARY_MARGIN_Q88

    if lo_margin <= theta_i <= hi_margin:
        return FP_ONE

    if theta_i < lo_margin:
        dist_from_edge = theta_i
        range_val = lo_margin
    else:
        dist_from_edge = V_MAX_Q88 - theta_i
        range_val = lo_margin

    if range_val <= 0:
        return BOUNDARY_FLOOR_WEIGHT_Q88

    scale = fp.fp_div(dist_from_edge, range_val)
    ramp = fp.fp_mul(FP_ONE - BOUNDARY_FLOOR_WEIGHT_Q88, scale)
    weight = BOUNDARY_FLOOR_WEIGHT_Q88 + ramp

    if weight < BOUNDARY_FLOOR_WEIGHT_Q88:
        weight = BOUNDARY_FLOOR_WEIGHT_Q88
    if weight > FP_ONE:
        weight = FP_ONE

    return weight

def forced_inward_sign(theta_i):
    if theta_i < BOUNDARY_FORCE_INWARD_Q88:
        return 1
    if theta_i > V_MAX_Q88 - BOUNDARY_FORCE_INWARD_Q88:
        return -1
    return 0

def snap_to_voltage_grid(v):
    half_step = V_STEP_Q88 // 2
    if v >= 0:
        grid_idx = (v + half_step) // V_STEP_Q88
    else:
        grid_idx = (v - half_step) // V_STEP_Q88
    result = grid_idx * V_STEP_Q88
    if result < V_MIN_Q88:
        result = V_MIN_Q88
    if result > V_MAX_Q88:
        result = V_MAX_Q88
    return result

def spsa_compute_probe(s, a_gain, c_gain, disable_boundary_weight):
    for i in range(NUM_SECTIONS):
        if disable_boundary_weight:
            weight = FP_ONE
        else:
            weight = boundary_weight(s.theta[i])

        s.c_k[i] = fp.fp_mul(c_gain, weight)

        force_sign = forced_inward_sign(s.theta[i])
        if force_sign != 0:
            sign = force_sign
        else:
            sign = rng_sign([s.rng])  # Note: need mutable state
            # Actually, we need to update s.rng properly
            # Let me handle this differently

    return s

# Actually, let me redo this properly. The issue is that rng_sign needs to
# update the state in place. In Python, integers are immutable, so I need
# to return the new state.

def spsa_compute_probe(s, out_plus, out_minus, a_gain, c_gain, disable_boundary_weight):
    for i in range(NUM_SECTIONS):
        if disable_boundary_weight:
            weight = FP_ONE
        else:
            weight = boundary_weight(s.theta[i])

        s.c_k[i] = fp.fp_mul(c_gain, weight)

        force_sign = forced_inward_sign(s.theta[i])
        if force_sign != 0:
            sign = force_sign
        else:
            r = rng_next(s.rng)
            s.rng = r
            sign = 1 if (r & 1) else -1

        s.delta[i] = FP_ONE if sign > 0 else -FP_ONE

        perturbation = fp.fp_mul(s.c_k[i], s.delta[i])
        out_plus[i] = fp.fp_clamp(s.theta[i] + perturbation, V_MIN_Q88, V_MAX_Q88)
        out_minus[i] = fp.fp_clamp(s.theta[i] - perturbation, V_MIN_Q88, V_MAX_Q88)

    return s

def spsa_apply_result(s, y_plus, y_minus, a_gain):
    y_diff = y_plus - y_minus

    for i in range(NUM_SECTIONS):
        denom = fp.fp_mul(s.c_k[i], s.delta[i])
        denom2 = denom + denom

        grad = fp.fp_div(y_diff, denom2)
        grad = fp.fp_clamp(grad, -FP_ONE * 4, FP_ONE * 4)

        s.last_grad_estimate[i] = grad

        step = fp.fp_mul(a_gain, grad)
        new_theta = s.theta[i] + step
        new_theta = fp.fp_clamp(new_theta, V_MIN_Q88, V_MAX_Q88)
        new_theta = snap_to_voltage_grid(new_theta)
        new_theta = fp.fp_clamp(new_theta, V_MIN_Q88, V_MAX_Q88)

        s.theta[i] = new_theta

    return s


# === Bandit — mirrors c/src/bandit.c ===

import math

# Generate LUT (same as C)
BANDIT_LUT_SIZE = 128

def _generate_sqrt_ln_lut():
    lut = []
    for i in range(BANDIT_LUT_SIZE):
        if i == 0:
            val = 0.0
        else:
            val = math.sqrt(math.log(i + 1))
        q88 = round(val * 256)
        if q88 > 32767:
            q88 = 32767
        lut.append(q88)
    return lut

SQRT_LN_LUT = _generate_sqrt_ln_lut()

def _isqrt(n):
    """Integer square root (matches C implementation)."""
    result = 0
    bit = 1 << 30
    while bit > n:
        bit >>= 2
    while bit != 0:
        if n >= result + bit:
            n -= result + bit
            result = (result >> 1) + bit
        else:
            result >>= 1
        bit >>= 2
    return result

class BanditState:
    def __init__(self):
        self.q_value = [[0]*NUM_ARMS for _ in range(NUM_CONTEXT_BUCKETS)]
        self.count = [[0]*NUM_ARMS for _ in range(NUM_CONTEXT_BUCKETS)]
        self.total_count = 0

def bandit_init():
    return BanditState()

def discretize_context(drift_estimate, noise_sigma_estimate):
    if drift_estimate < DRIFT_LOW_THRESH_Q88:
        drift_level = 0
    elif drift_estimate > DRIFT_HIGH_THRESH_Q88:
        drift_level = 2
    else:
        drift_level = 1

    if noise_sigma_estimate > NOISE_HIGH_THRESH_Q88:
        noise_level = 1
    else:
        noise_level = 0

    return drift_level * 2 + noise_level

def bandit_select_arm(s, context_bucket):
    best_arm = 0
    best_score = FP_MIN

    lut_idx = s.total_count
    if lut_idx >= BANDIT_LUT_SIZE:
        lut_idx = BANDIT_LUT_SIZE - 1
    sqrt_ln_N = SQRT_LN_LUT[lut_idx]

    for a in range(NUM_ARMS):
        if s.count[context_bucket][a] == 0:
            return a

        temp = fp.fp_mul(C_EXPLORE_Q88, sqrt_ln_N)
        sqrt_n = _isqrt(s.count[context_bucket][a] + 1)
        sqrt_n_q88 = sqrt_n << FP_SHIFT
        bonus = fp.fp_div(temp, sqrt_n_q88)

        score = s.q_value[context_bucket][a] + bonus

        if score > best_score:
            best_score = score
            best_arm = a

    return best_arm

def bandit_update(s, context_bucket, arm, reward):
    s.count[context_bucket][arm] += 1
    s.total_count += 1

    effective_count = s.count[context_bucket][arm]
    if effective_count > 255:
        effective_count = 255

    alpha = FP_ONE // effective_count
    if alpha == 0:
        alpha = 1

    diff = reward - s.q_value[context_bucket][arm]
    s.q_value[context_bucket][arm] += fp.fp_mul(alpha, diff)

    return s


# === ARM_PROFILES ===

ARM_PROFILES_PY = [
    (ARM0_A_Q88, ARM0_C_Q88),
    (ARM1_A_Q88, ARM1_C_Q88),
    (ARM2_A_Q88, ARM2_C_Q88),
    (ARM3_A_Q88, ARM3_C_Q88),
]


# === SPSA sub-states ===

SPSA_SUB_IDLE = 0
SPSA_SUB_SET_PLUS = 1
SPSA_SUB_MEASURE_PLUS = 2
SPSA_SUB_SET_MINUS = 3
SPSA_SUB_MEASURE_MINUS = 4
SPSA_SUB_APPLY = 5


# === PolCtrl — mirrors c/src/polctrl.c ===

INITIAL_VOLTAGE_Q88 = 7680  # 30 * 256
ALPHA_DRIFT_Q88 = ALPHA_SLOW_Q88

class PolCtrlState:
    def __init__(self):
        self.baseline = baseline_init()
        self.fsm = fsm_init()
        self.spsa = SpsaState()
        self.bandit = bandit_init()

        self.spsa_sub = SPSA_SUB_IDLE
        self.spsa_settle_counter = 0
        self.y_plus = 0
        self.y_minus = 0

        self.bandit_iter_counter = 0
        self.current_arm = 0
        self.current_context = 0

        self.reward_power_sum = 0
        self.reward_power_count = 0
        self.reward_boundary_sum = 0
        self.reward_movement_sum = 0

        self.drift_estimate = 0
        self.periodic_probe_counter = 0
        self.step_count = 0

        self.current_gain_a = ARM_PROFILES_PY[0][0]
        self.current_gain_c = ARM_PROFILES_PY[0][1]

def polctrl_init(rng_seed):
    state = PolCtrlState()
    for i in range(NUM_SECTIONS):
        state.spsa.theta[i] = INITIAL_VOLTAGE_Q88
    state.spsa.rng = rng_init(rng_seed)
    return state

def _compute_boundary_fraction(spsa):
    count = 0
    for i in range(NUM_SECTIONS):
        w = boundary_weight(spsa.theta[i])
        if w < FP_ONE:
            count += 1
    return (count * FP_ONE) // NUM_SECTIONS

def _compute_movement(old_theta, new_spsa):
    total = 0
    for i in range(NUM_SECTIONS):
        diff = new_spsa.theta[i] - old_theta[i]
        total += fp.fp_abs(diff)
    return total

def _update_drift(current_drift, spsa):
    sum_val = 0
    for i in range(NUM_SECTIONS):
        sum_val += fp.fp_abs(spsa.last_grad_estimate[i])
    mean_grad = sum_val // NUM_SECTIONS
    diff = mean_grad - current_drift
    return current_drift + fp.fp_mul(ALPHA_DRIFT_Q88, diff)

def polctrl_step(state, beatnote_reading):
    """Returns (state, actuate, voltages)."""
    state.step_count += 1

    actuate = 0
    voltages = list(state.spsa.theta)

    # Dead-zone status
    currently_in_deadzone = (state.spsa_sub == SPSA_SUB_IDLE and
                             state.fsm.mode != STATE_SEARCH)

    # Update baseline
    state.baseline = baseline_update(state.baseline, beatnote_reading,
                                      currently_in_deadzone)

    # Z-score
    zscore = baseline_zscore(state.baseline)

    # Update FSM
    state.fsm = fsm_update(state.fsm, zscore,
                           state.baseline.y_fast,
                           state.baseline.y_slow)

    # SPSA sub-state machine
    if state.spsa_sub == SPSA_SUB_IDLE:
        should_actuate = fsm_should_actuate(state.fsm, zscore)
        periodic_probe = fsm_check_periodic_probe(state.fsm)

        if should_actuate or periodic_probe:
            if state.fsm.mode == STATE_SEARCH:
                state.current_gain_a = SEARCH_GAIN_A_Q88
                state.current_gain_c = SEARCH_GAIN_C_Q88
            else:
                state.current_context = discretize_context(
                    state.drift_estimate, state.baseline.noise_sigma)
                state.current_arm = bandit_select_arm(state.bandit,
                                                       state.current_context)
                state.current_gain_a = ARM_PROFILES_PY[state.current_arm][0]
                state.current_gain_c = ARM_PROFILES_PY[state.current_arm][1]

            state.spsa_sub = SPSA_SUB_SET_PLUS

    elif state.spsa_sub == SPSA_SUB_SET_PLUS:
        out_plus = [0] * NUM_SECTIONS
        out_minus = [0] * NUM_SECTIONS
        disable_bw = 1 if state.fsm.mode == STATE_SEARCH else 0

        state.spsa = spsa_compute_probe(state.spsa, out_plus, out_minus,
                                         state.current_gain_a,
                                         state.current_gain_c,
                                         disable_bw)
        actuate = 1
        voltages = list(out_plus)
        state.spsa_settle_counter = SPSA_SETTLE_SAMPLES
        state.spsa_sub = SPSA_SUB_MEASURE_PLUS

    elif state.spsa_sub == SPSA_SUB_MEASURE_PLUS:
        if state.spsa_settle_counter > 0:
            state.spsa_settle_counter -= 1
        else:
            state.y_plus = state.baseline.y_slow
            state.spsa_sub = SPSA_SUB_SET_MINUS

    elif state.spsa_sub == SPSA_SUB_SET_MINUS:
        theta_minus = [0] * NUM_SECTIONS
        for i in range(NUM_SECTIONS):
            perturbation = fp.fp_mul(state.spsa.c_k[i], state.spsa.delta[i])
            theta_minus[i] = fp.fp_clamp(
                state.spsa.theta[i] - perturbation,
                V_MIN_Q88, V_MAX_Q88)

        actuate = 1
        voltages = theta_minus
        state.spsa_settle_counter = SPSA_SETTLE_SAMPLES
        state.spsa_sub = SPSA_SUB_MEASURE_MINUS

    elif state.spsa_sub == SPSA_SUB_MEASURE_MINUS:
        if state.spsa_settle_counter > 0:
            state.spsa_settle_counter -= 1
        else:
            state.y_minus = state.baseline.y_slow
            state.spsa_sub = SPSA_SUB_APPLY

    elif state.spsa_sub == SPSA_SUB_APPLY:
        # Save old theta for movement computation
        old_theta = list(state.spsa.theta)

        state.spsa = spsa_apply_result(state.spsa, state.y_plus,
                                        state.y_minus,
                                        state.current_gain_a)

        actuate = 1
        voltages = list(state.spsa.theta)

        # Update drift estimate
        state.drift_estimate = _update_drift(state.drift_estimate, state.spsa)

        # Reward accumulator
        state.reward_power_sum += state.baseline.y_slow
        state.reward_power_count += 1
        state.reward_boundary_sum += _compute_boundary_fraction(state.spsa)
        state.reward_movement_sum += _compute_movement(
            old_theta, state.spsa)

        # Bandit window check
        state.bandit_iter_counter += 1
        if (state.bandit_iter_counter >= BANDIT_WINDOW_ITERATIONS and
            state.fsm.mode != STATE_SEARCH):
            mean_power = fp.fp_div(state.reward_power_sum,
                                   state.reward_power_count << FP_SHIFT)
            mean_boundary = fp.fp_div(state.reward_boundary_sum,
                                      state.reward_power_count << FP_SHIFT)
            mean_movement = fp.fp_div(state.reward_movement_sum,
                                      state.reward_power_count << FP_SHIFT)
            reward = (mean_power
                      - fp.fp_mul(LAMBDA_BOUNDARY_Q88, mean_boundary)
                      - fp.fp_mul(MU_MOVEMENT_Q88, mean_movement))

            state.bandit = bandit_update(state.bandit,
                                         state.current_context,
                                         state.current_arm,
                                         reward)

            state.reward_power_sum = 0
            state.reward_power_count = 0
            state.reward_boundary_sum = 0
            state.reward_movement_sum = 0
            state.bandit_iter_counter = 0

        state.spsa_sub = SPSA_SUB_IDLE

    return state, actuate, voltages
