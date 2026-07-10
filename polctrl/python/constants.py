"""
constants.py — Single source of truth for all numeric constants.

These values MUST match the #define values in c/include/polctrl.h.
Tested by test_parity_c_vs_python.py.
"""

from fixedpoint import FP_SHIFT, FP_ONE, FP_MAX, FP_MIN

# === Fixed-point format (mirrors c/include/fixedpoint.h) ===
# Q8.8 on int16_t: range +/-127.996, resolution 1/256 ~= 0.0039

# === Physical ranges ===
NUM_SECTIONS = 4

V_MIN_VOLT = 0       # Minimum voltage [V]
V_MAX_VOLT = 60      # Maximum voltage [V]
V_STEP_VOLT = 0.1    # Voltage step resolution [V]

# === Beatnote measurement range ===
BEATNOTE_MIN_DBM = -65.0   # Physical detector range minimum [dBm]
BEATNOTE_MAX_DBM = -35.0   # Physical detector range maximum [dBm]

# === Simulator defaults ===
CHANNEL_CEILING_DBM = -38.0   # Default best achievable power [dBm]

# === Sampling ===
SAMPLING_PERIOD_MS = 1   # 1 ms per sample

# === EMA time constants (in samples, at 1ms sampling) ===
TAU_FAST_MS = 8    # Fast EMA: detects sudden drops
TAU_SLOW_MS = 75   # Slow EMA: measures y+/y- in SPSA, baseline

# EMA alpha = dt / tau = 1 / tau (in samples)
# In Q8.8: alpha_fast = 1/8 = 0.125 -> 32, alpha_slow = 1/75 ~= 0.0133 -> ~3
ALPHA_FAST_Q88 = round(FP_ONE / TAU_FAST_MS)   # 32
ALPHA_SLOW_Q88 = round(FP_ONE / TAU_SLOW_MS)   # 3

# === Dead-zone / SEARCH thresholds (as multiples of sigma) ===
K1_DEADZONE_NUM = 5    # k1 = 5/2 = 2.5
K1_DEADZONE_DEN = 2
K2_SEARCH_NUM = 8      # k2 = 8/1 = 8.0
K2_SEARCH_DEN = 1

# === Hysteresis ===
HYSTERESIS_WINDOWS = 5   # Consecutive good windows to exit SEARCH -> TRACK

# === Adaptive baseline ===
# Target decay time constant: ~60 seconds at 1ms sampling = 60000 samples
# Per-step decay: baseline -= BASELINE_DECAY_Q88 each step when y_slow < baseline
# We want a slow drift down. 60000 steps * decay ~= meaningful range.
# Internal normalized unit range is roughly [-65..-35] -> mapped to Q8.8.
# After mapping, range is ~30 units. Over 60000 steps, decay of 30/60000 ~= 0.0005
# In Q8.8: 0.0005 * 256 ~= 0.128 -> round to 1 (smallest nonzero Q8.8 step)
# This gives ~30000 steps to decay 1 full unit, which is ~30s for 1 dBm-equivalent.
# Documented as simplification: actual decay rate depends on scale mapping.
BASELINE_DECAY_Q88 = 1   # Smallest nonzero step down per iteration

# Cold-start warmup iterations
COLD_START_WARMUP = 200   # Iterations before baseline is initialized

# === Boundary zone (actuator edge avoidance) ===
BOUNDARY_MARGIN_VOLT = 5         # Below 5V from 0 or 60, damping starts
BOUNDARY_FLOOR_WEIGHT = 0.2      # Minimum weight at edges -> Q8.8: 51
BOUNDARY_FLOOR_WEIGHT_Q88 = round(BOUNDARY_FLOOR_WEIGHT * 256)  # 51
BOUNDARY_FORCE_INWARD_VOLT = 2   # Below this margin, force inward direction

# === Periodic probe (exploration ping in dead-zone) ===
# Once every ~30 seconds at 1ms sampling
PERIODIC_PROBE_INTERVAL = 30000   # samples

# === Bandit ===
NUM_ARMS = 4
NUM_CONTEXT_BUCKETS = 6

# Bandit update interval (iterations of SPSA between bandit updates)
BANDIT_WINDOW_ITERATIONS = 50

# UCB1 exploration constant
C_EXPLORE = 2   # In Q8.8: 2.0 * 256 = 512
C_EXPLORE_Q88 = 512

# Reward shaping weights
LAMBDA_BOUNDARY = 1   # Weight for boundary proximity penalty
MU_MOVEMENT = 1       # Weight for movement penalty

# === SPSA gain profiles (a_gain, c_gain) for each arm ===
# These are initial guesses; tuned empirically in Phase 8.
# Values in Q8.8.
# Format: (a_gain_q88, c_gain_q88)
ARM_PROFILES = [
    (1 * 256, 1 * 256),    # Arm 0: small a, small c (conservative)
    (1 * 256, 4 * 256),    # Arm 1: small a, large c (cautious explore)
    (4 * 256, 1 * 256),    # Arm 2: large a, small c (aggressive exploit)
    (4 * 256, 4 * 256),    # Arm 3: large a, large c (aggressive explore)
]

# === SEARCH mode gain profile (overrides bandit) ===
SEARCH_GAIN_A_Q88 = 8 * 256     # a_gain = 8.0
SEARCH_GAIN_C_Q88 = 8 * 256     # c_gain = 8.0

# === Voltage grid ===
# V_STEP_VOLT = 0.1V -> in Q8.8: 0.1 * 256 = 25.6 -> round to 26
V_STEP_Q88 = round(V_STEP_VOLT * 256)   # 26
V_MIN_Q88 = round(V_MIN_VOLT * 256)     # 0
V_MAX_Q88 = round(V_MAX_VOLT * 256)     # 15360

# === Context discretization ===
# Drift estimate: mean |grad| across 4 sections, EMA-smoothed
# Bins: 3 drift levels x 2 noise levels = 6 buckets
# drift_bucket = 0 (low), 1 (mid), 2 (high)
# noise_bucket = 0 (low), 1 (high)
# context = drift_bucket * 2 + noise_bucket
DRIFT_LOW_THRESH_Q88 = 5      # Below this = low drift (Q8.8, ~0.02)
DRIFT_HIGH_THRESH_Q88 = 50    # Above this = high drift (Q8.8, ~0.195)
NOISE_HIGH_THRESH_Q88 = 20    # Above this = high noise (Q8.8, ~0.078)
