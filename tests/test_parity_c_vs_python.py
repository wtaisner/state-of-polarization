"""
test_parity_c_vs_python.py — Parity tests between C library and Python reference.

This is the MOST CRITICAL test in the project. It verifies that the compiled C
library produces bit-identical results to the Python reference implementation.

Any discrepancy indicates a bug in either implementation.

Tests:
1. Struct sizeof parity
2. Constant parity (C #define vs Python constants)
3. RNG parity (xorshift32)
4. Fixed-point arithmetic parity
5. Full polctrl_step trajectory parity (10000 steps)
"""
import os
import sys
import re
import ctypes
import subprocess

import pytest

from bindings import (
    get_lib, BaselineState, FsmState, SpsaState, BanditState,
    PolCtrlOutput, PolCtrlState, SpsaGainProfile,
    STATE_TRACK, STATE_SEARCH, STATE_RECOVERY,
    SPSA_SUB_IDLE, SPSA_SUB_SET_PLUS, SPSA_SUB_MEASURE_PLUS,
    SPSA_SUB_SET_MINUS, SPSA_SUB_MEASURE_MINUS, SPSA_SUB_APPLY,
    polctrl_init, polctrl_step,
)
import fixedpoint as fp
from reference_impl import (
    rng_init as py_rng_init, rng_next as py_rng_next,
    baseline_init as py_baseline_init,
    baseline_update as py_baseline_update,
    baseline_zscore as py_baseline_zscore,
    fsm_init as py_fsm_init, fsm_update as py_fsm_update,
    fsm_should_actuate as py_fsm_should_actuate,
    boundary_weight as py_boundary_weight,
    forced_inward_sign as py_forced_inward_sign,
    snap_to_voltage_grid as py_snap_to_voltage_grid,
    spsa_compute_probe as py_spsa_compute_probe,
    spsa_apply_result as py_spsa_apply_result,
    bandit_init as py_bandit_init,
    discretize_context as py_discretize_context,
    bandit_select_arm as py_bandit_select_arm,
    bandit_update as py_bandit_update,
    polctrl_init as py_polctrl_init,
    polctrl_step as py_polctrl_step,
    SQRT_LN_LUT, _isqrt,
)
import constants as const

lib = get_lib()

C_DIR = os.path.join(os.path.dirname(__file__), '..', 'c')


# ===========================================================================
# 1. Struct sizeof parity
# ===========================================================================

class TestStructParity:
    """Verify ctypes struct sizes match C struct sizes."""

    @classmethod
    @pytest.fixture(scope='class')
    def c_sizes(cls):
        """Run size_check binary and parse output."""
        subprocess.run(['make', 'size_check'], cwd=C_DIR, check=True,
                       capture_output=True)
        result = subprocess.run(
            [os.path.join(C_DIR, 'size_check')],
            capture_output=True, text=True, check=True)
        sizes = {}
        for line in result.stdout.strip().split('\n'):
            parts = line.split('=')
            if len(parts) == 2:
                key = parts[0].strip()
                val = int(parts[1].strip())
                sizes[key] = val
        return sizes

    def test_fp_t_size(self, c_sizes):
        assert ctypes.sizeof(ctypes.c_int16) == c_sizes['sizeof(fp_t)']

    def test_polctrl_output_size(self, c_sizes):
        assert ctypes.sizeof(PolCtrlOutput) == c_sizes['sizeof(PolCtrlOutput)']

    def test_polctrl_state_size(self, c_sizes):
        assert ctypes.sizeof(PolCtrlState) == c_sizes['sizeof(PolCtrlState)']

    def test_baseline_state_size(self, c_sizes):
        assert ctypes.sizeof(BaselineState) == c_sizes['sizeof(BaselineState)']

    def test_fsm_state_size(self, c_sizes):
        assert ctypes.sizeof(FsmState) == c_sizes['sizeof(FsmState)']

    def test_spsa_state_size(self, c_sizes):
        assert ctypes.sizeof(SpsaState) == c_sizes['sizeof(SpsaState)']

    def test_bandit_state_size(self, c_sizes):
        assert ctypes.sizeof(BanditState) == c_sizes['sizeof(BanditState)']

    def test_spsa_gain_profile_size(self, c_sizes):
        assert ctypes.sizeof(SpsaGainProfile) == c_sizes['sizeof(SpsaGainProfile)']


# ===========================================================================
# 2. Constant parity (C #define vs Python constants)
# ===========================================================================

class TestConstantParity:
    """Verify C #define values match Python constants."""

    @classmethod
    @pytest.fixture(scope='class')
    def c_defines(cls):
        """Parse #define values from polctrl.h."""
        header_path = os.path.join(C_DIR, 'include', 'polctrl.h')
        with open(header_path) as f:
            content = f.read()
        defines = {}
        # Match: #define NAME value (with optional comment)
        pattern = r'#define\s+(\w+)\s+(\S+)'
        for match in re.finditer(pattern, content):
            name = match.group(1)
            val_str = match.group(2)
            # Skip non-numeric defines
            try:
                if val_str.startswith('0x') or val_str.startswith('0X'):
                    val = int(val_str, 16)
                elif val_str.startswith('-'):
                    val = int(val_str)
                else:
                    val = int(val_str)
                defines[name] = val
            except ValueError:
                pass  # Skip string/float defines
        return defines

    def test_num_sections(self, c_defines):
        assert const.NUM_SECTIONS == c_defines.get('NUM_SECTIONS')

    def test_voltage_constants(self, c_defines):
        assert const.V_MIN_Q88 == c_defines.get('V_MIN_Q88')
        assert const.V_MAX_Q88 == c_defines.get('V_MAX_Q88')
        assert const.V_STEP_Q88 == c_defines.get('V_STEP_Q88')

    def test_tau_constants(self, c_defines):
        assert const.TAU_FAST_MS == c_defines.get('TAU_FAST_MS')
        assert const.TAU_SLOW_MS == c_defines.get('TAU_SLOW_MS')
        assert const.ALPHA_FAST_Q88 == c_defines.get('ALPHA_FAST_Q88')
        assert const.ALPHA_SLOW_Q88 == c_defines.get('ALPHA_SLOW_Q88')

    def test_threshold_constants(self, c_defines):
        assert const.K1_DEADZONE_Q88 == c_defines.get('K1_DEADZONE_Q88')
        assert const.K2_SEARCH_Q88 == c_defines.get('K2_SEARCH_Q88')

    def test_hysteresis(self, c_defines):
        assert const.HYSTERESIS_WINDOWS == c_defines.get('HYSTERESIS_WINDOWS')

    def test_baseline_decay(self, c_defines):
        assert const.BASELINE_DECAY_Q88 == c_defines.get('BASELINE_DECAY_Q88')

    def test_cold_start_warmup(self, c_defines):
        assert const.COLD_START_WARMUP == c_defines.get('COLD_START_WARMUP')

    def test_boundary_constants(self, c_defines):
        assert const.BOUNDARY_MARGIN_Q88 == c_defines.get('BOUNDARY_MARGIN_Q88')
        assert const.BOUNDARY_FLOOR_WEIGHT_Q88 == c_defines.get('BOUNDARY_FLOOR_WEIGHT_Q88')
        assert const.BOUNDARY_FORCE_INWARD_Q88 == c_defines.get('BOUNDARY_FORCE_INWARD_Q88')

    def test_periodic_probe(self, c_defines):
        assert const.PERIODIC_PROBE_INTERVAL == c_defines.get('PERIODIC_PROBE_INTERVAL')

    def test_bandit_constants(self, c_defines):
        assert const.NUM_ARMS == c_defines.get('NUM_ARMS')
        assert const.NUM_CONTEXT_BUCKETS == c_defines.get('NUM_CONTEXT_BUCKETS')
        assert const.BANDIT_WINDOW_ITERATIONS == c_defines.get('BANDIT_WINDOW_ITERATIONS')
        assert const.C_EXPLORE_Q88 == c_defines.get('C_EXPLORE_Q88')

    def test_arm_profiles(self, c_defines):
        assert const.ARM_PROFILES[0][0] == c_defines.get('ARM0_A_Q88')
        assert const.ARM_PROFILES[0][1] == c_defines.get('ARM0_C_Q88')
        assert const.ARM_PROFILES[3][0] == c_defines.get('ARM3_A_Q88')
        assert const.ARM_PROFILES[3][1] == c_defines.get('ARM3_C_Q88')

    def test_search_gains(self, c_defines):
        assert const.SEARCH_GAIN_A_Q88 == c_defines.get('SEARCH_GAIN_A_Q88')
        assert const.SEARCH_GAIN_C_Q88 == c_defines.get('SEARCH_GAIN_C_Q88')

    def test_context_thresholds(self, c_defines):
        assert const.DRIFT_LOW_THRESH_Q88 == c_defines.get('DRIFT_LOW_THRESH_Q88')
        assert const.DRIFT_HIGH_THRESH_Q88 == c_defines.get('DRIFT_HIGH_THRESH_Q88')
        assert const.NOISE_HIGH_THRESH_Q88 == c_defines.get('NOISE_HIGH_THRESH_Q88')


# ===========================================================================
# 3. RNG parity
# ===========================================================================

class TestRngParity:

    def test_rng_sequence(self):
        """xorshift32 produces same sequence in C and Python."""
        seed = 42
        c_state = lib.rng_init(seed)
        py_state = py_rng_init(seed)

        for _ in range(1000):
            # C: rng_next takes pointer, modifies in place
            c_state_ptr = ctypes.c_uint32(c_state)
            c_val = lib.rng_next(ctypes.byref(c_state_ptr))
            c_state = c_state_ptr.value

            py_state = py_rng_next(py_state)
            assert c_val == py_state, \
                f"RNG mismatch: C={c_val}, Py={py_state}"
            assert c_state == py_state, \
                f"RNG state mismatch: C={c_state}, Py={py_state}"


# ===========================================================================
# 4. Full polctrl_step trajectory parity (THE CRITICAL TEST)
# ===========================================================================

class TestTrajectoryParity:
    """Verify C and Python produce identical controller trajectories."""

    def test_trajectory_10000_steps(self):
        """
        Run 10000 steps with identical inputs and compare theta trajectories.
        Any bit difference is a failure.
        """
        seed = 42
        c_state = polctrl_init(seed)
        py_state = py_polctrl_init(seed)

        # Verify initial state matches
        for i in range(const.NUM_SECTIONS):
            assert c_state.spsa.theta[i] == py_state.spsa.theta[i], \
                f"Initial theta[{i}] mismatch: C={c_state.spsa.theta[i]}, Py={py_state.spsa.theta[i]}"
        assert c_state.spsa.rng == py_state.spsa.rng, \
            f"Initial RNG mismatch: C={c_state.spsa.rng}, Py={py_state.spsa.rng}"

        # Generate deterministic input sequence
        import random
        rng = random.Random(123)
        readings = [rng.gauss(6912, 128) for _ in range(10000)]
        readings_q88 = [max(0, min(7680, int(r))) for r in readings]

        mismatches = 0
        first_mismatch = None

        for step, reading in enumerate(readings_q88):
            # C step
            c_out = PolCtrlOutput()
            c_state = lib.polctrl_step(c_state, ctypes.c_int16(reading),
                                        ctypes.byref(c_out))

            # Python step
            py_state, py_actuate, py_voltages = py_polctrl_step(py_state, reading)

            # Compare actuate flag
            if c_out.actuate != py_actuate:
                if first_mismatch is None:
                    first_mismatch = (
                        f"Step {step}: actuate mismatch: C={c_out.actuate}, Py={py_actuate}")
                mismatches += 1

            # Compare voltages
            for i in range(const.NUM_SECTIONS):
                if c_out.voltages[i] != py_voltages[i]:
                    if first_mismatch is None:
                        first_mismatch = (
                            f"Step {step}: voltage[{i}] mismatch: "
                            f"C={c_out.voltages[i]}, Py={py_voltages[i]}")
                    mismatches += 1

            # Compare theta (the most important invariant)
            for i in range(const.NUM_SECTIONS):
                if c_state.spsa.theta[i] != py_state.spsa.theta[i]:
                    if first_mismatch is None:
                        first_mismatch = (
                            f"Step {step}: theta[{i}] mismatch: "
                            f"C={c_state.spsa.theta[i]}, Py={py_state.spsa.theta[i]}")
                    mismatches += 1

            # Compare FSM mode
            if c_state.fsm.mode != py_state.fsm.mode:
                if first_mismatch is None:
                    first_mismatch = (
                        f"Step {step}: FSM mode mismatch: "
                        f"C={c_state.fsm.mode}, Py={py_state.fsm.mode}")
                mismatches += 1

            # Compare spsa_sub
            if c_state.spsa_sub != py_state.spsa_sub:
                if first_mismatch is None:
                    first_mismatch = (
                        f"Step {step}: spsa_sub mismatch: "
                        f"C={c_state.spsa_sub}, Py={py_state.spsa_sub}")
                mismatches += 1

            # Compare RNG state
            if c_state.spsa.rng != py_state.spsa.rng:
                if first_mismatch is None:
                    first_mismatch = (
                        f"Step {step}: RNG mismatch: "
                        f"C={c_state.spsa.rng}, Py={py_state.spsa.rng}")
                mismatches += 1

        assert mismatches == 0, \
            f"{mismatches} mismatches found. First: {first_mismatch}"

    def test_trajectory_with_simulator(self):
        """Run with actual simulator readings for 2000 steps."""
        from simulator import scenario_slow_drift

        sim_c = scenario_slow_drift(rng_seed=42)
        sim_py = scenario_slow_drift(rng_seed=42)

        seed = 99
        c_state = polctrl_init(seed)
        py_state = py_polctrl_init(seed)

        voltages = [30.0] * const.NUM_SECTIONS

        for step in range(2000):
            # Get readings from both simulators (should be identical with same seed)
            reading_c = sim_c.step(voltages)
            reading_py = sim_py.step(voltages)

            # Convert to Q8.8
            reading_q88 = max(0, min(7680, int((reading_c + 65) * 256)))

            # C step
            c_out = PolCtrlOutput()
            c_state = lib.polctrl_step(c_state, ctypes.c_int16(reading_q88),
                                        ctypes.byref(c_out))

            # Python step
            py_state, py_actuate, py_voltages = py_polctrl_step(py_state, reading_q88)

            # Compare theta
            for i in range(const.NUM_SECTIONS):
                assert c_state.spsa.theta[i] == py_state.spsa.theta[i], \
                    f"Step {step}: theta[{i}] mismatch: " \
                    f"C={c_state.spsa.theta[i]}, Py={py_state.spsa.theta[i]}"

            # Use C voltages for next simulator step
            if c_out.actuate:
                voltages = [fp.fp_to_float(v) for v in c_out.voltages]


# ===========================================================================
# 5. Module-level parity (sub-module functions)
# ===========================================================================

class TestModuleParity:
    """Verify individual module functions match between C and Python."""

    def test_baseline_update_parity(self):
        """baseline_update produces identical results."""
        import random
        rng = random.Random(42)
        c_bs = lib.baseline_init()
        py_bs = py_baseline_init()

        for _ in range(500):
            reading = rng.randint(0, 7680)
            in_dz = rng.randint(0, 1)

            c_bs = lib.baseline_update(c_bs, reading, in_dz)
            py_bs = py_baseline_update(py_bs, reading, in_dz)

            assert c_bs.y_fast == py_bs.y_fast, \
                f"y_fast mismatch: C={c_bs.y_fast}, Py={py_bs.y_fast}"
            assert c_bs.y_slow == py_bs.y_slow, \
                f"y_slow mismatch: C={c_bs.y_slow}, Py={py_bs.y_slow}"
            assert c_bs.baseline == py_bs.baseline, \
                f"baseline mismatch: C={c_bs.baseline}, Py={py_bs.baseline}"
            assert c_bs.noise_sigma == py_bs.noise_sigma, \
                f"sigma mismatch: C={c_bs.noise_sigma}, Py={py_bs.noise_sigma}"
            assert c_bs.initialized == py_bs.initialized

    def test_fsm_update_parity(self):
        """fsm_update produces identical results."""
        import random
        rng = random.Random(42)
        c_fsm = lib.fsm_init()
        py_fsm = py_fsm_init()

        for step in range(500):
            zscore = rng.randint(0, 5000)
            y_fast = rng.randint(0, 7680)
            y_slow = rng.randint(0, 7680)

            c_fsm = lib.fsm_update(c_fsm, zscore, y_fast, y_slow)
            py_fsm = py_fsm_update(py_fsm, zscore, y_fast, y_slow)

            assert c_fsm.mode == py_fsm.mode, \
                f"Step {step}: mode mismatch: C={c_fsm.mode}, Py={py_fsm.mode}"
            assert c_fsm.consecutive_good_windows == py_fsm.consecutive_good_windows

    def test_bandit_select_parity(self):
        """bandit_select_arm produces identical results."""
        c_bandit = lib.bandit_init()
        py_bandit = py_bandit_init()

        for trial in range(100):
            for bucket in range(const.NUM_CONTEXT_BUCKETS):
                c_arm = lib.bandit_select_arm(c_bandit, bucket)
                py_arm = py_bandit_select_arm(py_bandit, bucket)
                assert c_arm == py_arm, \
                    f"Trial {trial}, bucket {bucket}: arm mismatch: C={c_arm}, Py={py_arm}"

                # Update with same reward
                reward = 100 + trial
                c_bandit = lib.bandit_update(c_bandit, bucket, c_arm, reward)
                py_bandit = py_bandit_update(py_bandit, bucket, py_arm, reward)

                # Verify Q values match
                for a in range(const.NUM_ARMS):
                    assert c_bandit.q_value[bucket][a] == py_bandit.q_value[bucket][a], \
                        f"Q value mismatch: bucket={bucket}, arm={a}, " \
                        f"C={c_bandit.q_value[bucket][a]}, Py={py_bandit.q_value[bucket][a]}"

    def test_isqrt_parity(self):
        """Integer square root matches between C and Python."""
        # isqrt is internal to bandit.c, test via bandit_select_arm
        # which uses isqrt internally
        c_bandit = lib.bandit_init()
        py_bandit = py_bandit_init()

        # Force some counts to exercise isqrt
        for i in range(200):
            c_bandit = lib.bandit_update(c_bandit, 0, 0, 100)
            py_bandit = py_bandit_update(py_bandit, 0, 0, 100)

        # Both should select the same arm
        c_arm = lib.bandit_select_arm(c_bandit, 0)
        py_arm = py_bandit_select_arm(py_bandit, 0)
        assert c_arm == py_arm, f"Arm mismatch after 200 updates: C={c_arm}, Py={py_arm}"
