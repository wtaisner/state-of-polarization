"""
Tests for the SPSA optimizer with per-coordinate boundary weighting.
"""
import os
import sys
import ctypes

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from bindings import get_lib, SpsaState
import fixedpoint as fp
from constants import (
    NUM_SECTIONS, V_MIN_Q88, V_MAX_Q88, V_STEP_Q88,
    BOUNDARY_MARGIN_Q88, BOUNDARY_FLOOR_WEIGHT_Q88,
    BOUNDARY_FORCE_INWARD_Q88,
)

lib = get_lib()


def make_spsa(theta_volts=None, rng_seed=42):
    """Create an SpsaState with given initial voltages."""
    s = SpsaState()
    if theta_volts is None:
        theta_volts = [30.0] * NUM_SECTIONS
    for i in range(NUM_SECTIONS):
        s.theta[i] = fp.fp_from_float(theta_volts[i])
        s.last_grad_estimate[i] = 0
        s.delta[i] = 0
        s.c_k[i] = 0
    s.rng = lib.rng_init(rng_seed)
    return s


class TestBoundaryWeight:
    """Test boundary_weight function."""

    def test_center_is_one(self):
        """In center range, weight = 1.0."""
        for v in [10.0, 20.0, 30.0, 40.0, 50.0]:
            theta = fp.fp_from_float(v)
            w = lib.boundary_weight(theta)
            assert w == fp.FP_ONE, f"weight at {v}V should be 1.0, got {fp.fp_to_float(w)}"

    def test_edge_decreases(self):
        """At edges, weight decreases toward BOUNDARY_FLOOR_WEIGHT."""
        w_at_1v = lib.boundary_weight(fp.fp_from_float(1.0))
        w_at_59v = lib.boundary_weight(fp.fp_from_float(59.0))
        assert w_at_1v < fp.FP_ONE
        assert w_at_59v < fp.FP_ONE
        assert w_at_1v >= BOUNDARY_FLOOR_WEIGHT_Q88
        assert w_at_59v >= BOUNDARY_FLOOR_WEIGHT_Q88

    def test_symmetry(self):
        """Weight is symmetric: w(v) == w(60-v)."""
        for v in [0.5, 1.0, 2.0, 3.0, 4.0, 4.5]:
            w1 = lib.boundary_weight(fp.fp_from_float(v))
            w2 = lib.boundary_weight(fp.fp_from_float(60.0 - v))
            assert abs(w1 - w2) <= 1, f"Asymmetric: w({v})={w1}, w({60-v})={w2}"

    def test_at_boundary(self):
        """At 0V and 60V, weight = BOUNDARY_FLOOR_WEIGHT."""
        w0 = lib.boundary_weight(fp.fp_from_float(0.0))
        w60 = lib.boundary_weight(fp.fp_from_float(60.0))
        assert abs(w0 - BOUNDARY_FLOOR_WEIGHT_Q88) <= 2
        assert abs(w60 - BOUNDARY_FLOOR_WEIGHT_Q88) <= 2


class TestForcedInward:
    """Test forced_inward_sign function."""

    def test_near_zero_forces_positive(self):
        """Below BOUNDARY_FORCE_INWARD_VOLT, sign is +1."""
        for v in [0.0, 0.5, 1.0, 1.5, 1.99]:
            sign = lib.forced_inward_sign(fp.fp_from_float(v))
            assert sign == 1, f"Expected +1 at {v}V, got {sign}"

    def test_near_max_forces_negative(self):
        """Above 60 - BOUNDARY_FORCE_INWARD_VOLT, sign is -1."""
        for v in [58.01, 59.0, 59.5, 60.0]:
            sign = lib.forced_inward_sign(fp.fp_from_float(v))
            assert sign == -1, f"Expected -1 at {v}V, got {sign}"

    def test_center_allows_random(self):
        """In center, sign is 0 (random allowed)."""
        for v in [5.0, 10.0, 30.0, 50.0, 55.0]:
            sign = lib.forced_inward_sign(fp.fp_from_float(v))
            assert sign == 0, f"Expected 0 at {v}V, got {sign}"


class TestSnapToGrid:
    """Test snap_to_voltage_grid function."""

    def test_already_on_grid(self):
        """Values already on grid stay unchanged."""
        # Grid points are multiples of V_STEP_Q88 (26)
        for n in [0, 100, 300, 590]:
            v = n * V_STEP_Q88
            result = lib.snap_to_voltage_grid(v)
            assert result == v, f"Grid point {v} (n={n}) snapped to {result}"

    def test_snaps_to_nearest(self):
        """Off-grid values snap to nearest 0.1V."""
        # 30.05V should snap to 30.1V (26*300=7800) or 30.0V (26*299=7774)
        # 30.0 in Q8.8 = 7680, 30.1 = 7706
        v = fp.fp_from_float(30.05)
        result = lib.snap_to_voltage_grid(v)
        # Should be close to either 30.0 or 30.1
        assert abs(result - 7680) <= V_STEP_Q88 or abs(result - 7706) <= V_STEP_Q88

    def test_clamped_to_range(self):
        """Snapping doesn't go outside [0, 60]."""
        result = lib.snap_to_voltage_grid(fp.fp_from_float(-5.0))
        assert result == V_MIN_Q88
        result = lib.snap_to_voltage_grid(fp.fp_from_float(65.0))
        assert result == V_MAX_Q88

class TestSpsaComputeProbe:
    """Test spsa_compute_probe function."""

    def test_probe_voltages_in_range(self):
        """Probe voltages are always in [0, 60]."""
        for seed in range(10):
            s = make_spsa(theta_volts=[30.0]*4, rng_seed=seed)
            plus = (ctypes.c_int16 * 4)()
            minus = (ctypes.c_int16 * 4)()
            s = lib.spsa_compute_probe(s, plus, minus,
                                        fp.fp_from_float(1.0), fp.fp_from_float(1.0), 0)
            for i in range(4):
                assert V_MIN_Q88 <= plus[i] <= V_MAX_Q88
                assert V_MIN_Q88 <= minus[i] <= V_MAX_Q88

    def test_boundary_reduces_perturbation(self):
        """Section near boundary has smaller perturbation than center."""
        # Section 0 at 58V (near boundary), section 1 at 30V (center)
        s = make_spsa(theta_volts=[58.0, 30.0, 30.0, 30.0], rng_seed=42)
        plus = (ctypes.c_int16 * 4)()
        minus = (ctypes.c_int16 * 4)()
        c_gain = fp.fp_from_float(4.0)
        s = lib.spsa_compute_probe(s, plus, minus, c_gain, c_gain, 0)

        # Perturbation size = |plus - minus| / 2
        perturb_0 = abs(plus[0] - minus[0])
        perturb_1 = abs(plus[1] - minus[1])
        assert perturb_0 < perturb_1, \
            f"Boundary perturbation ({perturb_0}) should be < center ({perturb_1})"

    def test_forced_inward_near_zero(self):
        """Section < 2V always perturbs inward (positive)."""
        s = make_spsa(theta_volts=[1.0, 30.0, 30.0, 30.0], rng_seed=42)
        for trial in range(20):
            plus = (ctypes.c_int16 * 4)()
            minus = (ctypes.c_int16 * 4)()
            s = lib.spsa_compute_probe(s, plus, minus,
                                        fp.fp_from_float(1.0), fp.fp_from_float(1.0), 0)
            # delta should be +1 (inward), so theta_plus > theta > theta_minus
            # Actually, plus = theta + c*delta, minus = theta - c*delta
            # If delta = +1: plus > theta > minus -> plus > minus
            assert plus[0] > minus[0], \
                f"Trial {trial}: expected inward perturbation (plus > minus)"

    def test_disable_boundary_weight(self):
        """When disable_boundary_weight=1, perturbation magnitude is same
        at boundary and center (for values where clamping doesn't interfere)."""
        # Use 55V (not 58V) to avoid clamping at V_MAX
        s = make_spsa(theta_volts=[55.0, 30.0, 30.0, 30.0], rng_seed=42)
        plus = (ctypes.c_int16 * 4)()
        minus = (ctypes.c_int16 * 4)()
        c_gain = fp.fp_from_float(4.0)
        s = lib.spsa_compute_probe(s, plus, minus, c_gain, c_gain, 1)

        perturb_0 = abs(plus[0] - minus[0])
        perturb_1 = abs(plus[1] - minus[1])
        # With boundary weight disabled, c_k is same for both.
        # Perturbation = 2 * c_k * |delta|. delta is ±1 for both.
        # But delta may differ (random sign), so magnitude is same.
        assert abs(perturb_0 - perturb_1) <= 1, \
            f"Perturbation should be equal: boundary={perturb_0}, center={perturb_1}"


class TestSpsaApplyResult:
    """Test spsa_apply_result function."""

    def test_convergence_on_quadratic(self):
        """SPSA converges toward optimum on a synthetic quadratic function."""
        import numpy as np

        # Quadratic objective: f(theta) = -sum((theta - theta_opt)^2)
        theta_opt = np.array([20.0, 40.0, 15.0, 35.0])

        def objective(theta_volts):
            diff = np.array(theta_volts) - theta_opt
            return -np.sum(diff**2)

        def objective_q88(theta_q88):
            theta_volts = [fp.fp_to_float(t) for t in theta_q88]
            val = objective(theta_volts)
            # Scale to internal units: val is in [-something, 0]
            # Map to [0, ~30] range
            normalized = val + 4500  # shift up
            normalized = max(normalized, 0)
            return fp.fp_from_float(normalized / 150)  # scale down

        s = make_spsa(theta_volts=[30.0]*4, rng_seed=42)
        a_gain = fp.fp_from_float(2.0)
        c_gain = fp.fp_from_float(2.0)

        initial_dist = sum(abs(fp.fp_to_float(s.theta[i]) - theta_opt[i])
                          for i in range(4))

        for _ in range(200):
            plus = (ctypes.c_int16 * 4)()
            minus = (ctypes.c_int16 * 4)()
            s = lib.spsa_compute_probe(s, plus, minus, a_gain, c_gain, 0)

            y_plus = objective_q88(list(plus))
            y_minus = objective_q88(list(minus))

            s = lib.spsa_apply_result(s, y_plus, y_minus, a_gain)

        final_dist = sum(abs(fp.fp_to_float(s.theta[i]) - theta_opt[i])
                        for i in range(4))

        assert final_dist < initial_dist, \
            f"SPSA should converge: initial={initial_dist}, final={final_dist}"

    def test_theta_always_in_range(self):
        """Fuzz test: theta always in [0, 60] after apply_result."""
        import random
        rng = random.Random(42)

        s = make_spsa(theta_volts=[30.0]*4, rng_seed=42)
        a_gain = fp.fp_from_float(4.0)
        c_gain = fp.fp_from_float(4.0)

        for _ in range(1000):
            plus = (ctypes.c_int16 * 4)()
            minus = (ctypes.c_int16 * 4)()
            s = lib.spsa_compute_probe(s, plus, minus, a_gain, c_gain, 0)

            # Random y values
            y_plus = rng.randint(0, 7680)
            y_minus = rng.randint(0, 7680)
            s = lib.spsa_apply_result(s, y_plus, y_minus, a_gain)

            for i in range(4):
                assert V_MIN_Q88 <= s.theta[i] <= V_MAX_Q88, \
                    f"theta[{i}]={s.theta[i]} out of range"

    def test_theta_on_grid(self):
        """Theta is always on 0.1V grid (or at V_MIN/V_MAX boundary) after apply_result."""
        import random
        rng = random.Random(42)

        s = make_spsa(theta_volts=[30.0]*4, rng_seed=42)
        a_gain = fp.fp_from_float(4.0)
        c_gain = fp.fp_from_float(4.0)

        for _ in range(100):
            plus = (ctypes.c_int16 * 4)()
            minus = (ctypes.c_int16 * 4)()
            s = lib.spsa_compute_probe(s, plus, minus, a_gain, c_gain, 0)

            y_plus = rng.randint(0, 7680)
            y_minus = rng.randint(0, 7680)
            s = lib.spsa_apply_result(s, y_plus, y_minus, a_gain)

            for i in range(4):
                theta = s.theta[i]
                # Either on grid, or at V_MIN/V_MAX (which may not be exact grid points)
                on_grid = (theta % V_STEP_Q88 == 0)
                at_boundary = (theta == V_MIN_Q88 or theta == V_MAX_Q88)
                assert on_grid or at_boundary, \
                    f"theta[{i}]={theta} not on grid (mod {V_STEP_Q88} = {theta % V_STEP_Q88}) " \
                    f"and not at boundary"

    def test_degenerate_c_gain_no_crash(self):
        """c_gain = 0 doesn't crash or produce NaN."""
        s = make_spsa(theta_volts=[30.0]*4, rng_seed=42)
        plus = (ctypes.c_int16 * 4)()
        minus = (ctypes.c_int16 * 4)()
        # c_gain = 0 -> c_k = 0 -> division by zero in gradient
        s = lib.spsa_compute_probe(s, plus, minus,
                                    fp.fp_from_float(1.0), 0, 0)
        # plus == minus == theta (no perturbation)
        for i in range(4):
            assert plus[i] == s.theta[i]
            assert minus[i] == s.theta[i]

        # apply_result should not crash
        s = lib.spsa_apply_result(s, 1000, 500, fp.fp_from_float(1.0))
        # theta should still be in range
        for i in range(4):
            assert V_MIN_Q88 <= s.theta[i] <= V_MAX_Q88
