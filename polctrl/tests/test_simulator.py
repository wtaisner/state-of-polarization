"""
Tests for the physical simulator.
Sanity checks: power is maximal when SOP matches, monotonicity near optimum,
dBm range is reasonable.
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from simulator import (
    PolarizationSimulator,
    normalize,
    rotation_matrix,
    angle_between,
    scenario_stable,
    scenario_slow_drift,
    scenario_fast_drift,
    scenario_regime_switch,
    scenario_cold_start,
    scenario_sudden_fade,
    scenario_channel_degradation,
)
from constants import (
    NUM_SECTIONS, V_MIN_VOLT, V_MAX_VOLT,
    BEATNOTE_MIN_DBM, BEATNOTE_MAX_DBM, CHANNEL_CEILING_DBM,
)


class TestSimulatorBasics:
    """Basic sanity checks for the simulator."""

    def test_power_maximal_when_sop_matches(self):
        """Power should be at ceiling when SOP_out == SOP_reference."""
        sim = PolarizationSimulator(
            initial_sop_input=[1.0, 0.0, 0.0],
            sop_reference=[1.0, 0.0, 0.0],
            noise_sigma_dbm=0.0,
            drift_amplitude=0.0,
        )
        # With zero voltages, SOP_out == SOP_input == SOP_reference
        power = sim.step([0.0] * NUM_SECTIONS)
        assert abs(power - CHANNEL_CEILING_DBM) < 0.1, \
            f"Expected ~{CHANNEL_CEILING_DBM} dBm, got {power}"

    def test_power_low_when_sop_orthogonal(self):
        """Power should be low when SOP is orthogonal to reference."""
        sim = PolarizationSimulator(
            initial_sop_input=[0.0, 1.0, 0.0],
            sop_reference=[1.0, 0.0, 0.0],
            noise_sigma_dbm=0.0,
            drift_amplitude=0.0,
        )
        power = sim.step([0.0] * NUM_SECTIONS)
        # cos²(90°/2) = cos²(45°) = 0.5, so ~3 dB below ceiling
        assert power < CHANNEL_CEILING_DBM - 2, \
            f"Expected power below ceiling - 2 dB, got {power}"

    def test_dBm_range(self):
        """All power readings should be within physical detector range."""
        sim = PolarizationSimulator(noise_sigma_dbm=2.0, drift_amplitude=0.5)
        rng = np.random.RandomState(123)
        for _ in range(1000):
            voltages = rng.uniform(0, 60, size=NUM_SECTIONS)
            power = sim.step(voltages)
            assert BEATNOTE_MIN_DBM - 5 <= power <= BEATNOTE_MAX_DBM + 5, \
                f"Power {power} outside expected range"

    def test_no_drift_no_noise_stable(self):
        """With zero drift and zero noise, power should be constant for fixed voltages."""
        sim = PolarizationSimulator(
            noise_sigma_dbm=0.0,
            drift_amplitude=0.0,
            initial_sop_input=[0.3, 0.5, 0.8],
        )
        voltages = [30.0] * NUM_SECTIONS
        powers = [sim.step(voltages) for _ in range(100)]
        assert np.std(powers) < 0.01, \
            f"Power should be constant, std={np.std(powers)}"

    def test_monotonicity_near_optimum(self):
        """
        Small movement toward optimum should increase expected power
        (averaged over noise).
        """
        sim = PolarizationSimulator(
            noise_sigma_dbm=0.1,
            drift_amplitude=0.0,
            initial_sop_input=[0.3, 0.5, 0.8],
            rng_seed=42,
        )

        # Find a good voltage setting via coarse search
        best_v, best_p = sim.get_optimal_voltages()

        # Now test: moving from a suboptimal point toward optimum increases power
        # Use a point away from optimum
        suboptimal_v = [0.0, 0.0, 0.0, 0.0]
        # Midpoint between suboptimal and optimal
        mid_v = [(suboptimal_v[i] + best_v[i]) / 2 for i in range(NUM_SECTIONS)]

        # Average power over many steps (noise + drift = 0)
        sim2 = PolarizationSimulator(
            noise_sigma_dbm=0.1,
            drift_amplitude=0.0,
            initial_sop_input=[0.3, 0.5, 0.8],
            rng_seed=42,
        )
        p_sub = np.mean([sim2.step(suboptimal_v) for _ in range(200)])
        p_mid = np.mean([sim2.step(mid_v) for _ in range(200)])

        sim3 = PolarizationSimulator(
            noise_sigma_dbm=0.1,
            drift_amplitude=0.0,
            initial_sop_input=[0.3, 0.5, 0.8],
            rng_seed=42,
        )
        p_opt = np.mean([sim3.step(best_v) for _ in range(200)])

        assert p_sub < p_mid < p_opt, \
            f"Monotonicity violated: sub={p_sub}, mid={p_mid}, opt={p_opt}"

    def test_voltage_clamping(self):
        """Voltages outside [0, 60] should be clamped, not crash."""
        sim = PolarizationSimulator(noise_sigma_dbm=0.0, drift_amplitude=0.0)
        # Should not crash
        power = sim.step([-10.0, 100.0, 50.0, -5.0])
        assert BEATNOTE_MIN_DBM <= power <= BEATNOTE_MAX_DBM

    def test_reproducible_with_seed(self):
        """Same seed produces same trajectory."""
        sim1 = PolarizationSimulator(rng_seed=123, noise_sigma_dbm=0.5,
                                     drift_amplitude=0.3)
        sim2 = PolarizationSimulator(rng_seed=123, noise_sigma_dbm=0.5,
                                     drift_amplitude=0.3)
        voltages = [30.0] * NUM_SECTIONS
        for _ in range(100):
            p1 = sim1.step(voltages)
            p2 = sim2.step(voltages)
            assert abs(p1 - p2) < 1e-10, f"Non-reproducible: {p1} vs {p2}"


class TestScenarios:
    """Test that all scenario presets can be created and run."""

    @pytest.mark.parametrize("scenario_fn", [
        scenario_stable,
        scenario_slow_drift,
        scenario_fast_drift,
        scenario_regime_switch,
        scenario_cold_start,
        scenario_sudden_fade,
        scenario_channel_degradation,
    ])
    def test_scenario_runs(self, scenario_fn):
        """Each scenario should run for at least 100 steps without error."""
        sim = scenario_fn(rng_seed=42)
        voltages = [30.0] * NUM_SECTIONS
        for _ in range(100):
            power = sim.step(voltages)
            assert BEATNOTE_MIN_DBM - 10 <= power <= BEATNOTE_MAX_DBM + 10

    def test_stable_scenario_low_variance(self):
        """Stable scenario should have low power variance with fixed voltages."""
        sim = scenario_stable()
        voltages = [30.0] * NUM_SECTIONS
        powers = [sim.step(voltages) for _ in range(500)]
        # With no drift, variance should be small (only noise)
        assert np.std(powers) < 1.0

    def test_sudden_fade_triggers_drop(self):
        """Sudden fade scenario should show a power drop around step 5000."""
        # Use custom sim with zero drift to isolate the fade effect
        sim = PolarizationSimulator(
            channel_ceiling_dbm=CHANNEL_CEILING_DBM,
            noise_mode='white',
            noise_sigma_dbm=0.1,
            drift_amplitude=0.0,
            initial_sop_input=[0.3, 0.5, 0.8],
            sop_reference=[1.0, 0.0, 0.0],
            rng_seed=42,
        )
        fade_step = 5000
        original_step = sim.step

        def faded_step(voltages, dt_ms=1.0):
            if sim._step_count == fade_step:
                # Flip the input SOP to a very different position
                sim.sop_input = normalize(np.array([-0.3, -0.5, -0.8]))
            return original_step(voltages, dt_ms)

        sim.step = faded_step

        # Find good voltages for initial SOP
        best_v, _ = sim.get_optimal_voltages()
        # Run with optimal voltages (fixed — controller hasn't adapted yet)
        powers = [sim.step(best_v) for _ in range(6000)]
        # Before fade, power should be near ceiling
        pre_fade = np.mean(powers[4500:4900])
        # After fade, power should drop (old voltages don't match new SOP)
        post_fade = np.mean(powers[5000:5400])
        assert post_fade < pre_fade - 3, \
            f"Expected power drop: pre={pre_fade}, post={post_fade}"

    def test_channel_degradation_lowers_ceiling(self):
        """Channel degradation should lower the achievable ceiling over time."""
        sim = scenario_channel_degradation()
        initial_ceiling = sim.channel_ceiling_dbm
        for _ in range(10000):
            sim.step([0.0] * NUM_SECTIONS)
        assert sim.channel_ceiling_dbm < initial_ceiling, \
            f"Ceiling should decrease: {sim.channel_ceiling_dbm} < {initial_ceiling}"

    def test_regime_switch_changes_drift(self):
        """Regime switch should increase drift rate after switch point."""
        sim = scenario_regime_switch()
        # Run 10000 steps with zero voltages
        powers = [sim.step([0.0] * NUM_SECTIONS) for _ in range(10000)]
        # First half should have less variance than second half
        first_half_var = np.std(powers[:4000])
        second_half_var = np.std(powers[6000:])
        assert second_half_var > first_half_var, \
            f"Expected more variance in fast-drift regime: first={first_half_var}, second={second_half_var}"
