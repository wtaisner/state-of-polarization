"""
Tests for the adaptive baseline and sigma estimation module.
Tests through ctypes bindings to the C implementation.
"""
import os
import sys
import ctypes

import pytest

from bindings import get_lib, BaselineState
import fixedpoint as fp

lib = get_lib()


class TestBaseline:
    """Test adaptive baseline and noise sigma estimation."""

    def test_init(self):
        s = lib.baseline_init()
        assert s.baseline == 0
        assert s.noise_sigma >= 1  # epsilon
        assert s.y_fast == 0
        assert s.y_slow == 0
        assert s.initialized == 0
        assert s.warmup_counter == 0

    def test_cold_start_returns_large_zscore(self):
        """During cold-start, z-score should be large (forces SEARCH)."""
        s = lib.baseline_init()
        z = lib.baseline_zscore(s)
        assert z == fp.FP_MAX  # 32767

    def test_warmup_initializes_baseline(self):
        """After COLD_START_WARMUP iterations, baseline is initialized."""
        from constants import COLD_START_WARMUP
        s = lib.baseline_init()
        reading = fp.fp_from_float(27.0)  # -38 dBm -> 27 in internal units

        for _ in range(COLD_START_WARMUP):
            s = lib.baseline_update(s, reading, 1)  # in deadzone

        assert s.initialized == 1
        # Baseline should be close to the reading (y_slow converges)
        assert abs(s.baseline - s.y_slow) < 100  # within ~0.4 internal units

    def test_baseline_fast_rise(self):
        """Skok w górę y_slow -> baseline reaguje natychmiast."""
        from constants import COLD_START_WARMUP
        s = lib.baseline_init()
        reading_low = fp.fp_from_float(20.0)
        reading_high = fp.fp_from_float(27.0)

        # Warmup with low reading
        for _ in range(COLD_START_WARMUP):
            s = lib.baseline_update(s, reading_low, 1)
        assert s.initialized == 1
        old_baseline = s.baseline

        # Feed high reading until y_slow catches up
        for _ in range(500):
            s = lib.baseline_update(s, reading_high, 1)

        # Baseline should have risen to track y_slow
        assert s.baseline > old_baseline
        assert s.baseline >= s.y_slow - 10  # baseline >= y_slow (tracks up)

    def test_baseline_slow_fall(self):
        """Trwały spadek y_slow -> baseline schodzi wolno."""
        from constants import COLD_START_WARMUP, BASELINE_DECAY_Q88
        s = lib.baseline_init()
        reading_high = fp.fp_from_float(27.0)
        reading_low = fp.fp_from_float(20.0)

        # Warmup with high reading
        for _ in range(COLD_START_WARMUP):
            s = lib.baseline_update(s, reading_high, 1)
        assert s.initialized == 1
        baseline_at_peak = s.baseline

        # Feed low reading for M iterations
        M = 100
        for _ in range(M):
            s = lib.baseline_update(s, reading_low, 1)

        # Baseline should have dropped, but slowly
        drop = baseline_at_peak - s.baseline
        # Should have dropped by at most M * BASELINE_DECAY_Q88
        # (since y_slow < baseline triggers decay)
        assert drop > 0, "Baseline should have dropped"
        assert drop <= M * BASELINE_DECAY_Q88 + 10, \
            f"Baseline dropped too fast: {drop} > {M * BASELINE_DECAY_Q88}"

    def test_sigma_not_contaminated_by_real_drop(self):
        """Wstrzyknięcie prawdziwego spadku (nie w dead-zone) -> sigma nie rośnie."""
        from constants import COLD_START_WARMUP
        s = lib.baseline_init()
        reading = fp.fp_from_float(27.0)

        # Warmup
        for _ in range(COLD_START_WARMUP):
            s = lib.baseline_update(s, reading, 1)
        sigma_before = s.noise_sigma

        # Feed a large drop, but NOT in deadzone (currently_in_deadzone=0)
        low_reading = fp.fp_from_float(15.0)
        for _ in range(50):
            s = lib.baseline_update(s, low_reading, 0)  # NOT in deadzone

        # Sigma should not have increased significantly
        # (it's only updated when in deadzone)
        assert s.noise_sigma <= sigma_before + 5, \
            f"Sigma contaminated: {s.noise_sigma} > {sigma_before + 5}"

    def test_sigma_converges_to_noise_level(self):
        """Czysty szum bez dryfu -> sigma zbiega do wartości bliskiej rzeczywistej."""
        from constants import COLD_START_WARMUP
        s = lib.baseline_init()

        # Feed noisy readings around a stable value
        import random
        rng = random.Random(42)
        base_reading = fp.fp_from_float(27.0)

        for _ in range(COLD_START_WARMUP + 5000):
            noise = fp.fp_from_float(rng.gauss(0, 0.5))
            reading = base_reading + noise
            s = lib.baseline_update(s, reading, 1)  # in deadzone

        # Sigma uses |y_raw - y_slow| which tracks the noise well.
        # The EMA smoothing introduces some lag, but after 5000 iterations
        # it should be close to the true sigma.
        sigma_float = fp.fp_to_float(s.noise_sigma)
        assert 0.2 < sigma_float < 1.0, \
            f"Sigma {sigma_float} not close to expected ~0.5"
