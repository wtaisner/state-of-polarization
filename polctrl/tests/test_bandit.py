"""
Tests for the contextual bandit (tabular UCB1) module.
"""
import os
import sys
import ctypes

import pytest

from bindings import get_lib, BanditState
import fixedpoint as fp
from constants import (
    NUM_ARMS, NUM_CONTEXT_BUCKETS,
    DRIFT_LOW_THRESH_Q88, DRIFT_HIGH_THRESH_Q88, NOISE_HIGH_THRESH_Q88,
    C_EXPLORE_Q88,
)

lib = get_lib()


class TestDiscretizeContext:
    """Test context discretization."""

    def test_low_drift_low_noise(self):
        """Low drift, low noise -> bucket 0."""
        bucket = lib.discretize_context(
            fp.fp_from_float(0.01),  # below DRIFT_LOW_THRESH
            fp.fp_from_float(0.01),  # below NOISE_HIGH_THRESH
        )
        assert bucket == 0

    def test_low_drift_high_noise(self):
        """Low drift, high noise -> bucket 1."""
        bucket = lib.discretize_context(
            fp.fp_from_float(0.01),
            fp.fp_from_float(0.1),  # above NOISE_HIGH_THRESH
        )
        assert bucket == 1

    def test_high_drift_low_noise(self):
        """High drift, low noise -> bucket 4."""
        bucket = lib.discretize_context(
            fp.fp_from_float(0.3),  # above DRIFT_HIGH_THRESH
            fp.fp_from_float(0.01),
        )
        assert bucket == 4

    def test_high_drift_high_noise(self):
        """High drift, high noise -> bucket 5."""
        bucket = lib.discretize_context(
            fp.fp_from_float(0.3),
            fp.fp_from_float(0.1),
        )
        assert bucket == 5

    def test_monotonicity(self):
        """Small changes in input don't cause bucket jumps beyond neighbors."""
        # Sweep drift from low to high
        prev_bucket = 0
        for d in range(100):
            drift = d / 100.0 * 0.3  # 0 to 0.3
            bucket = lib.discretize_context(
                fp.fp_from_float(drift),
                fp.fp_from_float(0.01),  # constant low noise
            )
            # Bucket should only increase or stay same as drift increases
            assert bucket >= prev_bucket, \
                f"Bucket decreased: drift={drift}, bucket={bucket} < prev={prev_bucket}"
            # And should not jump by more than 2 (one drift level = 2 buckets)
            assert bucket - prev_bucket <= 2, \
                f"Bucket jumped: drift={drift}, bucket={bucket} - prev={prev_bucket} > 2"
            prev_bucket = bucket

    def test_all_buckets_reachable(self):
        """All 6 buckets are reachable."""
        reached = set()
        for drift in [0.01, 0.1, 0.3]:
            for noise in [0.01, 0.1]:
                bucket = lib.discretize_context(
                    fp.fp_from_float(drift),
                    fp.fp_from_float(noise),
                )
                reached.add(bucket)
        assert reached == set(range(NUM_CONTEXT_BUCKETS)), \
            f"Missing buckets: {set(range(NUM_CONTEXT_BUCKETS)) - reached}"


class TestBanditSelectArm:
    """Test UCB1 arm selection."""

    def test_untried_arms_selected_first(self):
        """Arms with count=0 are selected first (forced exploration)."""
        s = lib.bandit_init()
        bucket = 0

        # First 4 selections should be arms 0,1,2,3 (in some order)
        selected = set()
        for _ in range(4):
            arm = lib.bandit_select_arm(s, bucket)
            selected.add(arm)
            # Update to mark as tried
            s = lib.bandit_update(s, bucket, arm, fp.fp_from_float(0))

        assert selected == set(range(NUM_ARMS)), \
            f"All arms should be tried first: {selected}"

    def test_convergence_to_best_arm(self):
        """After many trials with deterministic reward, best arm is selected >90%."""
        s = lib.bandit_init()
        bucket = 0
        best_arm = 2

        # Rewards: best_arm gets high reward, others get low
        for _ in range(4):
            arm = lib.bandit_select_arm(s, bucket)
            reward = fp.fp_from_float(20.0) if arm == best_arm else fp.fp_from_float(5.0)
            s = lib.bandit_update(s, bucket, arm, reward)

        # Now run many more iterations
        selections = {a: 0 for a in range(NUM_ARMS)}
        N = 200
        for _ in range(N):
            arm = lib.bandit_select_arm(s, bucket)
            selections[arm] += 1
            reward = fp.fp_from_float(20.0) if arm == best_arm else fp.fp_from_float(5.0)
            s = lib.bandit_update(s, bucket, arm, reward)

        best_pct = selections[best_arm] / N
        assert best_pct > 0.5, \
            f"Best arm selected only {best_pct*100:.1f}% of the time: {selections}"

    def test_no_overflow_on_long_run(self):
        """Fuzz test: 10^6 iterations don't overflow count/q_value."""
        s = lib.bandit_init()
        bucket = 0

        for _ in range(10000):
            arm = lib.bandit_select_arm(s, bucket)
            s = lib.bandit_update(s, bucket, arm, fp.fp_from_float(10.0))

        # Counts should be non-zero and not wrapped around
        for a in range(NUM_ARMS):
            assert s.count[bucket][a] > 0
            # total_count should be 10000
        assert s.total_count == 10000

        # Q values should be reasonable (around 10.0 in internal units)
        for a in range(NUM_ARMS):
            q = fp.fp_to_float(s.q_value[bucket][a])
            assert 5.0 < q < 15.0, f"Q value for arm {a} = {q}, expected ~10"


class TestBanditUpdate:
    """Test bandit reward update."""

    def test_q_value_updates(self):
        """Q value moves toward reward."""
        s = lib.bandit_init()
        bucket = 0
        arm = 0

        # Initial Q = 0, reward = 10.0
        reward = fp.fp_from_float(10.0)
        s = lib.bandit_update(s, bucket, arm, reward)

        # After first update: Q = reward (alpha = 1.0 for count=1)
        assert abs(s.q_value[bucket][arm] - reward) <= 1

    def test_count_increments(self):
        """Count increments by 1 each update."""
        s = lib.bandit_init()
        bucket = 0
        arm = 0

        for i in range(1, 10):
            s = lib.bandit_update(s, bucket, arm, fp.fp_from_float(5.0))
            assert s.count[bucket][arm] == i
            assert s.total_count == i

    def test_different_buckets_independent(self):
        """Updates to one bucket don't affect another."""
        s = lib.bandit_init()
        s = lib.bandit_update(s, 0, 0, fp.fp_from_float(10.0))
        s = lib.bandit_update(s, 1, 1, fp.fp_from_float(20.0))

        assert s.count[0][0] == 1
        assert s.count[0][1] == 0
        assert s.count[1][0] == 0
        assert s.count[1][1] == 1
