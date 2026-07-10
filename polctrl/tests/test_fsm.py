"""
Tests for the FSM (TRACK/SEARCH/RECOVERY) module.
"""
import os
import sys
import ctypes

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from bindings import get_lib, FsmState, STATE_TRACK, STATE_SEARCH, STATE_RECOVERY
import fixedpoint as fp
from constants import K1_DEADZONE_Q88, K2_SEARCH_Q88, HYSTERESIS_WINDOWS

lib = get_lib()


class TestFsm:
    """Test FSM state transitions and dead-zone gate."""

    def test_init(self):
        s = lib.fsm_init()
        assert s.mode == STATE_TRACK
        assert s.consecutive_good_windows == 0
        assert s.periodic_probe_counter == 0

    def test_low_zscore_no_actuation(self):
        """zscore below k1 -> should_actuate returns 0."""
        s = lib.fsm_init()
        zscore = fp.fp_from_float(0.5)  # well below k1=2.5
        assert lib.fsm_should_actuate(s, zscore) == 0

    def test_high_zsearch_triggers_search(self):
        """zscore above k2 -> SEARCH mode."""
        s = lib.fsm_init()
        zscore = K2_SEARCH_Q88 + 1  # just above k2
        s = lib.fsm_update(s, zscore, 0, 0)
        assert s.mode == STATE_SEARCH

    def test_search_to_recovery_requires_hysteresis(self):
        """SEARCH -> RECOVERY -> TRACK requires HYSTERESIS_WINDOWS good windows."""
        s = lib.fsm_init()
        # Enter SEARCH
        s = lib.fsm_update(s, K2_SEARCH_Q88 + 1, 0, 0)
        assert s.mode == STATE_SEARCH

        # Drop zscore below k1 -> should enter RECOVERY
        s = lib.fsm_update(s, fp.fp_from_float(0.5), 0, 0)
        assert s.mode == STATE_RECOVERY
        assert s.consecutive_good_windows == 1

        # Need HYSTERESIS_WINDOWS - 1 more good windows
        for i in range(HYSTERESIS_WINDOWS - 2):
            s = lib.fsm_update(s, fp.fp_from_float(0.5), 0, 0)
            assert s.mode == STATE_RECOVERY  # still in RECOVERY

        # Final good window -> TRACK
        s = lib.fsm_update(s, fp.fp_from_float(0.5), 0, 0)
        assert s.mode == STATE_TRACK

    def test_no_oscillation_around_k1(self):
        """zscore migoczące wokół k1 nie powoduje natychmiastowego przełączenia."""
        s = lib.fsm_init()
        # Enter SEARCH first
        s = lib.fsm_update(s, K2_SEARCH_Q88 + 1, 0, 0)
        assert s.mode == STATE_SEARCH

        # Now oscillate zscore around k1
        for _ in range(10):
            # Below k1 -> start counting
            s = lib.fsm_update(s, K1_DEADZONE_Q88 - 10, 0, 0)
            if s.mode == STATE_RECOVERY:
                break
            # Above k2 -> back to SEARCH
            s = lib.fsm_update(s, K2_SEARCH_Q88 + 1, 0, 0)

        # After oscillation, should not be in TRACK (hysteresis prevents it)
        # (Could be in SEARCH or RECOVERY depending on exact sequence)
        assert s.mode != STATE_TRACK or s.consecutive_good_windows == 0

    def test_search_mode_always_actuates(self):
        """In SEARCH mode, should_actuate always returns 1."""
        s = lib.fsm_init()
        s = lib.fsm_update(s, K2_SEARCH_Q88 + 1, 0, 0)
        assert s.mode == STATE_SEARCH
        # Even with low zscore
        assert lib.fsm_should_actuate(s, fp.fp_from_float(0.1)) == 1

    def test_sudden_fade_detection(self):
        """y_fast << y_slow triggers SEARCH immediately."""
        s = lib.fsm_init()
        y_slow = fp.fp_from_float(27.0)  # 6912
        y_fast = fp.fp_from_float(10.0)  # much lower -> sudden fade
        s = lib.fsm_update(s, fp.fp_from_float(0.5), y_fast, y_slow)
        assert s.mode == STATE_SEARCH

    def test_no_sudden_fade_when_close(self):
        """y_fast close to y_slow does not trigger sudden fade."""
        s = lib.fsm_init()
        y_slow = fp.fp_from_float(27.0)
        y_fast = fp.fp_from_float(26.5)  # small difference
        s = lib.fsm_update(s, fp.fp_from_float(0.5), y_fast, y_slow)
        assert s.mode == STATE_TRACK  # no transition

    def test_periodic_probe(self):
        """Periodic probe fires after PERIODIC_PROBE_INTERVAL samples."""
        from constants import PERIODIC_PROBE_INTERVAL
        s = lib.fsm_init()

        # Should not fire before interval
        for _ in range(PERIODIC_PROBE_INTERVAL - 1):
            assert lib.fsm_check_periodic_probe(ctypes.byref(s)) == 0

        # Should fire on the PERIODIC_PROBE_INTERVAL-th call
        assert lib.fsm_check_periodic_probe(ctypes.byref(s)) == 1

        # Counter resets after firing
        assert s.periodic_probe_counter == 0

    def test_periodic_probe_disabled_in_search(self):
        """Periodic probe is disabled in SEARCH mode."""
        from constants import PERIODIC_PROBE_INTERVAL
        s = lib.fsm_init()
        s.mode = STATE_SEARCH

        # Run for well past the interval
        for _ in range(PERIODIC_PROBE_INTERVAL + 100):
            result = lib.fsm_check_periodic_probe(ctypes.byref(s))
            assert result == 0  # never fires in SEARCH

    def test_recovery_to_search_on_regression(self):
        """In RECOVERY, if zscore > k2, go back to SEARCH."""
        s = lib.fsm_init()
        # Enter SEARCH
        s = lib.fsm_update(s, K2_SEARCH_Q88 + 1, 0, 0)
        # Enter RECOVERY
        s = lib.fsm_update(s, fp.fp_from_float(0.5), 0, 0)
        assert s.mode == STATE_RECOVERY
        # Regression
        s = lib.fsm_update(s, K2_SEARCH_Q88 + 1, 0, 0)
        assert s.mode == STATE_SEARCH
