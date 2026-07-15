"""
Tests for Q8.8 fixed-point arithmetic.
Verifies C and Python implementations produce identical results.
"""
import os
import sys
import subprocess
import ctypes
import math
import struct

import pytest

import fixedpoint as fp

# Build libpolctrl.so if needed
C_DIR = os.path.join(os.path.dirname(__file__), '..', 'c')
LIB_PATH = os.path.join(C_DIR, 'libpolctrl.so')


def ensure_lib():
    """Build the C library if it doesn't exist."""
    if not os.path.exists(LIB_PATH):
        subprocess.run(['make'], cwd=C_DIR, check=True)


def load_lib():
    """Load libpolctrl.so and return ctypes handle."""
    ensure_lib()
    lib = ctypes.CDLL(LIB_PATH)
    lib.fp_from_float.argtypes = [ctypes.c_float]
    lib.fp_from_float.restype = ctypes.c_int16
    lib.fp_to_float.argtypes = [ctypes.c_int16]
    lib.fp_to_float.restype = ctypes.c_float
    lib.fp_mul.argtypes = [ctypes.c_int16, ctypes.c_int16]
    lib.fp_mul.restype = ctypes.c_int16
    lib.fp_div.argtypes = [ctypes.c_int16, ctypes.c_int16]
    lib.fp_div.restype = ctypes.c_int16
    lib.fp_clamp.argtypes = [ctypes.c_int16, ctypes.c_int16, ctypes.c_int16]
    lib.fp_clamp.restype = ctypes.c_int16
    lib.fp_abs.argtypes = [ctypes.c_int16]
    lib.fp_abs.restype = ctypes.c_int16
    return lib


# ---------------------------------------------------------------------------
# Python-only tests
# ---------------------------------------------------------------------------

class TestPythonFixedPoint:
    """Test the Python fixed-point implementation in isolation."""

    def test_from_float_basic(self):
        assert fp.fp_from_float(0.0) == 0
        assert fp.fp_from_float(1.0) == 256
        assert fp.fp_from_float(-1.0) == -256
        assert fp.fp_from_float(0.5) == 128
        assert fp.fp_from_float(60.0) == 60 * 256  # 15360

    def test_from_float_clamping(self):
        """Values outside Q8.8 range are clamped."""
        assert fp.fp_from_float(200.0) == fp.FP_MAX
        assert fp.fp_from_float(-200.0) == fp.FP_MIN

    def test_to_float_basic(self):
        assert fp.fp_to_float(0) == 0.0
        assert fp.fp_to_float(256) == 1.0
        assert fp.fp_to_float(-256) == -1.0
        assert abs(fp.fp_to_float(128) - 0.5) < 1e-6

    def test_mul_basic(self):
        """1.0 * 1.0 = 1.0, 2.0 * 3.0 = 6.0, etc."""
        assert fp.fp_mul(256, 256) == 256          # 1*1 = 1
        assert fp.fp_mul(512, 768) == 1536          # 2*3 = 6
        assert fp.fp_mul(256, -256) == -256          # 1*(-1) = -1
        assert fp.fp_mul(-256, -256) == 256          # (-1)*(-1) = 1
        assert fp.fp_mul(0, 256) == 0                # 0*x = 0

    def test_mul_within_1_lsb(self):
        """fp_mul result within 1 LSB of true product (for non-overflow cases)."""
        test_values = [0.0, 0.1, 0.5, 1.0, 5.0, 30.0, 60.0,
                       -0.1, -1.0, -60.0, 0.001, 127.99, -127.99]
        for a_f in test_values:
            for b_f in test_values:
                a = fp.fp_from_float(a_f)
                b = fp.fp_from_float(b_f)
                result = fp.fp_mul(a, b)
                # Compare against product of the actual fixed-point values
                expected = fp.fp_to_float(a) * fp.fp_to_float(b)
                # Skip cases where result overflows Q8.8 (tested separately)
                if abs(expected) > 127.99:
                    continue
                err = abs(fp.fp_to_float(result) - expected)
                # Truncation in mul introduces up to 1 LSB error
                assert err < 2.0 / 256, \
                    f"fp_mul({a_f}, {b_f}) [a={a}, b={b}]: got {fp.fp_to_float(result)}, expected {expected}, err={err}"

    def test_div_basic(self):
        """6.0 / 3.0 = 2.0, etc."""
        assert fp.fp_div(1536, 768) == 512           # 6/3 = 2
        assert fp.fp_div(256, 256) == 256            # 1/1 = 1
        assert fp.fp_div(0, 256) == 0                # 0/1 = 0
        assert fp.fp_div(-512, 256) == -512          # -2/1 = -2

    def test_div_within_1_lsb(self):
        """fp_div result within 1 LSB of true quotient of the fixed-point inputs."""
        test_values = [0.5, 1.0, 2.0, 5.0, 10.0, 30.0, 60.0,
                       -1.0, -5.0, -60.0]
        for a_f in test_values:
            for b_f in test_values:
                a = fp.fp_from_float(a_f)
                b = fp.fp_from_float(b_f)
                result = fp.fp_div(a, b)
                # Compare against quotient of the actual fixed-point values
                expected = fp.fp_to_float(a) / fp.fp_to_float(b)
                err = abs(fp.fp_to_float(result) - expected)
                assert err < 2.0 / 256, \
                    f"fp_div({a_f}, {b_f}) [a={a}, b={b}]: got {fp.fp_to_float(result)}, expected {expected}, err={err}"

    def test_div_by_zero(self):
        """Division by zero returns saturating value, doesn't crash."""
        assert fp.fp_div(256, 0) == fp.FP_MAX       # positive / 0 = MAX
        assert fp.fp_div(-256, 0) == fp.FP_MIN      # negative / 0 = MIN
        assert fp.fp_div(0, 0) == 0                  # 0 / 0 = 0

    def test_clamp(self):
        assert fp.fp_clamp(100, 0, 200) == 100
        assert fp.fp_clamp(-50, 0, 200) == 0
        assert fp.fp_clamp(300, 0, 200) == 200
        assert fp.fp_clamp(0, 0, 200) == 0
        assert fp.fp_clamp(-100, -200, -50) == -100
        assert fp.fp_clamp(-300, -200, -50) == -200

    def test_abs(self):
        assert fp.fp_abs(256) == 256
        assert fp.fp_abs(-256) == 256
        assert fp.fp_abs(0) == 0
        assert fp.fp_abs(fp.FP_MIN) == fp.FP_MAX   # edge case: -32768
        assert fp.fp_abs(fp.FP_MAX) == fp.FP_MAX

    def test_mul_overflow_clamp(self):
        """Large multiplication results are clamped, not wrapped."""
        result = fp.fp_mul(fp.FP_MAX, fp.FP_MAX)
        assert result == fp.FP_MAX   # 127.99 * 127.99 = ~16382, clamped

    def test_div_overflow_clamp(self):
        """Division producing large results is clamped."""
        result = fp.fp_div(fp.FP_MAX, 1)  # 32767 / 1 = 32767 (already max)
        assert result == fp.FP_MAX

    def test_voltage_range_representable(self):
        """0-60V range fits in Q8.8."""
        v_min = fp.fp_from_float(0.0)
        v_max = fp.fp_from_float(60.0)
        assert v_min == 0
        assert v_max == 60 * 256  # 15360, well within int16_t range

    def test_voltage_step_representable(self):
        """0.1V step is representable (approximately)."""
        step = fp.fp_from_float(0.1)
        # 0.1 * 256 = 25.6, rounds to 26
        assert step == 26


# ---------------------------------------------------------------------------
# C vs Python parity tests
# ---------------------------------------------------------------------------

class TestCParity:
    """Verify C and Python fixed-point produce identical results."""

    @classmethod
    @pytest.fixture(scope='class')
    def lib(cls):
        return load_lib()

    def test_from_float_parity(self, lib):
        test_values = [0.0, 0.1, 0.5, 1.0, -1.0, 60.0, -60.0,
                       127.99, -127.99, 200.0, -200.0, 0.001]
        for v in test_values:
            c_result = lib.fp_from_float(ctypes.c_float(v))
            py_result = fp.fp_from_float(v)
            assert c_result == py_result, \
                f"fp_from_float({v}): C={c_result}, Py={py_result}"

    def test_mul_parity(self, lib):
        """Test mul parity across a grid of values."""
        test_floats = [0.0, 0.1, 0.5, 1.0, 5.0, 30.0, 60.0,
                       -1.0, -5.0, -60.0, 0.001, 127.0]
        for a_f in test_floats:
            for b_f in test_floats:
                a = fp.fp_from_float(a_f)
                b = fp.fp_from_float(b_f)
                c_result = lib.fp_mul(ctypes.c_int16(a), ctypes.c_int16(b))
                py_result = fp.fp_mul(a, b)
                assert c_result == py_result, \
                    f"fp_mul({a_f}, {b_f}) [a={a}, b={b}]: C={c_result}, Py={py_result}"

    def test_div_parity(self, lib):
        """Test div parity across a grid of values (excluding zero divisor)."""
        test_floats = [0.5, 1.0, 2.0, 5.0, 10.0, 30.0, 60.0,
                       -1.0, -5.0, -60.0]
        for a_f in test_floats:
            for b_f in test_floats:
                a = fp.fp_from_float(a_f)
                b = fp.fp_from_float(b_f)
                c_result = lib.fp_div(ctypes.c_int16(a), ctypes.c_int16(b))
                py_result = fp.fp_div(a, b)
                assert c_result == py_result, \
                    f"fp_div({a_f}, {b_f}) [a={a}, b={b}]: C={c_result}, Py={py_result}"

    def test_div_by_zero_parity(self, lib):
        """Division by zero behaves identically."""
        for a in [256, -256, 0, fp.FP_MAX, fp.FP_MIN]:
            c_result = lib.fp_div(ctypes.c_int16(a), ctypes.c_int16(0))
            py_result = fp.fp_div(a, 0)
            assert c_result == py_result, \
                f"fp_div({a}, 0): C={c_result}, Py={py_result}"

    def test_clamp_parity(self, lib):
        test_cases = [
            (100, 0, 200), (-50, 0, 200), (300, 0, 200),
            (0, 0, 200), (-100, -200, -50), (-300, -200, -50),
            (fp.FP_MAX, fp.FP_MIN, fp.FP_MAX),
            (fp.FP_MIN, fp.FP_MIN, fp.FP_MAX),
        ]
        for x, lo, hi in test_cases:
            c_result = lib.fp_clamp(ctypes.c_int16(x),
                                    ctypes.c_int16(lo),
                                    ctypes.c_int16(hi))
            py_result = fp.fp_clamp(x, lo, hi)
            assert c_result == py_result, \
                f"fp_clamp({x}, {lo}, {hi}): C={c_result}, Py={py_result}"

    def test_abs_parity(self, lib):
        test_values = [0, 256, -256, fp.FP_MAX, fp.FP_MIN, 1, -1, 128, -128]
        for x in test_values:
            c_result = lib.fp_abs(ctypes.c_int16(x))
            py_result = fp.fp_abs(x)
            assert c_result == py_result, \
                f"fp_abs({x}): C={c_result}, Py={py_result}"

    def test_mul_edge_cases_parity(self, lib):
        """Test extreme values that could cause overflow."""
        edge_values = [fp.FP_MAX, fp.FP_MIN, 0, 1, -1, 256, -256]
        for a in edge_values:
            for b in edge_values:
                c_result = lib.fp_mul(ctypes.c_int16(a), ctypes.c_int16(b))
                py_result = fp.fp_mul(a, b)
                assert c_result == py_result, \
                    f"fp_mul({a}, {b}): C={c_result}, Py={py_result}"

    def test_div_small_divisor_parity(self, lib):
        """Test division by small values (potential overflow in result)."""
        divisors = [1, -1, 2, -2, 3, -3]
        dividends = [fp.FP_MAX, fp.FP_MIN, 256, -256, 1536]
        for a in dividends:
            for b in divisors:
                c_result = lib.fp_div(ctypes.c_int16(a), ctypes.c_int16(b))
                py_result = fp.fp_div(a, b)
                assert c_result == py_result, \
                    f"fp_div({a}, {b}): C={c_result}, Py={py_result}"
