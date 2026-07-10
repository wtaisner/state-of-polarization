"""
Fixed-point arithmetic: Q8.8 format, mirroring c/src/fixedpoint.c exactly.

All operations produce bit-identical results to the C implementation,
including overflow/wrapping and rounding behavior.

Conventions (must match C):
  - fp_mul: arithmetic right shift (>> on signed int = truncation toward -inf)
  - fp_div: C integer division (truncation toward zero)
  - Overflow: clamp to int16_t range [-32768, 32767] after each operation
"""

import struct

FP_SHIFT = 8
FP_ONE = 1 << FP_SHIFT
FP_MAX = 32767
FP_MIN = -32768


def _to_int16(x):
    """Simulate int16_t: mask to 16 bits and sign-extend."""
    x = x & 0xFFFF
    if x >= 0x8000:
        x -= 0x10000
    return x


def _to_int32(x):
    """Simulate int32_t: mask to 32 bits and sign-extend."""
    x = x & 0xFFFFFFFF
    if x >= 0x80000000:
        x -= 0x100000000
    return x


def _c_div(a, b):
    """
    Integer division truncating toward zero (matching C's / operator).
    Python's // truncates toward -inf, so we need this wrapper.
    """
    if b == 0:
        raise ZeroDivisionError
    q = abs(a) // abs(b)
    if (a < 0) != (b < 0):
        return -q
    return q


def fp_from_float(x: float) -> int:
    """Convert float to Q8.8. ONLY for tests/initialization."""
    scaled = x * FP_ONE
    # Round to nearest (matching C implementation)
    if scaled >= 0:
        scaled = int(scaled + 0.5)
    else:
        scaled = int(scaled - 0.5)
    if scaled > FP_MAX:
        scaled = FP_MAX
    if scaled < FP_MIN:
        scaled = FP_MIN
    return _to_int16(scaled)


def fp_to_float(x: int) -> float:
    """Convert Q8.8 to float. ONLY for tests/debugging."""
    return x / FP_ONE


def fp_mul(a: int, b: int) -> int:
    """
    Multiply two Q8.8 values. Uses int32_t intermediate.
    Arithmetic right shift (truncation toward -inf), matching C >>.
    """
    temp = _to_int32(a * b)
    # Python's >> on negative integers does arithmetic shift (toward -inf),
    # matching C's >> on signed integers in gcc.
    temp = temp >> FP_SHIFT
    if temp > FP_MAX:
        temp = FP_MAX
    if temp < FP_MIN:
        temp = FP_MIN
    return _to_int16(temp)


def fp_div(a: int, b: int) -> int:
    """
    Divide two Q8.8 values. Uses int32_t intermediate.
    C integer division (truncation toward zero).
    Division by zero returns saturating value.
    """
    if b == 0:
        if a > 0:
            return FP_MAX
        if a < 0:
            return FP_MIN
        return 0

    temp = _to_int32((a << FP_SHIFT))
    temp = _c_div(temp, b)

    if temp > FP_MAX:
        temp = FP_MAX
    if temp < FP_MIN:
        temp = FP_MIN
    return _to_int16(temp)


def fp_clamp(x: int, lo: int, hi: int) -> int:
    """Clamp x to [lo, hi]."""
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def fp_abs(x: int) -> int:
    """Absolute value. Guards against FP_MIN overflow."""
    if x < 0:
        if x == FP_MIN:
            return FP_MAX
        return -x
    return x
