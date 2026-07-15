#include "fixedpoint.h"

/*
 * Q8.8 fixed-point arithmetic implementation.
 *
 * All arithmetic uses int32_t as intermediate type to prevent overflow
 * during multiplication and division, then results are clamped back to
 * int16_t (fp_t).
 *
 * Rounding convention:
 *   - fp_mul: arithmetic right shift (truncation toward -inf).
 *     This matches Python's >> operator on signed integers.
 *   - fp_div: C integer division (truncation toward zero).
 *     Python mirror uses explicit truncation-toward-zero to match.
 *
 * Both choices are fully deterministic and produce bit-identical results
 * between C and Python.
 */

fp_t fp_from_float(float x)
{
    float scaled = x * (float)FP_ONE;
    /* Round to nearest */
    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }
    if (scaled > (float)FP_MAX) scaled = (float)FP_MAX;
    if (scaled < (float)FP_MIN) scaled = (float)FP_MIN;
    return (fp_t)scaled;
}

float fp_to_float(fp_t x)
{
    return (float)x / (float)FP_ONE;
}

fp_t fp_mul(fp_t a, fp_t b)
{
    int32_t temp = (int32_t)a * (int32_t)b;
    /*
     * Arithmetic right shift (>> on signed int in gcc = sign-extending).
     * This truncates toward negative infinity, matching Python's >>.
     * Result range: temp is at most ~32767*32767 ~= 1.07e9, fits int32_t.
     * After >> 8: at most ~4.19e6, must clamp to int16_t.
     */
    temp = temp >> FP_SHIFT;
    if (temp > FP_MAX) temp = FP_MAX;
    if (temp < FP_MIN) temp = FP_MIN;
    return (fp_t)temp;
}

fp_t fp_div(fp_t a, fp_t b)
{
    int32_t temp;

    /* Division by zero: return saturating value */
    if (b == 0) {
        if (a > 0)  return FP_MAX;
        if (a < 0)  return FP_MIN;
        return 0;
    }

    /*
     * (a << FP_SHIFT) / b in Q8.8.
     * a is int16_t [-32768, 32767], << 8 gives [-8388608, 8388608], fits int32_t.
     * C division truncates toward zero.
     */
    temp = ((int32_t)a << FP_SHIFT) / (int32_t)b;

    if (temp > FP_MAX) temp = FP_MAX;
    if (temp < FP_MIN) temp = FP_MIN;
    return (fp_t)temp;
}

fp_t fp_clamp(fp_t x, fp_t lo, fp_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

fp_t fp_abs(fp_t x)
{
    if (x < 0) {
        /* Guard against FP_MIN (-32768) where -x would overflow */
        if (x == FP_MIN) return FP_MAX;
        return (fp_t)(-x);
    }
    return x;
}
