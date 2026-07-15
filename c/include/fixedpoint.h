#ifndef FIXEDPOINT_H
#define FIXEDPOINT_H

#include <stdint.h>

/*
 * Fixed-point arithmetic: Q8.8 format on int16_t.
 *
 * Range: approx +/-127.996
 * Resolution: 1/256 ~= 0.0039
 *
 * 8 bits signed integer part, 8 bits fractional.
 *
 * Convention: fp_from_float / fp_to_float are ONLY for tests and
 * compile-time constant initialization. The algorithm core (baseline,
 * spsa, fsm, bandit, polctrl) must NOT use float/double.
 */

typedef int16_t fp_t;

#define FP_SHIFT 8
#define FP_ONE   (1 << FP_SHIFT)

/* Maximum/minimum representable values */
#define FP_MAX   32767
#define FP_MIN   (-32768)

/* Conversion — ONLY for tests/initialization, never in algorithm core! */
fp_t  fp_from_float(float x);
float fp_to_float(fp_t x);

/* Arithmetic via int32_t intermediate to avoid overflow */
fp_t fp_mul(fp_t a, fp_t b);
fp_t fp_div(fp_t a, fp_t b);

/* Utilities */
fp_t fp_clamp(fp_t x, fp_t lo, fp_t hi);
fp_t fp_abs(fp_t x);

#endif /* FIXEDPOINT_H */
