#include "fixedpoint.h"

/* Placeholder — full implementation in Phase 1 */
fp_t fp_from_float(float x) {
    (void)x;
    return 0;
}

float fp_to_float(fp_t x) {
    return (float)x / (float)FP_ONE;
}

fp_t fp_mul(fp_t a, fp_t b) {
    (void)a; (void)b;
    return 0;
}

fp_t fp_div(fp_t a, fp_t b) {
    (void)a; (void)b;
    return 0;
}

fp_t fp_clamp(fp_t x, fp_t lo, fp_t hi) {
    (void)lo; (void)hi;
    return x;
}

fp_t fp_abs(fp_t x) {
    return (x < 0) ? (fp_t)(-x) : x;
}
