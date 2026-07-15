#include "baseline.h"
#include "polctrl.h"  /* for ALPHA_FAST_Q88, ALPHA_SLOW_Q88, etc. */

/*
 * Adaptive baseline and sigma estimation.
 *
 * EMA update: y_new = y_old + alpha * (y_raw - y_old)
 *           = y_old + (y_raw - y_old) * alpha / FP_ONE
 *
 * In fixed-point: we use fp_mul for the alpha multiplication.
 * alpha_fast = 32/256 = 0.125, alpha_slow = 3/256 ~= 0.0117
 *
 * Baseline logic:
 *   - If y_slow > baseline: baseline = y_slow (fast rise)
 *   - Else: baseline -= BASELINE_DECAY_Q88 (slow fall)
 *
 * Sigma update (only in dead-zone):
 *   sigma = EMA of |y_fast - y_slow|
 *   This measures short-term fluctuation as noise proxy.
 */

/* Small epsilon to avoid division by zero in z-score */
#define SIGMA_EPS_Q88  1

BaselineState baseline_init(void)
{
    BaselineState s;
    s.baseline = 0;
    s.noise_sigma = SIGMA_EPS_Q88;
    s.y_fast = 0;
    s.y_slow = 0;
    s.initialized = 0;
    s.warmup_counter = 0;
    return s;
}

BaselineState baseline_update(BaselineState s, fp_t y_raw,
                              uint8_t currently_in_deadzone)
{
    fp_t diff;

    /* Update fast EMA: y_fast += alpha_fast * (y_raw - y_fast) */
    diff = y_raw - s.y_fast;
    s.y_fast = s.y_fast + fp_mul(ALPHA_FAST_Q88, diff);

    /* Update slow EMA: y_slow += alpha_slow * (y_raw - y_slow) */
    diff = y_raw - s.y_slow;
    s.y_slow = s.y_slow + fp_mul(ALPHA_SLOW_Q88, diff);

    /* Cold-start warmup */
    if (!s.initialized) {
        s.warmup_counter++;
        if (s.warmup_counter >= COLD_START_WARMUP) {
            s.initialized = 1;
            s.baseline = s.y_slow;
        }
        /* During cold-start, sigma still updates (for early noise estimate) */
    }

    /* Baseline update (only after initialization) */
    if (s.initialized) {
        if (s.y_slow > s.baseline) {
            /* Fast rise: baseline tracks up immediately */
            s.baseline = s.y_slow;
        } else {
            /* Slow fall: decay by small constant step */
            s.baseline = s.baseline - BASELINE_DECAY_Q88;
        }
    }

    /* Noise sigma update (only in dead-zone, to avoid contaminating
     * sigma with real signal drops).
     * Uses |y_raw - y_slow| as noise proxy — this is a better estimator
     * than |y_fast - y_slow| because y_slow tracks the mean and the
     * deviation of the raw signal from it captures the full noise. */
    if (currently_in_deadzone) {
        fp_t resid = fp_abs(y_raw - s.y_slow);
        /* sigma = EMA of |y_raw - y_slow| using alpha_slow */
        s.noise_sigma = s.noise_sigma + fp_mul(ALPHA_SLOW_Q88,
                                                resid - s.noise_sigma);
        /* Ensure sigma doesn't go below epsilon */
        if (s.noise_sigma < SIGMA_EPS_Q88) {
            s.noise_sigma = SIGMA_EPS_Q88;
        }
    }

    return s;
}

fp_t baseline_zscore(BaselineState s)
{
    if (!s.initialized) {
        /* During cold-start, return large value to force SEARCH */
        return FP_MAX;
    }

    /* z = (baseline - y_slow) / max(sigma, eps) */
    fp_t diff = s.baseline - s.y_slow;
    fp_t sigma = s.noise_sigma;
    if (sigma < SIGMA_EPS_Q88) {
        sigma = SIGMA_EPS_Q88;
    }
    return fp_div(diff, sigma);
}
