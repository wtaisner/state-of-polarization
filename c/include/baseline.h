#ifndef BASELINE_H
#define BASELINE_H

#include <stdint.h>
#include "fixedpoint.h"

/*
 * Adaptive baseline and noise sigma estimation.
 *
 * Tracks:
 *   - y_fast: fast EMA of power (detects sudden drops)
 *   - y_slow: slow EMA of power (baseline reference, SPSA measurement)
 *   - baseline: estimated achievable power ceiling
 *   - noise_sigma: estimated measurement noise std dev
 *
 * Input unit: Q8.8 "shifted dBm" = (dBm + 65) * 256.
 * See polctrl_internal.h for scaling convention.
 */

typedef struct {
    fp_t baseline;       /* Estimated achievable ceiling (Q8.8) */
    fp_t noise_sigma;    /* Estimated noise std dev (Q8.8) */
    fp_t y_fast;         /* Fast EMA (Q8.8) */
    fp_t y_slow;         /* Slow EMA (Q8.8) */
    uint8_t initialized; /* 0 during cold-start, 1 after warmup */
    uint16_t warmup_counter; /* Counts samples during cold-start */
} BaselineState;

/* Initialize baseline state (cold-start). */
BaselineState baseline_init(void);

/*
 * Update baseline with new raw reading.
 *
 * Parameters:
 *   s                      - current state
 *   y_raw                  - raw power reading (Q8.8)
 *   currently_in_deadzone  - 1 if controller is in dead-zone (sigma updated only then)
 *
 * Returns updated state.
 */
BaselineState baseline_update(BaselineState s, fp_t y_raw,
                              uint8_t currently_in_deadzone);

/*
 * Compute z-score: (baseline - y_slow) / max(noise_sigma, eps).
 * Positive = degradation. Returns large value during cold-start.
 */
fp_t baseline_zscore(BaselineState s);

#endif /* BASELINE_H */
