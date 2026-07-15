#ifndef BANDIT_H
#define BANDIT_H

#include <stdint.h>
#include "fixedpoint.h"
#include "fsm.h"  /* for SpsaGainProfile */

/*
 * Contextual bandit (tabular UCB1) for adaptive SPSA gain selection.
 *
 * Context: (drift_estimate, noise_sigma_estimate) -> 6 buckets
 *   3 drift levels (low/mid/high) x 2 noise levels (low/high)
 *
 * Arms: 4 predefined SPSA gain profiles.
 *
 * UCB1 selection:
 *   arm = argmax_a( Q[bucket][a] + C * sqrt(ln(N+1) / (n[bucket][a]+1)) )
 *
 * sqrt and ln implemented via LUT (no FPU needed).
 * Q-value update: EMA with fixed alpha (avoids precision issues with large counts).
 */

#define BANDIT_LUT_SIZE 128

typedef struct {
    fp_t q_value[NUM_CONTEXT_BUCKETS][NUM_ARMS];
    uint32_t count[NUM_CONTEXT_BUCKETS][NUM_ARMS];
    uint32_t total_count;
} BanditState;

/* Predefined gain profiles for each arm (defined in bandit.c) */
extern const SpsaGainProfile ARM_PROFILES[NUM_ARMS];

/* Initialize bandit state. */
BanditState bandit_init(void);

/*
 * Discretize context (drift, noise) into bucket index [0, NUM_CONTEXT_BUCKETS).
 * drift_estimate: EMA of mean |grad| (Q8.8)
 * noise_sigma_estimate: estimated noise sigma (Q8.8)
 */
uint8_t discretize_context(fp_t drift_estimate, fp_t noise_sigma_estimate);

/*
 * Select arm using UCB1.
 * Returns arm index [0, NUM_ARMS).
 */
uint8_t bandit_select_arm(BanditState s, uint8_t context_bucket);

/*
 * Update bandit with observed reward.
 * Uses EMA update: Q += alpha * (reward - Q), alpha = 1/min(count+1, 255).
 */
BanditState bandit_update(BanditState s, uint8_t context_bucket, uint8_t arm,
                          fp_t reward);

#endif /* BANDIT_H */
