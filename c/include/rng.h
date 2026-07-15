#ifndef RNG_H
#define RNG_H

#include <stdint.h>

/*
 * Deterministic xorshift32 PRNG.
 *
 * Same algorithm in C and Python (python/fixedpoint.py reference).
 * Seed of 0 is mapped to 1 (xorshift32 cannot use 0 as state).
 */

typedef uint32_t rng_state_t;

/* Initialize PRNG state from seed. */
rng_state_t rng_init(uint32_t seed);

/*
 * Generate next random 32-bit value.
 * Updates state in place.
 */
uint32_t rng_next(rng_state_t *state);

/*
 * Generate a random sign: returns +1 or -1.
 * Uses one bit from rng_next.
 */
int8_t rng_sign(rng_state_t *state);

#endif /* RNG_H */
