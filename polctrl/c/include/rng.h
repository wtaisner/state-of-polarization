#ifndef RNG_H
#define RNG_H

#include <stdint.h>

/* Deterministic xorshift32 PRNG */

typedef uint32_t rng_state_t;

rng_state_t rng_init(uint32_t seed);
uint32_t    rng_next(rng_state_t *state);

#endif /* RNG_H */
