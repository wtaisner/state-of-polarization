#include "rng.h"

/*
 * xorshift32 PRNG (George Marsaglia).
 *
 *   x ^= x << 13;
 *   x ^= x >> 17;
 *   x ^= x << 5;
 *
 * Period: 2^32 - 1. Cannot use 0 as state (maps to 0 forever).
 * Seed 0 is replaced with 1.
 *
 * This exact sequence is mirrored in Python for parity testing.
 */

rng_state_t rng_init(uint32_t seed)
{
    return (seed == 0) ? 1u : seed;
}

uint32_t rng_next(rng_state_t *state)
{
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

int8_t rng_sign(rng_state_t *state)
{
    uint32_t r = rng_next(state);
    return (r & 1u) ? 1 : -1;
}
