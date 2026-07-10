#include "rng.h"

/* Placeholder — full implementation in Phase 3 */
rng_state_t rng_init(uint32_t seed) {
    return (seed == 0) ? 1 : seed;
}

uint32_t rng_next(rng_state_t *state) {
    *state ^= *state << 13;
    *state ^= *state >> 17;
    *state ^= *state << 5;
    return *state;
}
