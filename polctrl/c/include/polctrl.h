#ifndef POLCTRL_H
#define POLCTRL_H

#include <stdint.h>
#include "fixedpoint.h"

/*
 * Public API for the polarization controller.
 * Full documentation in Phase 3g / README_HAL.md.
 */

#define NUM_SECTIONS 4

typedef struct {
    uint8_t actuate;
    fp_t voltages[NUM_SECTIONS];
} PolCtrlOutput;

/* Placeholder state — full definition in polctrl_internal.h */
typedef struct PolCtrlState {
    uint32_t placeholder;
} PolCtrlState;

#endif /* POLCTRL_H */
