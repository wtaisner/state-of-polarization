/*
 * size_check.c — helper binary printing sizeof of all public structures.
 * Used by test_parity_c_vs_python.py to verify ctypes layout matches C.
 */
#include <stdio.h>
#include "polctrl.h"
#include "fixedpoint.h"
#include "polctrl_internal.h"
#include "baseline.h"
#include "spsa.h"
#include "fsm.h"
#include "bandit.h"

int main(void) {
    printf("sizeof(fp_t) = %zu\n", sizeof(fp_t));
    printf("sizeof(PolCtrlOutput) = %zu\n", sizeof(PolCtrlOutput));
    printf("sizeof(PolCtrlState) = %zu\n", sizeof(PolCtrlState));
    printf("sizeof(BaselineState) = %zu\n", sizeof(BaselineState));
    printf("sizeof(FsmState) = %zu\n", sizeof(FsmState));
    printf("sizeof(SpsaState) = %zu\n", sizeof(SpsaState));
    printf("sizeof(BanditState) = %zu\n", sizeof(BanditState));
    printf("sizeof(SpsaGainProfile) = %zu\n", sizeof(SpsaGainProfile));
    printf("sizeof(SpsaSubState) = %zu\n", sizeof(SpsaSubState));
    printf("FP_SHIFT = %d\n", FP_SHIFT);
    printf("FP_ONE = %d\n", FP_ONE);
    printf("NUM_SECTIONS = %d\n", NUM_SECTIONS);
    printf("NUM_ARMS = %d\n", NUM_ARMS);
    printf("NUM_CONTEXT_BUCKETS = %d\n", NUM_CONTEXT_BUCKETS);
    return 0;
}
