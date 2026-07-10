"""
bindings.py — ctypes wrapper for libpolctrl.so.

Provides a clean Pythonic interface to the C polarization controller.
All struct layouts match the C definitions exactly (verified by parity tests).
"""

import ctypes
import os
import subprocess

# === Path setup ===
_HERE = os.path.dirname(os.path.abspath(__file__))
_C_DIR = os.path.join(os.path.dirname(_HERE), 'c')
_LIB_PATH = os.path.join(_C_DIR, 'libpolctrl.so')


def _ensure_lib():
    """Build libpolctrl.so if it doesn't exist."""
    if not os.path.exists(_LIB_PATH):
        subprocess.run(['make'], cwd=_C_DIR, check=True)


def _load_lib():
    """Load and configure the C library."""
    _ensure_lib()
    lib = ctypes.CDLL(_LIB_PATH)

    # Fixed-point functions
    lib.fp_from_float.argtypes = [ctypes.c_float]
    lib.fp_from_float.restype = ctypes.c_int16
    lib.fp_to_float.argtypes = [ctypes.c_int16]
    lib.fp_to_float.restype = ctypes.c_float
    lib.fp_mul.argtypes = [ctypes.c_int16, ctypes.c_int16]
    lib.fp_mul.restype = ctypes.c_int16
    lib.fp_div.argtypes = [ctypes.c_int16, ctypes.c_int16]
    lib.fp_div.restype = ctypes.c_int16
    lib.fp_clamp.argtypes = [ctypes.c_int16, ctypes.c_int16, ctypes.c_int16]
    lib.fp_clamp.restype = ctypes.c_int16
    lib.fp_abs.argtypes = [ctypes.c_int16]
    lib.fp_abs.restype = ctypes.c_int16

    # RNG functions
    lib.rng_init.argtypes = [ctypes.c_uint32]
    lib.rng_init.restype = ctypes.c_uint32
    lib.rng_next.argtypes = [ctypes.POINTER(ctypes.c_uint32)]
    lib.rng_next.restype = ctypes.c_uint32
    lib.rng_sign.argtypes = [ctypes.POINTER(ctypes.c_uint32)]
    lib.rng_sign.restype = ctypes.c_int8

    # Baseline functions
    lib.baseline_init.argtypes = []
    lib.baseline_init.restype = BaselineState
    lib.baseline_update.argtypes = [BaselineState, ctypes.c_int16, ctypes.c_uint8]
    lib.baseline_update.restype = BaselineState
    lib.baseline_zscore.argtypes = [BaselineState]
    lib.baseline_zscore.restype = ctypes.c_int16

    # FSM functions
    lib.fsm_init.argtypes = []
    lib.fsm_init.restype = FsmState
    lib.fsm_update.argtypes = [FsmState, ctypes.c_int16,
                               ctypes.c_int16, ctypes.c_int16]
    lib.fsm_update.restype = FsmState
    lib.fsm_should_actuate.argtypes = [FsmState, ctypes.c_int16]
    lib.fsm_should_actuate.restype = ctypes.c_uint8
    lib.fsm_check_periodic_probe.argtypes = [ctypes.POINTER(FsmState)]
    lib.fsm_check_periodic_probe.restype = ctypes.c_uint8
    lib.fsm_gain_for_mode.argtypes = [ctypes.c_int, SpsaGainProfile]
    lib.fsm_gain_for_mode.restype = SpsaGainProfile

    # SPSA functions
    lib.spsa_compute_probe.argtypes = [
        SpsaState,
        ctypes.POINTER(ctypes.c_int16),  # out_plus
        ctypes.POINTER(ctypes.c_int16),  # out_minus
        ctypes.c_int16,  # a_gain
        ctypes.c_int16,  # c_gain
        ctypes.c_uint8,  # disable_boundary_weight
    ]
    lib.spsa_compute_probe.restype = SpsaState
    lib.spsa_apply_result.argtypes = [
        SpsaState, ctypes.c_int16, ctypes.c_int16, ctypes.c_int16
    ]
    lib.spsa_apply_result.restype = SpsaState
    lib.boundary_weight.argtypes = [ctypes.c_int16]
    lib.boundary_weight.restype = ctypes.c_int16
    lib.forced_inward_sign.argtypes = [ctypes.c_int16]
    lib.forced_inward_sign.restype = ctypes.c_int8
    lib.snap_to_voltage_grid.argtypes = [ctypes.c_int16]
    lib.snap_to_voltage_grid.restype = ctypes.c_int16

    # Bandit functions
    lib.bandit_init.argtypes = []
    lib.bandit_init.restype = BanditState
    lib.discretize_context.argtypes = [ctypes.c_int16, ctypes.c_int16]
    lib.discretize_context.restype = ctypes.c_uint8
    lib.bandit_select_arm.argtypes = [BanditState, ctypes.c_uint8]
    lib.bandit_select_arm.restype = ctypes.c_uint8
    lib.bandit_update.argtypes = [BanditState, ctypes.c_uint8,
                                   ctypes.c_uint8, ctypes.c_int16]
    lib.bandit_update.restype = BanditState

    # PolCtrl functions
    lib.polctrl_init.argtypes = [ctypes.c_uint32]
    lib.polctrl_init.restype = PolCtrlState
    lib.polctrl_step.argtypes = [
        PolCtrlState, ctypes.c_int16,
        ctypes.POINTER(PolCtrlOutput)
    ]
    lib.polctrl_step.restype = PolCtrlState

    return lib


# === ctypes Structure definitions ===

class BaselineState(ctypes.Structure):
    _fields_ = [
        ("baseline", ctypes.c_int16),
        ("noise_sigma", ctypes.c_int16),
        ("y_fast", ctypes.c_int16),
        ("y_slow", ctypes.c_int16),
        ("initialized", ctypes.c_uint8),
        ("warmup_counter", ctypes.c_uint16),
    ]


class FsmState(ctypes.Structure):
    _fields_ = [
        ("mode", ctypes.c_int),  # enum ControllerMode
        ("consecutive_good_windows", ctypes.c_uint16),
        ("periodic_probe_counter", ctypes.c_uint32),
    ]


class SpsaGainProfile(ctypes.Structure):
    _fields_ = [
        ("a_gain", ctypes.c_int16),
        ("c_gain", ctypes.c_int16),
    ]


class SpsaState(ctypes.Structure):
    _fields_ = [
        ("theta", ctypes.c_int16 * 4),
        ("last_grad_estimate", ctypes.c_int16 * 4),
        ("rng", ctypes.c_uint32),
        ("delta", ctypes.c_int16 * 4),
        ("c_k", ctypes.c_int16 * 4),
    ]


class BanditState(ctypes.Structure):
    _fields_ = [
        ("q_value", (ctypes.c_int16 * 4) * 6),  # [6][4]
        ("count", (ctypes.c_uint32 * 4) * 6),   # [6][4]
        ("total_count", ctypes.c_uint32),
    ]


class PolCtrlOutput(ctypes.Structure):
    _fields_ = [
        ("actuate", ctypes.c_uint8),
        ("voltages", ctypes.c_int16 * 4),
    ]


# SPSA sub-state enum values
SPSA_SUB_IDLE = 0
SPSA_SUB_SET_PLUS = 1
SPSA_SUB_MEASURE_PLUS = 2
SPSA_SUB_SET_MINUS = 3
SPSA_SUB_MEASURE_MINUS = 4
SPSA_SUB_APPLY = 5

# FSM mode enum values
STATE_TRACK = 0
STATE_SEARCH = 1
STATE_RECOVERY = 2


class PolCtrlState(ctypes.Structure):
    _fields_ = [
        ("baseline", BaselineState),
        ("fsm", FsmState),
        ("spsa", SpsaState),
        ("bandit", BanditState),
        ("spsa_sub", ctypes.c_int),  # enum SpsaSubState
        ("spsa_settle_counter", ctypes.c_uint16),
        ("y_plus", ctypes.c_int16),
        ("y_minus", ctypes.c_int16),
        ("bandit_iter_counter", ctypes.c_uint16),
        ("current_arm", ctypes.c_uint8),
        ("current_context", ctypes.c_uint8),
        ("reward_power_sum", ctypes.c_int16),
        ("reward_power_count", ctypes.c_uint16),
        ("reward_boundary_sum", ctypes.c_int16),
        ("reward_movement_sum", ctypes.c_int16),
        ("drift_estimate", ctypes.c_int16),
        ("periodic_probe_counter", ctypes.c_uint32),
        ("step_count", ctypes.c_uint32),
        ("current_gain", SpsaGainProfile),
    ]


# === Pythonic wrapper functions ===

_lib = None


def get_lib():
    """Get or lazily load the C library."""
    global _lib
    if _lib is None:
        _lib = _load_lib()
    return _lib


def polctrl_init(rng_seed):
    """Initialize controller. Returns PolCtrlState."""
    lib = get_lib()
    return lib.polctrl_init(ctypes.c_uint32(rng_seed))


def polctrl_step(state, beatnote_reading):
    """
    Advance controller by one step.

    Parameters:
        state: PolCtrlState
        beatnote_reading: int (Q8.8, (dBm + 65) * 256)

    Returns:
        (new_state, output) where output is PolCtrlOutput
    """
    lib = get_lib()
    out = PolCtrlOutput()
    new_state = lib.polctrl_step(state, ctypes.c_int16(beatnote_reading),
                                  ctypes.byref(out))
    return new_state, out
