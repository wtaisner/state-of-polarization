"""
scenario_runner.py — Run the full stack (simulator + C controller) on all
scenarios from Phase 2, generating plots for visual inspection.

Usage:
    python3 scripts/scenario_runner.py

Outputs:
    scripts/output/*.png — plots for each scenario

This is NOT an automated test (no assertions) — it's a tool for visual
inspection by a human.
"""

import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt

# Add project paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, os.path.join(PROJECT_DIR, 'python'))

from bindings import polctrl_init, polctrl_step, PolCtrlOutput
from simulator import (
    PolarizationSimulator,
    scenario_stable, scenario_slow_drift, scenario_fast_drift,
    scenario_regime_switch, scenario_cold_start, scenario_sudden_fade,
    scenario_channel_degradation,
)
import fixedpoint as fp
from constants import NUM_SECTIONS, CHANNEL_CEILING_DBM

OUTPUT_DIR = os.path.join(SCRIPT_DIR, 'output')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Simulation parameters
N_STEPS = 20000  # 20 seconds at 1ms sampling
RNG_SEED = 42


def dbm_to_q88(dbm):
    """Convert dBm to internal Q8.8 unit: (dBm + 65) * 256."""
    val = int((dbm + 65) * 256)
    return max(0, min(7680, val))


def q88_to_dbm(q88):
    """Convert Q8.8 internal unit back to dBm."""
    return q88 / 256.0 - 65.0


def run_scenario(name, sim, n_steps=N_STEPS, seed=RNG_SEED):
    """Run a scenario and return data for plotting."""
    import ctypes
    from bindings import get_lib
    lib = get_lib()

    state = lib.polctrl_init(ctypes.c_uint32(seed))
    voltages = [30.0] * NUM_SECTIONS

    power_log = []
    voltages_log = [[] for _ in range(NUM_SECTIONS)]
    fsm_log = []
    spsa_sub_log = []
    arm_log = []

    for step in range(n_steps):
        # Get power from simulator
        power_dbm = sim.step(voltages)
        power_log.append(power_dbm)

        # Convert to Q8.8
        reading_q88 = dbm_to_q88(power_dbm)

        # Run controller
        out = PolCtrlOutput()
        state = lib.polctrl_step(state, ctypes.c_int16(reading_q88),
                                  ctypes.byref(out))

        # Update voltages if actuated
        if out.actuate:
            voltages = [fp.fp_to_float(out.voltages[i]) for i in range(NUM_SECTIONS)]

        for i in range(NUM_SECTIONS):
            voltages_log[i].append(voltages[i])

        fsm_log.append(state.fsm.mode)
        spsa_sub_log.append(state.spsa_sub)
        arm_log.append(state.current_arm)

    return {
        'power': np.array(power_log),
        'voltages': [np.array(v) for v in voltages_log],
        'fsm_mode': np.array(fsm_log),
        'spsa_sub': np.array(spsa_sub_log),
        'arm': np.array(arm_log),
    }


def plot_scenario(name, data, sim):
    """Generate plots for a scenario."""
    fig, axes = plt.subplots(4, 1, figsize=(14, 16), sharex=True)
    t = np.arange(len(data['power'])) / 1000.0  # seconds

    # Power
    ax = axes[0]
    ax.plot(t, data['power'], 'b-', linewidth=0.5, alpha=0.7)
    ax.axhline(y=CHANNEL_CEILING_DBM, color='r', linestyle='--', alpha=0.5,
               label=f'Ceiling ({CHANNEL_CEILING_DBM} dBm)')
    ax.set_ylabel('Beat note (dBm)')
    ax.set_title(f'Scenario: {name}')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Voltages
    ax = axes[1]
    colors = ['C0', 'C1', 'C2', 'C3']
    for i in range(NUM_SECTIONS):
        ax.plot(t, data['voltages'][i], colors[i], linewidth=0.5,
                label=f'Section {i}')
    ax.set_ylabel('Voltage (V)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # FSM mode
    ax = axes[2]
    ax.plot(t, data['fsm_mode'], 'k-', linewidth=1)
    ax.set_ylabel('FSM mode')
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(['TRACK', 'SEARCH', 'RECOVERY'])
    ax.grid(True, alpha=0.3)

    # Bandit arm
    ax = axes[3]
    ax.plot(t, data['arm'], 'g-', linewidth=1)
    ax.set_ylabel('Bandit arm')
    ax.set_xlabel('Time (s)')
    ax.set_yticks([0, 1, 2, 3])
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    output_path = os.path.join(OUTPUT_DIR, f'{name}.png')
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"  Saved: {output_path}")


def main():
    print("PolCtrl Scenario Runner")
    print("=" * 50)

    scenarios = [
        ('stable', scenario_stable),
        ('slow_drift', scenario_slow_drift),
        ('fast_drift', scenario_fast_drift),
        ('regime_switch', scenario_regime_switch),
        ('cold_start', scenario_cold_start),
        ('sudden_fade', scenario_sudden_fade),
        ('channel_degradation', scenario_channel_degradation),
    ]

    for name, scenario_fn in scenarios:
        print(f"\nRunning: {name}...")
        sim = scenario_fn(rng_seed=RNG_SEED)
        data = run_scenario(name, sim)
        plot_scenario(name, data, sim)

        # Summary stats
        power = data['power']
        print(f"  Power: mean={np.mean(power):.1f} dBm, "
              f"min={np.min(power):.1f}, max={np.max(power):.1f}")
        print(f"  FSM: TRACK={np.sum(data['fsm_mode']==0)}, "
              f"SEARCH={np.sum(data['fsm_mode']==1)}, "
              f"RECOVERY={np.sum(data['fsm_mode']==2)}")
        movement = sum(np.sum(np.abs(np.diff(data['voltages'][i])))
                       for i in range(NUM_SECTIONS))
        print(f"  Total movement: {movement:.1f} V")

    print(f"\nDone. Plots saved to {OUTPUT_DIR}/")


if __name__ == '__main__':
    main()
