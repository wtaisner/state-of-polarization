"""
simulator.py — Physical model of the fiber-optic link with polarization controller.

Models:
  - SOP as a point on the Poincaré sphere (normalized Stokes vector s1,s2,s3)
  - 4-section piezoelectric fiber squeezer controller (retarders)
  - Beat note power: I = cos²(angle(SOP_out, SOP_ref) / 2), scaled to dBm
  - SOP drift: Ornstein-Uhlenbeck process on azimuth/elevation
  - Measurement noise: (a) white Gaussian, (b) interferometric composite
  - Configurable channel ceiling (best achievable power)

Actuator model:
  Sections {0,1} rotate SOP around axis A = (1,0,0) [s1 axis].
  Sections {2,3} rotate SOP around axis B = (0,1,0) [s2 axis, orthogonal to A].
  Rotation angle: phi_i = (V_i / V_max) * MAX_ROTATION_RAD
  Sequential application: SOP_out = R3 * R2 * R1 * R0 * SOP_in

  This is a simplification of real fiber squeezers but captures the key
  properties: monotonicity, smoothness, and existence of a local maximum
  in power as a function of voltages.
"""

import numpy as np

from constants import (
    NUM_SECTIONS, V_MIN_VOLT, V_MAX_VOLT,
    BEATNOTE_MIN_DBM, BEATNOTE_MAX_DBM, CHANNEL_CEILING_DBM,
)


def normalize(v):
    """Normalize a 3D vector."""
    n = np.linalg.norm(v)
    if n < 1e-15:
        return np.array([1.0, 0.0, 0.0])
    return v / n


def rotation_matrix(axis, angle):
    """
    Rodrigues' rotation formula for 3D rotation around axis by angle.
    axis: normalized 3D vector
    angle: radians
    Returns 3x3 rotation matrix.
    """
    c = np.cos(angle)
    s = np.sin(angle)
    t = 1.0 - c
    x, y, z = axis
    return np.array([
        [t*x*x + c,   t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z, t*y*y + c,   t*y*z - s*x],
        [t*x*z - s*y, t*y*z + s*x, t*z*z + c  ],
    ])


def angle_between(v1, v2):
    """Angle between two vectors (radians), clamped to [0, pi]."""
    dot = np.dot(normalize(v1), normalize(v2))
    dot = np.clip(dot, -1.0, 1.0)
    return np.arccos(dot)


class PolarizationSimulator:
    """
    Physical simulator of the fiber-optic link.

    Parameters (all configurable for different scenarios):
        channel_ceiling_dbm: Best achievable power when SOP is perfectly matched.
        noise_mode: 'white' or 'interferometric'
        noise_sigma_dbm: Std dev of white measurement noise (dBm).
        drift_tau_ms: OU time constant for SOP drift (ms). Larger = slower drift.
        drift_amplitude: OU amplitude for SOP drift (radians).
        initial_sop_input: Initial input SOP (before controller). Default: random.
        sop_reference: Reference SOP (target). Default: (1,0,0).
        max_rotation_rad: Max rotation per section at V_max. Default: 2*pi.
        rng_seed: Random seed for reproducibility.
        degradation_rate: Rate of channel ceiling decrease (dBm per ms).
                         Default: 0 (no degradation).
    """

    def __init__(self,
                 channel_ceiling_dbm=CHANNEL_CEILING_DBM,
                 noise_mode='white',
                 noise_sigma_dbm=0.5,
                 drift_tau_ms=1000.0,
                 drift_amplitude=0.0,
                 initial_sop_input=None,
                 sop_reference=None,
                 max_rotation_rad=2.0 * np.pi,
                 rng_seed=42,
                 degradation_rate=0.0):

        self.channel_ceiling_dbm = channel_ceiling_dbm
        self.noise_mode = noise_mode
        self.noise_sigma_dbm = noise_sigma_dbm
        self.drift_tau_ms = drift_tau_ms
        self.drift_amplitude = drift_amplitude
        self.max_rotation_rad = max_rotation_rad
        self.degradation_rate = degradation_rate  # dBm per ms

        # Reference SOP (target)
        if sop_reference is not None:
            self.sop_reference = normalize(np.array(sop_reference, dtype=float))
        else:
            self.sop_reference = np.array([1.0, 0.0, 0.0])

        # Initial input SOP (before controller)
        self.rng = np.random.RandomState(rng_seed)
        if initial_sop_input is not None:
            self.sop_input = normalize(np.array(initial_sop_input, dtype=float))
        else:
            self.sop_input = self._random_sop()

        # OU process state for drift (azimuth and elevation)
        self._drift_azimuth = 0.0
        self._drift_elevation = 0.0

        # Interferometric noise parameters (pre-generated)
        self._noise_phases = self.rng.uniform(0, 2*np.pi, size=5)
        self._noise_freqs = np.array([0.5, 1.3, 2.7, 4.1, 7.9])  # Hz
        self._noise_amps = np.array([0.3, 0.2, 0.15, 0.1, 0.05]) * noise_sigma_dbm
        self._noise_time = 0.0  # ms

        # Axis definitions for actuator
        self._axis_A = np.array([1.0, 0.0, 0.0])  # s1 axis
        self._axis_B = np.array([0.0, 1.0, 0.0])  # s2 axis

        self._step_count = 0

    def _random_sop(self):
        """Generate a random point on the unit sphere."""
        v = self.rng.randn(3)
        return normalize(v)

    def _apply_actuator(self, sop, voltages):
        """Apply 4-section retarder transformation to SOP."""
        result = sop.copy()
        for i in range(NUM_SECTIONS):
            v_clamped = np.clip(voltages[i], V_MIN_VOLT, V_MAX_VOLT)
            angle = (v_clamped / V_MAX_VOLT) * self.max_rotation_rad
            if i < 2:
                axis = self._axis_A
            else:
                axis = self._axis_B
            R = rotation_matrix(axis, angle)
            result = R @ result
        return normalize(result)

    def _update_drift(self, dt_ms):
        """Update SOP input via Ornstein-Uhlenbeck process on azimuth/elevation."""
        if self.drift_amplitude == 0.0:
            return

        tau = self.drift_tau_ms
        sigma = self.drift_amplitude

        # OU update: dX = -X/tau * dt + sigma * sqrt(dt) * dW
        dt = dt_ms
        sqrt_dt = np.sqrt(dt)
        dW_az = self.rng.randn() * sqrt_dt
        dW_el = self.rng.randn() * sqrt_dt

        self._drift_azimuth += (-self._drift_azimuth / tau) * dt + sigma * dW_az
        self._drift_elevation += (-self._drift_elevation / tau) * dt + sigma * dW_el

        # Apply drift to SOP input via small rotation
        # Rotate around s3 axis by azimuth drift, then around s2 by elevation
        R_az = rotation_matrix(np.array([0.0, 0.0, 1.0]),
                               self._drift_azimuth * dt / tau)
        R_el = rotation_matrix(np.array([0.0, 1.0, 0.0]),
                               self._drift_elevation * dt / tau)
        self.sop_input = normalize(R_el @ (R_az @ self.sop_input))

    def _add_noise(self, power_dbm, dt_ms):
        """Add measurement noise to power reading."""
        if self.noise_mode == 'white':
            noise = self.rng.randn() * self.noise_sigma_dbm
        elif self.noise_mode == 'interferometric':
            # Composite: sum of sinusoids + white noise
            t = self._noise_time / 1000.0  # convert ms to seconds
            composite = 0.0
            for i in range(len(self._noise_freqs)):
                composite += self._noise_amps[i] * np.sin(
                    2 * np.pi * self._noise_freqs[i] * t + self._noise_phases[i])
            white = self.rng.randn() * self.noise_sigma_dbm * 0.3
            noise = composite + white
        else:
            noise = 0.0

        self._noise_time += dt_ms
        return power_dbm + noise

    def _compute_power_dbm(self, sop_out):
        """
        Compute beat note power in dBm.
        I = cos²(angle(SOP_out, SOP_ref) / 2), scaled to dBm.
        """
        ang = angle_between(sop_out, self.sop_reference)
        # Normalized intensity [0, 1]
        intensity = np.cos(ang / 2.0) ** 2

        # Convert to dBm relative to channel ceiling
        eps = 1e-10
        power_dbm = self.channel_ceiling_dbm + 10.0 * np.log10(intensity + eps)

        # Clip to physical detector range
        power_dbm = np.clip(power_dbm, BEATNOTE_MIN_DBM, BEATNOTE_MAX_DBM)
        return float(power_dbm)

    def step(self, voltages, dt_ms=1.0):
        """
        Advance simulation by one step.

        Parameters:
            voltages: array of 4 voltages [V] for the piezo sections.
            dt_ms: time step in milliseconds (default 1ms).

        Returns:
            Noisy power reading in dBm.
        """
        # Update drift
        self._update_drift(dt_ms)

        # Apply actuator to input SOP
        sop_out = self._apply_actuator(self.sop_input, voltages)

        # Compute power
        power = self._compute_power_dbm(sop_out)

        # Add noise
        power_noisy = self._add_noise(power, dt_ms)

        # Apply channel degradation
        if self.degradation_rate != 0.0:
            self.channel_ceiling_dbm -= self.degradation_rate * dt_ms
            self.channel_ceiling_dbm = max(self.channel_ceiling_dbm,
                                           BEATNOTE_MIN_DBM + 1.0)

        self._step_count += 1
        return power_noisy

    def get_optimal_voltages(self):
        """
        Find voltages that best match SOP_input to SOP_reference.
        Uses grid search (for testing/debugging only).
        Returns the best voltage tuple found.
        """
        best_power = -100.0
        best_v = [30.0] * NUM_SECTIONS
        # Coarse search
        grid = np.linspace(0, 60, 13)  # 5V steps
        for v0 in grid:
            for v1 in grid:
                for v2 in grid:
                    for v3 in grid:
                        v = [v0, v1, v2, v3]
                        sop_out = self._apply_actuator(self.sop_input, v)
                        p = self._compute_power_dbm(sop_out)
                        if p > best_power:
                            best_power = p
                            best_v = v
        return best_v, best_power


# ===========================================================================
# Scenario presets
# ===========================================================================

def scenario_stable(rng_seed=42):
    """No drift, only measurement noise. Tests dead-zone."""
    return PolarizationSimulator(
        channel_ceiling_dbm=CHANNEL_CEILING_DBM,
        noise_mode='white',
        noise_sigma_dbm=0.3,
        drift_tau_ms=10000.0,
        drift_amplitude=0.0,
        initial_sop_input=[0.3, 0.5, 0.8],
        rng_seed=rng_seed,
    )


def scenario_slow_drift(rng_seed=42):
    """Slow, steady drift."""
    return PolarizationSimulator(
        channel_ceiling_dbm=CHANNEL_CEILING_DBM,
        noise_mode='white',
        noise_sigma_dbm=0.5,
        drift_tau_ms=2000.0,
        drift_amplitude=0.3,
        initial_sop_input=[0.3, 0.5, 0.8],
        rng_seed=rng_seed,
    )


def scenario_fast_drift(rng_seed=42):
    """Fast drift."""
    return PolarizationSimulator(
        channel_ceiling_dbm=CHANNEL_CEILING_DBM,
        noise_mode='interferometric',
        noise_sigma_dbm=1.0,
        drift_tau_ms=200.0,
        drift_amplitude=1.5,
        initial_sop_input=[0.3, 0.5, 0.8],
        rng_seed=rng_seed,
    )


def scenario_regime_switch(rng_seed=42):
    """
    First half: slow drift, second half: fast drift.
    Tests bandit adaptivity.
    """
    sim = PolarizationSimulator(
        channel_ceiling_dbm=CHANNEL_CEILING_DBM,
        noise_mode='white',
        noise_sigma_dbm=0.5,
        drift_tau_ms=2000.0,
        drift_amplitude=0.3,
        initial_sop_input=[0.3, 0.5, 0.8],
        rng_seed=rng_seed,
    )
    # We'll handle the regime switch in the step method via a wrapper
    sim._regime_switch_step = 5000  # Switch after 5 seconds (5000 ms)
    sim._fast_drift_tau = 200.0
    sim._fast_drift_amp = 1.5
    sim._original_step = sim.step

    def switched_step(voltages, dt_ms=1.0):
        if sim._step_count >= sim._regime_switch_step:
            sim.drift_tau_ms = sim._fast_drift_tau
            sim.drift_amplitude = sim._fast_drift_amp
        return sim._original_step(voltages, dt_ms)

    sim.step = switched_step
    return sim


def scenario_cold_start(rng_seed=42):
    """Start from random, bad SOP. Tests SEARCH mode."""
    rng = np.random.RandomState(rng_seed)
    bad_sop = normalize(rng.randn(3))
    return PolarizationSimulator(
        channel_ceiling_dbm=CHANNEL_CEILING_DBM,
        noise_mode='white',
        noise_sigma_dbm=0.5,
        drift_tau_ms=1000.0,
        drift_amplitude=0.2,
        initial_sop_input=bad_sop,
        rng_seed=rng_seed,
    )


def scenario_sudden_fade(rng_seed=42):
    """
    Sudden large SOP jump mid-scenario.
    Tests y_fast detection and SEARCH trigger.
    """
    sim = PolarizationSimulator(
        channel_ceiling_dbm=CHANNEL_CEILING_DBM,
        noise_mode='white',
        noise_sigma_dbm=0.5,
        drift_tau_ms=1000.0,
        drift_amplitude=0.1,
        initial_sop_input=[0.3, 0.5, 0.8],
        rng_seed=rng_seed,
    )
    sim._fade_step = 5000
    sim._original_step = sim.step

    def faded_step(voltages, dt_ms=1.0):
        if sim._step_count == sim._fade_step:
            # Sudden SOP jump (simulating cable hit)
            rng = np.random.RandomState(rng_seed + 999)
            new_sop = normalize(rng.randn(3))
            sim.sop_input = new_sop
        return sim._original_step(voltages, dt_ms)

    sim.step = faded_step
    return sim


def scenario_channel_degradation(rng_seed=42):
    """
    Slow, permanent channel ceiling decrease.
    Tests that adaptive baseline goes down without continuous SEARCH.
    """
    return PolarizationSimulator(
        channel_ceiling_dbm=CHANNEL_CEILING_DBM,
        noise_mode='white',
        noise_sigma_dbm=0.5,
        drift_tau_ms=5000.0,
        drift_amplitude=0.1,
        initial_sop_input=[0.3, 0.5, 0.8],
        rng_seed=rng_seed,
        degradation_rate=0.001,  # -0.001 dBm per ms = -1 dBm per second
    )
