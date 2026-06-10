"""Plant model for the FOC-rig simulator (modeled rig — pure Python, no deps).

The plant is the *mechanism*: a single rotary DOF (the motor shaft, 1:1 with the
firmware's `shaft_angle` in radians), with reflected carriage inertia, Coulomb +
viscous friction, two hall endstops at the travel ends, and hard stops beyond
them. The `SoftFirmware` feeds it a motor torque each control step and reads back
angle / velocity / hall levels.

`AnalyticPlant` is a 1-DOF semi-implicit Euler integrator with runtime fault
hooks (wear, hall slip/loss, glitches). Defaults are calibrated to the hardware
oracle in CLAUDE.md (travel ≈200 rad, v_safe ≈109, |Vq| ≈1.7 V at a stroke).
(A Genesis physics plant existed behind the same `Plant` interface but was
removed — the analytic model matched the oracle and needs no heavy deps.)

Direction convention (matches firmware `g_home_dir = +1`): **+velocity / +angle
moves toward MIN**. So the MIN hall sits at high absolute angle and MAX at low.
This is the as-built rig convention (firmware comment block in main.cpp).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

MIN = 0
MAX = 1


@dataclass
class PlantConfig:
    # --- electromechanical (defaults tuned to the CLAUDE.md hardware oracle:
    #     travel ~200 rad, v_safe ~109, Vq peaking ~1.6 V of 3 V at 60 rad/s) ---
    kv: float = 1000.0            # motor KV [rpm/V] -> Kt = Ke = 9.549 / KV
    phase_resistance: float = 0.15  # R [ohm] in the Vq->Iq model
    inertia: float = 2.0e-4       # J reflected to the shaft [kg·m^2]
    coulomb_friction: float = 5.0e-3   # static/Coulomb torque [N·m]
    viscous_friction: float = 2.0e-4   # viscous torque coeff [N·m/(rad/s)]
    friction_v_eps: float = 0.5   # rad/s; smooths the Coulomb sign change

    # --- geometry (absolute shaft angle, rad). +angle -> toward MIN. ---
    hall_min_pos: float = 100.0   # MIN hall triggers when angle >= this
    hall_max_pos: float = -100.0  # MAX hall triggers when angle <= this
    hall_clear_frac: float = 0.225  # clear space beyond each hall, as frac of travel
    start_angle: float = 0.0      # boot position (center; MIN must be CLEAR at boot)

    # --- supply / noise ---
    supply_voltage: float = 12.0
    iq_noise_a: float = 0.0       # std-dev of additive Iq measurement noise [A]
    angle_noise_rad: float = 0.0  # std-dev of additive sensor-angle noise [rad]

    @property
    def travel(self) -> float:
        return abs(self.hall_min_pos - self.hall_max_pos)

    @property
    def hard_stop_min(self) -> float:
        """Physical hard stop just past the MIN hall (high-angle end)."""
        return self.hall_min_pos + self.hall_clear_frac * self.travel

    @property
    def hard_stop_max(self) -> float:
        return self.hall_max_pos - self.hall_clear_frac * self.travel

    @property
    def kt(self) -> float:
        return 9.549 / self.kv


class _Faults:
    """Runtime-injectable faults, shared by both plant implementations."""
    def __init__(self, cfg: PlantConfig):
        self.cfg = cfg
        self.hall_offset = [0.0, 0.0]   # additive drift of each hall trigger pos (slip)
        self.hall_disabled = [False, False]
        self.coulomb = cfg.coulomb_friction
        self.viscous = cfg.viscous_friction
        self._rng_state = 0x2545F4914F6CDD1D  # deterministic; no Math.random/time

    # deterministic xorshift -> N(0,1)-ish; keeps runs reproducible across resume
    def _noise(self) -> float:
        x = self._rng_state & 0xFFFFFFFFFFFFFFFF
        x ^= (x << 13) & 0xFFFFFFFFFFFFFFFF
        x ^= x >> 7
        x ^= (x << 17) & 0xFFFFFFFFFFFFFFFF
        self._rng_state = x
        u = (x / 0xFFFFFFFFFFFFFFFF) * 2.0 - 1.0
        return u  # uniform[-1,1]; std ~0.577, good enough for jitter

    def friction_torque(self, v: float) -> float:
        return -self.coulomb * math.tanh(v / self.cfg.friction_v_eps) - self.viscous * v

    def hall_levels(self, angle: float):
        """Raw digital pin levels (True = HIGH = clear, False = LOW = magnet
        present = triggered), A3144 active-low semantics handled in firmware."""
        c = self.cfg
        min_present = (angle >= c.hall_min_pos + self.hall_offset[MIN]) and not self.hall_disabled[MIN]
        max_present = (angle <= c.hall_max_pos + self.hall_offset[MAX]) and not self.hall_disabled[MAX]
        return (not min_present, not max_present)   # raw HIGH unless present


class Plant:
    """Interface every plant honors. Angle is the continuous shaft angle [rad]."""
    cfg: PlantConfig
    faults: _Faults

    def reset(self) -> None: ...
    def apply_torque(self, tau: float) -> None: ...
    def step(self, dt: float) -> None: ...
    @property
    def angle(self) -> float: ...
    @property
    def velocity(self) -> float: ...
    def halls(self):
        """(min_raw_high, max_raw_high)."""
        return self.faults.hall_levels(self.angle)

    def sensor_angle(self) -> float:
        """Angle as the AS5600 would report it (with optional injected noise)."""
        n = self.cfg.angle_noise_rad
        return self.angle + (self.faults._noise() * n if n else 0.0)

    # --- fault injection ---
    def set_friction(self, *, coulomb=None, viscous=None):
        if coulomb is not None:
            self.faults.coulomb = coulomb
        if viscous is not None:
            self.faults.viscous = viscous

    def slip_hall(self, which: int, delta: float):
        self.faults.hall_offset[which] += delta

    def disable_hall(self, which: int, disabled: bool = True):
        self.faults.hall_disabled[which] = disabled


class AnalyticPlant(Plant):
    """Pure-Python 1-DOF semi-implicit Euler integrator with hard-stop clamping."""
    def __init__(self, cfg: PlantConfig | None = None):
        self.cfg = cfg or PlantConfig()
        self.faults = _Faults(self.cfg)
        self._tau = 0.0
        self.reset()

    def reset(self):
        self._angle = self.cfg.start_angle
        self._vel = 0.0
        self._tau = 0.0

    def apply_torque(self, tau: float):
        self._tau = tau

    def step(self, dt: float):
        if dt <= 0:
            return
        net = self._tau + self.faults.friction_torque(self._vel)
        a = net / self.cfg.inertia
        self._vel += a * dt
        self._angle += self._vel * dt
        # hard-stop contact: clamp position, kill velocity heading into the wall
        lo, hi = self.cfg.hard_stop_max, self.cfg.hard_stop_min
        if self._angle >= hi:
            self._angle = hi
            if self._vel > 0:
                self._vel = 0.0
        elif self._angle <= lo:
            self._angle = lo
            if self._vel < 0:
                self._vel = 0.0

    @property
    def angle(self):
        return self._angle

    @property
    def velocity(self):
        return self._vel


def make_plant(kind: str = "analytic", cfg: PlantConfig | None = None, **kw) -> Plant:
    kind = (kind or "analytic").lower()
    if kind in ("analytic", "lite", "fast"):
        return AnalyticPlant(cfg)
    raise ValueError(f"unknown plant kind: {kind!r}")
