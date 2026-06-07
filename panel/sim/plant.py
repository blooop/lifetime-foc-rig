"""Plant models for the FOC-rig simulator.

The plant is the *mechanism*: a single rotary DOF (the motor shaft, 1:1 with the
firmware's `shaft_angle` in radians), with reflected carriage inertia, Coulomb +
viscous friction, two hall endstops at the travel ends, and hard stops beyond
them. The `SoftFirmware` feeds it a motor torque each control step and reads back
angle / velocity / hall levels.

Two implementations, one interface (`Plant`):
  * `GenesisPlant`  — Genesis physics (rigid-body integration + true contact at
    the hard stops). The high-fidelity plant; needs the `sim` pixi env.
  * `AnalyticPlant` — a pure-Python 1-DOF integrator with the same dynamics and
    fault hooks. No heavy deps, so the firmware/protocol tests run in the default
    `pixi run test` env, and CI stays fast.

Both share `PlantConfig` and the hall/contact geometry, so a test written against
`AnalyticPlant` exercises the same firmware code path as a Genesis run.

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
    """Pure-Python 1-DOF semi-implicit Euler integrator with hard-stop clamping.
    Same dynamics/faults as GenesisPlant; used for fast, dependency-free tests."""
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


class GenesisPlant(Plant):
    """Genesis-backed plant. Physics is a single **hinge** DOF (the motor shaft,
    1:1 with the firmware's position): its inertia is set by joint `armature`
    (== cfg.inertia), which — unlike body mass — Genesis does not floor, so the
    dynamics match the CLAUDE.md oracle. Hard stops are the joint range; Coulomb+
    viscous friction is applied as torque (runtime-injectable for wear).

    For the viewer (`show_viewer`), the scene additionally carries a **visual-only
    linear carriage** on a slide DOF plus a rail and hall/hard-stop markers. The
    carriage is teleported each step to the shaft position (it is not force-driven,
    so Genesis's body-mass floor is irrelevant) — so you watch a correct linear
    shuttle while the physics stays the validated rotary DOF. Headless runs use the
    rotor-only scene, identical to what was validated against the oracle."""
    _gs_inited = False

    def __init__(self, cfg: PlantConfig | None = None, show_viewer: bool = False,
                 timestep: float = 1e-3):
        self.cfg = cfg or PlantConfig()
        self.faults = _Faults(self.cfg)
        self._tau = 0.0
        self.timestep = timestep        # must match the control dt (1/control_hz)
        self._carriage_dof = None
        self._build(show_viewer)

    def _mjcf(self, visual: bool) -> str:
        c = self.cfg
        lo, hi = c.hard_stop_max, c.hard_stop_min   # joint range (e.g. -145 .. +145)
        # physics shaft: hinge with armature == cfg.inertia (matches the oracle).
        rotor = f"""
    <body name="rotor" pos="0 -1e4 0">    <!-- off-screen; physics only -->
      <joint name="shaft" type="hinge" axis="0 0 1" limited="true"
             range="{lo} {hi}" armature="{c.inertia}" damping="0" frictionloss="0"/>
      <geom type="box" size="0.01 0.01 0.01" mass="0.001"
            rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>"""
        if not visual:
            return f"""
<mujoco model="foc_rig">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="{self.timestep}"/>
  <worldbody>{rotor}
  </worldbody>
</mujoco>
"""
        # viewer scene: rotor (hidden) + a visual carriage slaved to it + markers.
        return f"""
<mujoco model="foc_rig">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="{self.timestep}"/>
  <worldbody>
    <!-- static rail + markers, VISUAL ONLY. +x heads toward MIN. -->
    <geom name="rail" type="box" pos="0 0 -7" size="{c.travel * 0.85:.2f} 5 1"
          rgba="0.45 0.45 0.5 1" contype="0" conaffinity="0"/>
    <geom name="min_hall" type="box" pos="{c.hall_min_pos} 0 0" size="1.5 7 7"
          rgba="0.15 0.8 0.25 1" contype="0" conaffinity="0"/>
    <geom name="max_hall" type="box" pos="{c.hall_max_pos} 0 0" size="1.5 7 7"
          rgba="0.15 0.45 0.95 1" contype="0" conaffinity="0"/>
    <geom name="hardstop_a" type="box" pos="{hi} 0 0" size="3 10 10"
          rgba="0.9 0.15 0.1 1" contype="0" conaffinity="0"/>
    <geom name="hardstop_b" type="box" pos="{lo} 0 0" size="3 10 10"
          rgba="0.9 0.15 0.1 1" contype="0" conaffinity="0"/>{rotor}
    <body name="carriage" pos="0 0 0">
      <joint name="carriage_slide" type="slide" axis="1 0 0" damping="0"/>
      <geom name="carriage" type="box" size="11 8 8" mass="1.0"
            rgba="1 0.7 0.1 1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"""

    def _viewer_options(self, gs):
        # frame the whole track (x in [-hard, +hard]) from above and to the side
        span = self.cfg.hard_stop_min
        return gs.options.ViewerOptions(
            res=(1280, 720), camera_fov=45,
            camera_pos=(0.0, -2.2 * span, 0.9 * span),
            camera_lookat=(0.0, 0.0, 0.0), camera_up=(0.0, 0.0, 1.0),
        )

    def _build(self, show_viewer):
        import tempfile
        import genesis as gs
        if not GenesisPlant._gs_inited:
            gs.init(backend=gs.cpu, logging_level="warning")
            GenesisPlant._gs_inited = True
        self._gs = gs
        with tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False) as f:
            f.write(self._mjcf(visual=show_viewer))
            path = f.name
        # Pin the scene step to our control dt with a single substep, so one
        # scene.step() advances exactly `timestep` of physics (Genesis defaults to
        # dt=0.01 with substeps, which would advance ~10x per call -> a 10x-light
        # effective inertia).
        self._scene = gs.Scene(
            show_viewer=show_viewer,
            sim_options=gs.options.SimOptions(dt=self.timestep, substeps=1),
            viewer_options=self._viewer_options(gs) if show_viewer else None,
        )
        self._ent = self._scene.add_entity(gs.morphs.MJCF(file=path))
        self._scene.build()
        self._dof = self._ent.get_joint("shaft").dof_idx_local
        if show_viewer:
            self._carriage_dof = self._ent.get_joint("carriage_slide").dof_idx_local
        # Pure force control: zero Genesis's built-in per-DOF PD controller (else
        # it holds the joint and fights control_dofs_force) and widen the applied-
        # force range so our motor torque isn't clamped.
        idx = [self._dof]
        for setter, val in (("set_dofs_kp", [0.0]), ("set_dofs_kv", [0.0])):
            fn = getattr(self._ent, setter, None)
            if fn:
                fn(val, idx)
        frng = getattr(self._ent, "set_dofs_force_range", None)
        if frng:
            frng([-1.0e3], [1.0e3], idx)
        self.reset()

    def reset(self):
        self._ent.set_dofs_position([self.cfg.start_angle], [self._dof])
        self._ent.set_dofs_velocity([0.0], [self._dof])
        if self._carriage_dof is not None:
            self._ent.set_dofs_position([self.cfg.start_angle], [self._carriage_dof])
        self._tau = 0.0

    def apply_torque(self, tau: float):
        self._tau = tau

    def step(self, dt: float):
        # friction is modeled in our torque (not Genesis damping) so wear is
        # injectable at runtime; Genesis integrates inertia + hard-stop range.
        if self._carriage_dof is not None:    # slave the visual carriage to the shaft
            self._ent.set_dofs_position([self.angle], [self._carriage_dof])
        total = self._tau + self.faults.friction_torque(self.velocity)
        self._ent.control_dofs_force([total], [self._dof])
        self._scene.step()

    @property
    def angle(self):
        return float(self._ent.get_dofs_position([self._dof])[0])

    @property
    def velocity(self):
        return float(self._ent.get_dofs_velocity([self._dof])[0])


def make_plant(kind: str = "genesis", cfg: PlantConfig | None = None, **kw) -> Plant:
    kind = (kind or "genesis").lower()
    if kind == "genesis":
        return GenesisPlant(cfg, **kw)
    if kind in ("analytic", "lite", "fast"):
        return AnalyticPlant(cfg)
    raise ValueError(f"unknown plant kind: {kind!r}")
