"""SoftFirmware — a Python port of firmware/src/main.cpp.

It is the "soft ESP32": it owns the same control loops, limits, homing, glitch
filter, watchdog and telemetry as the real firmware, talks the same Commander
M/E/P protocol, and emits the same monitor / E / S / boot lines. It reads
angle/velocity/hall levels from a `Plant`, computes the motor voltage `Vq` with
the real SimpleFOC control math, converts it to a torque via the voltage model
(`Iq=(Vq−Ke·ω)/R`, `τ=Kt·Iq`) and applies that torque to the plant.

This is what makes the whole real GUI/lifecycle/analysis stack run unchanged: the
firmware *behavior* is reproduced here, not delegated to the physics engine.

Section markers mirror main.cpp so the port can be diffed against it.
"""
from __future__ import annotations

import math

from .plant import Plant, PlantConfig

# ---- firmware constants (main.cpp) ----
WATCHDOG_MS = 3000
ENDSTOP_DEBOUNCE = 3
HOMING_TIMEOUT_MS = 15000
OVERTRAVEL_FRAC = 0.20
OVERTRAVEL_SAFE = 0.5
HOME_CENTER_TOL = 0.05
UNDERVOLTAGE_THRES = 11.1

# control types (SimpleFOC MotionControlType: 0=torque,1=velocity,2=angle)
TORQUE, VELOCITY, ANGLE = 0, 1, 2

# homing phases
HOME_IDLE, HOME_SEEK_MIN, HOME_SEEK_MAX, HOME_CENTER = 0, 1, 2, 3


def _clampf(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class PIDController:
    """Faithful port of SimpleFOC PIDController (trapezoidal I, output ramp,
    symmetric limit). dt is passed in (== loop period, as on the MCU)."""
    def __init__(self, P, I, D, ramp, limit):
        self.P, self.I, self.D = P, I, D
        self.output_ramp = ramp
        self.limit = limit
        self.error_prev = 0.0
        self.integral_prev = 0.0
        self.output_prev = 0.0

    def __call__(self, error, dt):
        if dt <= 0 or dt > 0.5:
            dt = 1e-3
        proportional = self.P * error
        integral = self.integral_prev + self.I * dt * 0.5 * (error + self.error_prev)
        integral = _clampf(integral, -self.limit, self.limit)
        derivative = self.D * (error - self.error_prev) / dt
        output = proportional + integral + derivative
        output = _clampf(output, -self.limit, self.limit)
        if self.output_ramp > 0:
            rate = (output - self.output_prev) / dt
            if rate > self.output_ramp:
                output = self.output_prev + self.output_ramp * dt
            elif rate < -self.output_ramp:
                output = self.output_prev - self.output_ramp * dt
        self.integral_prev = integral
        self.output_prev = output
        self.error_prev = error
        return output

    def reset(self):
        self.error_prev = self.integral_prev = self.output_prev = 0.0


class LowPassFilter:
    """Faithful port of SimpleFOC LowPassFilter."""
    def __init__(self, Tf):
        self.Tf = Tf
        self.y_prev = 0.0

    def __call__(self, x, dt):
        if dt < 0:
            dt = 1e-3
        elif dt > 0.3:
            self.y_prev = x
            return x
        alpha = self.Tf / (self.Tf + dt)
        y = alpha * self.y_prev + (1.0 - alpha) * x
        self.y_prev = y
        return y


class FilteredAS5600:
    """Time-aware glitch filter (main.cpp FilteredAS5600). Operates on the
    continuous sensor angle; rejects physically-impossible jumps."""
    def __init__(self):
        self.glitches = 0
        self.armed = False
        self.max_speed = 200.0
        self.floor_step = 0.10
        self._last = 0.0
        self._t_last = 0
        self._have_last = False

    def get(self, a, now_us):
        if not self._have_last:
            self._last = a
            self._t_last = now_us
            self._have_last = True
            return a
        if not self.armed:
            self._last = a
            self._t_last = now_us
            return a
        dt = (now_us - self._t_last) * 1e-6
        d = a - self._last
        if abs(d) > self.max_speed * dt + self.floor_step:
            self.glitches += 1
            return self._last        # drop; keep last/t_last so it self-heals
        self._last = a
        self._t_last = now_us
        return a


class Endstop:
    """Debounced hall endstop (main.cpp Endstop). Fed the raw pin level
    (True = HIGH = clear). A3144 active-low: LOW => magnet present => triggered."""
    def __init__(self):
        self.enabled = True
        self.active_low = True
        self.triggered = False
        self.last_read = True
        self.count = 0
        self.just_triggered = False
        self.pin = 0

    def update(self, raw_high: bool):
        r = raw_high
        if r == self.last_read:
            if self.count < ENDSTOP_DEBOUNCE:
                self.count += 1
        else:
            self.count = 0
            self.last_read = r
        prev = self.triggered
        if self.count >= ENDSTOP_DEBOUNCE:
            t = (r is False) if self.active_low else (r is True)
            self.triggered = self.enabled and t
        if not self.enabled:
            self.triggered = False
        if self.triggered and not prev:
            self.just_triggered = True


class SoftFirmware:
    def __init__(self, plant: Plant, emit, cfg: PlantConfig | None = None,
                 watchdog_ms: int = WATCHDOG_MS):
        self.plant = plant
        self.emit = emit                 # emit(str) -> queue a serial output line
        self.cfg = cfg or plant.cfg
        self._pending_glitch = 0.0
        # sim-time watchdog window. Scaled up for accelerated runs so the GUI's
        # wall-clock 1 Hz keepalive (which arrives every `speed` sim-seconds)
        # doesn't false-trip it. Survives reset().
        self.watchdog_ms = watchdog_ms
        self.reset()

    # ----------------------------- boot / reset -----------------------------
    def reset(self):
        """Mirror setup(): re-init state, re-run 'calibration', boot DISABLED."""
        c = self.cfg
        self.plant.reset()
        # motor params (setup())
        self.controller = VELOCITY
        self.target = 0.0
        self.enabled = False
        self.voltage_limit = 3.0
        self.velocity_limit = 100.0
        self.PID_velocity = PIDController(0.05, 1.0, 0.0, 1000.0, self.voltage_limit)
        self.LPF_velocity = LowPassFilter(0.02)
        self.P_angle = PIDController(10.0, 0.0, 0.0, 1e6, self.velocity_limit)
        self.monitor_downsample = 100
        self._mon_cnt = 0
        # electrical model
        self.Kt = self.Ke = 9.549 / c.kv
        self.R = c.phase_resistance
        # sensor / control state
        self.sensor = FilteredAS5600()
        self.shaft_angle = self.plant.angle
        self.shaft_velocity = 0.0
        self._prev_angle = self.shaft_angle
        self.Uq = 0.0
        self.Iq = 0.0
        # endstops
        self.esMin = Endstop()
        self.esMax = Endstop()
        # homing / position
        self.home_phase = HOME_IDLE
        self.homed = False
        self.home_offset = 0.0
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.home_t0 = 0
        self.home_speed = 20.0
        self.home_dir = +1
        self.opp_cleared = False
        self.prev_controller = VELOCITY
        # soft limits
        self.soft_enabled = False
        self.soft_min = -1e6
        self.soft_max = 1e6
        # backstop
        self.backstop_armed = False
        self.backstop_margin = 0.0
        self.backstop_fired = 0
        self.v_safe = 1e6
        # motion profile
        self.profile_enabled = True
        self.max_accel = 300.0
        self.prof_vel = 0.0
        self.prof_pos = 0.0
        self.cmd_target = 0.0
        self.last_written = 0.0
        self.prof_init = False
        self.prof_mode = VELOCITY
        # watchdog / loop timing
        self.last_cmd_ms = 0
        self._t_prev_us = 0
        self._es_t_last_ms = 0
        # boot text + clean-calibration markers (panel keys 'Motor ready')
        self.emit(f"Calibrating motor...Current voltage{c.supply_voltage:.2f}")
        self.emit("MOT: Align sensor.")
        self.emit("MOT: sensor_direction==CCW")
        self.emit("MOT: PP check: OK!")
        self.emit("MOT: Zero elec. angle: 5.00")
        self.sensor.armed = True        # arm glitch rejection after calibration
        self.emit("Motor ready (DISABLED) - connect panel (115200), then Enable.")

    def pet_watchdog(self, now_ms):
        self.last_cmd_ms = now_ms

    # ------------------------------ commands -------------------------------
    def handle(self, line: str, now_ms: int):
        if not line:
            return
        c = line[0]
        rest = line[1:]
        if c == 'M':
            self.do_motor(rest, now_ms)
        elif c == 'E':
            self.do_endstop(rest, now_ms)
        elif c == 'P':
            self.do_profile(rest, now_ms)
        # unknown letters ignored (matches Commander: no handler)

    def do_motor(self, s, now_ms):
        self.pet_watchdog(now_ms)
        if not s:
            return
        k = s[0]
        v = s[1:]
        if k == 'C':                         # motion control type
            try:
                self.controller = int(float(v))
            except ValueError:
                pass
        elif k == 'E':                       # enable/disable
            self.enabled = (self._toi(v) != 0)
            if not self.enabled:
                self.Uq = 0.0
        elif k == 'L':                       # limits
            if v[:1] == 'U':
                self.voltage_limit = self._tof(v[1:])
                self.PID_velocity.limit = self.voltage_limit
            elif v[:1] == 'V':
                self.velocity_limit = self._tof(v[1:])
                self.P_angle.limit = self.velocity_limit
            # 'C' current limit: ignored (voltage-mode only)
        elif k == 'V':                       # velocity PID / LPF
            sub, val = v[:1], self._tof(v[1:])
            if sub == 'P':
                self.PID_velocity.P = val
            elif sub == 'I':
                self.PID_velocity.I = val
            elif sub == 'D':
                self.PID_velocity.D = val
            elif sub == 'F':
                self.LPF_velocity.Tf = val
        elif k == 'A':                       # angle P
            if v[:1] == 'P':
                self.P_angle.P = self._tof(v[1:])
        elif k == 'M':                       # monitor downsample (MD)
            if v[:1] == 'D':
                self.monitor_downsample = max(1, self._toi(v[1:]))
        elif k == 'T':
            pass                             # torque control type: stays voltage
        else:                                # bare number -> target
            try:
                self.target = float(s)
            except ValueError:
                pass

    def do_profile(self, s, now_ms):
        self.pet_watchdog(now_ms)
        if not s:
            return
        if s[0] == 'A':
            self.max_accel = max(self._tof(s[1:]), 1.0)
            self.emit(f"Profile accel={self.max_accel:.1f} rad/s^2")
        elif s[0] == 'E':
            self.profile_enabled = self._toi(s[1:]) != 0
            self.emit(f"Motion profile {'ON' if self.profile_enabled else 'OFF'}")

    def do_endstop(self, s, now_ms):
        self.pet_watchdog(now_ms)
        if not s:
            return
        k = s[0]
        if k == 'K':
            return                            # keepalive: only pets the watchdog
        elif k == 'H':
            self.start_homing(now_ms)
        elif k == 'X':
            if self.homing_active():
                self.stop_homing("Homing aborted.")
        elif k == 'Z':
            self.home_offset = self.shaft_angle
            self.homed = True
            self.emit("Zero set at current position.")
        elif k == 'S':
            self.home_speed = abs(self._tof(s[1:]))
            self.emit(f"Homing seek speed={self.home_speed:.2f} rad/s")
        elif k == 'D':
            self.home_dir = -1 if self._tof(s[1:]) < 0 else 1
            self.emit(f"Seek-MIN dir={'-' if self.home_dir < 0 else '+'} velocity")
        elif k == 'A':
            self.cfg_endstop(self.esMin, s[1:], 'A')
        elif k == 'B':
            self.cfg_endstop(self.esMax, s[1:], 'B')
        elif k == 'L':
            if s[1:2] == 'E':
                self.soft_enabled = self._toi(s[2:]) != 0
                self.emit(f"Soft limits {'ON' if self.soft_enabled else 'OFF'}")
            elif s[1:2] == 'N':
                self.soft_min = self._tof(s[2:])
                self.emit(f"Soft min={self.soft_min:.3f}")
            elif s[1:2] == 'X':
                self.soft_max = self._tof(s[2:])
                self.emit(f"Soft max={self.soft_max:.3f}")

    def cfg_endstop(self, es: Endstop, s, tag):
        if not s:
            return
        if s[0] == 'E':
            es.enabled = self._toi(s[1:]) != 0
        elif s[0] == 'L':
            es.active_low = self._toi(s[1:]) != 0
        elif s[0] == 'P':
            es.pin = self._toi(s[1:])
        self.emit(f"Endstop {tag}: en={int(es.enabled)} active_low={int(es.active_low)} pin={es.pin}")

    @staticmethod
    def _tof(x):
        try:
            return float(x)
        except (ValueError, TypeError):
            return 0.0

    @staticmethod
    def _toi(x):
        try:
            return int(float(x))
        except (ValueError, TypeError):
            return 0

    # ------------------------------ homing FSM -----------------------------
    def homing_active(self):
        return self.home_phase != HOME_IDLE

    def start_homing(self, now_ms):
        if not self.enabled:
            self.emit("Home refused: enable the motor first (ME1).")
            return
        if not (self.esMin.enabled and self.esMax.enabled):
            self.emit("Home refused: enable both endstops first.")
            return
        if self.homing_active():
            return
        self.backstop_armed = False
        self.backstop_fired = 0
        self.prev_controller = self.controller
        self.controller = VELOCITY
        self.home_phase = HOME_SEEK_MIN
        self.opp_cleared = False
        self.home_t0 = now_ms
        self.emit(f"Auto-home: seeking MIN @ {self.home_speed:.1f} rad/s...")

    def stop_homing(self, msg):
        self.target = 0.0
        self.controller = self.prev_controller
        self.home_phase = HOME_IDLE
        if msg:
            self.emit(msg)

    def homing_step(self, now_ms):
        if not self.homing_active():
            return
        if not self.enabled:
            self.stop_homing("Homing aborted (motor disabled).")
            return
        if now_ms - self.home_t0 > HOMING_TIMEOUT_MS:
            self.stop_homing("Homing timeout.")
            return
        if self.home_phase == HOME_SEEK_MIN:
            if not self.esMax.triggered:
                self.opp_cleared = True
            if self.esMin.triggered:
                self.angle_min = self.shaft_angle
                self.target = 0.0
                self.home_phase = HOME_SEEK_MAX
                self.home_t0 = now_ms
                self.emit(f"  MIN @ {self.angle_min:.3f} rad; seeking MAX...")
            elif self.opp_cleared and self.esMax.triggered:
                self.stop_homing("Auto-home aborted: hit MAX while seeking MIN. Flip Seek-MIN direction.")
            else:
                self.target = self.home_dir * self.home_speed
        elif self.home_phase == HOME_SEEK_MAX:
            if self.esMax.triggered:
                self.angle_max = self.shaft_angle
                self.home_offset = 0.5 * (self.angle_min + self.angle_max)
                self.soft_min = min(self.angle_min, self.angle_max) - self.home_offset
                self.soft_max = max(self.angle_min, self.angle_max) - self.home_offset
                self.homed = True
                travel = abs(self.angle_max - self.angle_min)
                self.backstop_margin = OVERTRAVEL_FRAC * travel
                self.v_safe = math.sqrt(2.0 * self.max_accel * OVERTRAVEL_SAFE * OVERTRAVEL_FRAC * travel)
                self.backstop_fired = 0
                self.backstop_armed = True
                self.controller = ANGLE
                self.target = self.home_offset
                self.home_phase = HOME_CENTER
                self.home_t0 = now_ms
                self.emit(f"  MAX @ {self.angle_max:.3f} rad; center={self.home_offset:.3f}, "
                          f"travel={travel:.3f} rad; centering...")
                self.emit(f"  Backstop armed: margin={self.backstop_margin:.3f} rad past each "
                          f"endstop; v_safe={self.v_safe:.2f} rad/s")
            else:
                self.target = -self.home_dir * self.home_speed
        elif self.home_phase == HOME_CENTER:
            self.target = self.home_offset
            if abs(self.shaft_angle - self.home_offset) < HOME_CENTER_TOL:
                self.home_phase = HOME_IDLE
                self.emit("Auto-home complete: centered (position 0).")

    # --------------------------- travel limits -----------------------------
    def position_from_home(self):
        return self.shaft_angle - self.home_offset

    def enforce_travel_limits(self):
        minT, maxT = self.esMin.triggered, self.esMax.triggered
        angle_mode = (self.controller == ANGLE)
        d = (self.target - self.shaft_angle) if angle_mode else self.target
        into_min = (self.home_dir * d > 0.0)
        into_max = (self.home_dir * d < 0.0)
        if (minT and into_min) or (maxT and into_max):
            self.target = self.shaft_angle if angle_mode else 0.0
        if self.backstop_armed:
            past_min = (self.home_dir * (self.shaft_angle - self.angle_min) > self.backstop_margin) and into_min
            past_max = (self.home_dir * (self.shaft_angle - self.angle_max) < -self.backstop_margin) and into_max
            if past_min or past_max:
                self.backstop_fired = 1 if past_min else 2
                self.enabled = False
                self.Uq = 0.0
                self.target = self.shaft_angle if angle_mode else 0.0
                self.emit(f"!! OVERTRAVEL BACKSTOP past {'MIN' if past_min else 'MAX'} "
                          f"-> motor DISABLED (re-enable + re-home)")
            if not angle_mode:
                self.target = _clampf(self.target, -self.v_safe, self.v_safe)
        if self.homed and self.soft_enabled:
            if angle_mode:
                lo = self.home_offset + self.soft_min
                hi = self.home_offset + self.soft_max
                self.target = _clampf(self.target, lo, hi)
            else:
                pos = self.position_from_home()
                if pos <= self.soft_min and self.target < 0:
                    self.target = 0.0
                if pos >= self.soft_max and self.target > 0:
                    self.target = 0.0

    # --------------------------- motion profile ----------------------------
    def apply_motion_profile(self, dt):
        if not self.profile_enabled or dt <= 0.0:
            self.prof_init = False
            return
        if not self.prof_init or self.controller != self.prof_mode:
            self.prof_mode = self.controller
            self.prof_vel = self.shaft_velocity
            self.prof_pos = self.shaft_angle
            self.cmd_target = self.target
            self.last_written = math.nan
            self.prof_init = True
        cmd = self.target
        if math.isnan(self.last_written) or cmd != self.last_written:
            self.cmd_target = cmd
        if self.controller == ANGLE:
            vmax = self.velocity_limit
            to_go = self.cmd_target - self.prof_pos
            decel = (self.prof_vel * self.prof_vel) / (2.0 * self.max_accel)
            desired = 0.0 if abs(to_go) <= decel else (vmax if to_go > 0 else -vmax)
            maxdv = self.max_accel * dt
            self.prof_vel += _clampf(desired - self.prof_vel, -maxdv, maxdv)
            self.prof_pos += self.prof_vel * dt
            if abs(self.cmd_target - self.prof_pos) < 1e-3 and abs(self.prof_vel) < maxdv:
                self.prof_pos = self.cmd_target
                self.prof_vel = 0.0
            out = self.prof_pos
        elif self.controller == VELOCITY:
            maxdv = self.max_accel * dt
            self.prof_vel += _clampf(self.cmd_target - self.prof_vel, -maxdv, maxdv)
            out = self.prof_vel
        else:
            out = self.cmd_target
        self.target = out
        self.last_written = out

    # ------------------------------- control -------------------------------
    def _compute_uq(self, dt):
        """SimpleFOC voltage-mode move(): produce voltage.q from the active mode."""
        if self.controller == VELOCITY:
            uq = self.PID_velocity(self.target - self.shaft_velocity, dt)
        elif self.controller == ANGLE:
            vel_sp = self.P_angle(self.target - self.shaft_angle, dt)
            vel_sp = _clampf(vel_sp, -self.velocity_limit, self.velocity_limit)
            uq = self.PID_velocity(vel_sp - self.shaft_velocity, dt)
        else:  # torque-voltage: target IS the voltage command
            uq = self.target
        return _clampf(uq, -self.voltage_limit, self.voltage_limit)

    # -------------------------------- loop ---------------------------------
    def loop(self, now_us: int):
        """One control iteration (mirrors main.cpp loop()). The caller steps the
        plant after this returns, then advances the clock."""
        dt = 0.0 if self._t_prev_us == 0 else (now_us - self._t_prev_us) * 1e-6
        if dt > 0.05:
            dt = 0.05
        self._t_prev_us = now_us
        now_ms = now_us // 1000

        self.sensor.max_speed = max(2.0 * self.velocity_limit, 40.0)

        # loopFOC: read (glitch-filtered) sensor angle + derive filtered velocity
        raw = self.plant.sensor_angle() + self._pending_glitch
        self._pending_glitch = 0.0
        self.shaft_angle = self.sensor.get(raw, now_us)
        if dt > 0:
            vel_raw = (self.shaft_angle - self._prev_angle) / dt
            self.shaft_velocity = self.LPF_velocity(vel_raw, dt)
        self._prev_angle = self.shaft_angle

        # endstops (after loopFOC, before move) + slip edges
        hmin, hmax = self.plant.halls()
        self.esMin.update(hmin)
        self.esMax.update(hmax)
        if self.esMin.just_triggered:
            self.esMin.just_triggered = False
            self.emit(f"S\t0\t{self.shaft_angle:.5f}")
        if self.esMax.just_triggered:
            self.esMax.just_triggered = False
            self.emit(f"S\t1\t{self.shaft_angle:.5f}")

        self.homing_step(now_ms)
        self.enforce_travel_limits()
        self.apply_motion_profile(dt)

        # serial-heartbeat watchdog
        if self.enabled and not self.homing_active() and (now_ms - self.last_cmd_ms) > self.watchdog_ms:
            self.enabled = False
            self.Uq = 0.0
            self.emit("!! Comms watchdog: no command -> motor DISABLED")

        # move(): voltage -> current -> torque (the electromechanical model)
        self.Uq = self._compute_uq(dt)
        self.Iq = (self.Uq - self.Ke * self.shaft_velocity) / self.R
        tau = self.Kt * self.Iq if self.enabled else 0.0
        self.plant.apply_torque(tau)

        self._monitor()
        self._stream_endstops(now_ms)

    # ----------------------------- telemetry -------------------------------
    def _monitor(self):
        self._mon_cnt += 1
        if self._mon_cnt < self.monitor_downsample:
            return
        self._mon_cnt = 0
        # 7-var set: target Vq Vd Iq(mA) Id(mA) vel angle  (panel reads 0,1,3,5,6;
        # Iq/Id are in MILLIAMPS, mirroring the firmware's c.q*1000 — panel /1000s).
        iq_ma = self.Iq * 1000.0
        if self.cfg.iq_noise_a:
            iq_ma += self.plant.faults._noise() * self.cfg.iq_noise_a * 1000.0
        self.emit("\t".join(f"{x:.4f}" for x in (
            self.target, self.Uq, 0.0, iq_ma, 0.0, self.shaft_velocity, self.shaft_angle)))

    def _stream_endstops(self, now_ms):
        if now_ms - self._es_t_last_ms < 50:    # ~20 Hz
            return
        self._es_t_last_ms = now_ms
        self.emit("E\t{}\t{}\t{}\t{}\t{:.4f}\t{}".format(
            1 if self.esMin.triggered else 0,
            1 if self.esMax.triggered else 0,
            1 if self.homed else 0,
            self.home_phase,
            self.position_from_home(),
            self.backstop_fired))

    # ----------------------------- fault hooks -----------------------------
    def inject_sensor_glitch(self, magnitude_rad: float):
        """One-shot additive spike on the next sensor read (tests FilteredAS5600)."""
        self._pending_glitch = magnitude_rad
