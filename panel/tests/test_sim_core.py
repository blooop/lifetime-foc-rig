"""Deterministic tests for the SoftFirmware port + AnalyticPlant (no Genesis,
no threads, no Qt). Drives the firmware control loop with a manual clock so the
homing FSM, travel limits, backstop, watchdog and glitch filter are all exercised
exactly as main.cpp would run them.

Run: pixi run test   (or: cd panel && pytest tests/test_sim_core.py)
"""
import math

import pytest

from sim.plant import AnalyticPlant, PlantConfig, MIN, MAX
from sim.soft_firmware import SoftFirmware


class Rig:
    """Manual-clock driver: 1 ms control ticks, AnalyticPlant stepped each tick."""
    def __init__(self, cfg=None, dt=1e-3, on_step=None):
        self.lines = []
        self.plant = AnalyticPlant(cfg or PlantConfig())
        self.fw = SoftFirmware(self.plant, self.lines.append)
        self.dt = dt
        self.us = 0
        self.on_step = on_step          # scenario callback(fw, plant, now_us, dt)

    def send(self, *cmds):
        for c in cmds:
            self.fw.handle(c, self.us // 1000)

    def run(self, seconds, keepalive=True):
        n = int(seconds / self.dt)
        for i in range(n):
            # 1 Hz watchdog keepalive, like the GUI/lifecycle
            if keepalive and self.us % 1_000_000 == 0:
                self.fw.handle("EK", self.us // 1000)
            if self.on_step:
                self.on_step(self.fw, self.plant, self.us, self.dt)
            self.fw.loop(self.us)
            self.plant.step(self.dt)
            self.us += int(self.dt * 1e6)

    def home(self):
        """Enable + auto-home, run until centered (or timeout)."""
        self.send("ME1", "EH")
        for _ in range(20):
            self.run(1.0)
            if self.fw.home_phase == 0 and self.fw.homed:
                return
        raise AssertionError("homing did not complete")

    def slip_lines(self):
        return [l for l in self.lines if l.startswith("S\t")]


def test_boot_emits_motor_ready():
    rig = Rig()
    assert any("Motor ready" in l for l in rig.lines)
    assert any("PP check: OK!" in l for l in rig.lines)
    assert rig.fw.enabled is False          # boots DISABLED


def test_monitor_is_7_var_with_iq_in_ma():
    rig = Rig()
    rig.send("ME1", "MC1", "M5")
    rig.run(0.5)
    telem = [l for l in rig.lines if "\t" in l and not l.startswith(("E\t", "S\t"))]
    assert telem, "no 7-var monitor lines emitted"
    parts = telem[-1].split("\t")
    assert len(parts) == 7                  # target Vq Vd Iq Id vel angle
    # Iq field is in mA: |Iq[A]| modeled small at 5 rad/s -> |mA| should be > the amps value
    iq_ma = float(parts[3])
    assert abs(iq_ma) < 50_000              # sane bound (well under the old 25 A*1000 garbage)


def test_homing_completes_and_measures_travel():
    cfg = PlantConfig()
    rig = Rig(cfg)
    rig.home()
    assert rig.fw.homed
    assert rig.fw.home_phase == 0
    # center is midpoint of the two halls; travel ~ |hall_min - hall_max|
    assert rig.fw.home_offset == pytest.approx(0.5 * (cfg.hall_min_pos + cfg.hall_max_pos), abs=2.0)
    travel = abs(rig.fw.angle_max - rig.fw.angle_min)
    assert travel == pytest.approx(cfg.travel, abs=8.0)
    assert rig.fw.backstop_armed


def test_v_safe_matches_overtravel_formula():
    cfg = PlantConfig()
    rig = Rig(cfg)
    rig.home()
    travel = abs(rig.fw.angle_max - rig.fw.angle_min)
    expect = math.sqrt(2.0 * rig.fw.max_accel * 0.5 * 0.20 * travel)
    assert rig.fw.v_safe == pytest.approx(expect, rel=1e-6)
    # oracle: ~109 rad/s at accel 300, travel ~200, 20% margin
    assert rig.fw.v_safe == pytest.approx(109.0, abs=6.0)


def test_home_refused_without_enable():
    rig = Rig()
    rig.send("EH")                          # not enabled
    rig.run(0.2)
    assert rig.fw.home_phase == 0 and not rig.fw.homed
    assert any("Home refused" in l for l in rig.lines)


def test_velocity_cycle_triggers_hall_and_emits_slip():
    rig = Rig()
    rig.home()
    rig.send("MC1", "M-25")                 # toward MAX (negative velocity)
    rig.run(8.0)
    maxslip = [l for l in rig.slip_lines() if l.startswith("S\t1")]
    assert maxslip, "MAX hall edge never latched an S line"
    which, angle = maxslip[-1].split("\t")[1:]
    assert float(angle) == pytest.approx(rig.plant.cfg.hall_max_pos, abs=5.0)


def test_hard_endstop_clamps_target():
    rig = Rig()
    rig.home()
    rig.send("MC1", "M-25")                 # drive into MAX
    rig.run(10.0)
    # sitting on MAX, commanded further into it -> firmware holds at the limit
    assert rig.fw.esMax.triggered
    assert rig.plant.angle >= rig.plant.cfg.hall_max_pos - 2.0   # didn't blow past the hall


def test_watchdog_disables_when_silent():
    rig = Rig()
    rig.send("ME1", "MC1", "M5")
    rig.run(1.0)
    assert rig.fw.enabled
    rig.run(4.0, keepalive=False)           # go silent > WATCHDOG_MS (3 s)
    assert not rig.fw.enabled
    assert any("watchdog" in l.lower() for l in rig.lines)


def test_missed_hall_trips_backstop():
    rig = Rig()
    rig.home()
    rig.plant.disable_hall(MIN, True)       # MIN hall fails
    rig.send("MC1", "M25")                  # drive toward MIN past where it should stop
    rig.run(10.0)
    assert rig.fw.backstop_fired == 1       # tripped past MIN
    assert not rig.fw.enabled               # disable-on-trip


def test_glitch_filter_rejects_spike_without_runaway():
    rig = Rig()
    rig.home()
    rig.send("MC1", "M10")
    rig.run(2.0)
    before = rig.fw.sensor.glitches
    rig.fw.inject_sensor_glitch(5.0)        # impossible jump
    rig.run(0.5)
    assert rig.fw.sensor.glitches > before  # spike was rejected
    # motor still tracking (not cut out): velocity stayed near command
    assert rig.fw.shaft_velocity == pytest.approx(10.0, abs=4.0)
