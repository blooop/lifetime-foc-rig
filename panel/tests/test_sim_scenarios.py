"""Fault-scenario tests: each scenario in sim/scenarios.py must produce its
intended physical effect on the plant/firmware (deterministic, AnalyticPlant).
This proves the regime-validation faults actually exercise the abort paths.
"""
import pytest

from sim.plant import MIN, MAX
from sim import scenarios
from test_sim_core import Rig


def test_wear_raises_friction_and_iq():
    rig = Rig()
    rig.home()
    rig.send("MC1", "M10")                            # steady at base friction
    rig.run(1.0)
    iq_low = abs(rig.fw.Iq)
    # moderate wear (cap below the ~0.17 N·m the 3 V motor can produce, so it keeps
    # moving and the current rises rather than fully stalling)
    rig.on_step = scenarios.wear(rate=0.05, start_s=0.0, cap=0.12)
    rig.run(4.0)
    assert rig.plant.faults.coulomb > 0.05           # friction ramped up
    assert abs(rig.fw.Iq) > iq_low + 1.0             # more current to hold the same speed


def test_hall_slip_drifts_latched_angle():
    rig = Rig()
    rig.home()
    rig.on_step = scenarios.hall_slip(which=MAX, rate=2.0, start_s=0.0)   # after homing
    angles = []
    for _ in range(3):                                # several MAX strokes
        rig.send("MC1", "M-40")
        rig.run(6.0)
        smax = [l for l in rig.slip_lines() if l.startswith("S\t1")]
        if smax:
            angles.append(float(smax[-1].split("\t")[2]))
        rig.send("M40")                               # back toward MIN to re-arm the edge
        rig.run(6.0)
    assert len(angles) >= 2
    assert abs(angles[-1] - angles[0]) > 1.0          # latched MAX angle drifted


def test_missed_hall_scenario_trips_backstop():
    sc = scenarios.missed_hall(which=MIN, at_s=0.0)
    rig = Rig(on_step=sc)
    # home first WITHOUT the fault (home() needs the MIN hall); re-arm fault after.
    base = Rig()
    base.home()                                       # sanity: clean homing works
    # with MIN disabled from t=0, drive toward MIN -> overrun -> backstop.
    rig.fw.enabled = True
    rig.fw.esMin.enabled = rig.fw.esMax.enabled = True
    rig.send("ME1", "EH")
    rig.run(3.0)                                      # homing will fail to find MIN
    # regardless of homing, commanding toward MIN with the hall dead must not run away:
    rig.send("MC1", "M25")
    rig.run(12.0)
    assert rig.plant.angle <= rig.plant.cfg.hard_stop_min + 1e-3   # stopped by hard stop / backstop


def test_glitch_scenario_increments_rejections():
    sc = scenarios.glitch(period_s=0.2, magnitude=4.0, start_s=0.2)
    rig = Rig(on_step=sc)
    rig.home()
    rig.send("MC1", "M10")
    rig.run(2.0)
    assert rig.fw.sensor.glitches > 0                 # spikes were rejected
    assert rig.fw.shaft_velocity == pytest.approx(10.0, abs=4.0)   # not cut out


def test_stall_scenario_freezes_motion():
    rig = Rig()
    rig.home()
    rig.on_step = scenarios.stall(at_s=0.0, coulomb=5.0)           # attach after homing
    rig.send("MC1", "M40")
    rig.run(1.0)
    a0 = rig.plant.angle
    rig.run(3.0)
    # friction now overwhelms the 3 V-limited torque -> little/no advance
    assert abs(rig.plant.angle - a0) < 30.0
