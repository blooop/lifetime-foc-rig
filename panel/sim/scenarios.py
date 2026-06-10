"""Fault-injection scenarios — the point of the physics sim: reproduce, safely,
the failure modes the lifecycle aborts exist to catch.

Each scenario is a factory returning an `on_step(firmware, plant, now_us, dt)`
callback that SimSerial invokes every control tick. They mutate the plant's fault
state (friction, hall offsets, hall disable) or inject sensor glitches over sim
time, so the real firmware + lifecycle code path reacts exactly as it would on a
degrading rig.

Fault -> abort mapping: hall_slip -> slip-span abort; wear -> sustained-Iq dwell;
missed_hall -> overtravel backstop (disable-on-trip); stall -> position-progress
stall detector; glitch -> FilteredAS5600 rejection (must NOT cut the motor out).
"""
from __future__ import annotations

from .plant import MIN, MAX


def _clock():
    """Returns an elapsed(now_us) fn anchored to the first call, so `start_s`/
    `at_s` mean 'seconds after the fault becomes active' regardless of when it's
    attached (e.g. after homing)."""
    t0 = [None]

    def elapsed(now_us):
        t = now_us * 1e-6
        if t0[0] is None:
            t0[0] = t
        return t - t0[0]
    return elapsed


def clean(**_):
    def on_step(fw, plant, now_us, dt):
        pass
    return on_step


def hall_slip(which=MIN, rate=0.05, start_s=0.0, **_):
    """Drift a hall's trigger angle (rad/s of sim time) — exercises S-line slip
    tracking, per-end drift, and the slip-span abort."""
    el = _clock()

    def on_step(fw, plant, now_us, dt):
        if el(now_us) >= start_s:
            plant.faults.hall_offset[which] += rate * dt
    return on_step


def wear(rate=0.02, start_s=2.0, cap=0.5, **_):
    """Ramp Coulomb friction (N·m per s) — exercises the τ(pos)/E_stroke trends
    and the sustained-Iq abort (0.5 s dwell)."""
    el = _clock()
    base = [None]

    def on_step(fw, plant, now_us, dt):
        if base[0] is None:
            base[0] = plant.faults.coulomb
        e = el(now_us)
        if e >= start_s:
            plant.faults.coulomb = min(base[0] + rate * (e - start_s), cap)
    return on_step


def missed_hall(which=MIN, at_s=8.0, **_):
    """Disable a hall after `at_s` so the next approach overruns it — exercises
    the 20% overtravel backstop (disable-on-trip) and the Genesis hard stop."""
    el = _clock()

    def on_step(fw, plant, now_us, dt):
        if el(now_us) >= at_s:
            plant.disable_hall(which, True)
    return on_step


def stall(at_s=8.0, coulomb=2.0, **_):
    """Slam friction so high the motor can't advance — exercises the
    position-progress stall detector (>6 s no advance while moving)."""
    el = _clock()

    def on_step(fw, plant, now_us, dt):
        if el(now_us) >= at_s:
            plant.faults.coulomb = coulomb
    return on_step


def glitch(period_s=0.5, magnitude=3.0, start_s=2.0, **_):
    """Periodically inject an impossible sensor jump — exercises the
    FilteredAS5600 rejection (must NOT cut the motor out)."""
    el = _clock()
    nxt = [start_s]

    def on_step(fw, plant, now_us, dt):
        e = el(now_us)
        if e >= nxt[0]:
            fw.inject_sensor_glitch(magnitude)
            nxt[0] = e + period_s
    return on_step


SCENARIOS = {
    "clean": clean,
    "hall_slip": hall_slip,
    "wear": wear,
    "missed_hall": missed_hall,
    "stall": stall,
    "glitch": glitch,
}


def make_scenario(name: str, **params):
    if not name or name == "clean":
        return None
    try:
        factory = SCENARIOS[name]
    except KeyError:
        raise SystemExit(f"unknown scenario {name!r}; choices: {', '.join(SCENARIOS)}")
    return factory(**params)
