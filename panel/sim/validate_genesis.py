"""Validate the GenesisPlant against the firmware + the CLAUDE.md oracle.

Drives SoftFirmware over a GenesisPlant with a manual 1 kHz clock (no threads):
auto-homes, checks travel/center/v_safe, then runs a velocity stroke into MAX and
confirms a hall S-line latches near the expected angle. This is the high-fidelity
counterpart to tests/test_sim_core.py (which uses the AnalyticPlant).

Run:  pixi run -e sim python -m sim.validate_genesis      (from panel/)
"""
import sys

from sim.plant import GenesisPlant, PlantConfig, MIN, MAX
from sim.soft_firmware import SoftFirmware


def main():
    cfg = PlantConfig()
    lines = []
    plant = GenesisPlant(cfg, timestep=1e-3)
    fw = SoftFirmware(plant, lines.append)
    dt, us = 1e-3, 0

    def run(seconds, keepalive=True):
        nonlocal us
        for _ in range(int(seconds / dt)):
            if keepalive and us % 1_000_000 == 0:
                fw.handle("EK", us // 1000)
            fw.loop(us)
            plant.step(dt)
            us += 1000

    print("== boot ==")
    assert any("Motor ready" in l for l in lines), "no Motor ready"

    print("== auto-home ==")
    fw.handle("ME1", 0)
    fw.handle("EH", 0)
    for _ in range(20):
        run(1.0)
        if fw.home_phase == 0 and fw.homed:
            break
    assert fw.homed and fw.home_phase == 0, "homing did not complete"
    travel = abs(fw.angle_max - fw.angle_min)
    print(f"  homed: center={fw.home_offset:.3f}  travel={travel:.2f} rad  v_safe={fw.v_safe:.2f}")
    assert abs(travel - cfg.travel) < 10, f"travel {travel} != ~{cfg.travel}"
    assert abs(fw.v_safe - 109.0) < 8, f"v_safe {fw.v_safe} != ~109"

    print("== velocity stroke into MAX ==")
    fw.handle("MC1", us // 1000)
    fw.handle("M-25", us // 1000)
    run(8.0)
    smax = [l for l in lines if l.startswith("S\t1")]
    assert smax, "MAX hall never latched an S line"
    ang = float(smax[-1].split("\t")[2])
    print(f"  MAX hall latched at shaft_angle={ang:.2f} (expected ~{cfg.hall_max_pos})")
    assert abs(ang - cfg.hall_max_pos) < 6, "MAX latch angle off"

    # peak Vq during the run (oracle: well under the 3 V limit)
    vq_peak = max(abs(float(l.split("\t")[1])) for l in lines
                  if "\t" in l and not l.startswith(("E\t", "S\t")) and len(l.split("\t")) == 7)
    print(f"  peak |Vq| = {vq_peak:.2f} V (limit 3.0)")
    assert vq_peak < 3.0

    print("RESULT: PASS — GenesisPlant reproduces homing/travel/v_safe/hall + Vq headroom")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as e:
        print("RESULT: FAIL —", e)
        sys.exit(1)
