#!/usr/bin/env python3
"""Launch the FOC rig against the physics simulation (Genesis plant + SoftFirmware).

The GUI/lifecycle/analysis stack is unchanged — this just sets FOC_SIM so the
SerialWorker opens a SimSerial instead of a real port, configures the plant, and
starts either the GUI panel or a headless lifecycle run.

Examples (from panel/, e.g. `pixi run -e sim gui-sim`):
  python run_sim.py                              # GUI against the Genesis plant
  python run_sim.py --viewer                     # + live Genesis carriage view
  python run_sim.py --scenario wear              # GUI with a wear fault injected
  python run_sim.py --lifecycle --cycles 30 --vmeas 20 --speed 4
  python run_sim.py --plant analytic --lifecycle --cycles 50   # no-Genesis, fast
"""
import argparse
import os
import sys


def main():
    ap = argparse.ArgumentParser(description="Run the FOC rig in simulation")
    ap.add_argument("--lifecycle", action="store_true", help="headless lifecycle run instead of the GUI")
    ap.add_argument("--plant", default="genesis", choices=["genesis", "analytic"])
    ap.add_argument("--speed", type=float, default=1.0, help="sim speed (>1 = faster than real-time)")
    ap.add_argument("--hz", type=float, default=1000.0, help="control/physics rate [Hz]")
    ap.add_argument("--viewer", action="store_true", help="show the Genesis viewer")
    ap.add_argument("--scenario", default="clean", help="fault scenario (see sim/scenarios.py)")
    # lifecycle passthrough
    ap.add_argument("--cycles", type=int, default=30)
    ap.add_argument("--vmeas", type=float, default=20.0)
    args, extra = ap.parse_known_args()

    os.environ["FOC_SIM"] = "1"
    from sim.sim_serial import configure
    from sim.scenarios import make_scenario
    configure(plant=args.plant, speed=args.speed, control_hz=args.hz,
              viewer=args.viewer, on_step=make_scenario(args.scenario))

    # Genesis (torch/taichi) can double-free during interpreter finalization at
    # exit. Work + log flushing finish before then, so bypass the buggy atexit
    # handlers with os._exit once the run/app returns. Only in the sim launcher;
    # the real-hardware run_lifecycle path keeps normal cleanup.
    code = 0
    try:
        if args.lifecycle:
            sys.argv = ["run_lifecycle", "--cycles", str(args.cycles),
                        "--vmeas", str(args.vmeas)] + extra
            import run_lifecycle
            run_lifecycle.main()
        else:
            import runpy
            runpy.run_module("foc_panel", run_name="__main__")
    except SystemExit as e:
        code = e.code if isinstance(e.code, int) else 0
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


if __name__ == "__main__":
    main()
