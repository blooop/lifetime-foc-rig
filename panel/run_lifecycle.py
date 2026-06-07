#!/usr/bin/env python3
"""Headless lifecycle endurance runner — no GUI, for long unattended/overnight runs.

Reuses the same SerialWorker (auto-reconnect) and LifecycleController as the panel, so
behavior and logging are identical; only the visualization is dropped. Logs land in the
repo under panel/lifecycle_runs/<timestamp>/ (summary.csv, profile.csv, config/state json).

Examples (from the repo root):
  pixi run lifecycle --cycles 5000 --vmeas 3.0
  pixi run lifecycle --resume lifecycle_runs/20260606_120000

The serial port is auto-detected (CH340 by USB VID); override with --port or the FOC_PORT
env var. Only one program may own the port — close the GUI panel first. 12 V must be on so
the board can calibrate. Ctrl-C stops cleanly (disables the motor, finalizes logs).
"""
import sys, os, argparse, signal
from PyQt5 import QtCore
from foc_panel import SerialWorker
from lifecycle import LifecycleController, LifecycleConfig


def main():
    ap = argparse.ArgumentParser(description='Headless rolling-drive lifecycle test')
    ap.add_argument('--cycles', type=int, default=1000)
    ap.add_argument('--vmeas', type=float, default=3.0, help='measure/cycle speed [rad/s]')
    ap.add_argument('--iq-abort', type=float, default=5.0, help='|Iq| abort threshold [A]')
    ap.add_argument('--slip-abort', type=float, default=20.0, help='span drift abort [%%]')
    ap.add_argument('--kv', type=float, default=1000.0, help='nameplate KV [rpm/V]')
    ap.add_argument('--kt', type=float, default=0.0, help='Kt override [N·m/A]; 0 = use 9.549/KV')
    ap.add_argument('--r', type=float, default=0.15, help='phase resistance R [ohm] for the Vq torque model')
    ap.add_argument('--bins', type=int, default=100)
    ap.add_argument('--resume', default='', help='run dir to resume the cycle count from')
    ap.add_argument('--port', default=None, help='serial port (default: auto-detect / $FOC_PORT)')
    # --- physics simulation (no hardware) ---
    ap.add_argument('--sim', action='store_true', help='run against the physics sim (Genesis plant)')
    ap.add_argument('--plant', default='genesis', choices=['genesis', 'analytic'])
    ap.add_argument('--speed', type=float, default=1.0, help='sim speed factor (>1 = faster than real-time)')
    ap.add_argument('--scenario', default='clean', help='fault scenario (see sim/scenarios.py)')
    args = ap.parse_args()

    if args.sim:
        os.environ['FOC_SIM'] = '1'
        from sim.sim_serial import configure
        from sim.scenarios import make_scenario
        configure(plant=args.plant, speed=args.speed, on_step=make_scenario(args.scenario))

    app = QtCore.QCoreApplication(sys.argv)
    cfg = LifecycleConfig(v_measure=args.vmeas, target_cycles=args.cycles,
                          iq_abort=args.iq_abort, slip_abort_frac=args.slip_abort / 100.0,
                          kv=args.kv, kt_override=args.kt, phase_resistance=args.r,
                          n_bins=args.bins, resume_dir=args.resume)
    worker = SerialWorker(port=args.port)
    lc = LifecycleController(cfg)
    lc.logline.connect(lambda s: print(s, flush=True))
    lc.status.connect(lambda st: print(
        f"  cycle {st['cycle']}/{st['target']} span={st.get('span')} "
        f"E_fwd={st.get('E_fwd')} E_back={st.get('E_back')}", flush=True)
        if st.get('phase') == 'run' else None)
    lc.finished.connect(lambda reason: (print(f"FINISHED: {reason}", flush=True), worker.stop(), app.quit()))

    # Ctrl-C -> clean stop (disable motor, finalize logs).
    signal.signal(signal.SIGINT, lambda *_: lc.stop('SIGINT'))

    worker.start()
    QtCore.QTimer.singleShot(1500, lambda: lc.start(worker))   # let the port open/calibrate first
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
