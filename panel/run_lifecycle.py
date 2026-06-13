#!/usr/bin/env python3
"""Lifecycle endurance runner — reuses the same SerialWorker (auto-reconnect) and
LifecycleController as the panel, so behavior and logging are identical. Logs land in
the repo under panel/lifecycle_runs/<timestamp>/ (summary.csv, profile.csv, config/state json).

Two visualizations:
  * headless (default on real hardware) — console output only, for long unattended runs.
  * live view (default against the modeled rig) — opens the full control panel (2D rig
    view + live plots) and the wear-trend window, so a sim run looks just like driving
    the real rig. `--view` forces it on hardware too; `--headless` forces it off.

Examples (from the repo root):
  pixi run lifecycle --cycles 5000 --vmeas 3.0           # hardware, headless
  pixi run lifecycle --cycles 3                          # no board -> modeled rig + live view
  pixi run lifecycle --cycles 30 --vmeas 20 --speed 4 --scenario wear
  pixi run lifecycle --view --cycles 100                 # hardware run WITH the live view
  pixi run lifecycle --resume lifecycle_runs/20260606_120000

The serial port is auto-detected (CH340 by USB VID); override with --port or the FOC_PORT
env var. With NO board detected the run targets the modeled rig (sim/, pure Python) and
says so loudly — `--sim` forces that even with a board attached. Only one program may own
the port — close the GUI panel first. 12 V must be on so the board can calibrate. Ctrl-C
stops cleanly (disables the motor, finalizes logs).
"""
import sys, os, argparse, signal
from PyQt5 import QtCore
from foc_panel import SerialWorker, resolve_backend
from lifecycle import LifecycleController, LifecycleConfig


def _build_cfg(args, sim):
    return LifecycleConfig(v_measure=args.vmeas, target_cycles=args.cycles,
                           iq_abort=args.iq_abort, slip_abort_frac=args.slip_abort / 100.0,
                           kv=args.kv, kt_override=args.kt, phase_resistance=args.r,
                           n_bins=args.bins, resume_dir=args.resume, sim=sim)


def run_headless(args, cfg):
    """Console-only run — no display needed (overnight/CI/unattended)."""
    app = QtCore.QCoreApplication(sys.argv)
    worker = SerialWorker(port=args.port)
    lc = LifecycleController(cfg)
    lc.logline.connect(lambda s: print(s, flush=True))
    lc.status.connect(lambda st: print(
        f"  cycle {st['cycle']}/{st['target']} span={st.get('span')} "
        f"E_fwd={st.get('E_fwd')} E_back={st.get('E_back')}", flush=True)
        if st.get('phase') == 'run' else None)
    lc.finished.connect(lambda reason: (print(f"FINISHED: {reason}", flush=True), worker.stop(), app.quit()))
    signal.signal(signal.SIGINT, lambda *_: lc.stop('SIGINT'))   # clean stop: disable + finalize logs
    worker.start()
    QtCore.QTimer.singleShot(1500, lambda: lc.start(worker))     # let the port open/calibrate first
    sys.exit(app.exec_())


def run_with_view(args, cfg):
    """Open the full control panel (2D rig view + live plots) and the wear-trend
    window, then auto-start the lifecycle — a sim run that looks like the real rig.
    The window stays open after the run so the wear trends can be inspected."""
    from PyQt5 import QtWidgets
    import foc_panel
    app = QtWidgets.QApplication(sys.argv)
    p = foc_panel.Panel()
    # the controller's torque model reads these panel fields via its kt/r providers
    p.kv.setValue(args.kv); p.kt_override.setValue(args.kt); p.res.setValue(args.r)
    p.autohome_chk.setChecked(False)        # the lifecycle owns homing; don't double-home on connect
    p.show()

    def _go():
        lc = p.start_lifecycle(cfg)
        lc.finished.connect(lambda reason: print(f"FINISHED: {reason}", flush=True))
    QtCore.QTimer.singleShot(1500, _go)     # let the port open/calibrate first

    # Ctrl-C -> close the window so closeEvent stops the run cleanly (disable + finalize).
    signal.signal(signal.SIGINT, lambda *_: p.close())
    _sig = QtCore.QTimer(); _sig.start(200); _sig.timeout.connect(lambda: None)
    sys.exit(app.exec_())


def main():
    ap = argparse.ArgumentParser(description='Rolling-drive lifecycle endurance test')
    ap.add_argument('--cycles', type=int, default=1000)
    ap.add_argument('--vmeas', type=float, default=3.0, help='measure/cycle speed [rad/s]')
    ap.add_argument('--iq-abort', type=float, default=5.0, help='|Iq| abort threshold [A]')
    ap.add_argument('--slip-abort', type=float, default=20.0, help='span drift abort [%%]')
    ap.add_argument('--kv', type=float, default=1000.0, help='nameplate KV [rpm/V]')
    ap.add_argument('--kt', type=float, default=0.0, help='Kt override [N·m/A]; 0 = use 9.549/KV')
    ap.add_argument('--r', type=float, default=0.15, help='phase resistance R [ohm] for the Vq torque model')
    ap.add_argument('--bins', type=int, default=100)
    ap.add_argument('--resume', default='', help='run dir to resume the cycle count from (headless only)')
    ap.add_argument('--port', default=None, help='serial port (default: auto-detect / $FOC_PORT)')
    # --- live view ---
    ap.add_argument('--view', action='store_true', help='show the live GUI (rig view + plots); default ON against the modeled rig')
    ap.add_argument('--headless', action='store_true', help='console only, no window (default on real hardware)')
    # --- modeled rig (no hardware) ---
    ap.add_argument('--sim', action='store_true', help='force the modeled rig even if a board is attached')
    ap.add_argument('--speed', type=float, default=1.0, help='sim speed factor (>1 = faster than real-time; modeled rig only)')
    ap.add_argument('--scenario', default='clean', help='fault scenario, modeled rig only (see sim/scenarios.py)')
    args = ap.parse_args()

    if args.port:
        os.environ['FOC_PORT'] = args.port    # honored by both resolve_backend and the GUI's worker
    if args.sim:
        os.environ['FOC_SIM'] = '1'
    sim, _port = resolve_backend(args.port)   # auto: board present -> real rig, none -> modeled
    if sim:
        os.environ['FOC_SIM'] = '1'           # make the worker's choice explicit/deterministic
        from sim.sim_serial import configure
        from sim.scenarios import make_scenario
        configure(speed=args.speed, on_step=make_scenario(args.scenario))
        print('## SIMULATED RIG — no board detected; running the modeled rig '
              '(plug in the rig or set FOC_PORT/FOC_SIM=0 for hardware)', flush=True)

    # Default: live view against the modeled rig (watch it run), headless on real
    # hardware (long unattended runs). Either flag overrides.
    view = args.view or (sim and not args.headless)
    if view and args.resume:
        print('# note: --resume is headless-only; ignoring the live view for this run', flush=True)
        view = False

    cfg = _build_cfg(args, sim)
    (run_with_view if view else run_headless)(args, cfg)


if __name__ == '__main__':
    main()
