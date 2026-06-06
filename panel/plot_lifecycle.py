#!/usr/bin/env python3
"""View a FINISHED lifecycle run from its CSVs, in the same wear-trend/heatmap window
used live. Replays summary.csv (energy + slip vs cycle) and profile.csv (the τ(pos)
heatmap) into a LifecycleWindow.

    pixi run plot                # newest run under panel/lifecycle_runs/
    pixi run plot <run_dir>      # a specific run directory (abs, or relative to runs root)
"""
import sys, os, csv, glob, json
from PyQt5 import QtWidgets
from foc_panel import LifecycleWindow
from lifecycle import RUNS_ROOT


def _fnum(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return float('nan')


def _latest_run():
    dirs = [d for d in glob.glob(os.path.join(RUNS_ROOT, '*')) if os.path.isdir(d)]
    return max(dirs, key=os.path.getmtime) if dirs else None


def _resolve(arg):
    if not arg:
        return _latest_run()
    for cand in (arg, os.path.join(RUNS_ROOT, arg), os.path.join(RUNS_ROOT, os.path.basename(arg))):
        if os.path.isdir(cand):
            return cand
    return arg


def load(win, run_dir):
    # trend plots from the per-cycle summary
    with open(os.path.join(run_dir, 'summary.csv')) as f:
        for row in csv.DictReader(f):
            win.add_status({'phase': 'run', 'cycle': int(_fnum(row['cycle'])),
                            'span': _fnum(row['span']),
                            'E_fwd': _fnum(row['E_fwd']), 'E_back': _fnum(row['E_back']),
                            'min_angle': _fnum(row['min_angle']), 'max_angle': _fnum(row['max_angle']),
                            'target': 0})
    # heatmap from the binned profile: per cycle, mean |tau| per position bin (both dirs)
    nbins = 100
    cfg = os.path.join(run_dir, 'config.json')
    if os.path.exists(cfg):
        try:
            nbins = int(json.load(open(cfg)).get('n_bins', 100))
        except Exception:
            pass
    pp = os.path.join(run_dir, 'profile.csv')
    per_cycle = {}
    if os.path.exists(pp):
        with open(pp) as f:
            for row in csv.DictReader(f):
                c = int(_fnum(row['cycle'])); b = int(_fnum(row['bin']))
                per_cycle.setdefault(c, {}).setdefault(b, []).append(abs(_fnum(row['tau'])))
    for c in sorted(per_cycle):
        bins = per_cycle[c]
        win.add_profile(c, [(sum(bins[b]) / len(bins[b])) if bins.get(b) else float('nan')
                            for b in range(nbins)])


def main():
    run_dir = _resolve(sys.argv[1] if len(sys.argv) > 1 else None)
    if not run_dir or not os.path.isdir(run_dir):
        sys.exit(f"No lifecycle run found (looked under {RUNS_ROOT}). "
                 f"Run one first, or pass a dir: pixi run plot <run_dir>")
    if not os.path.exists(os.path.join(run_dir, 'summary.csv')):
        sys.exit(f"{run_dir} has no summary.csv — not a lifecycle run dir.")
    app = QtWidgets.QApplication(sys.argv)
    win = LifecycleWindow()
    win.setWindowTitle(f"Lifecycle — {os.path.basename(run_dir)}")
    load(win, run_dir)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
