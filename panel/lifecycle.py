#!/usr/bin/env python3
"""Lifecycle / endurance characterization of the linear rolling-contact drive.

Panel-driven, one speed, measure-every-cycle. The firmware owns real-time safety
(reverse-at-hall, the 5% overtravel backstop, the comms watchdog) and emits a precise
`S` line latched at each hall edge; THIS controller owns the cycle counter, sequencing,
metrics, and all logging (everything stays inside the repo under ./lifecycle_runs/).

Per cycle (one there-and-back between the two halls) we record:
  - slip: shaft-angle at each hall edge -> span (motor-rotation-per-stroke) + per-end drift
  - torque-vs-position: tau = Kt*Iq binned along the stroke, both directions
  - energy: E_stroke = integral(tau d-theta) for the forward and back strokes

Aborts: target cycle count (clean), torque/Iq anomaly, slip-span anomaly, backstop-fired
or hall timeout. Any abort disables the motor (ME0), flags the reason, finalizes logs.

GUI-agnostic (a plain QObject) so it runs equally from foc_panel.py or run_lifecycle.py.
It talks to a SerialWorker via worker.send(cmd) and its telem/slip/endstop/ready signals.
"""
import os, json, csv, math, time, subprocess, platform
from dataclasses import dataclass, field, asdict
from datetime import datetime
from PyQt5 import QtCore

RUNS_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lifecycle_runs')


@dataclass
class LifecycleConfig:
    v_measure: float = 3.0            # rad/s, constant sweep/cycle speed
    target_cycles: int = 1000         # stop cleanly after this many full cycles
    n_bins: int = 100                 # position bins for the tau(pos) profile
    iq_abort: float = 5.0             # |Iq| [A] anomaly -> abort (seizure/jam)
    slip_abort_frac: float = 0.20     # |span-span0|/span0 beyond this -> abort (gross slip / lost hall)
    stall_timeout_s: float = 6.0      # position stops progressing this long while moving -> abort
                                      # (travel-agnostic; catches a jam/dead hall, not just a slow stroke)
    prog_eps: float = 0.5             # rad of position change that counts as "still moving"
    heartbeat_s: float = 1.0          # keepalive cadence (must be < firmware WATCHDOG_MS)
    kv: float = 1000.0                # nameplate KV [rpm/V] -> Kt = Ke = 9.549/KV
    kt_override: float = 0.0          # explicit Kt [N·m/A]; 0 = use 9.549/KV
    phase_resistance: float = 0.15    # ohm; for the Vq torque model Iq=(Vq-Ke·ω)/R
    out_root: str = RUNS_ROOT
    resume_dir: str = ''              # if set, resume cycle count from this run dir's state.json


class LifecycleController(QtCore.QObject):
    status = QtCore.pyqtSignal(dict)      # live status for the UI
    finished = QtCore.pyqtSignal(str)     # reason string ('target reached', 'abort: ...')
    logline = QtCore.pyqtSignal(str)      # human-readable progress for the log pane
    cycle_profile = QtCore.pyqtSignal(int, list)  # cycle#, per-bin mean tau (len n_bins, NaN if empty)

    def __init__(self, cfg: LifecycleConfig, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self.worker = None
        self.kt_provider = None       # optional callable -> live Kt [N·m/A] (panel supplies its kt())
        self.r_provider = None        # optional callable -> live phase resistance R [ohm]
        self.running = False
        self.phase = 'idle'               # idle | homing | run | parking
        self._home_started = False
        self._park_center = 0.0           # absolute shaft-angle of travel center (park target)
        self._park_t0 = 0.0
        self._park_tol = 3.0
        self._park_reason = ''
        self.cycle = 0
        self.direction = 1                # sign of commanded velocity
        self.start_end = None             # which hall (0/1) defines the cycle boundary
        self.span0 = None
        self.E0 = None
        self._samples = []                # [(shaft_angle, tau)] for the current stroke
        self._pending = []                # finalized strokes since the last completed cycle
        self.cur_min_angle = None
        self.cur_max_angle = None
        self._prog_pos = None         # last position-from-home where we saw real progress
        self._prog_t = 0.0
        self._iq_over_t = None        # time the model Iq first exceeded the abort threshold
        self._t0 = 0.0
        self._inhibitor = None
        self._summary_f = self._summary_w = None
        self._profile_f = self._profile_w = None
        self.run_dir = ''
        self._hb = QtCore.QTimer(self)
        self._hb.timeout.connect(self._heartbeat)

    # ---------------- lifecycle ----------------
    def start(self, worker):
        if self.running:
            return
        self.worker = worker
        self._t0 = time.monotonic()
        if self.cfg.resume_dir:
            self._resume_state(self.cfg.resume_dir)
        self._open_logs()
        self._inhibit_sleep()
        worker.telem.connect(self._on_telem)
        worker.slip.connect(self._on_slip)
        worker.endstop.connect(self._on_endstop)
        worker.ready.connect(self._on_ready)
        self.running = True
        self._begin_homing()
        self._hb.start(int(self.cfg.heartbeat_s * 1000))
        self._emit_status()

    def stop(self, reason='stopped by user'):
        if not self.running:
            return
        self._finish(reason)

    # ---------------- board sequencing ----------------
    def _begin_homing(self):
        self.phase = 'homing'
        self._home_started = False
        w = self.worker
        w.send('PE1')                                   # ensure motion profiling on
        w.send('MC1')                                   # velocity mode
        w.send(f'MLV{max(self.cfg.v_measure * 1.5, 5):.1f}')
        w.send('ME1')                                   # enable (required to home)
        w.send('EH')                                    # auto-home MIN->MAX->center (arms backstop)
        self.logline.emit('# homing (MIN -> MAX -> center); backstop will arm')

    def _begin_run(self):
        self.phase = 'run'
        self.start_end = None
        self._samples = []
        self._pending = []
        self.cur_min_angle = self.cur_max_angle = None
        self._prog_pos = None
        self._prog_t = time.monotonic()
        self._iq_over_t = None
        self.direction = 1
        self.worker.send('MC1')   # homing leaves the motor in ANGLE mode -> back to velocity
        self.worker.send(f'M{self.direction * self.cfg.v_measure:.3f}')
        self.logline.emit(f'# cycling @ {self.cfg.v_measure:.2f} rad/s from cycle {self.cycle}')

    # ---------------- telemetry slots ----------------
    def _on_ready(self):
        # Board reset (e.g. a serial reconnect auto-resets it) -> lost homing. Re-home and
        # resume; the cycle counter lives here so it survives the reset.
        if self.running:
            self.logline.emit('# board reset detected -> re-homing and resuming')
            self._begin_homing()

    def _on_endstop(self, mn, mx, homed, phase_int, pos, backstop=0):
        if not self.running:
            return
        if backstop:
            self._finish(f'abort: overtravel backstop past {"MIN" if backstop == 1 else "MAX"}')
            return
        if self.phase == 'homing':
            if phase_int != 0:
                self._home_started = True
            elif self._home_started and homed:
                self._begin_run()
        elif self.phase == 'run':
            # progress watchdog: note when the carriage actually advances (travel-agnostic)
            if self._prog_pos is None or abs(pos - self._prog_pos) > self.cfg.prog_eps:
                self._prog_pos = pos
                self._prog_t = time.monotonic()
        elif self.phase == 'parking':
            if abs(pos) < self._park_tol:           # near center, well clear of the limits
                self._finish(self._park_reason)

    def _on_telem(self, t, vq, iq, v, a):
        # Model-based torque from the (clean) voltage signal — the board's measured Iq
        # (arg `iq`, monitor index 3) is currently unusable (ADC bring-up TODO), so we
        # estimate: Iq=(Vq-Ke·ω)/R, τ=Kt·Iq, with Kt=Ke=9.549/KV.
        if not self.running or self.phase != 'run':
            return
        kt = self._kt()
        iq_model = (vq - kt * v) / self._r()
        # Sustained-breach abort: the model Iq spikes briefly on every accel/reversal
        # (high Vq while ω lags) — that's normal, not a fault. Only abort if it stays
        # over the threshold for IQ_DWELL_S continuously (a real jam/seizure persists).
        now = time.monotonic()
        if abs(iq_model) > self.cfg.iq_abort:
            if self._iq_over_t is None:
                self._iq_over_t = now
            elif now - self._iq_over_t > self.IQ_DWELL_S:
                self._finish(f'abort: sustained torque/current anomaly ({iq_model:+.2f} A > {self.cfg.iq_abort} A)')
                return
        else:
            self._iq_over_t = None
        self._samples.append((a, kt * iq_model))

    IQ_DWELL_S = 0.5      # s the model Iq must stay over threshold before it counts as a fault

    def _kt(self):
        if self.kt_provider is not None:
            return self.kt_provider()
        return self.cfg.kt_override if self.cfg.kt_override > 0 else 9.549 / max(self.cfg.kv, 1.0)

    def _r(self):
        r = self.r_provider() if self.r_provider is not None else self.cfg.phase_resistance
        return max(r, 1e-3)

    def _on_slip(self, which, angle):
        # A precise hall arrival (firmware latched shaft_angle at the debounced edge).
        if not self.running or self.phase != 'run':
            return
        if which == 0:
            self.cur_min_angle = angle
        else:
            self.cur_max_angle = angle
        self._last_arrival_t = time.monotonic()

        if self.start_end is None:
            # First contact defines the cycle boundary; discard the partial center->end stroke.
            self.start_end = which
        else:
            stroke = self._finalize_stroke(which)
            if stroke:
                self._pending.append(stroke)
            if which == self.start_end:
                self._complete_cycle()

        self.direction = -self.direction
        self._samples = []
        self.worker.send(f'M{self.direction * self.cfg.v_measure:.3f}')

    # ---------------- metrics ----------------
    def _finalize_stroke(self, arrived_at):
        s = self._samples
        if len(s) < 2:
            return None
        E = 0.0
        peak = 0.0
        abs_sum = 0.0
        for i in range(1, len(s)):
            dtheta = s[i][0] - s[i - 1][0]
            tau = 0.5 * (s[i][1] + s[i - 1][1])
            E += tau * dtheta
            peak = max(peak, abs(s[i][1]))
            abs_sum += abs(s[i][1])
        return dict(dir='to_min' if arrived_at == 0 else 'to_max',
                    E=E, peak=peak, mean=abs_sum / (len(s) - 1),
                    samples=list(s))

    def _complete_cycle(self):
        self.cycle += 1
        strokes = self._pending
        self._pending = []
        E_fwd = strokes[0]['E'] if len(strokes) >= 1 else float('nan')
        E_back = strokes[1]['E'] if len(strokes) >= 2 else float('nan')
        peak = max((st['peak'] for st in strokes), default=float('nan'))
        mean = (sum(st['mean'] for st in strokes) / len(strokes)) if strokes else float('nan')
        span = (abs(self.cur_max_angle - self.cur_min_angle)
                if self.cur_min_angle is not None and self.cur_max_angle is not None else float('nan'))
        if self.span0 is None and not math.isnan(span):
            self.span0 = span
            self.E0 = (E_fwd if not math.isnan(E_fwd) else 0) + (E_back if not math.isnan(E_back) else 0)
            self.logline.emit(f'# baseline: span0={self.span0:.4f} rad, E0={self.E0:.4g}')

        t_s = time.monotonic() - self._t0
        self._summary_w.writerow([self.cycle, f'{t_s:.2f}',
                                  _f(self.cur_min_angle), _f(self.cur_max_angle), _f(span),
                                  f'{E_fwd:.6g}', f'{E_back:.6g}', f'{peak:.6g}', f'{mean:.6g}'])
        self._summary_f.flush()
        self._write_profile(strokes)
        self._save_state()
        self._emit_status(span=span, E_fwd=E_fwd, E_back=E_back)

        # slip-anomaly abort
        if self.span0 and not math.isnan(span) and abs(span - self.span0) > self.cfg.slip_abort_frac * self.span0:
            self._finish(f'abort: slip/span anomaly (span {span:.3f} vs span0 {self.span0:.3f})')
            return
        # clean completion -> park at center first (a cycle ends AT a hall limit, and
        # leaving the carriage on the MIN switch breaks boot — GPIO5 is a strapping pin).
        if self.cycle >= self.cfg.target_cycles:
            self._begin_park(f'target reached ({self.cfg.target_cycles} cycles)')

    PARK_TIMEOUT = 40.0   # s; give up parking and disable anyway

    def _begin_park(self, reason):
        if self.cur_min_angle is None or self.cur_max_angle is None:
            self._finish(reason); return        # no center reference -> just stop
        self.phase = 'parking'
        self._park_reason = reason
        span = abs(self.cur_max_angle - self.cur_min_angle)
        # "Centered enough" tolerance: the firmware home-offset and our cycling-latched
        # midpoint differ by a few rad, and the goal is only to be well clear of the
        # limits — so accept a generous band around center (still ~halfway from a limit).
        self._park_tol = max(3.0, 0.05 * span)
        self._park_center = 0.5 * (self.cur_min_angle + self.cur_max_angle)
        self._park_t0 = time.monotonic()
        self.worker.send('MC2')                 # angle mode
        self.worker.send(f'M{self._park_center:.4f}')
        self.logline.emit('# parking at center (clear of the limits) before disabling…')

    def _write_profile(self, strokes):
        if self.cur_min_angle is None or self.cur_max_angle is None:
            return
        a_lo = min(self.cur_min_angle, self.cur_max_angle)
        a_hi = max(self.cur_min_angle, self.cur_max_angle)
        home_off = 0.5 * (a_lo + a_hi)
        span = max(a_hi - a_lo, 1e-6)
        nb = self.cfg.n_bins
        comb_sum = [0.0] * nb       # combined over both directions, for the heatmap row
        comb_cnt = [0] * nb
        for st in strokes:
            sums = [0.0] * nb
            cnts = [0] * nb
            for ang, tau in st['samples']:
                b = int((ang - a_lo) / span * nb)
                b = 0 if b < 0 else (nb - 1 if b >= nb else b)
                sums[b] += tau; cnts[b] += 1
                comb_sum[b] += abs(tau); comb_cnt[b] += 1
            for b in range(nb):
                if cnts[b]:
                    pos = (a_lo + (b + 0.5) / nb * span) - home_off
                    self._profile_w.writerow([self.cycle, st['dir'], b, f'{pos:.4f}', f'{sums[b]/cnts[b]:.6g}'])
        self._profile_f.flush()
        row = [(comb_sum[b] / comb_cnt[b]) if comb_cnt[b] else float('nan') for b in range(nb)]
        self.cycle_profile.emit(self.cycle, row)

    # ---------------- heartbeat / timeout ----------------
    def _heartbeat(self):
        if not self.running:
            return
        if self.phase == 'run':
            now = time.monotonic()
            if self._prog_pos is not None and now - self._prog_t > self.cfg.stall_timeout_s:
                self._finish('abort: carriage stalled (no position progress — jam or dead hall)')
                return
            self.worker.send(f'M{self.direction * self.cfg.v_measure:.3f}')   # re-issue + pet watchdog
        elif self.phase == 'parking':
            if time.monotonic() - self._park_t0 > self.PARK_TIMEOUT:
                self._finish(self._park_reason + ' [park timed out]')
                return
            self.worker.send(f'M{self._park_center:.4f}')                      # hold center + pet
        else:
            self.worker.send('EK')                                            # keepalive while homing

    # ---------------- logging / state / sleep ----------------
    def _open_logs(self):
        if self.cfg.resume_dir and os.path.isdir(self.cfg.resume_dir):
            self.run_dir = self.cfg.resume_dir
        else:
            os.makedirs(self.cfg.out_root, exist_ok=True)
            self.run_dir = os.path.join(self.cfg.out_root, datetime.now().strftime('%Y%m%d_%H%M%S'))
            os.makedirs(self.run_dir, exist_ok=True)
        new_summary = not os.path.exists(os.path.join(self.run_dir, 'summary.csv'))
        self._summary_f = open(os.path.join(self.run_dir, 'summary.csv'), 'a', newline='')
        self._summary_w = csv.writer(self._summary_f)
        if new_summary:
            self._summary_w.writerow(['cycle', 't_s', 'min_angle', 'max_angle', 'span',
                                      'E_fwd', 'E_back', 'peak_tau', 'mean_tau'])
        new_profile = not os.path.exists(os.path.join(self.run_dir, 'profile.csv'))
        self._profile_f = open(os.path.join(self.run_dir, 'profile.csv'), 'a', newline='')
        self._profile_w = csv.writer(self._profile_f)
        if new_profile:
            self._profile_w.writerow(['cycle', 'dir', 'bin', 'pos', 'tau'])
        self._summary_f.flush(); self._profile_f.flush()
        with open(os.path.join(self.run_dir, 'config.json'), 'w') as f:
            json.dump(asdict(self.cfg), f, indent=2)
        self.logline.emit(f'# logging to {self.run_dir}')

    def _save_state(self):
        try:
            with open(os.path.join(self.run_dir, 'state.json'), 'w') as f:
                json.dump(dict(cycle=self.cycle, span0=self.span0, E0=self.E0), f)
        except Exception:
            pass

    def _resume_state(self, d):
        try:
            with open(os.path.join(d, 'state.json')) as f:
                st = json.load(f)
            self.cycle = int(st.get('cycle', 0))
            self.span0 = st.get('span0')
            self.E0 = st.get('E0')
            self.logline.emit(f'# resumed at cycle {self.cycle} from {d}')
        except Exception as e:
            self.logline.emit(f'# resume failed ({e}); starting fresh')

    def _inhibit_sleep(self):
        # Best-effort: keep the machine awake for the run (caffeinate on macOS,
        # systemd-inhibit on Linux). Harmless no-op if the tool is missing.
        if platform.system() == 'Darwin':
            cmd = ['caffeinate', '-i', '-s']          # prevent idle + system sleep
        else:
            cmd = ['systemd-inhibit', '--what=idle:sleep:shutdown', '--who=foc-lifecycle',
                   '--why=lifecycle endurance test', 'sleep', 'infinity']
        try:
            self._inhibitor = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            self._inhibitor = None

    def _finish(self, reason):
        self._hb.stop()
        try:
            if self.worker:
                self.worker.send('EX'); self.worker.send('M0'); self.worker.send('ME0')
        except Exception:
            pass
        self.running = False
        self.phase = 'idle'
        self._save_state()
        for f in (self._summary_f, self._profile_f):
            try:
                if f: f.flush(); f.close()
            except Exception:
                pass
        if self._inhibitor:
            try: self._inhibitor.terminate()
            except Exception: pass
            self._inhibitor = None
        if self.worker:
            for sig, slot in ((self.worker.telem, self._on_telem), (self.worker.slip, self._on_slip),
                              (self.worker.endstop, self._on_endstop), (self.worker.ready, self._on_ready)):
                try: sig.disconnect(slot)
                except Exception: pass
        self.logline.emit(f'# lifecycle finished: {reason} (cycle {self.cycle})')
        self.finished.emit(reason)

    def _emit_status(self, span=float('nan'), E_fwd=float('nan'), E_back=float('nan')):
        self.status.emit(dict(running=self.running, phase=self.phase, cycle=self.cycle,
                              target=self.cfg.target_cycles, span=span, span0=self.span0,
                              min_angle=self.cur_min_angle, max_angle=self.cur_max_angle,
                              E_fwd=E_fwd, E_back=E_back, run_dir=self.run_dir))


def _f(x):
    return '' if x is None else f'{x:.5f}'
