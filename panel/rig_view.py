#!/usr/bin/env python3
"""2D rig view — a live schematic of the linear carriage rig.

One widget for both worlds: it draws whatever is on the far side of the serial
seam, purely from the firmware's E-line telemetry (position-from-home, endstop
states, homing phase, backstop). With a board attached it is a representation of
the REAL rig; with no board the SerialWorker runs the modeled rig (sim/, pure
Python) and the view carries a clearly visible SIM badge.

Geometry is *learned*, not assumed: every time an endstop reports triggered
while homed, that hall's marker snaps to the reported position — so the drawing
tracks the as-built rig (including slip) instead of a config file. Until the
first trigger it is seeded at ±100 rad (the known ~200 rad travel). Backstop
lines are drawn at hall ± 20 % of travel, the firmware's overtravel margin.

Standalone:  pixi run viewer   (embedded in the GUI panel as well)
  - real rig detected -> passive watch (board boots disabled; wave a magnet to
    check endstop wiring, or drive it from another session's firmware state)
  - no rig -> the modeled rig homes and cycles hall-to-hall so there is
    something to watch (it is a sim; nothing physical moves)
"""
import argparse
import sys

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

MIN, MAX = 0, 1
OVERTRAVEL_FRAC = 0.20          # firmware backstop margin (main.cpp)
SEED_HALL = (100.0, -100.0)     # pos-from-home seeds until learned (travel ~200 rad)
HOME_PHASE = {1: 'homing: seek MIN', 2: 'homing: seek MAX', 3: 'homing: centering'}


class RigView(QtWidgets.QWidget):
    """Rail + carriage + MIN/MAX hall markers + backstop lines, driven by the
    `endstop` signal. Call `set_sim()` once the worker has resolved the backend."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.sim = None                      # None until the worker decides
        self.hall = list(SEED_HALL)          # learned [MIN, MAX] pos-from-home
        self._learned = [False, False]
        self._prev_trig = [0, 0]             # for rising-edge learning

        self.badge = QtWidgets.QLabel('detecting rig…')
        self.badge.setStyleSheet(self._badge_css('#666'))
        self.state_lbl = QtWidgets.QLabel('—')
        self.state_lbl.setStyleSheet('font-family:monospace;')
        top = QtWidgets.QHBoxLayout()
        top.addWidget(self.badge)
        top.addWidget(self.state_lbl, 1)

        self.plot = pg.PlotWidget()
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addLayout(top)
        lay.addWidget(self.plot, 1)
        self._build_scene()

    @staticmethod
    def _badge_css(bg):
        return (f'font-weight:bold;color:white;background:{bg};'
                'border-radius:4px;padding:2px 10px;')

    def _build_scene(self):
        p = self.plot
        p.setMouseEnabled(x=False, y=False)
        p.hideAxis('left')
        p.setYRange(-40, 40)
        p.setLabel('bottom', 'position from home (rad)')
        self.rail = p.plot([0, 0], [0, 0], pen=pg.mkPen((130, 130, 140), width=8))
        self.hall_lines, self.stop_lines = [], []
        for col, name in (((40, 200, 70), 'MIN / home'), ((50, 120, 240), 'MAX')):
            ln = pg.InfiniteLine(angle=90, pen=pg.mkPen(col, width=3),
                                 label=name, labelOpts={'position': 0.9, 'color': col})
            p.addItem(ln)
            self.hall_lines.append(ln)
        for _ in range(2):                  # overtravel backstops (must never be reached)
            ln = pg.InfiniteLine(angle=90,
                                 pen=pg.mkPen((230, 50, 40), width=2, style=QtCore.Qt.DashLine),
                                 label='backstop', labelOpts={'position': 0.1, 'color': (230, 50, 40)})
            p.addItem(ln)
            self.stop_lines.append(ln)
        self.carriage = pg.ScatterPlotItem(size=46, symbol='s', pxMode=True,
                                           brush=pg.mkBrush(255, 180, 20),
                                           pen=pg.mkPen((90, 60, 0), width=2))
        self.carriage.setData([0], [0])
        p.addItem(self.carriage)
        self._apply_geometry()

    def _apply_geometry(self):
        mn, mx = self.hall
        travel = abs(mn - mx) or 1.0
        away = 1.0 if mn >= mx else -1.0     # direction past MIN, away from center
        stops = (mn + away * OVERTRAVEL_FRAC * travel,
                 mx - away * OVERTRAVEL_FRAC * travel)
        for ln, x in zip(self.hall_lines, self.hall):
            ln.setPos(x)
        for ln, x in zip(self.stop_lines, stops):
            ln.setPos(x)
        span = max(abs(stops[0]), abs(stops[1]))
        self.rail.setData([-span, span], [0, 0])
        self.plot.setXRange(-span * 1.1, span * 1.1, padding=0)

    # ---- slots ----
    def set_sim(self, sim):
        self.sim = bool(sim)
        if self.sim:
            self.badge.setText('SIMULATED RIG — modeled in software, no hardware')
            self.badge.setStyleSheet(self._badge_css('#b9770e'))
        else:
            self.badge.setText('REAL RIG')
            self.badge.setStyleSheet(self._badge_css('#27ae60'))

    def on_endstop(self, mn, mx, homed, phase, pos, backstop=0):
        # Learn hall positions only while homed and not homing — that is when the
        # pos-from-home frame is stable (homing itself redefines the zero). Latch
        # the RISING edge only: the first triggered frame is closest to the switch
        # edge; later frames sit deeper in the zone (decel + reversal overshoot).
        if homed and phase == 0:
            updated = False
            for which, trig in ((MIN, mn), (MAX, mx)):
                if trig and not self._prev_trig[which]:
                    self.hall[which] = pos
                    self._learned[which] = True
                    updated = True
            if updated:
                self._apply_geometry()
        self._prev_trig = [mn, mx]
        self.carriage.setData([pos], [0])
        if backstop:
            state = f'⛔ BACKSTOP past {"MIN" if backstop == 1 else "MAX"} — motor disabled'
        elif phase:
            state = HOME_PHASE.get(phase, 'homing…')
        else:
            trig = ' & '.join(n for n, f in (('MIN', mn), ('MAX', mx)) if f)
            state = f'{trig} endstop active' if trig else ('homed' if homed else 'not homed')
        self.state_lbl.setText(f'{state}   pos={pos:+8.1f} rad')


class RigWindow(QtWidgets.QWidget):
    """Standalone window around RigView: opens the SerialWorker (auto-detect).
    Real rig -> passive representation. Modeled rig -> home + cycle demo."""

    def __init__(self, v_cycle=60.0):
        super().__init__()
        self.setWindowTitle('FOC rig — 2D view')
        self.resize(1100, 320)
        self.v = v_cycle
        self.direction = -1            # after homing, first stroke heads toward MAX
        self.cycling = False

        self.view = RigView()
        lay = QtWidgets.QVBoxLayout(self)
        lay.addWidget(self.view)

        from foc_panel import SerialWorker
        self.worker = SerialWorker()
        self.worker.sim_mode.connect(self._on_sim_mode)
        self.worker.endstop.connect(self.view.on_endstop)
        self.worker.endstop.connect(self._on_endstop)
        self.worker.ready.connect(self._on_ready)
        self.worker.start()

        self._ka = QtCore.QTimer(self)        # 1 Hz watchdog keepalive
        self._ka.timeout.connect(lambda: self.worker.send('EK'))
        self._ka.start(1000)

    def _on_sim_mode(self, sim):
        self.view.set_sim(sim)
        self.setWindowTitle('FOC rig — 2D view (SIMULATED RIG)' if sim
                            else 'FOC rig — 2D view (real rig)')

    def _on_ready(self):
        # Only the modeled rig gets driven; a real board stays disabled (this
        # window is a representation, not a controller — use the GUI to drive it).
        if not self.worker.sim:
            return
        for cmd in ('PE1', 'MC1', f'MLV{max(self.v * 1.5, 5):.0f}', 'ME1', 'EH'):
            self.worker.send(cmd)
        self.cycling = False

    def _on_endstop(self, mn, mx, homed, phase, pos, backstop):
        if not self.worker.sim or backstop or not homed or phase:
            return
        # demo: bounce hall-to-hall once homing has settled at center
        if not self.cycling:
            self.worker.send('MC1')
            self.worker.send(f'M{self.direction * self.v:.2f}')
            self.cycling = True
        elif mn and self.direction > 0:
            self.direction = -1
            self.worker.send(f'M{self.direction * self.v:.2f}')
        elif mx and self.direction < 0:
            self.direction = 1
            self.worker.send(f'M{self.direction * self.v:.2f}')

    def closeEvent(self, e):
        try:
            if self.worker.sim:
                self.worker.send('ME0')
                self.worker.msleep(60)
            self.worker.stop()
        except Exception:
            pass
        super().closeEvent(e)


def main(argv=None):
    ap = argparse.ArgumentParser(description='2D rig view (auto-detects the real rig; '
                                             'no board -> modeled-rig demo)')
    ap.add_argument('--vcycle', type=float, default=60.0,
                    help='demo cycle speed [rad/s] (modeled rig only)')
    ap.add_argument('--speed', type=float, default=1.0,
                    help='sim speed factor (modeled rig only; >1 = faster than real-time)')
    ap.add_argument('--shot', default=None, help='save a PNG after --shot-delay and quit')
    ap.add_argument('--shot-delay', type=float, default=22.0)
    args = ap.parse_args(argv)

    from sim.sim_serial import configure
    configure(speed=args.speed)     # no-op unless the modeled rig ends up running

    app = QtWidgets.QApplication(sys.argv)
    w = RigWindow(v_cycle=args.vcycle)
    w.show()
    if args.shot:   # grab the window to a PNG once it's homed + cycling (verification)
        def _grab():
            w.grab().save(args.shot)
            print(f'saved {args.shot}', flush=True)
            app.quit()
        QtCore.QTimer.singleShot(int(args.shot_delay * 1000), _grab)
    # let Ctrl-C work
    t = QtCore.QTimer(); t.start(200); t.timeout.connect(lambda: None)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
