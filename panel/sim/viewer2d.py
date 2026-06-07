"""2D top-down carriage view (pyqtgraph) — watch the simulated shuttle home and
cycle between the endstops in real time.

Reuses the real stack: a SerialWorker talks to the SimSerial (FOC_SIM), so the
firmware homes/cycles exactly as it would; this window just draws the carriage at
the reported position-from-home, with green MIN / blue MAX hall markers and red
hard-stop lines. No 3D camera, no spin, no Genesis viewer throttle.

Run:  pixi run -e sim viewer            (or: python run_sim.py --viewer)
"""
import os
import sys

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

from sim.plant import PlantConfig


class Carriage2D(QtWidgets.QWidget):
    def __init__(self, v_cycle=60.0, plant="analytic", speed=1.0):
        super().__init__()
        self.cfg = PlantConfig()
        self.v = v_cycle
        self.direction = -1          # after homing, first stroke heads toward MAX
        self.cycling = False
        self.setWindowTitle("FOC rig — carriage (sim)")
        self.resize(1100, 320)

        self.banner = QtWidgets.QLabel("connecting…")
        self.banner.setStyleSheet("font-size:15px; padding:4px;")
        self.plot = pg.PlotWidget()
        lay = QtWidgets.QVBoxLayout(self)
        lay.addWidget(self.banner)
        lay.addWidget(self.plot, 1)

        self._build_scene()

        # ---- sim-backed worker ----
        os.environ["FOC_SIM"] = "1"
        from sim.sim_serial import configure
        configure(plant=plant, speed=speed)
        from foc_panel import SerialWorker
        self.worker = SerialWorker()
        self.worker.ready.connect(self._on_ready)
        self.worker.endstop.connect(self._on_endstop)
        self.worker.start()

        self._ka = QtCore.QTimer(self)      # 1 Hz watchdog keepalive
        self._ka.timeout.connect(lambda: self.worker.send("EK"))
        self._ka.start(1000)

    def _build_scene(self):
        c = self.cfg
        p = self.plot
        p.setMouseEnabled(x=False, y=False)
        p.hideAxis("left")
        p.setXRange(-c.hard_stop_min * 1.1, c.hard_stop_min * 1.1)
        p.setYRange(-40, 40)
        p.setLabel("bottom", "position from home (rad)")
        # rail
        p.plot([-c.hard_stop_min, c.hard_stop_min], [0, 0],
               pen=pg.mkPen((130, 130, 140), width=8))
        # hall markers (working limits): MIN at +, MAX at -
        for x, col, name in ((c.hall_min_pos, (40, 200, 70), "MIN / home"),
                             (c.hall_max_pos, (50, 120, 240), "MAX")):
            p.addItem(pg.InfiniteLine(pos=x, angle=90,
                      pen=pg.mkPen(col, width=3),
                      label=name, labelOpts={"position": 0.9, "color": col}))
        # hard stops (must never be reached)
        for x in (c.hard_stop_min, c.hard_stop_max):
            p.addItem(pg.InfiniteLine(pos=x, angle=90,
                      pen=pg.mkPen((230, 50, 40), width=2, style=QtCore.Qt.DashLine),
                      label="hard stop", labelOpts={"position": 0.1, "color": (230, 50, 40)}))
        # the carriage
        self.carriage = pg.ScatterPlotItem(size=46, symbol="s", pxMode=True,
                                            brush=pg.mkBrush(255, 180, 20),
                                            pen=pg.mkPen((90, 60, 0), width=2))
        self.carriage.setData([0], [0])
        p.addItem(self.carriage)

    # ---- worker signals ----
    def _on_ready(self):
        # enable + auto-home; cycling starts once homing settles at center
        for cmd in ("PE1", "MC1", f"MLV{max(self.v * 1.5, 5):.0f}", "ME1", "EH"):
            self.worker.send(cmd)
        self.cycling = False

    def _on_endstop(self, mn, mx, homed, phase, pos, backstop):
        self.carriage.setData([pos], [0])
        ph = {0: "idle", 1: "seek MIN", 2: "seek MAX", 3: "centering"}.get(phase, "?")
        state = "BACKSTOP TRIPPED" if backstop else (f"homing: {ph}" if phase else
                ("cycling" if self.cycling else "homed"))
        self.banner.setText(
            f"{state}   pos={pos:7.1f} rad   "
            f"MIN {'● TRIG' if mn else '○ clear'}   MAX {'● TRIG' if mx else '○ clear'}")
        if homed and phase == 0:
            if not self.cycling:
                self.worker.send("MC1")
                self.worker.send(f"M{self.direction * self.v:.2f}")
                self.cycling = True
            elif mn and self.direction > 0:
                self.direction = -1
                self.worker.send(f"M{self.direction * self.v:.2f}")
            elif mx and self.direction < 0:
                self.direction = 1
                self.worker.send(f"M{self.direction * self.v:.2f}")

    def closeEvent(self, e):
        try:
            self.worker.send("ME0")
            self.worker.msleep(60)
            self.worker.stop()
        except Exception:
            pass
        super().closeEvent(e)


def main(v_cycle=60.0, plant="analytic", speed=1.0, shot=None, shot_delay=22.0):
    app = QtWidgets.QApplication(sys.argv)
    w = Carriage2D(v_cycle=v_cycle, plant=plant, speed=speed)
    w.show()
    if shot:   # grab the window to a PNG after it's homed + cycling, then quit (for verification)
        def _grab():
            w.grab().save(shot)
            print(f"saved {shot}", flush=True)
            app.quit()
        QtCore.QTimer.singleShot(int(shot_delay * 1000), _grab)
    # let Ctrl-C work
    t = QtCore.QTimer(); t.start(200); t.timeout.connect(lambda: None)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
