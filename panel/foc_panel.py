#!/usr/bin/env python3
"""Control panel for the MKS ESP32 FOC board (SimpleFOC Commander 'M').
Radio-button modes, target slider, live plots, PID tuning, and a relay-feedback
auto-tuner for the velocity loop. Voltage-based control only (no foc_current)."""
import sys, queue, time, math
from collections import deque
import serial
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

PORT = '/dev/ttyUSB0'
BAUD = 115200

MODES = {
    'Torque (V)': (['MC0', 'MT0'], -2.0, 2.0, 'V'),
    'Velocity':   (['MC1'],        -20.0, 20.0, 'rad/s'),
    'Angle':      (['MC2'],        -12.57, 12.57, 'rad'),
}
SLIDER_STEPS = 1000
PLOT_PTS = 500


class SerialWorker(QtCore.QThread):
    line = QtCore.pyqtSignal(str)
    telem = QtCore.pyqtSignal(float, float, float, float)  # target, Vq, velocity, angle
    endstop = QtCore.pyqtSignal(int, int, int, int, float)  # minTrig, maxTrig, homed, homing, pos
    ready = QtCore.pyqtSignal()
    tune_status = QtCore.pyqtSignal(str)
    tune_done = QtCore.pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self._run = True
        self._tx = queue.Queue()
        self.ser = None
        self._pending_tune = None
        self.tuning = False
        self.T = None

    # ---- public (GUI thread) ----
    def send(self, cmd):
        self._tx.put(cmd)

    def start_autotune(self, params):
        self._pending_tune = params

    def abort_tune(self):
        if self.tuning:
            self._tune_finish(False, 'aborted')

    def reset_board(self):
        if self.ser:
            self.ser.rts = True; self.msleep(100); self.ser.rts = False

    def stop(self):
        self._run = False
        self.wait(1500)

    # ---- thread ----
    def run(self):
        try:
            self.ser = serial.Serial()
            self.ser.port, self.ser.baudrate, self.ser.timeout = PORT, BAUD, 0.05
            self.ser.dtr = False; self.ser.rts = False
            self.ser.open()
        except Exception as e:
            self.line.emit(f"!! cannot open {PORT}: {e}")
            return
        self.line.emit(f"opened {PORT} @ {BAUD}")
        while self._run:
            if self._pending_tune and not self.tuning:
                self._tune_init(self._pending_tune); self._pending_tune = None
            try:
                while True:
                    self.ser.write((self._tx.get_nowait() + "\n").encode())
            except queue.Empty:
                pass
            try:
                raw = self.ser.readline()
            except Exception as e:
                self.line.emit(f"!! serial error: {e}"); break
            if not raw:
                continue
            s = raw.decode(errors='replace').strip()
            if not s:
                continue
            if s.startswith('E\t'):   # endstop/position telemetry (distinct from the motor monitor)
                p = s.split('\t')
                if len(p) >= 6:
                    try:
                        self.endstop.emit(int(p[1]), int(p[2]), int(p[3]), int(p[4]), float(p[5]))
                        continue
                    except ValueError:
                        pass
                self.line.emit(s); continue
            parts = s.split('\t')
            if len(parts) >= 7:
                try:
                    v = [float(p) for p in parts]
                except ValueError:
                    self.line.emit(s); continue
                self.telem.emit(v[0], v[1], v[5], v[6])
                if self.tuning:
                    self._tune_step(time.monotonic(), v[5])
                continue
            self.line.emit(s)
            if 'Motor ready' in s:
                self.ready.emit()
        if self.ser:
            self.ser.close()

    # ---- relay auto-tune (worker thread) ----
    def _w(self, cmd):
        self.ser.write((cmd + "\n").encode())

    def _tune_init(self, p):
        self.T = dict(d=p['d'], bias=p['bias'], eps=p['eps'],
                      max_cycles=p['max_cycles'], v_abort=p['v_abort'], timeout=p['timeout'],
                      t0=time.monotonic(), vq=0.0, last_sign=0, ups=[],
                      vmin=1e9, vmax=-1e9, warmup=2)
        for c in ('MC0', 'MT0', 'ME1', 'MMD10', 'M0'):   # torque-V, enable, fast telemetry
            self._w(c)
        self.tuning = True
        self.tune_status.emit(f"relay running (d={p['d']} V, bias={p['bias']} rad/s)…")

    def _tune_step(self, t, vel):
        T = self.T
        el = t - T['t0']
        if abs(vel) > T['v_abort']:
            self._tune_finish(False, f"overspeed {vel:.1f} rad/s"); return
        if el > T['timeout']:
            self._tune_finish(False, "timeout (no clean oscillation)"); return
        # relay with hysteresis
        nv = T['vq']
        if vel < T['bias'] - T['eps']:
            nv = +T['d']
        elif vel > T['bias'] + T['eps']:
            nv = -T['d']
        if nv != T['vq']:
            T['vq'] = nv; self._w(f"M{nv:.3f}")
        # up-crossing of bias -> period markers
        sign = 1 if vel > T['bias'] else -1
        if T['last_sign'] == -1 and sign == 1:
            T['ups'].append(el)
        T['last_sign'] = sign
        if len(T['ups']) >= T['warmup']:
            T['vmin'] = min(T['vmin'], vel); T['vmax'] = max(T['vmax'], vel)
        if len(T['ups']) >= T['max_cycles']:
            self._tune_finish(True)

    def _tune_finish(self, ok, reason=''):
        T = self.T
        self._w('M0'); self._w('MMD100')
        res = {'success': False, 'reason': reason}
        ups = T['ups']
        if ok and len(ups) >= T['warmup'] + 2:
            periods = [ups[i + 1] - ups[i] for i in range(len(ups) - 1)]
            steady = periods[T['warmup']:] or periods
            Tu = sum(steady) / len(steady)
            a = (T['vmax'] - T['vmin']) / 2.0
            denom = math.sqrt(max(a * a - T['eps'] * T['eps'], 1e-6))
            Ku = 4.0 * T['d'] / (math.pi * denom)
            P = 0.45 * Ku
            I = P / (2.2 * Tu) if Tu > 0 else 0.0
            self._w(f"MVP{P:.4f}"); self._w(f"MVI{I:.4f}"); self._w("MVD0")
            res = dict(success=True, Ku=Ku, Tu=Tu, a=a, P=P, I=I, cycles=len(steady))
        # back to a safe state
        self._w('M0'); self._w('MC1'); self._w('ME0')
        self.tuning = False
        self.tune_done.emit(res)


class Panel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('MKS ESP32 FOC — Control Panel')
        self.resize(1040, 680)
        self.t0 = time.monotonic()
        self.buf_t = deque(maxlen=PLOT_PTS)
        self.buf_tar = deque(maxlen=PLOT_PTS)
        self.buf_vel = deque(maxlen=PLOT_PTS)
        self.buf_ang = deque(maxlen=PLOT_PTS)
        self.buf_tau = deque(maxlen=PLOT_PTS)
        self.worker = SerialWorker()
        self.worker.line.connect(self.on_line)
        self.worker.telem.connect(self.on_telem)
        self.worker.endstop.connect(self.on_endstop)
        self.worker.ready.connect(self.on_ready)
        self.worker.tune_status.connect(lambda s: self.tune_lbl.setText(s))
        self.worker.tune_done.connect(self.on_tune_done)
        self._build()
        self.set_controls_enabled(False)
        self.worker.start()

    def _build(self):
        main = QtWidgets.QHBoxLayout(self)
        left = QtWidgets.QVBoxLayout()
        main.addLayout(left, 0)

        # mode radios
        mbox = QtWidgets.QGroupBox('Control mode'); ml = QtWidgets.QHBoxLayout(mbox)
        self.mode_group = QtWidgets.QButtonGroup(self)
        for i, name in enumerate(MODES):
            rb = QtWidgets.QRadioButton(name); self.mode_group.addButton(rb, i); ml.addWidget(rb)
            if name == 'Velocity': rb.setChecked(True)
        self.mode_group.buttonClicked.connect(self.on_mode)
        left.addWidget(mbox)

        # target
        tbox = QtWidgets.QGroupBox('Target'); tl = QtWidgets.QGridLayout(tbox)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal); self.slider.setRange(-SLIDER_STEPS, SLIDER_STEPS)
        self.slider.valueChanged.connect(self.on_slider)
        self.spin = QtWidgets.QDoubleSpinBox(); self.spin.setDecimals(2); self.spin.valueChanged.connect(self.on_spin)
        self.unit_lbl = QtWidgets.QLabel('rad/s')
        z = QtWidgets.QPushButton('Center / 0'); z.clicked.connect(lambda: self.spin.setValue(0))
        tl.addWidget(self.slider, 0, 0, 1, 3); tl.addWidget(self.spin, 1, 0); tl.addWidget(self.unit_lbl, 1, 1); tl.addWidget(z, 1, 2)
        left.addWidget(tbox)

        # buttons
        b = QtWidgets.QHBoxLayout()
        self.enable_btn = QtWidgets.QPushButton('Enable'); self.enable_btn.setCheckable(True); self.enable_btn.toggled.connect(self.on_enable)
        self.stop_btn = QtWidgets.QPushButton('STOP'); self.stop_btn.setStyleSheet('font-weight:bold;color:white;background:#c0392b;'); self.stop_btn.clicked.connect(self.on_stop)
        self.reset_btn = QtWidgets.QPushButton('Reset board'); self.reset_btn.clicked.connect(self.worker.reset_board)
        b.addWidget(self.enable_btn); b.addWidget(self.stop_btn); b.addWidget(self.reset_btn)
        left.addLayout(b)

        # limits
        lbox = QtWidgets.QGroupBox('Limits'); ll = QtWidgets.QFormLayout(lbox)
        self.vlim = QtWidgets.QDoubleSpinBox(); self.vlim.setRange(0, 12); self.vlim.setValue(1.0); self.vlim.setSingleStep(0.5)
        self.vlim.editingFinished.connect(lambda: self.worker.send(f"MLU{self.vlim.value():.2f}"))
        self.velim = QtWidgets.QDoubleSpinBox(); self.velim.setRange(0, 100); self.velim.setValue(20); self.velim.setSingleStep(5)
        self.velim.editingFinished.connect(lambda: self.worker.send(f"MLV{self.velim.value():.1f}"))
        ll.addRow('Voltage limit [V]', self.vlim); ll.addRow('Velocity limit [rad/s]', self.velim)
        left.addWidget(lbox)

        # motion profile (acceleration-limited / trapezoidal)
        pbox = QtWidgets.QGroupBox('Motion profile'); pl = QtWidgets.QFormLayout(pbox)
        self.prof_en = QtWidgets.QCheckBox('enable trapezoidal profiling'); self.prof_en.setChecked(True)
        self.prof_en.toggled.connect(lambda on: self.worker.send(f"PE{1 if on else 0}"))
        self.prof_acc = QtWidgets.QDoubleSpinBox(); self.prof_acc.setRange(1, 2000); self.prof_acc.setValue(50); self.prof_acc.setSingleStep(5)
        self.prof_acc.editingFinished.connect(lambda: self.worker.send(f"PA{self.prof_acc.value():.1f}"))
        pl.addRow(self.prof_en); pl.addRow('Acceleration [rad/s²]', self.prof_acc)
        left.addWidget(pbox)

        # endstops & homing
        self._build_endstops(left)

        # tuning
        gbox = QtWidgets.QGroupBox('PID tuning'); gl = QtWidgets.QFormLayout(gbox)
        def mk(lo, hi, val, step, dec, cmd):
            sb = QtWidgets.QDoubleSpinBox(); sb.setRange(lo, hi); sb.setValue(val); sb.setSingleStep(step); sb.setDecimals(dec)
            sb.editingFinished.connect(lambda c=cmd, s=sb: self.worker.send(f"{c}{s.value():.4f}"))
            return sb
        self.velP = mk(0, 5, 0.05, 0.01, 4, 'MVP'); self.velI = mk(0, 50, 1.0, 0.5, 3, 'MVI')
        self.angP = mk(0, 100, 10.0, 1.0, 2, 'MAP'); self.velF = mk(0, 0.5, 0.02, 0.005, 3, 'MVF')
        gl.addRow('Velocity P', self.velP); gl.addRow('Velocity I', self.velI)
        gl.addRow('Angle P', self.angP); gl.addRow('Velocity LPF Tf', self.velF)
        left.addWidget(gbox)

        # auto-tune
        abox = QtWidgets.QGroupBox('Auto-tune velocity (relay)'); al = QtWidgets.QFormLayout(abox)
        self.at_d = QtWidgets.QDoubleSpinBox(); self.at_d.setRange(0.05, 5); self.at_d.setValue(0.4); self.at_d.setSingleStep(0.1); self.at_d.setDecimals(2)
        self.at_bias = QtWidgets.QDoubleSpinBox(); self.at_bias.setRange(0, 20); self.at_bias.setValue(3.0); self.at_bias.setSingleStep(0.5)
        self.at_eps = QtWidgets.QDoubleSpinBox(); self.at_eps.setRange(0.05, 5); self.at_eps.setValue(0.4); self.at_eps.setSingleStep(0.1); self.at_eps.setDecimals(2)
        al.addRow('Relay amplitude d [V]', self.at_d)
        al.addRow('Speed bias [rad/s]', self.at_bias)
        al.addRow('Hysteresis ε [rad/s]', self.at_eps)
        self.at_btn = QtWidgets.QPushButton('Run auto-tune'); self.at_btn.clicked.connect(self.on_autotune)
        al.addRow(self.at_btn)
        self.tune_lbl = QtWidgets.QLabel('idle'); self.tune_lbl.setWordWrap(True)
        al.addRow(self.tune_lbl)
        left.addWidget(abox)

        # torque estimate params (model-based, no current sensor)
        ebox = QtWidgets.QGroupBox('Torque estimate'); el = QtWidgets.QFormLayout(ebox)
        self.res = QtWidgets.QDoubleSpinBox(); self.res.setRange(0.01, 10); self.res.setValue(0.15); self.res.setSingleStep(0.01); self.res.setDecimals(3)
        self.kv = QtWidgets.QDoubleSpinBox(); self.kv.setRange(1, 5000); self.kv.setValue(1000); self.kv.setSingleStep(50)
        el.addRow('Phase resistance R [Ω]', self.res)
        el.addRow('KV [rpm/V]', self.kv)
        left.addWidget(ebox)

        # live readouts
        rbox = QtWidgets.QGroupBox('Live'); rl = QtWidgets.QFormLayout(rbox)
        self.t_lbl = QtWidgets.QLabel('—'); self.v_lbl = QtWidgets.QLabel('—'); self.a_lbl = QtWidgets.QLabel('—')
        self.i_lbl = QtWidgets.QLabel('—'); self.q_lbl = QtWidgets.QLabel('—'); self.pos_lbl = QtWidgets.QLabel('—')
        for w in (self.t_lbl, self.v_lbl, self.a_lbl, self.i_lbl, self.q_lbl, self.pos_lbl): w.setStyleSheet('font-family:monospace;font-size:15px;')
        rl.addRow('Target', self.t_lbl); rl.addRow('Velocity [rad/s]', self.v_lbl); rl.addRow('Angle [rad]', self.a_lbl)
        rl.addRow('Est. current [A]', self.i_lbl); rl.addRow('Est. torque [mN·m]', self.q_lbl)
        rl.addRow('Position (home) [rad]', self.pos_lbl)
        left.addWidget(rbox)
        left.addStretch(1)

        # right: plots + log
        right = QtWidgets.QVBoxLayout(); main.addLayout(right, 1)
        pg.setConfigOptions(antialias=True)
        self.plot = pg.PlotWidget(title='Live telemetry'); self.plot.addLegend(); self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel('bottom', 'time', 's')
        self.c_tar = self.plot.plot(pen=pg.mkPen('y', width=2), name='target')
        self.c_vel = self.plot.plot(pen=pg.mkPen('c', width=2), name='velocity')
        self.c_ang = self.plot.plot(pen=pg.mkPen('m', width=1), name='angle')
        right.addWidget(self.plot, 3)
        self.plot2 = pg.PlotWidget(title='Estimated torque'); self.plot2.showGrid(x=True, y=True, alpha=0.3)
        self.plot2.setLabel('left', 'torque', 'mN·m'); self.plot2.setLabel('bottom', 'time', 's')
        self.plot2.setXLink(self.plot)
        self.c_tau = self.plot2.plot(pen=pg.mkPen('g', width=2), name='torque')
        right.addWidget(self.plot2, 2)
        self.log = QtWidgets.QPlainTextEdit(); self.log.setReadOnly(True); self.log.setMaximumBlockCount(300)
        self.log.setStyleSheet('font-family:monospace;font-size:11px;')
        right.addWidget(self.log, 1)

        self.on_mode()

    # ---- endstops & homing UI ----
    def _dot(self, text):
        d = QtWidgets.QLabel(text); d.setAlignment(QtCore.Qt.AlignCenter)
        d.setMinimumWidth(64); d.setStyleSheet(self._dot_css('#888'))
        return d

    @staticmethod
    def _dot_css(bg):
        return f'font-family:monospace;font-weight:bold;color:white;background:{bg};border-radius:4px;padding:2px 6px;'

    def _build_endstops(self, parent):
        box = QtWidgets.QGroupBox('Endstops & homing'); v = QtWidgets.QVBoxLayout(box)

        # live status indicators (stream from boot, even with the motor disabled)
        srow = QtWidgets.QHBoxLayout()
        self.es_min_dot = self._dot('MIN ?'); self.es_max_dot = self._dot('MAX ?'); self.homed_dot = self._dot('not homed')
        srow.addWidget(self.es_min_dot); srow.addWidget(self.es_max_dot); srow.addWidget(self.homed_dot); srow.addStretch(1)
        v.addLayout(srow)

        # safety banner — reflects any active endstop / homing
        self.safety_lbl = QtWidgets.QLabel('endstops clear'); self.safety_lbl.setWordWrap(True)
        self.safety_lbl.setStyleSheet(self._dot_css('#27ae60'))
        v.addWidget(self.safety_lbl)

        # per-endstop config (enable / pin / active-low)
        def es_config(tag, default_pin):
            row = QtWidgets.QHBoxLayout()
            en = QtWidgets.QCheckBox(f'{tag} enable'); en.setChecked(True)
            en.toggled.connect(lambda on, t=tag: self.worker.send(f"E{t}E{1 if on else 0}"))
            pin = QtWidgets.QSpinBox(); pin.setRange(0, 39); pin.setValue(default_pin); pin.setPrefix('GPIO ')
            pin.editingFinished.connect(lambda t=tag, s=pin: self.worker.send(f"E{t}P{s.value()}"))
            low = QtWidgets.QCheckBox('active-low'); low.setChecked(True)
            low.toggled.connect(lambda on, t=tag: self.worker.send(f"E{t}L{1 if on else 0}"))
            row.addWidget(en); row.addWidget(pin); row.addWidget(low); row.addStretch(1)
            return row
        v.addLayout(es_config('A', 5))    # A = MIN / home  (GPIO5 strapping pin)
        v.addLayout(es_config('B', 23))   # B = MAX

        # homing controls
        hl = QtWidgets.QFormLayout()
        self.home_speed = QtWidgets.QDoubleSpinBox(); self.home_speed.setRange(0.1, 20); self.home_speed.setValue(20.0); self.home_speed.setSingleStep(0.5)
        self.home_speed.editingFinished.connect(lambda: self.worker.send(f"ES{self.home_speed.value():.2f}"))
        self.home_dir = QtWidgets.QComboBox(); self.home_dir.addItems(['MIN = − velocity', 'MIN = + velocity'])
        self.home_dir.setCurrentIndex(1)   # default +velocity toward MIN (matches firmware)
        self.home_dir.currentIndexChanged.connect(lambda i: self.worker.send('ED1' if i else 'ED-1'))
        hl.addRow('Seek speed [rad/s]', self.home_speed); hl.addRow('Seek-MIN direction', self.home_dir)
        v.addLayout(hl)
        hb = QtWidgets.QHBoxLayout()
        self.home_btn = QtWidgets.QPushButton('Auto-home (MIN→MAX→center)'); self.home_btn.clicked.connect(self.on_home)
        self.zero_btn = QtWidgets.QPushButton('Set zero here'); self.zero_btn.clicked.connect(lambda: self.worker.send('EZ'))
        hb.addWidget(self.home_btn); hb.addWidget(self.zero_btn)
        v.addLayout(hb)

        # soft travel limits (home-relative)
        sl = QtWidgets.QFormLayout()
        self.soft_en = QtWidgets.QCheckBox('enable soft travel limits')
        self.soft_en.toggled.connect(lambda on: self.worker.send(f"ELE{1 if on else 0}"))
        self.soft_min = QtWidgets.QDoubleSpinBox(); self.soft_min.setRange(-1000, 1000); self.soft_min.setValue(-6.28); self.soft_min.setSingleStep(0.5)
        self.soft_min.editingFinished.connect(lambda: self.worker.send(f"ELN{self.soft_min.value():.3f}"))
        self.soft_max = QtWidgets.QDoubleSpinBox(); self.soft_max.setRange(-1000, 1000); self.soft_max.setValue(6.28); self.soft_max.setSingleStep(0.5)
        self.soft_max.editingFinished.connect(lambda: self.worker.send(f"ELX{self.soft_max.value():.3f}"))
        sl.addRow(self.soft_en); sl.addRow('Min travel [rad]', self.soft_min); sl.addRow('Max travel [rad]', self.soft_max)
        v.addLayout(sl)

        parent.addWidget(box)

    HOME_PHASE = {1: 'AUTO-HOME… seeking MIN', 2: 'AUTO-HOME… seeking MAX', 3: 'AUTO-HOME… centering'}

    def on_home(self):
        # firmware refuses unless the motor is already enabled (safety posture)
        if not self.enable_btn.isChecked():
            self.log.appendPlainText('# Auto-home: enable the motor first'); return
        self.worker.send('EH'); self.log.appendPlainText('# auto-home: MIN → MAX → center…')

    def on_endstop(self, mn, mx, homed, phase, pos):
        self.es_min_dot.setText('MIN HIT' if mn else 'MIN clr'); self.es_min_dot.setStyleSheet(self._dot_css('#c0392b' if mn else '#27ae60'))
        self.es_max_dot.setText('MAX HIT' if mx else 'MAX clr'); self.es_max_dot.setStyleSheet(self._dot_css('#c0392b' if mx else '#27ae60'))
        self.homed_dot.setText('HOMED' if homed else 'not homed'); self.homed_dot.setStyleSheet(self._dot_css('#2980b9' if homed else '#888'))
        self.pos_lbl.setText(f"{pos:+.3f}")
        if phase:
            self.safety_lbl.setText(self.HOME_PHASE.get(phase, 'AUTO-HOME…')); self.safety_lbl.setStyleSheet(self._dot_css('#2980b9'))
        elif mn or mx:
            which = ' & '.join(n for n, f in (('MIN', mn), ('MAX', mx)) if f)
            self.safety_lbl.setText(f'⚠ {which} endstop ACTIVE — motion into limit blocked'); self.safety_lbl.setStyleSheet(self._dot_css('#c0392b'))
        else:
            self.safety_lbl.setText('endstops clear'); self.safety_lbl.setStyleSheet(self._dot_css('#27ae60'))

    # ---- helpers ----
    def current_mode(self): return list(MODES)[self.mode_group.checkedId()]

    def set_controls_enabled(self, on):
        for w in (self.slider, self.spin, self.enable_btn, self.stop_btn, self.at_btn, self.home_btn):
            w.setEnabled(on)

    # ---- slots ----
    def on_mode(self, *_):
        name = self.current_mode(); cmds, lo, hi, unit = MODES[name]
        for c in cmds: self.worker.send(c)
        self.unit_lbl.setText(unit)
        self.spin.blockSignals(True); self.slider.blockSignals(True)
        self.spin.setRange(lo, hi); self.spin.setValue(0); self.slider.setValue(0)
        self.spin.blockSignals(False); self.slider.blockSignals(False)
        self._lo, self._hi = lo, hi
        self.worker.send('M0')
        self.log.appendPlainText(f"# mode -> {name}")

    def on_slider(self, val):
        f = self._lo + (val + SLIDER_STEPS) / (2 * SLIDER_STEPS) * (self._hi - self._lo)
        self.spin.blockSignals(True); self.spin.setValue(f); self.spin.blockSignals(False)
        self.worker.send(f"M{f:.3f}")

    def on_spin(self, f):
        frac = (f - self._lo) / (self._hi - self._lo) if self._hi > self._lo else 0.5
        self.slider.blockSignals(True); self.slider.setValue(int(round(frac * 2 * SLIDER_STEPS - SLIDER_STEPS))); self.slider.blockSignals(False)
        self.worker.send(f"M{f:.3f}")

    def on_enable(self, on):
        self.worker.send('ME1' if on else 'ME0'); self.enable_btn.setText('Enabled' if on else 'Enable')

    def on_stop(self):
        self.worker.abort_tune()
        self.worker.send('EX')   # abort any homing in progress
        self.worker.send('ME0'); self.worker.send('M0')
        self.spin.setValue(0); self.enable_btn.setChecked(False)
        self.tune_lbl.setText('idle'); self.log.appendPlainText('# STOP')

    def on_autotune(self):
        p = dict(d=self.at_d.value(), bias=self.at_bias.value(), eps=self.at_eps.value(),
                 max_cycles=12, v_abort=max(2 * self.at_bias.value() + 10, 25), timeout=8.0)
        self.set_controls_enabled(False); self.at_btn.setEnabled(False)
        self.enable_btn.setChecked(True)
        self.tune_lbl.setText('starting…'); self.log.appendPlainText('# auto-tune started')
        self.worker.start_autotune(p)

    def on_tune_done(self, r):
        self.set_controls_enabled(True)
        self.enable_btn.setChecked(False)  # ends disabled (safe)
        if r.get('success'):
            self.velP.blockSignals(True); self.velI.blockSignals(True)
            self.velP.setValue(r['P']); self.velI.setValue(r['I'])
            self.velP.blockSignals(False); self.velI.blockSignals(False)
            msg = (f"OK: Ku={r['Ku']:.3f}, Tu={r['Tu']*1000:.0f}ms, amp={r['a']:.2f} "
                   f"→ P={r['P']:.4f}, I={r['I']:.3f} (applied)")
        else:
            msg = f"FAILED: {r.get('reason','?')}. Try larger d / different bias."
        self.tune_lbl.setText(msg); self.log.appendPlainText('# auto-tune ' + msg)

    def on_ready(self):
        self.set_controls_enabled(True); self.enable_btn.setChecked(False); self.worker.send('ME0')
        self.log.appendPlainText('# board ready (DISABLED) — set mode/target, then Enable')

    def on_line(self, s): self.log.appendPlainText(s)

    def on_telem(self, t, vq, v, a):
        # model-based torque estimate: Kt = Ke = 9.549/KV ; Iq = (Vq - Ke*w)/R ; tau = Kt*Iq
        kt = 9.549 / max(self.kv.value(), 1.0)
        iq = (vq - kt * v) / max(self.res.value(), 1e-3)
        tau = kt * iq
        self.t_lbl.setText(f"{t:.3f}"); self.v_lbl.setText(f"{v:.3f}"); self.a_lbl.setText(f"{a:.3f}")
        self.i_lbl.setText(f"{iq:+.2f}"); self.q_lbl.setText(f"{tau*1000:+.1f}")
        now = time.monotonic() - self.t0
        self.buf_t.append(now); self.buf_tar.append(t); self.buf_vel.append(v); self.buf_ang.append(a)
        self.buf_tau.append(tau * 1000)
        self.c_tar.setData(self.buf_t, self.buf_tar)
        self.c_vel.setData(self.buf_t, self.buf_vel)
        self.c_ang.setData(self.buf_t, self.buf_ang)
        self.c_tau.setData(self.buf_t, self.buf_tau)

    def closeEvent(self, e):
        try:
            self.worker.abort_tune(); self.worker.send('ME0'); self.worker.msleep(80)
        except Exception:
            pass
        self.worker.stop(); e.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    p = Panel(); p.show()
    sys.exit(app.exec_())
