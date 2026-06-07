"""SimSerial — a `serial.Serial` look-alike backed by SoftFirmware + a Plant.

This is the injection point: `SerialWorker.run()` builds a `serial.Serial()` and
then drives it through the exact same API (`port/baudrate/timeout/dtr/rts`,
`open/close/write/readline`). Swap in `SimSerial` (via `FOC_SIM=1`) and the whole
GUI / lifecycle / analysis stack runs against the simulation unchanged.

A background thread owns the sim clock: each tick it drains queued commands,
runs one `firmware.loop()`, steps the plant, and paces to wall-clock (real-time
by default; `FOC_SIM_SPEED` accelerates). Telemetry the firmware emits is queued
for `readline()`. Toggling `rts` True (the panel's reset pulse) reboots the board.
"""
from __future__ import annotations

import os
import queue
import threading
import time

from .plant import PlantConfig, make_plant
from .soft_firmware import SoftFirmware

# Defaults; overridable via configure() (run_sim.py) or env vars.
_OPTS = {
    "plant": os.environ.get("FOC_SIM_PLANT", "genesis"),
    "speed": float(os.environ.get("FOC_SIM_SPEED", "1.0")),
    "control_hz": float(os.environ.get("FOC_SIM_HZ", "1000")),
    "viewer": os.environ.get("FOC_SIM_VIEWER", "") not in ("", "0"),
    "cfg": None,          # PlantConfig | None
    "on_step": None,      # callable(firmware, plant, now_us, dt) | None  (scenarios)
}


def configure(**opts):
    """Set sim options before the panel/lifecycle opens the port (run_sim.py)."""
    _OPTS.update({k: v for k, v in opts.items() if v is not None})


class SimSerial:
    def __init__(self, *args, **kwargs):
        self.port = kwargs.get("port")
        self.baudrate = kwargs.get("baudrate", 115200)
        self.timeout = kwargs.get("timeout", 0.05)
        self.dtr = False
        self._rts = False
        self.is_open = False
        self._inq: queue.Queue[str] = queue.Queue()
        self._outq: queue.Queue[str] = queue.Queue()
        self._reset_pending = False
        self._run = False
        self._thread = None
        self.plant = None
        self.firmware = None

    # ---- rts: a rising edge is the board-reset pulse ----
    @property
    def rts(self):
        return self._rts

    @rts.setter
    def rts(self, v):
        v = bool(v)
        if v and not self._rts:
            self._reset_pending = True
        self._rts = v

    # ---- serial.Serial API ----
    def open(self):
        cfg = _OPTS["cfg"] or PlantConfig()
        kw = {}
        if _OPTS["plant"] == "genesis":
            kw["timestep"] = 1.0 / max(50.0, _OPTS["control_hz"])   # physics dt == control dt
            if _OPTS["viewer"]:
                kw["show_viewer"] = True
        self.plant = make_plant(_OPTS["plant"], cfg, **kw)
        # scale the sim-time watchdog up for accelerated runs (keepalive is wall-clock)
        wd = int(3000 * max(1.0, _OPTS["speed"]))
        self.firmware = SoftFirmware(self.plant, self._emit, cfg, watchdog_ms=wd)
        self.is_open = True
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def close(self):
        self._run = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self.is_open = False

    def write(self, data):
        if isinstance(data, (bytes, bytearray)):
            data = data.decode(errors="replace")
        for line in data.split("\n"):
            line = line.strip()
            if line:
                self._inq.put(line)
        return len(data)

    def readline(self):
        try:
            s = self._outq.get(timeout=self.timeout if self.timeout else 0.05)
        except queue.Empty:
            return b""
        return (s + "\n").encode()

    def flush(self):
        pass

    @property
    def in_waiting(self):
        return self._outq.qsize()

    # ---- internals ----
    def _emit(self, line: str):
        self._outq.put(line)

    def _loop(self):
        opts = _OPTS
        rate = max(50.0, opts["control_hz"])
        dt = 1.0 / rate
        dt_us = int(dt * 1e6)
        speed = max(1e-3, opts["speed"])
        on_step = opts["on_step"]
        sim_us = 0
        wall0 = time.perf_counter()
        while self._run:
            if self._reset_pending:
                self._reset_pending = False
                self._drain(self._inq)
                self.firmware.reset()
                sim_us = 0
                wall0 = time.perf_counter()
            # drain queued commands
            while True:
                try:
                    cmd = self._inq.get_nowait()
                except queue.Empty:
                    break
                self.firmware.handle(cmd, sim_us // 1000)
            if on_step:
                on_step(self.firmware, self.plant, sim_us, dt)
            self.firmware.loop(sim_us)
            self.plant.step(dt)
            sim_us += dt_us
            # pace to wall clock
            target = wall0 + (sim_us * 1e-6) / speed
            slack = target - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            elif slack < -0.5:                 # fell behind badly -> resync clock
                wall0 = time.perf_counter() - (sim_us * 1e-6) / speed

    @staticmethod
    def _drain(q):
        try:
            while True:
                q.get_nowait()
        except queue.Empty:
            pass
