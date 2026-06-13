# MKS ESP32 FOC — workspace

Tooling + custom GUI for driving an MKS ESP32 FOC V2.0 board (SimpleFOC).
Hardware: A2212/13T 1000KV motor (7 pole pairs), AS5600 magnetic encoder (I2C),
12 V+ motor supply. Board enumerates as a CH340 — `/dev/ttyUSB0` on Linux,
`/dev/tty.usbserial-*` / `/dev/tty.wchusbserial*` on macOS (auto-detected; needs a
real *data* USB cable). Runs on Linux and macOS via [pixi](https://pixi.sh).

## Layout
- `firmware/` — PlatformIO project. `src/main.cpp` is the active sketch
  (safe voltage-based SimpleFOC firmware: velocity/angle/torque-voltage,
  boots disabled, 1 V limit, armed glitch filter, 100 kHz I2C, fast I2C timeout).
- `panel/foc_panel.py` — custom PyQt5 control panel: mode radios, target slider,
  enable/STOP, limits, live PID fields, a 2D rig view, live plots (target/vel/angle
  + estimated torque), and a relay-feedback velocity auto-tuner. Talks the SimpleFOC
  Commander protocol over serial (motor letter `M`).
- `panel/rig_view.py` — 2D rig view (embedded in the panel; standalone via
  `pixi run viewer`): rail, carriage, hall markers (learned live from telemetry),
  backstop lines.
- `panel/sim/` — the modeled rig (pure-Python firmware port + plant). The GUI,
  viewer, and lifecycle runner **auto-detect**: board present → real rig; no board
  → the modeled rig with a clear **SIMULATED RIG** badge. `FOC_SIM=1`/`FOC_SIM=0`
  forces sim/hardware. See `panel/sim/README.md`.

## Features
- Voltage-based velocity / angle / torque control with live tuning + auto-tune.
- **Dual hall endstops, auto-home (MIN→MAX→center), soft limits, and a general
  trapezoidal motion profiler** — see [ENDSTOPS.md](ENDSTOPS.md). Extra Commander
  letters: `E` (endstops/homing) and `P` (motion profile).
- **Model torque estimate** (τ = Kt·(Vq−Ke·ω)/R) from the clean Vq signal; set R/KV,
  Kt-override for calibration. The inline current sense is linked read-only (`skip_align`);
  its measured Iq is now usable (verified) and is the reference for calibrating the model.
- **5% overtravel backstop** (auto-armed by a full home; disables the motor if a
  failed hall lets the carriage run 5% of travel past where the endstop should be),
  plus a **serial-heartbeat watchdog** (de-energizes if the panel goes silent).
- **Lifecycle endurance test** for the linear rolling-contact drive: cycles
  hall-to-hall and logs torque-per-position (energy/stroke) + slip (motor-rotation
  per stroke) vs cycle#. Runs from the panel ("Lifecycle test" group + wear-trend
  window) or headless via `run_lifecycle.py`. Logs stay in the repo under
  `panel/lifecycle_runs/<timestamp>/`.

## Environment (pixi)
All toolchains (PlatformIO + Python/PyQt5/pyqtgraph/pyserial/numpy) are managed by
[pixi](https://pixi.sh) — no system Python or venvs needed. One-time, from the repo root:
```
pixi install
```
This works on Linux and macOS (Intel + Apple Silicon).

The board uses a CH340 USB-serial chip. Linux has the driver in-kernel; recent macOS
usually does too. If the port doesn't appear on macOS, install the WCH driver:
```
pixi run install-driver
```
(then approve it in System Settings > Privacy & Security and replug the board).

## Commands
Run everything via pixi tasks (from the repo root). The serial port is auto-detected
(CH340 by USB VID); override with `FOC_PORT=/dev/tty.usbserial-XXXX` or `--port` if you
have multiple serial devices.
```
pixi run build       # compile firmware
pixi run flash       # build + flash over USB (auto-detects the port)
pixi run monitor     # serial monitor (close the panel first)
pixi run gui         # PyQt control panel + 2D rig view (12 V on so the board calibrates)
pixi run viewer      # standalone 2D rig view (real board: passive watch; no board: sim demo)
pixi run lifecycle --cycles 5000 --vmeas 3.0   # endurance run (close the panel first)
pixi run plot        # view a finished run's wear trends + τ(pos) heatmap (newest, or pass a run dir)
```
`gui`/`viewer`/`lifecycle` need no board: with none detected they run the **modeled
rig** (`panel/sim/`) and badge themselves **SIMULATED RIG** — same code path, same
telemetry, nothing physical. Sim lifecycle runs record `"sim": true` in `config.json`.
A lifecycle run against the modeled rig opens the **live GUI** (2D rig view + plots +
wear trends) by default so it looks just like driving the real rig; on real hardware it
stays headless for unattended runs. Override with `--view` / `--headless`.
Runs log to `panel/lifecycle_runs/<timestamp>/` (`summary.csv`, `profile.csv`). The panel
shows live trends during a run; `pixi run plot` reopens that view for a finished run.

## Tests (no hardware required)
Both layers run on a desktop with no board attached, so development can continue offline.
```
pixi run test       # Python: panel serial-parse, find_serial_port, autotune math,
                    #         torque model, and the full LifecycleController state machine
pixi run test-fw    # firmware: native (host) unit tests for the safety/motion math
```
- **Python** (`panel/tests/`, pytest): PyQt-coupled code runs headless via
  `QT_QPA_PLATFORM=offscreen`. The `LifecycleController` is driven through a `FakeWorker`
  test double (records `send()`s, emits the telem/slip/endstop/ready signals) plus a
  controllable clock — homing→run→park sequencing, cycle counting, slip/Iq/backstop/stall
  aborts, and CSV/resume are all exercised with no serial link.
- **Firmware** (`firmware/test/`, PlatformIO `native` + Unity): the load-bearing safety math
  lives in `firmware/src/control_logic.h` as pure, globals-free functions (travel-limit clamp,
  overtravel backstop, `v_safe` sizing, the time-aware glitch filter, and the trapezoidal
  motion profile); `main.cpp` calls them on-target. `pixi run build` is also a compile smoke
  test for the real board. CI (`.github/workflows/ci.yml`) runs all three on push/PR.

## Notes
- foc_current torque control caused a runaway here — firmware is voltage-based only.
  The current sense is linked **read-only** (`skip_align`; torque_controller stays voltage); never foc_current.
- Torque readout is a **Vq model estimate** (set R + KV in the panel). The current sense's
  measured Iq **is** usable (verified) — the monitor emits it in mA, the panel converts to amps —
  but torque stays model-based for now; measured Iq is the calibration reference / future torque source.
- The 5% backstop and lifecycle slip metric arm/track off a full auto-home (`EH`), not a bare `EZ`.
- Only one program may own the serial port at a time — close the panel before `monitor`/`lifecycle`.
- Original makerbase repo (docs/schematics/examples) is a separate read-only reference checkout
  (location varies by setup).
