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
  enable/STOP, limits, live PID fields, live plots (target/vel/angle + estimated
  torque), and a relay-feedback velocity auto-tuner. Talks the SimpleFOC
  Commander protocol over serial (motor letter `M`).

## Features
- Voltage-based velocity / angle / torque control with live tuning + auto-tune.
- **Dual hall endstops, auto-home (MIN→MAX→center), soft limits, and a general
  trapezoidal motion profiler** — see [ENDSTOPS.md](ENDSTOPS.md). Extra Commander
  letters: `E` (endstops/homing) and `P` (motion profile).
- **Model torque estimate** (τ = Kt·(Vq−Ke·ω)/R) from the clean Vq signal; set R/KV,
  Kt-override for calibration. The inline current sense is linked read-only (`skip_align`)
  for future measured-Iq torque, but its ADC reading isn't usable yet (bring-up TODO).
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
pixi run panel       # launch the PyQt control panel (12 V on so the board calibrates)
pixi run lifecycle --cycles 5000 --vmeas 3.0   # headless endurance run (close the panel first)
```

## Notes
- foc_current torque control caused a runaway here — firmware is voltage-based only.
  The current sense is linked **read-only** (`skip_align`; torque_controller stays voltage); never foc_current.
- Torque readout is a **Vq model estimate** (set R + KV in the panel). The current sense's
  measured Iq is not usable yet (ESP32 ADC bring-up TODO) — that's why torque stays model-based.
- The 5% backstop and lifecycle slip metric arm/track off a full auto-home (`EH`), not a bare `EZ`.
- Only one program may own the serial port at a time — close the panel before `monitor`/`lifecycle`.
- Original makerbase repo (docs/schematics/examples) is a separate read-only reference checkout
  (location varies by setup).
