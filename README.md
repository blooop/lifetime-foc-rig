# MKS ESP32 FOC — workspace

Tooling + custom GUI for driving an MKS ESP32 FOC V2.0 board (SimpleFOC).
Hardware: A2212/13T 1000KV motor (7 pole pairs), AS5600 magnetic encoder (I2C),
12 V+ motor supply. Board enumerates as a CH340 on `/dev/ttyUSB0`.

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

## Toolchains (kept in ~/.venvs, do not move)
- `~/.venvs/pio` — PlatformIO (build/flash)
- `~/.venvs/focstudio` — Python + PyQt5/pyqtgraph/pyserial (the GUI)

## Commands
Build + flash firmware:
```
cd ~/projects/mks-foc/firmware && ~/.venvs/pio/bin/pio run -t upload --upload-port /dev/ttyUSB0
```
Launch the control panel (needs the 12 V supply on so the board calibrates):
```
cd ~/projects/mks-foc/panel && ~/.venvs/focstudio/bin/python foc_panel.py
```

## Notes
- foc_current torque control caused a runaway here — firmware is voltage-based only.
- Torque readout is a model estimate: set Phase resistance R (measure it) and KV in the panel.
- Original makerbase repo (docs/schematics/examples) is the separate ~/projects/MKS-ESP32FOC checkout.
