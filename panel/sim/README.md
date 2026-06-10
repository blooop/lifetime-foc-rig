# Modeled rig (simulation behind the serial seam)

Run the **entire real software stack** — `foc_panel.py` GUI, `rig_view.py` 2D
view, `lifecycle.py`, auto-tuner, CSV logging, `plot_lifecycle.py` — against a
modeled rig instead of the MKS ESP32 board, with **no hardware**. Used to
evaluate and validate testing regimes and analysis offline.

**You don't opt in** — the stack auto-detects. `pixi run gui` / `viewer` /
`lifecycle` use the real board when one is present and fall back to the modeled
rig when not, badging themselves **SIMULATED RIG** (GUI/viewer badge, headless
`##` banner, `"sim": true` in a lifecycle run's `config.json`). Force with
`FOC_SIM=1` (sim even with a board) or `FOC_SIM=0` (wait for hardware).

## How it works

The whole stack talks to hardware through one object — `SerialWorker`
(`foc_panel.py`). `resolve_backend()` decides once per session: real port →
`serial.Serial()`, no board (or `FOC_SIM=1`) → **`SimSerial`**; nothing else in
the GUI/lifecycle/analysis changes. The choice is latched — a mid-run USB drop
keeps retrying the board, it never silently becomes a sim.

```
GUI / viewer / lifecycle / analysis   (unchanged)
        │  SerialWorker ── resolve_backend(): board? ──▶ serial.Serial
        ▼                              └─ no board ────▶ SimSerial
SimSerial         serial.Serial look-alike; a thread runs the sim clock
  SoftFirmware    Python port of firmware/src/main.cpp (control, homing,
                  limits, backstop, watchdog, glitch filter, telemetry,
                  Vq→Iq→τ electrical model)
  AnalyticPlant   the mechanism: 1 rotary DOF, inertia, friction, hall
                  endstops, hard stops (pure Python, semi-implicit Euler)
```

`SoftFirmware` reproduces the firmware *behavior*; the plant integrates the
mechanical dynamics. (A Genesis physics plant once sat behind the same `Plant`
interface but was removed — the analytic model matched the hardware oracle and
needs no torch/taichi deps or separate pixi env.)

## Usage

```
pixi run gui          # no board attached -> GUI + 2D rig view vs the modeled rig
pixi run viewer       # no board -> the modeled rig homes + cycles in the 2D view
pixi run lifecycle --cycles 30 --vmeas 20 --speed 4 --scenario wear   # headless
FOC_SIM=1 pixi run gui                # force the sim even with a board attached
FOC_SIM_SPEED=4 FOC_SIM_SCENARIO=hall_slip pixi run gui   # env-var equivalents
```

A lifecycle run against the modeled rig opens the **live GUI** by default (the full
control panel: 2D rig view + telemetry plots + the wear-trend window), so a sim run
looks just like driving the real rig. On real hardware it stays headless for long
unattended runs. `run_lifecycle.py` flags: `--view` (force the live GUI on hardware),
`--headless` (console only even in sim), `--sim` (force the modeled rig), `--speed N`,
`--scenario NAME`. Plain `pixi run test` covers the firmware port + scenarios + the
rig view against the AnalyticPlant.

### Fault scenarios (`scenarios.py`)

`clean` · `hall_slip` · `wear` · `missed_hall` · `stall` · `glitch` — each
reproduces a failure mode the lifecycle aborts exist to catch (slip-span,
sustained-Iq dwell, overtravel backstop, position stall, glitch rejection).

## Time model

Real-time by default (sim clock locked to wall clock) so the controller's
wall-clock detectors (6 s stall, 0.5 s Iq dwell, 1 Hz keepalive) stay valid.
`--speed N` / `FOC_SIM_SPEED=N` accelerates for analysis-only/long runs; the
firmware watchdog window is scaled by the speed so the wall-clock keepalive
doesn't false-trip.

## Calibration (vs the CLAUDE.md hardware oracle)

Default `PlantConfig` reproduces, with no extra tuning:
travel **≈200 rad**, `v_safe` **≈109 rad/s**, MAX hall latch ≈ −100, peak
**|Vq| ≈ 1.7 V** of the 3 V limit at a stroke. Tune `PlantConfig` (R, KV,
inertia, friction, hall/hard-stop geometry, noise) in `plant.py` for other
regimes.

## Files

`plant.py` (PlantConfig + AnalyticPlant + fault hooks) · `soft_firmware.py`
(main.cpp port) · `sim_serial.py` (serial seam + clock) · `scenarios.py`
(faults). The 2D view lives one level up (`panel/rig_view.py`) because it
represents the real rig too.
