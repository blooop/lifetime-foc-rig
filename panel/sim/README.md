# Physics-based simulation (Genesis plant behind the serial seam)

Run the **entire real software stack** — `foc_panel.py` GUI, `lifecycle.py`,
auto-tuner, CSV logging, `plot_lifecycle.py` — against a physics simulation
instead of the MKS ESP32 board, with **no hardware**. Used to evaluate and
validate testing regimes and analysis. Design rationale: `../../GENESIS_SIM_PLAN.md`.

## How it works

The whole stack talks to hardware through one object — `SerialWorker`
(`foc_panel.py`). Set `FOC_SIM=1` and it opens a **`SimSerial`** instead of a real
port; nothing else in the GUI/lifecycle/analysis changes.

```
GUI / lifecycle / analysis   (unchanged)
        │  SerialWorker  ── FOC_SIM=1 ──▶ SimSerial
        ▼
SimSerial         serial.Serial look-alike; a thread runs the sim clock
  SoftFirmware    Python port of firmware/src/main.cpp (control, homing,
                  limits, backstop, watchdog, glitch filter, telemetry,
                  Vq→Iq→τ electrical model)
  Plant           the mechanism: 1 rotary DOF, inertia, friction, hall
                  endstops, hard stops
                    • GenesisPlant  — Genesis physics (real contact dynamics)
                    • AnalyticPlant — pure-Python 1-DOF (fast, dep-free; tests/CI)
```

`SoftFirmware` reproduces the firmware *behavior*; Genesis only integrates the
mechanical dynamics and the hard-stop contact. The firmware is fed angle/velocity
and applies a torque each control tick.

## Usage

From `panel/` (Genesis runs in the isolated `sim` pixi env):

```
pixi run -e sim gui-sim                       # GUI vs the Genesis plant
pixi run -e sim gui-sim --viewer              # + live Genesis carriage view
pixi run -e sim gui-sim --scenario wear       # GUI with a wear fault injected
pixi run -e sim lifecycle-sim --cycles 30 --vmeas 20 --speed 2
python run_sim.py --plant analytic --lifecycle --cycles 50 --speed 10   # no Genesis, fast
```

`run_lifecycle.py --sim [--plant analytic|genesis] [--speed N] [--scenario NAME]`
also works for the headless runner. Plain `pixi run test` covers the firmware port
+ scenarios against the AnalyticPlant (no Genesis needed).

### Fault scenarios (`scenarios.py`)

`clean` · `hall_slip` · `wear` · `missed_hall` · `stall` · `glitch` — each
reproduces a failure mode the lifecycle aborts exist to catch (slip-span,
sustained-Iq dwell, overtravel backstop, position stall, glitch rejection). See
`GENESIS_SIM_PLAN.md` §7 for the fault→abort mapping.

## Time model

Real-time by default (sim clock locked to wall clock) so the controller's
wall-clock detectors (6 s stall, 0.5 s Iq dwell, 1 Hz keepalive) stay valid.
`--speed N` accelerates for analysis-only/long runs; the firmware watchdog window
is scaled by the speed so the wall-clock keepalive doesn't false-trip. Genesis CPU
runs real-time with headroom at **500 Hz–1 kHz** physics (≈3.8×/1.9× real-time;
2 kHz only manages ~0.96× — don't).

## Calibration (vs the CLAUDE.md hardware oracle)

Default `PlantConfig` reproduces, with no extra tuning:
travel **≈200 rad**, `v_safe` **≈109 rad/s**, MAX hall latch ≈ −100, peak
**|Vq| ≈ 1.7 V** of the 3 V limit at a stroke. Tune `PlantConfig` (R, KV, inertia,
friction, hall/hard-stop geometry, noise) in `plant.py` for other regimes.

## Genesis gotchas (baked into the code — don't regress)

- **torch is a peer dep** of `genesis-world`, not auto-installed → the `sim` pixi
  feature pins the CPU-only torch wheel via the PyTorch index.
- **`pymeshlab`** only ships `manylinux_2_35` wheels → the `sim` feature raises the
  pixi `system-requirements` libc floor (system glibc 2.39).
- the `sim` env is **`no-default-feature`** so it dodges the conda numpy pin that
  conflicts with Genesis's pypi deps (PyQt5 etc. are re-declared in the feature).
- **MJCF angles default to degrees** → `plant.py` sets `<compiler angle="radian"/>`
  (else the ±145 *rad* joint limits become ±145°≈±2.53 rad and the carriage freezes).
- **Genesis scene defaults to multiple substeps** per `scene.step()` (~10× extra
  physics per call → 10×-light effective inertia) → we pin
  `SimOptions(dt=control_dt, substeps=1)`.
- the DOF needs **force control** (`set_dofs_kp/kv=0`, wide `force_range`) so
  `control_dofs_force` isn't fought by a default PD controller.

### Known issue (cosmetic)

A headless **Genesis** lifecycle run can print `double free or corruption` during
Genesis/torch teardown **after** the run completes and `FINISHED` prints — logs
are flushed every cycle, so nothing is lost and `plot_lifecycle.py` reads the run
fine. It's a Genesis C++ teardown race, not a logic error, and does **not** occur
with `--plant analytic`. Prefer the analytic plant for bulk/CI/unattended runs and
Genesis for interactive fidelity; `run_sim.py` `os._exit`s to keep the GUI-close
path clean.

## Files

`plant.py` (Plant interface + Genesis/Analytic) · `soft_firmware.py` (main.cpp
port) · `sim_serial.py` (serial seam + clock) · `scenarios.py` (faults) ·
`run_sim.py` (launcher, in `panel/`) · `smoke_test.py` / `throughput_probe.py`
(Phase-0 gate) · `validate_genesis.py` (Genesis-vs-oracle check).
