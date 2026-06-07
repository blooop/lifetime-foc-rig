# Plan — Comprehensive physics-based simulation (Genesis as the plant)

Goal: run the **entire real software stack** (`foc_panel.py` GUI, `lifecycle.py`
controller, auto-tuner, CSV logging, `plot_lifecycle.py` analysis) against a
physics simulation instead of the MKS ESP32 board — with **no hardware** — to
evaluate and validate testing regimes and analysis. Genesis supplies the
mechanical plant; a Python "soft-ESP32" supplies the firmware/control/protocol
behavior. They meet at the one seam the whole stack already uses: the serial port.

This is the *high-fidelity* sibling of the lightweight behavioral sim (being built
separately). Same seam, swappable plant. See **§10** for how they relate.

> **Branch:** `sim-genesis` (worktree at `../lifetime-foc-rig-sim`), off `main`,
> kept separate from the `add-test-suite` branch where the lightweight tests are
> being built.
>
> **STATUS: IMPLEMENTED ✅** — all phases done (§11). Code in `panel/sim/`
> (usage: `panel/sim/README.md`). 15 tests green (`pixi run test`); Genesis
> validated against the oracle and end-to-end through the full GUI/lifecycle
> stack (home → cycle → log → park → resume).

---

## 0. Phase-0 smoke test — DONE ✅ (gate passed)

Verified in the `sim` pixi env on this machine (CPU; Quadro M2200 / Maxwell is
unsupported by Genesis's GPU path, as expected):

- **Genesis 1.1.0** imports, `gs.init(backend=gs.cpu)` works.
- Built a 1-DOF torque-controlled revolute rotor (`gs.morphs.MJCF`, one `hinge`
  joint), applied constant torque, stepped — shaft accelerates correctly
  (0 → 3.8 rad/s over 2 s at the modeled inertia). **The plant contract
  (torque-in → angle/velocity-out) works.** (`panel/sim/smoke_test.py`)
- **Throughput:** ~0.52 ms/step, ~1,920 steps/s for the 1-DOF scene on CPU.
  (`panel/sim/throughput_probe.py`)

**Install gotchas found & fixed (now baked into §3):**
1. Genesis does **not** auto-install torch (peer dep) → add it explicitly,
   CPU-only wheel via the PyTorch index.
2. Its `pymeshlab` dep only ships `manylinux_2_35` wheels → raise the pixi
   `system-requirements` libc floor (system glibc here is 2.39).
3. Conda's pinned `numpy` conflicts with Genesis's pypi deps → give the sim
   environment `no-default-feature = true` so it doesn't inherit it.

**Time-model consequence (revises §6):** real-time factor by physics rate —
2000 Hz → 0.96× (fails real-time), **1000 Hz → 1.92×**, **500 Hz → 3.84×**,
250 Hz → 7.67×. So target **500 Hz–1 kHz** physics, not 2 kHz.

---

## 1. The seam (why this is clean)

Every Python consumer talks to hardware through one object — `SerialWorker`
(`panel/foc_panel.py:46`). It:
- opens a `serial.Serial()` (`:92`), sets `port/baudrate/timeout/dtr/rts`,
- writes Commander lines `…\n` (`:116`),
- toggles `rts` to **reset the board** (`reset_board`, `:76`),
- parses three inbound streams (`_read_loop`, `:110`):
  - **7-var monitor** (tab-sep): `target Vq Vd Iq(mA) Id(mA) vel angle` → panel reads idx 0,1,3,5,6
  - **`E\t…`** endstop line (20 Hz): `minTrig maxTrig homed phase posFromHome backstopFired`
  - **`S\t…`** slip edge: `which shaft_angle`
  - free-text logs; `"Motor ready"` raises the `ready` signal.

`lifecycle.py` never touches serial directly — only `worker.send()` + the
`telem/endstop/slip/ready` Qt signals. **So a simulator only has to be a serial
port that speaks this protocol.** Drop in a `SimSerial` that quacks like
`serial.Serial`, and the GUI/lifecycle/analysis/auto-tune/reconnect logic all run
unmodified.

---

## 2. Architecture

```
┌─────────────────────────────────────────────────────────────┐
│ REAL, UNCHANGED: foc_panel.py · lifecycle.py · plot_lifecycle │
│                       (PyQt + analysis)                       │
└───────────────┬─────────────────────────────────────────────┘
                │ SerialWorker  ← only injection point (FOC_SIM=1 → SimSerial)
┌───────────────▼─────────────────────────────────────────────┐
│ SimSerial            quacks like serial.Serial               │
│   .write(cmd) → command queue   .readline() ← telemetry queue│
│   .rts=True/False → board reset (re-run "calibration")       │
├──────────────────────────────────────────────────────────────┤
│ SoftFirmware (Python port of firmware/src/main.cpp)          │
│   • Commander M/E/P parsing   • velocity & angle PID + LPF   │
│   • motion profile (trapezoid) • endstop debounce + homing FSM│
│   • travel limits + 20% backstop • serial watchdog            │
│   • FilteredAS5600 glitch filter • monitor/E/S/boot emitters  │
│   • electrical model: Vq → Iq=(Vq−Ke·ω)/R → τ=Kt·Iq           │
│   runs loop() at a fixed control rate on the SIM clock        │
├──────────────────────────────────────────────────────────────┤
│ GenesisPlant         the physics  (verified: gs.cpu, 1.1.0)  │
│   • 1 actuated revolute DOF = motor shaft (1:1 with           │
│     firmware shaft_angle; ~200 rad travel)                    │
│   • reflected carriage inertia, Coulomb + viscous friction    │
│   • joint-limit contact at the physical HARD STOPS            │
│   • hall magnets at true min/max angles (sensor windows)      │
│   • AS5600 model: wrapped angle + injectable I2C glitches     │
│   • fault injection: hall slip, friction/wear ramp, missed    │
│     hall, binding, sensor dropout  (§7)                       │
└──────────────────────────────────────────────────────────────┘
```

**Division of labor:** Genesis integrates the *mechanical* dynamics (inertia,
friction, contact); SoftFirmware does *everything the ESP32 does* — the actual
control loops, limits, telemetry, and the Vq→torque electromechanical model. We
deliberately do **not** offload control to Genesis's built-in PD actuator: the
whole point is to validate the real firmware's PID/profile/limit/abort behavior,
so that logic must be the firmware port, not the engine's solver. Genesis is fed
a **torque** each control step (`control_dofs_force`) and returns angle/velocity
(`get_dofs_position`/`get_dofs_velocity`) — verified working in Phase 0.

---

## 3. Environment & install  (verified)

- **Backend: CPU.** GPU here is a Quadro M2200 (Maxwell, CC 5.2) — unsupported by
  Genesis's GPU path. `gs.init(backend=gs.cpu)`. 1-DOF runs real-time with
  headroom on CPU (§0). If a modern CUDA GPU appears, flip to `gs.gpu`.
- **Isolate the heavy deps** in a pixi `sim` feature/environment so the default
  `pixi install` (PlatformIO + PyQt) stays lean. Working config:
  ```toml
  [feature.sim.system-requirements]
  libc = "2.35"                 # accept the manylinux_2_35 pymeshlab wheels

  [feature.sim.dependencies]
  python = "3.12.*"

  [feature.sim.pypi-dependencies]
  genesis-world = "*"
  torch = { version = "*", index = "https://download.pytorch.org/whl/cpu" }

  [environments]
  sim = { features = ["sim"], no-default-feature = true }
  ```
  `no-default-feature` is load-bearing: it dodges the conda `numpy` pin that
  conflicts with Genesis's pypi deps. Pin the exact `genesis-world`/`torch`
  versions once stable (Phase 0 used genesis 1.1.0).
- **GUI coexistence (open item):** the GUI and sim run in one process, so the
  `sim` env will eventually also need PyQt5 alongside Genesis. Phase 0 kept the
  env minimal (python+genesis+torch). When wiring the GUI, add `pyqt`/`pyqtgraph`
  to the sim feature and re-check the numpy/Qt solve; if it fights, fall back to
  running the GUI in the default env and the sim as a separate process bridged by
  a PTY/socket `SimSerial`. Decide in Phase 1.

---

## 4. Components to build  (`panel/sim/`)

| File | Responsibility |
|------|----------------|
| `sim_serial.py` | `SimSerial`: `serial.Serial` look-alike (`port/baudrate/timeout/dtr/rts`, `open/close/write/readline/in_waiting`). `write` → cmd queue; `readline` → telemetry queue (respects `timeout`); `rts`-rising → `firmware.reset()`. Owns the sim thread/clock. |
| `soft_firmware.py` | Faithful port of `main.cpp`: Commander `M/E/P` parser, `velocity`/`angle`/`torque-V` modes, SimpleFOC velocity PID (P=0.05 I=1 D=0, `output_ramp`, `LPF Tf=0.02`) and angle P (=10), `applyMotionProfile`, `enforceTravelLimits` + backstop + `v_safe`, homing FSM (`HOME_SEEK_MIN→MAX→CENTER`), `Endstop` debounce + `just_triggered`, `FilteredAS5600` time-aware glitch filter, watchdog (3 s), boot/undervoltage/calibration text + `Motor ready`, monitor (downsample) + `E`/`S` emitters. **Electrical model**: `Iq=(Vq−Ke·ω)/R`, `τ=Kt·Iq`, `Kt=Ke=9.549/KV`. |
| `genesis_plant.py` | Genesis scene: one revolute joint (shaft), reflected inertia, Coulomb+viscous friction, joint-limit contact at hard stops; hall trigger windows; AS5600 wrap + glitch hooks; `apply_torque(τ)`, `step(dt)`, `read_angle()/read_velocity()/read_halls()`; fault-injection API (§7). Optional viewer for a live carriage view. |
| `plant_config.py` | One dataclass of physical params: motor `R`, `KV`, pole pairs, supply V, voltage/vel limits, inertia, friction coeffs, hall positions, hard-stop positions, travel (~200 rad), noise levels. Defaults tuned to the hardware-verified numbers (§8). |
| `run_sim.py` | Entry point: build plant+firmware+SimSerial, then launch GUI or headless lifecycle against it; flags for fault scenarios and real-time factor. |
| `scenarios/*.py` (or TOML) | Named fault scenarios (§7) for repeatable regime/analysis validation. |

**Injection into existing code (tiny):** in `SerialWorker.run` swap the one
`serial.Serial()` for `SimSerial()` when `FOC_SIM=1` (env) — one guarded line.
Add `--sim` passthrough to `foc_panel.py`/`run_lifecycle.py` that sets the env.
No other change to the real stack.

(`smoke_test.py` + `throughput_probe.py` already live here from Phase 0.)

---

## 5. Fidelity decisions (modeled exactly vs approximated)

**Exact (ported from firmware, because the tests exercise them):**
- Velocity PID + velocity LPF, angle P, `output_ramp`; Vq saturation at
  `voltage_limit`; `MLV/MLU/MVP…/MAP/MVF/MMD` runtime effects.
- Motion profile (trapezoidal vel + full trapezoidal angle move), re-seed on mode
  change.
- Endstop debounce (3 reads), active-low, `just_triggered` edges.
- Homing FSM incl. the wrong-direction abort and the angle-mode centering.
- Travel limits, 20% overtravel backstop (disable-on-trip, no latch), `v_safe`
  derivation, soft limits.
- Glitch filter time-aware rejection (`max_speed·dt + floor`), armed-after-init.
- Watchdog (3 s) + the GUI/lifecycle 1 Hz keepalive interaction.
- Telemetry exactly: 7-var monitor with **Iq in mA** (`c.q*1000`), `E` 20 Hz,
  `S` per edge, boot/calibration/`Motor ready`, reset-on-RTS.

**Physics from Genesis (the value-add over a behavioral model):**
- True 2nd-order shaft dynamics with reflected carriage inertia.
- Coulomb + viscous friction (and its growth = wear).
- **Contact at the hard stops** — so a *missed hall* produces a physically real
  ram into the stop, and we can confirm the firmware backstop catches it first.

**Approximated / out of scope:** per-phase commutation, SVPWM, Clarke/Park,
thermal, real I2C bus timing. Voltage-mode FOC means torque ≈ `Kt·(Vq−Ke·ω)/R`;
we model that directly (matches what the firmware's monitor reports and what the
torque estimate uses), not the 3-phase electrical detail.

---

## 6. Time model (decision — now data-backed by §0)

**Default: real-time (sim clock locked to wall clock).** The real controller's
detectors are on `time.monotonic()` (6 s stall, 0.5 s Iq dwell, 1 Hz heartbeat),
while the firmware's `millis()` is the sim clock. Locking them keeps every
wall-clock timer valid — essential for faithfully validating the abort logic.

- **Physics/control rate: 500 Hz–1 kHz** (Phase 0: 1 kHz → 1.9×, 500 Hz → 3.8×
  real-time; 2 kHz only managed 0.96× and would fall behind). 500 Hz–1 kHz is
  ample for voltage-mode velocity/position control of a 1-DOF rig — we don't
  simulate kHz commutation. Set `monitor_downsample` so telemetry lands ~20 Hz
  (e.g. ÷50 at 1 kHz) to match the hardware feel.
- Optional `--speed N` accelerates the sim clock for analysis-only/long-endurance
  runs (caveat: the controller's wall-clock detectors then see compressed
  sim-time — use for CSV/analysis shakeout, not abort-timing validation). Bulk
  fast runs are better served by the lightweight sim (§10).

---

## 7. Fault-injection catalog — this is what "validates the testing regime"

Each fault maps to a lifecycle abort/feature so we can prove the regime catches
what it's meant to, which is unsafe/destructive to reproduce on real hardware:

| Inject | Exercises / should trigger |
|--------|----------------------------|
| **Hall position slip** (drift trigger angle over cycles) | `S`-line slip span + per-end drift tracking; slip-span abort; wear-trend plots |
| **Friction / load ramp** (wear) | `τ(pos)` binning, `E_stroke=∫τ·dθ` trend; sustained-Iq abort (0.5 s dwell) |
| **Single Iq spike** (accel/reversal) | confirms the dwell filter does **not** false-trip on transient spikes |
| **Missed / failed hall** (no trigger once) | carriage overruns → **20% backstop** disable-on-trip; Genesis hard-stop contact as the last resort |
| **Carriage bind / stall** (friction → motion stops) | position-progress stall detector (>6 s no advance) |
| **Inverted seek direction** | homing wrong-direction abort ("hit MAX while seeking MIN") |
| **AS5600 glitch / dropout** | `FilteredAS5600` rejection without cutting out legit fast motion |
| **Serial silence** (GUI idle) | 3 s comms watchdog disable; keepalive prevents it |
| **Mid-run board reset** (toggle RTS) | reconnect → `ready` → lifecycle re-home/resume path |

Scenarios are named + parameterized so a regime change can be re-run identically.

---

## 8. Validation / acceptance (does the sim behave like the rig?)

Oracle = the hardware-verified numbers in `CLAUDE.md`. The sim is "good enough"
when, with default params, it reproduces:
- Clean homing log: seek MIN → MAX → center; **travel ≈ 200 rad**, center = 0.
- **`v_safe` ≈ 109 rad/s** at accel 300, 20% margin (derived, not hand-set).
- Velocity tracking with **Vq peaking ~1.6 V** of the 3 V limit at 60 rad/s
  (plenty of headroom to 100 rad/s) — checks `R/KV` defaults.
- A full lifecycle: home → cycle hall-to-hall → per-cycle slip/τ/E logged to
  `summary.csv`+`profile.csv` → **park at center before disable** → resume after
  a reset. `plot_lifecycle.py` opens the run and renders trends/heatmap.
- Each §7 fault produces exactly its expected abort/feature, and no false trips
  on clean runs.

Add a small `panel/tests/test_sim_*.py` (offscreen Qt, like the existing suite)
asserting the protocol round-trips and each fault → expected abort.

---

## 9. Risks & mitigations

- ~~Genesis CPU install / API churn~~ → **retired by Phase 0** (installs + runs on
  CPU; config pinned in §3). Still pin exact versions before relying on it.
- **Real-time CPU budget** → measured: fine at 500 Hz–1 kHz (§0/§6). A heavier
  scene (visualizer, contact-rich) costs more; re-measure if added.
- **Engine lock-in** → the SoftFirmware + SimSerial seam is engine-agnostic. If
  Genesis ever blocks us, a SciPy ODE plant with the same
  `apply_torque/step/read` interface drops in with no change above it. (Genesis
  is the plant, not the contract.)
- **Porting drift from `main.cpp`** → keep the port structured 1:1 with the
  delimited firmware sections; cross-check against the firmware native tests
  (`firmware/test/`, `control_logic.h`) where they overlap; the §8 oracle is the
  backstop.
- **Rotary vs linear modeling** → start rotary (1:1 with firmware shaft_angle,
  no transmission-ratio bugs — Phase 0 used exactly this); add a prismatic+ratio
  refinement only if a test needs true linear contact in metres.

---

## 10. Relationship to the lightweight sim (other agent, `add-test-suite`)

Same `SimSerial` seam, two interchangeable plants:
- **Lightweight** (behavioral, no engine): fast, deterministic, faster-than-real-
  time bulk runs (5000-cycle endurance, CI, analysis shakeout).
- **Genesis** (this plan): real contact dynamics, friction/inertia integration,
  mechanical fault realism, live visualization; real-time fidelity.

**Coordinate the plant interface before §4:** if the lightweight side defines a
plant contract (`apply_torque(τ)` / `step(dt)` / `read_angle()` /
`read_velocity()` / `read_halls()`), `GenesisPlant` implements the **same** one so
they're drop-in swappable behind one SoftFirmware. The firmware port itself
should ideally be shared between both, not duplicated.

---

## 11. Phasing / milestones

0. ~~**Smoke-test Genesis** on CPU in a pixi `sim` env~~ — **DONE ✅** (§0).
1. ~~**SimSerial + SoftFirmware + plant**; prove the seam end-to-end; GUI/sim env
   coexistence~~ — **DONE ✅** (env solves with PyQt5 + Genesis together).
2. ~~**GenesisPlant**: revolute DOF, inertia, friction, hard-stop contact, hall
   windows; tuned to the §8 oracle~~ — **DONE ✅** (travel 200.15, v_safe 109.59,
   |Vq| 1.68 V — matches oracle out of the box).
3. ~~**Full control fidelity**: PID/profile/limits/backstop/homing/glitch/watchdog~~
   — **DONE ✅** (full main.cpp port, 10 deterministic tests).
4. ~~**Lifecycle end-to-end**: home→cycle→log→park→resume~~ — **DONE ✅** (summary
   + profile CSVs written; resume-on-reset exercised).
5. ~~**Fault injection + scenarios + tests**~~ — **DONE ✅** (6 scenarios, 5 tests).
6. ~~**Docs**~~ — **DONE ✅** (`panel/sim/README.md`, pixi `-e sim` tasks, CLAUDE.md).

**Gotchas found during the build** (all fixed + documented in
`panel/sim/README.md`): torch is a Genesis peer-dep (not auto-installed); MJCF
angles default to degrees (joint limits were ±145° not ±145 rad); Genesis scenes
default to multiple substeps per `step()` (10×-light inertia) — pinned to
`substeps=1, dt=control_dt`; the DOF needs explicit force control; Genesis/torch
can double-free at interpreter exit (`run_sim.py` `os._exit`-s after the run).

---

## 12. Out of scope
3-phase commutation/SVPWM detail, thermal, true I2C timing, the
firmware-flash/PlatformIO path (unchanged), multi-motor (rig is 1-DOF).
