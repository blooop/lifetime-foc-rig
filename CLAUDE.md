# CLAUDE.md — MKS ESP32 FOC workspace

Tooling + a custom GUI for driving an **MKS ESP32 FOC V2.0** board with the
SimpleFOC library. This is a working setup; the notes below are the load-bearing
facts to keep it working — follow them.

## Hardware
- Board: MKS ESP32 FOC V2.0 (ESP32). Enumerates as a **CH340** — `/dev/ttyUSB0` on Linux, `/dev/tty.usbserial-*` / `/dev/tty.wchusbserial*` on macOS (auto-detected by VID; needs a real *data* USB cable).
- Motor: **A2212/13T 1000KV**, 12N14P = **7 pole pairs** → `BLDCMotor(7)`.
- Sensor: **AS5600** magnetic encoder, I2C, on connector **J5** (Motor 0): **SDA=GPIO19, SCL=GPIO18**.
  - Wiring (soldered + noise-proofed — keep it that way): VCC→3.3V, GND→GND, SDA→SDA, SCL→SCL, **DIR tied to GND**, 100nF cap across VCC↔GND at the chip, GND twisted with SDA/SCL, short leads away from the motor phase wires. A diametric magnet sits over the chip.
- Driver: Motor 0 phase pins `BLDCDriver3PWM(32, 33, 25, 12)`, motor wires on the A0/B0/C0 terminal.
- Power: needs a **separate 12 V+ motor supply** (board has an 11.1 V undervoltage gate; USB alone ≈4.9 V won't spin it). Flash over USB; power 12 V on to calibrate/run.
- Endstops: two **HW-477 (A3144 digital hall)** modules on the FREE **sensor port 1**, powered from that port's **GND + 3.3 V** rail. Onboard 10k pull-up references 3.3 V → idles HIGH, **ESP32-safe (NOT 5 V logic)**. Output is **active-low** (LOW = magnet present = triggered), non-latching.
  - **Endstop A (MIN / home) → GPIO 5** — **ESP32 strapping pin: must read HIGH at boot**, so the MIN switch must be physically CLEAR at power-up. Firmware only ever configures it as INPUT. (Pins matched to as-built wiring — verified on hardware: MIN-end module is on GPIO 5.)
  - **Endstop B (MAX) → GPIO 23** (plain GPIO).

## Layout
- `firmware/` — PlatformIO project (`board=esp32dev`, `framework=arduino`, SimpleFOC pinned to `Arduino-FOC.git#v2.2.1`). Active sketch: `src/main.cpp`.
- `panel/foc_panel.py` — custom PyQt5 control panel (GUI); `panel/lifecycle.py` + `panel/run_lifecycle.py` — endurance test.
- Original makerbase docs/schematics/examples are a *separate* read-only reference checkout (location varies by setup).

## Toolchains — pixi (portable: Linux + macOS, Intel + Apple Silicon)
Everything (PlatformIO + Python/PyQt5/pyqtgraph/pyserial/numpy) is managed by **pixi**
(`pixi.toml` at the repo root) — no system Python or venvs. One-time: **`pixi install`**.
Python is pinned to 3.12 (the conda-forge PlatformIO is fine there; it was the 3.14
pixi-global build that broke). The old `~/.venvs/pio` and `~/.venvs/focstudio` are
deprecated/fallback only — don't add new dependencies on them.

## Commands
Run via pixi tasks from the repo root. **Serial port is auto-detected** (CH340 by USB
VID); override with `FOC_PORT=<dev>` or `--port`/`--upload-port` if multiple devices.
```
pixi run install-driver   # CH340 USB-serial driver (macOS; no-op on Linux)
pixi run build       # compile firmware
pixi run flash       # build + flash over USB (auto-detect port)
pixi run monitor     # serial monitor @115200 (close the panel first)
pixi run panel       # launch the GUI (12 V on so the board calibrates)
pixi run lifecycle --cycles 5000 --vmeas 3.0   # headless endurance run
pixi run plot        # view a finished run (newest, or `pixi run plot <run_dir>`)
```
Only one program can own the serial port at a time — close the GUI before `monitor`/`lifecycle`.

## Current firmware (`firmware/src/main.cpp`) — safe, voltage-based
- Modes: **velocity** (default), **angle**, **torque-voltage**. Voltage-based control only.
- `voltage_limit=1.0 V`, `velocity_limit=20`, `PID_velocity P=0.05 I=1.0 D=0`, `LPF_velocity.Tf=0.02`, `P_angle.P=10`.
- I2C **100 kHz** with `I2Cone.setTimeOut(25)` (a noisy read fails in 25 ms instead of blocking ~1 s).
- **Armed-after-initFOC glitch filter** (`FilteredAS5600`): disarmed during alignment (else it trips `Failed to notice movement`), armed while running. Do not make it always-on. Rejection is **time-aware** (`max_speed·dt + floor_step`), not a fixed angle step — a fixed `0.6 rad` step falsely rejected legit motion at high speed (≈20 rad/s) when a serial/I2C stall stretched the loop, briefly cutting out the motor. Keep it time-scaled.
- **Boots DISABLED** (`motor.disable()` after `initFOC`) — user enables explicitly.
- Streams the **full 7-variable monitor set** at `monitor_downsample=100` (GUIs read velocity at stream index 5, angle at index 6 — a 3-var monitor leaves live feedback blank). Index 3/4 carry `Iq`/`Id` from the current sense (see below — **not yet usable**).
- **Read-only current sense**: `InlineCurrentSense(0.01f, 50.0f, 39, 36)` (Motor-0 inline shunts, makerbase example #14), linked via `current_sense.init()` + **`current_sense.skip_align = true`** + `motor.linkCurrentSense()` before `initFOC()`. `skip_align` is **required**: the current-sense `driverAlign()` fails on this board (small/noisy shunt current at the align voltage) and a failure aborts the WHOLE `initFOC` → no commutation → railed current on enable. `torque_controller` stays `voltage` (never `foc_current` — runaway). **Status: the reading is currently UNUSABLE** — at idle (zero real current) `Iq` reads ±impossible values (~10 A mean, ±25 A swing), an ESP32 ADC bring-up problem. So **torque is the Vq model estimate** for now (`τ = Kt·(Vq−Ke·ω)/R`); fixing the ADC for real measured `Iq` is a follow-up. Motor control itself is unaffected (verified: spins & tracks target with the sense linked).
- Exposes the SimpleFOC **Commander** under motor letter **`M`**. Clean calibration log: `sensor_direction==CCW`, `PP check: OK!`, `Zero elec. angle ~5`.
- **Endstops / homing / travel limits** live in one delimited section of `main.cpp` (`ENDSTOPS / HOMING / LIMITS`). Read both each loop (after `loopFOC`, before `move`), **debounced** (3 consecutive reads). `enforceTravelLimits()` clamps `motor.target` out of any triggered limit each loop. The hard-endstop clamp keys off **`g_home_dir`** (sign of velocity that heads toward MIN; **default `+1` for this rig** — `-1` drove the wrong way), so limiting and homing share one physically-correct direction convention; soft limits are position-based as a homed backstop. Backing away is always allowed. Non-blocking **3-phase auto-home state machine**: seek MIN → seek MAX → drive to the measured **center** (which becomes position 0); records the endstop positions as soft limits (left disabled). Default seek speed **20 rad/s**. If seek-MIN reaches the MAX switch instead (inverted direction), it **aborts** rather than ramming the endstop.
- **Motion profile** (`MOTION PROFILE` section, `applyMotionProfile()` in the loop before `move`): a general acceleration-limited setpoint generator so commanded motion is smooth (no slamming reversals). Velocity/torque modes get a trapezoidal-velocity slew; angle mode gets a full trapezoidal position move (accel to `velocity_limit`, decelerate to a stop). It intercepts whatever Commander/homing/limits wrote to `motor.target`, shapes it, writes it back. Re-seeds from the live shaft on any control-mode change (no jumps). Default accel **50 rad/s²** (`g_max_accel`). Torque mode passes through unshaped (keeps the autotuner intact). This replaced the earlier homing-only `output_ramp` softening. **Homing requires the motor already enabled and both endstops enabled** (refuses otherwise). Endstop pins, active-level, enable, seek speed/direction, and soft limits are runtime-configurable (constants for defaults, `E` commands at runtime).
- Streams a **second telemetry line** distinct from the 7-var monitor: `E\t<minTrig>\t<maxTrig>\t<homed>\t<homePhase 0=idle,1=seekMin,2=seekMax,3=center>\t<pos-from-home>\t<backstopFired 0=none/1=pastMin/2=pastMax>` at ~20 Hz. **Streams from boot and during the undervoltage wait (USB-only power), motor disabled** — so endstop wiring is verifiable by waving a magnet before any motion.
- **5% overtravel backstop** (lifecycle safety): auto-armed at the end of a full `EH` home (NOT a bare `EZ` — it has no travel reference). Lines = endstop shaft-angle ± `OVERTRAVEL_FRAC`(=0.05)·travel, recomputed each home so they track slip. If a missed/failed hall lets the carriage run past a line **while heading further past** (backing away is allowed), `enforceTravelLimits()` calls `motor.disable()` — **disable-on-trip, NO latch** (re-enable manually + re-home), not defeatable. On arming it also derives `g_v_safe = sqrt(2·accel·0.5·0.05·travel)` and clamps velocity-mode `|target|` so a reverse-on-trigger stop stays well inside the 5% margin. Purpose: the hall switches are the **working** limits (hit every cycle); separate **physical hard stops sit beyond them and must never be reached**.
- **Hall-edge slip latch**: `Endstop.just_triggered` (rising edge) → `loop()` emits `S\t<which 0=min/1=max>\t<shaft_angle>` once per edge (continuous angle latched at ~kHz, immune to the 20 Hz E-line jitter). The panel owns the cycle counter; this is the precise slip/“motor-rotation-per-stroke” signal.
- **Serial-heartbeat watchdog**: any command pets `g_last_cmd_ms`; if the motor is enabled **and not homing** and no command arrives for `WATCHDOG_MS`(=3 s), `loop()` disables the driver. Suppressed during homing and while idle-disabled so it won't false-trip. The panel sends a ~1 Hz `EK` keepalive.

## Commander serial protocol (what the GUI sends)
`MC0`=torque-voltage, `MC1`=velocity, `MC2`=angle · `M<x>`=target · `ME1`/`ME0`=enable/disable ·
`MLU<v>`=voltage limit, `MLV<v>`=velocity limit · `MVP/MVI/MVD<x>`=velocity PID · `MAP<x>`=angle P ·
`MVF<x>`=velocity LPF Tf · `MMD<n>`=monitor downsample. Opening the port auto-resets the board (re-calibrates) — harmless.
Endstops use a second Commander letter **`E`**: `EH`=auto-home (MIN→MAX→center), `EX`=abort homing, `EZ`=zero here, `EK`=watchdog keepalive (no side effect), `ES<v>`=seek speed,
`ED-1`/`ED1`=seek-MIN direction (− / + velocity), `EAE/EAL/EAP`+`EBE/EBL/EBP`=endstop A/B enable/active-low/pin,
`ELE<0|1>`=soft-limit enable, `ELN<v>`/`ELX<v>`=soft min/max travel (home-relative rad).
Motion profile uses a third letter **`P`**: `PA<v>`=acceleration [rad/s²], `PE<0|1>`=enable/disable profiling.

## GUI (`panel/foc_panel.py`)
PyQt5 + pyqtgraph. The left control column is in a **scroll area** (never forces the window
taller than the screen); **PID tuning is a collapsible group, collapsed by default**.
Mode radio buttons, target slider, Enable/STOP, limit fields, (collapsible) PID-tuning fields,
two live plots (target/vel/angle, and torque), a measured-torque readout, and a
**relay-feedback velocity auto-tuner**.
- **Torque is a model estimate** from `Vq`: `Iq=(Vq−Ke·ω)/R`, `τ=Kt·Iq`, `Kt=Ke=9.549/KV`. Set
  **Phase resistance R** and **KV**; a **Kt-override** field is a calibration hook (0 = use KV). (The
  read-only current sense is linked but its ADC reading isn't usable yet — bring-up TODO.) `SerialWorker`
  auto-reconnects on a serial drop (re-homes/resumes via the `ready` signal).
- **Auto-tune** only applies to the velocity loop (torque-voltage has no PID loop; the current loop is too fast to tune over serial).
- **Motion profile** group: enable trapezoidal profiling + acceleration [rad/s²] field (`PE`/`PA`). Lower accel = gentler ramps/reversals.
- **Endstops & homing** group: live MIN/MAX/homed indicators (green=clear, red=triggered) + a safety banner that turns red and names the active endstop when motion is limited (and shows the auto-home phase while running; turns dark-red on a backstop trip); per-endstop enable/pin/active-low; seek speed + seek-MIN direction; **Auto-home (MIN→MAX→center)** (refuses unless enabled) and **Set zero here** buttons; soft-limit enable + min/max travel fields. Position-from-home shows in the Live box. Indicators update from boot, so wiring can be checked before enabling the motor.
- **Lifecycle test** group (`panel/lifecycle.py`, `LifecycleController`): target cycles, measure speed, Iq/slip abort thresholds, Start/Stop. Drives the rolling-drive endurance test — homes, then cycles hall-to-hall at one speed measuring every cycle. Per cycle it logs slip (`S`-line span + per-end drift), binned `τ(pos)`, and `E_stroke=∫τ·dθ` (fwd/back). Tiered CSVs (`summary.csv` + `profile.csv`) under repo-local `panel/lifecycle_runs/<timestamp>/`, flushed each cycle; cycle count persisted (`state.json`) for crash-resume; sleep inhibited (`systemd-inhibit`). Aborts: target reached (clean), **sustained** Iq anomaly (0.5 s dwell — the Vq-model Iq spikes harmlessly on every accel/reversal, so a single-sample threshold false-trips), slip-span anomaly, backstop-fired, and a **position-progress stall** detector (no advance for >6 s while moving — travel-agnostic; replaces a fixed per-stroke timeout that false-tripped on the ~200 rad travel at low speed). On clean completion it **parks the carriage at center before disabling** — a cycle ends AT a hall limit, and leaving the carriage on the MIN switch breaks the next boot (GPIO5 strapping pin). A separate **wear-trend window** plots E-vs-cycle, span/per-end-drift-vs-cycle, and a `τ(pos)` heatmap (live during a run; **reopen a finished run with `pixi run plot [run_dir]`** — `panel/plot_lifecycle.py` replays the CSVs into the same window). Headless equivalent: `run_lifecycle.py` (same controller, no GUI). Verified on hardware end-to-end (home→cycle→measure→log→park); travel ≈ 200 rad, `v_safe` ≈ 16 rad/s.

## Hard constraints / do-not-break
- **Do NOT use `TorqueControlType::foc_current`.** It caused a motor runaway here (current-loop instability) that target=0 could not stop. Stay voltage-based. The current sensor **is linked READ-ONLY** (`torque_controller` stays `voltage`, `skip_align=true`) — never link it as a control loop. Its ADC reading isn't usable yet, so torque is the Vq model estimate; if you revisit measured `Iq`, fix the ADC, don't touch the torque controller.
- **A real STOP must disable the driver (`ME0`), not just zero the target (`M0`).**
- **Keep the motor enabled-by-user, low voltage_limit, motor clamped** when testing.
- **Launching the panel:** never put `pkill -f foc_panel.py` in the *same* shell command that launches it — the launcher's own command line contains `foc_panel.py`, so pkill kills its own parent shell (symptom: empty output, exit 1). Kill in a separate call: `pgrep -f "[f]oc_panel.py" | xargs -r kill -9`, then launch with no pkill.

## Open / next ideas
- Angle mode can hunt; `P_angle` is the knob (try ~5 if it oscillates). Run the velocity auto-tuner first.
- Lifecycle torque is trend-valid with nameplate Kt; calibrate Kt (panel override field) for absolute mN·m.
- A Rust/egui rewrite was discussed but **dropped** — stay on Python.
