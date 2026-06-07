# CLAUDE.md — lifetime-foc-rig (MKS ESP32 FOC workspace)

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
- `firmware/` — PlatformIO project (`board=esp32dev`, `framework=arduino`, SimpleFOC pinned to `Arduino-FOC.git#v2.2.1`). Active sketch: `src/main.cpp`; pure safety/motion math in `src/control_logic.h`; host unit tests in `test/`.
- `panel/foc_panel.py` — custom PyQt5 control panel (GUI); `panel/lifecycle.py` + `panel/run_lifecycle.py` — endurance test. Tests in `panel/tests/`.
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
pixi run gui         # launch the GUI (12 V on so the board calibrates)
pixi run lifecycle --cycles 5000 --vmeas 3.0   # headless endurance run
pixi run plot        # view a finished run (newest, or `pixi run plot <run_dir>`)
pixi run test        # Python tests (panel/tests) — no hardware needed
pixi run test-fw     # native firmware safety-logic unit tests — no hardware needed
```
Only one program can own the serial port at a time — close the GUI before `monitor`/`lifecycle`.

## Tests (no hardware — for offline development)
Two suites run with no board attached; CI (`.github/workflows/ci.yml`) runs both + `build`.
- **Python** (`panel/tests/`, pytest, `pixi run test`): headless Qt via `QT_QPA_PLATFORM=offscreen`.
  Covers `parse_line`/`find_serial_port`/`ziegler_nichols`/torque model and the **whole**
  `LifecycleController` state machine, driven through a `FakeWorker` double + a controllable
  `clock` fixture (see `panel/tests/conftest.py`). To test board-coupled logic, emit on the
  FakeWorker's signals / call controller slots directly — never open a real serial port.
- **Firmware** (`firmware/test/`, PlatformIO `native`+Unity, `pixi run test-fw`): the safety/
  motion math is extracted into **`firmware/src/control_logic.h`** as pure globals-free functions
  (`computeTravelLimits`, `backstopMargin`/`vSafeFromMargin`, `glitchIsBad`, `profileVelStep`/
  `profileAngleStep`); `main.cpp` wires the `motor`/globals into them. Keep that header free of
  `Arduino.h`/`SimpleFOC.h` or the native build breaks. `platformio.ini` pins `default_envs =
  esp32dev` so `pio run` never tries to build the firmware under the `native` env.

## Current firmware (`firmware/src/main.cpp`) — safe, voltage-based
- Modes: **velocity** (default), **angle**, **torque-voltage**. Voltage-based control only.
- `voltage_limit=3.0 V`, `velocity_limit=100`, `PID_velocity P=0.05 I=1.0 D=0`, `LPF_velocity.Tf=0.02`, `P_angle.P=10`. (Raised from 1.0 V / 20 rad/s for high-speed runs — see the overtravel note below; halls must sit with ≥20% travel of clear space beyond them.) **Verified on hardware 2026-06-06**: 12 V on, clean calibration, homes & arms `v_safe=109.67` (margin 40 rad = 20% of ~200 rad travel), reaches 60 rad/s with **`Vq` peaking ~1.6 V of the 3 V limit** (huge headroom — 100 rad/s well within reach), no glitch-filter cutouts, hard-endstop clamp + center-park both work. NB: command velocity only *after* an `EH` home has fully settled (phase→idle at center) — commanding mid-home lets `homingStep()`/the endstop clamp override the target (looks like a stall).
- I2C **100 kHz** with `I2Cone.setTimeOut(25)` (a noisy read fails in 25 ms instead of blocking ~1 s).
- **Armed-after-initFOC glitch filter** (`FilteredAS5600`): disarmed during alignment (else it trips `Failed to notice movement`), armed while running. Do not make it always-on. Rejection is **time-aware** (`max_speed·dt + floor_step`), not a fixed angle step — a fixed `0.6 rad` step falsely rejected legit motion at high speed (≈20 rad/s) when a serial/I2C stall stretched the loop, briefly cutting out the motor. Keep it time-scaled. `max_speed` is **driven each `loop()` from `2·motor.velocity_limit` (40 rad/s floor)** so a runtime `MLV` change scales the filter too — it must never sit below the commanded ceiling or it rejects real motion. There is no fixed firmware speed cap left; the GUI owns the velocity ceiling via `MLV`.
- **Boots DISABLED** (`motor.disable()` after `initFOC`) — user enables explicitly.
- Streams the **full 7-variable monitor set** at `monitor_downsample=100` (GUIs read velocity at stream index 5, angle at index 6 — a 3-var monitor leaves live feedback blank). Index 3/4 carry `Iq`/`Id` from the current sense **in milliamps** (`monitor()` prints `c.q*1000`) — consumers must scale by 1e-3.
- **Read-only current sense**: `InlineCurrentSense(0.01f, 50.0f, 39, 36)` (Motor-0 inline shunts, makerbase example #14), linked via `current_sense.init()` + **`current_sense.skip_align = true`** + `motor.linkCurrentSense()` before `initFOC()`. `skip_align` is **required**: the current-sense `driverAlign()` fails on this board (small/noisy shunt current at the align voltage) and a failure aborts the WHOLE `initFOC` → no commutation → railed current on enable. `torque_controller` stays `voltage` (never `foc_current` — runaway). **Status: the reading is USABLE** — verified on hardware 2026-06-06. At idle (motor disabled, 12 V on) both phase ADCs sit at the INA240 VS/2 ≈ 1.65 V bias (~1849 cts via Arduino, ~1815 via SimpleFOC's legacy `adcRead`), and SimpleFOC `Iq`/`Id` read ~0 ± 25 **mA**. Under load (spinning ~2 rad/s) measured `Iq` tracks the Vq model in sign/timing/magnitude (peaks ~3 A). The old "±25 A garbage / ESP32 ADC bring-up" note was a **units bug**: the monitor emits `Iq` in mA (`c.q*1000`) and the panel read it as amps (1000× inflation) — fixed at `foc_panel.py` (the `telem.emit` divides `v[3]` by 1000). **Torque is still the Vq model estimate** (`τ = Kt·(Vq−Ke·ω)/R`) — the panel/lifecycle ignore measured `Iq` for now — but measured `Iq` is now trustworthy and runs ~1.3–1.5× the model, i.e. the model's nameplate `R`/`KV` under-reads; calibrate against measured `Iq` (or switch torque to it) as the next step. Motor control itself is unaffected (spins & tracks target with the sense linked).
- Exposes the SimpleFOC **Commander** under motor letter **`M`**. Clean calibration log: `sensor_direction==CCW`, `PP check: OK!`, `Zero elec. angle ~5`.
- **Endstops / homing / travel limits** live in one delimited section of `main.cpp` (`ENDSTOPS / HOMING / LIMITS`). Read both each loop (after `loopFOC`, before `move`), **debounced** (3 consecutive reads). `enforceTravelLimits()` clamps `motor.target` out of any triggered limit each loop. The hard-endstop clamp keys off **`g_home_dir`** (sign of velocity that heads toward MIN; **default `+1` for this rig** — `-1` drove the wrong way), so limiting and homing share one physically-correct direction convention; soft limits are position-based as a homed backstop. Backing away is always allowed. Non-blocking **3-phase auto-home state machine**: seek MIN → seek MAX → drive to the measured **center** (which becomes position 0); records the endstop positions as soft limits (left disabled). Default seek speed **20 rad/s**. If seek-MIN reaches the MAX switch instead (inverted direction), it **aborts** rather than ramming the endstop.
- **Motion profile** (`MOTION PROFILE` section, `applyMotionProfile()` in the loop before `move`): a general acceleration-limited setpoint generator so commanded motion is smooth (no slamming reversals). Velocity/torque modes get a trapezoidal-velocity slew; angle mode gets a full trapezoidal position move (accel to `velocity_limit`, decelerate to a stop). It intercepts whatever Commander/homing/limits wrote to `motor.target`, shapes it, writes it back. Re-seeds from the live shaft on any control-mode change (no jumps). Default accel **300 rad/s²** (`g_max_accel`) — sized so a stop from the 100 rad/s top speed fits the overtravel margin (at the old 50 rad/s² a 100 rad/s stop needed ~½ the travel). Torque mode passes through unshaped (keeps the autotuner intact). This replaced the earlier homing-only `output_ramp` softening. **Homing requires the motor already enabled and both endstops enabled** (refuses otherwise). Endstop pins, active-level, enable, seek speed/direction, and soft limits are runtime-configurable (constants for defaults, `E` commands at runtime).
- Streams a **second telemetry line** distinct from the 7-var monitor: `E\t<minTrig>\t<maxTrig>\t<homed>\t<homePhase 0=idle,1=seekMin,2=seekMax,3=center>\t<pos-from-home>\t<backstopFired 0=none/1=pastMin/2=pastMax>` at ~20 Hz. **Streams from boot and during the undervoltage wait (USB-only power), motor disabled** — so endstop wiring is verifiable by waving a magnet before any motion.
- **20% overtravel backstop** (lifecycle safety): auto-armed at the end of a full `EH` home (NOT a bare `EZ` — it has no travel reference). Lines = endstop shaft-angle ± `OVERTRAVEL_FRAC`(=0.20)·travel, recomputed each home so they track slip. If a missed/failed hall lets the carriage run past a line **while heading further past** (backing away is allowed), `enforceTravelLimits()` calls `motor.disable()` — **disable-on-trip, NO latch** (re-enable manually + re-home), not defeatable. On arming it also derives `g_v_safe = sqrt(2·accel·0.5·0.20·travel)` and clamps velocity-mode `|target|` so a reverse-on-trigger stop stays well inside the margin. (Margin widened from 5% to 20% so `v_safe` allows the 100 rad/s top speed — the halls must have ≥20% of travel of clear physical space beyond them, with the hard stops past that.) Purpose: the hall switches are the **working** limits (hit every cycle); separate **physical hard stops sit beyond them and must never be reached**.
- **Hall-edge slip latch**: `Endstop.just_triggered` (rising edge) → `loop()` emits `S\t<which 0=min/1=max>\t<shaft_angle>` once per edge (continuous angle latched at ~kHz, immune to the 20 Hz E-line jitter). The panel owns the cycle counter; this is the precise slip/“motor-rotation-per-stroke” signal.
- **Serial-heartbeat watchdog**: any command pets `g_last_cmd_ms`; if the motor is enabled **and not homing** and no command arrives for `WATCHDOG_MS`(=3 s), `loop()` disables the driver. Suppressed during homing and while idle-disabled so it won't false-trip. Both the GUI (a 1 Hz `QTimer` in the main window) and the lifecycle controller send a ~1 Hz `EK` keepalive — **without it the board silently disables the motor 3 s after Enable while the GUI sits idle, and a later Auto-home is then refused** (the symptom that prompted adding the GUI keepalive).

## Commander serial protocol (what the GUI sends)
`MC0`=torque-voltage, `MC1`=velocity, `MC2`=angle · `M<x>`=target · `ME1`/`ME0`=enable/disable ·
`MLU<v>`=voltage limit, `MLV<v>`=velocity limit · `MVP/MVI/MVD<x>`=velocity PID · `MAP<x>`=angle P ·
`MVF<x>`=velocity LPF Tf · `MMD<n>`=monitor downsample. The panel **pulses a reset on connect** (RTS), so the board re-runs calibration on every panel startup. On each (re)connect the panel then **pushes its operating ceilings** (`push_limits()`: `MLU` voltage, `MLV` velocity, `PA` accel, `PE` profiling) so the GUI fields — not the firmware boot defaults — are the source of truth without a reflash; `PA` is sent before any `EH` home because `v_safe` is derived from accel at home time. (Skipped while a lifecycle run owns the connection.) Ctrl-C in the terminal shuts the panel down cleanly (closeEvent → `ME0`).
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
  read-only current sense's measured `Iq` is now usable — the panel converts the monitor's mA field to
  amps — but torque still uses the Vq model; measured `Iq` is the calibration reference.) `SerialWorker`
  auto-reconnects on a serial drop (re-homes/resumes via the `ready` signal).
- **Auto-tune** only applies to the velocity loop (torque-voltage has no PID loop; the current loop is too fast to tune over serial).
- **Motion profile** group: enable trapezoidal profiling + acceleration [rad/s²] field (`PE`/`PA`). Lower accel = gentler ramps/reversals.
- **Endstops & homing** group: live MIN/MAX/homed indicators (green=clear, red=triggered) + a safety banner that turns red and names the active endstop when motion is limited (and shows the auto-home phase while running; turns dark-red on a backstop trip); per-endstop enable/pin/active-low; seek speed + seek-MIN direction; **Auto-home (MIN→MAX→center)** (refuses unless enabled) and **Set zero here** buttons; soft-limit enable + min/max travel fields. Position-from-home shows in the Live box. Indicators update from boot, so wiring can be checked before enabling the motor. An **"Auto-home on connect"** checkbox (default **on**) makes the panel enable the motor + both endstops and run the homing sweep automatically on every connect/reconnect (the carriage moves at startup); it no-ops while a lifecycle run is active, since `LifecycleController` owns its own re-home on reconnect.
- **Lifecycle test** group (`panel/lifecycle.py`, `LifecycleController`): target cycles, measure speed, Iq/slip abort thresholds, Start/Stop. Drives the rolling-drive endurance test — homes, then cycles hall-to-hall at one speed measuring every cycle. Per cycle it logs slip (`S`-line span + per-end drift), binned `τ(pos)`, and `E_stroke=∫τ·dθ` (fwd/back). Tiered CSVs (`summary.csv` + `profile.csv`) under repo-local `panel/lifecycle_runs/<timestamp>/`, flushed each cycle; cycle count persisted (`state.json`) for crash-resume; sleep inhibited (`systemd-inhibit`). Aborts: target reached (clean), **sustained** Iq anomaly (0.5 s dwell — the Vq-model Iq spikes harmlessly on every accel/reversal, so a single-sample threshold false-trips), slip-span anomaly, backstop-fired, and a **position-progress stall** detector (no advance for >6 s while moving — travel-agnostic; replaces a fixed per-stroke timeout that false-tripped on the ~200 rad travel at low speed). On clean completion it **parks the carriage at center before disabling** — a cycle ends AT a hall limit, and leaving the carriage on the MIN switch breaks the next boot (GPIO5 strapping pin). A separate **wear-trend window** plots E-vs-cycle, span/per-end-drift-vs-cycle, and a `τ(pos)` heatmap (live during a run; **reopen a finished run with `pixi run plot [run_dir]`** — `panel/plot_lifecycle.py` replays the CSVs into the same window). Headless equivalent: `run_lifecycle.py` (same controller, no GUI). Verified on hardware end-to-end (home→cycle→measure→log→park); travel ≈ 200 rad, `v_safe` ≈ 109 rad/s (at accel 300, 20% margin).

## Hard constraints / do-not-break
- **Do NOT use `TorqueControlType::foc_current`.** It caused a motor runaway here (current-loop instability) that target=0 could not stop. Stay voltage-based. The current sensor **is linked READ-ONLY** (`torque_controller` stays `voltage`, `skip_align=true`) — never link it as a control loop. Its measured `Iq` reading is usable (verified) but torque still uses the Vq model; if you switch torque to measured `Iq`, keep `torque_controller=voltage` and feed it only to the model/readout — don't touch the torque controller.
- **A real STOP must disable the driver (`ME0`), not just zero the target (`M0`).**
- **Keep the motor enabled-by-user, low voltage_limit, motor clamped** when testing.
- **Launching the panel:** never put `pkill -f foc_panel.py` in the *same* shell command that launches it — the launcher's own command line contains `foc_panel.py`, so pkill kills its own parent shell (symptom: empty output, exit 1). Kill in a separate call: `pgrep -f "[f]oc_panel.py" | xargs -r kill -9`, then launch with no pkill.

## Open / next ideas
- Angle mode can hunt; `P_angle` is the knob (try ~5 if it oscillates). Run the velocity auto-tuner first.
- Lifecycle torque is trend-valid with nameplate Kt; calibrate Kt (panel override field) for absolute mN·m.
- A Rust/egui rewrite was discussed but **dropped** — stay on Python.
