# Endstops, Homing & Motion Profiles

Dual hall-effect endstop support, an auto-home routine, soft travel limits, and a
general trapezoidal motion profiler for the MKS ESP32 FOC V2.0 firmware
(`firmware/src/main.cpp`) and the PyQt5 control panel (`panel/foc_panel.py`).

All of this is voltage-based and respects the workspace's safety posture: the board
boots **disabled**, the user enables explicitly, and a real STOP disables the driver.

---

## 1. Hardware & wiring

Two **HW-477** modules (A3144 digital hall switch) on the **free sensor port 1**.
Each module has an onboard 10 kΩ pull-up + decoupling cap + LED, powered from the
port's **GND / 3.3 V** rail. Because the pull-up references 3.3 V, the signal idles
HIGH at 3.3 V and is **ESP32-safe — this is NOT 5 V logic**.

| Endstop | Role        | GPIO    | Notes |
|---------|-------------|---------|-------|
| A       | MIN / home  | **5**   | **ESP32 strapping pin** — must read HIGH at boot |
| B       | MAX         | **23**  | plain GPIO |

- A3144 output is **active-low**: `HIGH = clear`, `LOW = magnet present = TRIGGERED`.
  Non-latching (releases when the magnet leaves).
- **GPIO 5 is a strapping pin.** It must be HIGH at power-up or the ESP32 won't boot
  normally. The A3144 idles HIGH (clear), so the MIN switch must be **physically
  clear at power-up**. Firmware only ever configures it as `INPUT` (never an output).
- Both pins use `INPUT_PULLUP` (harmless given the onboard pull-up, a safe default).
- Reads are **debounced** (3 consecutive equal samples) because motor phase wiring
  runs nearby and can inject noise.

> The MIN/MAX → GPIO mapping was matched to the as-built wiring and verified on
> hardware (a full-GPIO scan found the signals on 5 and 23). If you rewire, change
> `ENDSTOP_A_PIN` / `ENDSTOP_B_PIN` (or the pin fields in the GUI).

### Manual wiring check (do this first)

Endstop state streams as an `E\t…` line **from boot** — even on USB-only power,
before the motor is ever enabled, and during the undervoltage wait. Open the panel
(or read the serial directly) and **wave a magnet over each module**: the matching
MIN/MAX indicator should flip to red, and the module's own LED should light. Verify
this before trusting any motion-limiting behaviour.

---

## 2. Behaviour

### Direction-aware travel limiting
Every control loop (after `loopFOC`, before `move`), `enforceTravelLimits()` clamps
`motor.target` so the motor can **never drive further into a triggered endstop**,
while still allowing motion **backing away**. The "which way is MIN" convention is a
single sign, `g_home_dir` (default `+1` for this rig — `-1` drove the wrong way), and
both homing and limiting share it, so limiting always matches the physical wiring
regardless of mode (velocity, torque-voltage, or angle).

### Auto-home (3-phase) → center
Pressing **Auto-home** (serial `EH`) runs a non-blocking state machine:

1. **Seek MIN** — drive slowly toward the MIN switch until it triggers; capture the
   shaft angle.
2. **Seek MAX** — reverse and drive toward the MAX switch until it triggers; capture
   the shaft angle.
3. **Center** — drive to the midpoint and hold it. That center becomes **position 0**
   (so MIN ≈ −½·travel, MAX ≈ +½·travel). The measured endstop positions are recorded
   as soft limits (left disabled for you to enable).

Safety:
- Requires the **motor already enabled** *and* **both endstops enabled** (refuses
  otherwise — consistent with the boots-disabled posture).
- Default seek speed **20 rad/s**; each phase has a 15 s timeout.
- If seek-MIN reaches the **MAX** switch instead (inverted direction), it **aborts**
  with a message rather than ramming the endstop.

### Soft travel limits
Once homed, optional soft limits (home-relative radians) stop motion before the
physical endstops, as a backstop in addition to the hard endstops. Position-based, so
they're correct in every mode.

### Motion profile (general, trapezoidal)
`applyMotionProfile()` runs every loop before `move()` and shapes whatever
Commander / homing / limits wrote to `motor.target`:

- **Velocity & torque-seek modes** → trapezoidal-velocity slew (the commanded
  velocity ramps at a fixed acceleration and reverses through zero smoothly).
- **Angle mode** → full trapezoidal position move (accelerate up to `velocity_limit`,
  cruise, decelerate to a clean stop at the target).
- Re-seeds from the live shaft angle/velocity on any control-mode change, so there
  are no jumps (including the homing seek→center handoff).
- **Torque mode passes through unshaped**, so the relay auto-tuner is unaffected.

Default acceleration **50 rad/s²** (`g_max_accel`). Lower it for gentler motion,
disable it for raw instant commands. This is what makes homing reversals smooth
instead of brutal; it applies to normal velocity/angle moves too.

---

## 3. Serial protocol

The firmware exposes three SimpleFOC Commander letters. Existing **`M`** (motor) is
unchanged; **`E`** (endstops) and **`P`** (motion profile) are new.

### `E` — endstops / homing / soft limits
| Command | Effect |
|---------|--------|
| `EH` | Auto-home: seek MIN → seek MAX → center (motor must be enabled) |
| `EX` | Abort homing |
| `EZ` | Set current shaft angle as home/zero (no motion) |
| `ES<v>` | Homing seek speed [rad/s], e.g. `ES20` |
| `ED1` / `ED-1` | Seek-MIN direction: `D1` = MIN is the +velocity way (default), `D-1` = −velocity |
| `EAE<0\|1>` / `EBE<0\|1>` | Enable endstop A / B |
| `EAL<0\|1>` / `EBL<0\|1>` | Active-low for endstop A / B |
| `EAP<n>` / `EBP<n>` | Set GPIO pin for endstop A / B (re-configured as INPUT) |
| `ELE<0\|1>` | Soft-limit enable |
| `ELN<v>` / `ELX<v>` | Soft min / max travel (home-relative rad) |

### `P` — motion profile
| Command | Effect |
|---------|--------|
| `PA<v>` | Acceleration [rad/s²] |
| `PE<0\|1>` | Enable / disable trapezoidal profiling |

### Telemetry
Alongside the existing 7-field motor monitor (`target, Vq, Vd, Iq, Id, vel, angle`),
the firmware streams a distinct endstop line at ~20 Hz:

```
E\t<minTrig>\t<maxTrig>\t<homed>\t<homePhase>\t<position-from-home>
        0/1        0/1       0/1   0=idle,1=seekMin,2=seekMax,3=center   rad
```

The GUI parses the `E\t` prefix separately, so the monitor stream and the velocity
auto-tuner (which reads velocity at index 5) are untouched.

---

## 4. GUI controls (`panel/foc_panel.py`)

- **Motion profile** group: enable trapezoidal profiling + acceleration [rad/s²].
- **Endstops & homing** group:
  - Live **MIN / MAX / homed** indicators (green = clear, red = triggered, blue = homed).
  - A **safety banner** that turns red and names the active endstop when motion is
    limited, and shows the auto-home phase while running.
  - Per-endstop **enable / GPIO / active-low**.
  - **Seek speed** and **seek-MIN direction**.
  - **Auto-home (MIN→MAX→center)** (refuses unless enabled) and **Set zero here**.
  - **Soft-limit** enable + min/max travel fields.
  - **Position (home)** readout in the Live box.

---

## 5. Tuning & troubleshooting

| Symptom | Knob / cause |
|---------|--------------|
| Homing drives the wrong way / rams an endstop | Flip **Seek-MIN direction** (`ED`). Limiting follows the same convention. |
| Reversals / moves too aggressive | Lower **Acceleration** (`PA`), e.g. 20–30 rad/s². Disable profiling for raw behaviour. |
| Motor briefly "cut out" at high speed | Fixed: the AS5600 glitch filter is now **time-aware** (`max_speed·dt + floor`) instead of a fixed 0.6 rad step, which falsely rejected legit fast motion when a serial/I2C stall stretched the loop. Keep it time-scaled. |
| MIN/MAX indicators swapped | Swap `ENDSTOP_A_PIN`/`ENDSTOP_B_PIN` (or the GUI pin fields) to match wiring. |
| Endstop never triggers but module LED lights | Signal wire not reaching the GPIO — check the DO→GPIO connection (a full-GPIO scan sketch is the fastest way to find where a signal lands). |

---

## 6. Preserved constraints

- **No `foc_current`** — voltage-based control only (a current-loop runaway happened
  here once; stay voltage-based).
- A real **STOP disables the driver** (`ME0`), not just `M0`. The GUI STOP also aborts
  homing (`EX`).
- Board **boots disabled**; the user enables. **Auto-home requires the motor enabled.**
- The endstop logic lives in one delimited section of `main.cpp`
  (`ENDSTOPS / HOMING / LIMITS` + `MOTION PROFILE`) so it's easy to find.
