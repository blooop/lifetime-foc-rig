// MKS ESP32 FOC V2.0 | SAFE control firmware v2, single motor M0 | SimpleFOC 2.2.1
// Voltage-based control only (no current sense / no foc_current). Boots DISABLED.
// Improvements: fast I2C timeout (no 1s stalls on a bad read) + armed glitch filter.
// USB serial @115200. Modes via Commander 'M': torque(V)=MC0, velocity=MC1, angle=MC2.
// Dual hall endstops + homing + travel limits via Commander 'E' (see ENDSTOPS below).

#include <Arduino.h>
#include <SimpleFOC.h>

// Glitch filter armed only AFTER initFOC (won't break alignment); rejects the bad
// values that come with occasional I2C hiccups so commutation doesn't lose lock.
class FilteredAS5600 : public MagneticSensorI2C {
public:
  FilteredAS5600() : MagneticSensorI2C(AS5600_I2C) {}
  volatile uint32_t glitches = 0;
  bool  armed = false;
  // Reject only physically-impossible jumps: allow up to max_speed*dt (+ a small
  // floor) per read. A FIXED angle threshold falsely rejected legit motion at high
  // speed (e.g. 20 rad/s homing) whenever a serial/I2C stall stretched the loop
  // period — the motor would briefly "cut out". Scaling by elapsed time keeps real
  // glitch rejection without tripping on fast travel, and self-heals after a stall.
  float max_speed  = 40.0f;   // rad/s, ~2x velocity_limit headroom
  float floor_step = 0.10f;   // rad, tolerate sensor jitter at tiny dt
  float getSensorAngle() override {
    float a = MagneticSensorI2C::getSensorAngle();
    uint32_t now = micros();
    if (!have_last) { last = a; t_last = now; have_last = true; return a; }
    if (!armed)     { last = a; t_last = now; return a; }
    float dt = (uint32_t)(now - t_last) * 1e-6f;          // s (unsigned wrap-safe)
    float d  = a - last;
    if (d >  _PI) d -= _2PI; else if (d < -_PI) d += _2PI;
    if (fabsf(d) > max_speed * dt + floor_step) {         // impossible -> drop, keep
      glitches++; return last;                            // last/t_last so it self-heals
    }
    last = a; t_last = now; return a;
  }
private:
  float last = 0.0f; uint32_t t_last = 0; bool have_last = false;
};

FilteredAS5600 sensor;
TwoWire I2Cone = TwoWire(0);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

#define UNDERVOLTAGE_THRES 11.1
float get_vin_Volt() { return analogReadMilliVolts(13) * 8.5 / 1000; }

// ========================= ENDSTOPS / HOMING / LIMITS =========================
// Dual hall-effect endstops on the FREE sensor port 1. Modules are HW-477
// (A3144 digital hall switch): each has an onboard 10k pull-up + cap + LED,
// powered from the port's GND/3.3V rail. The pull-up references 3.3V so the
// signal idles HIGH at 3.3V and is ESP32-safe (this is NOT 5V logic).
//   A3144 output: HIGH = clear, LOW = magnet present => TRIGGERED (active-low).
//   Non-latching: releases when the magnet leaves.
//
//   Endstop A (MIN / home) -> GPIO 5   (!! ESP32 STRAPPING PIN !!)
//   Endstop B (MAX)        -> GPIO 23  (plain GPIO)
//   (pins matched to the as-built wiring: the MIN-end module is on GPIO 5.)
//
// !! GPIO 5 must read HIGH at boot or the ESP32 won't boot normally. The A3144
//    idles HIGH (clear), so the endstop on GPIO 5 (MIN) must be PHYSICALLY CLEAR
//    at power-up. We
//    only ever configure it as an INPUT (never drive it as an OUTPUT). Both pins
//    use INPUT_PULLUP (harmless given the onboard pull-up, a safe default), and
//    reads are debounced (N consecutive equal samples) since motor phase wiring
//    runs nearby and can inject noise.
//
// Direction convention (used for travel limiting): negative target/motion drives
// toward MIN (endstop A); positive drives toward MAX (endstop B). Homing direction
// is configurable independently.
//
// Manual test path: endstop state streams as an "E\t..." line from boot (even on
// USB-only power, before the motor is ever enabled) and during the undervoltage
// wait — wave a magnet and watch MIN/MAX in the GUI (or serial) to verify wiring.

#define ENDSTOP_A_PIN     5       // Endstop A = MIN / home  (ESP32 strapping pin: INPUT only)
#define ENDSTOP_B_PIN     23      // Endstop B = MAX         (plain GPIO)
#define ENDSTOP_DEBOUNCE  3       // consecutive equal reads required to change state
#define HOMING_TIMEOUT_MS 15000UL

struct Endstop {
  uint8_t pin        = 0;
  bool    enabled    = true;
  bool    active_low = true;      // A3144: LOW = magnet present = triggered
  bool    triggered  = false;     // debounced, enable-gated state
  bool    last_read  = true;      // last raw level (idles HIGH)
  uint8_t count      = 0;

  // INPUT only (safe for the GPIO-5 strapping pin); INPUT_PULLUP is a safe default.
  void begin(uint8_t p) { pin = p; pinMode(pin, INPUT_PULLUP); last_read = digitalRead(pin); count = 0; }

  void update() {
    bool r = digitalRead(pin);
    if (r == last_read) { if (count < ENDSTOP_DEBOUNCE) count++; }
    else                { count = 0; last_read = r; }
    if (count >= ENDSTOP_DEBOUNCE) {
      bool t = active_low ? (r == LOW) : (r == HIGH);
      triggered = enabled && t;
    }
    if (!enabled) triggered = false;
  }
};

Endstop esMin, esMax;             // A = MIN/home, B = MAX

// Homing / position state. Auto-home is a 3-phase sweep: seek MIN, then seek MAX,
// then drive to the measured center — which becomes position 0.
enum HomingPhase { HOME_IDLE = 0, HOME_SEEK_MIN = 1, HOME_SEEK_MAX = 2, HOME_CENTER = 3 };
HomingPhase g_home_phase = HOME_IDLE;
bool  g_homed       = false;
float g_home_offset = 0.0f;       // shaft_angle of the travel center (= position 0)
float g_angle_min   = 0.0f;       // shaft_angle captured at the MIN switch
float g_angle_max   = 0.0f;       // shaft_angle captured at the MAX switch
unsigned long g_home_t0 = 0;
float g_home_speed  = 20.0f;      // rad/s (magnitude) used while seeking endstops
int   g_home_dir    = +1;         // sign of velocity that moves toward MIN (this rig: +1)
bool  g_opp_cleared = false;      // seek safety: opposite endstop seen clear at least once
#define HOME_CENTER_TOL 0.05f     // rad; "arrived at center" tolerance
MotionControlType g_prev_controller = MotionControlType::velocity;

bool homingActive() { return g_home_phase != HOME_IDLE; }

// Optional soft travel limits (home-relative radians), backstopped by the hard
// endstops. Wide-open until configured.
bool  g_soft_enabled = false;
float g_soft_min = -1.0e6f, g_soft_max = 1.0e6f;

float positionFromHome() { return motor.shaft_angle - g_home_offset; }

// Clamp motor.target each loop so it can never drive further INTO a triggered
// endstop (or past a soft limit), while still allowing motion backing AWAY.
// g_home_dir is the sign of motion that heads toward MIN; -g_home_dir heads toward
// MAX. We derive the intended motion direction the same way for every mode so the
// limiting always matches the physical wiring (not a fixed +/- assumption).
void enforceTravelLimits() {
  bool minT = esMin.triggered, maxT = esMax.triggered;
  bool angleMode = (motor.controller == MotionControlType::angle);
  // intended motion: velocity sign for vel/torque; for angle, sign of (target - pos)
  float d = angleMode ? (motor.target - motor.shaft_angle) : motor.target;

  // Hard endstops (always active, even un-homed).
  bool intoMin = (g_home_dir * d > 0.0f);   // heading toward MIN
  bool intoMax = (g_home_dir * d < 0.0f);   // heading toward MAX
  if ((minT && intoMin) || (maxT && intoMax)) {
    motor.target = angleMode ? motor.shaft_angle : 0.0f;   // hold / stop
  }

  // Soft limits (position-based; +velocity always increases position). Homed backstop.
  if (g_homed && g_soft_enabled) {
    if (angleMode) {
      float lo = g_home_offset + g_soft_min, hi = g_home_offset + g_soft_max;
      if (motor.target < lo) motor.target = lo;
      if (motor.target > hi) motor.target = hi;
    } else {
      float pos = positionFromHome();
      if (pos <= g_soft_min && motor.target < 0) motor.target = 0;
      if (pos >= g_soft_max && motor.target > 0) motor.target = 0;
    }
  }
}

// ----------------------------- MOTION PROFILE -----------------------------
// General acceleration-limited setpoint generator, applied to every move (incl.
// homing) so commanded motion is smooth instead of stepping. Velocity & torque
// modes get a trapezoidal-velocity slew (accel-limited setpoint); angle mode gets
// a full trapezoidal position move (accel up to velocity_limit, then decelerate to
// a stop at the target). It intercepts the value Commander/homing/limits wrote to
// motor.target, shapes it, and writes the shaped value back before move().
bool  g_profile_enabled = true;
float g_max_accel  = 50.0f;       // rad/s^2 — velocity slew rate & angle-move accel
float g_prof_vel   = 0.0f;        // profiled velocity state
float g_prof_pos   = 0.0f;        // profiled position state (angle mode)
float g_cmd_target = 0.0f;        // latched command (what Commander/homing requested)
float g_last_written = 0.0f;      // last value we wrote to motor.target
bool  g_prof_init  = false;
MotionControlType g_prof_mode = MotionControlType::velocity;

static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

void applyMotionProfile(float dt) {
  if (!g_profile_enabled || dt <= 0.0f) { g_prof_init = false; return; }  // pass-through
  // (Re)seed state from the real shaft on first run or any control-mode change,
  // so the profile starts where the motor actually is (no jump).
  if (!g_prof_init || motor.controller != g_prof_mode) {
    g_prof_mode  = motor.controller;
    g_prof_vel   = motor.shaft_velocity;
    g_prof_pos   = motor.shaft_angle;
    g_cmd_target = motor.target;
    g_last_written = NAN;          // force re-capture of the command below
    g_prof_init  = true;
  }
  // Capture a freshly-issued command (anything wrote motor.target since our write).
  float cmd = motor.target;
  if (isnan(g_last_written) || cmd != g_last_written) g_cmd_target = cmd;

  float out;
  if (motor.controller == MotionControlType::angle) {
    float vmax  = motor.velocity_limit;
    float toGo  = g_cmd_target - g_prof_pos;
    float decel = (g_prof_vel * g_prof_vel) / (2.0f * g_max_accel);  // braking distance
    float desired = (fabsf(toGo) <= decel) ? 0.0f : (toGo > 0 ? vmax : -vmax);
    float maxdv = g_max_accel * dt;
    g_prof_vel += clampf(desired - g_prof_vel, -maxdv, maxdv);
    g_prof_pos += g_prof_vel * dt;
    if (fabsf(g_cmd_target - g_prof_pos) < 1e-3f && fabsf(g_prof_vel) < maxdv) {
      g_prof_pos = g_cmd_target; g_prof_vel = 0.0f;     // settle exactly on target
    }
    out = g_prof_pos;
  } else if (motor.controller == MotionControlType::velocity) {
    float maxdv = g_max_accel * dt;
    g_prof_vel += clampf(g_cmd_target - g_prof_vel, -maxdv, maxdv);
    out = g_prof_vel;
  } else {
    out = g_cmd_target;            // torque-voltage etc.: pass through unshaped
  }
  motor.target   = out;
  g_last_written = out;
}

// Commander 'P' (motion profile): PA<v> accel [rad/s^2], PE<0|1> enable.
void doProfile(char* cmd) {
  switch (cmd[0]) {
    case 'A': g_max_accel = fmaxf(atof(cmd + 1), 1.0f); Serial.printf("Profile accel=%.1f rad/s^2\n", g_max_accel); break;
    case 'E': g_profile_enabled = atoi(cmd + 1) != 0;   Serial.printf("Motion profile %s\n", g_profile_enabled ? "ON" : "OFF"); break;
    default:  Serial.println(F("P? PA<accel> PE<0|1>")); break;
  }
}

void stopHoming(const char* msg) {     // cancel/abort without completing
  motor.target     = 0;
  motor.controller = g_prev_controller;
  g_home_phase = HOME_IDLE;
  if (msg) Serial.println(msg);
}

void startHoming() {
  // Per safety posture, auto-home requires the user to have enabled the motor first.
  if (!motor.enabled) { Serial.println(F("Home refused: enable the motor first (ME1).")); return; }
  if (!esMin.enabled || !esMax.enabled) { Serial.println(F("Home refused: enable both endstops first.")); return; }
  if (homingActive()) return;
  g_prev_controller = motor.controller;
  motor.controller  = MotionControlType::velocity;   // seek phases drive a velocity
  g_home_phase = HOME_SEEK_MIN;
  g_opp_cleared = false;          // arm the wrong-direction safety once MAX is seen clear
  g_home_t0 = millis();
  Serial.printf("Auto-home: seeking MIN @ %.1f rad/s...\n", g_home_speed);
}

// Non-blocking 3-phase auto-home (runs inside loop() so loopFOC keeps commutating):
//   SEEK_MIN -> SEEK_MAX -> CENTER. The travel center becomes position 0, and the
//   measured endstop positions are recorded as soft limits (left disabled).
void homingStep() {
  if (!homingActive()) return;
  if (!motor.enabled) { stopHoming("Homing aborted (motor disabled)."); return; }
  if (millis() - g_home_t0 > HOMING_TIMEOUT_MS) { stopHoming("Homing timeout."); return; }

  switch (g_home_phase) {
    case HOME_SEEK_MIN:
      if (!esMax.triggered) g_opp_cleared = true;       // MAX has been clear -> safety armed
      if (esMin.triggered) {
        g_angle_min  = motor.shaft_angle;
        motor.target = 0;
        g_home_phase = HOME_SEEK_MAX;
        g_home_t0    = millis();
        Serial.printf("  MIN @ %.3f rad; seeking MAX...\n", g_angle_min);
      } else if (g_opp_cleared && esMax.triggered) {
        // Hit MAX while seeking MIN -> direction is inverted; stop instead of ramming.
        stopHoming("Auto-home aborted: hit MAX while seeking MIN. Flip Seek-MIN direction.");
      } else {
        motor.target = g_home_dir * g_home_speed;       // toward MIN
      }
      break;
    case HOME_SEEK_MAX:
      if (esMax.triggered) {
        g_angle_max   = motor.shaft_angle;
        g_home_offset = 0.5f * (g_angle_min + g_angle_max);
        g_soft_min    = fminf(g_angle_min, g_angle_max) - g_home_offset;   // home-relative
        g_soft_max    = fmaxf(g_angle_min, g_angle_max) - g_home_offset;
        g_homed       = true;
        motor.controller = MotionControlType::angle;    // drive back to center
        motor.target  = g_home_offset;
        g_home_phase  = HOME_CENTER;
        g_home_t0     = millis();
        Serial.printf("  MAX @ %.3f rad; center=%.3f, travel=%.3f rad; centering...\n",
                      g_angle_max, g_home_offset, fabsf(g_angle_max - g_angle_min));
      } else {
        motor.target = -g_home_dir * g_home_speed;      // toward MAX
      }
      break;
    case HOME_CENTER:
      motor.target = g_home_offset;                     // angle mode holds center
      if (fabsf(motor.shaft_angle - g_home_offset) < HOME_CENTER_TOL) {
        g_home_phase = HOME_IDLE;                       // done; leave it holding center
        Serial.println(F("Auto-home complete: centered (position 0)."));
      }
      break;
    default: break;
  }
}

// Endstop/position telemetry: a line distinct from the 7-float motor monitor.
//   E <minTrig> <maxTrig> <homed> <homePhase 0=idle/1=seekMin/2=seekMax/3=center> <position-from-home>
unsigned long g_es_t_last = 0;
void streamEndstops() {
  unsigned long now = millis();
  if (now - g_es_t_last < 50) return;     // ~20 Hz, plenty for indicators
  g_es_t_last = now;
  Serial.printf("E\t%d\t%d\t%d\t%d\t%.4f\n",
                esMin.triggered ? 1 : 0, esMax.triggered ? 1 : 0,
                g_homed ? 1 : 0, (int)g_home_phase, positionFromHome());
}

// Per-endstop config: sub = E<0|1> enable, L<0|1> active-low, P<n> GPIO pin.
void cfgEndstop(Endstop& es, char* s, char tag) {
  switch (s[0]) {
    case 'E': es.enabled    = atoi(s + 1) != 0; break;
    case 'L': es.active_low = atoi(s + 1) != 0; break;
    case 'P': es.begin((uint8_t)atoi(s + 1));   break;   // re-config pin (INPUT only)
    default:  break;
  }
  Serial.printf("Endstop %c: en=%d active_low=%d pin=%d\n", tag, es.enabled, es.active_low, es.pin);
}

// Commander 'E' (endstops). cmd points at the chars AFTER the 'E':
//   H            auto-home: seek MIN, seek MAX, then center (motor must be enabled)
//   X            abort homing
//   Z            set current shaft angle as home/zero (no motion)
//   S<v>         homing seek speed [rad/s] (magnitude), e.g. ES20
//   D<v>         seek-MIN direction: D1 = MIN is the +velocity way (default), D-1 = -velocity
//   A<sub>       configure endstop A (MIN);  B<sub> configure endstop B (MAX)
//                  sub: E<0|1> enable, L<0|1> active-low, P<n> pin
//   LE<0|1>      soft-limit enable        (LN<v>/LX<v> set min/max travel, home-relative rad)
void doEndstop(char* cmd) {
  switch (cmd[0]) {
    case 'H': startHoming(); break;
    case 'X': if (homingActive()) stopHoming("Homing aborted."); break;
    case 'Z': g_home_offset = motor.shaft_angle; g_homed = true; Serial.println(F("Zero set at current position.")); break;
    case 'S': g_home_speed = fabsf(atof(cmd + 1)); Serial.printf("Homing seek speed=%.2f rad/s\n", g_home_speed); break;
    case 'D': g_home_dir = (atof(cmd + 1) < 0) ? -1 : 1; Serial.printf("Seek-MIN dir=%s velocity\n", g_home_dir < 0 ? "-" : "+"); break;
    case 'A': cfgEndstop(esMin, cmd + 1, 'A'); break;
    case 'B': cfgEndstop(esMax, cmd + 1, 'B'); break;
    case 'L':
      if      (cmd[1] == 'E') { g_soft_enabled = atoi(cmd + 2) != 0; Serial.printf("Soft limits %s\n", g_soft_enabled ? "ON" : "OFF"); }
      else if (cmd[1] == 'N') { g_soft_min = atof(cmd + 2); Serial.printf("Soft min=%.3f\n", g_soft_min); }
      else if (cmd[1] == 'X') { g_soft_max = atof(cmd + 2); Serial.printf("Soft max=%.3f\n", g_soft_max); }
      break;
    default: Serial.println(F("E? H X Z S<v> D<v> A.. B.. LE<0|1> LN<v> LX<v>")); break;
  }
}
// ======================= END ENDSTOPS / HOMING / LIMITS =======================

void setup() {
  Serial.begin(115200);

  pinMode(32, INPUT_PULLUP); pinMode(33, INPUT_PULLUP); pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP); pinMode(27, INPUT_PULLUP); pinMode(14, INPUT_PULLUP);
  analogReadResolution(12);

  // Endstops up first so wiring can be verified on USB-only power (motor unpowered).
  esMin.begin(ENDSTOP_A_PIN);
  esMax.begin(ENDSTOP_B_PIN);

  float v = get_vin_Volt();
  while (v <= UNDERVOLTAGE_THRES) {
    esMin.update(); esMax.update();
    v = get_vin_Volt();
    delay(200);
    Serial.printf("Waiting for power on, V=%.2f | endstops MIN=%d MAX=%d\n",
                  v, esMin.triggered ? 1 : 0, esMax.triggered ? 1 : 0);
  }
  Serial.printf("Calibrating motor...Current voltage%.2f\n", v);

  I2Cone.begin(19, 18, 100000UL);
  I2Cone.setTimeOut(25);     // ms — fail fast on a bad read instead of blocking ~1s
  sensor.init(&I2Cone);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = get_vin_Volt();
  driver.init();
  motor.linkDriver(&driver);

  motor.foc_modulation    = FOCModulationType::SpaceVectorPWM;
  motor.controller        = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;

  motor.PID_velocity.P = 0.05;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.02;
  motor.P_angle.P = 10;          // softened (was 20) for stable angle hold

  motor.voltage_limit  = 1.0;
  motor.velocity_limit = 20;

  motor.useMonitoring(Serial);
  motor.monitor_downsample = 100;
  motor.monitor_variables  = _MON_TARGET | _MON_VOLT_Q | _MON_VOLT_D
                           | _MON_CURR_Q | _MON_CURR_D | _MON_VEL | _MON_ANGLE;

  motor.init();
  motor.initFOC();
  sensor.armed = true;       // arm glitch rejection after calibration
  motor.target = 0;
  motor.disable();           // boot disabled

  command.add('M', doMotor, "motor");
  command.add('E', doEndstop, "endstop");
  command.add('P', doProfile, "profile");

  Serial.println(F("Motor ready (DISABLED) - connect panel (115200), then Enable."));
}

void loop() {
  static uint32_t t_prev = 0;
  uint32_t now = micros();
  float dt = (t_prev == 0) ? 0.0f : (uint32_t)(now - t_prev) * 1e-6f;
  if (dt > 0.05f) dt = 0.05f;   // cap after a stall so the profile can't lurch
  t_prev = now;

  motor.loopFOC();
  esMin.update();             // read endstops every control loop (after loopFOC,
  esMax.update();             //   before move) so shaft_angle is fresh
  homingStep();               // non-blocking; commands a seek velocity while homing
  enforceTravelLimits();      // clamp target out of triggered limits (hard + soft)
  applyMotionProfile(dt);     // accel-limit the command (trapezoidal) before move
  motor.move();
  motor.monitor();
  streamEndstops();
  command.run();
}
