// control_logic.h — pure, hardware-free safety/motion math for the FOC rig.
//
// These functions hold the *decisions* that main.cpp's enforceTravelLimits(),
// applyMotionProfile(), the FilteredAS5600 glitch filter, and the homing v_safe
// derivation make every loop. They take plain arguments (no SimpleFOC/Arduino
// types, no globals, no I/O) so they can be unit-tested on the host with
// `pio test -e native`. main.cpp wires the globals/motor object into these.
//
// Keep this header free of Arduino.h / SimpleFOC.h so the native build compiles.
#pragma once
#include <cmath>

static const float CL_PI  = 3.14159265358979f;
static const float CL_2PI = 6.28318530717959f;

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// --- overtravel backstop sizing (computed once per home) ---------------------
// margin = OVERTRAVEL_FRAC * travel ; v_safe so a reverse-on-trigger stop uses
// at most OVERTRAVEL_SAFE of that margin: v = sqrt(2 a · safe · frac · travel).
static inline float backstopMargin(float frac, float travel) {
  return frac * fabsf(travel);
}
static inline float vSafeFromMargin(float accel, float safe, float frac, float travel) {
  return sqrtf(2.0f * accel * safe * frac * fabsf(travel));
}

// --- AS5600 glitch filter ----------------------------------------------------
// True => this reading is a physically-impossible jump (drop it, keep the last).
// Rejection is TIME-AWARE: tolerate up to max_speed*dt (+ a small floor) per read,
// so legit fast motion through a stretched loop period is NOT falsely rejected.
static inline bool glitchIsBad(float last, float a, float dt,
                               float max_speed, float floor_step) {
  float d = a - last;
  if (d >  CL_PI) d -= CL_2PI; else if (d < -CL_PI) d += CL_2PI;   // wrap to (-pi, pi]
  return fabsf(d) > max_speed * dt + floor_step;
}

// --- travel limit clamp ------------------------------------------------------
struct LimitInputs {
  bool  angleMode;          // motor.controller == angle
  float target;             // motor.target as written by Commander/homing
  float shaft_angle;        // motor.shaft_angle
  bool  minT, maxT;         // debounced endstop states
  int   home_dir;           // sign of velocity heading toward MIN (+1 this rig)
  bool  backstop_armed;
  float angle_min, angle_max, backstop_margin, v_safe;
  bool  homed, soft_enabled;
  float home_offset, soft_min, soft_max;
};

struct LimitOutput {
  float target;             // clamped target to write back to motor.target
  int   backstop_fired;     // 0 none, 1 tripped past MIN, 2 tripped past MAX
  bool  disable;            // request motor.disable() (backstop trip)
};

// Mirrors enforceTravelLimits(): hard endstops -> backstop (disable + v_safe cap)
// -> soft limits, all in the g_home_dir convention so limiting matches the wiring.
static inline LimitOutput computeTravelLimits(const LimitInputs& in) {
  LimitOutput out{in.target, 0, false};
  // intended motion: velocity sign (vel/torque) or sign of (target - pos) (angle)
  float d = in.angleMode ? (in.target - in.shaft_angle) : in.target;
  bool intoMin = (in.home_dir * d > 0.0f);    // heading toward MIN
  bool intoMax = (in.home_dir * d < 0.0f);    // heading toward MAX

  // Hard endstops (active even un-homed): never drive further into a hit switch.
  if ((in.minT && intoMin) || (in.maxT && intoMax)) {
    out.target = in.angleMode ? in.shaft_angle : 0.0f;
  }

  // Overtravel backstop: a missed hall let the carriage run past the line while
  // heading FURTHER past -> kill the driver (backing away is allowed).
  if (in.backstop_armed) {
    bool pastMin = (in.home_dir * (in.shaft_angle - in.angle_min) >  in.backstop_margin) && intoMin;
    bool pastMax = (in.home_dir * (in.shaft_angle - in.angle_max) < -in.backstop_margin) && intoMax;
    if (pastMin || pastMax) {
      out.backstop_fired = pastMin ? 1 : 2;
      out.disable = true;
      out.target = in.angleMode ? in.shaft_angle : 0.0f;
    }
    // cap velocity-mode speed so a reverse-on-trigger stop stays inside the margin
    if (!in.angleMode) {
      if (out.target >  in.v_safe) out.target =  in.v_safe;
      if (out.target < -in.v_safe) out.target = -in.v_safe;
    }
  }

  // Soft limits (position-based), homed backstop.
  if (in.homed && in.soft_enabled) {
    if (in.angleMode) {
      float lo = in.home_offset + in.soft_min, hi = in.home_offset + in.soft_max;
      if (out.target < lo) out.target = lo;
      if (out.target > hi) out.target = hi;
    } else {
      float pos = in.shaft_angle - in.home_offset;
      if (pos <= in.soft_min && out.target < 0) out.target = 0;
      if (pos >= in.soft_max && out.target > 0) out.target = 0;
    }
  }
  return out;
}

// --- motion profile setpoint generator --------------------------------------
// Velocity / torque modes: accel-limited slew of the commanded velocity.
static inline float profileVelStep(float prof_vel, float cmd_target, float accel, float dt) {
  float maxdv = accel * dt;
  return prof_vel + clampf(cmd_target - prof_vel, -maxdv, maxdv);
}

// Angle mode: full trapezoidal position move (accel to vmax, decelerate to a stop
// at the target). Returns the updated {vel, pos}; `out` is the new motor.target.
struct ProfileState { float vel; float pos; };
static inline ProfileState profileAngleStep(ProfileState s, float cmd_target,
                                             float vmax, float accel, float dt) {
  float toGo  = cmd_target - s.pos;
  float decel = (s.vel * s.vel) / (2.0f * accel);    // braking distance at current vel
  float desired = (fabsf(toGo) <= decel) ? 0.0f : (toGo > 0 ? vmax : -vmax);
  float maxdv = accel * dt;
  s.vel += clampf(desired - s.vel, -maxdv, maxdv);
  s.pos += s.vel * dt;
  if (fabsf(cmd_target - s.pos) < 1e-3f && fabsf(s.vel) < maxdv) {
    s.pos = cmd_target; s.vel = 0.0f;                 // settle exactly on target
  }
  return s;
}
