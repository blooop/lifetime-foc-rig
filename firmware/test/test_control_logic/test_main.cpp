// Native (host) unit tests for the pure safety/motion math in src/control_logic.h.
// Run with `pio test -e native` (or `pixi run test-fw`). No board required.
#include <unity.h>
#include "control_logic.h"

void setUp() {}
void tearDown() {}

// --- a baseline "nothing limiting" input the tests tweak field-by-field --------
// Geometry consistent with home_dir = +1: toward-MIN = +angle, toward-MAX = -angle.
// So MIN sits at the +end (angle_min = 0) and MAX 200 rad away in -angle
// (angle_max = -200); the travel center (home_offset) is -100.
static LimitInputs clearInputs() {
  LimitInputs in{};
  in.angleMode = false;
  in.target = 0.0f; in.shaft_angle = 0.0f;
  in.minT = false; in.maxT = false; in.home_dir = +1;
  in.backstop_armed = false;
  in.angle_min = 0.0f; in.angle_max = -200.0f;
  in.backstop_margin = 40.0f; in.v_safe = 109.5f;
  in.homed = false; in.soft_enabled = false;
  in.home_offset = -100.0f; in.soft_min = -100.0f; in.soft_max = 100.0f;
  return in;
}

// ----------------------------------------------------------------- clampf
void test_clampf() {
  TEST_ASSERT_EQUAL_FLOAT(5.0f, clampf(5.0f, 0.0f, 10.0f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, clampf(-3.0f, 0.0f, 10.0f));
  TEST_ASSERT_EQUAL_FLOAT(10.0f, clampf(99.0f, 0.0f, 10.0f));
}

// ----------------------------------------------------------------- backstop sizing
void test_backstop_margin() {
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 40.0f, backstopMargin(0.20f, 200.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 40.0f, backstopMargin(0.20f, -200.0f));   // |travel|
}

void test_vsafe_allows_top_speed() {
  // accel=300, safe=0.5, frac=0.20, travel~200 -> v_safe ~= 109.5 rad/s, which is
  // the documented requirement: it must allow the 100 rad/s top speed.
  float v = vSafeFromMargin(300.0f, 0.5f, 0.20f, 200.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 109.54f, v);
  TEST_ASSERT_TRUE(v >= 100.0f);
}

// ----------------------------------------------------------------- glitch filter
void test_glitch_passes_legit_fast_motion() {
  // 20 rad/s homing with a stretched 50 ms loop = ~1 rad step. The OLD fixed
  // 0.6 rad threshold wrongly rejected this; the time-aware one must accept it.
  float dt = 0.05f, max_speed = 200.0f, floor = 0.10f;   // max_speed = 2*velocity_limit
  TEST_ASSERT_FALSE(glitchIsBad(0.0f, 1.0f, dt, max_speed, floor));
  TEST_ASSERT_FALSE(glitchIsBad(0.0f, 0.5f, dt, max_speed, floor));
}

void test_glitch_rejects_impossible_jump() {
  // 3 rad in 1 ms = 3000 rad/s -> impossible
  TEST_ASSERT_TRUE(glitchIsBad(0.0f, 3.0f, 0.001f, 200.0f, 0.10f));
}

void test_glitch_wraps_across_pi() {
  // continuous motion across the +/-pi seam is a tiny wrapped delta, not a glitch
  TEST_ASSERT_FALSE(glitchIsBad(3.10f, -3.10f, 0.01f, 200.0f, 0.10f));
}

// ----------------------------------------------------------------- travel limits
void test_hard_endstop_blocks_into_min() {
  LimitInputs in = clearInputs();
  in.minT = true; in.target = 5.0f;          // +velocity heads toward MIN (home_dir=+1)
  LimitOutput out = computeTravelLimits(in);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, out.target);
  TEST_ASSERT_FALSE(out.disable);
}

void test_hard_endstop_allows_backing_away() {
  LimitInputs in = clearInputs();
  in.minT = true; in.target = -5.0f;         // -velocity heads toward MAX (away from MIN)
  LimitOutput out = computeTravelLimits(in);
  TEST_ASSERT_EQUAL_FLOAT(-5.0f, out.target);
}

void test_backstop_trips_past_min() {
  LimitInputs in = clearInputs();
  in.backstop_armed = true;
  in.shaft_angle = 50.0f;                    // 50 past angle_min(0) > margin(40)
  in.target = 5.0f;                          // still heading further into MIN
  LimitOutput out = computeTravelLimits(in);
  TEST_ASSERT_TRUE(out.disable);
  TEST_ASSERT_EQUAL_INT(1, out.backstop_fired);
}

void test_backstop_does_not_trip_when_backing_away() {
  LimitInputs in = clearInputs();
  in.backstop_armed = true;
  in.shaft_angle = 50.0f;                    // past the line...
  in.target = -5.0f;                         // ...but heading back toward MAX
  LimitOutput out = computeTravelLimits(in);
  TEST_ASSERT_FALSE(out.disable);
  TEST_ASSERT_EQUAL_INT(0, out.backstop_fired);
}

void test_vsafe_caps_velocity_when_armed() {
  LimitInputs in = clearInputs();
  in.backstop_armed = true; in.v_safe = 109.5f;
  in.target = 150.0f;
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 109.5f, computeTravelLimits(in).target);
  in.target = -150.0f;
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -109.5f, computeTravelLimits(in).target);
}

void test_soft_limit_velocity_mode() {
  LimitInputs in = clearInputs();
  in.homed = true; in.soft_enabled = true;
  in.home_offset = 100.0f; in.soft_max = 90.0f; in.soft_min = -90.0f;
  in.shaft_angle = 195.0f;                   // pos-from-home = 95 >= soft_max(90)
  in.target = 5.0f;                          // moving further out -> blocked
  TEST_ASSERT_EQUAL_FLOAT(0.0f, computeTravelLimits(in).target);
}

void test_angle_mode_holds_at_endstop() {
  LimitInputs in = clearInputs();
  in.angleMode = true; in.shaft_angle = 10.0f;
  in.maxT = true; in.target = 5.0f;          // target < shaft -> heading toward MAX
  LimitOutput out = computeTravelLimits(in);
  TEST_ASSERT_EQUAL_FLOAT(10.0f, out.target); // hold current shaft angle
}

// ----------------------------------------------------------------- motion profile
void test_vel_profile_accel_limited() {
  // single step can't exceed accel*dt
  float v = profileVelStep(0.0f, 10.0f, 300.0f, 0.01f);   // maxdv = 3
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 3.0f, v);
}

void test_vel_profile_reaches_and_holds_target() {
  float v = 0.0f;
  for (int i = 0; i < 1000; i++) v = profileVelStep(v, 10.0f, 300.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 10.0f, v);
  // once at target it stays put
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 10.0f, profileVelStep(10.0f, 10.0f, 300.0f, 0.01f));
}

void test_angle_profile_settles_exactly_on_target() {
  ProfileState s{0.0f, 0.0f};
  for (int i = 0; i < 5000 &&
       !(fabsf(s.pos - 50.0f) < 1e-6f && s.vel == 0.0f); i++) {
    s = profileAngleStep(s, 50.0f, 100.0f, 300.0f, 0.001f);
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 50.0f, s.pos);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, s.vel);   // stops, no overshoot/hunt
}

void test_angle_profile_respects_velocity_limit() {
  ProfileState s{0.0f, 0.0f};
  float vpeak = 0.0f;
  for (int i = 0; i < 2000; i++) {
    s = profileAngleStep(s, 1000.0f, 30.0f, 300.0f, 0.001f);  // far target, vmax=30
    if (fabsf(s.vel) > vpeak) vpeak = fabsf(s.vel);
  }
  TEST_ASSERT_TRUE(vpeak <= 30.0f + 1e-3f);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_clampf);
  RUN_TEST(test_backstop_margin);
  RUN_TEST(test_vsafe_allows_top_speed);
  RUN_TEST(test_glitch_passes_legit_fast_motion);
  RUN_TEST(test_glitch_rejects_impossible_jump);
  RUN_TEST(test_glitch_wraps_across_pi);
  RUN_TEST(test_hard_endstop_blocks_into_min);
  RUN_TEST(test_hard_endstop_allows_backing_away);
  RUN_TEST(test_backstop_trips_past_min);
  RUN_TEST(test_backstop_does_not_trip_when_backing_away);
  RUN_TEST(test_vsafe_caps_velocity_when_armed);
  RUN_TEST(test_soft_limit_velocity_mode);
  RUN_TEST(test_angle_mode_holds_at_endstop);
  RUN_TEST(test_vel_profile_accel_limited);
  RUN_TEST(test_vel_profile_reaches_and_holds_target);
  RUN_TEST(test_angle_profile_settles_exactly_on_target);
  RUN_TEST(test_angle_profile_respects_velocity_limit);
  return UNITY_END();
}
