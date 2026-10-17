#include <gtest/gtest.h>

#include <cmath>

#include "arm_common/filters.hpp"
#include "arm_common/pid.hpp"

using namespace arm;

namespace {

// Minimal joint plant for simulation: a motor command u in [-1,1] produces
// angular acceleration (u * gain - friction * v). Good enough to check the
// controller's qualitative behaviour (convergence, no windup, no kick).
struct Plant {
  float pos = 0.0f;
  float vel = 0.0f;
  float gain = 40.0f;  // rad/s^2 per unit command
  float friction = 2.0f;
  void step(float u, float dt) {
    const float acc = u * gain - friction * vel;
    vel += acc * dt;
    pos += vel * dt;
  }
};

}  // namespace

TEST(Pid, ProportionalOnlyConvergesToStep) {
  Pid pid;
  PidGains g;
  g.kp = 2.0f;
  g.kd = 0.15f;
  g.out_limit = 1.0f;
  pid.setGains(g);
  Plant plant;
  const float dt = 0.001f;
  for (int i = 0; i < 3000; ++i) {
    const float u = pid.update(1.0f, 0.0f, plant.pos, plant.vel, dt);
    plant.step(u, dt);
  }
  EXPECT_NEAR(plant.pos, 1.0f, 0.05f);
  EXPECT_NEAR(plant.vel, 0.0f, 0.1f);
}

TEST(Pid, IntegralRemovesSteadyStateErrorUnderLoad) {
  Pid pid;
  PidGains g;
  g.kp = 2.0f;
  g.ki = 4.0f;
  g.kd = 0.15f;
  g.i_limit = 0.6f;
  g.out_limit = 1.0f;
  pid.setGains(g);
  Plant plant;
  const float dt = 0.001f;
  const float load = -0.3f;  // constant disturbance (gravity on a joint)
  for (int i = 0; i < 6000; ++i) {
    const float u = pid.update(0.5f, 0.0f, plant.pos, plant.vel, dt);
    plant.step(u + load, dt);
  }
  EXPECT_NEAR(plant.pos, 0.5f, 0.02f);
  // The integral has to be carrying the load.
  EXPECT_NEAR(pid.integral(), 0.3f, 0.05f);
}

TEST(Pid, IntegralIsClampedAndDoesNotWindUpWhileSaturated) {
  Pid pid;
  PidGains g;
  g.kp = 1.0f;
  g.ki = 50.0f;
  g.i_limit = 0.2f;
  g.out_limit = 1.0f;
  pid.setGains(g);
  // Plant stuck: measured position never moves, output saturates immediately.
  for (int i = 0; i < 2000; ++i) {
    const float u = pid.update(10.0f, 0.0f, 0.0f, 0.0f, 0.001f);
    EXPECT_LE(u, 1.0f);
  }
  EXPECT_TRUE(pid.saturated());
  EXPECT_LE(std::fabs(pid.integral()), 0.2f + 1e-6f);
}

TEST(Pid, NoDerivativeKickOnSetpointStep) {
  Pid pid;
  PidGains g;
  g.kp = 0.5f;
  g.kd = 0.5f;
  g.out_limit = 1.0f;
  pid.setGains(g);
  // Measured state at rest, setpoint jumps: derivative-on-measurement means the
  // output is exactly kp * e, with no extra spike from the step.
  pid.update(0.0f, 0.0f, 0.0f, 0.0f, 0.001f);
  const float u = pid.update(1.0f, 0.0f, 0.0f, 0.0f, 0.001f);
  EXPECT_FLOAT_EQ(u, 0.5f);
}

TEST(Pid, VelocityFeedForwardAndEffortFeedForwardAdd) {
  Pid pid;
  PidGains g;
  g.kff_v = 0.1f;
  g.out_limit = 1.0f;
  pid.setGains(g);
  const float u = pid.update(0.0f, 2.0f, 0.0f, 0.0f, 0.001f, 0.25f);
  EXPECT_FLOAT_EQ(u, 0.2f + 0.25f);
}

TEST(Pid, OutputClampedToLimit) {
  Pid pid;
  PidGains g;
  g.kp = 100.0f;
  g.out_limit = 0.7f;
  pid.setGains(g);
  EXPECT_FLOAT_EQ(pid.update(1.0f, 0.0f, 0.0f, 0.0f, 0.001f), 0.7f);
  EXPECT_FLOAT_EQ(pid.update(-1.0f, 0.0f, 0.0f, 0.0f, 0.001f), -0.7f);
}

TEST(LowPass, TracksStepWithExpectedTimeConstant) {
  LowPass lp;
  lp.setCutoff(10.0f);  // tau ~ 15.9 ms
  lp.reset(0.0f);
  float y = 0.0f;
  for (int i = 0; i < 16; ++i) y = lp.update(1.0f, 0.001f);
  EXPECT_NEAR(y, 1.0f - std::exp(-1.0f), 0.05f);
  for (int i = 0; i < 200; ++i) y = lp.update(1.0f, 0.001f);
  EXPECT_NEAR(y, 1.0f, 1e-3f);
}

TEST(LowPass, FirstSampleInitialises) {
  LowPass lp;
  lp.setCutoff(5.0f);
  EXPECT_FLOAT_EQ(lp.update(3.0f, 0.001f), 3.0f);
}
