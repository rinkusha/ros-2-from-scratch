#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "arm_joint_control/trapezoid_profile.hpp"

using arm_joint_control::planSynchronised;
using arm_joint_control::TrapezoidProfile;

namespace {

// Integrates the sampled velocity and checks it matches the sampled position.
void expectConsistent(const TrapezoidProfile& p, double v_max, double a_max) {
  const double dt = 1e-4;
  double q = p.sample(0.0).position;
  double v_prev = p.sample(0.0).velocity;
  for (double t = dt; t <= p.duration() + 1e-12; t += dt) {
    const auto s = p.sample(t);
    q += 0.5 * (v_prev + s.velocity) * dt;
    EXPECT_NEAR(s.position, q, 1e-4) << "t=" << t;
    EXPECT_LE(std::fabs(s.velocity), v_max + 1e-9);
    EXPECT_LE(std::fabs((s.velocity - v_prev) / dt), a_max + 1e-6);
    v_prev = s.velocity;
  }
}

}  // namespace

TEST(Trapezoid, LongMoveHasCruisePhase) {
  TrapezoidProfile p;
  const double T = p.plan(0.0, 2.0, 1.0, 2.0);  // v_max 1 rad/s, a 2 rad/s^2
  // t_acc = 0.5 s, d_acc = 0.25 rad, cruise = 1.5 rad at 1 rad/s = 1.5 s
  EXPECT_NEAR(T, 2.5, 1e-9);
  EXPECT_NEAR(p.sample(1.25).velocity, 1.0, 1e-9);
  EXPECT_NEAR(p.sample(T).position, 2.0, 1e-9);
  EXPECT_NEAR(p.sample(T).velocity, 0.0, 1e-9);
  expectConsistent(p, 1.0, 2.0);
}

TEST(Trapezoid, ShortMoveIsTriangular) {
  TrapezoidProfile p;
  const double T = p.plan(1.0, 1.1, 5.0, 2.0);  // d = 0.1, peak v = sqrt(0.2) < 5
  EXPECT_NEAR(T, 2.0 * std::sqrt(0.1 / 2.0), 1e-9);
  EXPECT_LT(std::fabs(p.sample(T / 2).velocity), 5.0);
  EXPECT_NEAR(p.sample(T / 2).velocity, std::sqrt(0.2), 1e-9);
  expectConsistent(p, 5.0, 2.0);
}

TEST(Trapezoid, NegativeDirection) {
  TrapezoidProfile p;
  const double T = p.plan(0.5, -1.5, 1.0, 4.0);
  EXPECT_GT(T, 0.0);
  EXPECT_LT(p.sample(T / 2).velocity, 0.0);
  EXPECT_NEAR(p.sample(T).position, -1.5, 1e-9);
  expectConsistent(p, 1.0, 4.0);
}

TEST(Trapezoid, ZeroMoveIsInstant) {
  TrapezoidProfile p;
  EXPECT_DOUBLE_EQ(p.plan(0.7, 0.7, 1.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(p.sample(0.0).position, 0.7);
  EXPECT_DOUBLE_EQ(p.sample(5.0).position, 0.7);
  EXPECT_DOUBLE_EQ(p.sample(5.0).velocity, 0.0);
}

TEST(Trapezoid, SamplesClampOutsideRange) {
  TrapezoidProfile p;
  const double T = p.plan(0.0, 1.0, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(p.sample(-1.0).position, 0.0);
  EXPECT_DOUBLE_EQ(p.sample(T + 10.0).position, 1.0);
}

TEST(Trapezoid, StretchedDurationLowersCruiseSpeed) {
  TrapezoidProfile fast, slow;
  const double T = fast.plan(0.0, 2.0, 1.0, 2.0);
  const double T2 = slow.planWithDuration(0.0, 2.0, 2.0, 2.0 * T);
  EXPECT_NEAR(T2, 2.0 * T, 1e-9);
  double v_peak = 0.0;
  for (double t = 0.0; t <= T2; t += 0.001) v_peak = std::max(v_peak, slow.sample(t).velocity);
  EXPECT_LT(v_peak, 1.0);
  EXPECT_NEAR(slow.sample(T2).position, 2.0, 1e-6);
  expectConsistent(slow, 1.0, 2.0);
}

TEST(Trapezoid, StretchShorterThanFeasibleFallsBackToOptimal) {
  TrapezoidProfile p;
  const double T = p.planWithDuration(0.0, 1.0, 1.0, 0.1);  // needs 2 s
  EXPECT_NEAR(T, 2.0, 1e-9);
}

TEST(Trapezoid, SynchronisedJointsArriveTogether) {
  std::vector<double> q0 = {0.0, 0.0, 1.0};
  std::vector<double> q1 = {3.0, 0.2, 1.0};  // long, short, stationary
  std::vector<double> v_max = {1.0, 1.0, 1.0};
  std::vector<double> a_max = {2.0, 2.0, 2.0};
  std::vector<TrapezoidProfile> profiles;
  const double T = planSynchronised(q0, q1, v_max, a_max, profiles);
  ASSERT_EQ(profiles.size(), 3u);
  for (const auto& p : profiles) EXPECT_NEAR(p.duration(), T, 1e-9);
  // The short joint must still be moving near the end (not finish early and wait).
  EXPECT_GT(std::fabs(profiles[1].sample(0.9 * T).velocity), 0.0);
  EXPECT_NEAR(profiles[1].sample(T).position, 0.2, 1e-9);
  EXPECT_DOUBLE_EQ(profiles[2].sample(0.5 * T).position, 1.0);
  for (std::size_t i = 0; i < 2; ++i) expectConsistent(profiles[i], v_max[i], a_max[i]);
}

TEST(Trapezoid, RejectsBadLimits) {
  TrapezoidProfile p;
  EXPECT_THROW(p.plan(0.0, 1.0, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(p.plan(0.0, 1.0, 1.0, -1.0), std::invalid_argument);
}
