#include "arm_joint_control/trapezoid_profile.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace arm_joint_control {

namespace {
constexpr double kEps = 1e-9;
}

void TrapezoidProfile::build(double q0, double q1, double v_cruise, double a_max) {
  q0_ = q0;
  q1_ = q1;
  const double d = std::fabs(q1 - q0);
  dir_ = (q1 >= q0) ? 1.0 : -1.0;
  a_ = a_max;
  v_ = v_cruise;
  if (d < kEps || v_cruise <= 0.0 || a_max <= 0.0) {
    v_ = 0.0;
    t_acc_ = 0.0;
    t_cruise_ = 0.0;
    t_total_ = 0.0;
    planned_ = true;
    return;
  }
  t_acc_ = v_ / a_;
  const double d_acc = 0.5 * a_ * t_acc_ * t_acc_;
  t_cruise_ = std::max(0.0, (d - 2.0 * d_acc) / v_);
  t_total_ = 2.0 * t_acc_ + t_cruise_;
  planned_ = true;
}

double TrapezoidProfile::plan(double q0, double q1, double v_max, double a_max) {
  if (v_max <= 0.0 || a_max <= 0.0) throw std::invalid_argument("v_max and a_max must be > 0");
  const double d = std::fabs(q1 - q0);
  if (d < kEps) {
    build(q0, q1, 0.0, a_max);
    return 0.0;
  }
  // Velocity reached at the midpoint of a pure triangle; cap at v_max.
  const double v_peak = std::min(v_max, std::sqrt(d * a_max));
  build(q0, q1, v_peak, a_max);
  return t_total_;
}

double TrapezoidProfile::planWithDuration(double q0, double q1, double a_max, double duration) {
  if (a_max <= 0.0) throw std::invalid_argument("a_max must be > 0");
  const double d = std::fabs(q1 - q0);
  if (d < kEps) {
    build(q0, q1, 0.0, a_max);
    t_total_ = std::max(0.0, duration);  // park for the requested time
    return t_total_;
  }
  const double t_min = 2.0 * std::sqrt(d / a_max);  // pure triangle
  if (duration <= t_min) {
    build(q0, q1, std::sqrt(d * a_max), a_max);
    return t_total_;
  }
  // Trapezoid with cruise speed v: T = d/v + v/a  ->  v^2 - T a v + d a = 0.
  // Take the smaller root (the larger one is the mirror solution with v > v_peak).
  const double disc = duration * duration * a_max * a_max - 4.0 * d * a_max;
  const double v = 0.5 * (duration * a_max - std::sqrt(std::max(0.0, disc)));
  build(q0, q1, v, a_max);
  t_total_ = duration;  // absorb floating point residue so joints line up exactly
  return t_total_;
}

ProfileSample TrapezoidProfile::sample(double t) const {
  ProfileSample s;
  if (!planned_) return s;
  const double d = std::fabs(q1_ - q0_);
  if (t <= 0.0 || v_ <= 0.0) {
    s.position = q0_;
    if (v_ <= 0.0 && t >= t_total_) s.position = q1_;
    return s;
  }
  if (t >= t_total_) {
    s.position = q1_;
    return s;
  }
  double p, v, a;
  const double d_acc = 0.5 * a_ * t_acc_ * t_acc_;
  if (t < t_acc_) {
    p = 0.5 * a_ * t * t;
    v = a_ * t;
    a = a_;
  } else if (t < t_acc_ + t_cruise_) {
    p = d_acc + v_ * (t - t_acc_);
    v = v_;
    a = 0.0;
  } else {
    const double tr = t_total_ - t;
    p = d - 0.5 * a_ * tr * tr;
    v = a_ * tr;
    a = -a_;
  }
  s.position = q0_ + dir_ * p;
  s.velocity = dir_ * v;
  s.acceleration = dir_ * a;
  return s;
}

double planSynchronised(const std::vector<double>& q0, const std::vector<double>& q1, const std::vector<double>& v_max,
                        const std::vector<double>& a_max, std::vector<TrapezoidProfile>& out) {
  const std::size_t n = q0.size();
  if (q1.size() != n || v_max.size() != n || a_max.size() != n) {
    throw std::invalid_argument("planSynchronised: vector sizes differ");
  }
  out.assign(n, TrapezoidProfile{});
  double t_common = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    t_common = std::max(t_common, out[i].plan(q0[i], q1[i], v_max[i], a_max[i]));
  }
  for (std::size_t i = 0; i < n; ++i) {
    out[i].planWithDuration(q0[i], q1[i], a_max[i], t_common);
  }
  return t_common;
}

}  // namespace arm_joint_control
