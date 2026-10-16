// Trapezoidal (acceleration-limited) point-to-point profile for one joint,
// plus a helper that time-synchronises several joints so they all arrive
// together. Pure C++, no ROS dependency, unit-tested in test/test_trapezoid.cpp.
#pragma once

#include <cstddef>
#include <vector>

namespace arm_joint_control {

struct ProfileSample {
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
};

class TrapezoidProfile {
 public:
  // Plans the time-optimal profile from q0 to q1 under |v| <= v_max and
  // |a| <= a_max. Degenerates to a triangle when the move is too short to
  // reach v_max. Returns the total duration in seconds.
  double plan(double q0, double q1, double v_max, double a_max);

  // Plans a profile that takes exactly `duration` seconds (>= the time-optimal
  // duration) by lowering the cruise velocity. Used to synchronise joints.
  // Returns the actual duration (== `duration`, or the time-optimal duration
  // when the request was shorter than feasible).
  double planWithDuration(double q0, double q1, double a_max, double duration);

  ProfileSample sample(double t) const;

  double duration() const { return t_total_; }
  double start() const { return q0_; }
  double goal() const { return q1_; }
  bool valid() const { return planned_; }

 private:
  void build(double q0, double q1, double v_cruise, double a_max);

  double q0_ = 0.0, q1_ = 0.0;
  double dir_ = 1.0;
  double v_ = 0.0, a_ = 0.0;
  double t_acc_ = 0.0, t_cruise_ = 0.0, t_total_ = 0.0;
  bool planned_ = false;
};

// Plans one profile per joint, all finishing at the same time.
// Returns the common duration; throws std::invalid_argument on size mismatch.
double planSynchronised(const std::vector<double>& q0, const std::vector<double>& q1, const std::vector<double>& v_max,
                        const std::vector<double>& a_max, std::vector<TrapezoidProfile>& out);

}  // namespace arm_joint_control
