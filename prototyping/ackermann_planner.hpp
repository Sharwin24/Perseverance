// ackermann_planner.hpp
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

struct RoverLimits {
  double wheel_diameter;   // [m]
  double wheel_speed_max;  // [rpm] rotational speed
  double steer_angle_max;  // [rad]
  double accel_max;        // [m/s^2]
  double decel_max;        // [m/s^2]

  double linear_speed_max() const {
    // convert rpm to m/s at tread: v = (pi * D) * rpm / 60
    return (wheel_diameter * M_PI / 60.0) * wheel_speed_max;
  }
};

struct PlanResult {
  Eigen::VectorXd t;       // time samples
  Eigen::VectorXd x;       // centerline x
  Eigen::VectorXd y;       // centerline y
  Eigen::VectorXd theta;   // heading
  Eigen::VectorXd kappa;   // curvature
  Eigen::VectorXd v;       // centerline speed

  std::map<std::string, Eigen::VectorXd> wheel_angles; // rad
  std::map<std::string, Eigen::VectorXd> wheel_speeds; // m/s
  std::map<std::string, Eigen::VectorXd> wheel_omegas; // rad/s
};

class AckermannPlanner {
public:
  using WheelMap = std::unordered_map<std::string, Eigen::Vector2d>; // body-frame (x fwd, y left)
  using NameSet = std::unordered_set<std::string>;

  AckermannPlanner(const WheelMap& wheel_locations,
    const NameSet& steerable,
    const RoverLimits& limits);

  PlanResult plan(const Eigen::Vector3d& start_pose,
    const Eigen::Vector3d& goal_pose,
    double dt = 0.02,
    double ds_prop = 0.01) const;

  double Rmin() const { return Rmin_; }
  double kappa_max() const { return kappa_max_; }

private:
  // geometry & limits
  WheelMap wheels_;
  NameSet steerable_;
  RoverLimits limits_;

  double kappa_max_{0.0};
  double Rmin_{0.0};

  // helpers
  static double wrap_to_pi(double a);
  double kappa_max_from_geometry(double steer_angle_max) const;
  std::unordered_map<std::string, double> steer_angles_from_kappa(double kappa) const;
  std::unordered_map<std::string, double> per_wheel_speed(double v_center,
    double kappa,
    const std::unordered_map<std::string, double>& wheel_angles) const;

  using Seg = std::pair<char, double>; // ('L'|'R'|'S', length_m)
  std::vector<Seg> dubins_shortest_path(const Eigen::Vector3d& q0,
    const Eigen::Vector3d& q1,
    double Rmin) const;

  void propagate_exact(const Eigen::Vector3d& q0,
    const std::vector<Seg>& segs,
    double Rmin,
    double ds,
    std::vector<double>& ss,
    std::vector<double>& xs,
    std::vector<double>& ys,
    std::vector<double>& ths,
    std::vector<double>& ks) const;

  void time_scale_with_wheels(const std::vector<double>& ss,
    const std::vector<double>& kappas,
    std::vector<double>& t,
    std::vector<double>& v) const;

  PlanResult sample_uniform_dt(const std::vector<double>& t,
    const std::vector<double>& ss,
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    const std::vector<double>& ths,
    const std::vector<double>& kappas,
    const std::vector<double>& v,
    double dt) const;
};
