// ackermann_planner_demo.cpp
#include "ackermann_planner.hpp"
#include <iostream>

int main() {
  // Example 4-wheel-steer geometry (meters): positions of wheel contact points in body frame
  AckermannPlanner::WheelMap wheels{
    {"front_left",  Eigen::Vector2d(+0.25, +0.20)},
    {"front_right", Eigen::Vector2d(+0.25, -0.20)},
    {"rear_left",   Eigen::Vector2d(-0.25, +0.20)},
    {"rear_right",  Eigen::Vector2d(-0.25, -0.20)},
  };
  AckermannPlanner::NameSet steerable{"front_left","front_right","rear_left","rear_right"};

  RoverLimits limits;
  limits.wheel_diameter = 0.20;        // 20 cm wheels
  limits.wheel_speed_max = 60.0;       // rpm
  limits.steer_angle_max = M_PI / 6.0;   // 30 deg
  limits.accel_max = 0.8;              // m/s^2
  limits.decel_max = 1.2;              // m/s^2

  AckermannPlanner planner(wheels, steerable, limits);

  Eigen::Vector3d start(0.0, 0.0, 0.0);
  Eigen::Vector3d goal(3.0, 2.0, M_PI / 2.0);
  auto traj = planner.plan(start, goal, 0.02, 0.01);

  std::cout << "Total time: " << traj.t(traj.t.size() - 1) << " s, samples: " << traj.t.size() << "\n";
  std::cout << "First sample kappa: " << traj.kappa(0) << ", v: " << traj.v(0) << "\n";
  std::cout << "Last sample kappa: " << traj.kappa(traj.kappa.size() - 1) << ", v: " << traj.v(traj.v.size() - 1) << "\n";
  return 0;
}
