#include "rover_sensors/sim/sim_imu.hpp"

SimulatedIMU::SimulatedIMU() : Node("simulated_imu") {
  this->imuPub = this->create_publisher<sensor_msgs::msg::Imu>("sensors/raw/imu", 10);
  double freq = this->declare_parameter<double>("sensor_frequency", 100.0);
  freq = this->get_parameter("sensor_frequency", freq);
  this->period = 1.0 / freq;
  this->timestep = 0.0;

  // Initialize the static transform broadcaster
  this->staticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  {
    // Publish the transform from the base_link to the sensor link
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "imu_link";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    this->staticTfBroadcaster->sendTransform(transformStamped);
  }

  timer = this->create_wall_timer(
    std::chrono::duration<double>(this->period),
    [this]() {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";
    // Simulate some motion: e.g., sinusoidal angular velocity and acceleration
    msg.linear_acceleration.x = 0.7 * std::sin(timestep);
    msg.linear_acceleration.y = 0.01 * std::cos(timestep);
    msg.linear_acceleration.z = -9.81; // gravity
    msg.angular_velocity.z = 0.05 * std::sin(0.5 * timestep);
    msg.orientation.w = 1.0;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    imuPub->publish(msg);
    this->timestep += this->period;
  }
  );
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatedIMU>());
  rclcpp::shutdown();
  return 0;
}