/**
 * @file line_follow_controller_node.cpp
 * @brief Line following controller using PD control on lateral and heading error.
 */

#include <algorithm>
#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/line_observation.hpp"

using namespace std::chrono_literals;

class LineFollowControllerNode : public rclcpp::Node {
public:
  LineFollowControllerNode() : Node("line_follow_controller_node") {
    // Declare parameters
    this->declare_parameter("control_rate_hz", 50.0);
    this->declare_parameter("kp_lateral", 2.0);
    this->declare_parameter("kd_lateral", 0.1);
    this->declare_parameter("kp_heading", 1.5);
    this->declare_parameter("kd_heading", 0.05);
    this->declare_parameter("base_speed_mps", 0.3);
    this->declare_parameter("max_lin_vel_mps", 0.5);
    this->declare_parameter("min_lin_vel_mps", 0.1);
    this->declare_parameter("max_ang_vel_rps", 1.5);
    this->declare_parameter("lost_line_timeout_s", 2.0);

    // Get parameters
    control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
    kp_lateral_ = this->get_parameter("kp_lateral").as_double();
    kd_lateral_ = this->get_parameter("kd_lateral").as_double();
    kp_heading_ = this->get_parameter("kp_heading").as_double();
    kd_heading_ = this->get_parameter("kd_heading").as_double();
    base_speed_ = this->get_parameter("base_speed_mps").as_double();
    max_lin_vel_ = this->get_parameter("max_lin_vel_mps").as_double();
    min_lin_vel_ = this->get_parameter("min_lin_vel_mps").as_double();
    max_ang_vel_ = this->get_parameter("max_ang_vel_rps").as_double();
    lost_line_timeout_ = this->get_parameter("lost_line_timeout_s").as_double();

    // Initialize state
    last_lateral_error_ = 0.0;
    last_heading_error_ = 0.0;
    line_valid_ = false;
    last_valid_time_ = this->now();

    // Subscriber
    line_sub_ = this->create_subscription<robot_interfaces::msg::LineObservation>(
        "/vision/line_observation",
        10,
        std::bind(
            &LineFollowControllerNode::line_callback,
            this,
            std::placeholders::_1));

    // Publisher
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/control/cmd_vel", 10);

    // Control timer
    double period_ms = 1000.0 / control_rate_hz_;
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(period_ms)),
        std::bind(&LineFollowControllerNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Line follow controller initialized");
  }

private:
  void line_callback(const robot_interfaces::msg::LineObservation::SharedPtr msg) {
    latest_observation_ = *msg;
    line_valid_ = msg->valid;
    if (line_valid_) {
      last_valid_time_ = this->now();
    }
  }

  void control_loop() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;

    // Check for line timeout
    double time_since_valid = (this->now() - last_valid_time_).seconds();
    if (!line_valid_ || time_since_valid > lost_line_timeout_) {
      // Lost line - stop
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "Line lost for %.1f seconds, stopping",
          time_since_valid);
      cmd_pub_->publish(cmd);
      return;
    }

    // Get errors
    double lateral_error = latest_observation_.lateral_error_m;
    double heading_error = latest_observation_.heading_error_rad;

    // Compute derivatives (simple finite difference)
    double dt = 1.0 / control_rate_hz_;
    double d_lateral = (lateral_error - last_lateral_error_) / dt;
    double d_heading = (heading_error - last_heading_error_) / dt;

    // PD control for angular velocity
    double omega = kp_lateral_ * lateral_error + kd_lateral_ * d_lateral +
                   kp_heading_ * heading_error + kd_heading_ * d_heading;

    // Clamp angular velocity
    omega = std::clamp(omega, -max_ang_vel_, max_ang_vel_);

    // Linear velocity (base speed, can be modulated by speed profile node)
    double v = base_speed_;
    v = std::clamp(v, min_lin_vel_, max_lin_vel_);

    // Update state
    last_lateral_error_ = lateral_error;
    last_heading_error_ = heading_error;

    // Publish command
    cmd.linear.x = v;
    cmd.angular.z = omega;
    cmd_pub_->publish(cmd);
  }

  // Parameters
  double control_rate_hz_;
  double kp_lateral_, kd_lateral_;
  double kp_heading_, kd_heading_;
  double base_speed_;
  double max_lin_vel_, min_lin_vel_;
  double max_ang_vel_;
  double lost_line_timeout_;

  // State
  robot_interfaces::msg::LineObservation latest_observation_;
  double last_lateral_error_;
  double last_heading_error_;
  bool line_valid_;
  rclcpp::Time last_valid_time_;

  // ROS interfaces
  rclcpp::Subscription<robot_interfaces::msg::LineObservation>::SharedPtr line_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineFollowControllerNode>());
  rclcpp::shutdown();
  return 0;
}
