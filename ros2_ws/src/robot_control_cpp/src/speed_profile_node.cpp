/**
 * @file speed_profile_node.cpp
 * @brief Trapezoidal speed profile node with acceleration limiting.
 *
 * Takes raw cmd_vel from controller and applies rate limiting + curvature-based slowdown.
 */

#include <algorithm>
#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/line_observation.hpp"

using namespace std::chrono_literals;

class SpeedProfileNode : public rclcpp::Node {
public:
  SpeedProfileNode() : Node("speed_profile_node") {
    // Declare parameters
    this->declare_parameter("v_max", 0.5);
    this->declare_parameter("v_min", 0.1);
    this->declare_parameter("a_max_accel", 2.0);
    this->declare_parameter("a_max_decel", 4.0);
    this->declare_parameter("alpha_max", 2.0);
    this->declare_parameter("curvature_slowdown_gain", 0.3);
    this->declare_parameter("error_slowdown_gain", 0.5);
    this->declare_parameter("heading_slowdown_gain", 0.2);
    this->declare_parameter("rate_hz", 50.0);

    // Get parameters
    v_max_ = this->get_parameter("v_max").as_double();
    v_min_ = this->get_parameter("v_min").as_double();
    a_max_accel_ = this->get_parameter("a_max_accel").as_double();
    a_max_decel_ = this->get_parameter("a_max_decel").as_double();
    alpha_max_ = this->get_parameter("alpha_max").as_double();
    k_curvature_ = this->get_parameter("curvature_slowdown_gain").as_double();
    k_error_ = this->get_parameter("error_slowdown_gain").as_double();
    k_heading_ = this->get_parameter("heading_slowdown_gain").as_double();
    rate_hz_ = this->get_parameter("rate_hz").as_double();

    // Initialize state
    v_cmd_ = 0.0;
    omega_cmd_ = 0.0;

    // Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/control/cmd_vel",
        10,
        std::bind(&SpeedProfileNode::cmd_vel_callback, this, std::placeholders::_1));

    line_sub_ = this->create_subscription<robot_interfaces::msg::LineObservation>(
        "/vision/line_observation",
        10,
        std::bind(&SpeedProfileNode::line_callback, this, std::placeholders::_1));

    // Publisher
    cmd_limited_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/control/cmd_vel_limited", 10);

    // Timer
    double period_ms = 1000.0 / rate_hz_;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(period_ms)),
        std::bind(&SpeedProfileNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Speed profile node initialized");
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    raw_v_ = msg->linear.x;
    raw_omega_ = msg->angular.z;
  }

  void line_callback(const robot_interfaces::msg::LineObservation::SharedPtr msg) {
    latest_line_ = *msg;
  }

  void timer_callback() {
    double dt = 1.0 / rate_hz_;

    // Compute target velocity based on path geometry
    double v_target = v_max_;

    if (latest_line_.valid) {
      double curvature = std::abs(latest_line_.curvature_1pm);
      double lateral_error = std::abs(latest_line_.lateral_error_m);
      double heading_error = std::abs(latest_line_.heading_error_rad);

      // Reduce speed based on curvature and errors
      v_target = v_max_ - k_curvature_ * curvature - k_error_ * lateral_error -
                 k_heading_ * heading_error;
    }

    // Use raw velocity as target if provided and lower
    if (raw_v_ < v_target && raw_v_ >= 0.0) {
      v_target = raw_v_;
    }

    // Clamp target
    v_target = std::clamp(v_target, v_min_, v_max_);

    // Apply asymmetric rate limiting: hard brake into turns, quick ramp on straights
    double a_limit = (v_target < v_cmd_) ? a_max_decel_ : a_max_accel_;
    double dv_max = a_limit * dt;
    double dv = std::clamp(v_target - v_cmd_, -dv_max, dv_max);
    v_cmd_ = std::clamp(v_cmd_ + dv, 0.0, v_max_);

    // Apply rate limiting to angular velocity
    double domega_max = alpha_max_ * dt;
    double domega = std::clamp(raw_omega_ - omega_cmd_, -domega_max, domega_max);
    omega_cmd_ += domega;

    // Hard stop: snap both to zero only when truly commanded to stop (v=0 AND omega=0).
    // When v=0 but omega!=0 (spin in place), allow omega through normally.
    if (raw_v_ <= 0.0 && std::abs(raw_omega_) < 0.01) {
      v_cmd_ = 0.0;
      omega_cmd_ = 0.0;
    } else if (raw_v_ <= 0.0) {
      v_cmd_ = 0.0;
    }

    // Publish limited command
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v_cmd_;
    cmd.angular.z = omega_cmd_;
    cmd_limited_pub_->publish(cmd);
  }

  // Parameters
  double v_max_, v_min_;
  double a_max_accel_, a_max_decel_, alpha_max_;
  double k_curvature_, k_error_, k_heading_;
  double rate_hz_;

  // State
  double v_cmd_;
  double omega_cmd_;
  double raw_v_ = 0.0;
  double raw_omega_ = 0.0;
  robot_interfaces::msg::LineObservation latest_line_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<robot_interfaces::msg::LineObservation>::SharedPtr line_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_limited_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedProfileNode>());
  rclcpp::shutdown();
  return 0;
}
