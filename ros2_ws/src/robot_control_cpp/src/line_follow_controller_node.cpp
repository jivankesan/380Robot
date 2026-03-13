/**
 * @file line_follow_controller_node.cpp
 * @brief Line following controller using PD control on lateral and heading error.
 *
 * Control runs directly in the observation callback so it is locked to the
 * camera frame rate. dt is computed from observation timestamps, giving accurate
 * derivatives regardless of camera fps.
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
    this->declare_parameter("kp_lateral", 2.0);
    this->declare_parameter("kd_lateral", 0.1);
    this->declare_parameter("kp_heading", 1.5);
    this->declare_parameter("kd_heading", 0.05);
    this->declare_parameter("base_speed_mps", 0.3);
    this->declare_parameter("max_lin_vel_mps", 0.5);
    this->declare_parameter("min_lin_vel_mps", 0.1);
    this->declare_parameter("max_ang_vel_rps", 1.5);
    this->declare_parameter("lost_line_timeout_s", 2.0);
    this->declare_parameter("turn_speed_gain", 0.5);
    this->declare_parameter("min_turn_speed_mps", 0.1);
    this->declare_parameter("turn_omega_deadband_rps", 0.3);

    // Get parameters
    kp_lateral_ = this->get_parameter("kp_lateral").as_double();
    kd_lateral_ = this->get_parameter("kd_lateral").as_double();
    kp_heading_ = this->get_parameter("kp_heading").as_double();
    kd_heading_ = this->get_parameter("kd_heading").as_double();
    base_speed_ = this->get_parameter("base_speed_mps").as_double();
    max_lin_vel_ = this->get_parameter("max_lin_vel_mps").as_double();
    min_lin_vel_ = this->get_parameter("min_lin_vel_mps").as_double();
    max_ang_vel_ = this->get_parameter("max_ang_vel_rps").as_double();
    lost_line_timeout_ = this->get_parameter("lost_line_timeout_s").as_double();
    turn_speed_gain_ = this->get_parameter("turn_speed_gain").as_double();
    min_turn_speed_ = this->get_parameter("min_turn_speed_mps").as_double();
    turn_omega_deadband_ = this->get_parameter("turn_omega_deadband_rps").as_double();

    // Initialize state
    last_lateral_error_ = 0.0;
    last_heading_error_ = 0.0;
    line_valid_ = false;
    last_valid_time_ = this->now();
    last_obs_time_ = this->now();
    first_obs_ = true;
    fps_frame_count_ = 0;
    fps_window_start_ = this->now();

    // Subscriber — control runs inside this callback, locked to camera rate
    line_sub_ = this->create_subscription<robot_interfaces::msg::LineObservation>(
        "/vision/line_observation",
        10,
        std::bind(
            &LineFollowControllerNode::line_callback,
            this,
            std::placeholders::_1));

    // Publisher
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/control/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Line follow controller initialized (camera-rate control)");
  }

private:
  void line_callback(const robot_interfaces::msg::LineObservation::SharedPtr msg) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;

    line_valid_ = msg->valid;
    if (line_valid_) {
      last_valid_time_ = this->now();
    }

    // FPS logging — print every 2 seconds
    fps_frame_count_++;
    double fps_elapsed = (this->now() - fps_window_start_).seconds();
    if (fps_elapsed >= 2.0) {
      double fps = fps_frame_count_ / fps_elapsed;
      RCLCPP_INFO(this->get_logger(), "Control loop FPS: %.1f", fps);
      fps_frame_count_ = 0;
      fps_window_start_ = this->now();
    }

    // Check for line timeout
    double time_since_valid = (this->now() - last_valid_time_).seconds();
    if (!line_valid_ || time_since_valid > lost_line_timeout_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "Line lost for %.1f seconds, stopping",
          time_since_valid);
      cmd_pub_->publish(cmd);
      last_lateral_error_ = 0.0;
      last_heading_error_ = 0.0;
      first_obs_ = true;
      return;
    }

    // Compute dt from observation timestamps for accurate derivatives
    rclcpp::Time obs_time(msg->stamp);
    double dt = first_obs_ ? 0.0 : (obs_time - last_obs_time_).seconds();
    last_obs_time_ = obs_time;
    first_obs_ = false;

    // Get errors
    double lateral_error = msg->lateral_error_m;
    double heading_error = msg->heading_error_rad;

    // Derivatives — only valid when dt is reasonable
    double d_lateral = 0.0;
    double d_heading = 0.0;
    if (dt > 0.001 && dt < 0.5) {
      d_lateral = (lateral_error - last_lateral_error_) / dt;
      d_heading = (heading_error - last_heading_error_) / dt;
    }

    // PD control for angular velocity
    double omega = kp_lateral_ * lateral_error + kd_lateral_ * d_lateral +
                   kp_heading_ * heading_error + kd_heading_ * d_heading;

    // Clamp angular velocity
    omega = std::clamp(omega, -max_ang_vel_, max_ang_vel_);

    // Linear velocity: slow down only when turn is sharp enough
    double excess_omega = std::max(0.0, std::abs(omega) - turn_omega_deadband_);
    double v = base_speed_ - turn_speed_gain_ * excess_omega;
    v = std::clamp(v, min_turn_speed_, max_lin_vel_);

    // Update state
    last_lateral_error_ = lateral_error;
    last_heading_error_ = heading_error;

    // Publish command
    cmd.linear.x = v;
    cmd.angular.z = omega;
    cmd_pub_->publish(cmd);
  }

  // Parameters
  double kp_lateral_, kd_lateral_;
  double kp_heading_, kd_heading_;
  double base_speed_;
  double max_lin_vel_, min_lin_vel_;
  double max_ang_vel_;
  double lost_line_timeout_;
  double turn_speed_gain_;
  double min_turn_speed_;
  double turn_omega_deadband_;

  // State
  double last_lateral_error_;
  double last_heading_error_;
  bool line_valid_;
  rclcpp::Time last_valid_time_;
  rclcpp::Time last_obs_time_;
  bool first_obs_;
  int fps_frame_count_;
  rclcpp::Time fps_window_start_;

  // ROS interfaces
  rclcpp::Subscription<robot_interfaces::msg::LineObservation>::SharedPtr line_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineFollowControllerNode>());
  rclcpp::shutdown();
  return 0;
}
