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
    this->declare_parameter("reacquire_lin_vel_mps", 0.0);
    this->declare_parameter("reacquire_ang_vel_rps", 1.0);
    this->declare_parameter("reacquire_heading_deadband_rad", 0.05);
    this->declare_parameter("reacquire_lateral_deadband_m", 0.01);
    this->declare_parameter("turn_enter_heading_rad", 0.35);
    this->declare_parameter("turn_enter_confirm_s", 0.10);
    this->declare_parameter("turn_exit_heading_rad", 0.2);
    this->declare_parameter("turn_exit_confirm_s", 0.12);
    this->declare_parameter("turn_max_time_s", 1.5);
    this->declare_parameter("turn_blind_duration_s", 1.2);
    this->declare_parameter("turn_lin_vel_mps", 0.12);
    this->declare_parameter("turn_omega_rps", 1.4);

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
    reacquire_lin_vel_ = this->get_parameter("reacquire_lin_vel_mps").as_double();
    reacquire_ang_vel_ = this->get_parameter("reacquire_ang_vel_rps").as_double();
    reacquire_heading_deadband_ =
        this->get_parameter("reacquire_heading_deadband_rad").as_double();
    reacquire_lateral_deadband_ =
        this->get_parameter("reacquire_lateral_deadband_m").as_double();
    turn_enter_heading_ = this->get_parameter("turn_enter_heading_rad").as_double();
    turn_enter_confirm_ = this->get_parameter("turn_enter_confirm_s").as_double();
    turn_exit_heading_ = this->get_parameter("turn_exit_heading_rad").as_double();
    turn_exit_confirm_ = this->get_parameter("turn_exit_confirm_s").as_double();
    turn_max_time_ = this->get_parameter("turn_max_time_s").as_double();
    turn_blind_duration_ = this->get_parameter("turn_blind_duration_s").as_double();
    turn_lin_vel_ = this->get_parameter("turn_lin_vel_mps").as_double();
    turn_omega_ = this->get_parameter("turn_omega_rps").as_double();

    // Initialize state
    last_lateral_error_ = 0.0;
    last_heading_error_ = 0.0;
    line_valid_ = false;
    last_valid_time_ = this->now();
    last_turn_dir_ = 0;
    ever_valid_ = false;
    in_reacquire_ = false;
    reacquire_dir_ = 0;
    last_heading_sign_ = 0;
    last_lateral_sign_ = 0;
    in_turn_mode_ = false;
    turn_dir_ = 0;
    turn_start_time_ = this->now();
    turn_exit_candidate_ = false;
    turn_exit_candidate_start_ = this->now();
    turn_entry_candidate_ = false;
    turn_entry_candidate_start_ = this->now();
    turn_line_was_lost_ = false;
    turn_line_lost_time_ = this->now();

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
      last_valid_observation_ = *msg;
      ever_valid_ = true;
      in_reacquire_ = false;
      turn_line_was_lost_ = false;

      if (std::abs(msg->heading_error_rad) > reacquire_heading_deadband_) {
        last_heading_sign_ = (msg->heading_error_rad > 0.0) ? 1 : -1;
      }
      if (std::abs(msg->lateral_error_m) > reacquire_lateral_deadband_) {
        last_lateral_sign_ = (msg->lateral_error_m > 0.0) ? 1 : -1;
      }
    }
  }

  void control_loop() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;

    // Check for line timeout
    double time_since_valid = (this->now() - last_valid_time_).seconds();
    if (!line_valid_) {
      // If we're committed to a turn, keep turning for turn_blind_duration_s after losing line
      if (in_turn_mode_) {
        if (!turn_line_was_lost_) {
          turn_line_was_lost_ = true;
          turn_line_lost_time_ = this->now();
        }
        double time_blind = (this->now() - turn_line_lost_time_).seconds();
        if (time_blind <= turn_blind_duration_) {
          double omega = static_cast<double>(turn_dir_) * turn_omega_;
          omega = std::clamp(omega, -max_ang_vel_, max_ang_vel_);
          double v = std::clamp(turn_lin_vel_, 0.0, max_lin_vel_);
          cmd.linear.x = v;
          cmd.angular.z = omega;
          cmd_pub_->publish(cmd);
          return;
        }
        in_turn_mode_ = false;
        turn_line_was_lost_ = false;
      }

      if (!ever_valid_ || time_since_valid > lost_line_timeout_) {
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

      // Reacquire: pivot toward last seen heading for a short grace period
      if (!in_reacquire_) {
        int dir = 0;
        if (last_heading_sign_ != 0) {
          dir = last_heading_sign_;
        } else if (last_lateral_sign_ != 0) {
          dir = last_lateral_sign_;
        } else if (last_turn_dir_ != 0) {
          dir = last_turn_dir_;
        } else {
          dir = 1;
        }
        reacquire_dir_ = dir;
        in_reacquire_ = true;
      }

      double omega = static_cast<double>(reacquire_dir_) * reacquire_ang_vel_;
      omega = std::clamp(omega, -max_ang_vel_, max_ang_vel_);
      double v = std::clamp(reacquire_lin_vel_, 0.0, max_lin_vel_);

      cmd.linear.x = v;
      cmd.angular.z = omega;
      cmd_pub_->publish(cmd);
      return;
    }

    // Get errors
    double lateral_error = latest_observation_.lateral_error_m;
    double heading_error = latest_observation_.heading_error_rad;

    // Enter turn mode only after heading error persists for turn_enter_confirm_s
    if (!in_turn_mode_) {
      if (std::abs(heading_error) > turn_enter_heading_) {
        if (!turn_entry_candidate_) {
          turn_entry_candidate_ = true;
          turn_entry_candidate_start_ = this->now();
        } else if ((this->now() - turn_entry_candidate_start_).seconds() >= turn_enter_confirm_) {
          in_turn_mode_ = true;
          turn_dir_ = (heading_error >= 0.0) ? 1 : -1;
          turn_start_time_ = this->now();
          turn_exit_candidate_ = false;
          turn_entry_candidate_ = false;
        }
      } else {
        turn_entry_candidate_ = false;
      }
    }

    // If in turn mode, decide when to exit
    if (in_turn_mode_) {
      double time_in_turn = (this->now() - turn_start_time_).seconds();
      if (time_in_turn > turn_max_time_) {
        in_turn_mode_ = false;
      } else if (std::abs(heading_error) < turn_exit_heading_) {
        if (!turn_exit_candidate_) {
          turn_exit_candidate_ = true;
          turn_exit_candidate_start_ = this->now();
        } else {
          double exit_dt = (this->now() - turn_exit_candidate_start_).seconds();
          if (exit_dt > turn_exit_confirm_) {
            in_turn_mode_ = false;
          }
        }
      } else {
        turn_exit_candidate_ = false;
      }
    }

    // Compute derivatives (simple finite difference)
    double dt = 1.0 / control_rate_hz_;
    double d_lateral = (lateral_error - last_lateral_error_) / dt;
    double d_heading = (heading_error - last_heading_error_) / dt;

    // PD control for angular velocity
    double omega = kp_lateral_ * lateral_error + kd_lateral_ * d_lateral +
                   kp_heading_ * heading_error + kd_heading_ * d_heading;

    if (in_turn_mode_) {
      // Keep turning in the committed direction, but allow PD to shape magnitude
      if (omega * static_cast<double>(turn_dir_) < 0.0) {
        omega = static_cast<double>(turn_dir_) * std::abs(omega);
      }
      if (std::abs(omega) < turn_omega_) {
        omega = static_cast<double>(turn_dir_) * turn_omega_;
      }
    }

    // Clamp angular velocity
    omega = std::clamp(omega, -max_ang_vel_, max_ang_vel_);

    // Linear velocity (base speed, can be modulated by speed profile node)
    double v = base_speed_;
    if (in_turn_mode_) {
      v = std::min(v, turn_lin_vel_);
    }
    v = std::clamp(v, min_lin_vel_, max_lin_vel_);

    // Update state
    last_lateral_error_ = lateral_error;
    last_heading_error_ = heading_error;
    if (std::abs(omega) > 1e-3) {
      last_turn_dir_ = (omega > 0.0) ? 1 : -1;
    }

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
  double reacquire_lin_vel_;
  double reacquire_ang_vel_;
  double reacquire_heading_deadband_;
  double reacquire_lateral_deadband_;
  double turn_enter_heading_;
  double turn_enter_confirm_;
  double turn_exit_heading_;
  double turn_exit_confirm_;
  double turn_max_time_;
  double turn_blind_duration_;
  double turn_lin_vel_;
  double turn_omega_;

  // State
  robot_interfaces::msg::LineObservation latest_observation_;
  double last_lateral_error_;
  double last_heading_error_;
  bool line_valid_;
  rclcpp::Time last_valid_time_;
  robot_interfaces::msg::LineObservation last_valid_observation_;
  int last_turn_dir_;
  bool ever_valid_;
  bool in_reacquire_;
  int reacquire_dir_;
  int last_heading_sign_;
  int last_lateral_sign_;
  bool in_turn_mode_;
  int turn_dir_;
  rclcpp::Time turn_start_time_;
  bool turn_exit_candidate_;
  rclcpp::Time turn_exit_candidate_start_;
  bool turn_entry_candidate_;
  rclcpp::Time turn_entry_candidate_start_;
  bool turn_line_was_lost_;
  rclcpp::Time turn_line_lost_time_;

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
