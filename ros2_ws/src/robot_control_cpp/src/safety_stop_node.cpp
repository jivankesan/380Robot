/**
 * @file safety_stop_node.cpp
 * @brief Safety monitoring node that stops the robot on faults or timeouts.
 */

#include <chrono>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/hw_status.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class SafetyStopNode : public rclcpp::Node {
public:
  SafetyStopNode() : Node("safety_stop_node") {
    // Declare parameters
    this->declare_parameter("cmd_vel_timeout_s", 0.5);
    this->declare_parameter("hw_status_timeout_s", 1.0);
    this->declare_parameter("min_battery_v", 10.0);
    this->declare_parameter("rate_hz", 20.0);

    // Get parameters
    cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout_s").as_double();
    hw_status_timeout_ = this->get_parameter("hw_status_timeout_s").as_double();
    min_battery_v_ = this->get_parameter("min_battery_v").as_double();
    rate_hz_ = this->get_parameter("rate_hz").as_double();

    // Initialize state
    estop_active_ = false;
    last_cmd_time_ = this->now();
    last_hw_status_time_ = this->now();
    hw_estop_ = false;
    battery_low_ = false;

    // Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/control/cmd_vel_limited",
        10,
        std::bind(&SafetyStopNode::cmd_vel_callback, this, std::placeholders::_1));

    hw_status_sub_ = this->create_subscription<robot_interfaces::msg::HwStatus>(
        "/hw/status",
        10,
        std::bind(&SafetyStopNode::hw_status_callback, this, std::placeholders::_1));

    // Publishers
    estop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/control/estop", 10);
    safe_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/hw/cmd_vel", 10);

    // Timer
    double period_ms = 1000.0 / rate_hz_;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(period_ms)),
        std::bind(&SafetyStopNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Safety stop node initialized");
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_cmd_ = *msg;
    last_cmd_time_ = this->now();
  }

  void hw_status_callback(const robot_interfaces::msg::HwStatus::SharedPtr msg) {
    last_hw_status_time_ = this->now();
    hw_estop_ = msg->estop;
    battery_low_ = (msg->battery_v > 0.0 && msg->battery_v < min_battery_v_);

    if (hw_estop_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000, "Hardware E-STOP active");
    }
    if (battery_low_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          5000,
          "Low battery: %.1fV",
          msg->battery_v);
    }
  }

  void timer_callback() {
    // Check for various fault conditions
    bool cmd_timeout = (this->now() - last_cmd_time_).seconds() > cmd_vel_timeout_;
    bool hw_timeout = (this->now() - last_hw_status_time_).seconds() > hw_status_timeout_;

    bool should_stop = cmd_timeout || hw_timeout || hw_estop_ || battery_low_;

    // Update estop state
    if (should_stop && !estop_active_) {
      estop_active_ = true;
      RCLCPP_WARN(this->get_logger(), "Safety stop activated");
      if (cmd_timeout) {
        RCLCPP_WARN(this->get_logger(), "  Reason: Command timeout");
      }
      if (hw_timeout) {
        RCLCPP_WARN(this->get_logger(), "  Reason: Hardware status timeout");
      }
      if (hw_estop_) {
        RCLCPP_WARN(this->get_logger(), "  Reason: Hardware E-STOP");
      }
      if (battery_low_) {
        RCLCPP_WARN(this->get_logger(), "  Reason: Low battery");
      }
    } else if (!should_stop && estop_active_) {
      estop_active_ = false;
      RCLCPP_INFO(this->get_logger(), "Safety stop cleared");
    }

    // Publish estop status
    std_msgs::msg::Bool estop_msg;
    estop_msg.data = estop_active_;
    estop_pub_->publish(estop_msg);

    // Publish safe command (zero if estopped, otherwise pass through)
    geometry_msgs::msg::Twist safe_cmd;
    if (estop_active_) {
      safe_cmd.linear.x = 0.0;
      safe_cmd.angular.z = 0.0;
    } else {
      safe_cmd = last_cmd_;
    }
    safe_cmd_pub_->publish(safe_cmd);
  }

  // Parameters
  double cmd_vel_timeout_;
  double hw_status_timeout_;
  double min_battery_v_;
  double rate_hz_;

  // State
  bool estop_active_;
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_hw_status_time_;
  bool hw_estop_;
  bool battery_low_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<robot_interfaces::msg::HwStatus>::SharedPtr hw_status_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyStopNode>());
  rclcpp::shutdown();
  return 0;
}
