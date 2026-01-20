/**
 * @file dummy_hw_node.cpp
 * @brief Dummy hardware node for development without actual Arduino.
 *
 * Simulates the hardware interface by printing commands and publishing fake telemetry.
 */

#include <chrono>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/claw_command.hpp"
#include "robot_interfaces/msg/hw_status.hpp"

using namespace std::chrono_literals;

class DummyHwNode : public rclcpp::Node {
public:
  DummyHwNode() : Node("dummy_hw_node") {
    // Declare parameters
    this->declare_parameter("print_commands", true);
    this->declare_parameter("telemetry_rate_hz", 20.0);
    this->declare_parameter("simulated_battery_v", 12.0);

    // Get parameters
    print_commands_ = this->get_parameter("print_commands").as_bool();
    telemetry_rate_hz_ = this->get_parameter("telemetry_rate_hz").as_double();
    battery_v_ = this->get_parameter("simulated_battery_v").as_double();

    // Initialize state
    left_enc_ = 0;
    right_enc_ = 0;
    claw_state_ = "open";

    // Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/control/cmd_vel_limited",
        10,
        std::bind(&DummyHwNode::cmd_vel_callback, this, std::placeholders::_1));

    claw_sub_ = this->create_subscription<robot_interfaces::msg::ClawCommand>(
        "/claw/cmd",
        10,
        std::bind(&DummyHwNode::claw_callback, this, std::placeholders::_1));

    // Publisher
    status_pub_ = this->create_publisher<robot_interfaces::msg::HwStatus>("/hw/status", 10);

    // Telemetry timer
    double period_ms = 1000.0 / telemetry_rate_hz_;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(period_ms)),
        std::bind(&DummyHwNode::publish_status, this));

    RCLCPP_INFO(this->get_logger(), "Dummy HW node initialized (no real hardware)");
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (print_commands_) {
      RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          500,
          "[DUMMY HW] Motor cmd: v=%.2f, omega=%.2f",
          msg->linear.x,
          msg->angular.z);
    }

    // Simulate encoder movement
    double dt = 0.02;  // Approximate update rate
    double v = msg->linear.x;
    double omega = msg->angular.z;
    double wheel_base = 0.15;

    double v_left = v - omega * wheel_base / 2.0;
    double v_right = v + omega * wheel_base / 2.0;

    // Ticks per meter (arbitrary)
    double ticks_per_m = 1000.0;
    left_enc_ += static_cast<int>(v_left * dt * ticks_per_m);
    right_enc_ += static_cast<int>(v_right * dt * ticks_per_m);
  }

  void claw_callback(const robot_interfaces::msg::ClawCommand::SharedPtr msg) {
    std::string mode_str;
    switch (msg->mode) {
      case robot_interfaces::msg::ClawCommand::MODE_OPEN:
        mode_str = "OPEN";
        claw_state_ = "open";
        break;
      case robot_interfaces::msg::ClawCommand::MODE_CLOSE:
        mode_str = "CLOSE";
        claw_state_ = "closed";
        break;
      case robot_interfaces::msg::ClawCommand::MODE_HOLD:
        mode_str = "HOLD";
        break;
      default:
        mode_str = "UNKNOWN";
    }

    if (print_commands_) {
      RCLCPP_INFO(
          this->get_logger(),
          "[DUMMY HW] Claw cmd: mode=%s, pos=%.2f",
          mode_str.c_str(),
          msg->position);
    }
  }

  void publish_status() {
    robot_interfaces::msg::HwStatus status;
    status.stamp = this->now().operator builtin_interfaces::msg::Time();
    status.battery_v = static_cast<float>(battery_v_);
    status.left_enc = left_enc_;
    status.right_enc = right_enc_;
    status.estop = false;
    status.last_error = "";

    status_pub_->publish(status);
  }

  // Parameters
  bool print_commands_;
  double telemetry_rate_hz_;
  double battery_v_;

  // Simulated state
  int left_enc_;
  int right_enc_;
  std::string claw_state_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<robot_interfaces::msg::ClawCommand>::SharedPtr claw_sub_;
  rclcpp::Publisher<robot_interfaces::msg::HwStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyHwNode>());
  rclcpp::shutdown();
  return 0;
}
