/**
 * @file serial_bridge_node.cpp
 * @brief Serial bridge to Arduino for motor and claw control.
 *
 * Protocol (ASCII, line-based):
 * - Motor command: M,<left_pwm>,<right_pwm>\n
 * - Claw command:  C,<mode>,<pos>\n
 * - Telemetry:     T,<battery_mv>,<left_enc>,<right_enc>,<estop>\n
 * - Error:         E,<code>,<msg>\n
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/claw_command.hpp"
#include "robot_interfaces/msg/hw_status.hpp"

using namespace std::chrono_literals;

class SerialBridgeNode : public rclcpp::Node {
public:
  SerialBridgeNode() : Node("serial_bridge_node"), serial_fd_(-1) {
    // Declare parameters
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("wheel_base_m", 0.15);
    this->declare_parameter("wheel_radius_m", 0.035);
    this->declare_parameter("max_pwm", 255);
    this->declare_parameter("cmd_rate_hz", 50.0);
    this->declare_parameter("watchdog_timeout_ms", 250);
    this->declare_parameter("telemetry_rate_hz", 20.0);
    this->declare_parameter("left_motor_gain", 1.0);
    this->declare_parameter("right_motor_gain", 1.0);
    this->declare_parameter("left_motor_reversed", false);
    this->declare_parameter("right_motor_reversed", false);
    this->declare_parameter("min_pwm", 60);
    this->declare_parameter("spin_pwm", 150);
    this->declare_parameter("spin_min_pwm", 40);
    this->declare_parameter("spin_ticks_per_side", 5);
    this->declare_parameter("servo1_home", 90);
    this->declare_parameter("servo1_carry", 135);
    this->declare_parameter("servo2_open", 70);
    this->declare_parameter("servo2_closed", 150);

    // Get parameters
    serial_port_ = this->get_parameter("serial_port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    wheel_base_ = this->get_parameter("wheel_base_m").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius_m").as_double();
    max_pwm_ = this->get_parameter("max_pwm").as_int();
    cmd_rate_hz_ = this->get_parameter("cmd_rate_hz").as_double();
    watchdog_ms_ = this->get_parameter("watchdog_timeout_ms").as_int();
    telemetry_rate_hz_ = this->get_parameter("telemetry_rate_hz").as_double();
    left_gain_ = this->get_parameter("left_motor_gain").as_double();
    right_gain_ = this->get_parameter("right_motor_gain").as_double();
    left_reversed_ = this->get_parameter("left_motor_reversed").as_bool();
    right_reversed_ = this->get_parameter("right_motor_reversed").as_bool();
    min_pwm_ = this->get_parameter("min_pwm").as_int();
    spin_pwm_ = this->get_parameter("spin_pwm").as_int();
    spin_min_pwm_ = this->get_parameter("spin_min_pwm").as_int();
    spin_ticks_per_side_ = this->get_parameter("spin_ticks_per_side").as_int();
    servo1_home_ = this->get_parameter("servo1_home").as_int();
    servo1_carry_ = this->get_parameter("servo1_carry").as_int();
    servo2_open_ = this->get_parameter("servo2_open").as_int();
    servo2_closed_ = this->get_parameter("servo2_closed").as_int();

    // Initialize state
    target_v_ = 0.0;
    target_omega_ = 0.0;
    last_cmd_time_ = this->now();
    claw_mode_ = 0;
    claw_position_ = 0.0;
    spin_toggle_ = false;
    spin_tick_count_ = 0;

    // Open serial port
    if (!open_serial()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Serial port opened: %s", serial_port_.c_str());
      // Reset claw to open/home on startup (matches test_pickup.py reset sequence)
      send_servo(2, servo2_open_);
      send_servo(1, servo1_home_);
    }

    // Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/control/cmd_vel_limited",
        10,
        std::bind(&SerialBridgeNode::cmd_vel_callback, this, std::placeholders::_1));

    claw_sub_ = this->create_subscription<robot_interfaces::msg::ClawCommand>(
        "/claw/cmd",
        10,
        std::bind(&SerialBridgeNode::claw_callback, this, std::placeholders::_1));

    // Publisher
    status_pub_ = this->create_publisher<robot_interfaces::msg::HwStatus>("/hw/status", 10);

    // Command timer
    double cmd_period_ms = 1000.0 / cmd_rate_hz_;
    cmd_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(cmd_period_ms)),
        std::bind(&SerialBridgeNode::send_commands, this));

    // Telemetry timer
    double tel_period_ms = 1000.0 / telemetry_rate_hz_;
    telemetry_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(tel_period_ms)),
        std::bind(&SerialBridgeNode::read_telemetry, this));

    RCLCPP_INFO(this->get_logger(), "Serial bridge node initialized");
  }

  ~SerialBridgeNode() {
    if (serial_fd_ >= 0) {
      // Stop motors before closing
      send_motor_command(0, 0);
      close(serial_fd_);
    }
  }

private:
  bool open_serial() {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_fd_, &tty) != 0) {
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    // Set baud rate
    speed_t baud;
    switch (baudrate_) {
      case 9600:
        baud = B9600;
        break;
      case 19200:
        baud = B19200;
        break;
      case 38400:
        baud = B38400;
        break;
      case 57600:
        baud = B57600;
        break;
      case 115200:
        baud = B115200;
        break;
      default:
        baud = B115200;
    }
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    // 8N1 mode
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    // Raw mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                     ICRNL);
    tty.c_oflag &= ~OPOST;

    // Timeouts
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    return true;
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    target_v_ = msg->linear.x;
    target_omega_ = msg->angular.z;
    last_cmd_time_ = this->now();
  }

  void claw_callback(const robot_interfaces::msg::ClawCommand::SharedPtr msg) {
    claw_mode_ = msg->mode;
    claw_position_ = msg->position;
    send_claw_command(claw_mode_, claw_position_);
  }

  void send_commands() {
    // Check watchdog
    double cmd_age_ms = (this->now() - last_cmd_time_).nanoseconds() / 1e6;
    if (cmd_age_ms > watchdog_ms_) {
      target_v_ = 0.0;
      target_omega_ = 0.0;
    }

    // Pure-spin mode: hardware cannot drive both wheels in opposite directions
    // simultaneously, so alternate M(-spin_pwm,0) / M(0,spin_pwm) each tick.
    // Scale PWM proportionally to requested omega so gentle corrections don't
    // fire the full spin_pwm (1.5 rad/s reference = full spin_pwm).
    if (std::abs(target_v_) < 0.01 && std::abs(target_omega_) > 0.1) {
      int dir = (target_omega_ > 0) ? 1 : -1;  // +1 = CCW, -1 = CW
      int effective_pwm = std::clamp(
        (int)(std::abs(target_omega_) / 1.5 * spin_pwm_),
        spin_min_pwm_, spin_pwm_);
      // Mode C (empirically verified): both wheels backward alternating
      int left_pwm  = left_reversed_  ? (dir * effective_pwm) : (-dir * effective_pwm);
      int right_pwm = right_reversed_ ? (dir * effective_pwm) : (-dir * effective_pwm);

      if (spin_toggle_) {
        send_motor_command(left_pwm, 0);
      } else {
        send_motor_command(0, right_pwm);
      }
      spin_tick_count_++;
      if (spin_tick_count_ >= spin_ticks_per_side_) {
        spin_toggle_ = !spin_toggle_;
        spin_tick_count_ = 0;
      }
      return;
    }
    spin_toggle_ = false;  // reset when not in pure-spin
    spin_tick_count_ = 0;

    // Convert twist to wheel velocities
    double v_left = target_v_ - target_omega_ * wheel_base_ / 2.0;
    double v_right = target_v_ + target_omega_ * wheel_base_ / 2.0;

    // Convert to wheel angular velocities
    double omega_left = v_left / wheel_radius_;
    double omega_right = v_right / wheel_radius_;

    // Normalize to [-1, 1] based on max expected wheel speed
    double max_wheel_omega = 10.0;  // ~95 RPM at max speed
    double norm_left = std::clamp(omega_left / max_wheel_omega, -1.0, 1.0);
    double norm_right = std::clamp(omega_right / max_wheel_omega, -1.0, 1.0);

    // Apply gains and reversal
    norm_left *= left_gain_;
    norm_right *= right_gain_;
    if (left_reversed_) norm_left = -norm_left;
    if (right_reversed_) norm_right = -norm_right;

    // Convert to PWM
    int pwm_left = static_cast<int>(norm_left * max_pwm_);
    int pwm_right = static_cast<int>(norm_right * max_pwm_);

    // During forward motion, never let either wheel reverse due to v/omega mismatch
    if (target_v_ > 0.0) {
      pwm_left  = std::max(0, pwm_left);
      pwm_right = std::max(0, pwm_right);
    }

    // Enforce minimum PWM so neither wheel stalls in the motor deadband
    if (pwm_left > 0 && pwm_left < min_pwm_) pwm_left = min_pwm_;
    if (pwm_left < 0 && pwm_left > -min_pwm_) pwm_left = -min_pwm_;
    if (pwm_right > 0 && pwm_right < min_pwm_) pwm_right = min_pwm_;
    if (pwm_right < 0 && pwm_right > -min_pwm_) pwm_right = -min_pwm_;

    send_motor_command(pwm_left, pwm_right);
  }

  void send_motor_command(int left_pwm, int right_pwm) {
    if (serial_fd_ < 0) {
      return;
    }

    std::stringstream ss;
    ss << "M," << left_pwm << "," << right_pwm << "\n";
    std::string cmd = ss.str();

    ssize_t written = write(serial_fd_, cmd.c_str(), cmd.length());
    if (written < 0) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000, "Serial write failed");
    }
  }

  void send_servo(int servo_num, int angle) {
    if (serial_fd_ < 0) {
      return;
    }
    std::stringstream ss;
    ss << "C," << servo_num << "," << angle << "\n";
    std::string cmd = ss.str();
    ssize_t written = write(serial_fd_, cmd.c_str(), cmd.length());
    if (written < 0) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000, "Serial write failed");
    }
  }

  void send_claw_command(uint8_t mode, float /*position*/) {
    // MODE_OPEN  (0): open gripper + home rotation
    // MODE_CLOSE (1): close gripper (rotation stays at home)
    // MODE_HOLD  (2): no-op — hold current position
    // MODE_ROTATE(3): rotate arm to carry position (gripper stays closed)
    switch (mode) {
      case robot_interfaces::msg::ClawCommand::MODE_OPEN:
        send_servo(2, servo2_open_);
        send_servo(1, servo1_home_);
        break;
      case robot_interfaces::msg::ClawCommand::MODE_CLOSE:
        send_servo(2, servo2_closed_);
        break;
      case robot_interfaces::msg::ClawCommand::MODE_ROTATE:
        send_servo(1, servo1_carry_);
        break;
      case robot_interfaces::msg::ClawCommand::MODE_HOLD:
      default:
        break;
    }
  }

  void read_telemetry() {
    if (serial_fd_ < 0) {
      // Publish dummy status when no serial
      robot_interfaces::msg::HwStatus status;
      status.stamp = this->now().operator builtin_interfaces::msg::Time();
      status.battery_v = 12.0;
      status.left_enc = 0;
      status.right_enc = 0;
      status.estop = false;
      status.last_error = "";
      status_pub_->publish(status);
      return;
    }

    // Read available data
    char buf[256];
    ssize_t n = read(serial_fd_, buf, sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = '\0';
      read_buffer_ += buf;

      // Process complete lines
      size_t pos;
      while ((pos = read_buffer_.find('\n')) != std::string::npos) {
        std::string line = read_buffer_.substr(0, pos);
        read_buffer_.erase(0, pos + 1);
        parse_telemetry(line);
      }
    }
  }

  void parse_telemetry(const std::string& line) {
    if (line.empty()) {
      return;
    }

    // Split by comma
    std::vector<std::string> parts;
    std::stringstream ss(line);
    std::string part;
    while (std::getline(ss, part, ',')) {
      parts.push_back(part);
    }

    if (parts.empty()) {
      return;
    }

    robot_interfaces::msg::HwStatus status;
    status.stamp = this->now().operator builtin_interfaces::msg::Time();

    if (parts[0] == "T" && parts.size() >= 5) {
      // Telemetry: T,<battery_mv>,<left_enc>,<right_enc>,<estop>
      try {
        status.battery_v = std::stof(parts[1]) / 1000.0f;
        status.left_enc = std::stoi(parts[2]);
        status.right_enc = std::stoi(parts[3]);
        status.estop = (parts[4] == "1");
        status.last_error = "";
        status_pub_->publish(status);
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse telemetry: %s", line.c_str());
      }
    } else if (parts[0] == "E" && parts.size() >= 3) {
      // Error: E,<code>,<msg> — log only, do NOT set estop; estop comes from T telemetry only
      RCLCPP_WARN(this->get_logger(), "Arduino error: %s", parts[2].c_str());
    }
  }

  // Parameters
  std::string serial_port_;
  int baudrate_;
  double wheel_base_;
  double wheel_radius_;
  int max_pwm_;
  double cmd_rate_hz_;
  int watchdog_ms_;
  double telemetry_rate_hz_;
  double left_gain_, right_gain_;
  bool left_reversed_, right_reversed_;
  int min_pwm_;
  int spin_pwm_;
  int spin_min_pwm_;
  int spin_ticks_per_side_;
  int servo1_home_, servo1_carry_;
  int servo2_open_, servo2_closed_;

  // Serial state
  int serial_fd_;
  std::string read_buffer_;

  // Command state
  double target_v_;
  double target_omega_;
  rclcpp::Time last_cmd_time_;
  uint8_t claw_mode_;
  float claw_position_;
  bool spin_toggle_;
  int spin_tick_count_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<robot_interfaces::msg::ClawCommand>::SharedPtr claw_sub_;
  rclcpp::Publisher<robot_interfaces::msg::HwStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
