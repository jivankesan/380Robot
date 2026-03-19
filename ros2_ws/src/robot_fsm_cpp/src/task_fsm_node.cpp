/**
 * @file task_fsm_node.cpp
 * @brief Task-level finite state machine for pick-and-place mission.
 *
 * States:
 * - INIT: Waiting for sensors to be ready
 * - FOLLOW_LINE_SEARCH: Following line, searching for target
 * - APPROACH_TARGET: Moving toward detected target
 * - PICKUP: Stopping and closing claw
 * - RETURN_FOLLOW_LINE: Following line back to drop zone
 * - DROP: Opening claw to release object
 * - DONE: Mission complete
 * - FAILSAFE_STOP: Error state
 */

#include <chrono>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/claw_command.hpp"
#include "robot_interfaces/msg/detections2_d.hpp"
#include "robot_interfaces/msg/hw_status.hpp"
#include "robot_interfaces/msg/line_observation.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

enum class State {
  INIT,
  FOLLOW_LINE_SEARCH,
  APPROACH_TARGET,
  PICKUP,
  RETURN_FOLLOW_LINE,
  DROP,
  DONE,
  FAILSAFE_STOP
};

std::string state_to_string(State state) {
  switch (state) {
    case State::INIT:
      return "INIT";
    case State::FOLLOW_LINE_SEARCH:
      return "FOLLOW_LINE_SEARCH";
    case State::APPROACH_TARGET:
      return "APPROACH_TARGET";
    case State::PICKUP:
      return "PICKUP";
    case State::RETURN_FOLLOW_LINE:
      return "RETURN_FOLLOW_LINE";
    case State::DROP:
      return "DROP";
    case State::DONE:
      return "DONE";
    case State::FAILSAFE_STOP:
      return "FAILSAFE_STOP";
    default:
      return "UNKNOWN";
  }
}

class TaskFsmNode : public rclcpp::Node {
public:
  TaskFsmNode() : Node("task_fsm_node") {
    // Declare parameters
    this->declare_parameter("target_class", "blue_circle");
    this->declare_parameter("detection_confidence_threshold", 0.5);
    this->declare_parameter("detection_stability_frames", 5);
    this->declare_parameter("target_center_tolerance_x", 0.15);
    this->declare_parameter("target_center_tolerance_y", 0.2);
    this->declare_parameter("target_size_close_threshold", 0.25);
    this->declare_parameter("pickup_stop_dwell_s", 0.4);
    this->declare_parameter("pickup_close_time_s", 1.0);
    this->declare_parameter("pickup_hold_time_s", 0.5);
    this->declare_parameter("drop_open_time_s", 1.0);
    this->declare_parameter("use_time_based_return", true);
    this->declare_parameter("return_time_s", 10.0);
    this->declare_parameter("min_approach_dwell_s", 2.0);
    this->declare_parameter("line_loss_timeout_s", 3.0);
    this->declare_parameter("min_init_dwell_s", 1.5);
    this->declare_parameter("rate_hz", 20.0);

    // Get parameters
    target_class_ = this->get_parameter("target_class").as_string();
    conf_threshold_ = this->get_parameter("detection_confidence_threshold").as_double();
    stability_frames_ = this->get_parameter("detection_stability_frames").as_int();
    center_tol_x_ = this->get_parameter("target_center_tolerance_x").as_double();
    center_tol_y_ = this->get_parameter("target_center_tolerance_y").as_double();
    size_close_threshold_ = this->get_parameter("target_size_close_threshold").as_double();
    pickup_stop_dwell_ = this->get_parameter("pickup_stop_dwell_s").as_double();
    pickup_close_time_ = this->get_parameter("pickup_close_time_s").as_double();
    pickup_hold_time_ = this->get_parameter("pickup_hold_time_s").as_double();
    drop_open_time_ = this->get_parameter("drop_open_time_s").as_double();
    use_time_return_ = this->get_parameter("use_time_based_return").as_bool();
    return_time_ = this->get_parameter("return_time_s").as_double();
    min_approach_dwell_ = this->get_parameter("min_approach_dwell_s").as_double();
    line_loss_timeout_ = this->get_parameter("line_loss_timeout_s").as_double();
    min_init_dwell_ = this->get_parameter("min_init_dwell_s").as_double();
    rate_hz_ = this->get_parameter("rate_hz").as_double();

    // Initialize state
    current_state_ = State::INIT;
    detection_count_ = 0;
    state_start_time_ = this->now();
    line_valid_ = false;
    last_line_valid_time_ = this->now();
    hw_ready_ = false;

    // Subscribers
    line_sub_ = this->create_subscription<robot_interfaces::msg::LineObservation>(
        "/vision/line_observation",
        10,
        std::bind(&TaskFsmNode::line_callback, this, std::placeholders::_1));

    det_sub_ = this->create_subscription<robot_interfaces::msg::Detections2D>(
        "/vision/detections",
        10,
        std::bind(&TaskFsmNode::detection_callback, this, std::placeholders::_1));

    hw_sub_ = this->create_subscription<robot_interfaces::msg::HwStatus>(
        "/hw/status",
        10,
        std::bind(&TaskFsmNode::hw_callback, this, std::placeholders::_1));

    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/control/estop",
        10,
        std::bind(&TaskFsmNode::estop_callback, this, std::placeholders::_1));

    // Publishers
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/control/fsm_state", 10);
    claw_pub_ = this->create_publisher<robot_interfaces::msg::ClawCommand>("/claw/cmd", 10);
    enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/control/enable", 10);

    // Timer
    double period_ms = 1000.0 / rate_hz_;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(period_ms)),
        std::bind(&TaskFsmNode::fsm_loop, this));

    RCLCPP_INFO(this->get_logger(), "Task FSM node initialized");
  }

private:
  void line_callback(const robot_interfaces::msg::LineObservation::SharedPtr msg) {
    line_valid_ = msg->valid;
    if (line_valid_) {
      last_line_valid_time_ = this->now();
    }
  }

  void detection_callback(const robot_interfaces::msg::Detections2D::SharedPtr msg) {
    latest_detections_ = *msg;
  }

  void hw_callback(const robot_interfaces::msg::HwStatus::SharedPtr /*msg*/) {
    hw_ready_ = true;
  }

  void estop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && current_state_ != State::FAILSAFE_STOP) {
      transition_to(State::FAILSAFE_STOP);
    }
  }

  void transition_to(State new_state) {
    if (new_state == current_state_) {
      return;
    }
    RCLCPP_INFO(
        this->get_logger(),
        "State transition: %s -> %s",
        state_to_string(current_state_).c_str(),
        state_to_string(new_state).c_str());
    current_state_ = new_state;
    state_start_time_ = this->now();
    detection_count_ = 0;
  }

  void fsm_loop() {
    // Publish current state
    std_msgs::msg::String state_msg;
    state_msg.data = state_to_string(current_state_);
    state_pub_->publish(state_msg);

    // Check for line loss (except in states where it's expected).
    // APPROACH_TARGET is excluded: the line detector intentionally marks the
    // line invalid once the blue circle is detected, so loss here is normal.
    if (current_state_ != State::INIT && current_state_ != State::PICKUP &&
        current_state_ != State::DROP && current_state_ != State::DONE &&
        current_state_ != State::FAILSAFE_STOP &&
        current_state_ != State::APPROACH_TARGET) {
      double line_loss_time = (this->now() - last_line_valid_time_).seconds();
      if (line_loss_time > line_loss_timeout_) {
        RCLCPP_WARN(this->get_logger(), "Line lost for too long, entering failsafe");
        transition_to(State::FAILSAFE_STOP);
        return;
      }
    }

    // State machine
    switch (current_state_) {
      case State::INIT:
        handle_init();
        break;
      case State::FOLLOW_LINE_SEARCH:
        handle_follow_line_search();
        break;
      case State::APPROACH_TARGET:
        handle_approach_target();
        break;
      case State::PICKUP:
        handle_pickup();
        break;
      case State::RETURN_FOLLOW_LINE:
        handle_return_follow_line();
        break;
      case State::DROP:
        handle_drop();
        break;
      case State::DONE:
        handle_done();
        break;
      case State::FAILSAFE_STOP:
        handle_failsafe();
        break;
    }
  }

  void handle_init() {
    // Disable controller output during init
    std_msgs::msg::Bool enable_msg;
    enable_msg.data = false;
    enable_pub_->publish(enable_msg);

    // Keep sending claw open every tick so the Arduino receives it even if
    // it was still booting when the first command arrived.
    robot_interfaces::msg::ClawCommand claw_cmd;
    claw_cmd.mode = 1;  // servo 1 = gripper
    claw_cmd.position = 0.0;
    claw_pub_->publish(claw_cmd);

    double wait_time = (this->now() - state_start_time_).seconds();

    // Enforce a minimum dwell so the Arduino has time to boot after the
    // serial port opens (DTR reset takes ~1 s) and so the claw open command
    // is sent repeatedly before the mission starts.
    if (wait_time < min_init_dwell_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "INIT: opening claw, waiting %.1f / %.1f s",
          wait_time, min_init_dwell_);
      return;
    }

    if (line_valid_ && hw_ready_) {
      RCLCPP_INFO(this->get_logger(), "Systems ready, starting mission");
      transition_to(State::FOLLOW_LINE_SEARCH);
    } else if (wait_time > 10.0) {
      RCLCPP_WARN(this->get_logger(), "Timeout waiting for systems");
      transition_to(State::FOLLOW_LINE_SEARCH);
    }
  }

  void handle_follow_line_search() {
    // Enable controller
    std_msgs::msg::Bool enable_msg;
    enable_msg.data = true;
    enable_pub_->publish(enable_msg);

    // Check for target detection
    auto target = find_target();
    if (target.has_value()) {
      detection_count_++;
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 500,
          "Blue circle seen (cx=%.2f score=%.2f) — stable count %d/%d",
          target->cx, target->score, detection_count_, stability_frames_);
      if (detection_count_ >= stability_frames_) {
        RCLCPP_INFO(this->get_logger(),
            "Blue circle locked — switching to approach (line follow disabled)");
        transition_to(State::APPROACH_TARGET);
      }
    } else {
      if (detection_count_ > 0) {
        RCLCPP_WARN(this->get_logger(), "Blue circle lost during search, resetting count");
      }
      detection_count_ = 0;
    }
  }

  void handle_approach_target() {
    // Disable the line follow controller — the blue circle controller takes
    // over publishing to /control/cmd_vel in this state.
    std_msgs::msg::Bool enable_msg;
    enable_msg.data = false;
    enable_pub_->publish(enable_msg);

    auto target = find_target();
    double dwell_time = (this->now() - state_start_time_).seconds();
    if (!target.has_value()) {
      detection_count_++;
      if (dwell_time > min_approach_dwell_ && detection_count_ > stability_frames_ * 2) {
        RCLCPP_WARN(this->get_logger(), "Lost target, returning to search");
        transition_to(State::FOLLOW_LINE_SEARCH);
      }
      return;
    }
    detection_count_ = 0;

    // Check if target is close enough for pickup
    double size = std::max(target->w, target->h);
    bool centered = std::abs(target->cx - 0.5) < center_tol_x_ &&
                    std::abs(target->cy - 0.5) < center_tol_y_;

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 300,
        "Approaching blue circle — cx=%.2f size=%.2f  need: size>=%.2f centered=%s",
        target->cx, size, size_close_threshold_, centered ? "YES" : "no");

    if (size >= size_close_threshold_ && centered) {
      RCLCPP_INFO(this->get_logger(),
          "Blue circle centered and close (size=%.2f cx=%.2f) — triggering pickup",
          size, target->cx);
      transition_to(State::PICKUP);
    }
  }

  void handle_pickup() {
    // Disable controller (stop motors)
    std_msgs::msg::Bool enable_msg;
    enable_msg.data = false;
    enable_pub_->publish(enable_msg);

    double state_time = (this->now() - state_start_time_).seconds();

    // Wait for motors to coast to zero before closing the claw.
    // The serial bridge watchdog stops motors within 250ms of last command;
    // pickup_stop_dwell_s gives a comfortable margin on top of that.
    if (state_time < pickup_stop_dwell_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
          "Waiting for robot to stop before claw close (%.2f / %.2f s)",
          state_time, pickup_stop_dwell_);
      return;
    }

    // Close claw
    robot_interfaces::msg::ClawCommand claw_cmd;
    claw_cmd.mode = 1;  // servo 1 = gripper
    claw_cmd.position = 1.0;
    claw_pub_->publish(claw_cmd);

    // Wait for claw to close and hold
    if (state_time > pickup_stop_dwell_ + pickup_close_time_ + pickup_hold_time_) {
      RCLCPP_INFO(this->get_logger(), "Pickup complete, returning to line");
      // Reset line-valid timer so RETURN_FOLLOW_LINE gets a fresh timeout
      // window to relocate the line (it was lost during approach + pickup).
      last_line_valid_time_ = this->now();
      transition_to(State::RETURN_FOLLOW_LINE);
    }
  }

  void handle_return_follow_line() {
    // Enable controller
    std_msgs::msg::Bool enable_msg;
    enable_msg.data = true;
    enable_pub_->publish(enable_msg);

    // Keep claw closed
    robot_interfaces::msg::ClawCommand claw_cmd;
    claw_cmd.mode = 1;  // servo 1 = gripper
    claw_cmd.position = 1.0;
    claw_pub_->publish(claw_cmd);

    // Check for drop zone (time-based for now)
    double state_time = (this->now() - state_start_time_).seconds();
    if (use_time_return_ && state_time > return_time_) {
      RCLCPP_INFO(this->get_logger(), "Return time elapsed, dropping");
      transition_to(State::DROP);
    }

    // TODO: Add marker-based drop zone detection
  }

  void handle_drop() {
    // Disable controller (stop)
    std_msgs::msg::Bool enable_msg;
    enable_msg.data = false;
    enable_pub_->publish(enable_msg);

    // Open claw
    robot_interfaces::msg::ClawCommand claw_cmd;
    claw_cmd.mode = 1;  // servo 1 = gripper
    claw_cmd.position = 0.0;
    claw_pub_->publish(claw_cmd);

    double state_time = (this->now() - state_start_time_).seconds();
    if (state_time > drop_open_time_) {
      RCLCPP_INFO(this->get_logger(), "Mission complete!");
      transition_to(State::DONE);
    }
  }

  void handle_done() {
    // Stay stopped
    std_msgs::msg::Bool enable_msg;
    enable_msg.data = false;
    enable_pub_->publish(enable_msg);
  }

  void handle_failsafe() {
    // Disable controller
    std_msgs::msg::Bool enable_msg;
    enable_msg.data = false;
    enable_pub_->publish(enable_msg);

    // Open claw for safety
    robot_interfaces::msg::ClawCommand claw_cmd;
    claw_cmd.mode = 1;  // servo 1 = gripper
    claw_cmd.position = 0.0;
    claw_pub_->publish(claw_cmd);
  }

  std::optional<robot_interfaces::msg::Detection2D> find_target() {
    for (const auto& det : latest_detections_.detections) {
      if (det.class_name == target_class_ && det.score >= conf_threshold_) {
        return det;
      }
    }
    return std::nullopt;
  }

  // Parameters
  std::string target_class_;
  double conf_threshold_;
  int stability_frames_;
  double center_tol_x_, center_tol_y_;
  double size_close_threshold_;
  double pickup_stop_dwell_;
  double pickup_close_time_, pickup_hold_time_;
  double drop_open_time_;
  bool use_time_return_;
  double return_time_;
  double min_approach_dwell_;
  double line_loss_timeout_;
  double min_init_dwell_;
  double rate_hz_;

  // State
  State current_state_;
  rclcpp::Time state_start_time_;
  int detection_count_;
  robot_interfaces::msg::Detections2D latest_detections_;
  bool line_valid_;
  rclcpp::Time last_line_valid_time_;
  bool hw_ready_;

  // ROS interfaces
  rclcpp::Subscription<robot_interfaces::msg::LineObservation>::SharedPtr line_sub_;
  rclcpp::Subscription<robot_interfaces::msg::Detections2D>::SharedPtr det_sub_;
  rclcpp::Subscription<robot_interfaces::msg::HwStatus>::SharedPtr hw_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<robot_interfaces::msg::ClawCommand>::SharedPtr claw_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskFsmNode>());
  rclcpp::shutdown();
  return 0;
}
