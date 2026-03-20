/**
 * @file task_fsm_node.cpp
 * @brief Task-level finite state machine for pick-and-place mission.
 *
 * States:
 * - INIT: Waiting for sensors to be ready
 * - FOLLOW_LINE_SEARCH: Following line, searching for target
 * - APPROACH_TARGET: Vision-based approach to blue circle (line following suppressed)
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
    this->declare_parameter("pickup_close_time_s", 1.0);
    this->declare_parameter("pickup_hold_time_s", 0.5);
    this->declare_parameter("drop_open_time_s", 1.0);
    this->declare_parameter("use_time_based_return", true);
    this->declare_parameter("return_time_s", 10.0);
    this->declare_parameter("require_line_follow", true);
    this->declare_parameter("line_loss_timeout_s", 3.0);
    this->declare_parameter("rate_hz", 20.0);

    // Approach parameters
    // Linear speed while driving toward the circle
    this->declare_parameter("approach_linear_speed_mps", 0.15);
    // P gain for angular correction (centering circle horizontally)
    this->declare_parameter("approach_kp_angular", 1.5);
    // Accepted vertical error: |top_y - 0.5| < this  (top_y = cy - h/2)
    this->declare_parameter("approach_top_tolerance_y", 0.15);
    // Accepted horizontal error: |cx - 0.5| < this
    this->declare_parameter("approach_center_tolerance_x", 0.20);
    // Stop publishing cmd_vel if circle unseen for this long (seconds)
    this->declare_parameter("approach_align_gate_x", 0.15);
    this->declare_parameter("approach_detection_timeout_s", 0.5);

    // Get parameters
    target_class_ = this->get_parameter("target_class").as_string();
    conf_threshold_ = this->get_parameter("detection_confidence_threshold").as_double();
    pickup_close_time_ = this->get_parameter("pickup_close_time_s").as_double();
    pickup_hold_time_ = this->get_parameter("pickup_hold_time_s").as_double();
    drop_open_time_ = this->get_parameter("drop_open_time_s").as_double();
    use_time_return_ = this->get_parameter("use_time_based_return").as_bool();
    return_time_ = this->get_parameter("return_time_s").as_double();
    require_line_follow_ = this->get_parameter("require_line_follow").as_bool();
    line_loss_timeout_ = this->get_parameter("line_loss_timeout_s").as_double();
    rate_hz_ = this->get_parameter("rate_hz").as_double();

    approach_speed_ = this->get_parameter("approach_linear_speed_mps").as_double();
    approach_kp_angular_ = this->get_parameter("approach_kp_angular").as_double();
    approach_top_tol_ = this->get_parameter("approach_top_tolerance_y").as_double();
    approach_center_tol_x_ = this->get_parameter("approach_center_tolerance_x").as_double();
    approach_align_gate_x_ = this->get_parameter("approach_align_gate_x").as_double();
    approach_det_timeout_ = this->get_parameter("approach_detection_timeout_s").as_double();

    // Initialize state
    current_state_ = State::INIT;
    state_start_time_ = this->now();
    line_valid_ = false;
    last_line_valid_time_ = this->now();
    hw_ready_ = false;
    last_detection_time_ = this->now();

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

    // target_locked: published by object detector when a full blue circle is in frame.
    // Triggers immediate transition from FOLLOW_LINE_SEARCH -> APPROACH_TARGET.
    target_locked_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/vision/target_locked",
        10,
        std::bind(&TaskFsmNode::target_locked_callback, this, std::placeholders::_1));

    // Publishers
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/control/fsm_state", 10);
    claw_pub_ = this->create_publisher<robot_interfaces::msg::ClawCommand>("/claw/cmd", 10);
    enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/control/enable", 10);
    // Used during APPROACH_TARGET to drive toward the circle (bypasses line follower)
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/control/cmd_vel", 10);

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
    // Track when we last had a valid target detection (used in approach)
    if (find_target().has_value()) {
      last_detection_time_ = this->now();
    }
  }

  void hw_callback(const robot_interfaces::msg::HwStatus::SharedPtr /*msg*/) {
    hw_ready_ = true;
  }

  void estop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && current_state_ != State::FAILSAFE_STOP) {
      transition_to(State::FAILSAFE_STOP);
    }
  }

  void target_locked_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && current_state_ == State::FOLLOW_LINE_SEARCH) {
      RCLCPP_INFO(this->get_logger(), "Full blue circle confirmed — entering approach");
      transition_to(State::APPROACH_TARGET);
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
  }

  void fsm_loop() {
    // Publish current state
    std_msgs::msg::String state_msg;
    state_msg.data = state_to_string(current_state_);
    state_pub_->publish(state_msg);

    // Line loss check — skipped during approach (navigating by vision),
    // skipped in motor-off states, and skipped entirely when not using line following.
    bool check_line = require_line_follow_ &&
                      current_state_ != State::INIT &&
                      current_state_ != State::APPROACH_TARGET &&
                      current_state_ != State::PICKUP &&
                      current_state_ != State::DROP &&
                      current_state_ != State::DONE &&
                      current_state_ != State::FAILSAFE_STOP;
    if (check_line) {
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
    publish_enable(false);

    double wait_time = (this->now() - state_start_time_).seconds();
    bool line_ok = !require_line_follow_ || line_valid_;
    if (line_ok && hw_ready_) {
      RCLCPP_INFO(this->get_logger(), "Systems ready, starting mission");
      transition_to(State::FOLLOW_LINE_SEARCH);
    } else if (wait_time > 10.0) {
      RCLCPP_WARN(this->get_logger(), "Timeout waiting for systems, proceeding anyway");
      transition_to(State::FOLLOW_LINE_SEARCH);
    }
  }

  void handle_follow_line_search() {
    // Line follower is in control — just keep it enabled.
    // Transition to APPROACH_TARGET happens via target_locked_callback.
    publish_enable(true);
  }

  void handle_approach_target() {
    // Suppress the line follow controller so it doesn't fight our cmd_vel.
    publish_enable(false);

    auto target = find_target();

    if (!target.has_value()) {
      // No circle visible: stop and wait. If it stays lost too long, coast.
      double lost_for = (this->now() - last_detection_time_).seconds();
      if (lost_for > approach_det_timeout_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "Circle lost during approach (%.1fs)", lost_for);
        publish_zero_cmd();
      }
      return;
    }

    // top_y: normalized y of the top edge of the circle (0 = image top, 1 = bottom).
    // Goal: drive forward until top_y ≈ 0.5 (top of circle at vertical midframe).
    double top_y = target->cy - target->h / 2.0;
    double error_x = target->cx - 0.5;  // positive = circle right of centre

    bool vertically_close = std::abs(top_y - 0.5) < approach_top_tol_;
    bool horizontally_centred = std::abs(error_x) < approach_center_tol_x_;

    if (vertically_close && horizontally_centred) {
      RCLCPP_INFO(this->get_logger(),
                  "Approach complete (top_y=%.2f cx=%.2f) — picking up", top_y, target->cx);
      publish_zero_cmd();
      transition_to(State::PICKUP);
      return;
    }

    // Only drive forward once the circle is roughly centred horizontally.
    // If too far off-axis, turn in place first — prevents the circle from
    // drifting off the frame edge as the robot approaches.
    bool aligned = std::abs(error_x) < approach_align_gate_x_;
    double linear = (aligned && top_y < 0.5) ? approach_speed_ : 0.0;
    double angular = -approach_kp_angular_ * error_x;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                         "APPROACH — top_y=%.3f (goal=0.5±%.2f)  cx=%.3f (goal=0.5±%.2f)  "
                         "lin=%.3f  ang=%.3f",
                         top_y, approach_top_tol_,
                         target->cx, approach_center_tol_x_,
                         linear, angular);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_vel_pub_->publish(cmd);
  }

  void handle_pickup() {
    publish_enable(false);

    double state_time = (this->now() - state_start_time_).seconds();

    robot_interfaces::msg::ClawCommand claw_cmd;
    claw_cmd.mode = robot_interfaces::msg::ClawCommand::MODE_CLOSE;
    claw_cmd.position = 1.0;
    claw_pub_->publish(claw_cmd);

    if (state_time > pickup_close_time_ + pickup_hold_time_) {
      RCLCPP_INFO(this->get_logger(), "Pickup complete, returning");
      transition_to(State::RETURN_FOLLOW_LINE);
    }
  }

  void handle_return_follow_line() {
    publish_enable(true);

    robot_interfaces::msg::ClawCommand claw_cmd;
    claw_cmd.mode = robot_interfaces::msg::ClawCommand::MODE_HOLD;
    claw_cmd.position = 1.0;
    claw_pub_->publish(claw_cmd);

    double state_time = (this->now() - state_start_time_).seconds();
    if (use_time_return_ && state_time > return_time_) {
      RCLCPP_INFO(this->get_logger(), "Return time elapsed, dropping");
      transition_to(State::DROP);
    }
  }

  void handle_drop() {
    publish_enable(false);

    robot_interfaces::msg::ClawCommand claw_cmd;
    claw_cmd.mode = robot_interfaces::msg::ClawCommand::MODE_OPEN;
    claw_cmd.position = 0.0;
    claw_pub_->publish(claw_cmd);

    double state_time = (this->now() - state_start_time_).seconds();
    if (state_time > drop_open_time_) {
      RCLCPP_INFO(this->get_logger(), "Mission complete!");
      transition_to(State::DONE);
    }
  }

  void handle_done() {
    publish_enable(false);
  }

  void handle_failsafe() {
    publish_enable(false);

    robot_interfaces::msg::ClawCommand claw_cmd;
    claw_cmd.mode = robot_interfaces::msg::ClawCommand::MODE_OPEN;
    claw_cmd.position = 0.0;
    claw_pub_->publish(claw_cmd);
  }

  // ---- helpers ----

  void publish_enable(bool enabled) {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    enable_pub_->publish(msg);
  }

  void publish_zero_cmd() {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
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
  double pickup_close_time_, pickup_hold_time_;
  double drop_open_time_;
  bool use_time_return_;
  double return_time_;
  double line_loss_timeout_;
  double rate_hz_;
  bool require_line_follow_;
  double approach_speed_;
  double approach_kp_angular_;
  double approach_top_tol_;
  double approach_center_tol_x_;
  double approach_align_gate_x_;
  double approach_det_timeout_;

  // State
  State current_state_;
  rclcpp::Time state_start_time_;
  robot_interfaces::msg::Detections2D latest_detections_;
  bool line_valid_;
  rclcpp::Time last_line_valid_time_;
  rclcpp::Time last_detection_time_;
  bool hw_ready_;

  // ROS interfaces
  rclcpp::Subscription<robot_interfaces::msg::LineObservation>::SharedPtr line_sub_;
  rclcpp::Subscription<robot_interfaces::msg::Detections2D>::SharedPtr det_sub_;
  rclcpp::Subscription<robot_interfaces::msg::HwStatus>::SharedPtr hw_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_locked_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<robot_interfaces::msg::ClawCommand>::SharedPtr claw_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskFsmNode>());
  rclcpp::shutdown();
  return 0;
}
