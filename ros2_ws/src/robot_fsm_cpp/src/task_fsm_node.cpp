/**
 * @file task_fsm_node.cpp
 * @brief Task-level finite state machine for pick-and-place mission.
 *
 * States:
 * - INIT:              Wait for sensors, hold claw open
 * - FOLLOW_LINE_SEARCH: Follow line, watch for blue circle
 * - APPROACH_TARGET:   Blue circle controller drives toward circle
 * - PICKUP:            Stop → close gripper (servo2) → rotate carry (servo1)
 * - TURN_AROUND:       Spin ~180° in place so robot faces back down the line
 * - RETURN_FOLLOW_LINE: Follow line back to drop zone
 * - DROP:              Open claw to release object
 * - DONE:              Mission complete, stopped
 * - FAILSAFE_STOP:     Error state, open claw
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
  TURN_AROUND,
  RETURN_FOLLOW_LINE,
  DROP,
  DONE,
  FAILSAFE_STOP
};

std::string state_to_string(State state) {
  switch (state) {
    case State::INIT:               return "INIT";
    case State::FOLLOW_LINE_SEARCH: return "FOLLOW_LINE_SEARCH";
    case State::APPROACH_TARGET:    return "APPROACH_TARGET";
    case State::PICKUP:             return "PICKUP";
    case State::TURN_AROUND:        return "TURN_AROUND";
    case State::RETURN_FOLLOW_LINE: return "RETURN_FOLLOW_LINE";
    case State::DROP:               return "DROP";
    case State::DONE:               return "DONE";
    case State::FAILSAFE_STOP:      return "FAILSAFE_STOP";
    default:                        return "UNKNOWN";
  }
}

class TaskFsmNode : public rclcpp::Node {
public:
  TaskFsmNode() : Node("task_fsm_node") {
    // Parameters
    this->declare_parameter("target_class", "blue_circle");
    this->declare_parameter("detection_confidence_threshold", 0.5);
    this->declare_parameter("detection_stability_frames", 5);
    this->declare_parameter("target_center_tolerance_x", 0.15);
    this->declare_parameter("target_center_tolerance_y", 0.2);
    this->declare_parameter("target_size_close_threshold", 0.25);
    this->declare_parameter("pickup_stop_dwell_s", 0.4);
    this->declare_parameter("pickup_close_time_s", 1.0);
    this->declare_parameter("pickup_rotate_time_s", 0.5);
    this->declare_parameter("turn_around_omega_rps", 1.5);
    this->declare_parameter("turn_around_time_s", 2.5);
    this->declare_parameter("drop_open_time_s", 1.0);
    this->declare_parameter("use_time_based_return", true);
    this->declare_parameter("return_time_s", 10.0);
    this->declare_parameter("min_approach_dwell_s", 2.0);
    this->declare_parameter("line_loss_timeout_s", 3.0);
    this->declare_parameter("min_init_dwell_s", 1.5);
    this->declare_parameter("rate_hz", 20.0);

    target_class_        = this->get_parameter("target_class").as_string();
    conf_threshold_      = this->get_parameter("detection_confidence_threshold").as_double();
    stability_frames_    = this->get_parameter("detection_stability_frames").as_int();
    center_tol_x_        = this->get_parameter("target_center_tolerance_x").as_double();
    center_tol_y_        = this->get_parameter("target_center_tolerance_y").as_double();
    size_close_threshold_= this->get_parameter("target_size_close_threshold").as_double();
    pickup_stop_dwell_   = this->get_parameter("pickup_stop_dwell_s").as_double();
    pickup_close_time_   = this->get_parameter("pickup_close_time_s").as_double();
    pickup_rotate_time_  = this->get_parameter("pickup_rotate_time_s").as_double();
    turn_around_omega_   = this->get_parameter("turn_around_omega_rps").as_double();
    turn_around_time_    = this->get_parameter("turn_around_time_s").as_double();
    drop_open_time_      = this->get_parameter("drop_open_time_s").as_double();
    use_time_return_     = this->get_parameter("use_time_based_return").as_bool();
    return_time_         = this->get_parameter("return_time_s").as_double();
    min_approach_dwell_  = this->get_parameter("min_approach_dwell_s").as_double();
    line_loss_timeout_   = this->get_parameter("line_loss_timeout_s").as_double();
    min_init_dwell_      = this->get_parameter("min_init_dwell_s").as_double();
    rate_hz_             = this->get_parameter("rate_hz").as_double();

    // State
    current_state_       = State::INIT;
    detection_count_     = 0;
    state_start_time_    = this->now();
    line_valid_          = false;
    last_line_valid_time_= this->now();
    hw_ready_            = false;

    // Subscribers
    line_sub_ = this->create_subscription<robot_interfaces::msg::LineObservation>(
        "/vision/line_observation", 10,
        std::bind(&TaskFsmNode::line_callback, this, std::placeholders::_1));

    det_sub_ = this->create_subscription<robot_interfaces::msg::Detections2D>(
        "/vision/detections", 10,
        std::bind(&TaskFsmNode::detection_callback, this, std::placeholders::_1));

    hw_sub_ = this->create_subscription<robot_interfaces::msg::HwStatus>(
        "/hw/status", 10,
        std::bind(&TaskFsmNode::hw_callback, this, std::placeholders::_1));

    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/control/estop", 10,
        std::bind(&TaskFsmNode::estop_callback, this, std::placeholders::_1));

    // Publishers
    state_pub_  = this->create_publisher<std_msgs::msg::String>("/control/fsm_state", 10);
    claw_pub_   = this->create_publisher<robot_interfaces::msg::ClawCommand>("/claw/cmd", 10);
    enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/control/enable", 10);
    cmd_pub_    = this->create_publisher<geometry_msgs::msg::Twist>("/control/cmd_vel", 10);

    double period_ms = 1000.0 / rate_hz_;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(period_ms)),
        std::bind(&TaskFsmNode::fsm_loop, this));

    RCLCPP_INFO(this->get_logger(), "Task FSM node initialized");
  }

private:
  // ── Helpers ──────────────────────────────────────────────────────────────

  void claw_gripper(float position) {
    // servo 2 = gripper: position 0.0 = open, 1.0 = closed
    robot_interfaces::msg::ClawCommand cmd;
    cmd.mode     = 2;
    cmd.position = position;
    claw_pub_->publish(cmd);
  }

  void claw_rotation(float position) {
    // servo 1 = rotation: position 0.0 = level, 1.0 = carry
    robot_interfaces::msg::ClawCommand cmd;
    cmd.mode     = 1;
    cmd.position = position;
    claw_pub_->publish(cmd);
  }

  void set_drive_enable(bool enabled) {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    enable_pub_->publish(msg);
  }

  void publish_turn(double omega) {
    geometry_msgs::msg::Twist twist;
    twist.angular.z = omega;
    cmd_pub_->publish(twist);
  }

  // ── Callbacks ─────────────────────────────────────────────────────────────

  void line_callback(const robot_interfaces::msg::LineObservation::SharedPtr msg) {
    line_valid_ = msg->valid;
    if (line_valid_) last_line_valid_time_ = this->now();
  }

  void detection_callback(const robot_interfaces::msg::Detections2D::SharedPtr msg) {
    latest_detections_ = *msg;
  }

  void hw_callback(const robot_interfaces::msg::HwStatus::SharedPtr /*msg*/) {
    hw_ready_ = true;
  }

  void estop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && current_state_ != State::FAILSAFE_STOP)
      transition_to(State::FAILSAFE_STOP);
  }

  void transition_to(State new_state) {
    if (new_state == current_state_) return;
    RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s",
        state_to_string(current_state_).c_str(),
        state_to_string(new_state).c_str());
    current_state_    = new_state;
    state_start_time_ = this->now();
    detection_count_  = 0;
    if (new_state == State::FAILSAFE_STOP) {
      claw_gripper(0.0);  // open gripper once on entry, not every tick
    }
  }

  // ── Main loop ─────────────────────────────────────────────────────────────

  void fsm_loop() {
    std_msgs::msg::String state_msg;
    state_msg.data = state_to_string(current_state_);
    state_pub_->publish(state_msg);

    // Line-loss warning — log only, do not failsafe.
    // The line follow controller already stops the robot when the line is lost.
    // Entering FAILSAFE_STOP would open the claw and drop the payload.
    if (current_state_ == State::FOLLOW_LINE_SEARCH ||
        current_state_ == State::RETURN_FOLLOW_LINE) {
      double line_loss_time = (this->now() - last_line_valid_time_).seconds();
      if (line_loss_time > line_loss_timeout_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Line lost for %.1f s — robot stopped, waiting to reacquire",
            line_loss_time);
      }
    }

    switch (current_state_) {
      case State::INIT:               handle_init();               break;
      case State::FOLLOW_LINE_SEARCH: handle_follow_line_search(); break;
      case State::APPROACH_TARGET:    handle_approach_target();    break;
      case State::PICKUP:             handle_pickup();             break;
      case State::TURN_AROUND:        handle_turn_around();        break;
      case State::RETURN_FOLLOW_LINE: handle_return_follow_line(); break;
      case State::DROP:               handle_drop();               break;
      case State::DONE:               handle_done();               break;
      case State::FAILSAFE_STOP:      handle_failsafe();           break;
    }
  }

  // ── State handlers ────────────────────────────────────────────────────────

  void handle_init() {
    set_drive_enable(false);
    claw_gripper(0.0);    // servo 2 open
    claw_rotation(0.0);   // servo 1 level

    double wait_time = (this->now() - state_start_time_).seconds();
    if (wait_time < min_init_dwell_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "INIT: claw open, waiting %.1f / %.1f s", wait_time, min_init_dwell_);
      return;
    }
    if (line_valid_ && hw_ready_) {
      RCLCPP_INFO(this->get_logger(), "Systems ready, starting mission");
      transition_to(State::FOLLOW_LINE_SEARCH);
    } else if (wait_time > 10.0) {
      RCLCPP_WARN(this->get_logger(), "Timeout waiting for systems, starting anyway");
      transition_to(State::FOLLOW_LINE_SEARCH);
    }
  }

  void handle_follow_line_search() {
    set_drive_enable(true);
    auto target = find_target();
    if (target.has_value()) {
      detection_count_++;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "Blue circle seen (cx=%.2f score=%.2f) — stable count %d/%d",
          target->cx, target->score, detection_count_, stability_frames_);
      if (detection_count_ >= stability_frames_) {
        RCLCPP_INFO(this->get_logger(), "Blue circle locked — switching to approach");
        transition_to(State::APPROACH_TARGET);
      }
    } else {
      if (detection_count_ > 0)
        RCLCPP_WARN(this->get_logger(), "Blue circle lost during search, resetting count");
      detection_count_ = 0;
    }
  }

  void handle_approach_target() {
    set_drive_enable(false);  // blue circle controller takes over cmd_vel

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

    double size = std::max(target->w, target->h);
    bool centered = std::abs(target->cx - 0.5) < center_tol_x_ &&
                    std::abs(target->cy - 0.5) < center_tol_y_;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
        "Approaching — cx=%.2f size=%.2f need:>=%.2f centered=%s",
        target->cx, size, size_close_threshold_, centered ? "YES" : "no");

    if (size >= size_close_threshold_ && centered) {
      RCLCPP_INFO(this->get_logger(),
          "Blue circle close and centered (size=%.2f) — pickup", size);
      transition_to(State::PICKUP);
    }
  }

  void handle_pickup() {
    set_drive_enable(false);
    double t = (this->now() - state_start_time_).seconds();

    // Phase 1: stop wheels — actively publish Twist(0,0)
    if (t < pickup_stop_dwell_) {
      publish_turn(0.0);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
          "PICKUP: stopping wheels (%.2f / %.2f s)", t, pickup_stop_dwell_);
      return;
    }

    // Phase 2: close gripper (servo 2)
    double close_end = pickup_stop_dwell_ + pickup_close_time_;
    if (t < close_end) {
      publish_turn(0.0);
      claw_gripper(1.0);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 400,
          "PICKUP: closing gripper (%.2f / %.2f s)", t, close_end);
      return;
    }

    // Phase 3: rotate claw to carry position (servo 1)
    double rotate_end = close_end + pickup_rotate_time_;
    if (t < rotate_end) {
      publish_turn(0.0);
      claw_gripper(1.0);
      claw_rotation(1.0);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 400,
          "PICKUP: rotating claw to carry (%.2f / %.2f s)", t, rotate_end);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Pickup complete — spinning to find red line");
    last_line_valid_time_ = this->now();  // reset so spin doesn't immediately exit
    transition_to(State::TURN_AROUND);
  }

  void handle_turn_around() {
    set_drive_enable(false);
    claw_gripper(1.0);   // keep gripper closed
    claw_rotation(1.0);  // keep carry position

    // Tank-spin in place: one wheel forward, one reverse (angular only, linear=0)
    publish_turn(turn_around_omega_);

    double t = (this->now() - state_start_time_).seconds();

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "TURN_AROUND: spinning %.1f / %.1f s", t, turn_around_time_);

    // Spin for the full turn_around_time_s before transitioning -- this ensures
    // a complete 180 deg regardless of whether the line is briefly visible.
    if (t >= turn_around_time_) {
      RCLCPP_INFO(this->get_logger(),
          "180 deg complete after %.1f s -- resuming line follow", t);
      last_line_valid_time_ = this->now();
      transition_to(State::RETURN_FOLLOW_LINE);
    }
  }

  void handle_return_follow_line() {
    set_drive_enable(true);
    claw_gripper(1.0);   // keep gripper closed
    claw_rotation(1.0);  // keep carry position

    double state_time = (this->now() - state_start_time_).seconds();
    if (use_time_return_ && state_time > return_time_) {
      RCLCPP_INFO(this->get_logger(), "Return time elapsed, dropping");
      transition_to(State::DROP);
    }
  }

  void handle_drop() {
    set_drive_enable(false);
    claw_gripper(0.0);   // open gripper
    claw_rotation(0.0);  // return to level

    double t = (this->now() - state_start_time_).seconds();
    if (t > drop_open_time_) {
      RCLCPP_INFO(this->get_logger(), "Mission complete!");
      transition_to(State::DONE);
    }
  }

  void handle_done() {
    set_drive_enable(false);
  }

  void handle_failsafe() {
    set_drive_enable(false);
  }

  std::optional<robot_interfaces::msg::Detection2D> find_target() {
    for (const auto& det : latest_detections_.detections) {
      if (det.class_name == target_class_ && det.score >= conf_threshold_)
        return det;
    }
    return std::nullopt;
  }

  // Parameters
  std::string target_class_;
  double conf_threshold_;
  int    stability_frames_;
  double center_tol_x_, center_tol_y_;
  double size_close_threshold_;
  double pickup_stop_dwell_;
  double pickup_close_time_;
  double pickup_rotate_time_;
  double turn_around_omega_;
  double turn_around_time_;
  double drop_open_time_;
  bool   use_time_return_;
  double return_time_;
  double min_approach_dwell_;
  double line_loss_timeout_;
  double min_init_dwell_;
  double rate_hz_;

  // State
  State              current_state_;
  rclcpp::Time       state_start_time_;
  int                detection_count_;
  robot_interfaces::msg::Detections2D latest_detections_;
  bool               line_valid_;
  rclcpp::Time       last_line_valid_time_;
  bool               hw_ready_;

  // ROS interfaces
  rclcpp::Subscription<robot_interfaces::msg::LineObservation>::SharedPtr line_sub_;
  rclcpp::Subscription<robot_interfaces::msg::Detections2D>::SharedPtr    det_sub_;
  rclcpp::Subscription<robot_interfaces::msg::HwStatus>::SharedPtr        hw_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                    estop_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                     state_pub_;
  rclcpp::Publisher<robot_interfaces::msg::ClawCommand>::SharedPtr        claw_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                       enable_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                 cmd_pub_;
  rclcpp::TimerBase::SharedPtr                                            timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskFsmNode>());
  rclcpp::shutdown();
  return 0;
}
