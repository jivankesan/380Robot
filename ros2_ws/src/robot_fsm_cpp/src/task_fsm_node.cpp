/**
 * @file task_fsm_node.cpp
 * @brief Task-level finite state machine for pick-and-place mission.
 *
 * States:
 * - INIT:               Wait for sensors, hold claw open
 * - FOLLOW_LINE_SEARCH: Follow red line, watch for blue circle
 * - APPROACH_TARGET:    Visual approach to blue circle; stop when cy threshold + centered
 * - PICKUP:             Stop → close gripper (servo2) → rotate claw to carry (servo1)
 * - TURN_AROUND:        Spin ~180° in place so robot faces back down the line
 * - RETURN_FOLLOW_LINE: Follow line back, watch for green drop-zone box
 * - APPROACH_DROP:      Visual approach to green box centroid
 * - DROP:               Rotate claw horizontal → open gripper to release
 * - REVERSE_TO_LINE:    Reverse back onto red line
 * - RETURN_HOME:        Follow red line back to start
 * - DONE:               Mission complete, stopped
 * - FAILSAFE_STOP:      Error state, open claw
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
  APPROACH_DROP,
  DROP,
  REVERSE_TO_LINE,
  RETURN_HOME,
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
    case State::APPROACH_DROP:      return "APPROACH_DROP";
    case State::DROP:               return "DROP";
    case State::REVERSE_TO_LINE:    return "REVERSE_TO_LINE";
    case State::RETURN_HOME:        return "RETURN_HOME";
    case State::DONE:               return "DONE";
    case State::FAILSAFE_STOP:      return "FAILSAFE_STOP";
    default:                        return "UNKNOWN";
  }
}

class TaskFsmNode : public rclcpp::Node {
public:
  TaskFsmNode() : Node("task_fsm_node") {
    // ── Pickup target (blue circle) ──────────────────────────────────────────
    this->declare_parameter("target_class", "blue_circle");
    this->declare_parameter("detection_confidence_threshold", 0.5);
    this->declare_parameter("detection_stability_frames", 2);
    this->declare_parameter("target_center_tolerance_x", 0.12);
    // Trigger pickup when the top edge of the blue circle bounding box reaches
    // this y position (0=top of frame, 1=bottom). 0.5 = middle of frame.
    // Increase to let robot get closer before grabbing; decrease to stop earlier.
    this->declare_parameter("blue_trigger_top_y", 0.50);

    // ── Pickup sequence ──────────────────────────────────────────────────────
    this->declare_parameter("pickup_stop_dwell_s", 0.4);
    this->declare_parameter("pickup_close_time_s", 1.0);
    this->declare_parameter("pickup_rotate_time_s", 0.5);

    // ── Turn around ──────────────────────────────────────────────────────────
    // Arc pivot: linear = omega * wheelbase/2 keeps inner wheel at 0,
    // outer wheel runs full forward. Both wheels stay non-negative.
    this->declare_parameter("turn_around_omega_rps", 4.0);
    this->declare_parameter("turn_around_linear_mps", 0.30);
    this->declare_parameter("turn_around_time_s", 4.0);

    // ── Drop target (green box) ──────────────────────────────────────────────
    this->declare_parameter("drop_target_class", "green_box");
    this->declare_parameter("drop_center_tolerance_x", 0.15);
    this->declare_parameter("drop_size_threshold", 0.30);
    this->declare_parameter("drop_detection_stability_frames", 5);

    // ── Drop sequence ────────────────────────────────────────────────────────
    this->declare_parameter("drop_rotate_time_s", 0.5);
    this->declare_parameter("drop_open_time_s", 1.0);

    // ── Reverse and return home ──────────────────────────────────────────────
    this->declare_parameter("reverse_time_s", 2.0);
    this->declare_parameter("reverse_speed_mps", 0.15);
    this->declare_parameter("return_home_time_s", 8.0);

    // ── Misc ─────────────────────────────────────────────────────────────────
    this->declare_parameter("use_time_based_return", true);
    this->declare_parameter("return_time_s", 10.0);
    this->declare_parameter("min_approach_dwell_s", 2.0);
    this->declare_parameter("line_loss_timeout_s", 3.0);
    this->declare_parameter("min_init_dwell_s", 1.5);
    this->declare_parameter("rate_hz", 20.0);

    target_class_          = this->get_parameter("target_class").as_string();
    conf_threshold_        = this->get_parameter("detection_confidence_threshold").as_double();
    stability_frames_      = this->get_parameter("detection_stability_frames").as_int();
    center_tol_x_          = this->get_parameter("target_center_tolerance_x").as_double();
    blue_trigger_top_y_    = this->get_parameter("blue_trigger_top_y").as_double();

    pickup_stop_dwell_     = this->get_parameter("pickup_stop_dwell_s").as_double();
    pickup_close_time_     = this->get_parameter("pickup_close_time_s").as_double();
    pickup_rotate_time_    = this->get_parameter("pickup_rotate_time_s").as_double();

    turn_around_omega_     = this->get_parameter("turn_around_omega_rps").as_double();
    turn_around_linear_    = this->get_parameter("turn_around_linear_mps").as_double();
    turn_around_time_      = this->get_parameter("turn_around_time_s").as_double();

    drop_class_            = this->get_parameter("drop_target_class").as_string();
    drop_center_tol_x_     = this->get_parameter("drop_center_tolerance_x").as_double();
    drop_size_threshold_   = this->get_parameter("drop_size_threshold").as_double();
    drop_stability_frames_ = this->get_parameter("drop_detection_stability_frames").as_int();

    drop_rotate_time_      = this->get_parameter("drop_rotate_time_s").as_double();
    drop_open_time_        = this->get_parameter("drop_open_time_s").as_double();

    reverse_time_          = this->get_parameter("reverse_time_s").as_double();
    reverse_speed_         = this->get_parameter("reverse_speed_mps").as_double();
    return_home_time_      = this->get_parameter("return_home_time_s").as_double();

    use_time_return_       = this->get_parameter("use_time_based_return").as_bool();
    return_time_           = this->get_parameter("return_time_s").as_double();
    min_approach_dwell_    = this->get_parameter("min_approach_dwell_s").as_double();
    line_loss_timeout_     = this->get_parameter("line_loss_timeout_s").as_double();
    min_init_dwell_        = this->get_parameter("min_init_dwell_s").as_double();
    rate_hz_               = this->get_parameter("rate_hz").as_double();

    // State
    current_state_         = State::INIT;
    detection_count_       = 0;
    drop_detection_count_  = 0;
    pickup_phase_          = -1;
    drop_phase_            = -1;
    state_start_time_      = this->now();
    line_valid_            = false;
    last_line_valid_time_  = this->now();
    hw_ready_              = false;

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
    // servo 2: 0.0 = open, 1.0 = closed
    robot_interfaces::msg::ClawCommand cmd;
    cmd.mode     = 2;
    cmd.position = position;
    claw_pub_->publish(cmd);
  }

  void claw_rotation(float position) {
    // servo 1: 0.0 = horizontal/level, 1.0 = carry position
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

  void publish_twist(double linear_x, double angular_z) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = linear_x;
    twist.angular.z = angular_z;
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
    RCLCPP_INFO(this->get_logger(), ">>> %s -> %s",
        state_to_string(current_state_).c_str(),
        state_to_string(new_state).c_str());
    current_state_        = new_state;
    state_start_time_     = this->now();
    detection_count_      = 0;
    drop_detection_count_ = 0;
    pickup_phase_         = -1;
    drop_phase_           = -1;
    if (new_state == State::FAILSAFE_STOP) {
      claw_gripper(0.0);  // open gripper once on entry, not every tick
    }
  }

  // ── Main loop ─────────────────────────────────────────────────────────────

  void fsm_loop() {
    std_msgs::msg::String state_msg;
    state_msg.data = state_to_string(current_state_);
    state_pub_->publish(state_msg);

    if (current_state_ == State::FOLLOW_LINE_SEARCH ||
        current_state_ == State::RETURN_FOLLOW_LINE ||
        current_state_ == State::RETURN_HOME) {
      double line_loss_time = (this->now() - last_line_valid_time_).seconds();
      if (line_loss_time > line_loss_timeout_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Line lost for %.1f s — waiting to reacquire", line_loss_time);
      }
    }

    switch (current_state_) {
      case State::INIT:               handle_init();               break;
      case State::FOLLOW_LINE_SEARCH: handle_follow_line_search(); break;
      case State::APPROACH_TARGET:    handle_approach_target();    break;
      case State::PICKUP:             handle_pickup();             break;
      case State::TURN_AROUND:        handle_turn_around();        break;
      case State::RETURN_FOLLOW_LINE: handle_return_follow_line(); break;
      case State::APPROACH_DROP:      handle_approach_drop();      break;
      case State::DROP:               handle_drop();               break;
      case State::REVERSE_TO_LINE:    handle_reverse_to_line();    break;
      case State::RETURN_HOME:        handle_return_home();        break;
      case State::DONE:               handle_done();               break;
      case State::FAILSAFE_STOP:      handle_failsafe();           break;
    }
  }

  // ── State handlers ────────────────────────────────────────────────────────

  void handle_init() {
    set_drive_enable(false);
    claw_gripper(0.0);
    claw_rotation(0.0);

    double wait_time = (this->now() - state_start_time_).seconds();
    if (wait_time < min_init_dwell_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "INIT: waiting %.1f / %.1f s", wait_time, min_init_dwell_);
      return;
    }
    if (line_valid_ && hw_ready_) {
      RCLCPP_INFO(this->get_logger(), "Systems ready — starting mission");
      transition_to(State::FOLLOW_LINE_SEARCH);
    } else if (wait_time > 10.0) {
      RCLCPP_WARN(this->get_logger(), "Timeout waiting for systems, starting anyway");
      transition_to(State::FOLLOW_LINE_SEARCH);
    }
  }

  void handle_follow_line_search() {
    set_drive_enable(true);
    auto target = find_detection(target_class_, conf_threshold_);
    if (target.has_value()) {
      // Phase 1: blue seen — stop line following immediately, start slow creep
      RCLCPP_INFO(this->get_logger(),
          "Blue circle seen (cx=%.2f top_y=%.2f) -- stopping line follow",
          target->cx, target->cy - target->h / 2.0);
      transition_to(State::APPROACH_TARGET);
    } else {
      detection_count_ = 0;
    }
  }

  // Phase 2: creep slowly toward blue circle until top edge reaches middle of frame.
  void handle_approach_target() {
    set_drive_enable(false);  // visual controller owns cmd_vel — creeps at 0.08 m/s

    auto target = find_detection(target_class_, conf_threshold_);
    double dwell_time = (this->now() - state_start_time_).seconds();

    if (!target.has_value()) {
      detection_count_++;
      if (dwell_time > min_approach_dwell_ && detection_count_ > stability_frames_ * 2) {
        RCLCPP_WARN(this->get_logger(), "Lost blue circle -- returning to search");
        transition_to(State::FOLLOW_LINE_SEARCH);
      }
      return;
    }
    detection_count_ = 0;

    double top_y = target->cy - target->h / 2.0;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
        "APPROACH: top_y=%.2f (trigger>=%.2f)", top_y, blue_trigger_top_y_);

    if (top_y >= blue_trigger_top_y_) {
      RCLCPP_INFO(this->get_logger(),
          "Blue circle top at mid-frame (top_y=%.2f) -- grabbing", top_y);
      transition_to(State::PICKUP);
    }
  }

  void handle_pickup() {
    set_drive_enable(false);
    double t = (this->now() - state_start_time_).seconds();

    // Phase 0: stop wheels
    if (t < pickup_stop_dwell_) {
      publish_twist(0.0, 0.0);
      if (pickup_phase_ != 0) {
        RCLCPP_INFO(this->get_logger(), "[PICKUP] Stopping wheels before grab");
        pickup_phase_ = 0;
      }
      return;
    }

    // Phase 1: close gripper
    double close_end = pickup_stop_dwell_ + pickup_close_time_;
    if (t < close_end) {
      publish_twist(0.0, 0.0);
      claw_gripper(1.0);
      if (pickup_phase_ != 1) {
        RCLCPP_INFO(this->get_logger(), "[PICKUP] Closing gripper — grabbing target");
        pickup_phase_ = 1;
      }
      return;
    }

    // Phase 2: rotate claw to carry position
    double rotate_end = close_end + pickup_rotate_time_;
    if (t < rotate_end) {
      publish_twist(0.0, 0.0);
      claw_gripper(1.0);
      claw_rotation(1.0);
      if (pickup_phase_ != 2) {
        RCLCPP_INFO(this->get_logger(), "[PICKUP] Rotating claw to carry position");
        pickup_phase_ = 2;
      }
      return;
    }

    RCLCPP_INFO(this->get_logger(), "[PICKUP] Complete — target secured");
    last_line_valid_time_ = this->now();
    transition_to(State::TURN_AROUND);
  }

  void handle_turn_around() {
    set_drive_enable(false);
    claw_gripper(1.0);
    claw_rotation(1.0);
    publish_twist(turn_around_linear_, turn_around_omega_);

    double t = (this->now() - state_start_time_).seconds();
    if (pickup_phase_ != 99) {
      RCLCPP_INFO(this->get_logger(), "[TURN] Spinning 180 deg (%.1f s)", turn_around_time_);
      pickup_phase_ = 99;  // reuse as turn-started flag
    }

    if (t >= turn_around_time_) {
      RCLCPP_INFO(this->get_logger(), "[TURN] 180 complete — returning to line");
      last_line_valid_time_ = this->now();
      transition_to(State::RETURN_FOLLOW_LINE);
    }
  }

  // Follow line back while watching for green drop-zone box.
  // Transitions to APPROACH_DROP when green box is stably detected.
  void handle_return_follow_line() {
    set_drive_enable(true);
    claw_gripper(1.0);
    claw_rotation(1.0);

    auto drop_target = find_detection(drop_class_, conf_threshold_);
    if (drop_target.has_value()) {
      drop_detection_count_++;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "Green box seen (cx=%.2f score=%.2f) — stable %d/%d",
          drop_target->cx, drop_target->score,
          drop_detection_count_, drop_stability_frames_);
      if (drop_detection_count_ >= drop_stability_frames_) {
        RCLCPP_INFO(this->get_logger(), "Green box locked — approaching drop zone");
        transition_to(State::APPROACH_DROP);
      }
    } else {
      drop_detection_count_ = 0;
    }

    // Fallback: drop at current position if green box never found
    double state_time = (this->now() - state_start_time_).seconds();
    if (use_time_return_ && state_time > return_time_) {
      RCLCPP_WARN(this->get_logger(), "Return timeout — dropping at current position");
      transition_to(State::DROP);
    }
  }

  // Visual approach controller steers toward green box centroid.
  // Stop when box fills drop_size_threshold and is centered.
  void handle_approach_drop() {
    set_drive_enable(false);  // visual approach controller owns cmd_vel
    claw_gripper(1.0);
    claw_rotation(1.0);

    auto target = find_detection(drop_class_, conf_threshold_);
    double dwell_time = (this->now() - state_start_time_).seconds();

    if (!target.has_value()) {
      drop_detection_count_++;
      if (dwell_time > min_approach_dwell_ && drop_detection_count_ > drop_stability_frames_ * 2) {
        RCLCPP_WARN(this->get_logger(), "Lost green box — returning to line follow");
        transition_to(State::RETURN_FOLLOW_LINE);
      }
      return;
    }
    drop_detection_count_ = 0;

    double size = std::max(target->w, target->h);
    bool centered_x = std::abs(target->cx - 0.5) < drop_center_tol_x_;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
        "APPROACH_DROP cx=%.2f size=%.2f (need>=%.2f) centered=%s",
        target->cx, size, drop_size_threshold_, centered_x ? "YES" : "no");

    if (size >= drop_size_threshold_ && centered_x) {
      RCLCPP_INFO(this->get_logger(), "At drop zone — dropping payload");
      transition_to(State::DROP);
    }
  }

  void handle_drop() {
    set_drive_enable(false);
    double t = (this->now() - state_start_time_).seconds();

    // Phase 0: rotate claw to horizontal (level) before opening
    if (t < drop_rotate_time_) {
      publish_twist(0.0, 0.0);
      claw_gripper(1.0);   // keep closed while rotating
      claw_rotation(0.0);  // rotate to horizontal
      if (drop_phase_ != 0) {
        RCLCPP_INFO(this->get_logger(), "[DROP] Rotating claw to horizontal");
        drop_phase_ = 0;
      }
      return;
    }

    // Phase 1: open gripper to release payload
    double open_end = drop_rotate_time_ + drop_open_time_;
    if (t < open_end) {
      publish_twist(0.0, 0.0);
      claw_gripper(0.0);
      claw_rotation(0.0);
      if (drop_phase_ != 1) {
        RCLCPP_INFO(this->get_logger(), "[DROP] Opening gripper — releasing payload");
        drop_phase_ = 1;
      }
      return;
    }

    RCLCPP_INFO(this->get_logger(), "[DROP] Complete — reversing to red line");
    transition_to(State::REVERSE_TO_LINE);
  }

  // Reverse at constant speed to reacquire the red line behind the robot.
  void handle_reverse_to_line() {
    set_drive_enable(false);
    claw_gripper(0.0);
    claw_rotation(0.0);
    publish_twist(-reverse_speed_, 0.0);

    double t = (this->now() - state_start_time_).seconds();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "REVERSE_TO_LINE %.1f / %.1f s", t, reverse_time_);

    if (t >= reverse_time_) {
      RCLCPP_INFO(this->get_logger(), "Back on line — following red line home");
      last_line_valid_time_ = this->now();
      transition_to(State::RETURN_HOME);
    }
  }

  // Follow red line back to start.
  void handle_return_home() {
    set_drive_enable(true);
    claw_gripper(0.0);
    claw_rotation(0.0);

    double state_time = (this->now() - state_start_time_).seconds();
    if (state_time > return_home_time_) {
      RCLCPP_INFO(this->get_logger(), "Mission complete!");
      transition_to(State::DONE);
    }
  }

  void handle_done() {
    set_drive_enable(false);
    publish_twist(0.0, 0.0);
  }

  void handle_failsafe() {
    set_drive_enable(false);
    claw_gripper(0.0);
    publish_twist(0.0, 0.0);
  }

  std::optional<robot_interfaces::msg::Detection2D> find_detection(
      const std::string& class_name, double threshold) {
    for (const auto& det : latest_detections_.detections) {
      if (det.class_name == class_name && det.score >= threshold)
        return det;
    }
    return std::nullopt;
  }

  // ── Parameters ───────────────────────────────────────────────────────────
  std::string target_class_;
  double conf_threshold_;
  int    stability_frames_;
  double center_tol_x_;
  double blue_trigger_top_y_;

  double pickup_stop_dwell_;
  double pickup_close_time_;
  double pickup_rotate_time_;

  double turn_around_omega_;
  double turn_around_linear_;
  double turn_around_time_;

  std::string drop_class_;
  double drop_center_tol_x_;
  double drop_size_threshold_;
  int    drop_stability_frames_;

  double drop_rotate_time_;
  double drop_open_time_;

  double reverse_time_;
  double reverse_speed_;
  double return_home_time_;

  bool   use_time_return_;
  double return_time_;
  double min_approach_dwell_;
  double line_loss_timeout_;
  double min_init_dwell_;
  double rate_hz_;

  // ── State ────────────────────────────────────────────────────────────────
  State              current_state_;
  rclcpp::Time       state_start_time_;
  int                detection_count_;
  int                drop_detection_count_;
  int                pickup_phase_;   // -1 = not started; tracks phase within PICKUP/TURN_AROUND
  int                drop_phase_;     // -1 = not started; tracks phase within DROP
  robot_interfaces::msg::Detections2D latest_detections_;
  bool               line_valid_;
  rclcpp::Time       last_line_valid_time_;
  bool               hw_ready_;

  // ── ROS interfaces ───────────────────────────────────────────────────────
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
