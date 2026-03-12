#include "xarm_planner/xarm_planner.h"

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "phidgets_msgs/msg/stepper_command.hpp"
#include "phidgets_msgs/msg/stepper_state.hpp"
#include "phidgets_msgs/srv/trigger.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float32.hpp"
#include "xarm_msgs/msg/robot_msg.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <memory>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

// Keep stepper force thresholds exactly as in Piezo_stepper.cpp (grams)
#define TOUCH_FORCE  -3.0
#define PROBE_FORCE -100.0

// Safety limits
constexpr double FORCE_STALE_SEC = 1.0;                  // max age of FUTEK data
constexpr auto   MAX_PRESS_DURATION = std::chrono::seconds(22);  // per-press timeout
constexpr float  FORCE_EPS = 0.01f;                      // minimum change to treat as "real" force update
constexpr float  FORCE_CEILING = -110.0f;                 // absolute max force (grams); triggers immediate retract

std::atomic<float> latest_force_atomic{0.0f};
phidgets_msgs::msg::StepperState::SharedPtr step_state;
std::atomic<double> last_force_time_sec{0.0};
std::atomic<float>  last_force_value{0.0f};
std::atomic<double> last_force_change_time_sec{0.0};
std::atomic<bool>   soft_stop_requested{false};

static void logPose(const geometry_msgs::msg::Pose& pose, rclcpp::Logger logger, int idx)
{
  RCLCPP_INFO(logger, "Target Pose %d:", idx);
  RCLCPP_INFO(logger, "  Position -> x: %.5f, y: %.5f, z: %.5f",
              pose.position.x, pose.position.y, pose.position.z);
  RCLCPP_INFO(logger, "  Orientation -> x: %.5f, y: %.5f, z: %.5f, w: %.5f",
              pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

// Attempt to move the system into a safe state:
//  - Retract the stepper toward its home target
//  - Move the arm to the provided home pose (if planning/execution succeed)
static void goToSafeState(
  const std::shared_ptr<rclcpp::Node> & node,
  const rclcpp::Publisher<phidgets_msgs::msg::StepperCommand>::SharedPtr & stepper_pub,
  xarm_planner::XArmPlanner & planner,
  const geometry_msgs::msg::Pose & home_pose);

// Check that FUTEK data is both fresh (messages arriving)
// and changing (value not stuck at a constant beyond noise).
static bool futek_is_live_and_changing(const std::shared_ptr<rclcpp::Node> & node)
{
  const double now_sec         = node->get_clock()->now().seconds();
  const double last_msg_sec    = last_force_time_sec.load(std::memory_order_relaxed);
  const double last_change_sec = last_force_change_time_sec.load(std::memory_order_relaxed);

  constexpr double MAX_MSG_AGE_SEC   = FORCE_STALE_SEC;  // must match stale guard
  constexpr double MAX_CONST_SEC     = 3.0;              // allowed duration with no real change

  if (now_sec - last_msg_sec > MAX_MSG_AGE_SEC) {
    RCLCPP_ERROR(
      node->get_logger(),
      "FUTEK handshake failed: no recent messages for %.3f s (> %.3f s)",
      now_sec - last_msg_sec, MAX_MSG_AGE_SEC);
    return false;
  }

  if (now_sec - last_change_sec > MAX_CONST_SEC) {
    RCLCPP_ERROR(
      node->get_logger(),
      "FUTEK handshake failed: force value unchanged for %.3f s (> %.3f s) beyond noise",
      now_sec - last_change_sec, MAX_CONST_SEC);
    return false;
  }

  return true;
}

// Max time to wait for stepper to start/stop (avoids hang when already at target)
constexpr auto STEPPER_MOVE_TIMEOUT = std::chrono::seconds(5);

static void moveStepperTo(
  const rclcpp::Publisher<phidgets_msgs::msg::StepperCommand>::SharedPtr & pub,
  const std::shared_ptr<rclcpp::Node> & node,
  double target,
  double velocity)
{
  phidgets_msgs::msg::StepperCommand cmd;
  cmd.mode     = phidgets_msgs::msg::StepperCommand::CONTROL_MODE_STEP;
  cmd.target   = target;
  cmd.velocity = velocity;

  pub->publish(cmd);

  std::this_thread::sleep_for(5ms);
  const auto deadline = std::chrono::steady_clock::now() + STEPPER_MOVE_TIMEOUT;
  while (rclcpp::ok() && (!step_state || !step_state->is_moving)) {
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_WARN(node->get_logger(), "moveStepperTo(%.2f): timeout waiting for is_moving (already at target?)", target);
      return;
    }
    std::this_thread::sleep_for(1ms);
  }
  const auto deadline2 = std::chrono::steady_clock::now() + STEPPER_MOVE_TIMEOUT;
  while (rclcpp::ok() && step_state && step_state->is_moving) {
    if (std::chrono::steady_clock::now() >= deadline2) {
      RCLCPP_WARN(node->get_logger(), "moveStepperTo(%.2f): timeout waiting for motion to finish", target);
      return;
    }
    std::this_thread::sleep_for(1ms);
  }
}

// Attempt to move the system into a safe state:
//  - Retract the stepper toward its home target
//  - Move the arm to the provided home pose (if planning/execution succeed)
static void goToSafeState(
  const std::shared_ptr<rclcpp::Node> & node,
  const rclcpp::Publisher<phidgets_msgs::msg::StepperCommand>::SharedPtr & stepper_pub,
  xarm_planner::XArmPlanner & planner,
  const geometry_msgs::msg::Pose & home_pose)
{
  RCLCPP_WARN(node->get_logger(), "Entering safe-state routine: retracting stepper and moving arm home");

  // Best-effort: retract stepper to its nominal home position.
  moveStepperTo(stepper_pub, node, 0.0, 2.0);

  // Best-effort: move arm to home pose. If planning fails, we log and continue.
  logPose(home_pose, node->get_logger(), -1);
  bool plan_suc = planner.planPoseTarget(home_pose);
  if (!plan_suc) {
    RCLCPP_ERROR(node->get_logger(), "Safe-state: planning failed for home pose; arm may require manual recovery");
  } else {
    bool exec_suc = planner.executePath();
    if (!exec_suc) {
      RCLCPP_ERROR(node->get_logger(), "Safe-state: execution failed for home pose; arm may require manual recovery");
    }
  }
}

// Returns true if force threshold was reached normally, false on abnormal exit.
// On abnormal exit, retracts for safety. On normal exit, just disengages in place.
static bool moveStepperUntilForce(
  const rclcpp::Publisher<phidgets_msgs::msg::StepperCommand>::SharedPtr & pub,
  const std::shared_ptr<rclcpp::Node> & node,
  double target,
  double velocity,
  double force_threshold)
{
  bool threshold_reached = false;

  phidgets_msgs::msg::StepperCommand cmd;
  cmd.mode     = phidgets_msgs::msg::StepperCommand::CONTROL_MODE_STEP;
  cmd.target   = target;
  cmd.velocity = velocity;
  pub->publish(cmd);

  std::this_thread::sleep_for(5ms);
  // #region agent log
  {
    using namespace std::chrono;
    const auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    const float f = latest_force_atomic.load(std::memory_order_relaxed);
    const bool has_state = (step_state != nullptr);
    const bool moving = has_state && step_state->is_moving;
    std::ofstream log("/home/roboarm-skytech/Documents/tactile-data-capture/.cursor/debug.log", std::ios::app);
    if (log.is_open()) {
      log << "{\"id\":\"log_untilforce_wait_start\",\"timestamp\":" << now_ms
          << ",\"location\":\"Piezo_arm_stepper_hit_test.cpp:moveStepperUntilForce\","
          << "\"message\":\"Waiting for is_moving (NO TIMEOUT)\","
          << "\"data\":{\"target\":" << target << ",\"velocity\":" << velocity
          << ",\"force\":" << f << ",\"has_state\":" << (has_state ? "true" : "false")
          << ",\"is_moving\":" << (moving ? "true" : "false")
          << "},\"runId\":\"debug-1\",\"hypothesisId\":\"H2\"}" << std::endl;
    }
  }
  // #endregion agent log
  const auto wait_deadline = std::chrono::steady_clock::now() + STEPPER_MOVE_TIMEOUT;
  while (rclcpp::ok() && (!step_state || !step_state->is_moving)) {
    if (std::chrono::steady_clock::now() >= wait_deadline) {
      RCLCPP_WARN(node->get_logger(),
        "moveStepperUntilForce(%.2f): timeout waiting for is_moving", target);
      // #region agent log
      {
        using namespace std::chrono;
        const auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        std::ofstream log("/home/roboarm-skytech/Documents/tactile-data-capture/.cursor/debug.log", std::ios::app);
        if (log.is_open()) {
          log << "{\"id\":\"log_untilforce_wait_timeout\",\"timestamp\":" << now_ms
              << ",\"location\":\"Piezo_arm_stepper_hit_test.cpp:moveStepperUntilForce\","
              << "\"message\":\"is_moving wait TIMED OUT\","
              << "\"data\":{\"target\":" << target
              << "},\"runId\":\"debug-1\",\"hypothesisId\":\"H2\"}" << std::endl;
        }
      }
      // #endregion agent log
      return false;
    }
    std::this_thread::sleep_for(1ms);
  }

  const auto press_start = std::chrono::steady_clock::now();
  // #region agent log
  {
    using namespace std::chrono;
    const auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    const float f = latest_force_atomic.load(std::memory_order_relaxed);
    std::ofstream log("/home/roboarm-skytech/Documents/tactile-data-capture/.cursor/debug.log", std::ios::app);
    if (log.is_open()) {
      log << "{\"id\":\"log_untilforce_loop_enter\",\"timestamp\":" << now_ms
          << ",\"location\":\"Piezo_arm_stepper_hit_test.cpp:moveStepperUntilForce\","
          << "\"message\":\"Entering force-check loop\","
          << "\"data\":{\"target\":" << target << ",\"velocity\":" << velocity
          << ",\"force\":" << f << ",\"threshold\":" << force_threshold
          << ",\"already_past\":" << (f < force_threshold ? "true" : "false")
          << "},\"runId\":\"debug-1\",\"hypothesisId\":\"H1\"}" << std::endl;
    }
  }
  // #endregion agent log
  while (rclcpp::ok() && step_state && step_state->is_moving) {
    auto now_ros = node->get_clock()->now();
    const double now_sec = now_ros.seconds();

    const double last_force_sec = last_force_time_sec.load(std::memory_order_relaxed);
    if (now_sec - last_force_sec > FORCE_STALE_SEC) {
      RCLCPP_ERROR(
        node->get_logger(),
        "FUTEK force data stale for %.3f s (> %.3f s), aborting stepper motion",
        now_sec - last_force_sec, FORCE_STALE_SEC);
      break;
    }

    if (std::chrono::steady_clock::now() - press_start > MAX_PRESS_DURATION) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Stepper press exceeded max duration of %.3f s, aborting",
        std::chrono::duration<double>(MAX_PRESS_DURATION).count());
      break;
    }

    if (soft_stop_requested.load(std::memory_order_relaxed)) {
      RCLCPP_WARN(
        node->get_logger(),
        "Soft stop requested during moveStepperUntilForce; breaking to retract");
      break;
    }

    const float f = latest_force_atomic.load(std::memory_order_relaxed);
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 1000,
      "Stepper moving: current force = %.3f g, threshold = %.3f g",
      f, force_threshold);
    if (f < force_threshold) {
      RCLCPP_INFO(node->get_logger(),
        "Force %.3f g crossed threshold %.3f g → stopping", f, force_threshold);
      threshold_reached = true;
      break;
    }
    std::this_thread::sleep_for(1ms);
  }

  if (threshold_reached) {
    phidgets_msgs::msg::StepperCommand stop_cmd;
    stop_cmd.mode     = phidgets_msgs::msg::StepperCommand::CONTROL_MODE_DISENGAGED;
    stop_cmd.target   = 0.0;
    stop_cmd.velocity = 0.0;
    pub->publish(stop_cmd);

    while (rclcpp::ok() && step_state && step_state->is_moving) {
      std::this_thread::sleep_for(1ms);
    }
  } else {
    RCLCPP_WARN(node->get_logger(), "Abnormal exit from press loop — retracting to home");
    moveStepperTo(pub, node, 0.0, 2.0);
  }

  return threshold_reached;
}

static void run_stepper_press_cycle(
  const rclcpp::Publisher<phidgets_msgs::msg::StepperCommand>::SharedPtr & stepper_pub,
  const std::shared_ptr<rclcpp::Node> & node)
{
  RCLCPP_INFO(node->get_logger(), "Starting stepper press cycle: touch then probe, then retract");

  // #region agent log
  {
    using namespace std::chrono;
    const auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    const float f = latest_force_atomic.load(std::memory_order_relaxed);
    std::ofstream log("/home/roboarm-skytech/Documents/tactile-data-capture/.cursor/debug.log", std::ios::app);
    if (log.is_open()) {
      log << "{\"id\":\"log_phase0_pre\",\"timestamp\":" << now_ms
          << ",\"location\":\"Piezo_arm_stepper_hit_test.cpp:run_stepper_press_cycle\","
          << "\"message\":\"Phase 0 (fast approach) START\","
          << "\"data\":{\"force\":" << f << "},\"runId\":\"debug-1\",\"hypothesisId\":\"H1\"}" << std::endl;
    }
  }
  // #endregion agent log

  // 0) Fast approach to near-contact position (no force check)
  moveStepperTo(stepper_pub, node, 8.0, 2.0);

  // #region agent log
  {
    using namespace std::chrono;
    const auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    const float f = latest_force_atomic.load(std::memory_order_relaxed);
    std::ofstream log("/home/roboarm-skytech/Documents/tactile-data-capture/.cursor/debug.log", std::ios::app);
    if (log.is_open()) {
      log << "{\"id\":\"log_phase0_post\",\"timestamp\":" << now_ms
          << ",\"location\":\"Piezo_arm_stepper_hit_test.cpp:run_stepper_press_cycle\","
          << "\"message\":\"Phase 0 (fast approach) DONE, Phase 1 (touch) START\","
          << "\"data\":{\"force\":" << f << ",\"touch_threshold\":" << TOUCH_FORCE
          << ",\"force_already_past_touch\":" << (f < TOUCH_FORCE ? "true" : "false")
          << "},\"runId\":\"debug-1\",\"hypothesisId\":\"H1\"}" << std::endl;
    }
  }
  // #endregion agent log

  // 1) Gentle approach until light touch — stays in place on success
  bool ok = moveStepperUntilForce(stepper_pub, node, 20.0, 0.5, TOUCH_FORCE);

  // #region agent log
  {
    using namespace std::chrono;
    const auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    const float f = latest_force_atomic.load(std::memory_order_relaxed);
    std::ofstream log("/home/roboarm-skytech/Documents/tactile-data-capture/.cursor/debug.log", std::ios::app);
    if (log.is_open()) {
      log << "{\"id\":\"log_phase1_post\",\"timestamp\":" << now_ms
          << ",\"location\":\"Piezo_arm_stepper_hit_test.cpp:run_stepper_press_cycle\","
          << "\"message\":\"Phase 1 (touch) DONE\","
          << "\"data\":{\"force\":" << f << ",\"touch_ok\":" << (ok ? "true" : "false")
          << "},\"runId\":\"debug-1\",\"hypothesisId\":\"H3\"}" << std::endl;
    }
  }
  // #endregion agent log

  // 2) Probe slightly deeper (only if stage 1 succeeded)
  if (ok) {
    // #region agent log
    {
      using namespace std::chrono;
      const auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
      const float f = latest_force_atomic.load(std::memory_order_relaxed);
      std::ofstream log("/home/roboarm-skytech/Documents/tactile-data-capture/.cursor/debug.log", std::ios::app);
      if (log.is_open()) {
        log << "{\"id\":\"log_phase2_pre\",\"timestamp\":" << now_ms
            << ",\"location\":\"Piezo_arm_stepper_hit_test.cpp:run_stepper_press_cycle\","
            << "\"message\":\"Phase 2 (probe) START\","
            << "\"data\":{\"force\":" << f << ",\"probe_threshold\":" << PROBE_FORCE
            << "},\"runId\":\"debug-1\",\"hypothesisId\":\"H5\"}" << std::endl;
      }
    }
    // #endregion agent log

    moveStepperUntilForce(stepper_pub, node, 25.0, 0.2, PROBE_FORCE);

    // #region agent log
    {
      using namespace std::chrono;
      const auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
      const float f = latest_force_atomic.load(std::memory_order_relaxed);
      std::ofstream log("/home/roboarm-skytech/Documents/tactile-data-capture/.cursor/debug.log", std::ios::app);
      if (log.is_open()) {
        log << "{\"id\":\"log_phase2_post\",\"timestamp\":" << now_ms
            << ",\"location\":\"Piezo_arm_stepper_hit_test.cpp:run_stepper_press_cycle\","
            << "\"message\":\"Phase 2 (probe) DONE\","
            << "\"data\":{\"force\":" << f
            << "},\"runId\":\"debug-1\",\"hypothesisId\":\"H5\"}" << std::endl;
      }
    }
    // #endregion agent log
  } else {
    // #region agent log
    {
      using namespace std::chrono;
      const auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
      const float f = latest_force_atomic.load(std::memory_order_relaxed);
      std::ofstream log("/home/roboarm-skytech/Documents/tactile-data-capture/.cursor/debug.log", std::ios::app);
      if (log.is_open()) {
        log << "{\"id\":\"log_phase2_skipped\",\"timestamp\":" << now_ms
            << ",\"location\":\"Piezo_arm_stepper_hit_test.cpp:run_stepper_press_cycle\","
            << "\"message\":\"Phase 2 (probe) SKIPPED because touch returned false\","
            << "\"data\":{\"force\":" << f
            << "},\"runId\":\"debug-1\",\"hypothesisId\":\"H3\"}" << std::endl;
      }
    }
    // #endregion agent log
  }

  // 3) Full retract to home (0)
  moveStepperTo(stepper_pub, node, 0.0, 2.0);

  RCLCPP_INFO(node->get_logger(), "Stepper press cycle complete");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("Piezo_arm_stepper_hit_test", node_options);
  RCLCPP_INFO(node->get_logger(), "Piezo_arm_stepper_hit_test start");

  // Arm configuration (same defaults as Piezo_arm_test)
  int dof;
  node->get_parameter_or("dof", dof, 7);
  std::string robot_type;
  node->get_parameter_or("robot_type", robot_type, std::string("xarm"));
  std::string group_name = robot_type;
  if (robot_type == "xarm" || robot_type == "lite")
    group_name = robot_type + std::to_string(dof);
  std::string prefix;
  node->get_parameter_or("prefix", prefix, std::string(""));
  if (!prefix.empty()) {
    group_name = prefix + group_name;
  }

  RCLCPP_INFO(node->get_logger(), "namespace: %s, group_name: %s",
              node->get_namespace(), group_name.c_str());

  xarm_planner::XArmPlanner planner(node, group_name);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Hard-coded piezo test poses (copied from Piezo_arm_test)
  std::vector<geometry_msgs::msg::Pose> poses;

  // Pose 0: Home / start pose
  {
    geometry_msgs::msg::Pose p;
    p.position.x = 0.2954;
    p.position.y = -0.385;
    p.position.z = 0.329;
    p.orientation.x = 1.0;
    p.orientation.y = 0.0;
    p.orientation.z = 0.0;
    p.orientation.w = 0.0;
    poses.push_back(p);
  }

  // Pose 1
  {
    geometry_msgs::msg::Pose p;
    p.position.x = 0.2620;
    p.position.y = -0.4543;
    p.position.z = 0.2097;
    p.orientation.x = 1.0;
    p.orientation.y = 0.0;
    p.orientation.z = 0.0;
    p.orientation.w = 0.0;
    poses.push_back(p);
  }

  // Pose 2
  {
    geometry_msgs::msg::Pose p;
    p.position.x = 0.2583;
    p.position.y = -0.4569;
    p.position.z = 0.2097;
    p.orientation.x = 1.0;
    p.orientation.y = 0.0;
    p.orientation.z = 0.0;
    p.orientation.w = 0.0;
    poses.push_back(p);
  }

  // Pose 3
  {
    geometry_msgs::msg::Pose p;
    p.position.x = 0.2721;
    p.position.y = -0.4573;
    p.position.z = 0.2097;
    p.orientation.x = 1.0;
    p.orientation.y = 0.0;
    p.orientation.z = 0.0;
    p.orientation.w = 0.0;
    poses.push_back(p);
  }

  // Pose 4: Return home (same as pose 0)
  {
    geometry_msgs::msg::Pose p;
    p.position.x = 0.2954;
    p.position.y = -0.385;
    p.position.z = 0.329;
    p.orientation.x = 1.0;
    p.orientation.y = 0.0;
    p.orientation.z = 0.0;
    p.orientation.w = 0.0;
    poses.push_back(p);
  }

  RCLCPP_INFO(node->get_logger(), "Poses size %ld", poses.size());

  // Stepper + FUTEK setup (same topics / behavior as Piezo_stepper)
  auto stepper_pub = node->create_publisher<phidgets_msgs::msg::StepperCommand>(
    "/phidgets_stepper/command", 125);

  auto state_sub = node->create_subscription<phidgets_msgs::msg::StepperState>(
    "/phidgets_stepper/state", 125,
    [&](const phidgets_msgs::msg::StepperState::SharedPtr msg) {
      step_state = msg;
      RCLCPP_INFO_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "Stepper state: is_moving=%d, is_engaged=%d, target=%.3f",
        msg->is_moving ? 1 : 0,
        msg->is_engaged ? 1 : 0,
        msg->target_position);
    });

  auto force_sub = node->create_subscription<std_msgs::msg::Float32>(
    "/futek_force", 125,
    [&](const std_msgs::msg::Float32::SharedPtr msg) {
      const double now_sec = node->get_clock()->now().seconds();
      const float new_f = msg->data;
      const float old_f = last_force_value.load(std::memory_order_relaxed);

      latest_force_atomic.store(new_f, std::memory_order_relaxed);
      last_force_time_sec.store(now_sec, std::memory_order_relaxed);

      // Track when the force meaningfully changes (beyond sensor noise).
      if (std::fabs(new_f - old_f) > FORCE_EPS) {
        last_force_value.store(new_f, std::memory_order_relaxed);
        last_force_change_time_sec.store(now_sec, std::memory_order_relaxed);
      }
      // Force ceiling watchdog: if force exceeds absolute limit, retract immediately.
      if (new_f < FORCE_CEILING &&
          !soft_stop_requested.load(std::memory_order_relaxed))
      {
        soft_stop_requested.store(true, std::memory_order_relaxed);
        RCLCPP_ERROR(node->get_logger(),
          "FORCE CEILING breached (%.3f g < %.3f g): emergency retract!",
          new_f, FORCE_CEILING);
        moveStepperTo(stepper_pub, node, 0.0, 2.0);
      }

      RCLCPP_INFO_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "FUTEK raw force = %.3f g", msg->data);
    });

  auto enc_client = node->create_client<phidgets_msgs::srv::Trigger>(
    "/phidgets_high_speed_encoder/zero");
  auto step_client = node->create_client<std_srvs::srv::Trigger>(
    "/phidgets_stepper/zero");

  // xArm E-stop watchdog: if the arm enters state 4 (STOPPED) or reports a non-zero
  // error code (typically from E-stop), automatically trigger a stepper retract.
  auto estop_sub = node->create_subscription<xarm_msgs::msg::RobotMsg>(
    "/ufactory/robot_states", 10,
    [node, stepper_pub](const xarm_msgs::msg::RobotMsg::SharedPtr msg) {
      if ((msg->state == 4 || msg->err != 0) &&
          !soft_stop_requested.load(std::memory_order_relaxed))
      {
        soft_stop_requested.store(true, std::memory_order_relaxed);
        RCLCPP_ERROR(node->get_logger(),
          "xArm E-stop detected (state=%d, err=%d): retracting stepper to home",
          msg->state, msg->err);
        moveStepperTo(stepper_pub, node, 0.0, 2.0);
      }
    });

  // Soft-stop retract service: external trigger to safely retract to home without killing the node.
  auto soft_stop_srv = node->create_service<std_srvs::srv::Trigger>(
    "piezo_stepper/soft_stop_retract",
    [node, stepper_pub](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
      soft_stop_requested.store(true, std::memory_order_relaxed);

      // #region agent log
      {
        using namespace std::chrono;
        const auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        std::ofstream log("/home/roboarm-skytech/Documents/tactile-data-capture/.cursor/debug.log", std::ios::app);
        if (log.is_open()) {
          log << "{\"id\":\"log_soft_stop_service_arm\",\"timestamp\":" << now_ms
              << ",\"location\":\"Piezo_arm_stepper_hit_test.cpp:soft_stop_retract\","
              << "\"message\":\"soft stop retract requested (arm test)\","
              << "\"data\":{},\"runId\":\"pre-fix-1\",\"hypothesisId\":\"H3\"}"
              << std::endl;
        }
      }
      // #endregion agent log

      RCLCPP_WARN(node->get_logger(),
                  "Soft-stop service (arm test) called: retracting stepper to home (0.0) now");
      moveStepperTo(stepper_pub, node, 0.0, 2.0);

      res->success = true;
      res->message = "Stepper retracted to 0.0 by soft-stop service (arm test)";
    });

  // CSV logger run-control services
  auto logger_start_client = node->create_client<std_srvs::srv::Trigger>(
    "hit_futek_logger/start_run");
  auto logger_stop_client = node->create_client<std_srvs::srv::Trigger>(
    "hit_futek_logger/stop_run");

  // Use SingleThreadedExecutor for callbacks, as in Piezo_stepper
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread spin_thread([&executor]() {
    executor->spin();
  });

  // Zero encoder and stepper once at start (same as Piezo_stepper)
  while (!enc_client->wait_for_service(2s)) {
    RCLCPP_INFO(node->get_logger(), "Waiting for Encoder /zero service...");
  }
  auto enc_req = std::make_shared<phidgets_msgs::srv::Trigger::Request>();
  enc_req->channel = 0;
  enc_client->async_send_request(enc_req);

  while (!step_client->wait_for_service(2s)) {
    RCLCPP_INFO(node->get_logger(), "Waiting for Stepper /zero service...");
  }
  auto step_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  step_client->async_send_request(step_req);

  rclcpp::sleep_for(2s);

  // Start a new logging run (fire-and-forget; we don't require the response).
  if (logger_start_client->wait_for_service(2s)) {
    auto log_start_req = std::make_shared<std_srvs::srv::Trigger::Request>();
    logger_start_client->async_send_request(log_start_req);
  } else {
    RCLCPP_WARN(node->get_logger(),
      "HIT/FUTEK CSV logger start_run service not available, proceeding without segmented CSV");
  }

  // Orchestrated sequence: home → pose1+press → pose2+press → pose3+press → home
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  // Ensure we start at home (pose 0)
  {
    if (!rclcpp::ok()) {
      RCLCPP_WARN(node->get_logger(),
                  "Shutdown requested before initial home move; skipping sequence and going to safe state");
      goToSafeState(node, stepper_pub, planner, poses[0]);
      goto shutdown_and_exit;
    }

    const auto& pose = poses[0];
    logPose(pose, node->get_logger(), 0);
    bool plan_suc = planner.planPoseTarget(pose);
    if (!plan_suc) {
      RCLCPP_WARN(node->get_logger(), "Planning failed for initial home pose");
    } else {
      planner.executePath();
    }
  }

  // For poses 1, 2, 3: move, then run one stepper press cycle
  for (int idx = 1; idx <= 3; ++idx) {
    if (!rclcpp::ok()) {
      RCLCPP_WARN(
        node->get_logger(),
        "Shutdown requested during sequence (before pose %d); moving to safe state", idx);
      goToSafeState(node, stepper_pub, planner, poses[0]);
      goto shutdown_and_exit;
    }

    const rclcpp::Time loop_start = steady_clock.now();
    const auto& pose = poses[static_cast<size_t>(idx)];

    logPose(pose, node->get_logger(), idx);

    bool plan_suc = planner.planPoseTarget(pose);
    if (!plan_suc) {
      RCLCPP_WARN(node->get_logger(), "Planning failed for index %d", idx);
      continue;
    }

    plan_suc = planner.executePath();
    if (!plan_suc) {
      RCLCPP_WARN(node->get_logger(), "Execution failed for index %d", idx);
      continue;
    }

    const rclcpp::Time loop_end = steady_clock.now();
    const double duration_s = (loop_end - loop_start).seconds();
    RCLCPP_INFO(node->get_logger(),
                "Completed arm move to pose %d in %.3f s", idx, duration_s);

    // Safety handshake: ensure FUTEK data is live and changing before pressing.
    if (!futek_is_live_and_changing(node)) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Skipping stepper press at pose %d because FUTEK handshake failed", idx);
      continue;
    }

    RCLCPP_INFO(
      node->get_logger(),
      "Starting stepper press at pose %d (coordinate site %d)", idx, idx);
    run_stepper_press_cycle(stepper_pub, node);
  }

  // Return to final home pose (poses[4])
  {
    if (!rclcpp::ok()) {
      RCLCPP_WARN(node->get_logger(),
                  "Shutdown requested before final home move; moving to safe state");
      goToSafeState(node, stepper_pub, planner, poses[0]);
      goto shutdown_and_exit;
    }

    const auto& pose = poses[4];
    logPose(pose, node->get_logger(), 4);
    bool plan_suc = planner.planPoseTarget(pose);
    if (!plan_suc) {
      RCLCPP_WARN(node->get_logger(), "Planning failed for final home pose");
    } else {
      planner.executePath();
    }
  }

  // Ensure stepper is fully retracted at end of test
  RCLCPP_INFO(node->get_logger(), "Final safety retract of stepper to 0.0");
  moveStepperTo(stepper_pub, node, 0.0, 2.0);

  RCLCPP_INFO(node->get_logger(), "Piezo_arm_stepper_hit_test over");

  // Stop the logging run and wait for response so logger closes the CSV before we exit.
  if (logger_stop_client->wait_for_service(2s)) {
    auto log_stop_req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_future = logger_stop_client->async_send_request(log_stop_req);
    if (result_future.wait_for(3s) == std::future_status::ready) {
      RCLCPP_INFO(node->get_logger(), "Logger stop_run completed");
    } else {
      RCLCPP_WARN(node->get_logger(), "Logger stop_run timed out");
    }
  }

shutdown_and_exit:
  executor->cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}

