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

using namespace std::chrono_literals;

// Very conservative initial thresholds (grams)
// Adjust after manual validation.
// Very conservative initial thresholds (grams)
// Adjust after manual validation.
#define TOUCH_FORCE  -3.0
#define PROBE_FORCE -40.0

// Safety limits
constexpr double FORCE_STALE_SEC = 1.0;                  // max age of FUTEK data
constexpr auto   MAX_PRESS_DURATION = std::chrono::seconds(22);  // per-press timeout
constexpr float  FORCE_EPS = 0.01f;                      // minimum change to treat as "real" force update
constexpr float  FORCE_CEILING = -50.0f;                 // absolute max force (grams); triggers immediate retract

std::atomic<float> latest_force_atomic{0.0f};
phidgets_msgs::msg::StepperState::SharedPtr step_state;
std::atomic<double> last_force_time_sec{0.0};
std::atomic<float>  last_force_value{0.0f};
std::atomic<double> last_force_change_time_sec{0.0};
std::atomic<bool>   soft_stop_requested{false};

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

void moveStepperTo(
  const rclcpp::Publisher<phidgets_msgs::msg::StepperCommand>::SharedPtr & pub,
  const std::shared_ptr<rclcpp::Node> & node,
  double target,
  double velocity)
{
  // #region agent log
  {
    using namespace std::chrono;
    const auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    std::ofstream log("/home/roboarm-skytech/Documents/tactile-data-capture/.cursor/debug.log", std::ios::app);
    if (log.is_open()) {
      log << "{\"id\":\"log_moveStepperTo_start\",\"timestamp\":" << now_ms
          << ",\"location\":\"Piezo_stepper.cpp:64\",\"message\":\"moveStepperTo start\","
          << "\"data\":{\"target\":" << target << ",\"velocity\":" << velocity
          << "},\"runId\":\"pre-fix-1\",\"hypothesisId\":\"H2\"}"
          << std::endl;
    }
  }
  // #endregion agent log

  phidgets_msgs::msg::StepperCommand cmd;
  cmd.mode     = phidgets_msgs::msg::StepperCommand::CONTROL_MODE_STEP;
  cmd.target   = target;
  cmd.velocity = velocity;

  pub->publish(cmd);

  std::this_thread::sleep_for(5ms);
  while (rclcpp::ok() && (!step_state || !step_state->is_moving)) {
    std::this_thread::sleep_for(1ms);
  }
  while (rclcpp::ok() && step_state && step_state->is_moving) {
    std::this_thread::sleep_for(1ms);
  }

  // #region agent log
  {
    using namespace std::chrono;
    const auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    std::ofstream log("/home/roboarm-skytech/Documents/tactile-data-capture/.cursor/debug.log", std::ios::app);
    if (log.is_open()) {
      const double reported_target = step_state ? step_state->target_position : 0.0;
      const int is_moving_flag = step_state && step_state->is_moving ? 1 : 0;
      log << "{\"id\":\"log_moveStepperTo_end\",\"timestamp\":" << now_ms
          << ",\"location\":\"Piezo_stepper.cpp:77\",\"message\":\"moveStepperTo end\","
          << "\"data\":{\"target\":" << target << ",\"velocity\":" << velocity
          << ",\"reported_target\":" << reported_target
          << ",\"is_moving\":" << is_moving_flag
          << "},\"runId\":\"pre-fix-1\",\"hypothesisId\":\"H2\"}"
          << std::endl;
    }
  }
  // #endregion agent log
}

// Returns true if force threshold was reached normally, false on abnormal exit.
// On abnormal exit (stale data, timeout, soft stop, shutdown), retracts for safety.
// On normal threshold exit, just stops the motor in place so the next stage can continue.
bool moveStepperUntilForce(
  const rclcpp::Publisher<phidgets_msgs::msg::StepperCommand>::SharedPtr & pub,
  const std::shared_ptr<rclcpp::Node> & node,
  double target,
  double velocity,
  double force_threshold)
{
  // #region agent log
  {
    using namespace std::chrono;
    const auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    std::ofstream log("/home/roboarm-skytech/Documents/tactile-data-capture/.cursor/debug.log", std::ios::app);
    if (log.is_open()) {
      log << "{\"id\":\"log_moveStepperUntilForce_start\",\"timestamp\":" << now_ms
          << ",\"location\":\"Piezo_stepper.cpp:moveStepperUntilForce\",\"message\":\"moveStepperUntilForce start\","
          << "\"data\":{\"target\":" << target << ",\"velocity\":" << velocity
          << ",\"force_threshold\":" << force_threshold
          << "},\"runId\":\"pre-fix-1\",\"hypothesisId\":\"H1\"}"
          << std::endl;
    }
  }
  // #endregion agent log

  bool threshold_reached = false;

  phidgets_msgs::msg::StepperCommand cmd;
  cmd.mode     = phidgets_msgs::msg::StepperCommand::CONTROL_MODE_STEP;
  cmd.target   = target;
  cmd.velocity = velocity;
  pub->publish(cmd);

  std::this_thread::sleep_for(5ms);
  while (rclcpp::ok() && (!step_state || !step_state->is_moving)) {
    std::this_thread::sleep_for(1ms);
  }

  const auto press_start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && step_state && step_state->is_moving) {
    auto now_ros = node->get_clock()->now();
    const double now_sec = now_ros.seconds();

    // Safety: abort if FUTEK force data is stale
    const double last_force_sec = last_force_time_sec.load(std::memory_order_relaxed);
    if (now_sec - last_force_sec > FORCE_STALE_SEC) {
      RCLCPP_ERROR(
        node->get_logger(),
        "FUTEK force data stale for %.3f s (> %.3f s), aborting stepper motion",
        now_sec - last_force_sec, FORCE_STALE_SEC);
      break;
    }

    // Safety: abort if this press runs too long
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

  // #region agent log
  {
    using namespace std::chrono;
    const auto now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    const float last_force = latest_force_atomic.load(std::memory_order_relaxed);
    const double last_time = last_force_time_sec.load(std::memory_order_relaxed);
    const bool ok_flag = rclcpp::ok();
    std::ofstream log("/home/roboarm-skytech/Documents/tactile-data-capture/.cursor/debug.log", std::ios::app);
    if (log.is_open()) {
      log << "{\"id\":\"log_moveStepperUntilForce_exit\",\"timestamp\":" << now_ms
          << ",\"location\":\"Piezo_stepper.cpp:moveStepperUntilForce\",\"message\":\"moveStepperUntilForce exit\","
          << "\"data\":{\"target\":" << target << ",\"velocity\":" << velocity
          << ",\"force_threshold\":" << force_threshold
          << ",\"last_force\":" << last_force
          << ",\"threshold_reached\":" << (threshold_reached ? "true" : "false")
          << ",\"rclcpp_ok\":" << (ok_flag ? "true" : "false")
          << "},\"runId\":\"pre-fix-1\",\"hypothesisId\":\"H1\"}"
          << std::endl;
    }
  }
  // #endregion agent log

  if (threshold_reached) {
    // Normal exit: just stop the motor in place (disengage) so next stage can continue from here
    phidgets_msgs::msg::StepperCommand stop_cmd;
    stop_cmd.mode     = phidgets_msgs::msg::StepperCommand::CONTROL_MODE_DISENGAGED;
    stop_cmd.target   = 0.0;
    stop_cmd.velocity = 0.0;
    pub->publish(stop_cmd);
  } else {
    // Abnormal exit: retract to safe home position
    RCLCPP_WARN(node->get_logger(), "Abnormal exit from press loop — retracting to home");
    moveStepperTo(pub, node, 0.0, 2.0);
  }

  return threshold_reached;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("Piezo_stepper");
  RCLCPP_INFO(node->get_logger(), "Piezo_stepper start");

  // Dedicated executor to process callbacks (subscribers, services) in the background
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread spin_thread([&executor]() {
    executor->spin();
  });

  // Stepper command publisher
  auto stepper_pub = node->create_publisher<phidgets_msgs::msg::StepperCommand>(
    "/phidgets_stepper/command", 125);

  // Stepper state subscriber
  auto state_sub = node->create_subscription<phidgets_msgs::msg::StepperState>(
    "/phidgets_stepper/state", 125,
    [&](const phidgets_msgs::msg::StepperState::SharedPtr msg) {
      step_state = msg;
      // Log basic stepper state (1 Hz)
      RCLCPP_INFO_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "Stepper state: is_moving=%d, is_engaged=%d, target=%.3f",
        msg->is_moving ? 1 : 0,
        msg->is_engaged ? 1 : 0,
        msg->target_position);
    });

  // FUTEK force subscriber
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
          log << "{\"id\":\"log_soft_stop_service\",\"timestamp\":" << now_ms
              << ",\"location\":\"Piezo_stepper.cpp:soft_stop_retract\",\"message\":\"soft stop retract requested\","
              << "\"data\":{},\"runId\":\"pre-fix-1\",\"hypothesisId\":\"H3\"}"
              << std::endl;
        }
      }
      // #endregion agent log

      RCLCPP_WARN(node->get_logger(),
                  "Soft-stop service called: retracting stepper to home (0.0) now");
      moveStepperTo(stepper_pub, node, 0.0, 2.0);

      res->success = true;
      res->message = "Stepper retracted to 0.0 by soft-stop service";
    });

  // CSV logger run-control services
  auto logger_start_client = node->create_client<std_srvs::srv::Trigger>(
    "hit_futek_logger/start_run");
  auto logger_stop_client = node->create_client<std_srvs::srv::Trigger>(
    "hit_futek_logger/stop_run");

  // Encoder zero service
  auto enc_client = node->create_client<phidgets_msgs::srv::Trigger>(
    "/phidgets_high_speed_encoder/zero");
  // Stepper zero service
  auto step_client = node->create_client<std_srvs::srv::Trigger>(
    "/phidgets_stepper/zero");

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

  RCLCPP_INFO(node->get_logger(), "Starting single stepper-force cycle...");

  // Safety handshake: ensure FUTEK data is live and changing before pressing.
  if (!futek_is_live_and_changing(node)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Aborting stepper-force cycle because FUTEK handshake failed");
  } else {
    // 0) Fast approach to near-contact position (no force check)
    moveStepperTo(stepper_pub, node, 8.0, 2.0);

    // 1) Gentle approach until light touch — stays in place on success
    bool ok = moveStepperUntilForce(stepper_pub, node, 20.0, 0.5, TOUCH_FORCE);

    // 2) Probe slightly deeper (only if stage 1 succeeded)
    if (ok) {
      moveStepperUntilForce(stepper_pub, node, 25.0, 0.2, PROBE_FORCE);
    }

    // 3) Full retract to home (0)
    moveStepperTo(stepper_pub, node, 0.0, 2.0);
  }

  RCLCPP_INFO(node->get_logger(), "Piezo_stepper cycle complete");

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

  // Stop executor and join spin thread cleanly
  executor->cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}

