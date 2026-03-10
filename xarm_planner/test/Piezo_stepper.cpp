#include <rclcpp/rclcpp.hpp>
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "phidgets_msgs/msg/stepper_command.hpp"
#include "phidgets_msgs/msg/stepper_state.hpp"
#include "phidgets_msgs/srv/trigger.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float32.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

// Very conservative initial thresholds (grams)
// Adjust after manual validation.
#define TOUCH_FORCE  -3.0
#define PROBE_FORCE -15.0

std::atomic<float> latest_force_atomic{0.0f};
phidgets_msgs::msg::StepperState::SharedPtr step_state;

void moveStepperTo(
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
  while (rclcpp::ok() && (!step_state || !step_state->is_moving)) {
    std::this_thread::sleep_for(1ms);
  }
  while (rclcpp::ok() && step_state && step_state->is_moving) {
    std::this_thread::sleep_for(1ms);
  }
}

void moveStepperUntilForce(
  const rclcpp::Publisher<phidgets_msgs::msg::StepperCommand>::SharedPtr & pub,
  const std::shared_ptr<rclcpp::Node> & node,
  double target,
  double velocity,
  double force_threshold)
{
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
    const float f = latest_force_atomic.load(std::memory_order_relaxed);
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 1000,
      "Stepper moving: current force = %.3f g, threshold = %.3f g",
      f, force_threshold);
    if (f < force_threshold) {
      RCLCPP_INFO(node->get_logger(),
        "Force %.3f g crossed threshold %.3f g → stopping", f, force_threshold);
      break;
    }
    std::this_thread::sleep_for(1ms);
  }

  phidgets_msgs::msg::StepperCommand stop_cmd;
  stop_cmd.mode     = phidgets_msgs::msg::StepperCommand::CONTROL_MODE_DISENGAGED;
  stop_cmd.target   = 0.0;
  stop_cmd.velocity = 0.0;
  pub->publish(stop_cmd);

  while (rclcpp::ok() && step_state && step_state->is_moving) {
    std::this_thread::sleep_for(1ms);
  }
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
      latest_force_atomic.store(msg->data, std::memory_order_relaxed);
      // Log raw FUTEK force as seen inside this node (1 Hz)
      RCLCPP_INFO_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "FUTEK raw force = %.3f g", msg->data);
    });

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

  RCLCPP_INFO(node->get_logger(), "Starting single stepper-force cycle...");

  // 1) Gentle approach until light touch
  moveStepperUntilForce(stepper_pub, node, 5.0, 0.5, TOUCH_FORCE);

  // 2) Probe slightly deeper
  moveStepperUntilForce(stepper_pub, node, 8.0, 0.2, PROBE_FORCE);

  // 3) Full retract to home (0)
  moveStepperTo(stepper_pub, node, 0.0, 2.0);

  RCLCPP_INFO(node->get_logger(), "Piezo_stepper cycle complete");

  // Stop executor and join spin thread cleanly
  executor->cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}

