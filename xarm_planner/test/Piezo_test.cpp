#include "xarm_planner/xarm_planner.h"
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometric_shapes/shape_operations.h>
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <std_msgs/msg/string.hpp>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <boost/variant/get.hpp>  // for boost::get<>


#include <fstream>
#include <sstream>
#include <vector>
#include <mutex>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <thread>

#include <filesystem>
#include <iomanip>
#include <atomic>


#include <chrono>
#include <memory>

#include "phidgets_msgs/msg/stepper_command.hpp"
#include "phidgets_msgs/msg/stepper_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "phidgets_msgs/srv/trigger.hpp"


// #define Z_AXIS_RAISE -0.016 // in m
#define TRAIL_WINDOW 0.3 // in seconds
// #define DISTAL_LOC_X 0.235 // in m
// #define DISTAL_LOC_Y 0.449
#define DISTAL_LOC_X 0.236 // in m
#define DISTAL_LOC_Y -0.512
#define CALIBRATION_X 0.0015
#define CALIBRATION_Y 0.019
#define CALIBRATION_Z 0.002 + 0.02

#define DISTAL_CENTER_SCALE 0.3

struct JointSample {
    int test_idx;
    rclcpp::Time stamp;
    double pos, vel;
};
struct StepperSample {
    int test_idx;
    rclcpp::Time stamp;
    bool is_moving;
};
struct TCPCommandSample {
    int test_idx;
    rclcpp::Time stamp;
    bool is_recording;
};
struct ForceSample {
    int test_idx;
    rclcpp::Time stamp;
    float       force;
};
struct LoopTimingSample {
  int            test_idx;
  rclcpp::Time   start_s;     // steady clock time (seconds from arbitrary zero)
  rclcpp::Time   end_s;       // steady clock time
  double         duration_s;  // (end - start).seconds()
  bool           plan_suc;
};

std::vector<LoopTimingSample> loop_log;

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[Piezo_test] Ctrl-C caught, exit process...\n");
    exit(-1);
}

void logPose(const geometry_msgs::msg::Pose& pose, rclcpp::Logger logger, int idx)
{
    RCLCPP_INFO(logger, "Target Pose %d:", idx);
    RCLCPP_INFO(logger, "  Position -> x: %.5f, y: %.5f, z: %.5f",
                pose.position.x, pose.position.y, pose.position.z);
    RCLCPP_INFO(logger, "  Orientation -> x: %.5f, y: %.5f, z: %.5f, w: %.5f",
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

std::vector<geometry_msgs::msg::Pose> loadCSVPoses (const std::string& filepath) {
    std::ifstream file(filepath);
    std::string line;
    std::vector<geometry_msgs::msg::Pose> poses;

    std::getline(file, line);

    while(std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str, z_str, qx, qy, qz, qw;

        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        std::getline(ss, z_str, ',');
        std::getline(ss, qx, ',');
        std::getline(ss, qy, ',');
        std::getline(ss, qz, ',');
        std::getline(ss, qw, ',');

        if (x_str.empty() || y_str.empty() || z_str.empty() || qx.empty() || qy.empty() || qz.empty() || qw.empty())
            continue;

        // NOTE: 0.06 for risers, 0.004 for distal mount base, 0.011 for below distal
        geometry_msgs::msg::Pose pose;
        pose.position.x = (std::stod(x_str) / 1000.0) + DISTAL_LOC_X + CALIBRATION_X;
        pose.position.y = (std::stod(y_str) / 1000.0) + DISTAL_LOC_Y + CALIBRATION_Y;
        pose.position.z = (std::stod(z_str) / 1000.0) + 0.004 + 0.06 + CALIBRATION_Z;

        pose.orientation.x = std::stod(qx);
        pose.orientation.y = std::stod(qy);
        pose.orientation.z = std::stod(qz);
        pose.orientation.w = std::stod(qw);

        poses.push_back(pose);
    }
    return poses;
}

phidgets_msgs::msg::StepperState::SharedPtr step_state;
std::mutex          log_mutex;
std::mutex encoder_file_mutex;
std::vector<JointSample>   joint_log;
std::vector<StepperSample> stepper_log;
std::vector<TCPCommandSample> command_log;
std::vector<ForceSample> force_log;
std::atomic<int> current_test_idx { 0 };

rclcpp::Time      last_stop_time;
rclcpp::Duration post_log_duration = rclcpp::Duration::from_seconds(TRAIL_WINDOW);  // 100 ms

std::atomic<float> latest_force_atomic{0.0f};
std::atomic<double> latest_enc_pos{0.0};
std::atomic<double> latest_enc_vel{0.0};

void moveStepperTo(
  rclcpp::Publisher<phidgets_msgs::msg::StepperCommand>::SharedPtr pub,
  std::shared_ptr<rclcpp::Node> node,
  double target,
  double velocity)
{
    phidgets_msgs::msg::StepperCommand cmd;
    cmd.mode     = phidgets_msgs::msg::StepperCommand::CONTROL_MODE_STEP;
    cmd.target   = target;
    cmd.velocity = velocity;

    // publish
    pub->publish(cmd);

    // busy‐wait until the driver acks “moving”
    // (give it a few ms to publish the first state)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    while (rclcpp::ok() &&
            (!step_state || !step_state->is_moving))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // now wait until it reports “stopped”
    while (rclcpp::ok() &&
            step_state->is_moving)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void moveStepperUntilForce(
  rclcpp::Publisher<phidgets_msgs::msg::StepperCommand>::SharedPtr pub,
  std::shared_ptr<rclcpp::Node> node,
  double         target,
  double         velocity,
  double         force_threshold)
{
    // Start the step _towards_ your target
    phidgets_msgs::msg::StepperCommand cmd;
    cmd.mode     = phidgets_msgs::msg::StepperCommand::CONTROL_MODE_STEP;
    cmd.target   = target;
    cmd.velocity = velocity;

    pub->publish(cmd);

    // Busy‑wait until it actually starts moving
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    while (rclcpp::ok() && (!step_state || !step_state->is_moving)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Then monitor force in a tight loop
    //    as soon as we see force > threshold, break out
    while (rclcpp::ok() && step_state->is_moving) {
        const float latest_force = latest_force_atomic.load(std::memory_order_relaxed);
        if (latest_force < force_threshold) {
            RCLCPP_INFO(node->get_logger(), "Force %.3f g exceeded threshold %.3f g → stopping",
                        latest_force, force_threshold);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    phidgets_msgs::msg::StepperCommand stop_cmd;
    stop_cmd.mode     = phidgets_msgs::msg::StepperCommand::CONTROL_MODE_DISENGAGED;
    stop_cmd.target   = 0.0;
    stop_cmd.velocity = 0.0;
    pub->publish(stop_cmd);

    // wait until driver reports fully stopped
    while (rclcpp::ok() && step_state && step_state->is_moving) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Wait during trail window
    auto trail_ms = static_cast<long>(TRAIL_WINDOW * 1000.0);
    auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0
            < std::chrono::milliseconds(trail_ms))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void moveStepperUntilForceGreater(
  rclcpp::Publisher<phidgets_msgs::msg::StepperCommand>::SharedPtr pub,
  std::shared_ptr<rclcpp::Node> node,
  double         target,
  double         velocity,
  double         force_threshold)
{
    // Start the step _towards_ your target
    phidgets_msgs::msg::StepperCommand cmd;
    cmd.mode     = phidgets_msgs::msg::StepperCommand::CONTROL_MODE_STEP;
    cmd.target   = target;
    cmd.velocity = velocity;

    pub->publish(cmd);

    // Busy‑wait until it actually starts moving
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    while (rclcpp::ok() && (!step_state || !step_state->is_moving)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Then monitor force in a tight loop
    //    as soon as we see force > threshold, break out
    while (rclcpp::ok() && step_state->is_moving) {
        const float latest_force = latest_force_atomic.load(std::memory_order_relaxed);
        if (latest_force > force_threshold) {
            RCLCPP_INFO(node->get_logger(), "Force %.3f g exceeded threshold %.3f g → stopping",
                        latest_force, force_threshold);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    phidgets_msgs::msg::StepperCommand stop_cmd;
    stop_cmd.mode     = phidgets_msgs::msg::StepperCommand::CONTROL_MODE_DISENGAGED;
    stop_cmd.target   = 0.0;
    stop_cmd.velocity = 0.0;
    pub->publish(stop_cmd);

    // wait until driver reports fully stopped
    while (rclcpp::ok() && step_state && step_state->is_moving) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Wait during trail window
    auto trail_ms = static_cast<long>(TRAIL_WINDOW * 1000.0);
    auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0
            < std::chrono::milliseconds(trail_ms))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void sendProbeState(
    std::shared_ptr<rclcpp::Node> node,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr zmq_pub,
    bool is_recording,
    int idx
)
{
    std_msgs::msg::String json_msg;

    rclcpp::Time t_s = node->now();

    if(is_recording) {
        json_msg.data =
        std::string("{") +
        "\"time_stamp\":" + std::to_string(t_s.seconds()) + "," +
        "\"event\":\"stepper_start\"," +
        "\"test_idx\":"   + std::to_string(idx) +
        std::string("}");
    } else {
        json_msg.data =
        std::string("{") +
        "\"time_stamp\":" + std::to_string(t_s.seconds()) + "," +
        "\"event\":\"stepper_end\"," +
        "\"test_idx\":"   + std::to_string(idx) +
        std::string("}");
    }

    zmq_pub->publish(json_msg);

    TCPCommandSample samp;
    samp.test_idx = current_test_idx;
    samp.stamp  = t_s;
    samp.is_recording = is_recording;
    command_log.push_back(samp);
}

#define CSV_PATH "/home/roboarm-skytech/Documents/roboarm_ws/sample_goals/sampled_points_with_quat_sorted.csv"
#define TOUCH_FORCE -6.0
#define PROBE_FORCE -100.0

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("Piezo_test", node_options);
    last_stop_time = node->get_clock()->now();
    RCLCPP_INFO(node->get_logger(), "Piezo_test start");

    signal(SIGINT, exit_sig_handler);

    int dof;
    node->get_parameter_or("dof", dof, 7);
    std::string robot_type;
    node->get_parameter_or("robot_type", robot_type, std::string("xarm"));
    std::string group_name = robot_type;
    if (robot_type == "xarm" || robot_type == "lite")
        group_name = robot_type + std::to_string(dof);
    std::string prefix;
    node->get_parameter_or("prefix", prefix, std::string(""));
    if (prefix != "") {
        group_name = prefix + group_name;
    }

    RCLCPP_INFO(node->get_logger(), "namespace: %s, group_name: %s", node->get_namespace(), group_name.c_str());

    robot_model_loader::RobotModelLoader robot_model_loader(node);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    RCLCPP_INFO(node->get_logger(), "Model Frame %s", kinematic_model->getModelFrame().c_str());

    xarm_planner::XArmPlanner planner(node, group_name);

    // Create planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);

    std::ostringstream date_ss;
    date_ss << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S");
    std::string date_str = date_ss.str();

    std::string base_dir = "data/data_out_" + date_str;
    std::filesystem::create_directories(base_dir);

    auto stepper_ofs = std::make_shared<std::ofstream>(
    base_dir + "/stepper_log.csv", std::ios::out | std::ios::trunc);
    auto force_ofs = std::make_shared<std::ofstream>(
    base_dir + "/force_log.csv", std::ios::out | std::ios::trunc);
    auto encoder_ofs = std::make_shared<std::ofstream>(
    base_dir + "/joint_log.csv", std::ios::out | std::ios::trunc);

    (*stepper_ofs) << "test_idx,time_s,is_moving\n";
    (*force_ofs)   << "test_idx,time_s,force_g\n";
    (*encoder_ofs) << "test_idx,time_s,pos,vel\n";

    auto client = node->create_client<phidgets_msgs::srv::Trigger>("/phidgets_high_speed_encoder/zero");
    while (!client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_INFO(node->get_logger(), "Waiting for Encoder /zero service...");
    }
    auto request = std::make_shared<phidgets_msgs::srv::Trigger::Request>();
    auto result_future = client->async_send_request(request);

    auto client_stepper = node->create_client<std_srvs::srv::Trigger>("/phidgets_stepper/zero");
    while (!client_stepper->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_INFO(node->get_logger(), "Waiting for Stepper /zero service...");
    }
    auto req_step = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_step = client_stepper->async_send_request(req_step);

    auto stepper_pub = node->create_publisher<phidgets_msgs::msg::StepperCommand>(
    "/phidgets_stepper/command", 125);

    auto state_sub = node->create_subscription<phidgets_msgs::msg::StepperState>(
    "/phidgets_stepper/state", 125,
    [&, stepper_ofs](const phidgets_msgs::msg::StepperState::SharedPtr msg) {
        if (step_state && step_state->is_moving && !msg->is_moving) {
            last_stop_time = node->now();
        }
        step_state = msg;

        {
            std::lock_guard<std::mutex> lock(log_mutex);
            const rclcpp::Time t = node->now();

            (*stepper_ofs) << current_test_idx << ","
                           << std::fixed << std::setprecision(9) << t.seconds() << ","
                           << (msg->is_moving ? 1 : 0) << "\n";
        }
    });

    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states_encoder", 125,
    [&, encoder_ofs](const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        const bool moving       = (step_state && step_state->is_moving);
        const bool in_trail_win = (node->now() < last_stop_time + post_log_duration);
        if (!(moving || in_trail_win)) return;

        if (msg->position.empty() || msg->velocity.empty()) return;

        latest_enc_pos.store(msg->position[0], std::memory_order_relaxed);
        latest_enc_vel.store(msg->velocity[0], std::memory_order_relaxed);

        const rclcpp::Time t = node->now();;

        {
            std::lock_guard<std::mutex> lk(encoder_file_mutex);
            (*encoder_ofs) << current_test_idx << ","
                           << std::fixed << std::setprecision(9) << t.seconds() << ","
                           << msg->position[0] << ","
                           << msg->velocity[0] << "\n";
        }
    });

    auto force_sub = node->create_subscription<std_msgs::msg::Float32>(
    "/futek_force", 125,
    [&, force_ofs](const std_msgs::msg::Float32::SharedPtr msg) {
        latest_force_atomic.store(msg->data, std::memory_order_relaxed);

        const bool moving = step_state && step_state->is_moving;
        const bool in_trail_win = (node->now() < last_stop_time + post_log_duration);

        if (moving || in_trail_win) {
            std::lock_guard<std::mutex> lock(log_mutex);
            const rclcpp::Time t = node->now();

            (*force_ofs) << current_test_idx << ","
                         << std::fixed << std::setprecision(9) << t.seconds() << ","
                         << msg->data << "\n";
        }
    });

    auto zmq_pub = node->create_publisher<std_msgs::msg::String>(
    "/to_remote", rclcpp::QoS(10));

    rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    bool plan_suc;
    int num_failed = 0;
    std::vector<int> failed_indices;

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    std::thread spin_thread([&exec](){
        exec.spin();
    });

    RCLCPP_INFO(node->get_logger(), "Zeroing Encoder...");
    auto req = std::make_shared<phidgets_msgs::srv::Trigger::Request>();
    req->channel = 0;
    result_future = client->async_send_request(req);

    rclcpp::sleep_for(std::chrono::milliseconds(2000));

    // Hard-coded test poses (in robot base frame, meters)
    std::vector<geometry_msgs::msg::Pose> poses;

    // Pose 0: X=262.4 mm, Y=-454.3 mm, Z=180.7 mm, RPY=(180°,0°,0°)
    {
        geometry_msgs::msg::Pose p;
        p.position.x = 0.2624;   // 262.4 mm
        p.position.y = -0.4543;  // -454.3 mm
        p.position.z = 0.1807;   // 180.7 mm
        // Roll=180°, Pitch=0°, Yaw=0° -> quaternion (1,0,0,0)
        p.orientation.x = 1.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        p.orientation.w = 0.0;
        poses.push_back(p);
    }

    // Pose 1: X=258.5 mm, Y=-456.3 mm, Z=180.7 mm, same orientation
    {
        geometry_msgs::msg::Pose p;
        p.position.x = 0.2585;   // 258.5 mm
        p.position.y = -0.4563;  // -456.3 mm
        p.position.z = 0.1807;   // 180.7 mm
        p.orientation.x = 1.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        p.orientation.w = 0.0;
        poses.push_back(p);
    }

    // Pose 2: X=272.1 mm, Y=-457.3 mm, Z=180.7 mm, same orientation
    {
        geometry_msgs::msg::Pose p;
        p.position.x = 0.2721;   // 272.1 mm
        p.position.y = -0.4573;  // -457.3 mm
        p.position.z = 0.1807;   // 180.7 mm
        p.orientation.x = 1.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        p.orientation.w = 0.0;
        poses.push_back(p);
    }

    RCLCPP_INFO(node->get_logger(), "Poses size %ld", poses.size());

    RCLCPP_INFO(node->get_logger(), "Starting Probing Loop...");

    for (size_t idx = 0; idx < poses.size(); ++idx) {
        const rclcpp::Time loop_start = steady_clock.now();
        current_test_idx = static_cast<int>(idx);

        const auto &pose = poses[idx];
        logPose(pose, node->get_logger(), static_cast<int>(idx));
        planner.planPoseTarget(pose);
        plan_suc = planner.executePath();

        if (!plan_suc) {
            num_failed++;
            failed_indices.push_back(static_cast<int>(idx));
        } else {
            moveStepperUntilForce(stepper_pub, node, 18.0, 5.0, TOUCH_FORCE);

            req = std::make_shared<phidgets_msgs::srv::Trigger::Request>();
            req->channel = 0;
            result_future = client->async_send_request(req);

            sendProbeState(node, zmq_pub, true, current_test_idx);
            rclcpp::sleep_for(std::chrono::milliseconds(20));

            moveStepperUntilForce(stepper_pub, node, 18.0, 1.0, PROBE_FORCE);
            moveStepperUntilForceGreater(stepper_pub, node, 0.0, 1.0, TOUCH_FORCE);
            rclcpp::sleep_for(std::chrono::milliseconds(20));
            sendProbeState(node, zmq_pub, false, current_test_idx);

            moveStepperTo(stepper_pub, node,  0.0, 10.0);
        }

        const rclcpp::Time loop_end = steady_clock.now();
        const double duration_s = (loop_end - loop_start).seconds();
        loop_log.push_back(LoopTimingSample{
            static_cast<int>(idx),
            loop_start,
            loop_end,
            duration_s,
            plan_suc
        });
    }

    exec.cancel();
    spin_thread.join();

    std::ofstream cmd_file(base_dir + "/command_log.csv");
    cmd_file << "test_idx,time_s,is_recording\n";
    for (auto &s : command_log) {
    cmd_file << s.test_idx << ","
            << std::fixed << std::setprecision(9)
            << s.stamp.seconds() << ","
            << (s.is_recording ? 1 : 0) << "\n";
    }

    std::ofstream fail_file(base_dir + "/failed_indices.csv");
    fail_file << "test_idx\n";
    for (int idx : failed_indices) {
        fail_file << idx << "\n";
    }

    std::ofstream loop_file(base_dir + "/loop_log.csv");
    loop_file << "test_idx,start_s,end_s,duration_s,plan_suc\n";
    for (const auto &e : loop_log) {
        loop_file << e.test_idx << ","
                << std::fixed << std::setprecision(9)
                << e.start_s.seconds() << ","
                << e.end_s.seconds()   << ","
                << e.duration_s        << ","
                << (e.plan_suc ? 1 : 0) << "\n";
    }
    loop_file.close();

    RCLCPP_INFO(node->get_logger(), "Number of points Failed: %d", num_failed);
    RCLCPP_INFO(node->get_logger(), "Piezo_test over");

    rclcpp::shutdown();
    return 0;
}

