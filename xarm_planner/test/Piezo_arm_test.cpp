#include "xarm_planner/xarm_planner.h"

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <vector>

void logPose(const geometry_msgs::msg::Pose& pose, rclcpp::Logger logger, int idx)
{
    RCLCPP_INFO(logger, "Target Pose %d:", idx);
    RCLCPP_INFO(logger, "  Position -> x: %.5f, y: %.5f, z: %.5f",
                pose.position.x, pose.position.y, pose.position.z);
    RCLCPP_INFO(logger, "  Orientation -> x: %.5f, y: %.5f, z: %.5f, w: %.5f",
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("Piezo_arm_test", node_options);
    RCLCPP_INFO(node->get_logger(), "Piezo_arm_test start");

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

    robot_model_loader::RobotModelLoader robot_model_loader(node);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    RCLCPP_INFO(node->get_logger(), "Model Frame %s",
                kinematic_model->getModelFrame().c_str());

    xarm_planner::XArmPlanner planner(node, group_name);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Hard-coded piezo test poses (meters, base frame)
    std::vector<geometry_msgs::msg::Pose> poses;

    // Home / start pose: 149, 0.3, 237.9 mm, RPY=(180°,0°,0°)
    {
        geometry_msgs::msg::Pose p;
        p.position.x = 0.149;    // 149 mm
        p.position.y = 0.0003;   // 0.3 mm
        p.position.z = 0.2379;   // 237.9 mm
        p.orientation.x = 1.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        p.orientation.w = 0.0;
        poses.push_back(p);
    }

    // Pose 1: 262.4, -454.3, 180.7 mm
    {
        geometry_msgs::msg::Pose p;
        p.position.x = 0.2624;
        p.position.y = -0.4543;
        p.position.z = 0.1807;
        p.orientation.x = 1.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        p.orientation.w = 0.0;
        poses.push_back(p);
    }

    // Pose 2: 258.5, -456.3, 180.7 mm
    {
        geometry_msgs::msg::Pose p;
        p.position.x = 0.2585;
        p.position.y = -0.4563;
        p.position.z = 0.1807;
        p.orientation.x = 1.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        p.orientation.w = 0.0;
        poses.push_back(p);
    }

    // Pose 3: 272.1, -457.3, 180.7 mm
    {
        geometry_msgs::msg::Pose p;
        p.position.x = 0.2721;
        p.position.y = -0.4573;
        p.position.z = 0.1807;
        p.orientation.x = 1.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        p.orientation.w = 0.0;
        poses.push_back(p);
    }

    // Return to home pose
    {
        geometry_msgs::msg::Pose p;
        p.position.x = 0.149;
        p.position.y = 0.0003;
        p.position.z = 0.2379;
        p.orientation.x = 1.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        p.orientation.w = 0.0;
        poses.push_back(p);
    }

    RCLCPP_INFO(node->get_logger(), "Poses size %ld", poses.size());

    rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    for (size_t idx = 0; idx < poses.size(); ++idx) {
        const rclcpp::Time loop_start = steady_clock.now();
        const auto& pose = poses[idx];

        logPose(pose, node->get_logger(), static_cast<int>(idx));

        bool plan_suc = planner.planPoseTarget(pose);
        if (!plan_suc) {
            RCLCPP_WARN(node->get_logger(), "Planning failed for index %zu", idx);
            continue;
        }

        plan_suc = planner.executePath();
        if (!plan_suc) {
            RCLCPP_WARN(node->get_logger(), "Execution failed for index %zu", idx);
            continue;
        }

        const rclcpp::Time loop_end = steady_clock.now();
        const double duration_s = (loop_end - loop_start).seconds();
        RCLCPP_INFO(node->get_logger(),
                    "Completed pose %zu in %.3f s", idx, duration_s);
    }

    RCLCPP_INFO(node->get_logger(), "Piezo_arm_test over");
    rclcpp::shutdown();
    return 0;
}

