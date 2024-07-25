#include <memory>
#include <thread>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>

struct Point {
  double x, y, z;
};

// Function to compute the distance between two points
double compute_distance(const Point &point1, const Point &point2) {
  return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2) + std::pow(point1.z - point2.z, 2));
}

// Function to compute repulsion force from an obstacle
Point compute_repulsion_force(const Point &current_position, const Point &obstacle_position, double safe_distance, double gain) {
  Point f_repulsion;
  double distance = compute_distance(current_position, obstacle_position);

  if (distance < safe_distance) {
    if (distance == 0) {
      distance = 1e-6;
    }
    double f_repulsion_strength = gain * (1.0 / distance - 1.0 / safe_distance) / (distance * distance);
    f_repulsion.x = f_repulsion_strength * (current_position.x - obstacle_position.x);
    f_repulsion.y = f_repulsion_strength * (current_position.y - obstacle_position.y);
    f_repulsion.z = f_repulsion_strength * (current_position.z - obstacle_position.z);
  }
  else {
    f_repulsion = {0, 0, 0};
  }

  return f_repulsion;
}

// Function to update the position using artificial potential field method
Point update_position(const Point &current_position, const Point &target_position, const Point &f_repulsion, double step_size) {
  Point updated_position;
  updated_position.x = current_position.x + step_size * (target_position.x - current_position.x + f_repulsion.x);
  updated_position.y = current_position.y + step_size * (target_position.y - current_position.y + f_repulsion.y);
  updated_position.z = current_position.z + step_size * (target_position.z - current_position.z + f_repulsion.z);
  return updated_position;
}

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("obstacle_avoidance", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("obstacle_avoidance");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools {
    node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()
  };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.3;
    msg.position.y = 0.0;
    msg.position.z = 0.5;
    return msg;
  }();

  // Parameters for the artificial potential field
  double safe_distance = 0.05;
  double gain = 10.0;
  double step_size = 0.05;

  // Initial position of the end effector
  auto const initial_pose = move_group_interface.getCurrentPose().pose;
  Point current_position = {initial_pose.position.x, initial_pose.position.y, initial_pose.position.z};

  // Target position
  Point target_position = {target_pose.position.x, target_pose.position.y, target_pose.position.z};

  // Obstacle position
  Point obstacle_position = {0.5, 0.0, 0.5};

  // Display the obstacle
  geometry_msgs::msg::Pose obstacle_pose;
  obstacle_pose.position.x = obstacle_position.x;
  obstacle_pose.position.y = obstacle_position.y;
  obstacle_pose.position.z = obstacle_position.z;
  obstacle_pose.orientation.w = 1.0;
  moveit_visual_tools.publishCuboid(obstacle_pose, 0.02, 0.02, 0.02, rviz_visual_tools::RED);
  moveit_visual_tools.trigger();

  // Path planning using artificial potential field
  std::vector<Point> path;
  path.push_back(current_position);
  while (compute_distance(current_position, target_position) > 0.01) {
    Point f_repulsion = compute_repulsion_force(current_position, obstacle_position, safe_distance, gain);
    current_position = update_position(current_position, target_position, f_repulsion, step_size);
    path.push_back(current_position);
  }

  // Move the robot arm along the calculated path
  for (const auto &point : path) {
    geometry_msgs::msg::Pose waypoint;
    waypoint.position.x = point.x;
    waypoint.position.y = point.y;
    waypoint.position.z = point.z;
    waypoint.orientation.w = 1.0;

    move_group_interface.setPoseTarget(waypoint);
    auto const [success, plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success) {
      move_group_interface.execute(plan);
      RCLCPP_INFO(logger, "Plan and execute successful!");
    }
    else {
      RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Display the current waypoint
    moveit_visual_tools.publishSphere(waypoint, rviz_visual_tools::GREEN, rviz_visual_tools::SMALL);
    moveit_visual_tools.trigger();
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
