#include <memory>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char ** argv)
{
  // initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>
  (
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  // create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  //spin up a single thred executor for moveit visual tools
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // create the MoveIt MoveGroup interface
  using moveit::planning_interface::MoveGroupInterface;
 // auto move_group_interface = MoveGroupInterface(node, "manipulator");
 // for ur10
auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Set moderate speeds for real robot operation
  move_group_interface.setMaxVelocityScalingFactor(0.20);     // 20% speed (moderate)
  move_group_interface.setMaxAccelerationScalingFactor(0.20); // 20% acceleration (moderate)
  RCLCPP_INFO(logger, "Speed set to 20% for safe but noticeable execution");

  // Move robot to current position first (to avoid planning from unknown state)
  RCLCPP_INFO(logger, "Getting current robot state...");
  auto current_state = move_group_interface.getCurrentState();
  
  // Get current pose to use same Z value
  auto current_pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(logger, "Current end-effector pose:");
  RCLCPP_INFO(logger, "  Position: [%.3f, %.3f, %.3f]", 
              current_pose.pose.position.x,
              current_pose.pose.position.y, 
              current_pose.pose.position.z);

  // USER'S REQUEST: Move to target pose with X=-0.36, Y=-0.55, Z = ABOVE table surface
  RCLCPP_INFO(logger, "Planning to target pose: X=-0.36, Y=-0.55, Z=0.90m (above table)");
  
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = -0.36;  // Your specified X
  target_pose.position.y = -0.55;  // Your specified Y  
  target_pose.position.z = 0.90;   // 90cm absolute height (safe above table at 75cm)
  
  // Keep current orientation (or use simple orientation)
  target_pose.orientation = current_pose.pose.orientation;
  // Alternative: Use simple orientation
  // target_pose.orientation.x = 0.0;
  // target_pose.orientation.y = 0.0;
  // target_pose.orientation.z = 0.0;
  // target_pose.orientation.w = 1.0;
  
  RCLCPP_INFO(logger, "Target pose: [%.3f, %.3f, %.3f] (safe above table)", 
              target_pose.position.x, target_pose.position.y, target_pose.position.z);
  
  // Use Cartesian pose planning
  move_group_interface.setPoseTarget(target_pose);

  //construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel()
  };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Visualize the target pose
  moveit_visual_tools.publishAxisLabeled(target_pose, "target_pose");
  moveit_visual_tools.trigger();

  //create closure functions to make drawing titles and prompting for user input easier
  auto const draw_title = [&moveit_visual_tools](auto text)
  {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    
  }();
  moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
};
  auto const prompt = [&moveit_visual_tools](auto text)
  {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
       jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur_manipulator")](auto const trajectory)
      // jmg = move_group_interface.getRobotModel()->getJointModelGroup("manipulator")](auto const trajectory)
  {
    moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
  };

  

  // create a plan to that target pose
  prompt("Press 'next' in the RvizVisualToolsGui window to plan to the Cartesian target");
  draw_title("Planning Cartesian motion");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // execute the plan if we found one
  if (success)
  {
    RCLCPP_INFO(logger, "Cartesian pose planning was successful!");
    RCLCPP_INFO(logger, "Robot will move to target: X=-0.36, Y=-0.55, Z=0.90 (above table)");
    
    // SAVE THE TRAJECTORY PLAN to hello_moveit package
    // Create planned_trajectory directory if it doesn't exist
    std::string package_path = "/home/shermin/ws_moveit/src/hello_moveit";
    std::string trajectory_dir = package_path + "/planned_trajectory";
    std::filesystem::create_directories(trajectory_dir);
    
    // Generate timestamp for unique filename
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream timestamp;
    timestamp << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
    
    std::string trajectory_file_path = trajectory_dir + "/trajectory_" + timestamp.str() + ".yaml";
    std::ofstream trajectory_file(trajectory_file_path);
    trajectory_file << "# MoveIt Trajectory Plan - Generated: " << timestamp.str() << "\n";
    trajectory_file << "# Target pose: X=-0.36, Y=-0.55, Z=0.90\n";
    trajectory_file << "joint_names: [";
    for (size_t i = 0; i < plan.trajectory.joint_trajectory.joint_names.size(); ++i) {
      trajectory_file << "'" << plan.trajectory.joint_trajectory.joint_names[i] << "'";
      if (i < plan.trajectory.joint_trajectory.joint_names.size() - 1) trajectory_file << ", ";
    }
    trajectory_file << "]\n";
    trajectory_file << "points:\n";
    for (const auto& point : plan.trajectory.joint_trajectory.points) {
      trajectory_file << "  - positions: [";
      for (size_t i = 0; i < point.positions.size(); ++i) {
        trajectory_file << point.positions[i];
        if (i < point.positions.size() - 1) trajectory_file << ", ";
      }
      trajectory_file << "]\n";
      trajectory_file << "    time_from_start: " << point.time_from_start.sec 
                      << "." << point.time_from_start.nanosec << "\n";
    }
    trajectory_file.close();
    RCLCPP_INFO(logger, "✅ Trajectory saved to %s", trajectory_file_path.c_str());
    
    // Option 2: Print trajectory info to console
    RCLCPP_INFO(logger, "📊 Trajectory contains %zu waypoints over %.2f seconds", 
                plan.trajectory.joint_trajectory.points.size(),
                plan.trajectory.joint_trajectory.points.back().time_from_start.sec + 
                plan.trajectory.joint_trajectory.points.back().time_from_start.nanosec * 1e-9);
    draw_trajectory_tool_path(plan.trajectory);
    moveit_visual_tools.trigger();
    
    // EXECUTION ENABLED FOR REAL ROBOT
    prompt("Press 'next' in the RvizVisualToolsGui window to execute the motion on REAL ROBOT");
    draw_title("Executing on Real Robot");
    moveit_visual_tools.trigger();
    
    auto execution_result = move_group_interface.execute(plan);
    if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "✅ Motion executed successfully on real robot!");
    } else {
      RCLCPP_ERROR(logger, "❌ Motion execution failed on real robot!");
    }
    
    RCLCPP_INFO(logger, "Real robot motion completed.");
  }
  else
  {
    draw_title("planning failed");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Failed to find a plan for the Cartesian target pose");
    RCLCPP_ERROR(logger, "This may be due to the kinematics solver issue we identified earlier");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
