#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "test_named_targets",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("test_named_targets");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  try {
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
    
    move_group_interface.setMaxVelocityScalingFactor(0.1);
    move_group_interface.setMaxAccelerationScalingFactor(0.1);
    
    RCLCPP_INFO(logger, "Testing named targets...");
    
    // Get all available named targets
    auto named_targets = move_group_interface.getNamedTargets();
    RCLCPP_INFO(logger, "Available named targets:");
    for (const auto& target : named_targets) {
      RCLCPP_INFO(logger, "  - %s", target.c_str());
    }
    
    if (named_targets.empty()) {
      RCLCPP_WARN(logger, "No named targets found in SRDF file!");
      RCLCPP_WARN(logger, "This means the SRDF file doesn't define poses like 'home' or 'ready'");
    }
    
    // Test common named targets
    std::vector<std::string> targets_to_test = {"home", "ready", "up"};
    
    for (const auto& target_name : targets_to_test) {
      RCLCPP_INFO(logger, "\n=== Testing named target: %s ===", target_name.c_str());
      
      try {
        move_group_interface.setNamedTarget(target_name);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
          RCLCPP_INFO(logger, "✓ Named target '%s' planning SUCCEEDED", target_name.c_str());
          RCLCPP_INFO(logger, "  Trajectory has %zu waypoints", plan.trajectory.joint_trajectory.points.size());
        } else {
          RCLCPP_ERROR(logger, "✗ Named target '%s' planning FAILED", target_name.c_str());
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "✗ Exception for named target '%s': %s", target_name.c_str(), e.what());
      }
    }
    
    // Test if we can get the current state as a named target
    RCLCPP_INFO(logger, "\n=== Testing current state retrieval ===");
    auto current_state = move_group_interface.getCurrentState();
    if (current_state) {
      std::vector<double> current_joints;
      current_state->copyJointGroupPositions("ur_manipulator", current_joints);
      
      RCLCPP_INFO(logger, "✓ Current state available");
      RCLCPP_INFO(logger, "Could create custom named targets from current positions");
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Exception: %s", e.what());
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}