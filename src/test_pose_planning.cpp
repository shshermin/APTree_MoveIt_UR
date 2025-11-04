#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "test_pose_planning",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("test_pose_planning");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  try {
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
    
    move_group_interface.setMaxVelocityScalingFactor(0.1);
    move_group_interface.setMaxAccelerationScalingFactor(0.1);
    
    RCLCPP_INFO(logger, "Testing Cartesian pose planning...");
    
    // Wait for robot state
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // Get current pose first
    auto current_pose = move_group_interface.getCurrentPose();
    RCLCPP_INFO(logger, "Current end-effector pose:");
    RCLCPP_INFO(logger, "  Position: [%.3f, %.3f, %.3f]", 
                current_pose.pose.position.x,
                current_pose.pose.position.y, 
                current_pose.pose.position.z);
    RCLCPP_INFO(logger, "  Orientation: [%.3f, %.3f, %.3f, %.3f]",
                current_pose.pose.orientation.x,
                current_pose.pose.orientation.y,
                current_pose.pose.orientation.z,
                current_pose.pose.orientation.w);
    
    // Test 1: Move to current pose (should be trivial)
    RCLCPP_INFO(logger, "\n=== Test 1: Move to current pose (trivial) ===");
    move_group_interface.setPoseTarget(current_pose.pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (move_group_interface.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success1) {
      RCLCPP_INFO(logger, "✓ Current pose planning SUCCEEDED");
    } else {
      RCLCPP_ERROR(logger, "✗ Current pose planning FAILED - This indicates kinematics issue!");
    }
    
    // Test 2: Small position offset (move Z up by 5cm)
    RCLCPP_INFO(logger, "\n=== Test 2: Small Z offset (+5cm) ===");
    geometry_msgs::msg::Pose offset_pose = current_pose.pose;
    offset_pose.position.z += 0.05; // 5cm up
    
    move_group_interface.setPoseTarget(offset_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    bool success2 = (move_group_interface.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success2) {
      RCLCPP_INFO(logger, "✓ Small Z offset planning SUCCEEDED");
    } else {
      RCLCPP_ERROR(logger, "✗ Small Z offset planning FAILED");
    }
    
    // Test 3: Check if the issue is with orientation
    RCLCPP_INFO(logger, "\n=== Test 3: Simplified orientation (identity quaternion) ===");
    geometry_msgs::msg::Pose simple_pose = current_pose.pose;
    simple_pose.orientation.x = 0.0;
    simple_pose.orientation.y = 0.0;
    simple_pose.orientation.z = 0.0;
    simple_pose.orientation.w = 1.0;
    
    move_group_interface.setPoseTarget(simple_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan3;
    bool success3 = (move_group_interface.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success3) {
      RCLCPP_INFO(logger, "✓ Simplified orientation planning SUCCEEDED");
    } else {
      RCLCPP_ERROR(logger, "✗ Simplified orientation planning FAILED");
    }
    
    // Test 4: Try position-only target
    RCLCPP_INFO(logger, "\n=== Test 4: Position-only target ===");
    move_group_interface.setPositionTarget(
      current_pose.pose.position.x,
      current_pose.pose.position.y,
      current_pose.pose.position.z + 0.05
    );
    
    moveit::planning_interface::MoveGroupInterface::Plan plan4;
    bool success4 = (move_group_interface.plan(plan4) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success4) {
      RCLCPP_INFO(logger, "✓ Position-only target SUCCEEDED");
    } else {
      RCLCPP_ERROR(logger, "✗ Position-only target FAILED");
    }
    
    // Test 5: Check planning timeout settings
    RCLCPP_INFO(logger, "\n=== Test 5: Checking planning settings ===");
    RCLCPP_INFO(logger, "Planning time: %.3f seconds", move_group_interface.getPlanningTime());
    RCLCPP_INFO(logger, "Goal tolerance: %.6f", move_group_interface.getGoalPositionTolerance());
    
    // Increase planning time and try again
    move_group_interface.setPlanningTime(30.0); // 30 seconds
    move_group_interface.setNumPlanningAttempts(10);
    
    RCLCPP_INFO(logger, "\n=== Test 6: Retry with extended planning time ===");
    move_group_interface.setPoseTarget(offset_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan6;
    bool success6 = (move_group_interface.plan(plan6) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success6) {
      RCLCPP_INFO(logger, "✓ Extended planning time SUCCEEDED");
    } else {
      RCLCPP_ERROR(logger, "✗ Extended planning time FAILED");
    }
    
    // Summary
    RCLCPP_INFO(logger, "\n=== CARTESIAN POSE PLANNING SUMMARY ===");
    RCLCPP_INFO(logger, "Current pose: %s", success1 ? "✓ PASS" : "✗ FAIL");
    RCLCPP_INFO(logger, "Small Z offset: %s", success2 ? "✓ PASS" : "✗ FAIL");
    RCLCPP_INFO(logger, "Simple orientation: %s", success3 ? "✓ PASS" : "✗ FAIL");
    RCLCPP_INFO(logger, "Position-only: %s", success4 ? "✓ PASS" : "✗ FAIL");
    RCLCPP_INFO(logger, "Extended time: %s", success6 ? "✓ PASS" : "✗ FAIL");
    
    if (!success1) {
      RCLCPP_ERROR(logger, "⚠️  CRITICAL: Can't plan to current pose - kinematics solver issue!");
    } else if (success1 && !success2) {
      RCLCPP_WARN(logger, "⚠️  Issue with IK solver or workspace limits");
    } else if (success1 && success2) {
      RCLCPP_INFO(logger, "🎉 Cartesian pose planning works! Original issue may be specific pose.");
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Exception: %s", e.what());
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}