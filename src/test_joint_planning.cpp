#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "test_joint_planning",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("test_joint_planning");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  try {
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
    
    // Set safe speeds
    move_group_interface.setMaxVelocityScalingFactor(0.1);
    move_group_interface.setMaxAccelerationScalingFactor(0.1);
    
    RCLCPP_INFO(logger, "Testing joint space planning...");
    
    // Get current joint values first
    auto current_state = move_group_interface.getCurrentState();
    std::vector<double> current_joints;
    current_state->copyJointGroupPositions("ur_manipulator", current_joints);
    
    RCLCPP_INFO(logger, "Current joint positions:");
    const auto joint_names = move_group_interface.getJointNames();
    for (size_t i = 0; i < current_joints.size() && i < joint_names.size(); ++i) {
      RCLCPP_INFO(logger, "  %s: %.3f", joint_names[i].c_str(), current_joints[i]);
    }
    
    // Test 1: Very small movement (move shoulder_lift by 10 degrees = 0.174 rad)
    RCLCPP_INFO(logger, "\n=== Test 1: Small joint movement (shoulder_lift +10°) ===");
    std::vector<double> target_joints = current_joints;
    target_joints[1] += 0.174; // shoulder_lift_joint is at index 1
    
    move_group_interface.setJointValueTarget(target_joints);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (move_group_interface.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success1) {
      RCLCPP_INFO(logger, "✓ Small joint movement planning SUCCEEDED");
      RCLCPP_INFO(logger, "  Trajectory has %zu waypoints", plan1.trajectory.joint_trajectory.points.size());
    } else {
      RCLCPP_ERROR(logger, "✗ Small joint movement planning FAILED");
    }
    
    // Test 2: Move to zero position (home-like)
    RCLCPP_INFO(logger, "\n=== Test 2: Move to zero position ===");
    std::vector<double> zero_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    move_group_interface.setJointValueTarget(zero_joints);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    bool success2 = (move_group_interface.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success2) {
      RCLCPP_INFO(logger, "✓ Zero position planning SUCCEEDED");
      RCLCPP_INFO(logger, "  Trajectory has %zu waypoints", plan2.trajectory.joint_trajectory.points.size());
    } else {
      RCLCPP_ERROR(logger, "✗ Zero position planning FAILED");
    }
    
    // Test 3: Move shoulder_lift to 90 degrees up (user's original request)
    RCLCPP_INFO(logger, "\n=== Test 3: Move shoulder_lift to 90° up ===");
    std::vector<double> shoulder_up_joints = current_joints;
    shoulder_up_joints[1] = -1.57; // -90 degrees (UR convention: negative is up)
    move_group_interface.setJointValueTarget(shoulder_up_joints);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan3;
    bool success3 = (move_group_interface.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success3) {
      RCLCPP_INFO(logger, "✓ Shoulder 90° up planning SUCCEEDED");
      RCLCPP_INFO(logger, "  Trajectory has %zu waypoints", plan3.trajectory.joint_trajectory.points.size());
    } else {
      RCLCPP_ERROR(logger, "✗ Shoulder 90° up planning FAILED");
    }
    
    // Summary
    RCLCPP_INFO(logger, "\n=== JOINT SPACE PLANNING SUMMARY ===");
    RCLCPP_INFO(logger, "Small movement: %s", success1 ? "✓ PASS" : "✗ FAIL");
    RCLCPP_INFO(logger, "Zero position: %s", success2 ? "✓ PASS" : "✗ FAIL");
    RCLCPP_INFO(logger, "Shoulder 90° up: %s", success3 ? "✓ PASS" : "✗ FAIL");
    
    if (success1 && success2 && success3) {
      RCLCPP_INFO(logger, "🎉 ALL joint space planning tests PASSED!");
      RCLCPP_INFO(logger, "   Problem is likely with Cartesian pose planning, not joint planning.");
    } else {
      RCLCPP_WARN(logger, "⚠️  Some joint space planning tests FAILED.");
      RCLCPP_WARN(logger, "   This indicates a deeper MoveIt configuration issue.");
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Exception: %s", e.what());
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}