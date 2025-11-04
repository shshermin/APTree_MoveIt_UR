#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "test_robot_state",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("test_robot_state");

  // Spin up a single thread executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  try {
    // Create the MoveIt MoveGroup interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
    
    RCLCPP_INFO(logger, "MoveGroup interface created successfully");
    
    // Wait a moment for robot state to be available
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // Get current robot state
    auto current_state = move_group_interface.getCurrentState();
    if (current_state) {
      RCLCPP_INFO(logger, "✓ Current robot state is available");
      
      // Get current joint values
      std::vector<double> joint_values;
      current_state->copyJointGroupPositions("ur_manipulator", joint_values);
      
      RCLCPP_INFO(logger, "Current joint values:");
      const auto joint_names = move_group_interface.getJointNames();
      for (size_t i = 0; i < joint_values.size() && i < joint_names.size(); ++i) {
        RCLCPP_INFO(logger, "  %s: %.3f", joint_names[i].c_str(), joint_values[i]);
      }
      
      // Get current pose
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
                  
      // Test planning scene connectivity
      auto robot_model = move_group_interface.getRobotModel();
      if (robot_model) {
        RCLCPP_INFO(logger, "✓ Robot model is available");
        RCLCPP_INFO(logger, "  Model name: %s", robot_model->getName().c_str());
        
        auto joint_model_group = robot_model->getJointModelGroup("ur_manipulator");
        if (joint_model_group) {
          RCLCPP_INFO(logger, "✓ Joint model group 'ur_manipulator' found");
          RCLCPP_INFO(logger, "  Number of joints: %zu", joint_model_group->getJointModels().size());
        } else {
          RCLCPP_ERROR(logger, "✗ Joint model group 'ur_manipulator' not found!");
        }
      } else {
        RCLCPP_ERROR(logger, "✗ Robot model not available!");
      }
      
    } else {
      RCLCPP_ERROR(logger, "✗ Current robot state is NOT available!");
      RCLCPP_ERROR(logger, "This could indicate:");
      RCLCPP_ERROR(logger, "  - robot_state_publisher not running");
      RCLCPP_ERROR(logger, "  - joint_states not being published");
      RCLCPP_ERROR(logger, "  - MoveIt not configured properly");
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Exception occurred: %s", e.what());
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}