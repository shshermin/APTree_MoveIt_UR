cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_visual_tools REQUIRED)

add_executable(hello_moveit src/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

# Test robot state connectivity
add_executable(test_robot_state src/test_robot_state.cpp)
target_include_directories(test_robot_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_robot_state PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  test_robot_state
  "moveit_ros_planning_interface"
  "rclcpp"
)

# Test joint space planning
add_executable(test_joint_planning src/test_joint_planning.cpp)
target_include_directories(test_joint_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_joint_planning PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  test_joint_planning
  "moveit_ros_planning_interface"
  "rclcpp"
)

# Test named targets
add_executable(test_named_targets src/test_named_targets.cpp)
target_include_directories(test_named_targets PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_named_targets PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  test_named_targets
  "moveit_ros_planning_interface"
  "rclcpp"
)

# Test pose planning
add_executable(test_pose_planning src/test_pose_planning.cpp)
target_include_directories(test_pose_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_pose_planning PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  test_pose_planning
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS hello_moveit test_robot_state test_joint_planning test_named_targets test_pose_planning
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py")

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.urdf" PATTERN "*.xacro")

# Install mesh files
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
  FILES_MATCHING PATTERN "*.dae" PATTERN "*.stl" PATTERN "*.obj")

# Install config files (including SRDF xacro files)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.rviz" PATTERN "*.yaml" PATTERN "*.xacro" PATTERN "*.srdf")

# Install Python scripts
install(PROGRAMS
  scripts/publish_scene.py
  scripts/publish_scene_mesh.py
  scripts/dynamic_scene_example.py
  scripts/move_to_task.py
  scripts/pick_task.py
  scripts/test_move_and_pick.py
  scripts/mtc_pick_task.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the .setup_assistant file
install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
