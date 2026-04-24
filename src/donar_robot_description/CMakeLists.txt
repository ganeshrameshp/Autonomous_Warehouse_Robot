cmake_minimum_required(VERSION 3.8)
project(donar_robot_description VERSION 1.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  urdf
  meshes
  config
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/capture_goal_poses.py
  scripts/cmd_vel_relay.py
  scripts/dynamic_obstacle_controller.py
  scripts/initial_pose_publisher.py
  scripts/mission_planner.py
  scripts/nav2_activator.py
  scripts/odometry_tf_broadcaster.py
  scripts/robot_description_publisher.py
  scripts/scan_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
