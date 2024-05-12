cmake_minimum_required(VERSION 3.8)
project(main)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies for C+
find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# find dependencies for python
find_package(ament_cmake_python REQUIRED)
find_package(rclpy)

set(dependencies "rclcpp" "std_msgs" "geometry_msgs")

# Adding Executables
add_executable(xarm6_velocity_publisher src/xarm6_velocity_publisher.cpp)


# Include directories for the executables
  target_include_directories(xarm6_velocity_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Compile features for the executables
target_compile_features(xarm6_velocity_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Dependencies for the executables
ament_target_dependencies(xarm6_velocity_publisher 
"rclcpp"
"std_msgs"
"controller_manager_msgs"
)

# ament_target_dependencies(panda
#   ${dependencies}
# )

# Install executables C+
install(TARGETS
  xarm6_velocity_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install executables Python
install(PROGRAMS
  scripts/converting_video_frames_to_point_clouds.py
  scripts/joint_position_controller.py
  scripts/joint_velocity_controller.py
  scripts/move_arm.py
  scripts/publishing_point_cloud_map.py
  scripts/publishing_point_cloud.py
  scripts/topic_listener.py
  scripts/xarm6_traj_controller_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch 
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