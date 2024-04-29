cmake_minimum_required(VERSION 3.8)
project(planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies for C+
find_package(ament_cmake REQUIRED)
find_package(ompl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include ${OMPL_INCLUDE_DIRS})

# find dependencies for python
find_package(ament_cmake_python REQUIRED)
find_package(rclpy)

set(dependencies "ompl" "rclcpp" "std_msgs")

# Adding Executables
add_executable(geometric_planning_for_rigid_body_in_3D src/geometric_planning_for_rigid_body_in_3D.cpp)

# Include directories for the executables
target_include_directories(geometric_planning_for_rigid_body_in_3D PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


# Compile features for the executables
target_compile_features(geometric_planning_for_rigid_body_in_3D PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Dependencies for the executables
ament_target_dependencies(geometric_planning_for_rigid_body_in_3D 
  ${dependencies}
  # "geometric_msgs"
)

install(TARGETS geometric_planning_for_rigid_body_in_3D
  DESTINATION lib/${PROJECT_NAME})

# Install executables Python
# install(PROGRAMS
# scripts/joint_position_controller.py
# DESTINATION lib/${PROJECT_NAME}
# )

# Install directories
# install(DIRECTORY launch 
#   DESTINATION share/${PROJECT_NAME}
# )

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
