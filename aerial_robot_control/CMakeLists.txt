cmake_minimum_required(VERSION 3.5)
project(aerial_robot_control)

# --- C++17 ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Eigen requires optimization to get good performance
# http://eigen.tuxfamily.org/index.php?title=FAQ#Optimization
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")

# --- Dependencies ---
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
find_package(aerial_robot_estimation REQUIRED)
find_package(aerial_robot_navigation REQUIRED)
find_package(aerial_robot_model REQUIRED)
find_package(aerial_robot_msgs REQUIRED)
find_package(spinal REQUIRED)
find_package(std_srvs REQUIRED)

set(ROS_DEPENDENCIES
  rclcpp
  pluginlib
  angles
  tf2
  aerial_robot_estimation
  aerial_robot_navigation
  aerial_robot_model
  aerial_robot_msgs
  spinal
  std_srvs
)

# --- Build control_utils library (CARE solver) ---
add_library(control_utils SHARED
  src/LQI/care.cpp
)

target_include_directories(control_utils
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(control_utils
  Eigen3::Eigen
)

# --- Build flight_control_plugin library ---
add_library(flight_control_pluginlib SHARED
  src/base/control_base.cpp
  src/PID/pose_pid_controller_base.cpp
  src/PID/pid.cpp
  src/PID/under_actuated_pid_controller.cpp
  src/PID/fully_actuated_pid_controller.cpp
  src/LQI/under_actuated_lqi_controller.cpp
  src/LQI/under_actuated_tilted_lqi_controller.cpp
)

ament_target_dependencies(flight_control_pluginlib
  ${ROS_DEPENDENCIES}
)

target_include_directories(flight_control_pluginlib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(flight_control_pluginlib
  control_utils
  Eigen3::Eigen
)

pluginlib_export_plugin_description_file(
  ${PROJECT_NAME}
  plugins/flight_control_plugins.xml
)

# --- Installation ---
install(
  TARGETS
    control_utils
    flight_control_pluginlib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY plugins
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/waypoint_pub_test.py
  scripts/circle_trajectory_following_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Export ---
ament_export_include_directories(include)
ament_export_libraries(
  control_utils
  flight_control_pluginlib
)
ament_export_dependencies(
  ${ROS_DEPENDENCIES}
  Eigen3
)

# --- Linters ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
