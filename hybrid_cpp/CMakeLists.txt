cmake_minimum_required(VERSION 3.10)
project(leg_odom_hybrid)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# GTSAM
set(GTSAM_DIR "$ENV{HOME}/Documents/local/gtsam-4.3/lib/cmake/GTSAM")
find_package(GTSAM REQUIRED HINTS ${GTSAM_DIR})
find_package(Eigen3 REQUIRED)

# KDL
find_package(orocos_kdl REQUIRED)

# === Offline evaluation (CSV input, no ROS2) ===
add_executable(leg_odom_hybrid leg_odom_hybrid.cpp)
target_include_directories(leg_odom_hybrid PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(leg_odom_hybrid gtsam)

# === KDL test ===
add_executable(test_kinematics test_kinematics.cpp)
target_include_directories(test_kinematics PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${orocos_kdl_INCLUDE_DIRS}
  /opt/ros/humble/include/kdl_parser
  /opt/ros/humble/include/urdf
  /opt/ros/humble/include/urdfdom
  /opt/ros/humble/include/urdfdom_headers
)
target_link_libraries(test_kinematics
  ${orocos_kdl_LIBRARIES}
  /opt/ros/humble/lib/libkdl_parser.so
)

# === ROS2 node (needs ament/colcon build) ===
# Build with: colcon build --packages-select leg_odom_hybrid
# For now, standalone build below (requires sourced ROS2):
find_package(rclcpp QUIET)
if(rclcpp_FOUND)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  add_executable(leg_odom_node leg_odom_node.cpp)
  target_include_directories(leg_odom_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GTSAM_INCLUDE_DIR}
    ${orocos_kdl_INCLUDE_DIRS}
    /opt/ros/humble/include/kdl_parser
    /opt/ros/humble/include/urdf
    /opt/ros/humble/include/urdfdom
    /opt/ros/humble/include/urdfdom_headers
  )
  target_link_libraries(leg_odom_node
    gtsam
    ${orocos_kdl_LIBRARIES}
    /opt/ros/humble/lib/libkdl_parser.so
  )
  ament_target_dependencies(leg_odom_node
    rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros builtin_interfaces
  )
  message(STATUS "ROS2 found - building leg_odom_node")
else()
  message(STATUS "ROS2 not found - skipping leg_odom_node (use colcon build)")
endif()
