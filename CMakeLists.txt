cmake_minimum_required(VERSION 3.20)
project(leg_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

# GTSAM (system aarch64 install at /usr/local)
find_package(GTSAM REQUIRED)

# KDL parser (use ament find)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)

# === ROS2 Node ===
add_executable(leg_odom_node src/leg_odom_node.cpp)
target_include_directories(leg_odom_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GTSAM_INCLUDE_DIR}
  ${orocos_kdl_INCLUDE_DIRS}
)
target_link_libraries(leg_odom_node
  gtsam
  ${orocos_kdl_LIBRARIES}
)
ament_target_dependencies(leg_odom_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros builtin_interfaces
  kdl_parser urdf
)

# === Simple translation-only node (IMU for R, leg FK for p) ===
add_executable(simple_leg_odom_node src/simple_leg_odom_node.cpp)
target_include_directories(simple_leg_odom_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${orocos_kdl_INCLUDE_DIRS}
)
target_link_libraries(simple_leg_odom_node
  ${orocos_kdl_LIBRARIES}
)
ament_target_dependencies(simple_leg_odom_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros builtin_interfaces
  kdl_parser urdf
)

# === Stage 1: FK-only node (gyro yaw + J·q̇ velocity, no Kalman) ===
add_executable(fk_only_node src/fk_only_node.cpp)
target_include_directories(fk_only_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${orocos_kdl_INCLUDE_DIRS}
)
target_link_libraries(fk_only_node
  ${orocos_kdl_LIBRARIES}
)
ament_target_dependencies(fk_only_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros builtin_interfaces
  kdl_parser urdf
)

# === Offline evaluation (CSV, no ROS2 deps at runtime) ===
add_executable(test_leg_odom src/test_leg_odom.cpp)
target_include_directories(test_leg_odom PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(test_leg_odom gtsam)

# === KDL test ===
add_executable(test_kinematics src/test_kinematics.cpp)
target_include_directories(test_kinematics PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${orocos_kdl_INCLUDE_DIRS}
)
target_link_libraries(test_kinematics
  ${orocos_kdl_LIBRARIES}
)
ament_target_dependencies(test_kinematics
  kdl_parser urdf
)

# Install
install(TARGETS leg_odom_node simple_leg_odom_node fk_only_node test_leg_odom test_kinematics
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/generate_sim_data.py
  scripts/evaluate_sim.py
  scripts/evaluate_hybrid.py
  scripts/bag_to_csv.py
  scripts/dump_leg_odom_csv.py
  scripts/eval_drift.py
  scripts/run_bag_test.sh
  scripts/diag_sim2real.py
  DESTINATION lib/${PROJECT_NAME}
)

# Renamed install: launch uses executable='joint_state_remapper' (no .py)
install(PROGRAMS scripts/joint_state_remapper.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_state_remapper
)

ament_package()
