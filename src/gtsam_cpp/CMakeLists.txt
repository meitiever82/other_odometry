cmake_minimum_required(VERSION 3.10)
project(leg_odom_gtsam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# GTSAM
set(GTSAM_DIR "$ENV{HOME}/Documents/local/gtsam-4.3/lib/cmake/GTSAM")
find_package(GTSAM REQUIRED HINTS ${GTSAM_DIR})

# Eigen (comes with GTSAM)
find_package(Eigen3 REQUIRED)

add_executable(leg_odom_gtsam leg_odom_gtsam.cpp)
target_include_directories(leg_odom_gtsam PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(leg_odom_gtsam
  gtsam
)
