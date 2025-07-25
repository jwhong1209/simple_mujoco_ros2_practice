cmake_minimum_required(VERSION 3.10)
project(simple_mujoco_controller)

# --- Set module path and include common project configuration
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)
include(ProjectConfig)

# --- set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Find and load build settings from external packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

# --- Create executable file of node
add_executable(${PROJECT_NAME} 
  src/simple_mujoco_controller.cpp
  src/simple_mujoco_controller_node.cpp

  ${PROJECT_COMMON_DIR}/src/robot/DoublePendulumModel.cpp
  ${PROJECT_COMMON_DIR}/src/motion/TrajectoryGenerator.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
)

target_include_directories(${PROJECT_NAME}
PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/simple_mujoco_controller

  ${PROJECT_COMMON_DIR}/include/robot
  ${PROJECT_COMMON_DIR}/include/motion
  ${PROJECT_COMMON_DIR}/include/utilities
)

ament_package()