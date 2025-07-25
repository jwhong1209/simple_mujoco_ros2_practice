# --- Set minimum required version of cmake, project name and compile options
cmake_minimum_required(VERSION 3.10)
project(simple_mujoco_ros2_interface)

# --- Set repo's absolute directory
add_definitions(-DPROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")

# --- set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- set compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wpedantic)
endif()

# --- Find and load build settings from external packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- Create executable file of node
file(GLOB_RECURSE SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc

)

add_executable(${PROJECT_NAME} ${SOURCES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
)

target_include_directories(${PROJECT_NAME}
PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/simple_mujoco_ros2_interface
)

target_link_libraries(${PROJECT_NAME}
  mujoco
  glfw
)

ament_package()