cmake_minimum_required(VERSION 3.10)
project(simple_mujoco_gui_plugin)

# ----- Set C++ standard & compile options ---------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set_source_files_properties(src/qcustomplot.cpp PROPERTIES
  COMPILE_FLAGS "-Wno-deprecated-declarations"
)

# ----- Find dependent packages --------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Qt5 COMPONENTS REQUIRED
  PrintSupport
  Widgets 
  Core 
  Gui
)

# ----- Define headers / sources / uis to create executable file of node ---------------------------
set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/simple_mujoco_gui_plugin.hpp
  include/qcustomplot.h
)

set(${PROJECT_NAME}_SRCS
  src/${PROJECT_NAME}/simple_mujoco_gui_plugin.cpp
  src/qcustomplot.cpp
)

set(${PROJECT_NAME}_UIS
  resource/simple_mujoco_gui_plugin.ui
)

# ----- Qt5 Auto MOC/UIC ---------------------------------------------------------------------------
qt5_add_resources(RESOURCES "${CMAKE_CURRENT_SOURCE_DIR}/resource/resources.qrc")
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})

# ----- Build : add & link libraries ---------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_MOCS}
  ${${PROJECT_NAME}_UIS_H}
  ${RESOURCES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib

  std_msgs
  sensor_msgs
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  include
  ${rclcpp_INCLUDE_DIRS}
  ${qt_gui_cpp_INCLUDE_DIRS}
  ${rqt_gui_cpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${qt_gui_cpp_LIBRARIES}
  ${rqt_gui_cpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  
  Qt5::PrintSupport
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
)

# ----- Installation -------------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(PROGRAMS scripts/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

# ----- Macro for ament package --------------------------------------------------------------------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
