cmake_minimum_required(VERSION 3.11)

project(smart_rviz_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

find_package(ament_cmake_auto REQUIRED)
find_package(umrr_ros2_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Test)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

set(CMAKE_AUTOMOC ON)

set(smart_rviz_plugin_SRCS
  src/smart_recorder.cpp
  src/smart_services.cpp
  src/smart_download.cpp
  src/smart_status.cpp
)

set(smart_rviz_plugin_HDRS
  include/smart_rviz_plugin/smart_recorder.hpp
  include/smart_rviz_plugin/smart_services.hpp
  include/smart_rviz_plugin/smart_download.hpp
  include/smart_rviz_plugin/smart_status.hpp
)

add_library(smart_rviz_plugin SHARED
  ${smart_rviz_plugin_SRCS}
  ${smart_rviz_plugin_HDRS}
)

set(dependencies
  umrr_ros2_msgs
  pluginlib
  Qt5
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  std_msgs
  OpenCV
  cv_bridge
)

ament_target_dependencies(smart_rviz_plugin
  ${dependencies}
)

target_include_directories(smart_rviz_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/smart_rviz_plugin>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


target_link_libraries(smart_rviz_plugin
  rviz_common::rviz_common
)


target_compile_definitions(smart_rviz_plugin PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS smart_rviz_plugin
  EXPORT smart_rviz_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_targets(smart_rviz_plugin HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_ogre_vendor
  sensor_msgs
  umrr_ros2_msgs
)

ament_package()
