cmake_minimum_required(VERSION 3.5)

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

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_AUTOMOC ON)

set(smart_rviz_plugin_SRCS
  src/smart_recorder.cpp
)

set(smart_rviz_plugin_HDRS
  include/smart_rviz_plugin/smart_recorder.hpp
)

add_library(smart_rviz_plugin SHARED
  ${smart_rviz_plugin_SRCS}
  ${smart_rviz_plugin_HDRS}
)

target_include_directories(smart_rviz_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/smart_rviz_plugin>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(smart_rviz_plugin PUBLIC
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  rviz_rendering::rviz_rendering
)

target_compile_definitions(smart_rviz_plugin PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_ogre_vendor
  sensor_msgs
)

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

ament_package()