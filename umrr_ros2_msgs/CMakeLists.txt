cmake_minimum_required(VERSION 3.11)
project(umrr_ros2_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/SetMode.srv"
    "srv/SetIp.srv"
    "srv/SendCommand.srv"
    "srv/FirmwareDownload.srv"
    "msg/CanTargetHeader.msg"
    "msg/PortTargetHeader.msg"
    "msg/PortObjectHeader.msg"
    "msg/CanObjectHeader.msg"
    DEPENDENCIES
      "geometry_msgs"
      "sensor_msgs"
      "std_msgs"
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
