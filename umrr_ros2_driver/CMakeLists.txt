# Copyright 2021 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# The initial version of the code was developed by Apex.AI and
# was thereafter adapted and extended by smartmicro.

cmake_minimum_required(VERSION 3.11)
project(umrr_ros2_driver)
include(FetchContent)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Threads REQUIRED)

set(ENABLE_CJSON_UNINSTALL OFF CACHE BOOL "Enable creating uninstall target")

#TODO use the new git tag when released as the option for unsetting the uninstall is not available!!
FetchContent_Declare(
    cJSON
    GIT_REPOSITORY https://github.com/DaveGamble/cJSON.git
    GIT_TAG b9eff8b02afd0e2612b695fb8d67d56839b126ce
)

FetchContent_GetProperties(cJSON)
if(NOT cJSON_POPULATED)
    FetchContent_Populate(cJSON)
    add_subdirectory(${cjson_SOURCE_DIR} ${cjson_BINARY_DIR})
endif()

FetchContent_Declare(json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.10.4
)

FetchContent_GetProperties(json)
if(NOT json_POPULATED)
    FetchContent_Populate(json)
    add_subdirectory(${json_SOURCE_DIR} ${json_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

ament_auto_find_build_dependencies()
add_subdirectory(smartmicro)
# Note that we don't need to link the radar node to osal as the node will be dynamically linked
# through the smart_access library to every library specified in the lib folder in the config file.
ament_auto_add_library(smartmicro_radar_node SHARED "src/smartmicro_radar_node.cpp")
target_include_directories(smartmicro_radar_node PUBLIC ${CMAKE_INSTALL_PREFIX}/include/)
target_link_libraries(smartmicro_radar_node smart_access nlohmann_json::nlohmann_json)

rclcpp_components_register_node(smartmicro_radar_node
    PLUGIN "smartmicro::drivers::radar::SmartmicroRadarNode"
    EXECUTABLE smartmicro_radar_node_exe
)

if(BUILD_TESTING)
    find_package(launch_testing_ament_cmake)
    add_launch_test(test/radar_node_test.launch.py
        TIMEOUT "500"
    )

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_smartmicro_radar_node test/test_smartmicro_radar_node.cpp)
    add_dependencies(test_smartmicro_radar_node smartmicro_radar_node osal)
    target_link_libraries(test_smartmicro_radar_node smartmicro_radar_node osal)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch param pcap)