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
    add_compile_options(-Wall -Wextra -Wpedantic -Wreorder)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Threads REQUIRED)

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

# specify the path to search for the libraries
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/smartmicro/libs)

install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/smartmicro/libs/libsmart_access.so"
    "${CMAKE_CURRENT_SOURCE_DIR}/smartmicro/libs/libosal.so"
    "${CMAKE_CURRENT_SOURCE_DIR}/smartmicro/libs/libcom_lib.so"
    "${CMAKE_CURRENT_SOURCE_DIR}/smartmicro/libs/libbasev1.0.2_user_interface.so"
    "${CMAKE_CURRENT_SOURCE_DIR}/smartmicro/libs/libumrr11_t132_automotivev1.1.1_user_interface.so"
    "${CMAKE_CURRENT_SOURCE_DIR}/smartmicro/libs/libumrr96_t153_automotivev1.2.1_user_interface.so"
DESTINATION lib)

set(LIB_PATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CONFIG_FOLDER_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config")
set(CONFIG_FILE_PATH "${CONFIG_FOLDER_PATH}/smart_access_config.json")
set(HW_INVENTORY_FILE_PATH "${CONFIG_FOLDER_PATH}/hw_inventory.json")
set(ROUTING_TABLE_FILE_PATH "${CONFIG_FOLDER_PATH}/routing_table.json")
configure_file(
    "${PROJECT_SOURCE_DIR}/cmake/smart_access_config.json.in" "${CONFIG_FILE_PATH}" @ONLY)
configure_file(
    "${PROJECT_SOURCE_DIR}/cmake/config_path.hpp.in"
    "${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/config_path.hpp" @ONLY)

ament_auto_add_library(smartmicro_radar_node SHARED "src/smartmicro_radar_node.cpp")

target_include_directories(smartmicro_radar_node PUBLIC ${CMAKE_INSTALL_PREFIX}/include/
smartmicro/release/
smartmicro/release/umrr11_t132_automotive_v1_1_1
smartmicro/release/umrr96_t153_automotive_v1_2_1)

# link smart_access_libs to the node
target_link_libraries(smartmicro_radar_node 
    smart_access
    nlohmann_json::nlohmann_json
    com_lib
    osal
    umrr11_t132_automotivev1.1.1_user_interface
    umrr96_t153_automotivev1.2.1_user_interface)

rclcpp_components_register_node(smartmicro_radar_node
    PLUGIN "smartmicro::drivers::radar::SmartmicroRadarNode"
    EXECUTABLE smartmicro_radar_node_exe
)

if(BUILD_TESTING)
   
    find_package(launch_testing_ament_cmake)
    add_launch_test(test/radar_node_test.launch.py
        TIMEOUT "150"
    )
    
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_smartmicro_radar_node test/test_smartmicro_radar_node.cpp)
    target_link_libraries(test_smartmicro_radar_node smartmicro_radar_node)

endif()

ament_auto_package(INSTALL_TO_SHARE config launch param)
