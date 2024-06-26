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
    add_compile_options(-Wall -Wextra -Wnarrowing -Wpedantic -Wno-unused-parameter -g)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Threads REQUIRED)
find_package(visualization_msgs REQUIRED)

fetchcontent_declare(json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.10.5
)

fetchcontent_getproperties(json)
if(NOT json_POPULATED)
    fetchcontent_populate(json)
    add_subdirectory(${json_SOURCE_DIR} ${json_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

ament_auto_find_build_dependencies()

set(SMARTMICRO_LIB_DIR "lib-linux-x86_64-gcc_9")

# specify the path to search for the libraries
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR})

install(FILES
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libsmart_access.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libosal.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libcom_lib.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libbasev1.0.2_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr11_t132_automotivev1.1.2_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr96_t153_automotivev1.2.2_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_automotivev1.1.1_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_automotivev2.0.0_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_automotivev2.1.1_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_automotivev2.2.1_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_automotivev2.4.1_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9d_t152_automotivev1.0.3_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9d_t152_automotivev1.2.2_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9d_t152_automotivev1.4.1_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrra4_automotivev1.0.1_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrra4_automotivev1.2.1_user_interface.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/${SMARTMICRO_LIB_DIR}/libumrr9f_t169_msev1.0.0_user_interface.so"
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

ament_auto_add_library(smartmicro_radar_node SHARED 
    "src/smartmicro_radar_node.cpp"
    "src/UpdateService.cpp"
    )

target_include_directories(smartmicro_radar_node PUBLIC ${CMAKE_INSTALL_PREFIX}/include/
smartmicro/include/
smartmicro/include/umrr11_t132_automotive_v1_1_2
smartmicro/include/umrr96_t153_automotive_v1_2_2
smartmicro/include/umrr9f_t169_automotive_v1_1_1
smartmicro/include/umrr9f_t169_automotive_v2_0_0
smartmicro/include/umrr9f_t169_automotive_v2_1_1
smartmicro/include/umrr9f_t169_automotive_v2_2_1
smartmicro/include/umrr9f_t169_automotive_v2_4_1
smartmicro/include/umrr9d_t152_automotive_v1_0_3
smartmicro/include/umrr9d_t152_automotive_v1_2_2
smartmicro/include/umrr9d_t152_automotive_v1_4_1
smartmicro/include/umrra4_automotive_v1_0_1
smartmicro/include/umrra4_automotive_v1_2_1
smartmicro/include/umrr9f_t169_mse_v1_0_0)

# link smart_access_lib-linux-x86_64-gcc_9 to the node
target_link_libraries(smartmicro_radar_node
    smart_access
    nlohmann_json::nlohmann_json
    com_lib
    osal
    umrr11_t132_automotivev1.1.2_user_interface
    umrr96_t153_automotivev1.2.2_user_interface
    umrr9f_t169_automotivev1.1.1_user_interface
    umrr9f_t169_automotivev2.0.0_user_interface
    umrr9f_t169_automotivev2.1.1_user_interface
    umrr9f_t169_automotivev2.2.1_user_interface
    umrr9f_t169_automotivev2.4.1_user_interface
    umrr9d_t152_automotivev1.0.3_user_interface
    umrr9d_t152_automotivev1.2.2_user_interface
    umrr9d_t152_automotivev1.4.1_user_interface
    umrra4_automotivev1.0.1_user_interface
    umrra4_automotivev1.2.1_user_interface
    umrr9f_t169_msev1.0.0_user_interface)

rclcpp_components_register_node(smartmicro_radar_node
    PLUGIN "smartmicro::drivers::radar::SmartmicroRadarNode"
    EXECUTABLE smartmicro_radar_node_exe
)

if(BUILD_TESTING)
    find_package(launch_testing_ament_cmake)
    add_launch_test(test/radar_node_test.launch.py TIMEOUT "100")
       
    # Not applying ros2 linters on external sources
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
        ament_cmake_copyright
        ament_cmake_lint_cmake
        ament_cmake_uncrustify
        ament_cmake_cpplint
        ament_cmake_flake8
        ament_cmake_cppcheck
        ament_cmake_pep257
    )

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    ament_copyright("src/smartmicro_radar_node.cpp")
    ament_cpplint("src/smartmicro_radar_node.cpp")
    ament_cppcheck("src/smartmicro_radar_node.cpp")

endif()

ament_auto_package(INSTALL_TO_SHARE config launch param)
