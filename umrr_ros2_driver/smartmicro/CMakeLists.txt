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
# The initial version of the code was developed by Apex.AI and
# was thereafter adapted and extended by smartmicro.

set(CMAKE_SYSTEM_NAME Linux)

include_directories("${cjson_SOURCE_DIR}")

install(FILES
    "${PROJECT_SOURCE_DIR}/smartmicro/lib/libsmart_access.so"
    "${PROJECT_SOURCE_DIR}/smartmicro/lib/libport_based_serial.so"
    DESTINATION lib)

add_library(smart_access SHARED IMPORTED GLOBAL)
set_target_properties(smart_access PROPERTIES
    IMPORTED_LOCATION "${PROJECT_SOURCE_DIR}/smartmicro/lib/libsmart_access.so"
    INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/smartmicro/smart_access_release")

set(LIB_PATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CONFIG_FOLDER_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config")
set(CONFIG_FILE_PATH "${CONFIG_FOLDER_PATH}/smart_access_config.json")
set(HW_INVENTORY_FILE_PATH "${CONFIG_FOLDER_PATH}/hw_inventory.json")
set(ROUTING_TABLE_FILE_PATH "${CONFIG_FOLDER_PATH}/routing_table.json")
set(USER_INTERFACE_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/user_interface")
configure_file(
    "${PROJECT_SOURCE_DIR}/cmake/smart_access_config.json.in" "${CONFIG_FILE_PATH}" @ONLY)
configure_file(
    "${PROJECT_SOURCE_DIR}/cmake/config_path.hpp.in"
    "${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/config_path.hpp" @ONLY)

add_library(osal SHARED
    osal/src/OSALImpl.cpp
    osal/src/HwInventory.cpp
    osal/src/can/CanIface.cpp
    osal/src/rs485/Rs485Iface.cpp
    osal/src/ethernet/MulticastIface.cpp
    osal/src/ethernet/UnicastIface.cpp
    osal/src/ethernet/UdpIface.cpp
    osal/src/utils/Utils.cpp)
target_include_directories(osal PUBLIC
    osal/include
    osal/include/can
    osal/include/utils
    ${cjson_SOURCE_DIR}
    )
target_link_libraries(osal cjson smart_access)
install(TARGETS osal DESTINATION lib)