project(SensorSimulator)

cmake_minimum_required(VERSION 3.13)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m64 -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -Wimplicit-fallthrough=0 -m64 -fPIC")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -m64 -fPIC")

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/${SMART_ACCESS_LIB_FOLDER})

add_executable(simulator
               src/simulator.cpp)

target_include_directories(simulator PUBLIC
                           ../release
                           release
                           release/umrr11_t132_automotive_v1_1_1
                           )

target_link_libraries(simulator
                      com_lib
                      osal
                      umrr11_t132_automotivev1.1.1_user_interface
                      dl)

install(TARGETS simulator DESTINATION bin)
