cmake_minimum_required(VERSION 3.16)
project(dynamic_uav_topo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(uav_manager
  src/node_uav_manager/uav_manager.cpp
)
target_include_directories(uav_manager PUBLIC include)
target_compile_options(uav_manager PRIVATE -Wall -Wextra -Wpedantic)

add_executable(node_uav_manager src/node_uav_manager/main.cpp)
target_link_libraries(node_uav_manager PRIVATE uav_manager)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_uav_manager tests/test_uav_manager.cpp)
  target_link_libraries(test_uav_manager PRIVATE uav_manager)
  add_test(NAME test_uav_manager COMMAND test_uav_manager)
endif()
