cmake_minimum_required(VERSION 3.8)
project(swarm_uav_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_interfaces REQUIRED)

add_library(uav_manager_core
  src/uav_manager.cpp
)
target_include_directories(uav_manager_core PUBLIC include)
target_compile_features(uav_manager_core PUBLIC cxx_std_20)

add_executable(swarm_uav_manager_node
  src/swarm_uav_manager_node.cpp
)
target_compile_features(swarm_uav_manager_node PUBLIC cxx_std_20)
target_include_directories(swarm_uav_manager_node PUBLIC include)
target_link_libraries(swarm_uav_manager_node uav_manager_core)
ament_target_dependencies(swarm_uav_manager_node rclcpp swarm_interfaces)

install(TARGETS
  uav_manager_core
  swarm_uav_manager_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
