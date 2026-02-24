cmake_minimum_required(VERSION 3.8)
project(swarm_topology_analyzer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_interfaces REQUIRED)

add_executable(swarm_topology_analyzer_node
  src/swarm_topology_analyzer_node.cpp
)
target_compile_features(swarm_topology_analyzer_node PUBLIC cxx_std_20)
ament_target_dependencies(swarm_topology_analyzer_node rclcpp swarm_interfaces)

add_executable(swarm_semantic_net_node
  src/swarm_semantic_net_node.cpp
)
target_compile_features(swarm_semantic_net_node PUBLIC cxx_std_20)
ament_target_dependencies(swarm_semantic_net_node rclcpp swarm_interfaces)

install(TARGETS
  swarm_topology_analyzer_node
  swarm_semantic_net_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
