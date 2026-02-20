cmake_minimum_required(VERSION 3.8)
project(swarm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Hotspot.msg"
  "msg/FireState.msg"
  "msg/LinkState.msg"
  "msg/MissionTarget.msg"
  "msg/MissionPlan.msg"
  "msg/UavState.msg"
  "msg/SwarmState.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
