cmake_minimum_required(VERSION 3.8)
project(swarm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneState.msg"
  "msg/SwarmState.msg"
  "msg/CommanderDrone.msg"
  "msg/CommanderState.msg"
  "srv/GoTo.srv"
  "srv/Formation.srv"
  "srv/SwarmCommand.srv"
  "srv/InjectFault.srv"
  "srv/SetMissionStatus.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
