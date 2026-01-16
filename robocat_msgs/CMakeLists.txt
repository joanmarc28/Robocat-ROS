cmake_minimum_required(VERSION 3.8)
project(robocat_msgs)

find_package(ament_cmake REQUIRED)
ament_package_xml()

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Telemetry.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
