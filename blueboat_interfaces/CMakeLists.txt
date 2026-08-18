cmake_minimum_required(VERSION 3.10)
project(blueboat_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RequestPath.srv"
  "msg/OmniscanProfile.msg"
  "msg/ProcessedSSSPing.msg"
  DEPENDENCIES std_msgs nav_msgs builtin_interfaces
)

ament_package()

