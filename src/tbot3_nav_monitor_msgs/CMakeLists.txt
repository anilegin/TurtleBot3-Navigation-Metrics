cmake_minimum_required(VERSION 3.8)
project(tbot3_nav_monitor_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavigationMetrics.msg"
)

ament_package()