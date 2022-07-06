cmake_minimum_required(VERSION 3.5)
project(mih_dbw_api_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(
  mih_dbw_api_msgs
  "msg/HighLevelControlCommand.msg"
  "msg/VehicleControlCommand.msg"
  "msg/VehicleOdometry.msg"
  DEPENDENCIES builtin_interfaces
)

ament_auto_package()
