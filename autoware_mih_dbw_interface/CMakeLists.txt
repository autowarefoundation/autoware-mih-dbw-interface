cmake_minimum_required(VERSION 3.5)
project(autoware_mih_dbw_interface)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(autoware_mih_message_convertion SHARED
  src/message_conversion.cpp
)

ament_auto_package()
