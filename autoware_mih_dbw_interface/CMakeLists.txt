cmake_minimum_required(VERSION 3.5)
project(autoware_mih_dbw_interface)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(autoware_mih_dbw_interface_node_lib SHARED
  src/message_conversion.cpp
  src/autoware_mih_dbw_interface_node.cpp
)

rclcpp_components_register_node(autoware_mih_dbw_interface_node_lib
  PLUGIN "autoware_mih_dbw_interface::AutowareMihDbwInterfaceNode"
  EXECUTABLE autoware_mih_dbw_interface_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
