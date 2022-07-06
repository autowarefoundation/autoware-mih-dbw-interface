#ifndef AUTOWARE_MIH_DBW_INTERFACE__MESSAGE_CONVERSION_HPP_
#define AUTOWARE_MIH_DBW_INTERFACE__MESSAGE_CONVERSION_HPP_

#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/high_level_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_odometry.hpp>

#include <mih_dbw_api_msgs/msg/vehicle_control_command.hpp>
#include <mih_dbw_api_msgs/msg/high_level_control_command.hpp>
#include <mih_dbw_api_msgs/msg/vehicle_odometry.hpp>

namespace autoware_mih_dbw_interface
{

using Autoware_HighLevelControlCommand = autoware_auto_msgs::msg::HighLevelControlCommand;
using Autoware_VehicleControlCommand = autoware_auto_msgs::msg::VehicleControlCommand;
using Autoware_VehicleOdometry = autoware_auto_msgs::msg::VehicleOdometry;

using MIH_HighLevelControlCommand = mih_dbw_api_msgs::msg::HighLevelControlCommand;
using MIH_VehicleControlCommand = mih_dbw_api_msgs::msg::VehicleControlCommand;
using MIH_VehicleOdometry = mih_dbw_api_msgs::msg::VehicleOdometry;

MIH_HighLevelControlCommand toMihDbwMessage(const Autoware_HighLevelControlCommand & aw_msg);
MIH_VehicleControlCommand toMihDbwMessage(const Autoware_VehicleControlCommand & aw_msg);
MIH_VehicleOdometry toMihDbwMessage(const Autoware_VehicleOdometry & aw_msg);

Autoware_HighLevelControlCommand toAutowareMessage(const MIH_HighLevelControlCommand & mih_msg);
Autoware_VehicleControlCommand toAutowareMessage(const MIH_VehicleControlCommand & mih_msg);
Autoware_VehicleOdometry toAutowareMessage(const MIH_VehicleOdometry & mih_msg);

}

#endif  // AUTOWARE_MIH_DBW_INTERFACE__MESSAGE_CONVERSION_HPP_
