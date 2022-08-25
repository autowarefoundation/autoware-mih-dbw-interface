// Copyright 2022 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_MIH_DBW_INTERFACE__MESSAGE_CONVERSION_HPP_
#define AUTOWARE_MIH_DBW_INTERFACE__MESSAGE_CONVERSION_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <mih_dbw_api_msgs/msg/vehicle_control_command.hpp>
#include <mih_dbw_api_msgs/msg/high_level_control_command.hpp>
#include <mih_dbw_api_msgs/msg/vehicle_odometry.hpp>

namespace autoware_mih_dbw_interface
{
using AW_AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
using AW_Odometry = nav_msgs::msg::Odometry;
using AW_SteeringReport = autoware_auto_vehicle_msgs::msg::SteeringReport;
using AW_VelocityReport = autoware_auto_vehicle_msgs::msg::VelocityReport;

using MIH_HighLevelControlCommand = mih_dbw_api_msgs::msg::HighLevelControlCommand;
using MIH_VehicleControlCommand = mih_dbw_api_msgs::msg::VehicleControlCommand;
using MIH_VehicleOdometry = mih_dbw_api_msgs::msg::VehicleOdometry;

MIH_HighLevelControlCommand toMihDbwMsgHCC(const AW_AckermannControlCommand & aw_msg);
MIH_VehicleControlCommand toMihDbwMsgVCC(const AW_AckermannControlCommand & aw_msg);
MIH_VehicleOdometry toMihDbwMsg(const AW_Odometry & aw_msg);

AW_AckermannControlCommand toAutowareMsg(const MIH_HighLevelControlCommand & mih_msg);
AW_AckermannControlCommand toAutowareMsg(const MIH_VehicleControlCommand & mih_msg);
AW_Odometry toAutowareMsg(const MIH_VehicleOdometry & mih_msg);
AW_SteeringReport toAutowareSteerMsg(const MIH_VehicleOdometry & mih_msg);
AW_VelocityReport toAutowareVelocityMsg(const MIH_VehicleOdometry & mih_msg);

}

#endif  // AUTOWARE_MIH_DBW_INTERFACE__MESSAGE_CONVERSION_HPP_
