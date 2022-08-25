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

#include <autoware_mih_dbw_interface/autoware_mih_dbw_interface_node.hpp>

namespace autoware_mih_dbw_interface
{
AutowareMihDbwInterfaceNode::AutowareMihDbwInterfaceNode(const rclcpp::NodeOptions & options)
: Node("autoware_mih_dbw_interface_node", options)
{
  using std::placeholders::_1;

  // Autoware
  sub_control_cmd_ = create_subscription<AW_AckermannControlCommand>(
    "~/input/control_command", 1,
    std::bind(&AutowareMihDbwInterfaceNode::onAutowareControlCommand, this, _1));
  pub_steering_ =
    create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("~/output/steering", 1);
  pub_velocity_ =
    create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("~/output/velocity", 1);

  // MIH
  sub_odometry_ = create_subscription<MIH_VehicleOdometry>(
    "~/input/odometry", 1, std::bind(&AutowareMihDbwInterfaceNode::onMihOdometry, this, _1));
  pub_high_control_cmd_ =
    create_publisher<MIH_HighLevelControlCommand>("~/output/high_level_control_command", 1);
  pub_control_cmd_ =
    create_publisher<MIH_VehicleControlCommand>("~/output/vehicle_control_command", 1);
}

void AutowareMihDbwInterfaceNode::onMihOdometry(
  const MIH_VehicleOdometry::ConstSharedPtr mih_odometry)
{
  const auto aw_steering_rpt = toAutowareSteerMsg(*mih_odometry);
  pub_steering_->publish(aw_steering_rpt);

  const auto aw_velocity_rpt = toAutowareVelocityMsg(*mih_odometry);
  pub_velocity_->publish(aw_velocity_rpt);
}

void AutowareMihDbwInterfaceNode::onAutowareControlCommand(
  const AW_AckermannControlCommand::ConstSharedPtr aw_control)
{
  const auto mih_high_control_cmd = toMihDbwMsgHCC(*aw_control);
  pub_high_control_cmd_->publish(mih_high_control_cmd);

  const auto mih_control_cmd = toMihDbwMsgVCC(*aw_control);
  pub_control_cmd_->publish(mih_control_cmd);
}

}  // namespace autoware_mih_dbw_interface

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_mih_dbw_interface::AutowareMihDbwInterfaceNode)
