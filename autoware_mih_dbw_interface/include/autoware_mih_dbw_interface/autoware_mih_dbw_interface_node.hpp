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

#ifndef AUTOWARE_MIH_DBW_INTERFACE__AUTOWARE_MIH_DBW_INTERFACE_NODE_HPP_
#define AUTOWARE_MIH_DBW_INTERFACE__AUTOWARE_MIH_DBW_INTERFACE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <autoware_mih_dbw_interface/message_conversion.hpp>

#include <memory>
#include <vector>

namespace autoware_mih_dbw_interface
{

class AutowareMihDbwInterfaceNode : public rclcpp::Node
{
public:
  explicit AutowareMihDbwInterfaceNode(const rclcpp::NodeOptions & options);
  ~AutowareMihDbwInterfaceNode() = default;

private:
  /** control command from Autoware */
  rclcpp::Subscription<AW_AckermannControlCommand>::SharedPtr sub_control_cmd_;

  /** steering to Autoware */
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr pub_steering_;

  /** velocity to Autoware */
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_velocity_;

  /** odometry from MIH DBW IF */
  rclcpp::Subscription<MIH_VehicleOdometry>::SharedPtr sub_odometry_;

  /** control command to MIH DBW IF (high-level command) */
  rclcpp::Publisher<MIH_HighLevelControlCommand>::SharedPtr pub_high_control_cmd_;

  /** control command to MIH DBW IF */
  rclcpp::Publisher<MIH_VehicleControlCommand>::SharedPtr pub_control_cmd_;

  /**
   * receive MIH DBW API odometry message and publish associated Autoware odometry
   */
  void onMihOdometry(const MIH_VehicleOdometry::ConstSharedPtr mih_odometry);

  /**
   * receive Autoware control message and publish associated MIH DBW API control command
   */
  void onAutowareControlCommand(const AW_AckermannControlCommand::ConstSharedPtr aw_control);
};

}  // namespace autoware_mih_dbw_interface

#endif  // AUTOWARE_MIH_DBW_INTERFACE__AUTOWARE_MIH_DBW_INTERFACE_NODE_HPP_
