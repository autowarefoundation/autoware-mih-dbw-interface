#include <autoware_mih_dbw_interface/message_conversion.hpp>

namespace autoware_mih_dbw_interface
{

MIH_HighLevelControlCommand toMihDbwMsgHCC(const AW_AckermannControlCommand & aw_msg)
{
  MIH_HighLevelControlCommand mih_msg;
  mih_msg.stamp = aw_msg.stamp;
  mih_msg.velocity = aw_msg.longitudinal.speed;
  mih_msg.curvature = 0.0;  // TODO(horibe): calc curvature with steering angle and wheelbase
  mih_msg.torque = 0.0;     // not supported in autoware yet
  return mih_msg;
}

MIH_VehicleControlCommand toMihDbwMsgVCC(const AW_AckermannControlCommand & aw_msg)
{
  MIH_VehicleControlCommand mih_msg;
  mih_msg.stamp = aw_msg.stamp;
  mih_msg.velocity = aw_msg.longitudinal.speed;
  mih_msg.long_accel = aw_msg.longitudinal.acceleration;
  mih_msg.front_wheel_angle = aw_msg.lateral.steering_tire_angle;
  mih_msg.front_wheel_angle_rotation_rate = aw_msg.lateral.steering_tire_rotation_rate;
  mih_msg.front_wheel_torque = 0.0;              // not supported in autoware yet
  mih_msg.rear_wheel_angle = 0.0;                // not supported in autoware yet
  mih_msg.rear_wheel_angle_rotation_rate = 0.0;  // not supported in autoware yet
  mih_msg.rear_wheel_torque = 0.0;               // not supported in autoware yet
  return mih_msg;
}

MIH_VehicleOdometry toMihDbwMsg(const AW_Odometry & aw_msg)
{
  MIH_VehicleOdometry mih_msg;
  mih_msg.stamp = aw_msg.header.stamp;
  mih_msg.wheel_speeds.push_back(aw_msg.twist.twist.linear.x);  // not supported
  mih_msg.yaw_rate = aw_msg.twist.twist.angular.z;              // not supported in autoware yet
  mih_msg.velocity = aw_msg.twist.twist.linear.x;
  mih_msg.front_wheel_angle = 0.0;  // not supported
  mih_msg.rear_wheel_angle = 0.0;   // not supported
  return mih_msg;
}

AW_AckermannControlCommand toAutowareMsg(const MIH_HighLevelControlCommand & mih_msg)
{
  AW_AckermannControlCommand aw_msg;
  aw_msg.stamp = mih_msg.stamp;
  aw_msg.lateral.stamp = mih_msg.stamp;
  aw_msg.lateral.steering_tire_angle = 0.0;          // TODO(horibe): calc steer from curvature
  aw_msg.lateral.steering_tire_rotation_rate = 0.0;  // not supported
  aw_msg.longitudinal.stamp = mih_msg.stamp;
  aw_msg.longitudinal.speed = mih_msg.velocity;
  aw_msg.longitudinal.acceleration = 0.0;  // not supported
  aw_msg.longitudinal.jerk = 0.0;          // not supported
  return aw_msg;
}

AW_AckermannControlCommand toAutowareMsg(const MIH_VehicleControlCommand & mih_msg)
{
  AW_AckermannControlCommand aw_msg;
  aw_msg.stamp = mih_msg.stamp;
  aw_msg.lateral.stamp = mih_msg.stamp;
  aw_msg.lateral.steering_tire_angle = mih_msg.front_wheel_angle;
  aw_msg.lateral.steering_tire_rotation_rate = mih_msg.front_wheel_angle_rotation_rate;
  aw_msg.longitudinal.stamp = mih_msg.stamp;
  aw_msg.longitudinal.speed = mih_msg.velocity;
  aw_msg.longitudinal.acceleration = mih_msg.long_accel;
  aw_msg.longitudinal.jerk = 0.0;  // not supported
  return aw_msg;
}

AW_Odometry toAutowareMsg(const MIH_VehicleOdometry & mih_msg)
{
  AW_Odometry aw_msg;
  aw_msg.header.frame_id = "map";
  aw_msg.child_frame_id = "base_link";
  aw_msg.header.stamp = mih_msg.stamp;
  aw_msg.twist.twist.linear.x = mih_msg.velocity;
  aw_msg.twist.twist.angular.z = mih_msg.yaw_rate;
  return aw_msg;
}
AW_SteeringReport toAutowareSteerMsg(const MIH_VehicleOdometry & mih_msg)
{
  AW_SteeringReport aw_msg;
  aw_msg.stamp = mih_msg.stamp;
  aw_msg.steering_tire_angle = mih_msg.front_wheel_angle;
  return aw_msg;
}

AW_VelocityReport toAutowareVelocityMsg(const MIH_VehicleOdometry & mih_msg)
{
  AW_VelocityReport aw_msg;
  aw_msg.header.frame_id = "base_link";
  aw_msg.header.stamp = mih_msg.stamp;
  aw_msg.heading_rate = mih_msg.yaw_rate;
  aw_msg.lateral_velocity = 0.0;  // not supported
  aw_msg.longitudinal_velocity = mih_msg.velocity;
  return aw_msg;
}

}  // namespace autoware_mih_dbw_interface
