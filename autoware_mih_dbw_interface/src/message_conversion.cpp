#include <autoware_mih_dbw_interface/message_conversion.hpp>

namespace autoware_mih_dbw_interface
{

  MIH_HighLevelControlCommand toMihDbwMessage(const Autoware_HighLevelControlCommand &aw_msg)
  {
    MIH_HighLevelControlCommand mih_msg;
    mih_msg.stamp = aw_msg.stamp;
    mih_msg.velocity = aw_msg.velocity_mps;
    mih_msg.curvature = aw_msg.curvature;
    mih_msg.torque = 0.0; // not supported in autoware yet
    return mih_msg;
  }

  MIH_VehicleControlCommand toMihDbwMessage(const Autoware_VehicleControlCommand &aw_msg)
  {
    MIH_VehicleControlCommand mih_msg;
    mih_msg.stamp = aw_msg.stamp;
    mih_msg.velocity = aw_msg.velocity_mps;
    mih_msg.long_accel = aw_msg.long_accel_mps2;
    mih_msg.front_wheel_angle = aw_msg.front_wheel_angle_rad;
    mih_msg.front_wheel_angle_rotation_rate = 0.0; // not supported in autoware yet
    mih_msg.front_wheel_torque = 0.0;              // not supported in autoware yet
    mih_msg.rear_wheel_angle = aw_msg.rear_wheel_angle_rad;
    mih_msg.rear_wheel_angle_rotation_rate = 0.0; // not supported in autoware yet
    mih_msg.rear_wheel_torque = 0.0;              // not supported in autoware yet
    return mih_msg;
  }

  MIH_VehicleOdometry toMihDbwMessage(const Autoware_VehicleOdometry &aw_msg)
  {
    MIH_VehicleOdometry mih_msg;
    mih_msg.stamp = aw_msg.stamp;
    mih_msg.wheel_speeds.push_back(aw_msg.velocity_mps); // each wheel speeds are not supported
    mih_msg.yaw_rate = 0.0;                              // not supported in autoware yet
    mih_msg.velocity = aw_msg.velocity_mps;
    mih_msg.front_wheel_angle = aw_msg.front_wheel_angle_rad;
    mih_msg.rear_wheel_angle = aw_msg.rear_wheel_angle_rad;
    return mih_msg;
  }

  Autoware_HighLevelControlCommand toAutowareMessage(const MIH_HighLevelControlCommand &mih_msg)
  {
    Autoware_HighLevelControlCommand aw_msg;
    aw_msg.stamp = mih_msg.stamp;
    aw_msg.curvature = mih_msg.curvature;
    aw_msg.velocity_mps = mih_msg.velocity;

    return aw_msg;

  }

  Autoware_VehicleControlCommand toAutowareMessage(const MIH_VehicleControlCommand &mih_msg)
  {
    Autoware_VehicleControlCommand aw_msg;
    aw_msg.stamp = mih_msg.stamp;
    aw_msg.velocity_mps = mih_msg.velocity;
    aw_msg.front_wheel_angle_rad = mih_msg.front_wheel_angle;
    aw_msg.long_accel_mps2 = mih_msg.long_accel;
    aw_msg.rear_wheel_angle_rad = mih_msg.rear_wheel_angle;
    return aw_msg;
  }

  Autoware_VehicleOdometry toAutowareMessage(const MIH_VehicleOdometry &mih_msg)
  {
    Autoware_VehicleOdometry aw_msg;
    aw_msg.stamp = mih_msg.stamp;
    aw_msg.velocity_mps = mih_msg.velocity;
    aw_msg.front_wheel_angle_rad = mih_msg.front_wheel_angle;
    aw_msg.rear_wheel_angle_rad = mih_msg.rear_wheel_angle;
    return aw_msg;
  }

}
