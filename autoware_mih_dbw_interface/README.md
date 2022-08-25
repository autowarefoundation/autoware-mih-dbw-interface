# autoware-mih-dbw-interface-node

This package converts the message between Autoware and MIH-DBW-IF.

## Interface

### input

- /control/command/control_cmd [`autoware_auto_control_msgs/msg/AckermannControlCommand`] : control command from Autoware
- /odometry_from_mih [`mih_dbw_api_msgs/msg/VehicleOdometry`] : odometry from MIH-DBW-IF


### output

- /vehicle/status/steering_status [`autoware_auto_vehicle_msgs/msg/SteeringReport`] : steering report to Autoware
- /vehicle/status/velocity_status [`autoware_auto_vehicle_msgs/msg/VelocityReport`] : velocity report to Autoware
- /vehicle_control_command_to_mih [`mih_dbw_api_msgs/msg/VehicleControlCommand`] : control command to MIH-DBW-IF
- /high_level_control_command_to_mih [`mih_dbw_api_msgs/msg/HighLevelControlCommand`] : high level control command to MIH-DBW-IF
