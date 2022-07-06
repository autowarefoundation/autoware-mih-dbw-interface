# Timestamp value in nanosecond or the unit defined by the system.
builtin_interfaces/Time stamp

# Actual vehicle speed in m/s
float32 velocity

# Actual front wheel turning radius.
float32 front_wheel_angle

# Actual rear wheel turning radius.
float32 rear_wheel_angle

# each tire rotation speed
float32[] wheel_speeds

# calculated by wheel speed difference or ackermann vehicle model
float32 yaw_rate
