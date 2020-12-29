# Description :

This controller drives two ez-Wheel Gen2 wheels as a differential-drive robot.

# Topics Published
+ `~odom` of type __`nav_msgs::Odometry`__ with x, y, theta and timestamp
+ `~joint_state` of type __`sensor_msgs::JointState`__ with the motor name, position (m), velocity (m/s) and timestamp

# Topics Subscribed
+ `~set_speed` of type __`geometry_msgs::Point`__ where x = left motor speed and y = right motor speed in rad/s

# Node parameters :
+ `baseline_m` : (__`double`__) Gap between the 2 wheels (m)
+ `pub_freq_hz` : (__`int`__) Publication frequency of odometry (Hz)
+ `watchdog_receive_ms` : (__`int`__) Watchdog delay before stopping the wheels if no command is received
+ `left_dbus_smc_service` : (__`string`__) dbus smc service corresponding to the left wheel
+ `right_dbus_smc_service` : (__`string`__) dbus smc service corresponding to the right wheel
