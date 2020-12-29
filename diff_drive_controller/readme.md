# Description :

This driver manages two ez-Wheels to drive a differential wheeled robot.

# Topics Published
+ `~odom` of type __nav_msgs::Odometry__ with x, y, theta and timestamp
+ `~joint_state` of type __sensor_msgs::JointState__ with the motor name, position (m), velocity (m/s) and timestamp

# Topics Subscribed
+ `~set_speed` of type __geometry_msgs::Point__ where x = left motor speed and y = right motor speed in rad/s

# Node parameters :
+ `baseline_m` : (__double__) Gap between the 2 wheels (m)
+ `pub_freq_hz` : (__int__) Publication frequency of odometry (Hz)
+ `watchdog_receive_ms` : (__int__) Watchdog delay before stopping the wheels if no command is received
+ `left_dbus_smc_service` : (__string__) dbus smc service corresponding to the left wheel
+ `right_dbus_smc_service` : (__string__) dbus smc service corresponding to the right wheel
