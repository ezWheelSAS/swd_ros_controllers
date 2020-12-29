# Description :

This driver manages __N__ ez-Wheels independently.

# Topics Published
For each wheel :
+ `~joint_state` of type __sensor_msgs::JointState__ with the motor name, position (m), velocity (m/s) and timestamp

# Topics Subscribed
For each wheel :
+ `~set_speed` of type __std_msgs::Float64__ with motor speed in rad/s

# Node parameters :
+ `pub_freq_hz` : (__int__) Publication frequency of joint state (Hz)
+ `watchdog_receive_ms` : (__int__) Watchdog delay before stopping the wheels if no command is received
+ `list_of_dbus_smc_services` : (__vector\<string\>__) List of dbus smc services corresponding to the wheels
