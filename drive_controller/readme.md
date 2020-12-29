# Description :

This driver manages __N__ ez-Wheel Gen2 wheels independently.

# Topics Published
For each wheel :
+ `~joint_state` of type __`sensor_msgs::JointState`__ with the motor name, position (m), velocity (m/s) and timestamp

# Topics Subscribed
For each wheel :
+ `~set_speed` of type __`std_msgs::Float64`__ with motor speed in rad/s

# Node parameters :
+ `pub_freq_hz` : (__`int`__) Publication frequency of joint state (Hz)
+ `watchdog_receive_ms` : (__`int`__) Watchdog delay before stopping the wheels if no command is received
+ `list_of_dbus_smc_services` : (__`vector<string>`__) List of dbus smc services corresponding to the wheels
