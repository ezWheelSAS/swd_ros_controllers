**ez-Wheel (c) 2021-03-26**

# EZW ROS Controllers

This package has been tested on ROS Melodic, it contains two ROS nodes:

-   `drive_controller` : A ROS controller for a single/multiple ez-Wheel Gen2 wheels individually.
-   `diff_drive_controller` : A ROS controller for a differential-drive mobile robot (2 wheels).

## The `drive_controller` node

This node manages **N** ez-Wheel Gen2 wheels independently.

### Topics Published

For each wheel :

-   `~joint_state` of type **`sensor_msgs::JointState`** with the motor name, position (m), velocity (m/s) and timestamp

### Topics Subscribed

For each wheel :

-   `~set_speed` of type **`std_msgs::Float64`** with motor speed in rad/s

### Node parameters :

-   `pub_freq_hz` : (**`int`**) Publication frequency of joint state (Hz)
-   `watchdog_receive_ms` : (**`int`**) Watchdog delay before stopping the wheels if no command is received
-   `motors_dict` : (**`map<string, string>`**) A map (dictionary) of motor names and their's associated `.ini` configuration files.

## The `diff_drive_controller` node

This controller drives two ez-Wheel Gen2 wheels as a differential-drive robot.

### Topics Published

-   `~odom` of type **`nav_msgs::Odometry`** with x, y, theta and timestamp
-   `~joint_state` of type **`sensor_msgs::JointState`** with the motor name, position (m), velocity (m/s) and timestamp

### Topics Subscribed

-   `~set_speed` of type **`geometry_msgs::Point`** where x = left motor speed and y = right motor speed in rad/s

### Node parameters :

-   `baseline_m` : (**`double`**) Gap between the 2 wheels (m)
-   `pub_freq_hz` : (**`int`**) Publication frequency of odometry (Hz)
-   `watchdog_receive_ms` : (**`int`**) Watchdog delay before stopping the wheels if no command is received
-   `left_config_file` : (**`string`**) Path to the `.ini` configuration file of the left motor
-   `right_config_file` : (**`string`**) Path to the `.ini` configuration file of the right motor
