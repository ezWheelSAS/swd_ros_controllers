# EZW ROS Controllers

This package has been tested on ROS Melodic and Noetic, it contains a ROS node to control two motors in a differential drive robot.


## The `diff_drive_controller` node

This controller drives two ez-Wheel SWD wheels as a differential-drive robot.


### Published Topics 

- `~odom` of type **`nav_msgs::Odometry`**: Pose an velocity of the robot, based on wheels encoders. Associated TFs are also published, unless disabled in parameters.
- `~safety` of type **`ezw_ros_controllers::SafetyFunctions`**: Safety messages communicated by the wheels via CANOpen, the message includes information about Safe Torque Off (STO), Safety Limited Speed (SLS), Safe Direction Indication (forward/backward) (SDI+/-), and Safe Brake Control (SBC) (currently SDI(neg) and SBC are not published).


### Subscribed Topics 

- `~cmd_vel` of type **`geometry_msgs::Twist`**: Target linear and angular velocities (when `control_mode:='Twist'`, this is the default).
- `~set_speed` of type **`geometry_msgs::Point`**: Target speeds in rad/s for left (`Point.x`) and right (`Point.y`) wheels (when `control_mode:='LeftRightSpeeds'`).
- `~soft_brake` of type **`std_msgs::String`**: Activate or release the soft brake, send `'disable'` to release the brake, or `'enable'` to activate it.


### Node parameters

- `baseline_m` of type (**`double`**): The distance (in meters) between the 2 wheels.
- `pub_freq_hz` of type (**`int`**): Frequency (in Hz) of published odometry and TFs (default `50`).
- `watchdog_receive_ms` of type (**`int`**): The delay (in milliseconds) before stopping the wheels if no command is received (default `1000`).
- `left_config_file` of type (**`string`**): Path to the `.ini` configuration file of the left motor.
- `right_config_file` of type (**`string`**): Path to the `.ini` configuration file of the right motor.
- `m_base_link` of type (**`string`**): Frame ID for the moving platform, used in Odometry and TFs (default `'base_link'`).
- `odom_frame` of type (**`string`**): Frame ID for odometry reference frame, used in Odometry and TFs (default `'odom'`).
- `publish_odom` of type (**`bool`**): Publish odometry messages (default `true`).
- `publish_tf` of type (**`bool`**): Publish odometry TF (default `true`).
- `publish_safety_functions` of type (**`bool`**): Publish **`ezw_ros_controllers::SafetyFunctions`** message (default `true`).
- `wheel_max_speed_rpm` of type (**`double`**): Maximum allowed wheel speed (in RPM), if a target speed of one of the wheels is above this limit, the controller will limit the speed of the two wheels without changing the robot's trajectory (default `75`).
- `wheel_safety_limited_speed_rpm` of type (**`double`**): Wheel safety limited speed (SLS) (in RPM), if an SLS signal is detected (from a security LiDAR for example), the wheel will be limited internally to the configured SLS limit, the ROS controller uses this value to limit the target speed sent to the motor in the SLS case (default `30`).
- `ref_wheel` of type (**`string`**): Internal parameter, used to select which wheels is set to a positive polarity (default `'Right'`).
- `control_mode` of type (**`string`**): This parameter selects the control mode of the robot, if `'Twist'` is selected, the node will subscribe to the `~cmd_vel` topic, if `'LeftRightSpeeds'` is selected, the node subscribe to `~set_speed` (default `'Twist'`).


## Custom message types
### The `SafetyFunctions` message
This message encodes the safety functions read from the SWD via CANOpen.

```
Header header
bool safe_torque_off
bool safe_brake_control (Reserved for future use)
bool safety_limited_speed
bool safe_direction_indication_pos
bool safe_direction_indication_neg (Reserved for future use)
```


**ez-Wheel (c) 2021-10-11**
