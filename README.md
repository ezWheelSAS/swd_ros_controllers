# ez-Wheel SWD ROS Controllers

This package has been tested on ROS Melodic and Noetic, it contains ROS nodes to control motors powered by the [ez-Wheel](https://www.ez-wheel.com) Safety Wheel Drive (SWD®) technology ([SWD® Core](https://www.ez-wheel.com/en/safety-gear-motor) or [SWD® 150](https://www.ez-wheel.com/en/swd-150-safety-wheel-drive)).

## Usage

The package comes with a preconfigured `.launch` file for the [SWD® Starter Kit](https://www.ez-wheel.com/en/development-kit-for-agv-and-amr):

```shell
roslaunch swd_ros_controllers swd_diff_drive_controller.launch
```

Or you can run it with a custom configuration, the minimum required parameters are:

```shell
rosrun swd_ros_controllers swd_diff_drive_controller \
                           _left_swd_config_file:="/path/to/swd_left.ini" \
                           _right_swd_config_file:="/path/to/swd_right.ini" \
                           _baseline:=0.485
```

## The `swd_diff_drive_controller` node

This controller drives two ez-Wheel SWD® wheels as a differential-drive robot.

### Node parameters

- `left_swd_config_file` of type **`string`**: Path to the `.ini` configuration file of the left motor (mandatory parameter).
- `right_swd_config_file` of type **`string`**: Path to the `.ini` configuration file of the right motor (mandatory parameter).
- `baseline_m` of type **`double`**: The distance (in meters) between the 2 wheels (mandatory parameter).
- `pub_freq_hz` of type **`int`**: Frequency (in Hz) of published odometry and TFs (default `50`).
- `command_timeout_ms` of type **`int`**: The delay (in milliseconds) before stopping the wheels if no command is received (default `1000`).
- `base_frame` of type **`string`**: Frame ID for the moving platform, used in odometry and TFs (default `'base_link'`) (see [REP-150](https://www.ros.org/reps/rep-0105.html) for more info).
- `odom_frame` of type **`string`**: Frame ID for the `odom` fixed frame used in odometry and TFs (default `'odom'`) (see [REP-150](https://www.ros.org/reps/rep-0105.html) for more info).
- `publish_odom` of type **`bool`**: Publish odometry messages (default `true`).
- `publish_tf` of type **`bool`**: Publish odometry TF (default `true`).
- `publish_safety_functions` of type **`bool`**: Publish **`ezw_ros_controllers::SafetyFunctions`** message (default `true`).
- `wheel_max_speed_rpm` of type **`double`**: Maximum allowed wheel speed (in RPM), if a target speed of one of the wheels is above this limit, the controller will limit the speed of the two wheels without changing the robot's trajectory (default `75.0`).
- `wheel_safety_limited_speed_rpm` of type **`double`**: Wheel safety limited speed (SLS) (in RPM), if an SLS signal is detected (from a security LiDAR for example), the wheel will be limited internally to the configured SLS limit, the ROS controller uses this value to limit the target speed sent to the motor in the SLS case (default `30.0`).
- `ref_wheel` of type **`string`**: Internal parameter, used to select which wheels is set to a positive polarity (default `'Right'`).
- `control_mode` of type **`string`**: This parameter selects the control mode of the robot, if `'Twist'` is selected, the node will subscribe to the `~cmd_vel` topic, if `'LeftRightSpeeds'` is selected, the node subscribe to `~set_speed` (default `'Twist'`).

### Subscribed Topics

- `~cmd_vel` of type **`geometry_msgs::Twist`**: Target linear and angular velocities (when `control_mode:='Twist'`, this is the default).
- `~set_speed` of type **`geometry_msgs::Point`**: Target speeds in rad/s for left (`Point.x`) and right (`Point.y`) wheels (when `control_mode:='LeftRightSpeeds'`).
- `~soft_brake` of type **`std_msgs::String`**: Activate or release the soft brake, send `'disable'` to release the brake, or `'enable'` to activate it.

### Published Topics

- `~odom` of type **`nav_msgs::Odometry`**: Pose an velocity of the robot, based on wheels encoders. Associated TFs are also published, unless disabled in parameters.
- `~safety` of type **`ezw_ros_controllers::SafetyFunctions`**: Safety messages communicated by the wheels via CANOpen, the message includes information about Safe Torque Off (STO), Safety Limited Speed (SLS), Safe Direction Indication (forward/backward) (SDI+/-), and Safe Brake Control (SBC) _(currently SDI(neg) and SBC are not published)_.

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

**[ez-Wheel](https://www.ez-wheel.com) © 2021**
