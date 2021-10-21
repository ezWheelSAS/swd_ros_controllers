# ez-Wheel SWD® ROS Controllers

## Overview
This package has been tested on ROS Melodic and Noetic, it contains ROS nodes to control motors powered by the [ez-Wheel](https://www.ez-wheel.com) Safety Wheel Drive (SWD®) technology.

<img src="https://www.ez-wheel.com/storage/image-product/visuels-swd-core-2-0-0.png" width="45%"></img> | <img src="https://www.ez-wheel.com/storage/image-product/roue-electrique-swd-150-2-0-0-0.png" width="45%"></img> | <img src="https://www.ez-wheel.com/storage/image-product/starterkit-ez-wheel-web-0-0-0.png" width="45%"></img> |
|------------|-------------|-------------|
| [SWD® Core](https://www.ez-wheel.com/en/safety-gear-motor) <br />Safety gear motor | [SWD® 150](https://www.ez-wheel.com/en/swd-150-safety-wheel-drive) <br />Safety wheel drive | [SWD® StarterKit](https://www.ez-wheel.com/en/development-kit-for-agv-and-amr) <br />Development kit for AGV and AMR |

## Prerequisites
- Two SWD® equipments
- An Ubuntu 20.04 ROS Machine
- `swd-services (>= 0.1.0)` installed

## Ubuntu repositories configuration

In order to install `swd_ros_controllers`, you need to add a third-party repository to `/etc/apt/sources.list` file.

```shell
echo "deb http://51.83.77.229:8081/apt-repo focal main" | sudo tee -a /etc/apt/sources.list
```

Then download and install the GPG key for the repository with the following bash command:

```shell
wget -qO - http://51.83.77.229:8081/archive.key | sudo apt-key add -
```

## Installation

On an Ubuntu based distro, install `ros-noetic-swd-ros-controllers` using the `apt` command:

```shell
sudo apt update && sudo apt install ros-noetic-swd-ros-controllers
```

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

### Parameters

- `left_swd_config_file` of type **`string`**: Path to the `.ini` configuration file of the left motor (mandatory parameter).
- `right_swd_config_file` of type **`string`**: Path to the `.ini` configuration file of the right motor (mandatory parameter).
- `baseline_m` of type **`double`**: The distance (in meters) between the 2 wheels (mandatory parameter).
- `pub_freq_hz` of type **`int`**: Frequency (in Hz) of published odometry and TFs (default `50`).
- `command_timeout_ms` of type **`int`**: The delay (in milliseconds) before stopping the wheels if no command is received (default `1000`).
- `base_frame` of type **`string`**: Frame ID for the moving platform, used in odometry and TFs (default `'base_link'`) (see [REP-150](https://www.ros.org/reps/rep-0105.html) for more info).
- `odom_frame` of type **`string`**: Frame ID for the `odom` fixed frame used in odometry and TFs (default `'odom'`) (see [REP-150](https://www.ros.org/reps/rep-0105.html) for more info).
- `publish_odom` of type **`bool`**: Publish odometry messages (default `true`).
- `publish_tf` of type **`bool`**: Publish odometry TF (default `true`).
- `publish_safety_functions` of type **`bool`**: Publish **`swd_ros_controllers::SafetyFunctions`** message (default `true`).
- `wheel_max_speed_rpm` of type **`double`**: Maximum allowed wheel speed (in RPM), if a target speed of one of the wheels is above this limit, the controller will limit the speed of the two wheels without changing the robot's trajectory (default `75.0`).
- `wheel_safety_limited_speed_rpm` of type **`double`**: Wheel safety limited speed (SLS) (in RPM), if an SLS signal is detected (from a security LiDAR for example), the wheel will be limited internally to the configured SLS limit, the ROS controller uses this value to limit the target speed sent to the motor in the SLS case (default `30.0`).
- `have_backward_sls` of type **`bool`**: Specifies if the robot have a backward SLS signal, coming for example from a back-facing security LiDAR. If an SLS signal is available for backward movements, set this to `true` to take it into account. Otherwise, set the parameter to `false`, this will limit all backward movements to the selected `wheel_safety_limited_speed_rpm` (default `false`).
- `positive_polarity_wheel` of type **`string`**: Internal parameter, used to select which wheels is set to a positive polarity (default `'Right'`).
- `control_mode` of type **`string`**: This parameter selects the control mode of the robot, if `'Twist'` is selected, the node will subscribe to the `~cmd_vel` topic, if `'LeftRightSpeeds'` is selected, the node subscribe to `~set_speed` (default `'Twist'`).

### Subscribed Topics

- `~cmd_vel` of type **`geometry_msgs::Twist`**: Target linear and angular velocities (when `control_mode:='Twist'`, this is the default).
- `~set_speed` of type **`geometry_msgs::Point`**: Target speeds in rad/s for left (`Point.x`) and right (`Point.y`) wheels (when `control_mode:='LeftRightSpeeds'`).
- `~soft_brake` of type **`std_msgs::Bool`**: Activate or release the soft brake, send `false` to release the brake, or `true` to activate it.

### Published Topics

- `~odom` of type **`nav_msgs::Odometry`**: Pose an velocity of the robot, based on wheels encoders. Associated TFs are also published, unless disabled in parameters.
- `~safety` of type **`swd_ros_controllers::SafetyFunctions`**: Safety messages communicated by the wheels via CANOpen, the message includes information about Safe Torque Off (STO), Safety Limited Speed (SLS), Safe Direction Indication (forward/backward) (SDI+/-), and Safe Brake Control (SBC).

## Custom message types

### The `swd_ros_controllers::SafetyFunctions` message

This message encodes the safety functions read from the SWD via CANOpen.

```
Header header
bool safe_torque_off
bool safe_brake_control
bool safety_limited_speed
bool safe_direction_indication_pos
bool safe_direction_indication_neg
```

## Compiling from source

To compile the package, you have to install `swd-services` package with

```shell
sudo apt-get update && sudo apt install swd-services
```

In the following instructions, replace `<rosdistro>` with the name of your ROS distro (e.g., `noetic`).

```shell
source /opt/ros/<rosdistro>/setup.bash
mkdir -p ~/ros_ws/src/
cd ~/ros_ws/src/
git clone https://github.com/ezWheelSAS/swd_ros_controllers.git
cd ..
catkin_make install
source ~/ros_ws/install/setup.bash
```

### Cross-compilation
TODO
 
## Support
For any questions, please [open a GitHub issue](https://github.com/ezWheelSAS/swd_ros_controllers/issues).

## About ez-Wheel®
**ez-Wheel®** is an innovative company founded in 2009 and located in Angoulême, France. 
The ez-Wheel company has developed the first industrial drive wheel, integrating electric traction motor, embedded electronics and rechargeable batteries.

This revolutionary solution, which quickly turns any manually handled platform into an electrically assisted one.
Our solutions have been adopted by hundreds of end-users to improve productivity and prevent work accidents caused by manual handling. 
Our products are used in a variety of applications, in fields of Automotive, Factory logistics, Warehouses, Food processing, Hospitals and Pharmaceutical industries.

The new SWD® product family targets industrial robotics applications, like Autonomous Mobile Robots (AMRs) and Automatic Guided Vehicles (AGVs).
It provides a unique solution for safety critical systems, which provides safety features according to the **ISO 3691-4** standard.

The [ez-Wheel®](https://www.ez-wheel.com) company has developed a unique know-how in embedded electronics, including safety critical systems, applied to battery powered electric traction.
