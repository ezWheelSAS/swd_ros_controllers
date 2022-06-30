/**
 * Copyright (C) 2021 ez-Wheel S.A.S.
 *
 * @file DiffDriveController.cpp
 */

#include "diff_drive_controller/DiffDriveController.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

#include <limits>

#include "ezw-smc-core/Config.hpp"
#include "ezw-smc-core/INMTService.hpp"

using namespace std::chrono_literals;

#define USE_SAFETY_CONTROL_WORD 0
#define VERBOSE_OUTPUT 0
#define TIMER_STATE_MACHINE_MS 1000
#define TIMER_SAFETY_MS 200

// Default values for parameters
#define DEFAULT_LEFT_CONFIG_FILE "/opt/ezw/usr/etc/ezw-smc-core/swd_left_config.ini"
#define DEFAULT_RIGHT_CONFIG_FILE "/opt/ezw/usr/etc/ezw-smc-core/swd_right_config.ini"
#define DEFAULT_ODOM_FRAME std::string("odom")
#define DEFAULT_BASE_FRAME std::string("base_link")
#define DEFAULT_CTRL_MODE std::string("Twist")
#define DEFAULT_MAX_WHEEL_SPEED_RPM 75.0  // 75 rpm Wheel => Motor (75 * 14 = 1050 rpm)
#define DEFAULT_MAX_SLS_WHEEL_RPM 30.0    // 30 rpm Wheel => Motor (30 * 14 = 490 rpm)
#define DEFAULT_PUB_FREQ_HZ 50
#define DEFAULT_WATCHDOG_MS 1000
#define DEFAULT_PUBLISH_ODOM true
#define DEFAULT_PUBLISH_TF true
#define DEFAULT_PUBLISH_SAFETY_FCNS true
#define DEFAULT_BACKWARD_SLS false
#define DEFAULT_FINE_ODOMETRY false

// Relative errors, used to calculate the covariance matrix in the odometry message
// Used as follow:
// d_dist_left +/- abs(d_dist_left) * LEFT_RELATIVE_ERROR
#define DEFAULT_LEFT_RELATIVE_ERROR 0.05  // 5% of error
#define DEFAULT_RIGHT_RELATIVE_ERROR 0.05

namespace ezw {
    namespace swd {
        DiffDriveController::DiffDriveController(std::shared_ptr<ros::NodeHandle> &p_nh) : m_nh(std::move(p_nh))
        {
            ROS_INFO("Initializing swd_diff_drive_controller, node name : %s", ros::this_node::getName().c_str());

            // Read parameters
            m_baseline_m = m_nh->param("baseline_m", 0.0);
            m_left_config_file = m_nh->param("left_swd_config_file", std::string(DEFAULT_LEFT_CONFIG_FILE));
            m_right_config_file = m_nh->param("right_swd_config_file", std::string(DEFAULT_RIGHT_CONFIG_FILE));
            m_pub_freq_hz = m_nh->param("pub_freq_hz", DEFAULT_PUB_FREQ_HZ);
            m_watchdog_receive_ms = m_nh->param("control_timeout_ms", DEFAULT_WATCHDOG_MS);
            m_base_frame = m_nh->param("base_frame", DEFAULT_BASE_FRAME);
            m_odom_frame = m_nh->param("odom_frame", DEFAULT_ODOM_FRAME);
            m_publish_odom = m_nh->param("publish_odom", DEFAULT_PUBLISH_ODOM);
            m_publish_tf = m_nh->param("publish_tf", DEFAULT_PUBLISH_TF);
            m_publish_safety = m_nh->param("publish_safety_functions", DEFAULT_PUBLISH_SAFETY_FCNS);
            m_have_backward_sls = m_nh->param("have_backward_sls", DEFAULT_BACKWARD_SLS);
            m_left_encoder_relative_error = m_nh->param("left_encoder_relative_error", DEFAULT_LEFT_RELATIVE_ERROR);
            m_right_encoder_relative_error = m_nh->param("right_encoder_relative_error", DEFAULT_RIGHT_RELATIVE_ERROR);
            m_fine_odometry = m_nh->param("fine_odometry", DEFAULT_FINE_ODOMETRY);
            double max_wheel_speed_rpm = m_nh->param("wheel_max_speed_rpm", DEFAULT_MAX_WHEEL_SPEED_RPM);
            double max_sls_wheel_speed_rpm = m_nh->param("wheel_safety_limited_speed_rpm", DEFAULT_MAX_SLS_WHEEL_RPM);
            std::string ctrl_mode = m_nh->param("control_mode", DEFAULT_CTRL_MODE);

            if (m_baseline_m <= 0) {
                ROS_ERROR("baseline_m parameter is mandatory and must be greater than 0");
                throw std::runtime_error("baseline_m parameter is mandatory and must be > 0");
            }

            if (m_pub_freq_hz <= 0) {
                m_pub_freq_hz = DEFAULT_PUB_FREQ_HZ;
                ROS_WARN(
                    "Invalid value %d for parameter 'pub_freq_hz', it must be greater than 0."
                    "Falling back to default (%d Hz).",
                    m_pub_freq_hz, DEFAULT_PUB_FREQ_HZ);
            }

            if (std::numeric_limits<double>::epsilon() >= m_left_encoder_relative_error) {
                m_left_encoder_relative_error = 0.001;
                ROS_WARN("'left_encoder_relative_error' set to 0, using 0.001 to prevent null uncertainties.");
            }

            if (std::numeric_limits<double>::epsilon() >= m_right_encoder_relative_error) {
                m_right_encoder_relative_error = 0.001;
                ROS_WARN("'right_encoder_relative_error' set to 0, using 0.001 to prevent null uncertainties.");
            }

            // Publishers
            if (m_publish_odom) {
                m_pub_odom = m_nh->advertise<nav_msgs::Odometry>("odom", 5);
            }

            if (m_publish_safety) {
                m_pub_safety = m_nh->advertise<swd_ros_controllers::SafetyFunctions>("safety", 5);
            }

            // Subscribers
            m_sub_brake = m_nh->subscribe("soft_brake", 5, &DiffDriveController::cbSoftBrake, this);

            if ("LeftRightSpeeds" == ctrl_mode) {
                m_sub_command = m_nh->subscribe("set_speed", 5, &DiffDriveController::cbSetSpeed, this);
            }
            else {
                m_sub_command = m_nh->subscribe("cmd_vel", 5, &DiffDriveController::cbCmdVel, this);
                if ("Twist" != ctrl_mode) {
                    ROS_WARN(
                        "Invalid value '%s' for parameter 'control_mode', accepted values: ['Twist' or 'LeftRightSpeeds']."
                        "Falling back to default (%s).",
                        ctrl_mode.c_str(), DEFAULT_CTRL_MODE.c_str());
                }
            }

            if (max_wheel_speed_rpm < 0.) {
                max_wheel_speed_rpm = DEFAULT_MAX_WHEEL_SPEED_RPM;
                ROS_ERROR(
                    "Invalid value %f for parameter 'wheel_max_speed_rpm', it should be a positive value. "
                    "Falling back to default (%f)",
                    max_wheel_speed_rpm, DEFAULT_MAX_WHEEL_SPEED_RPM);
            }

            if (max_sls_wheel_speed_rpm < 0.) {
                max_sls_wheel_speed_rpm = DEFAULT_MAX_SLS_WHEEL_RPM;
                ROS_ERROR(
                    "Invalid value %f for parameter 'wheel_safety_limited_speed_rpm', it should be a positive value. "
                    "Falling back to default (%f)",
                    max_sls_wheel_speed_rpm, DEFAULT_MAX_SLS_WHEEL_RPM);
            }

            // Initialize motors
            ROS_INFO("Motors config files, right : %s, left : %s", m_right_config_file.c_str(), m_left_config_file.c_str());

            ezw_error_t err;

            if (!m_right_config_file.empty()) {
                /* Config init */
                auto config = std::make_shared<ezw::smccore::Config>();
                err = config->load(m_right_config_file);
                if (err != ERROR_NONE) {
                    ROS_ERROR(
                        "Failed loading right motor's config file <%s>, CONTEXT_ID: %d, EZW_ERR: SMCService : "
                        "Config.init() return error code : %d",
                        m_right_config_file.c_str(), CON_APP, (int)err);
                    throw std::runtime_error("Failed loading right motor's config file");
                }

                m_right_wheel_diameter_m = config->getDiameter() * 1e-3;
                m_r_motor_reduction = config->getReduction();

                /* Init DBus client */
                std::string dbus_namespace = config->getDbusNamespace();
                std::transform(dbus_namespace.begin(), dbus_namespace.end(), dbus_namespace.begin(), ::tolower);

                std::string service_instance_name = "commonapi.ezw.smcservice." + dbus_namespace;

                err = m_right_controller.init(config->getContextId(), "local", service_instance_name);
                if (err != ERROR_NONE) {
                    ROS_ERROR(
                        "Failed initializing right motor, EZW_ERR: SMCService : "
                        "Controller::init() return error code : %d",
                        (int)err);
                    throw std::runtime_error("Failed initializing right motor");
                }
            }
            else {
                ROS_ERROR("Please specify the 'right_swd_config_file' parameter");
                throw std::runtime_error("Please specify the 'right_swd_config_file' parameter");
            }

            if (!m_left_config_file.empty()) {
                /* Config init */
                auto config = std::make_shared<ezw::smccore::Config>();
                err = config->load(m_left_config_file);
                if (err != ERROR_NONE) {
                    ROS_ERROR(
                        "Failed loading left motor's config file <%s>, CONTEXT_ID: %d, EZW_ERR: SMCService : "
                        "Config.init() return error code : %d",
                        m_left_config_file.c_str(), CON_APP, (int)err);
                    throw std::runtime_error("Failed initializing left motor");
                }

                m_left_wheel_diameter_m = config->getDiameter() * 1e-3;
                m_l_motor_reduction = config->getReduction();

                /* Init DBus client */
                std::string dbus_namespace = config->getDbusNamespace();
                std::transform(dbus_namespace.begin(), dbus_namespace.end(), dbus_namespace.begin(), ::tolower);

                std::string service_instance_name = "commonapi.ezw.smcservice." + dbus_namespace;

                err = m_left_controller.init(config->getContextId(), "local", service_instance_name);
                if (err != ERROR_NONE) {
                    ROS_ERROR(
                        "Failed initializing left motor, EZW_ERR: SMCService : "
                        "Controller::init() return error code : %d",
                        (int)err);
                    throw std::runtime_error("Failed initializing left motor");
                }
            }
            else {
                ROS_ERROR("Please specify the 'left_swd_config_file' parameter");
                throw std::runtime_error("Please specify the 'left_swd_config_file' parameter");
            }

            // Read initial encoders values
            err = m_fine_odometry ? m_left_controller.getFineOdometryValue(m_dist_left_prev_mm) : m_left_controller.getOdometryValue(m_dist_left_prev_mm);
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Failed initial reading from left motor, EZW_ERR: SMCService : "
                    "Controller::getOdometryValue() return error code : %d",
                    (int)err);
                throw std::runtime_error("Initial reading from left motor failed");
            }

            err = m_fine_odometry ? m_right_controller.getFineOdometryValue(m_dist_right_prev_mm) : m_right_controller.getOdometryValue(m_dist_right_prev_mm);
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Failed initial reading from right motor, EZW_ERR: SMCService : "
                    "Controller::getOdometryValue() return error code : %d",
                    (int)err);
                throw std::runtime_error("Initial reading from right motor failed");
            }

            ezw::smccore::IPDSService::PolarityParameters polarity_parameters;
            err = m_left_controller.getPolarityParameters(polarity_parameters);
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Failed reading the motor polarity, EZW_ERR: SMCService : "
                    "Controller::getPolarityParameters() return error code : %d",
                    (int)err);
                throw std::runtime_error("Failed reading the left motor polarity");
            }
            m_left_motor_polarity = polarity_parameters.velocity_polarity;

            ROS_INFO(
                "left motor polarity : %s", m_left_motor_polarity ? "True" : "False");

            // Set m_max_motor_speed_rpm from wheel_sls and motor_reduction
            m_max_motor_speed_rpm = static_cast<int32_t>(max_wheel_speed_rpm * m_l_motor_reduction);
            m_motor_sls_rpm = static_cast<int32_t>(max_sls_wheel_speed_rpm * m_l_motor_reduction);

            ROS_INFO(
                "Got parameter 'wheel_max_speed_rpm' = %f rpm. "
                "Setting maximum motor speed to %d rpm",
                max_wheel_speed_rpm, m_max_motor_speed_rpm);

            ROS_INFO(
                "Got parameter 'wheel_safety_limited_speed_rpm' = %f rpm. "
                "Setting maximum motor safety limited speed to %d rpm",
                max_sls_wheel_speed_rpm, m_motor_sls_rpm);

            // Create timers
            m_timer_watchdog = m_nh->createTimer(ros::Duration(m_watchdog_receive_ms / 1000.0), boost::bind(&DiffDriveController::cbWatchdog, this));
            m_timer_pds = m_nh->createTimer(ros::Duration(0.5), boost::bind(&DiffDriveController::cbTimerStateMachine, this));

            if (m_publish_odom || m_publish_tf) {
                m_timer_odom = m_nh->createTimer(ros::Duration(1.0 / m_pub_freq_hz), std::bind(&DiffDriveController::cbTimerOdom, this));
            }

            m_timer_safety = m_nh->createTimer(ros::Duration(TIMER_SAFETY_MS / 1000.0), std::bind(&DiffDriveController::cbTimerSafety, this));

            ROS_INFO("ez-Wheel's swd_diff_drive_controller initialized successfully!");
        }

        void DiffDriveController::cbTimerStateMachine()
        {
            static bool m_first_entry = true;

            // NMT state machine
            smccore::INMTService::NMTState nmt_state_l, nmt_state_r;
            smccore::IPDSService::PDSState pds_state_l, pds_state_r;
            ezw_error_t err_l, err_r;

            pds_state_l = pds_state_r = smccore::IPDSService::PDSState::SWITCH_ON_DISABLED;

            err_l = m_left_controller.getNMTState(nmt_state_l);
            if (ERROR_NONE != err_l) {
                ROS_ERROR(
                    "Failed to get the NMT state for left motor, EZW_ERR: SMCService : "
                    "Controller::getPDSState() return error code : %d",
                    (int)err_l);
            }

            err_r = m_right_controller.getNMTState(nmt_state_r);
            if (ERROR_NONE != err_r) {
                ROS_ERROR(
                    "Failed to get the NMT state for right motor, EZW_ERR: SMCService : "
                    "Controller::getPDSState() return error code : %d",
                    (int)err_r);
            }

            bool nmt_ok = (smccore::INMTService::NMTState::OPER == nmt_state_l) && (smccore::INMTService::NMTState::OPER == nmt_state_r);

            if (m_first_entry || m_nmt_ok != nmt_ok) {
                ROS_INFO("NMT state machine is %s.", nmt_ok ? "OK" : "not OK");
                m_nmt_ok = nmt_ok;
            }

            if (!m_nmt_ok) {
                // Broadcast NMT command PREOP to all canopen nodes
                ezw_error_t err = m_left_controller.broadcastNMTState(smccore::INMTService::NMTCommand::PREOP);
                if (ERROR_NONE != err) {
                    ROS_ERROR(
                        "Failed to broadcast NMT command PREOP"
                        "Controller::broadcastNMTState() return error code : %d",
                        (int)err);
                }
                else {
                    usleep((10) * 1000);
                    // Broadcast NMT command OPER to all canopen nodes
                    err = m_left_controller.broadcastNMTState(smccore::INMTService::NMTCommand::OPER);
                    if (ERROR_NONE != err) {
                        ROS_ERROR(
                            "Failed to broadcast NMT state OPER"
                            "Controller::broadcastNMTState() return error code : %d",
                            (int)err);
                    }
                }
            }

            bool pds_ok = false;

            // If NMT is operational, check the PDS state
            if (m_nmt_ok) {
                // PDS state machine
                err_l = m_left_controller.getPDSState(pds_state_l);
                err_r = m_right_controller.getPDSState(pds_state_r);

                if (ERROR_NONE != err_l) {
                    ROS_ERROR(
                        "Failed to get the PDS state for left motor, EZW_ERR: SMCService : "
                        "Controller::getPDSState() return error code : %d",
                        (int)err_l);
                }

                if (ERROR_NONE != err_r) {
                    ROS_ERROR(
                        "Failed to get the PDS state for right motor, EZW_ERR: SMCService : "
                        "Controller::getPDSState() return error code : %d",
                        (int)err_r);
                }

                pds_ok = (smccore::IPDSService::PDSState::OPERATION_ENABLED == pds_state_l) && (smccore::IPDSService::PDSState::OPERATION_ENABLED == pds_state_r);

                if (smccore::IPDSService::PDSState::OPERATION_ENABLED != pds_state_l) {
                    err_l = m_left_controller.enterInOperationEnabledState();
                }

                if (smccore::IPDSService::PDSState::OPERATION_ENABLED != pds_state_r) {
                    err_r = m_right_controller.enterInOperationEnabledState();
                }
            }

            if (m_first_entry || m_pds_ok != pds_ok) {
                ROS_INFO("PDS state machine is %s.", pds_ok ? "OK" : "not OK");
                m_pds_ok = pds_ok;
            }

            m_first_entry = false;
        }

        void DiffDriveController::cbSoftBrake(const std_msgs::Bool::ConstPtr &p_msg)
        {
            // true => Enable brake
            // false => Release brake
            ezw_error_t err = m_left_controller.setHalt(p_msg->data != 0);
            if (ERROR_NONE != err) {
                ROS_ERROR("SoftBrake: Failed %s left motor, EZW_ERR: %d", p_msg->data ? "braking" : "releasing", (int)err);
            }
            else {
                ROS_INFO("SoftBrake: Left motor's soft brake %s", p_msg->data ? "activated" : "disabled");
            }

            err = m_right_controller.setHalt(p_msg->data != 0);
            if (ERROR_NONE != err) {
                ROS_ERROR("SoftBrake: Failed %s right motor, EZW_ERR: %d", p_msg->data ? "braking" : "releasing", (int)err);
            }
            else {
                ROS_INFO("SoftBrake: Right motor's soft brake %s", p_msg->data ? "activated" : "disabled");
            }
        }

        void DiffDriveController::cbTimerOdom()
        {
            nav_msgs::Odometry msg_odom;

            int32_t left_dist_now_mm = 0, right_dist_now_mm = 0;

            ezw_error_t err_l, err_r;
            err_l = m_fine_odometry ? m_left_controller.getFineOdometryValue(left_dist_now_mm) : m_left_controller.getOdometryValue(left_dist_now_mm);      // In mm
            err_r = m_fine_odometry ? m_right_controller.getFineOdometryValue(right_dist_now_mm) : m_right_controller.getOdometryValue(right_dist_now_mm);  // In mm

            if (ERROR_NONE != err_l) {
                ROS_ERROR(
                    "Failed reading from left motor, EZW_ERR: SMCService : "
                    "Controller::getOdometryValue() return error code : %d",
                    (int)err_l);
                return;
            }

            if (ERROR_NONE != err_r) {
                ROS_ERROR(
                    "Failed reading from right motor, EZW_ERR: SMCService : "
                    "Controller::getOdometryValue() return error code : %d",
                    (int)err_r);
                return;
            }

            // Encoder difference between t and t-1
            double d_dist_left_m = static_cast<double>(left_dist_now_mm - m_dist_left_prev_mm) / 1000.0;
            double d_dist_right_m = static_cast<double>(right_dist_now_mm - m_dist_right_prev_mm) / 1000.0;

            // Error calculation (standard deviation)
            double d_dist_left_err_m = m_left_encoder_relative_error * std::abs(d_dist_left_m);
            double d_dist_right_err_m = m_right_encoder_relative_error * std::abs(d_dist_right_m);

            auto timestamp = ros::Time::now();

            // Kinematic model
            double d_dist_center = (d_dist_left_m + d_dist_right_m) / 2.0;
            double d_theta = (d_dist_right_m - d_dist_left_m) / m_baseline_m;

            // Error propagation (See https://en.wikipedia.org/wiki/Propagation_of_uncertainty#Non-linear_combinations)
            double d_dist_center_err = std::sqrt(std::pow(d_dist_left_err_m / 2.0, 2) + std::pow(d_dist_right_err_m / 2.0, 2));
            double d_theta_err = std::sqrt(std::pow(d_dist_left_err_m / m_baseline_m, 2) + std::pow(d_dist_right_err_m / m_baseline_m, 2));

            // Odometry model, integration of the diff drive kinematic model
            double x_now = m_x_prev + d_dist_center * std::cos(m_theta_prev);
            double y_now = m_y_prev + d_dist_center * std::sin(m_theta_prev);
            double theta_now = M_BOUND_ANGLE(m_theta_prev + d_theta);

            // Error propagation
            double x_now_err = std::sqrt(std::pow(m_x_prev_err, 2) + std::pow(std::cos(m_theta_prev) * d_dist_center_err, 2) + std::pow(-std::sin(m_theta_prev) * d_dist_center * m_theta_prev_err, 2));
            double y_now_err = std::sqrt(std::pow(m_y_prev_err, 2) + std::pow(std::sin(m_theta_prev) * d_dist_center_err, 2) + std::pow(std::cos(m_theta_prev) * d_dist_center * m_theta_prev_err, 2));
            double theta_now_err = std::sqrt(std::pow(m_theta_prev_err, 2) + std::pow(d_theta_err, 2));

            msg_odom.header.stamp = timestamp;
            msg_odom.header.frame_id = m_odom_frame;
            msg_odom.child_frame_id = m_base_frame;

            msg_odom.twist = geometry_msgs::TwistWithCovariance();
            msg_odom.twist.twist.linear.x = d_dist_center * m_pub_freq_hz;
            msg_odom.twist.twist.angular.z = d_theta * m_pub_freq_hz;

            // Set uncertainties for linear and angular velocities (6 * 6) matrix (x y z Rx Ry Rz)
            msg_odom.twist.covariance[0] = std::pow(d_dist_center_err * m_pub_freq_hz, 2);
            msg_odom.twist.covariance[35] = std::pow(d_theta_err * m_pub_freq_hz, 2);

            msg_odom.pose.pose.position.x = x_now;
            msg_odom.pose.pose.position.y = y_now;
            msg_odom.pose.pose.position.z = 0.0;

            tf2::Quaternion quat_orientation;
            quat_orientation.setRPY(0.0, 0.0, theta_now);
            msg_odom.pose.pose.orientation.x = quat_orientation.getX();
            msg_odom.pose.pose.orientation.y = quat_orientation.getY();
            msg_odom.pose.pose.orientation.z = quat_orientation.getZ();
            msg_odom.pose.pose.orientation.w = quat_orientation.getW();

            // Set uncertainties for x, y, and theta (Rz)
            msg_odom.pose.covariance[0] = std::pow(x_now_err, 2);
            msg_odom.pose.covariance[7] = std::pow(y_now_err, 2);
            msg_odom.pose.covariance[35] = std::pow(theta_now_err, 2);

            if (m_publish_odom) {
                m_pub_odom.publish(msg_odom);
            }

            if (m_publish_tf) {
                geometry_msgs::TransformStamped tf_odom_baselink;
                tf_odom_baselink.header.stamp = timestamp;
                tf_odom_baselink.header.frame_id = m_odom_frame;
                tf_odom_baselink.child_frame_id = m_base_frame;

                tf_odom_baselink.transform.translation.x = msg_odom.pose.pose.position.x;
                tf_odom_baselink.transform.translation.y = msg_odom.pose.pose.position.y;
                tf_odom_baselink.transform.translation.z = msg_odom.pose.pose.position.z;
                tf_odom_baselink.transform.rotation.x = msg_odom.pose.pose.orientation.x;
                tf_odom_baselink.transform.rotation.y = msg_odom.pose.pose.orientation.y;
                tf_odom_baselink.transform.rotation.z = msg_odom.pose.pose.orientation.z;
                tf_odom_baselink.transform.rotation.w = msg_odom.pose.pose.orientation.w;

                // Send TF
                m_tf2_br.sendTransform(tf_odom_baselink);
            }

            m_x_prev = x_now;
            m_y_prev = y_now;
            m_theta_prev = theta_now;
            m_x_prev_err = x_now_err;
            m_y_prev_err = y_now_err;
            m_theta_prev_err = theta_now_err;
            m_dist_left_prev_mm = left_dist_now_mm;
            m_dist_right_prev_mm = right_dist_now_mm;
        }

        ///
        /// \brief Change wheel speed (msg.x = left wheel, msg.y = right wheel) [rad/s]
        ///
        void DiffDriveController::cbSetSpeed(const geometry_msgs::PointConstPtr &p_speed)
        {
            m_timer_watchdog.stop();
            m_timer_watchdog.start();

            // Convert rad/s wheel speed to rpm motor speed
            auto left = static_cast<int32_t>(p_speed->x * m_l_motor_reduction * 60.0 / (2.0 * M_PI));
            auto right = static_cast<int32_t>(p_speed->y * m_r_motor_reduction * 60.0 / (2.0 * M_PI));

#if VERBOSE_OUTPUT
            ROS_INFO(
                "Got RightLeftSpeeds command: (left, right) = (%f, %f) rad/s. "
                "Calculated speeds (left, right) = (%d, %d) rpm",
                p_speed->x, p_speed->y, left, right);
#endif

            setSpeeds(left, right);
        }

        ///
        /// \brief Change robot velocity (linear [m/s], angular [rad/s])
        ///
        void DiffDriveController::cbCmdVel(const geometry_msgs::TwistPtr &p_cmd_vel)
        {
            m_timer_watchdog.stop();
            m_timer_watchdog.start();

            double left_vel, right_vel;

            // Control model (diff drive)
            left_vel = (2. * p_cmd_vel->linear.x - p_cmd_vel->angular.z * m_baseline_m) / m_left_wheel_diameter_m;
            right_vel = (2. * p_cmd_vel->linear.x + p_cmd_vel->angular.z * m_baseline_m) / m_right_wheel_diameter_m;

            // Convert rad/s wheel speed to rpm motor speed
            auto left = static_cast<int32_t>(left_vel * m_l_motor_reduction * 60.0 / (2.0 * M_PI));
            auto right = static_cast<int32_t>(right_vel * m_r_motor_reduction * 60.0 / (2.0 * M_PI));

#if VERBOSE_OUTPUT
            ROS_INFO(
                "Got Twist command: linear = %f m/s, angular = %f rad/s. "
                "Calculated speeds (left, right) = (%d, %d) rpm",
                p_cmd_vel->linear.x, p_cmd_vel->angular.z, left, right);
#endif

            setSpeeds(left, right);
        }

#define CONF_MAX_DELTA_SPEED_SLS 140  // in rpm motor
#define CONF_MAX_DELTA_SPEED 1000     // in rpm motor
#define CONF_MIN_SPEED 40             // in rpm motor

        ///
        /// \brief Change robot velocity (left in rpm, right in rpm)
        ///
        void DiffDriveController::setSpeeds(int32_t p_left_speed, int32_t p_right_speed)
        {
            ezw_error_t err = ERROR_NONE;

            // Get the outer motor speed
            int32_t faster_motor_speed = M_MAX(std::abs(p_left_speed), std::abs(p_right_speed));
            int32_t speed_limit = -1;
            bool max_limited = false;
            bool sls_limited = false;

            // Limit to the maximum allowed speed
            if (faster_motor_speed > m_max_motor_speed_rpm) {
                speed_limit = m_max_motor_speed_rpm;
                max_limited = true;
            }

            // Impose the safety limited speed (SLS) in backward movement when the robot doesn't have backward SLS signal.
            // For example, if it has only one forward-facing safety LiDAR, when the robot move backwards, there's no
            // safety guarantees, hence speed is limited to SLS, otherwise, the safety limit will be decided by the
            // presence of the SLS signal.
            if (!m_have_backward_sls && (p_left_speed < 0) && (p_right_speed < 0) && (faster_motor_speed > m_max_motor_speed_rpm)) {
                speed_limit = m_max_motor_speed_rpm;
            }

            // Reading SLS
            bool res_l, res_r;
            err = m_left_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::SLS_1, res_l);
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Error reading SLS from left motor, EZW_ERR: SMCService : "
                    "Controller::getSafetyFunctionCommand() return error code : %d",
                    (int)err);
            }

            err = m_right_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::SLS_1, res_r);
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Error reading SLS from right motor, EZW_ERR: SMCService : "
                    "Controller::getSafetyFunctionCommand() return error code : %d",
                    (int)err);
            }

            bool sls_signal = !(res_l || res_r);

            // If SLS detected, impose the safety limited speed (SLS)
            if (sls_signal && (faster_motor_speed > m_motor_sls_rpm)) {
                speed_limit = m_motor_sls_rpm;
                sls_limited = true;
            }

            // The left and right motors may have different speeds.
            // If we need to limit one of them, we need to scale the second motor speed.
            // This ensures a speed limitation without distorting the target path.
            if (-1 != speed_limit) {
                // If we enter here, we are sure that (faster_motor_speed > speed_limit).
                // Get the ratio between the outer (faster) motor, and the speed limit.
                double speed_ratio = static_cast<double>(speed_limit) / static_cast<double>(faster_motor_speed);

                // Scale right speed
                p_right_speed = static_cast<int32_t>(static_cast<double>(p_right_speed) * speed_ratio);

                // Scale left speed
                p_left_speed = static_cast<int32_t>(static_cast<double>(p_left_speed) * speed_ratio);

                if (max_limited && sls_limited) {
                    ROS_INFO(
                        "The target speed exceeds the MAX/SLS maximum speed limit (%d rpm). "
                        "Set speed to (left, right) (%d, %d) rpm",
                        speed_limit, p_left_speed, p_right_speed);
                }
                else if (sls_limited) {
                    ROS_INFO(
                        "The target speed exceeds the SLS maximum speed limit (%d rpm). "
                        "Set speed to (left, right) (%d, %d) rpm",
                        speed_limit, p_left_speed, p_right_speed);
                }
                else if (max_limited) {
                    ROS_INFO(
                        "The target speed exceeds the maximum speed limit (%d rpm). "
                        "Set speed to (left, right) (%d, %d) rpm",
                        speed_limit, p_left_speed, p_right_speed);
                }
            }

            // Get the delta wheel speed
            int32_t delta_wheel_speed = std::abs(p_left_speed - p_right_speed);
            int32_t delta_speed_limit = -1;

            // Limit to the maximum allowed delta speed
            if (delta_wheel_speed > CONF_MAX_DELTA_SPEED) {
                delta_speed_limit = CONF_MAX_DELTA_SPEED;
            }

            // If SLS detected, limit to the maximum allowed delta safety limited speed (SLS)
            if (sls_signal && (delta_wheel_speed > CONF_MAX_DELTA_SPEED_SLS)) {
                delta_speed_limit = CONF_MAX_DELTA_SPEED_SLS;
            }

            // The left and right wheels may have different speeds.
            // If we need to limit one of them, we need to scale the second wheel speed.
            // This ensures a delta speed limitation without distorting the target path.
            if (-1 != delta_speed_limit) {
                // Get the ratio between the max allowed delta speed limit, and the current delta speed limit.
                double delta_speed_ratio = static_cast<double>(delta_speed_limit) / static_cast<double>(delta_wheel_speed);

                // Scale right speed
                p_right_speed = static_cast<int32_t>(static_cast<double>(p_right_speed) * delta_speed_ratio);

                // Scale left speed
                p_left_speed = static_cast<int32_t>(static_cast<double>(p_left_speed) * delta_speed_ratio);

                ROS_INFO(
                    "The target speed exceeds the maximum delta speed limit (%d rpm). "
                    "Speed set to (left, right) (%d, %d) rpm",
                    delta_speed_limit, p_left_speed, p_right_speed);
            }

            // If not SLS detected && left minimum speed detected, impose the minimum speed
            bool left_min_limit = !sls_signal && std::abs(p_left_speed) > 1 && std::abs(p_left_speed) <= CONF_MIN_SPEED;

            // If not SLS detected && right minimum speed detected, impose the minimum speed
            bool right_min_limit = !sls_signal && std::abs(p_right_speed) > 1 && std::abs(p_right_speed) <= CONF_MIN_SPEED;

            if (left_min_limit || right_min_limit) {
                int32_t left_speed = p_left_speed;
                int32_t right_speed = p_right_speed;

                // Update left speed
                if (left_min_limit) {
                    p_left_speed = (p_left_speed > 0) ? CONF_MIN_SPEED : -CONF_MIN_SPEED;
                }

                // Update right speed
                if (right_min_limit) {
                    p_right_speed = (p_right_speed > 0) ? CONF_MIN_SPEED : -CONF_MIN_SPEED;
                }

                ROS_INFO(
                    "The target speed falls behind the minimum speed limit (left, right) (%d, %d rpm)."
                    "Set speed to (left, right) (%d, %d) rpm",
                    left_speed, right_speed, p_left_speed, p_right_speed);
            }

            // If the PDS state is not OPERATION_ENABLED, we send a nil speed.
            if (!m_pds_ok) {
                p_left_speed = p_right_speed = 0;
            }

            // Send the actual speed (in RPM) to left motor
            err = m_left_controller.setTargetVelocity(static_cast<int16_t>(p_left_speed));
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Failed setting velocity of left motor, EZW_ERR: SMCService : "
                    "Controller::setTargetVelocity() return error code : %d",
                    (int)err);
                return;
            }

            // Send the actual speed (in RPM) to right motor
            err = m_right_controller.setTargetVelocity(static_cast<int16_t>(p_right_speed));
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Failed setting velocity of right motor, EZW_ERR: SMCService : "
                    "Controller::setTargetVelocity() return error code : %d",
                    (int)err);
                return;
            }

#if VERBOSE_OUTPUT
            ROS_INFO("Speed sent to motors (left, right) = (%d, %d) rpm", p_left_speed, p_right_speed);
#endif
        }

        void DiffDriveController::cbTimerSafety()
        {
            static bool m_first_entry = true;

            swd_ros_controllers::SafetyFunctions msg;
            ezw_error_t err;
            bool res_l, res_r;

            if (!m_nmt_ok) {
                return;
            }

            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = m_base_frame;

            // Reading SBC
            err = m_left_controller.getSafetyFunctionStatus(ezw::smccore::ISafeMotionService::SafetyFunctionId::SBC_1, res_l);
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Error reading SBC from left motor, EZW_ERR: SMCService : "
                    "Controller::getSafetyFunctionStatus() return error code : %d",
                    (int)err);
            }

            err = m_right_controller.getSafetyFunctionStatus(ezw::smccore::ISafeMotionService::SafetyFunctionId::SBC_1, res_r);
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Error reading SBC from right motor, EZW_ERR: SMCService : "
                    "Controller::getSafetyFunctionStatus() return error code : %d",
                    (int)err);
            }

            msg.safe_brake_control = static_cast<uint8_t>(!(res_l || res_r));
            if (m_first_entry || msg.safe_brake_control != m_safety_msg.safe_brake_control) {
                ROS_INFO(msg.safe_brake_control ? "SBC enabled." : "SBC disabled.");
            }

            if (res_l != res_r) {
                ROS_ERROR("Inconsistant SBC for left and right motors, left=%d, right=%d.", res_l, res_r);
            }

            // Reading STO
            err = m_left_controller.getSafetyFunctionStatus(ezw::smccore::ISafeMotionService::SafetyFunctionId::STO, res_l);
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Error reading STO from left motor, EZW_ERR: SMCService : "
                    "Controller::getSafetyFunctionStatus() return error code : %d",
                    (int)err);
            }

            err = m_right_controller.getSafetyFunctionStatus(ezw::smccore::ISafeMotionService::SafetyFunctionId::STO, res_r);
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Error reading STO from right motor, EZW_ERR: SMCService : "
                    "Controller::getSafetyFunctionStatus() return error code : %d",
                    (int)err);
            }

            msg.safe_torque_off = static_cast<uint8_t>(res_l || res_r);
            if (m_first_entry || msg.safe_torque_off != m_safety_msg.safe_torque_off) {
                ROS_INFO(msg.safe_torque_off ? "STO enabled." : "STO disabled.");
            }

            if (res_l != res_r) {
                ROS_ERROR("Inconsistant STO for left and right motors, left=%d, right=%d.", res_l, res_r);
            }

            // Reading SDI
            bool sdi_l_p, sdi_l_n, sdi_r_p, sdi_r_n, sdi_p, sdi_n;

            err = m_left_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::SDIP_1, sdi_l_p);
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Error reading SDI+ from left motor, EZW_ERR: SMCService : "
                    "Controller::getSafetyFunctionCommand() return error code : %d",
                    (int)err);
            }

            err = m_left_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::SDIN_1, sdi_l_n);
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Error reading SDI- from left motor, EZW_ERR: SMCService : "
                    "Controller::getSafetyFunctionCommand() return error code : %d",
                    (int)err);
            }

            err = m_right_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::SDIP_1, sdi_r_p);
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Error reading SDI+ from right motor, EZW_ERR: SMCService : "
                    "Controller::getSafetyFunctionCommand() return error code : %d",
                    (int)err);
            }

            err = m_right_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::SDIN_1, sdi_r_n);
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Error reading SDI- from right motor, EZW_ERR: SMCService : "
                    "Controller::getSafetyFunctionCommand() return error code : %d",
                    (int)err);
            }

            if (m_left_motor_polarity) {
                sdi_p = !(sdi_l_n || sdi_r_p);
                sdi_n = !(sdi_l_p || sdi_r_n);
            }
            else {
                sdi_p = !(sdi_l_p || sdi_r_n);
                sdi_n = !(sdi_l_n || sdi_r_p);
            }

            msg.safe_direction_indication_forward = static_cast<uint8_t>(sdi_p);
            msg.safe_direction_indication_backward = static_cast<uint8_t>(sdi_n);

            if (m_first_entry || msg.safe_direction_indication_forward != m_safety_msg.safe_direction_indication_forward) {
                ROS_INFO(msg.safe_direction_indication_forward ? "SDIp enabled." : "SDIp disabled.");
            }

            if (m_first_entry || msg.safe_direction_indication_backward != m_safety_msg.safe_direction_indication_backward) {
                ROS_INFO(msg.safe_direction_indication_backward ? "SDIn enabled." : "SDIn disabled.");
            }

            // Reading SLS
            err = m_left_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::SLS_1, res_l);
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Error reading SLS from left motor, EZW_ERR: SMCService : "
                    "Controller::getSafetyFunctionCommand() return error code : %d",
                    (int)err);
            }

            err = m_right_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::SLS_1, res_r);
            if (ERROR_NONE != err) {
                ROS_ERROR(
                    "Error reading SLS from right motor, EZW_ERR: SMCService : "
                    "Controller::getSafetyFunctionCommand() return error code : %d",
                    (int)err);
            }

            msg.safety_limited_speed = static_cast<uint8_t>(!(res_l || res_r));
            if (m_first_entry || msg.safety_limited_speed != m_safety_msg.safety_limited_speed) {
                ROS_INFO(msg.safety_limited_speed ? "SLS enabled." : "SLS disabled.");
            }

#if VERBOSE_OUTPUT
            ROS_INFO("STO: %d, SDI+: %d, SDI-: %d, SLS: %d", msg.safe_torque_off, msg.safe_direction_indication_forward, msg.safe_direction_indication_backward, msg.safety_limited_speed);
#endif

            m_safety_msg_mtx.lock();
            m_safety_msg = msg;
            m_safety_msg_mtx.unlock();

            if (m_publish_safety) {
                m_pub_safety.publish(msg);
            }

            m_first_entry = false;
        }

        ///
        /// \brief A safety callback which gets activated if no control message
        /// have been received since `control_timeout_ms`
        ///
        void DiffDriveController::cbWatchdog()
        {
            setSpeeds(0, 0);
        }
    }  // namespace swd
}  // namespace ezw
