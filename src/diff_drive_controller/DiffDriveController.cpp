/**
 * Copyright (C) 2021 ez-Wheel S.A.S.
 *
 * @file DiffDriveController.cpp
 */

#include "diff_drive_controller/DiffDriveController.hpp"

#include "ezw-smc-core/CANOpenDispatcher.hpp"

#include "ezw-canopen-service/DBusClient.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

#define USE_SAFETY_CONTROL_WORD 0
#define VERBOSE_OUTPUT          0

// Default values for parameters
#define DEFAULT_ODOM_FRAME          std::string("odom")
#define DEFAULT_BASE_FRAME          std::string("base_link")
#define DEFAULT_REF_WHEEL           std::string("Right")
#define DEFAULT_CTRL_MODE           std::string("Twist")
#define DEFAULT_MAX_WHEEL_SPEED_RPM 75.0 // 75 rpm Wheel => Motor (75 * 14 = 1050 rpm)
#define DEFAULT_MAX_SLS_WHEEL_RPM   30.0 // 30 rpm Wheel => Motor (30 * 14 = 490 rpm)
#define DEFAULT_PUB_FREQ_HZ         50
#define DEFAULT_WATCHDOG_MS         1000
#define DEFAULT_PUBLISH_ODOM        true
#define DEFAULT_PUBLISH_TF          true
#define DEFAULT_PUBLISH_SAFETY_FCNS true

namespace ezw
{
    namespace swd
    {
        DiffDriveController::DiffDriveController(const std::shared_ptr<ros::NodeHandle> nh) : m_nh(nh)
        {
            ROS_INFO("Initializing swd_diff_drive_controller, node name : %s", ros::this_node::getName().c_str());

            // Read parameters
            m_baseline_m                        = m_nh->param("baseline_m", 0.0);
            m_left_config_file                  = m_nh->param("left_swd_config_file", std::string(""));
            m_right_config_file                 = m_nh->param("right_swd_config_file", std::string(""));
            m_pub_freq_hz                       = m_nh->param("pub_freq_hz", DEFAULT_PUB_FREQ_HZ);
            m_watchdog_receive_ms               = m_nh->param("control_timeout_ms", DEFAULT_WATCHDOG_MS);
            m_base_frame                        = m_nh->param("base_frame", DEFAULT_BASE_FRAME);
            m_odom_frame                        = m_nh->param("odom_frame", DEFAULT_ODOM_FRAME);
            m_publish_odom                      = m_nh->param("publish_odom", DEFAULT_PUBLISH_ODOM);
            m_publish_tf                        = m_nh->param("publish_tf", DEFAULT_PUBLISH_TF);
            m_publish_safety                    = m_nh->param("publish_safety_functions", DEFAULT_PUBLISH_SAFETY_FCNS);
            double      max_wheel_speed_rpm     = m_nh->param("wheel_max_speed_rpm", DEFAULT_MAX_WHEEL_SPEED_RPM);
            double      max_sls_wheel_speed_rpm = m_nh->param("wheel_safety_limited_speed_rpm", DEFAULT_MAX_SLS_WHEEL_RPM);
            std::string ref_wheel               = m_nh->param("ref_wheel", DEFAULT_REF_WHEEL);
            std::string ctrl_mode               = m_nh->param("control_mode", DEFAULT_CTRL_MODE);

            if ("Left" == ref_wheel) {
                m_left_wheel_polarity = -1;
            } else {
                m_left_wheel_polarity = 1;
                if ("Right" != ref_wheel) {
                    ROS_WARN("Invalid value '%s' for parameter 'ref_wheel', accepted values: ['Right' or 'Left']."
                             "Falling back to default (%s).",
                             ref_wheel.c_str(), DEFAULT_REF_WHEEL.c_str());
                }
            }

            if (m_baseline_m <= 0) {
                ROS_ERROR("baseline_m parameter is mandatory and must be greater than 0");
                throw std::runtime_error("baseline_m parameter is mandatory and must be > 0");
            }

            if (m_pub_freq_hz <= 0) {
                m_pub_freq_hz = DEFAULT_PUB_FREQ_HZ;
                ROS_WARN("Invalid value %d for parameter 'pub_freq_hz', it must be greater than 0."
                         "Falling back to default (%d Hz).",
                         m_pub_preq_hz, DEFAULT_PUB_FREQ_HZ);
            }

            // Publishers
            if (m_publish_odom) {
                m_pub_odom = m_nh->advertise<nav_msgs::Odometry>("odom", 5);
            }

            if (m_publish_safety) {
                m_pub_safety = m_nh->advertise<ezw_ros_controllers::SafetyFunctions>("safety", 5);
            }

            // Subscribers
            m_sub_brake = m_nh->subscribe("soft_brake", 5, &DiffDriveController::cbSoftBrake, this);

            if ("LeftRightSpeeds" == ctrl_mode) {
                m_sub_command = m_nh->subscribe("set_speed", 5, &DiffDriveController::cbSetSpeed, this);
            } else {
                m_sub_command = m_nh->subscribe("cmd_vel", 5, &DiffDriveController::cbCmdVel, this);
                if ("Twist" != ctrl_mode) {
                    ROS_WARN("Invalid value '%s' for parameter 'control_mode', accepted values: ['Twist' or 'LeftRightSpeeds']."
                             "Falling back to default (%s).",
                             ctrl_mode.c_str(), DEFAULT_CTRL_MODE.c_str());
                }
            }

            if (max_wheel_speed_rpm < 0.) {
                max_wheel_speed_rpm = DEFAULT_MAX_WHEEL_SPEED_RPM;
                ROS_ERROR("Invalid value %f for parameter 'wheel_max_speed_rpm', it should be a positive value. "
                          "Falling back to default (%f)",
                          max_wheel_speed_rpm, DEFAULT_MAX_WHEEL_SPEED_RPM);
            }

            if (max_sls_wheel_speed_rpm < 0.) {
                max_sls_wheel_speed_rpm = DEFAULT_MAX_SLS_WHEEL_RPM;
                ROS_ERROR("Invalid value %f for parameter 'wheel_safety_limited_speed_rpm', it should be a positive value. "
                          "Falling back to default (%f)",
                          max_sls_wheel_speed_rpm, DEFAULT_MAX_SLS_WHEEL_RPM);
            }

            // Initialize motors
            ROS_INFO("Motors config files, right : %s, left : %s", m_right_config_file.c_str(), m_left_config_file.c_str());

            ezw_error_t err;

            if ("" != m_right_config_file) {
                /* Config init */
                auto lConfig = std::make_shared<ezw::smccore::Config>();
                err          = lConfig->load(m_right_config_file);
                if (err != ERROR_NONE) {
                    ROS_ERROR("Failed loading right motor's config file <%s>, CONTEXT_ID: %d, EZW_ERR: SMCService : "
                              "Config.init() return error code : %d",
                              m_right_config_file.c_str(), CON_APP, (int)err);
                    throw std::runtime_error("Failed loading right motor's config file");
                }

                m_right_wheel_diameter_m = lConfig->getDiameter() * 1e-3;
                m_r_motor_reduction      = lConfig->getReduction();

                /* CANOpenService client init */
                auto lCOSClient = std::make_shared<ezw::canopenservice::DBusClient>();
                err             = lCOSClient->init();
                if (err != ERROR_NONE) {
                    ROS_ERROR("Failed initializing right motor, CONTEXT_ID: %d, EZW_ERR: SMCService : "
                              "COSDBusClient::init() return error code : %d",
                              lConfig->getContextId(), (int)err);
                    throw std::runtime_error("Failed initializing right motor");
                }

                /* CANOpenDispatcher */
                auto lCANOpenDispatcher = std::make_shared<ezw::smccore::CANOpenDispatcher>(lConfig, lCOSClient);
                err                     = lCANOpenDispatcher->init();
                if (err != ERROR_NONE) {
                    ROS_ERROR("Failed initializing right motor, CONTEXT_ID: %d, EZW_ERR: SMCService : "
                              "CANOpenDispatcher::init() return error code : %d",
                              lConfig->getContextId(), (int)err);
                    throw std::runtime_error("Failed initializing right motor");
                }

                err = m_right_controller.init(lConfig, lCANOpenDispatcher);
                if (ERROR_NONE != err) {
                    ROS_ERROR("Failed initializing right motor, EZW_ERR: SMCService : "
                              "Controller::init() return error code : %d",
                              (int)err);
                    throw std::runtime_error("Failed initializing right motor");
                }
            } else {
                ROS_ERROR("Please specify the 'right_swd_config_file' parameter");
                throw std::runtime_error("Please specify the 'right_swd_config_file' parameter");
            }

            if ("" != m_left_config_file) {
                /* Config init */
                auto lConfig = std::make_shared<ezw::smccore::Config>();
                err          = lConfig->load(m_left_config_file);
                if (err != ERROR_NONE) {
                    ROS_ERROR("Failed loading left motor's config file <%s>, CONTEXT_ID: %d, EZW_ERR: SMCService : "
                              "Config.init() return error code : %d",
                              m_right_config_file.c_str(), CON_APP, (int)err);
                    throw std::runtime_error("Failed initializing left motor");
                }

                m_left_wheel_diameter_m = lConfig->getDiameter() * 1e-3;
                m_l_motor_reduction     = lConfig->getReduction();

                /* CANOpenService client init */
                auto lCOSClient = std::make_shared<ezw::canopenservice::DBusClient>();
                err             = lCOSClient->init();
                if (err != ERROR_NONE) {
                    ROS_ERROR("Failed initializing left motor, EZW_ERR: SMCService : "
                              "COSDBusClient::init() return error code : %d",
                              (int)err);
                    throw std::runtime_error("Failed initializing left motor");
                }

                /* CANOpenDispatcher */
                auto lCANOpenDispatcher = std::make_shared<ezw::smccore::CANOpenDispatcher>(lConfig, lCOSClient);
                err                     = lCANOpenDispatcher->init();
                if (err != ERROR_NONE) {
                    ROS_ERROR("Failed initializing left motor, EZW_ERR: SMCService : "
                              "CANOpenDispatcher::init() return error code : %d",
                              (int)err);
                    throw std::runtime_error("Failed initializing left motor");
                }

                err = m_left_controller.init(lConfig, lCANOpenDispatcher);
                if (ERROR_NONE != err) {
                    ROS_ERROR("Failed initializing left motor, EZW_ERR: SMCService : "
                              "Controller::init() return error code : %d",
                              (int)err);
                    throw std::runtime_error("Failed initializing left motor");
                }
            } else {
                ROS_ERROR("Please specify the 'left_swd_config_file' parameter");
                throw std::runtime_error("Please specify the 'left_swd_config_file' parameter");
            }

            // Read initial encoders values
            err = m_left_controller.getPositionValue(m_dist_left_prev_mm);
            if (ERROR_NONE != err) {
                ROS_ERROR("Failed initial reading from left motor, EZW_ERR: SMCService : "
                          "Controller::getPositionValue() return error code : %d",
                          (int)err);
            }

            err = m_right_controller.getPositionValue(m_dist_right_prev_mm);
            if (ERROR_NONE != err) {
                ROS_ERROR("Failed initial reading from right motor, EZW_ERR: SMCService : "
                          "Controller::getPositionValue() return error code : %d",
                          (int)err);
            }

            // Set m_max_motor_speed_rpm from wheel_sls and motor_reduction
            m_max_motor_speed_rpm = static_cast<int32_t>(max_wheel_speed_rpm * m_l_motor_reduction);
            m_motor_sls_rpm       = static_cast<int32_t>(max_sls_wheel_speed_rpm * m_l_motor_reduction);

            ROS_INFO("Got parameter 'wheel_max_speed_rpm' = %f rpm. "
                     "Setting maximum motor speed to %d rpm",
                     max_wheel_speed_rpm, m_max_motor_speed_rpm);

            ROS_INFO("Got parameter 'wheel_safety_limited_speed_rpm' = %f rpm. "
                     "Setting maximum motor safety limited speed to %d rpm",
                     max_sls_wheel_speed_rpm, m_motor_sls_rpm);

            m_timer_watchdog = m_nh->createTimer(ros::Duration(m_watchdog_receive_ms / 1000.0), boost::bind(&DiffDriveController::cbWatchdog, this));
            m_timer_pds      = m_nh->createTimer(ros::Duration(1.0), boost::bind(&DiffDriveController::cbTimerStateMachine, this));

            if (m_publish_odom || m_publish_tf) {
                m_timer_odom = m_nh->createTimer(ros::Duration(1.0 / m_pub_freq_hz), boost::bind(&DiffDriveController::cbTimerOdom, this));
            }

            if (m_publish_safety) {
                m_timer_safety = m_nh->createTimer(ros::Duration(1.0 / 5.0), boost::bind(&DiffDriveController::cbTimerSafety, this));
            }

            ROS_INFO("ez-Wheel's swd_diff_drive_controller initialized successfully!");
        }

        void DiffDriveController::cbTimerStateMachine()
        {
            // NMT state machine
            smccore::Controller::NMTState nmt_state_l, nmt_state_r;
            smccore::Controller::PDSState pds_state_l, pds_state_r;
            ezw_error_t                   err_l, err_r;

            nmt_state_l = nmt_state_r = smccore::Controller::NMTState::UNKNOWN;
            pds_state_l = pds_state_r = smccore::Controller::PDSState::SWITCH_ON_DISABLED;

            err_l = m_left_controller.getNMTState(nmt_state_l);
            err_r = m_right_controller.getNMTState(nmt_state_r);

            if (ERROR_NONE != err_l) {
                ROS_ERROR("Failed to get the NMT state for left motor, EZW_ERR: SMCService : "
                          "Controller::getPDSState() return error code : %d",
                          (int)err_l);
            }

            if (ERROR_NONE != err_r) {
                ROS_ERROR("Failed to get the NMT state for right motor, EZW_ERR: SMCService : "
                          "Controller::getPDSState() return error code : %d",
                          (int)err_r);
            }

            if (smccore::Controller::NMTState::OPER != nmt_state_l) {
                err_l = m_left_controller.setNMTState(smccore::Controller::NMTCommand::OPER);
            }

            if (smccore::Controller::NMTState::OPER != nmt_state_r) {
                err_r = m_right_controller.setNMTState(smccore::Controller::NMTCommand::OPER);
            }

            if (ERROR_NONE != err_l && smccore::Controller::NMTState::OPER != nmt_state_l) {
                ROS_ERROR("Failed to set NMT state for left motor, EZW_ERR: SMCService : "
                          "Controller::setNMTState() return error code : %d",
                          (int)err_l);
            }

            if (ERROR_NONE != err_r && smccore::Controller::NMTState::OPER != nmt_state_r) {
                ROS_ERROR("Failed to set NMT state for right motor, EZW_ERR: SMCService : "
                          "Controller::setNMTState() return error code : %d",
                          (int)err_r);
            }

            m_nmt_ok = (smccore::Controller::NMTState::OPER == nmt_state_l) && (smccore::Controller::NMTState::OPER == nmt_state_r);

            // If NMT is operational, check the PDS state
            if (m_nmt_ok) {
                // PDS state machine
                err_l = m_left_controller.getPDSState(pds_state_l);
                err_r = m_right_controller.getPDSState(pds_state_r);

                if (ERROR_NONE != err_l) {
                    ROS_ERROR("Failed to get the PDS state for left motor, EZW_ERR: SMCService : "
                              "Controller::getPDSState() return error code : %d",
                              (int)err_l);
                }

                if (ERROR_NONE != err_r) {
                    ROS_ERROR("Failed to get the PDS state for right motor, EZW_ERR: SMCService : "
                              "Controller::getPDSState() return error code : %d",
                              (int)err_r);
                }

                if (smccore::Controller::PDSState::OPERATION_ENABLED != pds_state_l) {
                    err_l = m_left_controller.enterInOperationEnabledState();
                }

                if (smccore::Controller::PDSState::OPERATION_ENABLED != pds_state_r) {
                    err_r = m_right_controller.enterInOperationEnabledState();
                }

                if (ERROR_NONE != err_l && smccore::Controller::PDSState::OPERATION_ENABLED != pds_state_l) {
                    ROS_ERROR("Failed to set PDS state for left motor, EZW_ERR: SMCService : "
                              "Controller::enterInOperationEnabledState() return error code : %d",
                              (int)err_l);
                }

                if (ERROR_NONE != err_r && smccore::Controller::PDSState::OPERATION_ENABLED != pds_state_r) {
                    ROS_ERROR("Failed to set PDS state for right motor, EZW_ERR: SMCService : "
                              "Controller::enterInOperationEnabledState() return error code : %d",
                              (int)err_r);
                }
            }

            m_pds_ok = (smccore::Controller::PDSState::OPERATION_ENABLED == pds_state_l) && (smccore::Controller::PDSState::OPERATION_ENABLED == pds_state_r);
        }

        void DiffDriveController::cbSoftBrake(const std_msgs::String::ConstPtr &msg)
        {
            // "enable" or something else -> Stop
            // "disable" -> Release
            bool halt = ("disable" == msg->data) ? false : true;

            ezw_error_t err = m_left_controller.setHalt(halt);
            if (ERROR_NONE != err) {
                ROS_ERROR("SoftBrake: Failed %s left wheel, EZW_ERR: %d", halt ? "braking" : "releasing", (int)err);
            } else {
                ROS_INFO("SoftBrake: Left motor's soft brake %s", halt ? "activated" : "disabled");
            }

            err = m_right_controller.setHalt(halt);
            if (ERROR_NONE != err) {
                ROS_ERROR("SoftBrake: Failed %s right wheel, EZW_ERR: %d", halt ? "braking" : "releasing", (int)err);
            } else {
                ROS_INFO("SoftBrake: Right motor's soft brake %s", halt ? "activated" : "disabled");
            }
        }

        void DiffDriveController::cbTimerOdom()
        {
            nav_msgs::Odometry msg_odom;

            int32_t     left_dist_now_mm = 0, right_dist_now_mm = 0;
            ezw_error_t err_l, err_r;

            err_l = m_left_controller.getPositionValue(left_dist_now_mm);   // In mm
            err_r = m_right_controller.getPositionValue(right_dist_now_mm); // In mm

            if (ERROR_NONE != err_l) {
                ROS_ERROR("Failed reading from left motor, EZW_ERR: SMCService : "
                          "Controller::getPositionValue() return error code : %d",
                          (int)err_l);
                return;
            }

            if (ERROR_NONE != err_r) {
                ROS_ERROR("Failed reading from right motor, EZW_ERR: SMCService : "
                          "Controller::getPositionValue() return error code : %d",
                          (int)err_r);
                return;
            }

            // Encoder difference between t and t-1
            double d_dist_left  = static_cast<double>(left_dist_now_mm - m_dist_left_prev_mm) / 1000.0;
            double d_dist_right = static_cast<double>(right_dist_now_mm - m_dist_right_prev_mm) / 1000.0;

            ros::Time timestamp = ros::Time::now();

            // Kinematic model
            double d_dist_center = (d_dist_left + d_dist_right) / 2.0;
            double d_theta       = static_cast<double>(m_left_wheel_polarity) * (d_dist_right - d_dist_left) / m_baseline_m;

            // Odometry model, integration of the diff drive kinematic model
            double x_now     = m_x_prev + d_dist_center * std::cos(m_theta_prev);
            double y_now     = m_y_prev + d_dist_center * std::sin(m_theta_prev);
            double theta_now = boundAngle(m_theta_prev + d_theta);

            msg_odom.header.stamp    = timestamp;
            msg_odom.header.frame_id = m_odom_frame;
            msg_odom.child_frame_id  = m_base_frame;

            msg_odom.twist                 = geometry_msgs::TwistWithCovariance();
            msg_odom.twist.twist.linear.x  = d_dist_center * m_pub_freq_hz;
            msg_odom.twist.twist.angular.z = d_theta * m_pub_freq_hz;

            msg_odom.pose.pose.position.x = x_now;
            msg_odom.pose.pose.position.y = y_now;
            msg_odom.pose.pose.position.z = 0.0;

            tf2::Quaternion quat_orientation;
            quat_orientation.setRPY(0.0, 0.0, theta_now);
            msg_odom.pose.pose.orientation.x = quat_orientation.getX();
            msg_odom.pose.pose.orientation.y = quat_orientation.getY();
            msg_odom.pose.pose.orientation.z = quat_orientation.getZ();
            msg_odom.pose.pose.orientation.w = quat_orientation.getW();

            if (m_publish_odom) {
                m_pub_odom.publish(msg_odom);
            }

            if (m_publish_tf) {
                geometry_msgs::TransformStamped tf_odom_baselink;
                tf_odom_baselink.header.stamp    = timestamp;
                tf_odom_baselink.header.frame_id = m_odom_frame;
                tf_odom_baselink.child_frame_id  = m_base_frame;

                tf_odom_baselink.transform.translation.x = msg_odom.pose.pose.position.x;
                tf_odom_baselink.transform.translation.y = msg_odom.pose.pose.position.y;
                tf_odom_baselink.transform.translation.z = msg_odom.pose.pose.position.z;
                tf_odom_baselink.transform.rotation.x    = msg_odom.pose.pose.orientation.x;
                tf_odom_baselink.transform.rotation.y    = msg_odom.pose.pose.orientation.y;
                tf_odom_baselink.transform.rotation.z    = msg_odom.pose.pose.orientation.z;
                tf_odom_baselink.transform.rotation.w    = msg_odom.pose.pose.orientation.w;

                // Send TF
                m_tf2_br.sendTransform(tf_odom_baselink);
            }

            m_x_prev             = x_now;
            m_y_prev             = y_now;
            m_theta_prev         = theta_now;
            m_dist_left_prev_mm  = left_dist_now_mm;
            m_dist_right_prev_mm = right_dist_now_mm;
        }

        ///
        /// \brief Change wheel speed (msg.x = left wheel, msg.y = right wheel) [rad/s]
        ///
        void DiffDriveController::cbSetSpeed(const geometry_msgs::PointConstPtr &speed)
        {
            m_timer_watchdog.stop();
            m_timer_watchdog.start();

            // Convert rad/s wheel speed to rpm motor speed
            int32_t left  = static_cast<int32_t>(speed->x * m_l_motor_reduction * 60.0 / (2.0 * M_PI));
            int32_t right = static_cast<int32_t>(speed->y * m_l_motor_reduction * 60.0 / (2.0 * M_PI));

#if VERBOSE_OUTPUT
            ROS_INFO("Got RightLeftSpeeds command: (left, right) = (%f, %f) rad/s. "
                     "Calculated speeds (left, right) = (%d, %d) rpm",
                     speed->x, speed->y, left, right);
#endif

            setSpeeds(left, right);
        }

        ///
        /// \brief Change robot velocity (linear [m/s], angular [rad/s])
        ///
        void DiffDriveController::cbCmdVel(const geometry_msgs::TwistPtr &cmd_vel)
        {
            m_timer_watchdog.stop();
            m_timer_watchdog.start();

            double left_vel, right_vel;

            // Control model (diff drive)
            left_vel  = (2. * cmd_vel->linear.x - cmd_vel->angular.z * m_baseline_m) / m_left_wheel_diameter_m;
            right_vel = (2. * cmd_vel->linear.x + cmd_vel->angular.z * m_baseline_m) / m_right_wheel_diameter_m;

            // Convert rad/s wheel speed to rpm motor speed
            int32_t left  = static_cast<int32_t>(left_vel * m_l_motor_reduction * 60.0 / (2.0 * M_PI));
            int32_t right = static_cast<int32_t>(right_vel * m_r_motor_reduction * 60.0 / (2.0 * M_PI));

#if VERBOSE_OUTPUT
            ROS_INFO("Got Twist command: linear = %f m/s, angular = %f rad/s. "
                     "Calculated speeds (left, right) = (%d, %d) rpm",
                     cmd_vel->linear.x, cmd_vel->angular.z, left, right);
#endif

            setSpeeds(left, right);
        }

        ///
        /// \brief Change robot velocity (left in rpm, right in rpm)
        ///
        void DiffDriveController::setSpeeds(int32_t left_speed, int32_t right_speed)
        {
            // Get the outer wheel speed
            int32_t faster_wheel_speed = M_MAX(std::abs(left_speed), std::abs(right_speed));
            int32_t speed_limit        = -1;

            // Limit to the maximum allowed speed
            if (faster_wheel_speed > m_max_motor_speed_rpm) {
                speed_limit = m_max_motor_speed_rpm;
            }

            // In backward movement, impose the safety limited speed (SLS)
            // This assumes the robot to have only one safety LiDAR mounted on the front,
            // when the robot moves backward, there's no safety guarantees, so speed is limited to SLS
            if ((left_speed < 0) && (right_speed < 0) && (faster_wheel_speed > m_motor_sls_rpm)) {
                speed_limit = m_motor_sls_rpm;
            }

            // If SLS detected, impose the safety limited speed (SLS)
            if (m_safety_msg.safety_limited_speed && (faster_wheel_speed > m_motor_sls_rpm)) {
                speed_limit = m_motor_sls_rpm;
            }

            // The left and right wheels may have different speeds.
            // If we need to limit one of them, we need to scale the second wheel speed.
            // This ensures a speed limitation without distorting the target path.
            if (-1 != speed_limit) {
                // If we enter here, we are sure that (faster_wheel_speed > speed_limit).
                // Get the ratio between the outer (faster) wheel, and the speed limit.
                double speed_ratio = static_cast<double>(speed_limit) / static_cast<double>(faster_wheel_speed);

                // Get the faster wheel
                if (std::abs(left_speed) > std::abs(right_speed)) {
                    // Scale right_speed
                    right_speed = static_cast<int32_t>(static_cast<double>(right_speed) * speed_ratio);

                    // Limit the left_speed
                    left_speed = M_SIGN(left_speed) * speed_limit;
                } else {
                    // Scale left_speed
                    left_speed = static_cast<int32_t>(static_cast<double>(left_speed) * speed_ratio);

                    // Limit the right_speed
                    right_speed = M_SIGN(right_speed) * speed_limit;
                }

                ROS_WARN("The target speed exceeds the maximum speed limit (%d rpm). "
                         "Speed set to (left, right) (%d, %d) rpm",
                         speed_limit, left_speed, right_speed);
            }

            // Send the actual speed (in RPM) to left motor
            ezw_error_t err = m_left_controller.setTargetVelocity(left_speed);
            if (ERROR_NONE != err) {
                ROS_ERROR("Failed setting velocity of right motor, EZW_ERR: SMCService : "
                          "Controller::setTargetVelocity() return error code : %d",
                          (int)err);
                return;
            }

            // Send the actual speed (in RPM) to left motor
            err = m_right_controller.setTargetVelocity(right_speed);
            if (ERROR_NONE != err) {
                ROS_ERROR("Failed setting velocity of right motor, EZW_ERR: SMCService : "
                          "Controller::setTargetVelocity() return error code : %d",
                          (int)err);
                return;
            }

#if VERBOSE_OUTPUT
            ROS_INFO("Speed sent to motors (left, right) = (%d, %d) rpm", left_speed, right_speed);
#endif
        }

        void DiffDriveController::cbTimerSafety()
        {
            ezw_ros_controllers::SafetyFunctions msg;
            ezw_error_t                          err;
            bool                                 res_l, res_r;

#if USE_SAFETY_CONTROL_WORD
            ezw::smccore::Controller::SafetyWordType res;

            err = m_left_controller.getSafetyControlWord(ezw::smccore::Controller::SafetyControlWordId::SAFEIN_1, res);

            msg.safe_torque_off               = res.safety_function_2 && res.safety_function_3;
            msg.safe_direction_indication_pos = res.safety_function_2 && res.safety_function_3;
            msg.safety_limited_speed          = res.safety_function_4 && res.safety_function_5;

#if VERBOSE_OUTPUT
            ROS_INFO("STO: %d, SDI+: %d, SLS: %d", msg.safe_torque_off, msg.safe_direction_indication_pos, msg.safety_limited_speed);
#endif

            m_pub_safety.publish(msg);
#else
            if (m_nmt_ok) {
                msg.header.stamp = ros::Time::now();

                // Reading STO
                err = m_left_controller.getSafetyFunctionCommand(ezw::smccore::Controller::SafetyFunctionId::STO, res_l);
                if (ERROR_NONE != err) {
                    ROS_ERROR("Error reading STO from left motor, EZW_ERR: SMCService : "
                              "Controller::getSafetyFunctionCommand() return error code : %d",
                              (int)err);
                }

                err = m_right_controller.getSafetyFunctionCommand(ezw::smccore::Controller::SafetyFunctionId::STO, res_r);
                if (ERROR_NONE != err) {
                    ROS_ERROR("Error reading STO from right motor, EZW_ERR: SMCService : "
                              "Controller::getSafetyFunctionCommand() return error code : %d",
                              (int)err);
                }

                msg.safe_torque_off = !(res_l || res_r);

                if (res_l != res_r) {
                    ROS_ERROR("Inconsistant STO for left and right motors, left=%d, right=%d.", res_l, res_r);
                }

                // Reading SDI
                ezw::smccore::Controller::SafetyFunctionId safety_fcn_l, safety_fcn_r;

                if (m_left_wheel_polarity == 1) {
                    // Right
                    safety_fcn_l = ezw::smccore::Controller::SafetyFunctionId::SDIP_1;
                    safety_fcn_r = ezw::smccore::Controller::SafetyFunctionId::SDIN_1;
                } else {
                    // Left
                    safety_fcn_l = ezw::smccore::Controller::SafetyFunctionId::SDIN_1;
                    safety_fcn_r = ezw::smccore::Controller::SafetyFunctionId::SDIP_1;
                }

                err = m_left_controller.getSafetyFunctionCommand(safety_fcn_l, res_l);
                if (ERROR_NONE != err) {
                    ROS_ERROR("Error reading SDI from left motor, EZW_ERR: SMCService : "
                              "Controller::getSafetyFunctionCommand() return error code : %d",
                              (int)err);
                }

                err = m_right_controller.getSafetyFunctionCommand(safety_fcn_r, res_r);
                if (ERROR_NONE != err) {
                    ROS_ERROR("Error reading SDI from right motor, EZW_ERR: SMCService : "
                              "Controller::getSafetyFunctionCommand() return error code : %d",
                              (int)err);
                }

                msg.safe_direction_indication_pos = !(res_r || res_l);

                // Reading SLS
                err = m_left_controller.getSafetyFunctionCommand(ezw::smccore::Controller::SafetyFunctionId::SLS_1, res_l);
                if (ERROR_NONE != err) {
                    ROS_ERROR("Error reading SLS from left motor, EZW_ERR: SMCService : "
                              "Controller::getSafetyFunctionCommand() return error code : %d",
                              (int)err);
                }

                err = m_right_controller.getSafetyFunctionCommand(ezw::smccore::Controller::SafetyFunctionId::SLS_1, res_r);
                if (ERROR_NONE != err) {
                    ROS_ERROR("Error reading SLS from right motor, EZW_ERR: SMCService : "
                              "Controller::getSafetyFunctionCommand() return error code : %d",
                              (int)err);
                }

                msg.safety_limited_speed = !(res_r || res_l);

#if VERBOSE_OUTPUT
                ROS_INFO("STO: %d, SDI+: %d, SLS: %d", msg.safe_torque_off, msg.safe_direction_indication_pos, msg.safety_limited_speed);
#endif

                m_safety_msg_mtx.lock();
                m_safety_msg = msg;
                m_safety_msg_mtx.unlock();

                m_pub_safety.publish(msg);
            } else {
                ROS_WARN("NMT state machine is not OK, no valid SafetyFunctions message to publish");
            }
#endif
        }

        ///
        /// \brief Callback qui s'active si aucun message de déplacement n'est reçu
        /// depuis m_watchdog_receive_ms
        ///
        void DiffDriveController::cbWatchdog()
        {
            setSpeeds(0, 0);
        }
    } // namespace swd
} // namespace ezw
