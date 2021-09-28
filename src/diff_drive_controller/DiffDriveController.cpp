/**
 * Copyright (C) 2020 EZ-Wheel S.A.S.
 *
 * @file DiffDriveController.cpp
 */

#include "diff_drive_controller/DiffDriveController.hpp"

#include "ezw-smc-core/CANOpenDispatcher.hpp"

#include "ezw-canopen-service/DBusClient.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

namespace ezw
{
    namespace diffdrivecontroller
    {
        DiffDriveController::DiffDriveController(const std::shared_ptr<ros::NodeHandle> nh) : m_nh(nh)
        {
            m_pub_odom        = m_nh->advertise<nav_msgs::Odometry>("odom", 5);
            m_pub_joint_state = m_nh->advertise<sensor_msgs::JointState>("joint_state", 5);

            ROS_INFO("Node name : %s", ros::this_node::getName().c_str());

            m_baseline_m          = m_nh->param("baseline_m", 0.0);
            m_pub_freq_hz         = m_nh->param("pub_freq_hz", 50);
            m_watchdog_receive_ms = m_nh->param("watchdog_receive_ms", 500);
            m_base_link           = m_nh->param("base_link", std::string("baselink"));
            m_odom_frame          = m_nh->param("odom_frame", std::string("odom"));
            std::string ctrl_mode = m_nh->param("control_mode", std::string("Twist"));

            if ("Twist" == ctrl_mode) {
                m_sub_command = m_nh->subscribe("cmd_vel", 5, &DiffDriveController::cbCmdVel, this);
            } else if ("LeftRightSpeeds" == ctrl_mode) {
                m_sub_command = m_nh->subscribe("set_speed", 5, &DiffDriveController::cbSetSpeed, this);
            } else {
                ROS_ERROR("Invalid value '%s' for parameter 'control_mode', accepted values: ['Twist' (default) or 'LeftRightSpeeds']",
                          crtl_mode);
                return;
            }

            m_left_config_file  = m_nh->param("left_config_file", std::string(""));
            m_right_config_file = m_nh->param("right_config_file", std::string(""));

            if (m_baseline_m <= 0) {
                ROS_ERROR("baseline_m parameter is mandatory and must be > 0");
                return;
            }

            if (m_pub_freq_hz <= 0) {
                ROS_ERROR("pub_freq_hz parameter is mandatory and must be > 0");
                return;
            }

            int lContextId = CON_APP; // Canaux de log, donc on s'en fout.

            ROS_INFO("Motors config files, right : %s, left : %s", m_right_config_file.c_str(), m_left_config_file.c_str());

            ezw_error_t lError;

            if ("" != m_right_config_file) {
                /* Config init */
                auto lConfig = std::make_shared<ezw::smccore::Config>();
                lError       = lConfig->load(m_right_config_file);
                if (lError != ERROR_NONE) {
                    ROS_ERROR("Failed loading right motor's config file <%s>, CONTEXT_ID: %d, EZW_ERR: %s", m_right_config_file.c_str(), CON_APP,
                              EZW_STR(("SMCService :  Config.init() return error code : " + std::to_string(lError)).c_str()));
                    return;
                }

                m_right_wheel_diameter_m = lConfig->getDiameter() * 1e-3;
                m_r_motor_reduction      = lConfig->getReduction();

                /* CANOpenService client init */
                auto lCOSClient = std::make_shared<ezw::canopenservice::DBusClient>();
                lError          = lCOSClient->init();
                if (lError != ERROR_NONE) {
                    ROS_ERROR("Failed initializing right motor, CONTEXT_ID: %d, EZW_ERR: %s", lConfig->getContextId(),
                              EZW_STR(("SMCService : COSDBusClient::init() return error code : " + std::to_string(lError)).c_str()));
                    return;
                }

                /* CANOpenDispatcher */
                auto lCANOpenDispatcher = std::make_shared<ezw::smccore::CANOpenDispatcher>(lConfig, lCOSClient);
                lError                  = lCANOpenDispatcher->init();
                if (lError != ERROR_NONE) {
                    ROS_ERROR("Failed initializing right motor, CONTEXT_ID: %d, EZW_ERR: %s", lConfig->getContextId(),
                              EZW_STR(("SMCService : CANOpenDispatcher::init() return error code : " + std::to_string(lError)).c_str()));
                    return;
                }

                lError = m_right_controller.init(lConfig, lCANOpenDispatcher);
                if (ezw_error_t::ERROR_NONE != lError) {
                    ROS_ERROR("Failed initializing right motor, CONTEXT_ID: %d, EZW_ERR: %s", lConfig->getContextId(),
                              EZW_STR(("SMCService : Controller::init() return error code : " + std::to_string(lError)).c_str()));
                    return;
                }
            } else {
                ROS_ERROR("Please specify the right_config_file parameter");
                return;
            }

            if ("" != m_left_config_file) {
                /* Config init */
                auto lConfig = std::make_shared<ezw::smccore::Config>();
                lError       = lConfig->load(m_left_config_file);
                if (lError != ERROR_NONE) {
                    ROS_ERROR("Failed loading left motor's config file <%s>, CONTEXT_ID: %d, EZW_ERR: %s", m_right_config_file.c_str(), CON_APP,
                              EZW_STR(("SMCService :  Config.init() return error code : " + std::to_string(lError)).c_str()));
                    return;
                }

                m_left_wheel_diameter_m = lConfig->getDiameter() * 1e-3;
                m_l_motor_reduction     = lConfig->getReduction();

                /* CANOpenService client init */
                auto lCOSClient = std::make_shared<ezw::canopenservice::DBusClient>();
                lError          = lCOSClient->init();
                if (lError != ERROR_NONE) {
                    ROS_ERROR("Failed initializing left motor, CONTEXT_ID: %d, EZW_ERR: %s", lConfig->getContextId(),
                              EZW_STR(("SMCService : COSDBusClient::init() return error code : " + std::to_string(lError)).c_str()));
                    return;
                }

                /* CANOpenDispatcher */
                auto lCANOpenDispatcher = std::make_shared<ezw::smccore::CANOpenDispatcher>(lConfig, lCOSClient);
                lError                  = lCANOpenDispatcher->init();
                if (lError != ERROR_NONE) {
                    ROS_ERROR("Failed initializing left motor, CONTEXT_ID: %d, EZW_ERR: %s", lConfig->getContextId(),
                              EZW_STR(("SMCService : CANOpenDispatcher::init() return error code : " + std::to_string(lError)).c_str()));
                    return;
                }

                lError = m_left_controller.init(lConfig, lCANOpenDispatcher);
                if (ezw_error_t::ERROR_NONE != lError) {
                    ROS_ERROR("Failed initializing left motor, CONTEXT_ID: %d, EZW_ERR: %s", lConfig->getContextId(),
                              EZW_STR(("SMCService : Controller::init() return error code : " + std::to_string(lError)).c_str()));
                    return;
                }
            } else {
                ROS_ERROR("Please specify the left_config_file parameter");
                return;
            }

            // Init
            lError = m_left_controller.getPositionValue(m_dist_left_prev); // en mm
            if (ezw_error_t::ERROR_NONE != lError) {
                ROS_ERROR("Failed initial reading from left motor, EZW_ERR: %s",
                          EZW_STR(("SMCService : Controller::getPositionValue() return error code : " + std::to_string(lError)).c_str()));
                    return;
            }

            lError = m_right_controller.getPositionValue(m_dist_right_prev); // en mm
            if (ezw_error_t::ERROR_NONE != lError) {
                ROS_ERROR("Failed initial reading from right motor, EZW_ERR: %s",
                          EZW_STR(("SMCService : Controller::getPositionValue() return error code : " + std::to_string(lError)).c_str()));
            }

            m_timer_odom     = m_nh->createTimer(ros::Duration(1.0 / m_pub_freq_hz), boost::bind(&DiffDriveController::cbTimerOdom, this));
            m_timer_watchdog = m_nh->createTimer(ros::Duration(m_watchdog_receive_ms / 1000.0), boost::bind(&DiffDriveController::cbWatchdog, this));
        }

        void DiffDriveController::cbTimerOdom()
        {
            nav_msgs::Odometry      msg_odom;
            sensor_msgs::JointState msg_joint_state;
            geometry_msgs::TransformStamped tf_odom_baselink;

            // Toutes les longueurs sont en mètres
            int32_t left_dist_now = 0, right_dist_now = 0;

            ezw_error_t lError = m_left_controller.getPositionValue(left_dist_now); // en mm
            if (ezw_error_t::ERROR_NONE != lError) {
                ROS_ERROR("Failed reading from left motor, EZW_ERR: %s",
                          EZW_STR(("SMCService : Controller::getPositionValue() return error code : " + std::to_string(lError)).c_str()));
                return;
            }

            lError = m_right_controller.getPositionValue(right_dist_now); // en mm
            if (ezw_error_t::ERROR_NONE != lError) {
                ROS_ERROR("Failed reading from right motor, EZW_ERR: %s",
                          EZW_STR(("SMCService : Controller::getPositionValue() return error code : " + std::to_string(lError)).c_str()));
                return;
            }

            // Différence de l'odometrie entre t et t+1;
            double d_dist_left  = (left_dist_now - m_dist_left_prev) / 1000.0;
            double d_dist_right = (right_dist_now - m_dist_right_prev) / 1000.0;

            ros::Time timestamp   = ros::Time::now();

            // msg joint
            msg_joint_state.name.push_back("left_motor");
            msg_joint_state.name.push_back("right_motor");
            msg_joint_state.position.push_back(left_dist_now / 1000.0);
            msg_joint_state.position.push_back(right_dist_now / 1000.0);
            msg_joint_state.velocity.push_back(d_dist_left / m_pub_freq_hz);
            msg_joint_state.velocity.push_back(d_dist_right / m_pub_freq_hz);
            msg_joint_state.header.stamp = timestamp;

            // Kinematic model
            double d_dist_center = (d_dist_left + d_dist_right) / 2.0;
            double d_theta      = (d_dist_right - d_dist_left) / m_baseline_m;

            // Odometry model, integration of the diff drive kinematic model
            double x_now     = m_x_prev + d_dist_center * std::cos(m_theta_prev);
            double y_now     = m_y_prev + d_dist_center * std::sin(m_theta_prev);
            double theta_now = boundAngle(m_theta_prev + d_theta);

            msg_odom.header.stamp         = timestamp;
            msg_odom.header.frame_id      = m_odom_frame;
            msg_odom.child_frame_id       = m_base_link;

            msg_odom.twist                = geometry_msgs::TwistWithCovariance();
            msg_odom.twist.linear.x       = d_dist_center / m_pub_freq_hz;
            msg_odom.twist.angular.z      = d_theta / m_pub_freq_hz;

            msg_odom.pose.pose.position.x = x_now;
            msg_odom.pose.pose.position.y = y_now;
            msg_odom.pose.pose.position.z = 0;

            tf2::Quaternion quat_orientation;
            quat_orientation.setRPY(0, 0, theta_now);
            msg_odom.pose.pose.orientation.x = quat_orientation.getX();
            msg_odom.pose.pose.orientation.y = quat_orientation.getY();
            msg_odom.pose.pose.orientation.x = quat_orientation.getZ();
            msg_odom.pose.pose.orientation.x = quat_orientation.getW();

            m_pub_odom.publish(msg_odom);
            m_pub_joint_state.publish(msg_joint_state);

            ROS_INFO("Odometry : d_dist_left : %f, d_dist_right : %f\n Position x: %f, y: %f",
                     static_cast<double>(d_dist_left), static_cast<double>(d_dist_right),
                     msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y);

            tf_odom_baselink.header.stamp    = timestamp;
            tf_odom_baselink.header.frame_id = m_odom_frame;
            tf_odom_baselink.child_frame_id  = m_base_link;

            tf_odom_baselink.transform.translation.x = msg_odom.pose.pose.position.x;
            tf_odom_baselink.transform.translation.y = msg_odom.pose.pose.position.y;
            tf_odom_baselink.transform.translation.z = msg_odom.pose.pose.position.z;
            tf_odom_baselink.transform.rotation      = msg_odom.pose.pose.orientation;

            // Send TF
            m_tf2_br.sendTransform(tf_odom_baselink);
            
            m_x_prev          = x_now;
            m_y_prev          = y_now;
            m_theta_prev      = theta_now;
            m_dist_left_prev  = left_dist_now;
            m_dist_right_prev = right_dist_now;
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

            ROS_INFO("Set speed : left -> %f rad/s (%d RPM) // right -> %f rad/s (%d RPM)", speed->x, left, speed->y, right);

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

            ROS_INFO("Cmd Vel : linear -> %f m/s // angular -> %f rad/s // Left RPM : %d // Right RPM : %d",
                     cmd_vel->linear.x, cmd_vel->angular.z, left, right);

            setSpeeds(left, right);
        }

        void DiffDriveController::setSpeeds(int32_t left_speed, int32_t right_speed) {
            ezw_error_t lError = m_left_controller.setTargetVelocity(left_speed); // en rpm
            if (ezw_error_t::ERROR_NONE != lError) {
                ROS_ERROR("Failed setting velocity of right motor, EZW_ERR: %s",
                          EZW_STR(("SMCService : Controller::setTargetVelocity() return error code : " + std::to_string(lError)).c_str()));
                return;
            }

            lError = m_right_controller.setTargetVelocity(right_speed); // en rpm
            if (ezw_error_t::ERROR_NONE != lError) {
                ROS_ERROR("Failed setting velocity of right motor, EZW_ERR: %s",
                          EZW_STR(("SMCService : Controller::setTargetVelocity() return error code : " + std::to_string(lError)).c_str()));
                return;
            }
        }

        ///
        /// \brief Callback qui s'active si aucun message de déplacement n'est reçu
        /// depuis m_watchdog_receive_ms
        ///
        void DiffDriveController::cbWatchdog()
        {
            setSpeeds(0, 0);
        }
    } // namespace diffdrivecontroller
} // namespace ezw
