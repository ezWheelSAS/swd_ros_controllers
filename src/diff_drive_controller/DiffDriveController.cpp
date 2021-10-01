/**
 * Copyright (C) 2020 EZ-Wheel S.A.S.
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

namespace ezw
{
    namespace diffdrivecontroller
    {
        DiffDriveController::DiffDriveController(const std::shared_ptr<ros::NodeHandle> nh) : m_nh(nh)
        {
            m_pub_odom        = m_nh->advertise<nav_msgs::Odometry>("odom", 5);

            ROS_INFO("Node name : %s", ros::this_node::getName().c_str());

            m_baseline_m          = m_nh->param("baseline_m", 0.0);
            m_pub_freq_hz         = m_nh->param("pub_freq_hz", 50);
            m_watchdog_receive_ms = m_nh->param("watchdog_receive_ms", 1000);
            m_base_link           = m_nh->param("base_link", std::string("base_link"));
            m_odom_frame          = m_nh->param("odom_frame", std::string("odom"));
            m_ref_wheel           = m_nh->param("ref_wheel", -1);
            std::string ctrl_mode = m_nh->param("control_mode", std::string("Twist"));

            m_sub_brake = m_nh->subscribe("soft_brake", 5, &DiffDriveController::cbSoftBrake, this);

            if ("Twist" == ctrl_mode) {
                m_sub_command = m_nh->subscribe("cmd_vel", 5, &DiffDriveController::cbCmdVel, this);
            } else if ("LeftRightSpeeds" == ctrl_mode) {
                m_sub_command = m_nh->subscribe("set_speed", 5, &DiffDriveController::cbSetSpeed, this);
            } else {
                ROS_ERROR("Invalid value '%s' for parameter 'control_mode', accepted values: ['Twist' (default) or 'LeftRightSpeeds']", ctrl_mode.c_str());
                throw std::runtime_error("Invalid value for parameter 'control_mode'");
            }

            m_left_config_file  = m_nh->param("left_config_file", std::string(""));
            m_right_config_file = m_nh->param("right_config_file", std::string(""));

            if (m_baseline_m <= 0) {
                ROS_ERROR("baseline_m parameter is mandatory and must be > 0");
                throw std::runtime_error("baseline_m parameter is mandatory and must be > 0");
            }

            if (m_pub_freq_hz <= 0) {
                ROS_ERROR("pub_freq_hz parameter is mandatory and must be > 0");
                throw std::runtime_error("pub_freq_hz parameter is mandatory and must be > 0");
            }

            ROS_INFO("Motors config files, right : %s, left : %s", m_right_config_file.c_str(), m_left_config_file.c_str());

            ezw_error_t lError;

            if ("" != m_right_config_file) {
                /* Config init */
                auto lConfig = std::make_shared<ezw::smccore::Config>();
                lError = lConfig->load(m_right_config_file);
                if (lError != ERROR_NONE) {
                    ROS_ERROR("Failed loading right motor's config file <%s>, CONTEXT_ID: %d, EZW_ERR: SMCService : Config.init() return error code : %d",
                              m_right_config_file.c_str(), CON_APP, (int)lError);
                    throw std::runtime_error("Failed loading right motor's config file");
                }

                m_right_wheel_diameter_m = lConfig->getDiameter() * 1e-3;
                m_r_motor_reduction      = lConfig->getReduction();

                /* CANOpenService client init */
                auto lCOSClient = std::make_shared<ezw::canopenservice::DBusClient>();
                lError = lCOSClient->init();
                if (lError != ERROR_NONE) {
                    ROS_ERROR("Failed initializing right motor, CONTEXT_ID: %d, EZW_ERR: SMCService : COSDBusClient::init() return error code : %d",
                              lConfig->getContextId(), (int)lError);
                    throw std::runtime_error("Failed initializing right motor");
                }

                /* CANOpenDispatcher */
                auto lCANOpenDispatcher = std::make_shared<ezw::smccore::CANOpenDispatcher>(lConfig, lCOSClient);
                lError = lCANOpenDispatcher->init();
                if (lError != ERROR_NONE) {
                   ROS_ERROR("Failed initializing right motor, CONTEXT_ID: %d, EZW_ERR: SMCService : CANOpenDispatcher::init() return error code : %d",
                              lConfig->getContextId(), (int)lError);
                   throw std::runtime_error("Failed initializing right motor");
                }

                lError = m_right_controller.init(lConfig, lCANOpenDispatcher);
                if (ERROR_NONE != lError) {
                    ROS_ERROR("Failed initializing right motor, EZW_ERR: SMCService : Controller::init() return error code : %d", (int)lError);
                    throw std::runtime_error("Failed initializing right motor");
                }
            } else {
                ROS_ERROR("Please specify the right_config_file parameter");
                throw std::runtime_error("Please specify the right_config_file parameter");
            }

            if ("" != m_left_config_file) {
                /* Config init */
                auto lConfig = std::make_shared<ezw::smccore::Config>();
                lError = lConfig->load(m_left_config_file);
                if (lError != ERROR_NONE) {
                    ROS_ERROR("Failed loading left motor's config file <%s>, CONTEXT_ID: %d, EZW_ERR: SMCService : Config.init() return error code : %d",
                              m_right_config_file.c_str(), CON_APP, (int)lError);
                    throw std::runtime_error("Failed initializing left motor");
                }

                m_left_wheel_diameter_m = lConfig->getDiameter() * 1e-3;
                m_l_motor_reduction     = lConfig->getReduction();

                /* CANOpenService client init */
                auto lCOSClient = std::make_shared<ezw::canopenservice::DBusClient>();
                lError = lCOSClient->init();
                if (lError != ERROR_NONE) {
                    ROS_ERROR("Failed initializing left motor, EZW_ERR: SMCService : COSDBusClient::init() return error code : %d", (int)lError);
                    throw std::runtime_error("Failed initializing left motor");
                }

                /* CANOpenDispatcher */
                auto lCANOpenDispatcher = std::make_shared<ezw::smccore::CANOpenDispatcher>(lConfig, lCOSClient);
                lError = lCANOpenDispatcher->init();
                if (lError != ERROR_NONE) {
                    ROS_ERROR("Failed initializing left motor, EZW_ERR: SMCService : CANOpenDispatcher::init() return error code : %d", (int)lError);
                    throw std::runtime_error("Failed initializing left motor");
                }

                lError = m_left_controller.init(lConfig, lCANOpenDispatcher);
                if (ERROR_NONE != lError) {
                    ROS_ERROR("Failed initializing left motor, EZW_ERR: SMCService : Controller::init() return error code : %d", (int)lError);
                    throw std::runtime_error("Failed initializing left motor");
                }
            } else {
                ROS_ERROR("Please specify the left_config_file parameter");
                throw std::runtime_error("Please specify the left_config_file parameter");
            }

            // Init
            lError = m_left_controller.getPositionValue(m_dist_left_prev); // en mm
            if (ERROR_NONE != lError) {
                ROS_ERROR("Failed initial reading from left motor, EZW_ERR: SMCService : Controller::getPositionValue() return error code : %d", (int)lError);
                throw std::runtime_error("Failed initial reading from left motor");
            }

            lError = m_right_controller.getPositionValue(m_dist_right_prev); // en mm
            if (ERROR_NONE != lError) {
                ROS_ERROR("Failed initial reading from right motor, EZW_ERR: SMCService : Controller::getPositionValue() return error code : %d", (int)lError);
                throw std::runtime_error("Failed initial reading from right motor");
            }

            m_timer_odom     = m_nh->createTimer(ros::Duration(1.0 / m_pub_freq_hz), boost::bind(&DiffDriveController::cbTimerOdom, this));
            m_timer_watchdog = m_nh->createTimer(ros::Duration(m_watchdog_receive_ms / 1000.0), boost::bind(&DiffDriveController::cbWatchdog, this));
            m_timer_pds      = m_nh->createTimer(ros::Duration(1), boost::bind(&DiffDriveController::cbTimerPDS, this));
        }

        void DiffDriveController::cbTimerPDS()
        {
            smccore::IService::PDSState lPDSState = smccore::IService::PDSState::SWITCH_ON_DISABLED;
            ezw_error_t lError = m_left_controller.getPDSState(lPDSState);
            if (ERROR_NONE != lError) {
                ROS_ERROR("Failed to get the PDSState for left motor, EZW_ERR: SMCService : Controller::getPDSState() return error code : %d", (int)lError);
            }

            if (lPDSState != smccore::IService::PDSState::OPERATION_ENABLED) {
                m_left_controller.enterInOperationEnabledState();
            }

            lError = m_right_controller.getPDSState(lPDSState);
            if (ERROR_NONE != lError) {
                ROS_ERROR("Failed to get the PDSState for right motor, EZW_ERR: SMCService : Controller::getPDSState() return error code : %d", (int)lError);
            }

            if (lPDSState != smccore::IService::PDSState::OPERATION_ENABLED) {
                m_right_controller.enterInOperationEnabledState();
            }
        }

        void DiffDriveController::cbSoftBrake(const std_msgs::String::ConstPtr& msg)
        {
            // "enable" or something else -> Stop
            // "disable" -> Release
            bool halt = ("disable" == msg->data) ? false : true;

            ezw_error_t lError = m_left_controller.setHalt(halt);
            if (ERROR_NONE != lError) {
                ROS_ERROR("SoftBrake: Failed %s left wheel, EZW_ERR: %d", halt ? "braking" : "releasing", (int)lError);
            }

            lError = m_right_controller.setHalt(halt);
            if (ERROR_NONE != lError) {
                ROS_ERROR("SoftBrake: Failed %s right wheel, EZW_ERR: %d", halt ? "braking" : "releasing", (int)lError);
            }
        }

        void DiffDriveController::cbTimerOdom()
        {
            nav_msgs::Odometry              msg_odom;
            geometry_msgs::TransformStamped tf_odom_baselink;

            // Toutes les longueurs sont en mètres
            int32_t left_dist_now = 0, right_dist_now = 0;

            ezw_error_t lError = m_left_controller.getPositionValue(left_dist_now); // en mm
            if (ERROR_NONE != lError) {
                ROS_ERROR("Failed reading from left motor, EZW_ERR: SMCService : Controller::getPositionValue() return error code : %d", (int)lError);
                return;
            }

            lError = m_right_controller.getPositionValue(right_dist_now); // en mm
            if (ERROR_NONE != lError) {
                ROS_ERROR("Failed reading from right motor, EZW_ERR: SMCService : Controller::getPositionValue() return error code : %d", (int)lError);
                return;
            }

            // Différence de l'odometrie entre t et t+1;
            double d_dist_left  = (left_dist_now - m_dist_left_prev) / 1000.0;
            double d_dist_right = (right_dist_now - m_dist_right_prev) / 1000.0;

            ros::Time timestamp = ros::Time::now();

            // Kinematic model
            double d_dist_center = (d_dist_left + d_dist_right) / 2.0;
            double d_theta       = m_ref_wheel * (d_dist_right - d_dist_left) / m_baseline_m;

            // Odometry model, integration of the diff drive kinematic model
            double x_now     = m_x_prev + d_dist_center * std::cos(m_theta_prev);
            double y_now     = m_y_prev + d_dist_center * std::sin(m_theta_prev);
            double theta_now = boundAngle(m_theta_prev + d_theta);

            msg_odom.header.stamp    = timestamp;
            msg_odom.header.frame_id = m_odom_frame;
            msg_odom.child_frame_id  = m_base_link;

            msg_odom.twist                 = geometry_msgs::TwistWithCovariance();
            msg_odom.twist.twist.linear.x  = d_dist_center / m_pub_freq_hz;
            msg_odom.twist.twist.angular.z = d_theta / m_pub_freq_hz;

            msg_odom.pose.pose.position.x = x_now;
            msg_odom.pose.pose.position.y = y_now;
            msg_odom.pose.pose.position.z = 0;

            tf2::Quaternion quat_orientation;
            quat_orientation.setRPY(0, 0, theta_now);
            msg_odom.pose.pose.orientation.x = quat_orientation.getX();
            msg_odom.pose.pose.orientation.y = quat_orientation.getY();
            msg_odom.pose.pose.orientation.z = quat_orientation.getZ();
            msg_odom.pose.pose.orientation.w = quat_orientation.getW();

            m_pub_odom.publish(msg_odom);

            tf_odom_baselink.header.stamp    = timestamp;
            tf_odom_baselink.header.frame_id = m_odom_frame;
            tf_odom_baselink.child_frame_id  = m_base_link;

            tf_odom_baselink.transform.translation.x = msg_odom.pose.pose.position.x;
            tf_odom_baselink.transform.translation.y = msg_odom.pose.pose.position.y;
            tf_odom_baselink.transform.translation.z = msg_odom.pose.pose.position.z;
            tf_odom_baselink.transform.rotation.x = msg_odom.pose.pose.orientation.x;
            tf_odom_baselink.transform.rotation.y = msg_odom.pose.pose.orientation.y;
            tf_odom_baselink.transform.rotation.z = msg_odom.pose.pose.orientation.z;
            tf_odom_baselink.transform.rotation.w = msg_odom.pose.pose.orientation.w;

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

            ROS_INFO("Cmd Vel : linear -> %f m/s // angular -> %f rad/s // Left RPM : %d // Right RPM : %d", cmd_vel->linear.x, cmd_vel->angular.z, left, right);

            setSpeeds(left, right);
        }

        ///
        /// \brief Change robot velocity (left in rpm, right in rpm)
        ///
        void DiffDriveController::setSpeeds(int32_t left_speed, int32_t right_speed)
        {
            ezw_error_t lError = m_left_controller.setTargetVelocity(left_speed); // en rpm
            if (ERROR_NONE != lError) {
                ROS_ERROR("Failed setting velocity of right motor, EZW_ERR: SMCService : Controller::setTargetVelocity() return error code : %d", (int)lError);
                return;
            }

            lError = m_right_controller.setTargetVelocity(right_speed); // en rpm
            if (ERROR_NONE != lError) {
                ROS_ERROR("Failed setting velocity of right motor, EZW_ERR: SMCService : Controller::setTargetVelocity() return error code : %d", (int)lError);
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
}     // namespace ezw
