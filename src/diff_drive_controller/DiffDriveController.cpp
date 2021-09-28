/**
 * Copyright (C) 2020 EZ-Wheel S.A.S.
 *
 * @file DiffDriveController.cpp
 */

#include "diff_drive_controller/DiffDriveController.hpp"

#include "ezw-smc-core/CANOpenDispatcher.hpp"

#include "ezw-canopen-service/DBusClient.hpp"

#include <math.h>

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
            m_pubOdom       = m_nh->advertise<nav_msgs::Odometry>("odom", 5);
            m_pubJointState = m_nh->advertise<sensor_msgs::JointState>("joint_state", 5);
            m_subSetSpeed   = m_nh->subscribe("set_speed", 5, &DiffDriveController::cbSetSpeed, this);
            m_subCmdVel     = m_nh->subscribe("cmd_vel", 5, &DiffDriveController::cbCmdVel, this);

            ROS_INFO("Node name : %s", ros::this_node::getName().c_str());

            m_baseline_m          = m_nh->param("baseline_m", 0.0);
            m_pub_freq_hz         = m_nh->param("pub_freq_hz", 10);
            m_watchdog_receive_ms = m_nh->param("watchdog_receive_ms", 500);
            m_base_link           = m_nh->param("base_link", std::string("baselink"));
            m_odom_frame          = m_nh->param("odom_frame", std::string("odom"));

            m_left_config_file  = m_nh->param("left_config_file", std::string(""));
            m_right_config_file = m_nh->param("right_config_file", std::string(""));

            if (m_baseline_m <= 0) {
                throw std::runtime_error("baseline_m parameter is mandatory and must be > 0");
            }

            if (m_wheel_diameter_m <= 0) {
                throw std::runtime_error("baseline_m parameter is mandatory and must be > 0");
            }

            int lContextId = CON_APP; // Canaux de log, donc on s'en fout.

            /* Config init */

            ROS_INFO("Motors config files, right : %s, left : %s", m_right_config_file.c_str(), m_left_config_file.c_str());

            ezw_error_t lError;

            if ("" != m_right_config_file) {
                /* Config init */
                auto lConfig = std::make_shared<ezw::smccore::Config>();
                lError       = lConfig->load(m_right_config_file);
                if (lError != ERROR_NONE) {
                    EZW_LOG(CON_APP, EZW_LOG_ERROR, EZW_STR(("SMCService :  Config.init() return error code : " + std::to_string(lError)).c_str()));

                    return;
                }

                m_right_wheel_diameter_m = lConfig.getDiameter() * 1e-3;
                m_r_motor_reduction = lConfig.getReduction();

                /* CANOpenService client init */
                auto lCOSClient = std::make_shared<ezw::canopenservice::DBusClient>();
                lError          = lCOSClient->init();
                if (lError != ERROR_NONE) {
                    EZW_LOG(lConfig->getContextId(), EZW_LOG_ERROR, EZW_STR(("SMCService : COSDBusClient::init() return error code : " + std::to_string(lError)).c_str()));
                    return;
                }

                /* CANOpenDispatcher */
                auto lCANOpenDispatcher = std::make_shared<ezw::smccore::CANOpenDispatcher>(lConfig, lCOSClient);
                lError                  = lCANOpenDispatcher->init();
                if (lError != ERROR_NONE) {
                    EZW_LOG(lConfig->getContextId(), EZW_LOG_ERROR, EZW_STR(("SMCService : CANOpenDispatcher::init() return error code : " + std::to_string(lError)).c_str()));

                    return;
                }

                lError = m_rightController.init(lConfig, lCANOpenDispatcher);
                if (ezw_error_t::ERROR_NONE != lError) {
                    ROS_ERROR("Right motor initialization error : %d", lError);
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
                    EZW_LOG(CON_APP, EZW_LOG_ERROR, EZW_STR(("SMCService :  Config.init() return error code : " + std::to_string(lError)).c_str()));

                    return;
                }

                m_left_wheel_diameter_m = lConfig.getDiameter() * 1e-3;
                m_l_motor_reduction = lConfig.getReduction();

                /* CANOpenService client init */
                auto lCOSClient = std::make_shared<ezw::canopenservice::DBusClient>();
                lError          = lCOSClient->init();
                if (lError != ERROR_NONE) {
                    EZW_LOG(lConfig->getContextId(), EZW_LOG_ERROR, EZW_STR(("SMCService : COSDBusClient::init() return error code : " + std::to_string(lError)).c_str()));
                    return;
                }

                /* CANOpenDispatcher */
                auto lCANOpenDispatcher = std::make_shared<ezw::smccore::CANOpenDispatcher>(lConfig, lCOSClient);
                lError                  = lCANOpenDispatcher->init();
                if (lError != ERROR_NONE) {
                    EZW_LOG(lConfig->getContextId(), EZW_LOG_ERROR, EZW_STR(("SMCService : CANOpenDispatcher::init() return error code : " + std::to_string(lError)).c_str()));

                    return;
                }

                lError = m_leftController.init(lConfig, lCANOpenDispatcher);
                if (ezw_error_t::ERROR_NONE != lError) {
                    ROS_ERROR("Left motor initialization error : %d", lError);
                    return;
                }
            } else {
                ROS_ERROR("Please specify the left_config_file parameter");
                return;
            }

            // Init
            lError = m_leftController.getPositionValue(m_dLeft_prev); // en mm
            if (ezw_error_t::ERROR_NONE != lError) {
                ROS_ERROR("Left motor, getPositionValue error: %d", lError);
                return;
            }

            lError = m_rightController.getPositionValue(m_dRight_prev); // en mm
            if (ezw_error_t::ERROR_NONE != lError) {
                ROS_ERROR("Right motor, getPositionValue error: %d", lError);
                return;
            }

            m_timerOdom     = m_nh->createTimer(ros::Duration(1.0 / m_pub_freq_hz), boost::bind(&DiffDriveController::cbTimerOdom, this));
            m_timerWatchdog = m_nh->createTimer(ros::Duration(m_watchdog_receive_ms / 1000.0), boost::bind(&DiffDriveController::cbWatchdog, this));
        }

        void DiffDriveController::cbTimerOdom()
        {
            nav_msgs::Odometry      msgOdom;
            sensor_msgs::JointState msgJoint;

            // Toutes les longueurs sont en mètres
            int32_t dLeft_now = 0, dRight_now = 0;

            ezw_error_t lError = m_leftController.getPositionValue(dLeft_now); // en mm
            if (ezw_error_t::ERROR_NONE != lError) {
                ROS_ERROR("Left motor, getPositionValue error: %d", lError);
                return;
            }

            lError = m_rightController.getPositionValue(dRight_now); // en mm
            if (ezw_error_t::ERROR_NONE != lError) {
                ROS_ERROR("Right motor, getPositionValue error: %d", lError);
                return;
            }

            // Différence de l'odometrie entre t et t+1;
            double dLeft  = (dLeft_now - m_dLeft_prev) / 1000.0;
            double dRight = (dRight_now - m_dRight_prev) / 1000.0;

            ros::Time timestamp   = ros::Time::now();

            // msg joint
            msgJoint.name.push_back("left_motor");
            msgJoint.name.push_back("right_motor");
            msgJoint.position.push_back(dLeft_now / 1000.0);
            msgJoint.position.push_back(dRight_now / 1000.0);
            msgJoint.velocity.push_back(dLeft / m_pub_freq_hz);
            msgJoint.velocity.push_back(dRight / m_pub_freq_hz);
            msgJoint.header.stamp = timestamp;

            double dCenter = (dLeft + dRight) / 2.0;
            double phi     = (dRight - dLeft) / m_baseline_m;

            double x_now     = m_x_prev + dCenter * std::cos(m_theta_prev);
            double y_now     = m_y_prev + dCenter * std::sin(m_theta_prev);
            double theta_now = m_theta_prev + phi;

            msgOdom.twist                = geometry_msgs::TwistWithCovariance();
            msgOdom.pose.pose.position.x = x_now;
            msgOdom.pose.pose.position.y = y_now;
            msgOdom.pose.pose.position.z = 0;

            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, theta_now);
            msgOdom.pose.pose.orientation.x = quaternion.getX();
            msgOdom.pose.pose.orientation.y = quaternion.getY();
            msgOdom.pose.pose.orientation.x = quaternion.getZ();
            msgOdom.pose.pose.orientation.x = quaternion.getW();
            msgOdom.header.stamp            = timestamp;
            msgOdom.header.frame_id         = m_odom_link;

            m_pubOdom.publish(msgOdom);
            m_pubJointState.publish(msgJoint);

            ROS_INFO("Odometry : dLeft : %f, dRight : %f\n Position x: %f, y: %f",
                     static_cast<double>(dLeft), static_cast<double>(dRight),
                     msgOdom.pose.pose.position.x, msgOdom.pose.pose.position.y);

            geometry_msgs::TransformStamped tf_odom_baselink;
            .tf_odom_baselinkheader.stamp = timestamp;
            .tf_odom_baselinkheader.frame_id = m_odom_frame;
            .tf_odom_baselinkchild_frame_id = m_base_link;

            .tf_odom_baselinktransform.translation.x = msgOdom.pose.pose.position.x;
            .tf_odom_baselinktransform.translation.y = msgOdom.pose.pose.position.y;
            .tf_odom_baselinktransform.translation.z = msgOdom.pose.pose.position.z;
            .tf_odom_baselinktransform.rotation = msgOdom.pose.pose.orientation;

            m_tf2_br.sendTransform(tf_odom_baselink);
            
            m_x_prev      = x_now;
            m_y_prev      = y_now;
            m_theta_prev  = theta_now;
            m_dLeft_prev  = dLeft_now;
            m_dRight_prev = dRight_now;
        }

        ///
        /// \brief Change wheel speed (msg.x = left motor, msg.y = right motor) rad/s
        ///
        void DiffDriveController::cbSetSpeed(const geometry_msgs::PointConstPtr &speed)
        {
            m_timerWatchdog.stop();
            m_timerWatchdog.start();

            // Conversion rad/s en rpm
            int32_t left  = static_cast<int32_t>(speed->x * m_l_motor_reduction * 60.0 / (2.0 * M_PI));
            int32_t right = static_cast<int32_t>(speed->y * m_l_motor_reduction * 60.0 / (2.0 * M_PI));

            ROS_INFO("Set speed : left -> %f rad/s (%d RPM) // right -> %f rad/s (%d RPM)", speed->x, left, speed->y, right);

            ezw_error_t lError = m_leftController.setTargetVelocity(left); // en rpm
            if (ezw_error_t::ERROR_NONE != lError) {
                ROS_ERROR("Left motor, setTargetVelocity error: %d", lError);
                return;
            }

            lError = m_rightController.setTargetVelocity(right); // en mm
            if (ezw_error_t::ERROR_NONE != lError) {
                ROS_ERROR("Right motor, setTargetVelocity error: %d", lError);
                return;
            }
        }

        ///
        /// \brief Change robot velocity (linear m/s, angular rad/s)
        ///
        void DiffDriveController::cbCmdVel(const geometry_msgs::TwistPtr &cmd_vel)
        {
            m_timerWatchdog.stop();
            m_timerWatchdog.start();

            double left_vel, right_vel;

            left_vel = (2. * cmd_vel->linear.x - cmd_vel->angular.z * m_baseline_m) / m_left_wheel_diameter;
            right_vel = (2. * cmd_vel->linear.x + cmd_vel->angular.z * m_baseline_m) / m_right_wheel_diameter;

            // Conversion rad/s en rpm
            int32_t left  = static_cast<int32_t>(left_vel * m_l_motor_reduction * 60.0 / (2.0 * M_PI));
            int32_t right = static_cast<int32_t>(right_vel * m_r_motor_reduction * 60.0 / (2.0 * M_PI));

            ROS_INFO("Cmd Vel : linear -> %f m/s (%d RPM) // angular -> %f rad/s (%d RPM)", cmd_vel->linear.x, left, cmd_vel->angular.z, right);

            ezw_error_t lError = m_leftController.setTargetVelocity(left); // en rpm
            if (ezw_error_t::ERROR_NONE != lError) {
                ROS_ERROR("Left motor, setTargetVelocity error: %d", lError);
                return;
            }

            lError = m_rightController.setTargetVelocity(right); // en mm
            if (ezw_error_t::ERROR_NONE != lError) {
                ROS_ERROR("Right motor, setTargetVelocity error: %d", lError);
                return;
            }
        }

        ///
        /// \brief Callback qui s'active si aucun message de déplacement n'est reçu
        /// depuis m_watchdog_receive_ms
        ///
        void DiffDriveController::cbWatchdog()
        {
            geometry_msgs::PointPtr msg(new geometry_msgs::Point);

            msg->x = 0;
            msg->y = 0;
            msg->z = 0;
            cbSetSpeed(msg);
        }
    } // namespace diffdrivecontroller
} // namespace ezw
