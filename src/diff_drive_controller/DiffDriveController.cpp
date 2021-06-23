/**
 * Copyright (C) 2020 EZ-Wheel S.A.S.
 *
 * @file DiffDriveController.cpp
 */

#include <diff_drive_controller/DiffDriveController.hpp>

#include <math.h>

#include <nav_msgs/Odometry.h>

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

namespace ezw {
    namespace diffdrivecontroller {
        DiffDriveController::DiffDriveController(const std::shared_ptr<ros::NodeHandle> nh)
            : m_nh(nh)
        {
            m_pubOdom       = m_nh->advertise<nav_msgs::Odometry>("odom", 5);
            m_pubJointState = m_nh->advertise<sensor_msgs::JointState>("joint_state", 5);
            m_subSetSpeed
                = m_nh->subscribe("set_speed", 5, &DiffDriveController::cbSetSpeed, this);

            ROS_INFO("Node name : %s", ros::this_node::getName().c_str());

            m_baseline_m          = m_nh->param("baseline_m", 0.0);
            m_pub_freq_hz         = m_nh->param("pub_freq_hz", 10);
            m_watchdog_receive_ms = m_nh->param("watchdog_receive_ms", 500);

            m_left_config_file  = m_nh->param("left_config_file", std::string(""));
            m_right_config_file = m_nh->param("right_config_file", std::string(""));

            if (m_baseline_m <= 0) {
                throw std::runtime_error(
                                         "baseline_m parameter is mandatory and must be > 0");
            }

            int lContextId = CON_APP; // Canaux de log, donc on s'en fout.

            ROS_INFO("Motors config files, right : %s, left : %s",
                     m_right_config_file.c_str(), m_left_config_file.c_str());

            ezw_error_t err_code;

            if ("" != m_right_config_file) {
                err_code = m_rightController.init(m_right_config_file);
                if (ezw_error_t::ERROR_NONE != err_code) {
                    ROS_ERROR("Right motor initialization error : %d", err_code);
                    return;
                }
            } else {
                ROS_ERROR("Please specify the right_config_file parameter");
                return;
            }

            if ("" != m_left_config_file) {
                err_code = m_leftController.init(m_left_config_file);
                if (ezw_error_t::ERROR_NONE != err_code) {
                    ROS_ERROR("Left motor initialization error : %d", err_code);
                    return;
                }
            } else {
                ROS_ERROR("Please specify the left_config_file parameter");
                return;
            }

            // Init
            err_code = m_leftController.getPositionValue(m_dLeft_prev); // en mm
            if (ezw_error_t::ERROR_NONE != err_code) {
                ROS_ERROR("Left motor, getPositionValue error: %d", err_code);
                return;
            }

            err_code = m_rightController.getPositionValue(m_dRight_prev); // en mm
            if (ERROR_NONE != err_code) {
                ROS_ERROR("Right motor, getPositionValue error: %d", err_code);
                return;
            }

            m_timerOdom
                = m_nh->createTimer(ros::Duration(1.0 / m_pub_freq_hz),
                                    boost::bind(&DiffDriveController::cbTimerOdom, this));
            m_timerWatchdog
                = m_nh->createTimer(ros::Duration(m_watchdog_receive_ms / 1000.0),
                                    boost::bind(&DiffDriveController::cbWatchdog, this));
        }

        void DiffDriveController::cbTimerOdom()
        {
            nav_msgs::Odometry      msgOdom;
            sensor_msgs::JointState msgJoint;

            // Toutes les longueurs sont en mètres
            int32_t dLeft_now = 0, dRight_now = 0;

            ezw_error_t err_code = m_leftController.getPositionValue(dLeft_now); // en mm
            if (ezw_error_t::ERROR_NONE != err_code) {
                ROS_ERROR("Left motor, getPositionValue error: %d", err_code);
                return;
            }

            err_code = m_rightController.getPositionValue(dRight_now); // en mm
            if (ezw_error_t::ERROR_NONE != err_code) {
                ROS_ERROR("Right motor, getPositionValue error: %d", err_code);
                return;
            }

            // Différence de l'odometrie entre t et t+1;
            double dLeft  = (dLeft_now - m_dLeft_prev) / 1000.0;
            double dRight = (dRight_now - m_dRight_prev) / 1000.0;

            // msg joint
            msgJoint.name.push_back("left_motor");
            msgJoint.name.push_back("right_motor");
            msgJoint.position.push_back(dLeft_now / 1000.0);
            msgJoint.position.push_back(dRight_now / 1000.0);
            msgJoint.velocity.push_back(dLeft / m_pub_freq_hz);
            msgJoint.velocity.push_back(dRight / m_pub_freq_hz);
            ros::Time timestamp = ros::Time::now();
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

            m_pubOdom.publish(msgOdom);
            m_pubJointState.publish(msgJoint);

            ROS_INFO("Odometry : dLeft : %f, dRight : %f\n Position x: %f, y: %f",
                     static_cast<double>(dLeft), static_cast<double>(dRight),
                     msgOdom.pose.pose.position.x, msgOdom.pose.pose.position.y);

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
            int32_t left  = static_cast<int32_t>(speed->x * 60.0 / (2.0 * M_PI));
            int32_t right = static_cast<int32_t>(speed->y * 60.0 / (2.0 * M_PI));

            ROS_INFO("Set speed : left -> %f rad/s (%d RPM) // right -> %f rad/s (%d RPM)",
                     speed->x, left, speed->y, right);

            ezw_error_t err_code = m_leftController.setTargetVelocity(left); // en rpm
            if (ezw_error_t::ERROR_NONE != err_code) {
                ROS_ERROR("Left motor, setTargetVelocity error: %d", err_code);
                return;
            }

            err_code = m_rightController.setTargetVelocity(right); // en mm
            if (ezw_error_t::ERROR_NONE != err_code) {
                ROS_ERROR("Right motor, setTargetVelocity error: %d", err_code);
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
}     // namespace ezw
