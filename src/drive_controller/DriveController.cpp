/**
 * Copyright (C) 2020 EZ-Wheel S.A.S.
 *
 * @file DriveController.cpp
 */

#include <drive_controller/DriveController.hpp>

#include <math.h>

#include <ros/duration.h>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

namespace ezw {
    namespace drivecontroller {
        DriveController::DriveController(const std::string &name,
                                         const std::string &config_file,
                                         ros::NodeHandle *node, int watchdogTimeout,
                                         int pubFreqHz)
            : m_nodeHandle(node), m_name(name), m_pub_freq_hz(pubFreqHz),
            m_watchdog_receive_ms(watchdogTimeout)
        {
            m_pubJointState = m_nodeHandle->advertise<sensor_msgs::JointState>(
                                                                               m_name + "/joint_state", 5);
            m_subSetSpeed = m_nodeHandle->subscribe(m_name + "/set_speed", 5,
                                                    &DriveController::cbSetSpeed, this);

            ROS_INFO("Motor name : %s", m_name.c_str());

            auto errorCode = m_motorController.init(config_file);

            ROS_INFO("Init return status : %d", errorCode);

            ezw_log_init("SMCS", "EZW SMC Service for Linux");
            m_timerOdom = m_nodeHandle->createTimer(
                                                    ros::Duration(1 / m_pub_freq_hz),
                                                    boost::bind(&DriveController::cbTimerOdom, this));
            m_timerWatchdog = m_nodeHandle->createTimer(
                                                        ros::Duration(m_watchdog_receive_ms / 1000.0),
                                                        boost::bind(&DriveController::cbWatchdog, this));
        }

        void DriveController::cbTimerOdom()
        {
            sensor_msgs::JointState msgJoint;

            int32_t pos_now;

            ezw_error_t state = m_motorController.getPositionValue(pos_now); // en mm
            
            if (ERROR_NONE != state) {
                ROS_ERROR("Controler::getPositionValue returned: %d", state);
                return;
            }

            // Différence de l'odometrie entre t-1 et t
            double dPos     = (pos_now - m_pos_prev) / 1000.0;
            double velocity = dPos / m_pub_freq_hz;

            // msg joint
            msgJoint.name.push_back(m_name);
            msgJoint.position.push_back(pos_now / 1000.0);
            msgJoint.velocity.push_back(velocity);
            msgJoint.header.stamp = ros::Time::now();

            m_pubJointState.publish(msgJoint);
            m_pos_prev = pos_now;
        }

///
/// \brief Change wheel speed (rad/s)
///
        void DriveController::cbSetSpeed(const std_msgs::Float64::Ptr msg)
        {
            m_timerWatchdog.stop();
            m_timerWatchdog.start();

            // Conversion rad/s en rpm
            int16_t speed = static_cast<int16_t>(msg->data * 60 / (2 * M_PI));

            ROS_INFO("Set speed : left -> %f (rad/s) | %i (RPM)", msg->data, speed);

            auto state = m_motorController.setTargetVelocity(speed); // RPM

            if (ERROR_NONE != state) {
                ROS_ERROR("Controler::setTargetVelocity returned: %d", state);
                return;
            }
        }

///
/// \brief Callback qui s'active si aucun message de déplacement n'est reçu
/// depuis m_watchdog_receive_ms
///
        void DriveController::cbWatchdog()
        {
            std_msgs::Float64::Ptr msg(new std_msgs::Float64);

            msg->data = 0;
            cbSetSpeed(msg);
        }
    } // namespace drivecontroller
}     // namespace ezw
