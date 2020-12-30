/**
 * Copyright (C) 2020 EZ-Wheel S.A.S.
 *
 * @file DriveController.hpp
 */

#ifndef EZW_ROSCONTROLLERS_DRIVECONTROLLER_HPP
#define EZW_ROSCONTROLLERS_DRIVECONTROLLER_HPP

#include "ezw-smc-service/DBusClient.hpp"

#include <std_msgs/Float64.h>

#include <ros/node_handle.h>
#include <ros/timer.h>

namespace ezw {
    namespace drivecontroller
    {
        class DriveController
        {
            public:
                DriveController(const std::string &name, ros::NodeHandle *node, int watchdogTimeout, int pubFreqHz);

            private:
                ezw::smcservice::DBusClient m_dbusClient;
                std::shared_ptr<ros::NodeHandle> m_nodeHandle;
                const std::string &m_name;
                ros::Publisher m_pubJointState;
                ros::Subscriber m_subSetSpeed;
                int m_pos_prev;
                ros::Timer m_timerOdom, m_timerWatchdog;
                int m_pub_freq_hz, m_watchdog_receive_ms;

                void cbSetSpeed(std_msgs::Float64::Ptr speed);
                void cbTimerOdom();
                void cbWatchdog();
        };
    }
}

#endif /* EZW_ROSCONTROLLERS_DRIVECONTROLLER_HPP */
