/**
 * Copyright (C) 2020 EZ-Wheel S.A.S.
 *
 * @file DriveController.hpp
 */

#ifndef EZW_ROSCONTROLLERS_DRIVECONTROLLER_HPP
#define EZW_ROSCONTROLLERS_DRIVECONTROLLER_HPP

/* SMC core */
#include "ezw-smc-core/Config.hpp"
#include "ezw-smc-core/Controller.hpp"

#include <std_msgs/Float64.h>

#include <ros/node_handle.h>
#include <ros/timer.h>

namespace ezw
{
    namespace drivecontroller
    {
        /**
         * @brief Single Drive Controller for ez-Wheel Gen2 wheels
         */
        class DriveController {
          public:
            /**
             * @brief Class constructor
             *
             * @param[in] name Motor's name, it will be used for publishing/subscribing to
             * topics, each motor will subscribes to `/node/motor_name/set_speed` and
             * publishes to the `/node/motor_name/joint_state` topic.
             * @param[in, out] node Refernce to a ROS node handle, the same node handle
             * can be used for multiple DriveContoller instances.
             * @param[in] watchdodTimeout A timeout (in milliseconds), if no setpoint is
             * received after this timeout, the wheel stops.
             * @param[in] pubFreqHz Publishing frequency in Hz.
             */
            DriveController(const std::string &name, const std::string &config_file, ros::NodeHandle *node, int watchdogTimeout, int pubFreqHz);

          private:
            ezw::smccore::Controller         m_motorController;
            std::shared_ptr<ros::NodeHandle> m_nodeHandle;
            const std::string &              m_name;
            ros::Publisher                   m_pubJointState;
            ros::Subscriber                  m_subSetSpeed;
            int                              m_pos_prev;
            ros::Timer                       m_timerOdom, m_timerWatchdog;
            int                              m_pub_freq_hz, m_watchdog_receive_ms;

            void cbSetSpeed(std_msgs::Float64::Ptr speed);
            void cbTimerOdom();
            void cbWatchdog();
        };
    } // namespace drivecontroller
} // namespace ezw

#endif /* EZW_ROSCONTROLLERS_DRIVECONTROLLER_HPP */
