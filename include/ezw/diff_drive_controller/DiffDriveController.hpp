/**
 * Copyright (C) 2020 EZ-Wheel S.A.S.
 *
 * @file DiffDriveController.hpp
 */

#ifndef EZW_ROSCONTROLLERS_DIFFDRIVECONTROLLER_HPP
#define EZW_ROSCONTROLLERS_DIFFDRIVECONTROLLER_HPP

#include <ezw-smc-service/DBusClient.hpp>

#include <geometry_msgs/Point.h>

#include <ros/node_handle.h>
#include <ros/timer.h>

namespace ezw {
    namespace diffdrivecontroller
    {
        /**
         * @brief Differential Drive Controller for ez-Wheel Gen2 wheels
         *        It subscribes to the `/node/set_speed` topic (of type `geometry_msgs::Point`)
         *        The `x`, `y` components of the `/set_speed` message represents
         *        respectively the left and right motor speed in (rad/s)
         *        The controller publishes the odometry to `/node/odom` and the joint state to `/node/joint_state`
         */

        class DiffDriveController
        {
            public:
                /**
                 * @brief Class constructor
                 * @param[in, out] nh ROS node handle
                 */
                DiffDriveController(const std::shared_ptr<ros::NodeHandle> nh);

            private:
                ros::Publisher m_pubOdom, m_pubJointState;
                ros::Subscriber m_subSetSpeed;
                std::shared_ptr<ros::NodeHandle> m_nh;

                // Param
                double m_baseline_m;
                int m_pub_freq_hz, m_watchdog_receive_ms;
                std::string m_left_motor_name, m_right_motor_name;

                ros::Timer m_timerOdom, m_timerWatchdog;
                ezw::smcservice::DBusClient m_clientRight, m_clientLeft;
                double m_x_prev = 0, m_y_prev = 0, m_theta_prev = 0;
                bool m_rightMotorInitialized = false, m_leftMotorInitialized = false;
                int32_t m_dLeft_prev = 0, m_dRight_prev = 0;

                void cbSetSpeed(const geometry_msgs::PointConstPtr &speed);
                void cbTimerOdom(), cbWatchdog();
        };
    }
}

#endif /* EZW_ROSCONTROLLERS_DIFFDRIVECONTROLLER_HPP */
