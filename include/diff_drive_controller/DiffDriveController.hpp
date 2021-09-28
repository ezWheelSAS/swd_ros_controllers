/**
 * Copyright (C) 2020 EZ-Wheel S.A.S.
 *
 * @file DiffDriveController.hpp
 */

#ifndef EZW_ROSCONTROLLERS_DIFFDRIVECONTROLLER_HPP
#define EZW_ROSCONTROLLERS_DIFFDRIVECONTROLLER_HPP

/* SMC core */
#include "ezw-smc-core/Config.hpp"
#include "ezw-smc-core/Controller.hpp"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <ros/node_handle.h>
#include <ros/timer.h>

#include <tf2_ros/transform_broadcaster.h>

namespace ezw
{
    namespace diffdrivecontroller
    {
        /**
         * @brief Differential Drive Controller for ez-Wheel Gen2 wheels
         *        It subscribes to the `/node/set_speed` topic (of type
         * `geometry_msgs::Point`) The `x`, `y` components of the `/set_speed` message
         * represents respectively the left and right motor speed in (rad/s) The
         * controller publishes the odometry to `/node/odom` and the joint state to
         * `/node/joint_state`
         */

        class DiffDriveController {
          public:
            /**
             * @brief Class constructor
             * @param[in, out] nh ROS node handle
             */
            DiffDriveController(const std::shared_ptr<ros::NodeHandle> nh);

          private:
            ros::Publisher                   m_pubOdom, m_pubJointState;
            ros::Subscriber                  m_subCommand;
            std::shared_ptr<ros::NodeHandle> m_nh;
            tf2_ros::TransformBroadcaster    m_tf2_br;

            // Param
            double      m_baseline_m, m_left_wheel_diameter_m,
                        m_right_wheel_diameter_m, m_l_motor_reduction, m_r_motor_reduction;
            int         m_pub_freq_hz, m_watchdog_receive_ms;
            std::string m_odom_frame, m_base_link, m_left_config_file, m_right_config_file;

            ros::Timer               m_timerOdom, m_timerWatchdog;
            ezw::smccore::Controller m_leftController, m_rightController;

            double  m_x_prev = 0, m_y_prev = 0, m_theta_prev = 0;
            int32_t m_dLeft_prev = 0, m_dRight_prev = 0;

            void cbSetSpeed(const geometry_msgs::PointConstPtr &speed);
            void cbCmdVel(const geometry_msgs::TwistPtr &speed);
            void cbTimerOdom(), cbWatchdog();
        };
    } // namespace diffdrivecontroller
} // namespace ezw

#endif /* EZW_ROSCONTROLLERS_DIFFDRIVECONTROLLER_HPP */
