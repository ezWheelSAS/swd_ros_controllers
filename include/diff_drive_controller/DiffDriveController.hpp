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
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <cmath>
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
            ros::Publisher                   m_pub_odom, m_pub_joint_state;
            ros::Subscriber                  m_sub_command;
            std::shared_ptr<ros::NodeHandle> m_nh;
            tf2_ros::TransformBroadcaster    m_tf2_br;

            // Param
            double      m_baseline_m, m_left_wheel_diameter_m, m_right_wheel_diameter_m, m_l_motor_reduction, m_r_motor_reduction;
            int         m_pub_freq_hz, m_watchdog_receive_ms, m_ref_wheel;
            std::string m_odom_frame, m_base_link, m_left_config_file, m_right_config_file;

            ros::Timer               m_timer_odom, m_timer_watchdog, m_timer_pds;
            ezw::smccore::Controller m_left_controller, m_right_controller;

            double  m_x_prev = 0, m_y_prev = 0, m_theta_prev = 0;
            int32_t m_dist_left_prev = 0, m_dist_right_prev = 0;

            void setSpeeds(int32_t left_speed, int32_t right_speed);
            void cbSetSpeed(const geometry_msgs::PointConstPtr &speed);
            void cbCmdVel(const geometry_msgs::TwistPtr &speed);
            void cbSoftBrake(const std_msgs::String::ConstPtr& msg);
            void cbTimerOdom(), cbWatchdog(), cbTimerPDS();

            inline double boundAngle(double a)
            {
                return (a > M_PI) ? a - 2. * M_PI : ((a < -M_PI) ? a + 2. * M_PI : a);
            }
        };
    } // namespace diffdrivecontroller
}     // namespace ezw

#endif /* EZW_ROSCONTROLLERS_DIFFDRIVECONTROLLER_HPP */
