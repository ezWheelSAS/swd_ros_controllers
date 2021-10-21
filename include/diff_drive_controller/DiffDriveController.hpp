/**
 * Copyright (C) 2021 ez-Wheel S.A.S.
 *
 * @file DiffDriveController.hpp
 */

#ifndef EZW_ROSCONTROLLERS_DIFFDRIVECONTROLLER_HPP
#define EZW_ROSCONTROLLERS_DIFFDRIVECONTROLLER_HPP

/* SMC core */
#include "ezw-smc-core/Config.hpp"
#include "ezw-smc-core/Controller.hpp"

#include <swd_ros_controllers/SafetyFunctions.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <cmath>
#include <mutex>
#include <ros/node_handle.h>
#include <ros/timer.h>

#include <tf2_ros/transform_broadcaster.h>

#define M_MAX(a, b)      ((a) > (b) ? (a) : (b))
#define M_MIN(a, b)      ((a) < (b) ? (a) : (b))
#define M_SIGN(a)        ((a) > 0 ? 1 : -1)
#define M_BOUND_ANGLE(a) (((a) > M_PI) ? ((a)-2. * M_PI) : (((a) < -M_PI) ? ((a) + 2. * M_PI) : (a)))

namespace ezw
{
    namespace swd
    {
        /**
         * @brief Differential Drive Controller for ez-Wheel Gen2 wheels
         *        It subscribes to the `/node/set_speed` or `/node/cmd_vel` topics:
         * - `/node/set_speed` of type `geometry_msgs::Point`: The `x`, `y`
         *   components of the `/set_speed` message
         *   represents respectively the left and right motor speed in (rad/s)
         * - `/node/cmd_vel` of type `geometry_msgs::Twist`: The linear and angular
         *   velocities.
         * The controller publishes the odometry to `/node/odom` and TFs, the safety
         * functions to `/node/safety`.
         */

        class DiffDriveController {
          public:
            /**
             * @brief Class constructor
             * @param[in, out] nh ROS node handle
             */
            DiffDriveController(const std::shared_ptr<ros::NodeHandle> nh);

          private:
            ros::Publisher                   m_pub_odom, m_pub_safety;
            ros::Subscriber                  m_sub_command, m_sub_brake;
            std::shared_ptr<ros::NodeHandle> m_nh;
            tf2_ros::TransformBroadcaster    m_tf2_br;

            // Param
            double      m_baseline_m, m_left_wheel_diameter_m, m_right_wheel_diameter_m, m_l_motor_reduction, m_r_motor_reduction;
            int         m_pub_freq_hz, m_watchdog_receive_ms, m_left_wheel_polarity, m_max_motor_speed_rpm, m_motor_sls_rpm;
            std::string m_odom_frame, m_base_frame, m_left_config_file, m_right_config_file;
            bool        m_have_backward_sls, m_publish_odom, m_publish_tf, m_publish_safety, m_nmt_ok, m_pds_ok;

            ros::Timer               m_timer_odom, m_timer_watchdog, m_timer_pds, m_timer_safety;
            ezw::smccore::Controller m_left_controller, m_right_controller;

            std::mutex                           m_safety_msg_mtx;
            swd_ros_controllers::SafetyFunctions m_safety_msg;

            double  m_x_prev = 0.0, m_y_prev = 0.0, m_theta_prev = 0.0;
            int32_t m_dist_left_prev_mm = 0, m_dist_right_prev_mm = 0;

            void setSpeeds(int32_t left_speed, int32_t right_speed);
            void cbSetSpeed(const geometry_msgs::PointConstPtr &speed);
            void cbCmdVel(const geometry_msgs::TwistPtr &speed);
            void cbSoftBrake(const std_msgs::Bool::ConstPtr &msg);
            void cbTimerOdom(), cbWatchdog(), cbTimerStateMachine(), cbTimerSafety();

            inline double boundAngle(double a)
            {
                return (a > M_PI) ? a - 2. * M_PI : ((a < -M_PI) ? a + 2. * M_PI : a);
            }
        };
    } // namespace swd
} // namespace ezw

#endif /* EZW_ROSCONTROLLERS_DIFFDRIVECONTROLLER_HPP */
