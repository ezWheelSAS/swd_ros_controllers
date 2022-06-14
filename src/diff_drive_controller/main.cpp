/**
 * Copyright (C) 2021 ez-Wheel S.A.S.
 *
 * @file main.cpp
 */

#include <ros/console.h>
#include <ros/ros.h>

#include <cstdlib>
#include <diff_drive_controller/DiffDriveController.hpp>

using namespace std::chrono_literals;

auto main(int argc, char **argv) -> int
{
    ros::init(argc, argv, "DiffDriveController");

    // Wait for ROS master before starting the node.
    while (!ros::master::check()) {
        ROS_ERROR("Waiting for ROS master at %s", ros::master::getURI().c_str());
        std::this_thread::sleep_for(1s);
    }

    ROS_INFO("Ready !");

    auto nh = std::make_shared<ros::NodeHandle>("~");
    try {
        ezw::swd::DiffDriveController diff_drive_controller(nh);
        ros::spin();
    }
    catch (std::runtime_error &err) {
        ROS_ERROR("FATAL ERROR, exception '%s'", err.what());
        std::exit(EXIT_FAILURE);
    }
}
