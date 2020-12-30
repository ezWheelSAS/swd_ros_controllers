/**
 * Copyright (C) 2020 EZ-Wheel S.A.S.
 *
 * @file main.cpp
 */

#include <diff_drive_controller/DiffDriveController.hpp>

#include <ros/console.h>
#include <ros/ros.h>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    ros::init(argc, argv,
              "DiffDriveController"); // Doit être fait avant le ros::master::check()
                                      // sinon on ne connait pas l'addresse du ros master

    // This driver is designed to run a wheels. Frequently wheels will have startup before
    // controlling computer. So this node will be launched before ROS Master. This is why
    // we wait for ros master here before starting the node.
    while (!ros::master::check()) {
        ROS_ERROR("Wait for master at %s", ros::master::getURI().c_str());
        std::this_thread::sleep_for(1s);
    }

    std::cout << "Ready !" << std::endl;

    auto                                          nh = std::make_shared<ros::NodeHandle>("~");
    ezw::diffdrivecontroller::DiffDriveController diffDriveController(nh);

    ros::spin();
}
