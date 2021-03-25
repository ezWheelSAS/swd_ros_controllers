/**
 * Copyright (C) 2020 EZ-Wheel S.A.S.
 *
 * @file main.cpp
 */

#include <drive_controller/DriveController.hpp>
#include <iterator>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DriveController");
    while (!ros::master::check()) {
        std::cout << "Wait for master" << std::endl;
        std::this_thread::sleep_for(1s);
    }

    std::cout << "Ready !" << std::endl;
    ros::NodeHandle nh("~");

    ROS_INFO("Node name : %s", ros::this_node::getName().c_str());

    int pub_freq_hz         = nh.param("pub_freq_hz", 10);
    int watchdog_receive_ms = nh.param("watchdog_receive_ms", 500);

    // Map of {"motor name": "config file"}
    std::map<std::string, std::string> motors_dict;
    std::map<std::string, std::string> emptyMap;

    nh.getParam("motors_dict", motors_dict);

    if (motors_dict.empty()) {
        ROS_ERROR("motors_dict parameter is empty");
        exit(-1);
    }

    std::vector<std::shared_ptr<ezw::drivecontroller::DriveController>>
    listClients;

    for (auto const &iter : motors_dict) {
        ROS_INFO("Motor : %s -> Config file : %s", iter.first.c_str(),
                 iter.second.c_str());

        listClients.push_back(
                              std::make_shared<ezw::drivecontroller::DriveController>(
                                                                                      iter.first, iter.second, &nh, watchdog_receive_ms, pub_freq_hz));
    }

    ros::spin();
}
