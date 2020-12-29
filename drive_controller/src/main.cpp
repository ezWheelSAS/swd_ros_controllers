/**
 * Copyright (C) 2020 EZ-Wheel S.A.S.
 *
 * @file main.cpp
 */

#include <drive_controller/DriveController.hpp>

#include <ros/console.h>
#include <ros/ros.h>

using namespace std::chrono_literals;
using namespace ezw::drivecontroller;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "DriveController");
    while(!ros::master::check())
    {
        std::cout << "Wait for master" << std::endl;
        std::this_thread::sleep_for(1s);
    }

    std::cout << "Ready !" << std::endl;
    ros::NodeHandle nh("~");

    ROS_INFO("Node name : %s", ros::this_node::getName().c_str());

    int pub_freq_hz = nh.param("pub_freq_hz", 10);
    int watchdog_receive_ms = nh.param("watchdog_receive_ms", 500);

    std::vector<std::string> emptyList = {};
    std::vector<std::string> list_of_dbus_smc_services = nh.param("list_of_dbus_smc_services", emptyList);

    if(list_of_dbus_smc_services.empty())
    {
        ROS_ERROR("list_of_dbus_smc_services parameter is empty");
        exit(-1);
    }

    std::vector<std::shared_ptr<ezw::drivecontroller::DriveController>> listClients;

    for(const std::string& name : list_of_dbus_smc_services)
    {
        ROS_INFO("Name : %s", name.c_str());
        listClients.push_back(
            std::make_shared<ezw::drivecontroller::DriveController>(name, &nh, watchdog_receive_ms, pub_freq_hz));
    }
    ros::spin();
}
