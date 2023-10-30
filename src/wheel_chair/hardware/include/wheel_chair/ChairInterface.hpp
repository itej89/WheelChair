#pragma once


#include <iostream>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

#include "geometry_msgs/msg/twist.hpp"

// timer
class ChairInterface : public rclcpp::Node
{
    public:
        std::shared_ptr<serial::Serial> base_port;
        ChairInterface(std::string nodeName);
        void cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber;
};