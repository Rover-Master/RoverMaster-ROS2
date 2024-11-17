#pragma once

#include <chrono> // IWYU pragma: keep

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;
using time_point = std::chrono::system_clock::time_point;
// ROS2 Node
using Node = rclcpp::Node;
// ROS2 Timer
using Timer = rclcpp::TimerBase;
// Message Types
using Odometry = nav_msgs::msg::Odometry;
using Twist = geometry_msgs::msg::Twist;
using Imu = sensor_msgs::msg::Imu;
using Bool = std_msgs::msg::Bool;
// ROS Time
namespace rclcpp {
static inline const Time now() { return Clock(RCL_ROS_TIME).now(); };
} // namespace rclcpp

#define SET_ROS_STAMP(MSG, TS)                                                 \
  {                                                                            \
    MSG.header.stamp.sec =                                                     \
        std::chrono::time_point_cast<std::chrono::seconds>(TS)                 \
            .time_since_epoch()                                                \
            .count();                                                          \
    MSG.header.stamp.nanosec =                                                 \
        std::chrono::time_point_cast<std::chrono::nanoseconds>(TS)             \
            .time_since_epoch()                                                \
            .count() %                                                         \
        1000000000UL;                                                          \
  }
