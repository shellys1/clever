#pragma once

#include <string>
#include <memory>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

extern std::string local_frame;
extern ros::Duration transform_timeout;
extern tf2_ros::Buffer tf_buffer;

// Shortcut for reading ros::Duration params
inline void param(ros::NodeHandle nh, std::string, ros::Duration& container, double _default)
{
    double val;
    nh.param("state_timeout", val, _default);
    container = ros::Duration(val);
}
