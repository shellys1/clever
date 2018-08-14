#pragma once

#include <ros/ros.h>
#include <mavros_msgs/State.h>

extern mavros_msgs::State::ConstPtr state;

void initTelemetry(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);
