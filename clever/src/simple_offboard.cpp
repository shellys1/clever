#include <string>
#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <clever/Navigate.h>
#include <clever/SetPosition.h>

#include "simple_offboard.h"
#include "get_telemetry.h"

std::string local_frame;
ros::Duration transform_timeout;
tf2_ros::Buffer tf_buffer;
ros::Publisher attitude_pub, position_pub;
std::unique_ptr<ros::Publisher> pub = nullptr;
ros::Timer publish_timer;

/*
void publishMessage()
{
    ROS_INFO("Publish?");

    if (pub == nullptr) {
        return;
    }

    ros::Time stamp = ros::Time::now();
    // ros::Message *msg = getMessage();

    // pub->publish(msg);
}

bool handle(clever::SetPosition::Request& request, clever::SetPosition::Response& response)
{

}

void initPublisher(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
{
    // Create publisher timer
    double setpoint_rate;
    nh_priv.param("setpoint_rate", setpoint_rate, 30.0);
    // ros::Timer publish_timer = nh.createTimer(ros::Duration(setpoint_rate), &publishMessage);
}

void initServices(ros::NodeHandle nh)
{
    static ros::ServiceServer set_position = nh.advertiseService("set_position", &handle);
}
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_offboard");
    ros::NodeHandle nh, nh_priv("~");

    tf2_ros::TransformListener tf_listener(tf_buffer);

    nh_priv.param("local_frame", local_frame, std::string("local_origin"));
    param(nh_priv, "transform_timeout", transform_timeout, 0.01);

    // initPublisher(nh, nh_priv);
    // initServices();
    initTelemetry(nh, nh_priv);

    ROS_INFO("Simple offboard inited");
    ros::spin();
}
