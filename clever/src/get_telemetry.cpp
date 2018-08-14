// `get_telemetry` service implementation

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/State.h>
#include <clever/GetTelemetry.h>

#include "simple_offboard.h"

using namespace geometry_msgs;
using namespace sensor_msgs;

// Parameters
ros::Duration local_position_timeout;
ros::Duration state_timeout;
ros::Duration velocity_timeout;
ros::Duration global_position_timeout;
ros::Duration battery_timeout;

// Last received telemetry messages
mavros_msgs::State::ConstPtr state = nullptr;
PoseStamped::ConstPtr local_position = nullptr;
TwistStamped::ConstPtr velocity = nullptr;
NavSatFix::ConstPtr global_position = nullptr;
BatteryState::ConstPtr battery = nullptr;

// Common subcriber callback template that stores message to the variable
template<typename T, T& STORAGE>
void handleMessage(const T msg)
{
    STORAGE = msg;
}

bool getTelemetry(clever::GetTelemetry::Request& request, clever::GetTelemetry::Response& response)
{
    ros::Time stamp = ros::Time::now();

    // Default frame id local frame
    if (request.frame_id.empty()) {
        request.frame_id = local_frame;
    }

    response.frame_id = request.frame_id;
    response.x = NAN;
    response.y = NAN;
    response.z = NAN;
    response.lat = NAN;
    response.lon = NAN;
    response.alt = NAN;
    response.vx = NAN;
    response.vy = NAN;
    response.vz = NAN;
    response.pitch = NAN;
    response.roll = NAN;
    response.yaw = NAN;
    response.pitch_rate = NAN;
    response.roll_rate = NAN;
    response.yaw_rate = NAN;
    response.voltage = NAN;
    response.cell_voltage = NAN;

    if (state != nullptr && stamp - state->header.stamp < state_timeout) {
        response.connected = state->connected;
        response.armed = state->armed;
        response.mode = state->mode;
    }

    if (local_position != nullptr && stamp - local_position->header.stamp < local_position_timeout) {
        // transform pose
        geometry_msgs::PoseStamped pose = tf_buffer.transform(*local_position, request.frame_id, transform_timeout);

        response.x = pose.pose.position.x;
        response.y = pose.pose.position.y;
        response.z = pose.pose.position.z;
    }

    if (velocity != nullptr && stamp - velocity->header.stamp < velocity_timeout) {
        response.vx =  velocity->twist.linear.x;
        response.vy =  velocity->twist.linear.y;
        response.vz =  velocity->twist.linear.z;
    }

    if (global_position != nullptr && stamp - global_position->header.stamp < global_position_timeout) {
        response.lat = global_position->latitude;
        response.lon = global_position->longitude;
        response.alt = global_position->altitude;
    }

    if (battery != nullptr && stamp - battery->header.stamp < battery_timeout) {
        response.voltage = battery->voltage;
        if (!battery->cell_voltage.empty()) {
            response.cell_voltage = battery->cell_voltage[0];
        }
    }

    return true;
}

void initTelemetry(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
{
    param(nh_priv, "state_timeout", state_timeout, 3.0);
    param(nh_priv, "local_position_timeout", local_position_timeout, 3.0);
    param(nh_priv, "velocity_timeout", velocity_timeout, 3.0);
    param(nh_priv, "global_position_timeout", global_position_timeout, 10.0);
    param(nh_priv, "battery_timeout", battery_timeout, 3.0);

    static ros::Subscriber state_sub = nh.subscribe("mavros/state", 1, &handleMessage<mavros_msgs::StateConstPtr, state>);
    static ros::Subscriber local_position_sub = nh.subscribe("mavros/local_position/pose", 1, &handleMessage<PoseStampedConstPtr, local_position>);
    static ros::Subscriber velocity_sub = nh.subscribe("mavros/local_position/velocity", 1, &handleMessage<TwistStampedConstPtr, velocity>);
    static ros::Subscriber global_position_sub = nh.subscribe("mavros/global_position/global", 1, &handleMessage<NavSatFixConstPtr, global_position>);
    static ros::Subscriber battery_sub = nh.subscribe("mavros/battery", 1, &handleMessage<BatteryStateConstPtr, battery>);

    // static ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::PoseStampedConstPtr>("/mavros/local_position/velocity", 1, &handleMessage<geometry_msgs::PoseStampedConstPtr, &velocity>);

    static ros::ServiceServer get_telemetry = nh.advertiseService("get_telemetry", &getTelemetry);
}
