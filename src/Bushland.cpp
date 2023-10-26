// This is the Bushland node responsible for sending information about 
// Maps, Bush and Reservoir to the Hestia node

//--Includes-----------------------------------------------------------
#include <ros/ros.h>
#include "std_msgs/String.h"

#include "hestia/Maps.h"
#include "hestia/Reservoir.h"


// Callback function to receive messages from Odometry topic
void odometryCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

// Callback function to receive messages from Water topic
void waterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

// Callback function to receive messages from Gas topic
void gasCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

