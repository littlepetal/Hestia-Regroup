// This is the Hestia node responsible for receiving information from 
// the Bushland node for the operation of Device

//--Includes-----------------------------------------------------------
#include "ros.h"
#include "std_msgs/String.h"

#include "Device.h"

#include <sstream>

// Callback function to receive messages from Hazards topic
void hazardsCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

// Callback function to receive messages from Fires topic
void firesCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

