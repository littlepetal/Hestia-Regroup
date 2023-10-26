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

// Send messages about the state of the bushland to Hestia
int main(int argc, char **argv)
{
    // Perform remapping of ROS arguments and names that were provided at the 
    // command line
    ros::init(argc, argv, "Hestia");

    // Fully initialise the node
    // NodeHandle is the main access point to communications with the ROS system
    ros::NodeHandle n;

    // Publishers

    // Publisher object for publishing messages on Odometry topic
    // Set the size of the message queue to be 1000
    ros::Publisher odometryPub = n.advertise<std_msgs::String>("Odometry", 1000);

    // Publisher object for publishing messages on Water topic
    // Set the size of the message queue to be 1000
    ros::Publisher waterPub = n.advertise<std_msgs::String>("Water", 1000);

    // Publisher object for publishing messages on Gas topic
    // Set the size of the message queue to be 1000
    ros::Publisher gasPub = n.advertise<std_msgs::String>("Gas", 1000);

    // Read the data every 100ms
    ros::Rate loop_rate(10);

    // Subscribers

    // Subscriber object for subscribing to Hazards topic
    // Set the size of the message queue to be 1000
    ros::Subscriber hazardsSub = n.subscribe("Hazards", 1000, hazardsCallback);

    // Subscriber object for subscribing to Fires topic
    // Set the size of the message queue to be 1000
    ros::Subscriber firesSub = n.subscribe("Fires", 1000, firesCallback);
    

    // Count of the number of sent messages
    int sentMessagesCount = 0;

    // Check that the thread has not been terminated
    while(ros::ok)
    {
        // Message object to be published
        std_msgs::String msg;

        // Create messages
        std::stringstream ss;
        ss << "Odometry " << sentMessagesCount;
        msg.data = ss.str();

        // Format messages
        ROS_INFO("%s", msg.data.c_str());

        // Broadcast messages to subscribers
        odometryPub.publish(msg);

        // Call all the callbacks waiting to be called at that point in time
        // Block the main thread from exiting until ROS invokes a shutdown
        ros::spinOnce();

        // Thread sleep
        loop_rate.sleep();

        // Increment the number of sent messages
        sentMessagesCount++;
    }

    // Pump callbacks
    ros::spin();

    return 0;
}