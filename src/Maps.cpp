//--Includes-----------------------------------------------------------
#include "Maps.h"

#include <iostream>

//--Maps Implementation------------------------------------------
// Constructs a maps object
Maps::Maps()
{   
    ros::NodeHandle n;

    // Publishers

    // Once bot reach a bush tag, ask bush status
    // change to service client
    ros::Publisher firesPub = n.advertise<std_msgs::String>("Fires", 1000);
    // server call back receive a id


    //Publish priority list, need to change format
    ros::Publisher pathPub = n.advertise<std_msgs::String>("Fires", 1000);


    // Subscribers
    ros::Subscriber modeSub = n.subscribe("/operation_mode_topic", 10, &Maps::modeCallback, this);

    // Subscriber object for subscribing to the Odometry topic
    // Set the size of the message queue to be 1000
    ros::Subscriber odometrySub = n.subscribe("Odometry", 1000, odometryCallback);

    // Subscriber object for subscribing to the Water topic
    // Set the size of the message queue to be 1000
    ros::Subscriber waterSub = n.subscribe("Water", 1000, firesCallback);

}

// Deconstructs the maps object
Maps::~Maps()
{
    // std::cout << "Maps[DTor]: Maps signing off." << std::endl;
}

void Maps::modeCallback(){}

void firesCallback(){}

// Callback function to receive messages from Odometry topic
void odometryCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

// Adds the specified bush to the hazards map with its ID as the key
void Maps::AddHazard(int bushID, Bush* hazard)
{
    // Try to insert the bush into the hazards map
    bool insertion = hazardsMap.insert({bushID, hazard}).second;

    // Output information about the insertion
    if (insertion)
    {
        std::cout << "Bush " << bushID << " is now on the hazards map." << std::endl;
    }
    else
    {
        std::cout << "Bush " << bushID << " was already on the hazrds map." << std::endl;
    }
}

// Adds the specified bush to the fires map with its ID as the key
void Maps::AddFire(int bushID, Bush* fire)
{
    // Try to insert the bush into the fires map
    bool insertion = firesMap.insert({bushID, fire}).second;

    // Output information about the insertion
    if (insertion)
    {
        std::cout << "Bush " << bushID << " is now on the fires map." << std::endl;
    }
    else
    {
        std::cout << "Bush " << bushID << " was already on the fires map." << std::endl;
    }
}

// Returns the pointer to the hazard with the given bush ID
Bush* Maps::LocateHazard(int bushID)
{
    // Check that the desired gate exists
    if(hazardsMap.find(bushID) == hazardsMap.end())
    {
        return nullptr;
    }

    return hazardsMap[bushID];
}

// Returns the pointer to the bushfire with the given bush ID
Bush* Maps::LocateFire(int bushID)
{
    // Check that the desired gate exists
    if(firesMap.find(bushID) == firesMap.end())
    {
        return nullptr;
    }

    return firesMap[bushID];
}