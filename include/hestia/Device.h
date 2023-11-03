#ifndef DEVICE_H
#define DEVICE_H

//--Includes-----------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <utility>  // for std::pair
#include <thread>
#include <mutex>

//--Device Interface---------------------------------------------------
// Device is responsible for controlling the operation of the onboard 
// bushfire management devices. 
class Device
{
    public:

        // Constructs a device
        Device();

        // Deconstructs the device
        virtual ~Device();

        // Loads the device
        virtual void Load(int level);

        // Deploys the device
        virtual void Deploy(int bushID, int level);

    protected:

        // Keeps track of the number of available resources
        int availableResource;

        ros::Publisher loadResourcesPub;  // Publisher for loading onboard water and gas resources 
        ros::Publisher deployResourcesPub;  // Publisher for deploying onboard water and gas resources 
};

//--HydroBlaster Interface---------------------------------------------------
class HydroBlaster: public Device
{
    public:

        // Constructs a hydro blaster
        HydroBlaster();

        // Deconstructs the hydro blaster
        ~HydroBlaster();

        // Increases the number of available water by level amount
        void Load(int level);

        // Blasts water at the bush indicated by bushID
        void Deploy(int bushID, int level);

    private:

        // Publishes water blasted at fire of interest to monitor 
        // in the bushland node
        ros::Publisher waterBlasterPub;
        
};

//--FlameThrower Interface---------------------------------------------------
class FlameThrower: public Device
{
    public:

        // Constructs a flame thrower
        FlameThrower();

        // Deconstructs the flame thrower
        ~FlameThrower();

        // Increases the number of available gas by level amount
        void Load(int level);

        // Throws flames at the bush indicated by bushID
        void Deploy(int bushID, int level);
        
    private:
    
        // Publishes flame thrown at hazard of interest to monitor 
        // in the bushland node
        ros::Publisher flameThrowerPub;
};

#endif