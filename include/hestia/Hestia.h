#pragma once

//--Includes-----------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

//--Device Interface---------------------------------------------------
class Device
{
    public:

        // Constructs a device
        Device();

        // Deconstructs the device
        virtual ~Device();

        // Loads the device
        // virtual void Load(Reservoir* reservoir, int level);
        virtual void Load(int level);

        // Deploys the device
        // virtual void Deploy(Monitor* bushland, int bushID);
        virtual void Deploy(int bushID, int level);

    protected:

        // Keeps track of the number of available resources
        int availableResource;

        // Node handle for Hestia
        ros::NodeHandle nh;
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
        // void Load(Reservoir* reservoir, int level);
        void Load(int level);

        // Blasts water at the bush indicated by bushID
        // void Deploy(Monitor* bushland, int bushID);
        void Deploy(int bushID, int level);

    private:

        // Publish water blasted at fire of interest to monitor 
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
        // void Load(Reservoir* reservoir, int level);
        void Load(int level);

        // Throws flames at the bush indicated by bushID
        // void Deploy(Monitor* bushland, int bushID);
        void Deploy(int bushID, int level);
        
    private:
    
        // Publish flame thrown at hazard of interest to monitor 
        // in the bushland node
        ros::Publisher flameThrowerPub;

        
};
//--Hestia Interface---------------------------------------------------
class Hestia
{
    public:

        // Construct a Hestia
        Hestia();

        // Destruct the Hestia
        ~Hestia();

    private:

        // Keeps track of the amount of resources needed
        int requiredResource;

        // The ID of the last detected april tag
        int currentId;

        enum OperationMode
        {
            UNDEFINED = -1
            CONTROLLED_BURNING,
            FIRE_ELIMINATION
        };

        // Current Operation mode
        OperationMode mode;

        // The devices on Hestia
        HydroBlaster* hydroBlaster;
        FlameThrower* flameThrower;       

        // Node handle for Hestia
        ros::NodeHandle nh;

        // Subscribe to water loaded from reservoir 
        // in the bushland node
        ros::Subscriber waterTankSub;

        // Subscribe to gas loaded from reservoir 
        // in the bushland node
        ros::Subscriber gasTankSub;

        // Subscribe to operation mode from the user interface
        ros::Subscriber modeSub;

        // Subscribe to the currently detected april tag ID from april tag detector
        ros::Subscriber tagSub;
        
        // Callback for the water tank subscriber
        void WaterTankCallback(const std_msgs::Int32::ConstPtr& msg);

        // Callback for the gas tank subscriber
        void GasTankCallback(const std_msgs::Int32::ConstPtr& msg);

        // Callback for the operation mode subscriber
        void ModeCallback(const std_msgs::String::ConstPtr& msg);

        // Callback for the april tag subscriber
        void TagCallback(const std_msgs::Int32::ConstPtr& msg);
};

