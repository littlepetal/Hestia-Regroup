//--Includes-----------------------------------------------------------
#include "hestia/Device.h"

//--Device Implementation------------------------------------------
// Constructs an empty device
Device::Device()
{
    // Initialise number of available resource
    availableResource = 0;

    // Initialise publishers
    loadResourcesPub = nh.advertise<std_msg::Int32>("/loaded_resources", 10);
    deployResourcesPub = nh.advertise<std_msg::Int32>("/deployed_resources", 10);
}

// Virtual deconstructor
Device::~Device()
{
}

// Loads the device with resource
void Device::Load(int level)
{
    // Increases the number of available resources by level amount
    availableResource += level;  

    // Create resource level loaded message
    std_msgs::Int32 loadedMsg;
    loadedMsg.data = level;   

    // Publish the loaded message
    loadedResourcesPub.publish(loadedMsg); 
}

// Deploys the device using the available resources
void Device::Deploy(int bushID, int level)
{
    // Decrement number of available water
    availableResource -= level;

    // Format the message
    std_msgs::Int32 deployedMsg;
    message.data = level;

    // Publish the message
    waterBlasterPub.publish(deployedMsg);
}

//--HydroBlaster Implementation------------------------------------------
// Constructs a hydro blaster
HydroBlaster::HydroBlaster()
{
}

// Deconstructs a hydro blaster 
HydroBlaster::~HydroBlaster()
{
}

// Loads the device with water
void HydroBlaster::Load(int level)
{
    Device::Load(level);
}

// Blasts level amount of water at the fire indicated by the bushID
void HydroBlaster::Deploy(int bushID, int level)
{
    Device::Deploy(bushID, level);
}

//--FlameThrower Implementation------------------------------------------
// Constructs a flame thrower
FlameThrower::FlameThrower()
{
}

// Deconstructs the flame thrower
FlameThrower::~FlameThrower()
{
}

// Loads the device with gas
void FlameThrower::Load(int level)
{
    Device::Load(level);
}

// Throws level amount of flames at the hazard indicated by the bushID
void FlameThrower::Deploy(int bushID, int level)
{
    Device::Deploy(bushID, level);
}