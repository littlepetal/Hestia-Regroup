//--Includes-----------------------------------------------------------
#include "hestia/Device.h"

//--Device Implementation------------------------------------------
// Constructs an empty device. Initialises the load and deploy publishers
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
void Device::load(int level)
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
void Device::deploy(int bushID, int level)
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
// Constructs an empty hydro blaster
HydroBlaster::HydroBlaster()
{
}

// Deconstructs a hydro blaster 
HydroBlaster::~HydroBlaster()
{
}

// Loads the device with water
void HydroBlaster::load(int level)
{
    Device::load(level);
}

// Blasts level amount of water at the fire indicated by the bushID
void HydroBlaster::deploy(int bushID, int level)
{
    Device::deploy(bushID, level);
}

//--FlameThrower Implementation------------------------------------------
// Constructs an empty flame thrower
FlameThrower::FlameThrower()
{
}

// Deconstructs the flame thrower
FlameThrower::~FlameThrower()
{
}

// Loads the device with gas
void FlameThrower::load(int level)
{
    Device::load(level);
}

// Throws level amount of flames at the hazard indicated by the bushID
void FlameThrower::deploy(int bushID, int level)
{
    Device::deploy(bushID, level);
}