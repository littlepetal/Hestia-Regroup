//--Includes-----------------------------------------------------------
#include "hestia/Device.h"

//--Device Implementation------------------------------------------
// Constructs an empty device
Device::Device()
{
    // std::cout << "Device[CTor]: Device signing on." << std::endl;

    // Initialise number of available resource
    availableResource = 0;
}

// Virtual deconstructor
Device::~Device()
{
    // std::cout << "Device[DTor]: Device signing off." << std::endl;
}

// Loads the device with resource
void Device::Load(int level)
{

}

// Deploys the device using the available resources
void Device::Deploy(int bushID, int level)
{
   
}

//--HydroBlaster Implementation------------------------------------------
// Constructs a hydro blaster
HydroBlaster::HydroBlaster()
{
    // std::cout << "HydroBlaster[CTor]: HydroBlster signing on." << std::endl;
    // Initialise water blaster publisher
    waterBlasterPub = nh.advertise<std_msgs::Int32>("/water_blasted_to_id", 100);
}

// Deconstructs a hydro blaster 
HydroBlaster::~HydroBlaster()
{
    // std::cout << "HydroBlaster[DTor]: HydroBlster signing off." << std::endl;
}

// Loads the device with water
void HydroBlaster::Load(int level)
{
    // Increases the number of available water by level amount
    availableResource += level;  

    // Output result
    std::cout << availableResource << " water is now available for the hydro blaster" << std::endl;  
}

// Blasts level amount of water at the fire indicated by the bushID
void HydroBlaster::Deploy(int bushID, int level)
{
    // Check that there is enough water in the bushfire manager bot
    if (availableResource > level)
    {
        // Format string
        // std::String message = std::format("Hestia deployed %d water at the fire with ID %d", level, bushID);

        // Put out the fire
        // bushland->LocateFire(bushID)->EliminateFire();  

        // Format the message
        std_msgs::Int32 message;
        message.data = level;

        // Publish the message
        waterBlasterPub.publish(message);

        // Decrement number of available water
        availableResource -= level;
    }
    else
    {
        std::cout << "Not enough water... Fire elimination failed." << std::endl;
    }
}

//--FlameThrower Implementation------------------------------------------
// Constructs a flame thrower
FlameThrower::FlameThrower()
{
    // std::cout << "FlameThrower[CTor]: FlameThrower signing on." << std::endl;

    // Initialise flame thrower publisher
    flameThrowerPub = nh.advertise<std_msgs::Int32>("/flame_thrown_to_id", 100);
}

// Deconstructs the flame thrower
FlameThrower::~FlameThrower()
{
    // std::cout << "FlameThrower[DTor]: FlameThrower signing off." << std::endl;
}

// Loads the device with gas
void FlameThrower::Load(int level)
{
    // Increases the number of available water by level amount
    availableResource += level;  

    // Output result
    std::cout << availableResource << " gas is now available for the flame thrower" << std::endl;  
}

// Throws level amount of flames at the hazard indicated by the bushID
void FlameThrower::Deploy(int bushID, int level)
{
    // Check that there is enough gas in the bushfire manager bot
    if (availableResource > level)
    {
        // Format string
        // std::String message = std::format("Hestia deployed %d flames at the hazard with ID %d", level, bushID);

        // Burn the hazard
        // bushland->LocateFire(bushID)->EliminateFire();  

        // Format the message
        std_msgs::Int32 message;
        message.data = level;

        // Publish the message
        flameThrowerPub.publish(message);

        // Decrement number of available water
        availableResource -= level;
    }
    else
    {
        std::cout << "Not enough gas... Hazard elimination failed." << std::endl;
    }
}