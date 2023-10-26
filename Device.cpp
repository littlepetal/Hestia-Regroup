//--Includes-----------------------------------------------------------
#include "Device.h"

#include <iostream>

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
void Device::Load(Reservoir* reservoir, int level)
{

}

// Deploys the device using the available resources
void Device::Deploy(Maps* bushland, int bushID)
{
   
}

//--HydroBlaster Implementation------------------------------------------
// Constructs a hydro blaster
HydroBlaster::HydroBlaster()
{
    // std::cout << "HydroBlaster[CTor]: HydroBlster signing on." << std::endl;
}

// Deconstructs a hydro blaster 
HydroBlaster::~HydroBlaster()
{
    // std::cout << "HydroBlaster[DTor]: HydroBlster signing off." << std::endl;
}

// Loads the device with water
void HydroBlaster::Load(Reservoir* reservoir, int level)
{
    // Notification
    std::cout << "Loading " << level << " water for the hydro blaster..." << std::endl;

    // Attempt to receive water from reservoir
    if (reservoir->Supply(level))
    {
        // Increases the number of available water by level amount
        availableResource += level;  

        // Output result
        std::cout << availableResource << " water is now available for the hydro blaster" << std::endl;     
    }
    else
    {
        std::cout << "Insufficient water at the reservoir... Loading failed." << std::endl;
    }
}

// Blasts water at the fire indicated by the bushID
void HydroBlaster::Deploy(Maps* bushland, int bushID)
{
    // Notification
    std::cout << "Deploying hydro blaster..." << std::endl;

    // Check that there is enough water in the bushfire manager bot
    if (availableResource > 0)
    {
        // Put out the fire
        bushland->LocateFire(bushID)->EliminateFire();  

        // Decrement number of available water
        availableResource--;
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
}

// Deconstructs the flame thrower
FlameThrower::~FlameThrower()
{
    // std::cout << "FlameThrower[DTor]: FlameThrower signing off." << std::endl;
}

// Loads the device with gas
void FlameThrower::Load(Reservoir* reservoir, int level)
{
    // Notification
    std::cout << "Loading " << level << " gas for the flame thrower..." << std::endl;

    // Attempt to receive gas from reservoir
    if (reservoir->Supply(level))
    {
        // Increases the number of available gas by level amount
        availableResource += level;  

        // Output result
        std::cout << availableResource << " gas is now available for the flame thrower" << std::endl;     
    }
    else
    {
        std::cout << "Insufficient gas at the reservoir... Loading failed." << std::endl;
    }
}

// Throws flames at the hazard indicated by the bushID
void FlameThrower::Deploy(Maps* bushland, int bushID)
{
    // Notification
    std::cout << "Deploying flame thrower..." << std::endl;

    // Check that there is enough gas in the bushfire manager bot
    if (availableResource > 0)
    {
        // Rid the hazard
        bushland->LocateHazard(bushID)->ControlledBurning(); 

        // Decrement number of available gas
        availableResource--;
    }
    else
    {
        std::cout << "Not enough gas... Hazard elimination failed." << std::endl;
    }
}
