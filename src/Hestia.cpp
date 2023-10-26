// This is the Hestia node responsible for receiving information from 
// the Bushland node for the operation of Device

//--Includes-----------------------------------------------------------
#include "hestia/Hestia.h"

//--Hestia Implementation------------------------------------------
// Constructs Hestia
Hestia::Hestia()
{
    // Initialise the amount of resources required
    requiredResource = 0;

    // Initialise the hydro blaster and flame thrower
    hydroBlaster = new HydroBlaster();
    flameThrower = new FlameThrower();

    // Initialise water tank subscriber
    waterTankSub = nh.subscribe("/reservoir_water_status", 100, &Hestia::WaterTankCallback, this);

    // Initialise gas tank subscriber
    gasTankSub = nh.subscribe("/reservoir_gas_status", 100, &Hestia::GasTankCallback, this);

}

// Destructs Hestia
Hestia::~Hestia()
{

}

// Callback function to receive messages from reservoir water topic
void Hestia::WaterTankCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("I heard: [%d]", msg->data);

    if (msg->data > requiredResource)
    {
        hydroBlaster->Load(requiredResource);
    }
}

// Callback function to receive messages from reservoir gas topic
void Hestia::GasTankCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("I heard: [%d]", msg->data);

    if (msg->data > requiredResource)
    {
        flameThrower->Load(requiredResource);
    }
}

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
    waterBlasterPub = nh.advertise<std_msgs::String>("/water_blasted_to_id", 100);
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
        // Put out the fire
        // bushland->LocateFire(bushID)->EliminateFire();  
        waterBlasterPub.publish(level);

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
    flameThrowerPub = nh.advertise<std_msgs::String>("/flame_thrown_to_id", 100);
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
        // Put out the fire
        // bushland->LocateFire(bushID)->EliminateFire();  
        flameThrowerPub.publish(level);

        // Decrement number of available water
        availableResource -= level;
    }
    else
    {
        std::cout << "Not enough gas... Hazard elimination failed." << std::endl;
    }
}