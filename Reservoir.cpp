//--Includes-----------------------------------------------------------
#include "Reservoir.h"

#include <iostream>

//--Reservoir Implementation------------------------------------------
// Constructs an empty reservoir
Reservoir::Reservoir()
{
    // std::cout << "Reservoir[CTor]: Reservoir signing on." << std::endl;

    // Initialises the resource level of the reservoir
    resourceLevel = 0;

    // Initialises the name of the resource in the reservoir
    name = "";
}

// Destructs the reservoir
Reservoir::~Reservoir()
{
    // std::cout << "Reservoir[DTor]: Reservoir signing off." << std::endl;
}

int Reservoir::CheckLevel()
{
    return 0;
}

void Reservoir::TopUp(int level)
{

}

bool Reservoir::Supply(int level)
{
    return false;
}

// Constructs a water tank
WaterTank::WaterTank()
{
    // std::cout << "WaterTank[CTor]: WaterTank signing on." << std::endl;

    // Reservoir();
}

// Destructs a water tank
WaterTank::~WaterTank()
{
    // std::cout << "WaterTank[DTor]: WaterTank signing off." << std::endl;
}

// Returns the water level
int WaterTank::CheckLevel()
{
    // Output water level
    std::cout << "Reservoir now has " << resourceLevel << " water" << std::endl;

    // Indicate the water situation
    if (resourceLevel <= 10)
    {
        std::cout << "Water level is dangerously low" << std::endl;
    }
    else if (resourceLevel <= 50)
    {
        std::cout << "Water level is moderately low" << std::endl;
    }
    else
    {
        std::cout << "Water level is high" << std::endl;
    }
    
    return resourceLevel;
}

// Tops up the water level of the reservoir
void WaterTank::TopUp(int level)
{
    // Notification
    std::cout << "Topping up water in the resrvoir by " << level << "..." << std::endl;

    // Increase water level by level amount
    resourceLevel += level;
}

// Provide water for the bushfire management bot
bool WaterTank::Supply(int level)
{
    // Flag to indicate success of the supply process
    bool supplied = false;

    // Notification
    std::cout << "Supplying " << level << " water for the bushfire manager bot..." << std::endl;

    // Check that the reservoir has sufficient water to supply
    if (resourceLevel - level < 0)
    {
        std::cout << "Insufficient water level!" << std::endl;
    }
    else
    {
        // Decrease water level by level amount
        resourceLevel -= level;

        supplied = true;
    }

    return supplied;
}

// Constructs a gas tank
GasTank::GasTank()
{
    // std::cout << "GasTank[CTor]: GasTank signing on." << std::endl;
    // Reservoir();
}

// Destructs the gas tank
GasTank::~GasTank()
{
    // std::cout << "GasTank[DTor]: GasTank signing off." << std::endl;
}

// Returns the gas level
int GasTank::CheckLevel()
{
    // Output gas level
    std::cout << "Reservoir now has " << resourceLevel << " gas" << std::endl;

    // Indicate the gas situation
    if (resourceLevel <= 10)
    {
        std::cout << "Gas level is very low" << std::endl;
    }
    else if (resourceLevel <= 50)
    {
        std::cout << "Gas level is moderately low" << std::endl;
    }
    else
    {
        std::cout << "Gas level is high" << std::endl;
    }

    return resourceLevel;
}

// Tops up the gas level of the reservoir
void GasTank::TopUp(int level)
{
    // Notification
    std::cout << "Topping up gas in the resrvoir by " << level << "..." << std::endl;

    // Increase gas level by level amount
    resourceLevel += level;
}

// Provide gas for the bushfire management bot
bool GasTank::Supply(int level)
{
    // Flag to indicate success of the supply process
    bool supplied = false;

    // Notification
    std::cout << "Supplying " << level << " gas for the bushfire manager bot..." << std::endl;

    // Check that the reservoir has sufficient gas to supply
    if (resourceLevel - level < 0)
    {
        std::cout << "Insufficient gas level!" << std::endl;
    }
    else
    {
        // Decrease gas level by level amount
        resourceLevel -= level;  

        supplied = true;
    } 

    return supplied;
}