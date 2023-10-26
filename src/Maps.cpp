//--Includes-----------------------------------------------------------
#include "Maps.h"

#include <iostream>

//--Maps Implementation------------------------------------------
// Constructs a maps object
Maps::Maps()
{
    // std::cout << "Maps[CTor]: Maps signing on." << std::endl;
}

// Deconstructs the maps object
Maps::~Maps()
{
    // std::cout << "Maps[DTor]: Maps signing off." << std::endl;
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