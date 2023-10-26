#ifndef _MAPS_H
#define _MAPS_H

//--Includes-----------------------------------------------------------
#include "Bush.h"

#include <map>

//--Maps Interface---------------------------------------------------
class Maps
{
    public:

        // Contructs an empty map
        Maps();

        // Destructs the map
        ~Maps();

        // Adds the specified bush to the hazards map with its ID as the key
        void AddHazard(int bushID, Bush* hazard);

        // Returns the pointer to the hazardous bush with the given bush ID
        Bush* LocateHazard(int bushID);

        // Adds the specified bush to the fires map with its ID as the key
        void AddFire(int bushID, Bush* fire);

        // Returns the pointer to the bushfire with the given bush ID
        Bush* LocateFire(int bushID);

    private:

        // Keeps a map of the hazardous bushes
        std::map<int, Bush*> hazardsMap;

        // Keeps a map of the fires
        std::map<int, Bush*> firesMap;
};

#endif