#ifndef _DEVICE_H
#define _DEVICE_H

//--Includes-----------------------------------------------------------
#include "Maps.h"
#include "Reservoir.h"

//--Device Interface---------------------------------------------------
class Device
{
    public:

        // Constructs a device
        Device();

        // Deconstructs the device
        virtual ~Device();

        // Loads the device
        virtual void Load(Reservoir* reservoir, int level);

        // Deploys the device
        virtual void Deploy(Maps* bushland, int bushID);

    protected:

        // Keeps track of the number of available resources
        int availableResource;
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
        void Load(Reservoir* reservoir, int level);

        // Blasts water at the bush indicated by bushID
        void Deploy(Maps* bushland, int bushID);

    private:
        
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
        void Load(Reservoir* reservoir, int level);

        // Throws flames at the bush indicated by bushID
        void Deploy(Maps* bushland, int bushID);
        
    private:
        
};


#endif