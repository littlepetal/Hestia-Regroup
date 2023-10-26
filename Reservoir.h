#ifndef _RESERVOIR_H
#define _RESERVOIR_H

#include <string>

//--Reservoir Interface---------------------------------------------------
class Reservoir
{
    public:

        // Constructs an empty reservoir
        Reservoir();

        // Destructs the reservoir
        ~Reservoir();

        // Returns the resource level of the reservoir
        virtual int CheckLevel();

        // Tops up the resource level of the reservoir by level amount
        virtual void TopUp(int level);

        // Supplies level amount of resources for the device requesting it
        virtual bool Supply(int level);

    protected:

        // Indicates the resource level in the reservoir
        int resourceLevel;

        // A string for the name of the resource in the reservoir
        std::string name;

};

class WaterTank: public Reservoir
{
    public:

        // Contructs a water tank
        WaterTank();

        // Destructs a water tank
        ~WaterTank();
 
        // Returns the water level of the water tank
        int CheckLevel();

        // Tops up the water level of the water tank by level amount
        void TopUp(int level);

        // Supplies level amount of water for the device requesting it
        bool Supply(int level);   

    private:

};

class GasTank: public Reservoir
{
    public:
 
        // Contructs a gas tank
        GasTank();

        // Destructs a water tank
        ~GasTank();
 
        // Returns the gas level of the gas tank
        int CheckLevel();

        // Tops up the gas level of the gas tank by level amount
        void TopUp(int level);

        // Supplies level amount of gas for the device requesting it
        bool Supply(int level);   

    private:

};

#endif