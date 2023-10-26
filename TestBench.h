#ifndef _TESTBENCH_H
#define _TESTBENCH_H

//--Includes-----------------------------------------------------------
#include "Device.h"

#include <vector>

//--TestBench Interface---------------------------------------------------
class TestBench
{
    public:

        // Constructs a test bench
        TestBench();

        // Deconstructs the test bench
        ~TestBench();

        // Runs the tests
        void RunSufficientReservoirForFires();
        void RunSufficientReservoirForHazards();
        void RunInsufficientReservoirForFires();

        // void RunInsufficientReservoirForHazards();

    private:

        // Define constants
        const bool hazardous = true;
        const bool notHazardous = false;

        // Define constants
        const bool onFire = true;
        const bool notOnFire = false;

        // Keep track of the bushland
        Maps* bushland;

        // Keep track of the reservoir
        // Reservoir* reservoir;
        WaterTank* waterTank;
        GasTank* gasTank;

        // Keep track of the devices on the bushfire manager bot
        HydroBlaster* hydroBlaster;
        FlameThrower* flameThrower;

        // Keep track of all th ebushes
        std::vector<Bush*> bushes;

        // Display controlled burning outcome
        void CheckControlledBurningOutcome();

        // Display fire elimination outcome
        void CheckFireEliminationOutcome();
};

#endif
