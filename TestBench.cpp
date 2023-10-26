//--Includes-----------------------------------------------------------
#include "TestBench.h"

#include <iostream>

//--TestBench Implementation------------------------------------------
// Constructs a test bench
TestBench::TestBench()
{
    // std::cout << "TestBench[CTor]: TestBench signing on." << std::endl;

    // Create the bushland
    std::cout << "Initialising the bushland..." << std::endl;

    // Initialise the bushland
    bushland = new Maps();

    // Create the reservoir
    std::cout << "Initialising the reservoir..." << std::endl;

    // Initialise the reservoir
    // reservoir = new Reservoir();
    waterTank = new WaterTank();
    gasTank = new GasTank();

    // Create the bush fire control devices
    std::cout << "Setting up the bushfire control devices..." << std::endl;

    // Initialise the hydro blaster and flame thrower
    hydroBlaster = new HydroBlaster();
    flameThrower = new FlameThrower();
}

// Deconstructs the test bench
TestBench::~TestBench()
{
    // std::cout << "TestBench[DTor]: TestBench signing off." << std::endl;

    // Free up memory
    delete bushland;
    delete waterTank;
    delete gasTank;
    delete hydroBlaster;
    delete flameThrower;

    for (int i = 0; i < bushes.size(); i++)
    {
        delete bushes[i];
    }
}

// Simulates the situation where there is sufficient water supply
// at the reservoir, and the hydro blaster loads enough water to 
// eliminate all the fires in the bushland
void TestBench::RunSufficientReservoirForFires ()
{
    // Create the bushes and add them to the vector of bushes
    std::cout << "Creating the bushes..." << std::endl;

    bushes.push_back(new Bush(0, notHazardous, notOnFire));
    bushes.push_back(new Bush(1, notHazardous, onFire));
    bushes.push_back(new Bush(2, notHazardous, notOnFire));
    bushes.push_back(new Bush(3, notHazardous, onFire));
    bushes.push_back(new Bush(4, notHazardous, notOnFire));
    bushes.push_back(new Bush(5, notHazardous, onFire));
    bushes.push_back(new Bush(6, notHazardous, onFire));

    std::cout << std::endl;

    // Give the reservoir level 100 water and gas initially
    waterTank->TopUp(200);
    gasTank->TopUp(200);

    // Check the water and gas level of the reservoir
    waterTank->CheckLevel();
    gasTank->CheckLevel();

    // Load the bush fire control devices with the required resources
    hydroBlaster->Load(waterTank, 20);
    flameThrower->Load(gasTank, 10);

    // Perform fire elimination in the bushland
    std::cout << "\nPerforming fire elimination in the bushland...\n" << std::endl;

    // Add the bushes on fire into the fires map
    for (int i = 0; i < bushes.size(); i++)
    {
        if (bushes[i]->OnFire() == true)
        {
            // Save the bush to the map of hazards
            bushland->AddFire(bushes[i]->ID(), bushes[i]);

            // Eliminate the hazard
            hydroBlaster->Deploy(bushland, bushes[i]->ID());
        }    
    }

    CheckFireEliminationOutcome();
}

// Simulates the situation where there is sufficient gas supply
// at the reservoir, and the flame thrower loads enough gas to 
// eliminate all the hazards in the bushland
void TestBench::RunSufficientReservoirForHazards ()
{
    // Create the bushes and add them to the vector of bushes
    std::cout << "Creating the bushes..." << std::endl;

    bushes.push_back(new Bush(0, hazardous, notOnFire));
    bushes.push_back(new Bush(1, notHazardous, notOnFire));
    bushes.push_back(new Bush(2, hazardous, notOnFire));
    bushes.push_back(new Bush(3, hazardous, notOnFire));
    bushes.push_back(new Bush(4, notHazardous, notOnFire));

    std::cout << std::endl;

    // Give the reservoir level 100 water and gas initially
    waterTank->TopUp(100);
    gasTank->TopUp(100);

    // Check the water and gas level of the reservoir
    waterTank->CheckLevel();
    gasTank->CheckLevel();

    // Load the bush fire control devices with the required resources
    hydroBlaster->Load(waterTank, 10);
    flameThrower->Load(gasTank, 10);

    // Perform controlled burning in the bushland
    std::cout << "\nPerforming controlled burning in the bushland...\n" << std::endl;

    // Add the hazardous bushes into the hazard map
    for (int i = 0; i < bushes.size(); i++)
    {
        if (bushes[i]->IsHazard() == true)
        {
            // Save the bush to the map of hazards
            bushland->AddHazard(bushes[i]->ID(), bushes[i]);

            // Eliminate the hazard
            flameThrower->Deploy(bushland, bushes[i]->ID());
        }    
    }

    CheckControlledBurningOutcome();
}

// Simulates the situation where there is insufficient water supply
// at the reservoir, and the load request from the hydro blaster 
// was denied. As a result the bushfire manager bot could not
// eliminate any fire in the bushland
void TestBench::RunInsufficientReservoirForFires()
{
    // Create the bushes and add them to the vector of bushes
    std::cout << "Creating the bushes..." << std::endl;

    bushes.push_back(new Bush(0, notHazardous, notOnFire));
    bushes.push_back(new Bush(1, notHazardous, onFire));
    bushes.push_back(new Bush(2, notHazardous, notOnFire));
    bushes.push_back(new Bush(3, notHazardous, onFire));
    bushes.push_back(new Bush(4, notHazardous, notOnFire));
    bushes.push_back(new Bush(5, notHazardous, onFire));
    bushes.push_back(new Bush(6, notHazardous, onFire));

    std::cout << std::endl;

    // Give the reservoir level 100 water and gas initially
    waterTank->TopUp(3);
    gasTank->TopUp(200);

    // Check the water and gas level of the reservoir
    waterTank->CheckLevel();
    gasTank->CheckLevel();

    // Load the bush fire control devices with the required resources
    hydroBlaster->Load(waterTank, 20);
    flameThrower->Load(gasTank, 10);

    // Perform fire elimination in the bushland
    std::cout << "\nPerforming fire elimination in the bushland...\n" << std::endl;

    // Add the bushes on fire into the fires map
    for (int i = 0; i < bushes.size(); i++)
    {
        if (bushes[i]->OnFire() == true)
        {
            // Save the bush to the map of hazards
            bushland->AddFire(bushes[i]->ID(), bushes[i]);

            // Eliminate the hazard
            hydroBlaster->Deploy(bushland, bushes[i]->ID());
        }    
    }

    CheckFireEliminationOutcome();
}

// Checks for any remaining hazards in the bushland and displays
// whether the bushfire manager bot has successfully eliminated
// all hazards
void TestBench::CheckControlledBurningOutcome()
{
    // Check controlled burning outcome
    std::cout << "\nChecking result of controlled burning...\n" << std::endl;

    // Counter for any remaining hazards
    int remainingHazards = 0;

    // Count the number of remaining hazards
    for (int i = 0; i < bushes.size(); i++)
    {
        if (bushes[i]->IsHazard() == true)
        {
            // Increment the ramaining hazards counter
            remainingHazards++;
        }
    }

    std::cout << std::endl;

    // Indicate successfulness
    if (remainingHazards != 0)
    {
        std::cout << "Mission failed... Bad bot" << std::endl;
    }
    else
    {
        std::cout << "Mission succeeded... Good bot" << std:: endl;
    }
}

// Checks for any remaining fires in the bushland and displays
// whether the bushfire manager bot has successfully eliminated
// all fires
void TestBench::CheckFireEliminationOutcome()
{
    // Check fire elimination outcome
    std::cout << "\nChecking result of fire elimination...\n" << std::endl;

    // Counter for any remaining fires
    int remainingFires = 0;

    // Count the number of remaining fires
    for (int i = 0; i < bushes.size(); i++)
    {
        if (bushes[i]->OnFire() == true)
        {
            // Increment the ramaining fires counter
            remainingFires++;
        }
    }

    std::cout << std::endl;

    // Indicate successfulness
    if (remainingFires != 0)
    {
        std::cout << "Mission failed... Bad bot" << std::endl;
    }
    else
    {
        std::cout << "Mission succeeded... Good bot" << std:: endl;
    }
}