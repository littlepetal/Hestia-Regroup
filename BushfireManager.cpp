// This program simulates the high level functionalities of a bushfire manager bot
// The bot can perform controlled burning and fire elimination
// It is intended that this program is extended to include the use of OpenCV and
// ROS nodes to implement the simulations on the physical turtlebot

// TO DO: 
// Reduce similaer code
// Add simulations for more scenarios
// Add functionality to return to reservoir should there be insufficient resources onboard
// Add Camera
// Add ArUco tags and associated functionalities
// Add navigation functionalities

//--Includes-----------------------------------------------------------
#include "TestBench.h"

#include <stdio.h>
#include <iostream>

int main() 
{
    // Simulate fire elimination mode
    TestBench* testBench = new TestBench();
    testBench->RunSufficientReservoirForFires();

    // Simulate controlled burning mode
    // TestBench* testBench = new TestBench();
    // testBench->RunSufficientReservoirForHazards();

    // Simulate fire elimination mode with insufficient water in the reservoir
    // TestBench* testBench = new TestBench();
    // testBench->RunInsufficientReservoirForFires();

    // Free up memory
    delete testBench;

    return 0;
}