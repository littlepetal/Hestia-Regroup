//--Includes-----------------------------------------------------------
#include "hestia/Hestia.h"

int main(int argc, char** argv) {

    // Initial the Hestia node
    ros::init(argc, argv, "hestia_node");

    Hestia hestia;
    
    // Pump Callbacks
    ros::spin();
    
    return 0;
}