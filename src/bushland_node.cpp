//--Includes-----------------------------------------------------------
#include "hestia/Bushland.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "bushland");
    
    Bushland bushland;

    ros::spin();

    return 0;
}