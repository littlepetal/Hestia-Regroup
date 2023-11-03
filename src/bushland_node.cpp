//--Includes-----------------------------------------------------------
#include "hestia/Bushland.h"
#include "hestia/Visualizer.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "bushland");

    ros::NodeHandle nh;
    
    Bushland bushland(nh);
    Visualizer visualizer(nh, "visualization_marker", "map", "src/hestia/data/map_data.yaml");
    visualizer.run();

    ros::spin();

    return 0;
}