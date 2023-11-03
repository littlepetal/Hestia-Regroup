//--Includes-----------------------------------------------------------
#include "hestia/ApriltagDetector.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "apriltag_detector");

    ros::NodeHandle nh;

    AprilTagDetector detector(nh);

    ros::spin();
    
    return 0;
}