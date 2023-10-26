#include "hestia/apriltag_detector.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "apriltag_detector");
    AprilTagDetector detector;
    ros::spin();
    return 0;
}