#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

//--Includes-----------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <stdmsgs/Int32.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag25h9.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tagStandard41h12.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

//--AprilTagDetector Interface---------------------------------------------------
class AprilTagDetector 
{
    public:

        AprilTagDetector();
        ~AprilTagDetector();

        void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg);

    private:
    
        ros::NodeHandle nh;
        ros::Subscriber imageSub;
        ros::Publisher tagPub;
        apriltag_family_t* tf;
        apriltag_detector_t* td;

        // double computeTagSize(const apriltag_detection_t *det);
};

#endif