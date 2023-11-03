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

        // Constructs an Apriltag detector object
        AprilTagDetector();

        // Destructs the Apriltag detector object
        ~AprilTagDetector();
        
        // Callback function for the image subscriber
        void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg);

    private:

        // Node handle
        ros::NodeHandle nh;

        // Camera image subscriber
        ros::Subscriber imageSub;

        // Apriltag publisher
        ros::Publisher tagPub;

        // Frame for camera
        apriltag_family_t* tf;

        // ROS 
        apriltag_detector_t* td;
};

#endif