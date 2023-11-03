#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

//--Includes-----------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Int32.h>
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
// The ApriltagDetector class is responsible for using information from 
// the camera and the ROS Apriltag package to detect Apriltags in Hestia's vicinity
class AprilTagDetector 
{
    public:

        // Constructs an Apriltag detector object. Initialises the publishers 
        // and subscribers of the ApriltagDetector. Initialises the Apriltag detector
        // from the ROS Apriltag package
        AprilTagDetector();

        // Destructs the Apriltag detector object. Deallocates memory for the objects
        // created for using the ROS Apriltag package
        ~AprilTagDetector();
        
        // Callback function for the image subscriber. Receives images from the camera
        void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg);

    private:

        // Node handle
        ros::NodeHandle nh;

        // Camera image subscriber
        ros::Subscriber imageSub;

        // Detected Apriltag ID publisher
        ros::Publisher tagPub;

        // Apriltag family
        apriltag_family_t* tf;

        // ROS Apriltag package Apriltag detector
        apriltag_detector_t* td;
};

#endif