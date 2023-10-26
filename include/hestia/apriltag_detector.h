#pragma once

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

class AprilTagDetector {
private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher tags_pub_;
    apriltag_family_t *tf_;
    apriltag_detector_t *td_;

    // double computeTagSize(const apriltag_detection_t *det);

public:
    AprilTagDetector();
    ~AprilTagDetector();

    void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg);
};