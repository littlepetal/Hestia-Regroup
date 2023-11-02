//--Includes-----------------------------------------------------------
#include "hestia/apriltag_detector.h"

// #include <ros/ros.h>
// #include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Int32MultiArray.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <apriltag/apriltag.h>
// #include <apriltag/tag36h11.h>
// #include <apriltag/tag25h9.h>
// #include <apriltag/apriltag_pose.h>
// #include <apriltag/tagStandard41h12.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <cv_bridge/cv_bridge.h>

// class AprilTagDetector {
// private:
//     ros::NodeHandle nh;
//     ros::Subscriber imageSub;
//     ros::Publisher tags_pub_;
//     apriltag_family_t *tf;
//     apriltag_detector_t *td;

// public:
// AprilTagDetector::AprilTagDetector() : nh("~") {
//         ROS_INFO("Node started.");
//         imageSub = nh.subscribe("/camera/image_raw/compressed", 1, &AprilTagDetector::imageCallback, this);
//         tags_pub_ = nh.advertise<std_msgs::Int32MultiArray>("/tag_detections", 1);

//         // Initialize AprilTag stuff
//         tf = tag36h11_create();
//         td = apriltag_detector_create();
//         apriltag_detector_add_family(td, tf);
//         td->quad_decimate = 1.0;
//         td->quad_sigma = 0.0;
//     }

// AprilTagDetector::~AprilTagDetector() {
//         apriltag_detector_destroy(td);
//         tag36h11_destroy(tf);
//     }

// void AprilTagDetector::imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
//     ROS_INFO("Received image callback");
//         // Convert compressed ROS image to OpenCV format
//         cv::Mat compressedImage = cv::imdecode(cv::Mat(msg->data), 1);
//         cv::Mat grayImage;
//         cv::cvtColor(compressedImage, grayImage, CV_BGR2GRAY);
//         image_u8_t im = {.width = grayImage.cols, .height = grayImage.rows, .stride = grayImage.cols, .buf = grayImage.data};

//         // Detect AprilTags
//         zarray_t *detections = apriltag_detector_detect(td, &im);
//     ROS_INFO("Detected tags count: %d", zarray_size(detections));

//         // Extract IDs and publish
//         std_msgs::Int32MultiArray tag_ids;
//         for (int i = 0; i < zarray_size(detections); i++) {
//             apriltag_detection_t *det;
//             zarray_get(detections, i, &det);
//             tag_ids.data.push_back(det->id);
//         }

//         tags_pub_.publish(tag_ids);
//     ROS_INFO("Published tag detections");
//         apriltag_detections_destroy(detections);
//     }
// };



// #include <ros/ros.h>
// #include <sensor_msgs/CompressedImage.h>
// #include <std_msgs/Int32.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <apriltag/apriltag.h>
// #include <apriltag/tag36h11.h>
// #include <apriltag/tag25h9.h>
// #include <apriltag/apriltag_pose.h>
// #include <apriltag/tagStandard41h12.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <cv_bridge/cv_bridge.h>

// class AprilTagDetector {
// private:
//     ros::NodeHandle nh;
//     ros::Subscriber imageSub;
//     ros::Publisher tagPub;
//     apriltag_family_t* tf;
//     apriltag_detector_t* td;

// public:

//--AprilTagDetector Implementation---------------------------------------------------
AprilTagDetector::AprilTagDetector() : nh("~") 
{
    ROS_INFO("Node started.");

    imageSub = nh.subscribe("/camera/image_raw/compressed", 1, &AprilTagDetector::imageCallback, this);
    tagPub = nh.advertise<std_msgs::Int32>("/tag_detection", 1);

    // Initialise AprilTag stuff
    tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 1.0;
    td->quad_sigma = 0.0;
}

AprilTagDetector::~AprilTagDetector() 
{
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
}

double computeTagSize(const apriltag_detection_t* det) 
{
    double x0 = det->p[0][0], y0 = det->p[0][1];
    double x1 = det->p[1][0], y1 = det->p[1][1];
    double x2 = det->p[2][0], y2 = det->p[2][1];
    double x3 = det->p[3][0], y3 = det->p[3][1];
    
    // Estimate tag size as average of two diagonals
    double d1 = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
    double d2 = sqrt((x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3));
    return (d1 + d2) / 2.0;
}

void AprilTagDetector::imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) 
{
    ROS_INFO("Received image callback");

    // Convert compressed ROS image to OpenCV format
    cv::Mat compressedImage = cv::imdecode(cv::Mat(msg->data), 1);
    cv::Mat grayImage;
    cv::cvtColor(compressedImage, grayImage, CV_BGR2GRAY);
    image_u8_t im = {.width = grayImage.cols, .height = grayImage.rows, .stride = grayImage.cols, .buf = grayImage.data};

    // Detect AprilTags
    zarray_t* detections = apriltag_detector_detect(td, &im);
    ROS_INFO("Detected tags count: %d", zarray_size(detections));

    // Find the largest detected tag
    int largestTagId = -1;
    double largestTagSize = -1;

    for (int i = 0; i < zarray_size(detections); i++) 
    {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        double tagSize = computeTagSize(det);

        if (tagSize > largestTagSize) 
        {
            largestTagSize = tagSize;
            largestTagId = det->id;
        }
    }

    // Publish the ID of the largest detected tag
    if (largestTagId != -1) 
    {
        std_msgs::Int32 tagIdMsg;
        tagIdMsg.data = largestTagId;
        tagPub.publish(tagIdMsg);

        ROS_INFO("Published ID of the nearest tag: %d", largestTagId);
    }

    apriltag_detections_destroy(detections);
}
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "apriltag_detector");
//     AprilTagDetector detector;
//     ros::spin();
//     return 0;
// }
