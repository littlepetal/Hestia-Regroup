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
//     ros::NodeHandle nh_;
//     ros::Subscriber image_sub_;
//     ros::Publisher tags_pub_;
//     apriltag_family_t *tf_;
//     apriltag_detector_t *td_;

// public:
AprilTagDetector::AprilTagDetector() : nh_("~") {
        ROS_INFO("Node started.");
        image_sub_ = nh_.subscribe("/camera/image_raw/compressed", 1, &AprilTagDetector::imageCallback, this);
        tags_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/tag_detections", 1);

        // Initialize AprilTag stuff
        tf_ = tag36h11_create();
        td_ = apriltag_detector_create();
        apriltag_detector_add_family(td_, tf_);
        td_->quad_decimate = 1.0;
        td_->quad_sigma = 0.0;
    }

AprilTagDetector::~AprilTagDetector() {
        apriltag_detector_destroy(td_);
        tag36h11_destroy(tf_);
    }

void AprilTagDetector::imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    ROS_INFO("Received image callback");
        // Convert compressed ROS image to OpenCV format
        cv::Mat compressedImage = cv::imdecode(cv::Mat(msg->data), 1);
        cv::Mat grayImage;
        cv::cvtColor(compressedImage, grayImage, CV_BGR2GRAY);
        image_u8_t im = {.width = grayImage.cols, .height = grayImage.rows, .stride = grayImage.cols, .buf = grayImage.data};

        // Detect AprilTags
        zarray_t *detections = apriltag_detector_detect(td_, &im);
    ROS_INFO("Detected tags count: %d", zarray_size(detections));

        // Extract IDs and publish
        std_msgs::Int32MultiArray tag_ids;
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            tag_ids.data.push_back(det->id);
        }

        tags_pub_.publish(tag_ids);
    ROS_INFO("Published tag detections");
        apriltag_detections_destroy(detections);
    }
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
//     ros::NodeHandle nh_;
//     ros::Subscriber image_sub_;
//     ros::Publisher tag_pub_;
//     apriltag_family_t *tf_;
//     apriltag_detector_t *td_;

// public:
//     AprilTagDetector() : nh_("~") {
//         ROS_INFO("Node started.");
//         image_sub_ = nh_.subscribe("/camera/image_raw/compressed", 1, &AprilTagDetector::imageCallback, this);
//         tag_pub_ = nh_.advertise<std_msgs::Int32>("/nearest_tag", 1);

//         // Initialize AprilTag stuff
//         tf_ = tag36h11_create();
//         td_ = apriltag_detector_create();
//         apriltag_detector_add_family(td_, tf_);
//         td_->quad_decimate = 1.0;
//         td_->quad_sigma = 0.0;
//     }

//     ~AprilTagDetector() {
//         apriltag_detector_destroy(td_);
//         tag36h11_destroy(tf_);
//     }

//     double computeTagSize(const apriltag_detection_t *det) {
//         double x0 = det->p[0][0], y0 = det->p[0][1];
//         double x1 = det->p[1][0], y1 = det->p[1][1];
//         double x2 = det->p[2][0], y2 = det->p[2][1];
//         double x3 = det->p[3][0], y3 = det->p[3][1];
        
//         // Estimate tag size as average of two diagonals
//         double d1 = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
//         double d2 = sqrt((x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3));
//         return (d1 + d2) / 2.0;
//     }

//     void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
//         ROS_INFO("Received image callback");

//         // Convert compressed ROS image to OpenCV format
//         cv::Mat compressedImage = cv::imdecode(cv::Mat(msg->data), 1);
//         cv::Mat grayImage;
//         cv::cvtColor(compressedImage, grayImage, CV_BGR2GRAY);
//         image_u8_t im = {.width = grayImage.cols, .height = grayImage.rows, .stride = grayImage.cols, .buf = grayImage.data};

//         // Detect AprilTags
//         zarray_t *detections = apriltag_detector_detect(td_, &im);
//         ROS_INFO("Detected tags count: %d", zarray_size(detections));

//         // Find the largest detected tag
//         int largest_tag_id = -1;
//         double largest_tag_size = -1;
//         for (int i = 0; i < zarray_size(detections); i++) {
//             apriltag_detection_t *det;
//             zarray_get(detections, i, &det);
//             double tag_size = computeTagSize(det);
//             if (tag_size > largest_tag_size) {
//                 largest_tag_size = tag_size;
//                 largest_tag_id = det->id;
//             }
//         }

//         // Publish the ID of the largest detected tag
//         if (largest_tag_id != -1) {
//             std_msgs::Int32 tag_id_msg;
//             tag_id_msg.data = largest_tag_id;
//             tag_pub_.publish(tag_id_msg);
//             ROS_INFO("Published ID of the nearest tag: %d", largest_tag_id);
//         }

//         apriltag_detections_destroy(detections);
//     }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "apriltag_detector");
//     AprilTagDetector detector;
//     ros::spin();
//     return 0;
// }
