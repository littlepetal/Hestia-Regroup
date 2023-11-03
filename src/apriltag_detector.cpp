//--Includes-----------------------------------------------------------
#include "hestia/apriltag_detector.h"
#include <std_msgs/Int32MultiArray.h>

//--AprilTagDetector Implementation---------------------------------------------------
// ApriltagDetector constructor. Initialises the ApriltagDetector node, 
// subscriber for camera image, and publisher for Apriltag detection
AprilTagDetector::AprilTagDetector() : nh("~") 
{
    // Notification for starting the node
    ROS_INFO("Node started.");

    // Initialise image subscriber with image callback function
    imageSub = nh.subscribe("/camera/image_raw/compressed", 1, &AprilTagDetector::imageCallback, this);

    // Initialise Apriltag detection publisher with the Apriltag detection topic
    tagPub = nh.advertise<std_msgs::Int32>("/tag_detection", 1);

    // Initialise Apriltag 36h11 family
    tf = tag36h11_create();

    // Initialise Apriltag detector
    td = apriltag_detector_create();

    // Add Apriltag family
    apriltag_detector_add_family(td, tf);

    // Decimate input image by factor
    td->quad_decimate = 1.0;

    // Set no blur
    td->quad_sigma = 0.0;
}

// Destructor for ApriltagDetector. Deallocate memory for Apriltag constructs
AprilTagDetector::~AprilTagDetector() 
{
    // Deallocate memory for the Apriltag detector
    apriltag_detector_destroy(td);

    // Deallocate memory for the Apriltag family
    tag36h11_destroy(tf);
}

// Calculates tag size according to the pixel position of the four corners
double computeTagSize(const apriltag_detection_t* tag) 
{
    // Initialise the coordinates of the four corners
    double x0 = tag->p[0][0], y0 = tag->p[0][1];
    double x1 = tag->p[1][0], y1 = tag->p[1][1];
    double x2 = tag->p[2][0], y2 = tag->p[2][1];
    double x3 = tag->p[3][0], y3 = tag->p[3][1];
    
    // Estimate the tag size using the average lenght of the two diagonals
    double d1 = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
    double d2 = sqrt((x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3));

    // Return tag size
    return (d1 + d2) / 2.0;
}

// Callback function for receiving images from the camera
void AprilTagDetector::imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) 
{
    // Notification for receiving the callback
    ROS_INFO("Received image callback");

    // Convert compressed ROS image to OpenCV format
    cv::Mat compressedImage = cv::imdecode(cv::Mat(msg->data), 1);
    cv::Mat grayImage;
    cv::cvtColor(compressedImage, grayImage, CV_BGR2GRAY);
    image_u8_t im = {.width = grayImage.cols, .height = grayImage.rows, .stride = grayImage.cols, .buf = grayImage.data};

    // Detect Apriltags
    zarray_t* detections = apriltag_detector_detect(td, &im);

    // Notification for detecting Apriltags
    ROS_INFO("Detected tags count: %d", zarray_size(detections));

    // Initialise variables for the ID and size of the largest detected tag
    int largestTagId = -1;
    double largestTagSize = -1;

    // Find the largest detected tag
    for (int i = 0; i < zarray_size(detections); i++) 
    {
        // Get the tag size of the current tag from the array of detected tags
        apriltag_detection_t *tag;
        zarray_get(detections, i, &tag);
        double tagSize = computeTagSize(tag);

        // Update largest tag size if a larger tag is found
        if (tagSize > largestTagSize) 
        {
            largestTagSize = tagSize;
            largestTagId = tag->id;
        }
    }

    // Publish the ID of the largest detected tag (tag of interest)
    if (largestTagId != -1) 
    {
        // Initialise message
        std_msgs::Int32 tagIdMsg;

        // Add the ID of the largest detected Apriltag to the message
        tagIdMsg.data = largestTagId;

        // Publish the tag ID
        tagPub.publish(tagIdMsg);

        // Notification for publishing the ID of the detected tag 
        ROS_INFO("Published ID of the nearest tag: %d", largestTagId);
    }

    // Deallocate memory for the array of detected Apriltags
    apriltag_detections_destroy(detections);
}

