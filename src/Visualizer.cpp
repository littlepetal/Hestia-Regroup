// Visualizer.cpp

#include "hestia/Visualizer.h"

// Constructor implementation
Visualizer::Visualizer(ros::NodeHandle& nh, const std::string& marker_topic, const std::string& frame_id, const std::string& yaml_file)
    : nh(nh), marker_pub_(nh.advertise<visualization_msgs::Marker>(marker_topic, 10)),
      frame_id_(frame_id), yaml_file_(yaml_file), marker_id_(0) {
}

// Run function implementation
void Visualizer::run() {
    visualization_msgs::Marker marker;
    setupMarker(marker);

    YAML::Node config = YAML::LoadFile(yaml_file_);

    while (ros::ok()) {
        for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
            setupBushMarker(it, marker);
            marker_pub_.publish(marker);
        }
        ros::Duration(1.0).sleep();
    }
}

// Setup the marker with default properties
void Visualizer::setupMarker(visualization_msgs::Marker& marker) {
    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 1.0;
    marker.color.a = 1.0; // Set alpha to fully opaque
    marker.lifetime = ros::Duration(); // Marker persists indefinitely unless manually removed
}

// Setup individual bush markers based on the YAML configuration
void Visualizer::setupBushMarker(YAML::const_iterator it, visualization_msgs::Marker& marker) {
    std::string bushName = it->first.as<std::string>();
    YAML::Node bush_data = it->second;
    bool isOnFire = bush_data["onFire"].as<bool>(false); // Defaults to false if not present

    ROS_INFO_STREAM("Bush Name: " << bushName << ", On Fire: " << (isOnFire ? "Yes" : "No"));

    // Set marker color based on fire status
    marker.color.r = isOnFire ? 1.0f : 0.0f; // Red if on fire
    marker.color.g = !isOnFire ? 1.0f : 0.0f; // Green if not
    marker.color.b = 0.0f; // Blue component is 0

    marker.id = marker_id_++; // Increment marker ID
    marker.pose.position.x = bush_data["position"][0].as<double>(); // Set marker position
    marker.pose.position.y = bush_data["position"][1].as<double>();
    marker.pose.position.z = 0; // Z is set to 0 assuming a flat ground
    marker.pose.orientation.x = 0.0; // Orientation is neutral (no rotation)
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
}

// The main function would be in a separate file that includes Visualizer.h
// The YAML dependency should be handled in the CMakeLists.txt for proper linking.
