// Visualizer.cpp

#include "hestia/Visualizer.h"

// Constructor implementation
Visualiser::Visualiser(ros::NodeHandle& nh, const std::string& markerTopic, const std::string& frameId, const std::string& yamlFile)
    : markerPub(nh.advertise<visualization_msgs::Marker>(markerTopic, 10)),
      frameId(frameId), yamlFile(yamlFile), markerId(0) {
}

// Run function implementation
void Visualiser::run() {
    visualization_msgs::Marker marker;
    setupMarker(marker);

    YAML::Node config = YAML::LoadFile(yamlFile);

    while (ros::ok()) {
        for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
            setupBushMarker(it, marker);
            markerPub.publish(marker);
        }
        ros::Duration(1.0).sleep();
    }
}

// Setup the marker with default properties
void Visualiser::setupMarker(visualization_msgs::Marker& marker) {
    marker.header.frame_id = frameId;
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 1.0;
    marker.color.a = 1.0; 
    marker.lifetime = ros::Duration(); // Marker persists indefinitely unless manually removed
}

// Setup individual bush markers based on the YAML configuration
void Visualiser::setupBushMarker(YAML::const_iterator it, visualization_msgs::Marker& marker) {
    std::string bushName = it->first.as<std::string>();
    YAML::Node bushData = it->second;
    bool isOnFire = bushData["onFire"].as<bool>(false); // Defaults to false if not present

    ROS_INFO_STREAM("Bush Name: " << bushName << ", On Fire: " << (isOnFire ? "Yes" : "No"));

    // Set marker color based on fire status
    marker.color.r = isOnFire ? 1.0f : 0.0f; // Red if on fire
    marker.color.g = !isOnFire ? 1.0f : 0.0f; // Green if not
    marker.color.b = 0.0f; // Blue component is 0

    marker.id = markerId++; // Increment marker ID
    marker.pose.position.x = bushData["position"][0].as<double>(); // Set marker position
    marker.pose.position.y = bushData["position"][1].as<double>();
    marker.pose.position.z = 0; // Z is set to 0 assuming a flat ground
    marker.pose.orientation.x = 0.0; // Orientation is neutral (no rotation)
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
}

