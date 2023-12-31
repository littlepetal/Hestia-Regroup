#ifndef VISUALIZER_H
#define VISUALIZER_H

//--Includes-----------------------------------------------------------
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <yaml-cpp/yaml.h>

//--Hestia Interface---------------------------------------------------
// The Visualiser class is responsible for managing the visualization markers in ROS
// based on the YAML configuration file
class Visualiser 
{
    public:
        // Constructor of the Visualizer class
        // param nh An initialized ROS NodeHandle reference
        // param marker_topic The topic name where the markers will be published
        // param frame_id The frame ID in which the markers will be placed
        // param yaml_file The path to the YAML configuration file to be parsed
        Visualiser(ros::NodeHandle& nh, const std::string& markerTopic, const std::string& frameId, const std::string& yamlFile);

        // The run function contains the main loop, which repeatedly publishes markers
        // based on the YAML configuration file.
        void run();

    private:

        ros::Publisher markerPub; // ROS publisher for publishing markers
        std::string frameId; // Frame ID for the markers
        std::string yamlFile; // Path to the YAML configuration file
        int markerId; // Unique ID for each marker

        // Initializes the common properties of the markers
        void setupMarker(visualization_msgs::Marker& marker);

        // Sets up individual markers for each bush based on the YAML configuration
        void setupBushMarker(YAML::const_iterator it, visualization_msgs::Marker& marker);
};

#endif
