#ifndef BUSHLAND_H
#define BUSHLAND_H

//--Includes-----------------------------------------------------------
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "hestia/BushFire.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <cstdlib>
#include <visualization_msgs/Marker.h>
#include <string>
#include <map>
#include <visualization_msgs/Marker.h>
#include "Bush.h"
#include "Reservoir.h"

//--Bushland Interface---------------------------------------------------
class Bushland 
{
    public:

        Bushland(ros::NodeHandle& nh);  // Constructs a bushland
        
        void waterMsgCallback(const std_msgs::Int32::ConstPtr& msg);    // Water blaster callback     
        void goalCallback(const std_msgs::Int32::ConstPtr& msg);    // Navigation callback at goal       
        void tagDetectionCallback(const std_msgs::Int32::ConstPtr& msg);    // Apriltag detection callback     
        void fireInfoCallback(const hestia::BushFire::ConstPtr& msg);   // UI fires callback     
        void odomMsgCallback(const nav_msgs::Odometry::ConstPtr& msg);  // Tutlebot odometry callback 
        void modeCallback(const std_msgs::String::ConstPtr& msg);   // Operation mode callback

        // Save and update YAML map file
        void saveAndUpdate();

        // Sort the fire intensities in the YAML file
        void reMap();

    private:

        // Set enum for ID type
        enum tagId 
        {
            RESERVOIR = 0,
            BUSH1 = 1,
            BUSH2 = 2,
            BUSH3 = 3,
            BUSH4 = 4
        };
                
        ros::Subscriber tagDetectionSub; // Apriltag detector subscriber  
        ros::Subscriber fireSub;    // Flame thrower subscriber     
        ros::Subscriber odometrySub;    // Trutlebot odometry subscriber
        ros::Subscriber modeSub;    // Hestia operation mode subscriber
        ros::Subscriber waterSub;   // Water blaster subscriber
        ros::Subscriber goalSub;    // Navigation to goal coordinate subscriber       

        ros::Publisher waterPub;    // Required water for Hestia publisher  

        std::vector<Bush> bush; // Container of all bushes in the bushland        
        std::string mode;   // Name of current operation mode
        std::vector<Reservoir> reservoir;   // Container of the all reservoirs in the bushland        
        std::string filename = "src/hestia/data/map_data.yaml"; // Path of yaml file
        std::pair<float, float> currentOdometry;    // Current odometry of Hestia

        int waterReceived;//
        int detectedId; // Most recently detected Apriltag ID
};

#endif