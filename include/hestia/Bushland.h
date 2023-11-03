#ifndef BUSHLAND_H
#define BUSHLAND_H

//--Includes-----------------------------------------------------------
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "hestia/BushFire.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
// #include "Bush.h"
// #include "Reservoir.h"
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <cstdlib>
#include <visualization_msgs/Marker.h>
#include <string>
#include <map>
#include <visualization_msgs/Marker.h>

//--Reservoir Interface---------------------------------------------------
class Reservoir 
{
    public:

        // Apriltag ID of the reservoir
        int id;

        // Coordinate of the reservoir
        std::pair<float, float> position;

        // Constructs a reservoir
        Reservoir(int id, std::pair<float, float> pos) 
            : id(id), position(pos) 
        {}
};

//--Bush Interface---------------------------------------------------
class Bush 
{
    public:

        // Apriltag ID of the bush
        int id;

        // Coordinate of the bush
        std::pair<float, float> position;

        // Flag for whether the bush is on fire
        bool onFire;

        // The intensity of the fire at the bush
        int fireIntensity;

        // Flag for whether the bush is a fire hazard
        bool harzard;

        // Constructs a bush
        Bush(int id, std::pair<float, float> pos, bool fire, int intensity, bool harzard) 
            : id(id), position(pos), onFire(fire), fireIntensity(intensity), harzard(harzard) 
        {}

        // Returns the Apriltag ID of the bush
        int getTagID() const { return id; }

        // Places the bush at the given coordinate
        void setPosition(const std::pair<float, float>& pos) { position = pos; }

        // Sets the bush on fire at the given intensity
        void setFireStatus(bool fire, int intensity) 
        {
            onFire = fire;
            fireIntensity = fireIntensity + intensity;
        }
};

//--Bushland Interface---------------------------------------------------
class Bushland 
{
    public:

        // Adds a bush to the bushland
        void addBush(const Bush& bush) 
        {
            Bushland::bush.push_back(bush);
        }

        // Adds a reservoir to the bushland
        void addReservoir(const Reservoir& res) 
        {
            reservoir.push_back(res);
        }

        // Constructs a bushland
        Bushland(ros::NodeHandle& nh);

        // Water blaster callback
        void waterMsgCallback(const std_msgs::Int32::ConstPtr& msg);

        // Navigation callback at goal
        void goalCallback(const std_msgs::Int32::ConstPtr& msg);

        // Apriltag detection call back
        void tagDetectionCallback(const std_msgs::Int32::ConstPtr& msg);

        // UI fires callback 
        void fireInfoCallback(const hestia::BushFire::ConstPtr& msg);

        // Tutlebot odometry callback to save odometry for navigation
        void odomMsgCallback(const nav_msgs::Odometry::ConstPtr& msg);

        // UI operation mode callback
        void modeCallback(const std_msgs::String::ConstPtr& msg);

        // Save and update YAML file
        void saveAndUpdate();

        // Sort the fire intensities in the YAML file
        void reMap();

    private:

        enum tagId {
            RESERVOIR = 0,
            BUSH1 = 1,
            BUSH2 = 2,
            BUSH3 = 3,
            BUSH4 = 4
        };
        
        ros::NodeHandle &nh;
        
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