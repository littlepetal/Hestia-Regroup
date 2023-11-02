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
// #include "Monitor.h"
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <cstdlib>

//--Reservoir Interface---------------------------------------------------
class Reservoir 
{
    public:

        int id;
        std::pair<float, float> position;

        Reservoir(int id, std::pair<float, float> pos) 
            : id(id), position(pos) 
        {}
};

class Bush 
{
    public:

        int id;
        std::pair<float, float> position;
        bool onFire;
        int fireIntensity;
        bool harzard;

        Bush(int id, std::pair<float, float> pos, bool fire, int intensity, bool harzard) 
            : id(id), position(pos), onFire(fire), fireIntensity(intensity), harzard(harzard) 
        {}

        int getTagID() const { return id; }

        void setPosition(const std::pair<float, float>& pos) { position = pos; }

        void setFireStatus(bool fire, int intensity) 
        {
            onFire = fire;
            fireIntensity = fireIntensity + intensity;
        }
};

class Bushland 
{
    public:

        void addBush(const Bush& bush) 
        {
            bush.push_back(bush);
        }

        void addReservoir(const Reservoir& res) 
        {
            reservoir.push_back(res);
        }

        Bushland();

        void waterMsgCallback(const std_msgs::Int32::ConstPtr& msg);
        void goalCallback(const std_msgs::Int32::ConstPtr& msg);
        void tagDetectionCallback(const std_msgs::Int32::ConstPtr& msg);
        void fireInfoCallback(const hestia::BushFire::ConstPtr& msg);
        void odomMsgCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void modeCallback(const std_msgs::String::ConstPtr& msg);
        void saveAndUpdate();
        void reMap();

    private:

        ros::NodeHandle nh;
        ros::Subscriber tagDetectionSub;
        ros::Subscriber fireSub;
        ros::Subscriber odometrySub;
        ros::Subscriber modeSub;
        ros::Subscriber waterSub;
        ros::Subscriber goalSub;

        ros::Publisher firePub;
        ros::Publisher waterPub;
        
        
        std::vector<Bush> bush;
        std::string mode;
        std::vector<Reservoir> reservoir;

        // std::string filename = "map_data.yaml";
        std::string filename = "src/hestia/data/map_data.yaml";
        double turtleBotPose;
        int detectedId;
        std::pair<float, float> currentOdometry;
        int waterReceived;
};

#endif