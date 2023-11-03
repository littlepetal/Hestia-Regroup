#ifndef HESTIA_H
#define HESTIA_H

//--Includes-----------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <utility>  // for std::pair
#include <thread>
#include <mutex>

#include "Device.h"

//--Hestia Interface---------------------------------------------------
// Hestia is responsible for using Navstack to move the Turtlebot 
// to the goal destinations. On top of navigation, Hestia is responsible 
// for the onboard hydro blaster and flame thrower. Hestia loads 
// the hydro blaster with water and the flame thrower with gas.
// Water and gas are simplefied as resources. Hestia performs controlled
// burning and fire elimination by deploying the onboard devices at the bushes
class Hestia
{
    public:

        // Constructs Hestia and its Devices and Navstack client
        Hestia();

        // Destructs the Hestia and its Devices and Navstack client
        ~Hestia();

    private:

        std::mutex IdMutex;  // Mutual exclusion lock

        int mode;  // Current Operation mode
        int requiredResource;  // Amount of onboard resources required

        int detectedId;  // The Apriltag ID detected
        std::map<int, geometry_msgs::Pose> tagPose;  // Apriltag pose
        geometry_msgs::Pose originalPose;  // Previous pose

        std::pair<float, float> currentOdometry;  // Current odometry of Hestia

        ros::NodeHandle nh;  // Node handle for Hestia

        // The devices on Hestia
        HydroBlaster* hydroBlaster;
        FlameThrower* flameThrower;       

        ros::Subscriber modeSub; // Subscriber to the operation mode
        ros::Subscriber tagSub;  // Subscriber to the currently detected Apriltag ID
        ros::Subscriber requiredWaterSub;  // Subscriber to the amount of water required

        ros::Publisher detectedGoalPub; // Publisher goal detection
        ros::Publisher cmdPub;  // Publisher for Turtlebot velocity 

        // Callback for the operation mode subscriber
        void modeCallback(const std_msgs::String::ConstPtr& msg);

        // Callback for the Apriltag subscriber
        void tagCallback(const std_msgs::Int32::ConstPtr& msg);

        // Callback for the Turtlebot odometry subscriber
        void odomMsgCallback(const nav_msgs::Odometry::ConstPtr& msg);

        // Callback for the calculated required water amount from bushland
        void requiredWaterCallback(const std_msgs::Int32::ConstPtr& msg);

        // Perform the controlled burning or fire elimination operations
        void processDetectedID();

        // Navstack move base client 
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* moveBaseClient;

        // Drive Hestia to the given goal pose
        void moveToGoal(const geometry_msgs::Pose& goalPose);

        // Turns the Hestia bot by the given degrees
        void turn(float degrees);

        // Receives a YAML with the coordinates of all the reservoirs and bushes
        // Returns a vector of the coordinates of interest
        std::vector<std::pair<float, float>> getPositions(const std::string& filename);
};

#endif