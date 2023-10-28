#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "hestia/BushFire.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
// #include "Bush.h"
// #include "Monitor.h"
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <cstdlib>


class Reservoir {
public:
    int id;
    std::pair<float, float> position;

    Reservoir(int id, std::pair<float, float> pos) 
        : id(id), position(pos) {}
};

class Bush {
public:
    int id;
    std::pair<float, float> position;
    bool onFire;
    int fireIntensity;

    Bush(int id, std::pair<float, float> pos, bool fire, int intensity) 
        : id(id), position(pos), onFire(fire), fireIntensity(intensity) {}

    int getTagID() const { return id; }
    void setPosition(const std::pair<float, float>& pos) { position = pos; }
    void setFireStatus(bool fire, int intensity) {
        onFire = fire;
        fireIntensity = fireIntensity+intensity;
    }
};

class Bushland {
public:
    void addBush(const Bush& bush) {
        bushes_.push_back(bush);
    }

    void addReservoir(const Reservoir& res) {
        reservoirs.push_back(res);
    }

    Bushland();
    void waterMsgCallback(const std_msgs::Int32::ConstPtr& msg);
    void starterCallback(const std_msgs::Int32::ConstPtr& msg);
    void tagDetectionCallback(const std_msgs::Int32::ConstPtr& msg);
    void fireInfoCallback(const hestia::BushFire::ConstPtr& msg);
    void odomMsgCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void modeCallback(const std_msgs::String::ConstPtr& msg);
    void saveAndUpdate();
    void reMap();

private:
    ros::NodeHandle nh_;
    ros::Subscriber tag_detection_sub_;
    ros::Subscriber fire_info_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber mode_sub_;
    ros::Subscriber water_sub_;
    // ros::Subscriber starter_sub_;

    ros::Publisher fires_pub_;
    ros::Publisher water_pub_;
    
    
    std::vector<Bush> bushes_;
    // Monitor monitor_;
    std::string mode_;
    std::vector<Reservoir> reservoirs;
    // std::string filename = "map_data.yaml";
    std::string filename = "src/hestia/data/map_data.yaml";
    double tb3_pose_;
    int detected_id;
    std::pair<float, float> current_odom_;
    int water_received;

};
