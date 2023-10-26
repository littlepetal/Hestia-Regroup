#include "hestia/bushland.h"

Bushland::Bushland() {
    tag_detection_sub_ = nh_.subscribe("/tag_detection", 100, &Bushland::tagDetectionCallback, this);
    fire_info_sub_ = nh_.subscribe("/bush_fire_topic", 100, &Bushland::fireInfoCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 100, &Bushland::odomCallback, this);
    mode_sub_ = nh_.subscribe("/operation_mode_topic", 100, &Bushland::modeCallback, this);
}

void Bushland::tagDetectionCallback(const std_msgs::Int32::ConstPtr& msg) {
    msg->data 
    // // Find the bush with the detected tag and update its position
    // for (Bush& bush : bushes_) {
    //     if (bush.getTagID() == msg->tag_id) {
    //         bush.setPosition(current_odom_.pose.pose.position);
    //     }
    // }
}

void Bushland::fireInfoCallback(const hestia::BushFire::ConstPtr& msg) {
    // Find the bush with the given ID and update its fire status
    ROS_INFO("fire status received");
    for (Bush& bush : bushes_) {
        if (bush.getTagID() == msg->bush_id) {
            bush.setFireStatus(msg->is_on_fire, msg->fire_intensity);
        }
    }
    saveAndUpdate();
}

void Bushland::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom_ = *msg;
}

void Bushland::modeCallback(const std_msgs::String::ConstPtr& msg) {
    mode_ = msg->data;
}

void Bushland::saveAndUpdate() {
    YAML::Node config;
    if (std::ifstream(filename)) {
        config = YAML::LoadFile(filename);
    }

    for (Bush& bush : bushes_) {
        std::string bushKey = "bush" + std::to_string(bush.id);
        YAML::Node bush_position;
        bush_position.push_back(bush.position.first);
        bush_position.push_back(bush.position.second);

        if (config[bushKey]) {
            // Update only if bush has disappeared or fire intensity has increased
            if (!bush.onFire || bush.fireIntensity > config[bushKey]["fireIntensity"].as<int>()) {
                config[bushKey]["position"] = bush_position;
                config[bushKey]["onFire"] = bush.onFire;
                config[bushKey]["fireIntensity"] = bush.fireIntensity;
            }
        } else {
            // Add new bush
            config[bushKey]["position"] = bush_position;
            config[bushKey]["onFire"] = bush.onFire;
            config[bushKey]["fireIntensity"] = bush.fireIntensity;
        }
    }

    for (Reservoir& res : reservoirs) {
        std::string resKey = "reservoir" + std::to_string(res.id);
        YAML::Node res_position;
        res_position.push_back(res.position.first);
        res_position.push_back(res.position.second);

        config[resKey]["position"] = res_position;
    }

    std::ofstream fout(filename);
    fout << config;
    fout.close();
}
