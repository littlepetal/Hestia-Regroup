#include "hestia/bushland.h"

Bushland::Bushland() {
    tag_detection_sub_ = nh_.subscribe("/tag_detection", 100, &Bushland::tagDetectionCallback, this);
    fire_info_sub_ = nh_.subscribe("/bush_fire_topic", 100, &Bushland::fireInfoCallback, this);
    odom_sub_ = nh_.subscribe("odom", 100, &Bushland::odomMsgCallback, this);
    mode_sub_ = nh_.subscribe("/operation_mode_topic", 100, &Bushland::modeCallback, this);
}

// Callback function to update turtlebot's orientation using odometry data.
void Bushland::odomMsgCallback(const nav_msgs::Odometry::ConstPtr &msg)
{   
    current_odom_ = std::make_pair(msg->pose.pose.position.x, msg->pose.pose.position.y);
    // Compute yaw (heading) from quaternion.
    double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  
    tb3_pose_ = atan2(siny, cosy);
}

void Bushland::tagDetectionCallback(const std_msgs::Int32::ConstPtr& msg) {
    int detected_id = msg->data;
    
    // Check if the detected ID is between 2 and 5
    if (detected_id >= 2 && detected_id <= 5) {
        // Check if the bush with this ID already exists
        auto it = std::find_if(bushes_.begin(), bushes_.end(), [detected_id](const Bush& bush) { return bush.getTagID() == detected_id; });

        if (it == bushes_.end()) { // If the bush does not exist
            Bush new_bush(detected_id, current_odom_, false, 0);
            bushes_.push_back(new_bush);
        } else { 
            //  bool currentFireStatus = getFireDataFromSensor(detected_id);  // This is a placeholder. Replace with actual logic.
            // if (it->onFire != currentFireStatus) {
            //     it->setFireStatus(currentFireStatus, it->fireIntensity);  // Here, we update the fire status and keep the intensity same for simplicity
            // }
        }

        // Calculate the total water needed
        double total_water_needed = 0.0;
        for (const Bush& bush : bushes_) {
            if (bush.onFire) {
                total_water_needed += bush.fireIntensity;  // Here, we assume the fireIntensity directly indicates the amount of water needed
            }
        }
        
        // Sort the bushes based on fire intensity
        std::sort(bushes_.begin(), bushes_.end(), [](const Bush& a, const Bush& b) { return a.fireIntensity > b.fireIntensity; });

    }
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