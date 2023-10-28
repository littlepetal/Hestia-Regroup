#include "hestia/bushland.h"

Bushland::Bushland() {
    ROS_INFO("Bushland node started");
    tag_detection_sub_ = nh_.subscribe("/tag_detection", 100, &Bushland::tagDetectionCallback, this);
    fire_info_sub_ = nh_.subscribe("/bush_fire_topic", 100, &Bushland::fireInfoCallback, this);
    odom_sub_ = nh_.subscribe("odom", 100, &Bushland::odomMsgCallback, this);
    mode_sub_ = nh_.subscribe("/operation_mode_topic", 100, &Bushland::modeCallback, this);
    water_sub_ = nh_.subscribe("/water_dispensed",100, &Bushland::waterMsgCallback,this);
    // starter_sub_ = nh_.subscribe("/start_a_loop",100, &Bushland::starterCallback,this);

    water_pub_ = nh_.advertise<std_msgs::Int32>("/water_needed", 1);
    // fires_pub_ = nh_.advertise<std_msgs::Int32>("/tag_detection", 1);
}

void Bushland::waterMsgCallback(const std_msgs::Int32::ConstPtr &msg){
    water_received = msg->data;
}

// void Bushland::starterCallback(const std_msgs::Int32::ConstPtr &msg){
//     // Each time at starating point[]

// }

// Callback function to update turtlebot's orientation using odometry data.
void Bushland::odomMsgCallback(const nav_msgs::Odometry::ConstPtr &msg)
{   
    current_odom_ = std::make_pair(msg->pose.pose.position.x, msg->pose.pose.position.y);
    // Compute yaw (heading) from quaternion.
    double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  
    tb3_pose_ = atan2(siny, cosy);
}

// update map status    
void Bushland::tagDetectionCallback(const std_msgs::Int32::ConstPtr& msg) {
    int detected_id = msg->data;
    if (detected_id == 0) {
        if (mode_ == "Start Fire Eliminating"){
        // Calculate the total water needed
        std_msgs::Int32 water;
            
        int total_water_needed = 0;
        for (const Bush& bush : bushes_) {
            if (bush.onFire) {
                total_water_needed += bush.fireIntensity;  // Here, we assume the fireIntensity directly indicates the amount of water needed
            }
        }
        ROS_INFO("Water needed %d",total_water_needed);
        water.data = total_water_needed;
        water_pub_.publish(water);

        // Sort the bushes based on fire intensity
        std::sort(bushes_.begin(), bushes_.end(), [](const Bush& a, const Bush& b) { return a.fireIntensity > b.fireIntensity; });
        for (const Bush& bush : bushes_) {
            if (bush.onFire) {
                ROS_INFO("On fire bush ID: %d", bush.id);
            }
        }
        // reMap();
        }
    }
    // Check if the detected ID is between 2 and 5
    else if (detected_id >= 2 && detected_id <= 5) {
        // Check if the bush with this ID already exists
        auto it = std::find_if(bushes_.begin(), bushes_.end(), [detected_id](const Bush& bush) { return bush.getTagID() == detected_id; });

        if (it == bushes_.end()) { // If the bush does not exist
            Bush new_bush(detected_id, current_odom_, false, 0);
            bushes_.push_back(new_bush);
        } else { 
            // If the bush exists and is on fire, reduce its fire intensity
            if (it->onFire) {
                int new_intensity = it->fireIntensity - water_received;
                if (new_intensity <= 0) {
                    new_intensity = 0;
                    it->onFire = false;
                }
                it->setFireStatus(it->onFire, new_intensity);
            }
        }
    }
    else if (detected_id == 6) {
        Reservoir(detected_id, current_odom_); 
    }
    saveAndUpdate();
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
    ROS_INFO("%s mode",mode_.c_str());
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

void Bushland::reMap() {
    YAML::Node config;
    if (std::ifstream(filename)) {
        config = YAML::LoadFile(filename);
    }

    // 从YAML中提取并临时存储所有Bush和Reservoir的数据
    std::vector<YAML::Node> extracted_bushes;
    for (Bush& bush : bushes_) {
        std::string bushKey = "bush" + std::to_string(bush.id);
        if (config[bushKey]) {
            extracted_bushes.push_back(config[bushKey]);
        }
    }

    // 根据fireIntensity对提取的Bush数据进行排序
    std::sort(extracted_bushes.begin(), extracted_bushes.end(), [](const YAML::Node& a, const YAML::Node& b) { 
        return a["fireIntensity"].as<int>() > b["fireIntensity"].as<int>(); 
    });

    // 从YAML文件中清除Bush和Reservoir的条目
    for (Bush& bush : bushes_) {
        std::string bushKey = "bush" + std::to_string(bush.id);
        config.remove(bushKey);
    }
    for (Reservoir& res : reservoirs) {
        std::string resKey = "reservoir" + std::to_string(res.id);
        config.remove(resKey);
    }

    // 将排序后的Bush数据写回YAML文件
    for (YAML::Node& bush_node : extracted_bushes) {
        std::string bushKey = "bush" + std::to_string(bush_node["id"].as<int>());
        config[bushKey] = bush_node;
    }

    // 将Reservoir数据写回YAML文件
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