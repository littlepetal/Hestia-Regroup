//--Includes-----------------------------------------------------------
#include "hestia/bushland.h"

//--Bushland Implementation---------------------------------------------------
Bushland::Bushland() 
{
    ROS_INFO("Bushland node started");
    tagDetectionSub = nh.subscribe("/tag_detection", 100, &Bushland::tagDetectionCallback, this);
    fireSub = nh.subscribe("/bush_fire_topic", 100, &Bushland::fireInfoCallback, this);
    odometrySub = nh.subscribe("odom", 100, &Bushland::odomMsgCallback, this);
    modeSub = nh.subscribe("/operation_mode_topic", 100, &Bushland::modeCallback, this);
    waterSub = nh.subscribe("/water_dispensed", 100, &Bushland::waterMsgCallback, this);
    goalSub = nh.subscribe("/detected_goal_id", 100, &Bushland::goalCallback, this);

    waterPub = nh.advertise<std_msgs::Int32>("/water_needed", 1);
    firePub = nh.advertise<std_msgs::Int32>("/target", 1);
    waterReceived = 0;
}

void Bushland::waterMsgCallback(const std_msgs::Int32::ConstPtr &msg)
{
    waterReceived = msg->data;
}

void Bushland::goalCallback(const std_msgs::Int32::ConstPtr &msg)
{
    int goaId = msg->data;

    if (goaId >= 1 && goaId <= 4) 
    {
        ROS_INFO("Reached bush: %d and putout fire",goaId);

        // Check if the bush with this ID already exists
        auto it = std::find_if(bush.begin(), bush.end(), [goaId](const Bush& bush) { return bush.getTagID() == goaId; });

        // if (it == bush.end()) { // If the bush does not exist
        //     Bush new_bush(detectedId, currentOdometry, false, 0);
        //     bush.push_back(new_bush);
        // } else { 
        //     // If the bush exists and is on fire, reduce its fire intensity
        if (it->onFire) 
        {
            // int newFireIntensity = it->fireIntensity - waterReceived;
            int newFireIntensity = 0;

            if (newFireIntensity <= 0) 
            {
                newFireIntensity = 0;
                it->onFire = false;
            }
            it->setFireStatus(it->onFire, newFireIntensity);
        // }
        // }
        }
    }

    saveAndUpdate();
}

// Callback function to update turtlebot's orientation using odometry data.
void Bushland::odomMsgCallback(const nav_msgs::Odometry::ConstPtr &msg)
{   
    currentOdometry = std::make_pair(msg->pose.pose.position.x, msg->pose.pose.position.y);
    // Compute yaw (heading) from quaternion.
    double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  
    turtleBotPose = atan2(siny, cosy);
}

// update map status    
void Bushland::tagDetectionCallback(const std_msgs::Int32::ConstPtr& msg) 
{
    int detectedId = msg->data;

    if (detectedId == 0) 
    {
        ROS_INFO("Detect start point");

        if (reservoir.empty()) 
        {
            Reservoir newReservoir(detectedId, currentOdometry); 
            reservoir.push_back(newReservoir);
        }

        if (mode == "Start Fire Eliminating")
        {
            // Calculate the total water needed
            std_msgs::Int32 water;
                
            int requiredWater = 0;
            for (const Bush& bush : bush) 
            {
                if (bush.onFire) {
                    requiredWater += bush.fireIntensity;  // Here, we assume the fireIntensity directly indicates the amount of water needed
                }
            }

            ROS_INFO("Water needed %d",requiredWater);

            water.data = requiredWater;
            waterPub.publish(water);

            // Sort the bushes based on fire intensity
            std::sort(bush.begin(), bush.end(), [](const Bush& a, const Bush& b) { return a.fireIntensity > b.fireIntensity; });
            // reMap();

            ROS_INFO("Number of bushes after reMap: %zu", bush.size());
            if (!bush.empty() && bush.front().onFire) 
            {
                ROS_INFO("Bush with highest fire intensity ID: %d", bush.front().id);
                
                std_msgs::Int32 msg;
                // msg.data.push_back(bush.front().id);
                // fire_bush_ids_pub_.publish(msg);
            }
            
            // for (const Bush& bush : bush) {
            //     if (bush.onFire) {
            //         ROS_INFO("On fire bush ID: %d", bush.id);
            //     }
            // }
            
            // std_msgs::Int32MultiArray msg;
            // for (const Bush& bush : bush) {
            //     if (bush.onFire) {
            //         msg.data.push_back(bush.id);
            //     }
            // }
            // fire_bush_ids_pub_.publish(msg);
        }
    }
    // Check if the detected ID is between 2 and 5
    else if (detectedId >= 1 && detectedId <= 4) 
    {
        ROS_INFO("Detect bush: %d", detectedId);
        // Check if the bush with this ID already exists
        auto it = std::find_if(bush.begin(), bush.end(), [detectedId](const Bush& bush) { return bush.getTagID() == detectedId; });

        if (it == bush.end()) 
        { 
            // If the bush does not exist
            bool status = false;

            if (detectedId == 4)
            {
                status = true;
            }

            Bush new_bush(detectedId, currentOdometry, false, 0, status);
            bush.push_back(new_bush);
        } 
        else 
        { 
            // If the bush exists and is on fire, reduce its fire intensity
            if (it->onFire) 
            {
                // int newFireIntensity = it->fireIntensity - waterReceived;
                int newFireIntensity = 0;

                if (newFireIntensity <= 0) 
                {
                    newFireIntensity = 0;
                    it->onFire = false;
                }

                it->setFireStatus(it->onFire, newFireIntensity);
            }
        }
    }
    // else if (detectedId == 6) {
    //     Reservoir(detectedId, currentOdometry); 
    // }
    saveAndUpdate();
}

void Bushland::fireInfoCallback(const hestia::BushFire::ConstPtr& msg) 
{
    // Find the bush with the given ID and update its fire status
    ROS_INFO("fire status received");

    for (Bush& bush : bush) 
    {
        if (bush.getTagID() == msg->bushId) 
        {
            bush.setFireStatus(msg->isOnFire, msg->fireIntensity);
        }
    }

    saveAndUpdate();
}

void Bushland::modeCallback(const std_msgs::String::ConstPtr& msg) 
{
    mode = msg->data;
    ROS_INFO("%s mode", mode.c_str());
}

void Bushland::saveAndUpdate() 
{
    YAML::Node config;

    if (std::ifstream(filename)) 
    {
        config = YAML::LoadFile(filename);
    }

    for (Bush& bush : bush) 
    {
        std::string bushKey = "bush" + std::to_string(bush.id);
        YAML::Node bushPosition;
        bushPosition.push_back(bush.position.first);
        bushPosition.push_back(bush.position.second);

        if (config[bushKey]) 
        {
            // Update only if bush has disappeared or fire intensity has increased
            if (!bush.onFire || bush.fireIntensity > config[bushKey]["fireIntensity"].as<int>()) 
            {
                config[bushKey]["position"] = bushPosition;
                config[bushKey]["onFire"] = bush.onFire;
                config[bushKey]["fireIntensity"] = bush.fireIntensity;
                config[bushKey]["harzard"] = bush.harzard;  // store the harzard at
            }
        } 
        else 
        {
            // Add new bush
            config[bushKey]["position"] = bushPosition;
            config[bushKey]["onFire"] = bush.onFire;
            config[bushKey]["fireIntensity"] = bush.fireIntensity;
            config[bushKey]["harzard"] = bush.harzard;  // store the harzard at
        }
    }

    for (Reservoir& res : reservoir) 
    {
        std::string resKey = "reservoir" + std::to_string(res.id);
        YAML::Node reservoirPosition;
        reservoirPosition.push_back(res.position.first);
        reservoirPosition.push_back(res.position.second);

        config[resKey]["position"] = reservoirPosition;
    }

    std::ofstream fout(filename);
    fout << config;
    fout.close();
}

void Bushland::reMap() 
{
    ROS_INFO("Entering reMap function.");
    ROS_INFO("Number of bushes at start of reMap: %zu", bush.size());

    YAML::Node config;

    if (std::ifstream(filename)) 
    {
        config = YAML::LoadFile(filename);
    }

    // 从YAML中提取并临时存储所有Bush和Reservoir的数据
    std::vector<YAML::Node> extractedBush;
    for (Bush& bush : bush) 
    {
        std::string bushKey = "bush" + std::to_string(bush.id);
        if (config[bushKey]) 
        {
            extractedBush.push_back(config[bushKey]);
        }
    }

    // 根据fireIntensity对提取的Bush数据进行排序
    std::sort(extractedBush.begin(), extractedBush.end(), [](const YAML::Node& a, const YAML::Node& b) { 
        return a["fireIntensity"].as<int>() > b["fireIntensity"].as<int>(); 
    });

    // 从YAML文件中清除Bush和Reservoir的条目
    for (Bush& bush : bush) 
    {
        std::string bushKey = "bush" + std::to_string(bush.id);
        config.remove(bushKey);
    }

    for (Reservoir& res : reservoir) 
    {
        std::string resKey = "reservoir" + std::to_string(res.id);
        config.remove(resKey);
    }

    // 将排序后的Bush数据写回YAML文件
    for (YAML::Node& bushNode : extractedBush) 
    {
        std::string bushKey = "bush" + std::to_string(bushNode["id"].as<int>());
        config[bushKey] = bushNode;
    }

    // 将Reservoir数据写回YAML文件
    for (Reservoir& res : reservoir) 
    {
        std::string resKey = "reservoir" + std::to_string(res.id);
        YAML::Node reservoirPosition;
        reservoirPosition.push_back(res.position.first);
        reservoirPosition.push_back(res.position.second);
        config[resKey]["position"] = reservoirPosition;
    }

    std::ofstream fout(filename);
    fout << config;
    fout.close();
}