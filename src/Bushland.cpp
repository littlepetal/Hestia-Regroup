//--Includes-----------------------------------------------------------
#include "hestia/Bushland.h"

//--Bushland Implementation---------------------------------------------------
// Constructor for the Bushland class
Bushland::Bushland(ros::NodeHandle& nh) : nh(nh)
{
    ROS_INFO("Bushland node started");
    tagDetectionSub = nh.subscribe("/tag_detection", 100, &Bushland::tagDetectionCallback, this);
    fireSub = nh.subscribe("/bush_fire_topic", 100, &Bushland::fireInfoCallback, this);
    odometrySub = nh.subscribe("odom", 100, &Bushland::odomMsgCallback, this);
    modeSub = nh.subscribe("/operation_mode_topic", 100, &Bushland::modeCallback, this);
    waterSub = nh.subscribe("/water_dispensed", 100, &Bushland::waterMsgCallback, this);
    goalSub = nh.subscribe("/detected_goal_id", 100, &Bushland::goalCallback, this);

    waterPub = nh.advertise<std_msgs::Int32>("/water_needed", 1);
    waterReceived = 0;
}

// Receive water sprayed by hestia bot, used to update fire status
void Bushland::waterMsgCallback(const std_msgs::Int32::ConstPtr &msg)
{
    waterReceived = msg->data;
}

// Callback function that handles the goal ID received, indicating a bush to be burned or a fire to be extinguished
void Bushland::goalCallback(const std_msgs::Int32::ConstPtr &msg)
{
    int goalId = msg->data;

    if (mode == "Start Control Burning")
    {
        // If reached bush need to be burned, remove it from vector
        bush.erase(std::remove_if(bush.begin(), bush.end(), 
                          [goalId](const Bush& b) { return b.getTagID() == goalId; }), 
           bush.end()); 
    }
    else if (mode == "Start Fire Eliminating")
    {
        // id 1-4 indicates bush
        if (goalId >= 1 && goalId <= 4) 
        {
            ROS_INFO("Reached bush: %d and putout fire",goalId);
    
            // Check if the bush with this ID already exists
            auto it = std::find_if(bush.begin(), bush.end(), [goalId](const Bush& bush) { return bush.getTagID() == goalId; });
            
            // If the bush exists and is on fire, reduce its fire intensity
            if (it->onFire) 
            {
                int newFireIntensity = it->fireIntensity - waterReceived;
                if (newFireIntensity <= 0) 
                {
                    newFireIntensity = 0;
                    it->onFire = false;
                }
                // Update bush status
                it->setFireStatus(it->onFire, newFireIntensity);
            }
        }
    }
    saveAndUpdate();
}

// Callback function to update turtlebot's orientation using odometry data.
void Bushland::odomMsgCallback(const nav_msgs::Odometry::ConstPtr &msg)
{   
    currentOdometry = std::make_pair(msg->pose.pose.position.x, msg->pose.pose.position.y);
}

//  
void Bushland::tagDetectionCallback(const std_msgs::Int32::ConstPtr& msg) 
{
    int detectedId = msg->data;

    if (mode == "Start Mapping")
    {
        if (detectedId == 0)
        {
            if (reservoir.empty()) 
            {
                //
                Reservoir newReservoir(detectedId, currentOdometry); 
                reservoir.push_back(newReservoir);
            }
        }
        else if (detectedId >= 1 && detectedId <= 4) 
        {
            ROS_INFO("Detect bush: %d", detectedId);

            // Check if the bush with this ID already exists
            auto it = std::find_if(bush.begin(), bush.end(), [detectedId](const Bush& bush) { return bush.getTagID() == detectedId; });
            
            if (it == bush.end()) 
            { 
                // If the bush does not exist
                bool status = false;
                
                // Assume a bush to do control burning
                if (detectedId == 4)  {status = true; }
    
                // Create new bush object
                Bush new_bush(detectedId, currentOdometry, false, 0, status);
                bush.push_back(new_bush);
            } 
        }
    }
        
    else if (mode == "Start Fire Eliminating")
    {
        //At start point (reservoir position)
        if (detectedId == 0) 
        {
            ROS_INFO("Detect start point");
            std_msgs::Int32 water;
            int requiredWater = 0;
            
            // Calculate the total water needed
            for (const Bush& bush : bush) 
            {
                if (bush.onFire) {
                    requiredWater += bush.fireIntensity;
                }
            }

            // Publish water needed to Hestia, 
            ROS_INFO("Water needed %d",requiredWater);
            water.data = requiredWater;
            waterPub.publish(water);

            // Sort the bushes based on fire intensity, reorder the map so that hestia knows where to go first
            std::sort(bush.begin(), bush.end(), [](const Bush& a, const Bush& b) { return a.fireIntensity > b.fireIntensity; });
            reMap();
        }
    }
    saveAndUpdate();
}

// Callback to handle simulated bushfire infomation generated by user interface
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
    // Update the map
    saveAndUpdate();
}

// Callback to reseive hestia bot operation mode
void Bushland::modeCallback(const std_msgs::String::ConstPtr& msg) 
{
    mode = msg->data;
    ROS_INFO("%s mode", mode.c_str());
}

// Function to save the current state and update the configuration file
void Bushland::saveAndUpdate() 
{
    YAML::Node config;

    if (std::ifstream(filename)) 
    {
        config = YAML::LoadFile(filename);
    }

    // Create a set of bush IDs that are currently in the vector.
    std::set<int> currentBushIds;
    for (const Bush& bush : bush) 
    {
        currentBushIds.insert(bush.id);
    }

    // Go through the YAML file and remove entries that are not in the currentBushIds.
    std::vector<std::string> keysToRemove;
    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) 
    {
        // Assuming all bush keys start with "bush" prefix.
        if (it->first.as<std::string>().find("bush") == 0) 
        {
            int id = std::stoi(it->first.as<std::string>().substr(4)); // "bush" is 4 characters
            if (currentBushIds.find(id) == currentBushIds.end()) 
            {
                // If the ID is not found in currentBushIds, mark for removal.
                keysToRemove.push_back(it->first.as<std::string>());
            }
        }
    }

    // Remove the marked keys.
    for (const std::string& key : keysToRemove) 
    {
        config.remove(key);
    }

    // Update the config with the current state of the bushes in the vector.
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
                config[bushKey]["harzard"] = bush.harzard;
            }
        } 
        else if (!config[bushKey]) 
        {
            // If the bush is not already in the YAML, add it.
            config[bushKey]["position"] = bushPosition;
            config[bushKey]["onFire"] = bush.onFire;
            config[bushKey]["fireIntensity"] = bush.fireIntensity;
            config[bushKey]["harzard"] = bush.harzard;
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

    // Write the updated config back to the file.
    std::ofstream fout(filename);
    fout << config;
    fout.close();
}

// Function to remap the order of the bushes based on their fire intensity
void Bushland::reMap() 
{
    ROS_INFO("Entering reMap function.");
    ROS_INFO("Number of bushes at start of reMap: %zu", bush.size());

    YAML::Node config;

    if (std::ifstream(filename)) 
    {
        config = YAML::LoadFile(filename);
    }

    // Extract and temporarily store all Bush and Reservoir data from YAML
    std::vector<YAML::Node> extractedBush;
    for (Bush& bush : bush) 
    {
        std::string bushKey = "bush" + std::to_string(bush.id);
        if (config[bushKey]) 
        {
            extractedBush.push_back(config[bushKey]);
        }
    }

    // Sort the extracted Bush data according to fireIntensity
    std::sort(extractedBush.begin(), extractedBush.end(), [](const YAML::Node& a, const YAML::Node& b) { 
        return a["fireIntensity"].as<int>() > b["fireIntensity"].as<int>(); 
    });

    // Clear the Bush and Reservoir entries from the YAML file
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

    // Write the sorted Bush data back to the YAML file
    for (YAML::Node& bushNode : extractedBush) 
    {
        std::string bushKey = "bush" + std::to_string(bushNode["id"].as<int>());
        config[bushKey] = bushNode;
    }

    // Write Reservoir data back to YAML file
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
