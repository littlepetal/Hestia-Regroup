//--Includes-----------------------------------------------------------
#include "hestia/Hestia.h"

//--Hestia Implementation------------------------------------------
// Constructs Hestia
Hestia::Hestia()
{
    // Initialise the amount of resources required
    requiredResource = 0;

    // Initialise the detected ID
    detectedId = -1;

    // Initialise the operation mode
    mode = -1;

    // Initialise the hydro blaster and flame thrower
    hydroBlaster = new HydroBlaster();
    flameThrower = new FlameThrower();

    // Initialise a client???
    moveBaseClient = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);
    ROS_INFO("Waiting for the move_base action server to come up");
    moveBaseClient->waitForServer();
    ROS_INFO("Finished");

    // Initialise water tank subscriber
    waterTankSub = nh.subscribe("/reservoir_water_status", 100, &Hestia::WaterTankCallback, this);

    // Initialise gas tank subscriber
    gasTankSub = nh.subscribe("/reservoir_gas_status", 100, &Hestia::GasTankCallback, this);

    // Initialise operation mode subscriber
    modeSub = nh.subscribe("/operation_mode_topic", 100, &Hestia::ModeCallback, this);

    // Initialise april tag detection subscriber
    tagSub = nh.subscribe("/tag_detection", 100, &Hestia::TagCallback, this);
    cmdPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Decide where Hestia needs to go next
    // prioritySub = nh.subscribe("priority_list", 1, &Hestia::priorityCallback, this);

    odometrySub = nh.subscribe("odom", 100, &Hestia::odomMsgCallback, this);

    detectedGoalPub = nh.advertise<std_msgs::Int32>("/detected_goal_id", 10);
}

// Destructs Hestia. Deallocates memory for the Navstack move base client
Hestia::~Hestia()
{
    delete moveBaseClient;
}

// Callback function to receive messages from reservoir water topic
void Hestia::odomMsgCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    currentOdometry = std::make_pair(msg->pose.pose.position.x, msg->pose.pose.position.y);
    // // Compute yaw (heading) from quaternion.
    // double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    // double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  
    // tb3_pose_ = atan2(siny, cosy);
}


// Callback function to receive messages from reservoir water topic
void Hestia::WaterTankCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("I heard: [%d]", msg->data);

    // Load hestia with water
    if (msg->data > requiredResource)
    {
        hydroBlaster->Load(requiredResource);
    }
}

// Callback function to receive messages from reservoir gas topic
void Hestia::GasTankCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("I heard: [%d]", msg->data);

    // Load hestia with gas
    if (msg->data > requiredResource)
    {
        flameThrower->Load(requiredResource);
    }
}

// Callback function to receive the mode of operation from operation mode topic
void Hestia::ModeCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());

    // Set the operation mode of hestia depenidng on the signal
    if (strcmp(msg->data.c_str(), "Start Control Burning") == 0)
    {
        // Set hestia to controlled burning mode
        mode = 0;
    }
    else if (strcmp(msg->data.c_str(), "Start Fire Eliminating") == 0)
    {
        // Set hestia to fire elimination mode
        mode = 1;
    }
    ROS_INFO("mode: %d",mode);
}

// Callback function to receive the ID of the currently detected april tag
void Hestia::TagCallback(const std_msgs::Int32::ConstPtr& msg)
{
    detectedId = msg->data;

    if (detectedId == 0) 
    {
        std::thread(&Hestia::processDetectedID,this).detach();  // Start a new thread to process the detectedId
    }
}

void Hestia::requiredResourcesCallback(const std_msgs::Int32::ConstPtr& msg)
{

}

// Navigates Hestia and performs controlled burning or fire elimination 
// depending on the operation mode
void Hestia::processDetectedID() 
{
    // Keep track of thread
    int localId;
    {
        // Apply thread lock
        std::lock_guard<std::mutex> lock(IdMutex);
        localId = detectedId;
        detectedId = -1;  
    }

    // Process the locaId
    bool isProcessingGoals = false;

    // Wait till goal poses have been processed
    if (localId == 0 && !isProcessingGoals) 
    {
        // Get the coordinates of the reservoirs and bushes of interest
        std::vector<std::pair<float, float>> fireBushes = getPositions("src/hestia/data/map_data.yaml");

        // Initialise goal pose
        std::vector<geometry_msgs::Pose> goalPose;

        // Calculate the goal poses from the coordinates
        for (const auto& bush : fireBushes) 
        {
            geometry_msgs::Pose goal;
            goal.position.x = bush.first;
            goal.position.y = bush.second;

            // Assuming orientation is kept constant
            goal.orientation.w = 1.0;  

            // Add pose to vector of goal poses
            goalPose.push_back(goal);
        }

        // Iterate through the goalPose vector and set each as a goal to move towards
        for (const auto& pose : goalPose) 
        {
            moveToGoal(pose);

            // Rotate the robot in small increments and check for detectedId
            // Divides the 360 degrees into 10-degree increments
            int rotationIncrements = 36;  
            for (int i = 0; i < rotationIncrements; i++) 
            {
                if (detectedId != -1) 
                {
                    break;  // If a tag has been detected, break out of the loop
                }

                turn(10);  // Rotate by 10 degrees

                // Wait for half a second to give time for the callback to update detectedId
                ros::Duration(0.1).sleep();  
            }

            // Notification for Hestia turning
            ROS_INFO("turned loop");

            // If a tag is detected, stop the robot
            if (detectedId != -1) 
            {
                turn(90);   // Rotate by 90 degrees

                // Notification for detect and turn
                ROS_INFO("New tag detected and Turned 90");

                // Create goal message using the detected Apriltag ID
                std_msgs::Int32 goalMsg;
                goalMsg.data = detectedId;

                // Publish the goal message
                detectedGoalPub.publish(goalMsg);
                ros::Duration(5).sleep();
            }
        }

        // Reset the goal poses processing flag at the end
        isProcessingGoals = false;  
    }
}

// Helper function for processDetectedID. Drives Hestia to the given goal pose 
void Hestia::moveToGoal(const geometry_msgs::Pose& goalPose) 
{
    // Declare move message type
    move_base_msgs::MoveBaseGoal goal;

    // Make the goal message
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = goalPose;

    // Notification for sending the goal coordinates
    ROS_INFO("Sending goal to move_base: x = %f, y = %f", goalPose.position.x, goalPose.position.y);

    // Send the goal message to the client
    moveBaseClient->sendGoal(goal);
    moveBaseClient->waitForResult();

    // Determine if the goal is reached by Hestia
    if (moveBaseClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        // Notification for succeeding to reach the goal pose
        ROS_INFO("Goal reached!");
    }
    else 
    {
        // Notification for failing to reach the goal pose
        ROS_INFO("Failed to reach goal");
    }
}

// Turns the Hestia bot by the given degrees
void Hestia::turn(float degrees) 
{
    // Declare turn message type
    geometry_msgs::Twist twist;

    // Set a fixed rotation speed
    twist.angular.z = (degrees > 0) ? 1.0 : -1.0;  

    // Turning speed
    float duration = std::abs(degrees) / 162.72;

    // Set the publishing rate to 10Hz
    ros::Rate rate(10);  
    float elapsedTime = 0;
    while (elapsedTime < duration) 
    {
        // Publish the turning message
        cmdPub.publish(twist);

        // Notification for turning
        ROS_INFO("turning");

        rate.sleep();

        // Increment elapsed time by 1/10Hz
        elapsedTime += 0.1;  
    }

    // Stop the robot
    twist.angular.z = 0;

    // Publish the turning message
    cmdPub.publish(twist);
}

// Receives the coordinates of all the reservoirs and bushes from a YAML file
// Saves all the coordinates of the reservoirs
// Saves all the coordinates of either the hazards or fires
// Returns a vector of the coordinates of interest
std::vector<std::pair<float, float>> Hestia::getPositions(const std::string& filename) 
{
    // Initialise the YAML file
    YAML::Node config = YAML::LoadFile(filename);
    
    // Declare the object coordinates vector
    std::vector<std::pair<float, float>> positions;

    // Loop through all bushes and reservoirs presented in the YAML
    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) 
    {
        // Initialise the key for string processing
        std::string key = it->first.as<std::string>();

        // Declare a pair of coordinates
        std::pair<float, float> position;

        // Fire elimination mode
        if (mode == 1)
        {
            // Add all the reservoirs and all the bushes that are on fire to the positions vector
            if ((key.find("bush") != std::string::npos && it->second["onFire"].as<bool>()) 
                    || key.find("reservoir") != std::string::npos) 
            {
                // Gather object coordinates
                position.first = it->second["position"][0].as<float>();
                position.second = it->second["position"][1].as<float>();

                // Add to positions vector
                positions.push_back(position);
            }
        }
        // Controlled burning mode
        else if (mode == 0)
        {
            // Add all the reservoirs and all the bushes that are hazardous to the positions vector
            if ((key.find("bush") != std::string::npos && it->second["harzard"].as<bool>()) 
                    || key.find("reservoir") != std::string::npos) 
            {
                // Gather object coordinates
                position.first = it->second["position"][0].as<float>();
                position.second = it->second["position"][1].as<float>();

                // Add to positions vector
                positions.push_back(position);
            }
        }
    }

    // Return the vector of object coordinates
    return positions;
}
