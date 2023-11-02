// This is the Hestia node responsible for receiving information from 
// the Bushland node for the operation of Device

//--Includes-----------------------------------------------------------
#include "hestia/Hestia.h"
#include <thread>
#include <mutex>

int detectedId = -1;
std::mutex id_mutex;
// #include <format>

//--Hestia Implementation------------------------------------------
// Constructs Hestia
Hestia::Hestia(): originalPoseReceived(false) 
{
    // Initialise the amount of resources required
    requiredResource = 0;

    // Initialise the last detected ID
    currentId = -1;

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
    commandPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Decide where Hestia needs to go next
    // prioritySub = nh.subscribe("priority_list", 1, &Hestia::priorityCallback, this);

    odometrySub = nh.subscribe("odom", 100, &Hestia::odomMsgCallback, this);

    detectedGoalPub = nh.advertise<std_msgs::Int32>("/detected_goal_id", 10);
}

// Destructs Hestia
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
    bool isProcessingGoals = false;

    if (detectedId == 0 && !isProcessingGoals) 
    {
        std::vector<std::pair<float, float>> fireBushes = getPositions("src/hestia/data/map_data.yaml");

        std::vector<geometry_msgs::Pose> goalPose;
        for (const auto& bush : fireBushes) 
        {
            geometry_msgs::Pose goal;
            goal.position.x = bush.first;
            goal.position.y = bush.second;
            goal.orientation.w = 1.0;  // Assuming you want to keep the orientation constant
            goalPose.push_back(goal);
        }

        // Now, you can iterate through the goalPose vector and send each as a goal
        for (const auto& pose : goalPose) 
        {
            moveToGoal(pose);
            detectedId = -1;
            
            // Rotate the robot in small increments and check for detectedId
            int rotationIncrements = 108;  // This will divide the 360 degrees into 10-degree increments
            for (int i = 0; i < rotationIncrements; i++) 
            {
                if (detectedId != -1) 
                {
                    break;  // If a tag has been detected, break out of the loop
                }

                turn(10);  // Rotate by 10 degrees
                ros::Duration(0.1).sleep();  // Wait for half a second to give time for the callback to update detectedId
            }

            ROS_INFO("turned loop");

            // If a tag is detected, stop the robot
            if (detectedId != -1) 
            {
                turn(90);
                ROS_INFO("New tag detected and Turned 90");
                std_msgs::Int32 goalMsg;
                goalMsg.data = detectedId;
                detectedGoalPub.publish(goalMsg);
                ros::Duration(5).sleep();
            }
        }
        isProcessingGoals = false;  // Reset the flag at the end
    }
}

//     if (detectedId == 0) {
//         std::thread(&Hestia::processDetectedID,this).detach();  // Start a new thread to process the detectedId
//     }
// }
// void Hestia::processDetectedID() {
//     int local_id;
//     {
//         std::lock_guard<std::mutex> lock(id_mutex);
//         local_id = detectedId;
//         detectedId = -1;  // Reset or whatever logic you need
//     }
//     // Now, process the local_id as you need
//     bool isProcessingGoals = false;

//     if (local_id == 0 && !isProcessingGoals) {
//         std::vector<std::pair<float, float>> fireBushes = getPositions("src/hestia/data/map_data.yaml");

//         std::vector<geometry_msgs::Pose> goalPose;
//         for (const auto& bush : fireBushes) {
//             geometry_msgs::Pose goal;
//             goal.position.x = bush.first;
//             goal.position.y = bush.second;
//             goal.orientation.w = 1.0;  // Assuming you want to keep the orientation constant
//             goalPose.push_back(goal);
//         }

//         // Now, you can iterate through the goalPose vector and send each as a goal
//         for (const auto& pose : goalPose) {
//             moveToGoal(pose);

//             // Rotate the robot in small increments and check for detectedId
//             int rotationIncrements = 108;  // This will divide the 360 degrees into 10-degree increments
//             for (int i = 0; i < rotationIncrements; i++) {
//                 if (detectedId != -1) {
//                     break;  // If a tag has been detected, break out of the loop
//                 }
//                 turn(10);  // Rotate by 10 degrees
//                 ros::Duration(0.1).sleep();  // Wait for half a second to give time for the callback to update detectedId
//             }
//             ROS_INFO("turned loop");

//             // If a tag is detected, stop the robot
//             if (detectedId != -1) {
//                 turn(90);
//                 ROS_INFO("New tag detected and Turned 90");
//                 std_msgs::Int32 goalMsg;
//                 goalMsg.data = detectedId;
//                 detectedGoalPub.publish(goalMsg);
//                 ros::Duration(5).sleep();
//             }
//         }
//         isProcessingGoals = false;  // Reset the flag at the end
//     }
// }

// Drive Hestia to destination
void Hestia::moveToGoal(const geometry_msgs::Pose& goalPose) 
{
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = goalPose;

    ROS_INFO("Sending goal to move_base: x = %f, y = %f", goalPose.position.x, goalPose.position.y);
    moveBaseClient->sendGoal(goal);
    moveBaseClient->waitForResult();

    if (moveBaseClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("Goal reached!");
    }
    else 
    {
        ROS_INFO("Failed to reach goal");
    }
}

void Hestia::turn(float degrees) 
{
    
    // Assuming you have a publisher to cmd_vel to control the robot

    geometry_msgs::Twist twist;
    twist.angular.z = (degrees > 0) ? 1.0 : -1.0;  // Set a fixed rotation speed, adjust as needed

    float duration = std::abs(degrees) / 162.72;  // Your turning speed

    ros::Rate rate(10);  // 10Hz
    float elapsedTime = 0;
    while (elapsedTime < duration) 
    {
        commandPub.publish(twist);
        ROS_INFO("turning");
        rate.sleep();
        elapsedTime += 0.1;  // Since we're sleeping for 0.1 seconds
    }

    // Stop the robot
    twist.angular.z = 0;
    commandPub.publish(twist);
}


//--Device Implementation------------------------------------------
// Constructs an empty device
Device::Device()
{
    // std::cout << "Device[CTor]: Device signing on." << std::endl;

    // Initialise number of available resource
    availableResource = 0;
}

// Virtual deconstructor
Device::~Device()
{
    // std::cout << "Device[DTor]: Device signing off." << std::endl;
}

// Loads the device with resource
void Device::Load(int level)
{

}

// Deploys the device using the available resources
void Device::Deploy(int bushID, int level)
{
   
}

//--HydroBlaster Implementation------------------------------------------
// Constructs a hydro blaster
HydroBlaster::HydroBlaster()
{
    // std::cout << "HydroBlaster[CTor]: HydroBlster signing on." << std::endl;
    // Initialise water blaster publisher
    waterBlasterPub = nh.advertise<std_msgs::Int32>("/water_blasted_to_id", 100);
}

// Deconstructs a hydro blaster 
HydroBlaster::~HydroBlaster()
{
    // std::cout << "HydroBlaster[DTor]: HydroBlster signing off." << std::endl;
}

// Loads the device with water
void HydroBlaster::Load(int level)
{
    // Increases the number of available water by level amount
    availableResource += level;  

    // Output result
    std::cout << availableResource << " water is now available for the hydro blaster" << std::endl;  
}

// Blasts level amount of water at the fire indicated by the bushID
void HydroBlaster::Deploy(int bushID, int level)
{
    // Check that there is enough water in the bushfire manager bot
    if (availableResource > level)
    {
        // Format string
        // std::String message = std::format("Hestia deployed %d water at the fire with ID %d", level, bushID);

        // Put out the fire
        // bushland->LocateFire(bushID)->EliminateFire();  

        // Format the message
        std_msgs::Int32 message;
        message.data = level;

        // Publish the message
        waterBlasterPub.publish(message);

        // Decrement number of available water
        availableResource -= level;
    }
    else
    {
        std::cout << "Not enough water... Fire elimination failed." << std::endl;
    }
}

//--FlameThrower Implementation------------------------------------------
// Constructs a flame thrower
FlameThrower::FlameThrower()
{
    // std::cout << "FlameThrower[CTor]: FlameThrower signing on." << std::endl;

    // Initialise flame thrower publisher
    flameThrowerPub = nh.advertise<std_msgs::Int32>("/flame_thrown_to_id", 100);
}

// Deconstructs the flame thrower
FlameThrower::~FlameThrower()
{
    // std::cout << "FlameThrower[DTor]: FlameThrower signing off." << std::endl;
}

// Loads the device with gas
void FlameThrower::Load(int level)
{
    // Increases the number of available water by level amount
    availableResource += level;  

    // Output result
    std::cout << availableResource << " gas is now available for the flame thrower" << std::endl;  
}

// Throws level amount of flames at the hazard indicated by the bushID
void FlameThrower::Deploy(int bushID, int level)
{
    // Check that there is enough gas in the bushfire manager bot
    if (availableResource > level)
    {
        // Format string
        // std::String message = std::format("Hestia deployed %d flames at the hazard with ID %d", level, bushID);

        // Burn the hazard
        // bushland->LocateFire(bushID)->EliminateFire();  

        // Format the message
        std_msgs::Int32 message;
        message.data = level;

        // Publish the message
        flameThrowerPub.publish(message);

        // Decrement number of available water
        availableResource -= level;
    }
    else
    {
        std::cout << "Not enough gas... Hazard elimination failed." << std::endl;
    }
}