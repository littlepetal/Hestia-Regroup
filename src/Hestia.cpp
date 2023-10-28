// This is the Hestia node responsible for receiving information from 
// the Bushland node for the operation of Device

//--Includes-----------------------------------------------------------
#include "hestia/Hestia.h"

// #include <format>

//--Hestia Implementation------------------------------------------
// Constructs Hestia
Hestia::Hestia(): original_pose_received_(false) 
{
    
    // Initialise the amount of resources required
    requiredResource = 0;

    // Initialise the last detected ID
    currentId = -1;

    // Initialise the operation mode
    mode = UNDEFINED;

    // Initialise the hydro blaster and flame thrower
    hydroBlaster = new HydroBlaster();
    flameThrower = new FlameThrower();

    // Initialise a client???
    move_base_client_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);

    // Initialise water tank subscriber
    waterTankSub = nh.subscribe("/reservoir_water_status", 100, &Hestia::WaterTankCallback, this);

    // Initialise gas tank subscriber
    gasTankSub = nh.subscribe("/reservoir_gas_status", 100, &Hestia::GasTankCallback, this);

    // Initialise operation mode subscriber
    modeSub = nh.subscribe("/operation_mode_topic", 100, &Hestia::ModeCallback, this);

    // Initialise april tag detection subscriber
    tagSub = nh.subscribe("/tag_detection", 100, &Hestia::TagCallback, this);

    // Decide where Hestia needs to go next
    priority_sub_ = nh_.subscribe("priority_list", 1, &TurtleBot3DriveNode::priorityCallback, this);
}

// Destructs Hestia
Hestia::~Hestia()
{
    delete move_base_client_;
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
void Hesta::ModeCallBack(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());

    // Set the operation mode of hestia depenidng on the signal
    if (msg->data.c_str() == "Start Control Burning")
    {
        // Set hestia to controlled burning mode
        mode = CONTROLLED_BURNING;
    }
    else if (msg->data.c_str() == "Start Fire Eliminating")
    {
        // Set hestia to fire elimination mode
        mode = FIRE_ELIMINATION;
    }
}

// Callback function to receive the ID of the currently detected april tag
void Hestia::TagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    // ROS_INFO("I heard: [%d]", msg->data);

    // // Set the current ID
    // currentId = msg->data;

    for (const auto& detection : msg->detections) 
    {
        int id = detection.id[0];
        const geometry_msgs::PoseStamped& tag_pose = detection.pose;
        tag_poses_[id] = tag_pose.pose;
    }

    // If the original pose yet, save the current pose as the original pose.
    if (!original_pose_received_) 
    {
        original_pose_ = msg->detections[0].pose.pose;
        original_pose_received_ = true;
    }

    // If detected all 5 AprilTags, start the sequence.
    if (tag_poses_.size() == 5) 
    {
        // executeSequence();
    }
}

// Decide where Hestia needs to go next
void priorityCallback(const std_msgs::String::ConstPtr& msg) 
{
    // Parse the priority list with number id
    std::vector<std::string> priority_list;
    boost::split(priority_list, msg->data, boost::is_any_of(","));

    // Go to water base
    int water_base_id = 0;  // assuming water base has ID 0
    moveToGoal(tag_poses_[water_base_id]);

    // Go to fire locations based on priority list
    for (const auto& fire : priority_list) 
    {
        int fire_id = std::stoi(fire);
        moveToGoal(tag_poses_[fire_id]);

        // Go back to water base to refill
        moveToGoal(tag_poses_[water_base_id]);
    }
}

// Drive Hestia to destination
void moveToGoal(const geometry_msgs::Pose& goal_pose) 
{
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = goal_pose;

    move_base_client_->sendGoal(goal);
    move_base_client_->waitForResult();

    if (move_base_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("Goal reached!");
    } 
    else 
    {
        ROS_INFO("Failed to reach goal");
    }
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