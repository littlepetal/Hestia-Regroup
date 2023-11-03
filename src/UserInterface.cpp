//--Includes-----------------------------------------------------------
#include "hestia/UserInterface.h"
#include <QApplication>

//--UserInterface Implementation------------------------------------------
// Constructor for the UserInterface class. It is initialised with 
// a reference to a ROS NodeHandle and a Qt QWidget parent
// It initialises two ROS publishers, fire_pub and operation_mode_pub, 
// for topics related to bush fires and operation modes
UserInterface::UserInterface(ros::NodeHandle &nh, QWidget* parent) 
    : QWidget(parent), 
      firePub(nh.advertise<hestia::BushFire>("bush_fire_topic", 10)),
      operationModePub(nh.advertise<std_msgs::String>("operation_mode_topic", 10)) 
{

    // A Qt vertical layout (mainLayout) is created
    QVBoxLayout* mainLayout = new QVBoxLayout;

    // A random seed is initialised based on the current time
    std::srand(std::time(nullptr));

    // Creates UI components for each of the four bush objects, 
    // including labels, buttons, and line edits
    for (int i = 0; i < 4; i++) 
    {
        QHBoxLayout* bushLayout = new QHBoxLayout;

        QLabel* bushLabel = new QLabel(QString("Bush %1").arg(i+1), this);
        QPushButton* setFireButton = new QPushButton("Set Bushfire Level", this);
        QLineEdit* fireLevelLineEdit = new QLineEdit(this);
        QLineEdit* statusLineEdit = new QLineEdit(this);

        // The UI components are added to the bushLayout layout
        bushLayout->addWidget(bushLabel);
        bushLayout->addWidget(setFireButton);
        bushLayout->addWidget(fireLevelLineEdit);
        bushLayout->addWidget(statusLineEdit);

        // The bushLayout layout is added to the mainLayout layout
        mainLayout->addLayout(bushLayout);

        // Connect the button to a lambda function that calls setBushfireLevel
        // When the button is clicked, it will trigger the function to set the bush fire level
        connect(setFireButton, &QPushButton::clicked, this, [=]() 
        {
            setBushfireLevel(i);
        });
        
        // Store the UI components in a list for later access
        bushUIComponentsList.push_back({setFireButton, fireLevelLineEdit, statusLineEdit});
    }

    // Operation mode UI components
    QStringList modes = {"Start Mapping", "Start Control Burning", "Start Fire Eliminating"};
    for (const QString &mode : modes) 
    {
        QPushButton* operationButton = new QPushButton(mode, this);
        mainLayout->addWidget(operationButton);

        // When the operation button is clicked, it will set the operation mode accordingly
        connect(operationButton, &QPushButton::clicked, this, [=]() 
        {
            setOperationMode(mode);
        });
    }

    // Line edit to display the current operation mode
    operationModeLineEdit = new QLineEdit(this);
    mainLayout->addWidget(operationModeLineEdit);

    // Subscribe to bush status for each bush
    for (int i = 0; i < 4; i++) 
    {
        bushStatusSub[i] = nh.subscribe<hestia::BushFire>(
            QString("bush%1_status_topic").arg(i+1).toStdString(), 10,
            [this, i](const hestia::BushFire::ConstPtr& msg) {
                updateBushStatus(*msg, i);
            }
        );
    }

    setLayout(mainLayout);
}

UserInterface::~UserInterface() 
{

}

// Function to randomly set the bush fire level
void UserInterface::setBushfireLevel(int bushId) 
{
    // Log the bush ID for which the fire level is being set
    ROS_INFO("setBushfireLevel called for bush_id: %d", bushId);

    // Generate a random intensity between 1 and 3 for the bush fire
    int fireLevel = 1 + std::rand() % 3;
    // Update the line edit for the fire level with the randomly generated intensity
    bushUIComponentsList[bushId].fireLevelLineEdit->setText(QString::number(fireLevel));

    // Create a message to publish the bush fire event
    hestia::BushFire msg;
    msg.bushId = bushId;
    msg.isOnFire = true;
    msg.fireIntensity = fireLevel;
    
    // Publish the bush fire event and log the action
    ROS_INFO("Publishing fire message for bush_id: %d with intensity: %d", bushId, fireLevel);
    firePub.publish(msg);
    ROS_INFO("Fire message published for bush_id: %d", bushId);
}

// Function to update the UI with the status of a bush
void UserInterface::updateBushStatus(const hestia::BushFire &msg, int id) 
{
    // Form a status string indicating the bush ID and its fire level
    QString status = QString("Bush %1: Fire Level %2").arg(msg.bushId).arg(msg.fireIntensity);
    // Set the status line edit text to show the bush status
    bushUIComponentsList[id].statusLineEdit->setText(status);
}

// Function to set the current operation mode and publish it
void UserInterface::setOperationMode(const QString &mode) 
{
    // Update the operation mode line edit with the selected mode
    operationModeLineEdit->setText(mode);
    // Create a message with the operation mode
    std_msgs::String msg;
    msg.data = mode.toStdString();
    // Publish the operation mode message
    operationModePub.publish(msg);
}

