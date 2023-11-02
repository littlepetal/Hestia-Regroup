//--Includes-----------------------------------------------------------
#include "hestia/user_interface.h"
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
        connect(setFireButton, &QPushButton::clicked, this, [=]() 
        {
            setBushfireLevel(i);
        });

        bushUIComponentsList.push_back({setFireButton, fireLevelLineEdit, statusLineEdit});
    }

    // Operation mode UI components
    QStringList modes = {"Start Mapping", "Start Control Burning", "Start Fire Eliminating"};
    for (const QString &mode : modes) 
    {
        QPushButton* operationButton = new QPushButton(mode, this);
        mainLayout->addWidget(operationButton);
        connect(operationButton, &QPushButton::clicked, this, [=]() 
        {
            setOperationMode(mode);
        });
    }

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
    // Destructor: Cleanup if necessary
}

void UserInterface::setBushfireLevel(int bushId) 
{
    ROS_INFO("setBushfireLevel called for bush_id: %d", bushId);

    int fireLevel = 1 + std::rand() % 3; // Random intensity between 1 and 10
    bushUIComponentsList[bushId].fireLevelLineEdit->setText(QString::number(fireLevel));

    hestia::BushFire msg;
    msg.bushId = bushId;
    msg.isOnFire = true;
    msg.fireIntensity = fireLevel;
    
    ROS_INFO("Publishing fire message for bush_id: %d with intensity: %d", bushId, fireLevel);
    firePub.publish(msg);
    ROS_INFO("Fire message published for bush_id: %d", bushId);
}

void UserInterface::updateBushStatus(const hestia::BushFire &msg, int id) 
{
    QString status = QString("Bush %1: Fire Level %2").arg(msg.bushId).arg(msg.fireIntensity);
    bushUIComponentsList[id].statusLineEdit->setText(status);
}

void UserInterface::setOperationMode(const QString &mode) 
{
    operationModeLineEdit->setText(mode);
    std_msgs::String msg;
    msg.data = mode.toStdString();
    operationModePub.publish(msg);
}

