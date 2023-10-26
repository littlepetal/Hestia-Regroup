#include "hestia/user_interface.h"
#include <QApplication>

UserInterface::UserInterface(ros::NodeHandle &nh, QWidget *parent) 
    : QWidget(parent), 
      fire_pub(nh.advertise<hestia::BushFire>("bush_fire_topic", 10)),
      operation_mode_pub(nh.advertise<std_msgs::String>("operation_mode_topic", 10)) {

    QVBoxLayout *mainLayout = new QVBoxLayout;
    std::srand(std::time(nullptr));

    // Bush UI components
    for (int i = 0; i < 4; i++) {
        QHBoxLayout *bushLayout = new QHBoxLayout;

        QLabel *bushLabel = new QLabel(QString("Bush %1").arg(i+1), this);
        QPushButton *setFireButton = new QPushButton("Set Bushfire Level", this);
        QLineEdit *fireLevelLineEdit = new QLineEdit(this);
        QLineEdit *statusLineEdit = new QLineEdit(this);

        bushLayout->addWidget(bushLabel);
        bushLayout->addWidget(setFireButton);
        bushLayout->addWidget(fireLevelLineEdit);
        bushLayout->addWidget(statusLineEdit);

        mainLayout->addLayout(bushLayout);

        // Connect the button to a lambda function that calls setBushfireLevel
        connect(setFireButton, &QPushButton::clicked, this, [=]() {
            setBushfireLevel(i);
        });

        bushUIComponentsList.push_back({setFireButton, fireLevelLineEdit, statusLineEdit});
    }

    // Operation mode UI components
    QStringList modes = {"Start Mapping", "Start Control Burning", "Start Fire Eliminating"};
    for (const QString &mode : modes) {
        QPushButton *operationButton = new QPushButton(mode, this);
        mainLayout->addWidget(operationButton);
        connect(operationButton, &QPushButton::clicked, this, [=]() {
            setOperationMode(mode);
        });
    }

    operationModeLineEdit = new QLineEdit(this);
    mainLayout->addWidget(operationModeLineEdit);

    // Subscribe to bush status for each bush
    for (int i = 0; i < 4; i++) {
        bush_status_sub[i] = nh.subscribe<hestia::BushFire>(
            QString("bush%1_status_topic").arg(i+1).toStdString(),
            10,
            [this, i](const hestia::BushFire::ConstPtr& msg) {
                updateBushStatus(*msg, i);
            }
        );
    }

    setLayout(mainLayout);
}

UserInterface::~UserInterface() {
    // Destructor: Cleanup if necessary
}

void UserInterface::setBushfireLevel(int bush_id) {
    ROS_INFO("setBushfireLevel called for bush_id: %d", bush_id);

    int fireLevel = 1 + std::rand() % 10; // Random intensity between 1 and 10
    bushUIComponentsList[bush_id].fireLevelLineEdit->setText(QString::number(fireLevel));

    hestia::BushFire msg;
    msg.bush_id = bush_id;
    msg.is_on_fire = true;
    msg.fire_intensity = fireLevel;
    
    ROS_INFO("Publishing fire message for bush_id: %d with intensity: %d", bush_id, fireLevel);
    fire_pub.publish(msg);
    ROS_INFO("Fire message published for bush_id: %d", bush_id);
}

void UserInterface::updateBushStatus(const hestia::BushFire &msg, int id) {
    QString status = QString("Bush %1: Fire Level %2").arg(msg.bush_id).arg(msg.fire_intensity);
    bushUIComponentsList[id].statusLineEdit->setText(status);
}

void UserInterface::setOperationMode(const QString &mode) {
    operationModeLineEdit->setText(mode);
    std_msgs::String msg;
    msg.data = mode.toStdString();
    operation_mode_pub.publish(msg);
}

