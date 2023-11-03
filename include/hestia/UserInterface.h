#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

//--Includes-----------------------------------------------------------
#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <hestia/BushFire.h>

//--UserInterfce Interface---------------------------------------------------
// UserInterface inherits from QWidget, which is the base class for all UI objects in Qt
class UserInterface : public QWidget 
{
    // Create a QWidget object， This macro enables the class to use Qt's signal and slot mechanism
    Q_OBJECT

    public:
        // Constructor for the UserInterface class. It takes a
        // reference to a ROS NodeHandle and a Qt QWidget parent as arguments.
        // This allows the UserInterface to interact with ROS and Qt's parent-child system.
        UserInterface(ros::NodeHandle& nh, QWidget* parent = nullptr);
        ~UserInterface(); // Destructor for the UserInterface class

    private slots:
        // These are Qt slots which can be connected to Qt signals.
        // When the associated signal is emitted, the slot is executed.
        
        void setBushfireLevel(int bushId); // Slot to set the fire level for a specified bush
        void updateBushStatus(const hestia::BushFire& msg, int id); // Slot to update the UI with the bush's status
        void setOperationMode(const QString& mode); // Slot to set the current operation mode

    private:
        // This structure holds pointers to the UI components for each bush.
        struct BushUIComponents 
        {
            QPushButton* setFireButton; // Button to set the fire level
            QLineEdit* fireLevelLineEdit; // Line edit to display and set the fire level
            QLineEdit* statusLineEdit; // Line edit to display the status of the bush
        };

        std::vector<BushUIComponents> bushUIComponentsList; // Vector holding the UI components for all bushes
        QLineEdit* operationModeLineEdit; // Line edit to display the current operation mode

        // ROS publishers and subscribers
        ros::Publisher firePub; // ROS publisher for sending bush fire information
        ros::Publisher operationModePub; // ROS publisher for sending operation mode commands
        ros::Subscriber bushStatusSub[4]; // Array of ROS subscribers for receiving bush status updates
};

#endif
