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
class UserInterface : public QWidget 
{
    // Create a QWidget object
    Q_OBJECT

    public:

        UserInterface(ros::NodeHandle& nh, QWidget* parent = nullptr);
        ~UserInterface();

    private slots:

        void setBushfireLevel(int bushId);
        void updateBushStatus(const hestia::BushFire& msg, int id);
        void setOperationMode(const QString& mode);

    private:

        struct BushUIComponents 
        {
            QPushButton* setFireButton;
            QLineEdit* fireLevelLineEdit;
            QLineEdit* statusLineEdit;
        };

        std::vector<BushUIComponents> bushUIComponentsList;
        QLineEdit* operationModeLineEdit;

        ros::Publisher firePub;
        ros::Publisher operationModePub;
        ros::Subscriber bushStatusSub[4];
};

#endif
