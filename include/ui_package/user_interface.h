#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ui_package/BushFire.h>

class UserInterface : public QWidget {
    Q_OBJECT
public:
    UserInterface(ros::NodeHandle &nh, QWidget *parent = nullptr);
    ~UserInterface();

private slots:
    void setBushfireLevel(int bush_id);
    void updateBushStatus(const ui_package::BushFire &msg, int id);
    void setOperationMode(const QString &mode);

private:
    struct BushUIComponents {
        QPushButton *setFireButton;
        QLineEdit *fireLevelLineEdit;
        QLineEdit *statusLineEdit;
    };

    std::vector<BushUIComponents> bushUIComponentsList;
    QLineEdit *operationModeLineEdit;

    ros::Publisher fire_pub;
    ros::Publisher operation_mode_pub;
    ros::Subscriber bush_status_sub[4];
};

#endif // USER_INTERFACE_H
