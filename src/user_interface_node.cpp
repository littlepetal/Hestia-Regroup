//--Includes-----------------------------------------------------------
#include "hestia/UserInterface.h"
#include <QApplication>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ui_node");

    QApplication app(argc, argv);
    
    ros::NodeHandle nh;

    UserInterface ui(nh);
    ui.show();

    // Use ROS spin in a separate thread to keep both ROS and Qt running
    ros::AsyncSpinner spinner(1);
    spinner.start();

    return app.exec();
}
