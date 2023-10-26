#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <nav_msgs/Odometry.h>
#include <map>

class TagRecorder {
private:
    ros::NodeHandle nh_;
    ros::Subscriber tag_sub_;
    ros::Subscriber odom_sub_;
    std::map<int, nav_msgs::Odometry> recorded_tags_;

public:
    TagRecorder() : nh_("~") {
        tag_sub_ = nh_.subscribe("/tag_detections", 1, &TagRecorder::tagCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 1, &TagRecorder::odomCallback, this);
    }

    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
        for (const auto& detection : msg->detections) {
            if (recorded_tags_.find(detection.id) == recorded_tags_.end()) {
                // New tag detected
                nav_msgs::Odometry latest_odom;
                nh_.getParam("/latest_odom", latest_odom);  // Assuming you store latest odom as a param
                recorded_tags_[detection.id] = latest_odom;
            }
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        nh_.setParam("/latest_odom", *msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tag_recorder");
    TagRecorder recorder;
    ros::spin();
    return 0;
}
