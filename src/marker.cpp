#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <map>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_publisher");
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";  // Set frame
  marker.header.stamp = ros::Time::now();

  marker.ns = "basic_shapes";
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Load the YAML file
  YAML::Node config = YAML::LoadFile("src/hestia/data/map_data.yaml");

  int marker_id = 0;
  while (ros::ok()) {
    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
      std::string bush_name = it->first.as<std::string>();
      YAML::Node bush_data = it->second;

      bool isOnFire = false; // default value
      if(bush_data["onFire"]) {
        isOnFire = bush_data["onFire"].as<bool>();
      }
      ROS_INFO_STREAM("Bush Name: " << bush_name << ", On Fire: " << (isOnFire ? "Yes" : "No"));

      // Set marker color based on onFire value
      if (isOnFire) 
      {
        marker.color.r = 1.0f; // Red
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
      } 
      else if(!bush_data["onFire"])
      {
        marker.color.b = 1.0f; //blue
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
      }
      else 
      {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f; // Green
        marker.color.b = 0.0f;
      }

      
      marker.color.a = 1.0;  // Alpha (opacity)

      marker.id = marker_id++;

      marker.pose.position.x = bush_data["position"][0].as<double>();
      marker.pose.position.y = bush_data["position"][1].as<double>();
      marker.pose.position.z = 0;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker_pub.publish(marker);
    }
    ros::Duration(1.0).sleep();
  }
  return 0;
}