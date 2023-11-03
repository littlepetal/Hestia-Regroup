#ifndef RESERVOIR_H
#define RESERVOIR_H

//--Includes-----------------------------------------------------------
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "hestia/BushFire.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <cstdlib>
#include <visualization_msgs/Marker.h>
#include <string>
#include <map>
#include <visualization_msgs/Marker.h>

//--Reservoir Interface---------------------------------------------------
class Reservoir 
{
    public:

        // Apriltag ID of the reservoir
        int id;

        // Coordinate of the reservoir
        std::pair<float, float> position;

        // Constructs a reservoir
        Reservoir(int id, std::pair<float, float> pos) 
            : id(id), position(pos) 
        {}
};

#endif