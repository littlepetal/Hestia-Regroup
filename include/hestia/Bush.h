#ifndef BUSH_H
#define BUSH_H

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

//--Bush Interface---------------------------------------------------
class Bush 
{
    public:

        // Apriltag ID of the bush
        int id;

        // Coordinate of the bush
        std::pair<float, float> position;

        // Flag for whether the bush is on fire
        bool onFire;

        // The intensity of the fire at the bush
        int fireIntensity;

        // Flag for whether the bush is a fire hazard
        bool harzard;

        // Constructs a bush
        Bush(int id, std::pair<float, float> pos, bool fire, int intensity, bool harzard) 
            : id(id), position(pos), onFire(fire), fireIntensity(intensity), harzard(harzard) 
        {}

        // Returns the Apriltag ID of the bush
        int getTagID() const { return id; }

        // Places the bush at the given coordinate
        void setPosition(const std::pair<float, float>& pos) { position = pos; }

        // Sets the bush on fire at the given intensity
        void setFireStatus(bool fire, int intensity) 
        {
            onFire = fire;
            fireIntensity = fireIntensity + intensity;
        }
};

#endif