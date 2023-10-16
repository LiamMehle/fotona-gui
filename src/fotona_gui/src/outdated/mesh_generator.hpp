#pragma once
#include "ros/ros.h"
// assumes ROS was initialized
void setup_mesh_generator(ros::NodeHandle& n, char const* const destination_topic, char const* const source_topic);
