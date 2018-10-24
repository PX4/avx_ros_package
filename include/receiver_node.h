#pragma once

#include <iostream>
#include <fstream>
#include <math.h> 
#include <string>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "mavros_msgs/State.h"

class ReceiverNode
{
public:
  ReceiverNode();
  ~ReceiverNode();

private:

  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber pose_sub_;
  ros::Subscriber velocity_sub_;
  ros::Subscriber state_sub_;

  // Private variables
  bool currently_armed_ = false;

  // void readParams();

  void positionCallback(const geometry_msgs::PoseStamped msg);                  // position and orientation from PX4
  void velocityCallback(const geometry_msgs::TwistStamped msg);                 // velocity from PX4
  void stateCallback(const mavros_msgs::State msg);                             // state from PX4

  void printPose(geometry_msgs::PoseStamped pos);
  void printVelocity(geometry_msgs::TwistStamped vel); 

};
