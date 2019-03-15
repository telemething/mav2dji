/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

#include <vehicle_interface.hpp>

// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3Stamped.h>

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/CameraAction.h>
#include <dji_sdk/Gimbal.h>

namespace mav2dji
{

class vehicle_interface_djiros : public vehicle_interface
{
 public:

   explicit vehicle_interface_djiros();
   ~vehicle_interface_djiros();

   int init();
   vehicle_interface_ret activate();

 private:

   int DjiActivationSleepMs = 1000;
   int DjiActivationTimeoutMs = 10000;

   ros::NodeHandle      rosNodeHandle;
   ros::ServiceClient   drone_activation_service;

   vehicle_interface_ret connectToRos();

};

} /* namespace mav2dji*/