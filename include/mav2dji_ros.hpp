/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <mavvehiclelib.hpp>

namespace mav2dji
{

//class mav2dji_ros : mavvehiclelib::mavvehicleclient
class mav2dji_ros 
{
 public:

  // ROS node handle.
  ros::NodeHandle nodeHandle_;

  explicit mav2dji_ros(ros::NodeHandle nh);
  ~mav2dji_ros();

  void startVehicle();
  void stopVehicle();

  int vehicleMavMessageCallback(mavlink_message_t mavMsg);

 private:

  bool verbose = false;
  std::unique_ptr<mavvehiclelib::mavvehicle> mavvehicle_;
};

} /* namespace mav2dji*/