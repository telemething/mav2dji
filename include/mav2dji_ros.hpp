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
  void ProcessMavMessage(mavlink_message_t msg);

  // found this but I dont think it's current https://subak.io/code/px4/Firmware/build_px4fmu-v2_default/build_git_version.h.html
  //uint8_t px4_git_version_binary = 0x9c2dd48814a6ade1;
  uint8_t px4_git_version_binary = 0x9c;

  void printMavMessageInfo(const mavlink_message_t* msg, 
    std::string prefix, bool always);
  void processMAVLINK_MSG_ID_HEARTBEAT(mavlink_message_t msg);
  void processMAVLINK_MSG_ID_PARAM_REQUEST_LIST(mavlink_message_t msg);
  void processMAVLINK_MSG_ID_COMMAND_LONG(const mavlink_message_t* msg);
  void processMAVLINK_MSG_ID_MISSION_REQUEST_LIST(mavlink_message_t msg);
  void processMAVLINK_MSG_ID_MISSION_REQUEST(mavlink_message_t msg);
  void processMAVLINK_MSG_ID_MISSION_COUNT(mavlink_message_t msg);
  void processMAVLINK_MSG_ID_MISSION_SET_CURRENT(mavlink_message_t msg);
  void processMAVLINK_MSG_ID_MISSION_ITEM(mavlink_message_t msg);

  void processMAV_CMD_REQUEST_PROTOCOL_VERSION(const mavlink_message_t* msg);
  void processMAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES(const mavlink_message_t* msgIn);

  void mcu_unique_id(uint32_t *uid_96_bit);

};

} /* namespace mav2dji*/