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
//#include <mav_udp.hpp>
#include <mav_message_base.hpp>
#include <mav_message_mission.hpp>

namespace mav2dji
{

//class mav2dji_ros : mavvehiclelib::mavvehicleclient
class mav_message : public mav2dji_message_base
{
 public:

  explicit mav_message();
  ~mav_message();

  //void startVehicle();
  //void stopVehicle();

  void ProcessMavMessage(const mavlink_message_t* msg);
  void printMavMessageInfo(const mavlink_message_t* msg, std::string prefix, bool always); 
  //int vehicleMavMessageCallback(const mavlink_message_t* mavMsg);

 private:

  bool verbose = false;
  ros::NodeHandle rosNodeHandle;
  //std::shared_ptr<mavvehiclelib::mav_udp> mav_udp_;

  uint8_t *px4_git_version_binary;

  std::unique_ptr<mav2dji_mission>	mission_manager;
	//MavlinkParametersManager	_parameters_manager;
	//MavlinkFTP			_mavlink_ftp;
	//MavlinkLogHandler		_mavlink_log_handler;
	//MavlinkTimesync		_mavlink_timesync;

  void init();
    
  void processMAVLINK_MSG_ID_HEARTBEAT(const mavlink_message_t* msg);
  void processMAVLINK_MSG_ID_PARAM_REQUEST_LIST(const mavlink_message_t* msg);
  void processMAVLINK_MSG_ID_COMMAND_LONG(const mavlink_message_t* msg);

  void processMAV_CMD_REQUEST_PROTOCOL_VERSION(const mavlink_message_t* msg);
  void processMAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES(const mavlink_message_t* msgIn);

  void mcu_unique_id(uint32_t *uid_96_bit);

  void handle_message_command_int(const mavlink_message_t* msg);
  void handle_message_command_ack(const mavlink_message_t* msg);
  void handle_message_optical_flow_rad(const mavlink_message_t* msg);
  void handle_message_ping(const mavlink_message_t* msg);
  void handle_message_set_mode(const mavlink_message_t* msg);
  void handle_message_att_pos_mocap(const mavlink_message_t* msg);
  void handle_message_set_position_target_local_ned(const mavlink_message_t* msg);
  void handle_message_set_attitude_target(const mavlink_message_t* msg);
  void handle_message_set_actuator_control_target(const mavlink_message_t* msg);
  void handle_message_vision_position_estimate(const mavlink_message_t* msg);
  void handle_message_odometry(const mavlink_message_t* msg);
  void handle_message_gps_global_origin(const mavlink_message_t* msg);
  void handle_message_radio_status(const mavlink_message_t* msg);
  void handle_message_manual_control(const mavlink_message_t* msg);
  void handle_message_rc_channels_override(const mavlink_message_t* msg);
  void handle_message_distance_sensor(const mavlink_message_t* msg);
  void handle_message_follow_target(const mavlink_message_t* msg);
  void handle_message_landing_target(const mavlink_message_t* msg);
  void handle_message_adsb_vehicle(const mavlink_message_t* msg);
  void handle_message_collision(const mavlink_message_t* msg);
  void handle_message_gps_rtcm_data(const mavlink_message_t* msg);
  void handle_message_battery_status(const mavlink_message_t* msg);
  void handle_message_serial_control(const mavlink_message_t* msg);
  void handle_message_logging_ack(const mavlink_message_t* msg);
  void handle_message_play_tune(const mavlink_message_t* msg);
  void handle_message_obstacle_distance(const mavlink_message_t* msg);
  void handle_message_trajectory_representation_waypoints(const mavlink_message_t* msg);
  void handle_message_named_value_float(const mavlink_message_t* msg);
  void handle_message_debug(const mavlink_message_t* msg);
  void handle_message_debug_vect(const mavlink_message_t* msg);
  void handle_message_debug_float_array(const mavlink_message_t* msg);

};

} /* namespace mav2dji*/