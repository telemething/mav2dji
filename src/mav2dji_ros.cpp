/*
 * mav2dji_ros.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <mav2dji_ros.hpp>

//*****************************************************************************
//*
//*
//*
//******************************************************************************

namespace mav2dji 
{

//*****************************************************************************
//*
//*
//*
//******************************************************************************

mav2dji_ros::mav2dji_ros(ros::NodeHandle nh) : rosNodeHandle(nh) 
{
  init();
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

mav2dji_ros::~mav2dji_ros()
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void mav2dji_ros::init()
{
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_ros::startVehicle()
{
    //mavvehicle_ = std::make_shared<mavvehiclelib::mavvehicle>(std::bind(
    //  &mav2dji_ros::vehicleMavMessageCallback, this, std::placeholders::_1));

    mavvehicle_ = std::make_shared<mavvehiclelib::mavvehicle>();
    mission_manager = std::make_unique<mav2dji_mission>(rosNodeHandle, mavvehicle_);

    mavvehicle_->addMavMessageCallback(std::bind(
      &mav2dji_ros::vehicleMavMessageCallback, this, std::placeholders::_1));

    px4_git_version_binary = mavvehicle_->getGitVersion();

    mavvehicle_->startVehicle();
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_ros::stopVehicle()
{
    mavvehicle_->stopVehicle();
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_ros::printMavMessageInfo(const mavlink_message_t* msg, 
  std::string prefix, bool always)
{
  if(verbose | always)
    printf("\n%s : SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
	    prefix.c_str(), msg->sysid, msg->compid, msg->len, msg->msgid);
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int mav2dji_ros::vehicleMavMessageCallback(const mavlink_message_t* msg)
{
	printMavMessageInfo(msg, "Mavlink Message", false);
  ProcessMavMessage(msg);
  return 0;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_ros::ProcessMavMessage(const mavlink_message_t* msg)
{
  //mavlink_message_t msg = msgIn;

  int retcode = mission_manager->ProcessMavMessage(msg);

  if(retcode != 1)
  {
    if(0 > retcode)
    {
      printMavMessageInfo(msg, "--------- ERROR in mission_manager->ProcessMavMessage() ---------------", true);
    }

    return;
  }

  switch (msg->msgid)
  {
    case MAVLINK_MSG_ID_HEARTBEAT:
      processMAVLINK_MSG_ID_HEARTBEAT(msg);
      break;

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
      processMAVLINK_MSG_ID_PARAM_REQUEST_LIST(msg);
      break;

    case MAVLINK_MSG_ID_COMMAND_LONG:
      processMAVLINK_MSG_ID_COMMAND_LONG(msg);
      break;

    case MAVLINK_MSG_ID_COMMAND_INT:
      handle_message_command_int(msg);
      break;

    case MAVLINK_MSG_ID_COMMAND_ACK:
      handle_message_command_ack(msg);
      break;

    case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
      handle_message_optical_flow_rad(msg);
      break;

    case MAVLINK_MSG_ID_PING:
      handle_message_ping(msg);
      break;

    case MAVLINK_MSG_ID_SET_MODE:
      handle_message_set_mode(msg);
      break;

    case MAVLINK_MSG_ID_ATT_POS_MOCAP:
      handle_message_att_pos_mocap(msg);
      break;

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
      handle_message_set_position_target_local_ned(msg);
      break;

    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
      handle_message_set_attitude_target(msg);
      break;

    case MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET:
      handle_message_set_actuator_control_target(msg);
      break;

    case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
      handle_message_vision_position_estimate(msg);
      break;

    case MAVLINK_MSG_ID_ODOMETRY:
      handle_message_odometry(msg);
      break;

    case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
      handle_message_gps_global_origin(msg);
      break;

    case MAVLINK_MSG_ID_RADIO_STATUS:
      handle_message_radio_status(msg);
      break;

    case MAVLINK_MSG_ID_MANUAL_CONTROL:
      handle_message_manual_control(msg);
      break;

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
      handle_message_rc_channels_override(msg);
      break;

    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
      handle_message_distance_sensor(msg);
      break;

    case MAVLINK_MSG_ID_FOLLOW_TARGET:
      handle_message_follow_target(msg);
      break;

    case MAVLINK_MSG_ID_LANDING_TARGET:
      handle_message_landing_target(msg);
      break;

    case MAVLINK_MSG_ID_ADSB_VEHICLE:
      handle_message_adsb_vehicle(msg);
      break;

    case MAVLINK_MSG_ID_COLLISION:
      handle_message_collision(msg);
      break;

    case MAVLINK_MSG_ID_GPS_RTCM_DATA:
      handle_message_gps_rtcm_data(msg);
      break;

    case MAVLINK_MSG_ID_BATTERY_STATUS:
      handle_message_battery_status(msg);
      break;

    case MAVLINK_MSG_ID_SERIAL_CONTROL:
      handle_message_serial_control(msg);
      break;

    case MAVLINK_MSG_ID_LOGGING_ACK:
      handle_message_logging_ack(msg);
      break;

    case MAVLINK_MSG_ID_PLAY_TUNE:
      handle_message_play_tune(msg);
      break;

    case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
      handle_message_obstacle_distance(msg);
      break;

    case MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS:
      handle_message_trajectory_representation_waypoints(msg);
      break;

    case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
      handle_message_named_value_float(msg);
      break;

    case MAVLINK_MSG_ID_DEBUG:
      handle_message_debug(msg);
      break;

    case MAVLINK_MSG_ID_DEBUG_VECT:
      handle_message_debug_vect(msg);
      break;

    case MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY:
      handle_message_debug_float_array(msg);
      break;

    default:
        printf("\nUnhandled message: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
        msg->sysid, msg->compid, msg->len, msg->msgid);
  }
}

//*****************************************************************************

void mav2dji_ros::handle_message_command_int(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : command_int", true);  
}

void mav2dji_ros::handle_message_command_ack(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : command_ack", true);  
}

void mav2dji_ros::handle_message_optical_flow_rad(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : optical_flow_rad", true);  
}

void mav2dji_ros::handle_message_ping(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : message_ping", true);  
}

void mav2dji_ros::handle_message_set_mode(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : set_mode", true);  
}

void mav2dji_ros::handle_message_att_pos_mocap(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : att_pos_mocap", true);  
}

void mav2dji_ros::handle_message_set_position_target_local_ned(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : set_position_target_local_ned", true);  
}

void mav2dji_ros::handle_message_set_attitude_target(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : set_attitude_target", true);  
}

void mav2dji_ros::handle_message_set_actuator_control_target(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : set_actuator_control_target", true);  
}

void mav2dji_ros::handle_message_vision_position_estimate(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : vision_position_estimate", true);  
}

void mav2dji_ros::handle_message_odometry(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : odometry", true);  
}

void mav2dji_ros::handle_message_gps_global_origin(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : gps_global_origin", true);  
}

void mav2dji_ros::handle_message_radio_status(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : radio_status", true);  
}

void mav2dji_ros::handle_message_manual_control(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : manual_control", true);  
}

void mav2dji_ros::handle_message_rc_channels_override(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : rc_channels_override", true);  
}

void mav2dji_ros::handle_message_distance_sensor(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : distance_sensor", true);  
}

void mav2dji_ros::handle_message_follow_target(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : follow_target", true);  
}

void mav2dji_ros::handle_message_landing_target(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : landing_target", true);  
}

void mav2dji_ros::handle_message_adsb_vehicle(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : adsb_vehicle", true);  
}

void mav2dji_ros::handle_message_collision(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : collision", true);  
}

void mav2dji_ros::handle_message_gps_rtcm_data(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : gps_rtcm_data", true);  
}

void mav2dji_ros::handle_message_battery_status(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : battery_status", true);  
}

void mav2dji_ros::handle_message_serial_control(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : serial_control", true);  
}

void mav2dji_ros::handle_message_logging_ack(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : logging_ack", true);  
}

void mav2dji_ros::handle_message_play_tune(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : play_tune", true);  
}

void mav2dji_ros::handle_message_obstacle_distance(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : obstacle_distance", true);  
}

void mav2dji_ros::handle_message_trajectory_representation_waypoints(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : trajectory_representation_waypoints", true);  
}

void mav2dji_ros::handle_message_named_value_float(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : named_value_float", true);  
}

void mav2dji_ros::handle_message_debug(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : debug", true);  
}

void mav2dji_ros::handle_message_debug_vect(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : debug_vect", true);  
}

void mav2dji_ros::handle_message_debug_float_array(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : debug_float_array", true);  
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_ros::processMAVLINK_MSG_ID_HEARTBEAT(const mavlink_message_t* msg) 
{
	printMavMessageInfo(msg, "Mavlink Message : MAVLINK_MSG_ID_HEARTBEAT", false);
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_ros::processMAVLINK_MSG_ID_PARAM_REQUEST_LIST(const mavlink_message_t* msg)
{
	printMavMessageInfo(msg, "Mavlink Message : PARAM_REQUEST_LIST", true);

  mavlink_param_request_list_t req_list;
	mavlink_msg_param_request_list_decode(msg, &req_list);
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_ros::processMAVLINK_MSG_ID_COMMAND_LONG(const mavlink_message_t* msg)
{
  mavlink_command_long_t cmd;
  mavlink_msg_command_long_decode(msg,&cmd);
  switch(cmd.command)
  {
    case MAV_CMD_NAV_TAKEOFF:
      //dji_commands::set_takeoff();
      printMavMessageInfo(msg, "Mavlink Message : COMMAND_LONG : MAV_CMD_NAV_TAKEOFF", true);
    break;

    case MAV_CMD_NAV_LAND:
      //dji_commands::set_land();
      printMavMessageInfo(msg, "Mavlink Message : COMMAND_LONG : MAV_CMD_NAV_LAND", true);
    break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
      //dji_commands::set_return2home();
      printMavMessageInfo(msg, "Mavlink Message : COMMAND_LONG : MAV_CMD_NAV_RETURN_TO_LAUNCH", true);
    break;
    
    case MAV_CMD_REQUEST_PROTOCOL_VERSION:
      //dji_commands::set_return2home();
      printMavMessageInfo(msg, "Mavlink Message : COMMAND_LONG : MAV_CMD_REQUEST_PROTOCOL_VERSION", true);
      processMAV_CMD_REQUEST_PROTOCOL_VERSION(msg);
    break;
    
    case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
      //dji_commands::set_return2home();
      printMavMessageInfo(msg, "Mavlink Message : COMMAND_LONG : MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES", true);
      processMAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES(msg) ;
    break;
    
    default:
      printf("Mavlink Message : COMMAND_LONG : ??? Unhandled command ID : %d ???\n",cmd.command);
  }
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_ros::processMAV_CMD_REQUEST_PROTOCOL_VERSION(const mavlink_message_t* msgIn)
{
	mavlink_message_t msgResp = {};

  // lib checkin hash 5ce595557767fdf2e13ad8190e499fb53e2e92f1
  // spec checkin hash 747c3e73e9ad28e8011cdc5c98faf56752b1d32a

  uint16_t version = 200;
  uint16_t minVersion = 100;
  uint16_t maxVersion = 200;
  uint8_t spec_version_hash = 0x74;
  uint8_t library_version_hash = 0x5c;

  mavlink_msg_protocol_version_pack( 1, 1, &msgResp, version, 
    minVersion, maxVersion, &spec_version_hash, &library_version_hash);
  mavvehicle_->sendMavMessageToGcs(&msgResp);

  printMavMessageInfo(&msgResp, "Sent response to : MAV_CMD_REQUEST_PROTOCOL_VERSION", true);
}

//*****************************************************************************
//*
//* Description : This is supposed to be the 96 bit MCU Unique ID of the mcu,
//*   but we ain no mcu, so we just make up a guid.
//*   see: https://subak.io/code/ardupilot/ArduCopter-3.5/modules/PX4Firmware/src/modules/systemlib/mcu_version.c.html
//*
//*****************************************************************************

void mav2dji_ros::mcu_unique_id(uint32_t *uid_96_bit)
{
	uid_96_bit[0] = 0x01234567;
	uid_96_bit[1] = 0x01234567;
	uid_96_bit[2] = 0x01234567;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

/*void mav2dji_ros::processMAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES(const mavlink_message_t* msgIn) 
{
	//struct vehicle_status_s status;

	//MavlinkOrbSubscription *status_sub = this->add_orb_subscription(ORB_ID(vehicle_status));

	//if (status_sub->update(&status)) {
		mavlink_autopilot_version_t msg = {};

    //mavlink_msg_autopilot_version_pack()
    
		msg.capabilities = MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_COMMAND_INT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_FTP;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_FTP;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET;
		msg.flight_sw_version = 0;
		msg.middleware_sw_version = 0;
		msg.os_sw_version = 0;
		msg.board_version = 0;
		memcpy(&msg.flight_custom_version, &px4_git_version_binary, sizeof(msg.flight_custom_version));
		memcpy(&msg.middleware_custom_version, &px4_git_version_binary, sizeof(msg.middleware_custom_version));
		memset(&msg.os_custom_version, 0, sizeof(msg.os_custom_version));
		#ifdef CONFIG_CDCACM_VENDORID
		msg.vendor_id = CONFIG_CDCACM_VENDORID;
		#else
		msg.vendor_id = 0;
		#endif
		#ifdef CONFIG_CDCACM_PRODUCTID
		msg.product_id = CONFIG_CDCACM_PRODUCTID;
		#else
		msg.product_id = 0;
		#endif
		uint32_t uid[3];
		mcu_unique_id(uid);
		msg.uid = (((uint64_t)uid[1]) << 32) | uid[2];

    mavlink_msg_autopilot_version_pack(1,1,&MmsgOut, )

		//this->send_message(MAVLINK_MSG_ID_AUTOPILOT_VERSION, &msg);

    mavlink_message_t msgOut;

    mavlink_msg_autopilot_version_encode(1,1,&msgOut, &msg);

    mavvehicle_->sendMavMessageToGcs(&msgOut);
	//}
}*/

void mav2dji_ros::processMAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES(const mavlink_message_t* msgIn) 
{
	//struct vehicle_status_s status;

	//MavlinkOrbSubscription *status_sub = this->add_orb_subscription(ORB_ID(vehicle_status));

	//if (status_sub->update(&status)) 
  {
    //* @param capabilities  Bitmap of capabilities
    //* @param flight_sw_version  Firmware version number
    //* @param middleware_sw_version  Middleware version number
    //* @param os_sw_version  Operating system version number
    //* @param board_version  HW / board version (last 8 bytes should be silicon ID, if any)
    //* @param flight_custom_version  Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
    //* @param middleware_custom_version  Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
    //* @param os_custom_version  Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
    //* @param vendor_id  ID of the board vendor
    //* @param product_id  ID of the product
    //* @param uid  UID if provided by hardware (see uid2)
    //* @param uid2  UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise use uid)
    //* @return length of the message in bytes (excluding serial stream start sign)
		
    uint32_t uid3[3];
    mcu_unique_id(uid3);

    uint32_t flight_sw_version = 0; 
    uint32_t middleware_sw_version = 0;
    uint32_t os_sw_version = 0; 
    uint32_t board_version = 0;
    uint8_t  flight_custom_version[8];
    uint8_t  middleware_custom_version[8];
    uint8_t  os_custom_version[8];
    uint16_t vendor_id = 0;
    uint16_t product_id = 0;
		uint64_t uid = (((uint64_t)uid3[1]) << 32) | uid3[2];
    uint8_t  uid2[18];

    uint64_t capabilities =	
        MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT
		  | MAV_PROTOCOL_CAPABILITY_MISSION_INT
		  | MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT
		  | MAV_PROTOCOL_CAPABILITY_COMMAND_INT
		  //| MAV_PROTOCOL_CAPABILITY_FTP
		  | MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET
		  | MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED
		  | MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET
		  | MAV_PROTOCOL_CAPABILITY_MAVLINK2
		  | MAV_PROTOCOL_CAPABILITY_MISSION_FENCE
		  | MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;

    memcpy((void *)flight_custom_version, &px4_git_version_binary, sizeof(flight_custom_version));
		memcpy((void *)middleware_custom_version, &px4_git_version_binary, sizeof(middleware_custom_version));
		memset((void *)os_custom_version, 0, sizeof(os_custom_version));
		memset((void *)uid2, 0, sizeof(uid2));

    mavlink_message_t msgOut;

    mavlink_msg_autopilot_version_pack(1,1,&msgOut, capabilities, flight_sw_version, 
      middleware_sw_version, os_sw_version, board_version, flight_custom_version, 
      middleware_custom_version, os_custom_version, vendor_id, product_id, uid, uid2 );

    mavvehicle_->sendMavMessageToGcs(&msgOut);

    printMavMessageInfo(&msgOut, "Sent response to : MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES", true);
	}
}

}

