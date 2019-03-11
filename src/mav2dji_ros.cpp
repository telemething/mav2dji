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

mav2dji_ros::mav2dji_ros(ros::NodeHandle nh) : nodeHandle_(nh) 
{
  ROS_INFO("[tt_tracker] Node started.");
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
//*****************************************************************************

void mav2dji_ros::startVehicle()
{
    mavvehicle_ = std::make_unique<mavvehiclelib::mavvehicle>(std::bind(
      &mav2dji_ros::vehicleMavMessageCallback, this, std::placeholders::_1));

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

int mav2dji_ros::vehicleMavMessageCallback(mavlink_message_t msg)
{
	printMavMessageInfo(&msg, "Mavlink Message", false);
  ProcessMavMessage(msg);

  return 0;
}

//*****************************************************************************
//*

//*
//*
//*****************************************************************************

void mav2dji_ros::ProcessMavMessage(mavlink_message_t msgIn)
{
  mavlink_message_t msg = msgIn;

  switch (msg.msgid)
  {
    case MAVLINK_MSG_ID_HEARTBEAT:
      processMAVLINK_MSG_ID_HEARTBEAT(msg);
    break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
      processMAVLINK_MSG_ID_PARAM_REQUEST_LIST(msg);
    break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
      processMAVLINK_MSG_ID_COMMAND_LONG(&msg);
    break;
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
      processMAVLINK_MSG_ID_MISSION_REQUEST_LIST(msg);
    break;
    case MAVLINK_MSG_ID_MISSION_REQUEST:
      processMAVLINK_MSG_ID_MISSION_REQUEST(msg);
    break;
    case MAVLINK_MSG_ID_MISSION_COUNT:
      processMAVLINK_MSG_ID_MISSION_COUNT(msg);
    break;
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
      processMAVLINK_MSG_ID_MISSION_SET_CURRENT(msg);
    break;
    case MAVLINK_MSG_ID_MISSION_ITEM:
      processMAVLINK_MSG_ID_MISSION_ITEM(msg);
    break;
    default:
      printf("\nUnhandled message: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
			msg.sysid, msg.compid, msg.len, msg.msgid);
  }
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_ros::processMAVLINK_MSG_ID_HEARTBEAT(mavlink_message_t msg)
{
	printMavMessageInfo(&msg, "Mavlink Message : Hello", false);
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_ros::processMAVLINK_MSG_ID_PARAM_REQUEST_LIST(mavlink_message_t msg)
{
	printMavMessageInfo(&msg, "Mavlink Message : PARAM_REQUEST_LIST", true);
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

void mav2dji_ros::processMAVLINK_MSG_ID_MISSION_REQUEST_LIST(mavlink_message_t msg)
{

}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_ros::processMAVLINK_MSG_ID_MISSION_REQUEST(mavlink_message_t msg)
{

}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_ros::processMAVLINK_MSG_ID_MISSION_COUNT(mavlink_message_t msg)
{

}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_ros::processMAVLINK_MSG_ID_MISSION_SET_CURRENT(mavlink_message_t msg)
{

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

void mav2dji_ros::processMAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES(const mavlink_message_t* msgIn) 
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

		//this->send_message(MAVLINK_MSG_ID_AUTOPILOT_VERSION, &msg);

    mavlink_message_t msgOut;

    mavlink_msg_autopilot_version_encode(1,1,&msgOut, &msg);

    mavvehicle_->sendMavMessageToGcs(&msgOut);
	//}
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_ros::processMAVLINK_MSG_ID_MISSION_ITEM(mavlink_message_t recvMsg)
{
  /*if(recvMsg.compid != MAV_COMP_ID_MISSIONPLANNER) 
  {
    DJI2MAV_DEBUG("In Hotpoint the compid is %u.", 
      recvMsg.compid);
  }

  DJI2MAV_DEBUG("In Hotpoint mission item with status: %d.", 
    (int)m_status);

  switch(m_status) 
  {
    case loaded:
    case executing:
    case idle:
    case uploading:
    case paused:
    case error:
      return;
    case downloading:
    break;
  }

  mavlink_mission_item_t itemMsg;
  mavlink_msg_mission_item_decode(&recvMsg, &itemMsg);

  if(itemMsg.seq == 0) 
  {
    m_hp.setHotpointData(itemMsg.seq, itemMsg.command, 
    itemMsg.param1, itemMsg.param2, itemMsg.param3, 
    itemMsg.param4, itemMsg.x, itemMsg.y, itemMsg.z);
  } 
  else 
  {
    DJI2MAV_ERROR("Invalid sequence %u of mission item in " 
     "Hotpoint!", itemMsg.seq);
    return;
  }

  mavlink_message_t sendMsg;
  mavlink_msg_mission_ack_pack(getMySysid(), 
                        MAV_COMP_ID_MISSIONPLANNER, &sendMsg, 
                        recvMsg.sysid, recvMsg.compid, 
                        MAV_MISSION_ACCEPTED);
  sendMsgToMaster(sendMsg);
  m_status = loaded;
  m_hp.display();

  if(NULL != m_missionItemHook)
    m_missionItemHook(itemMsg.seq);*/

}

}

