/*
 * mav2dji_ros.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <mav2dji_mission.hpp>

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

mav2dji_mission::mav2dji_mission(ros::NodeHandle nh) : nodeHandle_(nh) 
{
  ROS_INFO("[tt_tracker] Node started.");
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

mav2dji_mission::~mav2dji_mission()
{
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::printMavMessageInfo(const mavlink_message_t* msg, 
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

void mav2dji_mission::ProcessMavMessage(const mavlink_message_t* msg)
{
  //mavlink_message_t msg = msgIn;

  switch (msg->msgid)
  {
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
      processMAVLINK_MSG_ID_MISSION_REQUEST_LIST(msg);
      break;

    case MAVLINK_MSG_ID_MISSION_REQUEST:
      processMAVLINK_MSG_ID_MISSION_REQUEST(msg);
      break;

    case MAVLINK_MSG_ID_MISSION_COUNT:
      handle_mission_count(msg);
      break;

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
      processMAVLINK_MSG_ID_MISSION_SET_CURRENT(msg);
      break;

    case MAVLINK_MSG_ID_MISSION_ITEM:
      processMAVLINK_MSG_ID_MISSION_ITEM(msg);
      break;

    default:
        printf("\nUnhandled message: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
        msg->sysid, msg->compid, msg->len, msg->msgid);
  }
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::processMAVLINK_MSG_ID_MISSION_REQUEST_LIST(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : MAVLINK_MSG_ID_MISSION_REQUEST_LIST", true);  
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::processMAVLINK_MSG_ID_MISSION_REQUEST(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : MAVLINK_MSG_ID_MISSION_REQUEST", true);  
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::send_mission_ack(uint8_t sysid, uint8_t compid, uint8_t type)
{
	mavlink_mission_ack_t wpa;

	wpa.target_system = sysid;
	wpa.target_component = compid;
	wpa.type = type;
	wpa.mission_type = _mission_type;

  // PUT BACK
	//mavlink_msg_mission_ack_send_struct(_mavlink->get_channel(), &wpa);
  //

	//PX4_DEBUG("WPM: Send MISSION_ACK type %u to ID %u", wpa.type, wpa.target_system);
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::handle_mission_count(const mavlink_message_t* msg)
{
  printMavMessageInfo(msg, "Mavlink Message : MAVLINK_MSG_ID_MISSION_COUNT", true);  

	mavlink_mission_count_t wpc;
	mavlink_msg_mission_count_decode(msg, &wpc);

	/*if (CHECK_SYSID_COMPID_MISSION(wpc)) {
		if (_state == MAVLINK_WPM_STATE_IDLE) {
			_time_last_recv = hrt_absolute_time();

			if (_transfer_in_progress) {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
				return;
			}

			_transfer_in_progress = true;
			_mission_type = (MAV_MISSION_TYPE)wpc.mission_type;

			if (wpc.count > current_max_item_count()) {
				PX4_DEBUG("WPM: MISSION_COUNT ERROR: too many waypoints (%d), supported: %d", wpc.count, current_max_item_count());

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_NO_SPACE);
				_transfer_in_progress = false;
				return;
			}

			if (wpc.count == 0) {
				PX4_DEBUG("WPM: MISSION_COUNT 0, clearing waypoints list and staying in state MAVLINK_WPM_STATE_IDLE");

				switch (_mission_type) {
				case MAV_MISSION_TYPE_MISSION:

					// alternate dataman ID anyway to let navigator know about changes 

					if (_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0) {
						update_active_mission(DM_KEY_WAYPOINTS_OFFBOARD_1, 0, 0);

					} else {
						update_active_mission(DM_KEY_WAYPOINTS_OFFBOARD_0, 0, 0);
					}

					break;

				case MAV_MISSION_TYPE_FENCE:
					update_geofence_count(0);
					break;

				case MAV_MISSION_TYPE_RALLY:
					update_safepoint_count(0);
					break;

				default:
					PX4_ERR("mission type %u not handled", _mission_type);
					break;
				}

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);
				_transfer_in_progress = false;
				return;
			}

			PX4_DEBUG("WPM: MISSION_COUNT %u from ID %u, changing state to MAVLINK_WPM_STATE_GETLIST", wpc.count, msg->sysid);

			_state = MAVLINK_WPM_STATE_GETLIST;
			_transfer_seq = 0;
			_transfer_partner_sysid = msg->sysid;
			_transfer_partner_compid = msg->compid;
			_transfer_count = wpc.count;
			_transfer_dataman_id = (_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 ? DM_KEY_WAYPOINTS_OFFBOARD_1 :
						DM_KEY_WAYPOINTS_OFFBOARD_0);	// use inactive storage for transmission
			_transfer_current_seq = -1;

			if (_mission_type == MAV_MISSION_TYPE_FENCE) {
				// We're about to write new geofence items, so take the lock. It will be released when
				// switching back to idle
				PX4_DEBUG("locking fence dataman items");

				int ret = dm_lock(DM_KEY_FENCE_POINTS);

				if (ret == 0) {
					_geofence_locked = true;

				} else {
					PX4_ERR("locking failed (%i)", errno);
				}
			}

		} else if (_state == MAVLINK_WPM_STATE_GETLIST) {
			_time_last_recv = hrt_absolute_time();

			if (_transfer_seq == 0) {
				// looks like our MISSION_REQUEST was lost, try again 
				PX4_DEBUG("WPM: MISSION_COUNT %u from ID %u (again)", wpc.count, msg->sysid);

			} else {
				PX4_DEBUG("WPM: MISSION_COUNT ERROR: busy, already receiving seq %u", _transfer_seq);

				_mavlink->send_statustext_critical("WPM: REJ. CMD: Busy");

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
				return;
			}

		} else {
			PX4_DEBUG("WPM: MISSION_COUNT ERROR: busy, state %i", _state);

			_mavlink->send_statustext_critical("WPM: IGN MISSION_COUNT: Busy");
			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			return;
		}

		send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
	}*/
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::processMAVLINK_MSG_ID_MISSION_SET_CURRENT(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : AVLINK_MSG_ID_MISSION_SET_CURRENT", true);  
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::processMAVLINK_MSG_ID_MISSION_ITEM(const mavlink_message_t* recvMsg)
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
