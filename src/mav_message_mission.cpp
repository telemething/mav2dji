/*
 * mav2dji_ros.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <mav_message_mission.hpp>

#define CHECK_SYSID_COMPID_MISSION(_msg) true

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

//mav2dji_mission::mav2dji_mission(
//	ros::NodeHandle nh, 
//	std::shared_ptr<mavvehiclelib::mav_udp> mavUdp) : nodeHandle_(nh)
//{
//	mav_udp_ = mavUdp;
//  ROS_INFO("[tt_tracker] Node started.");
//}

//mav2dji_mission::mav2dji_mission(
//	ros::NodeHandle nh) : nodeHandle_(nh)
//{
//  ROS_INFO("[tt_tracker] Node started.");
//}

mav2dji_mission::mav2dji_mission()
{
  //ROS_INFO("[tt_tracker] Node started.");
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
//* return 0 = handled, -1 = error, 1 = unhandled
//*
//*****************************************************************************

int mav2dji_mission::ProcessMavMessage(const mavlink_message_t* msg)
{
  //mavlink_message_t msg = msgIn;

  switch (msg->msgid)
  {
		case MAVLINK_MSG_ID_MISSION_ACK:
			handle_mission_ack(msg);
			break;

		case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
			handle_mission_set_current(msg);
			break;

		case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
			handle_mission_request_list(msg);
			break;

		case MAVLINK_MSG_ID_MISSION_REQUEST:
			handle_mission_request(msg);
			break;

		case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
			handle_mission_request_int(msg);
			break;

		case MAVLINK_MSG_ID_MISSION_COUNT:
			handle_mission_count(msg);
			break;

		case MAVLINK_MSG_ID_MISSION_ITEM:
			handle_mission_item(msg);
			break;

		case MAVLINK_MSG_ID_MISSION_ITEM_INT:
			handle_mission_item_int(msg);
			break;

		case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
			handle_mission_clear_all(msg);
			break;

    default:
        return 1;
	}
	
	return 0;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::send_mission_ack(uint8_t targetSysid, uint8_t targetCompid, uint8_t type)
{
	//mavlink_mission_ack_t wpa;

	//wpa.target_system = targetSysid;
	//wpa.target_component = targetCompid;
	//wpa.type = type;
	//wpa.mission_type = _mission_type;

  // PUT BACK
	//mavlink_msg_mission_ack_send_struct(_mavlink->get_channel(), &wpa);

	mavlink_message_t msgOut;

	mavlink_msg_mission_ack_pack(
		getMavlinkSystemId(), getMavlinkComponentId(), 
		&msgOut, targetSysid, targetCompid, type, _mission_type );
  
	sendMavMessageToGcs(&msgOut);

	PX4_DEBUG("WPM: Send MISSION_ACK type %u to ID %u", type, targetSysid);
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

uint16_t mav2dji_mission::current_max_item_count()
{
	if (_mission_type >= sizeof(MAX_COUNT) / sizeof(MAX_COUNT[0])) 
	{
		PX4_ERR("WPM: MAX_COUNT out of bounds (%u)", _mission_type);       
		return 0;
	}

	return MAX_COUNT[_mission_type];
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int mav2dji_mission::update_geofence_count(unsigned count)
{
	mission_stats_entry_s stats;
	stats.num_items = count;
	stats.update_counter = ++_geofence_update_counter; // this makes sure navigator will reload the fence data

	/// update stats in dataman 
	int res = dm_write(DM_KEY_FENCE_POINTS, 0, DM_PERSIST_POWER_ON_RESET, &stats, sizeof(mission_stats_entry_s));

	if (res == sizeof(mission_stats_entry_s)) {
		_count[MAV_MISSION_TYPE_FENCE] = count;

	} else {

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			//_mavlink->send_statustext_critical("Mission storage: Unable to write to microSD");
			send_statustext_critical("Mission storage: Unable to write to microSD");
		}

		return PX4_ERROR;
	}

	return PX4_OK;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int mav2dji_mission::update_safepoint_count(unsigned count)
{
	mission_stats_entry_s stats;
	stats.num_items = count;

	// update stats in dataman 
	int res = dm_write(DM_KEY_SAFE_POINTS, 0, DM_PERSIST_POWER_ON_RESET, &stats, sizeof(mission_stats_entry_s));

	if (res == sizeof(mission_stats_entry_s)) {
		_count[MAV_MISSION_TYPE_RALLY] = count;

	} else {

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			//_mavlink->send_statustext_critical("Mission storage: Unable to write to microSD");
			send_statustext_critical("Mission storage: Unable to write to microSD");
		}

		return PX4_ERROR;
	}

	return PX4_OK;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

mav2dji::MissionWaypointTask mav2dji_mission::Convert(mission_item_s missionItem)
{

}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int mav2dji_mission::update_active_mission(dm_item_t dataman_id, uint16_t count, int32_t seq)
{
	mission_s mission;
	mission.timestamp = hrt_absolute_time();
	mission.dataman_id = dataman_id;
	mission.count = count;
	mission.current_seq = seq;

	std::shared_ptr<mav2dji::vehicle_interface> vehicleInterface = VehicleInfo::getVehicleInterface();

	

	//*** new ***
	//make an async call, send missionItemList 

	// update mission state in dataman 

	// lock MISSION_STATE item 
	int dm_lock_ret = dm_lock(DM_KEY_MISSION_STATE);

	if (dm_lock_ret != 0) {
		PX4_ERR("DM_KEY_MISSION_STATE lock failed");
	}

	int res = dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission, sizeof(mission_s));

	// unlock MISSION_STATE item 
	if (dm_lock_ret == 0) 
	{
		dm_unlock(DM_KEY_MISSION_STATE);
	}

	if (res == sizeof(mission_s)) 
	{
		/// update active mission state 
		_dataman_id = dataman_id;
		_count[MAV_MISSION_TYPE_MISSION] = count;
		_current_seq = seq;
		_my_dataman_id = _dataman_id;

		// mission state saved successfully, publish offboard_mission topic 
		if (_offboard_mission_pub == nullptr) 
		{
			//*** TODO, figure out if we need to do anything
			//_offboard_mission_pub = orb_advertise(ORB_ID(mission), &mission);
		} 
		else 
		{
			//*** TODO, figure out if we need to do anything
			//orb_publish(ORB_ID(mission), _offboard_mission_pub, &mission);
		}

		return PX4_OK;

	} 
	else 
	{
		PX4_ERR("WPM: can't save mission state");

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) 
		{
			//_mavlink->send_statustext_critical("Mission storage: Unable to write to microSD");
			send_statustext_critical("Mission storage: Unable to write to microSD");
		}

		return PX4_ERROR;
	}
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::send_mission_request(uint8_t targetSysid, uint8_t targetCompid, uint16_t seq)
{
	if (seq < current_max_item_count()) 
	{
		_time_last_sent = hrt_absolute_time();

		mavlink_message_t msgOut;

		if (_int_mode) 
		{
			//mavlink_mission_request_int_t wpr;
			//wpr.target_system = targetSysid;
			//wpr.target_component = targetCompid;
			//wpr.seq = seq;
			//wpr.mission_type = _mission_type;
			//mavlink_msg_mission_request_int_send_struct(_mavlink->get_channel(), &wpr);

			mavlink_msg_mission_request_int_pack(
				getMavlinkSystemId(), getMavlinkComponentId(), 
				&msgOut, targetSysid, targetCompid, seq, _mission_type );

			PX4_DEBUG("WPM: Send MISSION_REQUEST_INT seq %u to ID %u", seq, targetSysid);

		} 
		else 
		{
			//mavlink_mission_request_t wpr;
			//wpr.target_system = targetSysid;
			//wpr.target_component = targetCompid;
			//wpr.seq = seq;
			//wpr.mission_type = _mission_type;

			mavlink_msg_mission_request_pack(
				getMavlinkSystemId(), getMavlinkComponentId(), 
				&msgOut, targetSysid, targetCompid, seq, _mission_type );

			//mavlink_msg_mission_request_send_struct(_mavlink->get_channel(), &wpr);

			PX4_DEBUG("WPM: Send MISSION_REQUEST seq %u to ID %u", seq, targetSysid);
		}

		sendMavMessageToGcs(&msgOut);
	} 
	else 
	{
		send_statustext_critical("ERROR: Waypoint index exceeds list capacity");
		PX4_DEBUG("WPM: Send MISSION_REQUEST ERROR: seq %u exceeds list capacity", seq);
	}
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int mav2dji_mission::parse_mavlink_mission_item(const mavlink_mission_item_t *mavlink_mission_item,
		struct mission_item_s *mission_item)
{
		if (mavlink_mission_item->frame == MAV_FRAME_GLOBAL ||
	    mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT ||
	    (_int_mode && (mavlink_mission_item->frame == MAV_FRAME_GLOBAL_INT ||
			   mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT))) {

		// Switch to int mode if that is what we are receiving
		if ((mavlink_mission_item->frame == MAV_FRAME_GLOBAL_INT ||
		     mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)) {
			_int_mode = true;
		}

		if (_int_mode) {
			//* The argument is actually a mavlink_mission_item_int_t in int_mode.
			// * mavlink_mission_item_t and mavlink_mission_item_int_t have the same
			// * alignment, so we can just swap float for int32_t. 
			const mavlink_mission_item_int_t *item_int
				= reinterpret_cast<const mavlink_mission_item_int_t *>(mavlink_mission_item);
			mission_item->lat = ((double)item_int->x) * 1e-7;
			mission_item->lon = ((double)item_int->y) * 1e-7;

		} else {
			mission_item->lat = (double)mavlink_mission_item->x;
			mission_item->lon = (double)mavlink_mission_item->y;
		}

		mission_item->altitude = mavlink_mission_item->z;

		if (mavlink_mission_item->frame == MAV_FRAME_GLOBAL ||
		    mavlink_mission_item->frame == MAV_FRAME_GLOBAL_INT) {
			mission_item->altitude_is_relative = false;

		} else if (mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT ||
			   mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
			mission_item->altitude_is_relative = true;
		}

		//* this field is shared with pitch_min (and circle_radius for geofence) in memory and
		//* exclusive in the MAVLink spec. Set it to 0 first
		//* and then set minimum pitch later only for the
		//* corresponding item
		
		mission_item->time_inside = 0.0f;

		switch (mavlink_mission_item->command) {
		case MAV_CMD_NAV_WAYPOINT:
			mission_item->nav_cmd = NAV_CMD_WAYPOINT;
			mission_item->time_inside = mavlink_mission_item->param1;
			mission_item->acceptance_radius = mavlink_mission_item->param2;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LOITER_UNLIM:
			mission_item->nav_cmd = NAV_CMD_LOITER_UNLIMITED;
			mission_item->loiter_radius = mavlink_mission_item->param3;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LOITER_TIME:
			mission_item->nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
			mission_item->time_inside = mavlink_mission_item->param1;
			mission_item->loiter_radius = mavlink_mission_item->param3;
			mission_item->loiter_exit_xtrack = (mavlink_mission_item->param4 > 0);
			// Yaw is only valid for multicopter but we set it always because
			// it's just ignored for fixedwing.
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LAND:
			mission_item->nav_cmd = NAV_CMD_LAND;
			// TODO: abort alt param1
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			mission_item->land_precision = mavlink_mission_item->param2;
			break;

		case MAV_CMD_NAV_TAKEOFF:
			mission_item->nav_cmd = NAV_CMD_TAKEOFF;
			mission_item->pitch_min = mavlink_mission_item->param1;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LOITER_TO_ALT:
			mission_item->nav_cmd = NAV_CMD_LOITER_TO_ALT;
			mission_item->force_heading = (mavlink_mission_item->param1 > 0);
			mission_item->loiter_radius = mavlink_mission_item->param2;
			mission_item->loiter_exit_xtrack = (mavlink_mission_item->param4 > 0);
			break;

		case MAV_CMD_NAV_ROI:
		case MAV_CMD_DO_SET_ROI:
			if ((int)mavlink_mission_item->param1 == MAV_ROI_LOCATION) {
				mission_item->nav_cmd = NAV_CMD_DO_SET_ROI;
				mission_item->params[0] = MAV_ROI_LOCATION;

				mission_item->params[6] = mavlink_mission_item->z;

			} else if ((int)mavlink_mission_item->param1 == MAV_ROI_NONE) {
				mission_item->nav_cmd = NAV_CMD_DO_SET_ROI;
				mission_item->params[0] = MAV_ROI_NONE;

			} else {
				return MAV_MISSION_INVALID_PARAM1;
			}

			break;

		case MAV_CMD_DO_SET_ROI_LOCATION:
			mission_item->nav_cmd = NAV_CMD_DO_SET_ROI_LOCATION;
			mission_item->params[6] = mavlink_mission_item->z;
			break;

		case MAV_CMD_NAV_VTOL_TAKEOFF:
		case MAV_CMD_NAV_VTOL_LAND:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_FENCE_RETURN_POINT:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			break;

		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			mission_item->vertex_count = (uint16_t)(mavlink_mission_item->param1 + 0.5f);
			break;

		case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
		case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			mission_item->circle_radius = mavlink_mission_item->param1;
			break;

		case MAV_CMD_NAV_RALLY_POINT:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			break;

		default:
			mission_item->nav_cmd = NAV_CMD_INVALID;

			PX4_DEBUG("Unsupported command %d", mavlink_mission_item->command);

			return MAV_MISSION_UNSUPPORTED;
		}

		mission_item->frame = mavlink_mission_item->frame;

	} else if (mavlink_mission_item->frame == MAV_FRAME_MISSION) {

		// this is a mission item with no coordinates

		mission_item->params[0] = mavlink_mission_item->param1;
		mission_item->params[1] = mavlink_mission_item->param2;
		mission_item->params[2] = mavlink_mission_item->param3;
		mission_item->params[3] = mavlink_mission_item->param4;
		mission_item->params[4] = mavlink_mission_item->x;
		mission_item->params[5] = mavlink_mission_item->y;
		mission_item->params[6] = mavlink_mission_item->z;

		switch (mavlink_mission_item->command) {
		case MAV_CMD_DO_JUMP:
			mission_item->nav_cmd = NAV_CMD_DO_JUMP;
			mission_item->do_jump_mission_index = mavlink_mission_item->param1;
			mission_item->do_jump_current_count = 0;
			mission_item->do_jump_repeat_count = mavlink_mission_item->param2;
			break;

		case MAV_CMD_NAV_ROI:
		case MAV_CMD_DO_SET_ROI: {
				const int roi_mode = mavlink_mission_item->param1;

				if (roi_mode == MAV_ROI_NONE || roi_mode == MAV_ROI_WPNEXT || roi_mode == MAV_ROI_WPINDEX) {
					mission_item->nav_cmd = NAV_CMD_DO_SET_ROI;

				} else {
					return MAV_MISSION_INVALID_PARAM1;
				}
			}
			break;

		case MAV_CMD_DO_CHANGE_SPEED:
		case MAV_CMD_DO_SET_HOME:
		case MAV_CMD_DO_SET_SERVO:
		case MAV_CMD_DO_LAND_START:
		case MAV_CMD_DO_TRIGGER_CONTROL:
		case MAV_CMD_DO_DIGICAM_CONTROL:
		case MAV_CMD_DO_MOUNT_CONFIGURE:
		case MAV_CMD_DO_MOUNT_CONTROL:
		case MAV_CMD_IMAGE_START_CAPTURE:
		case MAV_CMD_IMAGE_STOP_CAPTURE:
		case MAV_CMD_VIDEO_START_CAPTURE:
		case MAV_CMD_VIDEO_STOP_CAPTURE:
		case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
		case MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
		case MAV_CMD_SET_CAMERA_MODE:
		case MAV_CMD_DO_VTOL_TRANSITION:
		case MAV_CMD_NAV_DELAY:
		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
		case MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET:
		case MAV_CMD_DO_SET_ROI_NONE:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			break;

		default:
			mission_item->nav_cmd = NAV_CMD_INVALID;

			PX4_DEBUG("Unsupported command %d", mavlink_mission_item->command);

			return MAV_MISSION_UNSUPPORTED;
		}

		mission_item->frame = MAV_FRAME_MISSION;

	} else {
		PX4_DEBUG("Unsupported frame %d", mavlink_mission_item->frame);

		return MAV_MISSION_UNSUPPORTED_FRAME;
	}

	mission_item->autocontinue = mavlink_mission_item->autocontinue;
	// mission_item->index = mavlink_mission_item->seq;

	mission_item->origin = ORIGIN_MAVLINK;

	return MAV_MISSION_ACCEPTED;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::switch_to_idle_state()
{
	// when switching to idle, we *always* check if the lock was held and release it.
	// This is to ensure we don't end up in a state where we forget to release it.
	if (_geofence_locked) {
		dm_unlock(DM_KEY_FENCE_POINTS);
		_geofence_locked = false;

		PX4_DEBUG("unlocking geofence");
	}

	_state = MAVLINK_WPM_STATE_IDLE;
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

	if (CHECK_SYSID_COMPID_MISSION(wpc)) 
	{
		if (_state == MAVLINK_WPM_STATE_IDLE) 
		{
			_time_last_recv = hrt_absolute_time();

			if (_transfer_in_progress) 
			{
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
				return;
			}

			_transfer_in_progress = true;
			_mission_type = (MAV_MISSION_TYPE)wpc.mission_type;

			if (wpc.count > current_max_item_count()) 
			{
				PX4_DEBUG("WPM: MISSION_COUNT ERROR: too many waypoints (%d), supported: %d", wpc.count, current_max_item_count());

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_NO_SPACE);
				_transfer_in_progress = false;
				return;
			}

			if (wpc.count == 0) 
			{
				PX4_DEBUG("WPM: MISSION_COUNT 0, clearing waypoints list and staying in state MAVLINK_WPM_STATE_IDLE");

				switch (_mission_type) 
				{
				case MAV_MISSION_TYPE_MISSION:

					// alternate dataman ID anyway to let navigator know about changes 

					if (_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0) 
					{
						update_active_mission(DM_KEY_WAYPOINTS_OFFBOARD_1, 0, 0);
					} 
					else 
					{
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

			if (_mission_type == MAV_MISSION_TYPE_FENCE) 
			{
				// We're about to write new geofence items, so take the lock. It will be released when
				// switching back to idle
                
				PX4_DEBUG("locking fence dataman items");

				int ret = dm_lock(DM_KEY_FENCE_POINTS);

				if (ret == 0) 
				{
					_geofence_locked = true;

				} 
				else 
				{
					PX4_ERR("locking failed (%i)", errno);
				}
			}
		} 
		else if (_state == MAVLINK_WPM_STATE_GETLIST) 
		{
			_time_last_recv = hrt_absolute_time();

			if (_transfer_seq == 0) 
			{
				// looks like our MISSION_REQUEST was lost, try again 

				PX4_DEBUG("WPM: MISSION_COUNT %u from ID %u (again)", wpc.count, msg->sysid);
			} 
			else 
			{
				PX4_DEBUG("WPM: MISSION_COUNT ERROR: busy, already receiving seq %u", _transfer_seq);

				send_statustext_critical("WPM: REJ. CMD: Busy");

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
				return;
			}
		} 
		else
		{
			PX4_DEBUG("WPM: MISSION_COUNT ERROR: busy, state %i", _state);

			send_statustext_critical("WPM: IGN MISSION_COUNT: Busy");
			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			return;
		}

		send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
		printMavMessageInfo(msg, "Sent response to : MAVLINK_MSG_ID_MISSION_COUNT", true);
	}
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::handle_mission_request_list(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : MAVLINK_MSG_ID_MISSION_REQUEST_LIST", true);  
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::handle_mission_request_int(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : MAVLINK_MSG_ID_MISSION_REQUEST_INT", true);  
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::handle_mission_request(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : MAVLINK_MSG_ID_MISSION_REQUEST", true);  
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::handle_mission_set_current(const mavlink_message_t* msg) 
{
  printMavMessageInfo(msg, "Mavlink Message : MAVLINK_MSG_ID_MISSION_SET_CURRENT", true);  
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::handle_mission_ack(const mavlink_message_t* msg)
{
  printMavMessageInfo(msg, "Mavlink Message : MAVLINK_MSG_ID_MISSION_ACK", true);  
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::handle_mission_item(const mavlink_message_t* msg)
{
  printMavMessageInfo(msg, "Mavlink Message : MAVLINK_MSG_ID_MISSION_ITEM", true);  

	if (_int_mode) 
	{
		_int_mode = false;
	}

	handle_mission_item_both(msg);
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::handle_mission_item_int(const mavlink_message_t* msg)
{
  printMavMessageInfo(msg, "Mavlink Message : MAVLINK_MSG_ID_MISSION_ITEM_INT", true);  

	if (!_int_mode) 
	{
		_int_mode = true;
	}

	handle_mission_item_both(msg);
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::handle_mission_item_both(const mavlink_message_t *msg)
{
	// The mavlink_message could also contain a mavlink_mission_item_int_t. We ignore that here
	// and take care of it later in parse_mavlink_mission_item depending on _int_mode.

	printf("--- WPM ---: Got a new MISSION_ITEM\n");

	mavlink_mission_item_t wp;
	mavlink_msg_mission_item_decode(msg, &wp);

	if (CHECK_SYSID_COMPID_MISSION(wp)) 
	{

		if (wp.mission_type != _mission_type) 
		{
			PX4_WARN("WPM: Unexpected mission type (%u %u)", (int)wp.mission_type, (int)_mission_type);
			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			return;
		}

		if (_state == MAVLINK_WPM_STATE_GETLIST) 
		{
			_time_last_recv = hrt_absolute_time();

			if (wp.seq != _transfer_seq) {
				PX4_DEBUG("WPM: MISSION_ITEM ERROR: seq %u was not the expected %u", wp.seq, _transfer_seq);

				/* request next item again */
				send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
				return;
			}

		} 
		else if (_state == MAVLINK_WPM_STATE_IDLE) 
		{
			if (_transfer_seq == wp.seq + 1) 
			{
				// Assume this is a duplicate, where we already successfully got all mission items,
				// but the GCS did not receive the last ack and sent the same item again
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);

			} else {
				PX4_DEBUG("WPM: MISSION_ITEM ERROR: no transfer");

				//_mavlink->send_statustext_critical("IGN MISSION_ITEM: No transfer");
				send_statustext_critical("IGN MISSION_ITEM: No transfer");
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			}

			return;

		} 
		else 
		{
			PX4_DEBUG("WPM: MISSION_ITEM ERROR: busy, state %i", _state);

			//_mavlink->send_statustext_critical("IGN MISSION_ITEM: Busy");
			send_statustext_critical("IGN MISSION_ITEM: Busy");
			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			return;
		}

		struct mission_item_s mission_item = {};

		int ret = parse_mavlink_mission_item(&wp, &mission_item);

		if (ret != PX4_OK) 
		{
			PX4_DEBUG("WPM: MISSION_ITEM ERROR: seq %u invalid item", wp.seq);

			//_mavlink->send_statustext_critical("IGN MISSION_ITEM: Busy");
			send_statustext_critical("IGN MISSION_ITEM: Busy");

			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, ret);
			switch_to_idle_state();
			_transfer_in_progress = false;
			return;
		}

		bool write_failed = false;
		bool check_failed = false;

		switch (_mission_type) 
		{
			case MAV_MISSION_TYPE_MISSION: 
			{
					// check that we don't get a wrong item (hardening against wrong client implementations, the list here
					// does not need to be complete)
					if (mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION ||
							mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION ||
							mission_item.nav_cmd == MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION ||
							mission_item.nav_cmd == MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION ||
							mission_item.nav_cmd == MAV_CMD_NAV_RALLY_POINT) 
					{
						check_failed = true;
					} 
					else 
					{
						dm_item_t dm_item = _transfer_dataman_id;

						//*** new ***
						missionItemList.push_back(mission_item);

						write_failed = dm_write(dm_item, wp.seq, DM_PERSIST_POWER_ON_RESET, &mission_item,
									sizeof(struct mission_item_s)) != sizeof(struct mission_item_s);

						if (!write_failed) 
						{
							/* waypoint marked as current */
							if (wp.current) 
							{
								_transfer_current_seq = wp.seq;
							}
						}
					}
			}
			break;

			case MAV_MISSION_TYPE_FENCE: 
			{ // Write a geofence point
					mission_fence_point_s mission_fence_point;
					mission_fence_point.nav_cmd = mission_item.nav_cmd;
					mission_fence_point.lat = mission_item.lat;
					mission_fence_point.lon = mission_item.lon;
					mission_fence_point.alt = mission_item.altitude;

					if (mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION ||
							mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION) {
						mission_fence_point.vertex_count = mission_item.vertex_count;

						if (mission_item.vertex_count < 3) 
						{ // feasibility check
							PX4_ERR("Fence: too few vertices");
							check_failed = true;
							update_geofence_count(0);
						}

					} 
					else 
					{
						mission_fence_point.circle_radius = mission_item.circle_radius;
					}

					mission_fence_point.frame = mission_item.frame;

					if (!check_failed) 
					{
						write_failed = dm_write(DM_KEY_FENCE_POINTS, wp.seq + 1, DM_PERSIST_POWER_ON_RESET, &mission_fence_point,
									sizeof(mission_fence_point_s)) != sizeof(mission_fence_point_s);
					}

			}
			break;

			case MAV_MISSION_TYPE_RALLY: 
			{ // Write a safe point / rally point
					mission_save_point_s mission_save_point;
					mission_save_point.lat = mission_item.lat;
					mission_save_point.lon = mission_item.lon;
					mission_save_point.alt = mission_item.altitude;
					mission_save_point.frame = mission_item.frame;
					write_failed = dm_write(DM_KEY_SAFE_POINTS, wp.seq + 1, DM_PERSIST_POWER_ON_RESET, &mission_save_point,
								sizeof(mission_save_point_s)) != sizeof(mission_save_point_s);
			}
			break;

			default:
				//_mavlink->send_statustext_critical("Received unknown mission type, abort.");
				send_statustext_critical("Received unknown mission type, abort.");
				break;
		}

		if (write_failed || check_failed) 
		{
			PX4_DEBUG("WPM: MISSION_ITEM ERROR: error writing seq %u to dataman ID %i", wp.seq, _transfer_dataman_id);

			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);

			if (write_failed) 
			{
				//_mavlink->send_statustext_critical("Unable to write on micro SD");
				send_statustext_critical("Unable to write on micro SD");
			}

			switch_to_idle_state();
			_transfer_in_progress = false;
			return;
		}

		/* waypoint marked as current */
		if (wp.current) 
		{
			_transfer_current_seq = wp.seq;
		}

		printf("WPM: MISSION_ITEM seq %u processed\n", wp.seq);
		PX4_DEBUG("WPM: MISSION_ITEM seq %u received", wp.seq);

		_transfer_seq = wp.seq + 1;

		if (_transfer_seq == _transfer_count) 
		{
			/* got all new mission items successfully */
			printf("--- WPM ---: MISSION_ITEM got all %u items, current_seq=%u, changing state to MAVLINK_WPM_STATE_IDLE\n",
				  _transfer_count, _transfer_current_seq);

			PX4_DEBUG("WPM: MISSION_ITEM got all %u items, current_seq=%u, changing state to MAVLINK_WPM_STATE_IDLE",
				  _transfer_count, _transfer_current_seq);

			ret = 0;

			switch (_mission_type) 
			{
				case MAV_MISSION_TYPE_MISSION:
					ret = update_active_mission(_transfer_dataman_id, _transfer_count, _transfer_current_seq);
					break;

				case MAV_MISSION_TYPE_FENCE:
					ret = update_geofence_count(_transfer_count);
					break;

				case MAV_MISSION_TYPE_RALLY:
					ret = update_safepoint_count(_transfer_count);
					break;

				default:
					PX4_ERR("mission type %u not handled", _mission_type);
				break;
			}

			// Note: the switch to idle needs to happen after update_geofence_count is called, for proper unlocking order
			switch_to_idle_state();

			if (ret == PX4_OK) 
			{
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);
			} 
			else 
			{
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			}

			_transfer_in_progress = false;
		} 
		else 
		{
			printf("--- WPM ---: Requesting next MISSION_ITEM\n");

			/* request next item */
			send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
		}
	}
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::handle_mission_clear_all(const mavlink_message_t* msg)
{
  printMavMessageInfo(msg, "Mavlink Message : MAVLINK_MSG_ID_MISSION_CLEAR_ALL", true);  
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void mav2dji_mission::processMAVLINK_MSG_ID_MISSION_ITEM(const mavlink_message_t* recvMsg)
{
	printMavMessageInfo(recvMsg, "Mavlink Message : MAVLINK_MSG_ID_MISSION_ITEM", true);  

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
