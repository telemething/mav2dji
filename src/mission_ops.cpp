#include <mission_ops.hpp>

namespace mav2dji 
{

//***************************************************************************
//*
//* Convert
//*
//***************************************************************************

std::shared_ptr<dji_sdk::MissionWaypointTask> 
  MissionOps::Convert(const mav2dji::MissionWaypointTask* waypointTask )
{
    auto wpt = std::make_shared<dji_sdk::MissionWaypointTask>();

    wpt->velocity_range     = waypointTask->velocity_range;
    wpt->idle_velocity      = waypointTask->idle_velocity;
    wpt->action_on_finish   = waypointTask->action_on_finish;
    wpt->mission_exec_times = waypointTask->mission_exec_times;
    wpt->yaw_mode           = waypointTask->yaw_mode;
    wpt->trace_mode         = waypointTask->trace_mode;
    wpt->action_on_rc_lost  = waypointTask->action_on_rc_lost;
    wpt->gimbal_pitch_mode  = waypointTask->gimbal_pitch_mode;

    for(auto wPointIn : wpt->mission_waypoint )
    {
        dji_sdk::MissionWaypoint wPointOut;

        wPointOut.latitude                      = wPointIn.latitude;
        wPointOut.longitude                     = wPointIn.longitude;
        wPointOut.altitude                      = wPointIn.altitude;
        wPointOut.damping_distance              = wPointIn.damping_distance;
        wPointOut.target_yaw                    = wPointIn.target_yaw;
        wPointOut.target_gimbal_pitch           = wPointIn.target_gimbal_pitch;
        wPointOut.turn_mode                     = wPointIn.turn_mode;
        wPointOut.has_action                    = wPointIn.has_action;
        wPointOut.action_time_limit             = wPointIn.action_time_limit;
        wPointOut.waypoint_action.action_repeat = wPointIn.waypoint_action.action_repeat;

        std::copy(std::begin(wPointIn.waypoint_action.command_list),
                  std::end(wPointIn.waypoint_action.command_list), 
                  wPointOut.waypoint_action.command_list.begin());

        std::copy(std::begin(wPointIn.waypoint_action.command_parameter),
                  std::end(wPointIn.waypoint_action.command_parameter), 
                  wPointOut.waypoint_action.command_parameter.begin());

        wpt->mission_waypoint.push_back(wPointOut);
    }

    return wpt; 
}

  //***************************************************************************
  //*
  //* Convert
  //*
  //***************************************************************************

  std::shared_ptr<mav2dji::MissionWaypointTask> 
    MissionOps::Convert( const dji_sdk::MissionWaypointTask* waypointTask )
  {
    auto wpt = std::make_shared<mav2dji::MissionWaypointTask>();

    wpt->velocity_range     = waypointTask->velocity_range;
    wpt->idle_velocity      = waypointTask->idle_velocity;
    wpt->action_on_finish   = waypointTask->action_on_finish;
    wpt->mission_exec_times = waypointTask->mission_exec_times;
    wpt->yaw_mode           = waypointTask->yaw_mode;
    wpt->trace_mode         = waypointTask->trace_mode;
    wpt->action_on_rc_lost  = waypointTask->action_on_rc_lost;
    wpt->gimbal_pitch_mode  = waypointTask->gimbal_pitch_mode;

    for(auto wPointIn : wpt->mission_waypoint )
    {
        mav2dji::MissionWaypoint wPointOut;

        wPointOut.latitude                      = wPointIn.latitude;
        wPointOut.longitude                     = wPointIn.longitude;
        wPointOut.altitude                      = wPointIn.altitude;
        wPointOut.damping_distance              = wPointIn.damping_distance;
        wPointOut.target_yaw                    = wPointIn.target_yaw;
        wPointOut.target_gimbal_pitch           = wPointIn.target_gimbal_pitch;
        wPointOut.turn_mode                     = wPointIn.turn_mode;
        wPointOut.has_action                    = wPointIn.has_action;
        wPointOut.action_time_limit             = wPointIn.action_time_limit;
        wPointOut.waypoint_action.action_repeat = wPointIn.waypoint_action.action_repeat;

        std::copy(std::begin(wPointIn.waypoint_action.command_list),
                  std::end(wPointIn.waypoint_action.command_list), 
                  wPointOut.waypoint_action.command_list.begin());

        std::copy(std::begin(wPointIn.waypoint_action.command_parameter),
                  std::end(wPointIn.waypoint_action.command_parameter), 
                  wPointOut.waypoint_action.command_parameter.begin());

        wpt->mission_waypoint.push_back(wPointOut);
    }

    return wpt; 
  }

}