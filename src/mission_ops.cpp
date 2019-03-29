#include <mission_ops.hpp>

//#include <dji_sdk/MissionWaypoint.h>
//#include <dji_sdk/MissionWaypointAction.h>
//#include <dji_sdk/MissionWpSetSpeed.h>
//#include <dji_sdk/MissionWpGetSpeed.h>
//#include <dji_sdk/MissionWpGetInfo.h>

namespace mav2dji 
{

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

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
  

bool MissionOps::runWaypointMission(uint8_t numWaypoints, int responseTimeout)
{
  auto vehicleInterface = VehicleInfo::getVehicleInterface();

  double increment = 0.000001 / C_PI * 180;
  float start_alt = 10;

  auto wayPointList = createWaypoints(numWaypoints, increment, start_alt);

  mav2dji::MissionWaypointTask missionWaypointTask(10, 5, 
  MissionWaypointTask::finishActionEnum::FINISH_NO_ACTION, 1,
  MissionWaypointTask::yawModeEnum::YAW_MODE_AUTO,
  MissionWaypointTask::traceModeEnum::TRACE_POINT,
  MissionWaypointTask::rcLostActionEnum::ACTION_AUTO,
  MissionWaypointTask::gimbalPitchModeEnum::GIMBAL_PITCH_FREE,
  wayPointList );

  // Waypoint Mission: Init mission
  //ROS_INFO("Initializing Waypoint Mission..\n");
  /*if (initWaypointMission(waypointTask).result)
  {
    //ROS_INFO("Waypoint upload command sent successfully");
  }
  else
  {
    //ROS_WARN("Failed sending waypoint upload command");
    return false;
  }

  // Waypoint Mission: Start
  if (missionAction(DJI_MISSION_TYPE::WAYPOINT,
                    MISSION_ACTION::START)
        .result)
  {
    //ROS_INFO("Mission start command sent successfully");
  }
  else
  {
    //ROS_WARN("Failed sending mission start command");
    return false;
  }*/

  return true;
}

std::vector<mav2dji::MissionWaypoint>
MissionOps::createWaypoints(int numWaypoints, 
  double distanceIncrement, float altitude)
{
  std::vector<mav2dji::MissionWaypoint> wayPoints;

  double latitude = 0.0;
  double longitude = 0.0;

  mav2dji::MissionWaypointAction missionWaypointAction;

  mav2dji::MissionWaypoint originWayPoint( 
    latitude, longitude, altitude, 0.0, 0.0, 0.0, 
    mav2dji::MissionWaypoint::turnModeEnum::turnModeClockwise, 
    mav2dji::MissionWaypoint::hasActionEnum::hasActionNo, 
    100, missionWaypointAction );

  wayPoints.push_back(originWayPoint);

  // Start at origin
  double extAngle = 2 * M_PI / numWaypoints;

  // waypoints
  for (int i = 1; i < numWaypoints; i++)
  {
    latitude += distanceIncrement * cos(i * extAngle);
    longitude += distanceIncrement * sin(i * extAngle);
    altitude  += 1;

    mav2dji::MissionWaypoint wayPoint( 
      latitude, longitude, altitude, 0.0, 0.0, 0.0, 
      mav2dji::MissionWaypoint::turnModeEnum::turnModeClockwise, 
      mav2dji::MissionWaypoint::hasActionEnum::hasActionNo, 
      100, missionWaypointAction );

    wayPoints.push_back(wayPoint);
  }

  // return to origin
  wayPoints.push_back(originWayPoint);

  return wayPoints;
}

/*void MissionOps::setWaypointDefaults(WayPointSettings* wp)
{
  wp->damping         = 0;
  wp->yaw             = 0;
  wp->gimbalPitch     = 0;
  wp->turnMode        = 0;
  wp->hasAction       = 0;
  wp->actionTimeLimit = 100;
  wp->actionNumber    = 0;
  wp->actionRepeat    = 0;
  for (int i = 0; i < 16; ++i)
  {
    wp->commandList[i]      = 0;
    wp->commandParameter[i] = 0;
  }
}

void MissionOps::setWaypointInitDefaults(dji_sdk::MissionWaypointTask& waypointTask)
{
  waypointTask.velocity_range     = 10;
  waypointTask.idle_velocity      = 5;
  waypointTask.action_on_finish   = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
  waypointTask.mission_exec_times = 1;
  waypointTask.yaw_mode           = dji_sdk::MissionWaypointTask::YAW_MODE_AUTO;
  waypointTask.trace_mode         = dji_sdk::MissionWaypointTask::TRACE_POINT;
  waypointTask.action_on_rc_lost  = dji_sdk::MissionWaypointTask::ACTION_AUTO;
  waypointTask.gimbal_pitch_mode  = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
}

std::vector<DJI::OSDK::WayPointSettings>
MissionOps::createWaypoints(int numWaypoints, double distanceIncrement,
                float start_alt)
{
  // Create Start Waypoint
  WayPointSettings start_wp;
  setWaypointDefaults(&start_wp);
  start_wp.latitude  = gps_pos.latitude;
  start_wp.longitude = gps_pos.longitude;
  start_wp.altitude  = start_alt;
  //ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n", gps_pos.latitude,
  //         gps_pos.longitude, start_alt);

  std::vector<DJI::OSDK::WayPointSettings> wpVector =
    generateWaypointsPolygon(&start_wp, distanceIncrement, numWaypoints);
  return wpVector;
}

std::vector<DJI::OSDK::WayPointSettings>
MissionOps::generateWaypointsPolygon(WayPointSettings* start_data, double increment,
                         int num_wp)
{
  // Let's create a vector to store our waypoints in.
  std::vector<DJI::OSDK::WayPointSettings> wp_list;

  // Some calculation for the polygon
  double extAngle = 2 * M_PI / num_wp;

  // First waypoint
  start_data->index = 0;
  wp_list.push_back(*start_data);

  // Iterative algorithm
  for (int i = 1; i < num_wp; i++)
  {
    WayPointSettings  wp;
    WayPointSettings* prevWp = &wp_list[i - 1];
    setWaypointDefaults(&wp);
    wp.index     = i;
    wp.latitude  = (prevWp->latitude + (increment * cos(i * extAngle)));
    wp.longitude = (prevWp->longitude + (increment * sin(i * extAngle)));
    wp.altitude  = (prevWp->altitude + 1);
    wp_list.push_back(wp);
  }

  // Come back home
  start_data->index = num_wp;
  wp_list.push_back(*start_data);

  return wp_list;
}

void
MissionOps::uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wp_list,
                int responseTimeout, dji_sdk::MissionWaypointTask& waypointTask)
{
  dji_sdk::MissionWaypoint waypoint;
  for (std::vector<WayPointSettings>::iterator wp = wp_list.begin();
       wp != wp_list.end(); ++wp)
  {
    //ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude,
    //         wp->longitude, wp->altitude);
    waypoint.latitude            = wp->latitude;
    waypoint.longitude           = wp->longitude;
    waypoint.altitude            = wp->altitude;
    waypoint.damping_distance    = 0;
    waypoint.target_yaw          = 0;
    waypoint.target_gimbal_pitch = 0;
    waypoint.turn_mode           = 0;
    waypoint.has_action          = 0;
    waypointTask.mission_waypoint.push_back(waypoint);
  }
}*/


}