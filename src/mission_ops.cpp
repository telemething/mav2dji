#include <mission_ops.hpp>

namespace mav2dji 
{

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

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

  auto ret = vehicleInterface->MissionWpUpload(&missionWaypointTask);

  ret = vehicleInterface->MissionWpAction(
    vehicle_interface::MissionWpActionEnum::MissionWpActionStart);

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

/*


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

}*/


}