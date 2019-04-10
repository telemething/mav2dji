#pragma once 

/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <dji_sdk/MissionWaypoint.h>
#include <dji_sdk/MissionWaypointAction.h>
#include <dji_sdk/MissionWpSetSpeed.h>
#include <dji_sdk/MissionWpGetSpeed.h>
#include <dji_sdk/MissionWpGetInfo.h>

#include <vehicle_interface.hpp>
#include <vehicle_Info.hpp>

namespace mav2dji
{

class MissionOps
{
 public:

    MissionOps();
    ~MissionOps();

    static std::shared_ptr<dji_sdk::MissionWaypointTask> 
        Convert(const mav2dji::MissionWaypointTask* waypointTask );
    static std::shared_ptr<mav2dji::MissionWaypointTask> 
        Convert( const dji_sdk::MissionWaypointTask* waypointTask );

    //*** temp area ***

    bool runWaypointMission(uint8_t numWaypoints, int responseTimeout);

    std::vector<mav2dji::MissionWaypoint> createWaypoints(
      int numWaypoints, double distanceIncrement, float start_alt);

    /*void setWaypointDefaults(WayPointSettings* wp);

    void setWaypointInitDefaults(dji_sdk::MissionWaypointTask& waypointTask);

    std::vector<DJI::OSDK::WayPointSettings>
    createWaypoints(int numWaypoints, double distanceIncrement,
                    float start_alt);

    std::vector<DJI::OSDK::WayPointSettings>
    generateWaypointsPolygon(WayPointSettings* start_data, double increment,
                            int num_wp);

    void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wp_list,
                    int responseTimeout, dji_sdk::MissionWaypointTask& waypointTask);*/
};

}