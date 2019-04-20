/*
 * mav2dji_ros.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <vehicle_interface.hpp>

//*****************************************************************************
//*
//*
//*
//******************************************************************************

namespace mav2dji 
{

MissionWaypointAction::MissionWaypointAction() 
    : action_repeat(0)
    , command_list()
    , command_parameter()  
  {
    command_list.assign(0);
    command_parameter.assign(0);
  }

//*****************************************************************************
//*
//*
//*
//******************************************************************************

MissionWaypoint::MissionWaypoint()
    : latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , damping_distance(0.0)
    , target_yaw(0)
    , target_gimbal_pitch(0)
    , turn_mode(0)
    , has_action(0)
    , action_time_limit(0)
    , waypoint_action()
    {}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

MissionWaypoint::MissionWaypoint( double latitude, double longitude, float relativeAltitude, 
    float dampingDistance, int16_t targetYaw, int16_t targetGimbalPitch, 
    turnModeEnum turnMode, hasActionEnum hasAction, uint16_t actionTime, 
    MissionWaypointAction missionWaypointAction )
    : latitude(latitude)
    , longitude(longitude)
    , altitude(relativeAltitude)
    , damping_distance(dampingDistance)
    , target_yaw(targetYaw)
    , target_gimbal_pitch(targetGimbalPitch)
    , turn_mode(turnMode)
    , has_action(hasAction)
    , action_time_limit(actionTime)
    , waypoint_action(missionWaypointAction){}
  
//*****************************************************************************
//*
//*
//*
//******************************************************************************

MissionWaypointTask::MissionWaypointTask()
    : velocity_range(0.0)
    , idle_velocity(0.0)
    , action_on_finish(0)
    , mission_exec_times(0)
    , yaw_mode(0)
    , trace_mode(0)
    , action_on_rc_lost(0)
    , gimbal_pitch_mode(0)
    , mission_waypoint() {}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

MissionWaypointTask::MissionWaypointTask(float velocityRange, float idleVelocity, 
    finishActionEnum actionOnFinish, uint8_t  missionExecTimes, 
    yawModeEnum yawMode, traceModeEnum traceMode, 
    rcLostActionEnum actionOnRcLost, gimbalPitchModeEnum gimbalPitchMode,
    const std::vector<mav2dji::MissionWaypoint> missionWaypointList)
    : velocity_range(velocityRange)
    , idle_velocity(idleVelocity)
    , action_on_finish(actionOnFinish)
    , mission_exec_times(missionExecTimes)
    , yaw_mode(yawMode)
    , trace_mode(traceMode)
    , action_on_rc_lost(actionOnRcLost)
    , gimbal_pitch_mode(gimbalPitchMode)
    , mission_waypoint(missionWaypointList) {}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

vehicle_interface::vehicle_interface() 
{
}

vehicle_interface::vehicle_interface(
    std::shared_ptr<iVehicleTelemetry> vehicleTelemetry) : 
        vehicleTelemetry(vehicleTelemetry)
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

vehicle_interface::~vehicle_interface()
{
}

Util::OpRet vehicle_interface::setOffboardControlAllowed(bool value)
{
    
}

}