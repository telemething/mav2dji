/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

#include <dji_sdk/MissionWaypoint.h>
#include <dji_sdk/MissionWaypointAction.h>
#include <dji_sdk/MissionWpSetSpeed.h>
#include <dji_sdk/MissionWpGetSpeed.h>
#include <dji_sdk/MissionWpGetInfo.h>

namespace mav2dji
{

//*****************************************************************************
//*
//* # action_repeat
//* # lower 4 bit: Total number of actions
//* # hight 4 bit: Total running times
//* uint8 action_repeat
//* uint8[16] command_list
//* uint16[16] command_parameter
//* 
//***************************************************************************** 

struct MissionWaypointAction
{
  MissionWaypointAction() 
    : action_repeat(0)
    , command_list()
    , command_parameter()  
  {
    command_list.assign(0);
    command_parameter.assign(0);
  }

  typedef uint8_t _action_repeat_type;
  _action_repeat_type action_repeat;

  typedef boost::array<uint8_t, 16>  _command_list_type;
  _command_list_type command_list;

  typedef boost::array<uint16_t, 16>  _command_parameter_type;
  _command_parameter_type command_parameter;
}; // struct MissionWaypointAction

//*****************************************************************************
//*
//* float64 latitude          # degree
//* float64 longitude         # degree
//* float32 altitude          # relative altitude from takeoff point
//* float32 damping_distance  # Bend length (effective coordinated turn mode only)
//* int16 target_yaw          # Yaw (degree)
//* int16 target_gimbal_pitch # Gimbal pitch
//* uint8 turn_mode           # 0: clockwise, 1: counter-clockwise
//* uint8 has_action          # 0: no, 1: yes
//* uint16 action_time_limit
//* MissionWaypointAction waypoint_action
//* 
//***************************************************************************** 

struct MissionWaypoint
{
  MissionWaypoint()
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

  typedef double _latitude_type;
  _latitude_type latitude;

  typedef double _longitude_type;
  _longitude_type longitude;

  typedef float _altitude_type;
  _altitude_type altitude;

  typedef float _damping_distance_type;
  _damping_distance_type damping_distance;

  typedef int16_t _target_yaw_type;
  _target_yaw_type target_yaw;

  typedef int16_t _target_gimbal_pitch_type;
  _target_gimbal_pitch_type target_gimbal_pitch;

  typedef uint8_t _turn_mode_type;
  _turn_mode_type turn_mode;

  typedef uint8_t _has_action_type;
  _has_action_type has_action;

  typedef uint16_t _action_time_limit_type;
  _action_time_limit_type action_time_limit;

  typedef mav2dji::MissionWaypointAction _waypoint_action_type;
  _waypoint_action_type waypoint_action;
}; // struct MissionWaypoint

//*****************************************************************************
//*
//* return "# constant for action_on_finish
//* uint8 FINISH_NO_ACTION       = 0  # no action
//* uint8 FINISH_RETURN_TO_HOME  = 1  # return to home
//* uint8 FINISH_AUTO_LANDING    = 2  # auto landing
//* uint8 FINISH_RETURN_TO_POINT = 3  # return to point 0
//* uint8 FINISH_NO_EXIT         = 4  # infinite mode， no exit
//* 
//* # constant for yaw_mode
//* uint8 YAW_MODE_AUTO     = 0       # auto mode (point to next waypoint)
//* uint8 YAW_MODE_LOCK     = 1       # lock as an initial value
//* uint8 YAW_MODE_RC       = 2       # controlled by RC
//* uint8 YAW_MODE_WAYPOINT = 3       # use waypoint's yaw(tgt_yaw)
//* 
//* # constant for trace_mode
//* uint8 TRACE_POINT       = 0       # point to point, after reaching the target 
//*                                     waypoint hover, complete waypt action (if any), 
//*                                     then fly to the next waypt
//* uint8 TRACE_COORDINATED = 1       # 1: Coordinated turn mode, smooth transition 
//*                                     between waypts, no waypts task
//* 
//* # constants for action_on_rc_lost
//* uint8 ACTION_FREE       = 0       # exit waypoint and failsafe
//* uint8 ACTION_AUTO       = 1       # continue the waypoint
//* 
//* # constants for gimbal_pitch_mode
//* uint8 GIMBAL_PITCH_FREE = 0       # free mode, no control on gimbal
//* uint8 GIMBAL_PITCH_AUTO = 1       # auto mode, Smooth transition between waypoints on gimbal
//* 
//* float32 velocity_range    # Maximum speed joystick input(2~15m)
//* float32 idle_velocity     # Cruising Speed (without joystick input, no more than vel_cmd_range)
//* uint8 action_on_finish    # See constants above for possible actions
//* uint8 mission_exec_times  # 1: once ; 2: twice
//* uint8 yaw_mode            # see constants above for possible yaw modes
//* uint8 trace_mode          # see constants above for possible trace modes
//* uint8 action_on_rc_lost   # see constants above for possible actions
//* uint8 gimbal_pitch_mode   # see constants above for pissible gimbal modes
//* MissionWaypoint[] mission_waypoint  # a vector of waypoints
//* 
//***************************************************************************** 

struct MissionWaypointTask
{
  MissionWaypointTask()
    : velocity_range(0.0)
    , idle_velocity(0.0)
    , action_on_finish(0)
    , mission_exec_times(0)
    , yaw_mode(0)
    , trace_mode(0)
    , action_on_rc_lost(0)
    , gimbal_pitch_mode(0)
    , mission_waypoint() {}

   typedef float _velocity_range_type;
  _velocity_range_type velocity_range;

   typedef float _idle_velocity_type;
  _idle_velocity_type idle_velocity;

   typedef uint8_t _action_on_finish_type;
  _action_on_finish_type action_on_finish;

   typedef uint8_t _mission_exec_times_type;
  _mission_exec_times_type mission_exec_times;

   typedef uint8_t _yaw_mode_type;
  _yaw_mode_type yaw_mode;

   typedef uint8_t _trace_mode_type;
  _trace_mode_type trace_mode;

   typedef uint8_t _action_on_rc_lost_type;
  _action_on_rc_lost_type action_on_rc_lost;

   typedef uint8_t _gimbal_pitch_mode_type;
  _gimbal_pitch_mode_type gimbal_pitch_mode;

   //typedef std::vector<dji_sdk::MissionWaypoint>  _mission_waypoint_typey;
   typedef std::vector<mav2dji::MissionWaypoint>  _mission_waypoint_type;
  _mission_waypoint_type mission_waypoint;

  enum {
    FINISH_NO_ACTION = 0u,
    FINISH_RETURN_TO_HOME = 1u,
    FINISH_AUTO_LANDING = 2u,
    FINISH_RETURN_TO_POINT = 3u,
    FINISH_NO_EXIT = 4u,
    YAW_MODE_AUTO = 0u,
    YAW_MODE_LOCK = 1u,
    YAW_MODE_RC = 2u,
    YAW_MODE_WAYPOINT = 3u,
    TRACE_POINT = 0u,
    TRACE_COORDINATED = 1u,
    ACTION_FREE = 0u,
    ACTION_AUTO = 1u,
    GIMBAL_PITCH_FREE = 0u,
    GIMBAL_PITCH_AUTO = 1u,
  };
}; // struct MissionWaypointTask

class MissionOps
{
 public:

    static std::shared_ptr<dji_sdk::MissionWaypointTask> 
        Convert(const mav2dji::MissionWaypointTask* waypointTask );
    static std::shared_ptr<mav2dji::MissionWaypointTask> 
        Convert( const dji_sdk::MissionWaypointTask* waypointTask );

};



}