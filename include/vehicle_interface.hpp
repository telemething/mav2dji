/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

#include <string>
#include <ros/ros.h>
#include <util.hpp>
//#include <vehicle_telemetry.hpp>
//#include <vehicle_Info.hpp>
//#include <mission_ops.hpp>

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

class MissionWaypointAction
{
 public:

  MissionWaypointAction(); 

  typedef uint8_t _action_repeat_type;
  _action_repeat_type action_repeat;

  typedef boost::array<uint8_t, 16> _command_list_type;
  _command_list_type command_list;

  typedef boost::array<uint16_t, 16> _command_parameter_type;
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

class MissionWaypoint
{
  public:

  enum turnModeEnum 
  {
    turnModeClockwise = 0u,
    turnModeCounterClockwise = 1u,
  };

  enum hasActionEnum 
  {
    hasActionNo = 0u,
    hasActionYes = 1u,
  };

  MissionWaypoint();

  MissionWaypoint( double latitude, double longitude, float relativeAltitude, 
    float dampingDistance, int16_t targetYaw, int16_t targetGimbalPitch, 
    turnModeEnum turnMode, hasActionEnum hasAction, uint16_t actionTime, 
    MissionWaypointAction missionWaypointAction );

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

class MissionWaypointTask
{
 public:

  enum finishActionEnum {
    FINISH_NO_ACTION = 0u,
    FINISH_RETURN_TO_HOME = 1u,
    FINISH_AUTO_LANDING = 2u,
    FINISH_RETURN_TO_POINT = 3u,
    FINISH_NO_EXIT = 4u
  };
  
  enum yawModeEnum {
    YAW_MODE_AUTO = 0u,
    YAW_MODE_LOCK = 1u,
    YAW_MODE_RC = 2u,
    YAW_MODE_WAYPOINT = 3u,
  };

  enum traceModeEnum {
    TRACE_POINT = 0u,
    TRACE_COORDINATED = 1u,
  };
  
  enum rcLostActionEnum {
    ACTION_FREE = 0u,
    ACTION_AUTO = 1u,
  };
  
  enum gimbalPitchModeEnum {
    GIMBAL_PITCH_FREE = 0u,
    GIMBAL_PITCH_AUTO = 1u,
  };

  MissionWaypointTask();

  MissionWaypointTask(float velocityRange, float idleVelocity, 
    finishActionEnum actionOnFinish, uint8_t  missionExecTimes, 
    yawModeEnum yawMode, traceModeEnum traceMode, 
    rcLostActionEnum actionOnRcLost, gimbalPitchModeEnum gimbalPitchMode,
    const std::vector<mav2dji::MissionWaypoint> missionWaypointList);

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

   typedef std::vector<mav2dji::MissionWaypoint>  _mission_waypoint_type;
  _mission_waypoint_type mission_waypoint;

  }; // struct MissionWaypointTask

class vehicle_interface 
{
 public:

    enum CameraActionEnum {TakePhoto, StartVideo, StopVideo};
    enum VehicleTaskEnum {Takeoff, Land, GoHome};
    enum ControlAutority {TakeAuthority, ReleaseAuthority};
    enum MissionWpActionEnum 
    { MissionWpActionStart, MissionWpActionStop,
      MissionWpActionPause, MissionWpActionResume };

    struct DroneVersion{ std::string Hardware; uint32_t Version; bool IsValid = false; };

    explicit vehicle_interface();
    ~vehicle_interface();

    std::shared_ptr<ros::NodeHandle> rosNodeHandle;

    virtual Util::OpRet init() = 0;
    virtual Util::OpRet connectToPlatform() = 0;
    virtual Util::OpRet activate() = 0;
    virtual Util::OpRet startVehicleAsync() = 0;
    virtual Util::OpRet stopVehicle() = 0;

    virtual Util::OpRet VehicleTask(const VehicleTaskEnum task) = 0;
    virtual Util::OpRet MFIOConfig() = 0;
    virtual Util::OpRet MFIOSetValue() = 0;
    virtual Util::OpRet MissionHpAction() = 0;
    virtual Util::OpRet MissionHpGetInfo() = 0;
    virtual Util::OpRet MissionHpResetYaw() = 0;
    virtual Util::OpRet MissionHpUpdateRadius() = 0;
    virtual Util::OpRet MissionHpUpdateYawRate() = 0;
    virtual Util::OpRet MissionHpUpload() = 0;
    virtual Util::OpRet MissionWpAction( const MissionWpActionEnum missionWpAction) = 0;
    virtual Util::OpRet MissionWpGetInfo(
        std::shared_ptr<mav2dji::MissionWaypointTask>* waypointTask) = 0;
    virtual float MissionWpGetSpeed() = 0;
    virtual Util::OpRet MissionWpSetSpeed(float speed) = 0;
    virtual Util::OpRet MissionWpUpload(const mav2dji::MissionWaypointTask* waypointTask) = 0;
    virtual Util::OpRet SDKControlAuthority(const ControlAutority authority) = 0;
    virtual Util::OpRet SendMobileData() = 0;
    virtual Util::OpRet SetHardSync() = 0;
    virtual DroneVersion QueryDroneVersion() = 0;
    virtual Util::OpRet SetLocalPosRef() = 0;
    virtual Util::OpRet Stereo240pSubscription() = 0;
    virtual Util::OpRet StereoDepthSubscription() = 0;
    virtual Util::OpRet StereoVGASubscription() = 0;
    virtual Util::OpRet SetupCameraStream() = 0;   

};

} /* namespace mav2dji*/