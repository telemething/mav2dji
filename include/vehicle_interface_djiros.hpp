/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

#include <vehicle_interface.hpp>
#include <vehicle_Info.hpp>
#include <util.hpp>
#include <thread>

// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/CameraAction.h>
#include <dji_sdk/Gimbal.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/MissionWpUpload.h>

#include <dji_sdk/MissionWaypoint.h>
#include <dji_sdk/MissionWaypointAction.h>
#include <dji_sdk/MissionWpSetSpeed.h>
#include <dji_sdk/MissionWpGetSpeed.h>
#include <dji_sdk/MissionWpGetInfo.h>



namespace mav2dji
{

//******************************************************

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



//******************************************************

class vehicle_interface_djiros : public vehicle_interface
{
 private:

  int DjiActivationSleepMs = 1000;
  int DjiActivationTimeoutMs = 10000;

  std::thread vehicleRunWorkerThread;

  void testCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

  //std::shared_ptr<ros::NodeHandle> rosNodeHandle;
  ros::ServiceClient droneActivationService;
  ros::ServiceClient armVehicleService;
  ros::ServiceClient vehicleTaskService;
  ros::ServiceClient missionWpSetSpeedService;
  ros::ServiceClient missionWpGetSpeedService;
  ros::ServiceClient missionWpUploadService;
  ros::ServiceClient sDKControlAuthorityService;
  ros::ServiceClient queryDroneVersionService;
  ros::ServiceClient setLocalPosRefService;

  void vehicleRunWorker();

  ros::ServiceClient cameraActionService;

 public:

  enum CameraActionEnum {TakePhoto, StartVideo, StopVideo};
  enum VehicleTaskEnum {Takeoff, Land, GoHome};
  enum ControlAutority {TakeAuthority, ReleaseAuthority};
  struct DroneVersion{ std::string Hardware; uint32_t Version; bool IsValid = false; };

  explicit vehicle_interface_djiros();
  ~vehicle_interface_djiros();

  int init();
  vehicle_interface_ret connectToPlatform();
  vehicle_interface_ret activate();
  vehicle_interface_ret startVehicleAsync();
  vehicle_interface_ret stopVehicle();

  //***************************************************************************
  //*
  //* Convert
  //*
  //***************************************************************************

  std::shared_ptr<dji_sdk::MissionWaypointTask> 
    Convert( mav2dji::MissionWaypointTask waypointTask )
  {
    auto wpt = std::make_shared<dji_sdk::MissionWaypointTask>();

    return wpt; 
  }

  //***************************************************************************
  //*
  //* Convert
  //*
  //***************************************************************************

  std::shared_ptr<mav2dji::MissionWaypointTask> 
    Convert( dji_sdk::MissionWaypointTask waypointTask )
  {
    auto wpt = std::make_shared<mav2dji::MissionWaypointTask>();

    return wpt; 
  }

  //***************************************************************************
  //*
  //* /dji_sdk/activation (dji_sdk/Activation)
  //*
  //* The service to activate the drone with app ID and key pair. The 
  //* activation arguments should be specified in launch files. 
  //*
  //* Usage:
  //*   Response 	
  //*   bool result 	true--succeed 	false--invalid action
  //*
  //***************************************************************************

  Util::OpRet Activation()
  {
    return Util::OpRet::BuildError( 
      "Activation not implemented", true, true);
  }

  //***************************************************************************
  //*
  //* /dji_sdk/camera_action (dji_sdk/CameraAction)
  //*
  //*  Take photo or video via service, return true if successful. 
  //*
  //*  Usage:
  //*     Request 	
  //*     Uint8 Camera_action 0--Shoot Photo 1--Start video taking 2--Stop video taking
  //*      Response 	123
  //*      bool result 	true--succeed 	false--invalid action
  //*
  //***************************************************************************

  Util::OpRet CameraAction(CameraActionEnum action)
  {
    try
    {
        dji_sdk::CameraAction camAction;

        switch(action)
        {
          case TakePhoto: 
            camAction.request.camera_action = 
              dji_sdk::CameraAction::RequestType::CAMERA_ACTION_TAKE_PICTURE; 
            break;
          case StartVideo: 
            camAction.request.camera_action = 
              dji_sdk::CameraAction::RequestType::CAMERA_ACTION_START_RECORD; 
            break;
          case StopVideo: 
            camAction.request.camera_action = 
              dji_sdk::CameraAction::RequestType::CAMERA_ACTION_STOP_RECORD; 
            break;
        }

        cameraActionService.call(camAction);

        if(!camAction.response.result) 
          return Util::OpRet::BuildError(true, true, 
            "Could not perform camera action");

    }
    catch(const std::exception& e)
    {
      return Util::OpRet::UnwindStdException(e,
        "Could not perform camera action", true, true);
    }
    catch(...)
    {
        return Util::OpRet::BuildError(
          "Could not perform camera action. Unrecognized Exception", true, true);
    }

    ROS_INFO_STREAM("Camera action performed OK");
    return Util::OpRet();
  }

  //***************************************************************************
  //*
  //* /dji_sdk/drone_arm_control (dji_sdk/DroneArmControl)
  //*
  //*  Enable or disable vehicle's arm motors. 
  //*
  //*  Usage:
  //*      Request 	
  //*      uint8 arm 	1--enable vehicle arm motor 	else: disable arm motor
  //*      Response 	
  //*      bool result 	true--succeed 	false--invalid action
  //*
  //***************************************************************************

  Util::OpRet ArmVehicle(bool arm)
  {
    try
    {
        dji_sdk::DroneArmControl armControl;

        arm ? armControl.request.arm = armControl.request.ARM_COMMAND 
            : armControl.request.arm = armControl.request.DISARM_COMMAND;

        armVehicleService.call(armControl);

        if(!armControl.response.result) 
          return Util::OpRet::BuildError(true, true, 
            "Could not arm vehicle : ack.info: set = %i id = %i data=%i", 
            armControl.response.cmd_set, 
            armControl.response.cmd_id, 
            armControl.response.ack_data);
    }
    catch(const std::exception& e)
    {
      return Util::OpRet::UnwindStdException(e,
        "Could not arm vehicle", true, true);
    }
    catch(...)
    {
        return Util::OpRet::BuildError(
          "Could not arm vehicle. Unrecognized Exception", true, true);
    }

    ROS_INFO_STREAM("Vehicle Armed OK");
    return Util::OpRet();
  }

  //***************************************************************************
  //*
  //* /dji_sdk/drone_task_control (dji_sdk/DroneTaskControl)
  //*
  //*  Execute takeoff, landing or go home. 
  //*
  //*    Usage:
  //*        Request 	
  //*       uint8 task 	4--takeoff 	6--landing 	1--gohome
  //*        Response 	
  //*        bool result 	true--succeed 	false--failed
  //*
  //***************************************************************************

  Util::OpRet VehicleTask(VehicleTaskEnum task)
  {
    try
    {
        dji_sdk::DroneTaskControl vehTask;

        switch(task)
        {
          case Takeoff: 
            vehTask.request.task = dji_sdk::DroneTaskControl::RequestType::TASK_TAKEOFF; 
            break;
          case Land: 
            vehTask.request.task = dji_sdk::DroneTaskControl::RequestType::TASK_LAND; 
            break;
          case GoHome: 
            vehTask.request.task = dji_sdk::DroneTaskControl::RequestType::TASK_GOHOME; 
            break;
        }

        vehicleTaskService.call(vehTask);

        if(!vehTask.response.result) 
          return Util::OpRet::BuildError(true, true, 
            "Vehicle task could not be performed");
    }
    catch(const std::exception& e)
    {
      return Util::OpRet::UnwindStdException(e,
        "Vehicle task could not be performed", true, true);
    }
    catch(...)
    {
        return Util::OpRet::BuildError(
          "Vehicle task could not be performed. Unrecognized Exception", true, true);
    }

    ROS_INFO_STREAM("Vehicle task erformed OK");
    return Util::OpRet();
  }

 //***************************************************************************
  //*
  //* /dji_sdk/mfio_config (dji_sdk/MFIOConfig)
  //*
  //*    Config Multi-function IO. This service is unavailable on M100 
  //*
  //*    Usage:
  //*        Request 	
  //*        uint8 mode 0--PWM_OUT 1--PWM_IN 2--GPIO_OUT 3--GPIO_IN 4--ADC
  //*        uint8 channel 	0-7
  //*        uint32 init_on_time_us
  //*        uint16 pwm_freq
  //*
  //***************************************************************************

  Util::OpRet MFIOConfig()
  {
    return Util::OpRet::BuildError( 
      "MFIOConfig not implemented", true, true);
  }

  //***************************************************************************
  //*
  //* /dji_sdk/mfio_set_value (dji_sdk/MFIOSetValue)
  //*
  //*    Set MFIO value. This service is unavailable on M100 
  //*
  //*    Usage:
  //*        Request 	
  //*        uint8 channel 	0-7
  //*        uint32 init_on_time_us
  //*
  //***************************************************************************

  Util::OpRet MFIOSetValue()
  {
    return Util::OpRet::BuildError( 
      "MFIOSetValue not implemented", true, true);
  }

  //***************************************************************************
  //*
  //* /dji_sdk/mission_hotpoint_action (dji_sdk/MissionHpAction)
  //*
  //*    Service that start/stop/pause/resume the hotpoint mission. 
  //*
  //*    Usage:
  //*        Request 	
  //*        uint8 action 	0--start 	1--stop 	2--pause 	3--resume
  //*        Response 	
  //*        bool result 	true--succeed 	false--failed
  //*
  //***************************************************************************

  Util::OpRet MissionHpAction()
  {
    return Util::OpRet::BuildError( 
      "MissionHpAction not implemented", true, true);
  }

  //***************************************************************************
  //*
  //* /dji_sdk/mission_hotpoint_getInfo (dji_sdk/MissionHpGetInfo)
  //*
  //*   Return the hotpoint tasks info. Use rosmsg show 
  //*   dji_sdk/MissionHotpointTask for more detail 
  //*
  //*    Usage:
  //*        Response
  //*        MissionHotpointTask hotpoint_task
  //*
  //***************************************************************************

  Util::OpRet xxx()
  {
    return Util::OpRet::BuildError( 
      "xxx not implemented", true, true);
  }

  //***************************************************************************
  //*
  //* /dji_sdk/mission_hotpoint_resetYaw (dji_sdk/MissionHpResetYaw)
  //*
  //*    Resets the Yaw position of the vehicle 
  //*
  //*    Usage:
  //*        Response 	
  //*        bool result 	true--succeed 	false--failed
  //*
  //***************************************************************************

  Util::OpRet MissionHpGetInfo()
  {
    return Util::OpRet::BuildError( 
      "MissionHpGetInfo not implemented", true, true);
  }

  //***************************************************************************
  //*
  //* /dji_sdk/mission_hotpoint_updateRadius (dji_sdk/MissionHpUpdateRadius)
  //*
  //*    Update the radius of the hot point mission 
  //*
  //*    Usage:
  //*        Request
  //*       float32 radius
  //*        Response 	
  //*       bool result 	true--succeed 	false--failed
  //*
  //***************************************************************************

  Util::OpRet MissionHpUpdateRadius()
  {
    return Util::OpRet::BuildError( 
      "MissionHpUpdateRadius not implemented", true, true);
  }

  //***************************************************************************
  //*
  //* /dji_sdk/mission_hotpoint_updateYawRate (dji_sdk/MissionHpUpdateYawRate)
  //*
  //*    Update the rate of change for Yaw and the direction of the change. 
  //*
  //*    Usage:
  //*        Request
  //*        float32 yaw_rate
  //*       uint8 direction
  //*       Response 	
  //*        bool result 	true--succeed 	false--failed
  //*
  //***************************************************************************

  Util::OpRet MissionHpUpdateYawRate()
  {
    return Util::OpRet::BuildError( 
      "MissionHpUpdateYawRate not implemented", true, true);
  }

  //***************************************************************************
  //*
  //* /dji_sdk/mission_hotpoint_upload (dji_sdk/MissionHpUpload)
  //*
  //*   Upload a set of hotpoint tasks to the vehicle. Use rosmsg show 
  //*   dji_sdk/MissionHotpointTask for more detail 
  //*
  //*    Usage:
  //*        Request
  //*        MissionHotpointTask hotpoint_task
  //*        Response 	
  //*        bool result 	true--succeed 	false--failed
  //*
  //***************************************************************************

  Util::OpRet MissionHpUpload()
  {
    return Util::OpRet::BuildError( 
      "MissionHpUpload not implemented", true, true);
  }

  //***************************************************************************
  //*
  //* /dji_sdk/mission_waypoint_action (dji_sdk/MissionWpAction)
  //*
  //*    Start/stop/pause/resume waypoint action. 
  //*
  //*    Usage:
  //*        Request 	
  //*        uint8 action 	0--start 	1--stop 	2--pause 	3--resume
  //*        Response 	
  //*        bool result 	true--succeed 	false--failed
  //*
  //***************************************************************************

  Util::OpRet MissionWpAction()
  {
    return Util::OpRet::BuildError( 
      "MissionWpAction not implemented", true, true);
  }

  //***************************************************************************
  //*
  //* v/dji_sdk/mission_waypoint_getInfo (dji_sdk/MissionWpGetInfo)
  //*
  //*   Get the current waypoint tasks. Use rosmsg show 
  //*   dji_sdk/MissionWaypointTask for more detail 
  //*
  //*   Usage:
  //*        Response
  //*        MissionWaypointTask waypoint_task
  //*
  //***************************************************************************

  Util::OpRet MissionWpGetInfo(
    std::shared_ptr<mav2dji::MissionWaypointTask>* waypointTask)
  {
    try
    {
        dji_sdk::MissionWpGetInfo missionWpGetInfo;

        missionWpSetSpeedService.call(missionWpGetInfo);

        *waypointTask = Convert(missionWpGetInfo.response.waypoint_task);
    }
    catch(const std::exception& e)
    {
      return Util::OpRet::UnwindStdException(e,
        "Could not set mission speed", true, true);
    }
    catch(...)
    {
        return Util::OpRet::BuildError(
          "Could not set mission speed. Unrecognized Exception", true, true);
    }

    ROS_INFO_STREAM("Set mission speed OK");
    return Util::OpRet();
  }

  //***************************************************************************
  //*
  //* /dji_sdk/mission_waypoint_getSpeed (dji_sdk/MissionWpGetSpeed)
  //*
  //*    Return the waypoint velocity 
  //*
  //*    Usage:
  //*        Response
  //*        float32 speed
  //*
  //***************************************************************************

  float MissionWpGetSpeed()
  {
    try
    {
        dji_sdk::MissionWpGetSpeed missionWpGetSpeed;

        missionWpGetSpeedService.call(missionWpGetSpeed);

        //*** TODO * Can we validate the result?

        return missionWpGetSpeed.response.speed;
    }
    catch(const std::exception& e)
    {
      Util::OpRet::UnwindStdException(e,
        "Could not set mission speed", true, true);
    }
    catch(...)
    {
      Util::OpRet::BuildError(
        "Could not set mission speed. Unrecognized Exception", true, true);
    }

    ROS_INFO_STREAM("Set mission speed OK");
    return -1; //*** TODO * we can do better
  }

  //***************************************************************************
  //*
  //* /dji_sdk/mission_waypoint_setSpeed (dji_sdk/MissionWpSetSpeed)
  //*
  //*    Set the waypoint velocity. 
  //*
  //*   Usage:
  //*       Request
  //*       float32 speed
  //*       Response 	
  //*       bool result 	true--succeed 	false--failed
  //*
  //***************************************************************************

  Util::OpRet MissionWpGetSpeed(float speed)
  {
    try
    {
        dji_sdk::MissionWpSetSpeed missionWpSetSpeed;

        ros::ServiceClient missionWpSetSpeedService  = 
          rosNodeHandle->serviceClient<dji_sdk::MissionWpGetSpeed>
            ("/dji_sdk/mission_waypoint_setSpeed");

        missionWpSetSpeed.request.speed = speed;

        missionWpSetSpeedService.call(missionWpSetSpeed);

        if(!missionWpSetSpeed.response.result) 
          return Util::OpRet::BuildError(true, true, 
            "Could not set mission speed");
    }
    catch(const std::exception& e)
    {
      return Util::OpRet::UnwindStdException(e,
        "Could not set mission speed", true, true);
    }
    catch(...)
    {
        return Util::OpRet::BuildError(
          "Could not set mission speed. Unrecognized Exception", true, true);
    }

    ROS_INFO_STREAM("Set mission speed OK");
    return Util::OpRet();
  }

  //***************************************************************************
  //*
  //* /dji_sdk/mission_waypoint_upload (dji_sdk/MissionWpUpload)
  //*
  //*   Upload a new waypoint task, return true if succeed. Use rosmsg show 
  //*   dji_sdk/MissionWaypointTask for more detail 
  //*
  //*    Usage:
  //*        Request
  //*        MissionWaypointTask waypoint_task
  //*        Response 	
  //*        bool result 	true--succeed 	false--failed
  //*
  //***************************************************************************

  Util::OpRet MissionWpUpload(mav2dji::MissionWaypointTask waypointTask)
  {
    try
    {
        dji_sdk::MissionWpUpload missionWaypoint;

        auto wpt = Convert(waypointTask);

        missionWaypoint.request.waypoint_task = *wpt;

        missionWpUploadService.call(missionWaypoint);

        if(!missionWaypoint.response.result) 
          return Util::OpRet::BuildError(true, true, 
            "Could not upload mission waypoint : ack.info: set = %i id = %i data=%i", 
            missionWaypoint.response.cmd_set, 
            missionWaypoint.response.cmd_id, 
            missionWaypoint.response.ack_data);
    }
    catch(const std::exception& e)
    {
      return Util::OpRet::UnwindStdException(e,
      "Could not upload mission waypoint", true, true);
    }
    catch(...)
    {
        return Util::OpRet::BuildError(
          "Could not upload mission waypoint. Unrecognized Exception", true, true);
    }

    ROS_INFO_STREAM("Upload mission waypoint OK");
    return Util::OpRet();
  }

  //***************************************************************************
  //*
  //* /dji_sdk/sdk_control_authority (dji_sdk/SDKControlAuthority)
  //*
  //*    request/release the control authority 
  //*
  //*    Usage:
  //*        Request 	
  //*        uint8 control_enable 	1--request control 	0--release control
  //*        Response 	
  //*        bool result 	true--succeed 	false--failed
  //*
  //***************************************************************************

  Util::OpRet SDKControlAuthority(ControlAutority authority)
  {
    try
    {
        dji_sdk::SDKControlAuthority controlAuthority;

        switch(authority)
        {
          case TakeAuthority: 
            controlAuthority.request.control_enable = 
              dji_sdk::SDKControlAuthority::RequestType::REQUEST_CONTROL; 
            break;
          case ReleaseAuthority: 
            controlAuthority.request.control_enable = 
              dji_sdk::SDKControlAuthority::RequestType::RELEASE_CONTROL; 
            break;
        }

        sDKControlAuthorityService.call(controlAuthority);

        if(!controlAuthority.response.result) 
          return Util::OpRet::BuildError(true, true, 
            "Could not take/release authority : ack.info: set = %i id = %i data=%i", 
            controlAuthority.response.cmd_set, 
            controlAuthority.response.cmd_id, 
            controlAuthority.response.ack_data);
    }
    catch(const std::exception& e)
    {
      return Util::OpRet::UnwindStdException(e,
        "Could not take/release authority", true, true);
    }
    catch(...)
    {
        return Util::OpRet::BuildError(
          "Could not take/release authority. Unrecognized Exception", true, true);
    }

    ROS_INFO_STREAM("Take/release authority OK");
    return Util::OpRet();
  }

  //***************************************************************************
  //*
  //* /dji_sdk/send_data_to_mobile (dji_sdk/SendMobileData)
  //*
  //*   Send data to the mobile side. The length of the data is upper-limited 
  //*   to 100. 
  //*
  //*    Usage:
  //*        Request 	
  //*        uint8[] data 	length(data) <= 100
  //*      Response 	
  //*      bool result 	true--succeed 	false--failed
  //*
  //***************************************************************************

  Util::OpRet SendMobileData()
  {
    return Util::OpRet::BuildError( 
      "SendMobileData not implemented", true, true);
  }

  //***************************************************************************
  //*
  //* /dji_sdk/set_hardsyc (dji_sdk/SetHardSync)
  //*
  //*   Set Hard Sync. This service is unavailable on M100 
  //*
  //*   Usage:
  //*       Request 	
  //*      uint32 frequency 	frequency in Hz
  //*      uint16 tag 	the tag is to distinguish between different call
  //*      Response 	
  //*      bool result 	true--succeed 	false--failed
  //*
  //***************************************************************************

  Util::OpRet SetHardSync()
  {
    return Util::OpRet::BuildError( 
      "SetHardSync not implemented", true, true);
  }

  //***************************************************************************
  //*
  //* /dji_sdk/query_drone_version (dji_sdk/QueryDroneVersion)
  //*
  //*   Query drone firmware version. Available version list can be found in 
  //*   dji_sdk.h 
  //*
  //***************************************************************************

  DroneVersion QueryDroneVersion()
  {
    DroneVersion droneVersion;

    try
    {
        dji_sdk::QueryDroneVersion queryDroneVersion;

        queryDroneVersionService.call(queryDroneVersion);

        droneVersion.Hardware = queryDroneVersion.response.hardware;
        droneVersion.Version = queryDroneVersion.response.version;

        droneVersion.IsValid = true;
        return droneVersion;
    }
    catch(const std::exception& e)
    {
      Util::OpRet::UnwindStdException(e,
        "Unable to query drone version", true, true);
    }
    catch(...)
    {
      Util::OpRet::BuildError(
        "Unable to query drone version. Unrecognized Exception", true, true);
    }

    ROS_INFO_STREAM("Query drone version OK");
    return droneVersion;
  }


  //***************************************************************************
  //*
  //* /dji_sdk/set_local_pos_ref (dji_sdk/SetLocalPosRef)
  //*
  //*   Set the origin of the local position to be the current GPS coordinate. 
  //*   Fail if GPS health is low (<=3). 
  //*
  //***************************************************************************

  Util::OpRet SetLocalPosRef()
  {
    try
    {
        dji_sdk::SetLocalPosRef localPosRef;

        setLocalPosRefService.call(localPosRef);

        if(!localPosRef.response.result) 
          return Util::OpRet::BuildError( 
            "Could not set local position reference to current GPS coords", true, true);
    }
    catch(const std::exception& e)
    {
      return Util::OpRet::UnwindStdException(e,
        "Could not set local position reference to current GPS coords", true, true);
    }
    catch(...)
    {
        return Util::OpRet::BuildError(
          "Could not set local position reference to current GPS coords. Unrecognized Exception", true, true);
    }

    ROS_INFO_STREAM("Set local position reference to current GPS coords OK");
    return Util::OpRet();
  }

  //***************************************************************************
  //*
  //* /dji_sdk/stereo_240p_subscription (dji_sdk/Stereo240pSubscription)
  //*
  //*   subscribe to stereo images from the front-facing and down-facing 
  //*   cameras of M210 in 240x320 resolution. If unsubscribe_240p is 1, 
  //*   service will unsubscribe no matter what. This service is only available 
  //*   on M210. 
  //*
  //*    Usage:
  //*        Request 	
  //*        uint8 front_right_240p 	1--subscribe 	0--no behavior
  //*        uint8 front_left_240p 	1--subscribe 	0--no behavior
  //*        uint8 down_front_240p 	1--subscribe 	0--no behavior
  //*        uint8 down_back_240p 	1--subscribe 	0--no behavior
  //*        uint8 unsubscribe_240p 	1--unsubscribe 	0--no behavior
  //*        Response 	
  //*        bool result 	true--succeed 	false--failed
  //*
  //***************************************************************************

  Util::OpRet Stereo240pSubscription()
  {
    return Util::OpRet::BuildError( 
      "Stereo240pSubscription not implemented", true, true);
  }

  //***************************************************************************
  //*
  //* /dji_sdk/stereo_depth_subscription (dji_sdk/StereoDepthSubscription)
  //*
  //*   subscribe to stereo disparity map from the front-facing camera of M210 
  //*   in 240x320 resolution. If unsubscribe_240p is 1, service will unsubscribe 
  //*   no matter what. This service is only available on M210. 
  //*
  //*    Usage:
  //*        Request 	
  //*        uint8 front_depth_240p 	1--subscribe 	0--no behavior
  //*        uint8 unsubscribe_240p 	1--unsubscribe 	0--no behavior
  //*        Response 	
  //*       bool result 	true--succeed 	false--failed
  //*
  //***************************************************************************

  Util::OpRet StereoDepthSubscription()
  {
    return Util::OpRet::BuildError( 
      "StereoDepthSubscription not implemented", true, true);
  }

  //***************************************************************************
  //*
  //* /dji_sdk/stereo_vga_subscription (dji_sdk/StereoVGASubscription)
  //*
  //*   subscribe to stereo images from the front-facing camera of M210 in 
  //*   640x480 resolution. If unsubscribe_vga is 1, service will unsubscribe 
  //*   no matter what. This service is only available on M210. 
  //*
  //*    Usage:
  //*        Request 	
  //*        uint8 vga_freq 	0--20Hz 	1--10Hz
  //*        uint8 front_vga 	1--subscribe 	0--no behavior
  //*       uint8 unsubscribe_vga 	1--unsubscribe 	0--no behavior
  //*        Response 	
  //*        bool result 	true--succeed 	false--failed
  //*
  //***************************************************************************

  Util::OpRet StereoVGASubscription()
  {
    return Util::OpRet::BuildError( 
      "StereoVGASubscription not implemented", true, true);
  }

  //***************************************************************************
  //*
  //* /dji_sdk/setup_camera_stream (dji_sdk/SetupCameraStream)
  //*
  //*   subscribe to FPV and/or main camera images. This service is only 
  //*   available on M210. 
  //*
  //*    Usage:
  //*        Request 	
  //*        uint8 cameraType 	0--FPV camera 	1--main camera
  //*        uint8 start 	1--start 	0--stop
  //*        Response 	
  //*        bool result 	true--succeed 	false--failed
  //*
  //***************************************************************************

  Util::OpRet SetupCameraStream()
  {
    return Util::OpRet::BuildError( 
      "SetupCameraStream not implemented", true, true);
  }
};

} /* namespace mav2dji*/