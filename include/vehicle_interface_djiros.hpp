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

namespace mav2dji
{

class vehicle_interface_djiros : public vehicle_interface
{
 public:

  enum CameraActionEnum {TakePhoto, StartVideo, StopVideo};

  explicit vehicle_interface_djiros();
  ~vehicle_interface_djiros();

  int init();
  vehicle_interface_ret connectToPlatform();
  vehicle_interface_ret activate();
  vehicle_interface_ret startVehicleAsync();
  vehicle_interface_ret stopVehicle();

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
            camAction.request.camera_action = dji_sdk::CameraAction::RequestType::CAMERA_ACTION_TAKE_PICTURE; 
            break;
          case StartVideo: 
            camAction.request.camera_action = dji_sdk::CameraAction::RequestType::CAMERA_ACTION_START_RECORD; 
            break;
          case StopVideo: 
            camAction.request.camera_action = dji_sdk::CameraAction::RequestType::CAMERA_ACTION_STOP_RECORD; 
            break;
        }

        auto service  = rosNodeHandle->serviceClient<dji_sdk::DroneArmControl>
            ("/dji_sdk/camera_action");

        service.call(camAction);

        if(!camAction.response.result) 
          return Util::OpRet::BuildError(true, true, "Could not perform camera action");

    }
    catch(const std::exception& e)
    {
      return Util::OpRet::UnwindStdException(e,"Could not perform camera action", true, true);
    }
    catch(...)
    {
        return Util::OpRet::BuildError("Could not perform camera action. Unrecognized Exception", true, true);
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

        auto service  = rosNodeHandle->serviceClient<dji_sdk::DroneArmControl>
            ("/dji_sdk/drone_arm_control");

        service.call(armControl);

        if(!armControl.response.result) 
          return Util::OpRet::BuildError(true, true, 
            "Could not arm vehicle : ack.info: set = %i id = %i data=%i", 
            armControl.response.cmd_set, armControl.response.cmd_id, armControl.response.ack_data);
    }
    catch(const std::exception& e)
    {
      return Util::OpRet::UnwindStdException(e,"Could not arm vehicle", true, true);
    }
    catch(...)
    {
        return Util::OpRet::BuildError("Could not arm vehicle. Unrecognized Exception", true, true);
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

  //***************************************************************************
  //*
  //* /dji_sdk/query_drone_version (dji_sdk/QueryDroneVersion)
  //*
  //*   Query drone firmware version. Available version list can be found in 
  //*   dji_sdk.h 
  //*
  //***************************************************************************

  //***************************************************************************
  //*
  //* /dji_sdk/set_local_pos_ref (dji_sdk/SetLocalPosRef)
  //*
  //*   Set the origin of the local position to be the current GPS coordinate. 
  //*   Fail if GPS health is low (<=3). 
  //*
  //***************************************************************************

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

 private:

   int DjiActivationSleepMs = 1000;
   int DjiActivationTimeoutMs = 10000;

   std::thread vehicleRunWorkerThread;

   void testCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

   //std::shared_ptr<ros::NodeHandle> rosNodeHandle;
   ros::ServiceClient   drone_activation_service;

   void vehicleRunWorker();

};

} /* namespace mav2dji*/