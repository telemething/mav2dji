/*
 * mav2dji_ros.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <vehicle_interface_djiros.hpp>
#include <chrono>

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

VehicleInterfaceDjiros::VehicleInterfaceDjiros() 
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

VehicleInterfaceDjiros::VehicleInterfaceDjiros(
  std::shared_ptr<iVehicleTelemetry> vehicleTelemetry) : 
    vehicle_interface(vehicleTelemetry)
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

VehicleInterfaceDjiros::~VehicleInterfaceDjiros()
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

Util::OpRet VehicleInterfaceDjiros::init()
{
  try
  { 
    droneActivationService = 
        rosNodeHandle->serviceClient<dji_sdk::Activation>
            ("/dji_sdk/activation");

    cameraActionService  = 
        rosNodeHandle->serviceClient<dji_sdk::DroneArmControl>
            ("/dji_sdk/camera_action");

    armVehicleService  = 
        rosNodeHandle->serviceClient<dji_sdk::DroneArmControl>
            ("/dji_sdk/drone_arm_control");

    vehicleTaskService  = 
        rosNodeHandle->serviceClient<dji_sdk::DroneArmControl>
            ("/dji_sdk/drone_task_control");

    missionWpSetSpeedService  = 
          rosNodeHandle->serviceClient<dji_sdk::MissionWpGetInfo>
            ("/dji_sdk/mission_waypoint_getInfo");

    missionWpGetSpeedService = 
          rosNodeHandle->serviceClient<dji_sdk::MissionWpGetSpeed>
            ("/dji_sdk/mission_waypoint_getSpeed");

    missionWpUploadService  = 
          rosNodeHandle->serviceClient<dji_sdk::MissionWpUpload>
            ("/dji_sdk/mission_waypoint_upload");

    sDKControlAuthorityService  = 
        rosNodeHandle->serviceClient<dji_sdk::SDKControlAuthority>
            ("/dji_sdk/sdk_control_authority");

    queryDroneVersionService  = 
        rosNodeHandle->serviceClient<dji_sdk::QueryDroneVersion>
            ("/dji_sdk/query_drone_version");

    setLocalPosRefService  = 
        rosNodeHandle->serviceClient<dji_sdk::SetLocalPosRef>
            ("/dji_sdk/set_local_pos_ref");

    missionWpActionService  = 
        rosNodeHandle->serviceClient<dji_sdk::MissionWpAction>
            ("/dji_sdk/mission_waypoint_action");
  }
  catch(const std::exception& e)
  {
    return Util::OpRet::UnwindStdException(e,
      "Could not access ROS services", true, true);
  }
  catch(...)
  {
    return Util::OpRet::BuildError(
      "Could not access ROS services. Unrecognized Exception", true, true);
  }

  ROS_INFO("VehicleInterfaceDjiros::init() : Connected to ROS services OK");
  return Util::OpRet();
}

//***************************************************************************
//*
//* Convert
//*
//***************************************************************************

std::shared_ptr<dji_sdk::MissionWaypointTask> 
  VehicleInterfaceDjiros::Convert(const mav2dji::MissionWaypointTask* waypointTask )
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

    for(auto wPointIn : waypointTask->mission_waypoint )
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
    VehicleInterfaceDjiros::Convert( const dji_sdk::MissionWaypointTask* waypointTask )
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

    for(auto wPointIn : waypointTask->mission_waypoint )
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

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

void VehicleInterfaceDjiros::testCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  ros::Time startTime = ros::Time::now();
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

Util::OpRet VehicleInterfaceDjiros::connectToPlatform()
{
    int argc = 0;
    char* argv = (char*)"";
    bool contactedMaster = false;
    
    std::string rosNodeName = "mav2dji";

    try
    {
      ros::init(argc, &argv, rosNodeName);

      contactedMaster = !ros::master::check();

      if( contactedMaster )
        return Util::OpRet(
          Util::OpRet::resultEnum::failure, 
          "Could not contact ROS master node. Make sure ROS is running.");

      rosNodeHandle = std::make_shared<ros::NodeHandle>("~");
      VehicleInfo::params->readParams(rosNodeHandle);
    }
    catch(const std::exception& e)
    {
      if(contactedMaster)
        ROS_ERROR("Could not initialize ROS node. Exception: %s", e.what() );

      return Util::OpRet(
        Util::OpRet::resultEnum::failure, 
        "Could not initialize ROS node. Exception: " + std::string(e.what()) );
    }
    catch(...)
    {
      if(contactedMaster)
        ROS_ERROR("Could not initialize ROS node. Unrecognized Exception" );

      return Util::OpRet(
        Util::OpRet::resultEnum::failure, 
        "Could not initialize ROS node. Unrecognized Exception" );
    }

    init();

    ///*** TEST **************

    //topicSubscription2 = rosNodeHandle->subscribe(
	//	"/dji_sdk/gps_position", 1,
      //  &VehicleInterfaceDjiros::testCallback, this);

    //ros::spin();   
 
    ///*** TEST **************
    
    ROS_INFO("VehicleInterfaceDjiros::connectToRos() : Connected to Master Node OK");
    return Util::OpRet();
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

Util::OpRet VehicleInterfaceDjiros::activate()
{
    //auto ret = VehicleInterfaceDjiros::connectToPlatform();

    //if(!ret.Result == vehicle_interface_ret::resultEnum::success)
    //    return ret;

    // This is here for testing. It allows us to proceed without requiring
    // a connection to a live vehicle.
    if( VehicleInfo::params->VehicleInterface->fakeVehicleConnection )
        return Util::OpRet();

    try
    {
      ROS_INFO_STREAM("VehicleInterfaceDjiros::activate()");
        
      int RetryCount = 0;
      int RetryCountLimit = DjiActivationTimeoutMs / DjiActivationSleepMs;
      dji_sdk::Activation activation;

      if(!droneActivationService.isValid())
      {
        ROS_ERROR("VehicleInterfaceDjiros::activate() : '/dji_sdk/activation' is not installed.");
        rosNodeHandle->shutdown();
        return Util::OpRet(
          Util::OpRet::resultEnum::failure, 
          "Could not activate DJI vehicle. '/dji_sdk/activation' is not installed.");
      }

      ros::Duration timeout(20,0);

      ROS_INFO("VehicleInterfaceDjiros::activate() : waiting for '/dji_sdk/activation' to become available.");
      if(!droneActivationService.waitForExistence(timeout))
      {
        ROS_ERROR("VehicleInterfaceDjiros::activate() : '/dji_sdk/activation' service is not running. Check Drone connection.");
        rosNodeHandle->shutdown();
        return Util::OpRet(
          Util::OpRet::resultEnum::failure, 
          "Could not activate DJI vehicle. '/dji_sdk/activation' service is not running. Check Drone connection.");
      }

      droneActivationService.call(activation);

      if(!activation.response.result) 
      {
        ROS_ERROR("VehicleInterfaceDjiros::activate() : Could not activate DJI vehicle : ack.info: set = %i id = %i data=%i", 
          activation.response.cmd_set, activation.response.cmd_id, activation.response.ack_data);
        rosNodeHandle->shutdown();
        return Util::OpRet(
          Util::OpRet::resultEnum::failure, "Could not activate DJI vehicle. Check credentials");
      }
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("VehicleInterfaceDjiros::activate() : Exception : %s", e.what() );
      rosNodeHandle->shutdown();
      return Util::OpRet(
        Util::OpRet::resultEnum::failure, 
        "Could not activate DJI vehicle. Exception: " + std::string(e.what()) );
    }
    catch(...)
    {
      ROS_ERROR("Could not initialize ROS node. Unrecognized Exception" );
      rosNodeHandle->shutdown();
      return Util::OpRet(
        Util::OpRet::resultEnum::failure, 
        "Could not initialize ROS node. Unrecognized Exception" );
    }

    //return {activation.response.result, activation.response.cmd_set,
    //        activation.response.cmd_id, activation.response.ack_data};

    ROS_INFO("VehicleInterfaceDjiros::activate() : Drone activated OK");
    return Util::OpRet();
}

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

Util::OpRet VehicleInterfaceDjiros::startVehicleAsync()
{
    ROS_INFO("VehicleInterfaceDjiros::startVehicleAsync() : Starting Worker Thread");

    vehicleRunWorkerThread = std::thread(
        &VehicleInterfaceDjiros::vehicleRunWorker, this);

    return Util::OpRet();
}

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

void VehicleInterfaceDjiros::vehicleRunWorker()
{
    ROS_INFO("VehicleInterfaceDjiros::vehicleRunWorker() : Worker Thread Started OK");
    ros::spin();    
}

//*****************************************************************************
//*
//* shutdown();
//*
//******************************************************************************

Util::OpRet VehicleInterfaceDjiros::stopVehicle()
{
    ROS_INFO("VehicleInterfaceDjiros::vehicleRunWorker() : Stopping Worker Thread");
    ros::shutdown();
    return Util::OpRet();
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

  Util::OpRet VehicleInterfaceDjiros::Activation()
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

  Util::OpRet VehicleInterfaceDjiros::CameraAction(CameraActionEnum action)
  {
    try
    {
      ROS_INFO_STREAM("VehicleInterfaceDjiros::CameraAction()");

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

      if( VehicleInfo::params->VehicleInterface->fakeVehicleConnection )
        return Util::OpRet();

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

  Util::OpRet VehicleInterfaceDjiros::ArmVehicle(bool arm)
  {
    try
    {
      ROS_INFO_STREAM("VehicleInterfaceDjiros::ArmVehicle()");

      dji_sdk::DroneArmControl armControl;

      arm ? armControl.request.arm = armControl.request.ARM_COMMAND 
          : armControl.request.arm = armControl.request.DISARM_COMMAND;

      if( VehicleInfo::params->VehicleInterface->fakeVehicleConnection )
        return Util::OpRet();

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

  Util::OpRet VehicleInterfaceDjiros::VehicleTask(VehicleTaskEnum task)
  {
    try
    {
      ROS_INFO_STREAM("VehicleInterfaceDjiros::VehicleTask()");

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

      if( VehicleInfo::params->VehicleInterface->fakeVehicleConnection )
        return Util::OpRet();

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

  Util::OpRet VehicleInterfaceDjiros::MFIOConfig()
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

  Util::OpRet VehicleInterfaceDjiros::MFIOSetValue()
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

  Util::OpRet VehicleInterfaceDjiros::MissionHpAction()
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

  Util::OpRet VehicleInterfaceDjiros::MissionHpGetInfo()
  {
    return Util::OpRet::BuildError( 
      "MissionHpGetInfo not implemented", true, true);
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

  Util::OpRet VehicleInterfaceDjiros::MissionHpResetYaw()
  {
    return Util::OpRet::BuildError( 
      "MissionHpResetYaw not implemented", true, true);
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

  Util::OpRet VehicleInterfaceDjiros::MissionHpUpdateRadius()
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

  Util::OpRet VehicleInterfaceDjiros::MissionHpUpdateYawRate()
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

  Util::OpRet VehicleInterfaceDjiros::MissionHpUpload()
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

  Util::OpRet VehicleInterfaceDjiros::MissionWpAction( 
    const vehicle_interface::MissionWpActionEnum action )
  {
    try
    {
      ROS_INFO_STREAM("VehicleInterfaceDjiros::MissionWpAction()");

      dji_sdk::MissionWpAction missionWpAction;

      switch(action)
      {
        case MissionWpActionStart :
          missionWpAction.request.action = dji_sdk::MissionWpActionRequest::ACTION_START; 
          ROS_INFO_STREAM("Mission action : start");
          break;
        case MissionWpActionStop :
          missionWpAction.request.action = dji_sdk::MissionWpActionRequest::ACTION_STOP; 
          ROS_INFO_STREAM("Mission action : stop");
          break;
        case MissionWpActionPause :
          missionWpAction.request.action = dji_sdk::MissionWpActionRequest::ACTION_PAUSE; 
          ROS_INFO_STREAM("Mission action : pause");
          break;
        case MissionWpActionResume :
          missionWpAction.request.action = dji_sdk::MissionWpActionRequest::ACTION_RESUME; 
          ROS_INFO_STREAM("Mission action : continue");
          break;
      }

      if( VehicleInfo::params->VehicleInterface->fakeVehicleConnection )
        return Util::OpRet();

      // Local declaration to get avoid the error:
      // [ERROR] [1555546053.598870434]: Call to service [/dji_sdk/mission_waypoint_action] 
      // with md5sum [067ec5f79e77e0b4c0121e09e733b483] does not match md5sum when the handle 
      // was created ([eb13ac1f1354ccecb7941ee8fa2192e8])

      //auto missionWpActionService2  = 
      //  rosNodeHandle->serviceClient<dji_sdk::SetLocalPosRef>
      //      ("/dji_sdk/mission_waypoint_action");

      missionWpActionService.call(missionWpAction);

      if(!missionWpAction.response.result)
        return Util::OpRet::BuildError(true, true, 
            "Could set mission action : ack.info: set = %i id = %i data=%i", 
            missionWpAction.response.cmd_set, 
            missionWpAction.response.cmd_id, 
            missionWpAction.response.ack_data);
    }
    catch(const std::exception& e)
    {
      return Util::OpRet::UnwindStdException(e,
        "Could not set mission action", true, true);
    }
    catch(...)
    {
        return Util::OpRet::BuildError(
          "Could not set mission action. Unrecognized Exception", true, true);
    }

    ROS_INFO_STREAM("Set mission action OK");
    return Util::OpRet();
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

  Util::OpRet VehicleInterfaceDjiros::MissionWpGetInfo(
    std::shared_ptr<mav2dji::MissionWaypointTask>* waypointTask)
  {
    try
    {
      ROS_INFO_STREAM("VehicleInterfaceDjiros::MissionWpGetInfo()");

      dji_sdk::MissionWpGetInfo missionWpGetInfo;

      missionWpSetSpeedService.call(missionWpGetInfo);

      *waypointTask = Convert(&missionWpGetInfo.response.waypoint_task);
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

  float VehicleInterfaceDjiros::MissionWpGetSpeed()
  {
    try
    {
      ROS_INFO_STREAM("VehicleInterfaceDjiros::MissionWpSetSpeed()");

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

  Util::OpRet VehicleInterfaceDjiros::MissionWpSetSpeed(float speed)
  {
    try
    {
      ROS_INFO_STREAM("VehicleInterfaceDjiros::MissionWpSetSpeed()");

      dji_sdk::MissionWpSetSpeed missionWpSetSpeed;

      ros::ServiceClient missionWpSetSpeedService  = 
        rosNodeHandle->serviceClient<dji_sdk::MissionWpGetSpeed>
          ("/dji_sdk/mission_waypoint_setSpeed");

      missionWpSetSpeed.request.speed = speed;

      if( VehicleInfo::params->VehicleInterface->fakeVehicleConnection )
        return Util::OpRet();

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

  Util::OpRet VehicleInterfaceDjiros::MissionWpUpload(
    const mav2dji::MissionWaypointTask* waypointTask)
  {
    haveUploadedMission = false;
    
    try
    {
      ROS_INFO_STREAM("VehicleInterfaceDjiros::MissionWpUpload()");

      dji_sdk::MissionWpUpload missionWaypoint;

      auto wpt = Convert(waypointTask);

      missionWaypoint.request.waypoint_task = *wpt;

      if( VehicleInfo::params->VehicleInterface->fakeVehicleConnection )
        return Util::OpRet();

      missionWpUploadService.call(missionWaypoint);

        //*** TODO * the service is claiming failure, but it seems to work
        //*** so we return ok for now, but this must be fixed

      haveUploadedMission = true;
      ROS_INFO_STREAM("Upload mission waypoint OK");
      return Util::OpRet();

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

    haveUploadedMission = true;
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

  Util::OpRet VehicleInterfaceDjiros::SDKControlAuthority(ControlAutority authority)
  {
    try
    {
      ROS_INFO_STREAM("VehicleInterfaceDjiros::SDKControlAuthority()");

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

  Util::OpRet VehicleInterfaceDjiros::SendMobileData()
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

  Util::OpRet VehicleInterfaceDjiros::SetHardSync()
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

  VehicleInterfaceDjiros::DroneVersion VehicleInterfaceDjiros::QueryDroneVersion()
  {
    DroneVersion droneVersion;

    try
    {
      ROS_INFO_STREAM("VehicleInterfaceDjiros::QueryDroneVersion()");

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

  Util::OpRet VehicleInterfaceDjiros::SetLocalPosRef()
  {
    try
    {
      ROS_INFO_STREAM("VehicleInterfaceDjiros::SetLocalPosRef()");

      dji_sdk::SetLocalPosRef localPosRef;

      if( VehicleInfo::params->VehicleInterface->fakeVehicleConnection )
        return Util::OpRet();

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

  Util::OpRet VehicleInterfaceDjiros::Stereo240pSubscription()
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

  Util::OpRet VehicleInterfaceDjiros::StereoDepthSubscription()
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

  Util::OpRet VehicleInterfaceDjiros::StereoVGASubscription()
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

  Util::OpRet VehicleInterfaceDjiros::SetupCameraStream()
  {
    return Util::OpRet::BuildError( 
      "SetupCameraStream not implemented", true, true);
  }

  //***************************************************************************
  //*
  //* Chaange UAV to MAVLINK mode
  //*
  //* mavModeFlagCustomModeEnabled  0b00000001 Reserved for future use. 
  //* mavModeFlagTestEnabled  0b00000010 system has a test mode enabled. This 
  //*   flag is intended for temporary system tests and should not be used for 
  //*   stable implementations. 
  //* mavModeFlagAutoEnabled 0b00000100 autonomous mode enabled, system finds 
  //*   its own goal positions. Guided flag can be set or not, depends on the 
  //*   actual implementation. 
  //* mavModeFlagGuidedEnabled  0b00001000 guided mode enabled, system flies 
  //*   waypoints / mission items. 
  //* mavModeFlagStabilizeEnabled  0b00010000 system stabilizes electronically 
  //*   its attitude (and optionally position). It needs however further control 
  //*   inputs to move around. 
  //* mavModeFlagHilEnbaled 0b00100000 hardware in the loop simulation. All 
  //*   motors actuators are blocked, but internal software is full operational. 
  //* mavModeFlagManualInputEnabled 0b01000000 remote control input is enabled. 
  //* mavModeFlagSafteyArmed  0b10000000 MAV safety set to armed. Motors are 
  //*   enabled running can start. Ready to fly. Additional note: this flag is 
  //*   to be ignore when sent in the command MAV_CMD_DO_SET_MODE and 
  //*   MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be 
  //*   used to report the armed state. 
  //*
  //***************************************************************************

  Util::OpRet VehicleInterfaceDjiros::setMode(uint8_t baseMode, uint32_t customMode)
  {
      char const* format = "VehicleInterfaceDjiros::setMode():\r\n"
    " - MissionInt %d\r\n - CommandInt %d\r\n - ParamUnion %d\r\n - FTP %d\r\n"
    " - BaseBits %u : %02X\r\n - CustomBits %u : %08X\r\n - Custom %d\r\n"
    " - Test %d\r\n - Auto %d\r\n - Guided %d\r\n - Stabilize %d\r\n - HIL %d\r\n"
    " - Manual %d\r\n - Armed %d\r\n";
    
    printf(format,
      baseMode, baseMode, customMode, customMode,
      baseMode & MavModeFlag_t::mavModeFlagCustomModeEnabled,
      baseMode & MavModeFlag_t::mavModeFlagTestEnabled,
      baseMode & MavModeFlag_t::mavModeFlagAutoEnabled,
      baseMode & MavModeFlag_t::mavModeFlagGuidedEnabled,
      baseMode & MavModeFlag_t::mavModeFlagStabilizeEnabled,
      baseMode & MavModeFlag_t::mavModeFlagHilEnbaled,
      baseMode & MavModeFlag_t::mavModeFlagManualInputEnabled, 
      baseMode & MavModeFlag_t::mavModeFlagSafteyArmed );

    mavBaseMode = baseMode;
    mavCustomMode = customMode;

    //*** make changes

    if(baseMode & MavModeFlag_t::mavModeFlagCustomModeEnabled)
    {
    }

    if(baseMode & MavModeFlag_t::mavModeFlagTestEnabled)
    {
    }

    if(baseMode & MavModeFlag_t::mavModeFlagAutoEnabled)
    {
    }

    if(baseMode & MavModeFlag_t::mavModeFlagGuidedEnabled)
    {
    }

    if(baseMode & MavModeFlag_t::mavModeFlagStabilizeEnabled)
    {
    }

    if(baseMode & MavModeFlag_t::mavModeFlagHilEnbaled)
    {
    }

    if(baseMode & MavModeFlag_t::mavModeFlagManualInputEnabled)
    {
    }

    if(baseMode & MavModeFlag_t::mavModeFlagSafteyArmed)
    {
    }

    //*** reflect new vehicle state

    mavState = MavState_t::mavStateUninit;
    MavState_t::mavStateBoot;
    MavState_t::mavStateCalirating;
    MavState_t::mavStateStandby;
    MavState_t::mavStateActive; 
    MavState_t::mavStateCritical; 
    MavState_t::mavStateEmergency;
    MavState_t::mavStatePowerOff;
    MavState_t::mavStateFlightTermination;

    vehicleTelemetry->setBaseMode(baseMode);
    vehicleTelemetry->setCustomMode(customMode);
    setState(mavState);

    ROS_INFO_STREAM("VehicleInterfaceDjiros::setMode() OK");
    return Util::OpRet();
  }

  //***************************************************************************
  //*
  //* Set UAV to MAVLINK state. Set a class member to the state and inform
  //* the telemtry class of the state change. Normally this should not be called 
  //* from outside this class. Outside classes should call other methods to 
  //* effect state changes.
  //*
  //* mavStateUninit=0,  Uninitialized system, state is unknown. 
  //* mavStateBoot=1,  System is booting up. 
  //* mavStateCalirating=2,  System is calibrating and not flight-ready. 
  //* mavStateStandby=3,  System is grounded and on standby. It can be launched 
  //*  any time. 
  //* mavStateActive=4,  System is active and might be already airborne. Motors 
  //*  are engaged. 
  //* mavStateCritical=5,  System is in a non-normal flight mode. It can however 
  //*  still navigate. 
  //* mavStateEmergency=6,  System is in a non-normal flight mode. It lost control 
  //*  over parts or over the whole airframe. It is in mayday and going down.
  //* mavStatePowerOff=7,  System just initialized its power-down sequence, will 
  //*  shut down now. 
  //* mavStateFlightTermination=8,  System is terminating itself. 
  //*
  //***************************************************************************

  Util::OpRet VehicleInterfaceDjiros::setState(MavState_t newState)
  {
    printf("VehicleInterfaceDjiros::setState(): ");

    switch( newState )
    {
      case MavState_t::mavStateUninit:
        printf("Uninitialized\r\n");
      break;
      case MavState_t::mavStateBoot:
        printf("Booting\r\n");      
      break;
      case MavState_t::mavStateCalirating:
        printf("Calibrating\r\n");     
        break;
      case MavState_t::mavStateStandby:
        printf("Standby\r\n");      
      break;
      case MavState_t::mavStateActive:
        printf("Active\r\n");      
      break;
      case MavState_t::mavStateCritical: 
        printf("Critical\r\n");      
      break;
      case MavState_t::mavStateEmergency:
        printf("Emergency\r\n");      
      break;
      case MavState_t::mavStatePowerOff:
        printf("PoweringOff\r\n");      
      break;
      case MavState_t::mavStateFlightTermination:
        printf("FlightTermination\r\n");      
      break;
      default:
        printf("--- Unrecognized ---\r\n");  
      break;
    }

    vehicleTelemetry->setSystemStatus(newState);

    ROS_INFO_STREAM("VehicleInterfaceDjiros::setState() OK");
    return Util::OpRet(); 
  }

  //***************************************************************************
  //*
  //* React to arm/disarm. Call in it's own thread, it may take a while
  //*
  //***************************************************************************

  void VehicleInterfaceDjiros::ArmChange(bool armed)
  {
    printf("--- VehicleInterfaceDjiros::ArmChange() ---\r\n"); 

    if( armed )
    {
      printf("--- armed ---\r\n"); 

      // if we have a mission, then start it
      if(haveUploadedMission)
      {
          printf("--- we have a mission, set local position reference ---\r\n");  
          auto ret = SetLocalPosRef();

          if(ret.Result == Util::OpRet::resultEnum::failure)
          {
            printf("--- local pos ref failure ---\r\n");
          }
          else
          {
            printf("--- loacl pos ref success ---\r\n");

            printf("--- start the mission ---\r\n");  
            ret = MissionWpAction( 
              vehicle_interface::MissionWpActionEnum::MissionWpActionStart );

            if(ret.Result == Util::OpRet::resultEnum::failure)
            {
              printf("--- mission start failure ---\r\n");
            }
            else
            {
              printf("--- mission start success ---\r\n");
            }
          }
      }
      else
      {
        {
          printf("---- we dont have a mission ---\r\n"); 
        }
      }  
    } 
    else
    {
      printf("--- disarmed ---\r\n"); 
    }    
  }

  //***************************************************************************
  //*
  //* Arm or disarm UAV. This can be called from outside classes
  //*
  //***************************************************************************

  Util::OpRet VehicleInterfaceDjiros::armDisarm(bool arm)
  {
    // signal telemetry
    //vehicleTelemetry->setArmed(true);

    if(vehicleTelemetry->isArmed())
    {
      ROS_INFO_STREAM("VehicleInterfaceDjiros::armDisarm() vehicle already armed");
    }
    else
    {   
      // signal the vehicle to arm
      auto ret = ArmVehicle(arm);

      if(ret.Result == Util::OpRet::resultEnum::failure)
      {
        ROS_INFO_STREAM("VehicleInterfaceDjiros::armDisarm() Exception");
        return ret;
      }

      ArmChangeThread = std::thread(&VehicleInterfaceDjiros::ArmChange, this, arm);
    }

    ROS_INFO_STREAM("VehicleInterfaceDjiros::armDisarm() OK");
    return Util::OpRet();   
  }
}

