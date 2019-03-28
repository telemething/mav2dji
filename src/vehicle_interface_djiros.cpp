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

vehicle_interface_djiros::vehicle_interface_djiros()
{
  init();
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

vehicle_interface_djiros::~vehicle_interface_djiros()
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int vehicle_interface_djiros::init()
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

    return 0;
}

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

void vehicle_interface_djiros::testCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  ros::Time startTime = ros::Time::now();
}

ros::Subscriber topicSubscription2;

//*****************************************************************************
//*
//*
//*
//******************************************************************************

vehicle_interface_ret vehicle_interface_djiros::connectToPlatform()
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
            return vehicle_interface_ret(
                vehicle_interface_ret::resultEnum::failure, 
                "Could not contact ROS master node. Make sure ROS is running.");

        rosNodeHandle = std::make_shared<ros::NodeHandle>("~");

        VehicleInfo::params->readParams(rosNodeHandle);
    }
    catch(const std::exception& e)
    {
        if(contactedMaster)
            ROS_ERROR("Could not initialize ROS node. Exception: %s", e.what() );

        return vehicle_interface_ret(
            vehicle_interface_ret::resultEnum::failure, 
            "Could not initialize ROS node. Exception: " + std::string(e.what()) );
    }
    catch(...)
    {
        if(contactedMaster)
            ROS_ERROR("Could not initialize ROS node. Unrecognized Exception" );

        return vehicle_interface_ret(
            vehicle_interface_ret::resultEnum::failure, 
            "Could not initialize ROS node. Unrecognized Exception" );
    }

    ///*** TEST **************

    //topicSubscription2 = rosNodeHandle->subscribe(
	//	"/dji_sdk/gps_position", 1,
      //  &vehicle_interface_djiros::testCallback, this);

    //ros::spin();   
 
    ///*** TEST **************
    
    ROS_INFO("vehicle_interface_djiros::connectToRos() : Connected to Master Node OK");
    return vehicle_interface_ret(vehicle_interface_ret::resultEnum::success);
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

vehicle_interface_ret vehicle_interface_djiros::activate()
{
    //auto ret = vehicle_interface_djiros::connectToPlatform();

    //if(!ret.Result == vehicle_interface_ret::resultEnum::success)
    //    return ret;

    // This is here for testing. It allows us to proceed without requiring
    // a connection to a live vehicle.
    if( VehicleInfo::params->VehicleInterface->fakeVehicleConnection )
        return vehicle_interface_ret();

    try
    {
        int RetryCount = 0;
        int RetryCountLimit = DjiActivationTimeoutMs / DjiActivationSleepMs;
        dji_sdk::Activation activation;

        if(!droneActivationService.isValid())
        {
            ROS_ERROR("vehicle_interface_djiros::activate() : '/dji_sdk/activation' is not installed.");
            rosNodeHandle->shutdown();
            return vehicle_interface_ret(
                vehicle_interface_ret::resultEnum::failure, 
                "Could not activate DJI vehicle. '/dji_sdk/activation' is not installed.");
        }

        ros::Duration timeout(20,0);

        ROS_INFO("vehicle_interface_djiros::activate() : waiting for '/dji_sdk/activation' to become available.");
        if(!droneActivationService.waitForExistence(timeout))
        {
            ROS_ERROR("vehicle_interface_djiros::activate() : '/dji_sdk/activation' service is not running. Check Drone connection.");
            rosNodeHandle->shutdown();
            return vehicle_interface_ret(
                vehicle_interface_ret::resultEnum::failure, 
                "Could not activate DJI vehicle. '/dji_sdk/activation' service is not running. Check Drone connection.");
        }

        droneActivationService.call(activation);

        if(!activation.response.result) 
        {
            ROS_ERROR("vehicle_interface_djiros::activate() : Could not activate DJI vehicle : ack.info: set = %i id = %i data=%i", 
                activation.response.cmd_set, activation.response.cmd_id, activation.response.ack_data);
            rosNodeHandle->shutdown();
            return vehicle_interface_ret(
                vehicle_interface_ret::resultEnum::failure, "Could not activate DJI vehicle. Check credentials");
        }
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("vehicle_interface_djiros::activate() : Exception : %s", e.what() );
        rosNodeHandle->shutdown();
        return vehicle_interface_ret(
            vehicle_interface_ret::resultEnum::failure, 
            "Could not activate DJI vehicle. Exception: " + std::string(e.what()) );
    }
    catch(...)
    {
        ROS_ERROR("Could not initialize ROS node. Unrecognized Exception" );
        rosNodeHandle->shutdown();
        return vehicle_interface_ret(
            vehicle_interface_ret::resultEnum::failure, 
            "Could not initialize ROS node. Unrecognized Exception" );
    }

    //return {activation.response.result, activation.response.cmd_set,
    //        activation.response.cmd_id, activation.response.ack_data};

    ROS_INFO("vehicle_interface_djiros::activate() : Drone activated OK");
    return vehicle_interface_ret(vehicle_interface_ret::resultEnum::success);
}

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

vehicle_interface_ret vehicle_interface_djiros::startVehicleAsync()
{
    ROS_INFO("vehicle_interface_djiros::startVehicleAsync() : Starting Worker Thread");

    vehicleRunWorkerThread = std::thread(
        &vehicle_interface_djiros::vehicleRunWorker, this);

    return vehicle_interface_ret(vehicle_interface_ret::resultEnum::success);
}

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

void vehicle_interface_djiros::vehicleRunWorker()
{
    ROS_INFO("vehicle_interface_djiros::vehicleRunWorker() : Worker Thread Started OK");
    ros::spin();    
}

//*****************************************************************************
//*
//* shutdown();
//*
//******************************************************************************

vehicle_interface_ret vehicle_interface_djiros::stopVehicle()
{
    ROS_INFO("vehicle_interface_djiros::vehicleRunWorker() : Stopping Worker Thread");
    ros::shutdown();
    return vehicle_interface_ret(vehicle_interface_ret::resultEnum::success);
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

  Util::OpRet vehicle_interface_djiros::Activation()
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

  Util::OpRet vehicle_interface_djiros::CameraAction(CameraActionEnum action)
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

  Util::OpRet vehicle_interface_djiros::ArmVehicle(bool arm)
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

  Util::OpRet vehicle_interface_djiros::VehicleTask(VehicleTaskEnum task)
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

  Util::OpRet vehicle_interface_djiros::MFIOConfig()
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

  Util::OpRet vehicle_interface_djiros::MFIOSetValue()
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

  Util::OpRet vehicle_interface_djiros::MissionHpAction()
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

  Util::OpRet vehicle_interface_djiros::MissionHpGetInfo()
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

  Util::OpRet vehicle_interface_djiros::MissionHpResetYaw()
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

  Util::OpRet vehicle_interface_djiros::MissionHpUpdateRadius()
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

  Util::OpRet vehicle_interface_djiros::MissionHpUpdateYawRate()
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

  Util::OpRet vehicle_interface_djiros::MissionHpUpload()
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

  Util::OpRet vehicle_interface_djiros::MissionWpAction()
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

  Util::OpRet vehicle_interface_djiros::MissionWpGetInfo(
    std::shared_ptr<mav2dji::MissionWaypointTask>* waypointTask)
  {
    try
    {
        dji_sdk::MissionWpGetInfo missionWpGetInfo;

        missionWpSetSpeedService.call(missionWpGetInfo);

        *waypointTask = MissionOps::Convert(&missionWpGetInfo.response.waypoint_task);
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

  float vehicle_interface_djiros::MissionWpGetSpeed()
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

  Util::OpRet vehicle_interface_djiros::MissionWpGetSpeed(float speed)
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

  Util::OpRet vehicle_interface_djiros::MissionWpUpload(const mav2dji::MissionWaypointTask* waypointTask)
  {
    try
    {
        dji_sdk::MissionWpUpload missionWaypoint;

        auto wpt = MissionOps::Convert(waypointTask);

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

  Util::OpRet vehicle_interface_djiros::SDKControlAuthority(ControlAutority authority)
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

  Util::OpRet vehicle_interface_djiros::SendMobileData()
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

  Util::OpRet vehicle_interface_djiros::SetHardSync()
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

  vehicle_interface_djiros::DroneVersion vehicle_interface_djiros::QueryDroneVersion()
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

  Util::OpRet vehicle_interface_djiros::SetLocalPosRef()
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

  Util::OpRet vehicle_interface_djiros::Stereo240pSubscription()
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

  Util::OpRet vehicle_interface_djiros::StereoDepthSubscription()
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

  Util::OpRet vehicle_interface_djiros::StereoVGASubscription()
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

  Util::OpRet vehicle_interface_djiros::SetupCameraStream()
  {
    return Util::OpRet::BuildError( 
      "SetupCameraStream not implemented", true, true);
  }
}