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

}