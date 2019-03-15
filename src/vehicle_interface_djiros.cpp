/*
 * mav2dji_ros.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <vehicle_interface_djiros.hpp>
#include <chrono>
#include <thread>

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
    return 0;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

vehicle_interface_ret vehicle_interface_djiros::connectToRos()
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

        rosNodeHandle = std::make_unique<ros::NodeHandle>("~");
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
    auto ret = vehicle_interface_djiros::connectToRos();

    if(!ret.Result == vehicle_interface_ret::resultEnum::success)
        return ret;

    try
    {
        int RetryCount = 0;
        int RetryCountLimit = DjiActivationTimeoutMs / DjiActivationSleepMs;
        dji_sdk::Activation activation;

        drone_activation_service  = rosNodeHandle->serviceClient<dji_sdk::Activation>
            ("/dji_sdk/activation");

        if(!drone_activation_service.isValid())
        {
            ROS_ERROR("vehicle_interface_djiros::activate() : '/dji_sdk/activation' is not installed.");
            rosNodeHandle->shutdown();
            return vehicle_interface_ret(
                vehicle_interface_ret::resultEnum::failure, 
                "Could not activate DJI vehicle. '/dji_sdk/activation' is not installed.");
        }

        ros::Duration timeout(20,0);

        ROS_INFO("vehicle_interface_djiros::activate() : waiting for '/dji_sdk/activation' to become available.");
        if(!drone_activation_service.waitForExistence(timeout))
        {
            ROS_ERROR("vehicle_interface_djiros::activate() : '/dji_sdk/activation' service is not running. Check Drone connection.");
            rosNodeHandle->shutdown();
            return vehicle_interface_ret(
                vehicle_interface_ret::resultEnum::failure, 
                "Could not activate DJI vehicle. '/dji_sdk/activation' service is not running. Check Drone connection.");
        }

        drone_activation_service.call(activation);

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

}