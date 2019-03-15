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
    
    std::string rosNodeName = "mav2dji";

    try
    {
        ros::init(argc, &argv, rosNodeName);
        ros::NodeHandle nodeHandle("~");
        rosNodeHandle = nodeHandle;

        if( !ros::isInitialized() )
        vehicle_interface_ret(
            vehicle_interface_ret::resultEnum::failure, "Could not initialize ROS node. Make sure ROS is running.");
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        vehicle_interface_ret(
            vehicle_interface_ret::resultEnum::failure, 
            "Could not initialize ROS node. Exception: " + std::string(e.what()) );
    }
    
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

        drone_activation_service  = rosNodeHandle.serviceClient<dji_sdk::Activation>
            ("/dji_sdk/activation");

        while(RetryCount < RetryCountLimit)
        {
            drone_activation_service.call(activation);

            if(activation.response.result)
            break;
            
            ROS_WARN("vehicle_interface_djiros::activate() : try %i failed", RetryCount++);
            std::this_thread::sleep_for(std::chrono::milliseconds(DjiActivationSleepMs));
        }

        if(!activation.response.result) 
        {
            ROS_WARN("vehicle_interface_djiros::activate() : ack.info: set = %i id = %i", activation.response.cmd_set, activation.response.cmd_id);
            ROS_WARN("vehicle_interface_djiros::activate() : ack.data: %i", activation.response.ack_data);
            vehicle_interface_ret(
                vehicle_interface_ret::resultEnum::failure, "Could not activate DJI vehicle. Check credentials");
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        vehicle_interface_ret(
            vehicle_interface_ret::resultEnum::failure, 
            "Could not activate DJI vehicle. Exception: " + std::string(e.what()) );
    }
    
    //return {activation.response.result, activation.response.cmd_set,
    //        activation.response.cmd_id, activation.response.ack_data};

    return vehicle_interface_ret(vehicle_interface_ret::resultEnum::success);
}

}