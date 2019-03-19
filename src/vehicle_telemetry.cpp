/*
 * mav2dji_ros.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <vehicle_telemetry.hpp>
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

telemetry_source_global_position_int::telemetry_source_global_position_int()
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

telemetry_source_global_position_int::~telemetry_source_global_position_int()
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

//int telemetry_source::init(telemetry_source::Trigger trigger)
//{
//    return 0;
//}

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

telemetry_interface_ret telemetry_source_global_position_int::startTelemetryAsync()
{
    ROS_INFO("telemetry_interface::startVehicleAsync() : Starting Worker Thread");

    telemetryRunWorkerThread = std::thread(
        &telemetry_source_global_position_int::telemetryRunWorker, this);

    return telemetry_interface_ret(telemetry_interface_ret::resultEnum::success);
}

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

void telemetry_source_global_position_int::telemetryRunWorker()
{
    ROS_INFO("telemetry_interface::telemtryRunWorker() : Worker Thread Started OK");
    ros::spin();    
}

//*****************************************************************************
//*
//* shutdown();
//*
//******************************************************************************

telemetry_interface_ret telemetry_source_global_position_int::stopTelemetry()
{
    ROS_INFO("telemetry_interface::vehicleRunWorker() : Stopping Worker Thread");
    return telemetry_interface_ret(telemetry_interface_ret::resultEnum::success);
}

//********************************************
//********************************************

}