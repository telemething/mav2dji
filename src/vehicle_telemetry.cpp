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
    setTlemetryWorker(std::bind(
      &telemetry_source_global_position_int::telemetryRunWorker, this));
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

void telemetry_source_global_position_int::telemetryRunWorker()
{
    ROS_INFO("telemetry_source_global_position_int : Worker Thread Started OK");

    mavlink_message_t mavlinkMsg;

    while (ros::ok())
    {
        //* @param time_boot_ms [ms] Timestamp (time since system boot).
 		//* @param lat [degE7] Latitude, expressed
 		//* @param lon [degE7] Longitude, expressed
 		//* @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 		//* @param relative_alt [mm] Altitude above ground
 		//* @param vx [cm/s] Ground X Speed (Latitude, positive north)
 		//* @param vy [cm/s] Ground Y Speed (Longitude, positive east)
 		//* @param vz [cm/s] Ground Z Speed (Altitude, positive down)
 		//* @param hdg [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX

		int32_t lat = 47.4684732 *10000000L;
		int32_t lon = -121.7676069 *10000000L;
		int32_t alt = 50.0000000;
		int32_t relative_alt = 0.0;
		int32_t vx = 0;
		int16_t vy = 0;
		int16_t vz = 0;
		int16_t hdg = 10 * 100;

		mavlink_msg_global_position_int_pack(mavlinkSystemId, mavlinkComponentId, &mavlinkMsg, 
			microsSinceEpoch(), lat, lon, alt, relative_alt, vx, vy, vz, hdg );

		sendMavMessageToGcs(&mavlinkMsg);
					
        workerRosRate->sleep();
    }
}

}