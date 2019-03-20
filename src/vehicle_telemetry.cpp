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

void TelemetrySource_GlobalPositionInt::telemetryRunWorker()
{
    ROS_INFO("TelemetrySource_GlobalPositionInt : Worker Thread Started OK");

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

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

void TelemetrySource_Attitude::telemetryRunWorker()
{
    ROS_INFO("TelemetrySource_Attitude : Worker Thread Started OK");

    while (ros::ok())
    {
		mavlink_msg_attitude_pack(mavlinkSystemId, mavlinkComponentId, &mavlinkMsg, 
			microsSinceEpoch(), 1.2, 1.7, 3.14, 0.01, 0.02, 0.03);
					
		sendMavMessageToGcs(&mavlinkMsg);
        workerRosRate->sleep();
    }
}

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

void TelemetrySource_LocalPositionNed::telemetryRunWorker()
{
    ROS_INFO("TelemetrySource_LocalPositionNed : Worker Thread Started OK");

	float position[6] = {};

    while (ros::ok())
    {
		mavlink_msg_local_position_ned_pack(mavlinkSystemId, mavlinkComponentId, 
			&mavlinkMsg, microsSinceEpoch(), 
			position[0], position[1], position[2],
			position[3], position[4], position[5]);
					
		sendMavMessageToGcs(&mavlinkMsg);
        workerRosRate->sleep();
    }
}

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

void TelemetrySource_Heartbeat::telemetryRunWorker()
{
    ROS_INFO("TelemetrySource_Heartbeat : Worker Thread Started OK");

    while (ros::ok())
    {
		mavlink_msg_heartbeat_pack(mavlinkSystemId, mavlinkComponentId, 
			&mavlinkMsg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, 
			MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
					
		sendMavMessageToGcs(&mavlinkMsg);
        workerRosRate->sleep();
    }
}

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

void TelemetrySource_SysStatus::telemetryRunWorker()
{
    ROS_INFO("TelemetrySource_SysStatus : Worker Thread Started OK");

    while (ros::ok())
    {
		mavlink_msg_sys_status_pack(mavlinkSystemId, mavlinkComponentId, 
			&mavlinkMsg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
					
		sendMavMessageToGcs(&mavlinkMsg);
        workerRosRate->sleep();
    }
}

}