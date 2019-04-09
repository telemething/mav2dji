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
//* Heartbeat
//*
//* http://discuss.px4.io/t/confused-with-guided-mode-in-qgroundcontrol-using-mavlink-v2-protocol/8091
//*
//* type  Type of the system (quadrotor, helicopter, etc.). Components use the 
//*   same type as their associated system.
//* autopilot  Autopilot type / class.
//* base_mode  System mode bitmap.
//* custom_mode  A bitfield for use for autopilot-specific flags
//* system_status  System status flag.
//*
//* PX4 SITL starts like this
//* type            2           MAV_TYPE_QUADROTOR
//* autopilot       12          MAV_AUTOPILOT_PX4
//* base_mode       29          11101
//* custom_mode     5.05938e+07 11000001000000000000001000
//* system_status   3           MAV_STATE_STANDBY
//* mavlin_version  3
//*
//******************************************************************************

void TelemetrySource_Heartbeat::telemetryRunWorker()
{
  ROS_INFO("TelemetrySource_Heartbeat : Worker Thread Started OK");

  MAV_TYPE mavType = MAV_TYPE_QUADROTOR;
  MAV_AUTOPILOT mavAutoPilot = MAV_AUTOPILOT_PX4;
  //MAV_MODE base_mode = MAV_MODE_AUTO_ARMED;     // TODO * This needs to changed during flight
  uint8_t base_mode = 29;                         // TODO * This needs to changed during flight
  MAV_STATE system_status = MAV_STATE_STANDBY;    // TODO * This needs to changed during flight
  uint32_t custom_mode = 50593800;                // TODO * This needs to changed during flight

  while (ros::ok())
  {
		mavlink_msg_heartbeat_pack(
      mavlinkSystemId, 
      mavlinkComponentId, 
			&mavlinkMsg, 
      mavType,           
      mavAutoPilot,            
			base_mode,        
      custom_mode, 
      system_status);            
					
		sendMavMessageToGcs(&mavlinkMsg);
      workerRosRate->sleep();
  }
}

//*****************************************************************************
// SystemStatus
//*****************************************************************************

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

//*****************************************************************************
// GPS
//*****************************************************************************

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

void TelemetrySource_GlobalPositionInt::telemetryInit()
{
  if( VehicleInfo::params->VehicleInterface->fakeTelemtry )
  {
    int32_t tbs = 0;   //getTimeBootMs(gPosition.header);

    double latitude = 47.4684818;
    double longitude = -121.76819669999999;
    double altitude = 174;
    double height = 5;

    vehicleTelemetry->telemGlobalPositionInt.setLatLonAlt(
      &tbs, &latitude, &longitude, &altitude, &height);
  }

	topicSubscription = rosNodeHandle->subscribe(
		"/dji_sdk/gps_position", 1,
        &TelemetrySource_GlobalPositionInt::callback, this);
}

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
    mavlink_msg_global_position_int_encode( 
      mavlinkSystemId, mavlinkComponentId, &mavlinkMsg,
      vehicleTelemetry->telemGlobalPositionInt.getDataPtr());

		sendMavMessageToGcs(&mavlinkMsg);
    workerRosRate->sleep();
  }
}

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

void TelemetrySource_GlobalPositionInt::callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  ros::Time startTime = ros::Time::now();
  ros::Duration elapsed_time = startTime - lastCallbackStartTime;

  // don't waste power collecting faster than we report
  if(elapsed_time > workerRosRate->cycleTime())
  {
    lastCallbackStartTime = startTime;

    sensor_msgs::NavSatFix gPosition = *msg;
    int32_t tbs = getTimeBootMs(gPosition.header);

    vehicleTelemetry->telemGlobalPositionInt.setLatLonAlt(
      &tbs, &gPosition.latitude, &gPosition.longitude, 
      &gPosition.altitude, &gPosition.altitude);
  }
}

//*****************************************************************************
// Velocity
//*****************************************************************************

//*****************************************************************************
//*
//* 
//*
//*****************************************************************************

void TelemetrySource_Velocity::telemetryInit()
{
	topicSubscription = rosNodeHandle->subscribe( 
    sourceTopicName, 1, &TelemetrySource_Velocity::callback, this);
}

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

void TelemetrySource_Velocity::callback(const geometry_msgs::Vector3Stamped &msg)
{
  ros::Time startTime = ros::Time::now();
  ros::Duration elapsed_time = startTime - lastCallbackStartTime;

  // don't waste power collecting faster than we report
  if(elapsed_time > workerRosRate->cycleTime())
  {
    lastCallbackStartTime = startTime;

    int32_t tbs = getTimeBootMs(msg.header);

    vehicleTelemetry->telemGlobalPositionInt.setVelocity(
      &tbs, &msg.vector.x, &msg.vector.y, &msg.vector.z);
  }
}

//*****************************************************************************
// Attitude
//*****************************************************************************

//*****************************************************************************
//*
//* 
//*
//*****************************************************************************

void TelemetrySource_Attitude::telemetryInit()
{
	topicSubscription = rosNodeHandle->subscribe(
		"/dji_sdk/attitude", 1,
        &TelemetrySource_Attitude::callback, this);
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
	  /*mavlink_msg_attitude_pack(mavlinkSystemId, mavlinkComponentId, &mavlinkMsg, 
		microsSinceEpoch(), 1.2, 1.7, 3.14, 0.01, 0.02, 0.03);*/

    mavlink_msg_attitude_encode( 
      mavlinkSystemId, mavlinkComponentId, &mavlinkMsg,
      vehicleTelemetry->telemAttitude.getDataPtr());

		sendMavMessageToGcs(&mavlinkMsg);
    workerRosRate->sleep();
  }
}

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

void TelemetrySource_Attitude::callback(const geometry_msgs::QuaternionStamped &msg)
{
  ros::Time startTime = ros::Time::now();
  ros::Duration elapsed_time = startTime - lastCallbackStartTime;

  // don't waste power collecting faster than we report
  if(elapsed_time > workerRosRate->cycleTime())
  {
    lastCallbackStartTime = startTime;

    int32_t tbs = getTimeBootMs(msg.header);
    float wx = 0; //*** TODO
    float wy = 0;
    float wz = 0;

    vehicleTelemetry->telemAttitude.setQuaternion(
      &tbs, &msg.quaternion.x, &msg.quaternion.y, 
      &msg.quaternion.z, &msg.quaternion.w, &wx, &wy, &wz);
  }
}

//*****************************************************************************
// LocalPosition
//*****************************************************************************

//*****************************************************************************
//*
//* 
//*
//*****************************************************************************

void TelemetrySource_LocalPositionNed::telemetryInit()
{
	topicSubscription = rosNodeHandle->subscribe(
		"/dji_sdk/local_position", 1,
        &TelemetrySource_LocalPositionNed::callback, this);
}

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

void TelemetrySource_LocalPositionNed::telemetryRunWorker()
{
  ROS_INFO("TelemetrySource_LocalPositionNed : Worker Thread Started OK");

	//float position[6] = {};

  while (ros::ok())
  {
		/*mavlink_msg_local_position_ned_pack(mavlinkSystemId, mavlinkComponentId, 
			&mavlinkMsg, microsSinceEpoch(), 
			position[0], position[1], position[2],
			position[3], position[4], position[5]);*/
					
    mavlink_msg_local_position_ned_encode( 
      mavlinkSystemId, mavlinkComponentId, &mavlinkMsg,
      vehicleTelemetry->telemLocalPositionNed.getDataPtr());

	  sendMavMessageToGcs(&mavlinkMsg);
      workerRosRate->sleep();
  }
}

//*****************************************************************************
//*
//* 
//*
//******************************************************************************

void TelemetrySource_LocalPositionNed::callback(const geometry_msgs::PointStamped &msg)
{
  ros::Time startTime = ros::Time::now();
  ros::Duration elapsed_time = startTime - lastCallbackStartTime;

  // don't waste power collecting faster than we report
  if(elapsed_time > workerRosRate->cycleTime())
  {
    lastCallbackStartTime = startTime;

    int32_t tbs = getTimeBootMs(msg.header);
    float x = msg.point.x;
    float y = msg.point.y;
    float z = msg.point.z;

    vehicleTelemetry->telemLocalPositionNed.setPosition(
      &tbs, &x, &y, &z);
  }
}

}