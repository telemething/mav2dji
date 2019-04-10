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
//* type            2           MAV_TYPE_QUADROTOR
//* autopilot       12          MAV_AUTOPILOT_PX4
//* base_mode       157          11101
//* custom_mode     6.7371e+07 
//* system_status   4           MAV_STATE_STANDBY
//* mavlin_version  3
//*
//* type            2           MAV_TYPE_QUADROTOR
//* autopilot       12          MAV_AUTOPILOT_PX4
//* base_mode       157          11101
//* custom_mode     8.41482e+07 
//* system_status   4           MAV_STATE_STANDBY
//* mavlin_version  3
//*
//* type            2           MAV_TYPE_QUADROTOR
//* autopilot       12          MAV_AUTOPILOT_PX4
//* base_mode       157          11101
//* custom_mode     8.41482e+07 
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
//* SystemStatus
//* 
//* @param system_id ID of this system
//* @param component_id ID of this component (e.g. 200 for IMU)
//* @param msg The MAVLink message to compress the data into
//*
//* @param onboard_control_sensors_present  Bitmap showing which onboard 
//*   controllers and sensors are present. Value of 0: not present. Value of 1: present.
//* @param onboard_control_sensors_enabled  Bitmap showing which onboard 
//*   controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.
//* @param onboard_control_sensors_health  Bitmap showing which onboard controllers 
//*   and sensors are operational or have an error:  Value of 0: not enabled. 
//*   Value of 1: enabled.
//* @param load [d%] Maximum usage in percent of the mainloop time. Values: 
//*   [0-1000] - should always be below 1000
//* @param voltage_battery [mV] Battery voltage
//* @param current_battery [cA] Battery current, -1: autopilot does not measure 
//*   the current
//* @param battery_remaining [%] Remaining battery energy, -1: autopilot estimate 
//*   the remaining battery
//* @param drop_rate_comm [c%] Communication drop rate, (UART, I2C, SPI, CAN), 
//*   dropped packets on all links (packets that were corrupted on reception on the MAV)
//* @param errors_comm  Communication errors (UART, I2C, SPI, CAN), dropped packets 
//*   on all links (packets that were corrupted on reception on the MAV)
//* @param errors_count1  Autopilot-specific errors
//* @param errors_count2  Autopilot-specific errors
//* @param errors_count3  Autopilot-specific errors
//* @param errors_count4  Autopilot-specific errors
//*
//******************************************************************************

void TelemetrySource_SysStatus::telemetryRunWorker()
{
  ROS_INFO("TelemetrySource_SysStatus : Worker Thread Started OK");

  mavlink_message_t msg;

  // These values came from a PX4 SITL
  uint32_t onboard_control_sensors_present  = 2359340;  // 1001000000000000101100
  uint32_t onboard_control_sensors_enabled  = 2097170;  // 1000000000000000010010
  uint32_t onboard_control_sensors_health   = 2359340;  // 1001000000000000101100
  uint16_t load                             = 0; 
  uint16_t voltage_battery                  = 12149;
  int16_t current_battery                   = -100;
  int8_t battery_remaining                  = 100; 
  uint16_t drop_rate_comm                   = 0; 
  uint16_t errors_comm                      = 0;
  uint16_t errors_count1                    = 0; 
  uint16_t errors_count2                    = 0;
  uint16_t errors_count3                    = 0; 
  uint16_t errors_count4                    = 0;

  while (ros::ok())
  {				
		mavlink_msg_sys_status_pack(mavlinkSystemId, mavlinkComponentId, 
			&msg,onboard_control_sensors_present,
      onboard_control_sensors_enabled,
      onboard_control_sensors_health,
      load,voltage_battery,
      current_battery,
      battery_remaining,
      drop_rate_comm,
      errors_comm,
      errors_count1,
      errors_count2,
      errors_count3,
      errors_count4);
					
		sendMavMessageToGcs(&msg);
        workerRosRate->sleep();
  }
}

//*****************************************************************************
//* HomePosition
//* 
//* @param system_id ID of this system
//* @param component_id ID of this component (e.g. 200 for IMU)
//* @param msg The MAVLink message to compress the data into
//*
//* @param latitude [degE7] Latitude (WGS84)
//* @param longitude [degE7] Longitude (WGS84)
//* @param altitude [mm] Altitude (MSL). Positive for up.
//* @param x [m] Local X position of this position in the local coordinate frame
//* @param y [m] Local Y position of this position in the local coordinate frame
//* @param z [m] Local Z position of this position in the local coordinate frame
//* @param q  World to surface normal and heading transformation of the takeoff 
//*   position. Used to indicate the heading and slope of the ground
//* @param approach_x [m] Local X position of the end of the approach vector. 
//*   Multicopters should set this position based on their takeoff path. Grass-landing 
//*   fixed wing aircraft should set it the same way as multicopters. Runway-landing 
//*   fixed wing aircraft should set it to the opposite direction of the takeoff, 
//*   assuming the takeoff happened from the threshold / touchdown zone.
//* @param approach_y [m] Local Y position of the end of the approach vector. 
//*   Multicopters should set this position based on their takeoff path. Grass-landing 
//*   fixed wing aircraft should set it the same way as multicopters. Runway-landing 
//*   fixed wing aircraft should set it to the opposite direction of the takeoff, 
//*   assuming the takeoff happened from the threshold / touchdown zone.
//* @param approach_z [m] Local Z position of the end of the approach vector. 
//*   Multicopters should set this position based on their takeoff path. Grass-landing 
//*   fixed wing aircraft should set it the same way as multicopters. Runway-landing 
//*   fixed wing aircraft should set it to the opposite direction of the takeoff, 
//*   assuming the takeoff happened from the threshold / touchdown zone.
//* @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). 
//*   The receiving end can infer timestamp format (since 1.1.1970 or since system 
//*   boot) by checking for the magnitude the number.
//*
//******************************************************************************

void TelemetrySource_HomePosition::telemetryRunWorker()
{
  ROS_INFO("TelemetrySource_HomePosition : Worker Thread Started OK");

  mavlink_message_t msg;

  int32_t latitude = 47.4684818 * 1e7;            //*** TODO * Need to set from GPS before liftoff
  int32_t longitude = -121.76819669999999 * 1e7;  //*** TODO * Need to set from GPS before liftoff
  int32_t altitude = 174;
  float x = 0;
  float y = 0;
  float z = 0;
  float q[4] = {1,0,0,0};
  float approach_x = 0;
  float approach_y = 0;
  float approach_z = 0;
  uint64_t time_usec = getTimeBootMs();

  while (ros::ok())
  {				
		mavlink_msg_home_position_pack(
      mavlinkSystemId, 
      mavlinkComponentId, 
			&msg,
      latitude, 
      longitude, 
      altitude, 
      x, y, z, q, 
      approach_x, 
      approach_y, 
      approach_z, 
      time_usec);
					
		sendMavMessageToGcs(&msg);
        workerRosRate->sleep();
  }
}

//*****************************************************************************
//* ExtendedSysState
//* 
//* @param system_id ID of this system
//* @param component_id ID of this component (e.g. 200 for IMU)
//* @param msg The MAVLink message to compress the data into
//*
//* @param vtol_state  The VTOL state if applicable. Is set to 
//*   MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
//* @param landed_state  The landed state. Is set to MAV_LANDED_STATE_UNDEFINED 
//*   if landed state is unknown.
//*
//******************************************************************************

void TelemetrySource_ExtendedSysState::telemetryRunWorker()
{
  ROS_INFO("TelemetrySource_ExtendedSysState : Worker Thread Started OK");

  mavlink_message_t msg;

  // These values came from a PX4 SITL
  uint8_t vtol_state    = 0;
  uint8_t landed_state  = 1; 

  while (ros::ok())
  {			
		mavlink_msg_extended_sys_state_pack(mavlinkSystemId, mavlinkComponentId, 
			&msg,
      vtol_state,
      landed_state );
					
		sendMavMessageToGcs(&msg);
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