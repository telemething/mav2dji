/*
 * mav2dji_ros.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <vehicle_telemetry_sources.hpp>
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

TelemetrySource_Heartbeat::TelemetrySource_Heartbeat(){};
TelemetrySource_Heartbeat::~TelemetrySource_Heartbeat(){};
void TelemetrySource_Heartbeat::telemetryInit(){};

void TelemetrySource_Heartbeat::telemetryRunWorker()
{
  ROS_INFO("TelemetrySource_Heartbeat : Worker Thread Started OK");

  MAV_TYPE mavType = MAV_TYPE_QUADROTOR;
  MAV_AUTOPILOT mavAutoPilot = MAV_AUTOPILOT_PX4;      

  while (ros::ok())
  {
		mavlink_msg_heartbeat_pack(
      mavlinkSystemId, 
      mavlinkComponentId, 
			&mavlinkMsg, 
      mavType,           
      mavAutoPilot,            
			vehicleTelemetry->getBaseMode(),     
      vehicleTelemetry->getCustomMode(), 
      vehicleTelemetry->getSystemStatus() );            
					
		sendMavMessageToGcs(&mavlinkMsg);
      workerRosRate->sleep();
  }
}

//*****************************************************************************
//* SystemStatus
//* 
//* onboard_control_sensors_present  Bitmap showing which onboard 
//*   controllers and sensors are present. Value of 0: not present. Value of 1: present.
//* onboard_control_sensors_enabled  Bitmap showing which onboard 
//*   controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.
//* onboard_control_sensors_health  Bitmap showing which onboard controllers 
//*   and sensors are operational or have an error:  Value of 0: not enabled. 
//*   Value of 1: enabled.
//* load [d%] Maximum usage in percent of the mainloop time. Values: 
//*   [0-1000] - should always be below 1000
//* voltage_battery [mV] Battery voltage
//* current_battery [cA] Battery current, -1: autopilot does not measure 
//*   the current
//* battery_remaining [%] Remaining battery energy, -1: autopilot estimate 
//*   the remaining battery
//* drop_rate_comm [c%] Communication drop rate, (UART, I2C, SPI, CAN), 
//*   dropped packets on all links (packets that were corrupted on reception on the MAV)
//* errors_comm  Communication errors (UART, I2C, SPI, CAN), dropped packets 
//*   on all links (packets that were corrupted on reception on the MAV)
//* errors_count1  Autopilot-specific errors
//* errors_count2  Autopilot-specific errors
//* errors_count3  Autopilot-specific errors
//* errors_count4  Autopilot-specific errors
//*
//******************************************************************************

TelemetrySource_SysStatus::TelemetrySource_SysStatus(){};
TelemetrySource_SysStatus::~TelemetrySource_SysStatus(){};
void TelemetrySource_SysStatus::telemetryInit(){};

void TelemetrySource_SysStatus::telemetryRunWorker()
{
  ROS_INFO("TelemetrySource_SysStatus : Worker Thread Started OK");

  mavlink_message_t msg;

  // These values came from a PX4 SITL
  uint32_t onboard_control_sensors_present  = 2359340;  // 1001000000000000101100
  uint32_t onboard_control_sensors_enabled  = 2097170;  // 1000000000000000010010
  uint32_t onboard_control_sensors_health   = 2359340;  // 1001000000000000101100
  uint16_t load                             = 0; 
  //uint16_t voltage_battery                  = 12149;
  //int16_t current_battery                   = -100;
  //int8_t battery_remaining                  = 100; 
  uint16_t drop_rate_comm                   = 0; 
  uint16_t errors_comm                      = 0;
  uint16_t errors_count1                    = 0; 
  uint16_t errors_count2                    = 0;
  uint16_t errors_count3                    = 0; 
  uint16_t errors_count4                    = 0;

  while (ros::ok())
  {				
    auto battery = vehicleTelemetry->telemBatteryStatus.getSysBattStatus();

		mavlink_msg_sys_status_pack(mavlinkSystemId, mavlinkComponentId, 
			&msg,onboard_control_sensors_present,
      onboard_control_sensors_enabled,
      onboard_control_sensors_health,
      load,
      battery->voltageMv,
      battery->currentCa,
      battery->capacityRemaining,
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

TelemetrySource_HomePosition::TelemetrySource_HomePosition(){};
TelemetrySource_HomePosition::~TelemetrySource_HomePosition(){};
void TelemetrySource_HomePosition::telemetryInit(){};

void TelemetrySource_HomePosition::telemetryRunWorker()
{
  ROS_INFO("TelemetrySource_HomePosition : Worker Thread Started OK");

  mavlink_message_t msg;
  
  while (ros::ok())
  {				
    auto pos = vehicleTelemetry->telemGlobalPositionInt.getDataPtr();

    // if not valid then we have no gps data
    if(vehicleTelemetry->telemHomePosition.isValid())
    {
      uint64_t time_usec = getTimeBootMs();
      auto pos = vehicleTelemetry->telemHomePosition.getDataPtr();

      mavlink_msg_home_position_pack(
        mavlinkSystemId, 
        mavlinkComponentId, 
        &msg,
        pos->latitude, 
        pos->longitude, 
        pos->altitude, 
        pos->x, pos->y, pos->z, pos->q, 
        pos->approach_x, 
        pos->approach_y, 
        pos->approach_z, 
        time_usec);
            
      sendMavMessageToGcs(&msg);
    }
        
    workerRosRate->sleep();
  }
}

//*****************************************************************************
//* ExtendedSysState
//* 
//* vtol_state  The VTOL state if applicable. Is set to 
//*   MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
//* landed_state  The landed state. Is set to MAV_LANDED_STATE_UNDEFINED 
//*   if landed state is unknown.
//*
//* 0	MAV_LANDED_STATE_UNDEFINED	
//* 1	MAV_LANDED_STATE_ON_GROUND	
//* 2	MAV_LANDED_STATE_IN_AIR	    
//* 3	MAV_LANDED_STATE_TAKEOFF	  
//* 4	MAV_LANDED_STATE_LANDING
//*
//******************************************************************************

TelemetrySource_ExtendedSysState::TelemetrySource_ExtendedSysState(){};
TelemetrySource_ExtendedSysState::~TelemetrySource_ExtendedSysState(){};
void TelemetrySource_ExtendedSysState::telemetryInit(){};

void TelemetrySource_ExtendedSysState::telemetryRunWorker()
{
  ROS_INFO("TelemetrySource_ExtendedSysState : Worker Thread Started OK");

  mavlink_message_t msg;

  uint8_t vtol_state    = 0;
  uint8_t landed_state  = vehicleTelemetry->getLandedState(); 

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

TelemetrySource_GlobalPositionInt::TelemetrySource_GlobalPositionInt()
  {sourceTopicName = "/dji_sdk/gps_position";};

TelemetrySource_GlobalPositionInt::~TelemetrySource_GlobalPositionInt(){};

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

    double latitude = 47.4684818;           //*** TODO
    double longitude = -121.76819669999999;
    double altitude = 174;
    double height = 5;

    vehicleTelemetry->telemGlobalPositionInt.setLatLonAlt(
      &tbs, &latitude, &longitude, &altitude, &height);

    vehicleTelemetry->telemHomePosition.setPosition(
      &latitude, &longitude, &altitude);
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

    // set the home position to the first gps coord we see
    if(!vehicleTelemetry->telemHomePosition.isValid())
      vehicleTelemetry->telemHomePosition.setPosition(&gPosition.latitude, 
        &gPosition.longitude, &gPosition.altitude);
  }
}

//*****************************************************************************
// Velocity
//*****************************************************************************

TelemetrySource_Velocity::TelemetrySource_Velocity()
  {sourceTopicName = "/dji_sdk/velocity";};
TelemetrySource_Velocity::~TelemetrySource_Velocity(){};
void TelemetrySource_Velocity::telemetryRunWorker(){};

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

TelemetrySource_Attitude::TelemetrySource_Attitude()
  {sourceTopicName = "/dji_sdk/attitude";};

TelemetrySource_Attitude::~TelemetrySource_Attitude(){};

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

TelemetrySource_LocalPositionNed::TelemetrySource_LocalPositionNed()
  {sourceTopicName = "/dji_sdk/local_position";};

TelemetrySource_LocalPositionNed::~TelemetrySource_LocalPositionNed(){};

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

//*****************************************************************************
//*
//* Battery State
//*
//******************************************************************************

TelemetrySource_BatteryState::TelemetrySource_BatteryState()
{sourceTopicName = "/dji_sdk/battery_state";};

TelemetrySource_BatteryState::~TelemetrySource_BatteryState(){};

void TelemetrySource_BatteryState::telemetryInit()
{
	topicSubscription = rosNodeHandle->subscribe(
		sourceTopicName, 1,
        &TelemetrySource_BatteryState::callback, this);
}

void TelemetrySource_BatteryState::telemetryRunWorker(){};

void TelemetrySource_BatteryState::callback(const sensor_msgs::BatteryState &msg)
{
  ros::Time startTime = ros::Time::now();
  ros::Duration elapsed_time = startTime - lastCallbackStartTime;

  // don't waste power collecting faster than we report
  if(elapsed_time > workerRosRate->cycleTime())
  {
    lastCallbackStartTime = startTime;
    
    vehicleTelemetry->telemBatteryStatus.setState(
      msg.location, msg.voltage, msg.current, msg.percentage);
    //msg.voltage;
    //msg.current;
    //msg.charge;
    //msg.capacity;
    //msg.design_capacity;
    //msg.percentage;
    //msg.power_supply_status;
    //msg.power_supply_health;
    //msg.power_supply_technology;
    //msg.present;
    //msg.cell_voltage;
    //msg.location;
    //msg.serial_number;  
  }
}

//*****************************************************************************
//*
//* Flight Status
//*
//* DJI values from  
//* https://github.com/dji-sdk/Onboard-SDK-ROS/blob/3.7/dji_sdk/include/dji_sdk/dji_sdk.h:
//*
//* STATUS_STOPPED   = DJI::OSDK::VehicleStatus::FlightStatus::STOPED
//* STATUS_ON_GROUND = DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND
//* STATUS_IN_AIR    = DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR
//*
//* Mavlink values:
//*
//* MAV_LANDED_STATE_UNDEFINED	= 0
//* MAV_LANDED_STATE_ON_GROUND	= 1
//* MAV_LANDED_STATE_IN_AIR	    = 2
//* MAV_LANDED_STATE_TAKEOFF	  = 3
//* MAV_LANDED_STATE_LANDING    = 4
//*
//******************************************************************************

TelemetrySource_FlightStatus::TelemetrySource_FlightStatus()
      {sourceTopicName = "/dji_sdk/flight_status";};

TelemetrySource_FlightStatus::~TelemetrySource_FlightStatus(){};

void TelemetrySource_FlightStatus::telemetryInit()
{
	topicSubscription = rosNodeHandle->subscribe(
		sourceTopicName, 1,
        &TelemetrySource_FlightStatus::callback, this);
}

void TelemetrySource_FlightStatus::telemetryRunWorker(){};

void TelemetrySource_FlightStatus::callback(const std_msgs::UInt8 &msg)
{
  ros::Time startTime = ros::Time::now();
  ros::Duration elapsed_time = startTime - lastCallbackStartTime;

  MavLandedState_t mavLandedState = MavLandedState_t::MavLandedStateUndefined;

  // don't waste power collecting faster than we report
  if(elapsed_time > workerRosRate->cycleTime())
  {
    lastCallbackStartTime = startTime;

    switch(msg.data)
    {
      case DjiFlightStatus_t::STOPED :
        mavLandedState = MavLandedState_t::MavLandedStateUndefined;
      break;
      case DjiFlightStatus_t::ON_GROUND :
        mavLandedState = MavLandedState_t::MavLandedStateOnGround;
      break;
      case DjiFlightStatus_t::IN_AIR :
        mavLandedState = MavLandedState_t::MavLandedStateInAir;
      break;
    }

    // TODO : we need more processing to get five states from three states

    // TODO : do we need to update baseMode or customMode? 

    vehicleTelemetry->setLandedState(mavLandedState); 
  }
}

//*****************************************************************************
//*
//* Flight Status
//*
//* DJI values from  
//* https://github.com/dji-sdk/Onboard-SDK-ROS/blob/3.7/dji_sdk/include/dji_sdk/dji_sdk.h:
//*
//* This mode requires the user to manually control the aircraft to remain stable in air. 
//* MODE_MANUAL_CTRL=DJI::OSDK::VehicleStatus::DisplayMode::MODE_MANUAL_CTRL
//*
//* In this mode, the aircraft can keep attitude stabilization and only use the
//* barometer for positioning to control the altitude. 
//*
//* The aircraft can not autonomously locate and hover stably.
//* MODE_ATTITUDE=DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE
//*
//* The aircraft is in normal GPS mode. In normal GPS mode, the aircraft can
//* autonomously locate and hover stably. The sensitivity of the aircraft to the
//* command response is moderate.
//* MODE_P_GPS=DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS
//*
//* In hotpoint mode 
//* MODE_HOTPOINT_MODE=DJI::OSDK::VehicleStatus::DisplayMode::MODE_HOTPOINT_MODE
//*
//* In this mode, user can push the throttle stick to complete stable take-off. 
//* MODE_ASSISTED_TAKEOFF=DJI::OSDK::VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF
//*
//* In this mode, the aircraft will autonomously start motor, ascend and finally hover. 
//* MODE_AUTO_TAKEOFF=DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF
//*
//* In this mode, the aircraft can land autonomously. 
//* MODE_AUTO_LANDING=DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_LANDING
//*
//* In this mode, the aircraft can antonomously return the last recorded Home Point. 
//* There are three types of this mode: Smart RTH(Return-to-Home), Low Batterry RTH, and Failsafe RTTH.  
//* MODE_NAVI_GO_HOME=DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_GO_HOME
//*
//* In this mode, the aircraft is controled by SDK API. User can directly define the control mode of horizon
//* and vertical directions and send control datas to aircraft. 
//* MODE_NAVI_SDK_CTRL=DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL
//*
//* drone is forced to land, might due to low battery 
//* MODE_FORCE_AUTO_LANDING=DJI::OSDK::VehicleStatus::DisplayMode::MODE_FORCE_AUTO_LANDING
//*
//* drone will search for the last position where the rc is not lost 
//* MODE_SEARCH_MODE =DJI::OSDK::VehicleStatus::DisplayMode::MODE_SEARCH_MODE
//*
//* Mode for motor starting. Every time user unlock the motor, this will be the first mode. 
//* MODE_ENGINE_START = DJI::OSDK::VehicleStatus::DisplayMode::MODE_ENGINE_START
//*
//* Mavlink values:
//*
//* ???
//*
//******************************************************************************

TelemetrySource_DisplayMode::TelemetrySource_DisplayMode()
    {sourceTopicName = "/dji_sdk/display_mode";};
TelemetrySource_DisplayMode::~TelemetrySource_DisplayMode(){};

void TelemetrySource_DisplayMode::telemetryInit()
{
	topicSubscription = rosNodeHandle->subscribe(
		sourceTopicName, 1,
        &TelemetrySource_DisplayMode::callback, this);
}

void TelemetrySource_DisplayMode::telemetryRunWorker(){};

void TelemetrySource_DisplayMode::callback(const std_msgs::UInt8 &msg)
{
  if(currentDjiDisplayMode == msg.data )
  {
    //mode did not change, no need to act
    return;
  }

  currentDjiDisplayMode = static_cast<DjiDispalyMode_t>(msg.data);

  ros::Time startTime = ros::Time::now();
  ros::Duration elapsed_time = startTime - lastCallbackStartTime;

  //MavLandedState_t mavLandedState = MavLandedState_t::MavLandedStateUndefined;
  
  // don't waste power collecting faster than we report
  if(elapsed_time > workerRosRate->cycleTime())
  {
    lastCallbackStartTime = startTime;

    switch(msg.data)
    {
      case DjiDispalyMode_t::MODE_ASSISTED_TAKEOFF :
        printf("--- DJI DisplayMode : MODE_ASSISTED_TAKEOFF\r\n");
      break;
      case DjiDispalyMode_t::MODE_ATTITUDE :
        printf("--- DJI DisplayMode : MODE_ATTITUDE\r\n");
      break;
      case DjiDispalyMode_t::MODE_AUTO_LANDING :
        printf("--- DJI DisplayMode : MODE_AUTO_LANDING\r\n");
      break;
      case DjiDispalyMode_t::MODE_AUTO_TAKEOFF :
        printf("--- DJI DisplayMode : MODE_AUTO_TAKEOFF\r\n");
      break;
      case DjiDispalyMode_t::MODE_ENGINE_START :
        printf("--- DJI DisplayMode : MODE_ENGINE_START\r\n");
      break;
      case DjiDispalyMode_t::MODE_FORCE_AUTO_LANDING :
        printf("--- DJI DisplayMode : MODE_FORCE_AUTO_LANDING\r\n");
      break;
      case DjiDispalyMode_t::MODE_HOTPOINT_MODE :
        printf("--- DJI DisplayMode : MODE_HOTPOINT_MODE\r\n");
      break;
      case DjiDispalyMode_t::MODE_MANUAL_CTRL :
        printf("--- DJI DisplayMode : MODE_MANUAL_CTRL\r\n");
      break;
      case DjiDispalyMode_t::MODE_NAVI_GO_HOME :
        printf("--- DJI DisplayMode : MODE_NAVI_GO_HOME\r\n");
      break;
      case DjiDispalyMode_t::MODE_NAVI_SDK_CTRL :
        printf("--- DJI DisplayMode : MODE_NAVI_SDK_CTRL\r\n");
      break;
      case DjiDispalyMode_t::MODE_P_GPS :
        printf("--- DJI DisplayMode : MODE_P_GPS\r\n");
      break;
      case DjiDispalyMode_t::MODE_SEARCH_MODE :
        printf("--- DJI DisplayMode : MODE_SEARCH_MODE\r\n");
      break;
    }

    // TODO : we need more processing 

    //vehicleTelemetry->setLandedState(mavLandedState); 
    //vehicleTelemetry->setBaseMode(something); 
    //vehicleTelemetry->setCustomMode(something); 
  }
}

//*****************************************************************************
//*
//* Height above takeoff
//*
//******************************************************************************

TelemetrySource_HeightAboveTakeoff::TelemetrySource_HeightAboveTakeoff()
      {sourceTopicName = "/dji_sdk/height_above_takeoff";};

TelemetrySource_HeightAboveTakeoff::~TelemetrySource_HeightAboveTakeoff(){};

void TelemetrySource_HeightAboveTakeoff::telemetryInit()
{
	topicSubscription = rosNodeHandle->subscribe(
		sourceTopicName, 1,
        &TelemetrySource_HeightAboveTakeoff::callback, this);
}

void TelemetrySource_HeightAboveTakeoff::telemetryRunWorker(){};

void TelemetrySource_HeightAboveTakeoff::callback(const std_msgs::Float32 &msg)
{
  ros::Time startTime = ros::Time::now();
  ros::Duration elapsed_time = startTime - lastCallbackStartTime;

    // don't waste power collecting faster than we report
  if(elapsed_time > workerRosRate->cycleTime())
  {
    lastCallbackStartTime = startTime;
  }

  //msg.data; // TODO * is this useful?
}

} // namespace mav2dji 
