#pragma once 

/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <vehicle_Info.hpp>
#include <ivehicle_telemetry.hpp>
#include <vehicle_interface.hpp>
#include <thread>
#include <chrono>
#include <util.hpp>

// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/TimeReference.h>

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/CameraAction.h>
#include <dji_sdk/Gimbal.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

#include <sensors/sensor_attitude.hpp>
#include <sensors/sensor_global_position_int.hpp>
#include <sensors/sensor_local_position_ned.hpp>
#include <sensors/sensor_home_position.hpp>
#include <sensors/sensor_battery_state.hpp>

namespace mav2dji
{

class VehicleTelemetry;

//*****************************************************************************
//*
//*****************************************************************************

class TelemTrigger
{
  public:

    enum triggerTypeEnum {period, asAvail};
    triggerTypeEnum getTriggerType();
    int getTimeSpanMs();
    int getTimeSpanHz();
    std::shared_ptr<ros::Rate> getTimeSpanRate();

    explicit TelemTrigger();
    ~TelemTrigger();

    TelemTrigger(triggerTypeEnum triggerType, int timeSpanMs);

    TelemTrigger(int timeSpanMs);

  private:

    triggerTypeEnum triggerType_;
    int timeSpanMs_; 
};

//*****************************************************************************
//*
//*****************************************************************************

class TelemetrySource 
{
 public:


  TelemetrySource();
  ~TelemetrySource();

  TelemTrigger getTrigger();
  int sendMavMessageToGcs(const mavlink_message_t* msg);
  virtual int init(TelemTrigger trigger, std::shared_ptr<VehicleTelemetry> parent);
  virtual Util::OpRet startTelemetryAsync();
  static uint64_t microsSinceEpoch();
  int32_t getTimeBootMs(std_msgs::Header header);
  uint64_t getTimeBootMs();
  virtual Util::OpRet stopTelemetry();
  virtual void telemetryRunWorker() = 0;
  virtual void telemetryInit() = 0;

  std::shared_ptr<ros::Rate> workerRosRate;
  mavlink_message_t mavlinkMsg;
  ros::Time lastCallbackStartTime = ros::Time::now();
  ros::Subscriber topicSubscription;
  std::string sourceTopicName = "";
  int mavlinkSystemId = 1;      
  int mavlinkComponentId = 0;   
  std::shared_ptr<ros::NodeHandle> rosNodeHandle;
  std::shared_ptr<VehicleTelemetry> vehicleTelemetry;

  // vehicle state values

  //uint8_t landedState = MavLandedState::MavLandedStateOnGround;

  
 private:

  std::thread telemetryRunWorkerThread;
  TelemTrigger trigger_;
  uint64_t bootTime = microsSinceEpoch();
  MavlinkMessageInfo::mavMessageCallbackType sendMavMessageCallback;  
};

//*****************************************************************************
//*
//*****************************************************************************

class VehicleTelemetry : public iVehicleTelemetry
{
 public:

  typedef std::shared_ptr<Util::OpRet> TelemRet;

  SensorLocalPositionNed  telemLocalPositionNed;
  SensorAttitude          telemAttitude;
  SensorGlobalPositionInt telemGlobalPositionInt;
  SensorHomePosition      telemHomePosition;
  SensorBatteryStatus     telemBatteryStatus;

  explicit VehicleTelemetry();
  ~VehicleTelemetry();

  //int init();
  Util::OpRet addTelemetrySource(
    std::shared_ptr<TelemetrySource> telemSource, 
    TelemTrigger trigger, 
    std::shared_ptr<VehicleTelemetry> parent);
    
  Util::OpRet startTelemetrySourcesAsync();
  Util::OpRet stopTelemetrySources();

  void setBaseMode(uint8_t value);                      
  void setSystemStatus(uint8_t value); 
  void setCustomMode(uint32_t value);
  void setLandedState(uint8_t value);
  
  uint8_t getBaseMode();                   
  uint8_t getSystemStatus();  
  uint32_t getCustomMode();
  uint8_t getLandedState();

  void setArmed(bool isArmed);
  bool isArmed();
  
  void setCurrentAsHomePosition(const double* lat, 
    const double* lon, const double* alt);

  void setGpsHealth(uint8_t value);
  uint8_t getGpsHealth();
  
 private:

  std::vector<std::shared_ptr<TelemetrySource>> telemSources;

  // vehicle state values

  uint8_t landedState = MavLandedState::MavLandedStateOnGround;
  uint8_t baseMode = MavModeFlag::mavModeFlagCustomModeEnabled 
    + MavModeFlag::mavModeFlagAutoEnabled 
    + MavModeFlag::mavModeFlagGuidedEnabled 
    + MavModeFlag::mavModeFlagStabilizeEnabled 
    + MavModeFlag::mavModeFlagHilEnbaled 
    + MavModeFlag::mavModeFlagManualInputEnabled;                         
  uint8_t systemStatus = MavState::mavStateStandby;    
  uint32_t customMode = 50593800;
  uint8_t gpsHealth = 0;
};


} /* namespace mav2dji*/