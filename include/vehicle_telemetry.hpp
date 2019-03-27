/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

#include <vehicle_Info.hpp>
#include <vehicle_interface.hpp>
#include <thread>
#include <chrono>

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

namespace mav2dji
{

class TelemetryRet;
class VehicleTelemetry;
class TelemetrySource;

//*****************************************************************************
//*
//*****************************************************************************

class TelemetryRet
{
   public:

      enum resultEnum {success, failure};
      resultEnum Result;
      std::string Description;

      TelemetryRet(resultEnum result, std::string description )
      {
         Result = result;
         Description = description;
      }

      TelemetryRet(resultEnum result )
      {
         Result = result;
         Description = "";
      }

      TelemetryRet()
      {
         Result = resultEnum::success;
         Description = "";
      }

      ~TelemetryRet()
      {      }
};

//*****************************************************************************
//*
//*****************************************************************************

class TelemetrySource 
{
 public:

   //typedef std::function<void()> telemetryInitType;
   //typedef std::function<void()> telemetryRunWorkerType;

   class Trigger
   {
      public:

        enum triggerTypeEnum {period, asAvail};
        triggerTypeEnum getTriggerType(){return triggerType_;};
        int getTimeSpanMs(){return timeSpanMs_;}; 
        int getTimeSpanHz(){return 1000.0/(double)timeSpanMs_;}; 
        std::shared_ptr<ros::Rate> getTimeSpanRate()
        { return std::make_shared<ros::Rate>(1000.0/(double)timeSpanMs_); }

        explicit Trigger(){};
        ~Trigger(){};

        Trigger(triggerTypeEnum triggerType, int timeSpanMs)
        {
            triggerType_ = triggerType;
            timeSpanMs_ = timeSpanMs;
        }

        Trigger(int timeSpanMs)
        {
            triggerType_ = triggerTypeEnum::period;
            timeSpanMs_ = timeSpanMs;
        }

      private:

        triggerTypeEnum triggerType_;
        int timeSpanMs_; 
   };

   TelemetrySource(){};
   ~TelemetrySource(){};

   Trigger getTrigger(){ return trigger_;}

   int sendMavMessageToGcs(const mavlink_message_t* msg)
   { return sendMavMessageCallback(msg); };

   //void setTelemetryInit(telemetryInitType telemetryInit)
   //{ telemetryInit_ = telemetryInit; }

   //void setSelf(TelemetrySource)

   //void setTelemetryWorker(telemetryRunWorkerType telemetryRunWorker)
   //{ telemetryRunWorker_ = telemetryRunWorker; }

   virtual int init(Trigger trigger, std::shared_ptr<VehicleTelemetry> parent)
   { 
      vehicleTelemetry = parent;
      rosNodeHandle = VehicleInfo::getVehicleInterface()->rosNodeHandle;
      trigger_ = trigger; 
      workerRosRate = trigger_.getTimeSpanRate(); 
      
      //telemetryInit_();
      telemetryInit();
   };

   virtual TelemetryRet startTelemetryAsync()
   {
      mavlinkSystemId = VehicleInfo::getMavlinkSystemId();
      mavlinkComponentId = VehicleInfo::getMavlinkComponentId();
      sendMavMessageCallback = VehicleInfo::getSendMavMessageCallback();

      //telemetryRunWorkerThread = std::thread(telemetryRunWorker_);
      telemetryRunWorkerThread = std::thread(&TelemetrySource::telemetryRunWorker, this);
      return TelemetryRet(TelemetryRet::resultEnum::success);    
   }

   static uint64_t microsSinceEpoch()
   {
      struct timeval tv;
      
      uint64_t micros = 0;
      
      gettimeofday(&tv, NULL);  
      micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
      
      return micros;
   }

   int32_t getTimeBootMs(std_msgs::Header header)
   {
      return header.stamp.sec + header.stamp.nsec * 1000000;
   }
    
   virtual TelemetryRet stopTelemetry(){};
   std::shared_ptr<ros::Rate> workerRosRate;
   mavlink_message_t mavlinkMsg;
   ros::Time lastCallbackStartTime = ros::Time::now();
   ros::Subscriber topicSubscription;
   std::string sourceTopicName = "";

   int mavlinkSystemId = 1;      
   int mavlinkComponentId = 0;   
   std::shared_ptr<ros::NodeHandle> rosNodeHandle;
   std::shared_ptr<VehicleTelemetry> vehicleTelemetry;

   virtual void telemetryRunWorker() = 0;
   virtual void telemetryInit() = 0;

 private:

   std::thread telemetryRunWorkerThread;
   Trigger trigger_;
   //telemetryInitType telemetryInit_;
   //telemetryRunWorkerType telemetryRunWorker_;
   MavlinkMessageInfo::mavMessageCallbackType sendMavMessageCallback;  
};

//*****************************************************************************
//*
//*****************************************************************************

class VehicleTelemetry 
{
 public:

   typedef std::shared_ptr<TelemetryRet> TelemRet;

   SensorLocalPositionNed  telemLocalPositionNed;
   SensorAttitude          telemAttitude;
   SensorGlobalPositionInt telemGlobalPositionInt;

   explicit VehicleTelemetry(){};
   ~VehicleTelemetry(){};

    //int init();
   TelemetryRet addTelemetrySource(
        std::shared_ptr<TelemetrySource> telemSource, 
        TelemetrySource::Trigger trigger, 
        std::shared_ptr<VehicleTelemetry> parent)
   {
      telemSource->init(trigger, parent);
      telemSources.push_back(telemSource);

      return TelemetryRet();
   };
    
   TelemetryRet startTelemetrySourcesAsync()
   {
      for(auto ts : telemSources)
      {
         ts->startTelemetryAsync();
      }

      return TelemetryRet();
   }

   TelemetryRet stopTelemetrySources()
   {
      for(auto ts : telemSources)
      {
         ts->stopTelemetry();
      }

      return TelemetryRet();
   }

 private:

    std::vector<std::shared_ptr<TelemetrySource>> telemSources;

};

//*****************************************************************************
//*
//* /dji_sdk/gps_position (sensor_msgs/NavSatFix) 50 hz
//*
//*****************************************************************************

class TelemetrySource_GlobalPositionInt : public TelemetrySource
{
 public:

   explicit TelemetrySource_GlobalPositionInt()
      {sourceTopicName = "/dji_sdk/gps_position";};
   ~TelemetrySource_GlobalPositionInt(){};

 private:

   void telemetryInit();
   void telemetryRunWorker();
   void callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
};

//*****************************************************************************
//*
//* /dji_sdk/attitude (geometry_msgs/QuaternionStamped) rotation from FLU body 
//*   frame to ENU ground frame, 100 Hz. 
//*
//*****************************************************************************

class TelemetrySource_Attitude : public TelemetrySource
{
 public:

   explicit TelemetrySource_Attitude()
      {sourceTopicName = "/dji_sdk/attitude";};
   ~TelemetrySource_Attitude(){};

 private:

   void telemetryInit();
   void telemetryRunWorker();
   void callback(const geometry_msgs::QuaternionStamped &msg);
};

//*****************************************************************************
//*
//* /dji_sdk/local_position (geometry_msgs/PointStamped) Cartesian ENU frame, 
//*   set origin calling the /dji_sdk/set_local_pos_ref 
//*
//*****************************************************************************

class TelemetrySource_LocalPositionNed : public TelemetrySource
{
 public:

   explicit TelemetrySource_LocalPositionNed()
      {sourceTopicName = "/dji_sdk/local_position";};
   ~TelemetrySource_LocalPositionNed(){};

 private:

   void telemetryInit();
   void telemetryRunWorker();
   void callback(const geometry_msgs::PointStamped &msg);
};

//*****************************************************************************
//*
//*****************************************************************************

class TelemetrySource_Heartbeat : public TelemetrySource
{
 public:

   explicit TelemetrySource_Heartbeat(){};
   ~TelemetrySource_Heartbeat(){};

 private:

   void telemetryInit(){};
   void telemetryRunWorker();
};

//*****************************************************************************
//*
//*****************************************************************************

class TelemetrySource_SysStatus : public TelemetrySource
{
 public:

   explicit TelemetrySource_SysStatus(){};
   ~TelemetrySource_SysStatus(){};

 private:

   void telemetryInit(){};
   void telemetryRunWorker();
};


//*****************************************************************************
//*
//* /dji_sdk/velocity (geometry_msgs/Vector3Stamped)
//*
//*****************************************************************************

class TelemetrySource_Velocity : public TelemetrySource
{
 public:

   explicit TelemetrySource_Velocity()
      {sourceTopicName = "/dji_sdk/velocity";};
   ~TelemetrySource_Velocity(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const geometry_msgs::Vector3Stamped &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//* /dji_sdk/acceleration_ground_fused (geometry_msgs/Vector3Stamped) Fused 
//*   acceleration with respect to East-North-Up (ENU) ground frame, 100 Hz
//*
//*****************************************************************************

class TelemetrySource_AccelerationGroundFused : public TelemetrySource
{
 public:

   explicit TelemetrySource_AccelerationGroundFused()
      {sourceTopicName = "/dji_sdk/acceleration_ground_fused";};
   ~TelemetrySource_AccelerationGroundFused(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const geometry_msgs::Vector3Stamped &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//* /dji_sdk/angular_velocity_fused (geometry_msgs/Vector3Stamped) Fused 
//    angular rate (p,q,r) around Forward-Left-Up (FLU) body frame, 100 Hz
//*
//*****************************************************************************

class TelemetrySource_AngularVelocityFused : public TelemetrySource
{
 public:

   explicit TelemetrySource_AngularVelocityFused()
      {sourceTopicName = "/dji_sdk/angular_velocity_fused";};
   ~TelemetrySource_AngularVelocityFused(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const geometry_msgs::Vector3Stamped &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//* /dji_sdk/battery_state (sensor_msgs/BatteryState) 10 hz
//*
//*****************************************************************************

class TelemetrySource_BatteryState : public TelemetrySource
{
 public:

   explicit TelemetrySource_BatteryState()
      {sourceTopicName = "/dji_sdk/battery_state";};
   ~TelemetrySource_BatteryState(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const sensor_msgs::BatteryState &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//* dji_sdk/display_mode (std_msgs/UInt8)
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
//*****************************************************************************

class TelemetrySource_DisplayMode : public TelemetrySource
{
 public:

   explicit TelemetrySource_DisplayMode()
      {sourceTopicName = "/dji_sdk/display_mode";};
   ~TelemetrySource_DisplayMode(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const std_msgs::UInt8 &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//*  /dji_sdk/flight_status (std_msgs/UInt8)
//*
//*  STATUS_STOPPED   = DJI::OSDK::VehicleStatus::FlightStatus::STOPED
//*  STATUS_ON_GROUND = DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND
//*  STATUS_IN_AIR    = DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR
//*
//*****************************************************************************

class TelemetrySource_FlightStatus : public TelemetrySource
{
 public:

   explicit TelemetrySource_FlightStatus()
      {sourceTopicName = "/dji_sdk/flight_status";};
   ~TelemetrySource_FlightStatus(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const std_msgs::UInt8 &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//*  /dji_sdk/from_mobile_data (std_msgs/UInt8[])
//*
//*****************************************************************************

class TelemetrySource_FromMobileData : public TelemetrySource
{
 public:

   explicit TelemetrySource_FromMobileData()
      {sourceTopicName = "/dji_sdk/from_mobile_data";};
   ~TelemetrySource_FromMobileData(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const std_msgs::UInt8 &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//*  /dji_sdk/gimbal_angle (geometry_msgs/Vector3Stamped)
//*
//*****************************************************************************

class TelemetrySource_GimbalAngle : public TelemetrySource
{
 public:

   explicit TelemetrySource_GimbalAngle()
      {sourceTopicName = "/dji_sdk/gimbal_angle";};
   ~TelemetrySource_GimbalAngle(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const geometry_msgs::Vector3Stamped &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//*  /dji_sdk/gps_health (std_msgs/UInt8) strength 0-5, 5 is best, < 3 is bad
//*
//*****************************************************************************

class TelemetrySource_GpsHealth : public TelemetrySource
{
 public:

   explicit TelemetrySource_GpsHealth()
      {sourceTopicName = "/dji_sdk/gps_health";};
   ~TelemetrySource_GpsHealth(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const std_msgs::UInt8 &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//*  /dji_sdk/imu (sensor_msgs/Imu) 400 hz
//*
//*****************************************************************************

class TelemetrySource_Imu : public TelemetrySource
{
 public:

   explicit TelemetrySource_Imu()
      {sourceTopicName = "/dji_sdk/imu";};
   ~TelemetrySource_Imu(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const sensor_msgs::Imu &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//*  /dji_sdk/rc 0=roll, 1=pitch, 2=yaw, 3=throttle, 4=mode, 5=landing gear
//*
//*****************************************************************************

class TelemetrySource_Rc : public TelemetrySource
{
 public:

   explicit TelemetrySource_Rc()
      {sourceTopicName = "/dji_sdk/rc";};
   ~TelemetrySource_Rc(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const sensor_msgs::Joy &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//*  /dji_sdk/height_above_takeoff (std_msgs/Float32) valid after drone is 
//*  armed, after reference altitude set. 
//*
//*****************************************************************************

class TelemetrySource_HeightAboveTakeoff : public TelemetrySource
{
 public:

   explicit TelemetrySource_HeightAboveTakeoff()
      {sourceTopicName = "/dji_sdk/height_above_takeoff";};
   ~TelemetrySource_HeightAboveTakeoff(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const std_msgs::Float32 &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//*  /dji_sdk/trigger_time (sensor_msgs/TimeReference)
//*
//*****************************************************************************

class TelemetrySource_TriggerTime : public TelemetrySource
{
 public:

   explicit TelemetrySource_TriggerTime()
      {sourceTopicName = "/dji_sdk/trigger_time";};
   ~TelemetrySource_TriggerTime(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const sensor_msgs::TimeReference &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//*  /dji_sdk/local_position (geometry_msgs/PointStamped) Cartesian ENU frame, 
//   set origin calling the /dji_sdk/set_local_pos_ref 
//*
//*****************************************************************************

class TelemetrySource_LocalPosition : public TelemetrySource
{
 public:

   explicit TelemetrySource_LocalPosition()
      {sourceTopicName = "/dji_sdk/local_position";};
   ~TelemetrySource_LocalPosition(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const geometry_msgs::PointStamped &msg);
};


/*

* /dji_sdk/acceleration_ground_fused (geometry_msgs/Vector3Stamped) Fused acceleration with respect to East-North-Up (ENU) ground frame, 100 Hz
* /dji_sdk/angular_velocity_fused (geometry_msgs/Vector3Stamped) Fused angular rate (p,q,r) around Forward-Left-Up (FLU) body frame, 100 Hz
/dji_sdk/attitude (geometry_msgs/QuaternionStamped) rotation from FLU body frame to ENU ground frame, 100 Hz. 
* /dji_sdk/battery_state (sensor_msgs/BatteryState) 10 hz
* /dji_sdk/display_mode (std_msgs/UInt8)
   // This mode requires the user to manually control the aircraft to remain stable in air. 
   MODE_MANUAL_CTRL=DJI::OSDK::VehicleStatus::DisplayMode::MODE_MANUAL_CTRL,
   // In this mode, the aircraft can keep attitude stabilization and only use the
   // barometer for positioning to control the altitude. <br>
   // The aircraft can not autonomously locate and hover stably.
   MODE_ATTITUDE=DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE,
   // The aircraft is in normal GPS mode. In normal GPS mode, the aircraft can
   // autonomously locate and hover stably. The sensitivity of the aircraft to the
   //  command response is moderate.
   MODE_P_GPS=DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS,
   // In hotpoint mode 
   MODE_HOTPOINT_MODE=DJI::OSDK::VehicleStatus::DisplayMode::MODE_HOTPOINT_MODE,
   // In this mode, user can push the throttle stick to complete stable take-off. 
   MODE_ASSISTED_TAKEOFF=DJI::OSDK::VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF,
   // In this mode, the aircraft will autonomously start motor, ascend and finally hover. 
   MODE_AUTO_TAKEOFF=DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF,
   // In this mode, the aircraft can land autonomously. 
   MODE_AUTO_LANDING=DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_LANDING,
   // In this mode, the aircraft can antonomously return the last recorded Home Point. <br>
   // There are three types of this mode: Smart RTH(Return-to-Home), Low Batterry RTH, and Failsafe RTTH.  
   MODE_NAVI_GO_HOME=DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_GO_HOME,
   // In this mode, the aircraft is controled by SDK API. User can directly define the control mode of horizon
   // and vertical directions and send control datas to aircraft. 
   MODE_NAVI_SDK_CTRL=DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL,
   // drone is forced to land, might due to low battery 
   MODE_FORCE_AUTO_LANDING=DJI::OSDK::VehicleStatus::DisplayMode::MODE_FORCE_AUTO_LANDING,
   // drone will search for the last position where the rc is not lost 
   MODE_SEARCH_MODE =DJI::OSDK::VehicleStatus::DisplayMode::MODE_SEARCH_MODE,
   // Mode for motor starting. Every time user unlock the motor, this will be the first mode. 
   MODE_ENGINE_START = DJI::OSDK::VehicleStatus::DisplayMode::MODE_ENGINE_START

* /dji_sdk/flight_status (std_msgs/UInt8)
  STATUS_STOPPED   = DJI::OSDK::VehicleStatus::FlightStatus::STOPED,
  STATUS_ON_GROUND = DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND,
  STATUS_IN_AIR    = DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR

- /dji_sdk/from_mobile_data (std_msgs/UInt8[])
* /dji_sdk/gimbal_angle (geometry_msgs/Vector3Stamped)
* /dji_sdk/gps_health (std_msgs/UInt8) strength 0-5, 5 is best, < 3 is bad
/dji_sdk/gps_position (sensor_msgs/NavSatFix) 50 hz
* /dji_sdk/imu (sensor_msgs/Imu) 400 hz
* /dji_sdk/rc 0=roll, 1=pitch, 2=yaw, 3=throttle, 4=mode, 5=landing gear
/dji_sdk/velocity (geometry_msgs/Vector3Stamped)
* /dji_sdk/height_above_takeoff (std_msgs/Float32) valid after drone is armed, after reference altitude set. 
- /dji_sdk/trigger_time (sensor_msgs/TimeReference)
- /dji_sdk/local_position (geometry_msgs/PointStamped) Cartesian ENU frame, set origin calling the /dji_sdk/set_local_pos_ref 
- /dji_sdk/rtk_position (sensor_msgs/NavSatFix)
- /dji_sdk/rtk_velocity (geometry_msgs/Vector3)
- /dji_sdk/rtk_yaw (std_msgs/Int16)
- /dji_sdk/rtk_info_position (std_msgs/UInt8)
- /dji_sdk/rtk_info_yaw (std_msgs/UInt8)

- = didnt see
* = saw but dont service

*/

} /* namespace mav2dji*/