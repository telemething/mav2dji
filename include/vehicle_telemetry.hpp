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

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/CameraAction.h>
#include <dji_sdk/Gimbal.h>

#include <sensor_msgs/NavSatFix.h>

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
//*****************************************************************************

class TelemetrySource_GlobalPositionInt : public TelemetrySource
{
 public:

   explicit TelemetrySource_GlobalPositionInt(){};
   ~TelemetrySource_GlobalPositionInt(){};

 private:

    void telemetryInit();
    void telemetryRunWorker();
    void callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
};

//*****************************************************************************
//*
//*****************************************************************************

class TelemetrySource_Attitude : public TelemetrySource
{
 public:

   explicit TelemetrySource_Attitude(){};
   ~TelemetrySource_Attitude(){};

 private:

   void telemetryInit();
   void telemetryRunWorker();
   void callback(const geometry_msgs::QuaternionStamped &msg);
};

//*****************************************************************************
//*
//*****************************************************************************

class TelemetrySource_LocalPositionNed : public TelemetrySource
{
 public:

   explicit TelemetrySource_LocalPositionNed(){};
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
//*****************************************************************************

class TelemetrySource_Velocity : public TelemetrySource
{
 public:

   explicit TelemetrySource_Velocity(){};
   ~TelemetrySource_Velocity(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const geometry_msgs::Vector3Stamped &msg);
};

} /* namespace mav2dji*/