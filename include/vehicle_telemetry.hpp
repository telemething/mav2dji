/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

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

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/CameraAction.h>
#include <dji_sdk/Gimbal.h>

namespace mav2dji
{

//*****************************************************************************
//*
//*****************************************************************************

class telemetry_interface_ret
{
   public:

      enum resultEnum {success, failure};
      resultEnum Result;
      std::string Description;

      telemetry_interface_ret(resultEnum result, std::string description )
      {
         Result = result;
         Description = description;
      }

      telemetry_interface_ret(resultEnum result )
      {
         Result = result;
         Description = "";
      }
};

//*****************************************************************************
//*
//*****************************************************************************

class telemetry_source 
{
 public:

    class Trigger
    {
      public:

        enum triggeTypeEnum {period, asAvail};
        triggeTypeEnum getTriggeType(){return triggeType_;};
        int getTimeSpanMs(){return timeSpanMs_;}; 

        explicit Trigger(){};
        ~Trigger(){};

        Trigger(triggeTypeEnum triggeType, int timeSpanMs)
        {
            triggeType_ = triggeType;
            timeSpanMs_ = timeSpanMs;
        }

      private:

        triggeTypeEnum triggeType_;
        int timeSpanMs_; 
    };

    telemetry_source(){};
    ~telemetry_source(){};

    virtual int init(Trigger trigger){trigger_ = trigger;};
    virtual telemetry_interface_ret startTelemetryAsync()
    {
        ROS_INFO("telemetry_interface::startVehicleAsync() : Starting Worker Thread");

        telemetryRunWorkerThread = std::thread(
            &telemetry_source::telemetryRunWorker, this);

        return telemetry_interface_ret(telemetry_interface_ret::resultEnum::success);    
    }
    
    virtual telemetry_interface_ret stopTelemetry(){};
    virtual void telemetryRunWorker(){};

 private:

    std::thread telemetryRunWorkerThread;
    Trigger trigger_;
    
};

//*****************************************************************************
//*
//*****************************************************************************

class vehicle_telemetry 
{
 public:

   explicit vehicle_telemetry();
   ~vehicle_telemetry();

    //int init();
   telemetry_interface_ret addTelemetrySource(
        telemetry_source* telemSource, telemetry_source::Trigger trigger)
   {
      telemSource->init(trigger);
      telemSources.push_back(telemSource);
   };
    
   telemetry_interface_ret startTelemetrySourceAsync()
   {
      for(telemetry_source* ts : telemSources)
      {
         ts->startTelemetryAsync();
      }
   }

   telemetry_interface_ret stopTelemetrySource()
   {
      for(telemetry_source* ts : telemSources)
      {
         ts->stopTelemetry();
      }
   }

    
 private:

    std::vector<telemetry_source*> telemSources;

};

//*****************************************************************************
//*
//*****************************************************************************

class telemetry_source_global_position_int : public telemetry_source
{
 public:

    explicit telemetry_source_global_position_int();
    ~telemetry_source_global_position_int();

    int init(Trigger trigger){ telemetry_source::init(trigger); };
    telemetry_interface_ret startTelemetryAsync();
    telemetry_interface_ret stopTelemetry();

 private:

    std::thread telemetryRunWorkerThread;
    void telemetryRunWorker();
};




} /* namespace mav2dji*/