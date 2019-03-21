/*
 * mavvehiclelib.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

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

class ParamsRet
{
 public:

   enum resultEnum {success, failure};
   resultEnum Result;
   std::string Description;

   ParamsRet(resultEnum result, std::string description )
   {
      Result = result;
      Description = description;
   }

   ParamsRet(resultEnum result )
   {
      Result = result;
      Description = "";
   }

   ParamsRet()
   {
      Result = resultEnum::success;
      Description = "";
   }
};

//*****************************************************************************
//*
//*****************************************************************************

class ParamsApp
{
 public:

   ParamsApp(){};
   ~ParamsApp(){};
   ParamsRet readParams(){return ParamsRet();};   

   bool verbose = false;

 private:

};

//*****************************************************************************
//*
//*****************************************************************************

class ParamsVehicleInterface
{
 public:

   ParamsVehicleInterface(){};
   ~ParamsVehicleInterface(){};
   ParamsRet readParams();   

   bool verbose = false;
   bool fakeVehicleConnection = true;

 private:

};

//*****************************************************************************
//*
//*****************************************************************************

class Params
{
 public:

   explicit Params(){ init(); };
   ~Params(){};
   ParamsRet readParams(std::shared_ptr<ros::NodeHandle> nodeHandle);

   std::shared_ptr<ParamsApp> App;
   std::shared_ptr<ParamsVehicleInterface> VehicleInterface;

 private:

   int init();
};

} /* namespace mav2dji*/