#pragma once 

/*
 * mavvehiclelib.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

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

   ParamsRet(resultEnum result, std::string description );
   ParamsRet(resultEnum result );
   ParamsRet();
};

//*****************************************************************************
//*
//*****************************************************************************

class ParamsMavlink
{
 public:

  ParamsMavlink();
  ~ParamsMavlink();
  ParamsRet readParams();

  std::string vehicleUdpAddress = "";
  int vehicleUdpPort = 14551;
  std::string qgcUdpAddress = "192.168.1.12";
  int qgcUdpPort = 14550;
  int mavlinkSystemId = 1;
  int mavlinkComponentId = 0;

 private:

};

//*****************************************************************************
//*
//*****************************************************************************

class ParamsApp
{
 public:

   ParamsApp();
   ~ParamsApp();
   ParamsRet readParams();

   bool verbose = false;

 private:

};

//*****************************************************************************
//*
//*****************************************************************************

class ParamsVehicleInterface
{
 public:

   ParamsVehicleInterface();
   ~ParamsVehicleInterface();
   ParamsRet readParams();   

   bool verbose = false;
   bool fakeVehicleConnection = false;
   bool fakeTelemtry = false;

 private:

};

//*****************************************************************************
//*
//*****************************************************************************

class Params
{
 public:

   explicit Params();
   ~Params();
   ParamsRet readParams(std::shared_ptr<ros::NodeHandle> nodeHandle);

   std::shared_ptr<ParamsApp> App;
   std::shared_ptr<ParamsVehicleInterface> VehicleInterface;
   std::shared_ptr<ParamsMavlink> Mavlink;

 private:

   int init();
};

} /* namespace mav2dji*/