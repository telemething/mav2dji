/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

#include <string>
#include <ros/ros.h>

namespace mav2dji
{

class vehicle_interface_ret
{
   public:

      enum resultEnum {success, failure};
      resultEnum Result;
      std::string Description;

      vehicle_interface_ret(resultEnum result, std::string description )
      {
         Result = result;
         Description = description;
      }

      vehicle_interface_ret(resultEnum result )
      {
         Result = result;
         Description = "";
      }

      vehicle_interface_ret()
      {
         Result = resultEnum::success;
         Description = "";
      }
};

class vehicle_interface 
{
 public:

    explicit vehicle_interface();
    ~vehicle_interface();

    std::shared_ptr<ros::NodeHandle> rosNodeHandle;

    virtual int init();
    virtual vehicle_interface_ret connectToPlatform() = 0;
    virtual vehicle_interface_ret activate() = 0;
    virtual vehicle_interface_ret startVehicleAsync() = 0;
    virtual vehicle_interface_ret stopVehicle() = 0;

    
};

} /* namespace mav2dji*/