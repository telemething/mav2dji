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
#include <util.hpp>

namespace mav2dji
{

class vehicle_interface 
{
 public:

    explicit vehicle_interface();
    ~vehicle_interface();

    std::shared_ptr<ros::NodeHandle> rosNodeHandle;

    virtual int init();
    virtual Util::OpRet connectToPlatform() = 0;
    virtual Util::OpRet activate() = 0;
    virtual Util::OpRet startVehicleAsync() = 0;
    virtual Util::OpRet stopVehicle() = 0;

    
};

} /* namespace mav2dji*/