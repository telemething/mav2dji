/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

namespace mav2dji
{

class vehicle_ros_interface 
{
 public:

    explicit vehicle_ros_interface();
    ~vehicle_ros_interface();

    void init();

 private:
};

} /* namespace mav2dji*/