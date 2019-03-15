/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

#include <vehicle_interface.hpp>

namespace mav2dji
{

class vehicle_interface_djiros : public vehicle_interface
{
 public:

    explicit vehicle_interface_djiros();
    ~vehicle_interface_djiros();

    void init();

    //void activate();

 private:
};

} /* namespace mav2dji*/