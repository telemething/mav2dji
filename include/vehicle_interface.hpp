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

class vehicle_interface 
{
 public:

    explicit vehicle_interface();
    ~vehicle_interface();

    void init();

    void activate();

 private:
};

} /* namespace mav2dji*/