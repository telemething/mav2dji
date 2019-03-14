/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

#include <mavlink/common/mavlink.h>
#include <thread>
#include <mav_udp.hpp>
#include <mav_message_processor.hpp>

namespace mav2dji
{

class vehicle 
{
 public:

   explicit vehicle();
   ~vehicle();

   void startVehicle();
   void stopVehicle();
   int gotMavMessageCallback(const mavlink_message_t* msg);
   int sendMavMessageCallback(const mavlink_message_t* msg);

   int getMavlinkSystemId() { return mavlinkSystemId; }
   int getMavlinkComponentId() { return mavlinkComponentId; }
   int sendMavMessageToGcs(const mavlink_message_t* msg){ return udpConnection->sendMavMessageToGcs(msg);};

 private:

   int mavlinkSystemId;
   int mavlinkComponentId;

   bool verbose = false;
   std::shared_ptr<mavvehiclelib::mav_udp> udpConnection;
   std::shared_ptr<mav_message> mavMessageProcessor;

   void init();
};

} /* namespace mav2dji*/