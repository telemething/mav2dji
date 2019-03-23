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
#include <vehicle_telemetry.hpp>
#include <condition_variable>
#include <condition_variable>

namespace mav2dji
{

class vehicle 
{
 public:

  explicit vehicle(){};
  ~vehicle(){};

  int init();
  int startVehicle();
  int startVehicleAsync();
  int stopVehicle();
  int gotMavMessageCallback(const mavlink_message_t* msg);
  int sendMavMessageCallback(const mavlink_message_t* msg);

  int getMavlinkSystemId() { return mavlinkSystemId; }
  int getMavlinkComponentId() { return mavlinkComponentId; }
  int sendMavMessageToGcs(const mavlink_message_t* msg){ return udpConnection->sendMavMessageToGcs(msg);};
  bool isVehicleRunning();
   
 private:

  VehicleInfo vehicleInfo;
   
  int mavlinkSystemId;
  int mavlinkComponentId;
  int startVehicleRetCode;

  bool verbose = false;
  bool stopRunning = false;
  bool initHasCompleted = false;

  std::shared_ptr<mavvehiclelib::mav_udp> udpConnection;
  std::shared_ptr<mav_message> mavMessageProcessor;
  std::shared_ptr<vehicle_interface> vehicleInterface;
  std::shared_ptr<VehicleTelemetry> vehicleTelemetry;

  std::thread vehicleThread;

  void worker();
  mav2dji::TelemetryRet startTelemetry();
};

} /* namespace mav2dji*/