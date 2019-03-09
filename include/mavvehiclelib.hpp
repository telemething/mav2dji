/*
 * mavvehiclelib.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once
#include <mavlink/common/mavlink.h>
#include <functional>
#include <thread>

namespace mavvehiclelib
{

/*class mavvehicleclient
{
  public:

   virtual int vehicleMavMessageCallback(int arg)
   { return arg; }
};*/

class mavvehicle
{
 public:

   typedef std::function<int(mavlink_message_t)> mavMessageCallbackType;

   explicit mavvehicle();
   explicit mavvehicle(mavMessageCallbackType callback);
   ~mavvehicle();

   void startVehicle();
   void stopVehicle();

   void addMavMessageCallback(mavMessageCallbackType callback);
    
 private:

   std::string vehicleUdpAddress = "";
   int vehicleUdpPort = 14551;
   std::string qgcUdpAddress = "";
   int qgcUdpPort = 14550;
   int mavlinkSystemId = 1;
	int mavlinkComponentId = 1;
   bool verbose;

   mavMessageCallbackType mavMessageCallback;

   std::thread listenWorkerThread;
   std::thread sendWorkerThread;
   bool listenWorkerThreadShouldRun = false;
   bool sendWorkerThreadShouldRun = false;
   uint64_t microsSinceEpoch();

   void init();
   int createSocket(std::string localAddress, int localPort, bool blocking);
   void listenWorker(int sock, std::string fromAddress, int fromPort);
   void exampleLoop(int sock, std::string toAddress, int toPort);
};

} /* namespace mavvehicle*/