/*
 * mavvehiclelib.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once
#include <mavlink/common/mavlink.h>
#include <thread>

namespace mavvehiclelib
{

class mavvehicle
{
 public:

    explicit mavvehicle();
    ~mavvehicle();

    void startVehicle();
    void stopVehicle();

 private:

   std::string vehicleUdpAddress = "";
   int vehicleUdpPort = 14551;
   std::string qgcUdpAddress = "";
   int qgcUdpPort = 14550;
   int mavlinkSystemId = 1;
	int mavlinkComponentId = 1;

   std::thread listenWorkerThread;
   std::thread sendWorkerThread;
   bool listenWorkerThreadShouldRun = false;
   bool sendWorkerThreadShouldRun = false;
   uint64_t microsSinceEpoch();

   void init();
   int createSocket(std::string localAddress, int localPort);
   void listenWorker(int sock, std::string fromAddress, int fromPort);
   void exampleLoop(int sock);
};

} /* namespace mavvehicle*/