/*
 * mavvehiclelib.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

#include <vehicle_Info.hpp>
#include <thread>      
#include <mutex> 
#include <arpa/inet.h>

namespace mavvehiclelib
{

// this needs to be at least MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_CHECKSUM_BYTES 
//    + 7 + MAVLINK_SIGNATURE_BLOCK_LEN + 14. Let's round way up. 
#define UDP_BUFFER_LENGTH 2041

//*****************************************************************************
//*
//*****************************************************************************

class MavUdpRet
{
   public:

      enum resultEnum {success, failure};
      resultEnum Result;
      std::string Description;

      MavUdpRet(resultEnum result, std::string description )
      {
         Result = result;
         Description = description;
      }

      MavUdpRet(resultEnum result )
      {
         Result = result;
         Description = "";
      }

      MavUdpRet()
      {
         Result = resultEnum::success;
         Description = "";
      }
};

//*****************************************************************************
//*
//*****************************************************************************

class mav_udp
{
 public:

    //typedef std::function<int(const mavlink_message_t *)> mavMessageCallbackType;

    explicit mav_udp();
    explicit mav_udp(MavlinkMessageInfo::mavMessageCallbackType callback);
    ~mav_udp();

    int getMavlinkSystemId() { return mavlinkSystemId; }
    int getMavlinkComponentId() { return mavlinkComponentId; }

    MavUdpRet startConnection();
    void stopConnection();

    void addGotMavMessageCallback(MavlinkMessageInfo::mavMessageCallbackType callback);
    int sendMavMessageToGcs(const mavlink_message_t* pMsg);
    
 private:

      std::string vehicleUdpAddress = "";
      int vehicleUdpPort = 14551;
      std::string qgcUdpAddress = "";
      int qgcUdpPort = 14550;
      int mavlinkSystemId = 1;
      int mavlinkComponentId = 0;
      int vehicleMavlinkSocket = 0;
      struct sockaddr_in gcSockAddr; 
      bool verbose;
      uint8_t udpSendBuffer[UDP_BUFFER_LENGTH];

      MavlinkMessageInfo::mavMessageCallbackType mavMessageCallback;

      std::thread listenWorkerThread;
      std::thread sendWorkerThread;
      std::mutex sendMavMessageToGcsMutex; 
      bool listenWorkerThreadShouldRun = false;
      bool sendWorkerThreadShouldRun = false;
      uint64_t microsSinceEpoch();
      uint8_t *px4_git_version_binary;

      void init();
      void parseMavlink(uint8_t chan, uint8_t* c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status);
      int createSocket(std::string localAddress, int localPort, bool blocking);
      void listenWorker(int sock, std::string fromAddress, int fromPort);
      //void exampleLoop(int sock, std::string toAddress, int toPort);
};

} /* namespace mavvehicle*/