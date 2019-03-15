#pragma once

#include <memory>
#include <mavlink/common/mavlink.h>
#include <functional>

//typedef std::function<int(const mavlink_message_t *)> mavMessageCallbackType;

class MavlinkMessageInfo
{
  public:

    typedef std::function<int(const mavlink_message_t *)> mavMessageCallbackType;
};

// found this but I dont think it's current https://subak.io/code/px4/Firmware/build_px4fmu-v2_default/build_git_version.h.html
#define FIRMWARE_BUILD_VERSION 0x9c2dd48814a6ade1

class VehicleInfo
{
  public:

    VehicleInfo();
    ~VehicleInfo();

    static int getMavlinkSystemId();
    static int getMavlinkComponentId();

    static MavlinkMessageInfo::mavMessageCallbackType getSendMavMessageCallback();

    static MavlinkMessageInfo::mavMessageCallbackType getAddMavMessageCallback();

    static uint8_t* getPx4GitVersion() 
    {
      static union 
      {
        unsigned long gvbul = FIRMWARE_BUILD_VERSION;
        uint8_t gvbui8[8];
      } xvert;

		  return xvert.gvbui8;   
    }

    void setSendMavMessageCallback(MavlinkMessageInfo::mavMessageCallbackType callback);

    void setGotMavMessageCallback(MavlinkMessageInfo::mavMessageCallbackType callback);

    int setMavlinkSystemId(int val);
    int setMavlinkComponentId(int val);


};

