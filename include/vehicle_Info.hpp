#pragma once

#include <memory>
#include <functional>
#include <mavlink/common/mavlink.h>
#include <vehicle_interface_djiros.hpp>
#include <params.hpp>

//typedef std::function<int(const mavlink_message_t *)> mavMessageCallbackType;

class MavlinkMessageInfo
{
  public:

    typedef std::function<int(const mavlink_message_t *)> mavMessageCallbackType;
};

// found this but I dont think it's current 
// https://subak.io/code/px4/Firmware/build_px4fmu-v2_default/build_git_version.h.html
#define FIRMWARE_BUILD_VERSION 0x9c2dd48814a6ade1

class VehicleInfo
{
  public:

    VehicleInfo();
    ~VehicleInfo();

    // static accessors for consumers

    static int getMavlinkSystemId();
    static int getMavlinkComponentId();
    static MavlinkMessageInfo::mavMessageCallbackType getSendMavMessageCallback();
    static MavlinkMessageInfo::mavMessageCallbackType getAddMavMessageCallback();
    static std::shared_ptr<mav2dji::vehicle_interface> getVehicleInterface();
    static uint8_t* getPx4GitVersion();
    static std::shared_ptr<mav2dji::Params> params;

    // instance members for setters

    int setMavlinkSystemId(int val);
    int setMavlinkComponentId(int val);
    void setSendMavMessageCallback(MavlinkMessageInfo::mavMessageCallbackType callback);
    void setGotMavMessageCallback(MavlinkMessageInfo::mavMessageCallbackType callback);
    void setVehicleInterface(
      std::shared_ptr<mav2dji::vehicle_interface> vehicleInterface);
    mav2dji::ParamsRet readParams();

  private:

    std::shared_ptr<mav2dji::Params> params_;
};

