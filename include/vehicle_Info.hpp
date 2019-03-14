#pragma once

#include <mavlink/common/mavlink.h>
#include <functional>

//typedef std::function<int(const mavlink_message_t *)> mavMessageCallbackType;

class MavlinkMessageInfo
{
  public:

    typedef std::function<int(const mavlink_message_t *)> mavMessageCallbackType;
};

class VehicleInfo
{
  public:

    VehicleInfo();

    static int getMavlinkSystemId() { return 1; }
    static int getMavlinkComponentId() { return 1; }

  //  int setMavlinkSystemId(int val) { mavlinkSystemId = val; }
  //  int setMavlinkComponentId(int val) { mavlinkComponentId = val; }

  //private: 

  //  static int mavlinkSystemId;
  //  static int mavlinkComponentId;
};