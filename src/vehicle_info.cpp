/*
 * mav2dji_ros.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <vehicle_Info.hpp>

class ffxxcc
{
  public:
    MavlinkMessageInfo::mavMessageCallbackType sendMavMessageCallback;
    MavlinkMessageInfo::mavMessageCallbackType gotMavMessageCallback;
    std::shared_ptr<mav2dji::vehicle_interface> vehicleInterface;
    int mavlinkSystemId;
    int mavlinkComponentId;
};

static std::shared_ptr<ffxxcc> singleton;

//*****************************************************************************
//*
//*
//*
//******************************************************************************

VehicleInfo::VehicleInfo()
{
    if( nullptr == singleton )
        singleton = std::make_shared<ffxxcc>();
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

VehicleInfo::~VehicleInfo()
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

// instance members for setters

void VehicleInfo::setSendMavMessageCallback(MavlinkMessageInfo::mavMessageCallbackType callback)
{ singleton->sendMavMessageCallback = callback; }

void VehicleInfo::setGotMavMessageCallback(MavlinkMessageInfo::mavMessageCallbackType callback)
{ singleton->gotMavMessageCallback = callback; }

void VehicleInfo::setVehicleInterface(std::shared_ptr<mav2dji::vehicle_interface> vehicleInterface)
{ singleton->vehicleInterface = vehicleInterface; }

int VehicleInfo::setMavlinkSystemId(int val) 
{ singleton->mavlinkSystemId = val; }

int VehicleInfo::setMavlinkComponentId(int val) 
{ singleton->mavlinkComponentId = val; }

// static accessors for consumers

int VehicleInfo::getMavlinkSystemId() 
{ return singleton->mavlinkSystemId; }

int VehicleInfo::getMavlinkComponentId() 
{ return singleton->mavlinkComponentId; }

MavlinkMessageInfo::mavMessageCallbackType VehicleInfo::getSendMavMessageCallback()
{ return singleton->sendMavMessageCallback; }

MavlinkMessageInfo::mavMessageCallbackType VehicleInfo::getAddMavMessageCallback()
{ return singleton->gotMavMessageCallback; }

uint8_t* VehicleInfo::getPx4GitVersion() 
{
    static union 
    {
        unsigned long gvbul = FIRMWARE_BUILD_VERSION;
        uint8_t gvbui8[8];
    } xvert;

    return xvert.gvbui8;   
}
