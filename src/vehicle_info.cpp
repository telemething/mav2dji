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

void VehicleInfo::setSendMavMessageCallback(MavlinkMessageInfo::mavMessageCallbackType callback)
{ singleton->sendMavMessageCallback = callback; }

void VehicleInfo::setGotMavMessageCallback(MavlinkMessageInfo::mavMessageCallbackType callback)
{ singleton->gotMavMessageCallback = callback; }

int VehicleInfo::setMavlinkSystemId(int val) 
{ singleton->mavlinkSystemId = val; }

int VehicleInfo::setMavlinkComponentId(int val) 
{ singleton->mavlinkComponentId = val; }


//*** statics, for consumers

int VehicleInfo::getMavlinkSystemId() 
{ return singleton->mavlinkSystemId; }

int VehicleInfo::getMavlinkComponentId() 
{ return singleton->mavlinkComponentId; }

MavlinkMessageInfo::mavMessageCallbackType VehicleInfo::getSendMavMessageCallback()
{ return singleton->sendMavMessageCallback; }

MavlinkMessageInfo::mavMessageCallbackType VehicleInfo::getAddMavMessageCallback()
{ return singleton->gotMavMessageCallback; }
