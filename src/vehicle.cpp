/*
 * mav2dji_ros.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <vehicle.hpp>

//*****************************************************************************
//*
//*
//*
//******************************************************************************

namespace mav2dji 
{

//*****************************************************************************
//*
//*
//*
//******************************************************************************

vehicle::vehicle()
{
  init();
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

vehicle::~vehicle()
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void vehicle::init()
{
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void vehicle::startVehicle()
{
    //udpConnection = std::make_shared<mavvehiclelib::mav_udp>(std::bind(
    //  &vehicle::vehicleMavMessageCallback, this, std::placeholders::_1));

    udpConnection = std::make_shared<mavvehiclelib::mav_udp>();
    //mission_manager = std::make_unique<mav2dji_mission>(rosNodeHandle, mavvehicle_);

    mavMessageProcessor  = std::make_shared<mav_message>();

    udpConnection->addMavMessageCallback(std::bind(
      &vehicle::vehicleMavMessageCallback, this, std::placeholders::_1));

    //px4_git_version_binary = udpConnection->getGitVersion();

    udpConnection->startVehicle();
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void vehicle::stopVehicle()
{
    udpConnection->stopVehicle();
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int vehicle::vehicleMavMessageCallback(const mavlink_message_t* msg)
{
	mavMessageProcessor->printMavMessageInfo(msg, "Mavlink Message", false);
  mavMessageProcessor->ProcessMavMessage(msg);
  return 0;
}

}
