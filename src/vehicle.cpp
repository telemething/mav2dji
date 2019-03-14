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
    int mavlinkSystemId = 1;
    int mavlinkComponentId = 1;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void vehicle::startVehicle()
{
    udpConnection = std::make_shared<mavvehiclelib::mav_udp>();

    mavMessageProcessor  = std::make_shared<mav_message>();

    udpConnection->addGotMavMessageCallback(std::bind(
      &vehicle::gotMavMessageCallback, this, std::placeholders::_1));

    mavMessageProcessor->addSendMavMessageCallback(std::bind(
      &vehicle::sendMavMessageCallback, this, std::placeholders::_1));

    //px4_git_version_binary = udpConnection->getGitVersion();

    udpConnection->startConnection();
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void vehicle::stopVehicle()
{
    udpConnection->stopConnection();
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int vehicle::gotMavMessageCallback(const mavlink_message_t* msg)
{
	mavMessageProcessor->printMavMessageInfo(msg, "Got Mavlink", false);
  mavMessageProcessor->ProcessMavMessage(msg);
  return 0;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int vehicle::sendMavMessageCallback(const mavlink_message_t* msg)
{
	mavMessageProcessor->printMavMessageInfo(msg, "Sent Mavlink", false);
  return udpConnection->sendMavMessageToGcs(msg);
}

}
