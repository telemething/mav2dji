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

int vehicle::init()
{
    mavlinkSystemId = 1;
    mavlinkComponentId = 1;

    // register the vehicle's mavlink system and component IDs
    vehicleInfo.setMavlinkSystemId(mavlinkSystemId);
    vehicleInfo.setMavlinkComponentId(mavlinkComponentId);

    // register the 'send a mav message' callback
    vehicleInfo.setSendMavMessageCallback( std::bind(
    &vehicle::sendMavMessageCallback, this, std::placeholders::_1));

    // regsiter the 'got mav message' callback
    vehicleInfo.setGotMavMessageCallback( std::bind(
    &vehicle::gotMavMessageCallback, this, std::placeholders::_1));

    // create an instance of the vehicle specific interface class, cast as the 
    // interface base class

    vehicleInterface = std::make_shared<vehicle_interface_djiros>();
    vehicleInfo.setVehicleInterface(vehicleInterface);

    //vehicleInfo.setVehicleInterface( std::static_pointer_cast<vehicle_interface>
    //  (std::make_shared<vehicle_interface_djiros>()));
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int vehicle::startVehicle()
{
    udpConnection = std::make_shared<mavvehiclelib::mav_udp>();
    mavMessageProcessor  = std::make_shared<mav_message>();

    udpConnection->addGotMavMessageCallback(std::bind(
      &vehicle::gotMavMessageCallback, this, std::placeholders::_1));

    vehicle_interface_ret ret = vehicleInterface->activate();  

    if( ret.Result != vehicle_interface_ret::resultEnum::success )
    {
      printf("\n\n\n###### vehicle::startVehicle() Exception : %s ######\n\n\n", 
        ret.Description.c_str() );
      
      stopVehicle();

      return -1;
    }

    udpConnection->startConnection();

    //vehicleTelemetry = std::make_shared<vehicle_telemetry>();

    //vehicleTelemetry->addTelemetrySource(
    //  std::make_shared<telemetry_source_global_position_int>(), 
    //  telemetry_source::Trigger(telemetry_source::Trigger::triggeTypeEnum::period, 1000) );

    return 0;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int vehicle::stopVehicle()
{
    if(nullptr != udpConnection)
      udpConnection->stopConnection();
    
    if(nullptr != vehicleInterface)
      vehicleInterface->stopVehicle();

    stopRunning = true;
    return 0;
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

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

bool vehicle::isVehicleRunning()
{
  return !stopRunning;
}

}
