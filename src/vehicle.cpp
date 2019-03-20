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

  mav2dji::ParamsRet ret = vehicleInfo.readParams();

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

  vehicleTelemetry = std::make_shared<VehicleTelemetry>();
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

  auto ret = vehicleInterface->activate();  

  if( ret.Result != vehicle_interface_ret::resultEnum::success )
  {
    printf("\n\n\n###### vehicle::startVehicle() Exception : %s ######\n\n\n", 
      ret.Description.c_str() );
      
    stopVehicle();

    return -1;
  }

  auto ret2 = udpConnection->startConnection();  //*** TODO * Inspect ret

  auto ret3 = startTelemetry();    //*** TODO * Inspect ret

  return 0;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int vehicle::startTelemetry()
{

  auto ret = vehicleTelemetry->addTelemetrySource(
    std::make_shared<TelemetrySource_GlobalPositionInt>(), 
    TelemetrySource::Trigger(1000) );

  ret = vehicleTelemetry->addTelemetrySource(
    std::make_shared<TelemetrySource_Attitude>(), 
    TelemetrySource::Trigger(1000) );

  ret = vehicleTelemetry->addTelemetrySource(
    std::make_shared<TelemetrySource_LocalPositionNed>(), 
    TelemetrySource::Trigger(1000) );

  ret = vehicleTelemetry->addTelemetrySource(
    std::make_shared<TelemetrySource_Heartbeat>(), 
    TelemetrySource::Trigger(1000) );

  ret = vehicleTelemetry->addTelemetrySource(
    std::make_shared<TelemetrySource_SysStatus>(), 
    TelemetrySource::Trigger(1000) );

  ret = vehicleTelemetry->startTelemetrySourcesAsync(); //*** TODO * Inspect ret

  return 0;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int vehicle::stopVehicle()
{
  if(nullptr != vehicleTelemetry)
    vehicleTelemetry->stopTelemetrySources();
    
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
