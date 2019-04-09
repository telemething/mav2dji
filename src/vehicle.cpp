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

int vehicle::init()
{
  try
  {
    mavlinkSystemId = 1;
    mavlinkComponentId = 1;

    //mav2dji::ParamsRet ret = vehicleInfo.readParams();

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
    vehicleInterface = std::make_shared<VehicleInterfaceDjiros>();
    vehicleInfo.setVehicleInterface(vehicleInterface);

    vehicleTelemetry = std::make_shared<VehicleTelemetry>();
  }
  catch(const std::exception& e)
  {
    printf("\n\n\n###### vehicle::init() Exception : %s ######\n\n\n", 
    e.what() );    
    stopVehicle();
    return -1;
  }
  catch(...)
  {
    printf("\n\n\n###### vehicle::init() Uncategorized Exception ######\n\n\n" );    
    stopVehicle();
    return -1;
  }

  return 0;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int vehicle::startVehicleAsync()
{
  auto timeout = std::chrono::milliseconds(50);

  // start the vehicle on its own thread
  vehicleThread = std::thread(&vehicle::worker, this);

  // wait for a signal that startup has completed
  while(!initHasCompleted)
    std::this_thread::sleep_for(timeout);

  // return startup retcode
  return startVehicleRetCode;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

void vehicle::worker()
{
  // start the vehicle 
  startVehicleRetCode = startVehicle();

  // send the signal that startup has completed
  initHasCompleted = true;

  // we must spin on the thread which created the node
  ros::spin();
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int vehicle::startVehicle()
{
  try
  {
    auto ret = vehicleInterface->connectToPlatform();

    if(!ret.Result == Util::OpRet::resultEnum::success)
    {
      printf("\n\n\n###### vehicle::startVehicle() Exception : Unable to connect to platform : %s ######\n\n\n", 
        ret.Description.c_str() );
      stopVehicle();
      return -1;
    }

    udpConnection = std::make_shared<mavvehiclelib::mav_udp>();
    mavMessageProcessor  = std::make_shared<mav_message>();

    udpConnection->addGotMavMessageCallback(std::bind(
      &vehicle::gotMavMessageCallback, this, std::placeholders::_1));

    ret = vehicleInterface->activate();  

    if( ret.Result != Util::OpRet::resultEnum::success )
    {
      printf("\n\n\n###### vehicle::startVehicle() Exception : Unable to start vehicle interface : %s ######\n\n\n", 
        ret.Description.c_str() );
      stopVehicle();
      return -1;
    }

    auto ret2 = udpConnection->startConnection();  

    if( ret2.Result != mavvehiclelib::MavUdpRet::resultEnum::success )
    {
      printf("\n\n\n###### vehicle::startVehicle() Exception : Unable to start UDP connection : %s ######\n\n\n", 
        ret.Description.c_str() );    
      stopVehicle();
      return -1;
    }

    auto ret3 = startTelemetry();    

    if( ret3.Result != TelemetryRet::resultEnum::success )
    {
      printf("\n\n\n###### vehicle::startVehicle() Exception : Unable to start telemetry : %s ######\n\n\n", 
        ret.Description.c_str() );    
      stopVehicle();
      return -1;
    }
  }
  catch(const std::exception& e)
  {
    printf("\n\n\n###### vehicle::startVehicle() Exception : %s ######\n\n\n", 
    e.what() );    
    stopVehicle();
    return -1;
  }
  catch(...)
  {
    printf("\n\n\n###### vehicle::startVehicle() Uncategorized Exception ######\n\n\n" );    
    stopVehicle();
    return -1;
  }
  
  return 0;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

TelemetryRet vehicle::startTelemetry()
{
  std::vector<std::shared_ptr<TelemetrySource>> telemSources = 
    {std::make_shared<TelemetrySource_GlobalPositionInt>(), 
    std::make_shared<TelemetrySource_Attitude>(), 
    std::make_shared<TelemetrySource_LocalPositionNed>(), 
    std::make_shared<TelemetrySource_Heartbeat>(), 
    std::make_shared<TelemetrySource_SysStatus>(),
    std::make_shared<TelemetrySource_Velocity>(), 
    std::make_shared<TelemetrySource_ExtendedSysState>(), 
    std::make_shared<TelemetrySource_HomePosition>()
    };

  auto trigger = TelemetrySource::Trigger(1000);
  TelemetryRet ret;

  for( auto TS : telemSources )
  {
    ret = vehicleTelemetry->addTelemetrySource( TS, trigger, vehicleTelemetry );
    if( ret.Result == TelemetryRet::resultEnum::failure )
      return ret;
  }

  ret = vehicleTelemetry->startTelemetrySourcesAsync(); 

  return ret;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int vehicle::stopVehicle()
{
  try
  {
    if(nullptr != vehicleTelemetry)
      vehicleTelemetry->stopTelemetrySources();
      
    if(nullptr != udpConnection)
      udpConnection->stopConnection();
      
    if(nullptr != vehicleInterface)
      vehicleInterface->stopVehicle();
  }
  catch(...)
  {
    //*** Just leave
  }
  
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
