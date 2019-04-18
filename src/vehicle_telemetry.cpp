/*
 * mav2dji_ros.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <vehicle_telemetry.hpp>
#include <chrono>

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

TelemTrigger::triggerTypeEnum 
  TelemTrigger::getTriggerType()
  {return triggerType_;};

int TelemTrigger::getTimeSpanMs()
{return timeSpanMs_;}; 

int TelemTrigger::getTimeSpanHz()
{return 1000.0/(double)timeSpanMs_;}; 

std::shared_ptr<ros::Rate> TelemTrigger::getTimeSpanRate()
{ return std::make_shared<ros::Rate>(1000.0/(double)timeSpanMs_); }

//*****************************************************************************
//*
//*
//*
//******************************************************************************

TelemTrigger::TelemTrigger()
{};

//*****************************************************************************
//*
//*
//*
//******************************************************************************

TelemTrigger::~TelemTrigger()
{};

//*****************************************************************************
//*
//*
//*
//******************************************************************************

TelemTrigger::TelemTrigger(triggerTypeEnum triggerType, int timeSpanMs)
{
  triggerType_ = triggerType;
  timeSpanMs_ = timeSpanMs;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

TelemTrigger::TelemTrigger(int timeSpanMs)
{
  triggerType_ = triggerTypeEnum::period;
  timeSpanMs_ = timeSpanMs;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

TelemetrySource::TelemetrySource()
{};

//*****************************************************************************
//*
//*
//*
//******************************************************************************

TelemetrySource::~TelemetrySource()
{};

//*****************************************************************************
//*
//*
//*
//******************************************************************************

TelemTrigger TelemetrySource::getTrigger()
{ return trigger_;}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int TelemetrySource::sendMavMessageToGcs(const mavlink_message_t* msg)
{ return sendMavMessageCallback(msg); };

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int TelemetrySource::init(TelemTrigger trigger, std::shared_ptr<VehicleTelemetry> parent)
{ 
  vehicleTelemetry = parent;
  rosNodeHandle = VehicleInfo::getVehicleInterface()->rosNodeHandle;
  trigger_ = trigger; 
  workerRosRate = trigger_.getTimeSpanRate(); 
      
  //telemetryInit_();
  telemetryInit();
};

//*****************************************************************************
//*
//*
//*
//******************************************************************************

Util::OpRet TelemetrySource::startTelemetryAsync()
{
  mavlinkSystemId = VehicleInfo::getMavlinkSystemId();
  mavlinkComponentId = VehicleInfo::getMavlinkComponentId();
  sendMavMessageCallback = VehicleInfo::getSendMavMessageCallback();

  //telemetryRunWorkerThread = std::thread(telemetryRunWorker_);
  telemetryRunWorkerThread = std::thread(&TelemetrySource::telemetryRunWorker, this);
  return Util::OpRet(Util::OpRet::resultEnum::success);    
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

uint64_t TelemetrySource::microsSinceEpoch()
{
  struct timeval tv;
     
  uint64_t micros = 0;
      
  gettimeofday(&tv, NULL);  
  micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
      
  return micros;
}
   
//*****************************************************************************
//*
//*
//*
//******************************************************************************

int32_t TelemetrySource::getTimeBootMs(std_msgs::Header header)
{
  //*** TODO * We need to think about synchronizing with controller time
  return getTimeBootMs();
  //return header.stamp.sec + header.stamp.nsec * 1000000;
}
    
//*****************************************************************************
//*
//*
//*
//******************************************************************************

uint64_t TelemetrySource::getTimeBootMs()
{
  return microsSinceEpoch() - bootTime;
}
    
//*****************************************************************************
//*
//*
//*
//******************************************************************************

Util::OpRet TelemetrySource::stopTelemetry(){};

//*****************************************************************************
//*
//*
//*
//******************************************************************************

VehicleTelemetry::VehicleTelemetry()
{};

//*****************************************************************************
//*
//*
//*
//******************************************************************************

VehicleTelemetry::~VehicleTelemetry()
{};

//*****************************************************************************
//*
//*
//*
//******************************************************************************

Util::OpRet VehicleTelemetry::addTelemetrySource(
  std::shared_ptr<TelemetrySource> telemSource, 
  TelemTrigger trigger, 
  std::shared_ptr<VehicleTelemetry> parent)
{
  telemSource->init(trigger, parent);
  telemSources.push_back(telemSource);

  return Util::OpRet();
};
    
//*****************************************************************************
//*
//*
//*
//******************************************************************************

Util::OpRet VehicleTelemetry::startTelemetrySourcesAsync()
{
  for(auto ts : telemSources)
  {
    ts->startTelemetryAsync();
  }

  return Util::OpRet();
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

Util::OpRet VehicleTelemetry::stopTelemetrySources()
{
  for(auto ts : telemSources)
  {
    ts->stopTelemetry();
  }

  return Util::OpRet();
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void VehicleTelemetry::setBaseMode(uint8_t value)
{baseMode = value;}            

void VehicleTelemetry::setSystemStatus(uint8_t value)
{systemStatus = value;}    

void VehicleTelemetry::setCustomMode(uint32_t value)
{customMode = value;} 
  
void VehicleTelemetry::setLandedState(uint8_t value)
{landedState = value;} 
  
uint8_t VehicleTelemetry::getBaseMode()
{return baseMode;}      

uint8_t VehicleTelemetry::getSystemStatus()
{return systemStatus;}     

uint32_t VehicleTelemetry::getCustomMode()
{return customMode;} 

uint8_t VehicleTelemetry::getLandedState()
{return landedState;} 

void VehicleTelemetry::setArmed(bool arm)
{
  if(arm)
    baseMode |= MavModeFlag::mavModeFlagSafteyArmed;
  else
    baseMode &= ~MavModeFlag::mavModeFlagSafteyArmed;
}

bool VehicleTelemetry::isArmed()
{
  return baseMode & MavModeFlag::mavModeFlagSafteyArmed;
}

void VehicleTelemetry::setCurrentAsHomePosition(const double* lat, 
    const double* lon, const double* alt)
{
  // set member value to giver coords
  telemHomePosition.setPosition(lat, lon, alt);

  // send signal to the vehicle
  VehicleInfo::getVehicleInterface()->SetLocalPosRef(); 
}

}