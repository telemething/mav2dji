#pragma once

#include <mavlink/common/mavlink.h>

class iVehicleTelemetry 
{
 public:

   explicit iVehicleTelemetry();
   ~iVehicleTelemetry();

   virtual void setBaseMode(uint8_t value) = 0;                      
   virtual void setSystemStatus(uint8_t value) = 0; 
   virtual void setCustomMode(uint32_t value) = 0;
  
   virtual uint8_t getBaseMode() = 0;                   
   virtual uint8_t getSystemStatus() = 0;  
   virtual uint32_t getCustomMode() = 0;
  };