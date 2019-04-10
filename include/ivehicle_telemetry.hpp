#include <mavlink/common/mavlink.h>

class iVehicleTelemetry 
{
 public:

   virtual void setBaseMode(uint8_t value) = 0;                      
   virtual void setSystemStatus(MAV_STATE value) = 0; 
   virtual void setCustomMode(uint32_t value) = 0;
  
   virtual uint8_t getBaseMode() = 0;                   
   virtual MAV_STATE getSystemStatus() = 0;  
   virtual uint32_t getCustomMode() = 0;
  };