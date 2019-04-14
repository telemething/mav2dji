/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

#include "math.h"
#include <mavlink/common/mavlink_msg_battery_status.h>

namespace mav2dji 
{
    class SensorBatteryStatus 
    {
        public:

            typedef struct sysBatteryStatus
            {
                uint16_t voltageMv = INT16_MAX;
                int16_t currentCa = -1;
                int8_t  capacityRemaining = -1; 
            } sysBatteryStatus_t;

            //*****************************************************************
            //*
            //*
            //*
            //*****************************************************************
            
           SensorBatteryStatus() 
            {
                m_data.current_consumed = -1;
                m_data.energy_consumed = -1; 
                m_data.temperature = INT16_MAX; 
                m_data.voltages[10]; 
                m_data.current_battery = -1; 
                m_data.id = 0; 
                m_data.battery_function = MAV_BATTERY_FUNCTION_ALL; 
                m_data.type = MAV_BATTERY_TYPE_LIPO; 
                m_data.battery_remaining = -1; 
            }

            ~SensorBatteryStatus() 
            {}

            //*****************************************************************
            //*
            //*
            //*
            //*****************************************************************
            
            inline sysBatteryStatus_t* getSysBattStatus() 
            {
                return &sysBattStatus;
            }

            //*****************************************************************
            //*
            //*
            //*
            //*****************************************************************
            
            inline mavlink_battery_status_t* getDataPtr() 
            {
                return &m_data;
            }

            //*****************************************************************
            //*
            //*
            //*
            //*****************************************************************
            
            inline bool isValid() 
            {
                return isSet;
            }

            //*****************************************************************
            //*
            //*
            //*
            //*****************************************************************
            
            inline void setState( std::string location, float voltageV, 
                float currentA, float percentRemaining ) 
            {
                // TODO * right now we just record the status of the last
                // battery message received, we need to id the batteries
                // and average the volts and sum the currents.
                sysBattStatus.voltageMv = voltageV / 1000;
                sysBattStatus.currentCa = currentA; 
                sysBattStatus.capacityRemaining = percentRemaining; 

                isSet = true;
            }

            //*****************************************************************
            //*
            //* voltages            Battery voltage of cells, in millivolts 
            //*     (1 = 1 millivolt)
            //* current             Battery current, in 10*milliamperes 
            //*     (1 = 10 milliampere), -1: autopilot does not measure the current
            //* percentRemaining    Remaining battery energy: (0%: 0, 100%: 100), 
            //*     -1: autopilot does not estimate the remaining battery
            //*
            //*****************************************************************
            
            inline void setState( uint8_t id, uint16_t voltage, int16_t current, 
                int8_t percentRemaining ) 
            {
                m_data.current_battery = current; 
                m_data.battery_remaining = percentRemaining; 

                isSet = true;
            }

            //*****************************************************************
            //*
            //* currentConsumed     Consumed charge, in milliampere hours 
            //*     (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
            //* joulesConsumed     Consumed energy, in 100*Joules 
            //*     (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does 
            //*     not provide energy consumption estimate
            //* temperature         Temperature of the battery in centi-degrees 
            //*     celsius. INT16_MAX for unknown temperature.
            //* voltages[10]        Battery voltage of cells, in millivolts 
            //*     (1 = 1 millivolt)
            //* current             Battery current, in 10*milliamperes 
            //*     (1 = 10 milliampere), -1: autopilot does not measure the current
            //* id                  Battery ID
            //* function            Function of the battery
            //* type                Type (chemistry) of the battery
            //* percentRemaining    Remaining battery energy: (0%: 0, 100%: 100), 
            //*     -1: autopilot does not estimate the remaining battery
            //*
            //*****************************************************************
            
            inline void setState(int32_t currentConsumed, int32_t joulesConsumed,
                int16_t temperature, uint16_t voltages[10], int16_t current,
                uint8_t id, uint8_t function, uint8_t type, int8_t percentRemaining ) 
            {
                m_data.current_consumed = currentConsumed;
                m_data.energy_consumed = joulesConsumed; 
                m_data.temperature = temperature; 
                m_data.voltages[10]; 
                m_data.current_battery = current; 
                m_data.id = id; 
                m_data.battery_function = function; 
                m_data.type = type; 
                m_data.battery_remaining = percentRemaining; 

                isSet = true;
            }

        private:

            bool isSet = false;
            mavlink_battery_status_t m_data;
            sysBatteryStatus_t sysBattStatus;
    };

} //namespace mav2dji



