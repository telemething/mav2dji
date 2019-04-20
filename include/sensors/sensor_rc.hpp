/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

#include "math.h"
#include<vector> 

namespace mav2dji 
{
    class SensorRc 
    {
        public:

            //*****************************************************************
            //*
            //* RC_CHANNELS_SCALED : The scaled values of the RC channels received
            //* RC_CHANNELS_RAW : The RAW values of the RC channels received.
            //* RC_CHANNELS : The PPM values of the RC channels received.
            //* MANUAL_CONTROL : This message provides an API for manually 
            //*     controlling the vehicle using standard joystick axes 
            //*     nomenclature, along with a joystick-like input device.
            //* RC_CHANNELS_OVERRIDE : The RAW values of the RC channels sent 
            //*     to the MAV to override info received from the RC radio. 
            //*
            //*****************************************************************
            
           SensorRc() 
            {
            }

            ~SensorRc() 
            {}

            //*****************************************************************
            //*
            //*
            //*
            //*****************************************************************
            
            inline std::vector<float> getAxes() 
            {
                return m_axes;
            }

            //*****************************************************************
            //*
            //*
            //*
            //*****************************************************************
            
            inline std::vector<int32_t> getButtons() 
            {
                return m_buttons;
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
            
            inline void setState( std::vector<float> axes, std::vector<int32_t> buttons ) 
            {
                m_axes = axes;
                m_buttons = buttons;
                isSet = true;
            }

        private:

            bool isSet = false;
            std::vector<float> m_axes;
            std::vector<int32_t> m_buttons;
    };

} //namespace mav2dji



