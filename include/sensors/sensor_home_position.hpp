/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

#include "math.h"
#include <mavlink/common/mavlink_msg_home_position.h>

namespace mav2dji {

    class SensorHomePosition {
        public:
            SensorHomePosition() 
            {
            }

            ~SensorHomePosition() {
            }

            inline mavlink_home_position_t* getDataPtr() 
            {
                return &m_data;
            }

            inline bool isValid() 
            {
                return isSet;
            }

            //*****************************************************************
            //*
            //* latitude; /*< Latitude (WGS84), in degrees * 1E7*/
            //* longitude; /*< Longitude (WGS84, in degrees * 1E7*/
            //* altitude; /*< Altitude (AMSL), in meters * 1000 (positive for up)*/
            //*
            //*****************************************************************
            
            inline void setPosition(const double* lat, const double* lon, 
                const double* alt, float xLoc, float yLoc, float zLoc,
                float normal[4], float xApproach, float yApproach, float zApproach) 
            {
                m_data.latitude   = (int) (*lat * 1e7);
                m_data.longitude  = (int) (*lon * 1e7);
                m_data.altitude   = (int) (*alt * 1e7);
                m_data.x = xLoc;
                m_data.y = yLoc;
                m_data.z = zLoc;
                m_data.approach_x = xApproach;
                m_data.approach_y = yApproach;
                m_data.approach_z = zApproach;
                m_data.q[0] = normal[0];
                m_data.q[1] = normal[1];
                m_data.q[2] = normal[2];
                m_data.q[3] = normal[3];

                isSet = true;
            }

            //*****************************************************************
            //*
            //* latitude: Latitude (WGS84), in degrees * 1E7
            //* longitude: Longitude (WGS84, in degrees * 1E7
            //* altitude: Altitude (AMSL), in meters * 1000 (positive for up)
            //* x; Local X position of this position in the local coordinate frame
            //* y; Local Y position of this position in the local coordinate frame
            //* z; Local Z position of this position in the local coordinate frame*
            //* q[4];World to surface normal and heading transformation of the 
            //*     takeoff position. Used to indicate the heading and slope of the ground
            //* approach_x; Local X position of the end of the approach vector. 
            //*     Multicopters should set this position based on their takeoff 
            //*     path. Grass-landing fixed wing aircraft should set it the 
            //*     same way as multicopters. Runway-landing fixed wing aircraft 
            //*     should set it to the opposite direction of the takeoff, 
            //*     assuming the takeoff happened from the threshold / touchdown zone.
            //* approach_y; Local Y position of the end of the approach vector. 
            //*     Multicopters should set this position based on their takeoff 
            //*     path. Grass-landing fixed wing aircraft should set it the 
            //*     same way as multicopters. Runway-landing fixed wing aircraft 
            //*     should set it to the opposite direction of the takeoff, 
            //*     assuming the takeoff happened from the threshold / touchdown zone.            
            //* approach_z; Local Z position of the end of the approach vector.             
            //*     Multicopters should set this position based on their takeoff 
            //*     path. Grass-landing fixed wing aircraft should set it the 
            //*     same way as multicopters. Runway-landing fixed wing aircraft 
            //*     should set it to the opposite direction of the takeoff, 
            //*     assuming the takeoff happened from the threshold / touchdown zone.   
            //*         
            //*****************************************************************
            
            inline void setPosition(const double* lat, const double* lon, 
                const double* alt) 
            {
                m_data.latitude   = (int) (*lat * 1e7);
                m_data.longitude  = (int) (*lon * 1e7);
                m_data.altitude   = (int) (*alt * 1e7);
                m_data.x = 0;
                m_data.y = 0;
                m_data.z = 0;
                m_data.approach_x = 0;
                m_data.approach_y = 0;
                m_data.approach_z = 0;
                m_data.q[0] = 1;
                m_data.q[1] = 0;
                m_data.q[2] = 0;
                m_data.q[3] = 0;

                isSet = true;
            }

        private:

            bool isSet = false;
            mavlink_home_position_t m_data;
    };

} //namespace mav2dji



