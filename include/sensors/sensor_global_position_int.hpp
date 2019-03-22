/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

#include "math.h"
#include <mavlink/common/mavlink_msg_global_position_int.h>

namespace mav2dji {

    class SensorGlobalPositionInt {
        public:
            SensorGlobalPositionInt() 
            {
                m_data.time_boot_ms = 0;    //< [ms] Timestamp (time since system boot).
                m_data.lat = 0;             //< [degE7] Latitude, expressed
                m_data.lon = 0;             //< [degE7] Longitude, expressed
                m_data.alt = 0;             //< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
                m_data.relative_alt = 0;    //< [mm] Altitude above ground
                m_data.vx = 0;              //< [cm/s] Ground X Speed (Latitude, positive north)
                m_data.vy = 0;              //< [cm/s] Ground Y Speed (Longitude, positive east)
                m_data.vz = 0;              //< [cm/s] Ground Z Speed (Altitude, positive down)
                m_data.hdg = 0;             //< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
            }

            ~SensorGlobalPositionInt() {
            }

            inline mavlink_global_position_int_t* getDataPtr() {
                return &m_data;
            }

            inline void setLatLonAlt(const int32_t* ts, const double* lat, 
                    const double* lon, const double* alt, const double* height) 
            {
                m_data.time_boot_ms = *ts;
                m_data.lat          = (int) (*lat * 1e7);
                m_data.lon          = (int) (*lon * 1e7);
                m_data.alt          = (int) (*alt * 1e7);
                m_data.relative_alt = (int) (*height * 1e7);
            }

            inline void setVelocity(const int32_t* ts, const double* vx, 
                    const double* vy, const double* vz) 
            {
                m_data.time_boot_ms = *ts;
                m_data.vx           = *vx;
                m_data.vy           = *vy;
                m_data.vz           = *vz;
            }

           inline void setTimeBootMs(const int32_t* ts) {
                m_data.time_boot_ms = *ts;
            }

            inline void setLat(const double* lat) {
                m_data.lat = (int) (*lat * 1e7);
            }

            inline void setLat(const int32_t* lat) {
                m_data.lat = *lat;
            }

            inline void setLon(const double* lon) {
                m_data.lon = (int) (*lon * 1e7);
            }

            inline void setLon(const int32_t* lon) {
                m_data.lon = *lon;
            }

            inline void setAlt(const float* alt) {
                m_data.alt = (int) (*alt * 1e7);
            }

            inline void setAlt(const int32_t* alt) {
                m_data.alt = *alt;
            }

            inline void setRelativeAlt(const float* relativeAlt) {
                m_data.relative_alt = (int) (*relativeAlt * 1e7);
            }


            inline void setRelativeAlt(const int32_t* relativeAlt) {
                m_data.relative_alt = *relativeAlt;
            }


            inline void setVx(const float* vx) {
                m_data.vx = *vx;
            }


            inline void setVy(const float* vy) {
                m_data.vy = *vy;
            }


            inline void setVz(const float* vz) {
                m_data.vz = *vz;
            }


        private:
            mavlink_global_position_int_t m_data;


    };

} //namespace mav2dji



