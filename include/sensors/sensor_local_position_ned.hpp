/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

#include "math.h"
#include <mavlink/common/mavlink_msg_local_position_ned.h>

namespace mav2dji {

    class SensorLocalPositionNed {
        public:
            SensorLocalPositionNed() 
            {
                m_data.time_boot_ms = 0;
                m_data.x = 0;
                m_data.y = 0;
                m_data.z = 0;
                m_data.vx = 0;
                m_data.vy = 0;
                m_data.vz = 0;
            }

            ~SensorLocalPositionNed() {}

            inline const mavlink_local_position_ned_t* getDataPtr() {
                return &m_data;
            }

            inline void setTimeBootMs(const int32_t* ts) {
                m_data.time_boot_ms = *ts;
            }

            inline void setPosition(const int32_t* ts, 
                const float* x, const float* y, const float* z) 
            {
                m_data.time_boot_ms = *ts;
                m_data.x = *x;
                m_data.y = *y;
                m_data.z = *z;
            }

            inline void setVelocity(const int32_t* ts, 
                const float* vx, const float* vy, const float* vz) 
            {
                m_data.time_boot_ms = *ts;
                m_data.x = *vx;
                m_data.y = *vy;
                m_data.z = *vz;
            }

            inline void setX(const float* x) {
                m_data.x = *x;
            }

            inline void setY(const float* y) {
                m_data.y = *y;
            }

            inline void setZ(const float* z) {
                m_data.z = *z;
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
            mavlink_local_position_ned_t m_data;


    };

} //namespace mav2dji


