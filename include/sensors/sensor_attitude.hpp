/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

#include "math.h"
#include <mavlink/common/mavlink_msg_attitude.h>

namespace mav2dji 
{
    class SensorAttitude 
    {
        public:
            SensorAttitude() 
            {
                m_data.time_boot_ms = 0;
                m_data.roll = 0;
                m_data.pitch = 0;
                m_data.yaw = 0;
                m_data.rollspeed = 0;
                m_data.pitchspeed = 0;
                m_data.yawspeed = 0;
            }
            ~SensorAttitude() {}

            void setQuaternion(const int32_t* ts, const double* q0, 
                    const double* q1, const double* q2, const double* q3, 
                    const float* wx, const float* wy, const float* wz) 
            {
                m_data.time_boot_ms = *ts;
                setRoll(q0, q1, q2, q3);
                setPitch(q0, q1, q2, q3);
                setYaw(q0, q1, q2, q3);
                m_data.rollspeed = *wx;
                m_data.pitchspeed = *wy;
                m_data.yawspeed = *wz;
            }

            inline mavlink_attitude_t* getDataPtr() {
                return &m_data;
            }

            inline void setTimeBootMs(const int32_t* ts) {
                m_data.time_boot_ms = *ts;
            }

            inline void setRoll(const float* roll) {
                m_data.roll = *roll;
            }

            inline void setRoll(const double* q0, const double* q1, 
                    const double* q2, const double* q3) {

                m_data.roll = atan2(  2*( (*q0)*(*q1)+(*q2)*(*q3) ), 
                        1-2*( (*q1)*(*q1)+(*q2)*(*q2) )  );

            }

            inline void setPitch(const float* pitch) {
                m_data.pitch = *pitch;
            }

            inline void setPitch(const double* q0, const double* q1, 
                    const double* q2, const double* q3) {

                m_data.pitch = asin(  2*( (*q0)*(*q2)-(*q3)*(*q1) )  );

            }

            inline void setYaw(const float* yaw) {
                m_data.yaw = *yaw;
            }

            inline void setYaw(const double* q0, const double* q1, 
                    const double* q2, const double* q3) {

                m_data.yaw = atan2(  2*( (*q0)*(*q3)+(*q1)*(*q2) ), 
                        1-2*( (*q2)*(*q2)+(*q3)*(*q3) )  );

            }

            inline void setRollSpeed(const float* wx) {
                m_data.rollspeed = *wx;
            }

            inline void setPitchSpeed(const float* wy) {
                m_data.pitchspeed = *wy;
            }

            inline void setYawSpeed(const float* wz) {
                m_data.yawspeed = *wz;
            }

        private:
            mavlink_attitude_t m_data;


    };

} //namespace mav2dji


