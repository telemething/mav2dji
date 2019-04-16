#include <vehicle_telemetry.hpp>

namespace mav2dji
{



//*****************************************************************************
//*
//* /dji_sdk/gps_position (sensor_msgs/NavSatFix) 50 hz
//*
//*****************************************************************************

class TelemetrySource_GlobalPositionInt : public TelemetrySource
{
 public:

   explicit TelemetrySource_GlobalPositionInt();
   ~TelemetrySource_GlobalPositionInt();

 private:

   void telemetryInit();
   void telemetryRunWorker();
   void callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
};

//*****************************************************************************
//*
//* /dji_sdk/attitude (geometry_msgs/QuaternionStamped) rotation from FLU body 
//*   frame to ENU ground frame, 100 Hz. 
//*
//*****************************************************************************

class TelemetrySource_Attitude : public TelemetrySource
{
 public:

   explicit TelemetrySource_Attitude();
   ~TelemetrySource_Attitude();

 private:

   void telemetryInit();
   void telemetryRunWorker();
   void callback(const geometry_msgs::QuaternionStamped &msg);
};

//*****************************************************************************
//*
//* /dji_sdk/local_position (geometry_msgs/PointStamped) Cartesian ENU frame, 
//*   set origin calling the /dji_sdk/set_local_pos_ref 
//*
//*****************************************************************************

class TelemetrySource_LocalPositionNed : public TelemetrySource
{
 public:

   explicit TelemetrySource_LocalPositionNed();
   ~TelemetrySource_LocalPositionNed();

 private:

   void telemetryInit();
   void telemetryRunWorker();
   void callback(const geometry_msgs::PointStamped &msg);
};

//*****************************************************************************
//*
//*****************************************************************************

class TelemetrySource_Heartbeat : public TelemetrySource
{
 public:

   explicit TelemetrySource_Heartbeat();
   ~TelemetrySource_Heartbeat();

 private:

   void telemetryInit();
   void telemetryRunWorker();
};

//*****************************************************************************
//*
//*****************************************************************************

class TelemetrySource_SysStatus : public TelemetrySource
{
 public:

   explicit TelemetrySource_SysStatus();
   ~TelemetrySource_SysStatus();

 private:

   void telemetryInit();
   void telemetryRunWorker();
};

//*****************************************************************************
//*
//*****************************************************************************

class TelemetrySource_ExtendedSysState : public TelemetrySource
{
 public:

   explicit TelemetrySource_ExtendedSysState();
   ~TelemetrySource_ExtendedSysState();

 private:

   void telemetryInit();
   void telemetryRunWorker();
};

//*****************************************************************************
//*
//*****************************************************************************

class TelemetrySource_HomePosition : public TelemetrySource
{
 public:

   explicit TelemetrySource_HomePosition();
   ~TelemetrySource_HomePosition();

 private:

   void telemetryInit();
   void telemetryRunWorker();
};

//*****************************************************************************
//*
//* /dji_sdk/velocity (geometry_msgs/Vector3Stamped)
//*
//*****************************************************************************

class TelemetrySource_Velocity : public TelemetrySource
{
 public:

   explicit TelemetrySource_Velocity();
   ~TelemetrySource_Velocity();

 private:

   void telemetryInit();
   void telemetryRunWorker();
   void callback(const geometry_msgs::Vector3Stamped &msg);
};

//*****************************************************************************
//*
//* /dji_sdk/battery_state (sensor_msgs/BatteryState) 10 hz
//*
//*****************************************************************************

class TelemetrySource_BatteryState : public TelemetrySource
{
 public:

   explicit TelemetrySource_BatteryState();
   ~TelemetrySource_BatteryState();

 private:

   void telemetryInit();
   void telemetryRunWorker();
   void callback(const sensor_msgs::BatteryState &msg);
};

//*****************************************************************************
//*
//*  /dji_sdk/flight_status (std_msgs/UInt8)
//*
//*  STATUS_STOPPED   = DJI::OSDK::VehicleStatus::FlightStatus::STOPED
//*  STATUS_ON_GROUND = DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND
//*  STATUS_IN_AIR    = DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR
//*
//*****************************************************************************

class TelemetrySource_FlightStatus : public TelemetrySource
{
 public:

  explicit TelemetrySource_FlightStatus();
  ~TelemetrySource_FlightStatus();

  typedef enum DjiFlightStatus
  {
    STOPED    = 0,  
    ON_GROUND = 1,  
    IN_AIR    = 2 
  } DjiFlightStatus_t;

 private:

  void telemetryInit();
  void telemetryRunWorker();
  void callback(const std_msgs::UInt8 &msg);
};

//*****************************************************************************
//*
//* dji_sdk/display_mode (std_msgs/UInt8)
//*   
//* This mode requires the user to manually control the aircraft to remain stable in air. 
//* MODE_MANUAL_CTRL=DJI::OSDK::VehicleStatus::DisplayMode::MODE_MANUAL_CTRL
//*
//* In this mode, the aircraft can keep attitude stabilization and only use the
//* barometer for positioning to control the altitude. 
//*
//* The aircraft can not autonomously locate and hover stably.
//* MODE_ATTITUDE=DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE
//*
//* The aircraft is in normal GPS mode. In normal GPS mode, the aircraft can
//* autonomously locate and hover stably. The sensitivity of the aircraft to the
//* command response is moderate.
//* MODE_P_GPS=DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS
//*
//* In hotpoint mode 
//* MODE_HOTPOINT_MODE=DJI::OSDK::VehicleStatus::DisplayMode::MODE_HOTPOINT_MODE
//*
//* In this mode, user can push the throttle stick to complete stable take-off. 
//* MODE_ASSISTED_TAKEOFF=DJI::OSDK::VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF
//*
//* In this mode, the aircraft will autonomously start motor, ascend and finally hover. 
//* MODE_AUTO_TAKEOFF=DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF
//*
//* In this mode, the aircraft can land autonomously. 
//* MODE_AUTO_LANDING=DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_LANDING
//*
//* In this mode, the aircraft can antonomously return the last recorded Home Point. 
//* There are three types of this mode: Smart RTH(Return-to-Home), Low Batterry RTH, and Failsafe RTTH.  
//* MODE_NAVI_GO_HOME=DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_GO_HOME
//*
//* In this mode, the aircraft is controled by SDK API. User can directly define the control mode of horizon
//* and vertical directions and send control datas to aircraft. 
//* MODE_NAVI_SDK_CTRL=DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL
//*
//* drone is forced to land, might due to low battery 
//* MODE_FORCE_AUTO_LANDING=DJI::OSDK::VehicleStatus::DisplayMode::MODE_FORCE_AUTO_LANDING
//*
//* drone will search for the last position where the rc is not lost 
//* MODE_SEARCH_MODE =DJI::OSDK::VehicleStatus::DisplayMode::MODE_SEARCH_MODE
//*
//* Mode for motor starting. Every time user unlock the motor, this will be the first mode. 
//* MODE_ENGINE_START = DJI::OSDK::VehicleStatus::DisplayMode::MODE_ENGINE_START
//*
//*****************************************************************************

class TelemetrySource_DisplayMode : public TelemetrySource
{
 public:

  explicit TelemetrySource_DisplayMode();
  ~TelemetrySource_DisplayMode();

 private:

  typedef enum DjiDisplayMode
  {
    MODE_MANUAL_CTRL        = 0,
    MODE_ATTITUDE           = 1,
    MODE_P_GPS              = 6,
    MODE_HOTPOINT_MODE      = 9,
    MODE_ASSISTED_TAKEOFF   = 10,
    MODE_AUTO_TAKEOFF       = 11,
    MODE_AUTO_LANDING       = 12,
    MODE_NAVI_GO_HOME       = 15,
    MODE_NAVI_SDK_CTRL      = 17,
    MODE_FORCE_AUTO_LANDING = 33,
    MODE_SEARCH_MODE        = 40,
    MODE_ENGINE_START       = 41,
    MODE_UNINTIALIZED       = 256
  } DjiDispalyMode_t;

  DjiDispalyMode_t currentDjiDisplayMode = DjiDisplayMode::MODE_UNINTIALIZED;

  void telemetryInit();
  void telemetryRunWorker();
  void callback(const std_msgs::UInt8 &msg);
};

//*****************************************************************************
//*
//*  /dji_sdk/height_above_takeoff (std_msgs/Float32) valid after drone is 
//*  armed, after reference altitude set. 
//*
//*****************************************************************************

class TelemetrySource_HeightAboveTakeoff : public TelemetrySource
{
 public:

   explicit TelemetrySource_HeightAboveTakeoff();
   ~TelemetrySource_HeightAboveTakeoff();

 private:

   void telemetryInit();
   void telemetryRunWorker();
   void callback(const std_msgs::Float32 &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//* /dji_sdk/acceleration_ground_fused (geometry_msgs/Vector3Stamped) Fused 
//*   acceleration with respect to East-North-Up (ENU) ground frame, 100 Hz
//*
//*****************************************************************************

/*class TelemetrySource_AccelerationGroundFused : public TelemetrySource
{
 public:

   explicit TelemetrySource_AccelerationGroundFused()
      {sourceTopicName = "/dji_sdk/acceleration_ground_fused";};
   ~TelemetrySource_AccelerationGroundFused(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const geometry_msgs::Vector3Stamped &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//* /dji_sdk/angular_velocity_fused (geometry_msgs/Vector3Stamped) Fused 
//    angular rate (p,q,r) around Forward-Left-Up (FLU) body frame, 100 Hz
//*
//*****************************************************************************

class TelemetrySource_AngularVelocityFused : public TelemetrySource
{
 public:

   explicit TelemetrySource_AngularVelocityFused()
      {sourceTopicName = "/dji_sdk/angular_velocity_fused";};
   ~TelemetrySource_AngularVelocityFused(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const geometry_msgs::Vector3Stamped &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//*  /dji_sdk/from_mobile_data (std_msgs/UInt8[])
//*
//*****************************************************************************

class TelemetrySource_FromMobileData : public TelemetrySource
{
 public:

   explicit TelemetrySource_FromMobileData()
      {sourceTopicName = "/dji_sdk/from_mobile_data";};
   ~TelemetrySource_FromMobileData(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const std_msgs::UInt8 &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//*  /dji_sdk/gimbal_angle (geometry_msgs/Vector3Stamped)
//*
//*****************************************************************************

class TelemetrySource_GimbalAngle : public TelemetrySource
{
 public:

   explicit TelemetrySource_GimbalAngle()
      {sourceTopicName = "/dji_sdk/gimbal_angle";};
   ~TelemetrySource_GimbalAngle(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const geometry_msgs::Vector3Stamped &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//*  /dji_sdk/gps_health (std_msgs/UInt8) strength 0-5, 5 is best, < 3 is bad
//*
//*****************************************************************************

class TelemetrySource_GpsHealth : public TelemetrySource
{
 public:

   explicit TelemetrySource_GpsHealth()
      {sourceTopicName = "/dji_sdk/gps_health";};
   ~TelemetrySource_GpsHealth(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const std_msgs::UInt8 &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//*  /dji_sdk/imu (sensor_msgs/Imu) 400 hz
//*
//*****************************************************************************

class TelemetrySource_Imu : public TelemetrySource
{
 public:

   explicit TelemetrySource_Imu()
      {sourceTopicName = "/dji_sdk/imu";};
   ~TelemetrySource_Imu(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const sensor_msgs::Imu &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//*  /dji_sdk/rc 0=roll, 1=pitch, 2=yaw, 3=throttle, 4=mode, 5=landing gear
//*
//*****************************************************************************

class TelemetrySource_Rc : public TelemetrySource
{
 public:

   explicit TelemetrySource_Rc()
      {sourceTopicName = "/dji_sdk/rc";};
   ~TelemetrySource_Rc(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const sensor_msgs::Joy &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//*  /dji_sdk/trigger_time (sensor_msgs/TimeReference)
//*
//*****************************************************************************

class TelemetrySource_TriggerTime : public TelemetrySource
{
 public:

   explicit TelemetrySource_TriggerTime()
      {sourceTopicName = "/dji_sdk/trigger_time";};
   ~TelemetrySource_TriggerTime(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const sensor_msgs::TimeReference &msg);
};

//*****************************************************************************
//*
//*   --- not instanciated --------------------------------
//*
//*  /dji_sdk/local_position (geometry_msgs/PointStamped) Cartesian ENU frame, 
//   set origin calling the /dji_sdk/set_local_pos_ref 
//*
//*****************************************************************************

class TelemetrySource_LocalPosition : public TelemetrySource
{
 public:

   explicit TelemetrySource_LocalPosition()
      {sourceTopicName = "/dji_sdk/local_position";};
   ~TelemetrySource_LocalPosition(){};

 private:

   void telemetryInit();
   void telemetryRunWorker(){};
   void callback(const geometry_msgs::PointStamped &msg);
}; */


/*

* /dji_sdk/acceleration_ground_fused (geometry_msgs/Vector3Stamped) Fused acceleration with respect to East-North-Up (ENU) ground frame, 100 Hz
* /dji_sdk/angular_velocity_fused (geometry_msgs/Vector3Stamped) Fused angular rate (p,q,r) around Forward-Left-Up (FLU) body frame, 100 Hz
/dji_sdk/attitude (geometry_msgs/QuaternionStamped) rotation from FLU body frame to ENU ground frame, 100 Hz. 
* /dji_sdk/battery_state (sensor_msgs/BatteryState) 10 hz
* /dji_sdk/display_mode (std_msgs/UInt8)
   // This mode requires the user to manually control the aircraft to remain stable in air. 
   MODE_MANUAL_CTRL=DJI::OSDK::VehicleStatus::DisplayMode::MODE_MANUAL_CTRL,
   // In this mode, the aircraft can keep attitude stabilization and only use the
   // barometer for positioning to control the altitude. <br>
   // The aircraft can not autonomously locate and hover stably.
   MODE_ATTITUDE=DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE,
   // The aircraft is in normal GPS mode. In normal GPS mode, the aircraft can
   // autonomously locate and hover stably. The sensitivity of the aircraft to the
   //  command response is moderate.
   MODE_P_GPS=DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS,
   // In hotpoint mode 
   MODE_HOTPOINT_MODE=DJI::OSDK::VehicleStatus::DisplayMode::MODE_HOTPOINT_MODE,
   // In this mode, user can push the throttle stick to complete stable take-off. 
   MODE_ASSISTED_TAKEOFF=DJI::OSDK::VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF,
   // In this mode, the aircraft will autonomously start motor, ascend and finally hover. 
   MODE_AUTO_TAKEOFF=DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF,
   // In this mode, the aircraft can land autonomously. 
   MODE_AUTO_LANDING=DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_LANDING,
   // In this mode, the aircraft can antonomously return the last recorded Home Point. <br>
   // There are three types of this mode: Smart RTH(Return-to-Home), Low Batterry RTH, and Failsafe RTTH.  
   MODE_NAVI_GO_HOME=DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_GO_HOME,
   // In this mode, the aircraft is controled by SDK API. User can directly define the control mode of horizon
   // and vertical directions and send control datas to aircraft. 
   MODE_NAVI_SDK_CTRL=DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL,
   // drone is forced to land, might due to low battery 
   MODE_FORCE_AUTO_LANDING=DJI::OSDK::VehicleStatus::DisplayMode::MODE_FORCE_AUTO_LANDING,
   // drone will search for the last position where the rc is not lost 
   MODE_SEARCH_MODE =DJI::OSDK::VehicleStatus::DisplayMode::MODE_SEARCH_MODE,
   // Mode for motor starting. Every time user unlock the motor, this will be the first mode. 
   MODE_ENGINE_START = DJI::OSDK::VehicleStatus::DisplayMode::MODE_ENGINE_START

* /dji_sdk/flight_status (std_msgs/UInt8)
  STATUS_STOPPED   = DJI::OSDK::VehicleStatus::FlightStatus::STOPED,
  STATUS_ON_GROUND = DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND,
  STATUS_IN_AIR    = DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR

- /dji_sdk/from_mobile_data (std_msgs/UInt8[])
* /dji_sdk/gimbal_angle (geometry_msgs/Vector3Stamped)
* /dji_sdk/gps_health (std_msgs/UInt8) strength 0-5, 5 is best, < 3 is bad
/dji_sdk/gps_position (sensor_msgs/NavSatFix) 50 hz
* /dji_sdk/imu (sensor_msgs/Imu) 400 hz
* /dji_sdk/rc 0=roll, 1=pitch, 2=yaw, 3=throttle, 4=mode, 5=landing gear
/dji_sdk/velocity (geometry_msgs/Vector3Stamped)
* /dji_sdk/height_above_takeoff (std_msgs/Float32) valid after drone is armed, after reference altitude set. 
- /dji_sdk/trigger_time (sensor_msgs/TimeReference)
- /dji_sdk/local_position (geometry_msgs/PointStamped) Cartesian ENU frame, set origin calling the /dji_sdk/set_local_pos_ref 
- /dji_sdk/rtk_position (sensor_msgs/NavSatFix)
- /dji_sdk/rtk_velocity (geometry_msgs/Vector3)
- /dji_sdk/rtk_yaw (std_msgs/Int16)
- /dji_sdk/rtk_info_position (std_msgs/UInt8)
- /dji_sdk/rtk_info_yaw (std_msgs/UInt8)

- = didnt see
* = saw but dont service

*/

}
