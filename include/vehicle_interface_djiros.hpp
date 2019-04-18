#pragma once 

/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <vehicle_interface.hpp>
#include <vehicle_Info.hpp>
#include <mission_ops.hpp>
#include <util.hpp>
#include <thread>


// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/CameraAction.h>
#include <dji_sdk/Gimbal.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/MissionWpUpload.h>
#include <dji_sdk/MissionWpAction.h>

namespace mav2dji
{

class VehicleInterfaceDjiros : public vehicle_interface
{
 private:

  int DjiActivationSleepMs = 1000;
  int DjiActivationTimeoutMs = 10000;

  std::thread vehicleRunWorkerThread;

  void testCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

  //std::shared_ptr<ros::NodeHandle> rosNodeHandle;
  ros::ServiceClient droneActivationService;
  ros::ServiceClient armVehicleService;
  ros::ServiceClient vehicleTaskService;
  ros::ServiceClient missionWpSetSpeedService;
  ros::ServiceClient missionWpGetSpeedService;
  ros::ServiceClient missionWpUploadService;
  ros::ServiceClient sDKControlAuthorityService;
  ros::ServiceClient queryDroneVersionService;
  ros::ServiceClient setLocalPosRefService;
  ros::ServiceClient missionWpActionService;

  void vehicleRunWorker();

  ros::ServiceClient cameraActionService;

 public:

  explicit VehicleInterfaceDjiros();
  explicit VehicleInterfaceDjiros(std::shared_ptr<iVehicleTelemetry> vehicleTelemetry);
  ~VehicleInterfaceDjiros();

  Util::OpRet init();
  Util::OpRet connectToPlatform();
  Util::OpRet activate();
  Util::OpRet startVehicleAsync();
  Util::OpRet stopVehicle();

  Util::OpRet Activation();
  Util::OpRet CameraAction(const vehicle_interface::CameraActionEnum action);
  Util::OpRet ArmVehicle(bool arm);
  Util::OpRet VehicleTask(const VehicleTaskEnum task);
  Util::OpRet MFIOConfig();
  Util::OpRet MFIOSetValue();
  Util::OpRet MissionHpAction();
  Util::OpRet MissionHpGetInfo();
  Util::OpRet MissionHpResetYaw();
  Util::OpRet MissionHpUpdateRadius();
  Util::OpRet MissionHpUpdateYawRate();
  Util::OpRet MissionHpUpload();
  Util::OpRet MissionWpAction(const MissionWpActionEnum missionWpAction);
  Util::OpRet MissionWpGetInfo(
    std::shared_ptr<mav2dji::MissionWaypointTask>* waypointTask);
  float MissionWpGetSpeed();
  Util::OpRet MissionWpSetSpeed(float speed);
  Util::OpRet MissionWpUpload(const mav2dji::MissionWaypointTask* waypointTask);
  Util::OpRet SDKControlAuthority(const ControlAutority authority);
  Util::OpRet SendMobileData();
  Util::OpRet SetHardSync();
  DroneVersion QueryDroneVersion();
  Util::OpRet SetLocalPosRef();
  Util::OpRet Stereo240pSubscription();
  Util::OpRet StereoDepthSubscription();
  Util::OpRet StereoVGASubscription();
  Util::OpRet SetupCameraStream();

  //--------------------------

  Util::OpRet setMode(uint8_t baseMode, uint32_t customMode);
  Util::OpRet setState(MavState_t newState);
  Util::OpRet armDisarm(bool arm); 

 private:

  void ArmChange(bool arm);
  std::thread ArmChangeThread;

  std::shared_ptr<dji_sdk::MissionWaypointTask> 
    Convert(const mav2dji::MissionWaypointTask* waypointTask );
  std::shared_ptr<mav2dji::MissionWaypointTask> 
    Convert( const dji_sdk::MissionWaypointTask* waypointTask );


};

} /* namespace mav2dji*/