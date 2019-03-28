/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once

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

namespace mav2dji
{

class vehicle_interface_djiros : public vehicle_interface
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

  void vehicleRunWorker();

  ros::ServiceClient cameraActionService;

 public:

  enum CameraActionEnum {TakePhoto, StartVideo, StopVideo};
  enum VehicleTaskEnum {Takeoff, Land, GoHome};
  enum ControlAutority {TakeAuthority, ReleaseAuthority};
  struct DroneVersion{ std::string Hardware; uint32_t Version; bool IsValid = false; };

  explicit vehicle_interface_djiros();
  ~vehicle_interface_djiros();

  int init();
  vehicle_interface_ret connectToPlatform();
  vehicle_interface_ret activate();
  vehicle_interface_ret startVehicleAsync();
  vehicle_interface_ret stopVehicle();

  Util::OpRet Activation();
  Util::OpRet CameraAction(CameraActionEnum action);
  Util::OpRet ArmVehicle(bool arm);
  Util::OpRet VehicleTask(VehicleTaskEnum task);
  Util::OpRet MFIOConfig();
  Util::OpRet MFIOSetValue();
  Util::OpRet MissionHpAction();
  Util::OpRet MissionHpGetInfo();
  Util::OpRet MissionHpResetYaw();
  Util::OpRet MissionHpUpdateRadius();
  Util::OpRet MissionHpUpdateYawRate();
  Util::OpRet MissionHpUpload();
  Util::OpRet MissionWpAction();
  Util::OpRet MissionWpGetInfo(
    std::shared_ptr<mav2dji::MissionWaypointTask>* waypointTask);
  float MissionWpGetSpeed();
  Util::OpRet MissionWpGetSpeed(float speed);
  Util::OpRet MissionWpUpload(const mav2dji::MissionWaypointTask* waypointTask);
  Util::OpRet SDKControlAuthority(ControlAutority authority);
  Util::OpRet SendMobileData();
  Util::OpRet SetHardSync();
  DroneVersion QueryDroneVersion();
  Util::OpRet SetLocalPosRef();
  Util::OpRet Stereo240pSubscription();
  Util::OpRet StereoDepthSubscription();
  Util::OpRet StereoVGASubscription();
  Util::OpRet SetupCameraStream();
};

} /* namespace mav2dji*/