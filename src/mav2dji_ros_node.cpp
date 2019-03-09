/*
 * mav2dji_ros_node.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <mav2dji_ros.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mav2dji");
  ros::NodeHandle nodeHandle("~");
  mav2dji::mav2dji_ros mav2djiRos(nodeHandle);

  mav2djiRos.startVehicle();

  ros::spin();


  return 0;
}