/*
 * mav2dji_ros_node.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <mav_message.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mav2dji");
  ros::NodeHandle nodeHandle("~");
  mav2dji::mav_message mavmessage;

  mavmessage.startVehicle();

  

  ros::spin();


  return 0;
}