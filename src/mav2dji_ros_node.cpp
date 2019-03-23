/*
 * mav2dji_ros_node.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

/*#include <mav_message.hpp>

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "mav2dji");
  ros::NodeHandle nodeHandle("~");
  mav2dji::mav_message mavmessage;

  mavmessage.startVehicle();

  ros::spin();

  return 0;
}*/

#include <vehicle.hpp>

int main(int argc, char** argv) 
{
  mav2dji::vehicle djiVehicle;

  if( 0 > djiVehicle.init() )
    return -1;

  if( 0 > djiVehicle.startVehicleAsync())
    return -1;

  //*** TODO : Do we need something better here?
  while(djiVehicle.isVehicleRunning())
    sleep(1); 

  return 0;
}