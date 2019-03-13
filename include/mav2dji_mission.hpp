/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once 
// ROS
#include <ros/ros.h>
#include <mavvehiclelib.hpp>

namespace mav2dji
{

//class mav2dji_ros : mavvehiclelib::mavvehicleclient
class mav2dji_mission
{
 public:

    // ROS node handle.
    ros::NodeHandle nodeHandle_;

    explicit mav2dji_mission(ros::NodeHandle nh);
    ~mav2dji_mission();
    
    void ProcessMavMessage(const mavlink_message_t* msg);

 private:

    bool verbose = false;
    uint8_t _mission_type;
    bool _transfer_in_progress = false;


    void printMavMessageInfo(const mavlink_message_t* msg, 
        std::string prefix, bool always);
    
    void send_mission_ack(uint8_t sysid, uint8_t compid, uint8_t type);
    void processMAVLINK_MSG_ID_MISSION_REQUEST_LIST(const mavlink_message_t* msg);
    void processMAVLINK_MSG_ID_MISSION_REQUEST(const mavlink_message_t* msg);
    void handle_mission_count(const mavlink_message_t* msg);
    void processMAVLINK_MSG_ID_MISSION_SET_CURRENT(const mavlink_message_t* msg);
    void processMAVLINK_MSG_ID_MISSION_ITEM(const mavlink_message_t* msg);
};

} /* namespace mav2dji*/