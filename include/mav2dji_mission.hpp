/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once 

#include <mav2dji_message_base.hpp>

#  define NUM_MISSIONS_SUPPORTED 2000 // This allocates a file of around 181 kB on the SD card.


namespace mav2dji
{
    enum MAVLINK_WPM_STATES {
	MAVLINK_WPM_STATE_IDLE = 0,
	MAVLINK_WPM_STATE_SENDLIST,
	MAVLINK_WPM_STATE_GETLIST,
	MAVLINK_WPM_STATE_ENUM_END
};

enum MAVLINK_WPM_CODES {
	MAVLINK_WPM_CODE_OK = 0,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_ACTION_NOT_SUPPORTED,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_FRAME_NOT_SUPPORTED,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_OUT_OF_BOUNDS,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_MAX_NUMBER_EXCEEDED,
	MAVLINK_WPM_CODE_ENUM_END
};

//class mav2dji_ros : mavvehiclelib::mavvehicleclient
class mav2dji_mission : mav2dji_message_base
{
 public:

    // ROS node handle.
    ros::NodeHandle nodeHandle_;

    explicit mav2dji_mission(ros::NodeHandle nh, std::shared_ptr<mavvehiclelib::mavvehicle> mavVeh);
    ~mav2dji_mission();
    
    void ProcessMavMessage(const mavlink_message_t* msg);

 private:

    bool verbose = false;
    bool _transfer_in_progress = false;

    dm_item_t _dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;

	enum MAVLINK_WPM_STATES _state {MAVLINK_WPM_STATE_IDLE};	///< Current state
	enum MAV_MISSION_TYPE _mission_type {MAV_MISSION_TYPE_MISSION};	///< mission type of current transmission (only one at a time possible)

   	uint64_t		_time_last_recv{0};
	uint64_t		_time_last_sent{0};
   	unsigned		_filesystem_errcount{0};		///< File system error count

    struct mission_stats_entry_s 
    {
        uint16_t num_items;			/**< total number of items stored (excluding this one) */
        uint16_t update_counter;			/**< This counter is increased when (some) items change (this can wrap) */
    };

    static constexpr unsigned int	FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT =
            2;	///< Error count limit before stopping to report FS errors

    struct mission_s 
    {
        uint64_t timestamp;
        int32_t current_seq;
        uint16_t count;
        uint8_t dataman_id;
        uint8_t _padding0[1]; // required for logger
    };

    enum 
    {
        DM_KEY_SAFE_POINTS_MAX = 8,
        DM_KEY_FENCE_POINTS_MAX = 64,
        DM_KEY_WAYPOINTS_OFFBOARD_0_MAX = NUM_MISSIONS_SUPPORTED,
        DM_KEY_WAYPOINTS_OFFBOARD_1_MAX = NUM_MISSIONS_SUPPORTED,
        DM_KEY_WAYPOINTS_ONBOARD_MAX = NUM_MISSIONS_SUPPORTED,
        DM_KEY_MISSION_STATE_MAX = 1,
        DM_KEY_COMPAT_MAX = 1
    };

    uint16_t MAX_COUNT[3] = 
    {
		DM_KEY_WAYPOINTS_OFFBOARD_0_MAX,
		DM_KEY_FENCE_POINTS_MAX - 1,
		DM_KEY_SAFE_POINTS_MAX - 1
	};	/**< Maximum number of mission items for each type
					(fence & save points use the first item for the stats) */

   	dm_item_t   _transfer_dataman_id{DM_KEY_WAYPOINTS_OFFBOARD_1};  ///< Dataman storage ID for current transmission
   	uint16_t    _transfer_count{0};		                            ///< Items count in current transmission
	uint16_t    _transfer_seq{0};			                        ///< Item sequence in current transmission
	int32_t	    _transfer_current_seq{-1};		                    ///< Current item ID for current transmission (-1 means not initialized)
	bool		_geofence_locked{false};		///< if true, we currently hold the dm_lock for the geofence (transaction in progress)
    uint16_t    _geofence_update_counter = 0;
    bool		_int_mode{false};			///< Use accurate int32 instead of float
    uint16_t    _count[3] = { 0, 0, 0 };
    int32_t     _current_seq = 0;
   	orb_advert_t    _offboard_mission_pub{nullptr};


    uint16_t current_max_item_count();
    void printMavMessageInfo(const mavlink_message_t* msg, 
        std::string prefix, bool always);
    
    void send_mission_ack(uint8_t sysid, uint8_t compid, uint8_t type);
    int update_geofence_count(unsigned count);
    int update_safepoint_count(unsigned count);
    int update_active_mission(dm_item_t dataman_id, uint16_t count, int32_t seq);
    void send_mission_request(uint8_t sysid, uint8_t compid, uint16_t seq);
    void processMAVLINK_MSG_ID_MISSION_REQUEST_LIST(const mavlink_message_t* msg);
    void processMAVLINK_MSG_ID_MISSION_REQUEST(const mavlink_message_t* msg);
    void handle_mission_count(const mavlink_message_t* msg);
    void processMAVLINK_MSG_ID_MISSION_SET_CURRENT(const mavlink_message_t* msg);
    void processMAVLINK_MSG_ID_MISSION_ITEM(const mavlink_message_t* msg);
};

} /* namespace mav2dji*/