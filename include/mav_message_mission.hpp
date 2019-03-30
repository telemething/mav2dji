/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once 

#include <mav_message_base.hpp>
#include <vehicle_Info.hpp>

#  define NUM_MISSIONS_SUPPORTED 2000 // This allocates a file of around 181 kB on the SD card.

namespace math
{
    #include <float.h>

    template<typename _Tp>
    constexpr const _Tp &min(const _Tp &a, const _Tp &b)
    {
        return (a < b) ? a : b;
    }

    template<typename _Tp>
    constexpr const _Tp &max(const _Tp &a, const _Tp &b)
    {
        return (a > b) ? a : b;
    }

    template<typename _Tp>
    constexpr const _Tp &constrain(const _Tp &val, const _Tp &min_val, const _Tp &max_val)
    {
        return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
    }

    /** Constrain float values to valid values for int16_t.
     * Invalid values are just clipped to be in the range for int16_t. */
    inline int16_t constrainFloatToInt16(float value)
    {
        return (int16_t)math::constrain(value, (float)INT16_MIN, (float)INT16_MAX);
    }

    template<typename _Tp>
    inline constexpr bool isInRange(const _Tp &val, const _Tp &min_val, const _Tp &max_val)
    {
        return (min_val <= val) && (val <= max_val);
    }

    template<typename T>
    constexpr T radians(const T degrees)
    {
        return degrees * (static_cast<T>(M_PI) / static_cast<T>(180));
    }

    template<typename T>
    constexpr T degrees(const T radians)
    {
        return radians * (static_cast<T>(180) / static_cast<T>(M_PI));
    }

    /** Save way to check if float is zero */
    inline bool isZero(const float val)
    {
        return fabsf(val - 0.0f) < FLT_EPSILON;
    }

    /** Save way to check if double is zero */
    inline bool isZero(const double val)
    {
        return fabs(val - 0.0) < DBL_EPSILON;
    }
}

//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************

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
    //ros::NodeHandle nodeHandle_;

    //explicit mav2dji_mission(ros::NodeHandle nh, std::shared_ptr<mavvehiclelib::mav_udp> mavUdp);
    //explicit mav2dji_mission(ros::NodeHandle nh);
    explicit mav2dji_mission(){};
    ~mav2dji_mission(){};
    
    int ProcessMavMessage(const mavlink_message_t* msg);

 private:

    bool verbose = false;
    bool _transfer_in_progress = false;

    dm_item_t _dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;

	enum MAVLINK_WPM_STATES _state {MAVLINK_WPM_STATE_IDLE};	///< Current state
	enum MAV_MISSION_TYPE _mission_type {MAV_MISSION_TYPE_MISSION};	///< mission type of current transmission (only one at a time possible)

   	uint64_t		_time_last_recv{0};
	uint64_t		_time_last_sent{0};
   	unsigned		_filesystem_errcount{0};		///< File system error count

    enum ORIGIN 
    {
        ORIGIN_MAVLINK = 0,
        ORIGIN_ONBOARD
    };

    struct mission_item_s 
    {
        double lat;			/**< latitude in degrees				*/
        double lon;			/**< longitude in degrees				*/
        union 
        {
            struct 
            {
                union 
                {
                    float time_inside;		/**< time that the MAV should stay inside the radius before advancing in seconds */
                    float pitch_min;		/**< minimal pitch angle for fixed wing takeoff waypoints */
                    float circle_radius;		/**< geofence circle radius in meters (only used for NAV_CMD_NAV_FENCE_CIRCLE*) */
                };
                float acceptance_radius;	/**< default radius in which the mission is accepted as reached in meters */
                float loiter_radius;		/**< loiter radius in meters, 0 for a VTOL to hover, negative for counter-clockwise */
                float yaw;					/**< in radians NED -PI..+PI, NAN means don't change yaw		*/
                float ___lat_float;			/**< padding */
                float ___lon_float;			/**< padding */
                float altitude;				/**< altitude in meters	(AMSL)			*/
            };
            float params[7];				/**< array to store mission command values for MAV_FRAME_MISSION ***/
        };
        uint16_t nav_cmd;					/**< navigation command					*/
        int16_t do_jump_mission_index;		/**< index where the do jump will go to                 */
        uint16_t do_jump_repeat_count;		/**< how many times do jump needs to be done            */
        union 
        {
            uint16_t do_jump_current_count;		/**< count how many times the jump has been done	*/
            uint16_t vertex_count;			/**< Polygon vertex count (geofence)	*/
            uint16_t land_precision;		/**< Defines if landing should be precise: 0 = normal landing, 1 = opportunistic precision landing, 2 = required precision landing (with search)	*/
        };
        struct 
        {
            uint16_t frame : 4,					/**< mission frame */
                origin : 3,						/**< how the mission item was generated */
                loiter_exit_xtrack : 1,			/**< exit xtrack location: 0 for center of loiter wp, 1 for exit location */
                force_heading : 1,				/**< heading needs to be reached */
                altitude_is_relative : 1,		/**< true if altitude is relative from start point	*/
                autocontinue : 1,				/**< true if next waypoint should follow after this one */
                vtol_back_transition : 1;		/**< part of the vtol back transition sequence */
        };
    };

    struct mission_fence_point_s 
    {
        double lat;
        double lon;
        float alt;
        uint16_t nav_cmd;				/**< navigation command (one of MAV_CMD_NAV_FENCE_*) */
        union 
        {
            uint16_t vertex_count;			/**< number of vertices in this polygon */
            float circle_radius;			/**< geofence circle radius in meters (only used for NAV_CMD_NAV_FENCE_CIRCLE*) */
        };
        uint8_t frame;					/**< MAV_FRAME */
    };

    struct mission_save_point_s 
    {
        double lat;
        double lon;
        float alt;
        uint8_t frame;					/**< MAV_FRAME */
    };

    enum NAV_CMD 
    {
        NAV_CMD_IDLE = 0,
        NAV_CMD_WAYPOINT = 16,
        NAV_CMD_LOITER_UNLIMITED = 17,
        NAV_CMD_LOITER_TIME_LIMIT = 19,
        NAV_CMD_RETURN_TO_LAUNCH = 20,
        NAV_CMD_LAND = 21,
        NAV_CMD_TAKEOFF = 22,
        NAV_CMD_LOITER_TO_ALT = 31,
        NAV_CMD_DO_FOLLOW_REPOSITION = 33,
        NAV_CMD_VTOL_TAKEOFF = 84,
        NAV_CMD_VTOL_LAND = 85,
        NAV_CMD_DELAY = 93,
        NAV_CMD_DO_JUMP = 177,
        NAV_CMD_DO_CHANGE_SPEED = 178,
        NAV_CMD_DO_SET_HOME = 179,
        NAV_CMD_DO_SET_SERVO = 183,
        NAV_CMD_DO_LAND_START = 189,
        NAV_CMD_DO_SET_ROI_LOCATION = 195,
        NAV_CMD_DO_SET_ROI_WPNEXT_OFFSET = 196,
        NAV_CMD_DO_SET_ROI_NONE = 197,
        NAV_CMD_DO_SET_ROI = 201,
        NAV_CMD_DO_DIGICAM_CONTROL = 203,
        NAV_CMD_DO_MOUNT_CONFIGURE = 204,
        NAV_CMD_DO_MOUNT_CONTROL = 205,
        NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214,
        NAV_CMD_DO_SET_CAM_TRIGG_DIST = 206,
        NAV_CMD_SET_CAMERA_MODE = 530,
        NAV_CMD_IMAGE_START_CAPTURE = 2000,
        NAV_CMD_IMAGE_STOP_CAPTURE = 2001,
        NAV_CMD_DO_TRIGGER_CONTROL = 2003,
        NAV_CMD_VIDEO_START_CAPTURE = 2500,
        NAV_CMD_VIDEO_STOP_CAPTURE = 2501,
        NAV_CMD_DO_VTOL_TRANSITION = 3000,
        NAV_CMD_FENCE_RETURN_POINT = 5000,
        NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION = 5001,
        NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION = 5002,
        NAV_CMD_FENCE_CIRCLE_INCLUSION = 5003,
        NAV_CMD_FENCE_CIRCLE_EXCLUSION = 5004,
        NAV_CMD_INVALID = UINT16_MAX /* ensure that casting a large number results in a specific error */
    };

    template<typename Type>
    bool is_finite(Type x) 
    {
        return std::isfinite(x);
    }

    template<typename Type>
    Type wrap_2pi(Type x)
    {
        if (!is_finite(x)) 
        {
            return x;
        }

        int c = 0;

        while (x >= Type(2 * M_PI))
         {
            x -= Type(2 * M_PI);

            if (c++ > 100) 
            {
                return INFINITY;
            }
        }

        c = 0;

        while (x < Type(0)) 
        {
            x += Type(2 * M_PI);

            if (c++ > 100) 
            {
                return INFINITY;
            }
        }

        return x;
    }


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

    int parse_mavlink_mission_item(const mavlink_mission_item_t *mavlink_mission_item,
		struct mission_item_s *mission_item);
    void switch_to_idle_state();

    void send_mission_request(uint8_t sysid, uint8_t compid, uint16_t seq);
    void handle_mission_request_list(const mavlink_message_t* msg);
    void handle_mission_request(const mavlink_message_t* msg);
    void handle_mission_count(const mavlink_message_t* msg);
    void handle_mission_set_current(const mavlink_message_t* msg);
	void handle_mission_ack(const mavlink_message_t* msg);
	void handle_mission_request_int(const mavlink_message_t* msg);
	void handle_mission_item(const mavlink_message_t* msg);
	void handle_mission_item_int(const mavlink_message_t* msg);
    void handle_mission_item_both(const mavlink_message_t *msg);
	void handle_mission_clear_all(const mavlink_message_t* msg);

    void processMAVLINK_MSG_ID_MISSION_ITEM(const mavlink_message_t* msg);

    //*** new below ***

    mav2dji::MissionWaypoint Convert(mission_item_s missionItem);
    mav2dji::MissionWaypointTask Convert(
	    std::vector<mission_item_s> missionItem);
    std::shared_ptr<mav2dji::vehicle_interface> getVehicleInterface();

    std::vector<mission_item_s> missionItemList;

    std::shared_ptr<mav2dji::vehicle_interface> vehicleInterface;
};

} /* namespace mav2dji*/