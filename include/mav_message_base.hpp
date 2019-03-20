/*
 * mav2dji_ros.hpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#pragma once 
// ROS
//#include <ros/ros.h>
#include <vehicle_Info.hpp>
//#include <mav_udp.hpp>

#define PX4_ERROR (-1)
#define PX4_OK 0

#define _PX4_LOG_LEVEL_DEBUG	0
#define _PX4_LOG_LEVEL_INFO		1
#define _PX4_LOG_LEVEL_WARN		2
#define _PX4_LOG_LEVEL_ERROR	3
#define _PX4_LOG_LEVEL_PANIC	4

#define _MSG_PRIO_DEBUG		7
#define _MSG_PRIO_INFO		6
#define _MSG_PRIO_NOTICE	5
#define _MSG_PRIO_WARNING	4
#define _MSG_PRIO_ERROR		3
#define _MSG_PRIO_CRITICAL	2
#define _MSG_PRIO_ALERT		1
#define _MSG_PRIO_EMERGENCY	0

#define MODULE_NAME "dji2mav"

//*** Orb stuff, just to allow compiling
typedef void 	*orb_advert_t;
#define ORB_ID(_name) &mav2dji_message_base::orbDummy

#define __px4_log_modulename(level, fmt, ...) \
	do { \
		mav2dji_message_base::px4_log_modulename(level, MODULE_NAME, fmt, ##__VA_ARGS__); \
	} while(0)

#define __px4_log_omit(level, FMT, ...)   mav2dji_message_base::do_nothing(level, ##__VA_ARGS__)

#define PX4_PANIC(FMT, ...)	__px4_log_modulename(_PX4_LOG_LEVEL_PANIC, FMT, ##__VA_ARGS__)
#define PX4_ERR(FMT, ...)	__px4_log_modulename(_PX4_LOG_LEVEL_ERROR, FMT, ##__VA_ARGS__)
#define PX4_WARN(FMT, ...) 	__px4_log_modulename(_PX4_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)
#define PX4_DEBUG(FMT, ...) 	__px4_log_omit(_PX4_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

//****************

#define mavlink_log_critical(_pub, _text, ...) \
	do { \
	mavlink_vasprintf(_MSG_PRIO_CRITICAL, _pub, _text, ##__VA_ARGS__); \
		PX4_WARN(_text, ##__VA_ARGS__); \
	} while(0);



namespace mav2dji
{
//class mav2dji_ros : mavvehiclelib::mavvehicleclient
class mav2dji_message_base
{
 public:

    /** Types of items that the data manager can store */
    typedef enum {
        DM_KEY_SAFE_POINTS = 0,		/* Safe points coordinates, safe point 0 is home point */
        DM_KEY_FENCE_POINTS,		/* Fence vertex coordinates */
        DM_KEY_WAYPOINTS_OFFBOARD_0,	/* Mission way point coordinates sent over mavlink */
        DM_KEY_WAYPOINTS_OFFBOARD_1,	/* (alternate between 0 and 1) */
        DM_KEY_WAYPOINTS_ONBOARD,	/* Mission way point coordinates generated onboard */
        DM_KEY_MISSION_STATE,		/* Persistent mission state */
        DM_KEY_COMPAT,
        DM_KEY_NUM_KEYS			/* Total number of item types defined */
    } dm_item_t;

    explicit mav2dji_message_base();
    ~mav2dji_message_base();

    //int getMavlinkSystemId() { return mav_udp_->getMavlinkSystemId(); }
    //int getMavlinkComponentId() { return mav_udp_->getMavlinkComponentId(); }

    int getMavlinkSystemId() { return VehicleInfo::getMavlinkSystemId(); }
    int getMavlinkComponentId() { return VehicleInfo::getMavlinkComponentId(); }

    //void addSendMavMessageCallback(MavlinkMessageInfo::mavMessageCallbackType callback);

    //int sendMavMessageToGcs(const mavlink_message_t* msg){ return mav_udp_->sendMavMessageToGcs(msg);};
    int sendMavMessageToGcs(const mavlink_message_t* msg){ return sendMavMessageCallback(msg);};

    //std::shared_ptr<mavvehiclelib::mav_udp> mav_udp_;

    MavlinkMessageInfo::mavMessageCallbackType sendMavMessageCallback;

    static uint64_t microsSinceEpoch();
    uint64_t hrt_absolute_time();
    void send_statustext_critical(const char *string);

    typedef void* orb_advert_t;

    uint8_t _transfer_partner_sysid  = 0; 
    uint8_t _transfer_partner_compid = 0;

   	orb_advert_t _mavlink_log_pub{nullptr};
	orb_advert_t _telem_status_pub{nullptr};

    //*** dataman stuff, looks like eprom management, not suire if I care ***
    typedef enum 
    {
        DM_PERSIST_POWER_ON_RESET = 0,	/* Data survives all resets */
        DM_PERSIST_IN_FLIGHT_RESET,     /* Data survives in-flight resets only */
        DM_PERSIST_VOLATILE             /* Data does not survive resets */
    } dm_persitence_t;

    typedef enum 
    {
        DM_INIT_REASON_POWER_ON = 0,	/* Data survives resets */
        DM_INIT_REASON_IN_FLIGHT,		/* Data survives in-flight resets only */
        DM_INIT_REASON_VOLATILE			/* Data does not survive reset */
    } dm_reset_reason;

    int dm_lock(dm_item_t item){return 0;};
    int dm_trylock( dm_item_t item ){return 0;};
    void dm_unlock( dm_item_t item ){};
    int dm_clear( dm_item_t item ){return 0;};
    int dm_restart( dm_reset_reason restart_type){return 0;};
    ssize_t dm_read( dm_item_t item, unsigned index, void *buffer, size_t buflen ){return buflen;};
    ssize_t dm_write( dm_item_t  item, unsigned index, dm_persitence_t persistence, const void *buffer, size_t buflen ){return buflen;};
	static dm_item_t    _dataman_id;				                 ///< Global Dataman storage ID for active mission
	dm_item_t			_my_dataman_id{DM_KEY_WAYPOINTS_OFFBOARD_0}; ///< class Dataman storage ID

    //*** Orb stuff, maybe dont care

    struct orb_metadata 
    {
        const char *o_name;		/**< unique object name */
        const uint16_t o_size;		/**< object size */
        const uint16_t o_size_no_padding;	/**< object size w/o padding at the end (for logger) */
        const char *o_fields;		/**< semicolon separated list of fields (with type) */
    };
    
    static orb_metadata orbDummy;

    orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data){};
    int	orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data){};

    static inline void do_nothing(int level, ...){(void)level;}

    static void px4_log_modulename(int level, const char *moduleName, const char *fmt, ...)
    {
        //*** TODO
    }
    static void mavlink_vasprintf(int severity, orb_advert_t *mavlink_log_pub, const char *fmt, ...)
    {
        //*** TODO
    }

    
 private:

    bool verbose = false;
};

} /* namespace mav2dji*/



