/*
 * mav2dji_ros.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <mav_message_base.hpp>
#include <sys/time.h>

//*****************************************************************************
//*
//*
//*
//******************************************************************************

namespace mav2dji 
{

//*****************************************************************************
//*
//*
//*
//******************************************************************************

mav2dji_message_base::mav2dji_message_base() 
{
	sendMavMessageCallback = VehicleInfo::getSendMavMessageCallback();
	vehicleInterface = VehicleInfo::getVehicleInterface();
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

mav2dji_message_base::~mav2dji_message_base()
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

uint64_t mav2dji_message_base::microsSinceEpoch()
{
	struct timeval tv;
	
	uint64_t micros = 0;
	
	gettimeofday(&tv, NULL);  
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	
	return micros;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

uint64_t mav2dji_message_base::hrt_absolute_time()
{
    return microsSinceEpoch();    
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void mav2dji_message_base::send_statustext_critical(const char *string)
{
	//*** TODO : show to user?
	mavlink_log_critical(&_mavlink_log_pub, "%s", string);
	PX4_ERR("%s", string);
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int mav2dji_message_base::getMavlinkSystemId() { return VehicleInfo::getMavlinkSystemId(); }
int mav2dji_message_base::getMavlinkComponentId() { return VehicleInfo::getMavlinkComponentId(); }

int mav2dji_message_base::sendMavMessageToGcs(const mavlink_message_t* msg){ return sendMavMessageCallback(msg);};


int mav2dji_message_base::dm_lock(dm_item_t item){return 0;};
int mav2dji_message_base::dm_trylock( dm_item_t item ){return 0;};
void mav2dji_message_base::dm_unlock( dm_item_t item ){};
int mav2dji_message_base::dm_clear( dm_item_t item ){return 0;};
int mav2dji_message_base::dm_restart( dm_reset_reason restart_type){return 0;};
ssize_t mav2dji_message_base::dm_read( dm_item_t item, unsigned index, void *buffer, size_t buflen )
    { printf("\r\n###### dm_read not implemented ######\r\n"); throw std::runtime_error("dm_read not implemented");};
ssize_t mav2dji_message_base::dm_write( dm_item_t  item, unsigned index, dm_persitence_t persistence, const void *buffer, size_t buflen ){return buflen;};
orb_advert_t mav2dji_message_base::orb_advertise(const struct orb_metadata *meta, const void *data){};
int	mav2dji_message_base::orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data){};

inline void mav2dji_message_base::do_nothing(int level, ...){(void)level;}
inline void mav2dji_message_base::debugPrintf(int level, const char * fmt, ...)
{
    if(level >= logLevel)
    {
        va_list args;
        va_start(args, fmt);
        printf(fmt, args);
        va_end(args);
    }
    printf("\r\n");
}

void mav2dji_message_base::PX4_DEBUG(const char * fmt, ...)
{
    if(logLevel >= _PX4_LOG_LEVEL_DEBUG)
    {
        va_list args;
        va_start(args, fmt);
        printf(fmt, args);
        va_end(args);
    }
    printf("\r\n");
}

void mav2dji_message_base::px4_log_modulename(int level, const char *moduleName, const char *fmt, ...)
{
    //*** TODO
}
void mav2dji_message_base::mavlink_vasprintf(int severity, orb_advert_t *mavlink_log_pub, const char *fmt, ...)
{
    //*** TODO
}

}
