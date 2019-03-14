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

void mav2dji_message_base::addSendMavMessageCallback(MavlinkMessageInfo::mavMessageCallbackType callback)
{
	sendMavMessageCallback = callback;
}

}
