/*
 * mav2dji_ros.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <params.hpp>

namespace mav2dji
{

// this needs to be at least MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_CHECKSUM_BYTES 
//    + 7 + MAVLINK_SIGNATURE_BLOCK_LEN + 14. Let's round way up. 
#define UDP_BUFFER_LENGTH 2041

//*****************************************************************************
//*
//*
//*
//******************************************************************************

Params::Params()
{
    init();
}

Params::~Params()
{

}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int Params::init()
{
    App = std::make_shared<ParamsApp>();
    VehicleInterface = std::make_shared<ParamsVehicleInterface>();

    return 0;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

ParamsRet Params::readParams()
{
    ParamsRet ret = App->readParams();
    ret = VehicleInterface->readParams();

    //*** TODO : inspect ret

    return ret;
}
   
} /* namespace mav2dji*/