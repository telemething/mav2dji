//Onboard parameters for Vehicle 1
//
// Stack: PX4 Pro
// Vehicle: Multi-Rotor
// Version: 1.8.0 dev
// Git Revision: 93d2b092f0ec7483
//
// Vehicle-Id Component-Id Name Value Type

#include <mav_params.hpp>

namespace mav2dji
{

//*****************************************************************************
//*
//*****************************************************************************

MavParams::MavParams(){};
MavParams::~MavParams(){};

MavParams::paramValStruct::paramValStruct(char const* paramName, 
    const double value, const uint8_t type, uint16_t index) : 
        name(paramName), value(value), type(type), index(index) {}

struct MavParams::findParamByName
    {
        char const* name;
        findParamByName(char const* name) : name(name) {}
        bool operator () ( const paramValStruct& m ) const
        { return 0 == strcmp(m.name, name); }
    };

struct MavParams::findParamByIndex
    {
        uint16_t index;
        findParamByIndex(int16_t index) : index(index) {}
        bool operator () ( const paramValStruct& m ) const
        { return m.index == index; }
    };


MavParams::paramValStruct MavParams::fetchParam(char* paramName, const int16_t index)
{
    std::vector<paramValStruct>::iterator it;
    if(0 > index)
        it = std::find_if( paramValList.begin(), 
            paramValList.end(), findParamByName(paramName));
    else
        it = std::find_if( paramValList.begin(), 
            paramValList.end(), findParamByIndex(index));

    if (it == paramValList.end())
    {
        //we didn't find it, return empt val
        return paramValStruct("",0,0,0);
    } 

    return(*it);  
}

}


