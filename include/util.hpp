
#pragma once

#include <iostream>
#include <ros/ros.h>

namespace Util
{
class OpRet
{
   public:

    enum resultEnum {success, failure};
    resultEnum Result;
    std::string Description;

    OpRet(resultEnum result, std::string description )
    {
       Result = result;
       Description = description;
    }

    OpRet(resultEnum result )
    {
       Result = result;
       Description = "";
    }

    OpRet()
    {
       Result = resultEnum::success;
       Description = "";
    }
  
    static OpRet BuildError(bool printToConsole, bool printToRos, std::string format, ...)
    {   
        char buffer[1024];
        va_list args;
        va_start (args, format);
        vsnprintf (buffer, 1023, format.c_str(), args);
        std::string message(buffer);

        if(printToRos)
            ROS_ERROR_STREAM(buffer);

        if(printToConsole)
            std::cout << message << std::endl;

        return OpRet(
            OpRet::resultEnum::failure, message);
    }

    static OpRet BuildError(std::string message, bool printToConsole, bool printToRos)
    {   
        if(printToRos)
            ROS_ERROR_STREAM(message);

        if(printToConsole)
            std::cout << message << std::endl;

        return OpRet(
            OpRet::resultEnum::failure, message);
    }

    static OpRet UnwindStdException(const std::exception& e, 
        std::string prefix, bool printToConsole, bool printToRos)
    {
        std::string message = prefix +  " : Exception : " + std::string(e.what());
        
        return BuildError(message, printToConsole, printToRos);
    }
};
}