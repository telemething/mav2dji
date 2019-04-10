
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

    OpRet(resultEnum result, std::string description );

    OpRet(resultEnum result );

    OpRet();
  
    static OpRet BuildError(bool printToConsole, bool printToRos, std::string format, ...);

    static OpRet BuildError(std::string message, bool printToConsole, bool printToRos);

    static OpRet UnwindStdException(const std::exception& e, 
        std::string prefix, bool printToConsole, bool printToRos);
};
}