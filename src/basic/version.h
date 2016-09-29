#ifndef _SIMLOID_VERSION_
#define _SIMLOID_VERSION_

#include <string>

namespace version
{
    const std::string  prog_name = "SimloidTCP";
    const unsigned int major     = 0;     // interface changes
    const unsigned int minor     = 7;     // feature changes
    const unsigned int bugfixes  = 2;
    const std::string  name      = prog_name + " " + std::to_string(major)
                                             + "." + std::to_string(minor)
                                             + "." + std::to_string(bugfixes);
}

#endif
