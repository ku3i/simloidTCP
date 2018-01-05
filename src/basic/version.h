#ifndef _SIMLOID_VERSION_
#define _SIMLOID_VERSION_

#include <string>

namespace version
{
    const std::string  prog_name = "SimloidTCP";
    const unsigned int major     = 1;     // interface changes
    const unsigned int minor     = 0;     // feature changes
    const unsigned int bugfixes  = 0;
    const std::string  name      = prog_name + " " + std::to_string(major)
                                             + "." + std::to_string(minor)
                                             + "." + std::to_string(bugfixes);
}

#endif
