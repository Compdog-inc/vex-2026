#include "alliance.h"

Alliance vex::getCurrentAlliance()
{
#ifndef VEX
    return static_cast<Alliance>(getSimulationAlliance());
#else
    return Alliance::Red;
#endif
}