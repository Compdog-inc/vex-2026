#ifndef ALLIANCE_H
#define ALLIANCE_H

#include "vex.h"

enum class Alliance
{
    Red = 0,
    Blue = 1
};

namespace vex
{
    Alliance getCurrentAlliance();
    void setCurrentAlliance(Alliance alliance);
};

#endif