#include "alliance.h"

static Alliance currentAlliance = Alliance::Red;

Alliance vex::getCurrentAlliance()
{
    return currentAlliance;
}

void vex::setCurrentAlliance(Alliance alliance)
{
    currentAlliance = alliance;
}