#include "autointake.h"

AutoIntake::AutoIntake(Intake *intake)
    : intake(intake)
{
    addRequirement(intake);
}

void AutoIntake::initialize()
{
    intake->set(IntakeState::Intake);
}

void AutoIntake::execute()
{
    intake->set(IntakeState::Intake);
}

void AutoIntake::end(bool interrupted)
{
    intake->set(IntakeState::Off);
}

bool AutoIntake::isFinished()
{
    return false;
}
